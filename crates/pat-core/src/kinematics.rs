//! Forward and inverse kinematics for 3-, 4- and 5-axis machines.
//!
//! Joint values are **machine coordinates** (what the control's DRO shows in
//! G53): mm for linear axes, degrees for rotary axes, one entry per axis in
//! [`MachineDef::axes`] order.
//!
//! *Program* coordinates are what a non-TCP G-code program commands: the
//! linear joints shifted so that program `X0 Y0 Z0` with rotaries at zero puts
//! the tool tip on part zero (tool length compensation assumed active). Rotary
//! program values equal machine values.
//!
//! Inverse kinematics is analytic. With all axes defined at home in the base
//! frame (product of exponentials), the tool axis in the part frame is
//!
//! ```text
//! v = R_table(q)^T * R_head(q) * spindle_dir
//!   = Rot(a1, s1*θ1) * Rot(a2, s2*θ2) * spindle_dir
//! ```
//!
//! so the rotary angles come from Paden–Kahan subproblem 2 (two rotations) or
//! 1 (one rotation). With the rotary angles fixed, the tool tip position is an
//! affine function of the linear joints, so those come from one 3x3 solve.

use crate::machine::{AxisKind, Chain, MachineDef};
use crate::math::{Iso, Mat3, Vec3};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Tool tip and tool axis (unit, tip → spindle) in the work coordinate system.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub tip: Vec3,
    pub axis: Vec3,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct IkSolution {
    /// Machine joint values, one per axis.
    pub joints: Vec<f64>,
    pub within_limits: bool,
    /// The tool axis is (near) a singular direction, so at least one rotary
    /// angle was chosen arbitrarily (held at its previous value).
    pub singular: bool,
}

#[derive(Debug, Error, PartialEq)]
pub enum KinError {
    #[error("tool axis [{0:.4}, {1:.4}, {2:.4}] is not reachable by this machine's rotary axes")]
    OrientationUnreachable(f64, f64, f64),
    #[error("tool axis direction is zero")]
    ZeroAxis,
    #[error("linear axes cannot position the tool tip in this configuration")]
    LinearSingular,
    #[error("no solution within axis limits")]
    NoSolutionInLimits,
}

const ORIENT_TOL: f64 = 1e-6;
const LIMIT_TOL: f64 = 1e-6;

#[derive(Clone, Debug)]
pub struct Kinematics {
    def: MachineDef,
    tool_length: f64,
    head: Vec<usize>,
    table: Vec<usize>,
    linear: [usize; 3],
    rotary: Vec<usize>,
    /// Rotations in the order they compose for the tool axis: (axis, sign).
    rot_seq: Vec<(usize, f64)>,
    tip_home: Vec3,
    /// Machine joints that put the tip on part zero with rotaries at zero.
    program_zero: Vec<f64>,
}

impl Kinematics {
    pub fn new(def: MachineDef, tool_length: f64) -> Result<Kinematics, KinError> {
        let idx = |chain: Chain| -> Vec<usize> {
            (0..def.axes.len())
                .filter(|&i| def.axes[i].chain == chain)
                .collect()
        };
        let head = idx(Chain::Head);
        let table = idx(Chain::Table);
        let lin: Vec<usize> = (0..def.axes.len())
            .filter(|&i| def.axes[i].kind == AxisKind::Linear)
            .collect();
        let rotary: Vec<usize> = (0..def.axes.len())
            .filter(|&i| def.axes[i].kind == AxisKind::Rotary)
            .collect();
        let linear = [lin[0], lin[1], lin[2]];

        // R_table^T reverses the table chain and negates its angles.
        let mut rot_seq: Vec<(usize, f64)> = table
            .iter()
            .rev()
            .filter(|&&i| def.axes[i].is_rotary())
            .map(|&i| (i, -1.0))
            .collect();
        rot_seq.extend(
            head.iter()
                .filter(|&&i| def.axes[i].is_rotary())
                .map(|&i| (i, 1.0)),
        );

        let tip_home = def.spindle.gauge_point - def.spindle.direction * tool_length;
        let mut k = Kinematics {
            program_zero: vec![0.0; def.axes.len()],
            def,
            tool_length,
            head,
            table,
            linear,
            rotary,
            rot_seq,
            tip_home,
        };
        let zero = vec![0.0; k.def.axes.len()];
        k.program_zero = k
            .solve_linear(&zero, Vec3::ZERO)
            .ok_or(KinError::LinearSingular)?;
        Ok(k)
    }

    pub fn machine(&self) -> &MachineDef {
        &self.def
    }

    pub fn tool_length(&self) -> f64 {
        self.tool_length
    }

    pub fn axis_count(&self) -> usize {
        self.def.axes.len()
    }

    pub fn linear_axes(&self) -> [usize; 3] {
        self.linear
    }

    pub fn rotary_axes(&self) -> &[usize] {
        &self.rotary
    }

    fn joint_iso(&self, i: usize, v: f64) -> Iso {
        let a = &self.def.axes[i];
        match a.kind {
            AxisKind::Linear => Iso::translation(a.direction * v),
            AxisKind::Rotary => Iso::rotation_about(a.direction, a.point, v.to_radians()),
        }
    }

    fn chain_iso(&self, chain: &[usize], q: &[f64]) -> Iso {
        chain
            .iter()
            .fold(Iso::IDENTITY, |acc, &i| acc.then(&self.joint_iso(i, q[i])))
    }

    /// Tool tip and axis in the WCS for machine joints `q`.
    pub fn forward(&self, q: &[f64]) -> Pose {
        let th = self.chain_iso(&self.head, q);
        let tt = self.chain_iso(&self.table, q);
        let tt_inv = tt.inverse();
        let tip = tt_inv.apply_point(th.apply_point(self.tip_home)) - self.def.wcs_origin;
        let axis = tt_inv.apply_vec(th.apply_vec(self.def.spindle.direction));
        Pose { tip, axis }
    }

    /// Fill in the linear joints of `q` so the tool tip lands on `tip` (WCS),
    /// keeping its rotary values. `None` if the linear axes are degenerate.
    pub fn solve_linear(&self, q: &[f64], tip: Vec3) -> Option<Vec<f64>> {
        let mut q0 = q.to_vec();
        for &i in &self.linear {
            q0[i] = 0.0;
        }
        let f0 = self.forward(&q0).tip;
        let cols: Vec<Vec3> = self
            .linear
            .iter()
            .map(|&i| {
                let mut qi = q0.clone();
                qi[i] = 1.0;
                self.forward(&qi).tip - f0
            })
            .collect();
        let x = Mat3::from_cols(cols[0], cols[1], cols[2]).solve(tip - f0)?;
        for (k, &i) in self.linear.iter().enumerate() {
            q0[i] = x.get(k);
        }
        Some(q0)
    }

    /// Machine joints → program coordinates (non-TCP G-code values).
    pub fn to_program(&self, q: &[f64]) -> Vec<f64> {
        q.iter()
            .zip(&self.program_zero)
            .map(|(a, b)| a - b)
            .collect()
    }

    /// Program coordinates → machine joints.
    pub fn to_machine(&self, p: &[f64]) -> Vec<f64> {
        p.iter()
            .zip(&self.program_zero)
            .map(|(a, b)| a + b)
            .collect()
    }

    /// Axes whose values are outside their soft limits, with the value.
    pub fn limit_violations(&self, q: &[f64]) -> Vec<(usize, f64)> {
        self.def
            .axes
            .iter()
            .enumerate()
            .filter(|(i, a)| !a.within_limits(q[*i], LIMIT_TOL))
            .map(|(i, _)| (i, q[i]))
            .collect()
    }

    /// The rotary axis whose angle becomes undefined at the singularity
    /// (machines with two rotary axes only).
    pub fn primary_rotary(&self) -> Option<usize> {
        (self.rot_seq.len() >= 2).then(|| self.rot_seq[0].0)
    }

    /// Angle in degrees between the tool axis and the machine's singular
    /// direction (the first rotary axis in the composition). `None` for
    /// machines with fewer than two rotary axes.
    pub fn singularity_angle_deg(&self, axis_wcs: Vec3) -> Option<f64> {
        if self.rot_seq.len() < 2 {
            return None;
        }
        let a1 = self.def.axes[self.rot_seq[0].0].direction;
        let ang = a1.angle_to(axis_wcs).to_degrees();
        Some(ang.min(180.0 - ang))
    }

    /// All inverse-kinematics solutions for `pose`, best first.
    ///
    /// Ordering: solutions within limits first, then the one closest to
    /// `prev` (rotary motion dominates), or with the smallest rotary angles
    /// when there is no previous position.
    pub fn inverse_all(
        &self,
        pose: &Pose,
        prev: Option<&[f64]>,
    ) -> Result<Vec<IkSolution>, KinError> {
        let v = pose.axis.normalized().ok_or(KinError::ZeroAxis)?;
        let (branches, singular) = self.solve_rotary(v, prev)?;

        let mut out: Vec<IkSolution> = Vec::new();
        for branch in branches {
            // Candidate angle lists per rotary axis (wraps of ±360°).
            let lists: Vec<Vec<f64>> = self
                .rot_seq
                .iter()
                .zip(&branch)
                .map(|(&(i, _), &deg)| self.wrap_candidates(i, deg, prev.map(|p| p[i])))
                .collect();
            for combo in cartesian(&lists) {
                let mut q = prev
                    .map(|p| p.to_vec())
                    .unwrap_or_else(|| vec![0.0; self.axis_count()]);
                for (&(i, _), &deg) in self.rot_seq.iter().zip(&combo) {
                    q[i] = deg;
                }
                let Some(q) = self.solve_linear(&q, pose.tip) else {
                    return Err(KinError::LinearSingular);
                };
                let within_limits = self.limit_violations(&q).is_empty();
                if out.iter().any(|s| max_abs_diff(&s.joints, &q) < 1e-7) {
                    continue;
                }
                out.push(IkSolution {
                    joints: q,
                    within_limits,
                    singular,
                });
            }
        }
        let score = |s: &IkSolution| -> f64 {
            match prev {
                Some(p) => s
                    .joints
                    .iter()
                    .zip(p)
                    .enumerate()
                    .map(|(i, (a, b))| {
                        let w = if self.def.axes[i].is_rotary() {
                            1.0
                        } else {
                            1e-3
                        };
                        w * (a - b).abs()
                    })
                    .sum(),
                None => self.rotary.iter().map(|&i| s.joints[i].abs()).sum(),
            }
        };
        out.sort_by(|a, b| {
            b.within_limits.cmp(&a.within_limits).then(
                score(a)
                    .partial_cmp(&score(b))
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
        });
        Ok(out)
    }

    /// Best inverse-kinematics solution within limits.
    pub fn inverse(&self, pose: &Pose, prev: Option<&[f64]>) -> Result<IkSolution, KinError> {
        self.inverse_all(pose, prev)?
            .into_iter()
            .find(|s| s.within_limits)
            .ok_or(KinError::NoSolutionInLimits)
    }

    /// Rotary angle branches (degrees, in `rot_seq` order) for tool axis `v`.
    fn solve_rotary(
        &self,
        v: Vec3,
        prev: Option<&[f64]>,
    ) -> Result<(Vec<Vec<f64>>, bool), KinError> {
        let z0 = self.def.spindle.direction;
        let unreachable = || KinError::OrientationUnreachable(v.x, v.y, v.z);
        let prev_phi = |k: usize| -> f64 {
            let (i, s) = self.rot_seq[k];
            prev.map(|p| s * p[i].to_radians()).unwrap_or(0.0)
        };
        let to_deg = |k: usize, phi: f64| self.rot_seq[k].1 * phi.to_degrees();

        match self.rot_seq.len() {
            0 => {
                if z0.angle_to(v) > ORIENT_TOL {
                    return Err(unreachable());
                }
                Ok((vec![vec![]], false))
            }
            1 => {
                let a = self.def.axes[self.rot_seq[0].0].direction;
                if (a.dot(z0) - a.dot(v)).abs() > ORIENT_TOL {
                    return Err(unreachable());
                }
                match subproblem1(a, z0, v) {
                    Some(phi) => Ok((vec![vec![to_deg(0, phi)]], false)),
                    // Spindle parallel to the rotary axis: any angle works.
                    None => Ok((vec![vec![to_deg(0, prev_phi(0))]], true)),
                }
            }
            _ => {
                let a1 = self.def.axes[self.rot_seq[0].0].direction;
                let a2 = self.def.axes[self.rot_seq[1].0].direction;
                let sing_tol = self.def.singularity_tolerance_deg.to_radians().min(1e-3);
                let singular = a1.cross(v).norm() < sing_tol.sin();
                let cs = subproblem2_intermediate(a1, a2, z0, v).ok_or_else(unreachable)?;
                let mut branches = Vec::new();
                for c in cs {
                    let phi2 = subproblem1(a2, z0, c).unwrap_or_else(|| prev_phi(1));
                    let phi1 = if singular {
                        prev_phi(0)
                    } else {
                        subproblem1(a1, c, v).unwrap_or_else(|| prev_phi(0))
                    };
                    branches.push(vec![to_deg(0, phi1), to_deg(1, phi2)]);
                }
                Ok((branches, singular))
            }
        }
    }

    fn wrap_candidates(&self, i: usize, deg: f64, prev: Option<f64>) -> Vec<f64> {
        let a = &self.def.axes[i];
        if a.is_continuous() {
            let v = match prev {
                Some(p) => deg + 360.0 * ((p - deg) / 360.0).round(),
                None => normalize_deg(deg),
            };
            return vec![v];
        }
        let centre = prev.unwrap_or(0.0);
        let lo = a.min.unwrap_or(centre - 720.0);
        let hi = a.max.unwrap_or(centre + 720.0);
        let k_lo = ((lo - deg) / 360.0).ceil() as i64;
        let k_hi = ((hi - deg) / 360.0).floor() as i64;
        let mut v: Vec<f64> = (k_lo..=k_hi).map(|k| deg + 360.0 * k as f64).collect();
        if v.is_empty() {
            v.push(normalize_deg(deg));
        }
        v
    }
}

/// Angle into (-180, 180].
pub fn normalize_deg(d: f64) -> f64 {
    let r = d.rem_euclid(360.0);
    if r > 180.0 {
        r - 360.0
    } else {
        r
    }
}

fn max_abs_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0, f64::max)
}

fn cartesian(lists: &[Vec<f64>]) -> Vec<Vec<f64>> {
    lists.iter().fold(vec![vec![]], |acc, list| {
        acc.iter()
            .flat_map(|prefix| {
                list.iter().map(move |&x| {
                    let mut p = prefix.clone();
                    p.push(x);
                    p
                })
            })
            .collect()
    })
}

/// Paden–Kahan subproblem 1: angle φ with `Rot(k, φ) u = v` (after projecting
/// both onto the plane normal to `k`). `None` if `u` is parallel to `k`.
fn subproblem1(k: Vec3, u: Vec3, v: Vec3) -> Option<f64> {
    let up = u - k * k.dot(u);
    let vp = v - k * k.dot(v);
    if up.norm() < 1e-9 || vp.norm() < 1e-9 {
        return None;
    }
    Some(k.dot(up.cross(vp)).atan2(up.dot(vp)))
}

/// Paden–Kahan subproblem 2 for unit vectors: the intermediate vectors
/// `c = Rot(a2, φ2) u = Rot(a1, -φ1) v`. Returns one or two candidates, or
/// `None` when `v` is unreachable.
fn subproblem2_intermediate(a1: Vec3, a2: Vec3, u: Vec3, v: Vec3) -> Option<Vec<Vec3>> {
    let d = a1.dot(a2);
    let cross = a1.cross(a2);
    let cn2 = cross.dot(cross);
    if cn2 < 1e-12 {
        // Parallel rotary axes: effectively one rotation.
        if (a1.dot(u) - a1.dot(v)).abs() > ORIENT_TOL {
            return None;
        }
        return Some(vec![u]);
    }
    let alpha = (d * a2.dot(u) - a1.dot(v)) / (d * d - 1.0);
    let beta = (d * a1.dot(v) - a2.dot(u)) / (d * d - 1.0);
    let g2 = (u.dot(u) - alpha * alpha - beta * beta - 2.0 * alpha * beta * d) / cn2;
    if g2 < -1e-9 {
        return None;
    }
    let base = a1 * alpha + a2 * beta;
    if g2 <= 1e-12 {
        return Some(vec![base]);
    }
    let g = g2.sqrt();
    Some(vec![base + cross * g, base - cross * g])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subproblem1_quarter_turn() {
        let phi = subproblem1(Vec3::Z, Vec3::X, Vec3::Y).unwrap();
        assert!((phi - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn normalize() {
        assert_eq!(normalize_deg(270.0), -90.0);
        assert_eq!(normalize_deg(-180.0), 180.0);
        assert_eq!(normalize_deg(45.0), 45.0);
    }
}
