//! Simultaneous 5-axis finishing.
//!
//! **Surface finishing with tool-axis control** ([`surface_finish`]).
//!
//! 1. A zig-zag pattern laid out in a projection frame is dropped onto the
//!    part with a ball (exact sphere drop: faces, edges, vertices). Each
//!    point gets a gouge-free ball centre and a contact normal.
//! 2. With a ball end mill the tool axis can turn freely about the ball
//!    centre without moving the contact. [`AxisControl`] picks the preferred
//!    axis per point: fixed, surface normal with lead and tilt, or toward or
//!    away from a point or line.
//! 3. Each axis is checked:
//!    - the machine must reach it within its limits, continuing from the
//!      previous point;
//!    - with a scene, the shank, holder, head and the cutter's side above the
//!      ball must stay clear of the part, fixtures and machine.
//!
//!    If the preferred axis fails, rings of increasing tilt around it are
//!    searched, keeping the ball centre fixed and preferring continuity.
//! 4. Each pass's axes are smoothed and re-checked, and passes are joined
//!    along the surface where possible, otherwise by retract and approach.
//!
//! **Swarf (flank) milling** ([`swarf`]): a ruled wall given by bottom and
//! top rails is cut with the side of the tool. The axis follows the rulings
//! and the tool is offset by its radius along the wall normal. The deviation
//! between stations is reported, since twisted (non-developable) walls cannot
//! be cut exactly by a cylinder.

use crate::collision::{MoveClass, World};
use crate::geom::{sphere_drop_triangle, Aabb, Bvh, Mesh};
use crate::kinematics::{Kinematics, Pose};
use crate::math::{slerp, Mat3, Vec3};
use crate::post::ClMove;
use crate::rough::machining_frame;
use crate::stock::CutterShape;
use serde::{Deserialize, Serialize};

/// How the preferred tool axis is chosen at each point (WCS coordinates).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum AxisControl {
    Fixed {
        axis: Vec3,
    },
    /// Surface normal, leaning `lead_deg` toward the direction of travel
    /// and `tilt_deg` sideways.
    Normal {
        lead_deg: f64,
        tilt_deg: f64,
    },
    /// Axis line through `point`, spindle toward it.
    TowardPoint {
        point: Vec3,
    },
    /// Axis line through `point`, spindle away from it.
    FromPoint {
        point: Vec3,
    },
    TowardLine {
        point: Vec3,
        direction: Vec3,
    },
    FromLine {
        point: Vec3,
        direction: Vec3,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct AxisOptions {
    pub control: AxisControl,
    /// Search for a nearby collision-free axis when the preferred one fails.
    pub avoid: bool,
    /// Clearance for the tool assembly (mm).
    pub margin: f64,
    /// Largest deviation from the preferred axis when avoiding (degrees).
    pub max_tilt_deg: f64,
    pub tilt_step_deg: f64,
    /// Smoothing half-window (points); 0 disables.
    pub smooth: usize,
    /// Within this angle of the machine's singular direction, hold the
    /// primary rotary and solve only the other one (degrees). With a ball
    /// end mill the contact is unchanged; the axis moves by at most this.
    pub singularity_lock_deg: f64,
}

impl Default for AxisOptions {
    fn default() -> Self {
        AxisOptions {
            control: AxisControl::Normal {
                lead_deg: 10.0,
                tilt_deg: 0.0,
            },
            avoid: true,
            margin: 1.0,
            max_tilt_deg: 45.0,
            tilt_step_deg: 3.0,
            smooth: 4,
            singularity_lock_deg: 3.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct SurfaceParams {
    /// Ball end mill (corner radius = radius).
    pub cutter: CutterShape,
    /// Distance between passes (mm).
    pub stepover: f64,
    /// Point spacing along a pass (mm).
    pub step: f64,
    /// Projection direction of the pattern (WCS); passes run across it.
    pub projection: Vec3,
    /// Pass direction angle within the projection plane (degrees).
    pub angle_deg: f64,
    /// Stock left on the surface (mm).
    pub leave: f64,
    pub feed: f64,
    /// Retract distance along the tool axis between passes (mm).
    pub retract: f64,
    /// Clearance above the part for repositioning (mm).
    pub clearance: f64,
    /// Limit the pattern to this box in the projection frame (x, y), if given.
    pub region: Option<[f64; 4]>,
    pub axis: AxisOptions,
}

impl Default for SurfaceParams {
    fn default() -> Self {
        SurfaceParams {
            cutter: CutterShape::ball(6.0, 20.0),
            stepover: 0.5,
            step: 0.5,
            projection: Vec3::Z,
            angle_deg: 0.0,
            leave: 0.0,
            feed: 1500.0,
            retract: 5.0,
            clearance: 10.0,
            region: None,
            axis: AxisOptions::default(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FinishResult {
    pub moves: Vec<ClMove>,
    pub points: usize,
    /// Points where the axis had to move away from the preferred one.
    pub adjusted: usize,
    /// Points skipped because no reachable, collision-free axis was found.
    pub skipped: usize,
    /// Largest axis change between neighbouring points (degrees).
    pub max_axis_step_deg: f64,
    pub warnings: Vec<String>,
    pub summary: String,
}

/// One surface point: ball centre and contact normal (table frame).
#[derive(Clone, Copy, Debug)]
struct SurfPt {
    c: Vec3,
    n: Vec3,
}

/// Feasibility checks shared by both strategies.
struct Checker<'a> {
    kin: &'a Kinematics,
    world: Option<&'a World>,
    tool: Option<u32>,
    wcs: Vec3,
    margin: f64,
    class: MoveClass,
    lock_deg: f64,
}

impl Checker<'_> {
    /// The axis actually used for a wanted axis: near the singularity the
    /// primary rotary is held at its previous value, which changes the axis
    /// slightly.
    fn realise(&self, axis: Vec3, prev: Option<&[f64]>) -> Vec3 {
        let (Some(p), Some(s)) = (prev, self.kin.singularity_angle_deg(axis)) else {
            return axis;
        };
        if s >= self.lock_deg {
            return axis;
        }
        // Lock the primary rotary; the resulting axis is the FK of the
        // locked solution (tip does not matter for the direction).
        match self.kin.inverse_locked(
            &Pose {
                tip: Vec3::ZERO,
                axis,
            },
            p,
        ) {
            Some(sol) => self.kin.forward(&sol.joints).axis,
            None => axis,
        }
    }

    /// Joints for a ball centre `c` (table frame, radius `r`) with the tool
    /// along `axis` (adjusted near the singularity), if reachable and clear.
    /// Returns the axis used and the joints.
    fn ball(&self, c: Vec3, r: f64, axis: Vec3, prev: Option<&[f64]>) -> Option<(Vec3, Vec<f64>)> {
        let a = self.realise(axis, prev);
        let q = self.joints(c - a * r, a, prev)?;
        Some((a, q))
    }

    /// Joints for the tool tip `tip` (table frame) along `axis`, if reachable
    /// and collision free.
    fn joints(&self, tip: Vec3, axis: Vec3, prev: Option<&[f64]>) -> Option<Vec<f64>> {
        let near_singular = self
            .kin
            .singularity_angle_deg(axis)
            .is_some_and(|s| s < self.lock_deg);
        let sol = match (near_singular, prev) {
            (true, Some(p)) => self
                .kin
                .inverse_locked(
                    &Pose {
                        tip: tip - self.wcs,
                        axis,
                    },
                    p,
                )
                .filter(|s| s.within_limits)?,
            _ => self
                .kin
                .inverse(
                    &Pose {
                        tip: tip - self.wcs,
                        axis,
                    },
                    prev,
                )
                .ok()?,
        };
        if let Some(w) = self.world {
            if !w
                .check(self.kin, &sol.joints, self.tool, self.class, self.margin)
                .is_empty()
            {
                return None;
            }
        }
        // Avoid big rotary jumps between neighbouring points (branch flips).
        if let Some(p) = prev {
            for &i in self.kin.rotary_axes() {
                if (sol.joints[i] - p[i]).abs() > 60.0 {
                    return None;
                }
            }
        }
        Some(sol.joints)
    }
}

fn rotate(v: Vec3, about: Vec3, deg: f64) -> Vec3 {
    match about.normalized() {
        Some(k) => Mat3::rotation(k, deg.to_radians()).mul_vec(v),
        None => v,
    }
}

/// Preferred axis for a point (table frame), kept on the outer side.
fn preferred(ctrl: &AxisControl, pt: &SurfPt, travel: Vec3, wcs: Vec3) -> Vec3 {
    let a = match ctrl {
        AxisControl::Fixed { axis } => axis.normalized().unwrap_or(Vec3::Z),
        AxisControl::Normal { lead_deg, tilt_deg } => {
            let t = (travel - pt.n * travel.dot(pt.n))
                .normalized()
                .unwrap_or_else(|| perpendicular(pt.n));
            let side = pt.n.cross(t);
            let a = rotate(pt.n, side, *lead_deg);
            rotate(a, t, *tilt_deg)
        }
        AxisControl::TowardPoint { point } => (*point + wcs - pt.c).normalized().unwrap_or(pt.n),
        AxisControl::FromPoint { point } => (pt.c - (*point + wcs)).normalized().unwrap_or(pt.n),
        AxisControl::TowardLine { point, direction }
        | AxisControl::FromLine { point, direction } => {
            let d = direction.normalized().unwrap_or(Vec3::Z);
            let p = *point + wcs;
            let foot = p + d * (pt.c - p).dot(d);
            let away = (pt.c - foot).normalized().unwrap_or(pt.n);
            if matches!(ctrl, AxisControl::TowardLine { .. }) {
                -away
            } else {
                away
            }
        }
    };
    // The spindle must stay on the outside of the surface.
    if a.dot(pt.n) < 0.1 {
        let t = (a - pt.n * a.dot(pt.n))
            .normalized()
            .unwrap_or_else(|| perpendicular(pt.n));
        (pt.n * 0.1 + t * 0.995).normalized().unwrap_or(pt.n)
    } else {
        a
    }
}

fn perpendicular(v: Vec3) -> Vec3 {
    let h = if v.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    (h - v * h.dot(v)).normalized().unwrap_or(Vec3::X)
}

/// Choose a feasible axis near `pref` for ball centre `c` (radius `r`).
/// Returns (axis, joints, adjusted).
#[allow(clippy::too_many_arguments)]
fn choose_axis(
    ck: &Checker,
    c: Vec3,
    n: Vec3,
    r: f64,
    pref: Vec3,
    prev_axis: Option<Vec3>,
    prev_q: Option<&[f64]>,
    o: &AxisOptions,
) -> Option<(Vec3, Vec<f64>, bool)> {
    if let Some((a, q)) = ck.ball(c, r, pref, prev_q) {
        return Some((a, q, false));
    }
    if !o.avoid {
        return None;
    }
    let u = perpendicular(pref);
    let w = pref.cross(u);
    let rings = (o.max_tilt_deg / o.tilt_step_deg.max(0.5)).ceil() as usize;
    for k in 1..=rings {
        let tilt = (k as f64 * o.tilt_step_deg)
            .min(o.max_tilt_deg)
            .to_radians();
        let mut cands: Vec<Vec3> = (0..24)
            .map(|a| {
                let az = a as f64 * std::f64::consts::TAU / 24.0;
                (pref * tilt.cos() + (u * az.cos() + w * az.sin()) * tilt.sin())
                    .normalized()
                    .unwrap()
            })
            .filter(|a| a.dot(n) >= 0.1)
            .collect();
        // Try the candidates closest to the previous axis first.
        if let Some(pa) = prev_axis {
            cands.sort_by(|x, y| x.angle_to(pa).total_cmp(&y.angle_to(pa)));
        }
        for a in cands {
            if let Some((a, q)) = ck.ball(c, r, a, prev_q) {
                return Some((a, q, true));
            }
        }
    }
    None
}

/// 5-axis surface finishing of `parts` (table frame) with a ball end mill.
pub fn surface_finish(
    parts: &[&Mesh],
    kin: &Kinematics,
    world: Option<&World>,
    tool: Option<u32>,
    wcs_origin: Vec3,
    p: &SurfaceParams,
) -> FinishResult {
    let mut warnings = Vec::new();
    let r = p.cutter.radius;
    if (p.cutter.corner_radius - r).abs() > 1e-6 {
        warnings.push("surface finishing assumes a ball end mill; the cutter's corner radius differs from its radius".into());
    }
    let reff = r + p.leave;
    let f = machining_frame(wcs_origin, p.projection);
    let finv = f.inverse();
    let mut mesh = Mesh::default();
    for m in parts {
        mesh.append(m);
    }
    let local = mesh.transformed(&finv);
    let bvh = Bvh::build(&local);
    let bb = bvh.aabb();

    // Pattern lines in F.
    let ang = p.angle_deg.to_radians();
    let (ux, uy) = (ang.cos(), ang.sin());
    let (vx, vy) = (-uy, ux);
    let (mut x0, mut x1, mut y0, mut y1) = (
        bb.min.x - reff,
        bb.max.x + reff,
        bb.min.y - reff,
        bb.max.y + reff,
    );
    if let Some([a, b, c, d]) = p.region {
        x0 = x0.max(a);
        y0 = y0.max(b);
        x1 = x1.min(c);
        y1 = y1.min(d);
    }
    let corners = [(x0, y0), (x1, y0), (x0, y1), (x1, y1)];
    let proj = |(x, y): (f64, f64), (dx, dy): (f64, f64)| x * dx + y * dy;
    let (umin, umax) = corners
        .iter()
        .map(|&c| proj(c, (ux, uy)))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |a, v| {
            (a.0.min(v), a.1.max(v))
        });
    let (vmin, vmax) = corners
        .iter()
        .map(|&c| proj(c, (vx, vy)))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |a, v| {
            (a.0.min(v), a.1.max(v))
        });
    let drop = |x: f64, y: f64| -> Option<SurfPt> {
        if x < x0 || x > x1 || y < y0 || y > y1 {
            return None;
        }
        let q = Vec3::new(x, y, 0.0);
        let column = Aabb {
            min: Vec3::new(x - reff, y - reff, bb.min.z - 1.0),
            max: Vec3::new(x + reff, y + reff, bb.max.z + 1.0),
        };
        let mut best: Option<(f64, Vec3)> = None;
        bvh.any_in(&column, |t| {
            if let Some((h, cp)) = sphere_drop_triangle(q, Vec3::Z, reff, &local.tri(t)) {
                if best.is_none_or(|b| h > b.0) {
                    best = Some((h, cp));
                }
            }
            false
        });
        let (h, cp) = best?;
        let c = Vec3::new(x, y, h);
        let n = ((c - cp) * (1.0 / reff)).normalized()?;
        Some(SurfPt {
            c: f.apply_point(c),
            n: f.apply_vec(n),
        })
    };

    let mut lines: Vec<Vec<SurfPt>> = Vec::new();
    let nl = ((vmax - vmin) / p.stepover.max(0.01)).ceil() as usize;
    let np = ((umax - umin) / p.step.max(0.01)).ceil() as usize;
    for k in 0..=nl {
        let v = vmin + k as f64 * p.stepover;
        let mut run: Vec<SurfPt> = Vec::new();
        let mut runs: Vec<Vec<SurfPt>> = Vec::new();
        for i in 0..=np {
            let u = umin + i as f64 * p.step;
            match drop(u * ux + v * vx, u * uy + v * vy) {
                Some(pt) => run.push(pt),
                None => {
                    if run.len() > 1 {
                        runs.push(std::mem::take(&mut run));
                    }
                    run.clear();
                }
            }
        }
        if run.len() > 1 {
            runs.push(run);
        }
        if k % 2 == 1 {
            runs.reverse();
            for r in runs.iter_mut() {
                r.reverse();
            }
        }
        lines.extend(runs);
    }

    let ck = Checker {
        kin,
        world,
        tool,
        wcs: wcs_origin,
        margin: p.axis.margin,
        class: MoveClass::Finish,
        lock_deg: p.axis.singularity_lock_deg,
    };
    let mut out_moves: Vec<ClMove> = Vec::new();
    let (mut adjusted, mut skipped, mut total, mut rewinds) = (0usize, 0usize, 0usize, 0usize);
    let mut max_step: f64 = 0.0;
    // Safe height clears the part and every fixture (in the pattern frame).
    let fixture_top = world
        .map(|w| {
            w.table_meshes(crate::collision::Role::Fixture)
                .iter()
                .flat_map(|(_, m)| m.vertices.iter())
                .map(|&v| finv.apply_point(v).z)
                .fold(f64::NEG_INFINITY, f64::max)
        })
        .unwrap_or(f64::NEG_INFINITY);
    let z_safe = bb.max.z.max(fixture_top) + reff + p.clearance;
    let up = f.apply_vec(Vec3::Z);
    // Where to leave / join a cutting position: back out along the tool
    // axis until clear of the safe height (straight up if the tool is too
    // close to horizontal to get there along its axis).
    let out_point = |tip: Vec3, a: Vec3| -> Vec3 {
        let az = finv.apply_vec(a).z;
        let lz = finv.apply_point(tip).z;
        // Capped so steep tilts do not run the tool sideways off the machine.
        let d = if az > 0.2 {
            ((z_safe - lz) / az).clamp(p.retract, p.retract + 2.0 * p.clearance + 2.0 * reff)
        } else {
            p.retract
        };
        let pt = tip + a * d;
        let l = finv.apply_point(pt);
        if l.z < z_safe {
            f.apply_point(Vec3::new(l.x, l.y, z_safe))
        } else {
            pt
        }
    };
    let mut last_q: Option<Vec<f64>> = None;

    for line in &lines {
        // Axes for each point of this pass (splitting where infeasible).
        let mut segs: Vec<Vec<(SurfPt, Vec3, Vec<f64>)>> = vec![Vec::new()];
        let mut prev_axis: Option<Vec3> = None;
        let mut prev_q: Option<Vec<f64>> = last_q.clone();
        for (i, pt) in line.iter().enumerate() {
            total += 1;
            let a_idx = if i + 1 < line.len() { i + 1 } else { i };
            let b_idx = i.saturating_sub(1);
            let travel = line[a_idx].c - line[b_idx].c;
            let pref = preferred(&p.axis.control, pt, travel, wcs_origin);
            let attempt = choose_axis(
                &ck,
                pt.c,
                pt.n,
                r,
                pref,
                prev_axis,
                prev_q.as_deref(),
                &p.axis,
            );
            // Continuity failed (limits force another rotary branch): start a
            // new segment from a fresh solution instead of skipping points.
            let (attempt, fresh) = match attempt {
                Some(a) => (Some(a), false),
                None if prev_q.is_some() => (
                    choose_axis(&ck, pt.c, pt.n, r, pref, None, None, &p.axis),
                    true,
                ),
                None => (None, false),
            };
            match attempt {
                Some((a, q, adj)) => {
                    adjusted += adj as usize;
                    if fresh && !segs.last().unwrap().is_empty() {
                        rewinds += 1;
                        segs.push(Vec::new());
                    }
                    prev_axis = Some(a);
                    prev_q = Some(q.clone());
                    segs.last_mut().unwrap().push((*pt, a, q));
                }
                None => {
                    skipped += 1;
                    prev_axis = None;
                    if !segs.last().unwrap().is_empty() {
                        segs.push(Vec::new());
                    }
                }
            }
        }
        for seg in segs.iter_mut().filter(|s| s.len() > 1) {
            smooth_axes(seg, &ck, r, p.axis.smooth);
        }
        // The motion between neighbouring points must be clear too: split
        // passes where the midpoint fails.
        let mut pieces: Vec<Vec<(SurfPt, Vec3, Vec<f64>)>> = Vec::new();
        for seg in segs.into_iter().filter(|s| s.len() > 1) {
            let mut cur: Vec<(SurfPt, Vec3, Vec<f64>)> = Vec::new();
            for item in seg {
                if let Some(last) = cur.last() {
                    let mid = SurfPt {
                        c: last.0.c.lerp(item.0.c, 0.5),
                        n: slerp(last.0.n, item.0.n, 0.5),
                    };
                    let a = slerp(last.1, item.1, 0.5);
                    if ck.joints(mid.c - a * r, a, Some(&last.2)).is_none() {
                        pieces.push(std::mem::take(&mut cur));
                    }
                }
                cur.push(item);
            }
            pieces.push(cur);
        }
        for seg in pieces.into_iter().filter(|s| s.len() > 1) {
            for w in seg.windows(2) {
                max_step = max_step.max(w[0].1.angle_to(w[1].1).to_degrees());
            }
            // The control moves the tip in a straight line while the axis
            // turns, so the ball centre sags between points by about
            // r·(1 − cos(Δ/2)). Insert re-dropped points wherever the axis
            // turns more than 5°.
            let mut dense: Vec<(SurfPt, Vec3, Vec<f64>)> = Vec::with_capacity(seg.len());
            for w in seg.windows(2) {
                dense.push(w[0].clone());
                let turn = w[0].1.angle_to(w[1].1).to_degrees();
                let k = (turn / 5.0).ceil() as usize;
                let (la, lb) = (finv.apply_point(w[0].0.c), finv.apply_point(w[1].0.c));
                for j in 1..k {
                    let t = j as f64 / k as f64;
                    let pt = drop(la.x + (lb.x - la.x) * t, la.y + (lb.y - la.y) * t).unwrap_or(
                        SurfPt {
                            c: w[0].0.c.lerp(w[1].0.c, t),
                            n: slerp(w[0].0.n, w[1].0.n, t),
                        },
                    );
                    dense.push((pt, slerp(w[0].1, w[1].1, t), w[0].2.clone()));
                }
            }
            dense.push(seg.last().unwrap().clone());
            let seg = dense;
            // Travel upright at the safe height, turn to the start axis above
            // the start, come down along it, cut, then back out the same way.
            let (pt0, a0, _) = &seg[0];
            let start_tip = pt0.c - *a0 * r;
            let p0 = out_point(start_tip, *a0);
            out_moves.push(ClMove::Rapid {
                tip: p0 - wcs_origin,
                axis: up,
            });
            out_moves.push(ClMove::Rapid {
                tip: p0 - wcs_origin,
                axis: *a0,
            });
            out_moves.push(ClMove::Rapid {
                tip: start_tip + *a0 * p.retract - wcs_origin,
                axis: *a0,
            });
            for (pt, a, _) in seg.iter() {
                out_moves.push(ClMove::Feed {
                    tip: pt.c - *a * r - wcs_origin,
                    axis: *a,
                    feed: p.feed,
                });
            }
            let (ptn, an, qn) = seg.last().unwrap();
            let end_tip = ptn.c - *an * r;
            out_moves.push(ClMove::Feed {
                tip: end_tip + *an * p.retract - wcs_origin,
                axis: *an,
                feed: p.feed,
            });
            let pn = out_point(end_tip, *an);
            out_moves.push(ClMove::Rapid {
                tip: pn - wcs_origin,
                axis: *an,
            });
            out_moves.push(ClMove::Rapid {
                tip: pn - wcs_origin,
                axis: up,
            });
            last_q = Some(qn.clone());
        }
    }
    if max_step > 20.0 {
        warnings.push(format!("tool axis turns up to {max_step:.1}° between neighbouring points; consider a smaller step or more smoothing"));
    }
    let summary = format!(
        "{} passes, {} points: {} with the axis adjusted to avoid collisions or limits, {} skipped (no reachable clear axis), {} retract-and-rotate breaks for axis limits",
        lines.len(),
        total,
        adjusted,
        skipped,
        rewinds
    );
    if rewinds > lines.len() / 4 + 1 {
        warnings.push(format!(
            "{rewinds} passes had to break to rotate the rotaries to another branch; a different pass direction may avoid this"
        ));
    }
    FinishResult {
        moves: out_moves,
        points: total,
        adjusted,
        skipped,
        max_axis_step_deg: max_step,
        warnings,
        summary,
    }
}

/// Moving-average smoothing of the axes in a pass; each smoothed axis is
/// re-checked and kept only if still feasible.
fn smooth_axes(seg: &mut [(SurfPt, Vec3, Vec<f64>)], ck: &Checker, r: f64, half: usize) {
    if half == 0 || seg.len() < 3 {
        return;
    }
    for _ in 0..2 {
        let axes: Vec<Vec3> = seg.iter().map(|s| s.1).collect();
        for i in 0..seg.len() {
            let lo = i.saturating_sub(half);
            let hi = (i + half).min(seg.len() - 1);
            let sum = axes[lo..=hi].iter().fold(Vec3::ZERO, |acc, &a| acc + a);
            let Some(a) = sum.normalized() else { continue };
            if a.dot(seg[i].0.n) < 0.1 || a.angle_to(seg[i].1) < 1e-6 {
                continue;
            }
            let prev_q = if i > 0 {
                Some(seg[i - 1].2.clone())
            } else {
                None
            };
            if let Some((a, q)) = ck.ball(seg[i].0.c, r, a, prev_q.as_deref()) {
                seg[i].1 = a;
                seg[i].2 = q;
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Side {
    /// Tool on the left of the rails' direction (looking down the axis).
    Left,
    Right,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct SwarfParams {
    pub cutter: CutterShape,
    pub side: Side,
    /// Radial stock left on the wall (mm).
    pub leave: f64,
    /// Number of depth passes; each pass is `depth_step` higher than the next.
    pub passes: usize,
    pub depth_step: f64,
    /// Tip position relative to the bottom rail, along the axis (mm;
    /// negative = below the rail, e.g. to clear a floor fillet).
    pub tip_offset: f64,
    /// Station spacing along the rails (mm).
    pub step: f64,
    pub feed: f64,
    /// Lead-in / lead-out distance off the wall (mm).
    pub lead: f64,
    /// Retract distance along the axis (mm).
    pub retract: f64,
    pub margin: f64,
}

impl Default for SwarfParams {
    fn default() -> Self {
        SwarfParams {
            cutter: CutterShape::flat(10.0, 30.0),
            side: Side::Left,
            leave: 0.0,
            passes: 1,
            depth_step: 5.0,
            tip_offset: 0.0,
            step: 1.0,
            feed: 1000.0,
            lead: 3.0,
            retract: 10.0,
            margin: 1.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SwarfResult {
    pub moves: Vec<ClMove>,
    pub stations: usize,
    /// Largest distance between the cutter and the ruled wall between
    /// stations (mm); 0 for developable walls.
    pub max_deviation: f64,
    pub warnings: Vec<String>,
}

/// Resample a polyline to `n` points evenly spaced by arc length.
fn resample(poly: &[Vec3], n: usize) -> Vec<Vec3> {
    let mut cum = vec![0.0];
    for w in poly.windows(2) {
        cum.push(cum.last().unwrap() + w[0].distance(w[1]));
    }
    let total = *cum.last().unwrap();
    (0..n)
        .map(|k| {
            let s = total * k as f64 / (n - 1).max(1) as f64;
            let i = cum.partition_point(|&c| c < s).clamp(1, poly.len() - 1);
            let span = (cum[i] - cum[i - 1]).max(1e-12);
            poly[i - 1].lerp(poly[i], ((s - cum[i - 1]) / span).clamp(0.0, 1.0))
        })
        .collect()
}

/// Swarf-cut the ruled wall between `bottom` and `top` rails (WCS).
pub fn swarf(
    bottom: &[Vec3],
    top: &[Vec3],
    kin: &Kinematics,
    world: Option<&World>,
    tool: Option<u32>,
    wcs_origin: Vec3,
    p: &SwarfParams,
) -> SwarfResult {
    let mut warnings = Vec::new();
    if bottom.len() < 2 || top.len() < 2 {
        return SwarfResult {
            moves: vec![],
            stations: 0,
            max_deviation: 0.0,
            warnings: vec!["each rail needs at least two points".into()],
        };
    }
    let len: f64 = bottom.windows(2).map(|w| w[0].distance(w[1])).sum();
    let n = ((len / p.step.max(0.05)).ceil() as usize).max(2) + 1;
    let (b, t) = (resample(bottom, n), resample(top, n));
    let off = p.cutter.radius + p.leave;

    // Station frames: axis along the ruling, normal toward the tool side.
    let mut st: Vec<(Vec3, Vec3, Vec3)> = Vec::with_capacity(n); // (tip at bottom, axis, normal)
    for i in 0..n {
        let axis = (t[i] - b[i]).normalized().unwrap_or(Vec3::Z);
        let tan = (b[(i + 1).min(n - 1)] - b[i.saturating_sub(1)])
            .normalized()
            .unwrap_or(Vec3::X);
        let mut nrm = axis.cross(tan);
        if p.side == Side::Right {
            nrm = -nrm;
        }
        let nrm = (nrm - axis * nrm.dot(axis))
            .normalized()
            .unwrap_or(perpendicular(axis));
        st.push((b[i] + nrm * off + axis * p.tip_offset, axis, nrm));
    }

    // Deviation between stations: distance from the true ruling to the
    // interpolated cutter axis, minus the offset.
    let mut max_dev: f64 = 0.0;
    for i in 0..n - 1 {
        let (bm, tm) = (b[i].lerp(b[i + 1], 0.5), t[i].lerp(t[i + 1], 0.5));
        let tip = st[i].0.lerp(st[i + 1].0, 0.5);
        let a = slerp(st[i].1, st[i + 1].1, 0.5);
        for v in [0.0, 0.5, 1.0] {
            let pnt = bm.lerp(tm, v);
            let w = pnt - tip;
            let dist = (w - a * w.dot(a)).norm();
            max_dev = max_dev.max((dist - off).abs());
        }
    }

    let ck = Checker {
        kin,
        world,
        tool,
        wcs: wcs_origin,
        margin: p.margin,
        class: MoveClass::Feed,
        lock_deg: 0.0,
    };
    let mut moves = Vec::new();
    let mut prev_q: Option<Vec<f64>> = None;
    let mut blocked = 0usize;
    for pass in 0..p.passes.max(1) {
        let lift = (p.passes.max(1) - 1 - pass) as f64 * p.depth_step;
        let (tip0, a0, n0) = st[0];
        let tip0 = tip0 + a0 * lift;
        moves.push(ClMove::Rapid {
            tip: tip0 + n0 * p.lead + a0 * p.retract - wcs_origin,
            axis: a0,
        });
        moves.push(ClMove::Rapid {
            tip: tip0 + n0 * p.lead - wcs_origin,
            axis: a0,
        });
        for &(tip, a, _) in &st {
            let tip = tip + a * lift;
            match ck.joints(tip, a, prev_q.as_deref()) {
                Some(q) => prev_q = Some(q),
                None => blocked += 1,
            }
            moves.push(ClMove::Feed {
                tip: tip - wcs_origin,
                axis: a,
                feed: p.feed,
            });
        }
        let (tipn, an, nn) = st[n - 1];
        let tipn = tipn + an * lift;
        moves.push(ClMove::Feed {
            tip: tipn + nn * p.lead - wcs_origin,
            axis: an,
            feed: p.feed,
        });
        moves.push(ClMove::Rapid {
            tip: tipn + nn * p.lead + an * p.retract - wcs_origin,
            axis: an,
        });
    }
    if blocked > 0 {
        warnings.push(format!(
            "{blocked} stations are out of reach or collide; check the setup before running"
        ));
    }
    if max_dev > 0.01 {
        warnings.push(format!("the wall is twisted: the cutter deviates up to {max_dev:.3} mm from it between stations"));
    }
    SwarfResult {
        moves,
        stations: n,
        max_deviation: max_dev,
        warnings,
    }
}
