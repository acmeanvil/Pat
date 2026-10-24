//! Post-processing: cutter-location (CL) paths → G-code.
//!
//! Input is a list of tool-tip positions and tool-axis vectors in the WCS,
//! which is what every toolpath strategy in Pat produces. Two dialects:
//!
//! - [`Dialect::LinuxcncJoint`]: plain joint-space G-code for controls
//!   without tool centre point control. Each move is solved with inverse
//!   kinematics and subdivided until joint-space interpolation keeps the tool
//!   tip within tolerance of the intended straight line. Feed moves use G93
//!   inverse-time feed so the programmed tip feed holds while rotaries move.
//! - [`Dialect::FanucTcp`]: tool centre point output (`G43.4`). XYZ is the
//!   tool tip in the WCS; the control handles kinematics. Rotary angles still
//!   come from our inverse kinematics so branch choice stays deterministic.

use crate::kinematics::{KinError, Kinematics, Pose};
use crate::machine::MachineDef;
use crate::math::{slerp, Vec3};
use serde::{Deserialize, Serialize};
use std::fmt::Write as _;
use thiserror::Error;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ClTool {
    pub number: u32,
    /// Gauge length (mm).
    pub length: f64,
    #[serde(default)]
    pub diameter: Option<f64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum ClMove {
    Rapid {
        tip: Vec3,
        axis: Vec3,
    },
    /// `feed` is the tool-tip feed rate in mm/min.
    Feed {
        tip: Vec3,
        axis: Vec3,
        feed: f64,
    },
}

impl ClMove {
    pub fn pose(&self) -> Pose {
        match *self {
            ClMove::Rapid { tip, axis } | ClMove::Feed { tip, axis, .. } => Pose { tip, axis },
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ClProgram {
    #[serde(default)]
    pub name: String,
    pub tool: ClTool,
    #[serde(default)]
    pub spindle_rpm: Option<f64>,
    #[serde(default)]
    pub coolant: bool,
    pub moves: Vec<ClMove>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Dialect {
    LinuxcncJoint,
    FanucTcp,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct PostOptions {
    pub dialect: Dialect,
    /// Max tool-tip deviation from the intended path (mm), joint dialect only.
    pub tolerance: f64,
    /// Max tool-axis deviation from the intended orientation (degrees).
    pub angle_tolerance_deg: f64,
    /// Max recursive halvings per CL segment.
    pub max_depth: u32,
    pub decimals_linear: usize,
    pub decimals_rotary: usize,
    /// Replace runs of feed moves that lie on a circle (constant rotaries
    /// and Z) with G2/G3 arcs: points within `tolerance` of the circle.
    pub arcs: bool,
    /// Largest distance an arc may bulge away from each straight segment it
    /// replaces (mm).
    pub arc_sagitta: f64,
}

impl Default for PostOptions {
    fn default() -> Self {
        PostOptions {
            dialect: Dialect::LinuxcncJoint,
            tolerance: 0.005,
            angle_tolerance_deg: 0.05,
            max_depth: 12,
            decimals_linear: 4,
            decimals_rotary: 4,
            arcs: true,
            arc_sagitta: 0.02,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PostOutput {
    pub gcode: String,
    pub blocks: usize,
    pub warnings: Vec<String>,
}

#[derive(Debug, Error)]
pub enum PostError {
    #[error("kinematics: {0}")]
    Kinematics(#[from] KinError),
    #[error("CL move {index}: {source}")]
    Move { index: usize, source: KinError },
    #[error("CL program has no moves")]
    Empty,
    #[error("CL move {0}: feed must be positive")]
    BadFeed(usize),
}

pub fn post(def: &MachineDef, cl: &ClProgram, opts: &PostOptions) -> Result<PostOutput, PostError> {
    if cl.moves.is_empty() {
        return Err(PostError::Empty);
    }
    let kin = Kinematics::new(def.clone(), cl.tool.length)?;
    let mut w = Writer::new(&kin, opts);
    w.header(cl);

    let mut prev_q: Option<Vec<f64>> = None;
    let mut prev_pose: Option<Pose> = None;
    for (index, mv) in cl.moves.iter().enumerate() {
        let pose = mv.pose();
        let sol = kin
            .inverse(&pose, prev_q.as_deref())
            .map_err(|source| PostError::Move { index, source })?;
        if sol.singular {
            w.warnings.push(format!(
                "CL move {index}: tool axis is at the singularity; holding the primary rotary"
            ));
        }
        let q = sol.joints;
        match (mv, &prev_q, &prev_pose) {
            (ClMove::Rapid { .. }, _, _) | (_, None, _) | (_, _, None) => {
                if let ClMove::Feed { feed, .. } = mv {
                    if *feed <= 0.0 {
                        return Err(PostError::BadFeed(index));
                    }
                    w.warnings.push(format!(
                        "CL move {index}: first move is a feed move; output as a rapid"
                    ));
                }
                w.rapid(&q, &pose);
            }
            (ClMove::Feed { feed, .. }, Some(qa), Some(pa)) => {
                let feed = *feed;
                if feed <= 0.0 {
                    return Err(PostError::BadFeed(index));
                }
                for &i in kin.rotary_axes() {
                    let d = (q[i] - qa[i]).abs();
                    if d > 90.0 {
                        w.warnings.push(format!(
                            "CL move {index}: {} swings {d:.1}° in one cutting move",
                            def.axes[i].name
                        ));
                    }
                }
                match opts.dialect {
                    Dialect::FanucTcp => w.tcp_feed(&q, &pose, feed),
                    Dialect::LinuxcncJoint => {
                        let mut out = Vec::new();
                        linearize(&kin, opts, pa, &pose, qa, &q, 0, &mut out)
                            .map_err(|source| PostError::Move { index, source })?;
                        let mut a_pose = *pa;
                        for (p, qq) in out {
                            w.joint_feed(&qq, &a_pose, &p, feed);
                            a_pose = p;
                        }
                    }
                }
            }
        }
        prev_q = Some(q);
        prev_pose = Some(pose);
    }
    w.footer();
    Ok(PostOutput {
        blocks: w.blocks,
        gcode: w.out,
        warnings: w.warnings,
    })
}

/// Intended pose at parameter `t` along a CL segment.
fn pose_at(a: &Pose, b: &Pose, t: f64) -> Pose {
    Pose {
        tip: a.tip.lerp(b.tip, t),
        axis: slerp(a.axis, b.axis, t),
    }
}

/// Recursively subdivide until joint interpolation tracks the intended path.
#[allow(clippy::too_many_arguments)]
fn linearize(
    kin: &Kinematics,
    opts: &PostOptions,
    pa: &Pose,
    pb: &Pose,
    qa: &[f64],
    qb: &[f64],
    depth: u32,
    out: &mut Vec<(Pose, Vec<f64>)>,
) -> Result<(), KinError> {
    let deviates = [0.25, 0.5, 0.75].iter().any(|&t| {
        let want = pose_at(pa, pb, t);
        let q: Vec<f64> = qa.iter().zip(qb).map(|(x, y)| x + (y - x) * t).collect();
        let got = kin.forward(&q);
        got.tip.distance(want.tip) > opts.tolerance
            || got.axis.angle_to(want.axis).to_degrees() > opts.angle_tolerance_deg
    });
    if !deviates || depth >= opts.max_depth {
        out.push((*pb, qb.to_vec()));
        return Ok(());
    }
    let pm = pose_at(pa, pb, 0.5);
    let qm = kin.inverse(&pm, Some(qa))?.joints;
    linearize(kin, opts, pa, &pm, qa, &qm, depth + 1, out)?;
    linearize(kin, opts, &pm, pb, &qm, qb, depth + 1, out)
}

struct Writer<'a> {
    kin: &'a Kinematics,
    opts: &'a PostOptions,
    out: String,
    blocks: usize,
    warnings: Vec<String>,
    /// Last emitted word values, for modal suppression.
    last: Vec<Option<String>>,
    inverse_time: bool,
    tcp_on: bool,
    tool: u32,
    /// Numeric value of each axis word as last written (rounded).
    last_num: Vec<Option<f64>>,
    /// Feed moves waiting to be written (so arcs can be fitted).
    pending: Vec<Pending>,
}

struct Pending {
    /// Output axis values (rounded): program joints, or tool tip for TCP.
    vals: Vec<f64>,
    minutes: f64,
    feed: f64,
}

impl<'a> Writer<'a> {
    fn new(kin: &'a Kinematics, opts: &'a PostOptions) -> Self {
        Writer {
            kin,
            opts,
            out: String::new(),
            blocks: 0,
            warnings: Vec::new(),
            last: vec![None; kin.axis_count()],
            inverse_time: false,
            tcp_on: false,
            tool: 0,
            last_num: vec![None; kin.axis_count()],
            pending: Vec::new(),
        }
    }

    fn line(&mut self, s: &str) {
        self.out.push_str(s);
        self.out.push('\n');
        self.blocks += 1;
    }

    fn header(&mut self, cl: &ClProgram) {
        self.tool = cl.tool.number;
        let m = self.kin.machine();
        self.line("%");
        let dialect = match self.opts.dialect {
            Dialect::LinuxcncJoint => "linuxcnc-joint",
            Dialect::FanucTcp => "fanuc-tcp",
        };
        let title = if cl.name.is_empty() {
            "program".to_string()
        } else {
            sanitize(&cl.name)
        };
        self.line(&format!(
            "(Pat post: {title} / {} / {dialect})",
            sanitize(&m.name)
        ));
        self.line(&format!(
            "(T{} length {:.3})",
            cl.tool.number, cl.tool.length
        ));
        self.line("G21 G90 G17 G40 G49 G80 G94");
        self.line("G54");
        self.line(&format!("T{} M6", cl.tool.number));
        match self.opts.dialect {
            Dialect::LinuxcncJoint => self.line(&format!("G43 H{}", cl.tool.number)),
            Dialect::FanucTcp => {}
        }
        if let Some(s) = cl.spindle_rpm {
            self.line(&format!("S{} M3", fmt(s, 0)));
        }
        if cl.coolant {
            self.line("M8");
        }
    }

    fn footer(&mut self) {
        self.flush();
        if self.inverse_time {
            self.line("G94");
        }
        self.line("G49");
        self.line("M5");
        self.line("M9");
        self.line("M30");
        self.line("%");
    }

    /// Axis words for joint values, skipping unchanged ones.
    fn words(&mut self, q: &[f64], tcp_tip: Option<Vec3>) -> String {
        let prog = self.kin.to_program(q);
        let mut s = String::new();
        let lin = self.kin.linear_axes();
        for (i, ax) in self.kin.machine().axes.iter().enumerate() {
            let v = if let (Some(tip), Some(k)) = (tcp_tip, lin.iter().position(|&l| l == i)) {
                match ax.name.as_str() {
                    "X" => tip.x,
                    "Y" => tip.y,
                    "Z" => tip.z,
                    _ => tip.get(k),
                }
            } else {
                prog[i]
            };
            let d = if ax.is_rotary() {
                self.opts.decimals_rotary
            } else {
                self.opts.decimals_linear
            };
            let txt = fmt(v, d);
            if self.last[i].as_deref() != Some(txt.as_str()) {
                let _ = write!(s, " {}{}", ax.name, txt);
                self.last_num[i] = txt.parse().ok();
                self.last[i] = Some(txt);
            }
        }
        s
    }

    /// Output values (rounded as they will be written) for joints `q`.
    fn out_vals(&self, q: &[f64], tcp_tip: Option<Vec3>) -> Vec<f64> {
        let prog = self.kin.to_program(q);
        let lin = self.kin.linear_axes();
        self.kin
            .machine()
            .axes
            .iter()
            .enumerate()
            .map(|(i, ax)| {
                let v = match (tcp_tip, lin.iter().position(|&l| l == i)) {
                    (Some(tip), Some(k)) => match ax.name.as_str() {
                        "X" => tip.x,
                        "Y" => tip.y,
                        "Z" => tip.z,
                        _ => tip.get(k),
                    },
                    _ => prog[i],
                };
                let d = if ax.is_rotary() {
                    self.opts.decimals_rotary
                } else {
                    self.opts.decimals_linear
                };
                fmt(v, d).parse().unwrap_or(v)
            })
            .collect()
    }

    /// Write buffered feed moves, as arcs where they fit.
    fn flush(&mut self) {
        let pending = std::mem::take(&mut self.pending);
        if pending.is_empty() {
            return;
        }
        let names: Vec<String> = self
            .kin
            .machine()
            .axes
            .iter()
            .map(|a| a.name.clone())
            .collect();
        let idx = |n: &str| names.iter().position(|a| a == n);
        let start: Option<Vec<f64>> = self.last_num.iter().copied().collect();
        let (xi, yi, zi) = (idx("X"), idx("Y"), idx("Z"));
        let mut i = 0;
        while i < pending.len() {
            let arc = match (self.opts.arcs, &start, xi, yi, zi) {
                (true, Some(st), Some(x), Some(y), Some(z)) => {
                    let mut pts: Vec<&[f64]> = Vec::with_capacity(pending.len() + 1);
                    pts.push(st);
                    pts.extend(pending.iter().map(|p| p.vals.as_slice()));
                    // pts[k] is the position after pending[k - 1].
                    fit_arc(
                        &pts,
                        i,
                        (x, y, z),
                        self.kin.rotary_axes(),
                        (self.opts.tolerance, self.opts.arc_sagitta),
                        &pending,
                        self.opts.dialect,
                    )
                }
                _ => None,
            };
            match arc {
                Some((j, centre, ccw)) => {
                    let (x, y) = (xi.unwrap(), yi.unwrap());
                    let from: Vec<f64> = self.last_num.iter().map(|v| v.unwrap_or(0.0)).collect();
                    let end = &pending[j - 1].vals;
                    let d = self.opts.decimals_linear;
                    let (ii, jj) = (centre.0 - from[x], centre.1 - from[y]);
                    let feed = match self.opts.dialect {
                        Dialect::LinuxcncJoint => {
                            let minutes: f64 = pending[i..j].iter().map(|p| p.minutes).sum();
                            fmt(1.0 / minutes.max(1e-6), 3)
                        }
                        Dialect::FanucTcp => fmt(pending[j - 1].feed, 1),
                    };
                    let (tx, ty) = (fmt(end[x], d), fmt(end[y], d));
                    self.line(&format!(
                        "{} X{tx} Y{ty} I{} J{} F{feed}",
                        if ccw { "G3" } else { "G2" },
                        fmt(ii, d),
                        fmt(jj, d)
                    ));
                    self.last_num[x] = tx.parse().ok();
                    self.last_num[y] = ty.parse().ok();
                    self.last[x] = Some(tx);
                    self.last[y] = Some(ty);
                    i = j;
                }
                None => {
                    let p = &pending[i];
                    let mut w = String::new();
                    for (k, name) in names.iter().enumerate() {
                        let dec = if self.kin.machine().axes[k].is_rotary() {
                            self.opts.decimals_rotary
                        } else {
                            self.opts.decimals_linear
                        };
                        let txt = fmt(p.vals[k], dec);
                        if self.last[k].as_deref() != Some(txt.as_str()) {
                            let _ = write!(w, " {name}{txt}");
                            self.last_num[k] = txt.parse().ok();
                            self.last[k] = Some(txt);
                        }
                    }
                    if !w.is_empty() {
                        let f = match self.opts.dialect {
                            Dialect::LinuxcncJoint => fmt(1.0 / p.minutes.max(1e-6), 3),
                            Dialect::FanucTcp => fmt(p.feed, 1),
                        };
                        self.line(&format!("G1{w} F{f}"));
                    }
                    i += 1;
                }
            }
        }
    }

    fn rapid(&mut self, q: &[f64], pose: &Pose) {
        self.flush();
        if self.opts.dialect == Dialect::FanucTcp && !self.tcp_on {
            // Turn on TCP before the first positioning move.
            let tool = self.tool;
            self.line(&format!("G43.4 H{tool}"));
            self.tcp_on = true;
        }
        let tip = (self.opts.dialect == Dialect::FanucTcp).then_some(pose.tip);
        let w = self.words(q, tip);
        if !w.is_empty() {
            self.line(&format!("G0{w}"));
        }
    }

    fn tcp_feed(&mut self, q: &[f64], pose: &Pose, feed: f64) {
        let vals = self.out_vals(q, Some(pose.tip));
        self.pending.push(Pending {
            vals,
            minutes: 0.0,
            feed,
        });
    }

    fn joint_feed(&mut self, q: &[f64], from: &Pose, to: &Pose, feed: f64) {
        if !self.inverse_time {
            self.line("G93");
            self.inverse_time = true;
        }
        // Time from tip distance; pure reorientations use the angle in
        // degrees as the "distance" so they don't run instantaneously.
        let dist = from
            .tip
            .distance(to.tip)
            .max(from.axis.angle_to(to.axis).to_degrees());
        let minutes = (dist / feed).max(1e-6);
        let vals = self.out_vals(q, None);
        self.pending.push(Pending {
            vals,
            minutes,
            feed,
        });
    }
}

/// Longest arc starting at point `i` of `pts` (positions; segment k joins
/// pts[k] and pts[k+1]): returns (end index, centre (x, y), counter-clockwise).
fn fit_arc(
    pts: &[&[f64]],
    i: usize,
    (x, y, z): (usize, usize, usize),
    rot: &[usize],
    (tol, sagitta): (f64, f64),
    pending: &[Pending],
    dialect: Dialect,
) -> Option<(usize, (f64, f64), bool)> {
    let n = pts.len() - 1;
    let same = |a: &[f64], b: &[f64]| {
        (a[z] - b[z]).abs() < 1e-9 && rot.iter().all(|&r| (a[r] - b[r]).abs() < 1e-9)
    };
    let mut best = None;
    let mut j = i + 3;
    while j <= n {
        if !same(pts[i], pts[j]) || !same(pts[i], pts[j - 1]) {
            break;
        }
        if dialect == Dialect::FanucTcp && (pending[j - 1].feed - pending[i].feed).abs() > 1e-9 {
            break;
        }
        match circle_fit(&pts[i..=j], x, y, tol, sagitta) {
            Some((c, ccw)) => best = Some((j, c, ccw)),
            None => break,
        }
        j += 1;
    }
    best
}

/// Circle through the first, middle and last points if every point is
/// within `tol` of it, no segment's chord is more than `sagitta` from it,
/// and the points sweep monotonically (< 270°).
fn circle_fit(
    p: &[&[f64]],
    x: usize,
    y: usize,
    tol: f64,
    sagitta: f64,
) -> Option<((f64, f64), bool)> {
    let (a, b, c) = (p[0], p[p.len() / 2], p[p.len() - 1]);
    let (ax, ay, bx, by, cx, cy) = (a[x], a[y], b[x], b[y], c[x], c[y]);
    let d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if d.abs() < 1e-12 {
        return None;
    }
    let (a2, b2, c2) = (ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy);
    let ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
    let uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
    let r = (ax - ux).hypot(ay - uy);
    if !(0.05..=5000.0).contains(&r) {
        return None;
    }
    let mut sweep = 0.0;
    let mut sign = 0.0;
    for w in p.windows(2) {
        let (p0, p1) = (w[0], w[1]);
        if ((p1[x] - ux).hypot(p1[y] - uy) - r).abs() > tol {
            return None;
        }
        // The arc bulges from each chord by its sagitta.
        let (mx, my) = ((p0[x] + p1[x]) * 0.5, (p0[y] + p1[y]) * 0.5);
        if ((mx - ux).hypot(my - uy) - r).abs() > sagitta {
            return None;
        }
        let a0 = (p0[y] - uy).atan2(p0[x] - ux);
        let a1 = (p1[y] - uy).atan2(p1[x] - ux);
        let mut da = a1 - a0;
        if da > std::f64::consts::PI {
            da -= std::f64::consts::TAU;
        } else if da < -std::f64::consts::PI {
            da += std::f64::consts::TAU;
        }
        if da.abs() < 1e-12 {
            return None;
        }
        if sign == 0.0 {
            sign = da.signum();
        } else if da.signum() != sign {
            return None;
        }
        sweep += da.abs();
    }
    (sweep < 1.5 * std::f64::consts::PI).then_some(((ux, uy), sign > 0.0))
}

fn sanitize(s: &str) -> String {
    s.chars()
        .filter(|c| *c != '(' && *c != ')' && *c != ';')
        .collect()
}

/// Fixed-point number without trailing zeros (and no "-0").
pub fn fmt(v: f64, decimals: usize) -> String {
    let mut s = format!("{:.*}", decimals, v);
    if s.contains('.') {
        s = s.trim_end_matches('0').trim_end_matches('.').to_string();
    }
    if s == "-0" {
        s = "0".into();
    }
    s
}
