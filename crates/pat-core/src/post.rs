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
                self.last[i] = Some(txt);
            }
        }
        s
    }

    fn rapid(&mut self, q: &[f64], pose: &Pose) {
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
        let w = self.words(q, Some(pose.tip));
        if !w.is_empty() {
            self.line(&format!("G1{w} F{}", fmt(feed, 1)));
        }
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
        let w = self.words(q, None);
        if !w.is_empty() {
            self.line(&format!("G1{w} F{}", fmt(1.0 / minutes, 3)));
        }
    }
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
