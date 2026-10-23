//! G-code simulation and checking.
//!
//! Replays a program on a machine model the way the control would move the
//! axes, sampling each move, and reports:
//!
//! - axis soft-limit violations, including mid-move overtravel in TCP mode;
//! - moves that pass close to the kinematic singularity while the primary
//!   rotary swings;
//! - rotary flips (large rotary swings inside one cutting move);
//! - non-TCP moves whose tool tip strays from a straight line, i.e. the post
//!   did not linearise enough;
//! - feed problems (no feed set, missing F in inverse-time mode);
//! - unsupported or unknown codes.
//!
//! It also estimates cycle time and records axis travel ranges. The output is
//! plain serialisable data so a UI or an AI agent can consume it directly.

use crate::gcode::parse_program;
use crate::interp::{
    interpret, ArcCenter, Coords, Event, Feed, Motion, MotionKind, Plane, Severity,
};
use crate::kinematics::{KinError, Kinematics};
use crate::machine::MachineDef;
use crate::math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashSet};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct SimOptions {
    /// Tool lengths by tool number (mm).
    pub tool_lengths: BTreeMap<u32, f64>,
    /// Tool length before the first tool change, and for unknown tools.
    /// Defaults to the machine's `default_tool_length`.
    pub tool_length: Option<f64>,
    /// Allowed tool-tip deviation from a straight line on non-TCP moves (mm).
    pub tip_tolerance: f64,
    /// Rotary sampling step (degrees).
    pub sample_deg: f64,
    /// Cutting moves with a rotary swing larger than this are flagged (degrees).
    pub flip_threshold_deg: f64,
    /// Chord tolerance used to break arcs into lines (mm).
    pub arc_tolerance: f64,
    /// Record the machine state at the end of every move.
    pub record_trace: bool,
    /// Stop recording issues after this many.
    pub max_issues: usize,
}

impl Default for SimOptions {
    fn default() -> Self {
        SimOptions {
            tool_lengths: BTreeMap::new(),
            tool_length: None,
            tip_tolerance: 0.01,
            sample_deg: 0.5,
            flip_threshold_deg: 90.0,
            arc_tolerance: 0.002,
            record_trace: false,
            max_issues: 500,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Issue {
    pub line: usize,
    pub severity: Severity,
    pub code: String,
    pub message: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AxisRange {
    pub axis: String,
    pub min: f64,
    pub max: f64,
    pub limit_min: Option<f64>,
    pub limit_max: Option<f64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TracePoint {
    pub line: usize,
    pub rapid: bool,
    /// Machine joint values.
    pub joints: Vec<f64>,
    pub tip: Vec3,
    pub axis: Vec3,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Report {
    pub machine: String,
    /// No errors were found.
    pub ok: bool,
    pub errors: usize,
    pub warnings: usize,
    pub issues: Vec<Issue>,
    pub moves: usize,
    pub cutting_time_s: f64,
    pub rapid_time_s: f64,
    pub dwell_time_s: f64,
    pub cutting_length_mm: f64,
    pub axis_ranges: Vec<AxisRange>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trace: Option<Vec<TracePoint>>,
}

/// Fallback rapid rates when the machine file gives none.
const DEFAULT_RAPID_MM: f64 = 10_000.0;
const DEFAULT_RAPID_DEG: f64 = 3_600.0;

pub fn simulate(def: &MachineDef, program: &str, opts: &SimOptions) -> Report {
    let mut sim = Sim::new(def, opts);
    match parse_program(program) {
        Err(e) => sim.issue(e.line, Severity::Error, "parse", e.msg),
        Ok(blocks) => {
            let names: Vec<String> = def.axes.iter().map(|a| a.name.clone()).collect();
            for ev in interpret(&blocks, &names) {
                sim.event(ev);
            }
        }
    }
    sim.finish()
}

struct Sim<'a> {
    def: &'a MachineDef,
    opts: &'a SimOptions,
    kin: Option<Kinematics>,
    q: Vec<f64>,
    issues: Vec<Issue>,
    seen: HashSet<(usize, String)>,
    dropped: usize,
    moves: usize,
    cut_s: f64,
    rapid_s: f64,
    dwell_s: f64,
    cut_len: f64,
    lo: Vec<f64>,
    hi: Vec<f64>,
    trace: Option<Vec<TracePoint>>,
}

impl<'a> Sim<'a> {
    fn new(def: &'a MachineDef, opts: &'a SimOptions) -> Self {
        let n = def.axes.len();
        let mut s = Sim {
            def,
            opts,
            kin: None,
            q: vec![0.0; n],
            issues: Vec::new(),
            seen: HashSet::new(),
            dropped: 0,
            moves: 0,
            cut_s: 0.0,
            rapid_s: 0.0,
            dwell_s: 0.0,
            cut_len: 0.0,
            lo: vec![f64::INFINITY; n],
            hi: vec![f64::NEG_INFINITY; n],
            trace: opts.record_trace.then(Vec::new),
        };
        let len = opts.tool_length.unwrap_or(def.default_tool_length);
        s.set_tool_length(0, len);
        s
    }

    fn set_tool_length(&mut self, line: usize, len: f64) {
        match Kinematics::new(self.def.clone(), len) {
            Ok(k) => self.kin = Some(k),
            Err(e) => self.issue(line, Severity::Error, "kinematics", e.to_string()),
        }
    }

    fn issue(&mut self, line: usize, severity: Severity, code: &str, message: String) {
        // One issue per (line, code) keeps reports readable.
        if !self.seen.insert((line, code.to_string())) {
            return;
        }
        if self.issues.len() >= self.opts.max_issues {
            self.dropped += 1;
            return;
        }
        self.issues.push(Issue {
            line,
            severity,
            code: code.to_string(),
            message,
        });
    }

    fn event(&mut self, ev: Event) {
        match ev {
            Event::Note {
                line,
                severity,
                code,
                message,
            } => self.issue(line, severity, code, message),
            Event::ToolChange { line, tool } => {
                let len = match self.opts.tool_lengths.get(&tool) {
                    Some(&l) => l,
                    None => {
                        let l = self
                            .opts
                            .tool_length
                            .unwrap_or(self.def.default_tool_length);
                        self.issue(
                            line,
                            Severity::Warning,
                            "tool_length",
                            format!("no length given for T{tool}; using {l} mm"),
                        );
                        l
                    }
                };
                self.set_tool_length(line, len);
            }
            Event::Dwell { seconds, .. } => self.dwell_s += seconds,
            Event::End { .. } => {}
            Event::Motion(m) => self.motion(m),
        }
    }

    /// Current position expressed in the coordinates of `mode`.
    fn mode_coords(&self, kin: &Kinematics, mode: Coords, q: &[f64]) -> Vec<f64> {
        match mode {
            Coords::Machine => q.to_vec(),
            Coords::Program => kin.to_program(q),
            Coords::Tcp => {
                let tip = kin.forward(q).tip;
                let mut m = q.to_vec();
                for (k, &i) in kin.linear_axes().iter().enumerate() {
                    m[i] = tcp_component(&self.def.axes[i].name, tip, k);
                }
                m
            }
        }
    }

    /// Machine joints for a point given in `mode` coordinates.
    fn mode_to_joints(
        &self,
        kin: &Kinematics,
        mode: Coords,
        m: &[f64],
    ) -> Result<Vec<f64>, KinError> {
        match mode {
            Coords::Machine => Ok(m.to_vec()),
            Coords::Program => Ok(kin.to_machine(m)),
            Coords::Tcp => {
                let mut tip = Vec3::ZERO;
                for &i in &kin.linear_axes() {
                    match self.def.axes[i].name.as_str() {
                        "X" => tip.x = m[i],
                        "Y" => tip.y = m[i],
                        _ => tip.z = m[i],
                    }
                }
                kin.solve_linear(m, tip).ok_or(KinError::LinearSingular)
            }
        }
    }

    fn motion(&mut self, m: Motion) {
        let Some(kin) = self.kin.clone() else { return };
        let start = self.mode_coords(&kin, m.coords, &self.q);

        let mut end = start.clone();
        if m.kind == MotionKind::Home {
            // G28: listed axes return to machine zero.
            for (i, v) in m.axes.iter().enumerate() {
                if v.is_some() {
                    end[i] = 0.0;
                }
            }
        } else {
            for (i, v) in m.axes.iter().enumerate() {
                if let Some(v) = v {
                    end[i] = if m.incremental { start[i] + v } else { *v };
                }
            }
        }

        let points: Vec<Vec<f64>> = match &m.kind {
            MotionKind::Arc {
                clockwise,
                plane,
                center,
            } => {
                match arc_points(
                    &kin,
                    self.def,
                    &start,
                    &end,
                    *clockwise,
                    *plane,
                    *center,
                    self.opts.arc_tolerance,
                ) {
                    Ok(p) => p,
                    Err(msg) => {
                        self.issue(m.line, Severity::Error, "arc", msg);
                        vec![end]
                    }
                }
            }
            _ => vec![end],
        };

        let rapid = matches!(m.kind, MotionKind::Rapid | MotionKind::Home);
        // Inverse-time feed covers the whole block; share it across arc segments.
        let segs = points.len() as f64;
        let mut prev_m = start;
        for p in points {
            self.segment(&kin, &m, rapid, &prev_m, &p, segs);
            prev_m = p;
        }
        self.moves += 1;
    }

    fn segment(
        &mut self,
        kin: &Kinematics,
        m: &Motion,
        rapid: bool,
        a: &[f64],
        b: &[f64],
        segs: f64,
    ) {
        let line = m.line;
        let qa = self.q.clone();
        let qb = match self.mode_to_joints(kin, m.coords, b) {
            Ok(q) => q,
            Err(e) => {
                self.issue(line, Severity::Error, "kinematics", e.to_string());
                return;
            }
        };

        let rot = kin.rotary_axes();
        let max_rot = rot
            .iter()
            .map(|&i| (qb[i] - qa[i]).abs())
            .fold(0.0, f64::max);
        let n = if max_rot > 1e-9 {
            (max_rot / self.opts.sample_deg).ceil().max(2.0) as usize
        } else {
            1
        };

        let tip_a = kin.forward(&qa).tip;
        let tip_b = kin.forward(&qb).tip;
        let mut max_dev: f64 = 0.0;
        let mut min_sing = f64::INFINITY;
        let mut tip_len = 0.0;
        let mut prev_tip = tip_a;

        for k in 1..=n {
            let t = k as f64 / n as f64;
            let q = if k == n {
                qb.clone()
            } else {
                match m.coords {
                    Coords::Tcp => {
                        let mt: Vec<f64> = a.iter().zip(b).map(|(x, y)| x + (y - x) * t).collect();
                        match self.mode_to_joints(kin, m.coords, &mt) {
                            Ok(q) => q,
                            Err(e) => {
                                self.issue(line, Severity::Error, "kinematics", e.to_string());
                                return;
                            }
                        }
                    }
                    _ => qa.iter().zip(&qb).map(|(x, y)| x + (y - x) * t).collect(),
                }
            };
            self.check_limits(line, &q);
            let pose = kin.forward(&q);
            if let Some(s) = kin.singularity_angle_deg(pose.axis) {
                min_sing = min_sing.min(s);
            }
            if m.coords != Coords::Tcp {
                max_dev = max_dev.max(point_segment_distance(pose.tip, tip_a, tip_b));
            }
            tip_len += pose.tip.distance(prev_tip);
            prev_tip = pose.tip;
        }
        if n == 1 {
            self.check_limits(line, &qa);
        }

        if !rapid {
            if max_dev > self.opts.tip_tolerance {
                self.issue(
                    line,
                    Severity::Warning,
                    "tip_deviation",
                    format!(
                        "tool tip strays {max_dev:.3} mm from a straight line during this rotary move; \
                         linearise more finely or use TCP"
                    ),
                );
            }
            if let Some(p) = kin.primary_rotary() {
                let swing = (qb[p] - qa[p]).abs();
                if min_sing < self.def.singularity_tolerance_deg && swing > 1.0 {
                    self.issue(
                        line,
                        Severity::Warning,
                        "near_singularity",
                        format!(
                            "tool axis passes within {min_sing:.2}° of the singular direction while {} swings {swing:.1}°",
                            self.def.axes[p].name
                        ),
                    );
                }
            }
            for &i in rot {
                let d = (qb[i] - qa[i]).abs();
                if d > self.opts.flip_threshold_deg {
                    self.issue(
                        line,
                        Severity::Warning,
                        "rotary_flip",
                        format!(
                            "{} swings {d:.1}° during a cutting move",
                            self.def.axes[i].name
                        ),
                    );
                }
            }
        }

        // Time.
        if rapid {
            let t = (0..qa.len())
                .map(|i| {
                    let ax = &self.def.axes[i];
                    let rate = ax.max_rate.unwrap_or(if ax.is_rotary() {
                        DEFAULT_RAPID_DEG
                    } else {
                        DEFAULT_RAPID_MM
                    });
                    (qb[i] - qa[i]).abs() / rate
                })
                .fold(0.0, f64::max);
            self.rapid_s += t * 60.0;
        } else {
            let minutes = match m.feed {
                Feed::InverseTime(f) if f > 0.0 => 1.0 / f / segs,
                Feed::PerMinute(f) => {
                    let lin: f64 = kin
                        .linear_axes()
                        .iter()
                        .map(|&i| (b[i] - a[i]).powi(2))
                        .sum::<f64>()
                        .sqrt();
                    let dist = if lin > 1e-9 {
                        lin
                    } else {
                        rot.iter()
                            .map(|&i| (b[i] - a[i]).powi(2))
                            .sum::<f64>()
                            .sqrt()
                    };
                    dist / f
                }
                _ => 0.0,
            };
            self.cut_s += minutes * 60.0;
            self.cut_len += tip_len;
        }

        if let Some(tr) = self.trace.as_mut() {
            let pose = kin.forward(&qb);
            tr.push(TracePoint {
                line,
                rapid,
                joints: qb.clone(),
                tip: pose.tip,
                axis: pose.axis,
            });
        }
        self.q = qb;
    }

    fn check_limits(&mut self, line: usize, q: &[f64]) {
        for (i, &v) in q.iter().enumerate() {
            self.lo[i] = self.lo[i].min(v);
            self.hi[i] = self.hi[i].max(v);
        }
        let Some(kin) = &self.kin else { return };
        for (i, v) in kin.limit_violations(q) {
            let ax = &self.def.axes[i];
            let (bound, word) = if ax.min.is_some_and(|m| v < m) {
                (ax.min, "min")
            } else {
                (ax.max, "max")
            };
            self.issue(
                line,
                Severity::Error,
                "axis_limit",
                format!(
                    "{} = {v:.3} is beyond its {word} limit {:.3}",
                    ax.name,
                    bound.unwrap_or(f64::NAN)
                ),
            );
        }
    }

    fn finish(mut self) -> Report {
        if self.dropped > 0 {
            self.issues.push(Issue {
                line: 0,
                severity: Severity::Info,
                code: "truncated".into(),
                message: format!("{} more issues not shown", self.dropped),
            });
        }
        let errors = self
            .issues
            .iter()
            .filter(|i| i.severity == Severity::Error)
            .count();
        let warnings = self
            .issues
            .iter()
            .filter(|i| i.severity == Severity::Warning)
            .count();
        let axis_ranges = self
            .def
            .axes
            .iter()
            .enumerate()
            .filter(|(i, _)| self.lo[*i].is_finite())
            .map(|(i, a)| AxisRange {
                axis: a.name.clone(),
                min: self.lo[i],
                max: self.hi[i],
                limit_min: a.min,
                limit_max: a.max,
            })
            .collect();
        Report {
            machine: self.def.name.clone(),
            ok: errors == 0,
            errors,
            warnings,
            issues: self.issues,
            moves: self.moves,
            cutting_time_s: self.cut_s,
            rapid_time_s: self.rapid_s,
            dwell_time_s: self.dwell_s,
            cutting_length_mm: self.cut_len,
            axis_ranges,
            trace: self.trace,
        }
    }
}

fn tcp_component(name: &str, tip: Vec3, fallback: usize) -> f64 {
    match name {
        "X" => tip.x,
        "Y" => tip.y,
        "Z" => tip.z,
        _ => tip.get(fallback),
    }
}

fn point_segment_distance(p: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b - a;
    let l2 = ab.dot(ab);
    if l2 < 1e-18 {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / l2).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// Break an arc into straight segments (in the block's coordinate mode).
#[allow(clippy::too_many_arguments)]
fn arc_points(
    kin: &Kinematics,
    def: &MachineDef,
    start: &[f64],
    end: &[f64],
    clockwise: bool,
    plane: Plane,
    center: ArcCenter,
    tol: f64,
) -> Result<Vec<Vec<f64>>, String> {
    let idx = |n: &str| {
        def.axis_index(n)
            .ok_or_else(|| format!("arc needs a {n} axis"))
    };
    let (u, v, w) = match plane {
        Plane::XY => (idx("X")?, idx("Y")?, idx("Z")?),
        Plane::XZ => (idx("Z")?, idx("X")?, idx("Y")?),
        Plane::YZ => (idx("Y")?, idx("Z")?, idx("X")?),
    };
    for &i in kin.rotary_axes() {
        if (end[i] - start[i]).abs() > 1e-9 {
            return Err("arcs with rotary motion are not supported".into());
        }
    }
    let (su, sv, eu, ev) = (start[u], start[v], end[u], end[v]);
    // G18 is conventionally viewed from +Y, which reverses the sense.
    let cw = if plane == Plane::XZ {
        !clockwise
    } else {
        clockwise
    };
    let (cu, cv) = match center {
        ArcCenter::Offsets(o) => {
            let off = |i: usize| match def.axes[i].name.as_str() {
                "X" => o.x,
                "Y" => o.y,
                _ => o.z,
            };
            (su + off(u), sv + off(v))
        }
        ArcCenter::Radius(r) => {
            let (dx, dy) = (eu - su, ev - sv);
            let d = (dx * dx + dy * dy).sqrt();
            if d < 1e-9 || d > 2.0 * r.abs() + 1e-6 {
                return Err(format!("R{r} cannot connect these end points"));
            }
            let h = (r * r - d * d / 4.0).max(0.0).sqrt();
            let (mx, my) = (su + dx / 2.0, sv + dy / 2.0);
            // Positive R takes the short arc; the centre side depends on direction.
            let side = if (r > 0.0) == cw { 1.0 } else { -1.0 };
            (mx + side * h * dy / d, my - side * h * dx / d)
        }
    };
    let r0 = ((su - cu).powi(2) + (sv - cv).powi(2)).sqrt();
    let r1 = ((eu - cu).powi(2) + (ev - cv).powi(2)).sqrt();
    if (r0 - r1).abs() > 0.01_f64.max(r0 * 1e-3) {
        return Err(format!("arc radius mismatch: start {r0:.4}, end {r1:.4}"));
    }
    let a0 = (sv - cv).atan2(su - cu);
    let a1 = (ev - cv).atan2(eu - cu);
    let tau = std::f64::consts::TAU;
    let mut sweep = if cw {
        (a0 - a1).rem_euclid(tau)
    } else {
        (a1 - a0).rem_euclid(tau)
    };
    if sweep < 1e-9 {
        sweep = tau; // full circle
    }
    let step = if r0 > tol {
        2.0 * (1.0 - tol / r0).acos()
    } else {
        sweep
    };
    let n = ((sweep / step).ceil() as usize).clamp(1, 10_000);
    let mut pts = Vec::with_capacity(n);
    for k in 1..=n {
        let t = k as f64 / n as f64;
        if k == n {
            pts.push(end.to_vec());
            break;
        }
        let ang = if cw { a0 - sweep * t } else { a0 + sweep * t };
        let mut p = start.to_vec();
        p[u] = cu + r0 * ang.cos();
        p[v] = cv + r0 * ang.sin();
        p[w] = start[w] + (end[w] - start[w]) * t;
        pts.push(p);
    }
    Ok(pts)
}
