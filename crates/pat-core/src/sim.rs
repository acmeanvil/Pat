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
//! - G68.2 moves made without aligning the tool (missing G53.1);
//! - undefined work offsets and subroutines;
//! - unsupported or unknown codes.
//!
//! Subroutines are expanded first ([`crate::subs`]); canned cycles, work
//! offsets and tilted planes are executed as the control would.
//!
//! It also estimates cycle time (acceleration-aware when the machine file has
//! `max_accel` for every axis, see [`crate::planner`]) and records axis travel
//! ranges. The output is plain serialisable data so a UI or an AI agent can
//! consume it directly.

use crate::collision::{MoveClass, Role, World};
use crate::gcode::parse_program;
use crate::geom::{Aabb, Bvh};
use crate::interp::{
    interpret, ArcCenter, Coords, Cycle, Event, Feed, InterpOptions, Motion, MotionKind, Plane,
    Severity, TiltedPlane,
};
use crate::kinematics::Link;
use crate::kinematics::{KinError, Kinematics, Pose};
use crate::machine::MachineDef;
use crate::math::{Mat3, Vec3};
use crate::planner::{plan_time, PlanSeg};
use crate::stock::Voxels;
use crate::subs::expand;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashSet};
use std::sync::Arc;

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
    /// Work offsets (part zero in the base frame, like `wcs_origin`) that
    /// override or extend the machine file's, e.g. {"G55": [100, 0, 0]}.
    pub work_offsets: BTreeMap<String, Vec3>,
    /// Read `P` dwell words as milliseconds (Fanuc / Haas).
    pub dwell_p_milliseconds: bool,
    /// Tool, part, fixture and machine geometry. When present, every move
    /// is checked for collisions.
    #[serde(skip)]
    pub world: Option<Arc<World>>,
    /// Tool number in the spindle at program start (for collision checks
    /// before the first M6).
    pub initial_tool: Option<u32>,
    /// Collision sampling step along linear motion (mm).
    pub collision_step_mm: f64,
    /// Simulate material removal on the scene's stock bodies (needs a world
    /// with stock and tool cutter geometry): reports rapids into stock,
    /// holders rubbing stock, gouges into the part, and removed volume.
    pub simulate_stock: bool,
    /// Voxel size for stock simulation (mm).
    pub stock_resolution: f64,
    /// Cuts into the part shallower than this are not gouges (mm).
    pub gouge_tolerance: f64,
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
            work_offsets: BTreeMap::new(),
            dwell_p_milliseconds: false,
            world: None,
            initial_tool: None,
            collision_step_mm: 1.0,
            simulate_stock: false,
            stock_resolution: 0.5,
            gouge_tolerance: 0.05,
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
    /// Material-removal results when stock simulation ran.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stock: Option<StockReport>,
    /// Best available total time estimate (seconds).
    pub cycle_time_s: f64,
    /// "acceleration" (look-ahead planner) or "feed-rate" (no acceleration
    /// limits in the machine file).
    pub cycle_time_model: String,
    pub axis_ranges: Vec<AxisRange>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trace: Option<Vec<TracePoint>>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StockReport {
    pub resolution_mm: f64,
    pub initial_mm3: f64,
    pub removed_mm3: f64,
    pub remaining_mm3: f64,
}

/// Fallback rapid rates when the machine file gives none.
const DEFAULT_RAPID_MM: f64 = 10_000.0;
const DEFAULT_RAPID_DEG: f64 = 3_600.0;

pub fn simulate(def: &MachineDef, program: &str, opts: &SimOptions) -> Report {
    let mut sim = Sim::new(def, opts);
    match parse_program(program) {
        Err(e) => sim.issue(e.line, Severity::Error, "parse", e.msg),
        Ok(blocks) => {
            let (blocks, errors) = expand(&blocks);
            for e in errors {
                sim.issue(e.line, Severity::Error, "subroutine", e.msg);
            }
            let names: Vec<String> = def.axes.iter().map(|a| a.name.clone()).collect();
            let iopts = InterpOptions {
                dwell_p_milliseconds: opts.dwell_p_milliseconds,
            };
            for ev in interpret(&blocks, &names, &iopts) {
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
    tool_length: f64,
    wcs: String,
    plan: Vec<PlanSeg>,
    stop_next: bool,
    cycle_initial: Option<f64>,
    tool: Option<u32>,
    stock: Option<Voxels>,
    part_core: Option<Voxels>,
    stock_initial: f64,
    /// Tool-tip travel since the last holder-vs-stock check.
    since_rub_check: f64,
    last_tip: Option<Vec3>,
    /// Cutter pose (table frame) at the previous stock sample.
    last_pose: Option<Pose>,
}

/// Rotation of a G68.2 frame: intrinsic Z-X-Z Euler angles.
fn tilt_rotation(p: &TiltedPlane) -> Mat3 {
    let e = p.euler_deg;
    Mat3::rotation(Vec3::Z, e.x.to_radians())
        .mul_mat(&Mat3::rotation(Vec3::X, e.y.to_radians()))
        .mul_mat(&Mat3::rotation(Vec3::Z, e.z.to_radians()))
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
            tool_length: opts
                .initial_tool
                .and_then(|t| opts.tool_lengths.get(&t).copied())
                .or(opts.tool_length)
                .or_else(|| {
                    let t = opts.initial_tool?;
                    Some(opts.world.as_ref()?.tool(t)?.def.gauge_length)
                })
                .unwrap_or(def.default_tool_length),
            wcs: "G54".into(),
            plan: Vec::new(),
            stop_next: true,
            cycle_initial: None,
            tool: opts.initial_tool,
            stock: None,
            part_core: None,
            stock_initial: 0.0,
            since_rub_check: f64::INFINITY,
            last_tip: None,
            last_pose: None,
        };
        s.init_stock();
        s.rebuild_kinematics(0);
        s
    }

    fn work_offset(&self, name: &str) -> Option<Vec3> {
        let key = name.to_ascii_uppercase();
        self.opts
            .work_offsets
            .iter()
            .find(|(k, _)| k.to_ascii_uppercase() == key)
            .map(|(_, v)| *v)
            .or_else(|| self.def.work_offset(name))
    }

    /// Rebuild kinematics for the current tool length and work offset.
    fn rebuild_kinematics(&mut self, line: usize) {
        let origin = match self.work_offset(&self.wcs) {
            Some(o) => o,
            None => {
                let name = self.wcs.clone();
                self.issue(
                    line,
                    Severity::Error,
                    "work_offset",
                    format!("work offset {name} is not defined in the machine file or options; using G54"),
                );
                self.def.wcs_origin
            }
        };
        match Kinematics::new(self.def.with_wcs(origin), self.tool_length) {
            Ok(k) => self.kin = Some(k),
            Err(e) => self.issue(line, Severity::Error, "kinematics", e.to_string()),
        }
    }

    fn set_tool_length(&mut self, line: usize, len: f64) {
        self.tool_length = len;
        self.rebuild_kinematics(line);
    }

    fn select_wcs(&mut self, line: usize, wcs: &str) {
        if wcs != self.wcs {
            self.wcs = wcs.to_string();
            self.rebuild_kinematics(line);
        }
    }

    fn issue(&mut self, line: usize, severity: Severity, code: &str, message: String) {
        self.issue_keyed(line, severity, code, code, message);
    }

    /// Like [`Self::issue`], deduplicated on (line, `key`) instead of
    /// (line, code), so distinct collisions on one line are all reported.
    fn issue_keyed(
        &mut self,
        line: usize,
        severity: Severity,
        code: &str,
        key: &str,
        message: String,
    ) {
        // One issue per (line, key) keeps reports readable.
        if !self.seen.insert((line, key.to_string())) {
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
                self.tool = Some(tool);
                self.last_pose = None;
                let from_world = self
                    .opts
                    .world
                    .as_ref()
                    .and_then(|w| w.tool(tool))
                    .map(|t| t.def.gauge_length);
                if self.opts.world.is_some() && from_world.is_none() {
                    self.issue(
                        line,
                        Severity::Warning,
                        "tool_geometry",
                        format!(
                            "T{tool} has no geometry in the scene; it is not collision checked"
                        ),
                    );
                }
                let len = match self.opts.tool_lengths.get(&tool).copied().or(from_world) {
                    Some(l) => l,
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
            Event::Dwell { seconds, .. } => {
                self.dwell_s += seconds;
                self.stop_next = true;
            }
            Event::Stop { .. } | Event::End { .. } => self.stop_next = true,
            Event::Motion(m) => {
                self.select_wcs(m.line, &m.wcs.clone());
                self.motion(m);
            }
            Event::Cycle(c) => {
                self.select_wcs(c.line, &c.wcs.clone());
                self.cycle(c);
            }
            Event::AlignToolAxis { line, plane } => self.align_tool(line, &plane),
        }
    }

    /// G53.1: turn the rotaries so the tool axis matches the plane normal.
    fn align_tool(&mut self, line: usize, plane: &TiltedPlane) {
        let Some(kin) = self.kin.clone() else { return };
        let want = tilt_rotation(plane).mul_vec(Vec3::Z);
        let tip = kin.forward(&self.q).tip;
        match kin.inverse(&Pose { tip, axis: want }, Some(&self.q)) {
            Ok(sol) => {
                let mut axes = vec![None; self.q.len()];
                for &i in kin.rotary_axes() {
                    axes[i] = Some(sol.joints[i]);
                }
                let m = Motion {
                    line,
                    kind: MotionKind::Rapid,
                    axes,
                    incremental: false,
                    coords: Coords::Machine,
                    wcs: self.wcs.clone(),
                    feed: Feed::Missing,
                    exact_stop: true,
                };
                self.motion(m);
            }
            Err(e) => self.issue(
                line,
                Severity::Error,
                "tilted_plane",
                format!("G53.1 cannot align the tool with the plane: {e}"),
            ),
        }
    }

    /// Current position expressed in the coordinates of `mode`.
    fn mode_coords(&self, kin: &Kinematics, mode: Coords, q: &[f64]) -> Vec<f64> {
        match mode {
            Coords::Machine => q.to_vec(),
            Coords::Program => kin.to_program(q),
            Coords::Tcp | Coords::Tilted(_) => {
                let mut tip = kin.forward(q).tip;
                if let Coords::Tilted(p) = mode {
                    tip = tilt_rotation(&p).transpose().mul_vec(tip - p.origin);
                }
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
            Coords::Tcp | Coords::Tilted(_) => {
                let mut tip = Vec3::ZERO;
                for &i in &kin.linear_axes() {
                    match self.def.axes[i].name.as_str() {
                        "X" => tip.x = m[i],
                        "Y" => tip.y = m[i],
                        _ => tip.z = m[i],
                    }
                }
                if let Coords::Tilted(p) = mode {
                    tip = tilt_rotation(&p).mul_vec(tip) + p.origin;
                }
                kin.solve_linear(m, tip).ok_or(KinError::LinearSingular)
            }
        }
    }

    fn motion(&mut self, m: Motion) {
        let Some(kin) = self.kin.clone() else { return };
        if let Coords::Tilted(p) = m.coords {
            let normal = tilt_rotation(&p).mul_vec(Vec3::Z);
            let off = kin.forward(&self.q).axis.angle_to(normal).to_degrees();
            if off > 0.01 {
                self.issue(
                    m.line,
                    Severity::Warning,
                    "tilted_plane",
                    format!("tool axis is {off:.2}° off the G68.2 plane normal; missing G53.1?"),
                );
            }
        }
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
        if m.exact_stop {
            self.stop_next = true;
        }
    }

    /// Execute a drilling cycle as the control would, in the block's
    /// coordinate mode (so it also drills along a G68.2 plane's Z).
    fn cycle(&mut self, c: Cycle) {
        let Some(kin) = self.kin.clone() else { return };
        let Some(zi) = self.def.axis_index("Z") else {
            return;
        };
        let here = self.mode_coords(&kin, c.coords, &self.q);
        if c.first || self.cycle_initial.is_none() {
            self.cycle_initial = Some(here[zi]);
        }
        let initial = self.cycle_initial.unwrap_or(here[zi]);
        let r_level = if c.incremental { initial + c.r } else { c.r };
        let bottom = if c.incremental { r_level + c.z } else { c.z };
        if bottom > r_level {
            self.issue(
                c.line,
                Severity::Warning,
                "canned_cycle",
                "hole bottom is above the R plane".into(),
            );
        }
        let clear = if c.retract_to_r {
            r_level
        } else {
            initial.max(r_level)
        };
        let n_axes = self.q.len();

        let mv = |sim: &mut Self, kind: MotionKind, target: Vec<Option<f64>>| {
            let m = Motion {
                line: c.line,
                feed: if kind == MotionKind::Rapid {
                    Feed::Missing
                } else {
                    c.feed
                },
                kind,
                axes: target,
                incremental: false,
                coords: c.coords,
                wcs: c.wcs.clone(),
                exact_stop: true,
            };
            sim.motion(m);
        };
        let z_to = |z: f64| {
            let mut a = vec![None; n_axes];
            a[zi] = Some(z);
            a
        };
        let dwell = |sim: &mut Self, s: f64| {
            sim.dwell_s += s;
            sim.stop_next = true;
        };

        for _ in 0..c.repeats.max(1) {
            let cur = self.mode_coords(&kin, c.coords, &self.q);
            if cur[zi] < r_level {
                mv(self, MotionKind::Rapid, z_to(r_level));
            }
            let cur = self.mode_coords(&kin, c.coords, &self.q);
            let mut pos = vec![None; n_axes];
            for (i, v) in c.axes.iter().enumerate() {
                if let Some(v) = v {
                    pos[i] = Some(if c.incremental { cur[i] + v } else { *v });
                }
            }
            if pos.iter().any(|p| p.is_some()) {
                mv(self, MotionKind::Rapid, pos);
            }
            mv(self, MotionKind::Rapid, z_to(r_level));
            match c.kind {
                73 | 83 => {
                    let q = c.q.unwrap_or(r_level - bottom).max(1e-3);
                    let clearance = 0.254_f64.min(q * 0.5);
                    let mut depth = r_level;
                    while depth > bottom + 1e-9 {
                        let next = (depth - q).max(bottom);
                        if c.kind == 83 && depth < r_level {
                            mv(self, MotionKind::Rapid, z_to(depth + clearance));
                        }
                        mv(self, MotionKind::Linear, z_to(next));
                        if next > bottom + 1e-9 {
                            if c.kind == 83 {
                                mv(self, MotionKind::Rapid, z_to(r_level));
                            } else {
                                mv(self, MotionKind::Rapid, z_to(next + clearance));
                            }
                        }
                        depth = next;
                    }
                }
                _ => mv(self, MotionKind::Linear, z_to(bottom)),
            }
            match c.kind {
                82 | 88 | 89 => dwell(self, c.dwell_s),
                84 => dwell(self, c.dwell_s),
                _ => {}
            }
            match c.kind {
                84 | 85 | 89 => mv(self, MotionKind::Linear, z_to(r_level)),
                86 | 88 => self.stop_next = true,
                _ => {}
            }
            if c.kind == 88 {
                self.issue(
                    c.line,
                    Severity::Info,
                    "canned_cycle",
                    "G88 manual retract simulated as a rapid".into(),
                );
            }
            let cur = self.mode_coords(&kin, c.coords, &self.q);
            if (cur[zi] - clear).abs() > 1e-9 {
                mv(self, MotionKind::Rapid, z_to(clear));
            }
        }
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
        let mut n = if max_rot > 1e-9 {
            (max_rot / self.opts.sample_deg).ceil().max(2.0) as usize
        } else {
            1
        };
        if self.opts.world.is_some() {
            let max_lin = kin
                .linear_axes()
                .iter()
                .map(|&i| (qb[i] - qa[i]).abs())
                .fold(0.0, f64::max);
            // Stock removal sweeps exactly between samples, so the
            // collision step is fine-grained enough for it too.
            let step = self.opts.collision_step_mm.max(0.01);
            n = n.max((max_lin / step).ceil() as usize).clamp(1, 100_000);
        }

        let tip_a = kin.forward(&qa).tip;
        let tip_b = kin.forward(&qb).tip;
        let mut max_dev: f64 = 0.0;
        let mut min_sing = f64::INFINITY;
        let mut tip_len = 0.0;
        let mut prev_tip = tip_a;
        let mut samples: Vec<Vec<f64>> = Vec::with_capacity(n);

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
            self.check_collisions(kin, line, rapid, &q);
            self.stock_step(kin, line, rapid, &q);
            samples.push(q.clone());
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
        let seg_s = if rapid {
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
            t * 60.0
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
            minutes * 60.0
        };

        // Joint-space pieces for the cycle-time planner.
        let mut prev_q = qa.clone();
        for q in &samples {
            let dq: Vec<f64> = q.iter().zip(&prev_q).map(|(b, a)| b - a).collect();
            let stop_before = std::mem::take(&mut self.stop_next);
            self.plan.push(PlanSeg {
                dq,
                nominal_s: seg_s / samples.len() as f64,
                stop_before,
            });
            prev_q = q.clone();
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

    fn init_stock(&mut self) {
        if !self.opts.simulate_stock {
            return;
        }
        let Some(world) = self.opts.world.clone() else {
            self.issue(
                0,
                Severity::Warning,
                "stock",
                "stock simulation needs a scene".into(),
            );
            return;
        };
        let stocks = world.table_meshes(Role::Stock);
        if stocks.is_empty() {
            self.issue(
                0,
                Severity::Warning,
                "stock",
                "stock simulation needs a body with role = \"stock\"".into(),
            );
            return;
        }
        let res = self.opts.stock_resolution.max(0.05);
        let bounds = stocks
            .iter()
            .fold(Aabb::EMPTY, |b, (_, m)| b.union(m.aabb()))
            .expand(res);
        let mut v = Voxels::empty(bounds.min, bounds.max, res);
        for (_, m) in &stocks {
            v.fill_mesh(m, &Bvh::build(m));
        }
        self.stock_initial = v.volume();
        self.stock = Some(v);
        let parts = world.table_meshes(Role::Part);
        if !parts.is_empty() {
            let pb = parts
                .iter()
                .fold(Aabb::EMPTY, |b, (_, m)| b.union(m.aabb()))
                .expand(res);
            let mut p = Voxels::empty(pb.min, pb.max, res);
            for (_, m) in &parts {
                p.fill_mesh(m, &Bvh::build(m));
            }
            let k = ((self.opts.gouge_tolerance / res).ceil() as usize).max(1);
            self.part_core = Some(p.eroded(k));
        }
    }

    /// Material removal, gouge and rubbing checks at one sampled position.
    fn stock_step(&mut self, kin: &Kinematics, line: usize, rapid: bool, q: &[f64]) {
        if self.stock.is_none() {
            return;
        }
        let Some(world) = self.opts.world.clone() else {
            return;
        };
        let Some(asm) = self.tool.and_then(|t| world.tool(t)) else {
            return;
        };
        let Some(shape) = asm.cutter_shape() else {
            return;
        };
        let table = kin.link("table").unwrap_or(Link::Base);
        let frame = kin
            .link_iso(table, q)
            .inverse()
            .then(&kin.link_iso(Link::Tool, q));
        let tag = format!("T{}", asm.def.number);

        let cur = Pose {
            tip: frame.t,
            axis: frame.apply_vec(Vec3::Z),
        };
        let from = self.last_pose.unwrap_or(cur);
        self.last_pose = Some(cur);
        if let Some(core) = &self.part_core {
            if let Some(at) = core.sweep_hits(&from, &cur, &shape) {
                let at = at - kin.machine().wcs_origin;
                let msg = format!(
                    "{tag} cutter cuts into the part near X{:.2} Y{:.2} Z{:.2}",
                    at.x, at.y, at.z
                );
                self.issue(line, Severity::Error, "gouge", msg);
            }
        }
        let Some(mut stock) = self.stock.take() else {
            return;
        };
        let bounds = stock.bounds();
        // Rubbing is checked at the collision step, not every removal sample.
        let tip = frame.t;
        self.since_rub_check += self.last_tip.map_or(f64::INFINITY, |t| t.distance(tip));
        self.last_tip = Some(tip);
        let check_rub = self.since_rub_check >= self.opts.collision_step_mm;
        if check_rub {
            self.since_rub_check = 0.0;
        }
        for (z0, h, r) in asm
            .non_cutting_cylinders()
            .into_iter()
            .filter(|_| check_rub)
        {
            let local = Aabb {
                min: Vec3::new(-r, -r, z0),
                max: Vec3::new(r, r, z0 + h),
            };
            if local.transformed(&frame).overlaps(&bounds) && stock.cylinder_hits(&frame, z0, h, r)
            {
                let msg = format!("{tag} shank or holder rubs the remaining stock");
                self.issue(line, Severity::Error, "holder_rubs_stock", msg);
                break;
            }
        }
        if rapid {
            if stock.sweep_hits(&from, &cur, &shape).is_some() {
                stock.sweep(&from, &cur, &shape);
                self.issue(
                    line,
                    Severity::Error,
                    "rapid_into_stock",
                    format!("rapid move: {tag} cutter enters the stock"),
                );
            }
        } else {
            stock.sweep(&from, &cur, &shape);
        }
        self.stock = Some(stock);
    }

    fn check_collisions(&mut self, kin: &Kinematics, line: usize, rapid: bool, q: &[f64]) {
        let Some(world) = self.opts.world.clone() else {
            return;
        };
        let class = if rapid {
            MoveClass::Rapid
        } else {
            MoveClass::Feed
        };
        for c in world.check(kin, q, self.tool, class, 0.0) {
            let pose = kin.forward(q);
            let at = format!(
                "tip at X{:.3} Y{:.3} Z{:.3}",
                pose.tip.x, pose.tip.y, pose.tip.z
            );
            let key = format!("collision:{}:{}", c.a, c.b);
            if c.a.ends_with("cutter") && c.role == crate::collision::Role::Part {
                let msg = format!("rapid move with the {} inside {} ({at})", c.a, c.b);
                self.issue_keyed(line, Severity::Error, "rapid_into_part", &key, msg);
            } else {
                let msg = format!("{} hits {} ({at})", c.a, c.b);
                self.issue_keyed(line, Severity::Error, "collision", &key, msg);
            }
        }
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
        let feed_total = self.cut_s + self.rapid_s + self.dwell_s;
        let (cycle_time_s, cycle_time_model) = match plan_time(self.def, &self.plan) {
            Some(t) => (t + self.dwell_s, "acceleration".to_string()),
            None => (feed_total, "feed-rate".to_string()),
        };
        let stock = self.stock.as_ref().map(|v| StockReport {
            resolution_mm: v.res,
            initial_mm3: self.stock_initial,
            removed_mm3: self.stock_initial - v.volume(),
            remaining_mm3: v.volume(),
        });
        Report {
            stock,
            machine: self.def.name.clone(),
            cycle_time_s,
            cycle_time_model,
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
