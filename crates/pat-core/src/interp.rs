//! Modal G-code interpreter.
//!
//! Turns (subroutine-expanded) blocks into a stream of [`Event`]s with units
//! resolved and modal state applied. It deliberately knows nothing about
//! machine geometry: words are passed through as "which axis, what value,
//! absolute or incremental, in which coordinate mode", and the simulator
//! decides what they mean physically. That keeps TCP / non-TCP / tilted-plane
//! switching correct.

use crate::gcode::Block;
use crate::math::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Plane {
    XY,
    XZ,
    YZ,
}

/// A G68.2 tilted working plane: origin in the WCS and ZXZ Euler angles
/// (I, J, K) in degrees.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct TiltedPlane {
    pub origin: Vec3,
    pub euler_deg: Vec3,
}

/// How a block's axis words are to be read.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub enum Coords {
    /// Non-TCP program coordinates (machine joints shifted to part zero).
    Program,
    /// Tool centre point control (G43.4 / G234): XYZ is the tool tip in the
    /// WCS; rotary words are axis angles.
    Tcp,
    /// G68.2 tilted plane: XYZ is the tool tip in the tilted frame.
    Tilted(TiltedPlane),
    /// G53 machine coordinates.
    Machine,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub enum Feed {
    /// mm/min (or deg/min for pure rotary moves).
    PerMinute(f64),
    /// G93: the move takes 1/F minutes.
    InverseTime(f64),
    Missing,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub enum ArcCenter {
    /// I, J, K offsets from the start point (already in mm).
    Offsets(Vec3),
    Radius(f64),
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub enum MotionKind {
    Rapid,
    Linear,
    Arc {
        clockwise: bool,
        plane: Plane,
        center: ArcCenter,
    },
    /// G28: listed axes go to machine home.
    Home,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Motion {
    pub line: usize,
    pub kind: MotionKind,
    /// Value per machine axis (index matches the machine definition), already
    /// in mm / degrees. `None` = axis not programmed on this block.
    pub axes: Vec<Option<f64>>,
    pub incremental: bool,
    pub coords: Coords,
    /// Active work offset, e.g. "G54", "G59.1", "G54.1P3".
    pub wcs: String,
    pub feed: Feed,
    /// Exact stop at the end of this move (G61 / G9).
    pub exact_stop: bool,
}

/// A drilling-type canned cycle at one position (G73, G81–G89).
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Cycle {
    pub line: usize,
    /// 73, 81, 82, … 89.
    pub kind: u32,
    /// X/Y/etc. words for the hole position (same indexing as [`Motion::axes`]).
    pub axes: Vec<Option<f64>>,
    /// Hole bottom (mm). Incremental from R in G91.
    pub z: f64,
    /// Retract plane (mm). Incremental from the initial level in G91.
    pub r: f64,
    /// Peck depth (mm) for G73 / G83.
    pub q: Option<f64>,
    /// Dwell at the bottom (seconds).
    pub dwell_s: f64,
    /// G99: retract to R; otherwise (G98) to the initial level.
    pub retract_to_r: bool,
    pub incremental: bool,
    pub coords: Coords,
    pub wcs: String,
    pub feed: Feed,
    /// Repeat count (L / K), mainly with G91 hole spacing.
    pub repeats: u32,
    /// First block of this cycle sequence: the current Z is the initial level.
    pub first: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Error,
    Warning,
    Info,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub enum Event {
    Motion(Motion),
    Cycle(Cycle),
    /// G53.1: rotate the rotaries so the tool axis matches the tilted plane's Z.
    AlignToolAxis {
        line: usize,
        plane: TiltedPlane,
    },
    ToolChange {
        line: usize,
        tool: u32,
    },
    Dwell {
        line: usize,
        seconds: f64,
    },
    /// Program pause / spindle stop etc.: motion comes to a full stop.
    Stop {
        line: usize,
    },
    Note {
        line: usize,
        severity: Severity,
        code: &'static str,
        message: String,
    },
    End {
        line: usize,
    },
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct InterpOptions {
    /// Read `P` dwell words as milliseconds (Fanuc / Haas) instead of
    /// seconds (LinuxCNC).
    pub dwell_p_milliseconds: bool,
}

#[derive(Clone, Copy, PartialEq)]
enum Modal {
    Rapid,
    Linear,
    ArcCw,
    ArcCcw,
    Cycle(u32),
}

/// Interpret blocks for a machine with the given axis letters (in machine
/// definition order).
pub fn interpret(blocks: &[Block], axis_names: &[String], opts: &InterpOptions) -> Vec<Event> {
    let mut ev = Vec::new();
    let mut st = State {
        dwell_ms: opts.dwell_p_milliseconds,
        ..State::default()
    };
    for b in blocks {
        if b.block_delete {
            continue;
        }
        if !st.step(b, axis_names, &mut ev) {
            break;
        }
    }
    ev
}

struct State {
    inch: bool,
    incremental: bool,
    arc_incremental: bool,
    inverse_time: bool,
    feed: Option<f64>,
    tcp: bool,
    plane: Plane,
    modal: Option<Modal>,
    pending_tool: Option<u32>,
    wcs: String,
    tilt: Option<TiltedPlane>,
    exact_stop_mode: bool,
    retract_to_r: bool,
    cycle_z: Option<f64>,
    cycle_r: Option<f64>,
    cycle_q: Option<f64>,
    cycle_p: f64,
    cycle_first: bool,
    dwell_ms: bool,
}

impl Default for State {
    fn default() -> Self {
        State {
            inch: false,
            incremental: false,
            arc_incremental: true,
            inverse_time: false,
            feed: None,
            tcp: false,
            plane: Plane::XY,
            modal: None,
            pending_tool: None,
            wcs: "G54".into(),
            tilt: None,
            exact_stop_mode: false,
            retract_to_r: false,
            cycle_z: None,
            cycle_r: None,
            cycle_q: None,
            cycle_p: 0.0,
            cycle_first: false,
            dwell_ms: false,
        }
    }
}

fn code(v: f64) -> i32 {
    (v * 10.0).round() as i32
}

fn note(ev: &mut Vec<Event>, line: usize, severity: Severity, code: &'static str, message: String) {
    ev.push(Event::Note {
        line,
        severity,
        code,
        message,
    });
}

impl State {
    fn dwell_seconds(&self, p: f64) -> f64 {
        if self.dwell_ms {
            p / 1000.0
        } else {
            p
        }
    }

    /// Process one block; returns false at program end.
    fn step(&mut self, b: &Block, names: &[String], ev: &mut Vec<Event>) -> bool {
        let line = b.line;
        let scale_now = |inch: bool| if inch { 25.4 } else { 1.0 };

        let mut machine_coords = false;
        let mut home = false;
        let mut dwell = false;
        let mut exact_stop_once = false;
        let mut set_tilt = false;
        let mut align = false;
        let mut motion_this_block: Option<Modal> = None;

        for g in b.all('G') {
            match code(g) {
                0 => motion_this_block = Some(Modal::Rapid),
                10 => motion_this_block = Some(Modal::Linear),
                20 => motion_this_block = Some(Modal::ArcCw),
                30 => motion_this_block = Some(Modal::ArcCcw),
                40 => dwell = true,
                90 => exact_stop_once = true,
                100 => note(
                    ev,
                    line,
                    Severity::Warning,
                    "unsupported",
                    "G10 offset setting is not simulated; give offsets in the machine file or options".into(),
                ),
                170 => self.plane = Plane::XY,
                180 => self.plane = Plane::XZ,
                190 => self.plane = Plane::YZ,
                200 => self.inch = true,
                210 => self.inch = false,
                280 => home = true,
                400 => {}
                430 => self.tcp = false,
                434 | 435 | 2340 => self.tcp = true,
                490 => self.tcp = false,
                530 => machine_coords = true,
                531 => align = true,
                540 => self.wcs = "G54".into(),
                550 => self.wcs = "G55".into(),
                560 => self.wcs = "G56".into(),
                570 => self.wcs = "G57".into(),
                580 => self.wcs = "G58".into(),
                590 => self.wcs = "G59".into(),
                591 => self.wcs = "G59.1".into(),
                592 => self.wcs = "G59.2".into(),
                593 => self.wcs = "G59.3".into(),
                541 => match b.get('P') {
                    Some(p) => self.wcs = format!("G54.1P{}", p.round() as i64),
                    None => note(ev, line, Severity::Error, "work_offset", "G54.1 needs a P number".into()),
                },
                1100..=1290 if code(g) % 10 == 0 => self.wcs = format!("G{}", code(g) / 10),
                610 | 611 => self.exact_stop_mode = true,
                640 => self.exact_stop_mode = false,
                682 => set_tilt = true,
                683 => note(
                    ev,
                    line,
                    Severity::Error,
                    "unsupported",
                    "G68.3 (plane from tool axis) is not supported".into(),
                ),
                690 => self.tilt = None,
                730 | 810..=890 => {
                    let kind = (code(g) / 10) as u32;
                    if !matches!(kind, 73 | 81 | 82 | 83 | 84 | 85 | 86 | 88 | 89) {
                        note(
                            ev,
                            line,
                            Severity::Error,
                            "unsupported",
                            format!("G{kind} canned cycle is not supported"),
                        );
                    } else {
                        if self.modal != Some(Modal::Cycle(kind)) {
                            self.cycle_first = true;
                        }
                        motion_this_block = Some(Modal::Cycle(kind));
                    }
                }
                740 => note(ev, line, Severity::Error, "unsupported", "G74 left-hand tapping is not supported".into()),
                800 => {
                    if matches!(self.modal, Some(Modal::Cycle(_))) {
                        self.modal = None;
                    }
                }
                900 => self.incremental = false,
                910 => self.incremental = true,
                901 => self.arc_incremental = false,
                911 => self.arc_incremental = true,
                930 => self.inverse_time = true,
                940 => self.inverse_time = false,
                950 => note(
                    ev,
                    line,
                    Severity::Error,
                    "unsupported",
                    "G95 feed per revolution is not supported".into(),
                ),
                980 => self.retract_to_r = false,
                990 => self.retract_to_r = true,
                _ => note(ev, line, Severity::Warning, "unknown_gcode", format!("G{g} ignored")),
            }
        }

        let scale = scale_now(self.inch);

        // Feed word: in G93 it only applies to this block.
        let f_word = b.get('F');
        if let Some(f) = f_word {
            if !self.inverse_time {
                self.feed = Some(f * scale);
            }
        }

        if let Some(t) = b.get('T') {
            self.pending_tool = Some(t.round() as u32);
        }

        let mut end = false;
        for m in b.all('M') {
            match m.round() as i32 {
                0 | 1 => ev.push(Event::Stop { line }),
                6 => match self.pending_tool {
                    Some(tool) => ev.push(Event::ToolChange { line, tool }),
                    None => note(
                        ev,
                        line,
                        Severity::Warning,
                        "tool_change",
                        "M6 without a T word".into(),
                    ),
                },
                2 | 30 => end = true,
                _ => {}
            }
        }

        if dwell {
            // Fanuc also allows G4 X<seconds>; that X is not an axis move.
            let seconds = match (b.get('X'), b.get('P')) {
                (Some(x), _) => x,
                (None, Some(p)) => self.dwell_seconds(p),
                _ => 0.0,
            };
            ev.push(Event::Stop { line });
            ev.push(Event::Dwell { line, seconds });
            if end {
                ev.push(Event::End { line });
            }
            return !end;
        }

        if set_tilt {
            let p = b.get('P').unwrap_or(0.0);
            if p.round() != 0.0 {
                note(
                    ev,
                    line,
                    Severity::Error,
                    "unsupported",
                    "only G68.2 P0 (ZXZ Euler angles) is supported".into(),
                );
            } else {
                let w = |c: char| b.get(c).unwrap_or(0.0);
                self.tilt = Some(TiltedPlane {
                    origin: Vec3::new(w('X') * scale, w('Y') * scale, w('Z') * scale),
                    euler_deg: Vec3::new(w('I'), w('J'), w('K')),
                });
            }
            // G68.2's X/Y/Z are the plane origin, not a move.
            return true;
        }

        if align {
            match self.tilt {
                Some(plane) => ev.push(Event::AlignToolAxis { line, plane }),
                None => note(
                    ev,
                    line,
                    Severity::Error,
                    "tilted_plane",
                    "G53.1 without an active G68.2 plane".into(),
                ),
            }
        }

        // Axis words.
        let mut axes: Vec<Option<f64>> = vec![None; names.len()];
        let mut any_axis = false;
        for w in &b.words {
            if !matches!(w.letter, 'X' | 'Y' | 'Z' | 'A' | 'B' | 'C') {
                continue;
            }
            match names.iter().position(|n| n.starts_with(w.letter)) {
                Some(i) => {
                    let linear = matches!(w.letter, 'X' | 'Y' | 'Z');
                    axes[i] = Some(if linear { w.value * scale } else { w.value });
                    any_axis = true;
                }
                None => note(
                    ev,
                    line,
                    Severity::Error,
                    "unknown_axis",
                    format!("axis {} is not on this machine", w.letter),
                ),
            }
        }

        if let Some(m) = motion_this_block {
            self.modal = Some(m);
        }

        let coords = if machine_coords {
            Coords::Machine
        } else if let Some(t) = self.tilt {
            Coords::Tilted(t)
        } else if self.tcp {
            Coords::Tcp
        } else {
            Coords::Program
        };

        let feed_for = |st: &State, ev: &mut Vec<Event>| match st.inverse_time {
            true => match f_word {
                Some(f) => Feed::InverseTime(f),
                None => {
                    note(
                        ev,
                        line,
                        Severity::Error,
                        "missing_feed",
                        "G93 inverse-time mode needs an F word on every feed move".into(),
                    );
                    Feed::Missing
                }
            },
            false => match st.feed {
                Some(f) if f > 0.0 => Feed::PerMinute(f),
                _ => {
                    note(
                        ev,
                        line,
                        Severity::Error,
                        "missing_feed",
                        "feed move with no feed rate set".into(),
                    );
                    Feed::Missing
                }
            },
        };

        if home {
            ev.push(Event::Motion(Motion {
                line,
                kind: MotionKind::Home,
                axes,
                incremental: false,
                coords: Coords::Machine,
                wcs: self.wcs.clone(),
                feed: Feed::Missing,
                exact_stop: true,
            }));
        } else if let Some(Modal::Cycle(kind)) = self.modal {
            // Sticky cycle parameters.
            let z_idx = names.iter().position(|n| n == "Z");
            if let Some(zi) = z_idx {
                if let Some(z) = axes[zi].take() {
                    self.cycle_z = Some(z);
                }
            }
            if let Some(r) = b.get('R') {
                self.cycle_r = Some(r * scale);
            }
            if let Some(q) = b.get('Q') {
                self.cycle_q = Some(q.abs() * scale);
            }
            if let Some(p) = b.get('P') {
                self.cycle_p = self.dwell_seconds(p);
            }
            let has_position = axes.iter().any(|a| a.is_some());
            if has_position || motion_this_block.is_some() {
                match (self.cycle_z, self.cycle_r) {
                    (Some(z), Some(r)) => {
                        if (kind == 73 || kind == 83) && self.cycle_q.is_none() {
                            note(
                                ev,
                                line,
                                Severity::Error,
                                "canned_cycle",
                                format!("G{kind} needs a Q peck depth"),
                            );
                        }
                        let feed = feed_for(self, ev);
                        let repeats = b
                            .get('L')
                            .or_else(|| b.get('K'))
                            .map_or(1, |n| n.round().max(0.0) as u32);
                        ev.push(Event::Cycle(Cycle {
                            line,
                            kind,
                            axes,
                            z,
                            r,
                            q: self.cycle_q,
                            dwell_s: self.cycle_p,
                            retract_to_r: self.retract_to_r,
                            incremental: self.incremental,
                            coords,
                            wcs: self.wcs.clone(),
                            feed,
                            repeats,
                            first: self.cycle_first,
                        }));
                        self.cycle_first = false;
                    }
                    _ => note(
                        ev,
                        line,
                        Severity::Error,
                        "canned_cycle",
                        format!("G{kind} needs Z and R"),
                    ),
                }
            }
        } else if any_axis {
            match self.modal {
                None | Some(Modal::Cycle(_)) => note(
                    ev,
                    line,
                    Severity::Error,
                    "no_motion_mode",
                    "axis words with no motion mode (G0/G1/G2/G3) active".into(),
                ),
                Some(m) => {
                    let kind = match m {
                        Modal::Rapid => MotionKind::Rapid,
                        Modal::Linear => MotionKind::Linear,
                        _ => {
                            let center = match b.get('R') {
                                Some(r) => ArcCenter::Radius(r * scale),
                                None => ArcCenter::Offsets(Vec3::new(
                                    b.get('I').unwrap_or(0.0) * scale,
                                    b.get('J').unwrap_or(0.0) * scale,
                                    b.get('K').unwrap_or(0.0) * scale,
                                )),
                            };
                            if !self.arc_incremental && b.get('R').is_none() {
                                note(
                                    ev,
                                    line,
                                    Severity::Error,
                                    "unsupported",
                                    "absolute arc centres (G90.1) are not supported".into(),
                                );
                            }
                            MotionKind::Arc {
                                clockwise: m == Modal::ArcCw,
                                plane: self.plane,
                                center,
                            }
                        }
                    };
                    let feed = if kind == MotionKind::Rapid {
                        Feed::Missing
                    } else {
                        feed_for(self, ev)
                    };
                    ev.push(Event::Motion(Motion {
                        line,
                        kind,
                        axes,
                        incremental: self.incremental && !machine_coords,
                        coords,
                        wcs: self.wcs.clone(),
                        feed,
                        exact_stop: self.exact_stop_mode || exact_stop_once,
                    }));
                }
            }
        }

        if end {
            ev.push(Event::End { line });
            return false;
        }
        true
    }
}
