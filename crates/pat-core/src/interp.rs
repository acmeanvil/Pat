//! Modal G-code interpreter.
//!
//! Turns parsed blocks into a stream of [`Event`]s with units resolved and
//! modal state applied. It deliberately knows nothing about machine geometry:
//! words are passed through as "which axis, what value, absolute or
//! incremental, in which coordinate mode", and the simulator decides what
//! they mean physically. That keeps TCP / non-TCP switching correct.

use crate::gcode::Block;
use crate::math::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Plane {
    XY,
    XZ,
    YZ,
}

/// How a block's axis words are to be read.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Coords {
    /// Non-TCP program coordinates (machine joints shifted to part zero).
    Program,
    /// Tool centre point control (G43.4 / G234): XYZ is the tool tip in the
    /// WCS; rotary words are axis angles.
    Tcp,
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
    pub feed: Feed,
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
    ToolChange {
        line: usize,
        tool: u32,
    },
    Dwell {
        line: usize,
        seconds: f64,
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

#[derive(Clone, Copy, PartialEq)]
enum Modal {
    Rapid,
    Linear,
    ArcCw,
    ArcCcw,
}

/// Interpret blocks for a machine with the given axis letters (in machine
/// definition order).
pub fn interpret(blocks: &[Block], axis_names: &[String]) -> Vec<Event> {
    let mut ev = Vec::new();
    let mut st = State::default();
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
        }
    }
}

fn code(v: f64) -> i32 {
    (v * 10.0).round() as i32
}

impl State {
    /// Process one block; returns false at program end.
    fn step(&mut self, b: &Block, names: &[String], ev: &mut Vec<Event>) -> bool {
        let line = b.line;

        let mut machine_coords = false;
        let mut home = false;
        let mut motion_this_block: Option<Modal> = None;

        for g in b.all('G') {
            match code(g) {
                0 => motion_this_block = Some(Modal::Rapid),
                10 => motion_this_block = Some(Modal::Linear),
                20 => motion_this_block = Some(Modal::ArcCw),
                30 => motion_this_block = Some(Modal::ArcCcw),
                40 => {
                    let secs = b.get('P').unwrap_or(0.0);
                    ev.push(Event::Dwell {
                        line,
                        seconds: secs,
                    });
                }
                170 => self.plane = Plane::XY,
                180 => self.plane = Plane::XZ,
                190 => self.plane = Plane::YZ,
                200 => self.inch = true,
                210 => self.inch = false,
                280 => home = true,
                400 | 610 | 611 | 640 | 690 | 800 => {}
                430 => self.tcp = false,
                434 | 435 | 2340 => self.tcp = true,
                490 => self.tcp = false,
                530 => machine_coords = true,
                540 => {}
                550..=590 | 541 => note(
                    ev,
                    line,
                    Severity::Warning,
                    "work_offset",
                    format!("G{g} selected; only one work offset (G54) is modelled"),
                ),
                682 | 683 => note(
                    ev,
                    line,
                    Severity::Error,
                    "unsupported",
                    format!("G{g} tilted work plane is not simulated yet"),
                ),
                810..=890 => note(
                    ev,
                    line,
                    Severity::Warning,
                    "canned_cycle",
                    format!("G{g} canned cycle is not simulated; position after it may be wrong"),
                ),
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
                _ => note(
                    ev,
                    line,
                    Severity::Warning,
                    "unknown_gcode",
                    format!("G{g} ignored"),
                ),
            }
        }

        // Feed word: in G93 it only applies to this block.
        let scale = if self.inch { 25.4 } else { 1.0 };
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
                6 => {
                    if let Some(tool) = self.pending_tool {
                        ev.push(Event::ToolChange { line, tool });
                    } else {
                        ev.push(Event::Note {
                            line,
                            severity: Severity::Warning,
                            code: "tool_change",
                            message: "M6 without a T word".into(),
                        });
                    }
                }
                2 | 30 => end = true,
                _ => {}
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
                None => ev.push(Event::Note {
                    line,
                    severity: Severity::Error,
                    code: "unknown_axis",
                    message: format!("axis {} is not on this machine", w.letter),
                }),
            }
        }

        if let Some(m) = motion_this_block {
            self.modal = Some(m);
        }

        if home {
            ev.push(Event::Motion(Motion {
                line,
                kind: MotionKind::Home,
                axes,
                incremental: false,
                coords: Coords::Machine,
                feed: Feed::Missing,
            }));
        } else if any_axis {
            match self.modal {
                None => ev.push(Event::Note {
                    line,
                    severity: Severity::Error,
                    code: "no_motion_mode",
                    message: "axis words with no motion mode (G0/G1/G2/G3) active".into(),
                }),
                Some(m) => {
                    let kind = match m {
                        Modal::Rapid => MotionKind::Rapid,
                        Modal::Linear => MotionKind::Linear,
                        Modal::ArcCw | Modal::ArcCcw => {
                            let center = match b.get('R') {
                                Some(r) => ArcCenter::Radius(r * scale),
                                None => ArcCenter::Offsets(Vec3::new(
                                    b.get('I').unwrap_or(0.0) * scale,
                                    b.get('J').unwrap_or(0.0) * scale,
                                    b.get('K').unwrap_or(0.0) * scale,
                                )),
                            };
                            if !self.arc_incremental && b.get('R').is_none() {
                                ev.push(Event::Note {
                                    line,
                                    severity: Severity::Error,
                                    code: "unsupported",
                                    message: "absolute arc centres (G90.1) are not supported"
                                        .into(),
                                });
                            }
                            MotionKind::Arc {
                                clockwise: m == Modal::ArcCw,
                                plane: self.plane,
                                center,
                            }
                        }
                    };
                    let feed = match (&kind, self.inverse_time) {
                        (MotionKind::Rapid, _) => Feed::Missing,
                        (_, true) => {
                            match f_word {
                                Some(f) => Feed::InverseTime(f),
                                None => {
                                    ev.push(Event::Note {
                                    line,
                                    severity: Severity::Error,
                                    code: "missing_feed",
                                    message: "G93 inverse-time mode needs an F word on every feed move".into(),
                                });
                                    Feed::Missing
                                }
                            }
                        }
                        (_, false) => match self.feed {
                            Some(f) if f > 0.0 => Feed::PerMinute(f),
                            _ => {
                                ev.push(Event::Note {
                                    line,
                                    severity: Severity::Error,
                                    code: "missing_feed",
                                    message: "feed move with no feed rate set".into(),
                                });
                                Feed::Missing
                            }
                        },
                    };
                    let coords = if machine_coords {
                        Coords::Machine
                    } else if self.tcp {
                        Coords::Tcp
                    } else {
                        Coords::Program
                    };
                    ev.push(Event::Motion(Motion {
                        line,
                        kind,
                        axes,
                        incremental: self.incremental && !machine_coords,
                        coords,
                        feed,
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

fn note(ev: &mut Vec<Event>, line: usize, severity: Severity, code: &'static str, message: String) {
    ev.push(Event::Note {
        line,
        severity,
        code,
        message,
    });
}
