//! Pat core: machine kinematics, G-code simulation and post-processing for
//! 3-, 4- and 5-axis milling.
//!
//! The crate is host-agnostic. Fusion, a CLI or an AI agent all talk to it
//! through plain data (TOML machine files, G-code text, JSON reports).
//!
//! Layers:
//! - [`math`]: small vector / rotation / rigid-transform types.
//! - [`machine`]: machine definition (axes, chains, limits) loaded from TOML.
//! - [`kinematics`]: forward and inverse kinematics for any machine with three
//!   linear axes and up to two rotary axes, in any head/table arrangement.
//! - [`gcode`] and [`interp`]: G-code parsing and modal interpretation.
//! - [`sim`]: replays a program on a machine and reports problems.
//! - [`post`]: turns tool-tip + tool-axis paths (CL data) into G-code.

pub mod gcode;
pub mod interp;
pub mod kinematics;
pub mod machine;
pub mod math;
pub mod planner;
pub mod post;
pub mod sim;
pub mod subs;

pub use kinematics::{IkSolution, Kinematics, Pose};
pub use machine::MachineDef;
