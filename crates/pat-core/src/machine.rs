//! Machine definitions.
//!
//! A machine is two kinematic chains hanging off a fixed base frame (the
//! machine coordinate system):
//!
//! - the **head** chain carries the spindle,
//! - the **table** chain carries the workpiece.
//!
//! Axes in each chain are listed in order from the base outward. Every axis is
//! described in the base frame with all axes at zero ("home"), which is the
//! product-of-exponentials convention: the geometry never depends on the order
//! the joints are evaluated in.
//!
//! Conventions:
//! - Linear axes: a positive value moves the axis's body (head or table) along
//!   `direction`. A table that moves toward the operator for +Y therefore has
//!   `direction = [0, -1, 0]`.
//! - Rotary axes: a positive value rotates the axis's body right-handedly
//!   about `direction`, through `point`.
//! - Units are millimetres and degrees.
//! - `spindle.gauge_point` is the spindle gauge line at home; the tool tip sits
//!   `tool_length` below it along `-spindle.direction`.
//! - `wcs_origin` is part zero (G54) in the base frame with every axis at home.
//!   Its axes are parallel to the base axes.

use crate::math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashSet};
use thiserror::Error;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum AxisKind {
    Linear,
    Rotary,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Chain {
    Head,
    Table,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AxisDef {
    /// G-code letter: X, Y, Z, A, B or C.
    pub name: String,
    pub kind: AxisKind,
    pub chain: Chain,
    pub direction: Vec3,
    /// A point on the rotation line (rotary axes only).
    #[serde(default)]
    pub point: Vec3,
    /// Soft limits in machine coordinates. Omit both on a rotary axis for
    /// continuous (unlimited) rotation.
    #[serde(default)]
    pub min: Option<f64>,
    #[serde(default)]
    pub max: Option<f64>,
    /// Rapid rate in mm/min or deg/min. Used for cycle-time estimates.
    #[serde(default)]
    pub max_rate: Option<f64>,
    /// Acceleration limit in mm/s² or deg/s². When every axis has one, cycle
    /// time is estimated with an acceleration-limited look-ahead planner.
    #[serde(default)]
    pub max_accel: Option<f64>,
}

impl AxisDef {
    pub fn is_rotary(&self) -> bool {
        self.kind == AxisKind::Rotary
    }

    pub fn is_continuous(&self) -> bool {
        self.is_rotary() && self.min.is_none() && self.max.is_none()
    }

    pub fn within_limits(&self, v: f64, tol: f64) -> bool {
        self.min.is_none_or(|m| v >= m - tol) && self.max.is_none_or(|m| v <= m + tol)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpindleDef {
    pub gauge_point: Vec3,
    /// Tool axis direction at home, pointing from the tip toward the spindle.
    #[serde(default = "default_spindle_dir")]
    pub direction: Vec3,
}

fn default_spindle_dir() -> Vec3 {
    Vec3::Z
}

fn default_singularity_tol() -> f64 {
    1.0
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MachineDef {
    pub name: String,
    #[serde(default)]
    pub description: String,
    pub axes: Vec<AxisDef>,
    pub spindle: SpindleDef,
    pub wcs_origin: Vec3,
    /// Tool length used when none is given (mm).
    #[serde(default)]
    pub default_tool_length: f64,
    /// Tool axes within this many degrees of a singular direction are
    /// reported as near-singular.
    #[serde(default = "default_singularity_tol")]
    pub singularity_tolerance_deg: f64,
    /// Extra work offsets by name ("G55", "G59.1", "G54.1P3", "G110", …):
    /// part zero in the base frame with all axes home, like `wcs_origin`
    /// (which is G54).
    #[serde(default)]
    pub work_offsets: BTreeMap<String, Vec3>,
    /// Look-ahead cornering tolerance (mm) for the cycle-time planner: how far
    /// the control may round a corner to keep speed (GRBL-style junction
    /// deviation).
    #[serde(default = "default_junction_deviation")]
    pub junction_deviation: f64,
}

fn default_junction_deviation() -> f64 {
    0.02
}

#[derive(Debug, Error)]
pub enum MachineError {
    #[error("could not parse machine file: {0}")]
    Parse(String),
    #[error("invalid machine '{machine}': {msg}")]
    Invalid { machine: String, msg: String },
}

const ALLOWED_NAMES: [&str; 6] = ["X", "Y", "Z", "A", "B", "C"];

impl MachineDef {
    pub fn from_toml(src: &str) -> Result<MachineDef, MachineError> {
        let def: MachineDef =
            toml::from_str(src).map_err(|e| MachineError::Parse(e.to_string()))?;
        def.validated()
    }

    pub fn from_json(src: &str) -> Result<MachineDef, MachineError> {
        let def: MachineDef =
            serde_json::from_str(src).map_err(|e| MachineError::Parse(e.to_string()))?;
        def.validated()
    }

    /// Check the definition and normalise direction vectors.
    pub fn validated(mut self) -> Result<MachineDef, MachineError> {
        let invalid = |msg: String| MachineError::Invalid {
            machine: self.name.clone(),
            msg,
        };
        let mut seen = HashSet::new();
        let mut linear = 0;
        let mut rotary = 0;
        for axis in &self.axes {
            let upper = axis.name.to_ascii_uppercase();
            if !ALLOWED_NAMES.contains(&upper.as_str()) {
                return Err(invalid(format!(
                    "axis name '{}' must be one of X Y Z A B C",
                    axis.name
                )));
            }
            let named_linear = ["X", "Y", "Z"].contains(&upper.as_str());
            if named_linear != (axis.kind == AxisKind::Linear) {
                return Err(invalid(format!(
                    "axis '{}': X/Y/Z must be linear and A/B/C must be rotary",
                    axis.name
                )));
            }
            if !seen.insert(upper) {
                return Err(invalid(format!("duplicate axis '{}'", axis.name)));
            }
            if axis.direction.normalized().is_none() {
                return Err(invalid(format!(
                    "axis '{}' has a zero direction",
                    axis.name
                )));
            }
            if let (Some(lo), Some(hi)) = (axis.min, axis.max) {
                if lo > hi {
                    return Err(invalid(format!("axis '{}' has min > max", axis.name)));
                }
            }
            match axis.kind {
                AxisKind::Linear => linear += 1,
                AxisKind::Rotary => rotary += 1,
            }
        }
        if linear != 3 {
            return Err(invalid(format!(
                "need exactly 3 linear axes, found {linear}"
            )));
        }
        if rotary > 2 {
            return Err(invalid(format!(
                "at most 2 rotary axes are supported, found {rotary}"
            )));
        }
        if let Some(a) = self
            .axes
            .iter()
            .find(|a| a.max_accel.is_some_and(|v| v <= 0.0))
        {
            return Err(invalid(format!(
                "axis '{}' max_accel must be positive",
                a.name
            )));
        }
        if self.junction_deviation < 0.0 {
            return Err(invalid("junction_deviation must not be negative".into()));
        }
        if self.spindle.direction.normalized().is_none() {
            return Err(invalid("spindle direction is zero".into()));
        }
        for axis in &mut self.axes {
            axis.name = axis.name.to_ascii_uppercase();
            axis.direction = axis.direction.normalized().unwrap();
        }
        self.spindle.direction = self.spindle.direction.normalized().unwrap();
        Ok(self)
    }

    pub fn axis_index(&self, name: &str) -> Option<usize> {
        self.axes
            .iter()
            .position(|a| a.name.eq_ignore_ascii_case(name))
    }

    pub fn rotary_count(&self) -> usize {
        self.axes.iter().filter(|a| a.is_rotary()).count()
    }

    /// Part zero for a work offset name; G54 is `wcs_origin`.
    pub fn work_offset(&self, name: &str) -> Option<Vec3> {
        let key = name.to_ascii_uppercase();
        if key == "G54" {
            return Some(self.wcs_origin);
        }
        self.work_offsets
            .iter()
            .find(|(k, _)| k.to_ascii_uppercase() == key)
            .map(|(_, v)| *v)
    }

    /// Copy of this machine with part zero moved to `origin`.
    pub fn with_wcs(&self, origin: Vec3) -> MachineDef {
        MachineDef {
            wcs_origin: origin,
            ..self.clone()
        }
    }

    pub fn to_toml(&self) -> String {
        toml::to_string_pretty(self).expect("machine definitions always serialise")
    }
}
