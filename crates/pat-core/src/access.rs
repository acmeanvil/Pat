//! Accessibility planning: which tool directions reach a point without
//! collision, and the smallest tilt that avoids an obstacle.
//!
//! These are the building blocks for automatic tool-axis control. The
//! answers are explained, not just returned: callers get which bodies block
//! the preferred direction, how far the chosen direction clears everything,
//! and the machine joints that realise it. A UI or an AI agent can then say
//! "tilted 12° toward −Y because the T1 holder hits the vise".

use crate::collision::World;
use crate::kinematics::{Kinematics, Pose};
use crate::math::{Mat3, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct TiltOptions {
    /// Required clearance between tool assembly and obstacles (mm).
    pub margin: f64,
    /// Largest tilt away from the preferred axis to try (degrees).
    pub max_tilt_deg: f64,
    /// Tilt increment (degrees).
    pub tilt_step_deg: f64,
    /// Directions tried around each tilt ring.
    pub azimuth_steps: usize,
    /// Clearances above this are reported as this value (mm).
    pub clearance_cap: f64,
}

impl Default for TiltOptions {
    fn default() -> Self {
        TiltOptions {
            margin: 1.0,
            max_tilt_deg: 60.0,
            tilt_step_deg: 1.0,
            azimuth_steps: 36,
            clearance_cap: 25.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TiltResult {
    /// Chosen tool axis in the WCS, if any direction works.
    pub axis: Option<Vec3>,
    pub joints: Option<Vec<f64>>,
    /// Angle between the chosen and preferred axis (degrees).
    pub tilt_deg: f64,
    /// Clearance of the chosen direction (mm, capped).
    pub clearance: f64,
    /// What blocks the preferred axis, e.g. "T1 holder vs vise".
    pub blocked_by: Vec<String>,
    /// Plain-language summary.
    pub explanation: String,
    pub directions_tried: usize,
}

/// Unit vector perpendicular to `v`.
fn perpendicular(v: Vec3) -> Vec3 {
    let h = if v.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    (h - v * h.dot(v)).normalized().unwrap_or(Vec3::X)
}

/// `v` tilted by `tilt` radians toward azimuth `az` (radians) around it.
fn tilted(v: Vec3, tilt: f64, az: f64) -> Vec3 {
    let u = perpendicular(v);
    let w = v.cross(u);
    let dir = u * az.cos() + w * az.sin();
    (v * tilt.cos() + dir * tilt.sin())
        .normalized()
        .unwrap_or(v)
}

/// Is the direction usable: reachable within limits and collision free?
/// Returns the joints and the blocking contacts.
fn evaluate(
    kin: &Kinematics,
    world: &World,
    tool: u32,
    tip: Vec3,
    axis: Vec3,
    prev: Option<&[f64]>,
    margin: f64,
) -> (Option<Vec<f64>>, Vec<String>) {
    let Ok(sol) = kin.inverse(&Pose { tip, axis }, prev) else {
        return (None, vec!["out of axis limits".into()]);
    };
    let contacts = world.check_tool_only(kin, &sol.joints, tool, margin);
    let blocked: Vec<String> = contacts
        .iter()
        .map(|c| format!("{} vs {}", c.a, c.b))
        .collect();
    if blocked.is_empty() {
        (Some(sol.joints), blocked)
    } else {
        (None, blocked)
    }
}

/// Find the smallest tilt from `preferred` that reaches `tip` cleanly. Among
/// directions at that tilt, the one with the most clearance wins.
pub fn tilt_to_avoid(
    kin: &Kinematics,
    world: &World,
    tool: u32,
    tip: Vec3,
    preferred: Vec3,
    prev: Option<&[f64]>,
    opts: &TiltOptions,
) -> TiltResult {
    let preferred = preferred.normalized().unwrap_or(Vec3::Z);
    let mut tried = 1;
    let (ok, blocked_by) = evaluate(kin, world, tool, tip, preferred, prev, opts.margin);
    if let Some(q) = ok {
        let clearance = world.clearance(kin, &q, tool, opts.clearance_cap);
        return TiltResult {
            axis: Some(preferred),
            joints: Some(q),
            tilt_deg: 0.0,
            clearance,
            blocked_by,
            explanation: format!("preferred axis is clear ({clearance:.1} mm clearance)"),
            directions_tried: tried,
        };
    }

    let steps = (opts.max_tilt_deg / opts.tilt_step_deg).ceil() as usize;
    for k in 1..=steps {
        let tilt = (k as f64 * opts.tilt_step_deg).min(opts.max_tilt_deg);
        let mut best: Option<(f64, Vec3, Vec<f64>, f64)> = None; // (clearance, axis, q, az)
        for a in 0..opts.azimuth_steps.max(1) {
            let az = std::f64::consts::TAU * a as f64 / opts.azimuth_steps.max(1) as f64;
            let axis = tilted(preferred, tilt.to_radians(), az);
            tried += 1;
            if let (Some(q), _) = evaluate(kin, world, tool, tip, axis, prev, opts.margin) {
                let c = world.clearance(kin, &q, tool, opts.clearance_cap);
                if best.as_ref().is_none_or(|b| c > b.0) {
                    best = Some((c, axis, q, az));
                }
            }
        }
        if let Some((clearance, axis, q, _)) = best {
            let lean = describe_lean(preferred, axis);
            return TiltResult {
                axis: Some(axis),
                joints: Some(q),
                tilt_deg: tilt,
                clearance,
                explanation: format!(
                    "preferred axis blocked ({}); tilted {tilt:.1}° {lean} for {clearance:.1} mm clearance",
                    blocked_by.join(", ")
                ),
                blocked_by,
                directions_tried: tried,
            };
        }
    }
    TiltResult {
        axis: None,
        joints: None,
        tilt_deg: opts.max_tilt_deg,
        clearance: 0.0,
        explanation: format!(
            "no collision-free reachable direction within {:.0}° of the preferred axis ({})",
            opts.max_tilt_deg,
            blocked_by.join(", ")
        ),
        blocked_by,
        directions_tried: tried,
    }
}

/// "toward +X −Y" style description of how `axis` leans relative to `from`.
fn describe_lean(from: Vec3, axis: Vec3) -> String {
    let d = axis - from * from.dot(axis);
    let mut parts = Vec::new();
    for (v, name) in [(d.x, "X"), (d.y, "Y"), (d.z, "Z")] {
        if v.abs() > 0.25 * d.norm().max(1e-12) {
            parts.push(format!("{}{name}", if v > 0.0 { "+" } else { "−" }));
        }
    }
    if parts.is_empty() {
        "slightly".into()
    } else {
        format!("toward {}", parts.join(" "))
    }
}

/// Evenly spread unit directions within `max_angle_deg` of `centre`
/// (Fibonacci spiral on the spherical cap).
pub fn cap_directions(centre: Vec3, max_angle_deg: f64, count: usize) -> Vec<Vec3> {
    let centre = centre.normalized().unwrap_or(Vec3::Z);
    let u = perpendicular(centre);
    let frame = Mat3::from_cols(u, centre.cross(u), centre);
    let cos_max = max_angle_deg.to_radians().cos();
    let golden = std::f64::consts::PI * (3.0 - 5f64.sqrt());
    (0..count.max(1))
        .map(|i| {
            let z = 1.0 - (1.0 - cos_max) * (i as f64 + 0.5) / count.max(1) as f64;
            let r = (1.0 - z * z).max(0.0).sqrt();
            let th = golden * i as f64;
            frame.mul_vec(Vec3::new(r * th.cos(), r * th.sin(), z))
        })
        .collect()
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DirectionReach {
    pub axis: Vec3,
    pub reachable: bool,
    pub clearance: f64,
    pub blocked_by: Vec<String>,
}

/// Accessibility map for one point: for each candidate direction, whether
/// the machine can reach it within limits without collision.
pub fn reachable_directions(
    kin: &Kinematics,
    world: &World,
    tool: u32,
    tip: Vec3,
    directions: &[Vec3],
    margin: f64,
    clearance_cap: f64,
) -> Vec<DirectionReach> {
    directions
        .iter()
        .map(
            |&axis| match evaluate(kin, world, tool, tip, axis, None, margin) {
                (Some(q), _) => DirectionReach {
                    axis,
                    reachable: true,
                    clearance: world.clearance(kin, &q, tool, clearance_cap),
                    blocked_by: Vec::new(),
                },
                (None, blocked_by) => DirectionReach {
                    axis,
                    reachable: false,
                    clearance: 0.0,
                    blocked_by,
                },
            },
        )
        .collect()
}
