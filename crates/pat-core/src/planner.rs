//! Acceleration-limited cycle-time estimate.
//!
//! A look-ahead velocity planner in the style of GRBL / LinuxCNC trajectory
//! planners, run over the joint-space segments the simulator produced:
//!
//! 1. Each segment gets a nominal speed (programmed feed, or the rapid rate),
//!    capped by every axis's `max_rate`, and an acceleration limit, the
//!    largest path acceleration that keeps every axis within `max_accel`.
//! 2. Corner speeds come from the junction-deviation model: the control may
//!    round a corner by up to `junction_deviation` mm, which limits the speed
//!    through it.
//! 3. A backward and forward pass make every speed change reachable within
//!    the segment lengths. Each segment then gets a trapezoidal (or
//!    triangular) velocity profile.
//!
//! Distances are Euclidean in joint space, with degrees counted like
//! millimetres, the same simplification GRBL makes for rotary axes. This
//! gives a much better estimate than feed-rate arithmetic, especially for
//! short-segment 5-axis code. It still ignores jerk limits, servo lag and
//! the control's own block-processing limits.

use crate::machine::MachineDef;

/// One straight joint-space segment.
#[derive(Clone, Debug)]
pub struct PlanSeg {
    /// Joint deltas (mm / deg) in machine-axis order.
    pub dq: Vec<f64>,
    /// Time at programmed speed, ignoring acceleration (seconds).
    pub nominal_s: f64,
    /// The machine must come to rest before this segment starts.
    pub stop_before: bool,
}

/// True when every axis has an acceleration limit.
pub fn has_accel_data(def: &MachineDef) -> bool {
    def.axes.iter().all(|a| a.max_accel.is_some())
}

/// Total time in seconds for the segments, or `None` without acceleration data.
pub fn plan_time(def: &MachineDef, segs: &[PlanSeg]) -> Option<f64> {
    if !has_accel_data(def) {
        return None;
    }
    let n_ax = def.axes.len();
    // Per-segment geometry and limits.
    struct S {
        len: f64,
        dir: Vec<f64>,
        v_max: f64,
        accel: f64,
        stop_before: bool,
        idle_s: f64,
    }
    let mut v: Vec<S> = Vec::with_capacity(segs.len());
    for seg in segs {
        let len = seg.dq.iter().map(|d| d * d).sum::<f64>().sqrt();
        if len < 1e-9 {
            // Zero-length move: only its nominal time (if any) counts, and the
            // machine is at rest around it.
            v.push(S {
                len: 0.0,
                dir: vec![0.0; n_ax],
                v_max: 0.0,
                accel: 0.0,
                stop_before: true,
                idle_s: seg.nominal_s.max(0.0),
            });
            continue;
        }
        let dir: Vec<f64> = seg.dq.iter().map(|d| d / len).collect();
        let mut v_max = if seg.nominal_s > 1e-12 {
            len / seg.nominal_s
        } else {
            f64::INFINITY
        };
        let mut accel = f64::INFINITY;
        for (i, ax) in def.axes.iter().enumerate() {
            let c = dir[i].abs();
            if c < 1e-12 {
                continue;
            }
            if let Some(rate) = ax.max_rate {
                v_max = v_max.min(rate / 60.0 / c);
            }
            if let Some(a) = ax.max_accel {
                accel = accel.min(a / c);
            }
        }
        if !v_max.is_finite() {
            // No feed and no rate limit: treat as instantaneous.
            v_max = 1e12;
        }
        v.push(S {
            len,
            dir,
            v_max,
            accel,
            stop_before: seg.stop_before,
            idle_s: 0.0,
        });
    }

    let n = v.len();
    if n == 0 {
        return Some(0.0);
    }
    // Maximum entry speed of each segment from corners and neighbour caps.
    let mut entry = vec![0.0; n + 1]; // entry[n] = final exit speed = 0
    let mut entry_cap = vec![0.0; n];
    for k in 0..n {
        if v[k].len == 0.0 || v[k].stop_before || k == 0 || v[k - 1].len == 0.0 {
            entry_cap[k] = 0.0;
            continue;
        }
        let (p, c) = (&v[k - 1], &v[k]);
        let cos: f64 = p.dir.iter().zip(&c.dir).map(|(a, b)| a * b).sum();
        let a = p.accel.min(c.accel);
        let cap = p.v_max.min(c.v_max);
        entry_cap[k] = if cos > 0.999_999 {
            cap
        } else if cos < -0.999_999 || def.junction_deviation == 0.0 {
            0.0
        } else {
            // GRBL: theta is the angle between the reversed previous direction
            // and the next one.
            let sin_half = (0.5 * (1.0 + cos)).sqrt(); // sin(theta/2) with cos(theta) = -cos
            let sin_half = sin_half.min(0.999_999);
            let vj2 = a * def.junction_deviation * sin_half / (1.0 - sin_half);
            vj2.sqrt().min(cap)
        };
    }
    // Backward pass: must be able to decelerate to the next entry speed.
    for k in (0..n).rev() {
        let reach = (entry[k + 1] * entry[k + 1] + 2.0 * v[k].accel * v[k].len).sqrt();
        entry[k] = entry_cap[k].min(reach);
    }
    // Forward pass: must be able to accelerate from the previous entry speed.
    for k in 0..n {
        let reach = (entry[k] * entry[k] + 2.0 * v[k].accel * v[k].len).sqrt();
        if entry[k + 1] > reach {
            entry[k + 1] = reach;
        }
    }

    let mut total = 0.0;
    for k in 0..n {
        let s = &v[k];
        total += s.idle_s;
        if s.len == 0.0 {
            continue;
        }
        total += trapezoid_time(s.len, entry[k], entry[k + 1], s.v_max, s.accel);
    }
    Some(total)
}

/// Time to cover `len` starting at `v0`, ending at `v1`, cruising at most at
/// `vmax`, with acceleration `a`.
fn trapezoid_time(len: f64, v0: f64, v1: f64, vmax: f64, a: f64) -> f64 {
    if !a.is_finite() {
        return len / vmax;
    }
    let v0 = v0.min(vmax);
    let v1 = v1.min(vmax);
    let d_acc = (vmax * vmax - v0 * v0) / (2.0 * a);
    let d_dec = (vmax * vmax - v1 * v1) / (2.0 * a);
    if d_acc + d_dec <= len {
        (vmax - v0) / a + (vmax - v1) / a + (len - d_acc - d_dec) / vmax
    } else {
        let vp = ((2.0 * a * len + v0 * v0 + v1 * v1) / 2.0).sqrt();
        (vp - v0).max(0.0) / a + (vp - v1).max(0.0) / a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trapezoid_full_and_triangle() {
        // 100 mm at 50 mm/s with 100 mm/s²: 0.5 s up and 0.5 s down
        // (12.5 mm each), 75 mm cruising for 1.5 s.
        assert!((trapezoid_time(100.0, 0.0, 0.0, 50.0, 100.0) - 2.5).abs() < 1e-12);
        // 1 mm: never reaches cruise; peak sqrt(100) = 10 mm/s, 0.2 s total.
        assert!((trapezoid_time(1.0, 0.0, 0.0, 50.0, 100.0) - 0.2).abs() < 1e-12);
    }
}
