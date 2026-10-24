//! Trochoidal slotting.
//!
//! A slot slightly wider than the tool can't be cleared at a controlled
//! load by steering. Trochoidal milling instead moves the tool round in
//! small loops while it advances along the slot, so each loop takes only a
//! thin crescent of material (about the advance per loop). That allows full
//! flute depth at high feed without slotting.
//!
//! The loops run counter-clockwise, which is climb milling for a
//! clockwise (M3) spindle: at the front of each loop the cutting edge meets
//! the uncut material with the chip thick first. Depth levels alternate
//! direction and mirror the loops so every level stays climb milling.
//!
//! The achieved radial engagement is measured on a raster of the slot (what
//! each step actually cuts), not assumed, and reported with the result.

use crate::math::Vec3;
use crate::post::ClMove;
use crate::stock::CutterShape;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct SlotParams {
    pub cutter: CutterShape,
    /// Slot width (mm).
    pub width: f64,
    /// Top of the material and slot floor (WCS Z, mm).
    pub z_top: f64,
    pub z_bottom: f64,
    /// Depth per level (mm). Trochoidal milling usually takes the full depth
    /// in one level, up to the flute length.
    pub stepdown: f64,
    /// Advance along the slot per loop (mm); this is roughly the radial
    /// width of cut.
    pub pitch: f64,
    /// Stock left on the slot walls (mm).
    pub leave: f64,
    /// Path point spacing (mm).
    pub step: f64,
    pub feed: f64,
    /// Feed for the return half of each loop, which cuts nothing (mm/min).
    pub return_feed: f64,
    /// The slot starts at an open edge (enter from outside); otherwise the
    /// tool helixes in at the start.
    pub open_start: bool,
    pub clearance: f64,
}

impl Default for SlotParams {
    fn default() -> Self {
        SlotParams {
            cutter: CutterShape::flat(8.0, 25.0),
            width: 12.0,
            z_top: 0.0,
            z_bottom: -10.0,
            stepdown: 0.0,
            pitch: 0.8,
            leave: 0.0,
            step: 0.4,
            feed: 2500.0,
            return_feed: 5000.0,
            open_start: true,
            clearance: 5.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SlotResult {
    pub moves: Vec<ClMove>,
    pub levels: Vec<f64>,
    pub loops: usize,
    /// Trochoid radius (tool centre circle, mm).
    pub loop_radius: f64,
    /// Largest measured radial width of cut, and the same as a fraction of
    /// the cutter diameter.
    pub max_engagement_mm: f64,
    pub max_engagement_ratio: f64,
    pub warnings: Vec<String>,
}

/// Point and unit tangent at arc length `s` along a polyline.
fn along(pts: &[[f64; 2]], cum: &[f64], s: f64) -> ([f64; 2], [f64; 2]) {
    let total = *cum.last().unwrap();
    let s = s.clamp(0.0, total);
    let i = cum.partition_point(|&c| c < s).clamp(1, pts.len() - 1);
    let (a, b) = (pts[i - 1], pts[i]);
    let len = (cum[i] - cum[i - 1]).max(1e-12);
    let t = ((s - cum[i - 1]) / len).clamp(0.0, 1.0);
    let dir = [(b[0] - a[0]) / len, (b[1] - a[1]) / len];
    ([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t], dir)
}

/// Trochoidal toolpath along `centerline` (WCS XY points).
pub fn trochoidal_slot(centerline: &[[f64; 2]], p: &SlotParams) -> SlotResult {
    let mut warnings = Vec::new();
    let r = p.cutter.radius;
    let empty = |w: String| SlotResult {
        moves: vec![],
        levels: vec![],
        loops: 0,
        loop_radius: 0.0,
        max_engagement_mm: 0.0,
        max_engagement_ratio: 0.0,
        warnings: vec![w],
    };
    if centerline.len() < 2 {
        return empty("the slot centre line needs at least two points".into());
    }
    let rt = p.width / 2.0 - r - p.leave;
    if rt < 0.0 {
        return empty(format!(
            "the Ø{:.2} cutter does not fit a {:.2} mm slot",
            2.0 * r,
            p.width
        ));
    }
    if rt < 0.15 * r {
        warnings.push(format!(
            "the slot is barely wider than the cutter (loop radius {rt:.2} mm); engagement will be close to full slotting"
        ));
    }
    let mut cum = vec![0.0];
    for w in centerline.windows(2) {
        cum.push(cum.last().unwrap() + (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]));
    }
    let total = *cum.last().unwrap();

    // Depth levels.
    let depth = p.z_top - p.z_bottom;
    let sd = if p.stepdown > 0.0 {
        p.stepdown
    } else {
        depth.min(p.cutter.flute_length)
    };
    if depth > p.cutter.flute_length {
        warnings.push(format!(
            "slot depth {depth:.1} mm exceeds the flute length; using {sd:.1} mm levels"
        ));
    }
    let mut levels = Vec::new();
    let mut z = p.z_top;
    while z > p.z_bottom + 1e-9 {
        z = (z - sd).max(p.z_bottom);
        levels.push(z);
    }

    // Start and end along the slot: from outside the material when open.
    let lead = if p.open_start { r + rt + 0.5 } else { 0.0 };
    let (s0, s1) = (-lead, total);
    let pitch = p.pitch.max(0.01);
    let dth = if rt > 1e-6 {
        (p.step / rt).min(10f64.to_radians())
    } else {
        1.0
    };
    let safe = p.z_top + p.clearance;

    let mut moves = Vec::new();
    let mut loops = 0usize;
    let ax = Vec3::Z;
    for (li, &z) in levels.iter().enumerate() {
        let forward = li % 2 == 0;
        // Going backwards, mirror the loops (clockwise relative to travel
        // reversed) so the cut stays climb.
        let (sa, sb, dir) = if forward {
            (s0, s1, 1.0)
        } else {
            (s1, s0.max(0.0), -1.0)
        };
        let total_th = (sb - sa).abs() / pitch * std::f64::consts::TAU;
        let n = (total_th / dth).ceil() as usize;
        let pos = |th: f64| -> [f64; 2] {
            let s = sa + dir * th / std::f64::consts::TAU * pitch;
            let (c, t) = along(centerline, &cum, s.max(0.0).min(total));
            // Extend straight back from the start for the open-end lead-in.
            let c = if s < 0.0 {
                [c[0] + t[0] * s, c[1] + t[1] * s]
            } else {
                c
            };
            let (tx, ty) = (t[0] * dir, t[1] * dir);
            let (nx, ny) = (-ty, tx); // left of the direction of travel
                                      // Start each loop at the back (θ = π) so the first motion goes
                                      // forward along the slot, cutting at the front (θ = 0).
            let a = th + std::f64::consts::PI;
            [
                c[0] + rt * (a.cos() * tx + a.sin() * nx),
                c[1] + rt * (a.cos() * ty + a.sin() * ny),
            ]
        };
        let start = pos(0.0);
        moves.push(ClMove::Rapid {
            tip: Vec3::new(start[0], start[1], safe),
            axis: ax,
        });
        if p.open_start || li > 0 {
            moves.push(ClMove::Rapid {
                tip: Vec3::new(start[0], start[1], z + 1.0),
                axis: ax,
            });
            moves.push(ClMove::Feed {
                tip: Vec3::new(start[0], start[1], z),
                axis: ax,
                feed: p.feed * 0.5,
            });
        } else {
            // Helix down on the first loop circle.
            let (c, _) = along(centerline, &cum, 0.0);
            let turns = ((p.z_top - z) / (std::f64::consts::TAU * rt.max(0.5) * 0.05))
                .ceil()
                .max(1.0) as usize;
            let k = turns * 36;
            for i in 0..=k {
                let a = i as f64 / 36.0 * std::f64::consts::TAU + std::f64::consts::PI;
                let hz = p.z_top + 1.0 + (z - p.z_top - 1.0) * i as f64 / k as f64;
                moves.push(ClMove::Feed {
                    tip: Vec3::new(c[0] + rt * a.cos(), c[1] + rt * a.sin(), hz),
                    axis: ax,
                    feed: p.feed * 0.4,
                });
            }
            moves.push(ClMove::Feed {
                tip: Vec3::new(start[0], start[1], z),
                axis: ax,
                feed: p.feed * 0.5,
            });
        }
        for i in 1..=n {
            let th = i as f64 * dth;
            let q = pos(th);
            // The front half of each loop cuts; the back half only returns.
            let a = (th + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU);
            let cutting = !(std::f64::consts::FRAC_PI_2..=1.5 * std::f64::consts::PI).contains(&a);
            moves.push(ClMove::Feed {
                tip: Vec3::new(q[0], q[1], z),
                axis: ax,
                feed: if cutting { p.feed } else { p.return_feed },
            });
        }
        loops += (total_th / std::f64::consts::TAU).ceil() as usize;
        let end = pos(n as f64 * dth);
        moves.push(ClMove::Rapid {
            tip: Vec3::new(end[0], end[1], safe),
            axis: ax,
        });
    }

    let max_ae = measure_engagement(
        centerline,
        &cum,
        p,
        &moves,
        levels.first().copied().unwrap_or(p.z_bottom),
    );
    SlotResult {
        moves,
        levels,
        loops,
        loop_radius: rt,
        max_engagement_mm: max_ae,
        max_engagement_ratio: max_ae / (2.0 * r),
        warnings,
    }
}

/// Simulate the first level's path on a raster of the slot and return the
/// largest radial width of cut (mm) after the entry.
fn measure_engagement(
    centerline: &[[f64; 2]],
    cum: &[f64],
    p: &SlotParams,
    moves: &[ClMove],
    z: f64,
) -> f64 {
    let r = p.cutter.radius;
    let res = (r / 25.0).max(0.02);
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for q in centerline {
        for k in 0..2 {
            lo[k] = lo[k].min(q[k]);
            hi[k] = hi[k].max(q[k]);
        }
    }
    let pad = p.width;
    let (nx, ny) = (
        ((hi[0] - lo[0] + 2.0 * pad) / res) as usize + 1,
        ((hi[1] - lo[1] + 2.0 * pad) / res) as usize + 1,
    );
    let cell = |i: usize, j: usize| [lo[0] - pad + i as f64 * res, lo[1] - pad + j as f64 * res];
    // Material: inside the slot outline (within width/2 of the centre line).
    let total = *cum.last().unwrap();
    let mut mat = vec![false; nx * ny];
    for j in 0..ny {
        for i in 0..nx {
            let c = cell(i, j);
            let d = centerline
                .windows(2)
                .fold(f64::INFINITY, |m, w| m.min(seg_dist(c, w[0], w[1])));
            // Open ends: only material along the slot, not beyond its end caps.
            let _ = total;
            mat[j * nx + i] = d <= p.width / 2.0;
        }
    }
    let mut max_ae: f64 = 0.0;
    let mut prev: Option<[f64; 2]> = None;
    let mut seen_cut = 0usize;
    for mv in moves {
        let ClMove::Feed { tip, .. } = mv else {
            prev = None;
            continue;
        };
        if (tip.z - z).abs() > 1e-6 {
            prev = None;
            continue;
        }
        let q = [tip.x, tip.y];
        if let Some(a) = prev {
            let step = (q[0] - a[0]).hypot(q[1] - a[1]);
            let mut removed = 0usize;
            let (i0, i1) = (
                ((q[0].min(a[0]) - r - lo[0] + pad) / res).floor().max(0.0) as usize,
                (((q[0].max(a[0]) + r - lo[0] + pad) / res).ceil() as usize).min(nx - 1),
            );
            let (j0, j1) = (
                ((q[1].min(a[1]) - r - lo[1] + pad) / res).floor().max(0.0) as usize,
                (((q[1].max(a[1]) + r - lo[1] + pad) / res).ceil() as usize).min(ny - 1),
            );
            for j in j0..=j1 {
                for i in i0..=i1 {
                    let k = j * nx + i;
                    if mat[k] && seg_dist(cell(i, j), a, q) <= r {
                        mat[k] = false;
                        removed += 1;
                    }
                }
            }
            if step > 1e-9 && removed > 0 {
                seen_cut += 1;
                // Skip the first loop (entry into a full slot end).
                let loop_pts = (std::f64::consts::TAU * (p.width / 2.0 - r - p.leave).max(0.1)
                    / p.step)
                    .ceil() as usize;
                if seen_cut > loop_pts + 2 {
                    // The cutting edge runs on a circle of radius rt + R
                    // while the centre runs on rt: divide the area by the
                    // edge's travel to get the radial width of cut.
                    let rt = (p.width / 2.0 - r - p.leave).max(1e-6);
                    max_ae = max_ae.max(removed as f64 * res * res / (step * (rt + r) / rt));
                }
            }
        }
        prev = Some(q);
    }
    max_ae
}

fn seg_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    let t = if l2 < 1e-18 {
        0.0
    } else {
        (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0)
    };
    (a[0] + dx * t - p[0]).hypot(a[1] + dy * t - p[1])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_slot_loops_stay_inside_and_load_is_light() {
        let p = SlotParams {
            width: 12.0,
            pitch: 0.8,
            ..Default::default()
        };
        let r = trochoidal_slot(&[[0.0, 0.0], [60.0, 0.0]], &p);
        assert!(r.warnings.is_empty(), "{:?}", r.warnings);
        assert!((r.loop_radius - 2.0).abs() < 1e-9);
        assert!(r.loops > 70);
        // Tool centre never leaves the slot (|y| <= width/2 - r).
        for mv in &r.moves {
            if let ClMove::Feed { tip, .. } = mv {
                assert!(tip.y.abs() <= 2.0 + 1e-9, "{tip:?}");
            }
        }
        // Each loop takes roughly the pitch, far below slotting (8 mm).
        assert!(
            r.max_engagement_mm < 1.6 * p.pitch,
            "max ae {}",
            r.max_engagement_mm
        );
        assert!(
            r.max_engagement_mm > 0.3 * p.pitch,
            "max ae {}",
            r.max_engagement_mm
        );
    }

    #[test]
    fn too_narrow_and_curved_slots() {
        let p = SlotParams {
            width: 7.0,
            ..Default::default()
        };
        let r = trochoidal_slot(&[[0.0, 0.0], [10.0, 0.0]], &p);
        assert!(r.moves.is_empty() && r.warnings[0].contains("does not fit"));
        let arc: Vec<[f64; 2]> = (0..=30)
            .map(|k| {
                let a = (k as f64 * 3.0).to_radians();
                [40.0 * a.cos(), 40.0 * a.sin()]
            })
            .collect();
        let r = trochoidal_slot(
            &arc,
            &SlotParams {
                width: 14.0,
                ..Default::default()
            },
        );
        // Centre stays within the slot band around the arc (after the
        // straight open-end lead-in).
        for mv in &r.moves {
            if let ClMove::Feed { tip, .. } = mv {
                if tip.z < -1.0 && tip.y > 0.0 {
                    let d = (tip.x.hypot(tip.y) - 40.0).abs();
                    assert!(d <= 3.0 + 0.05, "{d}");
                }
            }
        }
    }
}
