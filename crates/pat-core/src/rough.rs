//! 3D adaptive roughing (z-level), in any fixed tool orientation.
//!
//! Works in a machining frame F whose +Z is the tool axis: identity for
//! 3-axis work, a tilted frame for 3+2. For each level:
//!
//! 1. **Where the tool may go.** The raster drop-cutter gives the lowest safe
//!    tip height over the part (grown by the stock to leave); the tool centre
//!    is allowed wherever that height is at or below the level.
//! 2. **Where material is.** The remaining (in-process) stock is looked at
//!    from above in F. Feeding a part-machined stock gives rest machining for
//!    free: only material that is still there gets cut.
//! 3. **Clearing.** [`crate::adaptive`] clears the level, and the passes are
//!    turned into 3D cutter-location moves (entries, helixes, links,
//!    retracts).
//!
//! The stock is updated by sweeping the cutter along every feed move, so the
//! result can feed the next operation.
//!
//! Levels are spaced by the step-down, plus extra levels at flat areas of the
//! part, so floors are roughed to an even stock allowance.

use crate::adaptive::{adaptive_clear, AdaptiveParams, PassKind};
use crate::geom::Mesh;
use crate::kinematics::Pose;
use crate::math::{Iso, Vec3};
use crate::post::ClMove;
use crate::raster::{assembly_limit, height_field, stock_top, Grid2};
use crate::stock::{CutterShape, Voxels};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct RoughParams {
    pub cutter: CutterShape,
    /// Depth per level (mm).
    pub stepdown: f64,
    /// Target radial width of cut (mm).
    pub stepover: f64,
    /// Material left on the part, radially and axially (mm).
    pub leave: f64,
    /// Raster cell size (mm).
    pub resolution: f64,
    /// Clearance above the stock for rapids (mm).
    pub clearance: f64,
    /// Helix ramp angle (degrees).
    pub ramp_angle_deg: f64,
    /// How far outside the stock the tool centre may travel (mm, beyond its radius).
    pub outside_margin: f64,
    pub feed: f64,
    pub plunge_feed: f64,
    pub link_feed: f64,
    /// Add levels at flat areas of the part.
    pub flat_levels: bool,
    /// Clearance kept between the tool assembly and fixtures / machine
    /// bodies (mm).
    pub obstacle_clearance: f64,
}

impl Default for RoughParams {
    fn default() -> Self {
        RoughParams {
            cutter: CutterShape::flat(10.0, 30.0),
            stepdown: 3.0,
            stepover: 1.5,
            leave: 0.3,
            resolution: 0.25,
            clearance: 5.0,
            ramp_angle_deg: 3.0,
            outside_margin: 2.0,
            feed: 2000.0,
            plunge_feed: 600.0,
            link_feed: 3000.0,
            flat_levels: true,
            obstacle_clearance: 2.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LevelStats {
    pub z: f64,
    pub passes: usize,
    pub entries: usize,
    pub cut_length_mm: f64,
    /// Reachable material the clearing could not get to (mm²).
    pub uncut_area_mm2: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RoughResult {
    /// Cutter-location moves in WCS coordinates (tool axis constant).
    pub moves: Vec<ClMove>,
    pub levels: Vec<LevelStats>,
    pub removed_mm3: f64,
    pub remaining_mm3: f64,
    /// Plain-language warnings (e.g. flute shorter than the cut depth).
    pub warnings: Vec<String>,
}

/// Rough `stock` down to `part` (both in the table frame) along frame `f`
/// (F → table frame). CL output is in WCS coordinates: table frame minus
/// `wcs_origin`. The stock is updated in place.
pub fn rough(
    parts: &[&Mesh],
    stock: &mut Voxels,
    f: &Iso,
    wcs_origin: Vec3,
    p: &RoughParams,
) -> RoughResult {
    rough_with(parts, &[], &[], stock, f, wcs_origin, p)
}

/// [`rough`] with the rest of the setup taken into account:
///
/// - `obstacles` are fixtures and table-side machine bodies (table frame).
///   The whole tool assembly keeps `obstacle_clearance` from them.
/// - `sections` are the non-cutting parts riding with the tool (shank,
///   holder, spindle housing) as (height above tip, radius). They are kept
///   clear of the part too, so holders never touch walls in deep pockets.
pub fn rough_with(
    parts: &[&Mesh],
    obstacles: &[&Mesh],
    sections: &[(f64, f64)],
    stock: &mut Voxels,
    f: &Iso,
    wcs_origin: Vec3,
    p: &RoughParams,
) -> RoughResult {
    let mut warnings = Vec::new();
    let before = stock.volume();
    let axis = f.apply_vec(Vec3::Z);
    let r = p.cutter.radius;

    // Stock extent in F (occupied voxels only, not the padded grid).
    let Some(sb) = stock.occupied_bounds() else {
        return RoughResult {
            moves: vec![],
            levels: vec![],
            removed_mm3: 0.0,
            remaining_mm3: 0.0,
            warnings: vec!["the stock is empty".into()],
        };
    };
    let corners: Vec<Vec3> = (0..8)
        .map(|i| {
            f.inverse().apply_point(Vec3::new(
                if i & 1 == 0 { sb.min.x } else { sb.max.x },
                if i & 2 == 0 { sb.min.y } else { sb.max.y },
                if i & 4 == 0 { sb.min.z } else { sb.max.z },
            ))
        })
        .collect();
    let lo = corners.iter().fold(
        Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        |a, c| Vec3::new(a.x.min(c.x), a.y.min(c.y), a.z.min(c.z)),
    );
    let hi = corners.iter().fold(
        Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        |a, c| Vec3::new(a.x.max(c.x), a.y.max(c.y), a.z.max(c.z)),
    );
    let pad = r + p.outside_margin;
    let gmin = Vec3::new(lo.x - pad, lo.y - pad, 0.0);
    let gmax = Vec3::new(hi.x + pad, hi.y + pad, 0.0);
    let res = p.resolution.max(0.02);

    // Part surface and tool limit.
    let mut hf: Grid2<f64> = Grid2::covering(gmin, gmax, res, f64::NEG_INFINITY);
    for m in parts {
        let h = height_field(m, f, gmin, gmax, res);
        for (a, b) in hf.data.iter_mut().zip(h.data) {
            *a = a.max(b);
        }
    }
    // Surface edges are sampled at cell centres and tool positions round to a
    // cell: together up to ~1.4 cells of error, so grow by 1.5 cells.
    let raster_margin = 1.5 * res;
    let mut limit = assembly_limit(&hf, &p.cutter, sections, p.leave, raster_margin);
    if !obstacles.is_empty() {
        // Obstacles far outside the stock still matter for wide sections
        // (spindle housing), so look at them over a padded grid. They are
        // kept `obstacle_clearance` away, so a coarser grid (with a matching
        // safety margin) is accurate enough and much cheaper.
        let ores = (res * 2.0).max(0.5);
        let reach = sections.iter().map(|s| s.1).fold(r, f64::max) + p.obstacle_clearance + ores;
        let pad = (reach / ores).ceil() * ores;
        let padv = Vec3::new(pad, pad, 0.0);
        let (bmin, bmax) = (gmin - padv, gmax + padv);
        let mut oh: Grid2<f64> = Grid2::covering(bmin, bmax, ores, f64::NEG_INFINITY);
        for m in obstacles {
            let h = height_field(m, f, bmin, bmax, ores);
            for (a, b) in oh.data.iter_mut().zip(h.data) {
                *a = a.max(b);
            }
        }
        let ol = assembly_limit(&oh, &p.cutter, sections, p.obstacle_clearance, 1.5 * ores);
        for j in 0..limit.ny {
            for i in 0..limit.nx {
                let (x, y) = limit.xy(i, j);
                if let Some((oi, oj)) = ol.cell(x, y) {
                    let k = limit.idx(i, j);
                    limit.data[k] = limit.data[k].max(*ol.get(oi, oj));
                }
            }
        }
    }
    let top = stock_top(stock, f, &hf, hi.z + 1.0, hi.z - lo.z + 2.0);
    let z_top = top.data.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if !z_top.is_finite() {
        warnings.push("no stock material in view from this direction".into());
        return RoughResult {
            moves: vec![],
            levels: vec![],
            removed_mm3: 0.0,
            remaining_mm3: before,
            warnings,
        };
    }
    let z_floor = lo.z;
    let safe_z = z_top + p.clearance;

    // Levels: step-down spacing plus flat areas.
    let mut levels: Vec<f64> = Vec::new();
    let mut z = z_top - p.stepdown;
    while z > z_floor + 1e-6 {
        levels.push(z);
        z -= p.stepdown;
    }
    if p.flat_levels {
        let mut counts: std::collections::BTreeMap<i64, usize> = std::collections::BTreeMap::new();
        let q = res * 0.05;
        for &v in &limit.data {
            if v.is_finite() && v < z_top - 1e-6 && v > z_floor + 1e-6 {
                *counts.entry((v / q).round() as i64).or_default() += 1;
            }
        }
        let need = (limit.data.len() / 200).max(20);
        for (&k, &c) in &counts {
            if c >= need {
                let fz = k as f64 * q;
                levels.retain(|&l| (l - fz).abs() > 0.2 * p.stepdown);
                levels.push(fz);
            }
        }
    }
    levels.sort_by(|a, b| b.total_cmp(a));
    levels.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    let max_depth = z_top - levels.last().copied().unwrap_or(z_top);
    if max_depth > p.cutter.flute_length {
        warnings.push(format!(
            "roughing depth {max_depth:.1} mm exceeds the flute length {:.1} mm; check the shank and holder against the walls",
            p.cutter.flute_length
        ));
    }

    let to_wcs = |x: f64, y: f64, z: f64| f.apply_point(Vec3::new(x, y, z)) - wcs_origin;
    let mut moves: Vec<ClMove> = Vec::new();
    let mut stats = Vec::new();
    let mut tool_down = false;
    // Sweep the stock along feed moves as they are emitted.
    let mut last: Option<Vec3> = None;
    let mut emit = |moves: &mut Vec<ClMove>, stock: &mut Voxels, tip: Vec3, feed: Option<f64>| {
        let table_tip = tip + wcs_origin;
        match feed {
            Some(fr) => {
                if let Some(a) = last {
                    stock.sweep(
                        &Pose { tip: a, axis },
                        &Pose {
                            tip: table_tip,
                            axis,
                        },
                        &p.cutter,
                    );
                }
                moves.push(ClMove::Feed {
                    tip,
                    axis,
                    feed: fr,
                });
            }
            None => moves.push(ClMove::Rapid { tip, axis }),
        }
        last = Some(table_tip);
    };

    let mut top_now = top.clone();
    for &z in &levels {
        // Stock as it stands after the levels above (and earlier setups).
        top_now = crate::raster::stock_top_after(stock, f, &top_now);
        let top = &top_now;
        // Shank, holder and housing must stay clear of the stock still
        // standing above the cut.
        let mut section_limit = limit.like(f64::NEG_INFINITY);
        for &(z0, sr) in sections {
            let m = crate::raster::disk_max(top, sr + 0.5 + raster_margin);
            for (a, b) in section_limit.data.iter_mut().zip(&m.data) {
                if b.is_finite() {
                    *a = a.max(b - z0 + 0.5);
                }
            }
        }
        let mut allowed = limit.like(false);
        let mut material = limit.like(false);
        for k in 0..limit.data.len() {
            allowed.data[k] = limit.data[k] <= z + 1e-9 && section_limit.data[k] <= z + 1e-9;
            material.data[k] = top.data[k] >= z + 0.5 * res.min(p.stepdown * 0.1);
        }
        let ap = AdaptiveParams {
            tool_radius: r,
            stepover: p.stepover,
            ..Default::default()
        };
        let result = adaptive_clear(&allowed, &material, &ap);
        let mut st = LevelStats {
            z,
            passes: 0,
            entries: 0,
            cut_length_mm: 0.0,
            uncut_area_mm2: result.uncut_reachable_area,
        };

        for pass in &result.passes {
            let first = pass.pts[0];
            match &pass.kind {
                PassKind::AirEntry | PassKind::HelixEntry { .. } => {
                    st.entries += 1;
                    if tool_down {
                        let at = moves.last().map(|m| m.pose().tip).unwrap();
                        let local = f.inverse().apply_point(at + wcs_origin);
                        emit(&mut moves, stock, to_wcs(local.x, local.y, safe_z), None);
                    }
                    // Start height: just above any material around the entry.
                    let local_top = cells_max(top, first, r * 2.0);
                    let start_z = local_top.max(z) + 1.0;
                    match pass.kind {
                        PassKind::HelixEntry { radius } if radius > 0.05 => {
                            let (cx, cy) = (first[0], first[1]);
                            emit(&mut moves, stock, to_wcs(cx + radius, cy, safe_z), None);
                            emit(&mut moves, stock, to_wcs(cx + radius, cy, start_z), None);
                            let drop_per_rev = std::f64::consts::TAU
                                * radius
                                * p.ramp_angle_deg.to_radians().tan();
                            let revs = ((start_z - z) / drop_per_rev.max(1e-3)).max(0.0);
                            let n = ((revs * 36.0).ceil() as usize).max(8);
                            for k in 1..=n {
                                let t = k as f64 / n as f64;
                                let a = t * revs * std::f64::consts::TAU;
                                let hz = start_z + (z - start_z) * t;
                                emit(
                                    &mut moves,
                                    stock,
                                    to_wcs(cx + radius * a.cos(), cy + radius * a.sin(), hz),
                                    Some(p.plunge_feed),
                                );
                            }
                            // One full circle at depth to clean the hole, then to the centre.
                            for k in 1..=36 {
                                let a = revs * std::f64::consts::TAU
                                    + k as f64 / 36.0 * std::f64::consts::TAU;
                                emit(
                                    &mut moves,
                                    stock,
                                    to_wcs(cx + radius * a.cos(), cy + radius * a.sin(), z),
                                    Some(p.plunge_feed),
                                );
                            }
                            emit(&mut moves, stock, to_wcs(cx, cy, z), Some(p.feed));
                        }
                        PassKind::HelixEntry { .. } => {
                            emit(&mut moves, stock, to_wcs(first[0], first[1], safe_z), None);
                            emit(&mut moves, stock, to_wcs(first[0], first[1], start_z), None);
                            emit(
                                &mut moves,
                                stock,
                                to_wcs(first[0], first[1], z),
                                Some(p.plunge_feed),
                            );
                        }
                        _ => {
                            emit(&mut moves, stock, to_wcs(first[0], first[1], safe_z), None);
                            emit(&mut moves, stock, to_wcs(first[0], first[1], z + 1.0), None);
                            emit(
                                &mut moves,
                                stock,
                                to_wcs(first[0], first[1], z),
                                Some(p.plunge_feed),
                            );
                        }
                    }
                    tool_down = true;
                }
                PassKind::Link => {
                    for q in pass.pts.iter().skip(1) {
                        emit(&mut moves, stock, to_wcs(q[0], q[1], z), Some(p.link_feed));
                    }
                }
                PassKind::Cut => {
                    st.passes += 1;
                    for w in pass.pts.windows(2) {
                        st.cut_length_mm += (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]);
                    }
                    for q in pass.pts.iter().skip(1) {
                        emit(&mut moves, stock, to_wcs(q[0], q[1], z), Some(p.feed));
                    }
                }
            }
        }
        // Retract between levels.
        if tool_down {
            let at = moves.last().map(|m| m.pose().tip).unwrap();
            let local = f.inverse().apply_point(at + wcs_origin);
            emit(&mut moves, stock, to_wcs(local.x, local.y, safe_z), None);
            tool_down = false;
        }
        stats.push(st);
    }
    let after = stock.volume();
    RoughResult {
        moves,
        levels: stats,
        removed_mm3: before - after,
        remaining_mm3: after,
        warnings,
    }
}

fn cells_max(g: &Grid2<f64>, c: [f64; 2], rad: f64) -> f64 {
    let mut m = f64::NEG_INFINITY;
    let n = (rad / g.res).ceil() as isize;
    if let Some((i, j)) = g.cell(c[0], c[1]) {
        for dj in -n..=n {
            for di in -n..=n {
                let (a, b) = (i as isize + di, j as isize + dj);
                if a >= 0 && b >= 0 && (a as usize) < g.nx && (b as usize) < g.ny {
                    m = m.max(*g.get(a as usize, b as usize));
                }
            }
        }
    }
    m
}

/// Frame for machining along `axis` with origin `origin` (both in the table
/// frame), keeping F's X close to the table X.
pub fn machining_frame(origin: Vec3, axis: Vec3) -> Iso {
    let z = axis.normalized().unwrap_or(Vec3::Z);
    let x0 = if z.x.abs() < 0.95 { Vec3::X } else { Vec3::Y };
    let x = (x0 - z * x0.dot(z)).normalized().unwrap_or(Vec3::X);
    let y = z.cross(x);
    Iso {
        r: crate::math::Mat3::from_cols(x, y, z),
        t: origin,
    }
}
