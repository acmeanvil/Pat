//! Continuous 4-axis (rotary) machining.
//!
//! Works in cylindrical coordinates around the rotary axis: x along the axis,
//! θ around it, r out from it. The tool always points at the axis (radially),
//! which is how a VMC with a 4th axis cuts.
//!
//! - [`radius_map`]: outermost part radius at each (x, θ).
//! - [`rotary_limit`]: lowest safe tip radius at each (x, θ) for a radial
//!   tool (cutter grown by the stock to leave, plus shank and holder). This is
//!   the cylindrical counterpart of the raster drop-cutter.
//! - [`rotary_rough`]: radial levels cut with continuous helical passes (X
//!   advances while the rotary spins), retracting around places the tool may
//!   not go. The material comes from the in-process stock, so rest machining
//!   works as in [`crate::rough`].
//! - [`rotary_finish`]: a fine helix following the surface.
//! - [`wrap`] / [`unwrap`]: map 2D paths onto a cylinder and back
//!   (engraving, flat patterns, previews).

use crate::geom::{segment_triangle_param, Aabb, Bvh, Mesh};
use crate::kinematics::Pose;
use crate::machine::{Chain, MachineDef};
use crate::math::Vec3;
use crate::post::ClMove;
use crate::stock::{CutterShape, Voxels};
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};

/// Cylindrical frame around a rotary axis, in the table frame.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RotaryFrame {
    /// A point on the axis.
    pub origin: Vec3,
    /// Axis direction (x grows along it).
    pub axis: Vec3,
    /// Direction of θ = 0 (perpendicular to the axis).
    pub zero: Vec3,
}

impl RotaryFrame {
    pub fn new(origin: Vec3, axis: Vec3, toward: Vec3) -> RotaryFrame {
        let a = axis.normalized().unwrap_or(Vec3::X);
        let c = (toward - a * toward.dot(a))
            .normalized()
            .unwrap_or_else(|| {
                let h = if a.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
                (h - a * h.dot(a)).normalized().unwrap()
            });
        RotaryFrame {
            origin,
            axis: a,
            zero: c,
        }
    }

    /// The machine's table rotary axis (exactly one), with θ = 0 toward the
    /// spindle.
    pub fn from_machine(def: &MachineDef) -> Option<RotaryFrame> {
        let mut table_rot = def
            .axes
            .iter()
            .filter(|a| a.is_rotary() && a.chain == Chain::Table);
        let ax = table_rot.next()?;
        if table_rot.next().is_some() {
            return None;
        }
        Some(RotaryFrame::new(
            ax.point,
            ax.direction,
            def.spindle.direction,
        ))
    }

    fn side(&self) -> Vec3 {
        self.axis.cross(self.zero)
    }

    /// Unit radial direction at angle θ (radians).
    pub fn radial(&self, theta: f64) -> Vec3 {
        self.zero * theta.cos() + self.side() * theta.sin()
    }

    pub fn point(&self, x: f64, theta: f64, r: f64) -> Vec3 {
        self.origin + self.axis * x + self.radial(theta) * r
    }

    /// (x, θ in [0, 2π), r) of a point.
    pub fn cyl(&self, p: Vec3) -> (f64, f64, f64) {
        let d = p - self.origin;
        let x = d.dot(self.axis);
        let (u, v) = (d.dot(self.zero), d.dot(self.side()));
        (x, v.atan2(u).rem_euclid(TAU), u.hypot(v))
    }
}

/// Values on an (x, θ) grid; θ wraps around.
#[derive(Clone, Debug, PartialEq)]
pub struct CylGrid {
    pub x0: f64,
    pub dx: f64,
    pub nx: usize,
    pub nt: usize,
    pub data: Vec<f64>,
}

impl CylGrid {
    pub fn new(x_min: f64, x_max: f64, dx: f64, nt: usize, fill: f64) -> CylGrid {
        let nx = (((x_max - x_min) / dx).ceil() as usize).max(1) + 1;
        CylGrid {
            x0: x_min,
            dx,
            nx,
            nt: nt.max(8),
            data: vec![fill; nx * nt.max(8)],
        }
    }

    pub fn dtheta(&self) -> f64 {
        TAU / self.nt as f64
    }

    pub fn x(&self, i: usize) -> f64 {
        self.x0 + i as f64 * self.dx
    }

    pub fn theta(&self, j: usize) -> f64 {
        j as f64 * self.dtheta()
    }

    pub fn get(&self, i: usize, j: usize) -> f64 {
        self.data[i * self.nt + j % self.nt]
    }

    pub fn set(&mut self, i: usize, j: usize, v: f64) {
        let k = i * self.nt + j % self.nt;
        self.data[k] = v;
    }

    /// Value at the nearest cell, or `None` outside the x range.
    pub fn at(&self, x: f64, theta: f64) -> Option<f64> {
        let i = ((x - self.x0) / self.dx).round();
        if i < 0.0 || i as usize >= self.nx {
            return None;
        }
        let j = (theta.rem_euclid(TAU) / self.dtheta()).round() as usize % self.nt;
        Some(self.get(i as usize, j))
    }

    /// Largest value among the cells around (x, θ): a conservative lookup.
    pub fn at_max(&self, x: f64, theta: f64) -> Option<f64> {
        let fi = (x - self.x0) / self.dx;
        if fi < -0.5 || fi > self.nx as f64 - 0.5 {
            return None;
        }
        let fj = theta.rem_euclid(TAU) / self.dtheta();
        let (i0, j0) = (fi.floor().max(0.0) as usize, fj.floor() as usize);
        let i1 = (i0 + 1).min(self.nx - 1);
        Some(
            [(i0, j0), (i1, j0), (i0, j0 + 1), (i1, j0 + 1)]
                .iter()
                .map(|&(i, j)| self.get(i, j))
                .fold(f64::NEG_INFINITY, f64::max),
        )
    }
}

/// Outermost radius of `meshes` (table frame) on each (x, θ) ray. −∞ where
/// the ray misses.
pub fn radius_map(
    meshes: &[&Mesh],
    f: &RotaryFrame,
    x_min: f64,
    x_max: f64,
    dx: f64,
    nt: usize,
    r_max: f64,
) -> CylGrid {
    let mut all = Mesh::default();
    for m in meshes {
        all.append(m);
    }
    let bvh = Bvh::build(&all);
    let mut g = CylGrid::new(x_min, x_max, dx, nt, f64::NEG_INFINITY);
    // Slight offsets keep rays off shared edges.
    let jitter = 1.3e-7;
    for i in 0..g.nx {
        for j in 0..g.nt {
            let (x, th) = (g.x(i) + jitter, g.theta(j) + jitter);
            let p0 = f.point(x, th, 0.0);
            let p1 = f.point(x, th, r_max);
            let q = Aabb::from_points(&[p0, p1]);
            let mut best = f64::NEG_INFINITY;
            bvh.any_in(&q, |t| {
                if let Some(w) = segment_triangle_param(p0, p1, &all.tri(t)) {
                    best = best.max(w * r_max);
                }
                false
            });
            g.set(i, j, best);
        }
    }
    g
}

/// Outer radius of the remaining voxel stock on each ray of `grid`.
pub fn stock_radius(stock: &Voxels, f: &RotaryFrame, grid: &CylGrid, r_max: f64) -> CylGrid {
    let mut out = CylGrid {
        data: vec![f64::NEG_INFINITY; grid.data.len()],
        ..grid.clone()
    };
    for i in 0..grid.nx {
        for j in 0..grid.nt {
            let th = grid.theta(j);
            let p = f.point(grid.x(i), th, r_max);
            if let Some(d) = stock.first_material_along(p, f.radial(th), r_max) {
                out.set(i, j, r_max + d);
            }
        }
    }
    out
}

/// Lowest tip radius per cell for a radial tool: the cutter grown by
/// `leave` (plus `margin` radially for raster error) and the non-cutting
/// `sections` (height above tip, radius) must clear the surface `rmap`.
pub fn rotary_limit(
    rmap: &CylGrid,
    cutter: &CutterShape,
    sections: &[(f64, f64)],
    leave: f64,
    margin: f64,
) -> CylGrid {
    let grown = CutterShape {
        radius: cutter.radius + leave + margin,
        corner_radius: cutter.corner_radius + leave,
        flute_length: cutter.flute_length,
    };
    let mut out = CylGrid {
        data: vec![f64::NEG_INFINITY; rmap.data.len()],
        ..rmap.clone()
    };
    let r_lo = rmap
        .data
        .iter()
        .copied()
        .filter(|v| v.is_finite() && *v > 0.0)
        .fold(f64::INFINITY, f64::min);
    let dt = rmap.dtheta();

    // One footprint pass: `reach` is the lateral radius, `lift(ρ)` how far
    // above the tip that part of the assembly starts.
    let pass = |out: &mut CylGrid, reach: f64, lift: &dyn Fn(f64) -> f64, stride: usize| {
        let r_ref = r_lo.max(reach).max(1e-6);
        let di_max = (reach / rmap.dx).ceil() as isize;
        let dj_max =
            (((reach / r_ref).min(1.0).asin() / dt).ceil() as isize).min(rmap.nt as isize / 4);
        let mut coarse = CylGrid {
            data: vec![f64::NEG_INFINITY; rmap.data.len()],
            ..rmap.clone()
        };
        for i in (0..rmap.nx).step_by(stride) {
            for j in (0..rmap.nt).step_by(stride) {
                let mut best = f64::NEG_INFINITY;
                for di in -di_max..=di_max {
                    let ii = i as isize + di;
                    if ii < 0 || ii as usize >= rmap.nx {
                        continue;
                    }
                    let dxv = di as f64 * rmap.dx;
                    for dj in -dj_max..=dj_max {
                        let jj = (j as isize + dj).rem_euclid(rmap.nt as isize) as usize;
                        let rp = rmap.get(ii as usize, jj);
                        if !rp.is_finite() {
                            continue;
                        }
                        let a = dj as f64 * dt;
                        let lat = rp * a.sin();
                        let rho = (dxv * dxv + lat * lat).sqrt();
                        if rho <= reach {
                            best = best.max(rp * a.cos() - lift(rho));
                        }
                    }
                }
                coarse.set(i, j, best);
            }
        }
        // Spread coarse results conservatively to every fine cell.
        for i in 0..rmap.nx {
            for j in 0..rmap.nt {
                let (ci, cj) = (i / stride * stride, j / stride * stride);
                let mut v = f64::NEG_INFINITY;
                for (a, b) in [
                    (ci, cj),
                    (ci + stride, cj),
                    (ci, cj + stride),
                    (ci + stride, cj + stride),
                ] {
                    if a < rmap.nx {
                        v = v.max(coarse.get(a, b % rmap.nt));
                    }
                }
                let k = i * rmap.nt + j;
                if v.is_finite() {
                    out.data[k] = out.data[k].max(v + leave);
                }
            }
        }
    };
    pass(
        &mut out,
        grown.radius,
        &|rho| grown.profile_height(rho).unwrap_or(0.0),
        1,
    );
    for &(z0, sr) in sections {
        // Wide sections are evaluated on a coarser lattice; grow them by
        // that lattice's spacing so the result stays conservative.
        let stride = ((sr / 8.0 / rmap.dx).floor() as usize).max(1);
        let pad = stride as f64 * rmap.dx * 1.5;
        pass(&mut out, sr + leave + margin + pad, &|_| z0, stride);
    }
    out
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct RotaryParams {
    pub cutter: CutterShape,
    /// Radial depth per level (mm).
    pub stepdown: f64,
    /// Helix pitch: axial advance per revolution (mm).
    pub stepover: f64,
    /// Stock left on the part (mm).
    pub leave: f64,
    /// Axial grid spacing (mm).
    pub dx: f64,
    /// Angular grid spacing (degrees).
    pub dtheta_deg: f64,
    /// Path point spacing along the surface (mm).
    pub step: f64,
    /// Clearance above the stock for retracts (mm).
    pub clearance: f64,
    pub feed: f64,
    pub plunge_feed: f64,
    /// Add levels at cylindrical areas of the part.
    pub flat_levels: bool,
    /// Axial range (frame x) the tool centre may cut in. Outside it the
    /// stock is held (chuck, tailstock) and must not be machined away.
    /// Default: the part's axial extent.
    pub x_range: Option<(f64, f64)>,
}

impl Default for RotaryParams {
    fn default() -> Self {
        RotaryParams {
            cutter: CutterShape::flat(6.0, 25.0),
            stepdown: 2.0,
            stepover: 2.0,
            leave: 0.3,
            dx: 0.5,
            dtheta_deg: 1.0,
            step: 1.0,
            clearance: 5.0,
            feed: 1500.0,
            plunge_feed: 500.0,
            flat_levels: true,
            x_range: None,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RotaryResult {
    pub moves: Vec<ClMove>,
    /// Radii of the roughing levels (empty for finishing).
    pub levels: Vec<f64>,
    pub cut_length_mm: f64,
    pub removed_mm3: f64,
    pub remaining_mm3: f64,
    pub warnings: Vec<String>,
}

/// Emits CL moves in WCS and keeps the voxel stock in sync.
struct Emitter<'a> {
    f: RotaryFrame,
    wcs: Vec3,
    cutter: CutterShape,
    stock: Option<&'a mut Voxels>,
    moves: Vec<ClMove>,
    last: Option<Pose>,
    cut_len: f64,
}

impl Emitter<'_> {
    fn go(&mut self, x: f64, th: f64, r: f64, feed: Option<f64>) {
        let tip = self.f.point(x, th, r);
        let axis = self.f.radial(th);
        let pose = Pose { tip, axis };
        match feed {
            Some(fr) => {
                if let Some(a) = self.last {
                    self.cut_len += a.tip.distance(tip);
                    if let Some(s) = self.stock.as_deref_mut() {
                        s.sweep(&a, &pose, &self.cutter);
                    }
                }
                self.moves.push(ClMove::Feed {
                    tip: tip - self.wcs,
                    axis,
                    feed: fr,
                });
            }
            None => self.moves.push(ClMove::Rapid {
                tip: tip - self.wcs,
                axis,
            }),
        }
        self.last = Some(pose);
    }

    /// Radial retract to `r_safe` at the current angle and position.
    fn retract(&mut self, r_safe: f64) {
        if let Some(p) = self.last {
            let (x, th, _) = self.f.cyl(p.tip);
            self.go(x, th, r_safe, None);
        }
    }
}

/// Continuous 4-axis roughing of `stock` to `parts` (table frame) around
/// frame `f`. `sections` are the non-cutting parts of the tool assembly.
/// CL output is in WCS (table frame minus `wcs_origin`).
pub fn rotary_rough(
    parts: &[&Mesh],
    sections: &[(f64, f64)],
    stock: &mut Voxels,
    f: &RotaryFrame,
    wcs_origin: Vec3,
    p: &RotaryParams,
) -> RotaryResult {
    let before = stock.volume();
    let mut warnings = Vec::new();
    let Some(sb) = stock.occupied_bounds() else {
        return RotaryResult {
            moves: vec![],
            levels: vec![],
            cut_length_mm: 0.0,
            removed_mm3: 0.0,
            remaining_mm3: 0.0,
            warnings: vec!["the stock is empty".into()],
        };
    };
    let (x_lo, x_hi, r_max) = cyl_extent(f, &sb);
    let tr = p.cutter.radius + p.leave + 2.0;
    let nt = (360.0 / p.dtheta_deg.max(0.1)).round() as usize;
    let rmap = radius_map(parts, f, x_lo - tr, x_hi + tr, p.dx, nt, r_max + 1.0);
    let margin = 1.5 * p.dx.max(p.dtheta_deg.to_radians() * r_max);
    let mut limit = rotary_limit(&rmap, &p.cutter, sections, p.leave, margin);
    // Keep the held ends: no cutting outside the axial range.
    let (xr0, xr1) = p.x_range.unwrap_or_else(|| part_x_extent(parts, f));
    for i in 0..limit.nx {
        let x = limit.x(i);
        if x < xr0 - 1e-9 || x > xr1 + 1e-9 {
            for j in 0..limit.nt {
                limit.set(i, j, f64::INFINITY);
            }
        }
    }
    let srad = stock_radius(stock, f, &rmap, r_max + 1.0);
    let r_top = srad.data.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if !r_top.is_finite() {
        warnings.push("no stock material around the rotary axis".into());
        return RotaryResult {
            moves: vec![],
            levels: vec![],
            cut_length_mm: 0.0,
            removed_mm3: 0.0,
            remaining_mm3: before,
            warnings,
        };
    }
    let r_floor = limit
        .data
        .iter()
        .copied()
        .filter(|v| v.is_finite())
        .fold(f64::INFINITY, f64::min)
        .max(0.0);
    let r_safe = r_top + p.clearance;

    // Levels.
    let mut levels = Vec::new();
    let mut r = r_top - p.stepdown;
    while r > r_floor + 1e-6 {
        levels.push(r);
        r -= p.stepdown;
    }
    levels.push(
        r_floor
            .max(r_top - p.stepdown * (levels.len() as f64 + 1.0))
            .max(r_floor),
    );
    if p.flat_levels {
        let mut counts: std::collections::BTreeMap<i64, usize> = std::collections::BTreeMap::new();
        let q = p.dx * 0.05;
        for &v in &limit.data {
            if v.is_finite() && v < r_top - 1e-6 {
                *counts.entry((v / q).round() as i64).or_default() += 1;
            }
        }
        let need = (limit.data.len() / 50).max(20);
        for (&k, &c) in &counts {
            if c >= need {
                let fr = k as f64 * q;
                levels.retain(|&l| (l - fr).abs() > 0.2 * p.stepdown);
                levels.push(fr);
            }
        }
    }
    levels.sort_by(|a, b| b.total_cmp(a));
    levels.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    levels.retain(|&l| l > 0.0);

    let mut em = Emitter {
        f: *f,
        wcs: wcs_origin,
        cutter: p.cutter,
        stock: Some(stock),
        moves: Vec::new(),
        last: None,
        cut_len: 0.0,
    };
    let pitch = p.stepover.max(0.05);
    for (li, &r) in levels.iter().enumerate() {
        // Axial span with material at this level.
        let srad_now = stock_radius(em.stock.as_deref().unwrap(), f, &rmap, r_max + 1.0);
        let mut span: Option<(f64, f64)> = None;
        for i in 0..srad_now.nx {
            if (0..srad_now.nt).any(|j| srad_now.get(i, j) >= r + 1e-6) {
                let x = srad_now.x(i);
                span = Some(span.map_or((x, x), |(a, b)| (a.min(x), b.max(x))));
            }
        }
        let Some((xa, xb)) = span else { continue };
        let (xa, xb) = (
            (xa - p.cutter.radius).max(xr0),
            (xb + p.cutter.radius).min(xr1),
        );
        if xa > xb {
            continue;
        }
        // Alternate the axial direction each level to avoid long repositioning.
        let forward = li % 2 == 0;
        let (x_start, x_end) = if forward { (xa, xb) } else { (xb, xa) };
        let sign = if forward { 1.0 } else { -1.0 };
        let dth = (p.step / r.max(1.0)).min(5f64.to_radians());
        let total = ((x_end - x_start).abs() / pitch * TAU / dth).ceil() as usize;
        let mut down = false;
        let mut th = 0.0f64;
        for k in 0..=total {
            let x = x_start + sign * (k as f64 * dth / TAU * pitch);
            let ok = limit.at_max(x, th).is_none_or(|l| l <= r + 1e-9);
            if ok {
                if !down {
                    em.go(x, th, r_safe, None);
                    em.go(x, th, r, Some(p.plunge_feed));
                    down = true;
                } else {
                    em.go(x, th, r, Some(p.feed));
                }
            } else if down {
                em.retract(r_safe);
                down = false;
            }
            th += dth;
        }
        if down {
            em.retract(r_safe);
        }
    }
    let cut = em.cut_len;
    let moves = em.moves;
    let after = stock.volume();
    RotaryResult {
        moves,
        levels,
        cut_length_mm: cut,
        removed_mm3: before - after,
        remaining_mm3: after,
        warnings,
    }
}

/// Continuous 4-axis finishing: a helix of pitch `stepover` that keeps the
/// tip on the safe surface (`leave` above the part).
pub fn rotary_finish(
    parts: &[&Mesh],
    sections: &[(f64, f64)],
    f: &RotaryFrame,
    wcs_origin: Vec3,
    p: &RotaryParams,
) -> RotaryResult {
    let mut all = Mesh::default();
    for m in parts {
        all.append(m);
    }
    let bb = all.aabb();
    let (x_lo, x_hi, r_max) = cyl_extent(f, &bb);
    let nt = (360.0 / p.dtheta_deg.max(0.1)).round() as usize;
    let tr = p.cutter.radius + 1.0;
    let rmap = radius_map(parts, f, x_lo - tr, x_hi + tr, p.dx, nt, r_max + 1.0);
    let margin = 1.5 * p.dx.max(p.dtheta_deg.to_radians() * r_max);
    let limit = rotary_limit(&rmap, &p.cutter, sections, p.leave, margin);
    let r_safe = r_max + p.clearance + p.leave;
    let mut em = Emitter {
        f: *f,
        wcs: wcs_origin,
        cutter: p.cutter,
        stock: None,
        moves: Vec::new(),
        last: None,
        cut_len: 0.0,
    };
    let pitch = p.stepover.max(0.01);
    let r_ref = rmap
        .data
        .iter()
        .copied()
        .filter(|v| v.is_finite())
        .fold(1.0, f64::max);
    let dth = (p.step / r_ref).min(2f64.to_radians());
    let total = ((x_hi - x_lo) / pitch * TAU / dth).ceil() as usize;
    let mut down = false;
    for k in 0..=total {
        let th = k as f64 * dth;
        let x = x_lo + th / TAU * pitch;
        match limit.at_max(x, th).filter(|v| v.is_finite()) {
            Some(r) => {
                if !down {
                    em.go(x, th, r_safe, None);
                    em.go(x, th, r, Some(p.plunge_feed));
                    down = true;
                } else {
                    em.go(x, th, r, Some(p.feed));
                }
            }
            None => {
                if down {
                    em.retract(r_safe);
                    down = false;
                }
            }
        }
    }
    if down {
        em.retract(r_safe);
    }
    RotaryResult {
        cut_length_mm: em.cut_len,
        moves: em.moves,
        levels: vec![],
        removed_mm3: 0.0,
        remaining_mm3: 0.0,
        warnings: vec![],
    }
}

/// Axial extent of the parts in frame x.
fn part_x_extent(parts: &[&Mesh], f: &RotaryFrame) -> (f64, f64) {
    parts
        .iter()
        .flat_map(|m| m.vertices.iter())
        .map(|&v| f.cyl(v).0)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), x| {
            (a.min(x), b.max(x))
        })
}

/// x range and largest radius of a box around the axis.
fn cyl_extent(f: &RotaryFrame, b: &Aabb) -> (f64, f64, f64) {
    let (mut lo, mut hi, mut rm) = (f64::INFINITY, f64::NEG_INFINITY, 0.0f64);
    for i in 0..8 {
        let c = Vec3::new(
            if i & 1 == 0 { b.min.x } else { b.max.x },
            if i & 2 == 0 { b.min.y } else { b.max.y },
            if i & 4 == 0 { b.min.z } else { b.max.z },
        );
        let (x, _, r) = f.cyl(c);
        lo = lo.min(x);
        hi = hi.max(x);
        rm = rm.max(r);
    }
    (lo, hi, rm)
}

/// Wrap 2D paths onto a cylinder of radius `radius`: u is the axial
/// position, v the arc length around the cylinder (v / radius = θ). The tip
/// runs `depth` below the surface; paths are resampled every `step` mm so
/// the wrapped moves follow the curvature.
#[allow(clippy::too_many_arguments)]
pub fn wrap(
    paths: &[Vec<[f64; 2]>],
    f: &RotaryFrame,
    wcs_origin: Vec3,
    radius: f64,
    depth: f64,
    step: f64,
    feed: f64,
    clearance: f64,
) -> Vec<ClMove> {
    let r_cut = radius - depth;
    let r_safe = radius + clearance;
    let mut em = Emitter {
        f: *f,
        wcs: wcs_origin,
        cutter: CutterShape::flat(0.0, 0.0),
        stock: None,
        moves: Vec::new(),
        last: None,
        cut_len: 0.0,
    };
    for path in paths.iter().filter(|p| !p.is_empty()) {
        let th = |v: f64| v / radius;
        let [u0, v0] = path[0];
        em.go(u0, th(v0), r_safe, None);
        em.go(u0, th(v0), r_cut, Some(feed * 0.3));
        for w in path.windows(2) {
            let ([ua, va], [ub, vb]) = (w[0], w[1]);
            let n = (((ub - ua).hypot(vb - va)) / step.max(1e-3))
                .ceil()
                .max(1.0) as usize;
            for s in 1..=n {
                let t = s as f64 / n as f64;
                em.go(
                    ua + (ub - ua) * t,
                    th(va + (vb - va) * t),
                    r_cut,
                    Some(feed),
                );
            }
        }
        em.retract(r_safe);
    }
    em.moves
}

/// Flatten CL moves back to (u = axial, v = arc length at `radius`,
/// r = tip radius) for previews and checks. θ is unwound continuously.
pub fn unwrap(moves: &[ClMove], f: &RotaryFrame, wcs_origin: Vec3, radius: f64) -> Vec<[f64; 3]> {
    let mut out = Vec::with_capacity(moves.len());
    let mut prev_th: Option<f64> = None;
    for m in moves {
        let (x, th, r) = f.cyl(m.pose().tip + wcs_origin);
        let th = match prev_th {
            Some(p) => th + TAU * ((p - th) / TAU).round(),
            None => {
                if th > PI {
                    th - TAU
                } else {
                    th
                }
            }
        };
        prev_th = Some(th);
        out.push([x, th * radius, r]);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_roundtrip() {
        let f = RotaryFrame::new(Vec3::new(1.0, 2.0, 3.0), Vec3::new(-1.0, 0.0, 0.0), Vec3::Z);
        let p = f.point(5.0, 1.2, 7.0);
        let (x, th, r) = f.cyl(p);
        assert!((x - 5.0).abs() < 1e-12 && (th - 1.2).abs() < 1e-12 && (r - 7.0).abs() < 1e-12);
        assert!(f.radial(0.0).distance(Vec3::Z) < 1e-12);
    }

    #[test]
    fn cylinder_radius_and_limit() {
        let f = RotaryFrame::new(Vec3::ZERO, Vec3::X, Vec3::Z);
        let cyl = Mesh::cylinder(Vec3::new(-20.0, 0.0, 0.0), Vec3::X, 10.0, 40.0, 180);
        let m = radius_map(&[&cyl], &f, -10.0, 10.0, 0.5, 180, 30.0);
        for v in &m.data {
            assert!((*v - 10.0).abs() < 0.01, "{v}");
        }
        // A ball tool's tip sits on the cylinder; with 0.2 leave it is 0.2 up.
        let lim = rotary_limit(&m, &CutterShape::ball(6.0, 20.0), &[], 0.2, 0.0);
        let v = lim.get(10, 0);
        assert!((v - 10.2).abs() < 0.02, "{v}");
        // A flat face touches a convex cylinder only at its centre.
        let flat = rotary_limit(&m, &CutterShape::flat(6.0, 20.0), &[], 0.0, 0.0);
        assert!((flat.get(10, 0) - 10.0).abs() < 0.01, "{}", flat.get(10, 0));
    }
}
