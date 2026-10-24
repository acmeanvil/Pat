//! Adaptive (constant-engagement) clearing on a 2D raster.
//!
//! Input: a grid marking where the tool *centre* may go (`allowed`) and
//! where material remains at this level (`material`). The tool walks in short
//! steps. At each step it tries a fan of headings and takes the one whose
//! newly cut area is closest to the target, `stepover × step`. That keeps the
//! radial load roughly constant, unlike offset or zig-zag paths, which slot
//! into corners. Material is kept on the right of the direction of travel,
//! which is climb milling with a clockwise (M3) spindle.
//!
//! When a pass runs out of material, the tool travels to the next uncut
//! region through already-cleared space (a keep-down link). It retracts and
//! re-enters only when no such path exists: through open air if possible,
//! otherwise with a helix or, in very narrow spots, a plunge. A final low-load
//! phase cleans up slivers left along walls.
//!
//! This is an original implementation of the general constant-engagement
//! idea; it is not derived from any GPL code.

use crate::raster::{disk_offsets, Grid2};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct AdaptiveParams {
    pub tool_radius: f64,
    /// Target radial width of cut (mm).
    pub stepover: f64,
    /// Path step length (mm). 0 = automatic.
    pub step: f64,
    /// Helix entry radius as a fraction of the tool radius.
    pub helix_ratio: f64,
    /// Highest acceptable load as a multiple of the target (except where
    /// unavoidable, e.g. right after entry).
    pub max_load: f64,
    /// Safety cap on path steps.
    pub max_steps: usize,
    /// Round off direction changes: each point may move up to this
    /// fraction of the stepover toward its neighbours' average (0 = off).
    pub smoothing: f64,
}

impl Default for AdaptiveParams {
    fn default() -> Self {
        AdaptiveParams {
            tool_radius: 5.0,
            stepover: 1.0,
            step: 0.0,
            helix_ratio: 0.5,
            max_load: 1.5,
            max_steps: 2_000_000,
            smoothing: 0.3,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum PassKind {
    Cut,
    /// Travel through cleared space at the level (tool stays down).
    Link,
    /// Enter from above at a point with no material at this level.
    AirEntry,
    /// Helix down around the first point (radius 0 = plunge).
    HelixEntry {
        radius: f64,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pass2 {
    pub kind: PassKind,
    pub pts: Vec<[f64; 2]>,
    /// Cut passes: load at each point relative to the target (1.0 = on target).
    #[serde(skip_serializing_if = "Vec::is_empty", default)]
    pub load: Vec<f64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AdaptiveResult {
    pub passes: Vec<Pass2>,
    /// Material the tool could have reached but did not (mm²).
    pub uncut_reachable_area: f64,
}

struct State<'a> {
    allowed: &'a Grid2<bool>,
    uncut: Grid2<bool>,
    reach: Vec<bool>,
    /// Start cells that led nowhere; never retried.
    blocked: Vec<bool>,
    r: f64,
    res: f64,
}

impl State<'_> {
    fn cell_area(&self) -> f64 {
        self.res * self.res
    }

    fn allowed_at(&self, x: f64, y: f64) -> bool {
        self.allowed
            .cell(x, y)
            .is_some_and(|(i, j)| *self.allowed.get(i, j))
    }

    fn segment_allowed(&self, a: [f64; 2], b: [f64; 2]) -> bool {
        let d = ((b[0] - a[0]).hypot(b[1] - a[1]) / (self.res * 0.5))
            .ceil()
            .max(1.0) as usize;
        (0..=d).all(|k| {
            let t = k as f64 / d as f64;
            self.allowed_at(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
        })
    }

    /// Cells (i, j) whose centres lie within `rad` of segment a–b.
    fn cells_near(&self, a: [f64; 2], b: [f64; 2], rad: f64, mut f: impl FnMut(usize, usize)) {
        let g = &self.uncut;
        let lo = |v: f64, o: f64| (((v - rad - o) / g.res).floor().max(0.0)) as usize;
        let hi = |v: f64, o: f64, n: usize| {
            ((((v + rad - o) / g.res).ceil()) as isize).clamp(0, n as isize - 1) as usize
        };
        let (i0, i1) = (lo(a[0].min(b[0]), g.x0), hi(a[0].max(b[0]), g.x0, g.nx));
        let (j0, j1) = (lo(a[1].min(b[1]), g.y0), hi(a[1].max(b[1]), g.y0, g.ny));
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let l2 = dx * dx + dy * dy;
        for j in j0..=j1 {
            for i in i0..=i1 {
                let (x, y) = g.xy(i, j);
                let t = if l2 < 1e-18 {
                    0.0
                } else {
                    (((x - a[0]) * dx + (y - a[1]) * dy) / l2).clamp(0.0, 1.0)
                };
                let (px, py) = (a[0] + dx * t - x, a[1] + dy * t - y);
                if px * px + py * py <= rad * rad {
                    f(i, j);
                }
            }
        }
    }

    /// Uncut area (mm²) under the tool at p.
    fn area(&self, p: [f64; 2]) -> f64 {
        let mut n = 0usize;
        self.cells_near(p, p, self.r, |i, j| {
            if *self.uncut.get(i, j) {
                n += 1;
            }
        });
        n as f64 * self.cell_area()
    }

    fn clear(&mut self, a: [f64; 2], b: [f64; 2], rad: f64) {
        let mut cells = Vec::new();
        self.cells_near(a, b, rad, |i, j| cells.push((i, j)));
        for (i, j) in cells {
            self.uncut.set(i, j, false);
        }
    }

    fn remaining(&self) -> usize {
        self.uncut
            .data
            .iter()
            .zip(&self.reach)
            .filter(|(u, r)| **u && **r)
            .count()
    }

    /// Unit vector from p toward the uncut material under the tool there.
    fn toward_material(&self, p: [f64; 2], rad: f64) -> Option<[f64; 2]> {
        let (mut sx, mut sy, mut n) = (0.0, 0.0, 0usize);
        self.cells_near(p, p, rad, |i, j| {
            if *self.uncut.get(i, j) {
                let (x, y) = self.uncut.xy(i, j);
                sx += x - p[0];
                sy += y - p[1];
                n += 1;
            }
        });
        let l = sx.hypot(sy);
        (n > 0 && l > 1e-12).then(|| [sx / l, sy / l])
    }
}

/// Heading that keeps material on the right (climb, CW spindle).
fn heading_with_material_right(to_material: [f64; 2]) -> f64 {
    // Right of heading h is h - 90°, so h = direction to material + 90°.
    to_material[1].atan2(to_material[0]) + std::f64::consts::FRAC_PI_2
}

pub fn adaptive_clear(
    allowed: &Grid2<bool>,
    material: &Grid2<bool>,
    p: &AdaptiveParams,
) -> AdaptiveResult {
    let r = p.tool_radius;
    let res = allowed.res;
    // Material reachable by some allowed tool position.
    let mut reach = vec![false; allowed.data.len()];
    let disk = disk_offsets(r, res);
    for j in 0..allowed.ny {
        for i in 0..allowed.nx {
            if !*allowed.get(i, j) {
                continue;
            }
            for &(di, dj, _) in &disk {
                let (a, b) = (i as isize + di, j as isize + dj);
                if a >= 0 && b >= 0 && (a as usize) < allowed.nx && (b as usize) < allowed.ny {
                    reach[b as usize * allowed.nx + a as usize] = true;
                }
            }
        }
    }
    let blocked = vec![false; allowed.data.len()];
    let mut st = State {
        allowed,
        uncut: material.clone(),
        reach,
        blocked,
        r,
        res,
    };
    let step = if p.step > 0.0 {
        p.step
    } else {
        (r * 0.25).min(p.stepover.max(res)).max(res)
    };
    let target = p.stepover * step;
    let mut passes: Vec<Pass2> = Vec::new();
    let mut pos: Option<[f64; 2]> = None;
    let mut steps = 0usize;

    // Phase 1 at a useful load; phase 2 picks up slivers.
    for min_area in [0.15 * target, st.cell_area() * 0.99] {
        loop {
            if st.remaining() == 0 || steps > p.max_steps {
                break;
            }
            let start = match pos.and_then(|q| find_link(&st, q, step, min_area)) {
                Some((path, at, h)) => {
                    if path.len() > 1 {
                        passes.push(Pass2 {
                            kind: PassKind::Link,
                            pts: path,
                            load: vec![],
                        });
                    }
                    Some((at, h))
                }
                None => find_entry(&mut st, step, min_area, p.helix_ratio, &mut passes),
            };
            let Some((at, h)) = start else { break };
            let (pass, end) = run_pass(
                &mut st,
                at,
                h,
                step,
                target,
                min_area,
                p.max_load,
                &mut steps,
                p.max_steps,
            );
            pos = Some(end);
            if pass.pts.len() > 1 {
                let mut pass = pass;
                smooth_pass(&st, &mut pass.pts, p.smoothing * p.stepover);
                passes.push(pass);
            } else if let Some((i, j)) = allowed.cell(at[0], at[1]) {
                // Could not move from here: never start here again.
                let k = allowed.idx(i, j);
                st.blocked[k] = true;
            }
        }
    }
    let uncut_reachable_area = st.remaining() as f64 * st.cell_area();
    AdaptiveResult {
        passes,
        uncut_reachable_area,
    }
}

#[allow(clippy::too_many_arguments)]
fn run_pass(
    st: &mut State,
    start: [f64; 2],
    mut h: f64,
    step: f64,
    target: f64,
    min_area: f64,
    max_load: f64,
    steps: &mut usize,
    max_steps: usize,
) -> (Pass2, [f64; 2]) {
    let mut p = start;
    let mut pts = vec![p];
    let mut load = vec![0.0];
    let fan: Vec<f64> = (-16..=16).map(|k| (k as f64 * 7.5).to_radians()).collect();
    let full: Vec<f64> = (-23..=24).map(|k| (k as f64 * 7.5).to_radians()).collect();
    let mut first = true;
    while *steps < max_steps {
        *steps += 1;
        let fan = if std::mem::take(&mut first) {
            &full
        } else {
            &fan
        };
        let mut best: Option<(f64, [f64; 2], f64, f64)> = None; // (score, q, heading, area)
        let mut least: Option<([f64; 2], f64, f64)> = None; // fallback: least overload
        for &d in fan {
            let hh = h + d;
            let q = [p[0] + step * hh.cos(), p[1] + step * hh.sin()];
            if !st.segment_allowed(p, q) {
                continue;
            }
            let a = st.area(q);
            if a < min_area {
                continue;
            }
            if a <= max_load * target {
                // Closest to target, gently preferring to keep going straight.
                let score = (a - target).abs() / target + 0.05 * d.abs();
                if best.as_ref().is_none_or(|b| score < b.0) {
                    best = Some((score, q, hh, a));
                }
            } else if least.as_ref().is_none_or(|l| a < l.2) {
                least = Some((q, hh, a));
            }
        }
        let (q, hh, a) = match (best, least) {
            (Some((_, q, hh, a)), _) => (q, hh, a),
            (None, Some(l)) => l,
            (None, None) => break,
        };
        st.clear(p, q, st.r);
        pts.push(q);
        load.push(a / target);
        p = q;
        h = hh;
    }
    (
        Pass2 {
            kind: PassKind::Cut,
            pts,
            load,
        },
        p,
    )
}

/// Laplacian smoothing of a cut pass. Interior points move toward the
/// average of their neighbours by at most `max_shift` (mm) from where they
/// were cut, and only where the tool may go. End points stay put, so links
/// and entries still join up.
fn smooth_pass(st: &State, pts: &mut [[f64; 2]], max_shift: f64) {
    if max_shift <= 0.0 || pts.len() < 3 {
        return;
    }
    let orig: Vec<[f64; 2]> = pts.to_vec();
    for _ in 0..4 {
        for i in 1..pts.len() - 1 {
            let avg = [
                (pts[i - 1][0] + pts[i + 1][0]) * 0.5,
                (pts[i - 1][1] + pts[i + 1][1]) * 0.5,
            ];
            let mut q = [
                pts[i][0] + 0.5 * (avg[0] - pts[i][0]),
                pts[i][1] + 0.5 * (avg[1] - pts[i][1]),
            ];
            // Clamp to the allowed shift from the original point.
            let (dx, dy) = (q[0] - orig[i][0], q[1] - orig[i][1]);
            let d = dx.hypot(dy);
            if d > max_shift {
                q = [
                    orig[i][0] + dx / d * max_shift,
                    orig[i][1] + dy / d * max_shift,
                ];
            }
            if st.segment_allowed(pts[i - 1], q) && st.segment_allowed(q, pts[i + 1]) {
                pts[i] = q;
            }
        }
    }
}

/// Largest change of direction between consecutive segments (degrees).
pub fn max_turn_deg(pts: &[[f64; 2]]) -> f64 {
    pts.windows(3)
        .map(|w| {
            let a = (w[1][1] - w[0][1]).atan2(w[1][0] - w[0][0]);
            let b = (w[2][1] - w[1][1]).atan2(w[2][0] - w[1][0]);
            let d = (b - a).rem_euclid(std::f64::consts::TAU);
            d.min(std::f64::consts::TAU - d).to_degrees()
        })
        .fold(0.0, f64::max)
}

/// Breadth-first search through cleared, allowed cells from `from` to a
/// cell next to material. Returns the (simplified) path, start point and
/// heading. Uses one distance-to-material map, so it is linear in grid size.
fn find_link(
    st: &State,
    from: [f64; 2],
    step: f64,
    min_area: f64,
) -> Option<(Vec<[f64; 2]>, [f64; 2], f64)> {
    let g = st.allowed;
    let (si, sj) = g.cell(from[0], from[1])?;
    let du = chamfer(g.nx, g.ny, g.res, |k| st.uncut.data[k], false);
    let safe = |k: usize| g.data[k] && du[k] > st.r + 0.5 * st.res;
    let near = |k: usize| du[k] <= st.r + step + st.res;
    let mut prev = vec![usize::MAX; g.data.len()];
    let start = g.idx(si, sj);
    prev[start] = start;
    let mut queue = VecDeque::from([start]);
    while let Some(k) = queue.pop_front() {
        let (i, j) = (k % g.nx, k / g.nx);
        if near(k) && !st.blocked[k] {
            let (x, y) = g.xy(i, j);
            if let Some((q, d)) = engaging_neighbour(st, [x, y], step, min_area) {
                let mut path = Vec::new();
                let mut c = k;
                loop {
                    let (px, py) = g.xy(c % g.nx, c / g.nx);
                    path.push([px, py]);
                    if prev[c] == c {
                        break;
                    }
                    c = prev[c];
                }
                path.push(from);
                path.reverse();
                let path = simplify(st, path);
                let to_mat = st.toward_material(q, st.r).unwrap_or(d);
                return Some((path, [x, y], heading_with_material_right(to_mat)));
            }
        }
        for (di, dj) in [
            (1isize, 0isize),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ] {
            let (a, b) = (i as isize + di, j as isize + dj);
            if a < 0 || b < 0 || a as usize >= g.nx || b as usize >= g.ny {
                continue;
            }
            let n = g.idx(a as usize, b as usize);
            if prev[n] == usize::MAX && safe(n) {
                prev[n] = k;
                queue.push_back(n);
            }
        }
    }
    None
}

/// A point one step from `c` where cutting would start with enough load.
fn engaging_neighbour(
    st: &State,
    c: [f64; 2],
    step: f64,
    min_area: f64,
) -> Option<([f64; 2], [f64; 2])> {
    (0..16).find_map(|k| {
        let a = k as f64 * std::f64::consts::TAU / 16.0;
        let d = [a.cos(), a.sin()];
        let q = [c[0] + d[0] * step, c[1] + d[1] * step];
        (st.segment_allowed(c, q) && st.area(q) >= min_area).then_some((q, d))
    })
}

/// Shortcut a cell path wherever a straight segment stays in cleared,
/// allowed space.
fn simplify(st: &State, path: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    if path.len() <= 2 {
        return path;
    }
    let clear_line = |a: [f64; 2], b: [f64; 2]| {
        let n = ((b[0] - a[0]).hypot(b[1] - a[1]) / (st.res * 0.5))
            .ceil()
            .max(1.0) as usize;
        (0..=n).all(|k| {
            let t = k as f64 / n as f64;
            let p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
            st.allowed_at(p[0], p[1]) && st.area(p) == 0.0
        })
    };
    let mut out = vec![path[0]];
    let mut i = 0;
    while i < path.len() - 1 {
        let mut j = path.len() - 1;
        while j > i + 1 && !clear_line(path[i], path[j]) {
            j -= 1;
        }
        out.push(path[j]);
        i = j;
    }
    out
}

/// Choose where to enter from above. Prefers open air next to material,
/// then a helix where one fits, then a plunge.
#[allow(clippy::needless_range_loop)] // parallel arrays indexed by cell
fn find_entry(
    st: &mut State,
    step: f64,
    min_area: f64,
    helix_ratio: f64,
    passes: &mut Vec<Pass2>,
) -> Option<([f64; 2], f64)> {
    let g = st.allowed;
    let du = chamfer(g.nx, g.ny, g.res, |k| st.uncut.data[k], false);
    // Air: tool disk empty, material within a step.
    for k in 0..g.data.len() {
        if !g.data[k]
            || st.blocked[k]
            || du[k] <= st.r + 0.5 * st.res
            || du[k] > st.r + step + st.res
        {
            continue;
        }
        let (x, y) = g.xy(k % g.nx, k / g.nx);
        if let Some((q, d)) = engaging_neighbour(st, [x, y], step, min_area) {
            let to_mat = st.toward_material(q, st.r).unwrap_or(d);
            passes.push(Pass2 {
                kind: PassKind::AirEntry,
                pts: vec![[x, y]],
                load: vec![],
            });
            return Some(([x, y], heading_with_material_right(to_mat)));
        }
    }
    // Helix (or plunge) at the allowed cell deepest inside the region that
    // has material under the tool.
    let inside = chamfer(g.nx, g.ny, g.res, |k| !g.data[k], true);
    let mut best: Option<(f64, usize)> = None;
    for k in 0..g.data.len() {
        if !g.data[k] || st.blocked[k] || du[k] > st.r {
            continue;
        }
        let score = inside[k] - du[k];
        if best.is_none_or(|b| score > b.0) {
            let (x, y) = g.xy(k % g.nx, k / g.nx);
            if st.area([x, y]) >= min_area {
                best = Some((score, k));
            }
        }
    }
    let (_, k) = best?;
    let (x, y) = g.xy(k % g.nx, k / g.nx);
    let c = [x, y];
    let radius = (st.r * helix_ratio).min((inside[k] - st.res).max(0.0));
    passes.push(Pass2 {
        kind: PassKind::HelixEntry { radius },
        pts: vec![c],
        load: vec![],
    });
    st.clear(c, c, st.r + radius);
    let to_mat = st
        .toward_material(c, st.r + radius + step)
        .unwrap_or([1.0, 0.0]);
    Some((c, heading_with_material_right(to_mat)))
}

/// Chamfer distance transform (mm) to the nearest source cell; with
/// `border`, cells outside the grid count as sources.
fn chamfer(nx: usize, ny: usize, r: f64, source: impl Fn(usize) -> bool, border: bool) -> Vec<f64> {
    let mut d: Vec<f64> = (0..nx * ny)
        .map(|k| if source(k) { 0.0 } else { f64::INFINITY })
        .collect();
    let diag = r * std::f64::consts::SQRT_2;
    for j in 0..ny {
        for i in 0..nx {
            let k = j * nx + i;
            if border && (i == 0 || j == 0 || i == nx - 1 || j == ny - 1) {
                d[k] = d[k].min(r);
            }
            if i > 0 {
                d[k] = d[k].min(d[k - 1] + r);
            }
            if j > 0 {
                d[k] = d[k].min(d[k - nx] + r);
                if i > 0 {
                    d[k] = d[k].min(d[k - nx - 1] + diag);
                }
                if i + 1 < nx {
                    d[k] = d[k].min(d[k - nx + 1] + diag);
                }
            }
        }
    }
    for j in (0..ny).rev() {
        for i in (0..nx).rev() {
            let k = j * nx + i;
            if i + 1 < nx {
                d[k] = d[k].min(d[k + 1] + r);
            }
            if j + 1 < ny {
                d[k] = d[k].min(d[k + nx] + r);
                if i + 1 < nx {
                    d[k] = d[k].min(d[k + nx + 1] + diag);
                }
                if i > 0 {
                    d[k] = d[k].min(d[k + nx - 1] + diag);
                }
            }
        }
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;

    /// Square pocket 40 x 30, tool Ø10 (centre allowed 5 mm in from walls).
    fn pocket(res: f64) -> (Grid2<bool>, Grid2<bool>) {
        let mut allowed = Grid2::covering(
            Vec3::new(-5.0, -5.0, 0.0),
            Vec3::new(45.0, 35.0, 0.0),
            res,
            false,
        );
        let mut material = allowed.like(false);
        for j in 0..allowed.ny {
            for i in 0..allowed.nx {
                let (x, y) = allowed.xy(i, j);
                let inside = (0.0..=40.0).contains(&x) && (0.0..=30.0).contains(&y);
                material.set(i, j, inside);
                allowed.set(i, j, (5.0..=35.0).contains(&x) && (5.0..=25.0).contains(&y));
            }
        }
        (allowed, material)
    }

    fn total_cut_len(r: &AdaptiveResult) -> f64 {
        r.passes
            .iter()
            .filter(|p| p.kind == PassKind::Cut)
            .flat_map(|p| {
                p.pts
                    .windows(2)
                    .map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]))
            })
            .sum()
    }

    #[test]
    fn clears_a_closed_pocket_with_bounded_load() {
        let (allowed, material) = pocket(0.25);
        let p = AdaptiveParams {
            tool_radius: 5.0,
            stepover: 1.5,
            ..Default::default()
        };
        let r = adaptive_clear(&allowed, &material, &p);
        assert!(
            r.uncut_reachable_area < 0.5,
            "left {} mm²",
            r.uncut_reachable_area
        );
        // Closed pocket: must start with a helix.
        let first = r.passes.iter().find(|p| p.kind != PassKind::Cut).unwrap();
        assert!(
            matches!(first.kind, PassKind::HelixEntry { radius } if radius > 1.0),
            "{:?}",
            first.kind
        );
        // Load: the large majority of the cutting stays near the target.
        let loads: Vec<f64> = r
            .passes
            .iter()
            .flat_map(|p| p.load.iter().skip(1).copied())
            .collect();
        let over = loads.iter().filter(|&&l| l > 1.5 + 1e-9).count();
        assert!(
            (over as f64) < 0.02 * loads.len() as f64,
            "{over} of {} steps overloaded",
            loads.len()
        );
        // Cut length is in the right ballpark: area / stepover.
        let len = total_cut_len(&r);
        assert!(
            len > 1200.0 * 0.7 / 1.5 && len < 1200.0 * 3.0 / 1.5,
            "cut length {len}"
        );
        // Tool centre never leaves the allowed region.
        for pass in &r.passes {
            for q in &pass.pts {
                assert!(
                    (5.0 - 0.3..=35.0 + 0.3).contains(&q[0])
                        && (5.0 - 0.3..=25.0 + 0.3).contains(&q[1]),
                    "{q:?}"
                );
            }
        }
    }

    #[test]
    fn smoothing_rounds_corners() {
        let (allowed, material) = pocket(0.25);
        let raw = AdaptiveParams {
            tool_radius: 5.0,
            stepover: 1.5,
            smoothing: 0.0,
            ..Default::default()
        };
        let smooth = AdaptiveParams {
            smoothing: 0.3,
            ..raw.clone()
        };
        let turn = |r: &AdaptiveResult| {
            let v: Vec<f64> = r
                .passes
                .iter()
                .filter(|p| p.kind == PassKind::Cut)
                .map(|p| max_turn_deg(&p.pts))
                .collect();
            v.iter().sum::<f64>() / v.len() as f64
        };
        let a = adaptive_clear(&allowed, &material, &raw);
        let b = adaptive_clear(&allowed, &material, &smooth);
        assert!(
            turn(&b) < 0.8 * turn(&a),
            "mean worst turn {} vs {}",
            turn(&b),
            turn(&a)
        );
        assert!(b.uncut_reachable_area < 0.5);
    }

    #[test]
    fn open_side_enters_from_air() {
        // Material only for x < 30, but the tool may go to x = 45: enter from air.
        let (mut allowed, mut material) = pocket(0.25);
        for j in 0..allowed.ny {
            for i in 0..allowed.nx {
                let (x, y) = allowed.xy(i, j);
                allowed.set(i, j, (5.0..=45.0).contains(&x) && (5.0..=25.0).contains(&y));
                material.set(i, j, *material.get(i, j) && x < 30.0);
            }
        }
        let r = adaptive_clear(
            &allowed,
            &material,
            &AdaptiveParams {
                tool_radius: 5.0,
                stepover: 1.5,
                ..Default::default()
            },
        );
        assert!(
            r.passes
                .iter()
                .all(|p| !matches!(p.kind, PassKind::HelixEntry { .. })),
            "no helix needed"
        );
        assert_eq!(
            r.passes
                .iter()
                .filter(|p| p.kind == PassKind::AirEntry)
                .count(),
            1
        );
        assert!(r.uncut_reachable_area < 0.5);
    }
}
