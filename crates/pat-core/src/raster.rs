//! 2D rasters for 3-axis-style planning in any tool orientation.
//!
//! Everything here works in a *machining frame* F whose +Z is the tool axis
//! (identity for plain 3-axis, a tilted frame for 3+2):
//!
//! - [`height_field`]: highest point of a mesh above each grid cell.
//! - [`tool_limit`]: the lowest tip height at each cell that keeps a cutter
//!   (inflated by the stock to leave) clear of a height field. This is a
//!   raster drop-cutter: the height field dilated by the inverse tool shape.
//! - [`stock_top`]: top of remaining voxel stock above each cell.

use crate::geom::{Aabb, Bvh, Mesh};
use crate::math::{Iso, Vec3};
use crate::stock::{CutterShape, Voxels};

#[derive(Clone, Debug, PartialEq)]
pub struct Grid2<T> {
    /// Centre of cell (0, 0).
    pub x0: f64,
    pub y0: f64,
    pub res: f64,
    pub nx: usize,
    pub ny: usize,
    pub data: Vec<T>,
}

impl<T: Clone> Grid2<T> {
    /// Grid covering `[min, max]` (x/y only) with cell centres `res` apart.
    pub fn covering(min: Vec3, max: Vec3, res: f64, fill: T) -> Grid2<T> {
        let nx = (((max.x - min.x) / res).ceil() as usize).max(1) + 1;
        let ny = (((max.y - min.y) / res).ceil() as usize).max(1) + 1;
        Grid2 {
            x0: min.x,
            y0: min.y,
            res,
            nx,
            ny,
            data: vec![fill; nx * ny],
        }
    }

    pub fn like<U: Clone>(&self, fill: U) -> Grid2<U> {
        Grid2 {
            x0: self.x0,
            y0: self.y0,
            res: self.res,
            nx: self.nx,
            ny: self.ny,
            data: vec![fill; self.nx * self.ny],
        }
    }

    pub fn idx(&self, i: usize, j: usize) -> usize {
        j * self.nx + i
    }

    pub fn get(&self, i: usize, j: usize) -> &T {
        &self.data[j * self.nx + i]
    }

    pub fn set(&mut self, i: usize, j: usize, v: T) {
        let k = j * self.nx + i;
        self.data[k] = v;
    }

    pub fn xy(&self, i: usize, j: usize) -> (f64, f64) {
        (self.x0 + i as f64 * self.res, self.y0 + j as f64 * self.res)
    }

    /// Nearest cell to a point, if inside.
    pub fn cell(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let i = ((x - self.x0) / self.res).round();
        let j = ((y - self.y0) / self.res).round();
        (i >= 0.0 && j >= 0.0 && (i as usize) < self.nx && (j as usize) < self.ny)
            .then_some((i as usize, j as usize))
    }
}

/// Cell offsets within `radius` (by cell centre), with their distance.
pub fn disk_offsets(radius: f64, res: f64) -> Vec<(isize, isize, f64)> {
    let r = (radius / res).ceil() as isize;
    let mut v = Vec::new();
    for dj in -r..=r {
        for di in -r..=r {
            let d = ((di * di + dj * dj) as f64).sqrt() * res;
            if d <= radius + 1e-9 {
                v.push((di, dj, d));
            }
        }
    }
    v
}

/// Highest point of `mesh` (given in the world/table frame) above each cell
/// of a grid in frame `f` (F → world). Cells with no surface are -∞.
pub fn height_field(mesh: &Mesh, f: &Iso, min: Vec3, max: Vec3, res: f64) -> Grid2<f64> {
    let local = mesh.transformed(&f.inverse());
    let bvh = Bvh::build(&local);
    let bb = bvh.aabb();
    let mut g = Grid2::covering(min, max, res, f64::NEG_INFINITY);
    let jitter = (res * 1.7e-7, res * 3.1e-7);
    for j in 0..g.ny {
        for i in 0..g.nx {
            let (x, y) = g.xy(i, j);
            let (x, y) = (x + jitter.0, y + jitter.1);
            let column = Aabb {
                min: Vec3::new(x, y, bb.min.z - 1.0),
                max: Vec3::new(x, y, bb.max.z + 1.0),
            };
            let mut top = f64::NEG_INFINITY;
            bvh.any_in(&column, |t| {
                if let Some(z) = crate::stock::vertical_hit(&local.tri(t), x, y) {
                    top = top.max(z);
                }
                false
            });
            g.set(i, j, top);
        }
    }
    g
}

/// Lowest tip height per cell so the cutter, grown by `leave` all round,
/// stays above the surface `h` (raster drop-cutter).
///
/// `raster_margin` grows the footprint further (radially) to cover sampling
/// error: surface edges and tool positions both fall between cell centres,
/// so a safe value is ¾ of the cell size.
pub fn tool_limit(
    h: &Grid2<f64>,
    cutter: &CutterShape,
    leave: f64,
    raster_margin: f64,
) -> Grid2<f64> {
    let grown = CutterShape {
        radius: cutter.radius + leave + raster_margin,
        corner_radius: cutter.corner_radius + leave,
        flute_length: cutter.flute_length,
    };
    let offs: Vec<(isize, isize, f64)> = disk_offsets(grown.radius, h.res)
        .into_iter()
        .map(|(di, dj, d)| (di, dj, grown.profile_height(d).unwrap_or(0.0)))
        .collect();
    let mut out = h.like(f64::NEG_INFINITY);
    for j in 0..h.ny {
        for i in 0..h.nx {
            let mut z = f64::NEG_INFINITY;
            for &(di, dj, lift) in &offs {
                let (a, b) = (i as isize + di, j as isize + dj);
                if a < 0 || b < 0 || a as usize >= h.nx || b as usize >= h.ny {
                    continue;
                }
                let s = *h.get(a as usize, b as usize);
                if s > f64::NEG_INFINITY {
                    z = z.max(s - lift);
                }
            }
            out.set(i, j, if z > f64::NEG_INFINITY { z + leave } else { z });
        }
    }
    out
}

/// Maximum of `h` over a disk of `radius` around each cell (−∞ stays −∞
/// where the disk holds no values). Row-wise sliding maxima keep this fast
/// for large radii.
pub fn disk_max(h: &Grid2<f64>, radius: f64) -> Grid2<f64> {
    let r = (radius / h.res).floor() as isize;
    let (nx, ny) = (h.nx, h.ny);
    // Half-width per row offset.
    let widths: Vec<usize> = (0..=r)
        .map(|dj| (((r * r - dj * dj) as f64).max(0.0).sqrt()).floor() as usize)
        .collect();
    let mut uniq = widths.clone();
    uniq.sort_unstable();
    uniq.dedup();
    // rowmax[w][j][i] = max of row j over [i - w, i + w].
    let mut rowmax: std::collections::HashMap<usize, Vec<f64>> = std::collections::HashMap::new();
    for &w in &uniq {
        let mut out = vec![f64::NEG_INFINITY; nx * ny];
        for j in 0..ny {
            let row = &h.data[j * nx..(j + 1) * nx];
            // Monotonic deque sliding-window maximum.
            let mut dq: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
            let mut next = 0usize;
            for i in 0..nx {
                let hi = (i + w).min(nx - 1);
                while next <= hi {
                    while dq.back().is_some_and(|&b| row[b] <= row[next]) {
                        dq.pop_back();
                    }
                    dq.push_back(next);
                    next += 1;
                }
                let lo = i.saturating_sub(w);
                while dq.front().is_some_and(|&f| f < lo) {
                    dq.pop_front();
                }
                out[j * nx + i] = dq.front().map_or(f64::NEG_INFINITY, |&f| row[f]);
            }
        }
        rowmax.insert(w, out);
    }
    let mut out = h.like(f64::NEG_INFINITY);
    for j in 0..ny as isize {
        for dj in -r..=r {
            let jj = j + dj;
            if jj < 0 || jj >= ny as isize {
                continue;
            }
            let rm = &rowmax[&widths[dj.unsigned_abs()]];
            for i in 0..nx {
                let v = rm[jj as usize * nx + i];
                let k = j as usize * nx + i;
                if v > out.data[k] {
                    out.data[k] = v;
                }
            }
        }
    }
    out
}

/// Lowest tip height per cell for a whole tool assembly: the shaped cutter
/// (grown by `leave`) plus non-cutting sections given as (height of their
/// lower end above the tip, radius), treated as cylinders extending upward.
pub fn assembly_limit(
    h: &Grid2<f64>,
    cutter: &CutterShape,
    sections: &[(f64, f64)],
    leave: f64,
    raster_margin: f64,
) -> Grid2<f64> {
    let mut lim = tool_limit(h, cutter, leave, raster_margin);
    for &(z0, r) in sections {
        let m = disk_max(h, r + leave + raster_margin);
        for (a, b) in lim.data.iter_mut().zip(&m.data) {
            if b.is_finite() {
                *a = a.max(b + leave - z0);
            }
        }
    }
    lim
}

/// Like [`stock_top`], but after cutting: material only disappears, so each
/// column starts at its previous top (`prev`) and empty columns stay empty.
pub fn stock_top_after(stock: &Voxels, f: &Iso, prev: &Grid2<f64>) -> Grid2<f64> {
    let dir = f.apply_vec(Vec3::Z);
    let mut out = prev.like(f64::NEG_INFINITY);
    for j in 0..prev.ny {
        for i in 0..prev.nx {
            let z0 = *prev.get(i, j);
            if !z0.is_finite() {
                continue;
            }
            let (x, y) = prev.xy(i, j);
            let from = z0 + stock.res;
            let p = f.apply_point(Vec3::new(x, y, from));
            // Columns emptied by cutting are searched once, then skipped.
            let bb = stock.bounds();
            let depth = (bb.max - bb.min).norm() + stock.res;
            if let Some(d) = stock.first_material_along(p, dir, depth) {
                out.set(i, j, from + d);
            }
        }
    }
    out
}

/// Top of the voxel stock above each cell, looking down frame `f`'s -Z from
/// height `z_from` (F coordinates). -∞ where there is no material.
pub fn stock_top(
    stock: &Voxels,
    f: &Iso,
    grid: &Grid2<f64>,
    z_from: f64,
    depth: f64,
) -> Grid2<f64> {
    let dir = f.apply_vec(Vec3::Z);
    let mut out = grid.like(f64::NEG_INFINITY);
    for j in 0..grid.ny {
        for i in 0..grid.nx {
            let (x, y) = grid.xy(i, j);
            let p = f.apply_point(Vec3::new(x, y, z_from));
            if let Some(d) = stock.first_material_along(p, dir, depth) {
                out.set(i, j, z_from + d);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disk_max_matches_brute_force() {
        let mut g = Grid2::covering(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 8.0, 0.0),
            0.5,
            f64::NEG_INFINITY,
        );
        let mut s = 12345u64;
        for v in g.data.iter_mut() {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
            if s >> 62 != 0 {
                *v = (s >> 40) as f64 / 1e6;
            }
        }
        for radius in [0.4, 1.0, 2.3, 3.0] {
            let fast = disk_max(&g, radius);
            let offs = disk_offsets((radius / g.res).floor() * g.res, g.res);
            for j in 0..g.ny {
                for i in 0..g.nx {
                    let mut m = f64::NEG_INFINITY;
                    for &(di, dj, _) in &offs {
                        let (a, b) = (i as isize + di, j as isize + dj);
                        if a >= 0 && b >= 0 && (a as usize) < g.nx && (b as usize) < g.ny {
                            m = m.max(*g.get(a as usize, b as usize));
                        }
                    }
                    assert_eq!(*fast.get(i, j), m, "r {radius} at {i},{j}");
                }
            }
        }
    }

    #[test]
    fn holder_keeps_the_tool_out_of_deep_narrow_spots() {
        // A 20 mm tall block: a Ø6 cutter with a 10 mm flute in a Ø20 holder
        // cannot reach the floor right next to it.
        let m = Mesh::cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 20.0));
        let g = height_field(
            &m,
            &Iso::IDENTITY,
            Vec3::new(-20.0, -20.0, 0.0),
            Vec3::new(30.0, 30.0, 0.0),
            0.25,
        );
        let cutter = CutterShape::flat(6.0, 10.0);
        let lim = assembly_limit(&g, &cutter, &[(10.0, 10.0)], 0.0, 0.0);
        let at = |x: f64| *lim.get(lim.cell(x, 5.0).unwrap().0, lim.cell(x, 5.0).unwrap().1);
        // 4 mm off the wall: the holder (r 10, 10 mm up) sits on the block top.
        assert!((at(-4.0) - 10.0).abs() < 1e-9, "{}", at(-4.0));
        // 11 mm off: clear of everything.
        assert_eq!(at(-11.0), f64::NEG_INFINITY);
    }

    #[test]
    fn height_field_of_a_block() {
        let m = Mesh::cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 5.0));
        let g = height_field(
            &m,
            &Iso::IDENTITY,
            Vec3::new(-5.0, -5.0, 0.0),
            Vec3::new(15.0, 15.0, 0.0),
            0.5,
        );
        let (i, j) = g.cell(5.0, 5.0).unwrap();
        assert!((g.get(i, j) - 5.0).abs() < 1e-9);
        let (i, j) = g.cell(-3.0, 5.0).unwrap();
        assert_eq!(*g.get(i, j), f64::NEG_INFINITY);
    }

    #[test]
    fn drop_cutter_around_a_block() {
        let m = Mesh::cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 5.0));
        let g = height_field(
            &m,
            &Iso::IDENTITY,
            Vec3::new(-10.0, -10.0, 0.0),
            Vec3::new(20.0, 20.0, 0.0),
            0.25,
        );
        let flat = tool_limit(&g, &CutterShape::flat(6.0, 20.0), 0.0, 0.0);
        // Flat Ø6: tool centre 2.5 mm off the block edge still lands on top.
        let at =
            |g: &Grid2<f64>, x: f64| *g.get(g.cell(x, 5.0).unwrap().0, g.cell(x, 5.0).unwrap().1);
        assert!((at(&flat, -2.5) - 5.0).abs() < 1e-9);
        assert_eq!(at(&flat, -3.5), f64::NEG_INFINITY);
        // Ball Ø6 at 2 mm off the edge sits lower: 5 - (3 - sqrt(9 - 4)).
        let ball = tool_limit(&g, &CutterShape::ball(6.0, 20.0), 0.0, 0.0);
        let expect = 5.0 - (3.0 - 5f64.sqrt());
        assert!(
            (at(&ball, -2.0) - expect).abs() < 1e-9,
            "{}",
            at(&ball, -2.0)
        );
        // Leaving 0.5 mm grows the footprint (with a 0.5 mm rounded corner,
        // the true offset of a sharp corner) and lifts the tip.
        let left = tool_limit(&g, &CutterShape::flat(6.0, 20.0), 0.5, 0.0);
        assert!((at(&left, -3.0) - 5.5).abs() < 1e-9);
        let corner = 5.5 - (0.5 - (0.25f64 - 0.0625).sqrt());
        assert!((at(&left, -3.25) - corner).abs() < 1e-9);
        assert!((at(&left, 5.0) - 5.5).abs() < 1e-9);
        // A raster margin pushes the footprint out without lifting the floor.
        let safe = tool_limit(&g, &CutterShape::flat(6.0, 20.0), 0.0, 0.5);
        assert!((at(&safe, -3.25) - 5.0).abs() < 1e-9);
        assert!((at(&safe, 5.0) - 5.0).abs() < 1e-9);
    }
}
