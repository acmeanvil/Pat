//! Voxel material model: in-process stock, cutter sweeps, gouge checks.
//!
//! The stock is a regular grid of occupied/empty voxels (one bit each).
//! Cutting clears every voxel whose centre lies inside the cutter at a
//! sampled pose; sweeping samples poses at most half a voxel apart. The same
//! grid answers "is there material here?", "what is the top of the material
//! along this direction?" (height fields for roughing in any orientation) and
//! "did the cutter enter the finished part?" (gouges).
//!
//! Voxels are coarse but simple and fully general for 3-, 4- and 5-axis
//! motion. Accuracy is ± one voxel; pick the resolution accordingly (0.25 to
//! 1 mm is typical for roughing checks).

use crate::geom::{Aabb, Bvh, Mesh};
use crate::kinematics::{frame_from_z, Pose};
use crate::math::{slerp, Iso, Vec3};
use serde::{Deserialize, Serialize};

/// Cutting part of a tool: radius, corner radius (0 = flat end mill,
/// = radius for a ball end mill) and flute length, in the tool frame (tip at
/// the origin, +Z toward the spindle).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CutterShape {
    pub radius: f64,
    pub corner_radius: f64,
    pub flute_length: f64,
}

impl CutterShape {
    pub fn flat(diameter: f64, flute_length: f64) -> Self {
        CutterShape {
            radius: diameter / 2.0,
            corner_radius: 0.0,
            flute_length,
        }
    }

    pub fn ball(diameter: f64, flute_length: f64) -> Self {
        CutterShape {
            radius: diameter / 2.0,
            corner_radius: diameter / 2.0,
            flute_length,
        }
    }

    /// Is a point (tool frame) inside the cutter?
    pub fn contains(&self, p: Vec3) -> bool {
        if p.z < 0.0 || p.z > self.flute_length {
            return false;
        }
        let rho2 = p.x * p.x + p.y * p.y;
        let rc = self.corner_radius.min(self.radius);
        if p.z >= rc {
            return rho2 <= self.radius * self.radius;
        }
        // Torus / sphere region below the corner centre line.
        let dz = rc - p.z;
        let reach = self.radius - rc + (rc * rc - dz * dz).max(0.0).sqrt();
        rho2 <= reach * reach
    }

    /// Height of the cutter surface above the tip at radial distance `rho`
    /// (`None` beyond the radius). Used for raster drop-cutter.
    pub fn profile_height(&self, rho: f64) -> Option<f64> {
        if rho > self.radius + 1e-12 {
            return None;
        }
        let rc = self.corner_radius.min(self.radius);
        let flat = self.radius - rc;
        if rho <= flat {
            return Some(0.0);
        }
        let d = rho - flat;
        Some(rc - (rc * rc - d * d).max(0.0).sqrt())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Voxels {
    /// Minimum corner of the grid.
    pub origin: Vec3,
    pub res: f64,
    pub n: [usize; 3],
    bits: Vec<u64>,
    /// Occupied-voxel count per 8x8x8 block, to skip empty space quickly.
    occ: Vec<u32>,
    nb: [usize; 3],
}

const B: usize = 8;

impl Voxels {
    pub fn empty(min: Vec3, max: Vec3, res: f64) -> Voxels {
        let n = [
            (((max.x - min.x) / res).ceil() as usize).max(1),
            (((max.y - min.y) / res).ceil() as usize).max(1),
            (((max.z - min.z) / res).ceil() as usize).max(1),
        ];
        let total = n[0] * n[1] * n[2];
        let nb = [n[0].div_ceil(B), n[1].div_ceil(B), n[2].div_ceil(B)];
        Voxels {
            origin: min,
            res,
            n,
            bits: vec![0; total.div_ceil(64)],
            occ: vec![0; nb[0] * nb[1] * nb[2]],
            nb,
        }
    }

    /// Same grid, no material.
    pub fn empty_like(&self) -> Voxels {
        Voxels::empty(self.origin, self.bounds().max, self.res)
    }

    fn block(&self, i: usize, j: usize, k: usize) -> usize {
        ((k / B) * self.nb[1] + j / B) * self.nb[0] + i / B
    }

    pub fn from_box(min: Vec3, max: Vec3, res: f64) -> Voxels {
        let mut v = Voxels::empty(min, max, res);
        for k in 0..v.n[2] {
            for j in 0..v.n[1] {
                for i in 0..v.n[0] {
                    let c = v.center(i, j, k);
                    if c.x <= max.x && c.y <= max.y && c.z <= max.z {
                        v.set(i, j, k, true);
                    }
                }
            }
        }
        v
    }

    /// Voxelise a closed mesh on a grid covering `bounds`.
    pub fn from_mesh(mesh: &Mesh, bvh: &Bvh, bounds: Aabb, res: f64) -> Voxels {
        let mut v = Voxels::empty(bounds.min, bounds.max, res);
        v.fill_mesh(mesh, bvh);
        v
    }

    fn idx(&self, i: usize, j: usize, k: usize) -> usize {
        (k * self.n[1] + j) * self.n[0] + i
    }

    pub fn get(&self, i: usize, j: usize, k: usize) -> bool {
        let x = self.idx(i, j, k);
        self.bits[x / 64] >> (x % 64) & 1 == 1
    }

    pub fn set(&mut self, i: usize, j: usize, k: usize, on: bool) {
        let x = self.idx(i, j, k);
        let was = self.bits[x / 64] >> (x % 64) & 1 == 1;
        if was == on {
            return;
        }
        let b = self.block(i, j, k);
        if on {
            self.bits[x / 64] |= 1 << (x % 64);
            self.occ[b] += 1;
        } else {
            self.bits[x / 64] &= !(1 << (x % 64));
            self.occ[b] -= 1;
        }
    }

    pub fn center(&self, i: usize, j: usize, k: usize) -> Vec3 {
        self.origin
            + Vec3::new(
                (i as f64 + 0.5) * self.res,
                (j as f64 + 0.5) * self.res,
                (k as f64 + 0.5) * self.res,
            )
    }

    pub fn bounds(&self) -> Aabb {
        Aabb {
            min: self.origin,
            max: self.origin
                + Vec3::new(self.n[0] as f64, self.n[1] as f64, self.n[2] as f64) * self.res,
        }
    }

    /// Bounding box of the occupied voxels (their full extent), if any.
    pub fn occupied_bounds(&self) -> Option<Aabb> {
        let mut b = Aabb::EMPTY;
        for bk in 0..self.nb[2] {
            for bj in 0..self.nb[1] {
                for bi in 0..self.nb[0] {
                    if self.occ[(bk * self.nb[1] + bj) * self.nb[0] + bi] == 0 {
                        continue;
                    }
                    for k in bk * B..((bk + 1) * B).min(self.n[2]) {
                        for j in bj * B..((bj + 1) * B).min(self.n[1]) {
                            for i in bi * B..((bi + 1) * B).min(self.n[0]) {
                                if self.get(i, j, k) {
                                    b = b.include(self.center(i, j, k));
                                }
                            }
                        }
                    }
                }
            }
        }
        (b.min.x <= b.max.x).then(|| b.expand(self.res * 0.5))
    }

    /// Voxel containing a point, if inside the grid.
    pub fn cell_of(&self, p: Vec3) -> Option<(usize, usize, usize)> {
        let f = |v: f64, o: f64, n: usize| {
            let c = ((v - o) / self.res).floor();
            (c >= 0.0 && (c as usize) < n).then_some(c as usize)
        };
        Some((
            f(p.x, self.origin.x, self.n[0])?,
            f(p.y, self.origin.y, self.n[1])?,
            f(p.z, self.origin.z, self.n[2])?,
        ))
    }

    pub fn material_at(&self, p: Vec3) -> bool {
        self.cell_of(p).is_some_and(|(i, j, k)| self.get(i, j, k))
    }

    pub fn count(&self) -> usize {
        self.bits.iter().map(|w| w.count_ones() as usize).sum()
    }

    pub fn volume(&self) -> f64 {
        self.count() as f64 * self.res.powi(3)
    }

    /// OR a closed mesh into the grid (column ray casting with parity).
    pub fn fill_mesh(&mut self, mesh: &Mesh, bvh: &Bvh) {
        let bb = bvh.aabb();
        let jitter = Vec3::new(self.res * 1.3e-7, self.res * 2.9e-7, 0.0);
        for j in 0..self.n[1] {
            for i in 0..self.n[0] {
                let c = self.center(i, j, 0) + jitter;
                if c.x < bb.min.x || c.x > bb.max.x || c.y < bb.min.y || c.y > bb.max.y {
                    continue;
                }
                let column = Aabb {
                    min: Vec3::new(c.x, c.y, bb.min.z - 1.0),
                    max: Vec3::new(c.x, c.y, bb.max.z + 1.0),
                };
                let mut hits: Vec<f64> = Vec::new();
                bvh.any_in(&column, |t| {
                    if let Some(z) = vertical_hit(&mesh.tri(t), c.x, c.y) {
                        hits.push(z);
                    }
                    false
                });
                hits.sort_by(f64::total_cmp);
                for pair in hits.chunks_exact(2) {
                    let (z0, z1) = (pair[0], pair[1]);
                    for k in 0..self.n[2] {
                        let z = self.origin.z + (k as f64 + 0.5) * self.res;
                        if z >= z0 && z <= z1 {
                            self.set(i, j, k, true);
                        }
                    }
                }
            }
        }
    }

    /// Erode by `steps` voxels (6-neighbourhood); used to get a part "core"
    /// that only counts gouges deeper than a tolerance.
    pub fn eroded(&self, steps: usize) -> Voxels {
        let mut cur = self.clone();
        for _ in 0..steps {
            let mut next = cur.empty_like();
            for k in 0..cur.n[2] {
                for j in 0..cur.n[1] {
                    for i in 0..cur.n[0] {
                        if !cur.get(i, j, k) {
                            continue;
                        }
                        let inner = i > 0
                            && j > 0
                            && k > 0
                            && i + 1 < cur.n[0]
                            && j + 1 < cur.n[1]
                            && k + 1 < cur.n[2]
                            && cur.get(i - 1, j, k)
                            && cur.get(i + 1, j, k)
                            && cur.get(i, j - 1, k)
                            && cur.get(i, j + 1, k)
                            && cur.get(i, j, k - 1)
                            && cur.get(i, j, k + 1);
                        if inner {
                            next.set(i, j, k, true);
                        }
                    }
                }
            }
            cur = next;
        }
        cur
    }

    /// Index range of voxels whose centres may lie in `bb`.
    fn range(&self, bb: &Aabb) -> Option<[std::ops::Range<usize>; 3]> {
        let r = |lo: f64, hi: f64, o: f64, n: usize| {
            let a = ((lo - o) / self.res - 0.5).ceil().max(0.0) as usize;
            let b = (((hi - o) / self.res - 0.5).floor() + 1.0).min(n as f64);
            if b <= 0.0 || a >= b as usize {
                None
            } else {
                Some(a..b as usize)
            }
        };
        Some([
            r(bb.min.x, bb.max.x, self.origin.x, self.n[0])?,
            r(bb.min.y, bb.max.y, self.origin.y, self.n[1])?,
            r(bb.min.z, bb.max.z, self.origin.z, self.n[2])?,
        ])
    }

    /// Visit occupied voxels inside a solid given by `inside` (tool-frame
    /// predicate) placed by `frame`, within the tool-frame box `local_box`.
    fn visit_in(
        &self,
        frame: &Iso,
        local_box: Aabb,
        inside: impl Fn(Vec3) -> bool,
        mut f: impl FnMut(usize, usize, usize) -> bool,
    ) -> bool {
        let Some([ri, rj, rk]) = self.range(&local_box.transformed(frame)) else {
            return false;
        };
        let inv = frame.inverse();
        self.visit_range(
            &ri,
            &rj,
            &rk,
            |i, j, k| inside(inv.apply_point(self.center(i, j, k))),
            &mut f,
        )
    }

    fn cutter_box(c: &CutterShape) -> Aabb {
        Aabb {
            min: Vec3::new(-c.radius, -c.radius, 0.0),
            max: Vec3::new(c.radius, c.radius, c.flute_length),
        }
    }

    /// Any material inside the cutter at this tool frame?
    pub fn cutter_hits(&self, frame: &Iso, c: &CutterShape) -> Option<Vec3> {
        let mut at = None;
        self.visit_in(
            frame,
            Self::cutter_box(c),
            |p| c.contains(p),
            |i, j, k| {
                at = Some(self.center(i, j, k));
                true
            },
        );
        at
    }

    /// Any material inside a cylinder (tool frame: z0..z0+h, radius r)?
    pub fn cylinder_hits(&self, frame: &Iso, z0: f64, h: f64, r: f64) -> bool {
        let bb = Aabb {
            min: Vec3::new(-r, -r, z0),
            max: Vec3::new(r, r, z0 + h),
        };
        self.visit_in(
            frame,
            bb,
            |p| p.z >= z0 && p.z <= z0 + h && p.x * p.x + p.y * p.y <= r * r,
            |_, _, _| true,
        )
    }

    /// Clear the cutter volume; returns the number of voxels removed.
    pub fn remove_cutter(&mut self, frame: &Iso, c: &CutterShape) -> usize {
        let mut hit = Vec::new();
        self.visit_in(
            frame,
            Self::cutter_box(c),
            |p| c.contains(p),
            |i, j, k| {
                hit.push((i, j, k));
                false
            },
        );
        for &(i, j, k) in &hit {
            self.set(i, j, k, false);
        }
        hit.len()
    }

    /// Sweep the cutter from pose `a` to pose `b` (tip + axis, linear tip
    /// motion, interpolated axis). Returns voxels removed.
    pub fn sweep(&mut self, a: &Pose, b: &Pose, c: &CutterShape) -> usize {
        let mut hit = Vec::new();
        self.swept_visit(a, b, c, |i, j, k| {
            hit.push((i, j, k));
            false
        });
        hit.sort_unstable();
        hit.dedup();
        for &(i, j, k) in &hit {
            self.set(i, j, k, false);
        }
        hit.len()
    }

    /// First material voxel inside the volume swept from `a` to `b`.
    pub fn sweep_hits(&self, a: &Pose, b: &Pose, c: &CutterShape) -> Option<Vec3> {
        let mut at = None;
        self.swept_visit(a, b, c, |i, j, k| {
            at = Some(self.center(i, j, k));
            true
        });
        at
    }

    /// Visit occupied voxels inside the swept cutter (may repeat voxels).
    ///
    /// With a constant tool axis the cylindrical part of the cutter is
    /// tested exactly in one pass: a point is inside the swept cylinder when
    /// the parameter interval that keeps it within the cutter's height
    /// overlaps the one that keeps it within its radius. Only the rounded
    /// tip (corner radius) and moves that change the axis are sampled.
    fn swept_visit(
        &self,
        a: &Pose,
        b: &Pose,
        c: &CutterShape,
        mut f: impl FnMut(usize, usize, usize) -> bool,
    ) -> bool {
        let fixed_axis = a.axis.angle_to(b.axis) < 1e-9;
        let rc = c.corner_radius.min(c.radius);
        if fixed_axis && a.tip.distance(b.tip) > self.res * 0.25 {
            let fa = tool_frame(a);
            let inv = fa.inverse();
            let v = inv.apply_vec(b.tip - a.tip);
            let z_lo = rc;
            if z_lo < c.flute_length {
                let local = Aabb {
                    min: Vec3::new(-c.radius, -c.radius, z_lo),
                    max: Vec3::new(c.radius, c.radius, c.flute_length),
                };
                let world_box = local
                    .transformed(&fa)
                    .union(local.transformed(&tool_frame(b)));
                if let Some([ri, rj, rk]) = self.range(&world_box) {
                    let inside =
                        |p: Vec3| swept_cylinder_contains(p, v, z_lo, c.flute_length, c.radius);
                    if self.visit_range(
                        &ri,
                        &rj,
                        &rk,
                        |i, j, k| inside(inv.apply_point(self.center(i, j, k))),
                        &mut f,
                    ) {
                        return true;
                    }
                }
            }
            if rc <= 0.0 {
                return false;
            }
            // Rounded tip: sample poses, testing only the bottom slab.
            let tip = CutterShape {
                flute_length: rc,
                ..*c
            };
            let n = ((a.tip.distance(b.tip) / (self.res * 0.5)).ceil() as usize).max(1);
            for s in 0..=n {
                let t = s as f64 / n as f64;
                let frame = tool_frame(&Pose {
                    tip: a.tip.lerp(b.tip, t),
                    axis: a.axis,
                });
                if self.visit_in(&frame, Self::cutter_box(&tip), |p| tip.contains(p), &mut f) {
                    return true;
                }
            }
            return false;
        }
        let dist = a.tip.distance(b.tip) + a.axis.angle_to(b.axis) * c.flute_length;
        let n = ((dist / (self.res * 0.5)).ceil() as usize).max(1);
        for s in 0..=n {
            let t = s as f64 / n as f64;
            let pose = Pose {
                tip: a.tip.lerp(b.tip, t),
                axis: slerp(a.axis, b.axis, t),
            };
            if self.visit_in(
                &tool_frame(&pose),
                Self::cutter_box(c),
                |p| c.contains(p),
                &mut f,
            ) {
                return true;
            }
        }
        false
    }

    fn visit_range(
        &self,
        ri: &std::ops::Range<usize>,
        rj: &std::ops::Range<usize>,
        rk: &std::ops::Range<usize>,
        inside: impl Fn(usize, usize, usize) -> bool,
        f: &mut impl FnMut(usize, usize, usize) -> bool,
    ) -> bool {
        for bk in rk.start / B..=(rk.end - 1) / B {
            for bj in rj.start / B..=(rj.end - 1) / B {
                for bi in ri.start / B..=(ri.end - 1) / B {
                    if self.occ[(bk * self.nb[1] + bj) * self.nb[0] + bi] == 0 {
                        continue;
                    }
                    let clip = |r: &std::ops::Range<usize>, b: usize| {
                        r.start.max(b * B)..r.end.min(b * B + B)
                    };
                    for k in clip(rk, bk) {
                        for j in clip(rj, bj) {
                            for i in clip(ri, bi) {
                                if self.get(i, j, k) && inside(i, j, k) && f(i, j, k) {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
        false
    }

    /// Top of the material seen looking down `-dir` through `p` (a point
    /// above the material): returns the height along `dir` of the first
    /// occupied voxel, measured from `p` (negative = below `p`), or `None`.
    pub fn first_material_along(&self, p: Vec3, dir: Vec3, max_dist: f64) -> Option<f64> {
        let step = self.res * 0.5;
        let n = (max_dist / step).ceil() as usize;
        (0..=n)
            .map(|s| s as f64 * step)
            .find(|&d| self.material_at(p - dir * d))
            .map(|d| -d)
    }
}

/// Is `p` inside the cylinder (z in [z0, z1], radius r) swept by `v` (all
/// in the start pose's tool frame)? Solves for the parameter range t in
/// [0, 1] where p - t·v is inside.
fn swept_cylinder_contains(p: Vec3, v: Vec3, z0: f64, z1: f64, r: f64) -> bool {
    // Height: z0 <= p.z - t v.z <= z1.
    let (mut ta, mut tb) = (0.0f64, 1.0f64);
    if v.z.abs() < 1e-12 {
        if p.z < z0 || p.z > z1 {
            return false;
        }
    } else {
        let (t1, t2) = ((p.z - z1) / v.z, (p.z - z0) / v.z);
        ta = ta.max(t1.min(t2));
        tb = tb.min(t1.max(t2));
        if ta > tb {
            return false;
        }
    }
    // Radius: |p_xy - t v_xy|² <= r².
    let a = v.x * v.x + v.y * v.y;
    let b = p.x * v.x + p.y * v.y;
    let c = p.x * p.x + p.y * p.y - r * r;
    if a < 1e-18 {
        return c <= 0.0;
    }
    let disc = b * b - a * c;
    if disc < 0.0 {
        return false;
    }
    let s = disc.sqrt();
    let (r1, r2) = ((b - s) / a, (b + s) / a);
    r1.max(ta) <= r2.min(tb)
}

/// Tool frame (tip origin, +Z along the axis) for a pose.
pub fn tool_frame(p: &Pose) -> Iso {
    Iso {
        r: frame_from_z(p.axis.normalized().unwrap_or(Vec3::Z)),
        t: p.tip,
    }
}

/// Z where the vertical line through (x, y) crosses a triangle.
pub(crate) fn vertical_hit(t: &[Vec3; 3], x: f64, y: f64) -> Option<f64> {
    let (a, b, c) = (t[0], t[1], t[2]);
    let d = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
    if d.abs() < 1e-18 {
        return None;
    }
    let l1 = ((b.y - c.y) * (x - c.x) + (c.x - b.x) * (y - c.y)) / d;
    let l2 = ((c.y - a.y) * (x - c.x) + (a.x - c.x) * (y - c.y)) / d;
    let l3 = 1.0 - l1 - l2;
    (l1 >= 0.0 && l2 >= 0.0 && l3 >= 0.0).then_some(l1 * a.z + l2 * b.z + l3 * c.z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cutter_shapes() {
        let ball = CutterShape::ball(10.0, 20.0);
        assert!(ball.contains(Vec3::new(0.0, 0.0, 0.1)));
        assert!(!ball.contains(Vec3::new(4.9, 0.0, 0.1))); // outside the sphere near the tip
        assert!(ball.contains(Vec3::new(4.9, 0.0, 5.0)));
        let flat = CutterShape::flat(10.0, 20.0);
        assert!(flat.contains(Vec3::new(4.9, 0.0, 0.1)));
        assert_eq!(flat.profile_height(3.0), Some(0.0));
        assert!((ball.profile_height(5.0).unwrap() - 5.0).abs() < 1e-12);
        assert!(ball.profile_height(5.1).is_none());
    }

    #[test]
    fn box_and_mesh_voxelisation_agree() {
        let (min, max) = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 6.0, 4.0));
        let a = Voxels::from_box(min, max, 0.5);
        let mesh = Mesh::cuboid(min, max);
        let b = Voxels::from_mesh(&mesh, &Bvh::build(&mesh), Aabb { min, max }, 0.5);
        assert_eq!(a.count(), 20 * 12 * 8);
        assert_eq!(a, b);
        assert!((a.volume() - 240.0).abs() < 1e-9);
    }

    #[test]
    fn slot_removal_volume() {
        // 10 mm flat end mill, 2 mm deep, 40 mm slot across a 60 x 30 x 10 block.
        let mut v = Voxels::from_box(Vec3::new(0.0, 0.0, 0.0), Vec3::new(60.0, 30.0, 10.0), 0.25);
        let c = CutterShape::flat(10.0, 20.0);
        let a = Pose {
            tip: Vec3::new(10.0, 15.0, 8.0),
            axis: Vec3::Z,
        };
        let b = Pose {
            tip: Vec3::new(50.0, 15.0, 8.0),
            axis: Vec3::Z,
        };
        let removed = v.sweep(&a, &b, &c) as f64 * 0.25f64.powi(3);
        let expect = (40.0 * 10.0 + std::f64::consts::PI * 25.0) * 2.0;
        assert!(
            (removed - expect).abs() / expect < 0.03,
            "{removed} vs {expect}"
        );
        assert!(!v.material_at(Vec3::new(30.0, 15.0, 9.0)));
        assert!(v.material_at(Vec3::new(30.0, 15.0, 7.0)));
        assert!(v.material_at(Vec3::new(30.0, 22.0, 9.0)));
        let top = v
            .first_material_along(Vec3::new(30.0, 15.0, 20.0), Vec3::Z, 30.0)
            .unwrap();
        assert!((top + 12.0).abs() <= 0.25, "top {top}");
    }

    #[test]
    fn exact_sweep_matches_sampled_sweep() {
        for cutter in [
            CutterShape::flat(8.0, 15.0),
            CutterShape::ball(8.0, 15.0),
            CutterShape {
                radius: 4.0,
                corner_radius: 1.0,
                flute_length: 15.0,
            },
        ] {
            for (a, b) in [
                (Vec3::new(5.0, 5.0, 6.0), Vec3::new(35.0, 20.0, 4.0)),
                (Vec3::new(20.0, 2.0, 8.0), Vec3::new(20.0, 28.0, 8.0)),
                (Vec3::new(10.0, 10.0, 9.0), Vec3::new(12.0, 11.0, 3.0)),
            ] {
                let axis = Vec3::new(0.2, -0.3, 1.0).normalized().unwrap();
                let (pa, pb) = (Pose { tip: a, axis }, Pose { tip: b, axis });
                let mut exact = Voxels::from_box(Vec3::ZERO, Vec3::new(40.0, 30.0, 10.0), 0.5);
                exact.sweep(&pa, &pb, &cutter);
                // Reference: dense pose sampling with the plain cutter test.
                let mut dense = Voxels::from_box(Vec3::ZERO, Vec3::new(40.0, 30.0, 10.0), 0.5);
                let n = 2000;
                for s in 0..=n {
                    let t = s as f64 / n as f64;
                    dense.remove_cutter(
                        &tool_frame(&Pose {
                            tip: a.lerp(b, t),
                            axis,
                        }),
                        &cutter,
                    );
                }
                let diff = (exact.count() as i64 - dense.count() as i64).abs();
                assert!(
                    diff <= 10,
                    "{cutter:?} {a:?}->{b:?}: exact {} vs dense {}",
                    exact.count(),
                    dense.count()
                );
            }
        }
    }
}
