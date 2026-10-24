//! Geometry for collision checking: triangle meshes, STL loading, simple
//! primitives, an AABB bounding-volume hierarchy, and exact intersection
//! tests.
//!
//! The tool side of every query is a stack of finite cylinders (see
//! [`crate::collision`]), so the core test is *cylinder vs triangle*, which is
//! exact: clip the triangle to the cylinder's height slab, then the distance
//! from the cylinder axis to the clipped polygon is zero if the axis pierces
//! it, else the smallest axis-to-edge distance (the radial distance is
//! convex, so its minimum lies on the boundary).

use crate::math::{Iso, Vec3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub const EMPTY: Aabb = Aabb {
        min: Vec3 {
            x: f64::INFINITY,
            y: f64::INFINITY,
            z: f64::INFINITY,
        },
        max: Vec3 {
            x: f64::NEG_INFINITY,
            y: f64::NEG_INFINITY,
            z: f64::NEG_INFINITY,
        },
    };

    pub fn from_points(pts: &[Vec3]) -> Aabb {
        pts.iter().fold(Aabb::EMPTY, |b, &p| b.include(p))
    }

    pub fn include(self, p: Vec3) -> Aabb {
        Aabb {
            min: Vec3::new(
                self.min.x.min(p.x),
                self.min.y.min(p.y),
                self.min.z.min(p.z),
            ),
            max: Vec3::new(
                self.max.x.max(p.x),
                self.max.y.max(p.y),
                self.max.z.max(p.z),
            ),
        }
    }

    pub fn union(self, o: Aabb) -> Aabb {
        self.include(o.min).include(o.max)
    }

    pub fn expand(self, d: f64) -> Aabb {
        let v = Vec3::new(d, d, d);
        Aabb {
            min: self.min - v,
            max: self.max + v,
        }
    }

    pub fn overlaps(&self, o: &Aabb) -> bool {
        self.min.x <= o.max.x
            && self.max.x >= o.min.x
            && self.min.y <= o.max.y
            && self.max.y >= o.min.y
            && self.min.z <= o.max.z
            && self.max.z >= o.min.z
    }

    pub fn centre(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    /// Bounding box of this box after a rigid transform.
    pub fn transformed(&self, iso: &Iso) -> Aabb {
        let mut b = Aabb::EMPTY;
        for i in 0..8 {
            let p = Vec3::new(
                if i & 1 == 0 { self.min.x } else { self.max.x },
                if i & 2 == 0 { self.min.y } else { self.max.y },
                if i & 4 == 0 { self.min.z } else { self.max.z },
            );
            b = b.include(iso.apply_point(p));
        }
        b
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Mesh {
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<[u32; 3]>,
}

impl Mesh {
    pub fn tri(&self, i: usize) -> [Vec3; 3] {
        let t = self.triangles[i];
        [
            self.vertices[t[0] as usize],
            self.vertices[t[1] as usize],
            self.vertices[t[2] as usize],
        ]
    }

    pub fn len(&self) -> usize {
        self.triangles.len()
    }

    pub fn is_empty(&self) -> bool {
        self.triangles.is_empty()
    }

    pub fn aabb(&self) -> Aabb {
        Aabb::from_points(&self.vertices)
    }

    pub fn transformed(&self, iso: &Iso) -> Mesh {
        Mesh {
            vertices: self.vertices.iter().map(|&v| iso.apply_point(v)).collect(),
            triangles: self.triangles.clone(),
        }
    }

    pub fn append(&mut self, other: &Mesh) {
        let base = self.vertices.len() as u32;
        self.vertices.extend_from_slice(&other.vertices);
        self.triangles.extend(
            other
                .triangles
                .iter()
                .map(|t| [t[0] + base, t[1] + base, t[2] + base]),
        );
    }

    /// Axis-aligned box.
    pub fn cuboid(min: Vec3, max: Vec3) -> Mesh {
        let v: Vec<Vec3> = (0..8)
            .map(|i| {
                Vec3::new(
                    if i & 1 == 0 { min.x } else { max.x },
                    if i & 2 == 0 { min.y } else { max.y },
                    if i & 4 == 0 { min.z } else { max.z },
                )
            })
            .collect();
        let quads = [
            [0, 2, 3, 1], // -z
            [4, 5, 7, 6], // +z
            [0, 1, 5, 4], // -y
            [2, 6, 7, 3], // +y
            [0, 4, 6, 2], // -x
            [1, 3, 7, 5], // +x
        ];
        let mut triangles = Vec::new();
        for q in quads {
            triangles.push([q[0], q[1], q[2]]);
            triangles.push([q[0], q[2], q[3]]);
        }
        Mesh {
            vertices: v,
            triangles,
        }
    }

    /// Closed cylinder from `base` along unit `axis`.
    pub fn cylinder(base: Vec3, axis: Vec3, radius: f64, height: f64, segments: usize) -> Mesh {
        let n = segments.max(3);
        let r = crate::kinematics::frame_from_z(axis);
        let col = |j: usize| Vec3::new(r.0[0][j], r.0[1][j], r.0[2][j]);
        let (ux, uy) = (col(0), col(1));
        let top = base + axis * height;
        let mut vertices = vec![base, top];
        for k in 0..n {
            let a = std::f64::consts::TAU * k as f64 / n as f64;
            let d = ux * (radius * a.cos()) + uy * (radius * a.sin());
            vertices.push(base + d);
            vertices.push(top + d);
        }
        let mut triangles = Vec::new();
        for k in 0..n {
            let (b0, t0) = (2 + 2 * k as u32, 3 + 2 * k as u32);
            let (b1, t1) = (2 + 2 * ((k + 1) % n) as u32, 3 + 2 * ((k + 1) % n) as u32);
            triangles.push([0, b1, b0]);
            triangles.push([1, t0, t1]);
            triangles.push([b0, b1, t1]);
            triangles.push([b0, t1, t0]);
        }
        Mesh {
            vertices,
            triangles,
        }
    }

    /// Parse binary or ASCII STL.
    pub fn from_stl(bytes: &[u8]) -> Result<Mesh, String> {
        let looks_ascii = bytes.starts_with(b"solid")
            && std::str::from_utf8(&bytes[..bytes.len().min(512)])
                .is_ok_and(|s| s.contains("facet"));
        if looks_ascii {
            return Self::from_ascii_stl(std::str::from_utf8(bytes).map_err(|e| e.to_string())?);
        }
        if bytes.len() < 84 {
            return Err("STL file is too short".into());
        }
        let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        if bytes.len() < 84 + n * 50 {
            return Err(format!(
                "binary STL says {n} triangles but the file is truncated"
            ));
        }
        let f = |o: usize| {
            f32::from_le_bytes([bytes[o], bytes[o + 1], bytes[o + 2], bytes[o + 3]]) as f64
        };
        let mut mesh = Mesh::default();
        for t in 0..n {
            let o = 84 + t * 50 + 12;
            let mut idx = [0u32; 3];
            for (k, slot) in idx.iter_mut().enumerate() {
                let p = o + k * 12;
                mesh.vertices.push(Vec3::new(f(p), f(p + 4), f(p + 8)));
                *slot = (mesh.vertices.len() - 1) as u32;
            }
            mesh.triangles.push(idx);
        }
        Ok(mesh)
    }

    fn from_ascii_stl(src: &str) -> Result<Mesh, String> {
        let mut mesh = Mesh::default();
        let mut pending: Vec<u32> = Vec::new();
        for (ln, line) in src.lines().enumerate() {
            let mut it = line.split_whitespace();
            if it.next() != Some("vertex") {
                continue;
            }
            let c: Vec<f64> = it
                .map(|s| s.parse::<f64>())
                .collect::<Result<_, _>>()
                .map_err(|e| format!("STL line {}: {e}", ln + 1))?;
            if c.len() != 3 {
                return Err(format!("STL line {}: vertex needs 3 numbers", ln + 1));
            }
            mesh.vertices.push(Vec3::new(c[0], c[1], c[2]));
            pending.push((mesh.vertices.len() - 1) as u32);
            if pending.len() == 3 {
                mesh.triangles.push([pending[0], pending[1], pending[2]]);
                pending.clear();
            }
        }
        if mesh.triangles.is_empty() {
            return Err("STL contains no triangles".into());
        }
        Ok(mesh)
    }
}

// ------------------------------------------------------------------ BVH

#[derive(Clone, Debug)]
struct Node {
    aabb: Aabb,
    /// Leaf: range into `order`. Inner: child indices.
    kind: NodeKind,
}

#[derive(Clone, Debug)]
enum NodeKind {
    Leaf { start: u32, len: u32 },
    Inner { left: u32, right: u32 },
}

/// Axis-aligned bounding-volume hierarchy over a mesh's triangles.
#[derive(Clone, Debug)]
pub struct Bvh {
    nodes: Vec<Node>,
    order: Vec<u32>,
    boxes: Vec<Aabb>,
}

const LEAF_SIZE: usize = 4;

impl Bvh {
    pub fn build(mesh: &Mesh) -> Bvh {
        let boxes: Vec<Aabb> = (0..mesh.len())
            .map(|i| Aabb::from_points(&mesh.tri(i)))
            .collect();
        let mut order: Vec<u32> = (0..mesh.len() as u32).collect();
        let mut nodes = Vec::new();
        if !order.is_empty() {
            let n = order.len();
            build_node(&boxes, &mut order, 0, n, &mut nodes);
        }
        Bvh {
            nodes,
            order,
            boxes,
        }
    }

    pub fn aabb(&self) -> Aabb {
        self.nodes.first().map_or(Aabb::EMPTY, |n| n.aabb)
    }

    /// Call `f` for every triangle whose box overlaps `query`. Stops early and
    /// returns true when `f` returns true.
    pub fn any_in(&self, query: &Aabb, mut f: impl FnMut(usize) -> bool) -> bool {
        if self.nodes.is_empty() {
            return false;
        }
        let mut stack = vec![0u32];
        while let Some(i) = stack.pop() {
            let node = &self.nodes[i as usize];
            if !node.aabb.overlaps(query) {
                continue;
            }
            match node.kind {
                NodeKind::Leaf { start, len } => {
                    for &t in &self.order[start as usize..(start + len) as usize] {
                        if self.boxes[t as usize].overlaps(query) && f(t as usize) {
                            return true;
                        }
                    }
                }
                NodeKind::Inner { left, right } => {
                    stack.push(left);
                    stack.push(right);
                }
            }
        }
        false
    }

    /// Is `p` inside the closed mesh? Ray-parity test along a slightly
    /// skewed +X ray (skewed to avoid grazing edges and vertices). Only
    /// meaningful for watertight meshes.
    pub fn contains(&self, mesh: &Mesh, p: Vec3) -> bool {
        let root = self.aabb();
        if !root.overlaps(&Aabb { min: p, max: p }) {
            return false;
        }
        let dir = Vec3::new(1.0, 0.000_731_9, 0.000_412_7)
            .normalized()
            .unwrap();
        let len = (root.max - root.min).norm() * 2.0 + 1.0;
        let end = p + dir * len;
        let query = Aabb::from_points(&[p, end]);
        let mut crossings = 0usize;
        self.any_in(&query, |t| {
            if segment_hits_triangle(p, end, &mesh.tri(t)) {
                crossings += 1;
            }
            false
        });
        crossings % 2 == 1
    }

    /// Do two meshes intersect? `b_to_a` maps mesh B's coordinates into A's.
    pub fn meshes_intersect(a: (&Mesh, &Bvh), b: (&Mesh, &Bvh), b_to_a: &Iso) -> bool {
        let (ma, ba) = a;
        let (mb, bb) = b;
        if ba.nodes.is_empty() || bb.nodes.is_empty() {
            return false;
        }
        let mut stack = vec![(0u32, 0u32)];
        while let Some((i, j)) = stack.pop() {
            let na = &ba.nodes[i as usize];
            let nb = &bb.nodes[j as usize];
            if !na.aabb.overlaps(&nb.aabb.transformed(b_to_a)) {
                continue;
            }
            match (&na.kind, &nb.kind) {
                (NodeKind::Leaf { start: sa, len: la }, NodeKind::Leaf { start: sb, len: lb }) => {
                    for &ta in &ba.order[*sa as usize..(*sa + *la) as usize] {
                        let tri_a = ma.tri(ta as usize);
                        for &tb in &bb.order[*sb as usize..(*sb + *lb) as usize] {
                            let tb = mb.tri(tb as usize).map(|p| b_to_a.apply_point(p));
                            if triangles_intersect(&tri_a, &tb) {
                                return true;
                            }
                        }
                    }
                }
                (NodeKind::Inner { left, right }, NodeKind::Leaf { .. }) => {
                    stack.push((*left, j));
                    stack.push((*right, j));
                }
                (NodeKind::Leaf { .. }, NodeKind::Inner { left, right }) => {
                    stack.push((i, *left));
                    stack.push((i, *right));
                }
                (
                    NodeKind::Inner {
                        left: al,
                        right: ar,
                    },
                    NodeKind::Inner {
                        left: bl,
                        right: br,
                    },
                ) => {
                    // Split the larger box.
                    let va = volume(&na.aabb);
                    let vb = volume(&nb.aabb);
                    if va >= vb {
                        stack.push((*al, j));
                        stack.push((*ar, j));
                    } else {
                        stack.push((i, *bl));
                        stack.push((i, *br));
                    }
                }
            }
        }
        false
    }
}

fn volume(b: &Aabb) -> f64 {
    let d = b.max - b.min;
    d.x * d.y * d.z
}

fn build_node(
    boxes: &[Aabb],
    order: &mut [u32],
    start: usize,
    end: usize,
    nodes: &mut Vec<Node>,
) -> u32 {
    let aabb = order[start..end]
        .iter()
        .fold(Aabb::EMPTY, |b, &t| b.union(boxes[t as usize]));
    let id = nodes.len() as u32;
    let len = end - start;
    if len <= LEAF_SIZE {
        nodes.push(Node {
            aabb,
            kind: NodeKind::Leaf {
                start: start as u32,
                len: len as u32,
            },
        });
        return id;
    }
    // Split at the median centroid along the longest axis.
    let centroids = order[start..end]
        .iter()
        .fold(Aabb::EMPTY, |b, &t| b.include(boxes[t as usize].centre()));
    let ext = centroids.max - centroids.min;
    let axis = if ext.x >= ext.y && ext.x >= ext.z {
        0
    } else if ext.y >= ext.z {
        1
    } else {
        2
    };
    let mid = start + len / 2;
    order[start..end].select_nth_unstable_by(len / 2, |&a, &b| {
        boxes[a as usize]
            .centre()
            .get(axis)
            .total_cmp(&boxes[b as usize].centre().get(axis))
    });
    nodes.push(Node {
        aabb,
        kind: NodeKind::Leaf { start: 0, len: 0 },
    }); // placeholder
    let left = build_node(boxes, order, start, mid, nodes);
    let right = build_node(boxes, order, mid, end, nodes);
    nodes[id as usize].kind = NodeKind::Inner { left, right };
    id
}

// ------------------------------------------------------------- primitives

/// Does a solid finite cylinder intersect a triangle? The cylinder's axis is
/// local +Z from z = 0 to z = `height`, radius `radius`; `tri` is given in
/// that local frame.
pub fn cylinder_hits_triangle(tri: &[Vec3; 3], radius: f64, height: f64) -> bool {
    // Quick reject on the slab and the radial box.
    let zmin = tri[0].z.min(tri[1].z).min(tri[2].z);
    let zmax = tri[0].z.max(tri[1].z).max(tri[2].z);
    if zmax < 0.0 || zmin > height {
        return false;
    }
    let out = |f: fn(&Vec3) -> f64| {
        let v = [f(&tri[0]), f(&tri[1]), f(&tri[2])];
        v.iter().all(|&x| x > radius) || v.iter().all(|&x| x < -radius)
    };
    if out(|p| p.x) || out(|p| p.y) {
        return false;
    }

    let poly = clip_slab(tri, 0.0, height);
    if poly.len() < 3 {
        // Degenerate (an edge or point on the slab boundary).
        return poly
            .windows(2)
            .any(|w| seg_dist_to_z_axis(w[0], w[1]) <= radius)
            || poly
                .iter()
                .any(|p| (p.x * p.x + p.y * p.y).sqrt() <= radius);
    }
    // Does the axis pierce the triangle inside the slab?
    let n = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
    if n.z.abs() > 1e-12 {
        let z = n.dot(tri[0]) / n.z;
        if (0.0..=height).contains(&z) && point_in_triangle(Vec3::new(0.0, 0.0, z), tri, n) {
            return true;
        }
    }
    (0..poly.len()).any(|k| seg_dist_to_z_axis(poly[k], poly[(k + 1) % poly.len()]) <= radius)
}

fn clip_slab(tri: &[Vec3; 3], z0: f64, z1: f64) -> Vec<Vec3> {
    let clip = |pts: Vec<Vec3>, keep: &dyn Fn(f64) -> bool, plane: f64| -> Vec<Vec3> {
        let mut out = Vec::new();
        for k in 0..pts.len() {
            let a = pts[k];
            let b = pts[(k + 1) % pts.len()];
            let (ia, ib) = (keep(a.z), keep(b.z));
            if ia {
                out.push(a);
            }
            if ia != ib {
                let t = (plane - a.z) / (b.z - a.z);
                out.push(a.lerp(b, t));
            }
        }
        out
    };
    let p = clip(tri.to_vec(), &|z| z >= z0, z0);
    if p.is_empty() {
        return p;
    }
    clip(p, &|z| z <= z1, z1)
}

/// Distance from the Z axis (x = y = 0) to a segment: the 2D distance from
/// the origin to the segment's XY projection.
fn seg_dist_to_z_axis(a: Vec3, b: Vec3) -> f64 {
    let (ax, ay, dx, dy) = (a.x, a.y, b.x - a.x, b.y - a.y);
    let l2 = dx * dx + dy * dy;
    let t = if l2 < 1e-18 {
        0.0
    } else {
        (-(ax * dx + ay * dy) / l2).clamp(0.0, 1.0)
    };
    let (px, py) = (ax + dx * t, ay + dy * t);
    (px * px + py * py).sqrt()
}

fn point_in_triangle(p: Vec3, t: &[Vec3; 3], n: Vec3) -> bool {
    let s0 = (t[1] - t[0]).cross(p - t[0]).dot(n);
    let s1 = (t[2] - t[1]).cross(p - t[1]).dot(n);
    let s2 = (t[0] - t[2]).cross(p - t[2]).dot(n);
    (s0 >= 0.0 && s1 >= 0.0 && s2 >= 0.0) || (s0 <= 0.0 && s1 <= 0.0 && s2 <= 0.0)
}

/// Segment `p0`–`p1` vs triangle (Möller–Trumbore).
pub fn segment_hits_triangle(p0: Vec3, p1: Vec3, t: &[Vec3; 3]) -> bool {
    let d = p1 - p0;
    let e1 = t[1] - t[0];
    let e2 = t[2] - t[0];
    let h = d.cross(e2);
    let a = e1.dot(h);
    if a.abs() < 1e-14 {
        return false; // parallel (coplanar contact is ignored)
    }
    let f = 1.0 / a;
    let s = p0 - t[0];
    let u = f * s.dot(h);
    if !(0.0..=1.0).contains(&u) {
        return false;
    }
    let q = s.cross(e1);
    let v = f * d.dot(q);
    if v < 0.0 || u + v > 1.0 {
        return false;
    }
    let w = f * e2.dot(q);
    (0.0..=1.0).contains(&w)
}

/// Where segment `p0`–`p1` crosses a triangle, as a fraction along the
/// segment (Möller–Trumbore), if it does.
pub fn segment_triangle_param(p0: Vec3, p1: Vec3, t: &[Vec3; 3]) -> Option<f64> {
    let d = p1 - p0;
    let e1 = t[1] - t[0];
    let e2 = t[2] - t[0];
    let h = d.cross(e2);
    let a = e1.dot(h);
    if a.abs() < 1e-14 {
        return None;
    }
    let f = 1.0 / a;
    let s = p0 - t[0];
    let u = f * s.dot(h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = f * d.dot(q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let w = f * e2.dot(q);
    (0.0..=1.0).contains(&w).then_some(w)
}

/// Two triangles intersect when an edge of one crosses the other.
/// (Exactly coplanar overlaps are not reported.)
pub fn triangles_intersect(a: &[Vec3; 3], b: &[Vec3; 3]) -> bool {
    let edges = |t: &[Vec3; 3]| [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])];
    edges(a)
        .iter()
        .any(|&(p, q)| segment_hits_triangle(p, q, b))
        || edges(b)
            .iter()
            .any(|&(p, q)| segment_hits_triangle(p, q, a))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tri(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [Vec3; 3] {
        [a.into(), b.into(), c.into()]
    }

    #[test]
    fn cylinder_vs_triangles() {
        // Horizontal triangle crossing the axis at z = 5.
        let t = tri([-10.0, -10.0, 5.0], [10.0, -10.0, 5.0], [0.0, 10.0, 5.0]);
        assert!(cylinder_hits_triangle(&t, 1.0, 10.0));
        assert!(!cylinder_hits_triangle(&t, 1.0, 4.0)); // below the plane
                                                        // Vertical wall 3 mm from the axis.
        let w = tri([3.0, -10.0, 0.0], [3.0, 10.0, 0.0], [3.0, 0.0, 10.0]);
        assert!(!cylinder_hits_triangle(&w, 2.9, 10.0));
        assert!(cylinder_hits_triangle(&w, 3.1, 10.0));
        // Wall only above the cylinder top.
        let hi = tri([0.5, -10.0, 11.0], [0.5, 10.0, 11.0], [0.5, 0.0, 20.0]);
        assert!(!cylinder_hits_triangle(&hi, 5.0, 10.0));
    }

    #[test]
    fn bvh_matches_brute_force() {
        let mut m = Mesh::cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(10.0, 10.0, 10.0));
        m.append(&Mesh::cylinder(
            Vec3::new(30.0, 0.0, 0.0),
            Vec3::Z,
            5.0,
            20.0,
            24,
        ));
        let bvh = Bvh::build(&m);
        for q in [
            Aabb {
                min: Vec3::new(-1.0, -1.0, -1.0),
                max: Vec3::new(1.0, 1.0, 1.0),
            },
            Aabb {
                min: Vec3::new(26.0, -1.0, 5.0),
                max: Vec3::new(27.0, 1.0, 6.0),
            },
            Aabb {
                min: Vec3::new(15.0, 15.0, 15.0),
                max: Vec3::new(16.0, 16.0, 16.0),
            },
        ] {
            let mut via_bvh = Vec::new();
            bvh.any_in(&q, |t| {
                via_bvh.push(t);
                false
            });
            via_bvh.sort();
            let brute: Vec<usize> = (0..m.len())
                .filter(|&t| Aabb::from_points(&m.tri(t)).overlaps(&q))
                .collect();
            assert_eq!(via_bvh, brute);
        }
    }

    #[test]
    fn mesh_pair_intersection() {
        let a = Mesh::cuboid(Vec3::ZERO, Vec3::new(10.0, 10.0, 10.0));
        let b = Mesh::cuboid(Vec3::ZERO, Vec3::new(4.0, 4.0, 4.0));
        let (ba, bb) = (Bvh::build(&a), Bvh::build(&b));
        // Poking through the +X face.
        assert!(Bvh::meshes_intersect(
            (&a, &ba),
            (&b, &bb),
            &Iso::translation(Vec3::new(8.0, 3.0, 3.0))
        ));
        // Clear of it.
        assert!(!Bvh::meshes_intersect(
            (&a, &ba),
            (&b, &bb),
            &Iso::translation(Vec3::new(11.0, 3.0, 3.0))
        ));
    }

    #[test]
    fn stl_roundtrip_ascii() {
        let src = "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t\n";
        let m = Mesh::from_stl(src.as_bytes()).unwrap();
        assert_eq!(m.len(), 1);
        // Binary with one triangle.
        let mut b = vec![0u8; 80];
        b.extend_from_slice(&1u32.to_le_bytes());
        for v in [0f32, 0., 1., 0., 0., 0., 1., 0., 0., 0., 1., 0.] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        b.extend_from_slice(&[0, 0]);
        let m = Mesh::from_stl(&b).unwrap();
        assert_eq!(m.tri(0)[1], Vec3::new(1.0, 0.0, 0.0));
    }
}
