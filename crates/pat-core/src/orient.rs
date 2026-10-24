//! Automatic 3+2 orientation selection.
//!
//! The question: which fixed tool directions, used one after another, let a
//! 3-axis-style strategy reach the whole part, and which can the machine
//! actually achieve?
//!
//! 1. Sample points evenly by area over the part surface, with outward
//!    normals. Normals are checked against the mesh, so STL winding does not
//!    matter.
//! 2. Candidate directions: an even spread over the reachable cone, plus the
//!    machine axes and the dominant face normals of the part (prismatic parts
//!    have faces aligned with them).
//! 3. A point counts as machinable from direction d if it faces d and a ray
//!    from the point along d escapes the part. Steep faces (walls relative to
//!    d) count only if the flank of the tool can reach down to them.
//!    A direction is usable if the machine can orient the tool that way
//!    within its limits (and, with a scene, without the tool assembly
//!    colliding above the part).
//! 4. Greedy cover: repeatedly take the direction that adds the most
//!    uncovered surface area, preferring smaller tilts on ties, until the
//!    target coverage is reached or nothing useful is left.
//!
//! Each chosen orientation comes with machine joints, G68.2 Euler angles
//! (consistent with [`crate::rough::machining_frame`]) and a plain-language
//! explanation.

use crate::access::cap_directions;
use crate::collision::World;
use crate::geom::{segment_hits_triangle, Aabb, Bvh, Mesh};
use crate::kinematics::{Kinematics, Pose};
use crate::math::Vec3;
use crate::rough::machining_frame;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct OrientOptions {
    /// Surface sample points.
    pub samples: usize,
    /// Directions spread over the cone (more are added automatically).
    pub candidates: usize,
    /// Largest tilt from +Z to consider (degrees).
    pub max_tilt_deg: f64,
    /// Stop once this fraction of the (considered) surface is covered.
    pub target_coverage: f64,
    /// Stop when the best next direction adds less than this fraction.
    pub min_gain: f64,
    pub max_setups: usize,
    /// Ignore faces whose normal is within this angle of straight down (the
    /// side the part sits on). 0 disables.
    pub ignore_down_deg: f64,
    /// Tool used for the clearance check when a scene is given.
    pub tool: Option<u32>,
    /// A face steeper than 60° to the tool axis is only machinable with the
    /// tool's flank if it extends no more than this far along the axis below
    /// the top of the part (≈ usable flute length, mm).
    pub max_wall_depth: f64,
}

impl Default for OrientOptions {
    fn default() -> Self {
        OrientOptions {
            samples: 3000,
            candidates: 400,
            max_tilt_deg: 120.0,
            target_coverage: 0.995,
            min_gain: 0.005,
            max_setups: 8,
            ignore_down_deg: 10.0,
            tool: None,
            max_wall_depth: 25.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Orientation {
    /// Tool axis in the WCS.
    pub axis: Vec3,
    pub tilt_deg: f64,
    /// Machine joints that put the tool along `axis` above part zero.
    pub joints: Vec<f64>,
    /// G68.2 X0 Y0 Z0 I J K (ZXZ Euler angles, degrees).
    pub g68_2: [f64; 3],
    /// Surface fraction this orientation adds.
    pub gain: f64,
    /// Coverage after adding it.
    pub coverage: f64,
    pub explanation: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OrientResult {
    pub orientations: Vec<Orientation>,
    /// Fraction of the considered surface no chosen direction reaches.
    pub uncovered: f64,
    pub surface_area_mm2: f64,
    /// Area ignored as resting on the table (mm²).
    pub ignored_area_mm2: f64,
    /// A few uncovered points (WCS) for display.
    pub uncovered_points: Vec<Vec3>,
}

/// Within 15° of perpendicular counts as seeing a face head-on.
const FACE_ON_COS: f64 = 0.966;

struct Sample {
    p: Vec3,
    n: Vec3,
    w: f64,
}

/// Choose 3+2 orientations for `parts` (table frame). Part zero is
/// `wcs_origin`.
pub fn select_orientations(
    parts: &[&Mesh],
    kin: &Kinematics,
    world: Option<&World>,
    wcs_origin: Vec3,
    o: &OrientOptions,
) -> OrientResult {
    let mut mesh = Mesh::default();
    for m in parts {
        mesh.append(m);
    }
    let bvh = Bvh::build(&mesh);
    // Containment per part: parity over a merged mesh is wrong where parts
    // touch or overlap.
    let solids: Vec<(&Mesh, Bvh)> = parts.iter().map(|m| (*m, Bvh::build(m))).collect();
    let inside = |q: Vec3| solids.iter().any(|(m, b)| b.contains(m, q));
    let bb = mesh.aabb();
    let size = (bb.max - bb.min).norm().max(1e-3);
    let total: f64 = (0..mesh.len()).map(|t| tri_area(&mesh.tri(t))).sum();

    let centre = (bb.min + bb.max) * 0.5;
    // A direction is usable if the machine reaches the part centre and every
    // bounding-box corner with the tool along it, without the tool or machine
    // bodies colliding (when a scene is given).
    let corners: Vec<Vec3> = (0..8)
        .map(|i| {
            Vec3::new(
                if i & 1 == 0 { bb.min.x } else { bb.max.x },
                if i & 2 == 0 { bb.min.y } else { bb.max.y },
                if i & 4 == 0 { bb.min.z } else { bb.max.z },
            )
        })
        .collect();
    let reach_of = |d: Vec3| -> Option<Vec<f64>> {
        let at = |p: Vec3, prev: Option<&[f64]>| {
            kin.inverse(
                &Pose {
                    tip: p - wcs_origin,
                    axis: d,
                },
                prev,
            )
            .ok()
        };
        let main = at(centre + d * (size * 0.5), None)?;
        for &c in std::iter::once(&centre).chain(&corners) {
            let sol = at(c, Some(&main.joints))?;
            if let Some(w) = world {
                let class = crate::collision::MoveClass::Feed;
                // Reaching into the part is the visibility test's job; here
                // only the machine and fixtures matter.
                let blocked = w
                    .check(kin, &sol.joints, o.tool, class, 0.0)
                    .iter()
                    .any(|h| h.role != crate::collision::Role::Part);
                if blocked {
                    return None;
                }
            }
        }
        Some(main.joints)
    };
    // Faces pointing down are only ignored (as the side the part rests on)
    // when the machine cannot point the tool up at them anyway. On a 4th
    // axis or a trunnion that tilts past 90°, they are real surface.
    let can_reach_underside = std::iter::once(-Vec3::Z)
        .chain([
            Vec3::new(0.0, 0.5, -0.866),
            Vec3::new(0.0, -0.5, -0.866),
            Vec3::new(0.5, 0.0, -0.866),
            Vec3::new(-0.5, 0.0, -0.866),
        ])
        .chain(cap_directions(-Vec3::Z, 60.0, 24))
        .any(|d| reach_of(d).is_some());
    let ignore_down = o.ignore_down_deg > 0.0 && !can_reach_underside;

    // 1. Area-weighted samples with outward normals.
    let mut rng = 0x9e37_79b9_7f4a_7c15u64;
    let mut rand = move || {
        rng ^= rng << 13;
        rng ^= rng >> 7;
        rng ^= rng << 17;
        (rng >> 11) as f64 / (1u64 << 53) as f64
    };
    let cum: Vec<f64> = (0..mesh.len())
        .scan(0.0, |acc, t| {
            *acc += tri_area(&mesh.tri(t));
            Some(*acc)
        })
        .collect();
    let eps = size * 1e-4;
    let mut samples = Vec::with_capacity(o.samples);
    let mut ignored = 0.0;
    let mut internal = 0.0;
    let w = total / o.samples.max(1) as f64;
    let down_cos = o.ignore_down_deg.to_radians().cos();
    for _ in 0..o.samples.max(1) {
        let x = rand() * total;
        let t = cum.partition_point(|&c| c < x).min(mesh.len() - 1);
        let tri = mesh.tri(t);
        let (mut u, mut v) = (rand(), rand());
        if u + v > 1.0 {
            u = 1.0 - u;
            v = 1.0 - v;
        }
        let p = tri[0] + (tri[1] - tri[0]) * u + (tri[2] - tri[0]) * v;
        let Some(mut n) = (tri[1] - tri[0]).cross(tri[2] - tri[0]).normalized() else {
            continue;
        };
        let (front, back) = (inside(p + n * eps), inside(p - n * eps));
        if front && back {
            internal += w;
            continue; // face between touching parts, not real surface
        }
        if front {
            n = -n;
        }
        if ignore_down && n.dot(-Vec3::Z) >= down_cos {
            ignored += w;
            continue;
        }
        samples.push(Sample { p, n, w });
    }
    let considered: f64 = samples.iter().map(|s| s.w).sum::<f64>().max(1e-12);

    // 2. Candidate directions.
    let mut dirs = cap_directions(Vec3::Z, o.max_tilt_deg, o.candidates);
    for d in [Vec3::Z, Vec3::X, -Vec3::X, Vec3::Y, -Vec3::Y] {
        dirs.push(d);
    }
    dirs.extend(dominant_normals(&samples, 0.02));
    dirs.retain(|d| d.angle_to(Vec3::Z).to_degrees() <= o.max_tilt_deg + 1e-9);
    let mut uniq: Vec<Vec3> = Vec::new();
    for d in dirs {
        if uniq.iter().all(|u| u.angle_to(d) > 0.5f64.to_radians()) {
            uniq.push(d);
        }
    }

    // 3. Usable directions and what each sees.
    let mut usable: Vec<(Vec3, Vec<f64>, Vec<bool>)> = Vec::new();
    for d in uniq {
        let Some(joints) = reach_of(d) else { continue };
        let top = mesh
            .vertices
            .iter()
            .map(|v| v.dot(d))
            .fold(f64::NEG_INFINITY, f64::max);
        let seen: Vec<bool> = samples
            .iter()
            .map(|s| {
                let c = s.n.dot(d);
                if c < -0.02 {
                    return false;
                }
                // Steep faces are cut with the flank: it must reach down to them.
                if c < 0.5 && top - s.p.dot(d) > o.max_wall_depth {
                    return false;
                }
                !ray_blocked(&mesh, &bvh, s.p + s.n * eps + d * eps, d, size * 2.0)
            })
            .collect();
        usable.push((d, joints, seen));
    }

    // 4. Greedy cover.
    let mut covered = vec![false; samples.len()];
    let mut out: Vec<Orientation> = Vec::new();
    let mut coverage = 0.0;
    while out.len() < o.max_setups && coverage < o.target_coverage {
        let mut best: Option<(f64, usize)> = None;
        for (k, (d, _, seen)) in usable.iter().enumerate() {
            let gain: f64 = samples
                .iter()
                .zip(seen)
                .zip(&covered)
                .filter(|((_, &s), &c)| s && !c)
                .map(|((smp, _), _)| smp.w)
                .sum::<f64>()
                / considered;
            // Prefer directions that see faces head-on (floors rather than
            // flank-cut walls), then smaller tilts.
            let face_on: f64 = samples
                .iter()
                .zip(seen)
                .zip(&covered)
                .filter(|((smp, &s), &c)| s && !c && smp.n.dot(*d) >= FACE_ON_COS)
                .map(|((smp, _), _)| smp.w)
                .sum::<f64>()
                / considered;
            let score = gain + face_on - 0.01 * d.angle_to(Vec3::Z) / std::f64::consts::PI;
            if best.is_none_or(|b| score > b.0) {
                best = Some((score, k));
            }
        }
        let Some((_, k)) = best else { break };
        let (d, joints, seen) = &usable[k];
        let gain: f64 = samples
            .iter()
            .zip(seen)
            .zip(&covered)
            .filter(|((_, &s), &c)| s && !c)
            .map(|((smp, _), _)| smp.w)
            .sum::<f64>()
            / considered;
        if gain < o.min_gain {
            break;
        }
        for (c, &s) in covered.iter_mut().zip(seen) {
            *c |= s;
        }
        coverage += gain;
        let tilt = d.angle_to(Vec3::Z).to_degrees();
        let rot: Vec<String> = kin
            .rotary_axes()
            .iter()
            .map(|&i| format!("{}{:.1}", kin.machine().axes[i].name, joints[i]))
            .collect();
        let lean = if tilt < 0.5 {
            "straight down".to_string()
        } else {
            format!("tilted {tilt:.0}° toward {}", heading(*d))
        };
        out.push(Orientation {
            axis: *d,
            tilt_deg: tilt,
            joints: joints.clone(),
            g68_2: euler_zxz(*d),
            gain,
            coverage,
            explanation: format!(
                "setup {}: tool {lean} ({}) reaches {:.1}% more of the surface, {:.1}% in total",
                out.len() + 1,
                if rot.is_empty() {
                    "no rotary".into()
                } else {
                    rot.join(" ")
                },
                gain * 100.0,
                coverage * 100.0
            ),
        });
    }
    let uncovered_points = samples
        .iter()
        .zip(&covered)
        .filter(|(_, &c)| !c)
        .take(200)
        .map(|(s, _)| s.p - wcs_origin)
        .collect();
    OrientResult {
        orientations: out,
        uncovered: (1.0 - coverage).max(0.0),
        surface_area_mm2: total - internal,
        ignored_area_mm2: ignored,
        uncovered_points,
    }
}

fn tri_area(t: &[Vec3; 3]) -> f64 {
    (t[1] - t[0]).cross(t[2] - t[0]).norm() * 0.5
}

fn ray_blocked(mesh: &Mesh, bvh: &Bvh, from: Vec3, d: Vec3, len: f64) -> bool {
    let to = from + d * len;
    let q = Aabb::from_points(&[from, to]);
    bvh.any_in(&q, |t| segment_hits_triangle(from, to, &mesh.tri(t)))
}

/// Normals shared by at least `min_frac` of the sampled area.
fn dominant_normals(samples: &[Sample], min_frac: f64) -> Vec<Vec3> {
    let total: f64 = samples.iter().map(|s| s.w).sum();
    let mut groups: Vec<(Vec3, f64)> = Vec::new();
    for s in samples {
        match groups
            .iter_mut()
            .find(|(n, _)| n.angle_to(s.n) < 2f64.to_radians())
        {
            Some(g) => g.1 += s.w,
            None => groups.push((s.n, s.w)),
        }
    }
    groups
        .into_iter()
        .filter(|(_, w)| *w >= min_frac * total)
        .map(|(n, _)| n)
        .collect()
}

fn heading(d: Vec3) -> String {
    let mut parts = Vec::new();
    for (v, name) in [(d.x, "X"), (d.y, "Y")] {
        if v.abs() > 0.2 {
            parts.push(format!("{}{name}", if v > 0.0 { "+" } else { "−" }));
        }
    }
    if parts.is_empty() {
        "Z".into()
    } else {
        parts.join(" ")
    }
}

/// G68.2 ZXZ Euler angles (degrees) of the machining frame for axis `d`.
pub fn euler_zxz(d: Vec3) -> [f64; 3] {
    let r = machining_frame(Vec3::ZERO, d).r.0;
    let beta = r[2][2].clamp(-1.0, 1.0).acos();
    if beta.sin().abs() < 1e-9 {
        let a = r[1][0].atan2(r[0][0]);
        return [a.to_degrees(), beta.to_degrees(), 0.0];
    }
    let alpha = r[0][2].atan2(-r[1][2]);
    let gamma = r[2][0].atan2(r[2][1]);
    [alpha.to_degrees(), beta.to_degrees(), gamma.to_degrees()]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Mat3;

    #[test]
    fn euler_matches_frame() {
        for d in [
            Vec3::Z,
            Vec3::new(0.0, -0.5, 0.866),
            Vec3::X,
            Vec3::new(0.3, 0.4, -0.2),
        ] {
            let d = d.normalized().unwrap();
            let [a, b, c] = euler_zxz(d);
            let r = Mat3::rotation(Vec3::Z, a.to_radians())
                .mul_mat(&Mat3::rotation(Vec3::X, b.to_radians()))
                .mul_mat(&Mat3::rotation(Vec3::Z, c.to_radians()));
            let f = machining_frame(Vec3::ZERO, d).r;
            for i in 0..3 {
                for j in 0..3 {
                    assert!((r.0[i][j] - f.0[i][j]).abs() < 1e-9, "{d:?}");
                }
            }
        }
    }
}
