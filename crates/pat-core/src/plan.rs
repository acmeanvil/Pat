//! Multi-step planning pipelines built from the lower-level modules.
//!
//! [`auto_rough`] is automatic 3+2 roughing:
//!
//! 1. Choose orientations ([`crate::orient`]).
//! 2. Rough from each one in turn ([`crate::rough`]). Each setup starts from
//!    the stock the previous setups left, so later setups only remove what
//!    is still there.
//! 3. Join the setups with safe transitions. The tool backs out to a sphere
//!    around the part and swings to the next orientation in small steps along
//!    that sphere, so joint-interpolated rapids never sweep through the work.

use crate::collision::World;
use crate::geom::Mesh;
use crate::kinematics::Kinematics;
use crate::math::{slerp, Vec3};
use crate::orient::{select_orientations, OrientOptions, Orientation};
use crate::post::ClMove;
use crate::rough::{machining_frame, rough_with, LevelStats, RoughParams};
use crate::stock::Voxels;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct AutoRoughParams {
    pub rough: RoughParams,
    pub orient: OrientOptions,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SetupSummary {
    pub orientation: Orientation,
    pub levels: Vec<LevelStats>,
    pub removed_mm3: f64,
    pub warnings: Vec<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AutoRoughResult {
    pub setups: Vec<SetupSummary>,
    pub moves: Vec<ClMove>,
    pub removed_mm3: f64,
    pub remaining_mm3: f64,
    /// Surface fraction no orientation could reach.
    pub uncovered: f64,
    pub summary: String,
}

/// Voxel stock from the scene's `stock` bodies (table frame).
pub fn stock_from_world(world: &World, resolution: f64) -> Option<Voxels> {
    let stocks = world.table_meshes(crate::collision::Role::Stock);
    if stocks.is_empty() {
        return None;
    }
    let res = resolution.max(0.05);
    let bounds = stocks
        .iter()
        .fold(crate::geom::Aabb::EMPTY, |b, (_, m)| b.union(m.aabb()))
        .expand(res);
    let mut v = Voxels::empty(bounds.min, bounds.max, res);
    for (_, m) in &stocks {
        v.fill_mesh(m, &crate::geom::Bvh::build(m));
    }
    Some(v)
}

/// Rough `stock` to `parts` (table frame) in as many 3+2 setups as needed.
pub fn auto_rough(
    parts: &[&Mesh],
    stock: &mut Voxels,
    kin: &Kinematics,
    world: Option<&World>,
    wcs_origin: Vec3,
    p: &AutoRoughParams,
) -> AutoRoughResult {
    let before = stock.volume();
    let orient = select_orientations(parts, kin, world, wcs_origin, &p.orient);

    // Radius of a sphere around part zero that clears the stock, for
    // transitions between setups.
    let sb = stock.bounds();
    let far = (0..8)
        .map(|i| {
            Vec3::new(
                if i & 1 == 0 { sb.min.x } else { sb.max.x },
                if i & 2 == 0 { sb.min.y } else { sb.max.y },
                if i & 4 == 0 { sb.min.z } else { sb.max.z },
            )
            .distance(wcs_origin)
        })
        .fold(0.0, f64::max);
    let safe_r = far + p.rough.clearance + p.rough.cutter.radius;

    let mut moves: Vec<ClMove> = Vec::new();
    let mut setups = Vec::new();
    let mut prev_axis: Option<Vec3> = None;
    for o in &orient.orientations {
        let f = machining_frame(wcs_origin, o.axis);
        let (obstacles, sections) = match world {
            Some(w) => (
                w.setup_obstacles(kin, &o.joints),
                w.head_sections(kin, p.orient.tool),
            ),
            None => (Vec::new(), Vec::new()),
        };
        let obs: Vec<&Mesh> = obstacles.iter().collect();
        let r = rough_with(parts, &obs, &sections, stock, &f, wcs_origin, &p.rough);
        if r.moves.is_empty() {
            setups.push(SetupSummary {
                orientation: o.clone(),
                levels: r.levels,
                removed_mm3: 0.0,
                warnings: r.warnings,
            });
            continue;
        }
        if let Some(a) = prev_axis {
            // Out along the old axis, round the sphere, in along the new one.
            let steps = ((a.angle_to(o.axis).to_degrees() / 10.0).ceil() as usize).max(1);
            for k in 0..=steps {
                let d = slerp(a, o.axis, k as f64 / steps as f64);
                moves.push(ClMove::Rapid {
                    tip: d * safe_r,
                    axis: d,
                });
            }
        }
        moves.extend(r.moves.iter().cloned());
        prev_axis = Some(o.axis);
        setups.push(SetupSummary {
            orientation: o.clone(),
            levels: r.levels,
            removed_mm3: r.removed_mm3,
            warnings: r.warnings,
        });
    }
    if let Some(a) = prev_axis {
        moves.push(ClMove::Rapid {
            tip: a * safe_r,
            axis: a,
        });
    }
    let after = stock.volume();
    let summary = format!(
        "{} setup(s), removed {:.0} mm³ ({:.0}% of the stock), {:.1}% of the part surface out of reach",
        setups.len(),
        before - after,
        100.0 * (before - after) / before.max(1e-9),
        orient.uncovered * 100.0
    );
    AutoRoughResult {
        setups,
        moves,
        removed_mm3: before - after,
        remaining_mm3: after,
        uncovered: orient.uncovered,
        summary,
    }
}
