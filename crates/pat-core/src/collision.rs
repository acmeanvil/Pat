//! Collision checking between the tool assembly, the part setup and the
//! machine.
//!
//! - **Tools** are stacks of cutter, shank and holder sections from the tip
//!   up, each a cylinder or cone. Cones are split into short cylinders using
//!   the larger radius, so checks err on the safe side.
//! - **Bodies** are triangle meshes (STL, box or cylinder) with a role:
//!   - `part`: finished geometry;
//!   - `stock`: raw material (not used for checks until material removal is
//!     simulated);
//!   - `fixture`: vises, clamps, plates;
//!   - `machine`: head, table, trunnion, enclosure parts.
//!
//!   Part, stock and fixture bodies are given in WCS coordinates and ride on
//!   the table. Machine bodies attach to any link (see
//!   [`Kinematics::link`]) and are given in the base frame at home.
//!
//! Rules:
//!
//! | Tool section | part           | fixture | machine |
//! |--------------|----------------|---------|---------|
//! | cutter       | rapids only    | always  | always  |
//! | shank        | always         | always  | always  |
//! | holder       | always         | always  | always  |
//!
//! Machine and fixture bodies on different moving groups (head, table, base)
//! are also checked against each other, e.g. spindle housing vs trunnion.
//!
//! Meshes should be closed (watertight): a tool section entirely inside a
//! body is found with a point-in-mesh test, which needs a closed surface.

use crate::geom::{cylinder_hits_triangle, Aabb, Bvh, Mesh};
use crate::kinematics::{Kinematics, Link};
use crate::math::{Iso, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum SegmentKind {
    Cutter,
    Shank,
    Holder,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ToolSegment {
    pub kind: SegmentKind,
    pub length: f64,
    /// Cylinder diameter; or give `diameter_bottom` and `diameter_top` for a cone.
    #[serde(default)]
    pub diameter: Option<f64>,
    #[serde(default)]
    pub diameter_bottom: Option<f64>,
    #[serde(default)]
    pub diameter_top: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ToolDef {
    pub number: u32,
    #[serde(default)]
    pub name: String,
    /// Tip to spindle gauge line (mm); this is the kinematic tool length.
    pub gauge_length: f64,
    /// Sections from the tip upward.
    pub segments: Vec<ToolSegment>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum Shape {
    Box {
        min: Vec3,
        max: Vec3,
    },
    Cylinder {
        base: Vec3,
        axis: Vec3,
        radius: f64,
        height: f64,
    },
    /// Path is relative to the file that defines the body.
    Stl {
        path: String,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Role {
    Part,
    Stock,
    Fixture,
    Machine,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyDef {
    pub name: String,
    pub role: Role,
    /// Link name for machine bodies ("base", "head", "table", or an axis
    /// letter). Part, stock and fixture bodies ride on the table.
    #[serde(default)]
    pub attach: Option<String>,
    /// Work offset whose part zero the geometry is relative to (part, stock
    /// and fixture bodies).
    #[serde(default)]
    pub wcs: Option<String>,
    pub shape: Shape,
    /// Set false to leave a body out of checks (e.g. a bed that touches the
    /// table by design).
    #[serde(default = "yes")]
    pub collide: bool,
}

fn yes() -> bool {
    true
}

/// Setup geometry and tools for one job.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SceneDef {
    #[serde(default)]
    pub tools: Vec<ToolDef>,
    #[serde(default)]
    pub bodies: Vec<BodyDef>,
}

impl SceneDef {
    pub fn from_toml(src: &str) -> Result<SceneDef, String> {
        toml::from_str(src).map_err(|e| e.to_string())
    }
}

/// One cylinder of a tool, in the tool frame (tip at origin, +Z up).
#[derive(Clone, Copy, Debug)]
struct Cyl {
    z0: f64,
    height: f64,
    radius: f64,
    kind: SegmentKind,
}

#[derive(Clone, Debug)]
pub struct ToolAssembly {
    pub def: ToolDef,
    cyls: Vec<Cyl>,
}

impl ToolAssembly {
    pub fn new(def: ToolDef) -> Result<ToolAssembly, String> {
        let mut cyls = Vec::new();
        let mut z = 0.0;
        for (i, s) in def.segments.iter().enumerate() {
            if s.length <= 0.0 {
                return Err(format!(
                    "tool T{} segment {i}: length must be positive",
                    def.number
                ));
            }
            let (d0, d1) = match (s.diameter, s.diameter_bottom, s.diameter_top) {
                (Some(d), _, _) => (d, d),
                (None, Some(a), Some(b)) => (a, b),
                _ => {
                    return Err(format!(
                        "tool T{} segment {i}: give diameter, or diameter_bottom and diameter_top",
                        def.number
                    ))
                }
            };
            // Split cones into steps of at most 0.5 mm radius change.
            let steps = (((d1 - d0).abs() / 2.0 / 0.5).ceil() as usize).clamp(1, 32);
            let h = s.length / steps as f64;
            for k in 0..steps {
                let ra = (d0 + (d1 - d0) * k as f64 / steps as f64) / 2.0;
                let rb = (d0 + (d1 - d0) * (k + 1) as f64 / steps as f64) / 2.0;
                cyls.push(Cyl {
                    z0: z + h * k as f64,
                    height: h,
                    radius: ra.max(rb),
                    kind: s.kind,
                });
            }
            z += s.length;
        }
        Ok(ToolAssembly { def, cyls })
    }

    /// Cutter diameter (largest cutter section).
    pub fn cutter_diameter(&self) -> f64 {
        self.cyls
            .iter()
            .filter(|c| c.kind == SegmentKind::Cutter)
            .map(|c| c.radius * 2.0)
            .fold(0.0, f64::max)
    }
}

#[derive(Clone, Debug)]
pub struct Body {
    pub name: String,
    pub role: Role,
    link: Link,
    group: Group,
    mesh: Mesh,
    bvh: Bvh,
    collide: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Group {
    Base,
    Head,
    Table,
}

/// A detected contact.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Contact {
    /// "T1 holder", "T1 cutter", or a body name.
    pub a: String,
    pub b: String,
    pub role: Role,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoveClass {
    Rapid,
    Feed,
}

#[derive(Clone, Debug, Default)]
pub struct World {
    bodies: Vec<Body>,
    tools: BTreeMap<u32, ToolAssembly>,
}

fn load_shape(shape: &Shape, dir: Option<&Path>) -> Result<Mesh, String> {
    Ok(match shape {
        Shape::Box { min, max } => Mesh::cuboid(*min, *max),
        Shape::Cylinder {
            base,
            axis,
            radius,
            height,
        } => {
            let axis = axis.normalized().ok_or("cylinder axis is zero")?;
            Mesh::cylinder(*base, axis, *radius, *height, 48)
        }
        Shape::Stl { path } => {
            let p: PathBuf = match dir {
                Some(d) => d.join(path),
                None => PathBuf::from(path),
            };
            let bytes = std::fs::read(&p).map_err(|e| format!("{}: {e}", p.display()))?;
            Mesh::from_stl(&bytes).map_err(|e| format!("{}: {e}", p.display()))?
        }
    })
}

impl World {
    /// Build from machine components (relative to `machine_dir`) and a job
    /// scene (relative to `scene_dir`).
    pub fn build(
        kin: &Kinematics,
        scene: &SceneDef,
        scene_dir: Option<&Path>,
        machine_dir: Option<&Path>,
    ) -> Result<World, String> {
        let def = kin.machine();
        let mut w = World::default();
        let table_end = kin.link("table").expect("table link always exists");
        let all = def
            .components
            .iter()
            .map(|b| (b, machine_dir))
            .chain(scene.bodies.iter().map(|b| (b, scene_dir)));
        for (b, dir) in all {
            let mut mesh =
                load_shape(&b.shape, dir).map_err(|e| format!("body '{}': {e}", b.name))?;
            let link = match b.role {
                Role::Machine => {
                    let name = b.attach.as_deref().unwrap_or("base");
                    kin.link(name)
                        .ok_or_else(|| format!("body '{}': unknown link '{name}'", b.name))?
                }
                _ => {
                    let wcs = b.wcs.as_deref().unwrap_or("G54");
                    let origin = def.work_offset(wcs).ok_or_else(|| {
                        format!("body '{}': work offset {wcs} is not defined", b.name)
                    })?;
                    mesh = mesh.transformed(&Iso::translation(origin));
                    table_end
                }
            };
            let group = match link {
                Link::Base => Group::Base,
                Link::Head(0) | Link::Table(0) => Group::Base,
                Link::Head(_) | Link::Tool => Group::Head,
                Link::Table(_) | Link::Part => Group::Table,
            };
            let bvh = Bvh::build(&mesh);
            w.bodies.push(Body {
                name: b.name.clone(),
                role: b.role,
                link,
                group,
                mesh,
                bvh,
                collide: b.collide,
            });
        }
        for t in &scene.tools {
            w.tools.insert(t.number, ToolAssembly::new(t.clone())?);
        }
        Ok(w)
    }

    pub fn tool(&self, number: u32) -> Option<&ToolAssembly> {
        self.tools.get(&number)
    }

    pub fn bodies(&self) -> impl Iterator<Item = (&str, Role)> {
        self.bodies.iter().map(|b| (b.name.as_str(), b.role))
    }

    /// All contacts at joints `q`. `margin` inflates the tool (mm).
    pub fn check(
        &self,
        kin: &Kinematics,
        q: &[f64],
        tool: Option<u32>,
        class: MoveClass,
        margin: f64,
    ) -> Vec<Contact> {
        let mut out = Vec::new();
        let placements: Vec<Iso> = self
            .bodies
            .iter()
            .map(|b| kin.link_iso(b.link, q))
            .collect();

        if let Some(asm) = tool.and_then(|t| self.tools.get(&t)) {
            let tool_iso = kin.link_iso(Link::Tool, q);
            for (b, place) in self.bodies.iter().zip(&placements) {
                if !b.collide || b.role == Role::Stock || b.group == Group::Head {
                    continue;
                }
                let to_local = place.inverse().then(&tool_iso);
                for kind in [SegmentKind::Holder, SegmentKind::Shank, SegmentKind::Cutter] {
                    if kind == SegmentKind::Cutter
                        && b.role == Role::Part
                        && class == MoveClass::Feed
                    {
                        continue; // cutting
                    }
                    let hit = asm
                        .cyls
                        .iter()
                        .filter(|c| c.kind == kind)
                        .any(|c| cylinder_hits_body(b, &to_local, c, margin));
                    if hit {
                        out.push(Contact {
                            a: format!("T{} {}", asm.def.number, kind_name(kind)),
                            b: b.name.clone(),
                            role: b.role,
                        });
                        break; // one report per body
                    }
                }
            }
        }

        // Body vs body across moving groups.
        for i in 0..self.bodies.len() {
            for j in i + 1..self.bodies.len() {
                let (a, b) = (&self.bodies[i], &self.bodies[j]);
                if !a.collide || !b.collide || a.group == b.group {
                    continue;
                }
                if a.role != Role::Machine && b.role != Role::Machine {
                    continue; // part/fixture never move relative to each other
                }
                if matches!(a.role, Role::Stock) || matches!(b.role, Role::Stock) {
                    continue;
                }
                let b_to_a = placements[i].inverse().then(&placements[j]);
                let a_to_b = b_to_a.inverse();
                let nested = || {
                    b.mesh
                        .vertices
                        .first()
                        .is_some_and(|&v| a.bvh.contains(&a.mesh, b_to_a.apply_point(v)))
                        || a.mesh
                            .vertices
                            .first()
                            .is_some_and(|&v| b.bvh.contains(&b.mesh, a_to_b.apply_point(v)))
                };
                if Bvh::meshes_intersect((&a.mesh, &a.bvh), (&b.mesh, &b.bvh), &b_to_a) || nested()
                {
                    out.push(Contact {
                        a: a.name.clone(),
                        b: b.name.clone(),
                        role: b.role,
                    });
                }
            }
        }
        out
    }

    /// Largest margin (up to `cap` mm) by which the tool clears everything,
    /// or 0 when it is already in contact.
    pub fn clearance(&self, kin: &Kinematics, q: &[f64], tool: u32, cap: f64) -> f64 {
        let free = |m: f64| self.check_tool_only(kin, q, tool, m).is_empty();
        if !free(0.0) {
            return 0.0;
        }
        if free(cap) {
            return cap;
        }
        let (mut lo, mut hi) = (0.0, cap);
        for _ in 0..14 {
            let mid = 0.5 * (lo + hi);
            if free(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo
    }

    /// Tool contacts only (feed move semantics), used for planning queries.
    pub fn check_tool_only(
        &self,
        kin: &Kinematics,
        q: &[f64],
        tool: u32,
        margin: f64,
    ) -> Vec<Contact> {
        let mut c = self.check(kin, q, Some(tool), MoveClass::Feed, margin);
        c.retain(|c| c.a.starts_with('T'));
        c
    }
}

fn kind_name(k: SegmentKind) -> &'static str {
    match k {
        SegmentKind::Cutter => "cutter",
        SegmentKind::Shank => "shank",
        SegmentKind::Holder => "holder",
    }
}

/// Cylinder `c` (in the tool frame, placed by `tool_to_body`) vs body mesh.
fn cylinder_hits_body(b: &Body, tool_to_body: &Iso, c: &Cyl, margin: f64) -> bool {
    let r = c.radius + margin;
    let z0 = c.z0 - margin;
    let h = c.height + 2.0 * margin;
    let cyl_to_body = tool_to_body.then(&Iso::translation(Vec3::new(0.0, 0.0, z0)));
    let local_box = Aabb {
        min: Vec3::new(-r, -r, 0.0),
        max: Vec3::new(r, r, h),
    };
    let query = local_box.transformed(&cyl_to_body);
    let body_to_cyl = cyl_to_body.inverse();
    let crosses = b.bvh.any_in(&query, |t| {
        let tri = b.mesh.tri(t).map(|p| body_to_cyl.apply_point(p));
        cylinder_hits_triangle(&tri, r, h)
    });
    // No surface crossing: the cylinder is either clear or wholly inside.
    crosses
        || b.bvh.contains(
            &b.mesh,
            cyl_to_body.apply_point(Vec3::new(0.0, 0.0, h * 0.5)),
        )
}
