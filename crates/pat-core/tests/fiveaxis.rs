//! Simultaneous 5-axis finishing: surface finishing with tool-axis control
//! and swarf, verified by posting and replaying on the trunnion with
//! collision and gouge checks.

mod common;

use common::*;
use pat_core::collision::{SceneDef, World};
use pat_core::fiveaxis::{
    surface_finish, swarf, AxisControl, AxisOptions, Side, SurfaceParams, SwarfParams,
};
use pat_core::geom::Mesh;
use pat_core::math::Vec3;
use pat_core::post::{post, ClMove, ClProgram, ClTool, Dialect, PostOptions};
use pat_core::sim::{simulate, Report, SimOptions};
use pat_core::stock::CutterShape;
use pat_core::{Kinematics, MachineDef};
use std::sync::Arc;

const TOOLS: &str = r#"
[[tools]]
number = 1
gauge_length = 90.0
segments = [
  { kind = "cutter", length = 15.0, diameter = 6.0, corner_radius = 3.0 },
  { kind = "shank", length = 25.0, diameter = 6.0 },
  { kind = "holder", length = 40.0, diameter = 24.0 },
]
[[tools]]
number = 2
gauge_length = 100.0
segments = [
  { kind = "cutter", length = 30.0, diameter = 10.0 },
  { kind = "shank", length = 10.0, diameter = 10.0 },
  { kind = "holder", length = 40.0, diameter = 30.0 },
]
"#;

const BUMP: &str = r#"
[[bodies]]
name = "base"
role = "part"
shape = { type = "box", min = [-35.0, -25.0, -20.0], max = [35.0, 25.0, 0.0] }
[[bodies]]
name = "bump"
role = "part"
shape = { type = "cylinder", base = [-30.0, 0.0, 0.0], axis = [1.0, 0.0, 0.0], radius = 15.0, height = 60.0 }
[[bodies]]
name = "skin base"
role = "stock"
shape = { type = "box", min = [-35.5, -25.5, -20.0], max = [35.5, 25.5, 0.5] }
[[bodies]]
name = "skin bump"
role = "stock"
shape = { type = "cylinder", base = [-30.0, 0.0, 0.0], axis = [1.0, 0.0, 0.0], radius = 15.5, height = 60.0 }
"#;

fn bump_parts(o: Vec3) -> Vec<Mesh> {
    vec![
        Mesh::cuboid(
            Vec3::new(-35.0, -25.0, -20.0) + o,
            Vec3::new(35.0, 25.0, 0.0) + o,
        ),
        Mesh::cylinder(Vec3::new(-30.0, 0.0, 0.0) + o, Vec3::X, 15.0, 60.0, 48),
    ]
}

fn world_for(m: &MachineDef, extra: &str, len: f64) -> (Kinematics, World) {
    let kin = Kinematics::new(m.clone(), len).unwrap();
    let scene = SceneDef::from_toml(&format!("{TOOLS}\n{BUMP}\n{extra}")).unwrap();
    let world = World::build(&kin, &scene, None, None).unwrap();
    (kin, world)
}

fn replay(m: &MachineDef, extra: &str, tool: u32, len: f64, moves: Vec<ClMove>) -> Report {
    let cl = ClProgram {
        name: "5x".into(),
        tool: ClTool {
            number: tool,
            length: len,
            diameter: None,
        },
        spindle_rpm: Some(15000.0),
        coolant: false,
        moves,
    };
    let out = post(
        m,
        &cl,
        &PostOptions {
            dialect: Dialect::FanucTcp,
            ..Default::default()
        },
    )
    .unwrap();
    let (_, world) = world_for(m, extra, len);
    let opts = SimOptions {
        world: Some(Arc::new(world)),
        simulate_stock: true,
        stock_resolution: 0.5,
        gouge_tolerance: 0.1,
        ..Default::default()
    };
    simulate(m, &out.gcode, &opts)
}

fn params() -> SurfaceParams {
    SurfaceParams {
        cutter: CutterShape::ball(6.0, 15.0),
        stepover: 1.5,
        step: 1.0,
        angle_deg: 0.0, // passes along the bump: no mid-pass rotary flips on this trunnion
        axis: AxisOptions {
            control: AxisControl::Normal {
                lead_deg: 10.0,
                tilt_deg: 0.0,
            },
            ..Default::default()
        },
        ..Default::default()
    }
}

#[test]
fn normal_axis_finishing_over_a_bump() {
    let m = machine(TRUNNION_AC);
    let o = m.wcs_origin;
    let parts = bump_parts(o);
    let refs: Vec<&Mesh> = parts.iter().collect();
    let (kin, world) = world_for(&m, "", 90.0);
    let r = surface_finish(&refs, &kin, Some(&world), Some(1), o, &params());
    assert_eq!(r.skipped, 0, "{}", r.summary);

    // Over the bump the ball centre sits exactly r + 15 from its axis line.
    let mut checked = 0;
    let mut axes = Vec::new();
    for mv in &r.moves {
        if let ClMove::Feed { tip, axis, .. } = mv {
            let c = *tip + *axis * 3.0;
            axes.push(*axis);
            if c.x.abs() < 25.0 && c.z > 8.0 {
                let d = (c.y * c.y + c.z * c.z).sqrt();
                assert!(
                    (d - 18.0).abs() < 0.05,
                    "centre {c:?} is {d} from the bump axis"
                );
                checked += 1;
            }
        }
    }
    assert!(checked > 100);
    // Simultaneous 5-axis: the axis follows the surface over a wide range.
    let spread = axes
        .iter()
        .map(|a| a.angle_to(Vec3::Z).to_degrees())
        .fold(0.0, f64::max);
    assert!(spread > 60.0, "axis only tilts {spread:.1}°");

    let rep = replay(&m, "", 1, 90.0, r.moves);
    assert!(
        rep.ok,
        "{:#?}",
        rep.issues.iter().take(8).collect::<Vec<_>>()
    );
}

#[test]
fn axis_avoids_a_clamp() {
    let m = machine(TRUNNION_AC);
    let o = m.wcs_origin;
    let parts = bump_parts(o);
    let refs: Vec<&Mesh> = parts.iter().collect();
    let clamp = r#"
[[bodies]]
name = "tall clamp"
role = "fixture"
shape = { type = "box", min = [-10.0, 27.0, 0.0], max = [10.0, 35.0, 60.0] }
"#;
    let (kin, world) = world_for(&m, clamp, 90.0);
    let p = SurfaceParams {
        region: Some([-12.0, -26.0, 12.0, 26.0]),
        ..params()
    };
    let r = surface_finish(&refs, &kin, Some(&world), Some(1), o, &p);
    assert!(
        r.adjusted > 0,
        "the clamp should force some tilting: {}",
        r.summary
    );
    let rep = replay(&m, clamp, 1, 90.0, r.moves);
    assert!(
        rep.ok,
        "{:#?}",
        rep.issues.iter().take(8).collect::<Vec<_>>()
    );
}

#[test]
fn point_and_line_axis_controls() {
    let m = machine(TRUNNION_AC);
    let o = m.wcs_origin;
    let parts = bump_parts(o);
    let refs: Vec<&Mesh> = parts.iter().collect();
    let kin = Kinematics::new(m.clone(), 90.0).unwrap();
    let focus = Vec3::new(0.0, 0.0, 80.0);
    let p = SurfaceParams {
        region: Some([-5.0, -10.0, 5.0, 10.0]),
        axis: AxisOptions {
            control: AxisControl::TowardPoint { point: focus },
            avoid: false,
            smooth: 0,
            singularity_lock_deg: 0.0,
            ..Default::default()
        },
        ..params()
    };
    let r = surface_finish(&refs, &kin, None, None, o, &p);
    let mut n = 0;
    for mv in &r.moves {
        if let ClMove::Feed { tip, axis, .. } = mv {
            let c = *tip + *axis * 3.0;
            let to_focus = (focus - c).normalized().unwrap();
            assert!(
                axis.angle_to(to_focus) < 1e-9,
                "axis should aim at the focus point"
            );
            n += 1;
        }
    }
    assert!(n > 20);
    // From a line along X below the bump: axes are radial to the bump.
    let p = SurfaceParams {
        axis: AxisOptions {
            control: AxisControl::FromLine {
                point: Vec3::ZERO,
                direction: Vec3::X,
            },
            avoid: false,
            smooth: 0,
            singularity_lock_deg: 0.0,
            ..Default::default()
        },
        ..p
    };
    let r = surface_finish(&refs, &kin, None, None, o, &p);
    for mv in &r.moves {
        if let ClMove::Feed { tip, axis, .. } = mv {
            let c = *tip + *axis * 3.0;
            assert!(
                axis.x.abs() < 1e-9
                    && (Vec3::new(0.0, c.y, c.z)
                        .normalized()
                        .unwrap()
                        .angle_to(*axis))
                        < 1e-9
            );
        }
    }
}

/// Convex prism along X from a YZ polygon.
fn prism(poly: &[(f64, f64)], x0: f64, x1: f64, o: Vec3) -> Mesh {
    let mut v = Vec::new();
    for &(y, z) in poly {
        v.push(Vec3::new(x0, y, z) + o);
    }
    for &(y, z) in poly {
        v.push(Vec3::new(x1, y, z) + o);
    }
    let n = poly.len() as u32;
    let mut t = Vec::new();
    for i in 1..n - 1 {
        t.push([0, i + 1, i]);
        t.push([n, n + i, n + i + 1]);
    }
    for i in 0..n {
        let j = (i + 1) % n;
        t.push([i, j, n + j]);
        t.push([i, n + j, n + i]);
    }
    Mesh {
        vertices: v,
        triangles: t,
    }
}

#[test]
fn swarf_on_a_sloped_wall() {
    let m = machine(TRUNNION_AC);
    let o = m.wcs_origin;
    // Wall from (y 0, z -15) up to (y -8, z 0): the part is on the -Y side.
    let wedge = r#"
[[bodies]]
name = "wedge"
role = "part"
shape = { type = "stl", path = "__wedge__" }
"#;
    let _ = wedge;
    let part = prism(
        &[(-30.0, -15.0), (0.0, -15.0), (-8.0, 0.0), (-30.0, 0.0)],
        -20.0,
        20.0,
        o,
    );
    let bottom = vec![Vec3::new(-20.0, 0.0, -15.0), Vec3::new(20.0, 0.0, -15.0)];
    let top = vec![Vec3::new(-20.0, -8.0, 0.0), Vec3::new(20.0, -8.0, 0.0)];
    let kin = Kinematics::new(m.clone(), 100.0).unwrap();
    let p = SwarfParams {
        cutter: CutterShape::flat(10.0, 30.0),
        side: Side::Right,
        ..Default::default()
    };
    let r = swarf(&bottom, &top, &kin, None, Some(2), o, &p);
    assert!(r.max_deviation < 1e-9, "planar wall: {}", r.max_deviation);
    assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    // Axis along the ruling, cutter tangent to the wall (centre line 5 mm off).
    let ruling = Vec3::new(0.0, -8.0, 15.0).normalized().unwrap();
    let wall_n = Vec3::new(0.0, 15.0, 8.0).normalized().unwrap(); // outward, +Y side
    for mv in &r.moves {
        if let ClMove::Feed { tip, axis, .. } = mv {
            if tip.x.abs() <= 20.0 + 1e-9 && (tip.y - 5.0 * wall_n.y).abs() < 1.0 {
                assert!(axis.angle_to(ruling) < 1e-9);
                let d = (*tip - Vec3::new(tip.x, 0.0, -15.0)).dot(wall_n);
                assert!((d - 5.0).abs() < 1e-9, "offset {d}");
            }
        }
    }
    // The cutter never enters the part.
    let core = pat_core::stock::Voxels::from_mesh(
        &part,
        &pat_core::geom::Bvh::build(&part),
        part.aabb().expand(0.5),
        0.5,
    )
    .eroded(1);
    let mut cut = core.clone();
    for w in r.moves.windows(2) {
        if let ClMove::Feed { .. } = w[1] {
            let a = pat_core::Pose {
                tip: w[0].pose().tip + o,
                axis: w[0].pose().axis,
            };
            let b = pat_core::Pose {
                tip: w[1].pose().tip + o,
                axis: w[1].pose().axis,
            };
            cut.sweep(&a, &b, &CutterShape::flat(10.0, 30.0));
        }
    }
    assert_eq!(cut.count(), core.count(), "swarf cut into the part");
}

#[test]
fn twisted_wall_reports_deviation() {
    let m = machine(TRUNNION_AC);
    let kin = Kinematics::new(m.clone(), 100.0).unwrap();
    // Top rail rotated relative to the bottom one: a hyperbolic paraboloid.
    let bottom = vec![Vec3::new(-20.0, 0.0, -15.0), Vec3::new(20.0, 0.0, -15.0)];
    let top = vec![Vec3::new(-20.0, -10.0, 0.0), Vec3::new(20.0, 5.0, 0.0)];
    let r = swarf(
        &bottom,
        &top,
        &kin,
        None,
        None,
        m.wcs_origin,
        &SwarfParams {
            side: Side::Right,
            step: 5.0,
            ..Default::default()
        },
    );
    assert!(r.max_deviation > 0.001, "{}", r.max_deviation);
    assert!(r.warnings.iter().any(|w| w.contains("twisted")) || r.max_deviation <= 0.01);
}
