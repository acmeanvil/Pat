//! Continuous 4-axis: roughing, finishing and wrapping, verified by posting
//! and replaying on the 4th-axis VMC with collision and stock simulation.

mod common;

use common::*;
use pat_core::collision::{SceneDef, World};
use pat_core::geom::Mesh;
use pat_core::math::Vec3;
use pat_core::post::{post, ClMove, ClProgram, ClTool, Dialect, PostOptions};
use pat_core::rotary::{rotary_finish, rotary_rough, unwrap, wrap, RotaryFrame, RotaryParams};
use pat_core::sim::{simulate, Report, SimOptions};
use pat_core::stock::{CutterShape, Voxels};
use pat_core::{Kinematics, MachineDef};
use std::sync::Arc;

const SCENE: &str = r#"
[[tools]]
number = 1
gauge_length = 90.0
segments = [
  { kind = "cutter", length = 25.0, diameter = 6.0 },
  { kind = "shank", length = 20.0, diameter = 6.0 },
  { kind = "holder", length = 40.0, diameter = 30.0 },
]
[[bodies]]
name = "round stock"
role = "stock"
shape = { type = "cylinder", base = [-25.0, 0.0, 0.0], axis = [1.0, 0.0, 0.0], radius = 14.0, height = 50.0 }
[[bodies]]
name = "square bar"
role = "part"
shape = { type = "box", min = [-20.0, -8.0, -8.0], max = [20.0, 8.0, 8.0] }
"#;

fn setup() -> (MachineDef, Mesh, Voxels, RotaryFrame) {
    let m = machine(VMC_4TH_A);
    let o = m.wcs_origin;
    let part = Mesh::cuboid(
        Vec3::new(-20.0, -8.0, -8.0) + o,
        Vec3::new(20.0, 8.0, 8.0) + o,
    );
    let cyl = Mesh::cylinder(Vec3::new(-25.0, 0.0, 0.0) + o, Vec3::X, 14.0, 50.0, 96);
    let stock = Voxels::from_mesh(
        &cyl,
        &pat_core::geom::Bvh::build(&cyl),
        cyl.aabb().expand(0.5),
        0.5,
    );
    let f = RotaryFrame::from_machine(&m).expect("one table rotary");
    (m, part, stock, f)
}

fn params() -> RotaryParams {
    RotaryParams {
        cutter: CutterShape::flat(6.0, 25.0),
        stepdown: 2.0,
        stepover: 2.5,
        leave: 0.3,
        ..Default::default()
    }
}

fn replay(m: &MachineDef, moves: Vec<ClMove>) -> Report {
    let cl = ClProgram {
        name: "rotary".into(),
        tool: ClTool {
            number: 1,
            length: 90.0,
            diameter: Some(6.0),
        },
        spindle_rpm: Some(8000.0),
        coolant: false,
        moves,
    };
    let out = post(
        m,
        &cl,
        &PostOptions {
            dialect: Dialect::LinuxcncJoint,
            ..Default::default()
        },
    )
    .unwrap();
    let kin = Kinematics::new(m.clone(), 90.0).unwrap();
    let world = World::build(&kin, &SceneDef::from_toml(SCENE).unwrap(), None, None).unwrap();
    let opts = SimOptions {
        world: Some(Arc::new(world)),
        simulate_stock: true,
        stock_resolution: 0.5,
        gouge_tolerance: 0.1,
        ..Default::default()
    };
    simulate(m, &out.gcode, &opts)
}

#[test]
fn rotary_roughing_a_square_bar_from_round_stock() {
    let (m, part, mut stock, f) = setup();
    let initial = stock.volume();
    let r = rotary_rough(
        &[&part],
        &[(25.0, 3.0), (45.0, 15.0)],
        &mut stock,
        &f,
        m.wcs_origin,
        &params(),
    );
    assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    assert!(r.levels.len() >= 2, "{:?}", r.levels);
    // The flats of the bar sit at radius 8 (+0.3 leave): a level lands there.
    assert!(
        r.levels.iter().any(|&l| (l - 8.3).abs() < 0.3),
        "{:?}",
        r.levels
    );
    // Most of the waste is gone, the bar is intact.
    let part_vol = 40.0 * 16.0 * 16.0;
    assert!(
        r.remaining_mm3 > part_vol - 400.0,
        "{} left",
        r.remaining_mm3
    );
    assert!(
        r.removed_mm3 > 0.5 * (initial - part_vol),
        "removed {} of {}",
        r.removed_mm3,
        initial - part_vol
    );
    // Continuous 4-axis: the A axis keeps turning.
    let unwound = unwrap(&r.moves, &f, m.wcs_origin, 10.0);
    let span = unwound
        .iter()
        .map(|p| p[1])
        .fold(f64::NEG_INFINITY, f64::max)
        / 10.0;
    assert!(
        span > 10.0 * std::f64::consts::TAU,
        "only {:.1} turns",
        span / std::f64::consts::TAU
    );

    let rep = replay(&m, r.moves);
    assert!(
        rep.ok,
        "{:#?}",
        rep.issues.iter().take(8).collect::<Vec<_>>()
    );
}

#[test]
fn rotary_finishing_follows_the_round_part() {
    let m = machine(VMC_4TH_A);
    let o = m.wcs_origin;
    let bar = Mesh::cylinder(Vec3::new(-15.0, 0.0, 0.0) + o, Vec3::X, 10.0, 30.0, 180);
    let f = RotaryFrame::from_machine(&m).unwrap();
    let p = RotaryParams {
        cutter: CutterShape::ball(6.0, 20.0),
        stepover: 0.5,
        leave: 0.0,
        ..Default::default()
    };
    let r = rotary_finish(&[&bar], &[], &f, o, &p);
    let feeds: Vec<Vec3> = r
        .moves
        .iter()
        .filter_map(|mv| match mv {
            ClMove::Feed { tip, .. } => Some(*tip + o),
            _ => None,
        })
        .collect();
    assert!(feeds.len() > 1000);
    // Over the bar's length the ball tip rides on the surface (within the
    // conservative raster margin), never inside it.
    for t in feeds.iter().filter(|t| (t.x - o.x).abs() < 12.0) {
        let (_, _, rad) = f.cyl(*t);
        assert!(
            (10.0 - 1e-6..=10.0 + 0.2).contains(&rad),
            "tip radius {rad}"
        );
    }
    // Every move is reachable on the 4th-axis machine.
    let kin = Kinematics::new(m, 90.0).unwrap();
    let mut prev: Option<Vec<f64>> = None;
    for mv in &r.moves {
        prev = Some(
            kin.inverse(&mv.pose(), prev.as_deref())
                .expect("reachable")
                .joints,
        );
    }
}

#[test]
fn wrap_and_unwrap_round_trip() {
    let m = machine(VMC_4TH_A);
    let o = m.wcs_origin;
    let f = RotaryFrame::from_machine(&m).unwrap();
    // A 10 x 20 rectangle (u along the axis, v around the circumference).
    let rect = vec![vec![
        [0.0, 0.0],
        [10.0, 0.0],
        [10.0, 20.0],
        [0.0, 20.0],
        [0.0, 0.0],
    ]];
    let moves = wrap(&rect, &f, o, 12.0, 0.5, 0.5, 600.0, 3.0);
    let flat = unwrap(&moves, &f, o, 12.0);
    let cuts: Vec<&[f64; 3]> = moves
        .iter()
        .zip(&flat)
        .filter(|(m, _)| matches!(m, ClMove::Feed { .. }))
        .map(|(_, p)| p)
        .collect();
    for p in &cuts {
        assert!((p[2] - 11.5).abs() < 1e-9, "depth: r = {}", p[2]);
        let on_edge = p[0].abs() < 1e-6
            || (p[0] - 10.0).abs() < 1e-6
            || p[1].abs() < 1e-6
            || (p[1] - 20.0).abs() < 1e-6;
        assert!(on_edge, "{p:?} is off the rectangle");
    }
    // 20 mm of arc on a 12 mm radius is 95.5 degrees of rotation.
    let max_v = cuts.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max);
    assert!((max_v - 20.0).abs() < 1e-6);
}

#[test]
fn indexed_four_axis_roughing_via_automatic_orientation() {
    use pat_core::plan::{auto_rough, AutoRoughParams};
    use pat_core::rough::RoughParams;
    let (m, part, mut stock, _) = setup();
    let kin = Kinematics::new(m.clone(), 90.0).unwrap();
    let world = World::build(&kin, &SceneDef::from_toml(SCENE).unwrap(), None, None).unwrap();
    let mut ap = AutoRoughParams {
        rough: RoughParams {
            cutter: CutterShape::flat(6.0, 25.0),
            stepdown: 2.0,
            stepover: 1.0,
            leave: 0.3,
            resolution: 0.5,
            ..Default::default()
        },
        ..Default::default()
    };
    ap.orient.tool = Some(1);
    let r = auto_rough(&[&part], &mut stock, &kin, Some(&world), m.wcs_origin, &ap);
    // Only radial directions are reachable on a 4th axis. With a 25 mm
    // flute, top and bottom index positions reach everything around the bar.
    assert!(r.setups.len() >= 2, "{}", r.summary);
    for s in &r.setups {
        assert!(
            s.orientation.axis.x.abs() < 1e-6,
            "{:?} is not radial",
            s.orientation.axis
        );
    }
    let o = m.wcs_origin;
    for p in [
        Vec3::new(0.0, 11.0, 0.0),
        Vec3::new(5.0, 0.0, -11.0),
        Vec3::new(-5.0, 0.0, 11.0),
    ] {
        assert!(!stock.material_at(p + o), "material left at {p:?}");
    }
    assert!(
        stock.material_at(Vec3::new(0.0, 7.0, 7.0) + o),
        "bar was cut"
    );
    let rep = replay(&m, r.moves);
    assert!(
        rep.ok,
        "{:#?}",
        rep.issues.iter().take(8).collect::<Vec<_>>()
    );
}
