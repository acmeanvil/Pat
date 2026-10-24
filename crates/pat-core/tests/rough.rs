//! End-to-end roughing: generate adaptive toolpaths, post them, then replay
//! the G-code with collision and material-removal checks.

mod common;

use common::*;
use pat_core::collision::{SceneDef, World};
use pat_core::geom::{Bvh, Mesh};
use pat_core::math::{Iso, Vec3};
use pat_core::post::{post, ClProgram, ClTool, Dialect, PostOptions};
use pat_core::rough::{machining_frame, rough, RoughParams};
use pat_core::sim::{simulate, SimOptions};
use pat_core::stock::{CutterShape, Voxels};
use pat_core::{Kinematics, MachineDef};
use std::sync::Arc;

/// 60 x 40 x 20 block with a 30 x 20 x 10 pocket, as non-overlapping boxes
/// (WCS coordinates; part zero at the top centre).
fn pocket_part() -> Vec<Mesh> {
    let b = |a: [f64; 3], c: [f64; 3]| Mesh::cuboid(a.into(), c.into());
    vec![
        b([-30.0, -20.0, -20.0], [30.0, 20.0, -10.0]),
        b([-30.0, -20.0, -10.0], [-15.0, 20.0, 0.0]),
        b([15.0, -20.0, -10.0], [30.0, 20.0, 0.0]),
        b([-15.0, -20.0, -10.0], [15.0, -10.0, 0.0]),
        b([-15.0, 10.0, -10.0], [15.0, 20.0, 0.0]),
    ]
}

const PART_VOLUME: f64 = 60.0 * 40.0 * 20.0 - 30.0 * 20.0 * 10.0;
const STOCK_MIN: [f64; 3] = [-32.0, -22.0, -20.0];
const STOCK_MAX: [f64; 3] = [32.0, 22.0, 3.0];

fn scene_toml(tool_d: f64) -> String {
    let mut s = format!(
        r#"
[[tools]]
number = 1
gauge_length = 90.0
segments = [
  {{ kind = "cutter", length = 25.0, diameter = {tool_d} }},
  {{ kind = "shank", length = 20.0, diameter = {tool_d} }},
  {{ kind = "holder", length = 40.0, diameter = 30.0 }},
]

[[bodies]]
name = "stock"
role = "stock"
shape = {{ type = "box", min = {STOCK_MIN:?}, max = {STOCK_MAX:?} }}
"#
    );
    for (i, m) in pocket_part().iter().enumerate() {
        let bb = m.aabb();
        s.push_str(&format!(
            "\n[[bodies]]\nname = \"part {i}\"\nrole = \"part\"\nshape = {{ type = \"box\", min = [{}, {}, {}], max = [{}, {}, {}] }}\n",
            bb.min.x, bb.min.y, bb.min.z, bb.max.x, bb.max.y, bb.max.z
        ));
    }
    s
}

/// Part meshes and stock voxels in the table frame for machine `m`.
fn setup(m: &MachineDef, res: f64) -> (Vec<Mesh>, Voxels) {
    let o = m.wcs_origin;
    let parts: Vec<Mesh> = pocket_part()
        .iter()
        .map(|p| p.transformed(&Iso::translation(o)))
        .collect();
    let stock = Voxels::from_box(Vec3::from(STOCK_MIN) + o, Vec3::from(STOCK_MAX) + o, res);
    (parts, stock)
}

fn params(d: f64) -> RoughParams {
    RoughParams {
        cutter: CutterShape::flat(d, 25.0),
        stepdown: 3.0,
        stepover: d * 0.15,
        leave: 0.3,
        resolution: 0.5,
        ..Default::default()
    }
}

fn replay(
    m: &MachineDef,
    moves: Vec<pat_core::post::ClMove>,
    tool_d: f64,
) -> pat_core::sim::Report {
    let cl = ClProgram {
        name: "rough".into(),
        tool: ClTool {
            number: 1,
            length: 90.0,
            diameter: Some(tool_d),
        },
        spindle_rpm: Some(10000.0),
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
    let scene = SceneDef::from_toml(&scene_toml(tool_d)).unwrap();
    let kin = Kinematics::new(m.clone(), 90.0).unwrap();
    let world = World::build(&kin, &scene, None, None).unwrap();
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
fn roughs_a_pocket_cleanly() {
    let m = machine(VMC_3AXIS);
    let (parts, mut stock) = setup(&m, 0.5);
    let refs: Vec<&Mesh> = parts.iter().collect();
    let f = machining_frame(m.wcs_origin, Vec3::Z);
    let r = rough(&refs, &mut stock, &f, m.wcs_origin, &params(6.0));
    assert!(r.warnings.is_empty(), "{:?}", r.warnings);
    // The pocket floor (z = -10 + 0.3 leave) is roughed as its own level.
    assert!(
        r.levels.iter().any(|l| (l.z - (-9.7)).abs() < 0.05),
        "{:?}",
        r.levels.iter().map(|l| l.z).collect::<Vec<_>>()
    );
    assert!(
        r.levels.iter().all(|l| l.uncut_area_mm2 < 2.0),
        "{:?}",
        r.levels
    );

    // What is left: the part plus a thin skin (leave + cusps + voxel error).
    let part_vol = PART_VOLUME;
    assert!(
        r.remaining_mm3 > part_vol - 500.0,
        "remaining {}",
        r.remaining_mm3
    );
    assert!(
        r.remaining_mm3 < part_vol + 8000.0,
        "remaining {}",
        r.remaining_mm3
    );

    // Replaying the posted G-code: no gouges, rapids into stock, rubbing or crashes.
    let rep = replay(&m, r.moves, 6.0);
    assert!(
        rep.ok,
        "{:#?}",
        rep.issues.iter().take(10).collect::<Vec<_>>()
    );
    let removed = rep.stock.as_ref().unwrap().removed_mm3;
    assert!(
        (removed - r.removed_mm3).abs() / r.removed_mm3 < 0.05,
        "{removed} vs {}",
        r.removed_mm3
    );
}

#[test]
fn rest_machining_only_cuts_what_is_left() {
    let m = machine(VMC_3AXIS);
    let o = m.wcs_origin;
    let f = machining_frame(o, Vec3::Z);

    // Fresh stock, small tool only.
    let (parts, mut fresh) = setup(&m, 0.5);
    let refs: Vec<&Mesh> = parts.iter().collect();
    let small_only = rough(&refs, &mut fresh, &f, o, &params(6.0));
    let small_len: f64 = small_only.levels.iter().map(|l| l.cut_length_mm).sum();

    // Big tool first, then the small tool on the in-process stock.
    let (_, mut stock) = setup(&m, 0.5);
    let big = rough(&refs, &mut stock, &f, o, &params(16.0));
    assert!(big.removed_mm3 > 0.5 * small_only.removed_mm3);
    let rest = rough(&refs, &mut stock, &f, o, &params(6.0));
    let rest_len: f64 = rest.levels.iter().map(|l| l.cut_length_mm).sum();
    assert!(
        rest_len < 0.6 * small_len,
        "rest {rest_len} vs full {small_len}"
    );
    // Both routes end with about the same material left.
    assert!(
        (rest.remaining_mm3 - small_only.remaining_mm3).abs() < 1500.0,
        "{} vs {}",
        rest.remaining_mm3,
        small_only.remaining_mm3
    );
}

#[test]
fn tilted_three_plus_two_rough_on_a_trunnion() {
    let m = machine(TRUNNION_AC);
    let o = m.wcs_origin;
    // Small block part, machined from 30° off vertical.
    let part = Mesh::cuboid(
        Vec3::new(-10.0, -10.0, -20.0) + o,
        Vec3::new(10.0, 10.0, -5.0) + o,
    );
    let mut stock = Voxels::from_box(
        Vec3::new(-14.0, -14.0, -20.0) + o,
        Vec3::new(14.0, 14.0, 2.0) + o,
        0.5,
    );
    let s = 30f64.to_radians();
    let axis = Vec3::new(0.0, -s.sin(), s.cos());
    let f = machining_frame(o, axis);
    let r = rough(&[&part], &mut stock, &f, o, &params(6.0));
    assert!(!r.moves.is_empty());
    assert!(r
        .moves
        .iter()
        .all(|mv| mv.pose().axis.angle_to(axis) < 1e-12));
    // Every move is reachable on the machine.
    let kin = Kinematics::new(m.clone(), 90.0).unwrap();
    let mut prev: Option<Vec<f64>> = None;
    for mv in &r.moves {
        let sol = kin.inverse(&mv.pose(), prev.as_deref()).expect("reachable");
        prev = Some(sol.joints);
    }
    // No point of the part was cut.
    let mut core =
        Voxels::from_mesh(&part, &Bvh::build(&part), part.aabb().expand(0.5), 0.5).eroded(1);
    let before = core.count();
    for w in r.moves.windows(2) {
        if let pat_core::post::ClMove::Feed { .. } = w[1] {
            let a = pat_core::Pose {
                tip: w[0].pose().tip + o,
                axis,
            };
            let b = pat_core::Pose {
                tip: w[1].pose().tip + o,
                axis,
            };
            core.sweep(&a, &b, &CutterShape::flat(6.0, 20.0));
        }
    }
    assert_eq!(core.count(), before, "the roughing path cut into the part");
}

#[test]
fn automatic_three_plus_two_roughs_under_an_overhang() {
    use pat_core::plan::{auto_rough, AutoRoughParams};
    let m = machine(TRUNNION_AC);
    let o = m.wcs_origin;
    let base = Mesh::cuboid(
        Vec3::new(-10.0, -10.0, -25.0) + o,
        Vec3::new(10.0, 10.0, -5.0) + o,
    );
    let plate = Mesh::cuboid(
        Vec3::new(-20.0, -20.0, -5.0) + o,
        Vec3::new(20.0, 20.0, 0.0) + o,
    );
    let mut stock = Voxels::from_box(
        Vec3::new(-22.0, -22.0, -25.0) + o,
        Vec3::new(22.0, 22.0, 2.0) + o,
        0.5,
    );
    let kin = Kinematics::new(m.clone(), 90.0).unwrap();
    let scene = SceneDef::from_toml(OVERHANG_SCENE).unwrap();
    let world = World::build(&kin, &scene, None, None).unwrap();
    let mut params = AutoRoughParams {
        rough: params(6.0),
        ..Default::default()
    };
    params.orient.tool = Some(1);
    let t0 = std::time::Instant::now();
    let r = auto_rough(&[&base, &plate], &mut stock, &kin, Some(&world), o, &params);
    println!("plan {:?}, {} moves", t0.elapsed(), r.moves.len());
    println!("{}", r.summary);
    for s in &r.setups {
        println!("  {}", s.orientation.explanation);
    }
    assert!(r.setups.len() >= 3, "{}", r.summary);
    // Material under the overhang (beside the base, below the plate) is gone.
    for p in [
        Vec3::new(16.0, 0.0, -15.0),
        Vec3::new(0.0, -16.0, -20.0),
        Vec3::new(-16.0, 5.0, -10.0),
    ] {
        assert!(
            !stock.material_at(p + o),
            "material left at {p:?}: {}",
            r.summary
        );
    }
    // ... but the part itself is intact.
    for p in [Vec3::new(9.0, 0.0, -15.0), Vec3::new(18.0, 18.0, -2.0)] {
        assert!(stock.material_at(p + o), "part cut at {p:?}");
    }

    // Post and replay on the real machine model (spindle housing, cradle and
    // platter geometry included) with stock simulation.
    let cl = ClProgram {
        name: "auto 3+2".into(),
        tool: ClTool {
            number: 1,
            length: 90.0,
            diameter: Some(6.0),
        },
        spindle_rpm: Some(12000.0),
        coolant: false,
        moves: r.moves,
    };
    let out = post(
        &m,
        &cl,
        &PostOptions {
            dialect: Dialect::FanucTcp,
            ..Default::default()
        },
    )
    .unwrap();
    let opts = SimOptions {
        world: Some(Arc::new(world)),
        simulate_stock: true,
        stock_resolution: 0.5,
        gouge_tolerance: 0.1,
        ..Default::default()
    };
    let t1 = std::time::Instant::now();
    let rep = simulate(&m, &out.gcode, &opts);
    println!("replay {:?}, {} blocks", t1.elapsed(), out.blocks);
    assert!(
        rep.ok,
        "{:#?}",
        rep.issues.iter().take(10).collect::<Vec<_>>()
    );
}

const OVERHANG_SCENE: &str = r#"
[[tools]]
number = 1
gauge_length = 90.0
segments = [
  { kind = "cutter", length = 25.0, diameter = 6.0 },
  { kind = "shank", length = 20.0, diameter = 6.0 },
  { kind = "holder", length = 40.0, diameter = 30.0 },
]
[[bodies]]
name = "stock"
role = "stock"
shape = { type = "box", min = [-22.0, -22.0, -25.0], max = [22.0, 22.0, 2.0] }
[[bodies]]
name = "base"
role = "part"
shape = { type = "box", min = [-10.0, -10.0, -25.0], max = [10.0, 10.0, -5.0] }
[[bodies]]
name = "plate"
role = "part"
shape = { type = "box", min = [-20.0, -20.0, -5.0], max = [20.0, 20.0, 0.0] }
"#;
