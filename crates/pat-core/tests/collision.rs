//! Tool / holder / fixture / machine collision checks and accessibility planning.

mod common;

use common::*;
use pat_core::access::{cap_directions, reachable_directions, tilt_to_avoid, TiltOptions};
use pat_core::collision::{MoveClass, SceneDef, World};
use pat_core::math::Vec3;
use pat_core::sim::{simulate, Report, SimOptions};
use pat_core::{Kinematics, MachineDef};
use std::sync::Arc;

/// T1: Ø10 cutter (20 long), Ø10 shank (20), Ø40 holder (40), then a cone
/// Ø40→Ø60 (20). Gauge length 100.
const TOOLS: &str = r#"
[[tools]]
number = 1
name = "10 mm flat end mill in a shrink-fit holder"
gauge_length = 100.0
segments = [
  { kind = "cutter", length = 20.0, diameter = 10.0 },
  { kind = "shank",  length = 20.0, diameter = 10.0 },
  { kind = "holder", length = 40.0, diameter = 40.0 },
  { kind = "holder", length = 20.0, diameter_bottom = 40.0, diameter_top = 60.0 },
]
"#;

fn scene(extra: &str) -> SceneDef {
    SceneDef::from_toml(&format!("{TOOLS}\n{extra}")).unwrap()
}

fn world(m: &MachineDef, s: &SceneDef) -> World {
    let k = Kinematics::new(m.clone(), 100.0).unwrap();
    World::build(&k, s, None, None).unwrap()
}

fn sim(m: &MachineDef, s: &SceneDef, src: &str) -> Report {
    let opts = SimOptions {
        world: Some(Arc::new(world(m, s))),
        ..Default::default()
    };
    simulate(m, src, &opts)
}

fn codes(r: &Report) -> Vec<&str> {
    r.issues.iter().map(|i| i.code.as_str()).collect()
}

const JAW: &str = r#"
[[bodies]]
name = "vise jaw"
role = "fixture"
shape = { type = "box", min = [20.0, -50.0, 0.0], max = [40.0, 50.0, 30.0] }
"#;

#[test]
fn holder_hits_fixture_only_when_low_enough() {
    let m = machine(VMC_3AXIS);
    let s = scene(JAW);
    // Holder (r = 20) starts 40 mm above the tip. With the tip at X10 it
    // reaches X30, over the jaw (X20..40, top Z30).
    let ok = sim(&m, &s, "T1 M6\nG43 H1\nG0 X10 Y0 Z50\nG1 Z-5 F300\nM30\n");
    assert!(ok.ok, "{:#?}", ok.issues);
    let bad = sim(&m, &s, "T1 M6\nG43 H1\nG0 X10 Y0 Z50\nG1 Z-15 F300\nM30\n");
    assert!(!bad.ok);
    let hit = bad
        .issues
        .iter()
        .find(|i| i.code == "collision")
        .expect("collision");
    assert_eq!(hit.line, 4);
    assert!(
        hit.message.contains("T1 holder hits vise jaw"),
        "{}",
        hit.message
    );
}

#[test]
fn rapid_into_part_but_feed_is_cutting() {
    let m = machine(VMC_3AXIS);
    let s = scene(
        r#"
[[bodies]]
name = "part"
role = "part"
shape = { type = "box", min = [-30.0, -10.0, -20.0], max = [-10.0, 10.0, 0.0] }
"#,
    );
    let feed = sim(
        &m,
        &s,
        "T1 M6\nG43 H1\nG0 X-20 Y0 Z50\nG0 Z2\nG1 Z-5 F300\nM30\n",
    );
    assert!(feed.ok, "{:#?}", feed.issues);
    let rapid = sim(&m, &s, "T1 M6\nG43 H1\nG0 X-20 Y0 Z50\nG0 Z-5\nM30\n");
    assert!(
        codes(&rapid).contains(&"rapid_into_part"),
        "{:#?}",
        rapid.issues
    );
}

#[test]
fn missing_tool_geometry_is_flagged() {
    let m = machine(VMC_3AXIS);
    let r = sim(&m, &scene(JAW), "T7 M6\nG0 X0 Y0 Z50\nM30\n");
    assert!(codes(&r).contains(&"tool_geometry"));
}

#[test]
fn machine_components_collide_across_groups() {
    let mut m = machine(VMC_3AXIS);
    let comps = SceneDef::from_toml(
        r#"
[[bodies]]
name = "spindle housing"
role = "machine"
attach = "head"
shape = { type = "box", min = [-60.0, -60.0, 400.0], max = [60.0, 60.0, 600.0] }

[[bodies]]
name = "table"
role = "machine"
attach = "base"
shape = { type = "box", min = [-400.0, -250.0, -40.0], max = [400.0, 250.0, 20.0] }
"#,
    )
    .unwrap();
    m.components = comps.bodies;
    let s = SceneDef::default();
    // Housing bottom is at 400 + Z: Z-370 clears the table top (20), Z-385 does not.
    let ok = sim(&m, &s, "G53 G0 X0 Y0 Z-370\nM30\n");
    assert!(ok.ok, "{:#?}", ok.issues);
    let bad = sim(&m, &s, "G53 G0 X0 Y0 Z-385\nM30\n");
    let hit = bad
        .issues
        .iter()
        .find(|i| i.code == "collision")
        .expect("collision");
    assert!(
        hit.message.contains("spindle housing hits table"),
        "{}",
        hit.message
    );
}

#[test]
fn trunnion_parts_move_with_the_table() {
    // Table-mounted geometry only: leave the machine's own components out.
    let mut m = machine(TRUNNION_AC);
    m.components.clear();
    let s = scene(
        r#"
[[bodies]]
name = "riser"
role = "fixture"
shape = { type = "box", min = [-30.0, 20.0, -50.0], max = [30.0, 60.0, 0.0] }
"#,
    );
    let w = world(&m, &s);
    let k = Kinematics::new(m.clone(), 100.0).unwrap();
    // Whatever the trunnion angles, a tip inside the riser (in WCS terms)
    // must collide and a tip far from it must not: table-mounted geometry
    // moves exactly with the kinematics.
    for a in [-30.0, 0.0, 45.0, 90.0] {
        for c in [0.0, 90.0, 200.0] {
            let rot = k.to_machine(&[0.0, 0.0, 0.0, a, c]);
            let inside = k.solve_linear(&rot, Vec3::new(0.0, 40.0, -10.0)).unwrap();
            let hits = w.check(&k, &inside, Some(1), MoveClass::Feed, 0.0);
            assert!(hits.iter().any(|h| h.b == "riser"), "A{a} C{c}: {hits:?}");
            let far = k.solve_linear(&rot, Vec3::new(0.0, -100.0, 30.0)).unwrap();
            let none = w.check(&k, &far, Some(1), MoveClass::Feed, 0.0);
            assert!(none.is_empty(), "A{a} C{c}: {none:?}");
        }
    }
}

#[test]
fn clearance_matches_geometry() {
    let m = machine(VMC_3AXIS);
    let s = scene(
        r#"
[[bodies]]
name = "low wall"
role = "fixture"
shape = { type = "box", min = [8.0, -50.0, -20.0], max = [30.0, 50.0, 10.0] }
"#,
    );
    let w = world(&m, &s);
    let k = Kinematics::new(m, 100.0).unwrap();
    let q = k.to_machine(&[0.0, 0.0, 0.0]);
    // Cutter edge at X5, wall at X8.
    let c = w.clearance(&k, &q, 1, 25.0);
    assert!((c - 3.0).abs() < 0.01, "clearance {c}");
}

const WALL: &str = r#"
[[bodies]]
name = "wall"
role = "fixture"
shape = { type = "box", min = [8.0, -40.0, 0.0], max = [30.0, 40.0, 60.0] }
"#;

#[test]
fn tilt_to_avoid_leans_away_from_the_wall() {
    let m = machine(TRUNNION_AC);
    let s = scene(WALL);
    let w = world(&m, &s);
    let k = Kinematics::new(m, 100.0).unwrap();
    let opts = TiltOptions {
        margin: 1.0,
        ..Default::default()
    };
    let r = tilt_to_avoid(&k, &w, 1, Vec3::ZERO, Vec3::Z, None, &opts);
    let axis = r.axis.expect("a direction should exist");
    assert!(r.tilt_deg > 0.0 && r.tilt_deg < 45.0, "{r:?}");
    assert!(axis.x < 0.0, "should lean away from +X wall: {axis:?}");
    assert!(
        r.blocked_by.iter().any(|b| b.contains("holder vs wall")),
        "{:?}",
        r.blocked_by
    );
    assert!(r.clearance >= 1.0);
    assert!(r.explanation.contains("tilted"), "{}", r.explanation);
    // The returned joints really are clear with the margin.
    let q = r.joints.unwrap();
    assert!(w.check_tool_only(&k, &q, 1, 1.0).is_empty());
    // And they realise the returned axis.
    assert!(k.forward(&q).axis.angle_to(axis) < 1e-9);

    // Minimal: one step less tilt must be blocked in every direction tried.
    let less = TiltOptions {
        max_tilt_deg: r.tilt_deg - 1.0,
        ..opts
    };
    assert!(tilt_to_avoid(&k, &w, 1, Vec3::ZERO, Vec3::Z, None, &less)
        .axis
        .is_none());
}

#[test]
fn preferred_axis_kept_when_clear() {
    let m = machine(TRUNNION_AC);
    let w = world(&m, &scene(WALL));
    let k = Kinematics::new(m, 100.0).unwrap();
    let r = tilt_to_avoid(
        &k,
        &w,
        1,
        Vec3::new(-60.0, 0.0, 0.0),
        Vec3::Z,
        None,
        &TiltOptions::default(),
    );
    assert_eq!(r.tilt_deg, 0.0);
    assert_eq!(r.axis, Some(Vec3::Z));
}

#[test]
fn accessibility_map_blocks_directions_toward_the_wall() {
    let m = machine(TRUNNION_AC);
    let w = world(&m, &scene(WALL));
    let k = Kinematics::new(m, 100.0).unwrap();
    let dirs = cap_directions(Vec3::Z, 40.0, 60);
    assert_eq!(dirs.len(), 60);
    assert!(dirs
        .iter()
        .all(|d| d.angle_to(Vec3::Z).to_degrees() <= 40.0 + 1e-9));
    let map = reachable_directions(&k, &w, 1, Vec3::ZERO, &dirs, 1.0, 25.0);
    let toward: Vec<_> = map.iter().filter(|d| d.axis.x > 0.3).collect();
    let away: Vec<_> = map.iter().filter(|d| d.axis.x < -0.3).collect();
    assert!(!toward.is_empty() && toward.iter().all(|d| !d.reachable));
    assert!(!away.is_empty() && away.iter().any(|d| d.reachable));
}
