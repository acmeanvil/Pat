mod common;

use common::*;
use pat_core::math::Vec3;
use pat_core::post::{post, ClMove, ClProgram, ClTool, Dialect, PostOptions};
use pat_core::sim::{simulate, Report, SimOptions};
use pat_core::MachineDef;
use std::collections::BTreeMap;

fn sim(m: &MachineDef, src: &str, tool_length: f64) -> Report {
    let opts = SimOptions {
        tool_lengths: BTreeMap::from([(1, tool_length)]),
        tool_length: Some(tool_length),
        record_trace: true,
        ..Default::default()
    };
    simulate(m, src, &opts)
}

fn codes(r: &Report) -> Vec<&str> {
    r.issues.iter().map(|i| i.code.as_str()).collect()
}

/// Ball-end style cone sweep: the tip runs round a circle while the tool
/// leans `lean` degrees outward, so both rotaries move continuously.
fn cone_path(radius: f64, z: f64, lean_deg: f64, steps: usize) -> Vec<ClMove> {
    let lean = lean_deg.to_radians();
    let mut moves = Vec::new();
    let axis_at = |t: f64| Vec3::new(lean.sin() * t.cos(), lean.sin() * t.sin(), lean.cos());
    let tip_at = |t: f64| Vec3::new(radius * t.cos(), radius * t.sin(), z);
    moves.push(ClMove::Rapid {
        tip: tip_at(0.0) + axis_at(0.0) * 30.0,
        axis: axis_at(0.0),
    });
    for k in 0..=steps {
        let t = std::f64::consts::TAU * k as f64 / steps as f64;
        moves.push(ClMove::Feed {
            tip: tip_at(t),
            axis: axis_at(t),
            feed: 1500.0,
        });
    }
    moves
}

/// Helical wrap on a 4th-axis part: tool always radial to the X axis.
fn wrap_path(radius: f64, length: f64, turns: f64, steps: usize) -> Vec<ClMove> {
    let mut moves = Vec::new();
    let at = |k: usize| {
        let s = k as f64 / steps as f64;
        let th = std::f64::consts::TAU * turns * s;
        let axis = Vec3::new(0.0, -th.sin(), th.cos());
        (Vec3::new(length * s, 0.0, 0.0) + axis * radius, axis)
    };
    let (t0, a0) = at(0);
    moves.push(ClMove::Rapid {
        tip: t0 + a0 * 20.0,
        axis: a0,
    });
    for k in 0..=steps {
        let (tip, axis) = at(k);
        moves.push(ClMove::Feed {
            tip,
            axis,
            feed: 800.0,
        });
    }
    moves
}

fn point_to_polyline(p: Vec3, pts: &[Vec3]) -> f64 {
    pts.windows(2)
        .map(|w| {
            let (a, b) = (w[0], w[1]);
            let ab = b - a;
            let l2 = ab.dot(ab);
            let t = if l2 < 1e-18 {
                0.0
            } else {
                ((p - a).dot(ab) / l2).clamp(0.0, 1.0)
            };
            p.distance(a + ab * t)
        })
        .fold(f64::INFINITY, f64::min)
}

fn roundtrip(m: &MachineDef, moves: Vec<ClMove>, tool_length: f64, dialect: Dialect) {
    let cl = ClProgram {
        name: "roundtrip".into(),
        tool: ClTool {
            number: 1,
            length: tool_length,
            diameter: Some(6.0),
        },
        spindle_rpm: Some(12000.0),
        coolant: false,
        moves,
    };
    let opts = PostOptions {
        dialect,
        ..Default::default()
    };
    let out = post(m, &cl, &opts).unwrap_or_else(|e| panic!("{}: post failed: {e}", m.name));
    let r = sim(m, &out.gcode, tool_length);
    assert!(
        r.ok,
        "{} {dialect:?}: {:#?}\n{}",
        m.name, r.issues, out.gcode
    );
    assert!(
        !codes(&r).contains(&"tip_deviation"),
        "{} {dialect:?}: tip deviation warnings: {:#?}",
        m.name,
        r.issues
    );

    // Every simulated cutting point must lie on the intended CL polyline.
    let cl_pts: Vec<Vec3> = cl.moves.iter().skip(1).map(|mv| mv.pose().tip).collect();
    let trace = r.trace.unwrap();
    let cutting: Vec<_> = trace.iter().filter(|t| !t.rapid).collect();
    assert!(!cutting.is_empty());
    for t in cutting {
        let d = point_to_polyline(t.tip, &cl_pts);
        assert!(
            d < 0.01,
            "{} {dialect:?}: line {} tip {:?} is {d:.4} mm off path",
            m.name,
            t.line,
            t.tip
        );
    }
    let last = trace.last().unwrap();
    assert!(last.tip.distance(*cl_pts.last().unwrap()) < 1e-3);
}

#[test]
fn post_then_simulate_five_axis_machines() {
    for (src, lean) in [
        (TRUNNION_AC, 35.0),
        (HEAD_TABLE_BC, 40.0),
        (HEAD_HEAD_CA, 40.0),
    ] {
        let m = machine(src);
        let len = m.default_tool_length;
        for dialect in [Dialect::LinuxcncJoint, Dialect::FanucTcp] {
            roundtrip(&m, cone_path(25.0, -5.0, lean, 72), len, dialect);
        }
    }
}

#[test]
fn post_then_simulate_fourth_axis_wrap() {
    let m = machine(VMC_4TH_A);
    for dialect in [Dialect::LinuxcncJoint, Dialect::FanucTcp] {
        roundtrip(&m, wrap_path(20.0, 60.0, 2.0, 200), 80.0, dialect);
    }
}

#[test]
fn joint_post_linearises_rotary_moves() {
    let m = machine(TRUNNION_AC);
    let cl = ClProgram {
        name: String::new(),
        tool: ClTool {
            number: 1,
            length: 100.0,
            diameter: None,
        },
        spindle_rpm: None,
        coolant: false,
        moves: cone_path(25.0, -5.0, 35.0, 8), // coarse: 45° per CL segment
    };
    let coarse = cl.moves.len();
    let out = post(&m, &cl, &PostOptions::default()).unwrap();
    let g1 = out.gcode.lines().filter(|l| l.starts_with("G1")).count();
    assert!(
        g1 > coarse * 3,
        "expected subdivision, got {g1} G1 blocks for {coarse} CL moves"
    );
    assert!(out.gcode.contains("G93"));
}

#[test]
fn flags_axis_limit() {
    let m = machine(TRUNNION_AC);
    let r = sim(&m, "G21 G90 G94\nG0 X0 Y0 Z50\nG1 A130 F1000\nM30\n", 100.0);
    assert!(!r.ok);
    assert!(codes(&r).contains(&"axis_limit"), "{:#?}", r.issues);
}

#[test]
fn flags_mid_move_overtravel_in_tcp() {
    // Both endpoints are inside the Y limit (250), but holding the tip at
    // Y248 while tilting swings it out to ~253 mm around A = 11°.
    let m = machine(TRUNNION_AC);
    let r = sim(
        &m,
        "G21 G90 G94\nG43.4 H1\nG0 X0 Y248 Z0 A0 C0\nG1 A25 F1000\nM30\n",
        100.0,
    );
    let hits: Vec<_> = r.issues.iter().filter(|i| i.code == "axis_limit").collect();
    assert_eq!(hits.len(), 1, "{:#?}", r.issues);
    assert_eq!(hits[0].line, 4);
    assert!(hits[0].message.starts_with("Y = 25"), "{}", hits[0].message);
    let y = r.axis_ranges.iter().find(|a| a.axis == "Y").unwrap();
    assert!(y.max > 252.0 && y.max < 254.0, "Y max {}", y.max);
}

#[test]
fn flags_missing_inverse_time_feed() {
    let m = machine(TRUNNION_AC);
    let r = sim(
        &m,
        "G21 G90\nG0 X0 Y0 Z50\nG93\nG1 X10 F2\nG1 X20\nM30\n",
        100.0,
    );
    assert!(!r.ok);
    let missing: Vec<_> = r
        .issues
        .iter()
        .filter(|i| i.code == "missing_feed")
        .collect();
    assert_eq!(missing.len(), 1);
    assert_eq!(missing[0].line, 5);
}

#[test]
fn flags_unknown_axis() {
    let m = machine(TRUNNION_AC);
    let r = sim(&m, "G0 X0 B10\n", 100.0);
    assert!(codes(&r).contains(&"unknown_axis"));
}

#[test]
fn flags_unlinearised_rotary_move() {
    // A big A move in joint mode with the tip 50 mm from the pivot traces an
    // arc, not a straight line.
    let m = machine(TRUNNION_AC);
    let r = sim(
        &m,
        "G21 G90 G94\nG0 X0 Y0 Z0 A0 C0\nG1 A45 F500\nM30\n",
        100.0,
    );
    assert!(codes(&r).contains(&"tip_deviation"), "{:#?}", r.issues);
}

#[test]
fn flags_singularity_and_flip() {
    let m = machine(TRUNNION_AC);
    let r = sim(
        &m,
        "G21 G90 G94\nG43.4 H1\nG0 X0 Y0 Z10 A0.3 C0\nG1 C180 F500\nM30\n",
        100.0,
    );
    let c = codes(&r);
    assert!(c.contains(&"near_singularity"), "{:#?}", r.issues);
    assert!(c.contains(&"rotary_flip"), "{:#?}", r.issues);
}

#[test]
fn arcs_and_cycle_time() {
    let m = machine(VMC_3AXIS);
    let r = sim(
        &m,
        "G21 G90 G94 G17\nG0 X10 Y0 Z0\nG2 X10 Y0 I-10 J0 F600\nG1 X110 F1000\nM30\n",
        80.0,
    );
    assert!(r.ok, "{:#?}", r.issues);
    let circle = std::f64::consts::TAU * 10.0;
    assert!(
        (r.cutting_length_mm - (circle + 100.0)).abs() < 0.05,
        "length {}",
        r.cutting_length_mm
    );
    let expect_s = circle / 600.0 * 60.0 + 6.0;
    assert!(
        (r.cutting_time_s - expect_s).abs() < 0.1,
        "time {} vs {}",
        r.cutting_time_s,
        expect_s
    );
}

#[test]
fn radius_arcs_match_offset_arcs() {
    let m = machine(VMC_3AXIS);
    let a = sim(&m, "G0 X0 Y0 Z0\nG2 X10 Y0 R10 F600\nM30\n", 80.0);
    let b = sim(&m, "G0 X0 Y0 Z0\nG2 X10 Y0 I5 J-8.660254 F600\nM30\n", 80.0);
    assert!(a.ok && b.ok, "{:#?} {:#?}", a.issues, b.issues);
    assert!((a.cutting_length_mm - b.cutting_length_mm).abs() < 0.01);
    // Short CW arc above the chord: 60° of a 10 mm radius.
    assert!((a.cutting_length_mm - 10.0 * std::f64::consts::FRAC_PI_3).abs() < 0.01);
}

#[test]
fn inch_units_convert() {
    let m = machine(VMC_3AXIS);
    let r = sim(&m, "G20 G90 G94\nG0 X0 Y0 Z0\nG1 X1 F10\nM30\n", 80.0);
    assert!((r.cutting_length_mm - 25.4).abs() < 1e-9);
    assert!((r.cutting_time_s - 6.0).abs() < 1e-9);
}

#[test]
fn arcs_replace_circular_runs() {
    // A full circle of radius 10 at constant Z on a 3-axis machine, plus a
    // straight run: the circle becomes arcs, the line stays G1.
    let m = machine(VMC_3AXIS);
    let mut moves = vec![ClMove::Rapid {
        tip: Vec3::new(10.0, 0.0, 5.0),
        axis: Vec3::Z,
    }];
    moves.push(ClMove::Feed {
        tip: Vec3::new(10.0, 0.0, -2.0),
        axis: Vec3::Z,
        feed: 500.0,
    });
    for k in 1..=360 {
        let a = (k as f64).to_radians();
        moves.push(ClMove::Feed {
            tip: Vec3::new(10.0 * a.cos(), 10.0 * a.sin(), -2.0),
            axis: Vec3::Z,
            feed: 800.0,
        });
    }
    for k in 1..=20 {
        moves.push(ClMove::Feed {
            tip: Vec3::new(10.0 + k as f64, 0.0, -2.0),
            axis: Vec3::Z,
            feed: 800.0,
        });
    }
    let cl = ClProgram {
        name: "arcs".into(),
        tool: ClTool {
            number: 1,
            length: 80.0,
            diameter: None,
        },
        spindle_rpm: None,
        coolant: false,
        moves,
    };
    for dialect in [Dialect::LinuxcncJoint, Dialect::FanucTcp] {
        let plain = post(
            &m,
            &cl,
            &PostOptions {
                dialect,
                arcs: false,
                ..Default::default()
            },
        )
        .unwrap();
        let arced = post(
            &m,
            &cl,
            &PostOptions {
                dialect,
                ..Default::default()
            },
        )
        .unwrap();
        let n_arcs = arced
            .gcode
            .lines()
            .filter(|l| l.starts_with("G3") || l.starts_with("G2"))
            .count();
        assert!(
            (2..=4).contains(&n_arcs),
            "{dialect:?}: {n_arcs} arcs\n{}",
            arced.gcode
        );
        assert!(
            arced.blocks + 300 < plain.blocks,
            "{} vs {}",
            arced.blocks,
            plain.blocks
        );
        // Same path and (nearly) the same time when simulated.
        let a = sim(&m, &arced.gcode, 80.0);
        let b = sim(&m, &plain.gcode, 80.0);
        assert!(a.ok && b.ok, "{:#?}", a.issues);
        assert!(
            (a.cutting_length_mm - b.cutting_length_mm).abs() < 0.1,
            "{} vs {}",
            a.cutting_length_mm,
            b.cutting_length_mm
        );
        assert!((a.cutting_time_s - b.cutting_time_s).abs() / b.cutting_time_s < 0.01);
        let end = a.trace.unwrap().last().unwrap().tip;
        assert!(end.distance(Vec3::new(30.0, 0.0, -2.0)) < 1e-3, "{end:?}");
    }
}
