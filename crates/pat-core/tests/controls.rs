//! Work offsets, canned cycles, subroutines, tilted planes and the
//! acceleration-aware cycle-time planner.

mod common;

use common::*;
use pat_core::math::Vec3;
use pat_core::sim::{simulate, Report, SimOptions, TracePoint};
use pat_core::MachineDef;
use std::collections::BTreeMap;

fn run(m: &MachineDef, src: &str, opts: SimOptions) -> Report {
    simulate(
        m,
        src,
        &SimOptions {
            record_trace: true,
            ..opts
        },
    )
}

fn tool(len: f64) -> SimOptions {
    SimOptions {
        tool_length: Some(len),
        ..Default::default()
    }
}

fn codes(r: &Report) -> Vec<&str> {
    r.issues.iter().map(|i| i.code.as_str()).collect()
}

fn trace(r: &Report) -> &[TracePoint] {
    r.trace.as_deref().unwrap()
}

fn close(a: Vec3, b: Vec3, tol: f64) -> bool {
    a.distance(b) < tol
}

// ---------------------------------------------------------------- offsets

#[test]
fn work_offsets_from_options_and_machine_file() {
    let mut m = machine(VMC_3AXIS);
    m.work_offsets
        .insert("G55".into(), Vec3::new(150.0, -20.0, 0.0));
    let opts = SimOptions {
        work_offsets: BTreeMap::from([("G56".into(), Vec3::new(-100.0, 0.0, 0.0))]),
        ..tool(80.0)
    };
    let r = run(
        &m,
        "G21 G90\nG54 G0 X0 Y0 Z10\nG55 X0 Y0\nG56 X5\nM30\n",
        opts,
    );
    assert!(r.ok, "{:#?}", r.issues);
    let t = trace(&r);
    // Machine X joint = part zero X + programmed X (head axes, no rotation).
    assert!((t[0].joints[0] - 0.0).abs() < 1e-9);
    assert!((t[1].joints[0] - 150.0).abs() < 1e-9 && (t[1].joints[1] + 20.0).abs() < 1e-9);
    assert!((t[2].joints[0] + 95.0).abs() < 1e-9);
}

#[test]
fn undefined_work_offset_is_an_error() {
    let m = machine(VMC_3AXIS);
    let r = run(&m, "G57 G0 X0 Y0 Z10\nM30\n", tool(80.0));
    assert!(!r.ok);
    assert!(codes(&r).contains(&"work_offset"));
}

#[test]
fn extended_offsets_parse() {
    let mut m = machine(VMC_3AXIS);
    m.work_offsets
        .insert("G54.1P7".into(), Vec3::new(10.0, 0.0, 0.0));
    m.work_offsets
        .insert("G59.1".into(), Vec3::new(20.0, 0.0, 0.0));
    let r = run(&m, "G54.1 P7 G0 X0 Y0 Z10\nG59.1 X0\nM30\n", tool(80.0));
    assert!(r.ok, "{:#?}", r.issues);
    assert!((trace(&r)[0].joints[0] - 10.0).abs() < 1e-9);
    assert!((trace(&r)[1].joints[0] - 20.0).abs() < 1e-9);
}

// ---------------------------------------------------------------- cycles

#[test]
fn g81_drills_each_hole_and_returns_to_initial_level() {
    let m = machine(VMC_3AXIS);
    let src = "G21 G90 G17\nG0 X0 Y0 Z10\nG98 G81 X10 Y10 Z-5 R2 F100\nX20\nG80\nM30\n";
    let r = run(&m, src, tool(80.0));
    assert!(r.ok, "{:#?}", r.issues);
    assert!(
        (r.cutting_length_mm - 14.0).abs() < 1e-9,
        "{}",
        r.cutting_length_mm
    );
    let t = trace(&r);
    let bottoms: Vec<_> = t.iter().filter(|p| (p.tip.z + 5.0).abs() < 1e-9).collect();
    assert_eq!(bottoms.len(), 2);
    assert!((bottoms[0].tip.x - 10.0).abs() < 1e-9 && (bottoms[1].tip.x - 20.0).abs() < 1e-9);
    assert!(
        (t.last().unwrap().tip.z - 10.0).abs() < 1e-9,
        "G98 returns to the initial Z"
    );
    // Feed-only time: 14 mm at 100 mm/min.
    assert!((r.cutting_time_s - 8.4).abs() < 1e-9);
}

#[test]
fn g99_retracts_to_r_and_g82_dwells() {
    let m = machine(VMC_3AXIS);
    let src = "G21 G90\nG0 X0 Y0 Z10\nG99 G82 X10 Y0 Z-5 R2 P0.5 F100\nX20\nG80\nM30\n";
    let r = run(&m, src, tool(80.0));
    assert!(r.ok, "{:#?}", r.issues);
    assert!((trace(&r).last().unwrap().tip.z - 2.0).abs() < 1e-9);
    assert!((r.dwell_time_s - 1.0).abs() < 1e-9);
}

#[test]
fn fanuc_dwell_in_milliseconds() {
    let m = machine(VMC_3AXIS);
    let opts = SimOptions {
        dwell_p_milliseconds: true,
        ..tool(80.0)
    };
    let r = run(&m, "G0 X0 Y0 Z10\nG4 P1500\nG4 X2\nM30\n", opts);
    assert!((r.dwell_time_s - 3.5).abs() < 1e-9, "{}", r.dwell_time_s);
}

#[test]
fn g83_pecks_to_depth() {
    let m = machine(VMC_3AXIS);
    let src = "G21 G90\nG0 X0 Y0 Z10\nG98 G83 X5 Y5 Z-5 R2 Q2 F100\nG80\nM30\n";
    let r = run(&m, src, tool(80.0));
    assert!(r.ok, "{:#?}", r.issues);
    let t = trace(&r);
    let min_z = t.iter().map(|p| p.tip.z).fold(f64::INFINITY, f64::min);
    assert!((min_z + 5.0).abs() < 1e-9);
    // Pecks to 0, -2, -4, -5 with a full retract to R between them.
    let retracts = t
        .windows(2)
        .filter(|w| w[1].rapid && (w[1].tip.z - 2.0).abs() < 1e-9 && w[0].tip.z < 1.0)
        .count();
    assert_eq!(retracts, 3);
    // Drilled 7 mm, plus re-feeding 0.254 mm of clearance before 3 pecks.
    assert!(
        (r.cutting_length_mm - (7.0 + 3.0 * 0.254)).abs() < 1e-9,
        "{}",
        r.cutting_length_mm
    );
}

#[test]
fn incremental_cycle_with_repeats() {
    let m = machine(VMC_3AXIS);
    // Initial Z 10; R -8 → R plane at 2; Z -7 → bottom at -5. Three holes 10 mm apart.
    let src = "G21 G90\nG0 X0 Y0 Z10\nG91 G98 G81 X10 Z-7 R-8 L3 F100\nG80 G90\nM30\n";
    let r = run(&m, src, tool(80.0));
    assert!(r.ok, "{:#?}", r.issues);
    let xs: Vec<f64> = trace(&r)
        .iter()
        .filter(|p| (p.tip.z + 5.0).abs() < 1e-9)
        .map(|p| p.tip.x)
        .collect();
    assert_eq!(xs, vec![10.0, 20.0, 30.0]);
}

#[test]
fn g84_and_g85_feed_out() {
    let m = machine(VMC_3AXIS);
    for (g, expect) in [(84, 14.0), (85, 14.0), (81, 7.0)] {
        let src = format!("G21 G90\nG0 X0 Y0 Z10\nG98 G{g} X0 Y0 Z-5 R2 F100\nG80\nM30\n");
        let r = run(&m, &src, tool(80.0));
        assert!(r.ok, "G{g}: {:#?}", r.issues);
        assert!(
            (r.cutting_length_mm - expect).abs() < 1e-9,
            "G{g}: {}",
            r.cutting_length_mm
        );
    }
}

#[test]
fn cycle_without_r_is_an_error() {
    let m = machine(VMC_3AXIS);
    let r = run(&m, "G0 X0 Y0 Z10\nG81 X1 Z-1 F100\nM30\n", tool(80.0));
    assert!(codes(&r).contains(&"canned_cycle"));
}

// ---------------------------------------------------------------- subroutines

#[test]
fn linuxcnc_subroutine_runs_twice() {
    let m = machine(VMC_3AXIS);
    let src = "o100 sub\nG91 G1 X10 F600\nG90\no100 endsub\nG21 G90\nG0 X0 Y0 Z0\no100 call\no100 call\nM2\n";
    let r = run(&m, src, tool(80.0));
    assert!(r.ok, "{:#?}", r.issues);
    assert!((r.cutting_length_mm - 20.0).abs() < 1e-9);
    assert!((trace(&r).last().unwrap().tip.x - 20.0).abs() < 1e-9);
}

#[test]
fn fanuc_subprogram_with_repeat_count() {
    let m = machine(VMC_3AXIS);
    let src =
        "%\nO1000\nG21 G90\nG0 X0 Y0 Z0\nM98 P2000 L3\nM30\nO2000\nG91 G1 X5 F600\nG90\nM99\n%\n";
    let r = run(&m, src, tool(80.0));
    assert!(r.ok, "{:#?}", r.issues);
    assert!((r.cutting_length_mm - 15.0).abs() < 1e-9);
}

#[test]
fn undefined_subroutine_is_reported() {
    let m = machine(VMC_3AXIS);
    let r = run(&m, "G0 X0 Y0 Z0\nM98 P9999\nM30\n", tool(80.0));
    assert!(!r.ok);
    assert!(codes(&r).contains(&"subroutine"));
}

// ---------------------------------------------------------------- tilted planes

#[test]
fn g68_2_plane_and_g53_1_alignment() {
    let m = machine(TRUNNION_AC);
    // Plane through (10, 0, 0) tilted 30° about its X axis.
    let src = "G21 G90 G94\nG43 H1\nG0 X0 Y0 Z50 A0 C0\nG68.2 X10 Y0 Z0 I0 J30 K0\nG53.1\nG0 X0 Y0 Z20\nG1 Z5 F500\nG69\nM30\n";
    let r = run(&m, src, tool(100.0));
    assert!(r.ok, "{:#?}", r.issues);
    assert!(!codes(&r).contains(&"tilted_plane"), "{:#?}", r.issues);
    let last = trace(&r).iter().rev().find(|p| !p.rapid).unwrap();
    let s = 30f64.to_radians();
    let normal = Vec3::new(0.0, -s.sin(), s.cos());
    assert!(close(last.axis, normal, 1e-9), "axis {:?}", last.axis);
    assert!(
        close(last.tip, Vec3::new(10.0, 0.0, 0.0) + normal * 5.0, 1e-6),
        "tip {:?}",
        last.tip
    );
}

#[test]
fn g68_2_without_alignment_warns() {
    let m = machine(TRUNNION_AC);
    let src = "G21 G90 G94\nG0 X0 Y0 Z50 A0 C0\nG68.2 X0 Y0 Z0 I0 J20 K0\nG1 X0 Y0 Z5 F500\nM30\n";
    let r = run(&m, src, tool(100.0));
    assert!(codes(&r).contains(&"tilted_plane"), "{:#?}", r.issues);
}

#[test]
fn drilling_cycle_follows_tilted_plane() {
    let m = machine(TRUNNION_AC);
    let src = "G21 G90 G94\nG0 X0 Y0 Z50 A0 C0\nG68.2 X0 Y0 Z0 I90 J45 K-90\nG53.1\nG0 X0 Y0 Z10\nG98 G81 X5 Y0 Z-3 R2 F200\nG80\nG69\nM30\n";
    let r = run(&m, src, tool(100.0));
    assert!(r.ok, "{:#?}", r.issues);
    // Euler (90, 45, -90): plane normal leans 45° toward +X.
    let s = 45f64.to_radians();
    let normal = Vec3::new(s.sin(), 0.0, s.cos());
    let x_dir = Vec3::new(s.cos(), 0.0, -s.sin());
    let bottom = x_dir * 5.0 - normal * 3.0;
    assert!(
        trace(&r)
            .iter()
            .any(|p| close(p.tip, bottom, 1e-6) && close(p.axis, normal, 1e-9)),
        "no hole bottom at {bottom:?}"
    );
    assert!((r.cutting_length_mm - 5.0).abs() < 1e-6);
}

// ---------------------------------------------------------------- planner

fn accel_machine(accel: f64, jd: f64) -> MachineDef {
    let mut m = machine(VMC_3AXIS);
    for a in &mut m.axes {
        a.max_accel = Some(accel);
        a.max_rate = Some(30_000.0);
    }
    m.junction_deviation = jd;
    // Part zero at machine home (tip 320 mm above base with an 80 mm tool),
    // so the opening G0 X0 Y0 Z0 does not move.
    m.wcs_origin = Vec3::new(0.0, 0.0, 320.0);
    m
}

#[test]
fn planner_trapezoid_on_a_single_move() {
    let m = accel_machine(1000.0, 0.02);
    // 100 mm at 100 mm/s with 1000 mm/s²: 0.1 s up + 0.1 s down (5 mm each)
    // + 90 mm cruise (0.9 s) = 1.1 s.
    let r = run(&m, "G0 X0 Y0 Z0\nG4 P0\nG1 X100 F6000\nM30\n", tool(80.0));
    assert_eq!(r.cycle_time_model, "acceleration");
    // The initial rapid is 0 mm; the cut dominates.
    assert!((r.cycle_time_s - 1.1).abs() < 1e-6, "{}", r.cycle_time_s);
}

#[test]
fn planner_blends_collinear_segments() {
    let m = accel_machine(1000.0, 0.02);
    let mut many = String::from("G0 X0 Y0 Z0\nG4 P0\n");
    for k in 1..=100 {
        many.push_str(&format!("G1 X{k} F6000\n"));
    }
    many.push_str("M30\n");
    let r_many = run(&m, &many, tool(80.0));
    assert!(
        (r_many.cycle_time_s - 1.1).abs() < 1e-6,
        "{}",
        r_many.cycle_time_s
    );
}

#[test]
fn planner_corners_and_exact_stop() {
    let m = accel_machine(1000.0, 0.02);
    let square = "G1 X50 F6000\nY50\nX0\nY0\nM30\n";
    let blended = run(&m, &format!("G0 X0 Y0 Z0\nG64\n{square}"), tool(80.0));
    let stopped = run(&m, &format!("G0 X0 Y0 Z0\nG61\n{square}"), tool(80.0));
    // Each 50 mm side from rest to rest: 0.1 + 0.1 + 0.4 = 0.6 s.
    assert!(
        (stopped.cycle_time_s - 2.4).abs() < 1e-6,
        "{}",
        stopped.cycle_time_s
    );
    // 90° corners with 0.02 mm deviation allow only a few mm/s, so blending
    // helps only slightly; it must never be slower.
    assert!(blended.cycle_time_s <= stopped.cycle_time_s + 1e-9);
    assert!(blended.cycle_time_s > 2.3, "{}", blended.cycle_time_s);
    // Feed-rate arithmetic would say 2.0 s.
    assert!((blended.cutting_time_s - 2.0).abs() < 1e-9);
}

#[test]
fn feed_rate_model_without_accel_data() {
    let mut m = machine(VMC_3AXIS);
    m.axes[2].max_accel = None;
    let r = run(&m, "G0 X0 Y0 Z0\nG1 X100 F6000\nM30\n", tool(80.0));
    assert_eq!(r.cycle_time_model, "feed-rate");
    assert!((r.cycle_time_s - (r.cutting_time_s + r.rapid_time_s)).abs() < 1e-9);
}
