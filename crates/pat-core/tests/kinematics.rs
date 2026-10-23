mod common;

use common::*;
use pat_core::math::Vec3;
use pat_core::{Kinematics, Pose};

fn close(a: Vec3, b: Vec3, tol: f64) -> bool {
    a.distance(b) < tol
}

#[test]
fn program_zero_puts_tip_on_part_zero() {
    for m in all_machines() {
        let k = Kinematics::new(m.clone(), m.default_tool_length).unwrap();
        let q = k.to_machine(&vec![0.0; m.axes.len()]);
        let p = k.forward(&q);
        assert!(
            close(p.tip, Vec3::ZERO, 1e-9),
            "{}: tip {:?}",
            m.name,
            p.tip
        );
        assert!(close(p.axis, Vec3::Z, 1e-12), "{}", m.name);
    }
}

#[test]
fn trunnion_known_orientations() {
    let m = machine(TRUNNION_AC);
    let k = Kinematics::new(m.clone(), 100.0).unwrap();
    let mut q = k.to_machine(&[0.0, 0.0, 0.0, 90.0, 0.0]);
    assert!(close(k.forward(&q).axis, Vec3::new(0.0, -1.0, 0.0), 1e-12));
    q[4] = 90.0; // +C turns the tool counter-clockwise about the part
    assert!(close(k.forward(&q).axis, Vec3::new(1.0, 0.0, 0.0), 1e-12));
}

#[test]
fn forward_inverse_roundtrip_all_machines() {
    let mut rng = Lcg(42);
    for m in all_machines() {
        let k = Kinematics::new(m.clone(), m.default_tool_length).unwrap();
        let mut checked = 0;
        for _ in 0..400 {
            let q: Vec<f64> = m
                .axes
                .iter()
                .map(|a| rng.range(a.min.unwrap_or(-360.0), a.max.unwrap_or(360.0)))
                .collect();
            let pose = k.forward(&q);
            let near_singular = k.singularity_angle_deg(pose.axis).is_some_and(|s| s < 2.0);

            // With the true joints as the previous state, IK must return them.
            let best = k.inverse(&pose, Some(&q)).unwrap();
            let back = k.forward(&best.joints);
            assert!(
                close(back.tip, pose.tip, 1e-6),
                "{}: tip {:?} vs {:?}",
                m.name,
                back.tip,
                pose.tip
            );
            assert!(back.axis.angle_to(pose.axis) < 1e-9, "{}", m.name);
            if !near_singular {
                for (i, (a, b)) in best.joints.iter().zip(&q).enumerate() {
                    assert!(
                        (a - b).abs() < 1e-6,
                        "{} axis {}: {a} vs {b}",
                        m.name,
                        m.axes[i].name
                    );
                }
            }

            // Every alternative solution must also reproduce the pose.
            for s in k.inverse_all(&pose, None).unwrap() {
                let p = k.forward(&s.joints);
                assert!(close(p.tip, pose.tip, 1e-6), "{}", m.name);
                assert!(p.axis.angle_to(pose.axis) < 1e-9, "{}", m.name);
            }
            checked += 1;
        }
        assert_eq!(checked, 400);
    }
}

#[test]
fn unreachable_orientation_is_an_error() {
    let k = Kinematics::new(machine(VMC_3AXIS), 80.0).unwrap();
    let pose = Pose {
        tip: Vec3::ZERO,
        axis: Vec3::new(0.0, 0.5, 0.866),
    };
    assert!(k.inverse(&pose, None).is_err());

    // A 4th-axis machine cannot tilt the tool toward X.
    let k = Kinematics::new(machine(VMC_4TH_A), 80.0).unwrap();
    let pose = Pose {
        tip: Vec3::ZERO,
        axis: Vec3::new(0.5, 0.0, 0.866),
    };
    assert!(k.inverse(&pose, None).is_err());
}

#[test]
fn limits_choose_the_reachable_branch() {
    // Tool leaning toward +Y needs A < 0 on this trunnion (A min is -30),
    // or A > 0 with C turned 180°. Leaning 60° is only reachable via C.
    let m = machine(TRUNNION_AC);
    let k = Kinematics::new(m, 100.0).unwrap();
    let s = 60f64.to_radians();
    let pose = Pose {
        tip: Vec3::ZERO,
        axis: Vec3::new(0.0, s.sin(), s.cos()),
    };
    let sol = k.inverse(&pose, None).unwrap();
    assert!(sol.within_limits);
    assert!((sol.joints[3] - 60.0).abs() < 1e-9, "A = {}", sol.joints[3]);
    assert!(
        (sol.joints[4].abs() - 180.0).abs() < 1e-9,
        "C = {}",
        sol.joints[4]
    );
}

#[test]
fn singular_pose_holds_primary_rotary() {
    let m = machine(TRUNNION_AC);
    let k = Kinematics::new(m, 100.0).unwrap();
    let prev = k.to_machine(&[0.0, 0.0, 0.0, 10.0, 37.0]);
    let sol = k
        .inverse(
            &Pose {
                tip: Vec3::ZERO,
                axis: Vec3::Z,
            },
            Some(&prev),
        )
        .unwrap();
    assert!(sol.singular);
    assert!((sol.joints[4] - 37.0).abs() < 1e-9);
    assert!(sol.joints[3].abs() < 1e-9);
}
