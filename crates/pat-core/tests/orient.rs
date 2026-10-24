//! Automatic 3+2 orientation selection.

mod common;

use common::*;
use pat_core::geom::Mesh;
use pat_core::math::Vec3;
use pat_core::orient::{select_orientations, OrientOptions};
use pat_core::Kinematics;

#[test]
fn plain_block_needs_one_setup() {
    let m = machine(TRUNNION_AC);
    let o = m.wcs_origin;
    let part = Mesh::cuboid(
        Vec3::new(-20.0, -15.0, -20.0) + o,
        Vec3::new(20.0, 15.0, 0.0) + o,
    );
    let kin = Kinematics::new(m, 100.0).unwrap();
    let r = select_orientations(&[&part], &kin, None, o, &OrientOptions::default());
    assert_eq!(r.orientations.len(), 1, "{:#?}", r.orientations);
    assert!(r.orientations[0].tilt_deg < 1.0);
    assert!(r.uncovered < 0.005);
    // The bottom face is ignored as resting on the table.
    assert!(
        (r.ignored_area_mm2 - 1200.0).abs() < 150.0,
        "{}",
        r.ignored_area_mm2
    );
}

#[test]
fn overhang_needs_side_setups() {
    // A 20 x 20 base under a 40 x 40 plate: the base walls are hidden from
    // above and need the tool horizontal from each side.
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
    let kin = Kinematics::new(m, 100.0).unwrap();
    let r = select_orientations(&[&base, &plate], &kin, None, o, &OrientOptions::default());
    let axes: Vec<Vec3> = r.orientations.iter().map(|o| o.axis).collect();
    assert!(
        r.orientations[0].tilt_deg < 1.0,
        "first setup should be from the top: {axes:?}"
    );
    // With a 25 mm flute, opposite sides can share the walls between them.
    let horizontal = axes.iter().filter(|a| a.z.abs() < 0.1).count();
    assert!(horizontal >= 2 && axes.len() <= 5, "{axes:?}");
    // Everything but the plate's underside (facing down, ignored) is covered.
    assert!(r.uncovered < 0.01, "uncovered {}", r.uncovered);
    // Each orientation is explained and reachable.
    for o in &r.orientations {
        assert!(o.explanation.contains("setup"), "{}", o.explanation);
        assert_eq!(o.joints.len(), 5);
    }
}

#[test]
fn short_flute_needs_all_four_sides() {
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
    let kin = Kinematics::new(m, 100.0).unwrap();
    let opts = OrientOptions {
        max_wall_depth: 8.0,
        ..Default::default()
    };
    let r = select_orientations(&[&base, &plate], &kin, None, o, &opts);
    let axes: Vec<Vec3> = r.orientations.iter().map(|o| o.axis).collect();
    for side in [Vec3::X, -Vec3::X, Vec3::Y, -Vec3::Y] {
        assert!(
            axes.iter().any(|a| a.angle_to(side) < 15f64.to_radians()),
            "missing side {side:?}: {axes:?}"
        );
    }
    assert!(r.uncovered < 0.01, "uncovered {}", r.uncovered);
}

#[test]
fn unreachable_directions_are_not_chosen() {
    // A 3-axis machine can only use +Z.
    let m = machine(VMC_3AXIS);
    let o = m.wcs_origin;
    let base = Mesh::cuboid(
        Vec3::new(-10.0, -10.0, -25.0) + o,
        Vec3::new(10.0, 10.0, -5.0) + o,
    );
    let plate = Mesh::cuboid(
        Vec3::new(-20.0, -20.0, -5.0) + o,
        Vec3::new(20.0, 20.0, 0.0) + o,
    );
    let kin = Kinematics::new(m, 80.0).unwrap();
    let r = select_orientations(&[&base, &plate], &kin, None, o, &OrientOptions::default());
    assert_eq!(r.orientations.len(), 1);
    assert!(
        r.uncovered > 0.2,
        "base walls are out of reach: {}",
        r.uncovered
    );
}
