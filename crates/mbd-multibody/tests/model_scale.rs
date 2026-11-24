//! The characteristic scale every model is built with: validated when
//! given, derived from the tree where the tree defines it, refused where it
//! does not.

#![cfg(test)]

use glam::DMat3;
use mbd_core::{ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_multibody::{ModelScale, MultibodyError, MultibodyModel, TimeScale};

fn body(mass_kg: f64, inertia: DMat3) -> ComponentTree {
    let spec = ComponentSpec {
        name: "body".to_owned(),
        placement: Placement::ROOT,
        dry_mass_kg: mass_kg,
        dry_inertia_body: inertia,
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint: JointSpec::FIXED,
    };
    ComponentTree::new([(spec, ComponentState::at_temperature(293.15))]).unwrap()
}

#[test]
fn every_part_of_a_scale_must_be_finite_and_positive() {
    let bad = [0.0, -1.0, f64::NAN, f64::INFINITY];
    for value in bad {
        for (field, scale) in [
            ("length_m", ModelScale::new(value, 1.0, 1.0)),
            ("mass_kg", ModelScale::new(1.0, value, 1.0)),
            ("time_s", ModelScale::new(1.0, 1.0, value)),
            (
                "acceleration_m_s2",
                ModelScale::with_time(1.0, 1.0, TimeScale::Acceleration(value)),
            ),
        ] {
            match scale {
                Err(MultibodyError::InvalidScale { field: f, .. }) => assert_eq!(f, field),
                other => panic!("{field} = {value}: {other:?}"),
            }
        }
    }
}

#[test]
fn an_acceleration_gives_the_time_to_cover_the_length() {
    let s = ModelScale::with_time(2.0e-3, 1.0e-6, TimeScale::Acceleration(9.81)).unwrap();
    assert_eq!(s.time_s(), (2.0e-3_f64 / 9.81).sqrt());
    assert_eq!(s.speed_m_s(), s.length_m() / s.time_s());
    assert_eq!(s.energy_j(), 1.0e-6 * s.speed_m_s() * s.speed_m_s());
}

#[test]
fn the_tree_supplies_length_and_mass_and_the_caller_the_time() {
    // Solid sphere of radius 0.5 m: I = 2/5 m r², so the radius of
    // gyration √(tr I / m) is √(6/5)·r.
    let (m, r) = (3.0, 0.5);
    let tree = body(m, DMat3::IDENTITY * (0.4 * m * r * r));
    let model = MultibodyModel::with_derived_scale(&tree, &[], TimeScale::Seconds(0.25)).unwrap();
    let s = model.scale();
    assert!((s.length_m() - (1.2_f64).sqrt() * r).abs() < 1e-15);
    assert_eq!(s.mass_kg(), m);
    assert_eq!(s.time_s(), 0.25);
}

#[test]
fn a_point_mass_has_no_extent_to_derive_a_length_from() {
    let point = body(1.0, DMat3::ZERO);
    let err = MultibodyModel::with_derived_scale(&point, &[], TimeScale::Seconds(1.0));
    assert!(
        matches!(
            err,
            Err(MultibodyError::InvalidScale {
                field: "length_m",
                ..
            })
        ),
        "{err:?}"
    );
}
