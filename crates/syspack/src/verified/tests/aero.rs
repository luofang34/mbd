//! Aerodynamic elements of composed packages: the capsule constructor, the
//! empty vacuum vehicle, the schema gate and the flapping-wing record.

use super::*;

#[test]
fn the_capsule_aero_reads_back_on_its_component() {
    let package = load_json(&downloaded_capsule()).unwrap();
    let elements = package.build_aero_elements().unwrap();
    assert_eq!(elements.len(), 1);
    assert_eq!(elements[0].component, package.component_id("root").unwrap());
    let mbd_aero::AeroElementKind::BluntCapsule(capsule) = &elements[0].kind else {
        panic!("{:?}", elements[0].kind);
    };
    let legacy = mbd_aero::BluntCapsuleAero::hypersonic_newtonian(
        1.0,
        2.0,
        DVec3::new(-2.0, 0.0, 0.0),
        0.1,
        1.2,
        -5.0,
        1.7415e-4,
    )
    .unwrap();
    assert_eq!(
        capsule, &legacy,
        "the composed record is the Newtonian capsule"
    );
}

#[test]
fn a_vacuum_package_has_no_aero_elements() {
    let mut value = downloaded_capsule();
    value["dynamics"] = serde_json::to_value(DynamicsSpec::vacuum()).unwrap();
    let package = load_json(&value).unwrap();
    assert!(package.build_aero_elements().unwrap().is_empty());
}

#[test]
fn any_other_schema_version_is_refused() {
    for version in [1, 2, 4] {
        let mut value = downloaded_capsule();
        value["schema_version"] = json!(version);
        assert!(matches!(
            load_json(&value),
            Err(SyspackError::UnsupportedSchema { found, expected: 3 }) if found == version
        ));
    }
}

#[test]
fn a_flapping_wing_record_builds_on_its_link() {
    let mut value = downloaded_capsule();
    value["dynamics"] = json!({"MultibodyComposed": {"aero_elements": [{
        "site": "hull",
        "kind": {"FlappingWing": {
            "root_m": [0.0, 0.0, 0.0],
            "span_axis": [0.0, 1.0, 0.0],
            "chord_axis": [0.0, 0.0, 1.0],
            "length_m": 2.39e-3,
            "chords_m": [0.6e-3, 0.9e-3, 0.9e-3, 0.6e-3],
            "rotation_axis_chord_fraction": 0.25,
            "center_of_pressure_chord_fraction": 0.25,
        }},
    }]}});
    let package = load_json(&value).unwrap();
    let elements = package.build_aero_elements().unwrap();
    assert!(matches!(
        elements[0].kind,
        mbd_aero::AeroElementKind::FlappingWing(_)
    ));
    value["dynamics"]["MultibodyComposed"]["aero_elements"][0]["kind"]["FlappingWing"]["chords_m"] =
        json!([]);
    assert!(matches!(
        load_json(&value),
        Err(SyspackError::InvalidAero(_))
    ));
}
