use serde_json::{Value, json};

use super::*;

fn component(key: &str, parent: Option<&str>) -> Value {
    json!({
        "key": key,
        "name": key,
        "parent": parent,
        "translation_m": [0.0, 0.0, 0.0],
        "dry_mass_kg": 1.0,
        "dry_inertia_body": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        "initial_fluid_mass_kg": 0.0,
        "initial_payload_mass_kg": 0.0,
        "attached": true,
        "thermal_capacity_j_k": 0.0,
        "nominal_power_w": 0.0,
    })
}

fn manifest() -> Value {
    json!({
        "schema_version": 1,
        "id": "test_vehicle-2",
        "name": "Test vehicle",
        "version": "1.0.0",
        "kind": "Spacecraft",
        "assets": {},
        "components": [component("root", None)],
        "dynamics": "RigidBodyVacuum",
        "devices": [],
    })
}

fn parse(value: &Value) -> Result<SyspackManifest, SyspackError> {
    SyspackManifest::parse(value.to_string().as_bytes())
}

fn jsbsim() -> Value {
    json!({"JsbsimXml": {
        "aircraft_dir": "aircraft",
        "model_name": "m",
        "entry_asset": "m.xml",
        "virtual_path": "aircraft/m/m.xml",
        "additional_assets": [],
    }})
}

fn declare_entry(value: &mut Value) {
    value["assets"]["m.xml"] = json!({"sha256": "0".repeat(64), "size_bytes": 0});
}

#[test]
fn a_minimal_manifest_parses_and_round_trips() {
    let parsed = parse(&manifest()).unwrap();
    assert_eq!(parsed.kind, SystemKind::Spacecraft);
    let text = serde_json::to_vec(&parsed).unwrap();
    assert_eq!(SyspackManifest::parse(&text).unwrap(), parsed);
}

#[test]
fn an_omitted_rotation_is_the_identity() {
    let parsed = parse(&manifest()).unwrap();
    assert_eq!(parsed.components[0].rotation_xyzw, [0.0, 0.0, 0.0, 1.0]);
}

#[test]
fn every_system_kind_parses() {
    for kind in [
        "FixedWing",
        "Rotorcraft",
        "Multirotor",
        "LunarLander",
        "ReentryCapsule",
        "Spacecraft",
        "GroundStation",
        "SurfaceVehicle",
        "TestBench",
        "GenericSystem",
    ] {
        let mut value = manifest();
        value["kind"] = json!(kind);
        parse(&value).unwrap();
    }
}

#[test]
fn an_unknown_kind_is_refused() {
    let mut value = manifest();
    value["kind"] = json!("Submarine");
    assert!(matches!(parse(&value), Err(SyspackError::ManifestParse(_))));
}

#[test]
fn a_stationary_system_declares_no_dynamics() {
    let mut value = manifest();
    value["kind"] = json!("GroundStation");
    value["dynamics"] = json!("None");
    let parsed = parse(&value).unwrap();
    assert_eq!(parsed.dynamics, DynamicsSpec::None);
    assert_eq!(parsed.dynamics.kind(), "None");
}

#[test]
fn omitted_controls_are_none_and_named_ones_stand_alone() {
    assert_eq!(
        parse(&manifest()).unwrap().controls,
        ControlsSpec::default()
    );
    let mut value = manifest();
    value["controls"] = json!({"has_magnetos": true, "has_wheel_brakes": true});
    let parsed = parse(&value).unwrap();
    assert_eq!(
        parsed.controls,
        ControlsSpec {
            has_magnetos: true,
            has_wheel_brakes: true,
            ..ControlsSpec::default()
        }
    );
    value["controls"]["has_afterburner"] = json!(true);
    assert!(matches!(parse(&value), Err(SyspackError::ManifestParse(_))));
}

#[test]
fn only_schema_one_is_read() {
    let mut value = manifest();
    value["schema_version"] = json!(2);
    assert!(matches!(
        parse(&value),
        Err(SyspackError::UnsupportedSchema {
            found: 2,
            expected: 1
        })
    ));
}

#[test]
fn package_ids_must_be_slugs() {
    for id in ["", "Upper", "has space", "dot.ted", "../up", "é"] {
        let mut value = manifest();
        value["id"] = json!(id);
        assert!(
            matches!(parse(&value), Err(SyspackError::InvalidPackageId { .. })),
            "{id:?}"
        );
    }
}

#[test]
fn empty_names_are_refused() {
    for field in ["name", "version"] {
        let mut value = manifest();
        value[field] = json!("");
        assert!(matches!(
            parse(&value),
            Err(SyspackError::EmptyField { .. })
        ));
    }
}

#[test]
fn unknown_fields_are_refused_at_every_level() {
    let mut top = manifest();
    top["surprise"] = json!(1);
    let mut record = manifest();
    record["components"][0]["surprise"] = json!(1);
    let mut asset = manifest();
    asset["assets"]["a"] = json!({"sha256": "0".repeat(64), "size_bytes": 0, "surprise": 1});
    let mut capsule = manifest();
    capsule["dynamics"] = json!({"RigidBodyBluntCapsule": {
        "base_radius_m": 1.0, "nose_radius_m": 2.0, "reference_point_body_m": [-2.0, 0.0, 0.0],
        "normal_force_slope": 0.1, "axial_drag_coeff": 1.0, "pitch_damping_coeff": -1.0,
        "sutton_graves_k": 1e-4, "surprise": 1,
    }});
    let mut device = manifest();
    device["devices"] = json!([{
        "id": "imu", "site": "root", "kind": {"Icm42688": {"seed": 1, "surprise": 1}},
    }]);
    let mut mounted = manifest();
    mounted["devices"] = json!([{
        "id": "imu", "site": "root", "kind": {"Icm42688": {"seed": 1}}, "surprise": 1,
    }]);
    for value in [top, record, asset, capsule, device, mounted] {
        assert!(
            matches!(parse(&value), Err(SyspackError::ManifestParse(_))),
            "{value}"
        );
    }
}

#[test]
fn unsafe_asset_paths_are_refused() {
    for path in [
        "",
        "/etc/passwd",
        "..",
        "../secret",
        "fdm/../../secret",
        "fdm/./a.xml",
        "./a.xml",
        "a\\b.xml",
        "..\\secret",
        "fdm//a.xml",
        "fdm/",
    ] {
        let mut value = manifest();
        value["assets"][path] = json!({"sha256": "0".repeat(64), "size_bytes": 0});
        assert!(
            matches!(parse(&value), Err(SyspackError::UnsafeAssetPath { .. })),
            "{path:?}"
        );
    }
}

#[test]
fn plain_relative_paths_are_accepted() {
    for path in ["a.xml", "fdm/engines/a.b.xml", "..hidden/file", "a..b"] {
        validate::check_asset_path(path).unwrap();
    }
}

#[test]
fn digests_must_be_sixty_four_hex_digits() {
    for sha256 in ["", "abc", &"g".repeat(64), &"0".repeat(63)] {
        let mut value = manifest();
        value["assets"]["a"] = json!({"sha256": sha256, "size_bytes": 0});
        assert!(matches!(
            parse(&value),
            Err(SyspackError::MalformedDigest { .. })
        ));
    }
}

#[test]
fn jsbsim_entry_must_be_a_declared_asset() {
    let mut value = manifest();
    value["dynamics"] = jsbsim();
    assert!(matches!(
        parse(&value),
        Err(SyspackError::UndeclaredAsset { path }) if path == "m.xml"
    ));
}

#[test]
fn jsbsim_additional_assets_must_be_declared_and_their_paths_safe() {
    let mut value = manifest();
    declare_entry(&mut value);
    value["dynamics"] = jsbsim();
    value["dynamics"]["JsbsimXml"]["additional_assets"] = json!([["e.xml", "aircraft/m/e.xml"]]);
    assert!(matches!(
        parse(&value),
        Err(SyspackError::UndeclaredAsset { path }) if path == "e.xml"
    ));
    value["dynamics"]["JsbsimXml"]["additional_assets"] = json!([["m.xml", "../e.xml"]]);
    assert!(matches!(
        parse(&value),
        Err(SyspackError::UnsafeAssetPath { .. })
    ));
}

#[test]
fn jsbsim_entry_must_be_where_jsbsim_will_look() {
    let mut value = manifest();
    declare_entry(&mut value);
    value["dynamics"] = jsbsim();
    value["dynamics"]["JsbsimXml"]["virtual_path"] = json!("aircraft/m/other.xml");
    assert!(matches!(
        parse(&value),
        Err(SyspackError::JsbsimEntryPath { expected, .. }) if expected == "aircraft/m/m.xml"
    ));
}

#[test]
fn a_system_needs_components_with_unique_keys_and_known_parents() {
    let mut empty = manifest();
    empty["components"] = json!([]);
    assert!(matches!(parse(&empty), Err(SyspackError::NoComponents)));

    let mut repeated = manifest();
    repeated["components"] = json!([component("a", None), component("a", None)]);
    assert!(matches!(
        parse(&repeated),
        Err(SyspackError::DuplicateComponent { key }) if key == "a"
    ));

    let mut orphan = manifest();
    orphan["components"] = json!([component("a", Some("ghost"))]);
    assert!(matches!(
        parse(&orphan),
        Err(SyspackError::UnknownParent { component, parent })
            if component == "a" && parent == "ghost"
    ));
}

#[test]
fn devices_need_unique_ids_known_sites_and_a_custom_type() {
    let imu = json!({"id": "imu", "site": "root", "kind": {"Icm42688": {"seed": 1}}});
    let mut repeated = manifest();
    repeated["devices"] = json!([imu, imu]);
    assert!(matches!(
        parse(&repeated),
        Err(SyspackError::DuplicateDevice { id }) if id == "imu"
    ));

    let mut lost = manifest();
    lost["devices"] = json!([{"id": "imu", "site": "ghost", "kind": {"Icm42688": {"seed": 1}}}]);
    assert!(matches!(
        parse(&lost),
        Err(SyspackError::UnknownDeviceSite { device, site })
            if device == "imu" && site == "ghost"
    ));

    let mut untyped = manifest();
    untyped["devices"] = json!([{
        "id": "x", "site": "root", "kind": {"Custom": {"type_id": "", "parameters": null}},
    }]);
    assert!(matches!(
        parse(&untyped),
        Err(SyspackError::EmptyField { .. })
    ));
}

#[test]
fn custom_parameters_survive_unchanged() {
    let mut value = manifest();
    let parameters = json!({"nested": {"gain": 0.1, "ids": [1, 2]}});
    value["devices"] = json!([{
        "id": "x", "site": "root",
        "kind": {"Custom": {"type_id": "vendor.widget", "parameters": parameters}},
    }]);
    let parsed = parse(&value).unwrap();
    let [device] = parsed.devices.as_slice() else {
        panic!("one device");
    };
    let DeviceKindSpec::Custom {
        type_id,
        parameters: read,
    } = &device.kind
    else {
        panic!("{:?}", device.kind);
    };
    assert_eq!(type_id, "vendor.widget");
    assert_eq!(*read, parameters);
}

/// Replay depends on `syspack.json` parsing to the very doubles written, which
/// `serde_json`'s default fast float path does not promise.
#[test]
fn every_double_survives_the_manifest_exactly() {
    let mut state = 0x2545_f491_4f6c_dd1du64;
    for _ in 0..20_000 {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let value = (state >> 11) as f64 / (1u64 << 40) as f64 - 2_000_000.0;
        let mut text = manifest();
        text["components"][0]["translation_m"] = json!([value, 0.0, 0.0]);
        let parsed = parse(&text).unwrap();
        assert_eq!(
            parsed.components[0].translation_m[0].to_bits(),
            value.to_bits()
        );
    }
}

#[test]
fn malformed_json_is_a_parse_error() {
    assert!(matches!(
        SyspackManifest::parse(b"{not json"),
        Err(SyspackError::ManifestParse(_))
    ));
}
