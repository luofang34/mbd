use std::collections::BTreeMap;

use glam::DVec3;
use mbd_core::{ComponentId, MbdError};
use serde_json::{Value, json};

use super::*;
use crate::{ControlsSpec, MemoryPackageSource, SystemKind};

const ZERO_INERTIA: [[f64; 3]; 3] = [[0.0; 3]; 3];

fn component(key: &str, parent: Option<&str>, x_m: f64, mass_kg: f64, inertia: f64) -> Value {
    json!({
        "key": key,
        "name": format!("The {key}"),
        "parent": parent,
        "translation_m": [x_m, 0.0, 0.0],
        "dry_mass_kg": mass_kg,
        "dry_inertia_body": [[inertia, 0.0, 0.0], [0.0, inertia, 0.0], [0.0, 0.0, inertia]],
        "initial_fluid_mass_kg": 0.0,
        "initial_payload_mass_kg": 0.0,
        "attached": true,
        "thermal_capacity_j_k": 0.0,
        "nominal_power_w": 0.0,
    })
}

fn capsule_dynamics() -> Value {
    json!({"RigidBodyBluntCapsule": {
        "base_radius_m": 1.0,
        "nose_radius_m": 2.0,
        "reference_point_body_m": [-2.0, 0.0, 0.0],
        "normal_force_slope": 0.1,
        "axial_drag_coeff": 1.2,
        "pitch_damping_coeff": -5.0,
        "sutton_graves_k": 1.7415e-4,
    }})
}

/// A vehicle nobody compiled: every part of it arrives as JSON.
fn downloaded_capsule() -> Value {
    let mut tank = component("tank", Some("hull"), -0.2, 5.0, 0.0);
    tank["initial_fluid_mass_kg"] = json!(40.0);
    let mut payload = component("payload_bay", Some("root"), 0.4, 2.0, 0.0);
    payload["initial_payload_mass_kg"] = json!(8.0);
    json!({
        "schema_version": 1,
        "id": "community-capsule",
        "name": "Community capsule",
        "version": "0.3.1",
        "kind": "ReentryCapsule",
        "assets": {},
        "components": [
            component("root", None, 0.0, 0.0, 0.0),
            component("hull", Some("root"), -0.5, 200.0, 80.0),
            tank,
            payload,
            component("imu_mount", Some("hull"), 0.1, 0.0, 0.0),
        ],
        "dynamics": capsule_dynamics(),
        "devices": [
            {"id": "imu", "site": "imu_mount", "kind": {"Icm42688": {"seed": 42}}},
            {"id": "baro", "site": "hull", "kind": {"Bmp390": {"bus_id": 1, "i2c_addr": 119}}},
            {"id": "bus", "site": "hull", "kind": {
                "Arinc429Channel": {"high_speed": true, "gap_bits": 4},
            }},
            {"id": "cam", "site": "payload_bay", "kind": {"PinholeCamera": {
                "offset_body_m": [0.4, 0.0, 0.1],
                "body_to_camera_xyzw": [0.5, 0.5, -0.5, 0.5],
                "width": 640, "height": 480,
                "fx": 500.0, "fy": 500.0, "cx": 319.5, "cy": 239.5,
                "rate_hz": 30,
            }}},
            {"id": "emulator", "site": "hull", "kind": {"Custom": {
                "type_id": "vendor.flight_computer",
                "parameters": {"rom": "not-built-here"},
            }}},
        ],
    })
}

fn load_json(value: &Value) -> Result<VerifiedSyspack, SyspackError> {
    let source = MemoryPackageSource::new(value.to_string().into_bytes(), BTreeMap::new());
    VerifiedSyspack::load(&source)
}

fn with_asset(digest: &Value, bytes: &[u8]) -> Result<VerifiedSyspack, SyspackError> {
    let mut manifest = downloaded_capsule();
    manifest["assets"]["data/table.bin"] = digest.clone();
    let assets = BTreeMap::from([("data/table.bin".to_owned(), bytes.to_vec())]);
    let source = MemoryPackageSource::new(manifest.to_string().into_bytes(), assets);
    VerifiedSyspack::load(&source)
}

/// SHA-256 of `abc`, FIPS 180-2's published example.
const ABC_SHA256: &str = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";

#[test]
fn a_downloaded_vehicle_builds_its_tree_by_key() {
    let package = load_json(&downloaded_capsule()).unwrap();
    assert_eq!(package.tree().len(), 5);
    assert_eq!(package.component_id("root").unwrap(), ComponentId(0));
    assert_eq!(package.component_id("imu_mount").unwrap(), ComponentId(4));
    assert!(matches!(
        package.component_id("ghost"),
        Err(SyspackError::UnknownComponent { key }) if key == "ghost"
    ));
    let (imu_m, _) = package
        .tree()
        .world_transform_in_root(ComponentId(4))
        .unwrap();
    assert!((imu_m - DVec3::new(-0.4, 0.0, 0.0)).length() < 1e-12);
    let mass = package.tree().mass_properties().unwrap();
    assert!((mass.total_mass_kg - 255.0).abs() < 1e-12);
}

#[test]
fn a_parent_may_be_listed_after_its_child() {
    let mut value = downloaded_capsule();
    value["components"] = json!([
        component("child", Some("late_root"), 1.0, 1.0, 0.0),
        component("late_root", None, 0.0, 1.0, 0.0),
    ]);
    value["devices"] = json!([]);
    let package = load_json(&value).unwrap();
    let child = package.tree().spec(ComponentId(0)).unwrap();
    assert_eq!(child.placement.parent, Some(ComponentId(1)));
}

#[test]
fn the_initial_state_comes_from_the_manifest() {
    let mut value = downloaded_capsule();
    value["components"][3]["attached"] = json!(false);
    let package = load_json(&value).unwrap();
    let tank = package.tree().state(ComponentId(2)).unwrap();
    assert_eq!(tank.fluid_mass_kg, 40.0);
    assert_eq!(tank.temperature_k, INITIAL_COMPONENT_TEMPERATURE_K);
    let bay = package.tree().state(ComponentId(3)).unwrap();
    assert_eq!(bay.payload_mass_kg, 8.0);
    assert!(!bay.attached);
    assert!((package.tree().mass_properties().unwrap().total_mass_kg - 245.0).abs() < 1e-12);
}

#[test]
fn an_asset_with_its_declared_digest_is_served() {
    let digest = json!({"sha256": ABC_SHA256, "size_bytes": 3});
    let package = with_asset(&digest, b"abc").unwrap();
    assert_eq!(package.asset("data/table.bin"), Some(&b"abc"[..]));
    assert_eq!(package.asset("data/other.bin"), None);
}

#[test]
fn digest_case_does_not_matter() {
    let digest = json!({"sha256": ABC_SHA256.to_uppercase(), "size_bytes": 3});
    with_asset(&digest, b"abc").unwrap();
}

#[test]
fn a_flipped_byte_of_the_same_length_fails_the_digest() {
    let digest = json!({"sha256": ABC_SHA256, "size_bytes": 3});
    assert!(matches!(
        with_asset(&digest, b"abd"),
        Err(SyspackError::AssetDigestMismatch { path, declared, actual })
            if path == "data/table.bin" && declared == ABC_SHA256 && actual != ABC_SHA256
    ));
}

#[test]
fn a_wrong_length_fails_before_the_digest() {
    let digest = json!({"sha256": ABC_SHA256, "size_bytes": 4});
    assert!(matches!(
        with_asset(&digest, b"abc"),
        Err(SyspackError::AssetSizeMismatch {
            declared: 4,
            actual: 3,
            ..
        })
    ));
}

#[test]
fn a_declared_asset_the_source_lacks_fails() {
    let mut manifest = downloaded_capsule();
    manifest["assets"]["gone.bin"] = json!({"sha256": ABC_SHA256, "size_bytes": 3});
    assert!(matches!(
        load_json(&manifest),
        Err(SyspackError::AssetMissing { path }) if path == "gone.bin"
    ));
}

#[test]
fn component_cycles_are_typed_by_key() {
    let mut value = downloaded_capsule();
    value["components"] = json!([
        component("a", Some("c"), 0.0, 1.0, 0.0),
        component("b", Some("a"), 0.0, 1.0, 0.0),
        component("c", Some("b"), 0.0, 1.0, 0.0),
    ]);
    value["devices"] = json!([]);
    assert!(matches!(
        load_json(&value),
        Err(SyspackError::ComponentCycle { key }) if key == "a"
    ));

    value["components"] = json!([component("self", Some("self"), 0.0, 1.0, 0.0)]);
    assert!(matches!(
        load_json(&value),
        Err(SyspackError::ComponentCycle { key }) if key == "self"
    ));
}

#[test]
fn non_physical_components_fail() {
    for (field, bad) in [
        ("dry_mass_kg", json!(-1.0)),
        ("rotation_xyzw", json!([0.0, 0.0, 0.0, 2.0])),
        (
            "dry_inertia_body",
            json!([[1.0, 0.5, 0.0], ZERO_INERTIA[1], ZERO_INERTIA[2]]),
        ),
        ("initial_fluid_mass_kg", json!(-1.0)),
    ] {
        let mut value = downloaded_capsule();
        value["components"][1][field] = bad;
        assert!(
            matches!(
                load_json(&value),
                Err(SyspackError::InvalidTree(MbdError::InvalidComponent { .. }))
            ),
            "{field}"
        );
    }
}

#[test]
fn a_tree_with_nothing_attached_fails() {
    let mut value = downloaded_capsule();
    value["components"][0]["attached"] = json!(false);
    assert!(matches!(
        load_json(&value),
        Err(SyspackError::InvalidTree(MbdError::NoAttachedMass))
    ));
}

#[test]
fn unphysical_capsule_parameters_fail_at_load() {
    let mut value = downloaded_capsule();
    value["dynamics"]["RigidBodyBluntCapsule"]["nose_radius_m"] = json!(0.5);
    assert!(matches!(
        load_json(&value),
        Err(SyspackError::InvalidAero(_))
    ));
}

#[test]
fn the_capsule_aero_reads_back_and_evaluates() {
    let package = load_json(&downloaded_capsule()).unwrap();
    assert_eq!(package.capsule_aero().unwrap().params().base_radius_m, 1.0);
    package.build_aero_model().unwrap();
}

#[test]
fn a_vacuum_package_has_an_aero_model_with_no_load() {
    let mut value = downloaded_capsule();
    value["dynamics"] = json!("RigidBodyVacuum");
    let package = load_json(&value).unwrap();
    let flow = mbd_core::AeroFlowState {
        velocity_air_body_m_s: DVec3::X * 100.0,
        angular_rate_body_rad_s: DVec3::ZERO,
        ambient: mbd_core::AmbientFlow {
            pressure_pa: 1.0e5,
            temperature_k: 288.0,
            density_kg_m3: 1.2,
            sound_speed_m_s: 340.0,
        },
        control_surfaces_rad: [0.0; 8],
    };
    let wrench = package.build_aero_model().unwrap().evaluate(&flow).unwrap();
    assert_eq!(wrench.force_body_n, DVec3::ZERO);
    assert!(matches!(
        package.capsule_aero(),
        Err(SyspackError::DynamicsMismatch { .. })
    ));
}

fn ground_station() -> Value {
    json!({
        "schema_version": 1,
        "id": "tracking-station",
        "name": "Tracking station",
        "version": "1.0.0",
        "kind": "GroundStation",
        "assets": {},
        "components": [
            component("mast", None, 0.0, 0.0, 0.0),
            component("receiver_rack", Some("mast"), 0.5, 0.0, 0.0),
        ],
        "dynamics": "None",
        "devices": [
            {"id": "baro", "site": "mast", "kind": {"Bmp390": {"bus_id": 0, "i2c_addr": 118}}},
            {"id": "link", "site": "receiver_rack", "kind": {
                "Arinc429Channel": {"high_speed": true, "gap_bits": 4},
            }},
            {"id": "modem", "site": "receiver_rack", "kind": {"Custom": {
                "type_id": "vendor.telemetry_modem",
                "parameters": {"band": "S"},
            }}},
        ],
    })
}

#[test]
fn a_massless_ground_station_with_mounted_devices_verifies() {
    let package = load_json(&ground_station()).unwrap();
    let manifest = package.manifest();
    assert_eq!(manifest.kind, SystemKind::GroundStation);
    assert_eq!(manifest.dynamics, DynamicsSpec::None);
    assert_eq!(manifest.controls, ControlsSpec::default());
    assert_eq!(manifest.devices.len(), 3);
    assert_eq!(
        package.component_id("receiver_rack").unwrap(),
        ComponentId(1)
    );
    assert_eq!(package.tree().len(), 2);
    assert!(matches!(
        package.tree().mass_properties(),
        Err(MbdError::NoAttachedMass)
    ));
}

#[test]
fn a_test_bench_keeps_its_custom_board_under_the_same_checks() {
    let mut value = ground_station();
    value["kind"] = json!("TestBench");
    value["devices"][2]["site"] = json!("ghost");
    assert!(matches!(
        load_json(&value),
        Err(SyspackError::UnknownDeviceSite { device, .. }) if device == "modem"
    ));
    value["devices"][2]["site"] = json!("mast");
    assert_eq!(
        load_json(&value).unwrap().manifest().kind,
        SystemKind::TestBench
    );
}

#[test]
fn a_stationary_system_has_no_aero_model_and_no_jsbsim_files() {
    let package = load_json(&ground_station()).unwrap();
    for error in [
        package.build_aero_model().err().unwrap(),
        package.capsule_aero().err().unwrap(),
    ] {
        assert!(matches!(
            error,
            SyspackError::DynamicsMismatch {
                declared: "None",
                ..
            }
        ));
    }
    assert!(matches!(
        package.jsbsim_files(),
        Err(SyspackError::DynamicsMismatch {
            needed: "JsbsimXml",
            ..
        })
    ));
    assert!(matches!(
        package.jsbsim_model(),
        Err(SyspackError::DynamicsMismatch { .. })
    ));
}

#[test]
fn a_stationary_system_is_verified_as_strictly_as_a_vehicle() {
    let mut tampered = ground_station();
    tampered["assets"]["firmware.bin"] = json!({"sha256": ABC_SHA256, "size_bytes": 3});
    let assets = BTreeMap::from([("firmware.bin".to_owned(), b"abd".to_vec())]);
    let source = MemoryPackageSource::new(tampered.to_string().into_bytes(), assets);
    assert!(matches!(
        VerifiedSyspack::load(&source),
        Err(SyspackError::AssetDigestMismatch { .. })
    ));

    let mut climbing = ground_station();
    climbing["assets"]["../firmware.bin"] = json!({"sha256": ABC_SHA256, "size_bytes": 3});
    assert!(matches!(
        load_json(&climbing),
        Err(SyspackError::UnsafeAssetPath { .. })
    ));
}

#[test]
fn jsbsim_files_map_declared_assets_to_their_virtual_paths() {
    let mut value = downloaded_capsule();
    value["assets"]["fdm/m.xml"] = json!({"sha256": ABC_SHA256, "size_bytes": 3});
    value["dynamics"] = json!({"JsbsimXml": {
        "aircraft_dir": "aircraft",
        "model_name": "m",
        "entry_asset": "fdm/m.xml",
        "virtual_path": "aircraft/m/m.xml",
        "additional_assets": [["fdm/m.xml", "aircraft/m/copy.xml"]],
    }});
    let assets = BTreeMap::from([("fdm/m.xml".to_owned(), b"abc".to_vec())]);
    let source = MemoryPackageSource::new(value.to_string().into_bytes(), assets);
    let package = VerifiedSyspack::load(&source).unwrap();
    assert_eq!(package.jsbsim_model().unwrap(), ("aircraft", "m"));
    assert_eq!(
        package.jsbsim_files().unwrap(),
        [
            ("aircraft/m/m.xml", &b"abc"[..]),
            ("aircraft/m/copy.xml", &b"abc"[..]),
        ]
    );
    assert!(matches!(
        package.build_aero_model(),
        Err(SyspackError::DynamicsMismatch {
            declared: "JsbsimXml",
            ..
        })
    ));
}
