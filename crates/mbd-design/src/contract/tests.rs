use std::collections::BTreeMap;

use serde_json::{Value, json};
use syspack::{MemoryPackageSource, VerifiedSyspack, sha256_hex};

use super::{FindingSeverity, audit_verified_syspack};

fn build_v1_minimal() -> Value {
    json!({
        "schema_version": 1,
        "id": "v1-craft",
        "name": "V1 Test Craft",
        "version": "1.0.0",
        "kind": "Spacecraft",
        "assets": {},
        "components": [{
            "key": "bus",
            "name": "Bus",
            "parent": null,
            "translation_m": [0.0, 0.0, 0.0],
            "dry_mass_kg": 400.0,
            "dry_inertia_body": [[20.0, 0.0, 0.0], [0.0, 20.0, 0.0], [0.0, 0.0, 20.0]],
            "initial_fluid_mass_kg": 0.0,
            "initial_payload_mass_kg": 0.0,
            "attached": true,
            "thermal_capacity_j_k": 50_000.0,
            "nominal_power_w": 0.0
        }],
        "dynamics": "RigidBodyVacuum",
        "devices": []
    })
}

fn load_with_provenance(manifest: Value, prov_bytes: Option<&[u8]>) -> VerifiedSyspack {
    let mut assets = BTreeMap::new();
    if let Some(bytes) = prov_bytes {
        assets.insert("provenance.json".to_owned(), bytes.to_vec());
    }
    let src = MemoryPackageSource::new(manifest.to_string().into_bytes(), assets);
    VerifiedSyspack::load(&src).unwrap()
}

#[test]
fn audits_v1_and_valid_v2_packages_as_passing() {
    let v1 = load_with_provenance(build_v1_minimal(), None);
    let r1 = audit_verified_syspack(&v1).unwrap();
    assert!(r1.passed());

    let prov = br#"{"spec-1":{"document":"Orbital Tug ICD","tier":"operating_manual"}}"#;
    let mut v2 = build_v1_minimal();
    v2["schema_version"] = json!(2);
    v2["assets"]["provenance.json"] = json!({
        "sha256": sha256_hex(prov),
        "size_bytes": prov.len(),
    });
    v2["subsystems"] = json!({
        "nominal_bus_voltage_v": 28.0,
        "tanks": [{
            "id": 1,
            "site": "bus",
            "center_local_m": [0.0, 0.0, 0.0],
            "geometry": { "Sphere": { "radius_m": 0.5 } },
            "species": "Hydrazine",
            "capacity_mass_kg": 400.0,
            "initial_mass_kg": 350.0,
            "feed_regime": "BladderOrDiaphragm"
        }],
        "provenance_asset": "provenance.json"
    });
    let pack = load_with_provenance(v2, Some(prov));
    let report = audit_verified_syspack(&pack).unwrap();
    assert!(report.passed());
}

#[test]
fn flags_overpacked_tank_exceeding_physical_geometry_volume() {
    let mut v2 = build_v1_minimal();
    v2["schema_version"] = json!(2);
    // Tiny 0.1 m radius sphere (~4.19 L) trying to hold 50 kg hydrazine -> impossible!
    v2["subsystems"] = json!({
        "tanks": [{
            "id": 7,
            "site": "bus",
            "center_local_m": [0.0, 0.0, 0.0],
            "geometry": { "Sphere": { "radius_m": 0.1 } },
            "species": "Hydrazine",
            "capacity_mass_kg": 50.0,
            "initial_mass_kg": 40.0,
            "feed_regime": "BladderOrDiaphragm"
        }]
    });
    let pack = load_with_provenance(v2, None);
    let report = audit_verified_syspack(&pack).unwrap();
    assert!(!report.passed());
    assert!(
        report
            .findings
            .iter()
            .any(|f| { f.code == "TANK_GEOMETRY_CAPACITY" && f.severity == FindingSeverity::Fail })
    );
}
