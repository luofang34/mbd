use std::collections::BTreeMap;

use serde_json::{Value, json};
use syspack::{MemoryPackageSource, VerifiedSyspack, sha256_hex};

use super::{FindingSeverity, audit_verified_syspack};

fn build_minimal() -> Value {
    json!({
        "schema_version": 3,
        "id": "minimal-craft",
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
        "dynamics": {"MultibodyComposed": {"aero_elements": []}},
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
fn audits_packages_with_and_without_provenance_as_passing() {
    let bare = load_with_provenance(build_minimal(), None);
    let r1 = audit_verified_syspack(&bare).unwrap();
    assert!(r1.passed());
    assert!(
        r1.findings
            .iter()
            .any(|f| { f.code == "PROVENANCE_TABLE" && f.severity == FindingSeverity::Warning })
    );

    let prov = br#"{"spec-1":{"document":"Orbital Tug ICD","tier":"operating_manual"}}"#;
    let mut tug = build_minimal();
    tug["assets"]["provenance.json"] = json!({
        "sha256": sha256_hex(prov),
        "size_bytes": prov.len(),
    });
    tug["subsystems"] = json!({
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
    let pack = load_with_provenance(tug, Some(prov));
    let report = audit_verified_syspack(&pack).unwrap();
    assert!(report.passed());
}

#[test]
fn flags_overpacked_tank_exceeding_physical_geometry_volume() {
    let mut tug = build_minimal();
    // Tiny 0.1 m radius sphere (~4.19 L) trying to hold 50 kg hydrazine -> impossible!
    tug["subsystems"] = json!({
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
    let pack = load_with_provenance(tug, None);
    let report = audit_verified_syspack(&pack).unwrap();
    assert!(!report.passed());
    assert!(
        report
            .findings
            .iter()
            .any(|f| { f.code == "TANK_GEOMETRY_CAPACITY" && f.severity == FindingSeverity::Fail })
    );
}
