use std::collections::BTreeMap;

use serde_json::{Value, json};

use crate::{MemoryPackageSource, PartRef, SyspackError, VerifiedSyspack, sha256_hex};

const DOCUMENT: &[u8] = b"{\"ratings\": []}";

fn part_manifest(kind: &str, document: &str) -> Value {
    json!({
        "schema_version": 1,
        "id": "mitsubishi-kasei-11",
        "name": "Mitsubishi Kasei 11",
        "version": "1.0.0",
        "kind": kind,
        "assets": {document: {"sha256": sha256_hex(DOCUMENT), "size_bytes": DOCUMENT.len()}},
        "components": [{
            "key": "case",
            "name": "Engine",
            "parent": null,
            "translation_m": [0.0, 0.0, 0.0],
            "dry_mass_kg": 725.0,
            "dry_inertia_body": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            "initial_fluid_mass_kg": 0.0,
            "initial_payload_mass_kg": 0.0,
            "attached": true,
            "thermal_capacity_j_k": 0.0,
            "nominal_power_w": 0.0,
        }],
        "dynamics": "None",
        "devices": [],
    })
}

fn source(manifest: &Value, document: &str) -> (MemoryPackageSource, Vec<u8>) {
    let bytes = manifest.to_string().into_bytes();
    let assets = BTreeMap::from([(document.to_owned(), DOCUMENT.to_vec())]);
    (MemoryPackageSource::new(bytes.clone(), assets), bytes)
}

fn engine() -> (MemoryPackageSource, Vec<u8>) {
    source(&part_manifest("Engine", "engine.json"), "engine.json")
}

#[test]
fn a_part_loads_against_its_own_pin_in_any_role() {
    let (source, bytes) = engine();
    let loaded = VerifiedSyspack::load(&source).unwrap();
    let pin = loaded.part_ref("engine1");
    assert_eq!(pin.manifest_sha256, sha256_hex(&bytes));
    let part = VerifiedSyspack::load_part(&source, &pin).unwrap();
    assert_eq!(part.asset("engine.json"), Some(DOCUMENT));
}

#[test]
fn a_different_revision_id_or_version_is_refused_by_role() {
    let (source, _) = engine();
    let good = VerifiedSyspack::load(&source).unwrap().part_ref("engine0");
    for (field, pin) in [
        (
            "manifest_sha256",
            PartRef {
                manifest_sha256: "b".repeat(64),
                ..good.clone()
            },
        ),
        (
            "id",
            PartRef {
                id: "mitsubishi-kasei-15".to_owned(),
                ..good.clone()
            },
        ),
        (
            "version",
            PartRef {
                version: "1.0.1".to_owned(),
                ..good.clone()
            },
        ),
    ] {
        let pin = if field == "manifest_sha256" {
            pin
        } else {
            PartRef {
                manifest_sha256: good.manifest_sha256.clone(),
                ..pin
            }
        };
        assert!(matches!(
            VerifiedSyspack::load_part(&source, &pin),
            Err(SyspackError::PartPinMismatch { role, field: f, .. }) if role == "engine0" && f == field
        ));
    }
}

#[test]
fn a_package_that_is_not_a_part_is_refused() {
    let mut manifest = part_manifest("GenericSystem", "engine.json");
    manifest["id"] = json!("mitsubishi-kasei-11");
    let (source, _) = source(&manifest, "engine.json");
    let pin = VerifiedSyspack::load(&source).unwrap().part_ref("engine0");
    assert!(matches!(
        VerifiedSyspack::load_part(&source, &pin),
        Err(SyspackError::PartPinMismatch { field: "kind", .. })
    ));
}
