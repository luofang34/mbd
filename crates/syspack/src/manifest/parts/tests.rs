use serde_json::{Value, json};

use crate::{SyspackError, SyspackManifest, SystemKind};

fn component() -> Value {
    json!({
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
    })
}

fn digest() -> Value {
    json!({"sha256": "0".repeat(64), "size_bytes": 0})
}

fn engine_part() -> Value {
    json!({
        "schema_version": 1,
        "id": "mitsubishi-kasei-11",
        "name": "Mitsubishi Kasei 11",
        "version": "1.0.0",
        "kind": "Engine",
        "assets": {"engine.json": digest()},
        "components": [component()],
        "dynamics": "None",
        "devices": [],
    })
}

fn pin(role: &str) -> Value {
    json!({
        "role": role,
        "id": "mitsubishi-kasei-11",
        "version": "1.0.0",
        "manifest_sha256": "a".repeat(64),
    })
}

fn vehicle(parts: Value) -> Value {
    json!({
        "schema_version": 1,
        "id": "twin",
        "name": "Twin",
        "version": "1.0.0",
        "kind": "FixedWing",
        "assets": {},
        "components": [component()],
        "dynamics": "None",
        "devices": [],
        "parts": parts,
    })
}

fn parse(value: &Value) -> Result<SyspackManifest, SyspackError> {
    SyspackManifest::parse(value.to_string().as_bytes())
}

#[test]
fn engine_and_propeller_parts_parse_with_their_documents() {
    assert_eq!(parse(&engine_part()).unwrap().kind, SystemKind::Engine);
    let mut propeller = engine_part();
    propeller["kind"] = json!("Propeller");
    propeller["assets"] = json!({"propeller.json": digest()});
    assert_eq!(parse(&propeller).unwrap().kind, SystemKind::Propeller);
    assert!(SystemKind::Engine.is_part() && SystemKind::Propeller.is_part());
    assert!(!SystemKind::InstrumentPack.is_part() && !SystemKind::FixedWing.is_part());
}

#[test]
fn a_part_must_be_a_static_holder_of_its_document() {
    let mut no_document = engine_part();
    no_document["assets"] = json!({"propeller.json": digest()});
    assert!(matches!(
        parse(&no_document),
        Err(SyspackError::PartDocumentMissing {
            document: "engine.json",
            ..
        })
    ));
    let mut moving = engine_part();
    moving["dynamics"] = json!("RigidBodyVacuum");
    assert!(matches!(
        parse(&moving),
        Err(SyspackError::PartShape { .. })
    ));
    let mut nested = engine_part();
    nested["parts"] = json!([pin("engine0")]);
    assert!(matches!(
        parse(&nested),
        Err(SyspackError::PartShape { .. })
    ));
    let mut with_pack = engine_part();
    with_pack["instrument_pack"] = json!({
        "id": "ga-analog-six-pack",
        "version": "1.0.0",
        "manifest_sha256": "a".repeat(64),
    });
    assert!(matches!(
        parse(&with_pack),
        Err(SyspackError::PartShape { .. })
    ));
}

#[test]
fn a_vehicle_pins_parts_in_distinct_slug_roles() {
    let manifest = parse(&vehicle(json!([pin("engine0"), pin("engine1")]))).unwrap();
    assert_eq!(manifest.parts.len(), 2);
    assert!(matches!(
        parse(&vehicle(json!([pin("engine0"), pin("engine0")]))),
        Err(SyspackError::DuplicatePartRole { role }) if role == "engine0"
    ));
    assert!(matches!(
        parse(&vehicle(json!([pin("Engine 0")]))),
        Err(SyspackError::InvalidPartRole { .. })
    ));
    let mut bad_digest = pin("engine0");
    bad_digest["manifest_sha256"] = json!("xyz");
    assert!(matches!(
        parse(&vehicle(json!([bad_digest]))),
        Err(SyspackError::MalformedDigest { .. })
    ));
    let mut bad_id = pin("engine0");
    bad_id["id"] = json!("Kasei 11");
    assert!(matches!(
        parse(&vehicle(json!([bad_id]))),
        Err(SyspackError::InvalidPackageId { .. })
    ));
    let mut extra = pin("engine0");
    extra["serial"] = json!("1");
    assert!(matches!(
        parse(&vehicle(json!([extra]))),
        Err(SyspackError::ManifestParse(_))
    ));
}

#[test]
fn a_manifest_without_parts_serialises_without_them() {
    let manifest = parse(&engine_part()).unwrap();
    let text = serde_json::to_string(&manifest).unwrap();
    assert!(!text.contains("\"parts\""), "{text}");
}
