use serde_json::{Value, json};

use crate::{SyspackError, SyspackManifest, SystemKind};

const SHA: &str = "0000000000000000000000000000000000000000000000000000000000000000";

fn typeface_pack() -> Value {
    json!({
        "schema_version": 1,
        "id": "us-ms33558",
        "name": "MS33558 instrument dial lettering",
        "version": "1.0.0",
        "kind": "TypefacePack",
        "assets": {
            "us-ms33558.idtf": {"sha256": SHA, "size_bytes": 10},
            "PROVENANCE.md": {"sha256": SHA, "size_bytes": 10},
        },
        "components": [],
        "dynamics": "None",
        "devices": [],
        "typeface": {
            "face_asset": "us-ms33558.idtf",
            "provenance_asset": "PROVENANCE.md",
            "licence": "LicenseRef-USGov-PD",
            "upstream": [{"name": "MS33558 (ASG), revision C", "url": "https://everyspec.com/MS-Specs/MS3/MS33000-MS33999/MS33558C_13545/"}],
        },
    })
}

fn parse(value: &Value) -> Result<SyspackManifest, SyspackError> {
    SyspackManifest::parse(value.to_string().as_bytes())
}

#[test]
fn a_typeface_pack_parses_with_no_components() {
    let manifest = parse(&typeface_pack()).unwrap();
    assert_eq!(manifest.kind, SystemKind::TypefacePack);
    assert_eq!(manifest.typeface.unwrap().face_asset, "us-ms33558.idtf");
}

/// A change that breaks a pack.
type Break = fn(&mut Value);

#[test]
fn rejects_typeface_packs_of_the_wrong_shape() {
    let cases: [(&str, Break); 5] = [
        ("no typeface", |v| {
            v.as_object_mut().map(|o| o.remove("typeface"));
        }),
        ("undeclared face", |v| {
            v["typeface"]["face_asset"] = json!("missing.idtf")
        }),
        ("a component", |v| {
            v["components"] = json!([{"key": "body", "name": "x", "parent": null,
                "translation_m": [0.0, 0.0, 0.0], "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "dry_mass_kg": 1.0, "dry_inertia_body": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                "initial_fluid_mass_kg": 0.0, "initial_payload_mass_kg": 0.0, "attached": true,
                "thermal_capacity_j_k": 0.0, "nominal_power_w": 0.0}]);
        }),
        ("plain http", |v| {
            v["typeface"]["upstream"][0]["url"] = json!("http://example.com")
        }),
        ("not a typeface pack", |v| {
            v["kind"] = json!("InstrumentPack")
        }),
    ];
    for (case, break_it) in cases {
        let mut value = typeface_pack();
        break_it(&mut value);
        assert!(parse(&value).is_err(), "{case}");
    }
}
