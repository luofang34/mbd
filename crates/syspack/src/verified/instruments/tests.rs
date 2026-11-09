use std::collections::BTreeMap;

use serde_json::{Value, json};

use super::*;
use crate::instrument_pack::tests::ga_panel;
use crate::{DeviceKindSpec, MemoryPackageSource, SyspackManifest, sha256_hex};

fn panel_component() -> Value {
    json!({
        "key": "panel",
        "name": "Instrument panel",
        "parent": null,
        "translation_m": [0.0, 0.0, 0.0],
        "dry_mass_kg": 0.0,
        "dry_inertia_body": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        "initial_fluid_mass_kg": 0.0,
        "initial_payload_mass_kg": 0.0,
        "attached": true,
        "thermal_capacity_j_k": 0.0,
        "nominal_power_w": 0.0,
    })
}

/// A pack manifest declaring `instruments` as its instruments.json.
fn pack_manifest(instruments: &[u8]) -> Value {
    json!({
        "schema_version": 1,
        "id": "ga-analog-six-pack",
        "name": "GA analog six-pack",
        "version": "1.0.0",
        "kind": "InstrumentPack",
        "assets": {INSTRUMENTS_ASSET: {
            "sha256": sha256_hex(instruments),
            "size_bytes": instruments.len(),
        }},
        "components": [panel_component()],
        "dynamics": "None",
        "devices": [],
    })
}

fn source(manifest: &Value, instruments: &[u8]) -> MemoryPackageSource {
    let assets = BTreeMap::from([(INSTRUMENTS_ASSET.to_owned(), instruments.to_vec())]);
    MemoryPackageSource::new(manifest.to_string().into_bytes(), assets)
}

fn pack() -> (MemoryPackageSource, Vec<u8>) {
    let instruments = ga_panel().to_string().into_bytes();
    let manifest = pack_manifest(&instruments);
    (
        source(&manifest, &instruments),
        manifest.to_string().into_bytes(),
    )
}

/// A vehicle with a pitot-static system that pins `pin`.
fn vehicle(pin: &InstrumentPackRef) -> Value {
    json!({
        "schema_version": 1,
        "id": "trainer",
        "name": "Trainer",
        "version": "1",
        "kind": "GroundStation",
        "assets": {},
        "components": [panel_component()],
        "dynamics": "None",
        "devices": [{"id": "main", "site": "panel", "kind": {"PitotStatic": {
            "pitot_body_m": [1.2, -4.8, -1.1],
            "static_body_m": [-0.5, -0.3, -0.6],
            "probe": "kollsman_d2_pitot_static_head",
            "total_line_tau_s": 0.05,
            "static_line_tau_s": 0.08,
        }}}],
        "instrument_pack": pin,
    })
}

#[test]
fn a_pack_loads_its_instruments() {
    let (source, manifest_bytes) = pack();
    let package = VerifiedSyspack::load(&source).unwrap();
    let document = package.instrument_pack().unwrap();
    assert_eq!(document.layout.len(), 5);
    assert_eq!(package.manifest_sha256(), sha256_hex(&manifest_bytes));
}

#[test]
fn a_pack_has_a_fixed_shape() {
    let instruments = ga_panel().to_string().into_bytes();
    let mut moving = pack_manifest(&instruments);
    moving["dynamics"] = json!("RigidBodyVacuum");
    let mut empty = pack_manifest(&instruments);
    empty["assets"] = json!({});
    let mut nested = pack_manifest(&instruments);
    nested["instrument_pack"] = json!({
        "id": "other", "version": "1", "manifest_sha256": sha256_hex(b"other"),
    });
    for manifest in [moving, empty, nested] {
        assert!(matches!(
            VerifiedSyspack::load(&source(&manifest, &instruments)),
            Err(SyspackError::InstrumentPackShape { .. })
        ));
    }
}

#[test]
fn a_pack_with_bad_instruments_does_not_load() {
    let mut panel = ga_panel();
    panel["layout"][0]["instrument"] = json!("clock");
    let instruments = panel.to_string().into_bytes();
    let manifest = pack_manifest(&instruments);
    assert!(matches!(
        VerifiedSyspack::load(&source(&manifest, &instruments)),
        Err(SyspackError::UnknownLayoutInstrument { .. })
    ));
}

#[test]
fn a_vehicle_pins_the_pack_by_digest() {
    let (source, _) = pack();
    let pin = VerifiedSyspack::load(&source)
        .unwrap()
        .instrument_pack_ref();
    let manifest = SyspackManifest::parse(vehicle(&pin).to_string().as_bytes()).unwrap();
    assert_eq!(manifest.instrument_pack.as_ref(), Some(&pin));
    assert!(matches!(
        manifest.devices[0].kind,
        DeviceKindSpec::PitotStatic { .. }
    ));
    let pinned = VerifiedSyspack::load_pinned(&source, &pin).unwrap();
    assert!(pinned.instrument_pack().is_some());
}

#[test]
fn a_pin_rejects_any_other_package() {
    let (source, _) = pack();
    let pin = VerifiedSyspack::load(&source)
        .unwrap()
        .instrument_pack_ref();
    let cases = [
        (
            InstrumentPackRef {
                manifest_sha256: sha256_hex(b"tampered"),
                ..pin.clone()
            },
            "manifest_sha256",
        ),
        (
            InstrumentPackRef {
                id: "other".to_owned(),
                ..pin.clone()
            },
            "id",
        ),
        (
            InstrumentPackRef {
                version: "2.0.0".to_owned(),
                ..pin.clone()
            },
            "version",
        ),
    ];
    for (wrong, field) in cases {
        match VerifiedSyspack::load_pinned(&source, &wrong) {
            Err(SyspackError::PinMismatch { field: found, .. }) => assert_eq!(found, field),
            other => panic!("expected a {field} mismatch, got {other:?}"),
        }
    }
}

#[test]
fn a_pin_rejects_a_vehicle_with_the_right_digest() {
    let pin = InstrumentPackRef {
        id: "ga-analog-six-pack".to_owned(),
        version: "1.0.0".to_owned(),
        manifest_sha256: sha256_hex(b"x"),
    };
    let mut manifest = vehicle(&pin);
    manifest["id"] = json!("ga-analog-six-pack");
    manifest["version"] = json!("1.0.0");
    manifest.as_object_mut().unwrap().remove("instrument_pack");
    let bytes = manifest.to_string().into_bytes();
    let source = MemoryPackageSource::new(bytes.clone(), BTreeMap::new());
    let wrong = InstrumentPackRef {
        manifest_sha256: sha256_hex(&bytes),
        ..pin
    };
    assert!(matches!(
        VerifiedSyspack::load_pinned(&source, &wrong),
        Err(SyspackError::PinMismatch { field: "kind", .. })
    ));
    let vehicle = VerifiedSyspack::load(&source).unwrap();
    assert!(vehicle.instrument_pack().is_none());
}

#[test]
fn a_malformed_pin_or_pitot_static_fails_validation() {
    let pin = InstrumentPackRef {
        id: "pack".to_owned(),
        version: "1".to_owned(),
        manifest_sha256: "abc".to_owned(),
    };
    let parse = |value: &Value| SyspackManifest::parse(value.to_string().as_bytes());
    assert!(matches!(
        parse(&vehicle(&pin)),
        Err(SyspackError::MalformedDigest { .. })
    ));
    let good = InstrumentPackRef {
        manifest_sha256: sha256_hex(b"x"),
        ..pin
    };
    let cases = [
        ("/devices/0/kind/PitotStatic/probe", json!(""), "probe"),
        (
            "/devices/0/kind/PitotStatic/static_line_tau_s",
            json!(-0.1),
            "static_line_tau_s",
        ),
    ];
    for (pointer, replacement, field) in cases {
        let mut value = vehicle(&good);
        *value.pointer_mut(pointer).unwrap() = replacement;
        match parse(&value) {
            Err(SyspackError::InvalidDevice { field: found, .. }) => assert_eq!(found, field),
            other => panic!("expected an invalid {field}, got {other:?}"),
        }
    }
}

#[test]
fn a_manifest_without_a_pin_serializes_as_before() {
    let (source, _) = pack();
    let package = VerifiedSyspack::load(&source).unwrap();
    let text = serde_json::to_string(package.manifest()).unwrap();
    assert!(!text.contains("instrument_pack"));
}
