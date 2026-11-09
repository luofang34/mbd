use std::collections::BTreeMap;

use serde_json::json;

use crate::{MemoryPackageSource, SyspackError, TypefacePin, VerifiedSyspack, sha256_hex};

const FACE: &[u8] = b"face bytes";
const PROVENANCE: &[u8] = b"provenance";

fn source() -> (MemoryPackageSource, String) {
    let asset = |bytes: &[u8]| json!({"sha256": sha256_hex(bytes), "size_bytes": bytes.len()});
    let manifest = json!({
        "schema_version": 1,
        "id": "us-ms33558",
        "name": "MS33558 lettering",
        "version": "1.0.0",
        "kind": "TypefacePack",
        "assets": {"face.idtf": asset(FACE), "PROVENANCE.md": asset(PROVENANCE)},
        "components": [],
        "dynamics": "None",
        "devices": [],
        "typeface": {
            "face_asset": "face.idtf",
            "provenance_asset": "PROVENANCE.md",
            "licence": "LicenseRef-USGov-PD",
        },
    })
    .to_string()
    .into_bytes();
    let digest = sha256_hex(&manifest);
    let assets = BTreeMap::from([
        ("face.idtf".to_owned(), FACE.to_vec()),
        ("PROVENANCE.md".to_owned(), PROVENANCE.to_vec()),
    ]);
    (MemoryPackageSource::new(manifest, assets), digest)
}

fn pin(digest: &str) -> TypefacePin {
    TypefacePin {
        id: "us-ms33558".to_owned(),
        version: "1.0.0".to_owned(),
        manifest_sha256: digest.to_owned(),
    }
}

#[test]
fn a_pinned_typeface_loads_and_yields_its_verified_face() {
    let (source, digest) = source();
    let pack = VerifiedSyspack::load_typeface(&source, &pin(&digest)).unwrap();
    assert_eq!(pack.typeface_face(), Some(FACE));
}

#[test]
fn a_stale_pin_refuses_the_typeface() {
    let (source, _) = source();
    let stale = pin(&"0".repeat(64));
    assert!(matches!(
        VerifiedSyspack::load_typeface(&source, &stale),
        Err(SyspackError::PinMismatch {
            field: "manifest_sha256",
            ..
        })
    ));
}
