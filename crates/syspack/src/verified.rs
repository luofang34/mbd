use std::collections::BTreeMap;

use mbd_core::{ComponentId, ComponentTree};
use sha2::{Digest, Sha256};

use crate::{
    DynamicsSpec, InstrumentPackDocument, PackageSource, PartDocument, SyspackError,
    SyspackManifest, SystemKind,
};

mod dynamics;
mod instruments;
mod parts;
mod tree;
mod typeface;

pub use tree::INITIAL_COMPONENT_TEMPERATURE_K;

#[cfg(test)]
mod tests;

/// A fully verified system package. Holding one proves every declared asset
/// matched its digest and every part of the manifest can be built.
#[derive(Debug, Clone, PartialEq)]
pub struct VerifiedSyspack {
    manifest: SyspackManifest,
    assets: BTreeMap<String, Vec<u8>>,
    tree: ComponentTree,
    component_ids: BTreeMap<String, ComponentId>,
    manifest_sha256: String,
    instrument_pack: Option<InstrumentPackDocument>,
    part: Option<PartDocument>,
}

/// SHA-256 of `bytes` as lowercase hexadecimal: the digest a vehicle pins an
/// instrument pack's `syspack.json` with, and the one asset entries declare.
pub fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn read_verified_assets(
    source: &dyn PackageSource,
    manifest: &SyspackManifest,
) -> Result<BTreeMap<String, Vec<u8>>, SyspackError> {
    let mut assets = BTreeMap::new();
    for (path, declared) in &manifest.assets {
        let bytes = source.read_asset_bytes(path)?;
        let actual = bytes.len() as u64;
        if actual != declared.size_bytes {
            return Err(SyspackError::AssetSizeMismatch {
                path: path.clone(),
                declared: declared.size_bytes,
                actual,
            });
        }
        let digest = sha256_hex(&bytes);
        if !digest.eq_ignore_ascii_case(&declared.sha256) {
            return Err(SyspackError::AssetDigestMismatch {
                path: path.clone(),
                declared: declared.sha256.clone(),
                actual: digest,
            });
        }
        assets.insert(path.clone(), bytes);
    }
    Ok(assets)
}

impl VerifiedSyspack {
    /// Reads, verifies and builds the package in `source`, failing on the
    /// first problem found.
    pub fn load(source: &dyn PackageSource) -> Result<Self, SyspackError> {
        let bytes = source.read_manifest_bytes()?;
        let manifest = SyspackManifest::parse(&bytes)?;
        let assets = read_verified_assets(source, &manifest)?;
        let (tree, component_ids) = tree::build_tree(&manifest)?;
        let instrument_pack = if manifest.kind == SystemKind::InstrumentPack {
            Some(instruments::parse_document(&assets)?)
        } else {
            None
        };
        let part = parts::parse_document(&manifest, &assets)?;
        let package = Self {
            manifest,
            assets,
            tree,
            component_ids,
            manifest_sha256: sha256_hex(&bytes),
            instrument_pack,
            part,
        };
        if matches!(
            package.manifest.dynamics,
            DynamicsSpec::RigidBodyVacuum | DynamicsSpec::RigidBodyBluntCapsule { .. }
        ) {
            package.build_aero_model()?;
        }
        Ok(package)
    }

    /// The manifest.
    pub fn manifest(&self) -> &SyspackManifest {
        &self.manifest
    }

    /// The verified bytes of the asset at `rel_path`, if it is declared.
    pub fn asset(&self, rel_path: &str) -> Option<&[u8]> {
        self.assets.get(rel_path).map(Vec::as_slice)
    }

    /// The component tree in its initial state.
    pub fn tree(&self) -> &ComponentTree {
        &self.tree
    }

    /// The id of the component with `key`.
    pub fn component_id(&self, key: &str) -> Result<ComponentId, SyspackError> {
        self.component_ids
            .get(key)
            .copied()
            .ok_or_else(|| SyspackError::UnknownComponent {
                key: key.to_owned(),
            })
    }
}
