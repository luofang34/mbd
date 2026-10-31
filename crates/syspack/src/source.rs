use std::borrow::Cow;
use std::collections::BTreeMap;

use crate::SyspackError;

#[cfg(not(target_arch = "wasm32"))]
mod directory;

#[cfg(not(target_arch = "wasm32"))]
pub use directory::DirectoryPackageSource;

#[cfg(test)]
mod tests;

/// Name of the manifest at the root of every package.
pub const MANIFEST_FILE: &str = "syspack.json";

/// Where a package's bytes come from. Sources are untrusted: everything they
/// return is verified by [`crate::VerifiedSyspack::load`].
pub trait PackageSource {
    /// The content of `syspack.json`.
    fn read_manifest_bytes(&self) -> Result<Vec<u8>, SyspackError>;

    /// The content of the asset at relative POSIX path `rel_path`.
    fn read_asset_bytes(&self, rel_path: &str) -> Result<Vec<u8>, SyspackError>;
}

/// A package held in memory: received over the network, or embedded in a
/// binary with no I/O at all.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MemoryPackageSource {
    manifest: Cow<'static, [u8]>,
    assets: BTreeMap<String, Cow<'static, [u8]>>,
}

impl MemoryPackageSource {
    /// A package whose bytes were received at run time.
    pub fn new(manifest: Vec<u8>, assets: BTreeMap<String, Vec<u8>>) -> Self {
        Self {
            manifest: Cow::Owned(manifest),
            assets: assets
                .into_iter()
                .map(|(path, bytes)| (path, Cow::Owned(bytes)))
                .collect(),
        }
    }

    /// A package compiled into a binary, borrowing its `include_bytes!`
    /// slices.
    pub fn from_embedded(manifest: &'static [u8], assets: &[(&str, &'static [u8])]) -> Self {
        Self {
            manifest: Cow::Borrowed(manifest),
            assets: assets
                .iter()
                .map(|&(path, bytes)| (path.to_owned(), Cow::Borrowed(bytes)))
                .collect(),
        }
    }
}

impl PackageSource for MemoryPackageSource {
    fn read_manifest_bytes(&self) -> Result<Vec<u8>, SyspackError> {
        Ok(self.manifest.clone().into_owned())
    }

    fn read_asset_bytes(&self, rel_path: &str) -> Result<Vec<u8>, SyspackError> {
        self.assets
            .get(rel_path)
            .map(|bytes| bytes.clone().into_owned())
            .ok_or_else(|| SyspackError::AssetMissing {
                path: rel_path.to_owned(),
            })
    }
}
