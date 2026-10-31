use std::io::ErrorKind;
use std::path::{Path, PathBuf};

use super::{MANIFEST_FILE, PackageSource};
use crate::SyspackError;
use crate::manifest::validate::check_asset_path;

/// A package unpacked on disk: `<dir>/syspack.json` and `<dir>/<asset>`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DirectoryPackageSource {
    dir: PathBuf,
}

impl DirectoryPackageSource {
    /// A package in `dir`. Nothing is read until the package is loaded.
    pub fn new(dir: impl Into<PathBuf>) -> Self {
        Self { dir: dir.into() }
    }

    /// Refuses a symlink anywhere under the package, which could point a
    /// checked path outside it.
    fn check_no_symlink(&self, rel_path: &str) -> Result<(), SyspackError> {
        let read_error = |source: std::io::Error| match source.kind() {
            ErrorKind::NotFound => SyspackError::AssetMissing {
                path: rel_path.to_owned(),
            },
            _ => SyspackError::AssetRead {
                path: rel_path.to_owned(),
                source,
            },
        };
        let mut current = self.dir.clone();
        for segment in Path::new(rel_path) {
            current.push(segment);
            let metadata = std::fs::symlink_metadata(&current).map_err(read_error)?;
            if metadata.file_type().is_symlink() {
                return Err(SyspackError::UnsafeAssetPath {
                    path: rel_path.to_owned(),
                    reason: "it passes through a symbolic link",
                });
            }
        }
        Ok(())
    }
}

impl PackageSource for DirectoryPackageSource {
    fn read_manifest_bytes(&self) -> Result<Vec<u8>, SyspackError> {
        let path = self.dir.join(MANIFEST_FILE);
        std::fs::read(&path).map_err(|source| SyspackError::ManifestRead {
            path: path.display().to_string(),
            source,
        })
    }

    fn read_asset_bytes(&self, rel_path: &str) -> Result<Vec<u8>, SyspackError> {
        check_asset_path(rel_path)?;
        self.check_no_symlink(rel_path)?;
        std::fs::read(self.dir.join(rel_path)).map_err(|source| SyspackError::AssetRead {
            path: rel_path.to_owned(),
            source,
        })
    }
}
