use std::collections::BTreeMap;

use super::VerifiedSyspack;
use crate::instrument_pack::INSTRUMENTS_ASSET;
use crate::{InstrumentPackDocument, InstrumentPackRef, PackageSource, SyspackError, SystemKind};

#[cfg(test)]
mod tests;

/// Parses the verified `instruments.json` of an instrument pack.
pub(super) fn parse_document(
    assets: &BTreeMap<String, Vec<u8>>,
) -> Result<InstrumentPackDocument, SyspackError> {
    let bytes = assets
        .get(INSTRUMENTS_ASSET)
        .ok_or(SyspackError::InstrumentPackShape {
            detail: "an instrument pack must declare instruments.json",
        })?;
    InstrumentPackDocument::parse(bytes)
}

fn check(field: &'static str, expected: &str, actual: &str) -> Result<(), SyspackError> {
    if expected == actual {
        return Ok(());
    }
    Err(SyspackError::PinMismatch {
        field,
        expected: expected.to_owned(),
        actual: actual.to_owned(),
    })
}

impl VerifiedSyspack {
    /// Loads the instrument pack in `source` and fails unless it is exactly
    /// the pack `pin` names: an instrument pack with that id and version
    /// whose `syspack.json` has that digest.
    pub fn load_pinned(
        source: &dyn PackageSource,
        pin: &InstrumentPackRef,
    ) -> Result<Self, SyspackError> {
        let package = Self::load(source)?;
        check(
            "manifest_sha256",
            &pin.manifest_sha256.to_ascii_lowercase(),
            &package.manifest_sha256,
        )?;
        check("id", &pin.id, &package.manifest.id)?;
        check("version", &pin.version, &package.manifest.version)?;
        if package.manifest.kind != SystemKind::InstrumentPack {
            return Err(SyspackError::PinMismatch {
                field: "kind",
                expected: "InstrumentPack".to_owned(),
                actual: format!("{:?}", package.manifest.kind),
            });
        }
        Ok(package)
    }

    /// SHA-256 of the package's `syspack.json` bytes, lowercase hexadecimal.
    pub fn manifest_sha256(&self) -> &str {
        &self.manifest_sha256
    }

    /// The pin a vehicle manifest needs to use this package as its
    /// instrument pack.
    pub fn instrument_pack_ref(&self) -> InstrumentPackRef {
        InstrumentPackRef {
            id: self.manifest.id.clone(),
            version: self.manifest.version.clone(),
            manifest_sha256: self.manifest_sha256.clone(),
        }
    }

    /// The validated instruments of an instrument pack; `None` for any
    /// other kind of package.
    pub fn instrument_pack(&self) -> Option<&InstrumentPackDocument> {
        self.instrument_pack.as_ref()
    }
}
