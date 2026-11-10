use std::collections::BTreeMap;

use super::VerifiedSyspack;
use crate::{
    EngineDocument, PackageSource, PartDocument, PartRef, PropellerDocument, SyspackError,
    SyspackManifest, SystemKind,
};

#[cfg(test)]
mod tests;

/// Parses the verified document of a part package and checks that it
/// describes this package; `None` for a package that is not a part.
pub(super) fn parse_document(
    manifest: &SyspackManifest,
    assets: &BTreeMap<String, Vec<u8>>,
) -> Result<Option<PartDocument>, SyspackError> {
    let Some(asset) = manifest.kind.part_document() else {
        return Ok(None);
    };
    let bytes = assets
        .get(asset)
        .ok_or_else(|| SyspackError::PartDocumentMissing {
            kind: format!("{:?}", manifest.kind),
            document: asset,
        })?;
    let document = match manifest.kind {
        SystemKind::Engine => PartDocument::Engine(Box::new(EngineDocument::parse(bytes)?)),
        SystemKind::Propeller => {
            PartDocument::Propeller(Box::new(PropellerDocument::parse(bytes)?))
        }
        _ => return Ok(None),
    };
    if document.id() != manifest.id {
        return Err(SyspackError::PartDocumentId {
            document: asset,
            document_id: document.id().to_owned(),
            package_id: manifest.id.clone(),
        });
    }
    Ok(Some(document))
}

impl VerifiedSyspack {
    /// The engine this package describes; `None` unless it is an engine part.
    pub fn engine_document(&self) -> Option<&EngineDocument> {
        match &self.part {
            Some(PartDocument::Engine(document)) => Some(document),
            _ => None,
        }
    }

    /// The propeller this package describes; `None` unless it is a propeller
    /// part.
    pub fn propeller_document(&self) -> Option<&PropellerDocument> {
        match &self.part {
            Some(PartDocument::Propeller(document)) => Some(document),
            _ => None,
        }
    }

    /// Loads the part in `source` and fails unless it is exactly the part
    /// `pin` names: a part package with that id and version whose
    /// `syspack.json` has that digest.
    pub fn load_part(source: &dyn PackageSource, pin: &PartRef) -> Result<Self, SyspackError> {
        let package = Self::load(source)?;
        let mismatch = |field, expected: &str, actual: &str| SyspackError::PartPinMismatch {
            role: pin.role.clone(),
            field,
            expected: expected.to_owned(),
            actual: actual.to_owned(),
        };
        let expected_digest = pin.manifest_sha256.to_ascii_lowercase();
        for (field, expected, actual) in [
            (
                "manifest_sha256",
                expected_digest.as_str(),
                package.manifest_sha256(),
            ),
            ("id", pin.id.as_str(), package.manifest().id.as_str()),
            (
                "version",
                pin.version.as_str(),
                package.manifest().version.as_str(),
            ),
        ] {
            if expected != actual {
                return Err(mismatch(field, expected, actual));
            }
        }
        if !package.manifest().kind.is_part() {
            return Err(mismatch(
                "kind",
                "a part kind",
                &format!("{:?}", package.manifest().kind),
            ));
        }
        Ok(package)
    }

    /// The pin a vehicle manifest needs to install this package in `role`.
    pub fn part_ref(&self, role: &str) -> PartRef {
        PartRef {
            role: role.to_owned(),
            id: self.manifest().id.clone(),
            version: self.manifest().version.clone(),
            manifest_sha256: self.manifest_sha256().to_owned(),
        }
    }
}
