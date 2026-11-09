use std::collections::BTreeSet;

use super::validate::{is_sha256_hex, is_slug, require};
use super::{DynamicsSpec, PartRef, SyspackManifest};
use crate::SyspackError;

#[cfg(test)]
mod tests;

impl SyspackManifest {
    /// A part must be a static holder of its document; a vehicle's part pins
    /// must name parts it could match, each in its own role.
    pub(super) fn validate_parts(&self) -> Result<(), SyspackError> {
        if let Some(document) = self.kind.part_document() {
            let shape = |detail| Err(SyspackError::PartShape { detail });
            if self.dynamics != DynamicsSpec::None {
                return shape("a part must declare dynamics None");
            }
            if self.instrument_pack.is_some() {
                return shape("a part cannot pin an instrument pack");
            }
            if !self.parts.is_empty() {
                return shape("a part cannot pin other parts");
            }
            if !self.assets.contains_key(document) {
                return Err(SyspackError::PartDocumentMissing {
                    kind: format!("{:?}", self.kind),
                    document,
                });
            }
        }
        let mut roles = BTreeSet::new();
        for pin in &self.parts {
            check_part_pin(pin)?;
            if !roles.insert(pin.role.as_str()) {
                return Err(SyspackError::DuplicatePartRole {
                    role: pin.role.clone(),
                });
            }
        }
        Ok(())
    }
}

fn check_part_pin(pin: &PartRef) -> Result<(), SyspackError> {
    if !is_slug(&pin.role) {
        return Err(SyspackError::InvalidPartRole {
            role: pin.role.clone(),
        });
    }
    if !is_slug(&pin.id) {
        return Err(SyspackError::InvalidPackageId { id: pin.id.clone() });
    }
    require("part version", &pin.version)?;
    if !is_sha256_hex(&pin.manifest_sha256) {
        return Err(SyspackError::MalformedDigest {
            path: format!("part {} in role {}", pin.id, pin.role),
            sha256: pin.manifest_sha256.clone(),
        });
    }
    Ok(())
}
