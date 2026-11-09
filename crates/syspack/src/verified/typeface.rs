use super::VerifiedSyspack;
use super::instruments::check;
use crate::{PackageSource, SyspackError, SystemKind, TypefacePin};

#[cfg(test)]
mod tests;

impl VerifiedSyspack {
    /// Loads the typeface pack in `source` and fails unless it is exactly
    /// the pack `pin` names: a typeface pack with that id and version whose
    /// `syspack.json` has that digest.
    pub fn load_typeface(
        source: &dyn PackageSource,
        pin: &TypefacePin,
    ) -> Result<Self, SyspackError> {
        let package = Self::load(source)?;
        check(
            "manifest_sha256",
            &pin.manifest_sha256.to_ascii_lowercase(),
            &package.manifest_sha256,
        )?;
        check("id", &pin.id, &package.manifest.id)?;
        check("version", &pin.version, &package.manifest.version)?;
        if package.manifest.kind != SystemKind::TypefacePack {
            return Err(SyspackError::PinMismatch {
                field: "kind",
                expected: "TypefacePack".to_owned(),
                actual: format!("{:?}", package.manifest.kind),
            });
        }
        Ok(package)
    }

    /// A typeface pack's face bytes, verified against the manifest.
    #[must_use]
    pub fn typeface_face(&self) -> Option<&[u8]> {
        let spec = self.manifest.typeface.as_ref()?;
        self.asset(&spec.face_asset)
    }
}
