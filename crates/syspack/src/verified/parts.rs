use super::VerifiedSyspack;
use crate::{PackageSource, PartRef, SyspackError};

#[cfg(test)]
mod tests;

impl VerifiedSyspack {
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
