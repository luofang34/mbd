use std::collections::BTreeSet;

use super::{DeviceKindSpec, DynamicsSpec, MountedDeviceSpec, SCHEMA_VERSION, SyspackManifest};
use crate::SyspackError;

const SHA256_HEX_DIGITS: usize = 64;

/// Rejects any path that is not a plain relative POSIX path inside the package.
pub(crate) fn check_asset_path(path: &str) -> Result<(), SyspackError> {
    let unsafe_path = |reason| {
        Err(SyspackError::UnsafeAssetPath {
            path: path.to_owned(),
            reason,
        })
    };
    if path.is_empty() {
        return unsafe_path("it is empty");
    }
    if path.starts_with('/') {
        return unsafe_path("it is absolute");
    }
    if path.contains('\\') {
        return unsafe_path("it contains a backslash");
    }
    for segment in path.split('/') {
        match segment {
            "" => return unsafe_path("it has an empty segment"),
            "." | ".." => return unsafe_path("it has a dot segment"),
            _ => {}
        }
    }
    Ok(())
}

fn is_slug(id: &str) -> bool {
    !id.is_empty()
        && id
            .chars()
            .all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || c == '-' || c == '_')
}

fn require(field: &'static str, value: &str) -> Result<(), SyspackError> {
    if value.is_empty() {
        return Err(SyspackError::EmptyField { field });
    }
    Ok(())
}

impl SyspackManifest {
    /// Checks everything that can be judged from the manifest alone.
    pub(super) fn validate(&self) -> Result<(), SyspackError> {
        if self.schema_version != SCHEMA_VERSION {
            return Err(SyspackError::UnsupportedSchema {
                found: self.schema_version,
                expected: SCHEMA_VERSION,
            });
        }
        if !is_slug(&self.id) {
            return Err(SyspackError::InvalidPackageId {
                id: self.id.clone(),
            });
        }
        require("name", &self.name)?;
        require("version", &self.version)?;
        self.validate_assets()?;
        self.validate_dynamics()?;
        let keys = self.validate_components()?;
        self.validate_devices(&keys)
    }

    fn validate_assets(&self) -> Result<(), SyspackError> {
        for (path, digest) in &self.assets {
            check_asset_path(path)?;
            let hexadecimal = digest.sha256.len() == SHA256_HEX_DIGITS
                && digest.sha256.chars().all(|c| c.is_ascii_hexdigit());
            if !hexadecimal {
                return Err(SyspackError::MalformedDigest {
                    path: path.clone(),
                    sha256: digest.sha256.clone(),
                });
            }
        }
        Ok(())
    }

    fn validate_dynamics(&self) -> Result<(), SyspackError> {
        let DynamicsSpec::JsbsimXml {
            aircraft_dir,
            model_name,
            entry_asset,
            virtual_path,
            additional_assets,
        } = &self.dynamics
        else {
            return Ok(());
        };
        let expected = format!("{aircraft_dir}/{model_name}/{model_name}.xml");
        if *virtual_path != expected {
            return Err(SyspackError::JsbsimEntryPath {
                expected,
                actual: virtual_path.clone(),
            });
        }
        check_asset_path(virtual_path)?;
        self.check_declared(entry_asset)?;
        for (asset, destination) in additional_assets {
            self.check_declared(asset)?;
            check_asset_path(destination)?;
        }
        Ok(())
    }

    fn check_declared(&self, path: &str) -> Result<(), SyspackError> {
        if self.assets.contains_key(path) {
            return Ok(());
        }
        Err(SyspackError::UndeclaredAsset {
            path: path.to_owned(),
        })
    }

    fn validate_components(&self) -> Result<BTreeSet<&str>, SyspackError> {
        if self.components.is_empty() {
            return Err(SyspackError::NoComponents);
        }
        let mut keys = BTreeSet::new();
        for component in &self.components {
            require("component key", &component.key)?;
            if !keys.insert(component.key.as_str()) {
                return Err(SyspackError::DuplicateComponent {
                    key: component.key.clone(),
                });
            }
        }
        for component in &self.components {
            if let Some(parent) = &component.parent
                && !keys.contains(parent.as_str())
            {
                return Err(SyspackError::UnknownParent {
                    component: component.key.clone(),
                    parent: parent.clone(),
                });
            }
        }
        Ok(keys)
    }

    fn validate_devices(&self, components: &BTreeSet<&str>) -> Result<(), SyspackError> {
        let mut ids = BTreeSet::new();
        for device in &self.devices {
            require("device id", &device.id)?;
            if !ids.insert(device.id.as_str()) {
                return Err(SyspackError::DuplicateDevice {
                    id: device.id.clone(),
                });
            }
            if !components.contains(device.site.as_str()) {
                return Err(SyspackError::UnknownDeviceSite {
                    device: device.id.clone(),
                    site: device.site.clone(),
                });
            }
            check_custom(device)?;
        }
        Ok(())
    }
}

fn check_custom(device: &MountedDeviceSpec) -> Result<(), SyspackError> {
    match &device.kind {
        DeviceKindSpec::Custom { type_id, .. } => require("custom device type_id", type_id),
        _ => Ok(()),
    }
}
