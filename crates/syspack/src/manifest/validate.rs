use std::collections::BTreeSet;

use super::{
    DeviceKindSpec, DynamicsSpec, InstrumentPackRef, MountedDeviceSpec, SCHEMA_VERSION,
    SyspackManifest, SystemKind,
};
use crate::SyspackError;
use crate::instrument_pack::INSTRUMENTS_ASSET;

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

fn is_sha256_hex(digest: &str) -> bool {
    digest.len() == SHA256_HEX_DIGITS && digest.chars().all(|c| c.is_ascii_hexdigit())
}

pub(crate) fn is_slug(id: &str) -> bool {
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
        self.validate_instrument_pack()?;
        if self.validate_typeface_pack()? {
            return Ok(());
        }
        let keys = self.validate_components()?;
        self.validate_devices(&keys)
    }

    fn validate_assets(&self) -> Result<(), SyspackError> {
        for (path, digest) in &self.assets {
            check_asset_path(path)?;
            if !is_sha256_hex(&digest.sha256) {
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

    /// A pack must be a static `instruments.json` holder; a vehicle's pin
    /// must name a pack it could match.
    fn validate_instrument_pack(&self) -> Result<(), SyspackError> {
        if self.kind == SystemKind::InstrumentPack {
            let shape = |detail| Err(SyspackError::InstrumentPackShape { detail });
            if self.dynamics != DynamicsSpec::None {
                return shape("an instrument pack must declare dynamics None");
            }
            if self.instrument_pack.is_some() {
                return shape("an instrument pack cannot pin another instrument pack");
            }
            if !self.assets.contains_key(INSTRUMENTS_ASSET) {
                return shape("an instrument pack must declare instruments.json");
            }
        }
        if let Some(pin) = &self.instrument_pack {
            check_pin(pin)?;
        }
        Ok(())
    }

    /// A typeface pack is a static, massless holder of a face and its
    /// provenance; only it describes a typeface. Whether it was one.
    fn validate_typeface_pack(&self) -> Result<bool, SyspackError> {
        let shape = |detail| Err(SyspackError::TypefacePackShape { detail });
        let Some(typeface) = &self.typeface else {
            if self.kind == SystemKind::TypefacePack {
                return shape("a typeface pack must describe its typeface");
            }
            return Ok(false);
        };
        if self.kind != SystemKind::TypefacePack {
            return shape("only a typeface pack describes a typeface");
        }
        if self.dynamics != DynamicsSpec::None || self.instrument_pack.is_some() {
            return shape("a typeface pack has no dynamics and pins nothing");
        }
        if !self.components.is_empty() || !self.devices.is_empty() {
            return shape("a typeface pack has no components or devices");
        }
        typeface.validate(|path| self.assets.contains_key(path))?;
        Ok(true)
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
            check_device_kind(device)?;
        }
        Ok(())
    }
}

fn check_pin(pin: &InstrumentPackRef) -> Result<(), SyspackError> {
    if !is_slug(&pin.id) {
        return Err(SyspackError::InvalidPackageId { id: pin.id.clone() });
    }
    require("instrument pack version", &pin.version)?;
    if !is_sha256_hex(&pin.manifest_sha256) {
        return Err(SyspackError::MalformedDigest {
            path: format!("instrument pack {}", pin.id),
            sha256: pin.manifest_sha256.clone(),
        });
    }
    Ok(())
}

fn check_device_kind(device: &MountedDeviceSpec) -> Result<(), SyspackError> {
    let invalid = |field, detail: &str| {
        Err(SyspackError::InvalidDevice {
            device: device.id.clone(),
            field,
            detail: detail.to_owned(),
        })
    };
    match &device.kind {
        DeviceKindSpec::Custom { type_id, .. } => require("custom device type_id", type_id),
        DeviceKindSpec::PitotStatic {
            pitot_body_m,
            static_body_m,
            probe,
            total_line_tau_s,
            static_line_tau_s,
        } => {
            if probe.is_empty() {
                return invalid("probe", "must not be empty");
            }
            if !pitot_body_m
                .iter()
                .chain(static_body_m)
                .all(|v| v.is_finite())
            {
                return invalid("position", "must be finite");
            }
            for (field, tau) in [
                ("total_line_tau_s", total_line_tau_s),
                ("static_line_tau_s", static_line_tau_s),
            ] {
                if !(tau.is_finite() && *tau >= 0.0) {
                    return invalid(field, "must be finite and non-negative");
                }
            }
            Ok(())
        }
        _ => Ok(()),
    }
}
