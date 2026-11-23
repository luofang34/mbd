use mbd_aero::AeroElement;

use super::VerifiedSyspack;
use crate::{DynamicsSpec, SyspackError};

const JSBSIM: &str = "JsbsimXml";
const COMPOSED: &str = "MultibodyComposed";

impl VerifiedSyspack {
    fn mismatch(&self, operation: &'static str, needed: &'static str) -> SyspackError {
        SyspackError::DynamicsMismatch {
            operation,
            needed,
            declared: self.manifest.dynamics.kind(),
        }
    }

    /// The aerodynamic elements of a composed multibody package, each
    /// bound to the component its manifest record names. A vacuum vehicle
    /// has none.
    pub fn build_aero_elements(&self) -> Result<Vec<AeroElement>, SyspackError> {
        let DynamicsSpec::MultibodyComposed { aero_elements } = &self.manifest.dynamics else {
            return Err(self.mismatch("build_aero_elements", COMPOSED));
        };
        aero_elements
            .iter()
            .map(|record| {
                Ok(AeroElement::new(
                    self.component_id(&record.site)?,
                    record.kind.build()?,
                ))
            })
            .collect()
    }

    /// The verified aircraft files as `(JSBSim virtual path, bytes)`, so a
    /// simulator can serve JSBSim with no file system.
    pub fn jsbsim_files(&self) -> Result<Vec<(&str, &[u8])>, SyspackError> {
        let DynamicsSpec::JsbsimXml {
            entry_asset,
            virtual_path,
            additional_assets,
            ..
        } = &self.manifest.dynamics
        else {
            return Err(self.mismatch("jsbsim_files", JSBSIM));
        };
        let entry = std::iter::once((entry_asset, virtual_path));
        let rest = additional_assets.iter().map(|(asset, path)| (asset, path));
        entry
            .chain(rest)
            .map(|(asset, path)| {
                let bytes = self
                    .asset(asset)
                    .ok_or_else(|| SyspackError::UndeclaredAsset {
                        path: asset.clone(),
                    })?;
                Ok((path.as_str(), bytes))
            })
            .collect()
    }

    /// The aircraft directory and model name JSBSim loads.
    pub fn jsbsim_model(&self) -> Result<(&str, &str), SyspackError> {
        match &self.manifest.dynamics {
            DynamicsSpec::JsbsimXml {
                aircraft_dir,
                model_name,
                ..
            } => Ok((aircraft_dir, model_name)),
            _ => Err(self.mismatch("jsbsim_model", JSBSIM)),
        }
    }
}
