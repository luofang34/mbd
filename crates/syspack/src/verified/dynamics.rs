use glam::DVec3;
use mbd_core::{AeroModel, BluntBodyCapsuleAero, BluntBodyCapsuleParams, VacuumAero};

use super::VerifiedSyspack;
use crate::{DynamicsSpec, SyspackError};

const JSBSIM: &str = "JsbsimXml";
const CAPSULE: &str = "RigidBodyBluntCapsule";
const RIGID_BODY: &str = "rigid-body";

impl VerifiedSyspack {
    fn mismatch(&self, operation: &'static str, needed: &'static str) -> SyspackError {
        SyspackError::DynamicsMismatch {
            operation,
            needed,
            declared: self.manifest.dynamics.kind(),
        }
    }

    /// The aerodynamics of a rigid-body package. A JSBSim package has none
    /// of its own, since JSBSim evaluates them inside the aircraft, and a
    /// stationary system has none at all.
    pub fn build_aero_model(&self) -> Result<Box<dyn AeroModel>, SyspackError> {
        match &self.manifest.dynamics {
            DynamicsSpec::RigidBodyVacuum => Ok(Box::new(VacuumAero)),
            DynamicsSpec::RigidBodyBluntCapsule { .. } => Ok(Box::new(self.capsule_aero()?)),
            DynamicsSpec::None | DynamicsSpec::JsbsimXml { .. } => {
                Err(self.mismatch("build_aero_model", RIGID_BODY))
            }
        }
    }

    /// The capsule aerodynamics of a blunt-capsule package, as their concrete
    /// type for callers that read the parameters back.
    pub fn capsule_aero(&self) -> Result<BluntBodyCapsuleAero, SyspackError> {
        let DynamicsSpec::RigidBodyBluntCapsule {
            base_radius_m,
            nose_radius_m,
            reference_point_body_m,
            normal_force_slope,
            axial_drag_coeff,
            pitch_damping_coeff,
            sutton_graves_k,
        } = self.manifest.dynamics
        else {
            return Err(self.mismatch("capsule_aero", CAPSULE));
        };
        BluntBodyCapsuleAero::new(BluntBodyCapsuleParams {
            base_radius_m,
            nose_radius_m,
            reference_point_body_m: DVec3::from_array(reference_point_body_m),
            normal_force_slope,
            axial_drag_coeff,
            pitch_damping_coeff,
            sutton_graves_k,
        })
        .map_err(SyspackError::InvalidAero)
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
