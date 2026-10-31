use glam::DVec3;

use crate::MbdError;

mod capsule;

pub use capsule::{BluntBodyCapsuleAero, BluntBodyCapsuleParams, SUTTON_GRAVES_K_EARTH};

#[cfg(test)]
mod tests;

/// Undisturbed air around the vehicle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AmbientFlow {
    /// Static pressure, Pa.
    pub pressure_pa: f64,
    /// Static temperature, K.
    pub temperature_k: f64,
    /// Density, kg/m³.
    pub density_kg_m3: f64,
    /// Speed of sound, m/s.
    pub sound_speed_m_s: f64,
}

/// The flow a pluggable aerodynamics model evaluates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroFlowState {
    /// Velocity of the vehicle relative to the air, body axes, m/s.
    pub velocity_air_body_m_s: DVec3,
    /// Angular rate relative to the air, body axes, rad/s.
    pub angular_rate_body_rad_s: DVec3,
    /// Undisturbed air.
    pub ambient: AmbientFlow,
    /// Control-surface deflections, rad; models define which slot is which.
    pub control_surfaces_rad: [f64; 8],
}

impl AeroFlowState {
    /// Rejects a flow a model cannot evaluate: anything non-finite, or
    /// negative density, which would flip the sign of every force.
    pub fn validate(&self) -> Result<(), MbdError> {
        let invalid = |field, value| Err(MbdError::InvalidFlow { field, value });
        if !self.velocity_air_body_m_s.is_finite() {
            return invalid("velocity_air_body_m_s", self.velocity_air_body_m_s.length());
        }
        if !self.angular_rate_body_rad_s.is_finite() {
            return invalid(
                "angular_rate_body_rad_s",
                self.angular_rate_body_rad_s.length(),
            );
        }
        let density = self.ambient.density_kg_m3;
        if !density.is_finite() || density < 0.0 {
            return invalid("ambient.density_kg_m3", density);
        }
        Ok(())
    }
}

/// Aerodynamic load at a model's reference station, independent of where the
/// centre of gravity currently is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroWrench {
    /// Reference station, body axes, metres.
    pub reference_point_body_m: DVec3,
    /// Force, body axes, N.
    pub force_body_n: DVec3,
    /// Moment about the reference station, body axes, N·m.
    pub moment_ref_body_nm: DVec3,
    /// Stagnation-point convective heat flux, W/m².
    pub stagnation_heat_flux_w_m2: f64,
}

impl AeroWrench {
    /// No load at all, referenced to the body origin.
    pub const ZERO: Self = Self {
        reference_point_body_m: DVec3::ZERO,
        force_body_n: DVec3::ZERO,
        moment_ref_body_nm: DVec3::ZERO,
        stagnation_heat_flux_w_m2: 0.0,
    };

    /// The moment about a live centre of gravity, N·m.
    pub fn moment_about_cg(&self, cg_body_m: DVec3) -> DVec3 {
        self.moment_ref_body_nm + (self.reference_point_body_m - cg_body_m).cross(self.force_body_n)
    }
}

/// An aerodynamics backend: maps the flow to a wrench, so a dynamics block
/// never depends on how the wrench was computed.
pub trait AeroModel {
    /// Evaluates the load for `flow`.
    fn evaluate(&self, flow: &AeroFlowState) -> Result<AeroWrench, MbdError>;
}

/// Airless flight: lunar descent and ascent, orbital coast.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct VacuumAero;

impl AeroModel for VacuumAero {
    fn evaluate(&self, _flow: &AeroFlowState) -> Result<AeroWrench, MbdError> {
        Ok(AeroWrench::ZERO)
    }
}
