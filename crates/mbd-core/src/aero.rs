use glam::DVec3;

use crate::{AmbientGas, MbdError};

#[cfg(test)]
mod tests;

/// The flow a pluggable aerodynamics model evaluates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroFlowState {
    /// Velocity of the vehicle relative to the air, body axes, m/s.
    pub velocity_air_body_m_s: DVec3,
    /// Angular rate relative to the air, body axes, rad/s.
    pub angular_rate_body_rad_s: DVec3,
    /// Undisturbed multi-species ambient gas.
    pub ambient: AmbientGas,
    /// Control-surface deflections, rad; models define which slot is which.
    pub control_surfaces_rad: [f64; 8],
}

impl AeroFlowState {
    /// Rejects a flow a model cannot evaluate: non-finite velocities,
    /// non-finite control surfaces, or invalid ambient gas properties.
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
        self.ambient.validate()?;
        for &deflection in &self.control_surfaces_rad {
            if !deflection.is_finite() {
                return invalid("control_surfaces_rad", deflection);
            }
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
    fn evaluate(&self, flow: &AeroFlowState) -> Result<AeroWrench, MbdError> {
        flow.validate()?;
        Ok(AeroWrench::ZERO)
    }
}
