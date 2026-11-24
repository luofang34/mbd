use glam::DVec3;
use mbd_core::MbdError;

use crate::flow::{AeroWrench, LinkFlow};

use crate::validation::{check_finite_vec, check_nonnegative, check_positive};

#[cfg(test)]
mod tests;

/// Configuration for an ejected pilot or cosmonaut test dummy (e.g., Vostok-1
/// SK-1 seat separation, K-36DM ejection).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HumanDummyParams {
    /// Reference frontal drag area, m².
    pub frontal_area_m2: f64,
    /// Characteristic height/chord for rotational damping, m.
    pub reference_length_m: f64,
    /// Isotropic or upright bluff drag coefficient.
    pub drag_coeff: f64,
    /// Tumbling rotational aerodynamic damping derivative; negative damps.
    pub damping_coeff: f64,
    /// Reference CG / pressure centre in dummy body frame, m.
    pub reference_point_body_m: DVec3,
}

/// Bluff-body aerodynamic drag and tumbling damping for an ejected dummy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HumanDummyAero {
    params: HumanDummyParams,
}

impl HumanDummyAero {
    /// Validates geometry and drag parameters.
    pub fn new(params: HumanDummyParams) -> Result<Self, MbdError> {
        check_positive("frontal_area_m2", params.frontal_area_m2)?;
        check_positive("reference_length_m", params.reference_length_m)?;
        check_nonnegative("drag_coeff", params.drag_coeff)?;
        if !params.damping_coeff.is_finite() {
            return Err(MbdError::InvalidAeroParameter {
                field: "damping_coeff",
                value: params.damping_coeff,
            });
        }
        check_finite_vec("reference_point_body_m", params.reference_point_body_m)?;
        Ok(Self { params })
    }

    /// Validated dummy aerodynamic configuration.
    pub const fn params(&self) -> &HumanDummyParams {
        &self.params
    }
}

impl HumanDummyAero {
    /// Load on the link for the flow at it.
    pub fn evaluate(&self, flow: &LinkFlow<'_>) -> Result<AeroWrench, MbdError> {
        flow.validate()?;
        let p = &self.params;
        let rho = flow.ambient.density_kg_m3;
        let speed = flow.velocity_air_link_m_s.length();
        if rho == 0.0 || speed == 0.0 {
            return Ok(AeroWrench {
                reference_point_link_m: p.reference_point_body_m,
                ..AeroWrench::ZERO
            });
        }
        let q = 0.5 * rho * speed;
        let force_link_n = -flow.velocity_air_link_m_s * (q * p.frontal_area_m2 * p.drag_coeff);
        let damp_scale =
            0.25 * rho * speed * p.frontal_area_m2 * p.reference_length_m * p.reference_length_m;
        let moment_ref_link_nm = flow.angular_rate_link_rad_s * (damp_scale * p.damping_coeff);
        Ok(AeroWrench {
            reference_point_link_m: p.reference_point_body_m,
            force_link_n,
            moment_ref_link_nm,
            stagnation_heat_flux_w_m2: 0.0,
        })
    }
}
