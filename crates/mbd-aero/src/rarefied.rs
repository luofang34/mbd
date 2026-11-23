use core::f64::consts::PI;

use glam::DVec3;
use mbd_core::MbdError;

use crate::flow::{AeroWrench, LinkFlow};

use crate::validation::{check_finite_vec, check_positive, check_unit_vec, invalid};

#[cfg(test)]
mod tests;

/// Configuration for a Sentman free-molecular flat-plate element in rarefied
/// thermospheric/VLEO flow (solar panels, radiometric baffles, ABEP intakes,
/// spacecraft bus walls).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RarefiedPanelParams {
    /// Projected face area of one side of the panel, m².
    pub area_m2: f64,
    /// Outward unit normal of the front face in component axes.
    pub normal_body: DVec3,
    /// Geometric centre of pressure of the panel in component axes, m.
    pub center_body_m: DVec3,
    /// Whether both front (`+normal`) and back (`-normal`) surfaces are exposed.
    pub double_sided: bool,
    /// Wall surface temperature, K.
    pub surface_temperature_k: f64,
    /// Normal thermal energy accommodation coefficient $\sigma_n \in [0, 1]$.
    pub energy_accommodation: f64,
    /// Tangential momentum accommodation coefficient $\sigma_t \in [0, 1]$.
    pub tangential_accommodation: f64,
}

/// Free-molecular Sentman aerodynamic panel using multi-species mixture gas
/// constant $R_{\text{mix}}$ and kinetic temperature $T_\infty$.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RarefiedPanelAero {
    params: RarefiedPanelParams,
    normal: DVec3,
}

impl RarefiedPanelAero {
    /// Validates geometry and thermal-accommodation parameters.
    pub fn new(params: RarefiedPanelParams) -> Result<Self, MbdError> {
        check_positive("area_m2", params.area_m2)?;
        let normal = check_unit_vec("normal_body", params.normal_body)?;
        check_finite_vec("center_body_m", params.center_body_m)?;
        check_positive("surface_temperature_k", params.surface_temperature_k)?;
        if !params.energy_accommodation.is_finite()
            || !(0.0..=1.0).contains(&params.energy_accommodation)
        {
            return Err(invalid("energy_accommodation", params.energy_accommodation));
        }
        if !params.tangential_accommodation.is_finite()
            || !(0.0..=1.0).contains(&params.tangential_accommodation)
        {
            return Err(invalid(
                "tangential_accommodation",
                params.tangential_accommodation,
            ));
        }
        Ok(Self { params, normal })
    }

    /// Validated configuration parameters.
    pub const fn params(&self) -> &RarefiedPanelParams {
        &self.params
    }

    fn one_face_force(
        &self,
        face_normal: DVec3,
        u_flow: DVec3,
        speed: f64,
        q_dyn: f64,
        speed_ratio: f64,
        t_inf: f64,
    ) -> DVec3 {
        let p = &self.params;
        let gamma_c = u_flow.dot(face_normal);
        let s_g = speed_ratio * gamma_c;
        let p_s = libm::exp(-(s_g * s_g)) / (speed_ratio * libm::sqrt(PI));
        let q_s = 1.0 + libm::erf(s_g);
        let z_s = gamma_c * q_s + p_s;

        let sn = p.energy_accommodation;
        let st = p.tangential_accommodation;
        let wall_term =
            libm::sqrt((PI / (speed_ratio * speed_ratio)) * (p.surface_temperature_k / t_inf));
        let cp = ((2.0 - sn) * (gamma_c * gamma_c + 0.5 / (speed_ratio * speed_ratio))
            + 0.5 * sn * wall_term * gamma_c)
            * q_s
            + ((2.0 - sn) * gamma_c + 0.5 * sn * wall_term) * p_s;
        let ct = st * z_s;

        let v_tan = u_flow - face_normal * gamma_c;
        let _ = speed;
        q_dyn * p.area_m2 * (-face_normal * cp - v_tan * ct)
    }
}

impl RarefiedPanelAero {
    /// Load on the link for the flow at it.
    pub fn evaluate(&self, flow: &LinkFlow<'_>) -> Result<AeroWrench, MbdError> {
        flow.validate()?;
        let p = &self.params;
        let rho = flow.ambient.density_kg_m3;
        let r_mix = flow.ambient.specific_gas_constant_j_kg_k();
        let t_inf = flow.ambient.temperature_k;
        let v_local =
            flow.velocity_air_link_m_s + flow.angular_rate_link_rad_s.cross(p.center_body_m);
        let speed = v_local.length();
        // Only the flow direction below divides by the speed. The load goes
        // to zero continuously with it: the speed-ratio floor bounds the
        // thermal terms while the dynamic pressure vanishes.
        if rho == 0.0 || r_mix <= 0.0 || t_inf <= 0.0 || speed == 0.0 {
            return Ok(AeroWrench {
                reference_point_link_m: p.center_body_m,
                ..AeroWrench::ZERO
            });
        }
        let c_mp = libm::sqrt(2.0 * r_mix * t_inf);
        let s = (speed / c_mp).max(1e-4);
        let q_dyn = 0.5 * rho * speed * speed;
        let u_flow = v_local / speed;

        let mut force = self.one_face_force(self.normal, u_flow, speed, q_dyn, s, t_inf);
        if p.double_sided {
            force += self.one_face_force(-self.normal, u_flow, speed, q_dyn, s, t_inf);
        }

        // Free-molecular energy flux approximation using thermal accommodation sigma_n.
        let cos_inc = u_flow.dot(self.normal);
        let inc_area = if p.double_sided {
            cos_inc.abs()
        } else {
            cos_inc.max(0.0)
        };
        let q_fm = p.energy_accommodation * 0.5 * rho * speed * speed * speed * inc_area;

        Ok(AeroWrench {
            reference_point_link_m: p.center_body_m,
            force_link_n: force,
            moment_ref_link_nm: DVec3::ZERO,
            stagnation_heat_flux_w_m2: q_fm,
        })
    }
}
