use core::f64::consts::PI;

use glam::DVec3;
use mbd_core::{AeroFlowState, AeroModel, AeroWrench, MbdError};

use crate::validation::{
    check_finite, check_finite_vec, check_nonnegative, check_positive, invalid, smoothstep01,
};

#[cfg(test)]
mod tests;

/// Configuration of a unified [`BluntCapsuleAero`] aerodynamic model covering
/// Vostok-1 spherical reentry, Gemini, Apollo CM lifting reentry, and Martian
/// blunt aeroshells across hypersonic, transonic, subsonic, and rarefied
/// Knudsen regimes.
///
/// Body `+x` runs along the symmetry axis out through the heat shield apex.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BluntCapsuleParams {
    /// Maximum heat-shield base radius, metres (`S = π r²`, `L = 2 r`).
    pub base_radius_m: f64,
    /// Spherical nose radius of the heat shield, metres (`>= base_radius_m`).
    pub nose_radius_m: f64,
    /// Moment reference point (heat shield centre of curvature), body axes, m.
    pub reference_point_body_m: DVec3,
    /// Normal-force slope `s` in `CN = s sin α cos α`.
    pub normal_force_slope: f64,
    /// Hypersonic Newtonian axial drag coefficient `CA0` in `CA = CA0 cos² α`.
    pub axial_drag_coeff: f64,
    /// Pitch/yaw aerodynamic damping derivative `Cmq`; negative damps.
    pub pitch_damping_coeff: f64,
    /// Optional explicit Sutton-Graves constant. When `None`, derived directly
    /// from the live [`mbd_core::AmbientGas::sutton_graves_k`] composition.
    pub sutton_graves_k: Option<f64>,
    /// Optional subsonic continuum drag coefficient (`M <= 0.7`). When `None`,
    /// pure modified-Newtonian `axial_drag_coeff` is used across all Mach numbers.
    pub subsonic_drag_coeff: Option<f64>,
    /// Additive transonic axial drag coefficient bump around Mach `1.05`.
    pub transonic_drag_rise: f64,
    /// Optional free-molecular drag coefficient (`Kn >> 1`). When `Some`,
    /// smoothly bridges high-altitude continuum flow to free-molecular flow.
    pub free_molecular_drag_coeff: Option<f64>,
}

/// Unified multi-regime blunt-body aerodynamics for capsules and aeroshells.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BluntCapsuleAero {
    params: BluntCapsuleParams,
}

impl BluntCapsuleAero {
    /// Validates `params` and constructs a multi-regime capsule aerodynamic model.
    pub fn new(params: BluntCapsuleParams) -> Result<Self, MbdError> {
        check_positive("base_radius_m", params.base_radius_m)?;
        check_positive("nose_radius_m", params.nose_radius_m)?;
        check_finite_vec("reference_point_body_m", params.reference_point_body_m)?;
        check_finite("normal_force_slope", params.normal_force_slope)?;
        check_nonnegative("axial_drag_coeff", params.axial_drag_coeff)?;
        check_finite("pitch_damping_coeff", params.pitch_damping_coeff)?;
        if let Some(k) = params.sutton_graves_k {
            check_nonnegative("sutton_graves_k", k)?;
        }
        if let Some(cd_sub) = params.subsonic_drag_coeff {
            check_nonnegative("subsonic_drag_coeff", cd_sub)?;
        }
        check_nonnegative("transonic_drag_rise", params.transonic_drag_rise)?;
        if let Some(cd_fm) = params.free_molecular_drag_coeff {
            check_nonnegative("free_molecular_drag_coeff", cd_fm)?;
        }
        Ok(Self { params })
    }

    /// Pure modified-Newtonian hypersonic capsule preset matching classical
    /// blunt-body reentry benchmarks across Earth and planetary entry.
    #[allow(clippy::too_many_arguments)]
    pub fn hypersonic_newtonian(
        base_radius_m: f64,
        nose_radius_m: f64,
        reference_point_body_m: DVec3,
        normal_force_slope: f64,
        axial_drag_coeff: f64,
        pitch_damping_coeff: f64,
        sutton_graves_k: f64,
    ) -> Result<Self, MbdError> {
        if !nose_radius_m.is_finite() || nose_radius_m < base_radius_m {
            return Err(invalid("nose_radius_m", nose_radius_m));
        }
        Self::new(BluntCapsuleParams {
            base_radius_m,
            nose_radius_m,
            reference_point_body_m,
            normal_force_slope,
            axial_drag_coeff,
            pitch_damping_coeff,
            sutton_graves_k: Some(sutton_graves_k),
            subsonic_drag_coeff: None,
            transonic_drag_rise: 0.0,
            free_molecular_drag_coeff: None,
        })
    }

    /// Validated configuration parameters.
    pub const fn params(&self) -> &BluntCapsuleParams {
        &self.params
    }

    fn effective_coeffs(&self, flow: &AeroFlowState, speed: f64, length: f64) -> (f64, f64) {
        let p = &self.params;
        let mach = if flow.ambient.sound_speed_m_s > 0.0 {
            speed / flow.ambient.sound_speed_m_s
        } else {
            0.0
        };
        let continuum_ca = match p.subsonic_drag_coeff {
            Some(cd_sub) => {
                let blend = smoothstep01((mach - 0.7) / 1.1);
                cd_sub + (p.axial_drag_coeff - cd_sub) * blend
            }
            None => p.axial_drag_coeff,
        };
        let dm = (mach - 1.05) / 0.25;
        let wave = p.transonic_drag_rise * libm::exp(-(dm * dm));
        let ca_continuum = continuum_ca + wave;

        if let Some(cd_fm) = p.free_molecular_drag_coeff {
            let kn = flow.ambient.knudsen_number(length);
            if kn.is_finite() && kn > 0.01 {
                let w = smoothstep01(libm::log10(kn / 0.01) / 2.5);
                let ca = ca_continuum + (cd_fm - ca_continuum) * w;
                let cn = p.normal_force_slope * (1.0 - w);
                return (ca, cn);
            }
        }
        (ca_continuum, p.normal_force_slope)
    }
}

impl AeroModel for BluntCapsuleAero {
    fn evaluate(&self, flow: &AeroFlowState) -> Result<AeroWrench, MbdError> {
        flow.validate()?;
        let params = &self.params;
        let density = flow.ambient.density_kg_m3;
        let velocity = flow.velocity_air_body_m_s;
        let speed = velocity.length();
        let area = PI * params.base_radius_m * params.base_radius_m;
        let length = 2.0 * params.base_radius_m;
        let (ca, cn_slope) = self.effective_coeffs(flow, speed, length);

        let transverse_velocity = DVec3::new(0.0, velocity.y, velocity.z);
        let force_body_n = -(0.5 * density * area * velocity.x.abs())
            * (DVec3::X * (ca * velocity.x) + transverse_velocity * cn_slope);

        let rate = flow.angular_rate_body_rad_s;
        let transverse_rate = DVec3::new(0.0, rate.y, rate.z);
        let moment_ref_body_nm = transverse_rate
            * (0.25 * density * speed * area * length * length * params.pitch_damping_coeff);

        let sg_k = params
            .sutton_graves_k
            .unwrap_or_else(|| flow.ambient.sutton_graves_k());
        Ok(AeroWrench {
            reference_point_body_m: params.reference_point_body_m,
            force_body_n,
            moment_ref_body_nm,
            stagnation_heat_flux_w_m2: sg_k
                * libm::sqrt(density / params.nose_radius_m)
                * speed
                * speed
                * speed,
        })
    }
}
