use glam::DVec3;
use mbd_core::{AeroFlowState, AeroModel, AeroWrench, MbdError};

use crate::validation::{
    check_finite, check_finite_vec, check_nonnegative, check_positive, control_slot, smoothstep01,
};

#[cfg(test)]
mod tests;

/// Configuration for a slender launch-vehicle stage, rocket core, or finned
/// sounding vehicle (`+x` forward out the nose, `z` down).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyAndFinParams {
    /// Aerodynamic reference cross-sectional area, m².
    pub ref_area_m2: f64,
    /// Aerodynamic reference diameter or length for moments/damping, m.
    pub ref_length_m: f64,
    /// Centre of pressure / reference station in component axes, m.
    pub reference_point_body_m: DVec3,
    /// Effective nose radius for Sutton-Graves aerodynamic heating, m.
    pub nose_radius_m: f64,
    /// Subsonic zero-lift axial drag coefficient.
    pub cd0_subsonic: f64,
    /// Supersonic zero-lift axial drag coefficient (`M >= 1.6`).
    pub cd0_supersonic: f64,
    /// Additive transonic wave-drag bump peaked at Mach `1.05`.
    pub transonic_drag_peak: f64,
    /// Normal-force slope wrt $\sin\alpha$ on the body + fixed fins.
    pub cn_alpha_body: f64,
    /// Pitch/yaw damping derivative $C_{mq} + C_{m\dot{\alpha}}$; negative damps.
    pub pitch_damping_coeff: f64,
    /// Roll damping derivative $C_{l_p}$; negative damps.
    pub roll_damping_coeff: f64,
    /// Combined planform area of controllable aerodynamic fins/vanes, m².
    pub fin_area_m2: f64,
    /// Distance from `reference_point_body_m` aft to the fin hinge line, m.
    pub fin_arm_m: f64,
    /// Fin lift curve slope per radian.
    pub fin_cl_alpha: f64,
    /// Fin stall deflection magnitude clamp, rad.
    pub fin_stall_rad: f64,
    /// Control-surface slot driving pitch fins (positive pitches nose up around `+y`).
    pub pitch_control_channel: Option<usize>,
    /// Control-surface slot driving yaw fins (positive yaws nose right around `+z`).
    pub yaw_control_channel: Option<usize>,
    /// Control-surface slot driving roll fins/ailerons (positive rolls right wing down around `+x`).
    pub roll_control_channel: Option<usize>,
}

/// Slender-body plus control-fin aerodynamics for launch vehicles and missiles.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyAndFinAero {
    params: BodyAndFinParams,
}

impl BodyAndFinAero {
    /// Validates `params` and constructs a slender-body aerodynamic element.
    pub fn new(params: BodyAndFinParams) -> Result<Self, MbdError> {
        check_positive("ref_area_m2", params.ref_area_m2)?;
        check_positive("ref_length_m", params.ref_length_m)?;
        check_finite_vec("reference_point_body_m", params.reference_point_body_m)?;
        check_positive("nose_radius_m", params.nose_radius_m)?;
        check_nonnegative("cd0_subsonic", params.cd0_subsonic)?;
        check_nonnegative("cd0_supersonic", params.cd0_supersonic)?;
        check_nonnegative("transonic_drag_peak", params.transonic_drag_peak)?;
        check_nonnegative("cn_alpha_body", params.cn_alpha_body)?;
        check_finite("pitch_damping_coeff", params.pitch_damping_coeff)?;
        check_finite("roll_damping_coeff", params.roll_damping_coeff)?;
        check_nonnegative("fin_area_m2", params.fin_area_m2)?;
        check_nonnegative("fin_arm_m", params.fin_arm_m)?;
        check_nonnegative("fin_cl_alpha", params.fin_cl_alpha)?;
        check_positive("fin_stall_rad", params.fin_stall_rad)?;
        Ok(Self { params })
    }

    /// Validated configuration parameters.
    pub const fn params(&self) -> &BodyAndFinParams {
        &self.params
    }

    fn axial_drag_coeff(&self, mach: f64) -> f64 {
        let p = &self.params;
        let sup_blend = smoothstep01((mach - 0.8) / 0.8);
        let base = p.cd0_subsonic + (p.cd0_supersonic - p.cd0_subsonic) * sup_blend;
        let dm = (mach - 1.05) / 0.22;
        base + p.transonic_drag_peak * libm::exp(-(dm * dm))
    }
}

impl AeroModel for BodyAndFinAero {
    fn evaluate(&self, flow: &AeroFlowState) -> Result<AeroWrench, MbdError> {
        flow.validate()?;
        let p = &self.params;
        let rho = flow.ambient.density_kg_m3;
        let v = flow.velocity_air_body_m_s;
        let speed = v.length();
        if rho == 0.0 || speed == 0.0 {
            return Ok(AeroWrench {
                reference_point_body_m: p.reference_point_body_m,
                ..AeroWrench::ZERO
            });
        }
        let mach = if flow.ambient.sound_speed_m_s > 0.0 {
            speed / flow.ambient.sound_speed_m_s
        } else {
            0.0
        };
        let q = 0.5 * rho * speed * speed;
        let u_dir = v / speed;
        let cd0 = self.axial_drag_coeff(mach);

        // Smooth cross-flow + axial drag without singularity at alpha = 0 or pi.
        let axial_force = -q * p.ref_area_m2 * cd0 * u_dir.x * u_dir.x.abs();
        let transverse = DVec3::new(0.0, u_dir.y, u_dir.z);
        let sin_alpha = transverse.length().clamp(0.0, 1.0);
        let induced_drag_n =
            -q * p.ref_area_m2 * (0.45 * p.cn_alpha_body * sin_alpha * sin_alpha) * u_dir;
        let body_normal_n =
            -q * p.ref_area_m2 * p.cn_alpha_body * u_dir.x.abs().max(0.25) * transverse;

        // Control fin loads clamped at fin stall.
        let de =
            control_slot(flow, p.pitch_control_channel).clamp(-p.fin_stall_rad, p.fin_stall_rad);
        let dr = control_slot(flow, p.yaw_control_channel).clamp(-p.fin_stall_rad, p.fin_stall_rad);
        let da =
            control_slot(flow, p.roll_control_channel).clamp(-p.fin_stall_rad, p.fin_stall_rad);
        let q_fin = q * p.fin_area_m2 * p.fin_cl_alpha;
        // Positive pitch fin deflection gives nose-up (+Y moment) via -Z? Wait: fin sits aft (-X).
        // Downward force (+Z) at (-fin_arm_m, 0, 0) produces (-fin_arm, 0, 0) x (0, 0, +Z) = (0, +fin_arm * Fz, 0) = +Y nose up!
        let fin_fz = q_fin * de;
        let fin_fy = -q_fin * dr;
        let fin_force = DVec3::new(0.0, fin_fy, fin_fz);
        let fin_moment = DVec3::new(
            q_fin * p.ref_length_m * 0.5 * da,
            p.fin_arm_m * fin_fz,
            -p.fin_arm_m * fin_fy,
        );

        let omega = flow.angular_rate_body_rad_s;
        let damp_scale = 0.25 * rho * speed * p.ref_area_m2 * p.ref_length_m * p.ref_length_m;
        let damp_moment = DVec3::new(
            omega.x * damp_scale * p.roll_damping_coeff,
            omega.y * damp_scale * p.pitch_damping_coeff,
            omega.z * damp_scale * p.pitch_damping_coeff,
        );

        let force_body_n =
            DVec3::new(axial_force, 0.0, 0.0) + induced_drag_n + body_normal_n + fin_force;
        let moment_ref_body_nm = fin_moment + damp_moment;
        let heat_flux = flow.ambient.sutton_graves_k()
            * libm::sqrt(rho / p.nose_radius_m)
            * speed
            * speed
            * speed;

        Ok(AeroWrench {
            reference_point_body_m: p.reference_point_body_m,
            force_body_n,
            moment_ref_body_nm,
            stagnation_heat_flux_w_m2: heat_flux,
        })
    }
}
