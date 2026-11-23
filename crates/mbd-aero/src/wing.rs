use core::f64::consts::PI;

use glam::DVec3;
use mbd_core::MbdError;

use crate::flow::{AeroWrench, LinkFlow};

use crate::validation::{
    check_finite, check_finite_vec, check_nonnegative, check_positive, check_unit_vec, invalid,
    smoothstep01,
};

#[cfg(test)]
mod tests;

/// Swept and variable-sweep lifting surface aerodynamic element (wings, canards,
/// horizontal stabilizers, and vertical fins).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WingSurfaceParams {
    /// Planform reference area, m².
    pub area_m2: f64,
    /// Mean aerodynamic chord, m.
    pub mean_chord_m: f64,
    /// Wing span, m.
    pub span_m: f64,
    /// Aspect ratio (`b² / S` or effective AR).
    pub aspect_ratio: f64,
    /// Quarter-chord sweep angle at zero sweep control, rad (`0..1.35`).
    pub sweep_quarter_chord_rad: f64,
    /// Optional control-surface slot modulating wing sweep angle, rad (e.g. Tu-160).
    pub sweep_control_channel: Option<usize>,
    /// Incompressible 3-D lift-curve slope per rad (`CL_alpha`).
    pub cl_alpha_incompressible: f64,
    /// Zero-angle-of-attack lift coefficient `CL0`.
    pub cl0: f64,
    /// Maximum achievable lift coefficient before stall.
    pub cl_max: f64,
    /// Stall angle of attack magnitude, rad.
    pub alpha_stall_rad: f64,
    /// Zero-lift parasitic drag coefficient `CD0`.
    pub cd0: f64,
    /// Oswald span efficiency factor `e` (`0.3..1.05`).
    pub oswald_efficiency: f64,
    /// Transonic wave-drag growth multiplier above effective critical Mach.
    pub wave_drag_factor: f64,
    /// Unswept critical divergence Mach number.
    pub critical_mach: f64,
    /// Zero-lift pitching moment coefficient about `ac_position_body_m`.
    pub cm0: f64,
    /// Aerodynamic centre position in component local frame, m.
    pub ac_position_body_m: DVec3,
    /// Unit forward chordwise direction in component frame (typically `+X`).
    pub chord_direction_body: DVec3,
    /// Unit positive-lift normal direction in component frame (typically `-Z` for wings, `+Y` for fins).
    pub lift_normal_body: DVec3,
    /// Optional control slot adding camber lift `flap_dcl * defl` and drag `flap_dcd * |defl|`.
    pub flap_channel: Option<usize>,
    /// Lift increment per radian of `flap_channel` deflection.
    pub flap_dcl: f64,
    /// Drag increment per radian of absolute `flap_channel` deflection.
    pub flap_dcd: f64,
    /// Optional control slot for differential control surfaces (`aileron_dcl * defl`).
    pub aileron_channel: Option<usize>,
    /// Lift increment per radian of `aileron_channel` deflection.
    pub aileron_dcl: f64,
}

/// Swept and variable-sweep lifting surface aerodynamics model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WingSurfaceAero {
    params: WingSurfaceParams,
    e_chord: DVec3,
    e_lift: DVec3,
    e_pitch: DVec3,
}

impl WingSurfaceAero {
    /// Validates geometry, orientation basis, and polar parameters.
    pub fn new(params: WingSurfaceParams) -> Result<Self, MbdError> {
        validate_wing_params(&params)?;
        let e_chord = check_unit_vec("chord_direction_body", params.chord_direction_body)?;
        let e_lift = check_unit_vec("lift_normal_body", params.lift_normal_body)?;
        if e_chord.dot(e_lift).abs() > 0.25 {
            return Err(invalid("lift_normal_body", e_chord.dot(e_lift)));
        }
        let e_lift_ortho = (e_lift - e_chord * e_chord.dot(e_lift)).normalize();
        let e_pitch = e_chord.cross(e_lift_ortho).normalize();
        Ok(Self {
            params,
            e_chord,
            e_lift: e_lift_ortho,
            e_pitch,
        })
    }

    /// Validated configuration parameters.
    pub const fn params(&self) -> &WingSurfaceParams {
        &self.params
    }

    fn coeffs(&self, alpha: f64, mach: f64, sweep: f64, flow: &LinkFlow<'_>) -> (f64, f64, f64) {
        let p = &self.params;
        let cos_sw = libm::cos(sweep).max(0.2);
        let m_n = mach * cos_sw;
        let slope = if mach >= 1.25 {
            let beta_s = libm::sqrt((mach * mach - 1.0).max(0.25));
            (4.0 / beta_s).min(p.cl_alpha_incompressible * 1.2)
        } else {
            let beta = libm::sqrt((1.0 - m_n * m_n).max(0.18));
            p.cl_alpha_incompressible * cos_sw / beta
        };
        let fl = flow.control(p.flap_channel);
        let ail = flow.control(p.aileron_channel);
        let linear_cl = (p.cl0 + slope * alpha + p.flap_dcl * fl + p.aileron_dcl * ail)
            .clamp(-p.cl_max, p.cl_max);
        let post_stall_cl = 1.8 * libm::sin(2.0 * alpha);
        let excess = (alpha.abs() - p.alpha_stall_rad) / 0.22;
        let stall_blend = if alpha.abs() > p.alpha_stall_rad {
            smoothstep01(excess)
        } else {
            0.0
        };
        let cl = linear_cl + (post_stall_cl - linear_cl) * stall_blend;

        let induced = (cl * cl) / (PI * p.aspect_ratio * p.oswald_efficiency);
        let stall_drag = 1.9 * libm::sin(alpha) * libm::sin(alpha) * stall_blend;
        let crit_eff = p.critical_mach / cos_sw;
        let wave = if mach > crit_eff {
            let dm = (mach - crit_eff).min(0.6);
            p.wave_drag_factor * dm * dm * dm
        } else {
            0.0
        };
        let cd = p.cd0 + induced + stall_drag + wave + p.flap_dcd * fl.abs();
        (cl, cd, p.cm0)
    }
}

impl WingSurfaceAero {
    /// Load on the link for the flow at it.
    pub fn evaluate(&self, flow: &LinkFlow<'_>) -> Result<AeroWrench, MbdError> {
        flow.validate()?;
        let p = &self.params;
        let rho = flow.ambient.density_kg_m3;
        let v_ac =
            flow.velocity_air_link_m_s + flow.angular_rate_link_rad_s.cross(p.ac_position_body_m);
        let a = v_ac.dot(self.e_chord);
        let b = v_ac.dot(self.e_lift);
        let v_plane_sq = a * a + b * b;
        if rho == 0.0 || v_plane_sq == 0.0 {
            return Ok(AeroWrench {
                reference_point_link_m: p.ac_position_body_m,
                ..AeroWrench::ZERO
            });
        }
        let v_plane = libm::sqrt(v_plane_sq);
        let alpha = libm::atan2(-b, a);
        let mach = if flow.ambient.sound_speed_m_s > 0.0 {
            v_ac.length() / flow.ambient.sound_speed_m_s
        } else {
            0.0
        };
        let sweep =
            (p.sweep_quarter_chord_rad + flow.control(p.sweep_control_channel)).clamp(0.0, 1.35);
        let (cl, cd, cm) = self.coeffs(alpha, mach, sweep, flow);
        let q = 0.5 * rho * v_plane_sq;
        let u_drag = -(self.e_chord * a + self.e_lift * b) / v_plane;
        let u_lift = (self.e_lift * a - self.e_chord * b) / v_plane;
        let force_link_n = q * p.area_m2 * (cl * u_lift + cd * u_drag);
        let moment_ref_link_nm = self.e_pitch * (q * p.area_m2 * p.mean_chord_m * cm);
        Ok(AeroWrench {
            reference_point_link_m: p.ac_position_body_m,
            force_link_n,
            moment_ref_link_nm,
            stagnation_heat_flux_w_m2: 0.0,
        })
    }
}

fn validate_wing_params(p: &WingSurfaceParams) -> Result<(), MbdError> {
    check_positive("area_m2", p.area_m2)?;
    check_positive("mean_chord_m", p.mean_chord_m)?;
    check_positive("span_m", p.span_m)?;
    check_positive("aspect_ratio", p.aspect_ratio)?;
    if !p.sweep_quarter_chord_rad.is_finite() || !(0.0..=1.35).contains(&p.sweep_quarter_chord_rad)
    {
        return Err(invalid(
            "sweep_quarter_chord_rad",
            p.sweep_quarter_chord_rad,
        ));
    }
    check_nonnegative("cl_alpha_incompressible", p.cl_alpha_incompressible)?;
    check_finite("cl0", p.cl0)?;
    check_nonnegative("cl_max", p.cl_max)?;
    check_positive("alpha_stall_rad", p.alpha_stall_rad)?;
    check_nonnegative("cd0", p.cd0)?;
    check_positive("oswald_efficiency", p.oswald_efficiency)?;
    check_nonnegative("wave_drag_factor", p.wave_drag_factor)?;
    check_positive("critical_mach", p.critical_mach)?;
    check_finite("cm0", p.cm0)?;
    check_finite_vec("ac_position_body_m", p.ac_position_body_m)?;
    check_finite("flap_dcl", p.flap_dcl)?;
    check_nonnegative("flap_dcd", p.flap_dcd)?;
    check_finite("aileron_dcl", p.aileron_dcl)
}
