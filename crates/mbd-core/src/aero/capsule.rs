use core::f64::consts::PI;

use glam::DVec3;

use super::{AeroFlowState, AeroModel, AeroWrench};
use crate::{MbdError, math};

/// Sutton-Graves stagnation-heating constant for Earth air, in SI units with
/// flux in W/m²: `k` in `q = k * sqrt(rho / nose_radius) * V^3`.
pub const SUTTON_GRAVES_K_EARTH: f64 = 1.7415e-4;

/// Configuration of a [`BluntBodyCapsuleAero`].
///
/// Body `+x` runs along the symmetry axis out through the heat shield, the
/// side that leads during entry, and the body is axisymmetric about that axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BluntBodyCapsuleParams {
    /// Maximum radius of the heat shield, metres; sets the reference area
    /// `π r²` and the reference length `2 r`.
    pub base_radius_m: f64,
    /// Spherical radius of the heat shield, metres.
    pub nose_radius_m: f64,
    /// Moment reference: the heat shield's centre of curvature, about which
    /// surface pressure has no moment because every surface normal passes
    /// through it. Lies on the symmetry axis, body axes, metres.
    pub reference_point_body_m: DVec3,
    /// `s` in `CN = s sin α cos α`.
    pub normal_force_slope: f64,
    /// `CA0` in `CA = CA0 cos² α`.
    pub axial_drag_coeff: f64,
    /// Pitch-damping derivative `Cmq`, per `ω·(2r)/(2V)`, applied to pitch and
    /// yaw rates; negative damps.
    pub pitch_damping_coeff: f64,
    /// Sutton-Graves constant, which depends on the atmosphere's composition.
    pub sutton_graves_k: f64,
}

/// Modified-Newtonian aerodynamics of an axisymmetric blunt capsule at
/// hypersonic speed.
///
/// Surface pressure follows `sin²` of the local inclination to the flow, so
/// the coefficients scale as `cos² α` and `sin α cos α` in the total angle of
/// attack `α` between the velocity and the symmetry axis. The force lies in
/// the plane of those two, which makes the body's only preferred direction the
/// one the centre of gravity gives it: an off-axis centre of gravity trims the
/// capsule at nonzero `α`, and rotating the vehicle about its velocity then
/// rotates the lift with it.
///
/// With the velocity leading the apex (`α` past 90°) the flow is mirrored
/// rather than modelled, which keeps drag opposing the motion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BluntBodyCapsuleAero {
    params: BluntBodyCapsuleParams,
}

impl BluntBodyCapsuleAero {
    /// Validates `params`: a heat shield cannot be a tighter sphere than its
    /// own rim allows, and the drag and heating constants cannot be negative.
    pub fn new(params: BluntBodyCapsuleParams) -> Result<Self, MbdError> {
        let invalid = |field, value| Err(MbdError::InvalidAeroParameter { field, value });
        if !params.base_radius_m.is_finite() || params.base_radius_m <= 0.0 {
            return invalid("base_radius_m", params.base_radius_m);
        }
        if !params.nose_radius_m.is_finite() || params.nose_radius_m < params.base_radius_m {
            return invalid("nose_radius_m", params.nose_radius_m);
        }
        if !params.reference_point_body_m.is_finite() {
            let distance = params.reference_point_body_m.length();
            return invalid("reference_point_body_m", distance);
        }
        if !params.normal_force_slope.is_finite() {
            return invalid("normal_force_slope", params.normal_force_slope);
        }
        if !params.axial_drag_coeff.is_finite() || params.axial_drag_coeff < 0.0 {
            return invalid("axial_drag_coeff", params.axial_drag_coeff);
        }
        if !params.pitch_damping_coeff.is_finite() {
            return invalid("pitch_damping_coeff", params.pitch_damping_coeff);
        }
        if !params.sutton_graves_k.is_finite() || params.sutton_graves_k < 0.0 {
            return invalid("sutton_graves_k", params.sutton_graves_k);
        }
        Ok(Self { params })
    }

    /// The validated configuration.
    pub fn params(&self) -> &BluntBodyCapsuleParams {
        &self.params
    }
}

impl AeroModel for BluntBodyCapsuleAero {
    /// Works in the velocity components rather than `α`: `q S cos α` factors
    /// become `½ ρ S |u| V`, so the force and moment fall smoothly to zero as
    /// the speed does, with no undefined angle at rest.
    fn evaluate(&self, flow: &AeroFlowState) -> Result<AeroWrench, MbdError> {
        flow.validate()?;
        let params = &self.params;
        let density = flow.ambient.density_kg_m3;
        let velocity = flow.velocity_air_body_m_s;
        let speed = velocity.length();
        let area = PI * params.base_radius_m * params.base_radius_m;
        let length = 2.0 * params.base_radius_m;

        let transverse_velocity = DVec3::new(0.0, velocity.y, velocity.z);
        let force_body_n = -(0.5 * density * area * velocity.x.abs())
            * (DVec3::X * (params.axial_drag_coeff * velocity.x)
                + transverse_velocity * params.normal_force_slope);

        let rate = flow.angular_rate_body_rad_s;
        let transverse_rate = DVec3::new(0.0, rate.y, rate.z);
        let moment_ref_body_nm = transverse_rate
            * (0.25 * density * speed * area * length * length * params.pitch_damping_coeff);

        Ok(AeroWrench {
            reference_point_body_m: params.reference_point_body_m,
            force_body_n,
            moment_ref_body_nm,
            stagnation_heat_flux_w_m2: params.sutton_graves_k
                * math::sqrt(density / params.nose_radius_m)
                * speed
                * speed
                * speed,
        })
    }
}
