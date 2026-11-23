use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, WrenchAtSite};

use crate::MultibodyError;
use crate::contact::{SurfaceMedium, SurfaceSample};

#[cfg(test)]
mod tests;

/// Specification of one steered/driven/braked rolling wheel assembly (aircraft
/// tricycle/taildragger landing gear or Mars rover 6-wheel bogie wheel).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelSpec {
    /// Component owning this wheel strut/axle.
    pub component: ComponentId,
    /// Axle hub attachment point in `component` local frame, m.
    pub hub_offset_local_m: DVec3,
    /// Tyre/wheel rolling radius, m.
    pub wheel_radius_m: f64,
    /// Suspension spring stiffness along strut normal, N/m.
    pub spring_n_m: f64,
    /// Suspension damper coefficient, N·s/m.
    pub damper_n_s_m: f64,
    /// Forward rolling unit vector in `component` frame (`steer = 0`).
    pub forward_axis_local: DVec3,
    /// Right-side axle unit vector in `component` frame.
    pub axle_axis_local: DVec3,
    /// Peak tire-soil friction coefficient.
    pub peak_mu: f64,
    /// Cornering stiffness wrt sideslip angle, N/rad.
    pub cornering_stiffness_n_rad: f64,
    /// Rolling resistance multiplier ($F_{rr} = C_{rr} F_n$).
    pub rolling_resistance_coeff: f64,
    /// Longitudinal tire tread slip stiffness, N per unit slip.
    pub longitudinal_stiffness_n: f64,
}

/// Operator/autopilot command to one wheel.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WheelCommand {
    /// Steering angle about vertical strut axis, rad.
    pub steer_angle_rad: f64,
    /// Motor drive torque applied forward to the wheel, N·m.
    pub drive_torque_nm: f64,
    /// Friction brake torque opposing spin, N·m (`>= 0`).
    pub brake_torque_nm: f64,
}

/// Mutable wheel spin and ground-load state.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WheelState {
    /// Wheel angular speed about its rolling axle, rad/s.
    pub spin_rate_rad_s: f64,
    /// Last resolved normal ground load, N.
    pub ground_load_n: f64,
}

/// Result of one [`evaluate_wheel`] step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelForceOutput {
    /// Reaction wrench applied at `hub_offset_local_m` for multibody dynamics.
    pub wrench: WrenchAtSite,
    /// Normal load magnitude, N.
    pub contact_load_n: f64,
    /// Normalized longitudinal slip ratio (`(omega*R - v_fwd) / max(|v|, 0.2)`).
    pub slip_ratio: f64,
    /// Tyre sideslip angle, rad.
    pub slip_angle_rad: f64,
}

/// Steps wheel suspension, rolling spin dynamics, longitudinal traction/braking,
/// and lateral cornering against `surface` with friction-circle saturation.
#[allow(clippy::too_many_arguments)]
pub fn evaluate_wheel(
    spec: &WheelSpec,
    state: &mut WheelState,
    cmd: &WheelCommand,
    hub_world_pos: DVec3,
    hub_world_vel: DVec3,
    rot_world_from_link: DQuat,
    surface: &SurfaceSample,
    wheel_spin_inertia_kg_m2: f64,
    dt_s: f64,
) -> Result<WheelForceOutput, MultibodyError> {
    if !dt_s.is_finite() || dt_s < 0.0 || wheel_spin_inertia_kg_m2 <= 0.0 {
        return Err(MultibodyError::InvalidParameter {
            field: "dt_or_wheel_inertia",
            value: dt_s,
        });
    }
    let normal = surface.outward_normal_world.normalize_or(DVec3::Z);
    let bottom_world = hub_world_pos - normal * spec.wheel_radius_m;
    let deflection = (surface.surface_point_world_m - bottom_world).dot(normal);
    if surface.medium != SurfaceMedium::Solid || deflection <= 0.0 {
        state.ground_load_n = 0.0;
        let torque_free =
            cmd.drive_torque_nm - cmd.brake_torque_nm.max(0.0) * state.spin_rate_rad_s.signum();
        state.spin_rate_rad_s += (torque_free / wheel_spin_inertia_kg_m2) * dt_s;
        return Ok(WheelForceOutput {
            wrench: WrenchAtSite::zero(spec.component),
            contact_load_n: 0.0,
            slip_ratio: 0.0,
            slip_angle_rad: 0.0,
        });
    }

    let v_n = hub_world_vel.dot(normal);
    let fn_mag = (spec.spring_n_m * deflection - spec.damper_n_s_m * v_n).max(0.0);
    state.ground_load_n = fn_mag;

    let fwd_raw = rot_world_from_link * spec.forward_axis_local;
    let fwd_unsteered = (fwd_raw - normal * fwd_raw.dot(normal)).normalize_or(DVec3::X);
    let right_raw = normal.cross(fwd_unsteered).normalize_or(DVec3::Y);
    let (sin_s, cos_s) = libm::sincos(cmd.steer_angle_rad);
    let fwd_t = (fwd_unsteered * cos_s + right_raw * sin_s).normalize();
    let lat_t = normal.cross(fwd_t).normalize();

    let (f_long, f_lat, slip_ratio, slip_angle) = compute_planar_tire_forces(
        spec,
        state.spin_rate_rad_s,
        hub_world_vel,
        fwd_t,
        lat_t,
        fn_mag,
        surface.mu_dynamic_override.unwrap_or(spec.peak_mu),
    );

    let road_torque = -f_long * spec.wheel_radius_m;
    let net_spin_torque = cmd.drive_torque_nm + road_torque
        - cmd.brake_torque_nm.max(0.0) * state.spin_rate_rad_s.signum();
    state.spin_rate_rad_s += (net_spin_torque / wheel_spin_inertia_kg_m2) * dt_s;

    let f_world = normal * fn_mag + fwd_t * f_long + lat_t * f_lat;
    Ok(WheelForceOutput {
        wrench: WrenchAtSite {
            site: spec.component,
            point_local_m: spec.hub_offset_local_m,
            force_local_n: rot_world_from_link.conjugate() * f_world,
            torque_local_nm: DVec3::ZERO,
        },
        contact_load_n: fn_mag,
        slip_ratio,
        slip_angle_rad: slip_angle,
    })
}

fn compute_planar_tire_forces(
    spec: &WheelSpec,
    spin_rate_rad_s: f64,
    hub_world_vel: DVec3,
    fwd_t: DVec3,
    lat_t: DVec3,
    fn_mag: f64,
    mu: f64,
) -> (f64, f64, f64, f64) {
    let v_fwd = hub_world_vel.dot(fwd_t);
    let v_lat = hub_world_vel.dot(lat_t);
    let v_ref = v_fwd
        .abs()
        .max(spin_rate_rad_s.abs() * spec.wheel_radius_m)
        .max(0.2);
    let slip_ratio = ((spin_rate_rad_s * spec.wheel_radius_m - v_fwd) / v_ref).clamp(-1.0, 1.0);
    let slip_angle = libm::atan2(v_lat, v_fwd.abs().max(0.25));

    let load_scale = (fn_mag / (fn_mag + 500.0)).max(0.1);
    let f_long_raw = spec.longitudinal_stiffness_n * slip_ratio * load_scale
        - spec.rolling_resistance_coeff * fn_mag * v_fwd.signum();
    let f_lat_raw = -spec.cornering_stiffness_n_rad * slip_angle * load_scale;

    let f_max = (mu * fn_mag).max(0.0);
    let planar_mag = libm::sqrt(f_long_raw * f_long_raw + f_lat_raw * f_lat_raw);
    if planar_mag > f_max && planar_mag > 1e-12 {
        let s = f_max / planar_mag;
        (f_long_raw * s, f_lat_raw * s, slip_ratio, slip_angle)
    } else {
        (f_long_raw, f_lat_raw, slip_ratio, slip_angle)
    }
}
