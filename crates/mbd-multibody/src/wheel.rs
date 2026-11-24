//! Rolling wheel on a sprung strut: suspension load, slip-based tyre forces
//! with friction-circle saturation, and the wheel's own spin as a
//! continuous state.

use glam::DVec3;
use mbd_core::{ComponentId, WrenchAtSite};

use crate::MultibodyError;
use crate::contact::{SurfaceMedium, SurfaceQuery, SurfaceSample};
use crate::element::{ElementContext, ForceElement};
use crate::model::layout::{read1, write1};
use crate::model::{Loads, ModelScale, PointKinematics};

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
    /// Low-speed and load regularisation of the slip model.
    pub tire: TireModelSpec,
}

/// Regularisation of the slip-based tyre model. Slip ratio and slip angle
/// divide by speed, so each needs a floor; the load scale softens the tyre
/// stiffnesses at light load.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TireModelSpec {
    /// Floor of the reference speed in the slip ratio, m/s.
    pub slip_speed_floor_m_s: f64,
    /// Floor of the forward speed in the slip angle, m/s.
    pub slip_angle_speed_floor_m_s: f64,
    /// Normal load at which the tyre stiffnesses reach half their nominal
    /// value, N: the scale is `F / (F + load_sensitivity_n)`.
    pub load_sensitivity_n: f64,
    /// Smallest load scale, keeping some grip at vanishing load.
    pub min_load_scale: f64,
}

impl TireModelSpec {
    /// Regularisation of a wheel of a few decimetres carrying hundreds of
    /// newtons: a rover or light-aircraft wheel.
    pub const DEFAULT: Self = Self {
        slip_speed_floor_m_s: 0.2,
        slip_angle_speed_floor_m_s: 0.25,
        load_sensitivity_n: 500.0,
        min_load_scale: 0.1,
    };

    /// Regularisation derived from the wheel itself: the speed floors are
    /// fractions (`1/8`, `1/6.4`) of the wheel's free-fall speed scale
    /// `√(g R)`, and the load sensitivity is its nominal load. These
    /// fractions reproduce [`TireModelSpec::DEFAULT`] for a 0.26 m wheel
    /// under Earth gravity carrying 500 N.
    pub fn for_wheel(wheel_radius_m: f64, nominal_load_n: f64, gravity_m_s2: f64) -> Self {
        let speed = libm::sqrt((gravity_m_s2 * wheel_radius_m).max(0.0));
        Self {
            slip_speed_floor_m_s: speed / 8.0,
            slip_angle_speed_floor_m_s: speed / 6.4,
            load_sensitivity_n: nominal_load_n,
            min_load_scale: Self::DEFAULT.min_load_scale,
        }
    }
}

impl Default for TireModelSpec {
    fn default() -> Self {
        Self::DEFAULT
    }
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

/// Wheel spin state.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct WheelState {
    /// Wheel angular speed about its rolling axle, rad/s.
    pub spin_rate_rad_s: f64,
}

/// Loads, telemetry and spin rate of change of a wheel at one state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelEvaluation {
    /// Ground reaction on the hub in world axes, N.
    pub force_world_n: DVec3,
    /// Normal load magnitude, N.
    pub contact_load_n: f64,
    /// Normalized longitudinal slip ratio.
    pub slip_ratio: f64,
    /// Tyre sideslip angle, rad.
    pub slip_angle_rad: f64,
    /// Angular acceleration of the wheel about its axle, rad/s².
    pub spin_accel_rad_s2: f64,
}

impl WheelEvaluation {
    /// The ground reaction as a wrench at the hub of `spec`.
    pub fn wrench(&self, spec: &WheelSpec, hub: &PointKinematics) -> WrenchAtSite {
        WrenchAtSite {
            site: spec.component,
            point_local_m: spec.hub_offset_local_m,
            force_local_n: hub.to_link(self.force_world_n),
            torque_local_nm: DVec3::ZERO,
        }
    }
}

impl WheelSpec {
    /// Suspension, traction, braking and cornering forces against `surface`
    /// with friction-circle saturation, and the spin acceleration: a pure
    /// function of its arguments, valid at any integrator stage.
    pub fn evaluate(
        &self,
        state: &WheelState,
        cmd: &WheelCommand,
        hub: &PointKinematics,
        surface: &SurfaceSample,
        wheel_spin_inertia_kg_m2: f64,
    ) -> Result<WheelEvaluation, MultibodyError> {
        if !wheel_spin_inertia_kg_m2.is_finite() || wheel_spin_inertia_kg_m2 <= 0.0 {
            return Err(MultibodyError::InvalidParameter {
                field: "wheel_spin_inertia_kg_m2",
                value: wheel_spin_inertia_kg_m2,
            });
        }
        let spin = state.spin_rate_rad_s;
        let brake = cmd.brake_torque_nm.max(0.0) * spin.signum();
        let normal = surface.outward_normal_world.normalize_or(DVec3::Z);
        let bottom_world = hub.position_world_m - normal * self.wheel_radius_m;
        let deflection = (surface.surface_point_world_m - bottom_world).dot(normal);
        if surface.medium != SurfaceMedium::Solid || deflection <= 0.0 {
            return Ok(WheelEvaluation {
                force_world_n: DVec3::ZERO,
                contact_load_n: 0.0,
                slip_ratio: 0.0,
                slip_angle_rad: 0.0,
                spin_accel_rad_s2: (cmd.drive_torque_nm - brake) / wheel_spin_inertia_kg_m2,
            });
        }

        // Suspension rate and tyre slip are against the surface's own
        // motion.
        let hub_world_vel = hub.velocity_world_m_s - surface.velocity_world_m_s;
        let v_n = hub_world_vel.dot(normal);
        let fn_mag = (self.spring_n_m * deflection - self.damper_n_s_m * v_n).max(0.0);

        let fwd_raw = hub.rotation_world_from_link * self.forward_axis_local;
        let fwd_unsteered = (fwd_raw - normal * fwd_raw.dot(normal)).normalize_or(DVec3::X);
        let right_raw = normal.cross(fwd_unsteered).normalize_or(DVec3::Y);
        let (sin_s, cos_s) = libm::sincos(cmd.steer_angle_rad);
        let fwd_t = (fwd_unsteered * cos_s + right_raw * sin_s).normalize();
        let lat_t = normal.cross(fwd_t).normalize();

        let (f_long, f_lat, slip_ratio, slip_angle) = compute_planar_tire_forces(
            self,
            spin,
            hub_world_vel,
            fwd_t,
            lat_t,
            fn_mag,
            surface.mu_dynamic_override.unwrap_or(self.peak_mu),
        );

        let road_torque = -f_long * self.wheel_radius_m;
        let net_spin_torque = cmd.drive_torque_nm + road_torque - brake;
        Ok(WheelEvaluation {
            force_world_n: normal * fn_mag + fwd_t * f_long + lat_t * f_lat,
            contact_load_n: fn_mag,
            slip_ratio,
            slip_angle_rad: slip_angle,
            spin_accel_rad_s2: net_spin_torque / wheel_spin_inertia_kg_m2,
        })
    }
}

/// A wheel as a force element; its one continuous state is the spin rate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Wheel {
    /// Wheel, suspension and tyre properties.
    pub spec: WheelSpec,
    /// Inertia of the wheel about its axle, kg·m².
    pub spin_inertia_kg_m2: f64,
    /// Command held over the step.
    pub command: WheelCommand,
}

impl<I: SurfaceQuery + ?Sized> ForceElement<I> for Wheel {
    fn state_len(&self) -> usize {
        1
    }

    fn state_scale(&self, model: &ModelScale, scale: &mut [f64]) {
        let radius = self.spec.wheel_radius_m;
        let floor = self.spec.tire.slip_speed_floor_m_s;
        scale.fill(if radius > 0.0 && floor > 0.0 {
            floor / radius
        } else {
            model.angular_rate_rad_s()
        });
    }

    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &[f64],
        _modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let hub = ctx
            .kinematics
            .point(self.spec.component, self.spec.hub_offset_local_m)?;
        let surface = ctx.inputs.surface_at(hub.position_world_m, ctx.time_s)?;
        let typed = WheelState {
            spin_rate_rad_s: read1(state, 0),
        };
        let out = self.spec.evaluate(
            &typed,
            &self.command,
            &hub,
            &surface,
            self.spin_inertia_kg_m2,
        )?;
        write1(state_dot, 0, out.spin_accel_rad_s2);
        loads.add_wrench(&out.wrench(&self.spec, &hub))
    }
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
        .max(spec.tire.slip_speed_floor_m_s);
    let slip_ratio = ((spin_rate_rad_s * spec.wheel_radius_m - v_fwd) / v_ref).clamp(-1.0, 1.0);
    let slip_angle = libm::atan2(v_lat, v_fwd.abs().max(spec.tire.slip_angle_speed_floor_m_s));

    let load_scale =
        (fn_mag / (fn_mag + spec.tire.load_sensitivity_n)).max(spec.tire.min_load_scale);
    let f_long_raw = spec.longitudinal_stiffness_n * slip_ratio * load_scale
        - spec.rolling_resistance_coeff * fn_mag * v_fwd.signum();
    let f_lat_raw = -spec.cornering_stiffness_n_rad * slip_angle * load_scale;

    let f_max = (mu * fn_mag).max(0.0);
    let planar_mag = libm::sqrt(f_long_raw * f_long_raw + f_lat_raw * f_lat_raw);
    // `f_max >= 0`, so exceeding it also guarantees a nonzero divisor.
    if planar_mag > f_max {
        let s = f_max / planar_mag;
        (f_long_raw * s, f_lat_raw * s, slip_ratio, slip_angle)
    } else {
        (f_long_raw, f_lat_raw, slip_ratio, slip_angle)
    }
}
