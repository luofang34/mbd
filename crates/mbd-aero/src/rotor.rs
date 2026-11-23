use core::f64::consts::PI;

use glam::DVec3;
use mbd_core::{AmbientGas, MbdError};

use crate::flow::{AeroWrench, LinkFlow};

use crate::validation::{
    check_finite, check_finite_vec, check_nonnegative, check_positive, check_unit_vec, invalid,
};

#[cfg(test)]
mod tests;

/// Configuration for a unified Blade-Element-Momentum (BEM) rotor or propeller
/// element, covering fixed-pitch/constant-speed propellers (C172, P-38G),
/// coaxial counter-rotating turboprops (Tu-95MS), and Mars `CO2` coaxial
/// helicopters (Ingenuity).
///
/// # Validity
///
/// Blade sections are conventional airfoils with a linear lift slope, a
/// stall clamp and a profile drag `cd0 + k/√Re`, whose second term follows
/// laminar skin-friction scaling (Blasius, `C_f ∝ Re^-1/2`). Airfoil
/// behaviour of that kind holds for section Reynolds numbers of order 10⁴
/// and above; below it, laminar separation changes lift slope and stall in
/// ways this model does not represent (Lissaman 1983, Annu. Rev. Fluid
/// Mech. 15:223; Mueller & DeLaurier 2003, Annu. Rev. Fluid Mech. 35:89).
/// The model does not clamp the Reynolds number: the friction term grows as
/// Re falls, and [`RotorWrench::mean_reynolds`] reports where a disk
/// operates so a host can check it. Insect-scale flapping is
/// [`crate::FlappingWingAero`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RotorBemParams {
    /// Outer blade tip radius, m.
    pub radius_m: f64,
    /// Hub cutout inner radius, m (`< radius_m`).
    pub hub_radius_m: f64,
    /// Number of blades per rotor disk (`>= 1`).
    pub blade_count: u8,
    /// Mean aerodynamic blade chord, m.
    pub chord_m: f64,
    /// Blade pitch twist at `hub_radius_m`, rad.
    pub root_pitch_rad: f64,
    /// Blade pitch twist at `radius_m`, rad.
    pub tip_pitch_rad: f64,
    /// Thin-airfoil lift curve slope per radian.
    pub cl_alpha: f64,
    /// Maximum section lift coefficient magnitude before stall.
    pub cl_max: f64,
    /// High-Reynolds minimum section parasitic drag coefficient.
    pub cd0_high_re: f64,
    /// Low-Reynolds viscous drag penalty factor (`cd0 = cd0_high_re + low_re_drag_factor / sqrt(Re)`).
    pub low_re_drag_factor: f64,
    /// Local blade Mach number at which compressibility wave drag begins.
    pub critical_tip_mach: f64,
    /// Induced inflow factor (`1.0` for isolated rotor, `~1.12..1.18` for coaxial pair interference).
    pub coaxial_interference_factor: f64,
    /// Spin direction sign around `shaft_axis_body`: `+1.0` or `-1.0`.
    pub rotation_direction: f64,
    /// Rotor hub centre in component frame, m.
    pub hub_position_body_m: DVec3,
    /// Unit shaft axis pointing along positive thrust direction in component frame.
    pub shaft_axis_body: DVec3,
    /// Number of radial blade annuli (`4..=32`).
    pub radial_stations: u8,
    /// Control channel carrying the rotor speed, rad/s. The host owns the
    /// drive train and supplies the speed it computes.
    pub omega_channel: usize,
    /// Control channel carrying collective pitch, rad; zero without one.
    pub collective_channel: Option<usize>,
    /// Control channel carrying lateral cyclic pitch, rad.
    pub cyclic_lat_channel: Option<usize>,
    /// Control channel carrying longitudinal cyclic pitch, rad.
    pub cyclic_lon_channel: Option<usize>,
}

/// Spin and pitch commands of a [`RotorBemAero`] disk.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RotorState {
    /// Rotor rotational speed magnitude, rad/s (`>= 0`).
    pub omega_rad_s: f64,
    /// Collective pitch offset added to geometric blade twist, rad.
    pub collective_rad: f64,
    /// Lateral cyclic pitch cosine/sine amplitude, rad.
    pub cyclic_lat_rad: f64,
    /// Longitudinal cyclic pitch cosine/sine amplitude, rad.
    pub cyclic_lon_rad: f64,
}

impl Default for RotorState {
    fn default() -> Self {
        Self {
            omega_rad_s: 0.0,
            collective_rad: 0.0,
            cyclic_lat_rad: 0.0,
            cyclic_lon_rad: 0.0,
        }
    }
}

/// Complete aerodynamic and shaft-power output of [`RotorBemAero::evaluate_state`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RotorWrench {
    /// Aerodynamic wrench at `hub_position_body_m`.
    pub aero: AeroWrench,
    /// Shaft aerodynamic power required to maintain `omega_rad_s`, W.
    pub shaft_power_w: f64,
    /// Blade tip Mach number relative to ambient gas sound speed.
    pub tip_mach: f64,
    /// Thrust-weighted mean blade section Reynolds number.
    pub mean_reynolds: f64,
}

/// Unified BEM rotor and propeller aerodynamics element.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RotorBemAero {
    params: RotorBemParams,
    axis: DVec3,
    e1: DVec3,
    e2: DVec3,
}

impl RotorBemAero {
    /// Validates geometry, radial discretization, and orientation basis.
    pub fn new(params: RotorBemParams) -> Result<Self, MbdError> {
        validate_rotor_params(&params)?;
        let axis = check_unit_vec("shaft_axis_body", params.shaft_axis_body)?;
        let (e1, e2) = orthonormal_disk_basis(axis);
        Ok(Self {
            params,
            axis,
            e1,
            e2,
        })
    }

    /// Validated rotor configuration.
    pub const fn params(&self) -> &RotorBemParams {
        &self.params
    }

    /// Evaluates thrust, in-plane forces, aerodynamic + reaction torque, tip Mach,
    /// mean Reynolds number, and shaft power for an arbitrary live [`RotorState`].
    pub fn evaluate_state(
        &self,
        flow: &LinkFlow<'_>,
        state: &RotorState,
    ) -> Result<RotorWrench, MbdError> {
        flow.validate()?;
        validate_rotor_state(state)?;
        let rho = flow.ambient.density_kg_m3;
        if rho == 0.0 || state.omega_rad_s == 0.0 {
            return Ok(RotorWrench {
                aero: AeroWrench {
                    reference_point_link_m: self.params.hub_position_body_m,
                    ..AeroWrench::ZERO
                },
                shaft_power_w: 0.0,
                tip_mach: 0.0,
                mean_reynolds: 0.0,
            });
        }
        let p = &self.params;
        let v_hub =
            flow.velocity_air_link_m_s + flow.angular_rate_link_rad_s.cross(p.hub_position_body_m);
        let v_axial = v_hub.dot(self.axis);
        let v_e1 = v_hub.dot(self.e1);
        let v_e2 = v_hub.dot(self.e2);
        let v_edge = libm::sqrt(v_e1 * v_e1 + v_e2 * v_e2);

        let n = usize::from(p.radial_stations);
        let dr = (p.radius_m - p.hub_radius_m) / (n as f64);
        let mut acc = BemAccum::default();
        for k in 0..n {
            let r = p.hub_radius_m + ((k as f64) + 0.5) * dr;
            let frac = (r - p.hub_radius_m) / (p.radius_m - p.hub_radius_m);
            let twist = p.root_pitch_rad + (p.tip_pitch_rad - p.root_pitch_rad) * frac;
            self.accumulate_annulus(
                &flow.ambient,
                state,
                v_axial,
                v_e1,
                v_e2,
                v_edge,
                r,
                dr,
                twist,
                &mut acc,
            );
        }
        let tip_mach = if flow.ambient.sound_speed_m_s > 0.0 {
            (state.omega_rad_s * p.radius_m + v_edge) / flow.ambient.sound_speed_m_s
        } else {
            0.0
        };
        // Reaction torque opposes rotor spin; cyclic & advancing-retreating disk moments tilt thrust/hub.
        let force_link_n = self.axis * acc.thrust - self.e1 * acc.h_e1 - self.e2 * acc.h_e2;
        let torque_body_nm = -self.axis * (p.rotation_direction * acc.torque)
            + self.e1 * acc.moment_e1
            + self.e2 * acc.moment_e2;
        Ok(RotorWrench {
            aero: AeroWrench {
                reference_point_link_m: p.hub_position_body_m,
                force_link_n,
                moment_ref_link_nm: torque_body_nm,
                stagnation_heat_flux_w_m2: 0.0,
            },
            shaft_power_w: acc.torque * state.omega_rad_s,
            tip_mach,
            mean_reynolds: acc.re_sum / (n as f64),
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn accumulate_annulus(
        &self,
        ambient: &AmbientGas,
        state: &RotorState,
        v_axial: f64,
        v_e1: f64,
        v_e2: f64,
        v_edge: f64,
        r: f64,
        dr: f64,
        twist_rad: f64,
        acc: &mut BemAccum,
    ) {
        let p = &self.params;
        let omega_r = state.omega_rad_s * r;
        let theta_mean = twist_rad + state.collective_rad;
        let vi_bound = omega_r * 0.6 + v_axial.abs() + 25.0;
        let v_i = solve_induced_inflow(
            p, ambient, omega_r, theta_mean, v_axial, v_edge, r, dr, vi_bound,
        );
        let az_samples = [0.0, 0.5 * PI, PI, 1.5 * PI];
        let mut dt_sum = 0.0;
        let mut dq_sum = 0.0;
        let mut h1_sum = 0.0;
        let mut h2_sum = 0.0;
        let mut m1_sum = 0.0;
        let mut m2_sum = 0.0;
        let mut re_sum = 0.0;
        for &psi in &az_samples {
            let (sin_p, cos_p) = libm::sincos(psi);
            let dir = p.rotation_direction;
            let u_t = (omega_r + dir * (-v_e1 * sin_p + v_e2 * cos_p)).max(0.01);
            let u_p = v_axial + p.coaxial_interference_factor * v_i;
            let theta = theta_mean + state.cyclic_lon_rad * cos_p + state.cyclic_lat_rad * sin_p;
            let (dt, dq, df_drag, re) = element_loads(p, ambient, u_t, u_p, theta, r, dr);
            dt_sum += dt;
            dq_sum += dq;
            re_sum += re;
            h1_sum += df_drag * (-sin_p * dir);
            h2_sum += df_drag * (cos_p * dir);
            m1_sum += dt * r * sin_p;
            m2_sum += -dt * r * cos_p;
        }
        let inv = 0.25;
        acc.thrust += dt_sum * inv;
        acc.torque += dq_sum * inv;
        acc.h_e1 += h1_sum * inv;
        acc.h_e2 += h2_sum * inv;
        acc.moment_e1 += m1_sum * inv;
        acc.moment_e2 += m2_sum * inv;
        acc.re_sum += re_sum * inv;
    }
}

impl RotorBemAero {
    /// The commands read from the flow's control channels.
    pub fn commands(&self, flow: &LinkFlow<'_>) -> RotorState {
        let p = &self.params;
        RotorState {
            omega_rad_s: flow.control(Some(p.omega_channel)),
            collective_rad: flow.control(p.collective_channel),
            cyclic_lat_rad: flow.control(p.cyclic_lat_channel),
            cyclic_lon_rad: flow.control(p.cyclic_lon_channel),
        }
    }

    /// Load, shaft power and blade diagnostics at the commands in the
    /// flow's control channels.
    pub fn evaluate(&self, flow: &LinkFlow<'_>) -> Result<RotorWrench, MbdError> {
        self.evaluate_state(flow, &self.commands(flow))
    }
}

#[derive(Default)]
struct BemAccum {
    thrust: f64,
    torque: f64,
    h_e1: f64,
    h_e2: f64,
    moment_e1: f64,
    moment_e2: f64,
    re_sum: f64,
}

#[allow(clippy::too_many_arguments)]
fn solve_induced_inflow(
    p: &RotorBemParams,
    ambient: &AmbientGas,
    omega_r: f64,
    theta: f64,
    v_axial: f64,
    v_edge: f64,
    r: f64,
    dr: f64,
    bound: f64,
) -> f64 {
    let mut lo = -bound.min(v_axial.max(0.0) * 0.5 + 10.0);
    let mut hi = bound;
    let res = |vi: f64| {
        let u_p = v_axial + p.coaxial_interference_factor * vi;
        let (dt_bet, _, _, _) = element_loads(p, ambient, omega_r, u_p, theta, r, dr);
        let phi = libm::atan2(u_p.abs().max(1e-3), omega_r.max(1e-3));
        let prandtl_arg =
            (0.5 * f64::from(p.blade_count) * (p.radius_m - r)) / (r * libm::sin(phi).max(0.02));
        let f_tip = (2.0 / PI) * libm::acos(libm::exp(-prandtl_arg.min(50.0)).min(1.0));
        let v_disk = libm::sqrt(v_edge * v_edge + u_p * u_p);
        let dt_mom = 4.0 * PI * r * ambient.density_kg_m3 * f_tip.max(0.1) * vi * v_disk * dr;
        dt_bet - dt_mom
    };
    if res(lo) <= 0.0 {
        return lo;
    }
    if res(hi) >= 0.0 {
        return hi;
    }
    for _ in 0..22 {
        let mid = 0.5 * (lo + hi);
        if res(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

fn element_loads(
    p: &RotorBemParams,
    ambient: &AmbientGas,
    u_t: f64,
    u_p: f64,
    theta: f64,
    r: f64,
    dr: f64,
) -> (f64, f64, f64, f64) {
    let w_sq = u_t * u_t + u_p * u_p;
    let w = libm::sqrt(w_sq);
    let phi = libm::atan2(u_p, u_t);
    let alpha = theta - phi;
    let mach_loc = if ambient.sound_speed_m_s > 0.0 {
        w / ambient.sound_speed_m_s
    } else {
        0.0
    };
    let beta = libm::sqrt((1.0 - mach_loc * mach_loc).clamp(0.15, 1.0));
    let cl = (p.cl_alpha * alpha / beta).clamp(-p.cl_max, p.cl_max);
    // Inviscid gas or a section at rest: the laminar friction term has no
    // meaning there and is left out.
    let re = if ambient.dynamic_viscosity_pa_s > 0.0 {
        ambient.density_kg_m3 * w * p.chord_m / ambient.dynamic_viscosity_pa_s
    } else {
        f64::INFINITY
    };
    let friction_cd = if re > 0.0 {
        p.low_re_drag_factor / libm::sqrt(re)
    } else {
        0.0
    };
    let wave_cd = if mach_loc > p.critical_tip_mach {
        let dm = (mach_loc - p.critical_tip_mach).min(0.6);
        1.8 * dm * dm
    } else {
        0.0
    };
    let stall_cd = 1.6 * libm::sin(alpha) * libm::sin(alpha);
    let cd = p.cd0_high_re + friction_cd + stall_cd + wave_cd;
    let factor = 0.5 * ambient.density_kg_m3 * w_sq * p.chord_m * dr * f64::from(p.blade_count);
    let (sin_phi, cos_phi) = libm::sincos(phi);
    let dt = factor * (cl * cos_phi - cd * sin_phi);
    let df_t = factor * (cl * sin_phi + cd * cos_phi);
    (dt, df_t * r, df_t, re)
}

fn orthonormal_disk_basis(axis: DVec3) -> (DVec3, DVec3) {
    let seed = if axis.x.abs() < 0.9 {
        DVec3::X
    } else {
        DVec3::Z
    };
    let e1 = axis.cross(seed).normalize();
    let e2 = axis.cross(e1).normalize();
    (e1, e2)
}

fn validate_rotor_params(p: &RotorBemParams) -> Result<(), MbdError> {
    check_positive("radius_m", p.radius_m)?;
    if !p.hub_radius_m.is_finite() || p.hub_radius_m <= 0.0 || p.hub_radius_m >= p.radius_m {
        return Err(invalid("hub_radius_m", p.hub_radius_m));
    }
    if p.blade_count == 0 {
        return Err(invalid("blade_count", 0.0));
    }
    check_positive("chord_m", p.chord_m)?;
    check_finite("root_pitch_rad", p.root_pitch_rad)?;
    check_finite("tip_pitch_rad", p.tip_pitch_rad)?;
    check_positive("cl_alpha", p.cl_alpha)?;
    check_positive("cl_max", p.cl_max)?;
    check_nonnegative("cd0_high_re", p.cd0_high_re)?;
    check_nonnegative("low_re_drag_factor", p.low_re_drag_factor)?;
    check_positive("critical_tip_mach", p.critical_tip_mach)?;
    if !p.coaxial_interference_factor.is_finite() || p.coaxial_interference_factor < 1.0 {
        return Err(invalid(
            "coaxial_interference_factor",
            p.coaxial_interference_factor,
        ));
    }
    if (p.rotation_direction - 1.0).abs() > 1e-6 && (p.rotation_direction + 1.0).abs() > 1e-6 {
        return Err(invalid("rotation_direction", p.rotation_direction));
    }
    check_finite_vec("hub_position_body_m", p.hub_position_body_m)?;
    if !(4..=32).contains(&p.radial_stations) {
        return Err(invalid("radial_stations", f64::from(p.radial_stations)));
    }
    Ok(())
}

fn validate_rotor_state(s: &RotorState) -> Result<(), MbdError> {
    check_nonnegative("omega_rad_s", s.omega_rad_s)?;
    check_finite("collective_rad", s.collective_rad)?;
    check_finite("cyclic_lat_rad", s.cyclic_lat_rad)?;
    check_finite("cyclic_lon_rad", s.cyclic_lon_rad)
}
