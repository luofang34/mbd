use glam::DVec3;
use mbd_core::MbdError;

use crate::flow::{AeroWrench, LinkFlow};

use crate::validation::{
    check_finite_vec, check_nonnegative, check_positive, check_unit_vec, invalid, smoothstep01,
};

#[cfg(test)]
mod tests;

/// Configuration for a unified subsonic and supersonic disk-gap-band (DGB)
/// parachute element covering Earth capsule main/drogue chutes (Vostok-1,
/// Gemini 8, Apollo 11) and Martian supersonic parachutes (Perseverance).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParachuteParams {
    /// Nominal inflated reference canopy area $S_0$, m².
    pub nominal_area_m2: f64,
    /// Subsonic steady drag coefficient ($M \le 0.7$).
    pub cd_subsonic: f64,
    /// Supersonic drag coefficient ($M \ge 1.8$), e.g. Mars DGB wake-loss schedule.
    pub cd_supersonic: f64,
    /// Transverse restoring normal-force slope wrt canopy incidence.
    pub cn_alpha: f64,
    /// Riser attachment point in component local axes, m.
    pub riser_attach_body_m: DVec3,
    /// Unit vector along canopy shroud lines pointing aft from `riser_attach_body_m`
    /// toward the canopy skirt.
    pub canopy_axis_body: DVec3,
    /// Characteristic canopy filling time constant $\tau$ at reference dynamic pressure, s.
    pub inflation_time_constant_s: f64,
    /// Forebody wake velocity deficit multiplier in `(0.0..=1.0]` (`1.0` = clean air).
    pub wake_velocity_ratio: f64,
    /// Rotational aerodynamic damping coefficient of the suspended canopy.
    pub rotational_damping_coeff: f64,
    /// Dynamic pressure at which the canopy fills with time constant
    /// `inflation_time_constant_s`, Pa.
    pub reference_dynamic_pressure_pa: f64,
    /// Control channel whose positive value fires the mortar; the
    /// deployment latches at the next accepted step boundary.
    pub deploy_channel: Option<usize>,
    /// Control channel carrying the reefing area fraction in `0..=1`; a
    /// disreefed canopy without one.
    pub reefing_channel: Option<usize>,
    /// Whether the canopy is already deployed when the state is created.
    pub deployed_at_start: bool,
    /// Inflation fraction when the state is created, `0..=1`.
    pub inflation_at_start: f64,
}

/// Deployment, reefing and inflation of a [`ParachuteCanopyAero`] at one
/// instant: the argument of its pure load function.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParachuteState {
    /// Whether the mortar/drogue has fired and lines are out.
    pub deployed: bool,
    /// Fractional inflation progress in `0.0..=1.0`.
    pub inflation_fraction: f64,
    /// Reefing skirt opening multiplier in `0.0..=1.0` (`1.0` = disreefed full canopy).
    pub reefing_area_fraction: f64,
}

impl Default for ParachuteState {
    fn default() -> Self {
        Self {
            deployed: false,
            inflation_fraction: 0.0,
            reefing_area_fraction: 1.0,
        }
    }
}

/// Unified subsonic/supersonic parachute canopy element.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParachuteCanopyAero {
    params: ParachuteParams,
    axis: DVec3,
}

impl ParachuteCanopyAero {
    /// Validates parameters and the canopy's starting state.
    pub fn new(params: ParachuteParams) -> Result<Self, MbdError> {
        check_positive("nominal_area_m2", params.nominal_area_m2)?;
        check_nonnegative("cd_subsonic", params.cd_subsonic)?;
        check_nonnegative("cd_supersonic", params.cd_supersonic)?;
        check_nonnegative("cn_alpha", params.cn_alpha)?;
        check_finite_vec("riser_attach_body_m", params.riser_attach_body_m)?;
        let axis = check_unit_vec("canopy_axis_body", params.canopy_axis_body)?;
        check_positive(
            "inflation_time_constant_s",
            params.inflation_time_constant_s,
        )?;
        if !params.wake_velocity_ratio.is_finite()
            || params.wake_velocity_ratio <= 0.0
            || params.wake_velocity_ratio > 1.0
        {
            return Err(invalid("wake_velocity_ratio", params.wake_velocity_ratio));
        }
        check_nonnegative("rotational_damping_coeff", params.rotational_damping_coeff)?;
        check_positive(
            "reference_dynamic_pressure_pa",
            params.reference_dynamic_pressure_pa,
        )?;
        validate_parachute_state(&ParachuteState {
            deployed: params.deployed_at_start,
            inflation_fraction: params.inflation_at_start,
            reefing_area_fraction: 1.0,
        })?;
        Ok(Self { params, axis })
    }

    /// Rate of the inflation fraction, 1/s: a deployed canopy fills
    /// toward full at `(1 − f)·k(q)/τ`, where `k = √(q/q_ref)` clamped to
    /// `0.5..=2` makes a canopy at higher dynamic pressure fill faster.
    pub fn inflation_rate(&self, state: &ParachuteState, dynamic_pressure_pa: f64) -> f64 {
        if !state.deployed {
            return 0.0;
        }
        let p = &self.params;
        let pressure_scale =
            libm::sqrt((dynamic_pressure_pa / p.reference_dynamic_pressure_pa).clamp(0.25, 4.0));
        (1.0 - state.inflation_fraction) * pressure_scale / p.inflation_time_constant_s
    }

    /// Dynamic pressure the canopy sees: the wake-reduced flow at the riser
    /// attachment, Pa.
    pub fn canopy_dynamic_pressure_pa(&self, flow: &LinkFlow<'_>) -> f64 {
        let v = self.canopy_velocity(flow);
        0.5 * flow.ambient.density_kg_m3 * v.length_squared()
    }

    fn canopy_velocity(&self, flow: &LinkFlow<'_>) -> DVec3 {
        flow.velocity_air_link_m_s * self.params.wake_velocity_ratio
            + flow
                .angular_rate_link_rad_s
                .cross(self.params.riser_attach_body_m)
    }

    /// Reefing area fraction commanded in the flow's control channels.
    pub fn reefing_fraction(&self, flow: &LinkFlow<'_>) -> f64 {
        self.params
            .reefing_channel
            .map_or(1.0, |ch| flow.control(Some(ch)))
    }

    /// Whether the flow's control channels command deployment.
    pub fn deploy_commanded(&self, flow: &LinkFlow<'_>) -> bool {
        flow.control(self.params.deploy_channel) > 0.0
    }

    /// Evaluates parachute forces and moments for an arbitrary live [`ParachuteState`].
    pub fn evaluate_state(
        &self,
        flow: &LinkFlow<'_>,
        state: &ParachuteState,
    ) -> Result<AeroWrench, MbdError> {
        flow.validate()?;
        validate_parachute_state(state)?;
        let p = &self.params;
        if !state.deployed || flow.ambient.density_kg_m3 == 0.0 {
            return Ok(AeroWrench {
                reference_point_link_m: p.riser_attach_body_m,
                ..AeroWrench::ZERO
            });
        }
        let v = self.canopy_velocity(flow);
        let speed = v.length();
        if speed == 0.0 {
            return Ok(AeroWrench {
                reference_point_link_m: p.riser_attach_body_m,
                ..AeroWrench::ZERO
            });
        }
        let mach = if flow.ambient.sound_speed_m_s > 0.0 {
            flow.velocity_air_link_m_s.length() / flow.ambient.sound_speed_m_s
        } else {
            0.0
        };
        let m_blend = smoothstep01((mach - 0.8) / 1.0);
        let cd = p.cd_subsonic + (p.cd_supersonic - p.cd_subsonic) * m_blend;
        let area_eff = p.nominal_area_m2 * state.inflation_fraction * state.reefing_area_fraction;
        let q = 0.5 * flow.ambient.density_kg_m3 * speed * speed;
        let u_flow = v / speed;

        // Axial projection along canopy axis + normal restoring component perpendicular to axis.
        let v_ax = v.dot(self.axis);
        let u_lat = u_flow - self.axis * (v_ax / speed);
        let drag_force = -q * area_eff * cd * u_flow;
        let normal_restoring = -q * area_eff * p.cn_alpha * u_lat;
        let total_force = drag_force + normal_restoring;

        let diameter = libm::sqrt(4.0 * area_eff.max(1e-6) / core::f64::consts::PI);
        let damp = -0.25
            * flow.ambient.density_kg_m3
            * speed
            * area_eff
            * diameter
            * diameter
            * p.rotational_damping_coeff
            * flow.angular_rate_link_rad_s;

        Ok(AeroWrench {
            reference_point_link_m: p.riser_attach_body_m,
            force_link_n: total_force,
            moment_ref_link_nm: damp,
            stagnation_heat_flux_w_m2: 0.0,
        })
    }

    /// Validated configuration parameters.
    pub const fn params(&self) -> &ParachuteParams {
        &self.params
    }
}

fn validate_parachute_state(state: &ParachuteState) -> Result<(), MbdError> {
    if !state.inflation_fraction.is_finite() || !(0.0..=1.0).contains(&state.inflation_fraction) {
        return Err(invalid("inflation_fraction", state.inflation_fraction));
    }
    if !state.reefing_area_fraction.is_finite()
        || !(0.0..=1.0).contains(&state.reefing_area_fraction)
    {
        return Err(invalid(
            "reefing_area_fraction",
            state.reefing_area_fraction,
        ));
    }
    Ok(())
}
