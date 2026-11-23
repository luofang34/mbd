use glam::{DQuat, DVec3};
use mbd_core::{AmbientGas, ComponentId, FaultState, ThermalLoad, WrenchAtSite};

use crate::error::{
    SystemsError, check_non_negative, check_positive, check_unit_interval, check_vec_finite,
    normalize_axis,
};
use crate::fluid::{FluidNetwork, TankId};

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;
const SEA_LEVEL_PRESSURE_PA: f64 = 101_325.0;

/// Specification for a monopropellant, bipropellant, or cold-gas rocket/RCS thruster.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiquidRocketSpec {
    /// Host component receiving the thrust wrench and thermal load.
    pub site: ComponentId,
    /// Nozzle gimbal/throat location in `site`'s local axes, m.
    pub mount_offset_local_m: DVec3,
    /// Unit nominal thrust direction acting on the vehicle in `site`'s local axes.
    pub thrust_axis_local: DVec3,
    /// Maximum 2-axis gimbal deflection magnitude, rad (`0.0` for fixed RCS/main nozzle).
    pub gimbal_max_rad: f64,
    /// Maximum vacuum thrust at full throttle, N.
    pub max_vacuum_thrust_n: f64,
    /// Vacuum specific impulse, s.
    pub isp_vac_s: f64,
    /// Sea-level specific impulse at 101,325 Pa ambient pressure, s (`<= isp_vac_s`).
    pub isp_sl_s: f64,
    /// Fuel tank feeding this engine/thruster (or single tank for monopropellant/cold gas).
    pub fuel_tank: Option<TankId>,
    /// Oxidizer tank feeding this engine (`None` for monopropellant or cold gas).
    pub oxidizer_tank: Option<TankId>,
    /// Oxidizer-to-fuel mass ratio (`mdot_ox / mdot_fuel`, `0.0` when `oxidizer_tank` is `None`).
    pub mixture_ratio_o_f: f64,
    /// Minimum throttle fraction when active (`0.0` for bang-bang RCS or deep-throttling engines).
    pub min_throttle: f64,
    /// First-order thrust spool-up time constant, s (`0.0` for instantaneous response).
    pub startup_tau_s: f64,
    /// First-order thrust tail-off time constant, s (`0.0` for instantaneous response).
    pub shutdown_tau_s: f64,
    /// Chamber/nozzle heat load transferred to `site` at 100% thrust, W.
    pub nominal_heat_to_site_w: f64,
}

impl LiquidRocketSpec {
    /// Validates geometry, thrust/Isp envelope, and tank mixture ratio.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_vec_finite(
            "liquid_rocket.mount_offset_local_m",
            self.mount_offset_local_m,
        )?;
        normalize_axis("liquid_rocket.thrust_axis_local", self.thrust_axis_local)?;
        check_non_negative("liquid_rocket.gimbal_max_rad", self.gimbal_max_rad)?;
        check_positive(
            "liquid_rocket.max_vacuum_thrust_n",
            self.max_vacuum_thrust_n,
        )?;
        check_positive("liquid_rocket.isp_vac_s", self.isp_vac_s)?;
        check_positive("liquid_rocket.isp_sl_s", self.isp_sl_s)?;
        if self.isp_sl_s > self.isp_vac_s {
            return Err(SystemsError::InvalidConfig {
                detail: alloc::format!(
                    "liquid rocket isp_sl_s ({}) exceeds isp_vac_s ({})",
                    self.isp_sl_s,
                    self.isp_vac_s
                ),
            });
        }
        check_non_negative("liquid_rocket.mixture_ratio_o_f", self.mixture_ratio_o_f)?;
        check_unit_interval("liquid_rocket.min_throttle", self.min_throttle)?;
        check_non_negative("liquid_rocket.startup_tau_s", self.startup_tau_s)?;
        check_non_negative("liquid_rocket.shutdown_tau_s", self.shutdown_tau_s)?;
        check_non_negative(
            "liquid_rocket.nominal_heat_to_site_w",
            self.nominal_heat_to_site_w,
        )?;
        Ok(())
    }
}

/// Dynamic runtime state of a [`LiquidRocketSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiquidRocketState {
    /// Normalized throttle command `[0.0, 1.0]`.
    pub throttle_cmd: f64,
    /// Gimbal pitch and yaw commands `[pitch_rad, yaw_rad]` about axes perpendicular to thrust.
    pub gimbal_cmd_rad: [f64; 2],
    /// Actual instantaneous normalized chamber pressure / thrust fraction.
    pub actual_throttle: f64,
    /// Injected engine/valve fault (`ShortCircuit` / `StuckOn` force uncommanded firing).
    pub fault: FaultState,
}

impl Default for LiquidRocketState {
    fn default() -> Self {
        Self {
            throttle_cmd: 0.0,
            gimbal_cmd_rad: [0.0, 0.0],
            actual_throttle: 0.0,
            fault: FaultState::Nominal,
        }
    }
}

/// Inputs for advancing one liquid/cold-gas/RCS thruster.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiquidRocketStepInput {
    /// Time step duration, s.
    pub dt_s: f64,
    /// Undisturbed ambient gas (provides static backpressure `ambient.pressure_pa`).
    pub ambient: AmbientGas,
    /// Propellant cut-off valve / pyro-isolation bit (shutting upstream manifold clears stuck firing).
    pub manifold_isolated: bool,
}

/// Force, propellant consumption, and heat output from one liquid rocket step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LiquidRocketStepOutput {
    /// Net wrench delivered to `spec.site` in its local frame.
    pub wrench_local: WrenchAtSite,
    /// Fuel mass withdrawn from `fuel_tank` over `dt_s`, kg.
    pub fuel_consumed_kg: f64,
    /// Oxidizer mass withdrawn from `oxidizer_tank` over `dt_s`, kg.
    pub oxidizer_consumed_kg: f64,
    /// Realized thrust magnitude after ambient backpressure correction, N.
    pub thrust_n: f64,
    /// Whether combustion was cut or throttled down due to tank unporting or depletion.
    pub propellant_starved: bool,
    /// Conductive/radiative heat load rejected into `spec.site`.
    pub thermal_load: ThermalLoad,
}

/// Steps a liquid rocket or cold-gas/RCS thruster over `dt_s`, strictly withdrawing
/// propellant from `fluids` and respecting `FaultState` and tank `FeedAvailability`.
pub fn step_liquid_rocket(
    spec: &LiquidRocketSpec,
    state: &mut LiquidRocketState,
    fluids: &mut FluidNetwork,
    input: LiquidRocketStepInput,
) -> Result<LiquidRocketStepOutput, SystemsError> {
    spec.validate()?;
    state.fault.validate()?;
    input.ambient.validate()?;
    check_non_negative("liquid_rocket.dt_s", input.dt_s)?;

    let target_throttle = if input.manifold_isolated {
        0.0
    } else {
        effective_throttle_target(spec, state)?
    };
    state.actual_throttle = advance_throttle_lag(
        state.actual_throttle,
        target_throttle,
        spec.startup_tau_s,
        spec.shutdown_tau_s,
        input.dt_s,
    );

    let vac_thrust_req = spec.max_vacuum_thrust_n * state.actual_throttle;
    let mdot_total = vac_thrust_req / (spec.isp_vac_s * STANDARD_GRAVITY_M_S2);
    let (fuel_req, ox_req) = split_propellant_demand(spec, mdot_total * input.dt_s);
    let (fuel_consumed_kg, oxidizer_consumed_kg, feed_scale) = withdraw_liquid_propellant(
        spec,
        fluids,
        input.dt_s,
        input.manifold_isolated,
        fuel_req,
        ox_req,
    )?;

    let propellant_starved = (fuel_req > 0.0 || ox_req > 0.0) && feed_scale < 0.999_999;
    let realized_fraction = state.actual_throttle * feed_scale;
    if propellant_starved {
        state.actual_throttle = realized_fraction;
    }

    let pressure_loss_max = spec.max_vacuum_thrust_n * (1.0 - spec.isp_sl_s / spec.isp_vac_s);
    let ambient_loss = pressure_loss_max * (input.ambient.pressure_pa / SEA_LEVEL_PRESSURE_PA);
    let thrust_n =
        (spec.max_vacuum_thrust_n * realized_fraction - ambient_loss * realized_fraction).max(0.0);

    let thrust_dir = gimbaled_thrust_direction(
        spec.thrust_axis_local,
        state.gimbal_cmd_rad,
        spec.gimbal_max_rad,
    );
    Ok(LiquidRocketStepOutput {
        wrench_local: WrenchAtSite {
            site: spec.site,
            point_local_m: spec.mount_offset_local_m,
            force_local_n: thrust_dir * thrust_n,
            torque_local_nm: DVec3::ZERO,
        },
        fuel_consumed_kg,
        oxidizer_consumed_kg,
        thrust_n,
        propellant_starved,
        thermal_load: ThermalLoad {
            component: spec.site,
            heat_rate_w: spec.nominal_heat_to_site_w * realized_fraction,
        },
    })
}

fn withdraw_liquid_propellant(
    spec: &LiquidRocketSpec,
    fluids: &mut FluidNetwork,
    dt_s: f64,
    manifold_isolated: bool,
    fuel_req: f64,
    ox_req: f64,
) -> Result<(f64, f64, f64), SystemsError> {
    if manifold_isolated {
        return Ok((0.0, 0.0, 0.0));
    }
    if dt_s <= 0.0 || (fuel_req <= 0.0 && ox_req <= 0.0) {
        return Ok((0.0, 0.0, 1.0));
    }
    let r_fuel = match spec.fuel_tank {
        Some(tid) if fuel_req > 0.0 => (fluids.usable_propellant_kg(tid)? / fuel_req).min(1.0),
        _ => 1.0,
    };
    let r_ox = match spec.oxidizer_tank {
        Some(tid) if ox_req > 0.0 => (fluids.usable_propellant_kg(tid)? / ox_req).min(1.0),
        _ => 1.0,
    };
    let feed_scale = r_fuel.min(r_ox).clamp(0.0, 1.0);
    let fuel_consumed_kg = match spec.fuel_tank {
        Some(tid) if fuel_req > 0.0 && feed_scale > 0.0 => {
            fluids.withdraw_propellant(tid, fuel_req * feed_scale)?
        }
        _ => 0.0,
    };
    let oxidizer_consumed_kg = match spec.oxidizer_tank {
        Some(tid) if ox_req > 0.0 && feed_scale > 0.0 => {
            fluids.withdraw_propellant(tid, ox_req * feed_scale)?
        }
        _ => 0.0,
    };
    Ok((fuel_consumed_kg, oxidizer_consumed_kg, feed_scale))
}

fn effective_throttle_target(
    spec: &LiquidRocketSpec,
    state: &LiquidRocketState,
) -> Result<f64, SystemsError> {
    check_unit_interval("liquid_rocket.throttle_cmd", state.throttle_cmd)?;
    let raw = match state.fault {
        FaultState::Nominal | FaultState::Leak { .. } | FaultState::SensorBias { .. } => {
            if state.throttle_cmd <= 0.0 {
                0.0
            } else {
                state.throttle_cmd.max(spec.min_throttle)
            }
        }
        FaultState::StuckOff | FaultState::Jammed => 0.0,
        FaultState::StuckOn { fraction } => fraction.min(2.0),
        FaultState::ShortCircuit => 1.0,
        FaultState::Degraded { efficiency } => state.throttle_cmd * efficiency,
    };
    Ok(raw)
}

fn split_propellant_demand(spec: &LiquidRocketSpec, total_kg: f64) -> (f64, f64) {
    if total_kg <= 0.0 {
        return (0.0, 0.0);
    }
    match (spec.fuel_tank, spec.oxidizer_tank) {
        (Some(_), Some(_)) => {
            let fuel = total_kg / (1.0 + spec.mixture_ratio_o_f);
            (fuel, total_kg - fuel)
        }
        (Some(_), None) => (total_kg, 0.0),
        (None, Some(_)) => (0.0, total_kg),
        (None, None) => (0.0, 0.0),
    }
}

fn advance_throttle_lag(current: f64, target: f64, tau_up: f64, tau_down: f64, dt_s: f64) -> f64 {
    if dt_s <= 0.0 {
        return current;
    }
    let tau = if target >= current { tau_up } else { tau_down };
    if tau <= 1.0e-6 {
        target
    } else {
        let alpha = 1.0 - libm::exp(-dt_s / tau);
        current + (target - current) * alpha
    }
}

fn gimbaled_thrust_direction(axis: DVec3, cmd_rad: [f64; 2], max_rad: f64) -> DVec3 {
    let base = axis.normalize_or(DVec3::X);
    if max_rad <= 0.0 {
        return base;
    }
    let [pitch_raw, yaw_raw] = cmd_rad;
    let pitch = pitch_raw.clamp(-max_rad, max_rad);
    let yaw = yaw_raw.clamp(-max_rad, max_rad);
    let ref_up = if base.z.abs() < 0.9 {
        DVec3::Z
    } else {
        DVec3::Y
    };
    let u = base.cross(ref_up).normalize();
    let v = base.cross(u);
    let rot = DQuat::from_axis_angle(u, pitch) * DQuat::from_axis_angle(v, yaw);
    (rot * base).normalize()
}
