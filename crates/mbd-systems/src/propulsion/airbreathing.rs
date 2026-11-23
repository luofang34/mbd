use glam::DVec3;
use mbd_core::{AmbientGas, ComponentId, FaultState, ThermalLoad, WrenchAtSite};

use crate::error::{
    SystemsError, check_non_negative, check_positive, check_unit_interval, check_vec_finite,
    normalize_axis,
};
use crate::fluid::{FluidNetwork, TankId};

const DRY_AIR_OXIDIZER_MASS_FRACTION: f64 = 0.231_4;
const SEA_LEVEL_DENSITY_KG_M3: f64 = 1.225;
const SEA_LEVEL_TEMP_K: f64 = 288.15;

/// Class of airbreathing propulsion unit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AirbreathingEngineKind {
    /// Subsonic / supersonic dry turbofan or turbojet.
    TurbojetOrTurbofan,
    /// Afterburning supersonic turbofan/turbojet (e.g. Concorde Olympus 593, Tu-160 NK-32).
    AfterburningTurbofan,
    /// Propeller or helicopter turboshaft engine delivering shaft torque + residual jet thrust.
    TurbopropOrTurboshaft,
    /// Aspirated piston engine with critical altitude / supercharger density lapse.
    Piston,
}

/// Static configuration for an airbreathing aircraft or rotorcraft engine.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirbreathingEngineSpec {
    /// Host component receiving thrust wrench and thermal load.
    pub site: ComponentId,
    /// Thrust line attachment point in `site`'s local frame, m.
    pub mount_offset_local_m: DVec3,
    /// Unit forward thrust direction in `site`'s local frame.
    pub thrust_axis_local: DVec3,
    /// Engine architectural type.
    pub kind: AirbreathingEngineKind,
    /// Sea-level static military/dry maximum thrust, N (`0.0` for pure shaft engine).
    pub sl_static_dry_thrust_n: f64,
    /// Thrust-specific fuel consumption in dry military regime, kg/(N·s).
    pub dry_tsfc_kg_n_s: f64,
    /// Afterburner thrust multiplier relative to dry thrust (`1.0` if non-afterburning).
    pub reheat_thrust_multiplier: f64,
    /// Afterburner TSFC multiplier relative to dry TSFC (`1.0` if non-afterburning).
    pub reheat_tsfc_multiplier: f64,
    /// Maximum sea-level shaft power for turboprop/turboshaft/piston engines, W (`0.0` for pure jets).
    pub sl_max_shaft_power_w: f64,
    /// Brake-specific fuel consumption on shaft power, kg/(W·s).
    pub shaft_bsfc_kg_w_s: f64,
    /// Design Mach limit above which supersonic inlet ram recovery or compression saturates.
    pub max_operating_mach: f64,
    /// Density ratio lapse exponent $\sigma^\alpha$ (typical `0.7` high-bypass, `1.0` aspirated piston).
    pub density_lapse_exponent: f64,
    /// Minimum ambient oxidizer mass fraction below which combustion flames out.
    pub min_oxidizer_mass_fraction: f64,
    /// Fuel tank feeding this engine.
    pub fuel_tank: TankId,
    /// First-order throttle response time constant, s.
    pub spool_tau_s: f64,
}

impl AirbreathingEngineSpec {
    /// Validates airbreathing engine configuration parameters.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_vec_finite(
            "airbreathing.mount_offset_local_m",
            self.mount_offset_local_m,
        )?;
        normalize_axis("airbreathing.thrust_axis_local", self.thrust_axis_local)?;
        check_non_negative(
            "airbreathing.sl_static_dry_thrust_n",
            self.sl_static_dry_thrust_n,
        )?;
        check_non_negative("airbreathing.dry_tsfc_kg_n_s", self.dry_tsfc_kg_n_s)?;
        if !self.reheat_thrust_multiplier.is_finite() || self.reheat_thrust_multiplier < 1.0 {
            return Err(SystemsError::InvalidParameter {
                field: "airbreathing.reheat_thrust_multiplier",
                value: self.reheat_thrust_multiplier,
            });
        }
        if !self.reheat_tsfc_multiplier.is_finite() || self.reheat_tsfc_multiplier < 1.0 {
            return Err(SystemsError::InvalidParameter {
                field: "airbreathing.reheat_tsfc_multiplier",
                value: self.reheat_tsfc_multiplier,
            });
        }
        check_non_negative(
            "airbreathing.sl_max_shaft_power_w",
            self.sl_max_shaft_power_w,
        )?;
        check_non_negative("airbreathing.shaft_bsfc_kg_w_s", self.shaft_bsfc_kg_w_s)?;
        check_positive("airbreathing.max_operating_mach", self.max_operating_mach)?;
        check_non_negative(
            "airbreathing.density_lapse_exponent",
            self.density_lapse_exponent,
        )?;
        check_unit_interval(
            "airbreathing.min_oxidizer_mass_fraction",
            self.min_oxidizer_mass_fraction,
        )?;
        check_non_negative("airbreathing.spool_tau_s", self.spool_tau_s)?;
        Ok(())
    }
}

/// Dynamic state of an [`AirbreathingEngineSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirbreathingEngineState {
    /// Commanded dry throttle `[0.0, 1.0]`.
    pub throttle_cmd: f64,
    /// Afterburner/reheat engagement `[0.0, 1.0]` (`0.0` = dry).
    pub afterburner_cmd: f64,
    /// Actual spooled dry core fraction `[0.0, 1.0]`.
    pub spool_fraction: f64,
    /// Whether flameout has occurred (oxidizer deficit, water ingestion, or fuel unporting).
    pub flamed_out: bool,
    /// Injected engine fault.
    pub fault: FaultState,
}

impl Default for AirbreathingEngineState {
    fn default() -> Self {
        Self {
            throttle_cmd: 0.0,
            afterburner_cmd: 0.0,
            spool_fraction: 0.0,
            flamed_out: false,
            fault: FaultState::Nominal,
        }
    }
}

/// Inputs to step an airbreathing engine over `dt_s`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirbreathingStepInput {
    /// Time step duration, s.
    pub dt_s: f64,
    /// Multi-species undisturbed ambient atmosphere (`oxidizer_mass_fraction`, `humidity_mass_fraction`, `sound_speed_m_s`).
    pub ambient: AmbientGas,
    /// True airspeed magnitude, m/s.
    pub true_airspeed_m_s: f64,
    /// Master fuel shutoff / fire handle closed.
    pub fuel_cutoff: bool,
}

/// Outputs emitted by [`step_airbreathing_engine`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirbreathingStepOutput {
    /// Forward jet/core thrust wrench delivered to `spec.site`.
    pub wrench_local: WrenchAtSite,
    /// Available shaft mechanical power delivered to propeller/rotor gearbox, W.
    pub shaft_power_w: f64,
    /// Fuel mass withdrawn from `spec.fuel_tank`, kg.
    pub fuel_consumed_kg: f64,
    /// Realized jet thrust magnitude, N.
    pub thrust_n: f64,
    /// Estimated waste heat into engine bay/nacelle, W.
    pub thermal_load: ThermalLoad,
}

/// Advances an airbreathing jet, afterburning turbojet, turboprop, or piston engine
/// over `dt_s` using [`AmbientGas`] oxidizer and humidity mass fractions and inlet Mach recovery.
pub fn step_airbreathing_engine(
    spec: &AirbreathingEngineSpec,
    state: &mut AirbreathingEngineState,
    fluids: &mut FluidNetwork,
    input: AirbreathingStepInput,
) -> Result<AirbreathingStepOutput, SystemsError> {
    spec.validate()?;
    state.fault.validate()?;
    input.ambient.validate()?;
    check_non_negative("airbreathing.dt_s", input.dt_s)?;
    check_non_negative("airbreathing.true_airspeed_m_s", input.true_airspeed_m_s)?;
    check_unit_interval("airbreathing.throttle_cmd", state.throttle_cmd)?;
    check_unit_interval("airbreathing.afterburner_cmd", state.afterburner_cmd)?;

    let ox_frac = input.ambient.oxidizer_mass_fraction();
    let humidity = input.ambient.humidity_mass_fraction();
    let comb_ok = !input.fuel_cutoff
        && !input.ambient.is_vacuum()
        && ox_frac >= spec.min_oxidizer_mass_fraction
        && humidity < 0.25;

    let cmd = faulted_airbreathing_cmd(state, comb_ok);
    advance_airbreathing_spool(spec, state, input.dt_s, cmd);

    let mach = if input.ambient.sound_speed_m_s > 1.0e-6 {
        input.true_airspeed_m_s / input.ambient.sound_speed_m_s
    } else {
        0.0
    };
    let env_factor = compute_airbreathing_env_factor(spec, &input, mach, ox_frac, humidity);

    let reheat = if matches!(spec.kind, AirbreathingEngineKind::AfterburningTurbofan) && comb_ok {
        state.afterburner_cmd * state.spool_fraction
    } else {
        0.0
    };

    let dry_thrust = spec.sl_static_dry_thrust_n * state.spool_fraction * env_factor;
    let total_thrust_req = dry_thrust * (1.0 + reheat * (spec.reheat_thrust_multiplier - 1.0));
    let shaft_power_req = spec.sl_max_shaft_power_w * state.spool_fraction * env_factor;

    let eff_tsfc = spec.dry_tsfc_kg_n_s * (1.0 + reheat * (spec.reheat_tsfc_multiplier - 1.0));
    let mdot_fuel = total_thrust_req * eff_tsfc + shaft_power_req * spec.shaft_bsfc_kg_w_s;
    let dm_req = mdot_fuel * input.dt_s;

    let fuel_consumed_kg = if input.dt_s > 0.0 && dm_req > 0.0 {
        fluids.withdraw_propellant(spec.fuel_tank, dm_req)?
    } else {
        0.0
    };
    let feed_ratio = if dm_req > 0.0 {
        (fuel_consumed_kg / dm_req).clamp(0.0, 1.0)
    } else if comb_ok {
        1.0
    } else {
        0.0
    };

    state.flamed_out = !comb_ok || (dm_req > 0.0 && feed_ratio < 0.999);
    let thrust_n = total_thrust_req * feed_ratio;
    let shaft_power_w = shaft_power_req * feed_ratio;
    let dir = spec.thrust_axis_local.normalize_or(DVec3::X);

    Ok(AirbreathingStepOutput {
        wrench_local: WrenchAtSite {
            site: spec.site,
            point_local_m: spec.mount_offset_local_m,
            force_local_n: dir * thrust_n,
            torque_local_nm: DVec3::ZERO,
        },
        shaft_power_w,
        fuel_consumed_kg,
        thrust_n,
        thermal_load: ThermalLoad {
            component: spec.site,
            heat_rate_w: mdot_fuel * feed_ratio * 1.2e7 * 0.08,
        },
    })
}

fn compute_airbreathing_env_factor(
    spec: &AirbreathingEngineSpec,
    input: &AirbreathingStepInput,
    mach: f64,
    ox_frac: f64,
    humidity: f64,
) -> f64 {
    let inlet_recovery = if mach > 1.0 {
        (1.0 - 0.075 * libm::pow(mach - 1.0, 1.35)).clamp(0.1, 1.0)
    } else {
        1.0
    };
    let mach_envelope = if mach > spec.max_operating_mach {
        (1.0 - (mach - spec.max_operating_mach) * 0.5).clamp(0.0, 1.0)
    } else {
        1.0 + 0.18 * mach * mach.min(2.2) * inlet_recovery
    };
    let sigma = (input.ambient.density_kg_m3 / SEA_LEVEL_DENSITY_KG_M3).max(0.0);
    let temp_ratio = if input.ambient.temperature_k > 0.0 {
        libm::sqrt(SEA_LEVEL_TEMP_K / input.ambient.temperature_k).clamp(0.7, 1.25)
    } else {
        0.0
    };
    let ox_ratio = (ox_frac / DRY_AIR_OXIDIZER_MASS_FRACTION).clamp(0.0, 1.5);
    let humidity_lapse = (1.0 - 2.5 * humidity).clamp(0.0, 1.0);
    libm::pow(sigma, spec.density_lapse_exponent)
        * temp_ratio
        * mach_envelope
        * ox_ratio
        * humidity_lapse
}

fn faulted_airbreathing_cmd(state: &AirbreathingEngineState, comb_ok: bool) -> f64 {
    if !comb_ok {
        return 0.0;
    }
    match state.fault {
        FaultState::StuckOff | FaultState::Jammed => 0.0,
        FaultState::StuckOn { fraction } => fraction.clamp(0.0, 1.0),
        FaultState::ShortCircuit => 1.0,
        FaultState::Degraded { efficiency } => state.throttle_cmd * efficiency,
        _ => state.throttle_cmd,
    }
}

fn advance_airbreathing_spool(
    spec: &AirbreathingEngineSpec,
    state: &mut AirbreathingEngineState,
    dt_s: f64,
    cmd: f64,
) {
    if dt_s <= 0.0 {
        return;
    }
    if spec.spool_tau_s <= 1.0e-6 {
        state.spool_fraction = cmd;
    } else {
        let alpha = 1.0 - libm::exp(-dt_s / spec.spool_tau_s);
        state.spool_fraction += (cmd - state.spool_fraction) * alpha;
    }
}
