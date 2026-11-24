use glam::DVec3;
use mbd_core::{
    AmbientGas, ComponentId, ElectricalLoad, FaultState, GasSpecies, PowerRailState, ThermalLoad,
    WrenchAtSite,
};

use crate::error::{
    SystemsError, check_non_negative, check_positive, check_unit_interval, check_vec_finite,
    normalize_axis,
};
use crate::fluid::{FluidNetwork, TankId};

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;

/// Electric ion / Hall-effect thruster driven by electrical bus power and stored propellant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElectricThrusterSpec {
    /// Host component carrying thruster.
    pub site: ComponentId,
    /// Thruster exit center in `site`'s local axes, m.
    pub mount_offset_local_m: DVec3,
    /// Unit thrust vector acting on vehicle in `site`'s local axes.
    pub thrust_axis_local: DVec3,
    /// Source tank (`FluidSpecies::Xenon`, `Krypton`, etc.).
    pub propellant_tank: TankId,
    /// Nominal effective exhaust velocity specific impulse, s.
    pub isp_s: f64,
    /// Total electrical-to-jet efficiency $\eta \in (0, 1]$.
    pub total_efficiency: f64,
    /// Maximum rated input electrical power, W.
    pub max_power_w: f64,
    /// Shedding priority on electrical bus.
    pub priority: u8,
}

impl ElectricThrusterSpec {
    /// Validates electric thruster configuration.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_vec_finite("electric.mount_offset_local_m", self.mount_offset_local_m)?;
        normalize_axis("electric.thrust_axis_local", self.thrust_axis_local)?;
        check_positive("electric.isp_s", self.isp_s)?;
        if !self.total_efficiency.is_finite()
            || self.total_efficiency <= 0.0
            || self.total_efficiency > 1.0
        {
            return Err(SystemsError::InvalidParameter {
                field: "electric.total_efficiency",
                value: self.total_efficiency,
            });
        }
        check_positive("electric.max_power_w", self.max_power_w)?;
        Ok(())
    }
}

/// Runtime state of an [`ElectricThrusterSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElectricThrusterState {
    /// Normalized commanded power throttle `[0.0, 1.0]`.
    pub power_throttle_cmd: f64,
    /// Optional fault state.
    pub fault: FaultState,
}

impl Default for ElectricThrusterState {
    fn default() -> Self {
        Self {
            power_throttle_cmd: 0.0,
            fault: FaultState::Nominal,
        }
    }
}

/// Inputs to step an electric thruster.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElectricThrusterStepInput {
    /// Step size, s.
    pub dt_s: f64,
    /// Bus power rail supplying `site`.
    pub power_rail: PowerRailState,
    /// Maximum electrical power allocated by bus scheduler, W.
    pub allocated_power_w: f64,
}

/// Output emitted by [`step_electric_thruster`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElectricThrusterStepOutput {
    /// Net thrust wrench in `site` axes.
    pub wrench_local: WrenchAtSite,
    /// Electrical demand submitted to bus.
    pub electrical_load: ElectricalLoad,
    /// Rejected thermal power `(1 - eta) * P_in` heating thruster power processor/body.
    pub thermal_load: ThermalLoad,
    /// Propellant mass consumed, kg.
    pub propellant_consumed_kg: f64,
    /// Realized thrust magnitude, N.
    pub thrust_n: f64,
}

/// Steps a power-limited electric ion or Hall thruster over `dt_s`.
///
/// Drops cleanly to zero during orbital eclipse or brownout when `power_rail.enabled == false`
/// or `allocated_power_w <= 0.0`, while conserving tank propellant mass.
pub fn step_electric_thruster(
    spec: &ElectricThrusterSpec,
    state: &ElectricThrusterState,
    fluids: &mut FluidNetwork,
    input: ElectricThrusterStepInput,
) -> Result<ElectricThrusterStepOutput, SystemsError> {
    spec.validate()?;
    state.fault.validate()?;
    check_non_negative("electric.dt_s", input.dt_s)?;
    check_non_negative("electric.allocated_power_w", input.allocated_power_w)?;
    check_unit_interval("electric.power_throttle_cmd", state.power_throttle_cmd)?;

    let fault_scale = match state.fault {
        FaultState::Nominal | FaultState::Leak { .. } | FaultState::SensorBias { .. } => {
            state.power_throttle_cmd
        }
        FaultState::StuckOff | FaultState::Jammed => 0.0,
        FaultState::StuckOn { fraction } => fraction.clamp(0.0, 1.0),
        FaultState::ShortCircuit => 1.0,
        FaultState::Degraded { efficiency } => state.power_throttle_cmd * efficiency,
    };

    let p_avail = if input.power_rail.enabled {
        (spec.max_power_w * fault_scale).min(input.allocated_power_w)
    } else {
        0.0
    };

    let v_e = spec.isp_s * STANDARD_GRAVITY_M_S2;
    let thrust_req = if p_avail > 0.0 {
        (2.0 * spec.total_efficiency * p_avail) / v_e
    } else {
        0.0
    };
    let mdot_req = thrust_req / v_e;
    let dm_req = mdot_req * input.dt_s;

    let dm_used = if input.dt_s > 0.0 && dm_req > 0.0 {
        fluids.withdraw_propellant(spec.propellant_tank, dm_req)?
    } else {
        0.0
    };
    let feed_ratio = if dm_req > 0.0 { dm_used / dm_req } else { 0.0 };
    let realized_power = p_avail * feed_ratio;
    let thrust_n = thrust_req * feed_ratio;
    let dir = spec.thrust_axis_local.normalize_or(DVec3::X);

    Ok(ElectricThrusterStepOutput {
        wrench_local: WrenchAtSite {
            site: spec.site,
            point_local_m: spec.mount_offset_local_m,
            force_local_n: dir * thrust_n,
            torque_local_nm: DVec3::ZERO,
        },
        electrical_load: ElectricalLoad {
            component: spec.site,
            power_w: realized_power,
            priority: spec.priority,
        },
        thermal_load: ThermalLoad {
            component: spec.site,
            heat_rate_w: realized_power * (1.0 - spec.total_efficiency),
        },
        propellant_consumed_kg: dm_used,
        thrust_n,
    })
}

/// Atmosphere-Breathing Electric Propulsion (ABEP) intake + thruster model for VLEO.
///
/// Explicitly resolves multi-species ambient densities (`AtomicO`, `N2`, and remainder)
/// from [`AmbientGas`] to compute intake ram mass capture, intake momentum capture drag,
/// optional buffer tank storage/drawdown, and net thruster force.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AbepThrusterSpec {
    /// Host component carrying intake and thruster.
    pub site: ComponentId,
    /// Thruster force application point in `site` local axes, m.
    pub mount_offset_local_m: DVec3,
    /// Unit thrust direction acting on vehicle in `site` local axes.
    pub thrust_axis_local: DVec3,
    /// Effective intake capture cross-sectional area perpendicular to ram flow, m².
    pub intake_area_m2: f64,
    /// Capture efficiency applied to incident atomic oxygen (`O`).
    pub atomic_o_collection_efficiency: f64,
    /// Capture efficiency applied to incident molecular nitrogen (`N2`).
    pub n2_collection_efficiency: f64,
    /// Effective exhaust specific impulse on collected thermospheric plasma, s.
    pub isp_s: f64,
    /// Power-processing + thruster efficiency $\eta \in (0, 1]$.
    pub total_efficiency: f64,
    /// Maximum electrical power input, W.
    pub max_power_w: f64,
    /// Optional intermediate buffer tank (`FluidSpecies::AirCollected`) smoothing orbital variation.
    pub buffer_tank: Option<TankId>,
    /// Electrical bus shedding priority.
    pub priority: u8,
}

impl AbepThrusterSpec {
    /// Validates ABEP intake and thruster parameters.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_vec_finite("abep.mount_offset_local_m", self.mount_offset_local_m)?;
        normalize_axis("abep.thrust_axis_local", self.thrust_axis_local)?;
        check_positive("abep.intake_area_m2", self.intake_area_m2)?;
        check_unit_interval(
            "abep.atomic_o_collection_efficiency",
            self.atomic_o_collection_efficiency,
        )?;
        check_unit_interval(
            "abep.n2_collection_efficiency",
            self.n2_collection_efficiency,
        )?;
        check_positive("abep.isp_s", self.isp_s)?;
        if !self.total_efficiency.is_finite()
            || self.total_efficiency <= 0.0
            || self.total_efficiency > 1.0
        {
            return Err(SystemsError::InvalidParameter {
                field: "abep.total_efficiency",
                value: self.total_efficiency,
            });
        }
        check_positive("abep.max_power_w", self.max_power_w)?;
        Ok(())
    }
}

/// Runtime state for [`AbepThrusterSpec`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AbepThrusterState {
    /// Normalized electrical power throttle `[0.0, 1.0]`.
    pub power_throttle_cmd: f64,
    /// Active fault state.
    pub fault: FaultState,
}

/// Inputs to step an ABEP intake/thruster over `dt_s`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AbepStepInput {
    /// Time step, s.
    pub dt_s: f64,
    /// Undisturbed thermospheric ambient gas containing partial `AtomicO` and `N2` densities.
    pub ambient: AmbientGas,
    /// True airspeed / orbital velocity magnitude relative to atmosphere, m/s.
    pub airspeed_m_s: f64,
    /// Cosine of intake alignment with incoming relative velocity (`<= 0` yields zero intake flux).
    pub intake_ram_alignment: f64,
    /// Electrical bus power rail supplying `site`.
    pub power_rail: PowerRailState,
    /// Allocated electrical power budget, W.
    pub allocated_power_w: f64,
}

/// Mass balance, ram drag, and net wrench returned by [`step_abep_thruster`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AbepStepOutput {
    /// Combined thruster force plus intake momentum capture drag wrench in `site` axes.
    pub net_wrench_local: WrenchAtSite,
    /// Propellant mass captured by intake during step, kg.
    pub captured_mass_kg: f64,
    /// Propellant mass exhausted by thruster during step, kg.
    pub exhausted_mass_kg: f64,
    /// Gross thruster force along `thrust_axis_local`, N.
    pub gross_thrust_n: f64,
    /// Ram intake momentum capture drag opposing thrust direction, N.
    pub intake_capture_drag_n: f64,
    /// Electrical power demanded by ABEP unit, W.
    pub electrical_load: ElectricalLoad,
}

/// Evaluates ABEP atomic-oxygen & $\text{N}_2$ intake collection, optional buffer tank storage,
/// power-limited jet acceleration, and ram momentum drag over `dt_s`.
pub fn step_abep_thruster(
    spec: &AbepThrusterSpec,
    state: &AbepThrusterState,
    fluids: &mut FluidNetwork,
    input: AbepStepInput,
) -> Result<AbepStepOutput, SystemsError> {
    spec.validate()?;
    state.fault.validate()?;
    input.ambient.validate()?;
    check_non_negative("abep.dt_s", input.dt_s)?;
    check_non_negative("abep.airspeed_m_s", input.airspeed_m_s)?;
    check_non_negative("abep.allocated_power_w", input.allocated_power_w)?;
    check_unit_interval("abep.power_throttle_cmd", state.power_throttle_cmd)?;

    let (mdot_captured, intake_capture_drag_n) = abep_capture_and_drag(spec, &input);
    let captured_mass_kg = mdot_captured * input.dt_s;
    let p_avail = abep_available_power(spec, state, &input);

    let v_e = spec.isp_s * STANDARD_GRAVITY_M_S2;
    let mdot_jet_power_limit = if p_avail > 0.0 {
        (2.0 * spec.total_efficiency * p_avail) / (v_e * v_e)
    } else {
        0.0
    };
    let dm_desired = mdot_jet_power_limit * input.dt_s;

    let mut exhausted_mass_kg = dm_desired.min(captured_mass_kg);
    let excess = (captured_mass_kg - exhausted_mass_kg).max(0.0);
    let deficit = (dm_desired - exhausted_mass_kg).max(0.0);

    if let Some(buf_id) = spec.buffer_tank {
        if excess > 0.0 {
            let _stored = fluids.deposit_fluid(buf_id, excess)?;
        } else if deficit > 0.0 {
            exhausted_mass_kg += fluids.withdraw_propellant(buf_id, deficit)?;
        }
    }

    let mdot_used = if input.dt_s > 0.0 {
        exhausted_mass_kg / input.dt_s
    } else {
        0.0
    };
    let gross_thrust_n = mdot_used * v_e;
    let realized_power = if dm_desired > 0.0 {
        p_avail * (exhausted_mass_kg / dm_desired).min(1.0)
    } else {
        0.0
    };

    let dir = spec.thrust_axis_local.normalize_or(DVec3::X);
    Ok(AbepStepOutput {
        net_wrench_local: WrenchAtSite {
            site: spec.site,
            point_local_m: spec.mount_offset_local_m,
            force_local_n: dir * (gross_thrust_n - intake_capture_drag_n),
            torque_local_nm: DVec3::ZERO,
        },
        captured_mass_kg,
        exhausted_mass_kg,
        gross_thrust_n,
        intake_capture_drag_n,
        electrical_load: ElectricalLoad {
            component: spec.site,
            power_w: realized_power,
            priority: spec.priority,
        },
    })
}

fn abep_capture_and_drag(spec: &AbepThrusterSpec, input: &AbepStepInput) -> (f64, f64) {
    let cos_ram = input.intake_ram_alignment.clamp(0.0, 1.0);
    let rho_ao = input.ambient.mass_density_of(GasSpecies::AtomicO);
    let rho_n2 = input.ambient.mass_density_of(GasSpecies::N2);
    let rho_other = (input.ambient.density_kg_m3 - rho_ao - rho_n2).max(0.0);
    let projected_flux_area = spec.intake_area_m2 * cos_ram;
    let mdot_ao =
        rho_ao * input.airspeed_m_s * projected_flux_area * spec.atomic_o_collection_efficiency;
    let mdot_n2 = (rho_n2 + rho_other)
        * input.airspeed_m_s
        * projected_flux_area
        * spec.n2_collection_efficiency;
    let intake_capture_drag_n =
        input.ambient.density_kg_m3 * input.airspeed_m_s * input.airspeed_m_s * projected_flux_area;
    (mdot_ao + mdot_n2, intake_capture_drag_n)
}

fn abep_available_power(
    spec: &AbepThrusterSpec,
    state: &AbepThrusterState,
    input: &AbepStepInput,
) -> f64 {
    if !input.power_rail.enabled {
        return 0.0;
    }
    let cmd_scale = match state.fault {
        FaultState::StuckOff | FaultState::Jammed => 0.0,
        FaultState::Degraded { efficiency } => state.power_throttle_cmd * efficiency,
        FaultState::StuckOn { fraction } => fraction.clamp(0.0, 1.0),
        FaultState::ShortCircuit => 1.0,
        _ => state.power_throttle_cmd,
    };
    (spec.max_power_w * cmd_scale).min(input.allocated_power_w)
}
