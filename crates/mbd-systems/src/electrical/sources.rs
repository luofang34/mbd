use glam::DVec3;
use mbd_core::{ComponentId, FaultState};

use crate::error::{
    SystemsError, check_non_negative, check_positive, check_unit_interval, normalize_axis,
};
use crate::fluid::{FluidNetwork, TankId};

const SECONDS_PER_YEAR: f64 = 31_557_600.0;
const SECONDS_PER_HOUR: f64 = 3600.0;

/// Solar photovoltaic wing / panel specification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolarArraySpec {
    /// Component carrying this array (e.g. rotating solar wing link).
    pub component: ComponentId,
    /// Gross photovoltaic cell area, m².
    pub area_m2: f64,
    /// Cell-normal unit vector in `component` local frame.
    pub normal_local: DVec3,
    /// Conversion efficiency at BOL (`0.0..=1.0`).
    pub efficiency: f64,
    /// Active cell packing factor (`0.0..=1.0`).
    pub packing_factor: f64,
    /// Annual radiation degradation fractional loss (`0.0..=0.5`).
    pub annual_degradation: f64,
    /// Nominal operating bus voltage, V.
    pub nominal_voltage_v: f64,
}

impl SolarArraySpec {
    /// Validates solar array parameters.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_positive("solar.area_m2", self.area_m2)?;
        normalize_axis("solar.normal_local", self.normal_local)?;
        check_unit_interval("solar.efficiency", self.efficiency)?;
        check_unit_interval("solar.packing_factor", self.packing_factor)?;
        if !self.annual_degradation.is_finite() || !(0.0..=0.5).contains(&self.annual_degradation) {
            return Err(SystemsError::InvalidParameter {
                field: "solar.annual_degradation",
                value: self.annual_degradation,
            });
        }
        check_positive("solar.nominal_voltage_v", self.nominal_voltage_v)?;
        Ok(())
    }
}

/// Dynamic state of a [`SolarArraySpec`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SolarArrayState {
    /// Deployed fraction (`0.0` stowed, `1.0` fully deployed).
    pub deployed_fraction: f64,
    /// Operating mission age for radiation degradation, years.
    pub age_years: f64,
    /// Active fault state (e.g. `Degraded`, `StuckOff`, `ShortCircuit`).
    pub fault: FaultState,
}

/// Evaluates electrical power generated (W) by a solar array under given sun direction & irradiance.
pub fn evaluate_solar_array(
    spec: &SolarArraySpec,
    state: &SolarArrayState,
    sun_dir_local: DVec3,
    irradiance_w_m2: f64,
    sunlit: bool,
    attached: bool,
) -> Result<f64, SystemsError> {
    spec.validate()?;
    state.fault.validate()?;
    check_unit_interval("solar.deployed_fraction", state.deployed_fraction)?;
    check_non_negative("solar.age_years", state.age_years)?;
    check_non_negative("solar.irradiance_w_m2", irradiance_w_m2)?;

    if !attached || !sunlit || irradiance_w_m2 <= 0.0 || state.deployed_fraction <= 0.0 {
        return Ok(0.0);
    }
    let fault_factor = match state.fault {
        FaultState::StuckOff | FaultState::ShortCircuit | FaultState::Jammed => 0.0,
        FaultState::Degraded { efficiency } => efficiency,
        _ => 1.0,
    };
    let sun_unit = sun_dir_local.normalize_or_zero();
    let cos_inc = spec.normal_local.normalize().dot(sun_unit).max(0.0);
    let age_factor = libm::pow(1.0 - spec.annual_degradation, state.age_years);
    Ok(spec.area_m2
        * spec.packing_factor
        * spec.efficiency
        * state.deployed_fraction
        * age_factor
        * fault_factor
        * irradiance_w_m2
        * cos_inc)
}

/// Primary (non-rechargeable) or secondary (rechargeable) electrochemical battery specification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatterySpec {
    /// Host component carrying battery pack.
    pub component: ComponentId,
    /// Total nameplate energy capacity, Wh.
    pub capacity_wh: f64,
    /// Open-circuit nominal terminal voltage, V.
    pub nominal_voltage_v: f64,
    /// Maximum charge power, W (`0.0` models primary non-rechargeable batteries).
    pub max_charge_w: f64,
    /// Maximum continuous discharge power, W.
    pub max_discharge_w: f64,
    /// Charge coulombic/energy efficiency `(0, 1]`.
    pub charge_efficiency: f64,
    /// Discharge efficiency `(0, 1]`.
    pub discharge_efficiency: f64,
    /// Minimum protected state-of-charge fraction `[0.0, 1.0)`.
    pub min_soc: f64,
    /// Equivalent series internal resistance causing voltage sag under load, Ohm.
    pub internal_resistance_ohm: f64,
}

impl BatterySpec {
    /// Validates battery parameters.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_positive("battery.capacity_wh", self.capacity_wh)?;
        check_positive("battery.nominal_voltage_v", self.nominal_voltage_v)?;
        check_non_negative("battery.max_charge_w", self.max_charge_w)?;
        check_non_negative("battery.max_discharge_w", self.max_discharge_w)?;
        if !self.charge_efficiency.is_finite()
            || self.charge_efficiency <= 0.0
            || self.charge_efficiency > 1.0
        {
            return Err(SystemsError::InvalidParameter {
                field: "battery.charge_efficiency",
                value: self.charge_efficiency,
            });
        }
        if !self.discharge_efficiency.is_finite()
            || self.discharge_efficiency <= 0.0
            || self.discharge_efficiency > 1.0
        {
            return Err(SystemsError::InvalidParameter {
                field: "battery.discharge_efficiency",
                value: self.discharge_efficiency,
            });
        }
        if !self.min_soc.is_finite() || self.min_soc < 0.0 || self.min_soc >= 1.0 {
            return Err(SystemsError::InvalidParameter {
                field: "battery.min_soc",
                value: self.min_soc,
            });
        }
        check_non_negative(
            "battery.internal_resistance_ohm",
            self.internal_resistance_ohm,
        )?;
        Ok(())
    }
}

/// Runtime state of a [`BatterySpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryState {
    /// Current stored energy, Wh.
    pub energy_wh: f64,
    /// Active fault state.
    pub fault: FaultState,
}

impl BatteryState {
    /// Initializes battery at full state-of-charge (`capacity_wh`).
    pub const fn full(spec: &BatterySpec) -> Self {
        Self {
            energy_wh: spec.capacity_wh,
            fault: FaultState::Nominal,
        }
    }

    /// Fractional state of charge in `[0.0, 1.0]`.
    pub fn soc(&self, spec: &BatterySpec) -> f64 {
        if spec.capacity_wh <= 0.0 {
            0.0
        } else {
            (self.energy_wh / spec.capacity_wh).clamp(0.0, 1.0)
        }
    }
}

/// Non-solar electrical power generation source kind.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PowerGeneratorKind {
    /// Radioisotope Thermoelectric Generator (e.g. Perseverance MMRTG).
    Rtg {
        /// Beginning-of-life electrical output, W.
        bol_electrical_w: f64,
        /// Beginning-of-life total isotope thermal output, W.
        bol_thermal_w: f64,
        /// Isotope half-life, years (Plutonium-238 ≈ 87.7 yr).
        half_life_years: f64,
    },
    /// Hydrogen/oxygen fuel cell (e.g. Gemini / Apollo SM) emitting electric power + waste heat.
    FuelCell {
        /// Rated maximum continuous electrical output, W.
        max_power_w: f64,
        /// Waste heat produced as a fraction of delivered electrical output (`~0.65` for AFC).
        waste_heat_ratio: f64,
        /// Total reactant consumption rate, kg/Wh.
        specific_reactant_kg_wh: f64,
        /// Optional H2 reactant tank id.
        h2_tank: Option<TankId>,
        /// Optional O2 reactant tank id.
        o2_tank: Option<TankId>,
    },
    /// Mechanical shaft-driven alternator or integrated drive generator (aircraft).
    ShaftAlternator {
        /// Maximum continuous electrical output, W.
        max_power_w: f64,
        /// Minimum gearbox shaft power below which generator drops offline, W.
        min_shaft_power_w: f64,
        /// Mechanical-to-electrical conversion efficiency.
        efficiency: f64,
    },
}

/// Generator unit specification mounted on a `ComponentTree` node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowerGeneratorSpec {
    /// Host component carrying the generator.
    pub component: ComponentId,
    /// Physical source model.
    pub kind: PowerGeneratorKind,
    /// Nominal output voltage, V.
    pub nominal_voltage_v: f64,
}

impl PowerGeneratorSpec {
    /// Validates generator configuration.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_positive("generator.nominal_voltage_v", self.nominal_voltage_v)?;
        match self.kind {
            PowerGeneratorKind::Rtg {
                bol_electrical_w,
                bol_thermal_w,
                half_life_years,
            } => {
                check_non_negative("rtg.bol_electrical_w", bol_electrical_w)?;
                check_non_negative("rtg.bol_thermal_w", bol_thermal_w)?;
                check_positive("rtg.half_life_years", half_life_years)?;
            }
            PowerGeneratorKind::FuelCell {
                max_power_w,
                waste_heat_ratio,
                specific_reactant_kg_wh,
                ..
            } => {
                check_non_negative("fuel_cell.max_power_w", max_power_w)?;
                check_non_negative("fuel_cell.waste_heat_ratio", waste_heat_ratio)?;
                check_non_negative("fuel_cell.specific_reactant_kg_wh", specific_reactant_kg_wh)?;
            }
            PowerGeneratorKind::ShaftAlternator {
                max_power_w,
                min_shaft_power_w,
                efficiency,
            } => {
                check_non_negative("alternator.max_power_w", max_power_w)?;
                check_non_negative("alternator.min_shaft_power_w", min_shaft_power_w)?;
                check_unit_interval("alternator.efficiency", efficiency)?;
            }
        }
        Ok(())
    }
}

/// Dynamic runtime state of a [`PowerGeneratorSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowerGeneratorState {
    /// Whether the generator breaker is closed/online.
    pub enabled: bool,
    /// Cumulative mission age for RTG decay, s.
    pub elapsed_s: f64,
    /// Fault injection mode.
    pub fault: FaultState,
}

impl Default for PowerGeneratorState {
    fn default() -> Self {
        Self {
            enabled: true,
            elapsed_s: 0.0,
            fault: FaultState::Nominal,
        }
    }
}

pub(crate) fn evaluate_generator(
    spec: &PowerGeneratorSpec,
    state: &mut PowerGeneratorState,
    dt_s: f64,
    shaft_power_w: f64,
    attached: bool,
    fluids: Option<&mut FluidNetwork>,
) -> Result<(f64, f64), SystemsError> {
    state.elapsed_s += dt_s.max(0.0);
    if !attached
        || !state.enabled
        || matches!(
            state.fault,
            FaultState::StuckOff | FaultState::ShortCircuit | FaultState::Jammed
        )
    {
        return Ok((0.0, 0.0));
    }
    let scale = match state.fault {
        FaultState::Degraded { efficiency } => efficiency,
        _ => 1.0,
    };
    match spec.kind {
        PowerGeneratorKind::Rtg {
            bol_electrical_w,
            bol_thermal_w,
            half_life_years,
        } => {
            let years = state.elapsed_s / SECONDS_PER_YEAR;
            let decay = libm::pow(0.5, years / half_life_years);
            let p_elec = bol_electrical_w * decay * scale;
            let q_rej = (bol_thermal_w * decay - p_elec).max(0.0);
            Ok((p_elec, q_rej))
        }
        PowerGeneratorKind::FuelCell {
            max_power_w,
            waste_heat_ratio,
            specific_reactant_kg_wh,
            h2_tank,
            o2_tank,
        } => {
            let p_nom = max_power_w * scale;
            let feed_ratio = if dt_s > 0.0 && p_nom > 0.0 && specific_reactant_kg_wh > 0.0 {
                if let Some(net) = fluids {
                    let dm_total = p_nom * (dt_s / SECONDS_PER_HOUR) * specific_reactant_kg_wh;
                    consume_fuel_cell_reactants(net, h2_tank, o2_tank, dm_total)?
                } else {
                    1.0
                }
            } else {
                1.0
            };
            let p_elec = p_nom * feed_ratio;
            Ok((p_elec, p_elec * waste_heat_ratio))
        }
        PowerGeneratorKind::ShaftAlternator {
            max_power_w,
            min_shaft_power_w,
            efficiency,
        } => {
            if shaft_power_w < min_shaft_power_w {
                Ok((0.0, 0.0))
            } else {
                let p_elec = (shaft_power_w * efficiency * scale).min(max_power_w);
                let q_rej = p_elec * ((1.0 / efficiency.max(0.05)) - 1.0);
                Ok((p_elec, q_rej))
            }
        }
    }
}

fn consume_fuel_cell_reactants(
    net: &mut FluidNetwork,
    h2_tank: Option<TankId>,
    o2_tank: Option<TankId>,
    dm_total: f64,
) -> Result<f64, SystemsError> {
    let dm_h2 = dm_total / 9.0;
    let dm_o2 = dm_total * (8.0 / 9.0);
    let r_h2 = match h2_tank {
        Some(t) if dm_h2 > 0.0 => (net.usable_propellant_kg(t)? / dm_h2).min(1.0),
        _ => 1.0,
    };
    let r_o2 = match o2_tank {
        Some(t) if dm_o2 > 0.0 => (net.usable_propellant_kg(t)? / dm_o2).min(1.0),
        _ => 1.0,
    };
    let feed_ratio = r_h2.min(r_o2).clamp(0.0, 1.0);
    if let Some(t) = h2_tank {
        let _used = net.withdraw_propellant(t, dm_h2 * feed_ratio)?;
    }
    if let Some(t) = o2_tank {
        let _used = net.withdraw_propellant(t, dm_o2 * feed_ratio)?;
    }
    Ok(feed_ratio)
}
