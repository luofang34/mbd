use alloc::vec::Vec;

use glam::DVec3;
use mbd_core::{
    ComponentId, ComponentTree, ElectricalLoad, FaultState, PowerRailState, ThermalLoad,
};

use super::sources::{
    BatterySpec, BatteryState, PowerGeneratorSpec, PowerGeneratorState, SolarArraySpec,
    SolarArrayState, evaluate_generator, evaluate_solar_array,
};
use crate::error::{SystemsError, check_non_negative};

const SECONDS_PER_HOUR: f64 = 3600.0;

/// Environment and mechanical-shaft inputs to an [`ElectricalBus::step`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElectricalEnvInput {
    /// Time step duration, s.
    pub dt_s: f64,
    /// Unit vector toward the Sun in each component's local frame (caller-resolved or uniform root dir).
    pub default_sun_dir_local: DVec3,
    /// Local solar irradiance magnitude, W/m² (`1361.0` at 1 AU, `~590.0` at Mars).
    pub solar_irradiance_w_m2: f64,
    /// False during planetary/lunar eclipse shadow.
    pub sunlit: bool,
    /// Mechanical shaft power available to `ShaftAlternator` units, W.
    pub default_shaft_power_w: f64,
}

/// Realized power-rail telemetry for a single host component after shedding and faults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ComponentRailOutput {
    /// Component owning this bus distribution node.
    pub component: ComponentId,
    /// Resulting bus rail state (`voltage_v`, `ripple_v_rms`, `enabled`).
    pub rail: PowerRailState,
}

/// Aggregate electrical & thermal outputs produced by [`ElectricalBus::step`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ElectricalStepOutput {
    /// Total electrical generation from attached arrays and generators, W.
    pub total_generated_w: f64,
    /// Total load power actually served, W.
    pub total_served_w: f64,
    /// Total requested load power shed due to power shortfall, W.
    pub total_shed_w: f64,
    /// Net power charging (`> 0`) or discharging (`< 0`) across batteries, W.
    pub net_battery_power_w: f64,
    /// Per-component realized bus rail states.
    pub component_rails: Vec<ComponentRailOutput>,
    /// Thermal waste heat rejected into component nodes (RTGs, fuel cells, $I^2 R$ battery heat).
    pub thermal_loads: Vec<ThermalLoad>,
    /// Indices into the submitted `loads` slice that could not be fully served (`0.0..=served`).
    pub granted_powers_w: Vec<f64>,
}

/// Prioritized spacecraft/aircraft electrical power distribution bus.
#[derive(Debug, Clone, Default)]
pub struct ElectricalBus {
    /// Nominal system DC voltage setpoint when powered, V.
    pub(super) nominal_bus_voltage_v: f64,
    pub(super) solar_arrays: Vec<(SolarArraySpec, SolarArrayState)>,
    pub(super) batteries: Vec<(BatterySpec, BatteryState)>,
    pub(super) generators: Vec<(PowerGeneratorSpec, PowerGeneratorState)>,
    pub(super) component_faults: Vec<(ComponentId, FaultState)>,
}

impl ElectricalBus {
    /// Creates an electrical bus operating at `nominal_bus_voltage_v` (e.g. `28.0` or `120.0`).
    pub fn new(nominal_bus_voltage_v: f64) -> Result<Self, SystemsError> {
        crate::error::check_positive("bus.nominal_bus_voltage_v", nominal_bus_voltage_v)?;
        Ok(Self {
            nominal_bus_voltage_v,
            solar_arrays: Vec::new(),
            batteries: Vec::new(),
            generators: Vec::new(),
            component_faults: Vec::new(),
        })
    }

    /// Registers a solar array wing/panel.
    pub fn add_solar_array(
        &mut self,
        spec: SolarArraySpec,
        state: SolarArrayState,
    ) -> Result<(), SystemsError> {
        spec.validate()?;
        state.fault.validate()?;
        self.solar_arrays.push((spec, state));
        Ok(())
    }

    /// Registers a battery pack.
    pub fn add_battery(
        &mut self,
        spec: BatterySpec,
        state: BatteryState,
    ) -> Result<(), SystemsError> {
        spec.validate()?;
        state.fault.validate()?;
        check_non_negative("battery.energy_wh", state.energy_wh)?;
        self.batteries.push((spec, state));
        Ok(())
    }

    /// Registers an RTG, fuel cell, or engine shaft alternator.
    pub fn add_generator(
        &mut self,
        spec: PowerGeneratorSpec,
        state: PowerGeneratorState,
    ) -> Result<(), SystemsError> {
        spec.validate()?;
        state.fault.validate()?;
        self.generators.push((spec, state));
        Ok(())
    }

    /// Injects or clears an electrical distribution fault on `component` (e.g. breaker trip / short).
    pub fn set_component_fault(
        &mut self,
        component: ComponentId,
        fault: FaultState,
    ) -> Result<(), SystemsError> {
        fault.validate()?;
        if let Some((_, existing)) = self
            .component_faults
            .iter_mut()
            .find(|(id, _)| *id == component)
        {
            *existing = fault;
        } else {
            self.component_faults.push((component, fault));
        }
        Ok(())
    }

    /// Returns nominal DC bus voltage setpoint, V.
    pub const fn nominal_bus_voltage_v(&self) -> f64 {
        self.nominal_bus_voltage_v
    }

    /// Returns live battery states (for telemetry / SOC inspection).
    pub fn batteries(&self) -> &[(BatterySpec, BatteryState)] {
        &self.batteries
    }

    /// Returns registered solar array specifications and states.
    pub fn solar_arrays(&self) -> &[(SolarArraySpec, SolarArrayState)] {
        &self.solar_arrays
    }

    /// Returns registered power generator specifications and states.
    pub fn generators(&self) -> &[(PowerGeneratorSpec, PowerGeneratorState)] {
        &self.generators
    }

    /// Returns active component electrical bus faults.
    pub fn component_faults(&self) -> &[(ComponentId, FaultState)] {
        &self.component_faults
    }

    /// Steps generation, prioritized load shedding, and battery charge/discharge over `dt_s`.
    pub fn step(
        &mut self,
        tree: &ComponentTree,
        loads: &[ElectricalLoad],
        env: &ElectricalEnvInput,
        mut fluids: Option<&mut crate::fluid::FluidNetwork>,
        sun_dir_for_component: impl Fn(ComponentId) -> DVec3,
    ) -> Result<ElectricalStepOutput, SystemsError> {
        check_non_negative("electrical.dt_s", env.dt_s)?;
        check_non_negative(
            "electrical.solar_irradiance_w_m2",
            env.solar_irradiance_w_m2,
        )?;

        let mut output = ElectricalStepOutput {
            granted_powers_w: alloc::vec![0.0; loads.len()],
            ..ElectricalStepOutput::default()
        };

        for (spec, state) in &self.solar_arrays {
            let attached = tree.is_attached(spec.component)?;
            let sun_dir = sun_dir_for_component(spec.component);
            output.total_generated_w += evaluate_solar_array(
                spec,
                state,
                sun_dir,
                env.solar_irradiance_w_m2,
                env.sunlit,
                attached,
            )?;
        }

        for (spec, state) in &mut self.generators {
            let attached = tree.is_attached(spec.component)?;
            let (p_elec, q_rej) = evaluate_generator(
                spec,
                state,
                env.dt_s,
                env.default_shaft_power_w,
                attached,
                fluids.as_deref_mut(),
            )?;
            output.total_generated_w += p_elec;
            if q_rej > 0.0 {
                output.thermal_loads.push(ThermalLoad {
                    component: spec.component,
                    heat_rate_w: q_rej,
                });
            }
        }

        let max_batt_discharge = self.max_available_battery_discharge_w(tree, env.dt_s)?;
        let mut remaining_supply_w = output.total_generated_w + max_batt_discharge;

        let mut order: Vec<usize> = (0..loads.len()).collect();
        order.sort_by_key(|&idx| loads.get(idx).map_or(u8::MAX, |l| l.priority));

        for idx in order {
            let Some(load) = loads.get(idx) else {
                continue;
            };
            let attached = tree.is_attached(load.component)?;
            let comp_ok = attached && !self.is_component_shorted_or_open(load.component);
            if !comp_ok || load.power_w <= 0.0 {
                continue;
            }
            let granted = load.power_w.min(remaining_supply_w);
            remaining_supply_w -= granted;
            output.total_served_w += granted;
            output.total_shed_w += load.power_w - granted;
            if let Some(slot) = output.granted_powers_w.get_mut(idx) {
                *slot = granted;
            }
        }

        self.settle_batteries(tree, env.dt_s, &mut output)?;
        self.build_rail_states(tree, loads, &mut output)?;
        Ok(output)
    }

    fn is_component_shorted_or_open(&self, component: ComponentId) -> bool {
        self.component_faults.iter().any(|(id, fault)| {
            *id == component
                && matches!(
                    fault,
                    FaultState::StuckOff | FaultState::ShortCircuit | FaultState::Jammed
                )
        })
    }

    fn max_available_battery_discharge_w(
        &self,
        tree: &ComponentTree,
        dt_s: f64,
    ) -> Result<f64, SystemsError> {
        let mut sum = 0.0;
        for (spec, state) in &self.batteries {
            if !tree.is_attached(spec.component)?
                || matches!(
                    state.fault,
                    FaultState::StuckOff | FaultState::ShortCircuit | FaultState::Jammed
                )
            {
                continue;
            }
            let min_e = spec.capacity_wh * spec.min_soc;
            let avail_wh = (state.energy_wh - min_e).max(0.0);
            let max_energy_w = if dt_s > 0.0 {
                (avail_wh * SECONDS_PER_HOUR / dt_s) * spec.discharge_efficiency
            } else {
                if avail_wh > 0.0 {
                    spec.max_discharge_w
                } else {
                    0.0
                }
            };
            sum += spec.max_discharge_w.min(max_energy_w);
        }
        Ok(sum)
    }

    fn settle_batteries(
        &mut self,
        tree: &ComponentTree,
        dt_s: f64,
        output: &mut ElectricalStepOutput,
    ) -> Result<(), SystemsError> {
        let balance_w = output.total_generated_w - output.total_served_w;
        output.net_battery_power_w = balance_w;
        if dt_s <= 0.0 || balance_w.abs() <= 1.0e-9 {
            return Ok(());
        }
        if balance_w < 0.0 {
            let mut need_w = -balance_w;
            for (spec, state) in &mut self.batteries {
                if need_w <= 0.0 || !tree.is_attached(spec.component)? {
                    continue;
                }
                if matches!(
                    state.fault,
                    FaultState::StuckOff | FaultState::ShortCircuit | FaultState::Jammed
                ) {
                    continue;
                }
                let min_e = spec.capacity_wh * spec.min_soc;
                let avail_wh = (state.energy_wh - min_e).max(0.0);
                let cap_w = (avail_wh * SECONDS_PER_HOUR / dt_s) * spec.discharge_efficiency;
                let draw_w = need_w.min(spec.max_discharge_w).min(cap_w);
                if draw_w > 0.0 {
                    let stored_drop_wh =
                        (draw_w / spec.discharge_efficiency) * (dt_s / SECONDS_PER_HOUR);
                    state.energy_wh = (state.energy_wh - stored_drop_wh).max(min_e);
                    need_w -= draw_w;
                    let i_rms = draw_w / spec.nominal_voltage_v;
                    output.thermal_loads.push(ThermalLoad {
                        component: spec.component,
                        heat_rate_w: i_rms * i_rms * spec.internal_resistance_ohm,
                    });
                }
            }
        } else {
            let mut surplus_w = balance_w;
            for (spec, state) in &mut self.batteries {
                if surplus_w <= 0.0
                    || spec.max_charge_w <= 0.0
                    || !tree.is_attached(spec.component)?
                {
                    continue;
                }
                let headroom_wh = (spec.capacity_wh - state.energy_wh).max(0.0);
                let accept_w = (headroom_wh * SECONDS_PER_HOUR / dt_s) / spec.charge_efficiency;
                let chg_w = surplus_w.min(spec.max_charge_w).min(accept_w);
                if chg_w > 0.0 {
                    state.energy_wh += chg_w * spec.charge_efficiency * (dt_s / SECONDS_PER_HOUR);
                    surplus_w -= chg_w;
                }
            }
        }
        Ok(())
    }

    fn build_rail_states(
        &self,
        tree: &ComponentTree,
        loads: &[ElectricalLoad],
        output: &mut ElectricalStepOutput,
    ) -> Result<(), SystemsError> {
        let mut seen: Vec<ComponentId> = Vec::new();
        for &(id, _) in &self.component_faults {
            if !seen.contains(&id) {
                seen.push(id);
            }
        }
        for (spec, _) in &self.batteries {
            if !seen.contains(&spec.component) {
                seen.push(spec.component);
            }
        }
        for load in loads {
            if !seen.contains(&load.component) {
                seen.push(load.component);
            }
        }

        let total_capacity_alive = output.total_generated_w > 0.0
            || self.max_available_battery_discharge_w(tree, 1.0)? > 0.0;
        for id in seen {
            let attached = tree.is_attached(id)?;
            let enabled =
                attached && total_capacity_alive && !self.is_component_shorted_or_open(id);
            let rail = if enabled {
                PowerRailState::nominal(self.nominal_bus_voltage_v)
            } else {
                PowerRailState {
                    voltage_v: 0.0,
                    ripple_v_rms: 0.0,
                    enabled: false,
                }
            };
            output.component_rails.push(ComponentRailOutput {
                component: id,
                rail,
            });
        }
        Ok(())
    }
}
