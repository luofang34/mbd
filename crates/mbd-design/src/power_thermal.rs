use glam::DVec3;
use mbd_core::{ComponentId, ComponentTree, ElectricalLoad, PowerRailState, ThermalLoad};
use mbd_systems::{ElectricalBus, ElectricalEnvInput, ThermalNetwork, ThermalStepEnv};
use serde::{Deserialize, Serialize};

use crate::error::{DesignError, check_non_negative, check_positive};

#[cfg(test)]
mod tests;

const SECONDS_PER_HOUR: f64 = 3600.0;

/// Full sunlit + eclipse orbit cycle parameters for solar/battery energy balance verification.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OrbitalPowerScenario {
    /// Duration of sunlit illumination per orbit/period, s.
    pub sunlit_duration_s: f64,
    /// Duration of eclipse shadow per orbit/period, s.
    pub eclipse_duration_s: f64,
    /// Integration step size, s.
    pub step_dt_s: f64,
    /// Solar irradiance during sunlit arc, W/m² (`1361.0` at 1 AU, `590.0` at Mars).
    pub solar_irradiance_w_m2: f64,
    /// Sun unit vector in vehicle root/component frame.
    pub sun_dir_local: [f64; 3],
    /// Mechanical shaft power available to alternators, W.
    pub shaft_power_w: f64,
}

/// Net battery state-of-charge and load-shedding outcome across an orbital power cycle.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PowerBalanceReport {
    /// Initial energy stored across all attached batteries, Wh.
    pub initial_stored_wh: f64,
    /// Final energy stored after sunlit + eclipse cycle, Wh.
    pub final_stored_wh: f64,
    /// Net change `final - initial` over one orbit, Wh (`>= 0` indicates energy sustainable).
    pub net_energy_delta_wh: f64,
    /// Worst-case minimum total state-of-charge fraction reached during eclipse.
    pub min_soc_fraction: f64,
    /// Total energy shed across non-critical loads due to bus power limits, Wh.
    pub total_shed_wh: f64,
    /// True if net orbit delta is non-negative and zero critical/active shedding occurred.
    pub energy_positive: bool,
}

/// Evaluates sunlit + eclipse electrical energy balance across one full cycle.
pub fn analyze_orbital_power_balance(
    bus: &ElectricalBus,
    tree: &ComponentTree,
    loads: &[ElectricalLoad],
    scenario: &OrbitalPowerScenario,
) -> Result<PowerBalanceReport, DesignError> {
    check_non_negative("power.sunlit_duration_s", scenario.sunlit_duration_s)?;
    check_non_negative("power.eclipse_duration_s", scenario.eclipse_duration_s)?;
    check_positive("power.step_dt_s", scenario.step_dt_s)?;
    check_non_negative(
        "power.solar_irradiance_w_m2",
        scenario.solar_irradiance_w_m2,
    )?;
    check_non_negative("power.shaft_power_w", scenario.shaft_power_w)?;
    let sun_dir = DVec3::from_array(scenario.sun_dir_local);
    if !sun_dir.is_finite() || sun_dir.length_squared() <= 1.0e-12 {
        return Err(DesignError::InvalidConfig {
            detail: "sun_dir_local must be finite and non-zero".into(),
        });
    }

    let mut sim_bus = bus.clone();
    let (initial_stored_wh, initial_soc) = battery_totals(&sim_bus);
    let mut min_soc = initial_soc;
    let mut total_shed_wh = 0.0;

    run_power_phase(
        &mut sim_bus,
        tree,
        loads,
        scenario,
        sun_dir,
        scenario.sunlit_duration_s,
        true,
        &mut min_soc,
        &mut total_shed_wh,
    )?;
    run_power_phase(
        &mut sim_bus,
        tree,
        loads,
        scenario,
        sun_dir,
        scenario.eclipse_duration_s,
        false,
        &mut min_soc,
        &mut total_shed_wh,
    )?;

    let (final_stored_wh, _) = battery_totals(&sim_bus);
    let net_energy_delta_wh = final_stored_wh - initial_stored_wh;
    Ok(PowerBalanceReport {
        initial_stored_wh,
        final_stored_wh,
        net_energy_delta_wh,
        min_soc_fraction: min_soc,
        total_shed_wh,
        energy_positive: net_energy_delta_wh >= -1.0e-6 && total_shed_wh <= 1.0e-6,
    })
}

#[allow(clippy::too_many_arguments)]
fn run_power_phase(
    bus: &mut ElectricalBus,
    tree: &ComponentTree,
    loads: &[ElectricalLoad],
    scenario: &OrbitalPowerScenario,
    sun_dir: DVec3,
    duration_s: f64,
    sunlit: bool,
    min_soc: &mut f64,
    total_shed_wh: &mut f64,
) -> Result<(), DesignError> {
    let mut remaining = duration_s;
    while remaining > 1.0e-9 {
        let dt_s = remaining.min(scenario.step_dt_s);
        let env = ElectricalEnvInput {
            dt_s,
            default_sun_dir_local: sun_dir,
            solar_irradiance_w_m2: scenario.solar_irradiance_w_m2,
            sunlit,
            default_shaft_power_w: scenario.shaft_power_w,
        };
        let out = bus.step(tree, loads, &env, None, |_| sun_dir)?;
        *total_shed_wh += out.total_shed_w * (dt_s / SECONDS_PER_HOUR);
        let (_, soc) = battery_totals(bus);
        *min_soc = (*min_soc).min(soc);
        remaining -= dt_s;
    }
    Ok(())
}

fn battery_totals(bus: &ElectricalBus) -> (f64, f64) {
    let total_wh: f64 = bus.batteries().iter().map(|(_, s)| s.energy_wh).sum();
    let total_cap: f64 = bus
        .batteries()
        .iter()
        .map(|(spec, _)| spec.capacity_wh)
        .sum();
    let soc = if total_cap > 0.0 {
        (total_wh / total_cap).clamp(0.0, 1.0)
    } else {
        1.0
    };
    (total_wh, soc)
}

/// Temperature bounds observed for one thermal node over a thermal steady-state sweep.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ThermalNodeBounds {
    /// Target component id (`ComponentId.0`).
    pub component_id: u32,
    /// Minimum node temperature observed, K.
    pub min_temp_k: f64,
    /// Maximum node temperature observed, K.
    pub max_temp_k: f64,
    /// Final node temperature at end of sweep, K.
    pub final_temp_k: f64,
}

/// Summary of thermal operating bounds over `steps` ticks under [`ThermalStepEnv`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ThermalEnvelopeReport {
    /// Recorded temperature bounds for each registered thermal node.
    pub nodes: Vec<ThermalNodeBounds>,
    /// Whether any registered node exceeded its `max_operating_temp_k`.
    pub any_overtemp: bool,
    /// Whether any registered node dropped below its `min_operating_temp_k`.
    pub any_undertemp: bool,
}

/// Advances a cloned [`ThermalNetwork`] over `steps` ticks to verify steady-state thermal limits.
pub fn analyze_thermal_envelope(
    net: &ThermalNetwork,
    tree: &ComponentTree,
    env: &ThermalStepEnv,
    loads: &[ThermalLoad],
    steps: u32,
) -> Result<ThermalEnvelopeReport, DesignError> {
    if steps == 0 {
        return Err(DesignError::InvalidConfig {
            detail: "steps must be >= 1".into(),
        });
    }
    let mut sim_net = net.clone();
    let registered_ids: Vec<ComponentId> = tree
        .ids()
        .filter(|id| sim_net.temperature_k(*id).is_ok())
        .collect();
    let mut bounds: Vec<ThermalNodeBounds> = registered_ids
        .iter()
        .filter_map(|&id| {
            sim_net.temperature_k(id).ok().map(|t| ThermalNodeBounds {
                component_id: id.0,
                min_temp_k: t,
                max_temp_k: t,
                final_temp_k: t,
            })
        })
        .collect();

    let mut any_overtemp = false;
    let mut any_undertemp = false;
    for _ in 0..steps {
        let out = sim_net.step(tree, env, loads, |_| PowerRailState::nominal(28.0))?;
        if !out.overtemp_nodes.is_empty() {
            any_overtemp = true;
        }
        if !out.undertemp_nodes.is_empty() {
            any_undertemp = true;
        }
        for slot in &mut bounds {
            if let Ok(t) = sim_net.temperature_k(ComponentId(slot.component_id)) {
                slot.min_temp_k = slot.min_temp_k.min(t);
                slot.max_temp_k = slot.max_temp_k.max(t);
                slot.final_temp_k = t;
            }
        }
    }

    Ok(ThermalEnvelopeReport {
        nodes: bounds,
        any_overtemp,
        any_undertemp,
    })
}
