use glam::DVec3;
use mbd_core::{ComponentTree, PowerRailState};
use mbd_systems::{FluidNetwork, TankId};
use serde::{Deserialize, Serialize};

use crate::error::{DesignError, check_finite, check_positive};

#[cfg(test)]
mod tests;

/// One fuel-loading and maneuvering load scenario for CG & static-margin sweep.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TankFillScenario {
    /// Human-readable loading condition label (e.g. `"takeoff_heavy"`, `"mach2_cruise_trim"`).
    pub name: String,
    /// Overrides `(tank_id, fill_fraction in 0.0..=1.0)` applied on top of baseline fluids.
    pub fill_fractions: Vec<(u16, f64)>,
    /// Local accelerometer specific force felt at tank sites in component axes, m/s².
    pub specific_force_local_m_s2: [f64; 3],
}

/// Mass, CG, and static margin outcome for one [`TankFillScenario`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CgScenarioResult {
    /// Scenario name.
    pub name: String,
    /// Total attached mass (structure + payload + loaded fluids), kg.
    pub total_mass_kg: f64,
    /// Combined center of gravity in root body axes, m.
    pub cg_body_m: [f64; 3],
    /// Longitudinal static margin `(cg_x - aero_center_x) / mac` (`> 0` stable with nose `+X`).
    pub static_margin_mac: f64,
}

/// Complete longitudinal CG travel and static stability envelope across fuel burn/trim scenarios.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CgEnvelopeReport {
    /// Per-scenario evaluations in input order.
    pub scenarios: Vec<CgScenarioResult>,
    /// Aftmost (smallest) X CG across scenarios, m.
    pub min_cg_x_m: f64,
    /// Forwardmost (largest) X CG across scenarios, m.
    pub max_cg_x_m: f64,
    /// Minimum longitudinal static margin in MAC fraction (`>= 0` indicates all stable).
    pub min_static_margin_mac: f64,
    /// Maximum longitudinal static margin in MAC fraction.
    pub max_static_margin_mac: f64,
    /// Whether every evaluated scenario has non-negative static margin.
    pub statically_stable_all: bool,
}

/// Evaluates vehicle CG and longitudinal static margin (`(cg_x - ac_x) / mac` with `+X` forward)
/// across multiple fuel load / trim states using [`FluidNetwork::mass_contributions`].
pub fn analyze_cg_envelope(
    tree: &ComponentTree,
    baseline_fluids: &FluidNetwork,
    scenarios: &[TankFillScenario],
    mac_m: f64,
    aero_center_x_m: f64,
) -> Result<CgEnvelopeReport, DesignError> {
    if scenarios.is_empty() {
        return Err(DesignError::InvalidConfig {
            detail: "at least one TankFillScenario is required".into(),
        });
    }
    check_positive("trim.mac_m", mac_m)?;
    check_finite("trim.aero_center_x_m", aero_center_x_m)?;

    let mut results = Vec::with_capacity(scenarios.len());
    let mut min_cg_x = f64::INFINITY;
    let mut max_cg_x = f64::NEG_INFINITY;
    let mut min_margin = f64::INFINITY;
    let mut max_margin = f64::NEG_INFINITY;

    for scenario in scenarios {
        if scenario.name.trim().is_empty() {
            return Err(DesignError::InvalidConfig {
                detail: "scenario name must not be empty".into(),
            });
        }
        let f_spec = DVec3::from_array(scenario.specific_force_local_m_s2);
        if !f_spec.is_finite() {
            return Err(DesignError::InvalidConfig {
                detail: format!("scenario {:?} has non-finite specific force", scenario.name),
            });
        }
        let mut fluids = baseline_fluids.clone();
        for &(tid_raw, fill) in &scenario.fill_fractions {
            if !fill.is_finite() || !(0.0..=1.0).contains(&fill) {
                return Err(DesignError::InvalidParameter {
                    field: "trim.fill_fraction",
                    value: fill,
                });
            }
            let tid = TankId(tid_raw);
            let cap = fluids.tank_spec(tid)?.capacity_mass_kg;
            fluids.tank_state_mut(tid)?.mass_kg = cap * fill;
        }
        let step_out = fluids.step(
            0.0,
            tree,
            |_| f_spec,
            |_| Some(0.0),
            |_| PowerRailState::nominal(28.0),
        )?;
        let mass_props = tree.mass_properties_with_contributions(&step_out.mass_contributions)?;
        let cg = mass_props.cg_body_m;
        let margin = (cg.x - aero_center_x_m) / mac_m;

        min_cg_x = min_cg_x.min(cg.x);
        max_cg_x = max_cg_x.max(cg.x);
        min_margin = min_margin.min(margin);
        max_margin = max_margin.max(margin);
        results.push(CgScenarioResult {
            name: scenario.name.clone(),
            total_mass_kg: mass_props.total_mass_kg,
            cg_body_m: cg.to_array(),
            static_margin_mac: margin,
        });
    }

    Ok(CgEnvelopeReport {
        scenarios: results,
        min_cg_x_m: min_cg_x,
        max_cg_x_m: max_cg_x,
        min_static_margin_mac: min_margin,
        max_static_margin_mac: max_margin,
        statically_stable_all: min_margin >= 0.0,
    })
}
