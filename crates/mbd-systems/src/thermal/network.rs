use alloc::vec::Vec;

use mbd_core::{
    AmbientGas, ComponentId, ComponentTree, ElectricalLoad, PowerRailState, ThermalLoad,
};

use crate::error::{SystemsError, check_non_negative, check_positive, check_unit_interval};

const STEFAN_BOLTZMANN_W_M2_K4: f64 = 5.670_374_419e-8;
const MIN_TEMPERATURE_K: f64 = 2.7;

/// Thermal lumped-capacitance node bound to a [`ComponentId`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalNodeSpec {
    /// Component id represented by this thermal node.
    pub component: ComponentId,
    /// Lumped thermal capacitance $m c_p$, J/K.
    pub heat_capacity_j_k: f64,
    /// External radiative surface area rejecting to environment/space, m².
    pub radiative_area_m2: f64,
    /// Surface infrared emissivity $\epsilon \in [0, 1]$.
    pub emissivity: f64,
    /// Solar optical absorptivity $\alpha_s \in [0, 1]$.
    pub solar_absorptivity: f64,
    /// Projected solar illumination area, m².
    pub solar_area_m2: f64,
    /// External convective wet area exposed to planetary atmosphere, m².
    pub convective_area_m2: f64,
    /// Minimum safe operating temperature bound, K.
    pub min_operating_temp_k: f64,
    /// Maximum safe operating temperature bound, K.
    pub max_operating_temp_k: f64,
}

impl ThermalNodeSpec {
    /// Validates node thermophysical properties.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_positive("thermal.heat_capacity_j_k", self.heat_capacity_j_k)?;
        check_non_negative("thermal.radiative_area_m2", self.radiative_area_m2)?;
        check_unit_interval("thermal.emissivity", self.emissivity)?;
        check_unit_interval("thermal.solar_absorptivity", self.solar_absorptivity)?;
        check_non_negative("thermal.solar_area_m2", self.solar_area_m2)?;
        check_non_negative("thermal.convective_area_m2", self.convective_area_m2)?;
        check_positive("thermal.min_operating_temp_k", self.min_operating_temp_k)?;
        check_positive("thermal.max_operating_temp_k", self.max_operating_temp_k)?;
        if self.min_operating_temp_k > self.max_operating_temp_k {
            return Err(SystemsError::InvalidConfig {
                detail: alloc::format!(
                    "thermal node {} min_operating_temp_k > max_operating_temp_k",
                    self.component.0
                ),
            });
        }
        Ok(())
    }
}

/// Runtime state of a [`ThermalNodeSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalNodeState {
    /// Current node temperature, K.
    pub temperature_k: f64,
}

/// Conductive or pumped-coolant thermal coupling link between two components.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalLinkSpec {
    /// First endpoint component id.
    pub node_a: ComponentId,
    /// Second endpoint component id.
    pub node_b: ComponentId,
    /// Inter-node thermal conductance, W/K.
    pub conductance_w_k: f64,
    /// If true, represents an actively pumped coolant loop requiring `power_w`.
    pub pumped_loop: bool,
    /// Pump electrical demand when active, W (`0.0` if passive conduction).
    pub pump_power_w: f64,
    /// If true, heat transfer drops to zero once either node detaches in `ComponentTree`.
    pub requires_attachment: bool,
    /// Electrical shedding priority for active coolant pumps.
    pub priority: u8,
}

impl ThermalLinkSpec {
    /// Validates thermal link endpoints and conductance.
    pub fn validate(&self) -> Result<(), SystemsError> {
        if self.node_a == self.node_b {
            return Err(SystemsError::InvalidConfig {
                detail: alloc::format!("thermal link connects node {} to itself", self.node_a.0),
            });
        }
        check_non_negative("thermal_link.conductance_w_k", self.conductance_w_k)?;
        check_non_negative("thermal_link.pump_power_w", self.pump_power_w)?;
        Ok(())
    }
}

/// Environment inputs to step [`ThermalNetwork`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalStepEnv {
    /// Time step duration, s.
    pub dt_s: f64,
    /// Undisturbed atmospheric state (`AmbientGas::VACUUM` in space).
    pub ambient: AmbientGas,
    /// Vehicle speed relative to ambient gas (used for convective film coefficient), m/s.
    pub airspeed_m_s: f64,
    /// Deep-space / sky radiative sink temperature, K (typically `2.7`–`250.0` K).
    pub radiative_sink_temp_k: f64,
    /// Incident solar irradiance magnitude, W/m².
    pub solar_irradiance_w_m2: f64,
    /// False during eclipse shadow.
    pub sunlit: bool,
}

/// Telemetry and limit-flag output from a [`ThermalNetwork::step`] call.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ThermalStepOutput {
    /// Nodes currently exceeding `max_operating_temp_k`.
    pub overtemp_nodes: Vec<ComponentId>,
    /// Nodes currently below `min_operating_temp_k`.
    pub undertemp_nodes: Vec<ComponentId>,
    /// Electrical power loads demanded by running coolant pump loops.
    pub electrical_loads: Vec<ElectricalLoad>,
}

/// Lumped-parameter multi-node thermal conductance, convection, and radiation network.
#[derive(Debug, Clone, Default)]
pub struct ThermalNetwork {
    nodes: Vec<(ThermalNodeSpec, ThermalNodeState)>,
    links: Vec<ThermalLinkSpec>,
}

impl ThermalNetwork {
    /// Creates an empty thermal network.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns all registered thermal node specifications and live states.
    pub fn nodes(&self) -> &[(ThermalNodeSpec, ThermalNodeState)] {
        &self.nodes
    }

    /// Returns all registered passive and pumped thermal exchange links.
    pub fn links(&self) -> &[ThermalLinkSpec] {
        &self.links
    }

    /// Builds a new [`ThermalNetwork`] keeping only nodes hosted in `comp_map` and links
    /// whose both endpoints remain inside the subset.
    pub fn remap_subset(
        &self,
        comp_map: &[(ComponentId, ComponentId)],
    ) -> Result<Self, SystemsError> {
        let mut out = Self::new();
        append_thermal_subset(self, comp_map, &mut out)?;
        Ok(out)
    }

    /// Merges two [`ThermalNetwork`] instances onto a combined component tree.
    pub fn merge_remapped(
        &self,
        self_comp_map: &[(ComponentId, ComponentId)],
        other: &Self,
        other_comp_map: &[(ComponentId, ComponentId)],
    ) -> Result<Self, SystemsError> {
        let mut out = self.remap_subset(self_comp_map)?;
        append_thermal_subset(other, other_comp_map, &mut out)?;
        Ok(out)
    }

    /// Registers a thermal node with initial temperature in Kelvin.
    pub fn add_node(
        &mut self,
        spec: ThermalNodeSpec,
        initial_temp_k: f64,
    ) -> Result<(), SystemsError> {
        spec.validate()?;
        check_positive("thermal.initial_temp_k", initial_temp_k)?;
        if self
            .nodes
            .iter()
            .any(|(n, _)| n.component == spec.component)
        {
            return Err(SystemsError::DuplicateId {
                kind: "thermal node",
                id: alloc::format!("{}", spec.component.0),
            });
        }
        self.nodes.push((
            spec,
            ThermalNodeState {
                temperature_k: initial_temp_k,
            },
        ));
        Ok(())
    }

    /// Registers a passive or pumped thermal conductance link between two registered nodes.
    pub fn add_link(&mut self, link: ThermalLinkSpec) -> Result<(), SystemsError> {
        link.validate()?;
        let _ = self.node_index(link.node_a)?;
        let _ = self.node_index(link.node_b)?;
        self.links.push(link);
        Ok(())
    }

    /// Reads current temperature of `component`, K.
    pub fn temperature_k(&self, component: ComponentId) -> Result<f64, SystemsError> {
        let idx = self.node_index(component)?;
        self.nodes
            .get(idx)
            .map(|(_, s)| s.temperature_k)
            .ok_or_else(|| SystemsError::UnknownId {
                kind: "thermal node",
                id: alloc::format!("{}", component.0),
            })
    }

    /// Synchronizes all stored thermal node temperatures into matching [`ComponentTree`] states.
    pub fn sync_to_tree(&self, tree: &mut ComponentTree) -> Result<(), SystemsError> {
        for (spec, state) in &self.nodes {
            let mut comp_state = *tree.state(spec.component)?;
            comp_state.temperature_k = state.temperature_k;
            tree.set_state(spec.component, comp_state)?;
        }
        Ok(())
    }

    /// Advances node temperatures using semi-implicit linearized radiation + conduction/convection.
    pub fn step(
        &mut self,
        tree: &ComponentTree,
        env: &ThermalStepEnv,
        external_loads: &[ThermalLoad],
        power_rail: impl Fn(ComponentId) -> PowerRailState,
    ) -> Result<ThermalStepOutput, SystemsError> {
        check_non_negative("thermal.dt_s", env.dt_s)?;
        env.ambient.validate()?;
        check_non_negative("thermal.airspeed_m_s", env.airspeed_m_s)?;
        check_positive("thermal.radiative_sink_temp_k", env.radiative_sink_temp_k)?;
        check_non_negative("thermal.solar_irradiance_w_m2", env.solar_irradiance_w_m2)?;

        let n_nodes = self.nodes.len();
        let mut net_power_w = alloc::vec![0.0; n_nodes];
        let mut diag_conductance_w_k = alloc::vec![0.0; n_nodes];
        let mut output = ThermalStepOutput::default();

        for load in external_loads {
            if let Ok(idx) = self.node_index(load.component)
                && let Some(slot) = net_power_w.get_mut(idx)
            {
                *slot += load.heat_rate_w;
            }
        }
        self.accumulate_links(
            tree,
            &power_rail,
            &mut net_power_w,
            &mut diag_conductance_w_k,
            &mut output,
        )?;
        self.update_node_temperatures(env, &net_power_w, &diag_conductance_w_k, &mut output);
        Ok(output)
    }

    fn accumulate_links(
        &self,
        tree: &ComponentTree,
        power_rail: &impl Fn(ComponentId) -> PowerRailState,
        net_power_w: &mut [f64],
        diag_conductance_w_k: &mut [f64],
        output: &mut ThermalStepOutput,
    ) -> Result<(), SystemsError> {
        for link in &self.links {
            let idx_a = self.node_index(link.node_a)?;
            let idx_b = self.node_index(link.node_b)?;
            let attached_ok = !link.requires_attachment
                || (tree.is_attached(link.node_a)? && tree.is_attached(link.node_b)?);
            let pump_ok = !link.pumped_loop || power_rail(link.node_a).enabled;
            if !attached_ok || !pump_ok {
                continue;
            }
            if link.pumped_loop && link.pump_power_w > 0.0 {
                output.electrical_loads.push(ElectricalLoad {
                    component: link.node_a,
                    power_w: link.pump_power_w,
                    priority: link.priority,
                });
            }
            let t_a = self
                .nodes
                .get(idx_a)
                .map_or(293.15, |(_, s)| s.temperature_k);
            let t_b = self
                .nodes
                .get(idx_b)
                .map_or(293.15, |(_, s)| s.temperature_k);
            let q_a_to_b = link.conductance_w_k * (t_a - t_b);
            if let Some(slot) = net_power_w.get_mut(idx_a) {
                *slot -= q_a_to_b;
            }
            if let Some(slot) = net_power_w.get_mut(idx_b) {
                *slot += q_a_to_b;
            }
            if let Some(slot) = diag_conductance_w_k.get_mut(idx_a) {
                *slot += link.conductance_w_k;
            }
            if let Some(slot) = diag_conductance_w_k.get_mut(idx_b) {
                *slot += link.conductance_w_k;
            }
        }
        Ok(())
    }

    fn update_node_temperatures(
        &mut self,
        env: &ThermalStepEnv,
        net_power_w: &[f64],
        diag_conductance_w_k: &[f64],
        output: &mut ThermalStepOutput,
    ) {
        let conv_h = convective_coeff_w_m2_k(&env.ambient, env.airspeed_m_s);
        let t_sink4 = libm::pow(env.radiative_sink_temp_k, 4.0);
        for (idx, (spec, state)) in self.nodes.iter_mut().enumerate() {
            let t = state.temperature_k;
            let rad_k = spec.emissivity * spec.radiative_area_m2 * STEFAN_BOLTZMANN_W_M2_K4;
            let q_rad = rad_k * (libm::pow(t, 4.0) - t_sink4);
            let g_rad = 4.0 * rad_k * (t * t * t);
            let q_solar = if env.sunlit {
                spec.solar_absorptivity * spec.solar_area_m2 * env.solar_irradiance_w_m2
            } else {
                0.0
            };
            let (q_conv, g_conv) = if !env.ambient.is_vacuum() && spec.convective_area_m2 > 0.0 {
                let g = conv_h * spec.convective_area_m2;
                (g * (t - env.ambient.temperature_k), g)
            } else {
                (0.0, 0.0)
            };
            let q_ext = net_power_w.get(idx).copied().unwrap_or(0.0);
            let g_link = diag_conductance_w_k.get(idx).copied().unwrap_or(0.0);
            let q_total = q_ext + q_solar - q_rad - q_conv;
            if env.dt_s > 0.0 {
                let denom = spec.heat_capacity_j_k + env.dt_s * (g_link + g_conv + g_rad);
                state.temperature_k = (t + (env.dt_s * q_total) / denom).max(MIN_TEMPERATURE_K);
            }
            if state.temperature_k > spec.max_operating_temp_k {
                output.overtemp_nodes.push(spec.component);
            } else if state.temperature_k < spec.min_operating_temp_k {
                output.undertemp_nodes.push(spec.component);
            }
        }
    }

    fn node_index(&self, component: ComponentId) -> Result<usize, SystemsError> {
        self.nodes
            .iter()
            .position(|(spec, _)| spec.component == component)
            .ok_or_else(|| SystemsError::UnknownId {
                kind: "thermal node",
                id: alloc::format!("{}", component.0),
            })
    }
}

fn lookup_comp(map: &[(ComponentId, ComponentId)], old: ComponentId) -> Option<ComponentId> {
    map.iter().find(|&&(k, _)| k == old).map(|&(_, v)| v)
}

fn append_thermal_subset(
    src: &ThermalNetwork,
    comp_map: &[(ComponentId, ComponentId)],
    dst: &mut ThermalNetwork,
) -> Result<(), SystemsError> {
    for &(mut spec, state) in &src.nodes {
        if let Some(new_comp) = lookup_comp(comp_map, spec.component) {
            spec.component = new_comp;
            dst.add_node(spec, state.temperature_k)?;
        }
    }
    for &link in &src.links {
        if let (Some(node_a), Some(node_b)) = (
            lookup_comp(comp_map, link.node_a),
            lookup_comp(comp_map, link.node_b),
        ) {
            dst.add_link(ThermalLinkSpec {
                node_a,
                node_b,
                ..link
            })?;
        }
    }
    Ok(())
}

fn convective_coeff_w_m2_k(ambient: &AmbientGas, airspeed_m_s: f64) -> f64 {
    if ambient.is_vacuum() || ambient.density_kg_m3 <= 0.0 {
        return 0.0;
    }
    let density_ratio = (ambient.density_kg_m3 / 1.225).clamp(0.0, 2.0);
    let h_free = 5.0 * libm::sqrt(density_ratio);
    let h_forced = 0.03 * airspeed_m_s * libm::pow(density_ratio, 0.8);
    h_free + h_forced
}
