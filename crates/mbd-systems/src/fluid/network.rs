use alloc::vec::Vec;

use glam::DVec3;
use mbd_core::{
    ComponentId, ComponentTree, DynamicMassContribution, ElectricalLoad, FaultState,
    PowerRailState, WrenchAtSite,
};

use super::tank::{FeedAvailability, TankId, TankSpec, TankState, TankStepInput, step_tank};
use crate::error::{SystemsError, check_finite, check_non_negative, check_positive};

/// Transfer line / boost-pump specification connecting two tanks of identical species.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransferLineSpec {
    /// Unique transfer line identifier.
    pub id: u16,
    /// Nominal source tank identifier.
    pub from_tank: TankId,
    /// Nominal destination tank identifier.
    pub to_tank: TankId,
    /// Maximum pump/valve mass flow magnitude, kg/s.
    pub max_flow_rate_kg_s: f64,
    /// Electrical power required per unit mass flow rate, W/(kg/s).
    pub power_w_per_kg_s: f64,
    /// Whether negative commanded flow reverses direction (`to_tank -> from_tank`).
    pub bidirectional: bool,
    /// Whether both host components must remain attached in `ComponentTree` for flow.
    pub requires_attachment: bool,
    /// Electrical shedding priority (`0` = critical).
    pub priority: u8,
}

impl TransferLineSpec {
    /// Validates parameters of a fluid transfer line.
    pub fn validate(&self) -> Result<(), SystemsError> {
        if self.from_tank == self.to_tank {
            return Err(SystemsError::InvalidConfig {
                detail: alloc::format!(
                    "transfer line {} connects tank {} to itself",
                    self.id,
                    self.from_tank.0
                ),
            });
        }
        check_positive("transfer.max_flow_rate_kg_s", self.max_flow_rate_kg_s)?;
        check_non_negative("transfer.power_w_per_kg_s", self.power_w_per_kg_s)?;
        Ok(())
    }
}

/// Runtime state of a [`TransferLineSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransferLineState {
    /// Commanded mass flow (`> 0` transfers `from -> to`; `< 0` reverses when bidirectional), kg/s.
    pub commanded_rate_kg_s: f64,
    /// Valve or boost-pump fault injection.
    pub fault: FaultState,
    /// Actual realized signed mass flow rate in the last step, kg/s.
    pub last_flow_kg_s: f64,
}

impl Default for TransferLineState {
    fn default() -> Self {
        Self {
            commanded_rate_kg_s: 0.0,
            fault: FaultState::Nominal,
            last_flow_kg_s: 0.0,
        }
    }
}

/// Aggregate outputs emitted by a [`FluidNetwork::step`] evaluation.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct FluidStepOutput {
    /// Total cryogenic boil-off vented across all tanks during this step, kg.
    pub total_boiloff_kg: f64,
    /// Total mass lost through injected tank leaks during this step, kg.
    pub total_leaked_kg: f64,
    /// Total conserved internal mass shifted between tanks during this step, kg.
    pub total_transferred_kg: f64,
    /// Dynamic liquid CG and inertia tensors per tank for `ComponentTree` aggregation.
    pub mass_contributions: Vec<DynamicMassContribution>,
    /// Slosh reaction wrenches emitted by tanks with active slosh modes.
    pub slosh_wrenches: Vec<WrenchAtSite>,
    /// Electrical loads consumed by active fluid boost pumps.
    pub electrical_loads: Vec<ElectricalLoad>,
}

/// Unified fluid storage and crossfeed/trim transfer network.
///
/// Conservation invariant: excluding external propulsion withdrawals, ABEP collections,
/// cryogenic boil-off, and leaks, total fluid mass across the network is conserved
/// across arbitrary multi-tank transfers to machine precision (`< 1e-12` kg).
#[derive(Debug, Clone, Default)]
pub struct FluidNetwork {
    pub(super) tanks: Vec<(TankSpec, TankState)>,
    pub(super) lines: Vec<(TransferLineSpec, TransferLineState)>,
}

impl FluidNetwork {
    /// Creates an empty fluid network.
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns all registered tank specifications and states.
    pub fn tanks(&self) -> &[(TankSpec, TankState)] {
        &self.tanks
    }

    /// Returns all registered transfer line specifications and states.
    pub fn transfer_lines(&self) -> &[(TransferLineSpec, TransferLineState)] {
        &self.lines
    }

    /// Registers a validated tank and initial state.
    pub fn add_tank(&mut self, spec: TankSpec, state: TankState) -> Result<(), SystemsError> {
        spec.validate()?;
        state.fault.validate()?;
        check_non_negative("tank_state.mass_kg", state.mass_kg)?;
        if state.mass_kg > spec.capacity_mass_kg {
            return Err(SystemsError::InvalidConfig {
                detail: alloc::format!(
                    "initial mass ({}) exceeds tank {} capacity ({})",
                    state.mass_kg,
                    spec.id.0,
                    spec.capacity_mass_kg
                ),
            });
        }
        if self
            .tanks
            .iter()
            .any(|(existing, _)| existing.id == spec.id)
        {
            return Err(SystemsError::DuplicateId {
                kind: "tank",
                id: alloc::format!("{}", spec.id.0),
            });
        }
        self.tanks.push((spec, state));
        Ok(())
    }

    /// Registers a transfer line between two registered tanks of identical [`super::FluidSpecies`].
    pub fn add_transfer_line(
        &mut self,
        spec: TransferLineSpec,
        state: TransferLineState,
    ) -> Result<(), SystemsError> {
        spec.validate()?;
        state.fault.validate()?;
        check_finite(
            "transfer_state.commanded_rate_kg_s",
            state.commanded_rate_kg_s,
        )?;
        if self
            .lines
            .iter()
            .any(|(existing, _)| existing.id == spec.id)
        {
            return Err(SystemsError::DuplicateId {
                kind: "transfer line",
                id: alloc::format!("{}", spec.id),
            });
        }
        let src_species = self.tank_spec(spec.from_tank)?.species;
        let dst_species = self.tank_spec(spec.to_tank)?.species;
        if src_species != dst_species {
            return Err(SystemsError::SpeciesMismatch {
                from: alloc::format!("{}", spec.from_tank.0),
                to: alloc::format!("{}", spec.to_tank.0),
            });
        }
        self.lines.push((spec, state));
        Ok(())
    }

    /// Returns static spec of `id`.
    pub fn tank_spec(&self, id: TankId) -> Result<&TankSpec, SystemsError> {
        self.tanks
            .iter()
            .find(|(spec, _)| spec.id == id)
            .map(|(spec, _)| spec)
            .ok_or_else(|| SystemsError::UnknownId {
                kind: "tank",
                id: alloc::format!("{}", id.0),
            })
    }

    /// Returns dynamic state of `id`.
    pub fn tank_state(&self, id: TankId) -> Result<&TankState, SystemsError> {
        self.tanks
            .iter()
            .find(|(spec, _)| spec.id == id)
            .map(|(_, state)| state)
            .ok_or_else(|| SystemsError::UnknownId {
                kind: "tank",
                id: alloc::format!("{}", id.0),
            })
    }

    /// Mutable access to `TankState`.
    pub fn tank_state_mut(&mut self, id: TankId) -> Result<&mut TankState, SystemsError> {
        self.tanks
            .iter_mut()
            .find(|(spec, _)| spec.id == id)
            .map(|(_, state)| state)
            .ok_or_else(|| SystemsError::UnknownId {
                kind: "tank",
                id: alloc::format!("{}", id.0),
            })
    }

    /// Updates commanded transfer rate (kg/s) on line `line_id`.
    pub fn set_line_command(
        &mut self,
        line_id: u16,
        commanded_rate_kg_s: f64,
    ) -> Result<(), SystemsError> {
        check_finite("transfer_state.commanded_rate_kg_s", commanded_rate_kg_s)?;
        let line = self
            .lines
            .iter_mut()
            .find(|(spec, _)| spec.id == line_id)
            .ok_or_else(|| SystemsError::UnknownId {
                kind: "transfer line",
                id: alloc::format!("{line_id}"),
            })?;
        line.1.commanded_rate_kg_s = commanded_rate_kg_s;
        Ok(())
    }

    /// Injects or clears a [`FaultState`] on transfer line `line_id`.
    pub fn set_line_fault(&mut self, line_id: u16, fault: FaultState) -> Result<(), SystemsError> {
        fault.validate()?;
        let line = self
            .lines
            .iter_mut()
            .find(|(spec, _)| spec.id == line_id)
            .ok_or_else(|| SystemsError::UnknownId {
                kind: "transfer line",
                id: alloc::format!("{line_id}"),
            })?;
        line.1.fault = fault;
        Ok(())
    }

    /// Total fluid mass currently stored across all tanks in the network, kg.
    pub fn total_fluid_mass_kg(&self) -> f64 {
        self.tanks.iter().map(|(_, state)| state.mass_kg).sum()
    }

    /// Total fluid mass currently stored for a specific `ComponentId`, kg.
    pub fn component_fluid_mass_kg(&self, component: ComponentId) -> f64 {
        self.tanks
            .iter()
            .filter(|(spec, _)| spec.component == component)
            .map(|(_, state)| state.mass_kg)
            .sum()
    }

    /// Live dynamic mass/CG/inertia entries for `ComponentTree::mass_properties_with_contributions`.
    ///
    /// Callers passing these contributions should leave `ComponentState::fluid_mass_kg` at `0.0`
    /// on covered tanks to avoid double counting.
    pub fn mass_contributions(&self) -> Result<Vec<DynamicMassContribution>, SystemsError> {
        let mut contributions = Vec::with_capacity(self.tanks.len());
        for (spec, state) in &self.tanks {
            contributions.push(spec.mass_contribution(state)?);
        }
        Ok(contributions)
    }

    /// Returns currently usable propellant mass in `tank_id` (`0.0` if unported, settled-out,
    /// faulted, or at/below `minimum_unusable_kg`).
    pub fn usable_propellant_kg(&self, tank_id: TankId) -> Result<f64, SystemsError> {
        let (spec, state) = self
            .tanks
            .iter()
            .find(|(s, _)| s.id == tank_id)
            .ok_or_else(|| SystemsError::UnknownId {
                kind: "tank",
                id: alloc::format!("{}", tank_id.0),
            })?;
        if state.availability != FeedAvailability::Available {
            return Ok(0.0);
        }
        Ok((state.mass_kg - spec.minimum_unusable_kg).max(0.0))
    }

    /// Withdraws up to `requested_kg` of usable propellant from `tank_id`, respecting
    /// [`FeedAvailability::Available`] and `minimum_unusable_kg`. Returns actual withdrawn mass, kg.
    pub fn withdraw_propellant(
        &mut self,
        tank_id: TankId,
        requested_kg: f64,
    ) -> Result<f64, SystemsError> {
        check_non_negative("withdraw.requested_kg", requested_kg)?;
        let (spec, state) = self.tank_pair_mut(tank_id)?;
        if requested_kg == 0.0 || state.availability != FeedAvailability::Available {
            return Ok(0.0);
        }
        let usable = (state.mass_kg - spec.minimum_unusable_kg).max(0.0);
        let withdrawn = requested_kg.min(usable);
        state.mass_kg -= withdrawn;
        if state.mass_kg <= spec.minimum_unusable_kg {
            state.availability = FeedAvailability::Depleted;
        }
        Ok(withdrawn)
    }

    /// Deposits collected fluid mass (e.g. from ABEP intake) into `tank_id`, capped at tank capacity.
    /// Returns the actual stored mass, kg.
    pub fn deposit_fluid(&mut self, tank_id: TankId, added_kg: f64) -> Result<f64, SystemsError> {
        check_non_negative("deposit.added_kg", added_kg)?;
        let (spec, state) = self.tank_pair_mut(tank_id)?;
        let headroom = (spec.capacity_mass_kg - state.mass_kg).max(0.0);
        let accepted = added_kg.min(headroom);
        state.mass_kg += accepted;
        Ok(accepted)
    }

    /// Advances all tanks (settling, boil-off, leaks, slosh) and executes conserved
    /// inter-tank transfers over `dt_s`.
    pub fn step(
        &mut self,
        dt_s: f64,
        tree: &ComponentTree,
        specific_force_local_m_s2: impl Fn(ComponentId) -> DVec3,
        heat_input_w: impl Fn(TankId) -> Option<f64>,
        power_rail: impl Fn(ComponentId) -> PowerRailState,
    ) -> Result<FluidStepOutput, SystemsError> {
        check_non_negative("fluid_step.dt_s", dt_s)?;
        let mut output = FluidStepOutput {
            mass_contributions: Vec::with_capacity(self.tanks.len()),
            slosh_wrenches: Vec::new(),
            electrical_loads: Vec::new(),
            ..FluidStepOutput::default()
        };

        for (spec, state) in &mut self.tanks {
            let step_out = step_tank(
                spec,
                state,
                TankStepInput {
                    dt_s,
                    specific_force_site_m_s2: specific_force_local_m_s2(spec.component),
                    heat_input_w: heat_input_w(spec.id),
                    relief_pressure_pa: None,
                },
            )?;
            output.total_boiloff_kg += step_out.boiloff_mass_kg;
            output.total_leaked_kg += step_out.leaked_mass_kg;
            if step_out.slosh_wrench_local.force_local_n.length_squared() > 0.0 {
                output.slosh_wrenches.push(step_out.slosh_wrench_local);
            }
        }

        if dt_s > 0.0 {
            self.execute_transfers(dt_s, tree, &power_rail, &mut output)?;
        }

        for (spec, state) in &self.tanks {
            output
                .mass_contributions
                .push(spec.mass_contribution(state)?);
        }
        Ok(output)
    }

    fn execute_transfers(
        &mut self,
        dt_s: f64,
        tree: &ComponentTree,
        power_rail: &impl Fn(ComponentId) -> PowerRailState,
        output: &mut FluidStepOutput,
    ) -> Result<(), SystemsError> {
        let line_count = self.lines.len();
        for idx in 0..line_count {
            self.step_single_line(idx, dt_s, tree, power_rail, output)?;
        }
        Ok(())
    }

    fn step_single_line(
        &mut self,
        idx: usize,
        dt_s: f64,
        tree: &ComponentTree,
        power_rail: &impl Fn(ComponentId) -> PowerRailState,
        output: &mut FluidStepOutput,
    ) -> Result<(), SystemsError> {
        let Some(&(spec, state_copy)) = self.lines.get(idx) else {
            return Ok(());
        };
        let signed_rate = effective_transfer_rate(spec, state_copy)?;
        if signed_rate == 0.0 {
            if let Some(entry) = self.lines.get_mut(idx) {
                entry.1.last_flow_kg_s = 0.0;
            }
            return Ok(());
        }
        let (src_id, dst_id, sign) = if signed_rate >= 0.0 {
            (spec.from_tank, spec.to_tank, 1.0)
        } else {
            (spec.to_tank, spec.from_tank, -1.0)
        };
        let src_idx = self.tank_index(src_id)?;
        let dst_idx = self.tank_index(dst_id)?;
        let Some((s_spec, s_state)) = self.tanks.get(src_idx) else {
            return Ok(());
        };
        let Some((d_spec, d_state)) = self.tanks.get(dst_idx) else {
            return Ok(());
        };
        let src_comp = s_spec.component;
        let src_usable = (s_state.mass_kg - s_spec.minimum_unusable_kg).max(0.0);
        let src_avail = s_state.availability;
        let dst_comp = d_spec.component;
        let dst_headroom = (d_spec.capacity_mass_kg - d_state.mass_kg).max(0.0);

        let attached_ok = !spec.requires_attachment
            || (tree.is_attached(src_comp)? && tree.is_attached(dst_comp)?);
        let rail_ok = spec.power_w_per_kg_s == 0.0 || power_rail(src_comp).enabled;

        let mut realized_mag_rate = 0.0;
        if attached_ok && rail_ok && src_avail == FeedAvailability::Available {
            let actual_dm = (signed_rate.abs() * dt_s).min(src_usable).min(dst_headroom);
            if actual_dm > 0.0 {
                if let Some((_, s_mut)) = self.tanks.get_mut(src_idx) {
                    s_mut.mass_kg -= actual_dm;
                }
                if let Some((_, d_mut)) = self.tanks.get_mut(dst_idx) {
                    d_mut.mass_kg += actual_dm;
                }
                output.total_transferred_kg += actual_dm;
                realized_mag_rate = actual_dm / dt_s;
                if spec.power_w_per_kg_s > 0.0 {
                    output.electrical_loads.push(ElectricalLoad {
                        component: src_comp,
                        power_w: realized_mag_rate * spec.power_w_per_kg_s,
                        priority: spec.priority,
                    });
                }
            }
        }
        if let Some(entry) = self.lines.get_mut(idx) {
            entry.1.last_flow_kg_s = sign * realized_mag_rate;
        }
        Ok(())
    }

    fn tank_index(&self, id: TankId) -> Result<usize, SystemsError> {
        self.tanks
            .iter()
            .position(|(spec, _)| spec.id == id)
            .ok_or_else(|| SystemsError::UnknownId {
                kind: "tank",
                id: alloc::format!("{}", id.0),
            })
    }

    fn tank_pair_mut(&mut self, id: TankId) -> Result<&mut (TankSpec, TankState), SystemsError> {
        self.tanks
            .iter_mut()
            .find(|(spec, _)| spec.id == id)
            .ok_or_else(|| SystemsError::UnknownId {
                kind: "tank",
                id: alloc::format!("{}", id.0),
            })
    }
}

fn effective_transfer_rate(
    spec: TransferLineSpec,
    state: TransferLineState,
) -> Result<f64, SystemsError> {
    state.fault.validate()?;
    let raw = match state.fault {
        FaultState::Nominal | FaultState::Leak { .. } | FaultState::SensorBias { .. } => {
            state.commanded_rate_kg_s
        }
        FaultState::StuckOff | FaultState::Jammed => 0.0,
        FaultState::StuckOn { fraction } => spec.max_flow_rate_kg_s * fraction,
        FaultState::ShortCircuit => spec.max_flow_rate_kg_s,
        FaultState::Degraded { efficiency } => state.commanded_rate_kg_s * efficiency,
    };
    let min_rate = if spec.bidirectional {
        -spec.max_flow_rate_kg_s
    } else {
        0.0
    };
    Ok(raw.clamp(min_rate, spec.max_flow_rate_kg_s))
}
