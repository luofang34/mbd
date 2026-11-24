use alloc::vec::Vec;

use mbd_core::ComponentId;

use super::network::{FluidNetwork, TransferLineSpec, TransferLineState};
use super::tank::TankId;
use crate::error::SystemsError;

fn lookup_comp(map: &[(ComponentId, ComponentId)], old: ComponentId) -> Option<ComponentId> {
    map.iter().find(|&&(k, _)| k == old).map(|&(_, v)| v)
}

fn lookup_tank(map: &[(TankId, TankId)], old: TankId) -> Option<TankId> {
    map.iter().find(|&&(k, _)| k == old).map(|&(_, v)| v)
}

impl FluidNetwork {
    /// Builds a new [`FluidNetwork`] retaining only tanks hosted on components present in
    /// `comp_map`, remapping `TankSpec::component` and preserving internal transfer lines
    /// whose both endpoints remain in the subset.
    pub fn remap_subset(
        &self,
        comp_map: &[(ComponentId, ComponentId)],
    ) -> Result<Self, SystemsError> {
        let mut out = Self::new();
        let mut kept_ids: Vec<TankId> = Vec::new();
        for (spec, state) in &self.tanks {
            if let Some(new_comp) = lookup_comp(comp_map, spec.component) {
                let mut cloned = spec.clone();
                cloned.component = new_comp;
                let tid = cloned.id;
                out.add_tank(cloned, *state)?;
                kept_ids.push(tid);
            }
        }
        for &(spec, state) in &self.lines {
            if kept_ids.contains(&spec.from_tank) && kept_ids.contains(&spec.to_tank) {
                out.add_transfer_line(spec, state)?;
            }
        }
        Ok(out)
    }

    /// Merges two [`FluidNetwork`] instances onto a combined component tree, remapping
    /// components and reassigning non-colliding `TankId`s / line IDs on `other`.
    /// Returns `(merged_network, other_tank_id_map)`.
    pub fn merge_remapped(
        &self,
        self_comp_map: &[(ComponentId, ComponentId)],
        other: &Self,
        other_comp_map: &[(ComponentId, ComponentId)],
    ) -> Result<(Self, Vec<(TankId, TankId)>), SystemsError> {
        let mut out = self.remap_subset(self_comp_map)?;
        let mut next_tank_id = out
            .tanks
            .iter()
            .map(|(s, _)| s.id.0)
            .max()
            .map_or(0_u16, |m| m.wrapping_add(1));
        let mut next_line_id = out
            .lines
            .iter()
            .map(|(s, _)| s.id)
            .max()
            .map_or(0_u16, |m| m.wrapping_add(1));

        let mut other_tank_map: Vec<(TankId, TankId)> = Vec::new();
        for (spec, state) in &other.tanks {
            let Some(new_comp) = lookup_comp(other_comp_map, spec.component) else {
                continue;
            };
            let new_tid = TankId(next_tank_id);
            next_tank_id = next_tank_id.wrapping_add(1);
            other_tank_map.push((spec.id, new_tid));
            let mut cloned = spec.clone();
            cloned.id = new_tid;
            cloned.component = new_comp;
            out.add_tank(cloned, *state)?;
        }
        append_remapped_lines(other, &other_tank_map, &mut next_line_id, &mut out)?;
        Ok((out, other_tank_map))
    }
}

fn append_remapped_lines(
    other: &FluidNetwork,
    other_tank_map: &[(TankId, TankId)],
    next_line_id: &mut u16,
    out: &mut FluidNetwork,
) -> Result<(), SystemsError> {
    for &(spec, state) in &other.lines {
        if let (Some(new_from), Some(new_to)) = (
            lookup_tank(other_tank_map, spec.from_tank),
            lookup_tank(other_tank_map, spec.to_tank),
        ) {
            let new_lid = *next_line_id;
            *next_line_id = (*next_line_id).wrapping_add(1);
            out.add_transfer_line(
                TransferLineSpec {
                    id: new_lid,
                    from_tank: new_from,
                    to_tank: new_to,
                    ..spec
                },
                TransferLineState {
                    last_flow_kg_s: state.last_flow_kg_s,
                    ..state
                },
            )?;
        }
    }
    Ok(())
}
