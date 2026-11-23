use mbd_core::ComponentId;

use super::bus::ElectricalBus;
use super::sources::PowerGeneratorKind;
use crate::error::SystemsError;
use crate::fluid::TankId;

fn lookup_comp(map: &[(ComponentId, ComponentId)], old: ComponentId) -> Option<ComponentId> {
    map.iter().find(|&&(k, _)| k == old).map(|&(_, v)| v)
}

fn remap_tank(map: &[(TankId, TankId)], old: Option<TankId>) -> Option<TankId> {
    old.map(|tid| {
        map.iter()
            .find(|&&(k, _)| k == tid)
            .map_or(tid, |&(_, v)| v)
    })
}

fn bus_from_voltage(nominal_bus_voltage_v: f64) -> Result<ElectricalBus, SystemsError> {
    if nominal_bus_voltage_v == 0.0 {
        Ok(ElectricalBus::default())
    } else {
        ElectricalBus::new(nominal_bus_voltage_v)
    }
}

impl ElectricalBus {
    /// Builds a new [`ElectricalBus`] containing only sources and faults whose host
    /// components appear in `comp_map`, remapped to their new IDs.
    pub fn remap_subset(
        &self,
        comp_map: &[(ComponentId, ComponentId)],
    ) -> Result<Self, SystemsError> {
        let mut out = bus_from_voltage(self.nominal_bus_voltage_v)?;
        append_mapped_items(self, comp_map, &[], &mut out)?;
        Ok(out)
    }

    /// Merges two [`ElectricalBus`] instances onto a combined component tree, remapping
    /// components via `self_comp_map` / `other_comp_map` and remapping `FuelCell` tank IDs
    /// on `other` via `other_tank_map`.
    pub fn merge_remapped(
        &self,
        self_comp_map: &[(ComponentId, ComponentId)],
        other: &Self,
        other_comp_map: &[(ComponentId, ComponentId)],
        other_tank_map: &[(TankId, TankId)],
    ) -> Result<Self, SystemsError> {
        let merged_v = if self.nominal_bus_voltage_v > 0.0 {
            self.nominal_bus_voltage_v
        } else {
            other.nominal_bus_voltage_v
        };
        let mut out = bus_from_voltage(merged_v)?;
        append_mapped_items(self, self_comp_map, &[], &mut out)?;
        append_mapped_items(other, other_comp_map, other_tank_map, &mut out)?;
        Ok(out)
    }
}

fn append_mapped_items(
    src: &ElectricalBus,
    comp_map: &[(ComponentId, ComponentId)],
    tank_map: &[(TankId, TankId)],
    dst: &mut ElectricalBus,
) -> Result<(), SystemsError> {
    for &(mut spec, state) in &src.solar_arrays {
        if let Some(new_comp) = lookup_comp(comp_map, spec.component) {
            spec.component = new_comp;
            dst.add_solar_array(spec, state)?;
        }
    }
    for &(mut spec, state) in &src.batteries {
        if let Some(new_comp) = lookup_comp(comp_map, spec.component) {
            spec.component = new_comp;
            dst.add_battery(spec, state)?;
        }
    }
    for &(mut spec, state) in &src.generators {
        if let Some(new_comp) = lookup_comp(comp_map, spec.component) {
            spec.component = new_comp;
            if let PowerGeneratorKind::FuelCell {
                max_power_w,
                waste_heat_ratio,
                specific_reactant_kg_wh,
                h2_tank,
                o2_tank,
            } = spec.kind
            {
                spec.kind = PowerGeneratorKind::FuelCell {
                    max_power_w,
                    waste_heat_ratio,
                    specific_reactant_kg_wh,
                    h2_tank: remap_tank(tank_map, h2_tank),
                    o2_tank: remap_tank(tank_map, o2_tank),
                };
            }
            dst.add_generator(spec, state)?;
        }
    }
    for &(comp, fault) in &src.component_faults {
        if let Some(new_comp) = lookup_comp(comp_map, comp) {
            dst.set_component_fault(new_comp, fault)?;
        }
    }
    Ok(())
}
