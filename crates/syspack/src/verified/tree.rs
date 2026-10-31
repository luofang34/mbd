use std::collections::BTreeMap;

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, MbdError, Placement};

use crate::{ComponentRecord, DynamicsSpec, SyspackError, SyspackManifest};

/// Temperature every component starts at, K. The manifest has no thermal
/// state: systems start at room temperature.
pub const INITIAL_COMPONENT_TEMPERATURE_K: f64 = 293.15;

fn component(
    record: &ComponentRecord,
    ids: &BTreeMap<String, ComponentId>,
) -> Result<(ComponentSpec, ComponentState), SyspackError> {
    let parent = match &record.parent {
        Some(key) => Some(
            ids.get(key)
                .copied()
                .ok_or_else(|| SyspackError::UnknownParent {
                    component: record.key.clone(),
                    parent: key.clone(),
                })?,
        ),
        None => None,
    };
    let [x, y, z, w] = record.rotation_xyzw;
    let spec = ComponentSpec {
        name: record.name.clone(),
        placement: Placement {
            parent,
            translation_m: DVec3::from_array(record.translation_m),
            // Stored as written so the tree's own unit check judges it.
            rotation: DQuat::from_xyzw(x, y, z, w),
        },
        dry_mass_kg: record.dry_mass_kg,
        dry_inertia_body: DMat3::from_cols_array_2d(&record.dry_inertia_body),
        thermal_capacity_j_k: record.thermal_capacity_j_k,
        nominal_power_w: record.nominal_power_w,
    };
    let state = ComponentState {
        fluid_mass_kg: record.initial_fluid_mass_kg,
        payload_mass_kg: record.initial_payload_mass_kg,
        attached: record.attached,
        ..ComponentState::at_temperature(INITIAL_COMPONENT_TEMPERATURE_K)
    };
    Ok((spec, state))
}

/// Builds the tree and its key lookup. Parents may appear in any order, and a
/// cycle among them is reported by key. A system that moves needs attached
/// mass to have dynamics at all; a stationary one may model no mass.
pub(super) fn build_tree(
    manifest: &SyspackManifest,
) -> Result<(ComponentTree, BTreeMap<String, ComponentId>), SyspackError> {
    let ids: BTreeMap<String, ComponentId> = manifest
        .components
        .iter()
        .zip((0u32..).map(ComponentId))
        .map(|(record, id)| (record.key.clone(), id))
        .collect();
    let components = manifest
        .components
        .iter()
        .map(|record| component(record, &ids))
        .collect::<Result<Vec<_>, _>>()?;
    let tree = ComponentTree::new(components).map_err(|error| match error {
        MbdError::HierarchyCycle { id } => cycle_error(manifest, id),
        other => SyspackError::InvalidTree(other),
    })?;
    if manifest.dynamics != DynamicsSpec::None {
        tree.mass_properties().map_err(SyspackError::InvalidTree)?;
    }
    Ok((tree, ids))
}

fn cycle_error(manifest: &SyspackManifest, id: ComponentId) -> SyspackError {
    let key = manifest
        .components
        .get(id.0 as usize)
        .map_or_else(|| id.to_string(), |record| record.key.clone());
    SyspackError::ComponentCycle { key }
}
