use mbd_core::{ComponentId, ComponentTree};

use crate::error::{SystemsError, check_non_negative};

#[cfg(test)]
mod tests;

/// Conserved transfer of crew, lunar/Martian samples, or movable cargo payload mass
/// between two components in a [`ComponentTree`] (e.g. LM -> CM prior to LM jettison).
///
/// Fails if `require_connected` is true and either component is detached, or if
/// `mass_kg` exceeds `from`'s current `payload_mass_kg`. Returns transferred `mass_kg`.
pub fn transfer_payload_mass(
    tree: &mut ComponentTree,
    from: ComponentId,
    to: ComponentId,
    mass_kg: f64,
    require_connected: bool,
) -> Result<f64, SystemsError> {
    check_non_negative("payload.mass_kg", mass_kg)?;
    let mut from_state = *tree.state(from)?;
    let mut to_state = *tree.state(to)?;

    if require_connected && (!tree.is_attached(from)? || !tree.is_attached(to)?) {
        return Err(SystemsError::Rejected {
            detail: alloc::format!(
                "cannot transfer payload between detached components {} and {}",
                from.0,
                to.0
            ),
        });
    }
    if from == to || mass_kg == 0.0 {
        if mass_kg > from_state.payload_mass_kg {
            return Err(SystemsError::Rejected {
                detail: alloc::format!(
                    "requested payload transfer ({} kg) exceeds source component {} payload ({} kg)",
                    mass_kg,
                    from.0,
                    from_state.payload_mass_kg
                ),
            });
        }
        return Ok(0.0);
    }

    if mass_kg > from_state.payload_mass_kg {
        return Err(SystemsError::Rejected {
            detail: alloc::format!(
                "requested payload transfer ({} kg) exceeds source component {} payload ({} kg)",
                mass_kg,
                from.0,
                from_state.payload_mass_kg
            ),
        });
    }

    from_state.payload_mass_kg -= mass_kg;
    to_state.payload_mass_kg += mass_kg;
    tree.set_state(from, from_state)?;
    tree.set_state(to, to_state)?;
    Ok(mass_kg)
}

/// Drops up to `dropped_mass_kg` of external/internal payload from `component`
/// (e.g. sonde drop, ballast release, or unmodeled cargo release), returning actual dropped mass, kg.
pub fn drop_payload_mass(
    tree: &mut ComponentTree,
    component: ComponentId,
    dropped_mass_kg: f64,
) -> Result<f64, SystemsError> {
    check_non_negative("payload.dropped_mass_kg", dropped_mass_kg)?;
    let mut state = *tree.state(component)?;
    if !tree.is_attached(component)? {
        return Err(SystemsError::Rejected {
            detail: alloc::format!(
                "cannot drop payload from detached component {}",
                component.0
            ),
        });
    }
    let released = dropped_mass_kg.min(state.payload_mass_kg.max(0.0));
    state.payload_mass_kg -= released;
    tree.set_state(component, state)?;
    Ok(released)
}
