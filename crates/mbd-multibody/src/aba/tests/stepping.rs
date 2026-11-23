//! Tree-in, tree-out RK4 stepping rigs for the reference tests. Each step
//! prepares a model from the tree, advances an explicit state with
//! [`Rk4`] through the public interface and writes the joints back, so the
//! tests exercise the same path a host does.

use glam::DVec3;
use mbd_core::{ComponentTree, DynamicMassContribution, WrenchAtSite};

use crate::{
    FloatingBaseState, ForceFn, Integrator, JointDriveInput, JointLoadInput, ModelOde,
    MultibodyError, MultibodyModel, Rk4, Workspace,
};

/// Everything a stage callback of [`step_rk4_with_loads`] supplies.
#[derive(Debug, Clone, PartialEq, Default)]
pub(crate) struct StageForces {
    /// Uniform gravity in world axes, m/s².
    pub(crate) gravity_world_m_s2: DVec3,
    /// Actuator commands, limited per joint by the actuator.
    pub(crate) joint_drives: Vec<JointDriveInput>,
    /// Constraint and mechanism joint loads, never limited.
    pub(crate) joint_loads: Vec<JointLoadInput>,
    /// External wrenches at component sites.
    pub(crate) external_wrenches: Vec<WrenchAtSite>,
}

/// One RK4 step under constant gravity, drives and wrenches.
pub(crate) fn step_rk4(
    tree: &mut ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    base_state: &FloatingBaseState,
    gravity_world_m_s2: DVec3,
    joint_drives: &[JointDriveInput],
    external_wrenches: &[WrenchAtSite],
    dt_s: f64,
) -> Result<FloatingBaseState, MultibodyError> {
    step_rk4_with_forces(tree, dynamic_masses, base_state, dt_s, |_, _, _| {
        Ok((
            gravity_world_m_s2,
            joint_drives.to_vec(),
            external_wrenches.to_vec(),
        ))
    })
}

/// One RK4 step whose gravity, drives and wrenches are re-evaluated at
/// every stage from `(tree, stage_base, stage_time_s)`.
pub(crate) fn step_rk4_with_forces<E, F>(
    tree: &mut ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    base_state: &FloatingBaseState,
    dt_s: f64,
    mut eval_forces: F,
) -> Result<FloatingBaseState, E>
where
    E: From<MultibodyError> + Into<MultibodyError>,
    F: FnMut(
        &ComponentTree,
        &FloatingBaseState,
        f64,
    ) -> Result<(DVec3, Vec<JointDriveInput>, Vec<WrenchAtSite>), E>,
{
    step_rk4_with_loads(tree, dynamic_masses, base_state, dt_s, |t, b, dt| {
        eval_forces(t, b, dt).map(|(gravity_world_m_s2, joint_drives, external_wrenches)| {
            StageForces {
                gravity_world_m_s2,
                joint_drives,
                joint_loads: Vec::new(),
                external_wrenches,
            }
        })
    })
}

/// [`step_rk4_with_forces`] whose stage callback also returns unlimited
/// joint loads.
pub(crate) fn step_rk4_with_loads<E, F>(
    tree: &mut ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    base_state: &FloatingBaseState,
    dt_s: f64,
    eval_forces: F,
) -> Result<FloatingBaseState, E>
where
    E: From<MultibodyError> + Into<MultibodyError>,
    F: FnMut(&ComponentTree, &FloatingBaseState, f64) -> Result<StageForces, E>,
{
    let model = MultibodyModel::new(tree, dynamic_masses)?;
    let view = tree.clone();
    let callback = std::cell::RefCell::new(eval_forces);
    let element = ForceFn(
        |ctx: &crate::ElementContext<'_, ()>, loads: &mut crate::Loads<'_>| {
            let forces = (callback.borrow_mut())(&view, ctx.kinematics.base(), ctx.time_s)
                .map_err(Into::into)?;
            loads.add_gravity(forces.gravity_world_m_s2);
            for d in &forces.joint_drives {
                loads.add_joint_drive(d)?;
            }
            for l in &forces.joint_loads {
                loads.add_joint_load(l)?;
            }
            for w in &forces.external_wrenches {
                loads.add_wrench(w)?;
            }
            Ok(())
        },
    );
    let mut state = model.state(tree, base_state, &element, &())?;
    let mut work = Workspace::new();
    let (x, modes) = state.parts_mut();
    let mut system = ModelOde {
        model: &model,
        elements: &element,
        inputs: &(),
        modes,
        work: &mut work,
    };
    Rk4::new().advance(&mut system, 0.0, dt_s, x)?;
    model.write_joints(&state, tree)?;
    Ok(state.base())
}
