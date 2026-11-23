//! Tree-based entry points: one-shot evaluations that prepare a model,
//! evaluate once and discard it. Repeated evaluation goes through
//! [`MultibodyModel`] with a caller-owned [`Workspace`].

use alloc::vec::Vec;

use glam::DVec3;
use mbd_core::{ComponentTree, DynamicMassContribution, WrenchAtSite};

use super::{FloatingBaseState, JointDriveInput, JointLoadInput, MultiBodyAccel, SystemMomentum};
use crate::MultibodyError;
use crate::element::UniformGravity;
use crate::model::{MultibodyModel, Workspace};
use crate::spatial::SpatialMat6;

/// Floating-base Featherstone Articulated-Body Algorithm (ABA) with automatic
/// zero-DoF [`RigidFastPath`](MultiBodyAccel::used_rigid_fast_path) dispatch.
pub struct MultiBodyDynamics;

impl MultiBodyDynamics {
    /// Solves exact forward dynamics $\ddot{\mathbf{q}}$ and floating-base
    /// acceleration for `tree` at `base_state`, with actuator commands only.
    pub fn forward_dynamics(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
        base_state: &FloatingBaseState,
        gravity_world_m_s2: DVec3,
        joint_drives: &[JointDriveInput],
        external_wrenches: &[WrenchAtSite],
    ) -> Result<MultiBodyAccel, MultibodyError> {
        Self::forward_dynamics_with_loads(
            tree,
            dynamic_masses,
            base_state,
            gravity_world_m_s2,
            joint_drives,
            &[],
            external_wrenches,
        )
    }

    /// [`Self::forward_dynamics`] with additional unlimited joint loads
    /// (constraints, springs, mechanisms) superposed on the actuator commands.
    pub fn forward_dynamics_with_loads(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
        base_state: &FloatingBaseState,
        gravity_world_m_s2: DVec3,
        joint_drives: &[JointDriveInput],
        joint_loads: &[JointLoadInput],
        external_wrenches: &[WrenchAtSite],
    ) -> Result<MultiBodyAccel, MultibodyError> {
        let model = MultibodyModel::new(tree, dynamic_masses)?;
        let elements = (
            UniformGravity(gravity_world_m_s2),
            joint_drives,
            joint_loads,
            external_wrenches,
        );
        let state = model.state(tree, base_state, &elements, &())?;
        let mut work = Workspace::new();
        let mut rates = alloc::vec![0.0; state.as_slice().len()];
        model.derivative(
            0.0,
            state.as_slice(),
            state.modes(),
            &elements,
            &(),
            &mut work,
            &mut rates,
        )?;
        Ok(MultiBodyAccel {
            base_spatial_accel_root: work.base_accel,
            joint_accels: Vec::from(work.joint_accels()),
            used_rigid_fast_path: work.used_rigid_fast_path,
        })
    }

    /// Computes total world mass, CoM, linear momentum, and angular momentum
    /// about the instantaneous system CoM across all attached links.
    pub fn system_momentum(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
        base_state: &FloatingBaseState,
    ) -> Result<SystemMomentum, MultibodyError> {
        let model = MultibodyModel::new(tree, dynamic_masses)?;
        let state = model.state(tree, base_state, &(), &())?;
        model.system_momentum(&state, &mut Workspace::new())
    }

    /// Computes total rigid-body spatial inertia in root axes assuming
    /// joints are momentarily frozen, used by mechanisms and Generalized Jacobians.
    pub fn locked_root_spatial_inertia(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
    ) -> Result<SpatialMat6, MultibodyError> {
        let model = MultibodyModel::new(tree, dynamic_masses)?;
        let state = model.state(tree, &FloatingBaseState::AT_REST, &(), &())?;
        model.locked_root_spatial_inertia(&state, &mut Workspace::new())
    }
}
