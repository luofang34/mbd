//! The explicit state value of a model and its force elements.

use alloc::vec::Vec;

use super::layout::{BASE_ATTITUDE, BASE_POSITION, read_quat, read3, write_quat, write3};
use crate::aba::FloatingBaseState;

/// Complete dynamic state of a [`crate::MultibodyModel`] with its force
/// elements: one flat continuous vector plus the elements' discrete modes.
///
/// The continuous vector is laid out as [`crate::StateLayout`] describes and
/// is what an [`crate::Integrator`] advances. The modes are piecewise
/// constant: only [`crate::ForceElement::transition`] changes them, at
/// accepted step boundaries. Cloning the value is a full snapshot; restoring
/// a clone resumes the trajectory bit for bit.
#[derive(Debug, Clone, PartialEq)]
pub struct ModelState {
    x: Vec<f64>,
    modes: Vec<f64>,
    nq: usize,
    nv: usize,
}

impl ModelState {
    pub(crate) fn zeroed(nq: usize, nv: usize, aux: usize, modes: usize) -> Self {
        Self {
            x: alloc::vec![0.0; nq + nv + aux],
            modes: alloc::vec![0.0; modes],
            nq,
            nv,
        }
    }

    /// The continuous state as one flat vector.
    pub fn as_slice(&self) -> &[f64] {
        &self.x
    }

    /// Mutable flat continuous state. A caller that edits quaternion blocks
    /// must leave them unit length ([`crate::StateLayout::retract`]).
    pub fn as_mut_slice(&mut self) -> &mut [f64] {
        &mut self.x
    }

    /// Discrete element modes, in element order.
    pub fn modes(&self) -> &[f64] {
        &self.modes
    }

    /// Mutable discrete element modes, for a host that reconfigures an
    /// element between steps.
    pub fn modes_mut(&mut self) -> &mut [f64] {
        &mut self.modes
    }

    /// Continuous state and modes together, as an integrator and its
    /// transition hook need them.
    pub fn parts_mut(&mut self) -> (&mut [f64], &mut [f64]) {
        (&mut self.x, &mut self.modes)
    }

    /// Generalized positions `q`: base pose, then joint coordinates.
    pub fn positions(&self) -> &[f64] {
        self.x.get(..self.nq).unwrap_or(&[])
    }

    /// Generalized rates `v`: base linear velocity in world axes, base
    /// angular velocity in root axes, then joint rates.
    pub fn velocities(&self) -> &[f64] {
        self.x.get(self.nq..self.nq + self.nv).unwrap_or(&[])
    }

    /// Continuous states owned by force elements, in element order.
    pub fn auxiliary(&self) -> &[f64] {
        self.x.get(self.nq + self.nv..).unwrap_or(&[])
    }

    /// Mutable element states, for a host that resets an element.
    pub fn auxiliary_mut(&mut self) -> &mut [f64] {
        self.x.get_mut(self.nq + self.nv..).unwrap_or(&mut [])
    }

    /// Pose and twist of the floating base.
    pub fn base(&self) -> FloatingBaseState {
        base_of(&self.x, self.nq)
    }

    /// Replaces the pose and twist of the floating base.
    pub fn set_base(&mut self, base: &FloatingBaseState) {
        write3(&mut self.x, BASE_POSITION, base.position_world_m);
        write_quat(&mut self.x, BASE_ATTITUDE, base.rotation_world_from_root);
        write3(&mut self.x, self.nq, base.linear_vel_world_m_s);
        write3(&mut self.x, self.nq + 3, base.angular_vel_root_rad_s);
    }
}

/// Base state read from a flat state whose rate block starts at `nq`.
pub(crate) fn base_of(x: &[f64], nq: usize) -> FloatingBaseState {
    FloatingBaseState {
        position_world_m: read3(x, BASE_POSITION),
        rotation_world_from_root: read_quat(x, BASE_ATTITUDE),
        linear_vel_world_m_s: read3(x, nq),
        angular_vel_root_rad_s: read3(x, nq + 3),
    }
}
