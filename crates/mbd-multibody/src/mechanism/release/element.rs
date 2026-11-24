//! Rail ejection and hinged-panel opening as force elements: the actuator
//! acts while the mechanism is armed, and the arm and end-of-travel latches
//! change only at accepted step boundaries. Separating the body is a change
//! of topology, so it stays with the host, which builds a new model after
//! [`super::release_from_rail`] or [`super::execute_impulse_release`].

use mbd_core::{ComponentId, JointKind};

use super::{HingeSwingSpec, HingeSwingState, RailSlideSpec, RailSlideState};
use crate::MultibodyError;
use crate::aba::{JointDriveInput, JointLoadInput};
use crate::element::{ElementContext, ForceElement};
use crate::model::layout::{read1, write1};
use crate::model::{Kinematics, Loads};

/// Reads two latches from a mode block.
fn latches(modes: &[f64]) -> (bool, bool) {
    (read1(modes, 0) != 0.0, read1(modes, 1) != 0.0)
}

fn write_latches(modes: &mut [f64], first: bool, second: bool) {
    write1(modes, 0, if first { 1.0 } else { 0.0 });
    write1(modes, 1, if second { 1.0 } else { 0.0 });
}

impl RailSlideState {
    /// Reads the state from a rail element's mode block.
    pub fn from_modes(modes: &[f64]) -> Self {
        let (fired, ready_to_release) = latches(modes);
        Self {
            fired,
            ready_to_release,
        }
    }

    /// Writes the state into a rail element's mode block, for a host that
    /// arms, resets or restores the mechanism between steps.
    pub fn write_modes(&self, modes: &mut [f64]) {
        write_latches(modes, self.fired, self.ready_to_release);
    }
}

impl HingeSwingState {
    /// Reads the state from a hinge element's mode block.
    pub fn from_modes(modes: &[f64]) -> Self {
        let (unlatched, ready_to_release) = latches(modes);
        Self {
            unlatched,
            ready_to_release,
        }
    }

    /// Writes the state into a hinge element's mode block, for a host that
    /// arms, resets or restores the mechanism between steps.
    pub fn write_modes(&self, modes: &mut [f64]) {
        write_latches(modes, self.unlatched, self.ready_to_release);
    }
}

/// The joint coordinate and rate of `component`, checked to be of the kind
/// the mechanism needs; `None` while it is detached.
fn joint_of(
    kinematics: &Kinematics<'_>,
    component: ComponentId,
    prismatic: bool,
) -> Result<Option<(f64, f64)>, MultibodyError> {
    if !kinematics.is_attached(component) {
        return Ok(None);
    }
    let fits = match kinematics.joint_kind(component)? {
        JointKind::Prismatic { .. } => prismatic,
        JointKind::Revolute { .. } => !prismatic,
        _ => false,
    };
    if !fits {
        return Err(MultibodyError::WrongJointKind { id: component });
    }
    let j = kinematics.joint(component)?;
    Ok(Some((j.q_scalar, j.q_vel_scalar)))
}

/// A guide-rail ejection (Vostok-1 SK-1 or K-36DM seat) as a force
/// element with two modes: fired, and ready to release.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RailSlide {
    /// Rail geometry and forces.
    pub spec: RailSlideSpec,
    /// Firing command held over the step; it latches at the next accepted
    /// boundary.
    pub fire: bool,
}

impl<I: ?Sized> ForceElement<I> for RailSlide {
    fn mode_len(&self) -> usize {
        2
    }

    fn has_transitions(&self) -> bool {
        true
    }

    /// While fired and not yet at the rail's end: the catapult as the
    /// rail's actuator command and Coulomb guide friction as a passive load
    /// that no actuator limit clips. At zero rate there is no sliding
    /// direction and the catapult acts alone.
    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        _state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        _state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let st = RailSlideState::from_modes(modes);
        if !st.fired || st.ready_to_release {
            return Ok(());
        }
        let Some((_, rate)) = joint_of(ctx.kinematics, self.spec.component, true)? else {
            return Ok(());
        };
        let friction = if rate == 0.0 {
            0.0
        } else {
            -self.spec.guide_friction_n * rate.signum()
        };
        let id = self.spec.component;
        loads.add_joint_drive(&JointDriveInput::scalar(id, self.spec.catapult_force_n))?;
        loads.add_joint_load(&JointLoadInput::scalar(id, friction))
    }

    fn transition(
        &self,
        ctx: &ElementContext<'_, I>,
        _state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        let before = RailSlideState::from_modes(modes);
        let Some((travel, _)) = joint_of(ctx.kinematics, self.spec.component, true)? else {
            return Ok(false);
        };
        let fired = before.fired || self.fire;
        let ready = before.ready_to_release || (fired && travel >= self.spec.rail_length_m);
        let after = RailSlideState {
            fired,
            ready_to_release: ready,
        };
        after.write_modes(modes);
        Ok(after != before)
    }
}

/// A hinged separation panel (Apollo SLA panel, fairing petal) as a force
/// element with two modes: unlatched, and ready to release.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HingeRelease {
    /// Hinge spring, damper and release angle.
    pub spec: HingeSwingSpec,
    /// Pyro command held over the step; it latches at the next accepted
    /// boundary.
    pub unlatch: bool,
}

impl<I: ?Sized> ForceElement<I> for HingeRelease {
    fn mode_len(&self) -> usize {
        2
    }

    fn has_transitions(&self) -> bool {
        true
    }

    /// While unlatched and short of the release angle: the opening spring
    /// and hinge damper as a passive joint load.
    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        _state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        _state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let st = HingeSwingState::from_modes(modes);
        if !st.unlatched || st.ready_to_release {
            return Ok(());
        }
        let Some((_, rate)) = joint_of(ctx.kinematics, self.spec.component, false)? else {
            return Ok(());
        };
        let torque = self.spec.spring_torque_nm - self.spec.damper_nm_s_rad * rate;
        loads.add_joint_load(&JointLoadInput::scalar(self.spec.component, torque))
    }

    fn transition(
        &self,
        ctx: &ElementContext<'_, I>,
        _state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        let before = HingeSwingState::from_modes(modes);
        let Some((angle, _)) = joint_of(ctx.kinematics, self.spec.component, false)? else {
            return Ok(false);
        };
        let open = before.unlatched || self.unlatch;
        let ready = before.ready_to_release || (open && angle >= self.spec.release_angle_rad);
        let after = HingeSwingState {
            unlatched: open,
            ready_to_release: ready,
        };
        after.write_modes(modes);
        Ok(after != before)
    }
}
