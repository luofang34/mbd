//! The force-element contract: anything that loads a model as a pure
//! function of time, kinematics, its own state and the inputs.

use alloc::boxed::Box;

use glam::DVec3;
use mbd_core::WrenchAtSite;

use crate::MultibodyError;
use crate::aba::{JointDriveInput, JointLoadInput};
use crate::model::{Kinematics, Loads, MultibodyModel, Resolution};

mod sets;

/// What a force element sees when it is evaluated.
#[derive(Debug)]
pub struct ElementContext<'a, I: ?Sized> {
    /// Time of the evaluation, s.
    pub time_s: f64,
    /// The model being evaluated.
    pub model: &'a MultibodyModel,
    /// The model block of the flat state being evaluated: positions, then
    /// rates. The operators of the model take it as their configuration.
    pub state: &'a [f64],
    /// Link poses and velocities at the evaluated state.
    pub kinematics: &'a Kinematics<'a>,
    /// Whatever the host supplies from outside the model: terrain, fluid,
    /// fields, commands.
    pub inputs: &'a I,
}

/// A force element with optional continuous state and optional discrete
/// events.
///
/// An element owns no mutable data. Its continuous state is a block of the
/// integrated state vector; its discrete state is a block of modes that only
/// [`ForceElement::transition`] changes. An integrator therefore needs to
/// know nothing about any element, and a snapshot of the state value
/// captures every element.
///
/// # Contract
///
/// - [`ForceElement::evaluate`] is a pure function of its arguments. It is
///   called at integrator stages, at perturbed states and at rejected trial
///   states, any number of times.
/// - [`ForceElement::transition`] is called only at the end of an accepted
///   step, never at a stage. Latches, cuts and phase changes belong there, so
///   the right-hand side stays smooth within a step.
/// - Elements are evaluated in a fixed order and add into [`Loads`] in that
///   order.
///
/// # Constraint elements
///
/// An element that enforces a constraint exactly (contact or loop closure
/// by Lagrange multipliers) needs the motion the other elements would
/// produce without it. It reports [`ForceElement::has_resolution`] and
/// implements [`ForceElement::resolve`], which runs after every element's
/// `evaluate` with a [`Resolution`]: the free generalized acceleration under
/// the loads accumulated so far, and the operators of the equations of
/// motion at this state. For constraint rows at body points it forms
/// `Jᵀ e_k` with [`Resolution::point_force_to_generalized`], `M⁻¹ Jᵀ e_k`
/// with [`Resolution::apply_inverse_mass`] and the Delassus matrix
/// `A = J M⁻¹ Jᵀ` with [`Resolution::point_velocity`]; the right-hand side
/// is the free acceleration of the points from
/// [`Resolution::point_acceleration`]. It solves `A λ = −a_free` for the
/// multipliers and adds `Jᵀ λ` as point forces through [`Loads`].
/// Stabilisation states go in its continuous block and active-set changes
/// in `transition`.
pub trait ForceElement<I: ?Sized> {
    /// Number of continuous states this element integrates.
    fn state_len(&self) -> usize {
        0
    }

    /// Number of discrete mode values this element keeps.
    fn mode_len(&self) -> usize {
        0
    }

    /// Whether [`ForceElement::transition`] can ever change anything; lets a
    /// model skip the boundary pass entirely.
    fn has_transitions(&self) -> bool {
        false
    }

    /// Writes the initial continuous state and modes.
    fn initial_state(&self, state: &mut [f64], modes: &mut [f64]) {
        state.fill(0.0);
        modes.fill(0.0);
    }

    /// Writes the characteristic magnitude of each continuous state, in its
    /// own unit. Tolerances and finite-difference steps scale with it.
    fn state_scale(&self, scale: &mut [f64]) {
        scale.fill(1.0);
    }

    /// Adds this element's loads at the given state and writes the rate of
    /// its continuous state.
    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError>;

    /// Whether [`ForceElement::resolve`] does anything; without any such
    /// element the model solves its accelerations once.
    fn has_resolution(&self) -> bool {
        false
    }

    /// Adds loads that depend on the motion the other elements produce:
    /// called once per evaluation after every element's `evaluate`, with the
    /// free acceleration and the operators of the equations of motion in
    /// `resolution`. Like `evaluate`, a pure function of its arguments.
    fn resolve(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &[f64],
        modes: &[f64],
        resolution: &mut Resolution<'_>,
        loads: &mut Loads<'_>,
    ) -> Result<(), MultibodyError> {
        let _unused = (ctx, state, modes, resolution, loads);
        Ok(())
    }

    /// Applies discrete events at an accepted step boundary. Returns whether
    /// the state or modes changed, in which case an integrator discards any
    /// derivative it kept from before the boundary.
    fn transition(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        let _unused = (ctx, state, modes);
        Ok(false)
    }
}

/// Uniform field acceleration in world axes, m/s².
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UniformGravity(pub DVec3);

impl<I: ?Sized> ForceElement<I> for UniformGravity {
    fn evaluate(
        &self,
        _ctx: &ElementContext<'_, I>,
        _state: &[f64],
        _modes: &[f64],
        loads: &mut Loads<'_>,
        _state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        loads.add_gravity(self.0);
        Ok(())
    }
}

/// A stateless element given as a closure of the evaluation context, for
/// loads that follow the state within a step: a central gravity field, a
/// support reaction, a feedback law.
#[derive(Debug, Clone, Copy)]
pub struct ForceFn<F>(pub F);

impl<I: ?Sized, F> ForceElement<I> for ForceFn<F>
where
    F: Fn(&ElementContext<'_, I>, &mut Loads<'_>) -> Result<(), MultibodyError>,
{
    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        _state: &[f64],
        _modes: &[f64],
        loads: &mut Loads<'_>,
        _state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        (self.0)(ctx, loads)
    }
}

/// A wrench held constant in its component's axes.
impl<I: ?Sized> ForceElement<I> for WrenchAtSite {
    fn evaluate(
        &self,
        _ctx: &ElementContext<'_, I>,
        _state: &[f64],
        _modes: &[f64],
        loads: &mut Loads<'_>,
        _state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        loads.add_wrench(self)
    }
}

/// An actuator command held constant.
impl<I: ?Sized> ForceElement<I> for JointDriveInput {
    fn evaluate(
        &self,
        _ctx: &ElementContext<'_, I>,
        _state: &[f64],
        _modes: &[f64],
        loads: &mut Loads<'_>,
        _state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        loads.add_joint_drive(self)
    }
}

/// An unlimited joint load held constant.
impl<I: ?Sized> ForceElement<I> for JointLoadInput {
    fn evaluate(
        &self,
        _ctx: &ElementContext<'_, I>,
        _state: &[f64],
        _modes: &[f64],
        loads: &mut Loads<'_>,
        _state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        loads.add_joint_load(self)
    }
}

/// Type-erased element, for hosts that assemble heterogeneous lists.
pub type BoxedElement<I> = Box<dyn ForceElement<I>>;
