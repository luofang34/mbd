//! A model with its elements and inputs as an [`OdeSystem`], and the
//! one-call step built on it.

use super::{Kinematics, ModelState, MultibodyModel, StateKind, Workspace};
use crate::MultibodyError;
use crate::element::{ElementContext, ForceElement};
use crate::integrate::{Integrator, OdeSystem, StepReport};

/// A model, its force elements and the inputs held over one step, viewed as
/// a first-order system on the flat state.
#[derive(Debug)]
pub struct ModelOde<'a, I: ?Sized, E: ?Sized> {
    /// The prepared model.
    pub model: &'a MultibodyModel,
    /// Force elements loading it.
    pub elements: &'a E,
    /// Host inputs, constant over the step.
    pub inputs: &'a I,
    /// Element modes; only the accepted-step hook changes them.
    pub modes: &'a mut [f64],
    /// Evaluation scratch.
    pub work: &'a mut Workspace,
}

impl<I: ?Sized, E: ForceElement<I> + ?Sized> OdeSystem for ModelOde<'_, I, E> {
    fn dim(&self) -> usize {
        self.model.layout.len() + self.elements.state_len()
    }

    fn derivative(&mut self, t: f64, x: &[f64], x_dot: &mut [f64]) -> Result<(), MultibodyError> {
        self.model.derivative(
            t,
            x,
            self.modes,
            self.elements,
            self.inputs,
            self.work,
            x_dot,
        )
    }

    fn retract(&self, x: &mut [f64]) {
        self.model.layout.retract(x);
    }

    fn describe(&self, kinds: &mut [StateKind], magnitudes: &mut [f64]) {
        let n = self.model.layout.len();
        let scale = self.model.scale;
        kinds.fill(StateKind::Auxiliary);
        for (slot, kind) in kinds.iter_mut().zip(self.model.layout.kinds()) {
            *slot = *kind;
        }
        for (slot, kind) in magnitudes.iter_mut().zip(self.model.layout.kinds()) {
            *slot = match kind {
                StateKind::Length => scale.length_m,
                StateKind::LinearRate => scale.length_m / scale.time_s,
                StateKind::AngularRate => 1.0 / scale.time_s,
                StateKind::Attitude | StateKind::Angle | StateKind::Auxiliary => 1.0,
            };
        }
        if let Some(aux) = magnitudes.get_mut(n..) {
            self.elements.state_scale(aux);
        }
    }

    fn accept(&mut self, t: f64, x: &mut [f64]) -> Result<bool, MultibodyError> {
        if !self.elements.has_transitions() {
            return Ok(false);
        }
        let base = self.model.update_frames(x, self.work)?;
        let kinematics = Kinematics {
            links: &self.model.links,
            order_of: &self.model.order_of,
            frames: &self.work.frames,
            base,
        };
        let n = self.model.layout.len();
        let (model_block, aux) =
            x.split_at_mut_checked(n)
                .ok_or(MultibodyError::StateLengthMismatch {
                    expected: n,
                    actual: 0,
                })?;
        let ctx = ElementContext {
            time_s: t,
            model: self.model,
            state: model_block,
            kinematics: &kinematics,
            inputs: self.inputs,
        };
        self.elements.transition(&ctx, aux, self.modes)
    }
}

impl MultibodyModel {
    /// Advances `state` from `t0` to exactly `t0 + dt` with `integrator`,
    /// applying element events at every accepted step boundary. On error
    /// `state` is left exactly as it was passed in.
    #[allow(clippy::too_many_arguments)]
    pub fn advance<I: ?Sized, E: ForceElement<I> + ?Sized, G: Integrator>(
        &self,
        integrator: &mut G,
        elements: &E,
        inputs: &I,
        state: &mut ModelState,
        t0: f64,
        dt: f64,
        work: &mut Workspace,
    ) -> Result<StepReport, MultibodyError> {
        // A failed step must not leave the state at an internal boundary:
        // keep the starting value and put it back.
        let (x, modes) = state.parts_mut();
        work.backup.0.clear();
        work.backup.0.extend_from_slice(x);
        work.backup.1.clear();
        work.backup.1.extend_from_slice(modes);
        let mut system = ModelOde {
            model: self,
            elements,
            inputs,
            modes,
            work,
        };
        let outcome = integrator.advance(&mut system, t0, dt, x);
        if outcome.is_err() {
            let (x, modes) = state.parts_mut();
            for (slot, saved) in x.iter_mut().zip(&work.backup.0) {
                *slot = *saved;
            }
            for (slot, saved) in modes.iter_mut().zip(&work.backup.1) {
                *slot = *saved;
            }
        }
        outcome
    }
}
