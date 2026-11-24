//! A model with its elements and inputs as an [`OdeSystem`], and the
//! one-call step built on it.

use super::layout::{BASE_POSITION, read3, write3};
use super::{Kinematics, ModelState, MultibodyModel, StateKind, Workspace};
use crate::MultibodyError;
use crate::element::{ElementContext, ForceElement};
use crate::integrate::{Integrator, OdeSystem, StepReport};

/// A model, its force elements and the inputs held over one step, viewed as
/// a first-order system on the flat state.
///
/// The vector this system integrates is the flat state with its translation
/// blocks in centre-of-mass coordinates
/// ([`MultibodyModel::root_to_centre_of_mass`]): the centre of mass's
/// position rate is its velocity and its acceleration is the net external
/// force over the mass, so no rotation or joint motion reaches the
/// trajectory through an integrator's truncation.
/// [`MultibodyModel::advance`] converts a [`ModelState`] on the way in and
/// out; a caller driving an integrator on this type converts with
/// [`MultibodyModel::root_to_centre_of_mass`] and
/// [`MultibodyModel::centre_of_mass_to_root`] itself.
#[derive(Debug)]
pub struct CentreOfMassOde<'a, I: ?Sized, E: ?Sized> {
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

impl<I: ?Sized, E: ForceElement<I> + ?Sized> OdeSystem for CentreOfMassOde<'_, I, E> {
    fn dim(&self) -> usize {
        self.model.layout.len() + self.elements.state_len()
    }

    fn derivative(&mut self, t: f64, x: &[f64], x_dot: &mut [f64]) -> Result<(), MultibodyError> {
        let mut root = core::mem::take(&mut self.work.root_state);
        let outcome = self.derivative_from(t, x, &mut root, x_dot);
        self.work.root_state = root;
        outcome
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
                StateKind::Length => scale.length_m(),
                StateKind::LinearRate => scale.speed_m_s(),
                StateKind::AngularRate => scale.angular_rate_rad_s(),
                StateKind::Attitude | StateKind::Angle | StateKind::Auxiliary => 1.0,
            };
        }
        if let Some(aux) = magnitudes.get_mut(n..) {
            self.elements.state_scale(&scale, aux);
        }
    }

    fn accept(&mut self, t: f64, x: &mut [f64]) -> Result<bool, MultibodyError> {
        if !self.elements.has_transitions() {
            return Ok(false);
        }
        let mut root = core::mem::take(&mut self.work.root_state);
        let outcome = self.accept_from(t, x, &mut root);
        self.work.root_state = root;
        outcome
    }
}

impl<I: ?Sized, E: ForceElement<I> + ?Sized> CentreOfMassOde<'_, I, E> {
    /// Derivative at the centre-of-mass coordinates `x`, with `root` as the
    /// buffer for the same state in root-origin coordinates.
    fn derivative_from(
        &mut self,
        t: f64,
        x: &[f64],
        root: &mut alloc::vec::Vec<f64>,
        x_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        root.clear();
        root.extend_from_slice(x);
        let base = self.model.centre_of_mass_to_root(root, self.work)?;
        self.model.derivative_at_frames(
            t,
            root,
            base,
            self.modes,
            self.elements,
            self.inputs,
            self.work,
            x_dot,
        )?;
        let nq = self.model.layout.position_len();
        write3(x_dot, BASE_POSITION, read3(x, nq));
        write3(x_dot, nq, self.model.centre_of_mass_accel(self.work)?);
        Ok(())
    }

    fn accept_from(
        &mut self,
        t: f64,
        x: &mut [f64],
        root: &mut alloc::vec::Vec<f64>,
    ) -> Result<bool, MultibodyError> {
        root.clear();
        root.extend_from_slice(x);
        let base = self.model.centre_of_mass_to_root(root, self.work)?;
        let kinematics = Kinematics {
            links: &self.model.links,
            order_of: &self.model.order_of,
            frames: &self.work.frames,
            base,
        };
        let n = self.model.layout.len();
        let short = MultibodyError::StateLengthMismatch {
            expected: n,
            actual: x.len(),
        };
        let model_block = root.get(..n).ok_or(short.clone())?;
        let aux = x.get_mut(n..).ok_or(short)?;
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
    /// applying element events at every accepted step boundary. The
    /// translation is integrated as the centre of mass's
    /// ([`CentreOfMassOde`]) and written back as the root origin's. On
    /// error `state` is left exactly as it was passed in.
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
        if dt == 0.0 {
            // Nothing to integrate: skip the change of coordinates, whose
            // round trip would cost the state a rounding.
            let mut system = CentreOfMassOde {
                model: self,
                elements,
                inputs,
                modes,
                work,
            };
            return integrator.advance(&mut system, t0, dt, x);
        }
        work.backup.0.clear();
        work.backup.0.extend_from_slice(x);
        work.backup.1.clear();
        work.backup.1.extend_from_slice(modes);
        let mut system = CentreOfMassOde {
            model: self,
            elements,
            inputs,
            modes,
            work,
        };
        let outcome = self
            .root_to_centre_of_mass(x, system.work)
            .and_then(|()| integrator.advance(&mut system, t0, dt, x))
            .and_then(|report| {
                self.centre_of_mass_to_root(x, system.work)?;
                Ok(report)
            });
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
