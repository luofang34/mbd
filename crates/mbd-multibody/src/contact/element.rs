//! A contact point as a force element: three continuous states (the bristle
//! deflection) and one mode (the friction latch).

use super::{ContactPointSpec, ContactPointState, SurfaceQuery};
use crate::MultibodyError;
use crate::element::{ElementContext, ForceElement};
use crate::model::Loads;
use crate::model::layout::{read1, read3, write1, write3};

impl ContactPointState {
    /// Reads the state from an element's continuous block and mode block.
    pub fn from_blocks(state: &[f64], modes: &[f64]) -> Self {
        Self {
            sticking: read1(modes, 0) != 0.0,
            tangent_deflection_world_m: read3(state, 0),
        }
    }

    /// Writes the state into an element's continuous block and mode block.
    pub fn write_blocks(&self, state: &mut [f64], modes: &mut [f64]) {
        write3(state, 0, self.tangent_deflection_world_m);
        write1(modes, 0, if self.sticking { 1.0 } else { 0.0 });
    }
}

impl<I: SurfaceQuery + ?Sized> ForceElement<I> for ContactPointSpec {
    fn state_len(&self) -> usize {
        3
    }

    fn mode_len(&self) -> usize {
        1
    }

    fn has_transitions(&self) -> bool {
        true
    }

    fn initial_state(&self, state: &mut [f64], modes: &mut [f64]) {
        ContactPointState::default().write_blocks(state, modes);
    }

    fn state_scale(&self, scale: &mut [f64]) {
        // The bristle deflects by about the normal penetration.
        let depth = self.damping_ramp_depth_m;
        scale.fill(if depth > 0.0 { depth } else { 1.0 });
    }

    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        if !ctx.kinematics.is_attached(self.component) {
            state_dot.fill(0.0);
            return Ok(());
        }
        let point = ctx.kinematics.point(self.component, self.offset_local_m)?;
        let surface = ctx.inputs.surface_at(point.position_world_m);
        let out = ContactPointSpec::evaluate(
            self,
            &ContactPointState::from_blocks(state, modes),
            &point,
            &surface,
        );
        write3(state_dot, 0, out.deflection_rate_world_m_s);
        loads.add_wrench(&out.wrench(self, &point))
    }

    fn transition(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        if !ctx.kinematics.is_attached(self.component) {
            return Ok(false);
        }
        let point = ctx.kinematics.point(self.component, self.offset_local_m)?;
        let surface = ctx.inputs.surface_at(point.position_world_m);
        let mut typed = ContactPointState::from_blocks(state, modes);
        let changed = ContactPointSpec::transition(self, &mut typed, &point, &surface);
        typed.write_blocks(state, modes);
        Ok(changed)
    }
}
