//! A tether as a force element between two anchors, each on a link of the
//! model or fixed in the world.

use glam::DVec3;
use mbd_core::ComponentId;

use super::{TetherCommand, TetherCutReason, TetherEnds, WinchTetherSpec, WinchTetherState};
use crate::MultibodyError;
use crate::element::{ElementContext, ForceElement};
use crate::model::layout::{read1, write1};
use crate::model::{Kinematics, Loads};

/// Where one end of a tether is attached.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TetherAnchor {
    /// A point fixed to a component of the model.
    Link {
        /// Component carrying the anchor.
        component: ComponentId,
        /// Anchor offset in the component's axes, m.
        offset_local_m: DVec3,
    },
    /// A point moving in the world independently of the model: the ground,
    /// or another vehicle whose motion the host holds over the step.
    World {
        /// Anchor position in world axes, m.
        position_world_m: DVec3,
        /// Anchor velocity in world axes, m/s.
        velocity_world_m_s: DVec3,
    },
}

/// A tether between two anchors. Its continuous states are the paid-out
/// length, the absorbed heat and the low-tension timer; its modes are the
/// intact flag and the cut reason.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tether {
    /// Line properties.
    pub spec: WinchTetherSpec,
    /// End A.
    pub anchor_a: TetherAnchor,
    /// End B.
    pub anchor_b: TetherAnchor,
    /// Paid-out length the element starts with, m.
    pub initial_length_m: f64,
    /// Command held over the step.
    pub command: TetherCommand,
}

const REASONS: [TetherCutReason; 4] = [
    TetherCutReason::Commanded,
    TetherCutReason::OverloadSnap,
    TetherCutReason::ThermalBurnThrough,
    TetherCutReason::TouchdownAutoCut,
];

impl WinchTetherState {
    /// Reads the state from an element's continuous block and mode block.
    pub fn from_blocks(state: &[f64], modes: &[f64]) -> Self {
        let code = read1(modes, 1);
        Self {
            intact: read1(modes, 0) != 0.0,
            deployed_length_m: read1(state, 0),
            absorbed_thermal_j: read1(state, 1),
            low_tension_timer_s: read1(state, 2),
            cut_reason: (1u8..)
                .zip(REASONS)
                .find_map(|(k, reason)| (f64::from(k) == code).then_some(reason)),
        }
    }

    /// Writes the state into an element's continuous block and mode block.
    pub fn write_blocks(&self, state: &mut [f64], modes: &mut [f64]) {
        write1(state, 0, self.deployed_length_m);
        write1(state, 1, self.absorbed_thermal_j);
        write1(state, 2, self.low_tension_timer_s);
        write1(modes, 0, if self.intact { 1.0 } else { 0.0 });
        let code = (1u8..)
            .zip(REASONS)
            .find_map(|(k, reason)| (Some(reason) == self.cut_reason).then_some(f64::from(k)));
        write1(modes, 1, code.unwrap_or(0.0));
    }
}

impl Tether {
    fn ends(&self, kin: &Kinematics<'_>) -> Result<TetherEnds, MultibodyError> {
        let resolve = |anchor: &TetherAnchor| match *anchor {
            TetherAnchor::Link {
                component,
                offset_local_m,
            } => kin
                .point(component, offset_local_m)
                .map(|p| (p.position_world_m, p.velocity_world_m_s)),
            TetherAnchor::World {
                position_world_m,
                velocity_world_m_s,
            } => Ok((position_world_m, velocity_world_m_s)),
        };
        let (anchor_a_world_m, vel_a_world_m_s) = resolve(&self.anchor_a)?;
        let (anchor_b_world_m, vel_b_world_m_s) = resolve(&self.anchor_b)?;
        Ok(TetherEnds {
            anchor_a_world_m,
            vel_a_world_m_s,
            anchor_b_world_m,
            vel_b_world_m_s,
        })
    }
}

fn apply(
    anchor: &TetherAnchor,
    force_world_n: DVec3,
    kin: &Kinematics<'_>,
    loads: &mut Loads<'_>,
) -> Result<(), MultibodyError> {
    let TetherAnchor::Link {
        component,
        offset_local_m,
    } = *anchor
    else {
        return Ok(());
    };
    let point = kin.point(component, offset_local_m)?;
    loads.add_force(component, offset_local_m, point.to_link(force_world_n))
}

impl<I: ?Sized> ForceElement<I> for Tether {
    fn state_len(&self) -> usize {
        3
    }

    fn mode_len(&self) -> usize {
        2
    }

    fn has_transitions(&self) -> bool {
        true
    }

    fn initial_state(&self, state: &mut [f64], modes: &mut [f64]) {
        WinchTetherState::intact_at(self.initial_length_m).write_blocks(state, modes);
    }

    fn state_scale(&self, scale: &mut [f64]) {
        let positive = |v: f64| if v > 0.0 { v } else { 1.0 };
        write1(scale, 0, positive(self.spec.max_length_m));
        write1(
            scale,
            1,
            positive(self.spec.burn_through_energy_j.unwrap_or(1.0)),
        );
        write1(scale, 2, positive(self.spec.auto_cut_dwell_s));
    }

    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let typed = WinchTetherState::from_blocks(state, modes);
        let ends = self.ends(ctx.kinematics)?;
        let out = self.spec.evaluate(&typed, &self.command, &ends);
        write1(state_dot, 0, out.deployed_length_rate_m_s);
        write1(state_dot, 1, out.absorbed_thermal_rate_w);
        write1(state_dot, 2, out.low_tension_timer_rate);
        apply(
            &self.anchor_a,
            out.force_on_a_world_n,
            ctx.kinematics,
            loads,
        )?;
        apply(
            &self.anchor_b,
            out.force_on_b_world_n,
            ctx.kinematics,
            loads,
        )
    }

    fn transition(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        let before = WinchTetherState::from_blocks(state, modes);
        let mut typed = before;
        let ends = self.ends(ctx.kinematics)?;
        self.spec.transition(&mut typed, &self.command, &ends);
        typed.write_blocks(state, modes);
        Ok(typed != before)
    }
}
