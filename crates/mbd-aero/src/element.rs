//! Aerodynamic elements as force elements of a multibody model: each is
//! bound to one component, sees the flow at that link and loads that link.

use glam::DVec3;
use mbd_core::{ComponentId, WrenchAtSite};
use mbd_multibody::{ElementContext, ForceElement, Kinematics, Loads, MultibodyError, SpatialMat6};

use crate::flow::{AeroInputs, AeroWrench, LinkFlow};
use crate::{
    BluntCapsuleAero, BodyAndFinAero, FlappingWingAero, HumanDummyAero, ParachuteCanopyAero,
    ParachuteState, RarefiedPanelAero, RotorBemAero, WingSurfaceAero,
};

#[cfg(test)]
mod tests;

/// The aerodynamic model of one element, in the axes of its link.
#[derive(Debug, Clone, PartialEq)]
pub enum AeroElementKind {
    /// Blunt planetary or reentry capsule.
    BluntCapsule(BluntCapsuleAero),
    /// Slender rocket body + fins.
    BodyAndFin(BodyAndFinAero),
    /// Swept / variable-sweep wing, stabilizer, or fin.
    WingSurface(WingSurfaceAero),
    /// Propeller or helicopter rotor disk (BEM).
    RotorBem(RotorBemAero),
    /// Rarefied free-molecular panel.
    RarefiedPanel(RarefiedPanelAero),
    /// Subsonic / supersonic parachute canopy, with its inflation fraction
    /// as one continuous state and its deployment latch as one mode.
    ParachuteCanopy(ParachuteCanopyAero),
    /// Ejected dummy or crew capsule occupant.
    HumanDummy(HumanDummyAero),
    /// Quasi-steady flapping insect wing, with added inertia.
    FlappingWing(FlappingWingAero),
}

/// An aerodynamic element bound to the component whose link it loads.
#[derive(Debug, Clone, PartialEq)]
pub struct AeroElement {
    /// Component the element is mounted on; its link carries the load.
    pub component: ComponentId,
    /// Aerodynamic model in that component's axes.
    pub kind: AeroElementKind,
}

/// Everything an element applies to its link at one instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroElementLoads {
    /// Load on the link, in link axes.
    pub wrench: AeroWrench,
    /// Added inertia of entrained air about the link origin, link axes.
    pub added_inertia: Option<SpatialMat6>,
    /// Rate of the element's continuous state (parachute inflation), 1/s.
    pub inflation_rate: f64,
}

impl AeroElement {
    /// Binds `kind` to `component`.
    pub const fn new(component: ComponentId, kind: AeroElementKind) -> Self {
        Self { component, kind }
    }

    /// Point of the link where the element samples the air, link axes, m.
    pub fn sample_point_link_m(&self) -> DVec3 {
        match &self.kind {
            AeroElementKind::BluntCapsule(m) => m.params().reference_point_body_m,
            AeroElementKind::BodyAndFin(m) => m.params().reference_point_body_m,
            AeroElementKind::WingSurface(m) => m.params().ac_position_body_m,
            AeroElementKind::RotorBem(m) => m.params().hub_position_body_m,
            AeroElementKind::RarefiedPanel(m) => m.params().center_body_m,
            AeroElementKind::ParachuteCanopy(m) => m.params().riser_attach_body_m,
            AeroElementKind::HumanDummy(m) => m.params().reference_point_body_m,
            AeroElementKind::FlappingWing(m) => m.params().root_link_m,
        }
    }

    /// The flow at the element's link: link motion relative to the air the
    /// host samples at the element, in link axes. Fails for a component
    /// outside the model's attached chain.
    pub fn flow<'a, I: AeroInputs + ?Sized>(
        &self,
        kinematics: &Kinematics<'_>,
        inputs: &'a I,
    ) -> Result<LinkFlow<'a>, MultibodyError> {
        let origin = kinematics.point(self.component, DVec3::ZERO)?;
        let sample = kinematics.point(self.component, self.sample_point_link_m())?;
        let air = inputs.air_at(sample.position_world_m);
        Ok(LinkFlow {
            velocity_air_link_m_s: origin.to_link(origin.velocity_world_m_s - air.wind_world_m_s),
            angular_rate_link_rad_s: origin.to_link(origin.angular_vel_world_rad_s),
            ambient: air.ambient,
            controls: inputs.controls(),
        })
    }

    /// The loads at `flow` with the element's continuous `state` and
    /// `modes` blocks: a pure function, which the force-element hook
    /// applies and a host may call for heat flux or other read-outs.
    pub fn loads(
        &self,
        flow: &LinkFlow<'_>,
        state: &[f64],
        modes: &[f64],
    ) -> Result<AeroElementLoads, MultibodyError> {
        let plain = |wrench| AeroElementLoads {
            wrench,
            added_inertia: None,
            inflation_rate: 0.0,
        };
        Ok(match &self.kind {
            AeroElementKind::BluntCapsule(m) => plain(m.evaluate(flow)?),
            AeroElementKind::BodyAndFin(m) => plain(m.evaluate(flow)?),
            AeroElementKind::WingSurface(m) => plain(m.evaluate(flow)?),
            AeroElementKind::RotorBem(m) => plain(m.evaluate(flow)?.aero),
            AeroElementKind::RarefiedPanel(m) => plain(m.evaluate(flow)?),
            AeroElementKind::HumanDummy(m) => plain(m.evaluate(flow)?),
            AeroElementKind::ParachuteCanopy(m) => {
                let canopy = parachute_state(m, flow, state, modes);
                AeroElementLoads {
                    wrench: m.evaluate_state(flow, &canopy)?,
                    added_inertia: None,
                    inflation_rate: m.inflation_rate(&canopy, m.canopy_dynamic_pressure_pa(flow)),
                }
            }
            AeroElementKind::FlappingWing(m) => {
                let out = m.evaluate(flow)?;
                AeroElementLoads {
                    wrench: out.wrench,
                    added_inertia: Some(out.added_inertia),
                    inflation_rate: 0.0,
                }
            }
        })
    }
}

/// The canopy state held in an element's blocks: inflation in the
/// continuous block, kept inside `0..=1` against integrator overshoot, the
/// deployment latch in the mode block, reefing from the controls.
fn parachute_state(
    m: &ParachuteCanopyAero,
    flow: &LinkFlow<'_>,
    state: &[f64],
    modes: &[f64],
) -> ParachuteState {
    ParachuteState {
        deployed: modes.first().is_some_and(|&d| d != 0.0),
        inflation_fraction: state.first().copied().unwrap_or(0.0).clamp(0.0, 1.0),
        reefing_area_fraction: m.reefing_fraction(flow).clamp(0.0, 1.0),
    }
}

impl<I: AeroInputs + ?Sized> ForceElement<I> for AeroElement {
    fn state_len(&self) -> usize {
        usize::from(matches!(self.kind, AeroElementKind::ParachuteCanopy(_)))
    }

    fn mode_len(&self) -> usize {
        usize::from(matches!(self.kind, AeroElementKind::ParachuteCanopy(_)))
    }

    fn has_transitions(&self) -> bool {
        matches!(self.kind, AeroElementKind::ParachuteCanopy(_))
    }

    fn initial_state(&self, state: &mut [f64], modes: &mut [f64]) {
        state.fill(0.0);
        modes.fill(0.0);
        if let AeroElementKind::ParachuteCanopy(m) = &self.kind {
            let p = m.params();
            if let Some(f) = state.first_mut() {
                *f = p.inflation_at_start;
            }
            if let Some(d) = modes.first_mut() {
                *d = if p.deployed_at_start { 1.0 } else { 0.0 };
            }
        }
    }

    fn evaluate(
        &self,
        ctx: &ElementContext<'_, I>,
        state: &[f64],
        modes: &[f64],
        loads: &mut Loads<'_>,
        state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        state_dot.fill(0.0);
        if !ctx.kinematics.is_attached(self.component) {
            return Ok(());
        }
        let flow = self.flow(ctx.kinematics, ctx.inputs)?;
        let out = self.loads(&flow, state, modes)?;
        if let Some(rate) = state_dot.first_mut() {
            *rate = out.inflation_rate;
        }
        if let Some(added) = out.added_inertia {
            loads.add_inertia(self.component, added)?;
        }
        loads.add_wrench(&WrenchAtSite {
            site: self.component,
            point_local_m: out.wrench.reference_point_link_m,
            force_local_n: out.wrench.force_link_n,
            torque_local_nm: out.wrench.moment_ref_link_nm,
        })
    }

    /// Latches a parachute's deployment once its command is seen at an
    /// accepted step boundary, so the load never switches inside a step.
    fn transition(
        &self,
        ctx: &ElementContext<'_, I>,
        _state: &mut [f64],
        modes: &mut [f64],
    ) -> Result<bool, MultibodyError> {
        let AeroElementKind::ParachuteCanopy(m) = &self.kind else {
            return Ok(false);
        };
        let Some(deployed) = modes.first_mut() else {
            return Ok(false);
        };
        if *deployed != 0.0 || !ctx.kinematics.is_attached(self.component) {
            return Ok(false);
        }
        let flow = self.flow(ctx.kinematics, ctx.inputs)?;
        if !m.deploy_commanded(&flow) {
            return Ok(false);
        }
        *deployed = 1.0;
        Ok(true)
    }
}
