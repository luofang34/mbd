use alloc::vec::Vec;

use glam::DVec3;
use mbd_core::{AeroFlowState, AeroModel, AeroWrench, ComponentId, MbdError, PoseTable};

use crate::{
    BluntCapsuleAero, BodyAndFinAero, HumanDummyAero, ParachuteCanopyAero, RarefiedPanelAero,
    RotorBemAero, WingSurfaceAero,
};

#[cfg(test)]
mod tests;

/// Pluggable aerodynamic element bound to a specific [`ComponentId`].
#[derive(Debug, Clone, PartialEq)]
pub enum AeroElement {
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
    /// Subsonic / supersonic parachute canopy.
    ParachuteCanopy(ParachuteCanopyAero),
    /// Ejected dummy or crew capsule occupant.
    HumanDummy(HumanDummyAero),
}

impl AeroModel for AeroElement {
    fn evaluate(&self, flow: &AeroFlowState) -> Result<AeroWrench, MbdError> {
        match self {
            Self::BluntCapsule(m) => m.evaluate(flow),
            Self::BodyAndFin(m) => m.evaluate(flow),
            Self::WingSurface(m) => m.evaluate(flow),
            Self::RotorBem(m) => m.evaluate(flow),
            Self::RarefiedPanel(m) => m.evaluate(flow),
            Self::ParachuteCanopy(m) => m.evaluate(flow),
            Self::HumanDummy(m) => m.evaluate(flow),
        }
    }
}

/// Pair associating one [`AeroElement`] with a component link in [`mbd_core::ComponentTree`].
#[derive(Debug, Clone, PartialEq)]
pub struct BoundAeroElement {
    /// Target component link whose live pose places this element in root axes.
    pub component: ComponentId,
    /// Aerodynamic element evaluated in `component`'s local axes.
    pub element: AeroElement,
}

/// Multi-element vehicle aerodynamics aggregator respecting live [`PoseTable`]
/// attachment state, joint articulation transforms, and relative link velocities.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct CompositeAeroModel {
    elements: Vec<BoundAeroElement>,
}

impl CompositeAeroModel {
    /// Builds a composite aerodynamic model from bound component elements.
    pub fn new(elements: Vec<BoundAeroElement>) -> Self {
        Self { elements }
    }

    /// Appends one element bound to `component`.
    pub fn push(&mut self, component: ComponentId, element: AeroElement) {
        self.elements.push(BoundAeroElement { component, element });
    }

    /// Borrowed slice of bound elements.
    pub fn elements(&self) -> &[BoundAeroElement] {
        &self.elements
    }

    /// Evaluates every attached element using its live [`mbd_core::LinkPose`] in
    /// `poses`, skipping detached stages and transforming each local wrench back
    /// to the vehicle root origin (`reference_point_body_m == DVec3::ZERO`).
    pub fn evaluate_with_poses(
        &self,
        flow: &AeroFlowState,
        poses: &PoseTable,
    ) -> Result<AeroWrench, MbdError> {
        flow.validate()?;
        let mut total_force = DVec3::ZERO;
        let mut total_moment_origin = DVec3::ZERO;
        let mut max_heat_flux = 0.0_f64;

        for item in &self.elements {
            let pose = poses.get(item.component)?;
            if !pose.attached {
                continue;
            }
            let rot_inv = pose.rotation_root_from_link.conjugate();
            let v_link_root = flow.velocity_air_body_m_s
                + flow.angular_rate_body_rad_s.cross(pose.position_root_m)
                + pose.linear_vel_root_m_s;
            let w_link_root = flow.angular_rate_body_rad_s + pose.angular_vel_root_rad_s;
            let local_flow = AeroFlowState {
                velocity_air_body_m_s: rot_inv * v_link_root,
                angular_rate_body_rad_s: rot_inv * w_link_root,
                ambient: flow.ambient,
                control_surfaces_rad: flow.control_surfaces_rad,
            };
            let local_w = item.element.evaluate(&local_flow)?;
            let force_root = pose.rotation_root_from_link * local_w.force_body_n;
            let moment_root = pose.rotation_root_from_link * local_w.moment_ref_body_nm;
            let ref_root = pose.position_root_m
                + pose.rotation_root_from_link * local_w.reference_point_body_m;
            total_force += force_root;
            total_moment_origin += moment_root + ref_root.cross(force_root);
            max_heat_flux = max_heat_flux.max(local_w.stagnation_heat_flux_w_m2);
        }

        Ok(AeroWrench {
            reference_point_body_m: DVec3::ZERO,
            force_body_n: total_force,
            moment_ref_body_nm: total_moment_origin,
            stagnation_heat_flux_w_m2: max_heat_flux,
        })
    }
}

impl AeroModel for CompositeAeroModel {
    fn evaluate(&self, flow: &AeroFlowState) -> Result<AeroWrench, MbdError> {
        flow.validate()?;
        let mut total_force = DVec3::ZERO;
        let mut total_moment_origin = DVec3::ZERO;
        let mut max_heat_flux = 0.0_f64;
        for item in &self.elements {
            let w = item.element.evaluate(flow)?;
            total_force += w.force_body_n;
            total_moment_origin += w.moment_about_cg(DVec3::ZERO);
            max_heat_flux = max_heat_flux.max(w.stagnation_heat_flux_w_m2);
        }
        Ok(AeroWrench {
            reference_point_body_m: DVec3::ZERO,
            force_body_n: total_force,
            moment_ref_body_nm: total_moment_origin,
            stagnation_heat_flux_w_m2: max_heat_flux,
        })
    }
}
