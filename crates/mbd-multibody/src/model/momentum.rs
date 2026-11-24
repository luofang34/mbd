//! Mass, centre of mass and momenta of a model at one state.
//!
//! Everything about the centre of mass is summed from positions and
//! velocities relative to the root origin, and the root's own position and
//! velocity are added once at the end. Moments about the centre of mass
//! then never difference two large numbers, so they hold their precision
//! far from the world origin and at orbital speed.

use glam::DVec3;

use super::{Kinematics, ModelState, MultibodyModel, Workspace};
use crate::MultibodyError;
use crate::aba::SystemMomentum;
use crate::aba::chain::{LinkFrame, PreparedLink};

/// Mass and motion of a model's centre of mass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CentreOfMass {
    /// Total attached mass, kg.
    pub mass_kg: f64,
    /// Centre of mass in world axes, m.
    pub position_world_m: DVec3,
    /// Centre of mass relative to the root origin in world axes, m.
    pub offset_from_root_world_m: DVec3,
    /// Inertial velocity of the centre of mass in world axes, m/s.
    pub velocity_world_m_s: DVec3,
    /// Velocity of the centre of mass relative to the root origin's, in
    /// world axes, m/s.
    pub velocity_from_root_world_m_s: DVec3,
}

/// Centre of mass of one link relative to the root origin, and its velocity
/// relative to the root origin's, world axes.
fn link_centre(link: &PreparedLink, f: &LinkFrame) -> (DVec3, DVec3) {
    (
        f.p_from_root + f.r_world * link.cg_local_m,
        f.r_world * (f.v.lin + f.v.ang.cross(link.cg_local_m)),
    )
}

pub(super) fn centre_of_mass(
    links: &[PreparedLink],
    frames: &[LinkFrame],
    root_position: DVec3,
    root_velocity: DVec3,
) -> Result<CentreOfMass, MultibodyError> {
    let (mut mass, mut first, mut momentum) = (0.0, DVec3::ZERO, DVec3::ZERO);
    for (link, f) in links.iter().zip(frames) {
        let (at, vel) = link_centre(link, f);
        mass += link.mass_kg;
        first += at * link.mass_kg;
        momentum += vel * link.mass_kg;
    }
    if mass <= 0.0 {
        return Err(MultibodyError::SingularMassMatrix);
    }
    let (offset, relative) = (first / mass, momentum / mass);
    Ok(CentreOfMass {
        mass_kg: mass,
        position_world_m: root_position + offset,
        offset_from_root_world_m: offset,
        velocity_world_m_s: root_velocity + relative,
        velocity_from_root_world_m_s: relative,
    })
}

impl Kinematics<'_> {
    /// Mass and motion of the system's centre of mass, for an element whose
    /// load acts there (a central gravity field, a gravity-gradient
    /// torque). One pass over the links.
    pub fn centre_of_mass(&self) -> Result<CentreOfMass, MultibodyError> {
        centre_of_mass(
            self.links,
            self.frames,
            self.base.position_world_m,
            self.base.linear_vel_world_m_s,
        )
    }
}

impl MultibodyModel {
    /// Total mass, centre of mass and world momenta of the model at `state`.
    pub fn system_momentum(
        &self,
        state: &ModelState,
        work: &mut Workspace,
    ) -> Result<SystemMomentum, MultibodyError> {
        let base = self.update_frames(state.as_slice(), work)?;
        let com = centre_of_mass(
            &self.links,
            &work.frames,
            base.position_world_m,
            base.linear_vel_world_m_s,
        )?;
        let mut angular = DVec3::ZERO;
        for (link, f) in self.links.iter().zip(&work.frames) {
            let (at, vel) = link_centre(link, f);
            let r = f.r_world;
            let spin = r * (link.inertia_cg_local * f.v.ang);
            let arm = at - com.offset_from_root_world_m;
            let relative = vel - com.velocity_from_root_world_m_s;
            angular += spin + arm.cross(relative * link.mass_kg);
        }
        Ok(SystemMomentum {
            total_mass_kg: com.mass_kg,
            com_world_m: com.position_world_m,
            linear_world_kg_m_s: com.velocity_world_m_s * com.mass_kg,
            angular_about_com_world_nms: angular,
        })
    }
}
