//! Sink into which force elements deposit everything they apply to a model
//! during one derivative evaluation.

use glam::DVec3;
use mbd_core::{ComponentId, MbdError, WrenchAtSite};

use crate::MultibodyError;
use crate::aba::chain::{LinkFrame, LinkLoads};
use crate::aba::{JointDriveInput, JointLoadInput};
use crate::spatial::{SpatialForce, SpatialMat6, SpatialMotion};

/// Motion of the medium an added inertia belongs to: the air or water a
/// link entrains, taken as a rigid whole in steady motion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MediumMotion {
    /// Inertial velocity of the medium at the link origin, world axes, m/s.
    pub velocity_world_m_s: DVec3,
    /// Inertial angular velocity of the medium, world axes, rad/s.
    pub angular_velocity_world_rad_s: DVec3,
}

impl MediumMotion {
    /// A medium at rest in the world.
    pub const AT_REST: Self = Self {
        velocity_world_m_s: DVec3::ZERO,
        angular_velocity_world_rad_s: DVec3::ZERO,
    };
}

/// Velocity-product force of `added` on the link at `frame`, in the frame
/// moving with the root origin at `base_velocity`.
///
/// Entrained fluid carries the impulse `I v` of the link's motion relative
/// to the medium, `v`, and reacts with the inertial rate of that impulse.
/// The solve supplies `I a` for its spatial acceleration `a`; what is left
/// is formed here: the transport of the impulse by the link's own turning
/// and by its origin's motion through the medium, and `I` times the part
/// of `v̇` that `a` does not hold, which comes from the root's velocity
/// and the medium's turning.
fn added_bias(
    added: &SpatialMat6,
    frame: &LinkFrame,
    base_velocity: DVec3,
    medium: &MediumMotion,
) -> SpatialForce {
    let to_link = frame.r_world.transpose();
    let spin = to_link * medium.angular_velocity_world_rad_s;
    let through = to_link * (base_velocity - medium.velocity_world_m_s);
    let origin_velocity = base_velocity + frame.r_world * frame.v.lin;
    let relative = SpatialMotion::new(frame.v.ang - spin, frame.v.lin + through);
    let transport = SpatialMotion::new(frame.v.ang, relative.lin);
    let rate = SpatialMotion::new(
        frame.v.ang.cross(spin),
        -frame.v.ang.cross(through)
            - to_link * medium.angular_velocity_world_rad_s.cross(origin_velocity),
    );
    transport.cross_force(added.mul_motion(relative)) + added.mul_motion(rate)
}

/// Loads of one derivative evaluation. Everything is summed in the order it
/// is added, so a fixed element order gives fixed bits.
#[derive(Debug)]
pub struct Loads<'a> {
    pub(crate) order_of: &'a [Option<usize>],
    pub(crate) links: &'a mut [LinkLoads],
    pub(crate) gravity: &'a mut Option<DVec3>,
    pub(crate) frames: &'a [LinkFrame],
    /// Inertial velocity of the root origin, which the frame velocities
    /// are relative to.
    pub(crate) base_velocity: DVec3,
}

impl Loads<'_> {
    fn slot(&mut self, id: ComponentId) -> Result<Option<&mut LinkLoads>, MultibodyError> {
        let order = self
            .order_of
            .get(id.0 as usize)
            .ok_or(MbdError::UnknownComponent { id })?;
        Ok(order.and_then(|idx| self.links.get_mut(idx)))
    }

    /// Adds a uniform field acceleration in world axes, m/s². It acts on
    /// every link's own mass and never on added inertia.
    pub fn add_gravity(&mut self, accel_world_m_s2: DVec3) {
        // The first contribution is stored as given: adding it to zero would
        // turn a negative zero component positive.
        *self.gravity = Some(
            self.gravity
                .map_or(accel_world_m_s2, |g| g + accel_world_m_s2),
        );
    }

    /// Adds an external wrench at a component site, in that component's
    /// axes. A wrench on a detached component is dropped.
    pub fn add_wrench(&mut self, wrench: &WrenchAtSite) -> Result<(), MultibodyError> {
        if let Some(slot) = self.slot(wrench.site)? {
            slot.wrench += SpatialForce::new(
                wrench.torque_local_nm + wrench.point_local_m.cross(wrench.force_local_n),
                wrench.force_local_n,
            );
        }
        Ok(())
    }

    /// Adds a force at `point_local_m` of `component`, given in that
    /// component's axes.
    pub fn add_force(
        &mut self,
        component: ComponentId,
        point_local_m: DVec3,
        force_local_n: DVec3,
    ) -> Result<(), MultibodyError> {
        self.add_wrench(&WrenchAtSite {
            site: component,
            point_local_m,
            force_local_n,
            torque_local_nm: DVec3::ZERO,
        })
    }

    /// Adds an actuator command. Commands on one joint are summed and the
    /// scalar sum is limited by the joint's actuator.
    pub fn add_joint_drive(&mut self, drive: &JointDriveInput) -> Result<(), MultibodyError> {
        if let Some(slot) = self.slot(drive.component)? {
            slot.drive.scalar += drive.effort_scalar;
            slot.drive.torque += drive.torque_3d_nm;
            slot.drive.force += drive.force_3d_n;
        }
        Ok(())
    }

    /// Adds a joint-space load that no actuator limit applies to.
    pub fn add_joint_load(&mut self, load: &JointLoadInput) -> Result<(), MultibodyError> {
        if let Some(slot) = self.slot(load.component)? {
            let sum = slot.load.get_or_insert_default();
            sum.scalar += load.effort_scalar;
            sum.torque += load.torque_3d_nm;
            sum.force += load.force_3d_n;
        }
        Ok(())
    }

    /// Adds inertia to a link about its origin, in link axes: the reaction
    /// of fluid entrained from `medium` to the link's motion relative to
    /// it. It enters the articulated inertia and carries no weight. Its
    /// velocity-product force follows the link's velocity relative to the
    /// medium, so a vehicle and its medium moving together at any speed
    /// behave as they do at rest, and a link turning with a turning medium
    /// feels nothing from it.
    pub fn add_inertia(
        &mut self,
        component: ComponentId,
        inertia: SpatialMat6,
        medium: &MediumMotion,
    ) -> Result<(), MultibodyError> {
        let order = self
            .order_of
            .get(component.0 as usize)
            .ok_or(MbdError::UnknownComponent { id: component })?;
        let Some(idx) = *order else {
            return Ok(());
        };
        if let (Some(slot), Some(frame)) = (self.links.get_mut(idx), self.frames.get(idx)) {
            slot.added_bias += added_bias(&inertia, frame, self.base_velocity, medium);
            slot.added_inertia = Some(slot.added_inertia.map_or(inertia, |i| i + inertia));
        }
        Ok(())
    }
}
