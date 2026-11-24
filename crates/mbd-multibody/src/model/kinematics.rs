//! Read-only pose and motion of every link at one state, handed to force
//! elements.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::{ComponentId, JointKind, JointState, LinkPose, MbdError};

use crate::MultibodyError;
use crate::aba::FloatingBaseState;
use crate::aba::chain::{LinkFrame, PreparedLink};

/// World pose and velocity of a point fixed to a link.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointKinematics {
    /// Point position in world axes, m.
    pub position_world_m: DVec3,
    /// Inertial velocity of the point in world axes, m/s.
    pub velocity_world_m_s: DVec3,
    /// Inertial angular velocity of the link in world axes, rad/s.
    pub angular_vel_world_rad_s: DVec3,
    /// World-from-link rotation.
    pub rotation_world_from_link: DMat3,
}

impl PointKinematics {
    /// A world-axes vector expressed in the link's axes.
    pub fn to_link(&self, world: DVec3) -> DVec3 {
        self.rotation_world_from_link.transpose() * world
    }

    /// World-from-link rotation as a quaternion.
    pub fn rotation_quat(&self) -> DQuat {
        DQuat::from_mat3(&self.rotation_world_from_link).normalize()
    }
}

/// Kinematics of a model at the state a derivative is being evaluated at.
#[derive(Debug, Clone, Copy)]
pub struct Kinematics<'a> {
    pub(crate) links: &'a [PreparedLink],
    pub(crate) order_of: &'a [Option<usize>],
    pub(crate) frames: &'a [LinkFrame],
    pub(crate) base: FloatingBaseState,
}

impl Kinematics<'_> {
    fn index(&self, id: ComponentId) -> Result<usize, MultibodyError> {
        let order = self
            .order_of
            .get(id.0 as usize)
            .ok_or(MbdError::UnknownComponent { id })?;
        order.ok_or(MultibodyError::DetachedComponent { id })
    }

    fn frame(&self, id: ComponentId) -> Result<&LinkFrame, MultibodyError> {
        self.frames
            .get(self.index(id)?)
            .ok_or(MultibodyError::DetachedComponent { id })
    }

    /// Kind of the joint connecting `component` to its parent.
    pub fn joint_kind(&self, component: ComponentId) -> Result<JointKind, MultibodyError> {
        self.links
            .get(self.index(component)?)
            .map(|l| l.joint.kind)
            .ok_or(MultibodyError::DetachedComponent { id: component })
    }

    /// Mass of `component` with everything riding on it, kg, and its centre
    /// of mass in the component's axes, m.
    pub fn link_mass(&self, component: ComponentId) -> Result<(f64, DVec3), MultibodyError> {
        self.links
            .get(self.index(component)?)
            .map(|l| (l.mass_kg, l.cg_local_m))
            .ok_or(MultibodyError::DetachedComponent { id: component })
    }

    /// Pose and twist of the floating base.
    pub const fn base(&self) -> &FloatingBaseState {
        &self.base
    }

    /// Whether `component` is part of the attached chain.
    pub fn is_attached(&self, component: ComponentId) -> bool {
        self.frame(component).is_ok()
    }

    /// Joint coordinate and rate of `component`.
    pub fn joint(&self, component: ComponentId) -> Result<JointState, MultibodyError> {
        Ok(self.frame(component)?.joint)
    }

    /// World kinematics of the point at `offset_local_m` on `component`.
    pub fn point(
        &self,
        component: ComponentId,
        offset_local_m: DVec3,
    ) -> Result<PointKinematics, MultibodyError> {
        let f = self.frame(component)?;
        Ok(PointKinematics {
            position_world_m: f.p_world + f.r_world * offset_local_m,
            velocity_world_m_s: self.base.linear_vel_world_m_s
                + f.r_world * (f.v.lin + f.v.ang.cross(offset_local_m)),
            angular_vel_world_rad_s: f.r_world * f.v.ang,
            rotation_world_from_link: f.r_world,
        })
    }

    /// Pose of `component` in the root frame with its velocity relative to
    /// the root, the quantities a [`mbd_core::PoseTable`] row holds.
    pub fn link_pose_in_root(&self, component: ComponentId) -> Result<LinkPose, MultibodyError> {
        let f = self.frame(component)?;
        let r0t = DMat3::from_quat(self.base.rotation_world_from_root).transpose();
        let position_root_m = r0t * f.p_from_root;
        let w0 = self.base.angular_vel_root_rad_s;
        Ok(LinkPose {
            id: component,
            position_root_m,
            rotation_root_from_link: DQuat::from_mat3(&(r0t * f.r_world)).normalize(),
            linear_vel_root_m_s: r0t * (f.r_world * f.v.lin) - w0.cross(position_root_m),
            angular_vel_root_rad_s: r0t * (f.r_world * f.v.ang) - w0,
            attached: true,
        })
    }
}
