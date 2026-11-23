//! Sink into which force elements deposit everything they apply to a model
//! during one derivative evaluation.

use glam::DVec3;
use mbd_core::{ComponentId, MbdError, WrenchAtSite};

use crate::MultibodyError;
use crate::aba::chain::LinkLoads;
use crate::aba::{JointDriveInput, JointLoadInput};
use crate::spatial::{SpatialForce, SpatialMat6};

/// Loads of one derivative evaluation. Everything is summed in the order it
/// is added, so a fixed element order gives fixed bits.
#[derive(Debug)]
pub struct Loads<'a> {
    pub(crate) order_of: &'a [Option<usize>],
    pub(crate) links: &'a mut [LinkLoads],
    pub(crate) gravity: &'a mut Option<DVec3>,
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
    /// of entrained fluid to the link's acceleration. It enters the
    /// articulated inertia and the velocity-product bias but carries no
    /// weight.
    pub fn add_inertia(
        &mut self,
        component: ComponentId,
        inertia: SpatialMat6,
    ) -> Result<(), MultibodyError> {
        if let Some(slot) = self.slot(component)? {
            slot.added_inertia = Some(slot.added_inertia.map_or(inertia, |i| i + inertia));
        }
        Ok(())
    }
}
