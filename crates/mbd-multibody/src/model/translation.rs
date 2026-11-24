//! Centre-of-mass translation coordinates: the form in which a model's
//! state is integrated.
//!
//! The stored state holds the root origin's position and velocity, which
//! mean the same whatever the mass distribution. Integrated directly they
//! tie translation to rotation: the origin of a tumbling body circles the
//! centre of mass, and the truncation of that circling accumulates in the
//! trajectory. The centre of mass itself obeys `r̈ = F / m`, with no term in
//! the attitude or the joints, so an integrator advancing it keeps the
//! orbit of a tumbling or articulating vehicle as exactly as a point
//! mass's. [`crate::CentreOfMassOde`] therefore presents the translation blocks of
//! the flat state as the centre of mass's position and velocity, and
//! [`MultibodyModel::advance`] converts on the way in and out.

use glam::DVec3;

use super::layout::{BASE_POSITION, write3};
use super::momentum::centre_of_mass;
use super::{MultibodyModel, Workspace};
use crate::MultibodyError;
use crate::aba::FloatingBaseState;

impl MultibodyModel {
    /// Rewrites the translation blocks of the flat state `x` from the root
    /// origin's position and velocity to the centre of mass's: the
    /// coordinates [`crate::CentreOfMassOde`] integrates.
    pub fn root_to_centre_of_mass(
        &self,
        x: &mut [f64],
        work: &mut Workspace,
    ) -> Result<(), MultibodyError> {
        let base = self.update_frames(x, work)?;
        let com = centre_of_mass(
            &self.links,
            &work.frames,
            base.position_world_m,
            base.linear_vel_world_m_s,
        )?;
        write3(x, BASE_POSITION, com.position_world_m);
        write3(x, self.layout.position_len(), com.velocity_world_m_s);
        Ok(())
    }

    /// Inverse of [`MultibodyModel::root_to_centre_of_mass`]. Leaves the
    /// frames of the rewritten state in `work` and returns its base state.
    pub fn centre_of_mass_to_root(
        &self,
        x: &mut [f64],
        work: &mut Workspace,
    ) -> Result<FloatingBaseState, MultibodyError> {
        // Offsets from the root origin do not depend on where it is, so
        // the frames built with the centre of mass standing in for it give
        // them, and only the link positions need moving afterwards.
        let held = self.update_frames(x, work)?;
        let com = centre_of_mass(&self.links, &work.frames, DVec3::ZERO, DVec3::ZERO)?;
        let base = FloatingBaseState {
            position_world_m: held.position_world_m - com.offset_from_root_world_m,
            linear_vel_world_m_s: held.linear_vel_world_m_s - com.velocity_from_root_world_m_s,
            ..held
        };
        write3(x, BASE_POSITION, base.position_world_m);
        write3(x, self.layout.position_len(), base.linear_vel_world_m_s);
        for f in &mut work.frames {
            f.p_world = base.position_world_m + f.p_from_root;
        }
        Ok(base)
    }

    /// Inertial acceleration of the centre of mass in world axes from the
    /// frames and the latest solve in `work`.
    pub(crate) fn centre_of_mass_accel(&self, work: &Workspace) -> Result<DVec3, MultibodyError> {
        let Some(root) = work.frames.first() else {
            return Err(MultibodyError::SingularMassMatrix);
        };
        if work.used_rigid_fast_path {
            // One rigid body: the centre of mass is a body-fixed point.
            let com = centre_of_mass(&self.links, &work.frames, DVec3::ZERO, DVec3::ZERO)?;
            let (w, alpha) = (
                root.r_world * root.v.ang,
                root.r_world * work.base_accel.ang,
            );
            let arm = com.offset_from_root_world_m;
            return Ok(work.base_linear_accel_world + alpha.cross(arm) + w.cross(w.cross(arm)));
        }
        let (mut mass, mut sum) = (0.0, DVec3::ZERO);
        let parts = self.links.iter().zip(&work.frames).zip(&work.aba.a);
        for ((link, f), a) in parts {
            let c = link.cg_local_m;
            let accel = a.lin + a.ang.cross(c) + f.v.ang.cross(f.v.lin + f.v.ang.cross(c));
            mass += link.mass_kg;
            sum += f.r_world * accel * link.mass_kg;
        }
        if mass <= 0.0 {
            return Err(MultibodyError::SingularMassMatrix);
        }
        Ok(sum / mass)
    }
}
