//! Linear operators of the equations of motion at a configuration: the
//! inverse mass matrix and the Jacobian of a body-fixed point.
//!
//! Generalized vectors here use the layout of the state's rate block:
//! `[base linear 3 | base angular 3 | joint entries ...]`, the base entries
//! in root axes and the joint entries in each joint's placed frame. A
//! generalized force is the dual: its product with a generalized velocity is
//! power.
//!
//! These are what a constraint solver needs, and a force element reaches
//! them through the [`Resolution`] handed to
//! [`crate::ForceElement::resolve`]. For constraint rows at points `p_k`
//! the Delassus operator `A = J M⁻¹ Jᵀ` is built one column per row:
//! `Jᵀ e_k` from the transposed Jacobian, `M⁻¹ Jᵀ e_k` in `O(n)` from the
//! articulated-body pass, and `J` applied to that for every row. The
//! right-hand side is the acceleration of the points under the free
//! generalized acceleration, `J v̇_free + J̇ v`, so the multipliers follow
//! from `A λ = −(J v̇_free + J̇ v)` and are applied as ordinary point forces.

use glam::DVec3;
use mbd_core::{ComponentId, JointKind, JointState};

use super::layout::{read_joint, read1, read3, write1, write3};
use super::state::base_of;
use super::{MultibodyModel, Workspace};
use crate::MultibodyError;
use crate::aba::FloatingBaseState;
use crate::aba::chain::{EffortTerms, link_frame};
use crate::integrate::check_dim;
use crate::spatial::{SpatialForce, SpatialMat6, SpatialMotion};

/// Joint state with the positions of `joint` and the rates read from the
/// generalized vector `v`, or zero rates without one.
fn with_rates(kind: &JointKind, joint: JointState, at: usize, v: Option<&[f64]>) -> JointState {
    let mut out = JointState {
        q_vel_scalar: 0.0,
        linear_vel_m_s: DVec3::ZERO,
        angular_vel_rad_s: DVec3::ZERO,
        ..joint
    };
    let Some(v) = v else {
        return out;
    };
    match kind {
        JointKind::Fixed => {}
        JointKind::Revolute { .. } | JointKind::Prismatic { .. } => {
            out.q_vel_scalar = read1(v, at);
        }
        JointKind::Spherical { .. } => out.angular_vel_rad_s = read3(v, at),
        JointKind::Compliant6Dof { .. } => {
            out.linear_vel_m_s = read3(v, at);
            out.angular_vel_rad_s = read3(v, at + 3);
        }
    }
    out
}

impl MultibodyModel {
    /// Frames at the configuration of `x` moving with the generalized
    /// velocity `v`, or at rest without one.
    fn frames_with(
        &self,
        x: &[f64],
        v: Option<&[f64]>,
        work: &mut Workspace,
    ) -> Result<FloatingBaseState, MultibodyError> {
        self.check_model_block(x)?;
        let nq = self.layout.position_len();
        let base = FloatingBaseState {
            linear_vel_root_m_s: v.map_or(DVec3::ZERO, |v| read3(v, 0)),
            angular_vel_root_rad_s: v.map_or(DVec3::ZERO, |v| read3(v, 3)),
            ..base_of(x, nq)
        };
        work.frames.clear();
        for link in &self.links {
            let at = link.slots.v.saturating_sub(nq);
            let kind = &link.joint.kind;
            let joint = with_rates(kind, read_joint(kind, link.slots, x), at, v);
            let parent = link.parent_order.and_then(|p| work.frames.get(p));
            let frame = link_frame(link, joint, parent, &base)?;
            work.frames.push(frame);
        }
        Ok(base)
    }

    /// Applies the inverse mass matrix at the configuration of the flat
    /// state `x`: writes `M⁻¹ τ` for the generalized force `τ`.
    ///
    /// This is the articulated-body pass with every state-dependent term
    /// removed: zero velocity, no gravity, no springs, dampers, stops or
    /// actuator limits. It costs one `O(n)` solve and never forms `M`. `M`
    /// here is the mass matrix of the links alone; inertia added by
    /// elements during an evaluation is included by
    /// [`Resolution::apply_inverse_mass`].
    pub fn apply_inverse_mass(
        &self,
        x: &[f64],
        generalized_force: &[f64],
        work: &mut Workspace,
        generalized_accel: &mut [f64],
    ) -> Result<(), MultibodyError> {
        self.inverse_mass(x, generalized_force, &[], work, generalized_accel)
    }

    fn inverse_mass(
        &self,
        x: &[f64],
        generalized_force: &[f64],
        added: &[Option<SpatialMat6>],
        work: &mut Workspace,
        generalized_accel: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let nv = self.layout.velocity_len();
        check_dim(nv, generalized_force.len())?;
        check_dim(nv, generalized_accel.len())?;
        let nq = self.layout.position_len();
        let base = self.frames_with(x, None, work)?;
        self.clear_loads(work);
        for (loads, inertia) in work.link_loads.iter_mut().zip(added) {
            loads.added_inertia = *inertia;
        }
        let tau = generalized_force;
        for (link, loads) in self.links.iter().zip(&mut work.link_loads) {
            let at = link.slots.v.saturating_sub(nq);
            match link.joint.kind {
                JointKind::Fixed => {}
                JointKind::Revolute { .. } | JointKind::Prismatic { .. } => {
                    loads.load.get_or_insert_default().scalar = read1(tau, at);
                }
                JointKind::Spherical { .. } => {
                    loads.load.get_or_insert_default().torque = read3(tau, at);
                }
                JointKind::Compliant6Dof { .. } => {
                    let load = loads.load.get_or_insert_default();
                    load.force = read3(tau, at);
                    load.torque = read3(tau, at + 3);
                }
            }
        }
        if let Some(root) = work.link_loads.first_mut() {
            root.wrench = SpatialForce::new(read3(tau, 3), read3(tau, 0));
        }
        self.solve(&base, work, EffortTerms::AppliedOnly)?;
        self.write_generalized_accel(work, generalized_accel);
        Ok(())
    }

    /// Writes the solved accelerations in `work` as a generalized vector.
    fn write_generalized_accel(&self, work: &Workspace, out: &mut [f64]) {
        let nq = self.layout.position_len();
        write3(out, 0, work.base_accel.lin);
        write3(out, 3, work.base_accel.ang);
        let mut accels = work.joint_accels.iter();
        for link in &self.links {
            let at = link.slots.v.saturating_sub(nq);
            if matches!(link.joint.kind, JointKind::Fixed) {
                continue;
            }
            let Some(a) = accels.next() else {
                break;
            };
            match link.joint.kind {
                JointKind::Fixed => {}
                JointKind::Revolute { .. } | JointKind::Prismatic { .. } => {
                    write1(out, at, a.q_accel_scalar);
                }
                JointKind::Spherical { .. } => write3(out, at, a.angular_accel_rad_s2),
                JointKind::Compliant6Dof { .. } => {
                    write3(out, at, a.linear_accel_m_s2);
                    write3(out, at + 3, a.angular_accel_rad_s2);
                }
            }
        }
    }

    /// Jacobian action `J v`: world velocity of the point at
    /// `offset_local_m` on `component` when the model, at the configuration
    /// of the flat state `x`, moves with the generalized velocity `v`.
    pub fn point_velocity(
        &self,
        x: &[f64],
        generalized_velocity: &[f64],
        component: ComponentId,
        offset_local_m: DVec3,
        work: &mut Workspace,
    ) -> Result<DVec3, MultibodyError> {
        check_dim(self.layout.velocity_len(), generalized_velocity.len())?;
        let idx = self.chain_index(component)?;
        self.frames_with(x, Some(generalized_velocity), work)?;
        let f = work
            .frames
            .get(idx)
            .ok_or(MultibodyError::DetachedComponent { id: component })?;
        Ok(f.r_world * (f.v.lin + f.v.ang.cross(offset_local_m)))
    }

    /// Transposed Jacobian action `Jᵀ f`: the generalized force equivalent
    /// to the world-axes force `force_world_n` applied at `offset_local_m`
    /// on `component`, at the configuration of the flat state `x`.
    pub fn point_force_to_generalized(
        &self,
        x: &[f64],
        component: ComponentId,
        offset_local_m: DVec3,
        force_world_n: DVec3,
        work: &mut Workspace,
        generalized_force: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let nv = self.layout.velocity_len();
        check_dim(nv, generalized_force.len())?;
        let nq = self.layout.position_len();
        let mut cursor = Some(self.chain_index(component)?);
        self.frames_with(x, None, work)?;
        generalized_force.fill(0.0);
        let mut wrench = SpatialForce::ZERO;
        let mut first = true;
        while let Some(idx) = cursor {
            let (Some(link), Some(frame)) = (self.links.get(idx), work.frames.get(idx)) else {
                break;
            };
            if first {
                let f_link = frame.r_world.transpose() * force_world_n;
                wrench = SpatialForce::new(offset_local_m.cross(f_link), f_link);
                first = false;
            }
            let at = link.slots.v.saturating_sub(nq);
            let out = &mut *generalized_force;
            match link.joint.kind {
                JointKind::Fixed => {}
                JointKind::Revolute { axis, .. } => write1(out, at, wrench.torque.dot(axis)),
                JointKind::Prismatic { axis, .. } => write1(out, at, wrench.force.dot(axis)),
                JointKind::Spherical { .. } => write3(out, at, frame.q_j_rot * wrench.torque),
                JointKind::Compliant6Dof { .. } => {
                    write3(out, at, frame.q_j_rot * wrench.force);
                    write3(out, at + 3, frame.q_j_rot * wrench.torque);
                }
            }
            if link.parent_order.is_none() {
                write3(out, 0, wrench.force);
                write3(out, 3, wrench.torque);
            } else {
                wrench = frame.x_parent_from_child.apply_force_to_parent(wrench);
            }
            cursor = link.parent_order;
        }
        Ok(())
    }

    /// World acceleration `J v̇ + J̇ v` of the point at `offset_local_m` on
    /// `component` when the model, at the flat state `x` (configuration and
    /// rates), has the generalized acceleration `v̇`.
    pub fn point_acceleration(
        &self,
        x: &[f64],
        generalized_accel: &[f64],
        component: ComponentId,
        offset_local_m: DVec3,
        work: &mut Workspace,
    ) -> Result<DVec3, MultibodyError> {
        let (nq, vdot) = (self.layout.position_len(), generalized_accel);
        check_dim(self.layout.velocity_len(), vdot.len())?;
        let target = self.chain_index(component)?;
        self.update_frames(x, work)?;
        let accel = &mut work.aba.c;
        accel.clear();
        accel.resize(self.links.len(), SpatialMotion::ZERO);
        for (idx, (link, f)) in self.links.iter().zip(&work.frames).enumerate() {
            let at = link.slots.v.saturating_sub(nq);
            let inv = f.q_j_rot.conjugate();
            let joint = match link.joint.kind {
                JointKind::Fixed => SpatialMotion::ZERO,
                JointKind::Revolute { axis, .. } => {
                    SpatialMotion::new(axis * read1(vdot, at), DVec3::ZERO)
                }
                JointKind::Prismatic { axis, .. } => {
                    SpatialMotion::new(DVec3::ZERO, axis * read1(vdot, at))
                }
                JointKind::Spherical { .. } => {
                    SpatialMotion::new(inv * read3(vdot, at), DVec3::ZERO)
                }
                // The joint's linear rate is kept in the placed frame, so
                // its child-frame derivative carries a rotation term.
                JointKind::Compliant6Dof { .. } => SpatialMotion::new(
                    inv * read3(vdot, at + 3),
                    inv * read3(vdot, at) - f.v_j_child.ang.cross(f.v_j_child.lin),
                ),
            };
            let a = match link.parent_order.and_then(|p| accel.get(p).copied()) {
                Some(parent) => {
                    f.x_parent_from_child.apply_motion_to_child(parent)
                        + f.v.cross_motion(f.v_j_child)
                        + joint
                }
                None => SpatialMotion::new(read3(vdot, 3), read3(vdot, 0)),
            };
            if idx == target {
                let p = offset_local_m;
                let centripetal = f.v.ang.cross(f.v.lin + f.v.ang.cross(p));
                return Ok(f.r_world * (a.lin + a.ang.cross(p) + centripetal));
            }
            if let Some(slot) = accel.get_mut(idx) {
                *slot = a;
            }
        }
        Err(MultibodyError::DetachedComponent { id: component })
    }

    fn chain_index(&self, id: ComponentId) -> Result<usize, MultibodyError> {
        let order = self
            .order_of
            .get(id.0 as usize)
            .ok_or(mbd_core::MbdError::UnknownComponent { id })?;
        order.ok_or(MultibodyError::DetachedComponent { id })
    }
}

/// What a resolving element works with: the free acceleration of the
/// evaluation in progress and the operators of the equations of motion at
/// its state, with the inertia other elements have added included.
#[derive(Debug)]
pub struct Resolution<'a> {
    pub(crate) model: &'a MultibodyModel,
    pub(crate) x: &'a [f64],
    pub(crate) free_accel: &'a [f64],
    pub(crate) added: &'a [Option<SpatialMat6>],
    pub(crate) scratch: &'a mut Workspace,
}

impl Resolution<'_> {
    /// Generalized acceleration under every load accumulated before the
    /// resolution phase.
    pub const fn free_acceleration(&self) -> &[f64] {
        self.free_accel
    }

    /// `M⁻¹ τ` at the evaluated configuration; see
    /// [`MultibodyModel::apply_inverse_mass`].
    pub fn apply_inverse_mass(
        &mut self,
        generalized_force: &[f64],
        generalized_accel: &mut [f64],
    ) -> Result<(), MultibodyError> {
        self.model.inverse_mass(
            self.x,
            generalized_force,
            self.added,
            self.scratch,
            generalized_accel,
        )
    }

    /// `J v` for a body-fixed point; see [`MultibodyModel::point_velocity`].
    pub fn point_velocity(
        &mut self,
        generalized_velocity: &[f64],
        component: ComponentId,
        offset_local_m: DVec3,
    ) -> Result<DVec3, MultibodyError> {
        self.model.point_velocity(
            self.x,
            generalized_velocity,
            component,
            offset_local_m,
            self.scratch,
        )
    }

    /// `Jᵀ f` for a force at a body-fixed point; see
    /// [`MultibodyModel::point_force_to_generalized`].
    pub fn point_force_to_generalized(
        &mut self,
        component: ComponentId,
        offset_local_m: DVec3,
        force_world_n: DVec3,
        generalized_force: &mut [f64],
    ) -> Result<(), MultibodyError> {
        self.model.point_force_to_generalized(
            self.x,
            component,
            offset_local_m,
            force_world_n,
            self.scratch,
            generalized_force,
        )
    }

    /// `J v̇ + J̇ v` for a body-fixed point at the evaluated state; see
    /// [`MultibodyModel::point_acceleration`].
    pub fn point_acceleration(
        &mut self,
        generalized_accel: &[f64],
        component: ComponentId,
        offset_local_m: DVec3,
    ) -> Result<DVec3, MultibodyError> {
        self.model.point_acceleration(
            self.x,
            generalized_accel,
            component,
            offset_local_m,
            self.scratch,
        )
    }
}
