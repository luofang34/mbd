//! Equations of motion of a prepared model as a pure function of state.

use glam::{DMat3, DQuat, DVec3};
use mbd_core::JointKind;

use super::layout::{BASE_ATTITUDE, BASE_POSITION, read_joint, write_quat, write1, write3};
use super::state::base_of;
use super::{Kinematics, Loads, ModelState, MultibodyModel, Resolution, Workspace};
use crate::MultibodyError;
use crate::aba::articulated::solve_aba;
use crate::aba::chain::{EffortTerms, LinkLoads, assemble_link, link_frame, validate_base};
use crate::aba::rigid::solve_rigid;
use crate::aba::{FloatingBaseState, SystemMomentum};
use crate::element::{ElementContext, ForceElement};
use crate::integrate::check_dim;
use crate::spatial::SpatialMat6;

/// The arguments of one derivative evaluation that every phase shares.
struct Pass<'a, I: ?Sized, E: ?Sized> {
    time_s: f64,
    model_block: &'a [f64],
    aux: &'a [f64],
    modes: &'a [f64],
    elements: &'a E,
    inputs: &'a I,
    base: FloatingBaseState,
}

impl<'a, I: ?Sized, E: ?Sized> Pass<'a, I, E> {
    fn context(
        &self,
        model: &'a MultibodyModel,
        kinematics: &'a Kinematics<'a>,
    ) -> ElementContext<'a, I> {
        ElementContext {
            time_s: self.time_s,
            model,
            state: self.model_block,
            kinematics,
            inputs: self.inputs,
        }
    }
}

/// `½ ω ⊗ q` for a rate given in the frame the rotation maps from
/// (`body = true`, `½ q ⊗ ω`) or the frame it maps to.
fn quat_rate(q: DQuat, w: DVec3, body: bool) -> DQuat {
    let w_q = DQuat::from_xyzw(w.x, w.y, w.z, 0.0);
    if body {
        (q * w_q) * 0.5
    } else {
        (w_q * q) * 0.5
    }
}

impl MultibodyModel {
    /// Fills the workspace's link frames from the flat state `x`, whose
    /// rates are taken from `x` itself.
    pub(crate) fn update_frames(
        &self,
        x: &[f64],
        work: &mut Workspace,
    ) -> Result<FloatingBaseState, MultibodyError> {
        self.check_model_block(x)?;
        let base = base_of(x, self.layout.position_len());
        validate_base(&base)?;
        work.frames.clear();
        for link in &self.links {
            let joint = read_joint(&link.joint.kind, link.slots, x);
            let parent = link.parent_order.and_then(|p| work.frames.get(p));
            let frame = link_frame(link, joint, parent, &base)?;
            work.frames.push(frame);
        }
        Ok(base)
    }

    /// Rejects a flat state too short to hold the model block.
    pub(crate) fn check_model_block(&self, x: &[f64]) -> Result<(), MultibodyError> {
        if x.len() >= self.layout.len() {
            return Ok(());
        }
        Err(MultibodyError::StateLengthMismatch {
            expected: self.layout.len(),
            actual: x.len(),
        })
    }

    pub(crate) fn clear_loads(&self, work: &mut Workspace) {
        work.link_loads.clear();
        work.link_loads
            .resize(self.links.len(), LinkLoads::default());
        work.gravity = None;
    }

    /// Link kinematics at `state`, for a host that needs world poses of
    /// attachment points between steps.
    pub fn kinematics<'a>(
        &'a self,
        state: &ModelState,
        work: &'a mut Workspace,
    ) -> Result<Kinematics<'a>, MultibodyError> {
        let base = self.update_frames(state.as_slice(), work)?;
        Ok(Kinematics {
            links: &self.links,
            order_of: &self.order_of,
            frames: &work.frames,
            base,
        })
    }

    /// Evaluates `ẋ = f(t, x, modes, inputs)` for the model loaded by
    /// `elements`.
    ///
    /// The result is a pure function of the arguments: `work` is scratch and
    /// nothing else is written. `x` is the flat continuous state (model
    /// block then element block) with unit quaternions; `x_dot` receives the
    /// rate of every component in the same layout, quaternion blocks as the
    /// embedded rate `q̇ = ½ q ⊗ ω`.
    #[allow(clippy::too_many_arguments)]
    pub fn derivative<I: ?Sized, E: ForceElement<I> + ?Sized>(
        &self,
        time_s: f64,
        x: &[f64],
        modes: &[f64],
        elements: &E,
        inputs: &I,
        work: &mut Workspace,
        x_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let n = self.layout.len();
        check_dim(n + elements.state_len(), x.len())?;
        check_dim(x.len(), x_dot.len())?;
        check_dim(elements.mode_len(), modes.len())?;
        let short = MultibodyError::StateLengthMismatch {
            expected: n,
            actual: x.len(),
        };
        let (model_block, aux) = x.split_at_checked(n).ok_or(short.clone())?;
        let (rates, aux_dot) = x_dot.split_at_mut_checked(n).ok_or(short)?;
        let base = self.update_frames(x, work)?;
        self.clear_loads(work);
        let pass = Pass {
            time_s,
            model_block,
            aux,
            modes,
            elements,
            inputs,
            base,
        };
        self.accumulate(&pass, work, aux_dot)?;
        if elements.has_resolution() {
            // Free motion under the loads so far, then the loads that
            // depend on it.
            self.solve(&base, work, EffortTerms::Full)?;
            self.write_rates(&base, work, rates);
            self.resolve(&pass, work, rates)?;
        }
        self.solve(&base, work, EffortTerms::Full)?;
        self.write_rates(&base, work, rates);
        Ok(())
    }

    /// First phase: every element's `evaluate` into the load buffers.
    fn accumulate<I: ?Sized, E: ForceElement<I> + ?Sized>(
        &self,
        pass: &Pass<'_, I, E>,
        work: &mut Workspace,
        aux_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        let kinematics = Kinematics {
            links: &self.links,
            order_of: &self.order_of,
            frames: &work.frames,
            base: pass.base,
        };
        let mut loads = Loads {
            order_of: &self.order_of,
            links: &mut work.link_loads,
            gravity: &mut work.gravity,
        };
        let ctx = pass.context(self, &kinematics);
        pass.elements
            .evaluate(&ctx, pass.aux, pass.modes, &mut loads, aux_dot)?;
        let gravity = work.gravity.unwrap_or(DVec3::ZERO);
        if !gravity.is_finite() {
            return Err(MultibodyError::InvalidParameter {
                field: "gravity_world_m_s2",
                value: gravity.length(),
            });
        }
        Ok(())
    }

    /// Second phase: every element's `resolve`, given the free generalized
    /// acceleration in the rate block of `rates`.
    fn resolve<I: ?Sized, E: ForceElement<I> + ?Sized>(
        &self,
        pass: &Pass<'_, I, E>,
        work: &mut Workspace,
        rates: &[f64],
    ) -> Result<(), MultibodyError> {
        work.added.clear();
        work.added
            .extend(work.link_loads.iter().map(|l| l.added_inertia));
        let kinematics = Kinematics {
            links: &self.links,
            order_of: &self.order_of,
            frames: &work.frames,
            base: pass.base,
        };
        let mut loads = Loads {
            order_of: &self.order_of,
            links: &mut work.link_loads,
            gravity: &mut work.gravity,
        };
        let mut resolution = Resolution {
            model: self,
            x: pass.model_block,
            free_accel: rates.get(self.layout.position_len()..).unwrap_or(&[]),
            added: &work.added,
            scratch: work.scratch.get_or_insert_default(),
        };
        let ctx = pass.context(self, &kinematics);
        pass.elements
            .resolve(&ctx, pass.aux, pass.modes, &mut resolution, &mut loads)
    }

    /// Solves the accelerations for the frames and loads in `work`.
    pub(crate) fn solve(
        &self,
        base: &FloatingBaseState,
        work: &mut Workspace,
        terms: EffortTerms,
    ) -> Result<(), MultibodyError> {
        work.links.clear();
        let mut added = false;
        let parts = self.links.iter().zip(&work.frames).zip(&work.link_loads);
        for ((link, frame), loads) in parts {
            added |= loads.added_inertia.is_some();
            work.links.push(assemble_link(link, frame, loads, terms));
        }
        work.joint_accels.clear();
        // The composite-body shortcut applies gravity through the summed
        // inertia, which is only right while no weightless inertia is added.
        work.used_rigid_fast_path = self.all_fixed && !added;
        let gravity = work.gravity.unwrap_or(DVec3::ZERO);
        work.base_accel = if work.used_rigid_fast_path {
            solve_rigid(&work.links, base, gravity, &mut work.aba)?
        } else {
            solve_aba(
                &work.links,
                &work.frames,
                gravity,
                &mut work.aba,
                &mut work.joint_accels,
            )?
        };
        Ok(())
    }

    /// Writes the rates of the model block from the solved accelerations.
    fn write_rates(&self, base: &FloatingBaseState, work: &Workspace, out: &mut [f64]) {
        let nq = self.layout.position_len();
        write3(out, BASE_POSITION, base.linear_vel_world_m_s());
        let q_dot = quat_rate(
            base.rotation_world_from_root,
            base.angular_vel_root_rad_s,
            true,
        );
        write_quat(out, BASE_ATTITUDE, q_dot);
        write3(out, nq, work.base_accel.lin);
        write3(out, nq + 3, work.base_accel.ang);
        let mut accels = work.joint_accels.iter();
        for (link, frame) in self.links.iter().zip(&work.frames) {
            let (j, slots) = (&frame.joint, link.slots);
            if matches!(link.joint.kind, JointKind::Fixed) {
                continue;
            }
            let Some(a) = accels.next() else {
                break;
            };
            match link.joint.kind {
                JointKind::Fixed => {}
                JointKind::Revolute { .. } | JointKind::Prismatic { .. } => {
                    write1(out, slots.q, j.q_vel_scalar);
                    write1(out, slots.v, a.q_accel_scalar);
                }
                JointKind::Spherical { .. } => {
                    // The joint rate is in the placed frame, the frame the
                    // joint rotation maps to.
                    let q_dot = quat_rate(j.rotation, j.angular_vel_rad_s, false);
                    write_quat(out, slots.q, q_dot);
                    write3(out, slots.v, a.angular_accel_rad_s2);
                }
                JointKind::Compliant6Dof { .. } => {
                    write3(out, slots.q, j.linear_vel_m_s);
                    let q_dot = quat_rate(j.rotation, j.angular_vel_rad_s, false);
                    write_quat(out, slots.q + 3, q_dot);
                    write3(out, slots.v, a.linear_accel_m_s2);
                    write3(out, slots.v + 3, a.angular_accel_rad_s2);
                }
            }
        }
    }

    /// Total mass, centre of mass and world momenta of the model at `state`.
    pub fn system_momentum(
        &self,
        state: &ModelState,
        work: &mut Workspace,
    ) -> Result<SystemMomentum, MultibodyError> {
        self.update_frames(state.as_slice(), work)?;
        let parts = || self.links.iter().zip(&work.frames);
        let mut total_mass = 0.0;
        let mut first_moment = DVec3::ZERO;
        for (link, f) in parts() {
            let cm_world = f.p_world + f.r_world * link.cg_local_m;
            total_mass += link.mass_kg;
            first_moment += cm_world * link.mass_kg;
        }
        if total_mass <= 0.0 {
            return Err(MultibodyError::SingularMassMatrix);
        }
        let com_world = first_moment / total_mass;
        let mut lin_mom = DVec3::ZERO;
        let mut ang_mom = DVec3::ZERO;
        for (link, f) in parts() {
            let r: DMat3 = f.r_world;
            let cm_world = f.p_world + r * link.cg_local_m;
            let w_world = r * f.v.ang;
            let v_cm_world = r * (f.v.lin + f.v.ang.cross(link.cg_local_m));
            let p_i = v_cm_world * link.mass_kg;
            lin_mom += p_i;
            let i_cm_world = r * link.inertia_cg_local * r.transpose();
            ang_mom += i_cm_world * w_world + (cm_world - com_world).cross(p_i);
        }
        Ok(SystemMomentum {
            total_mass_kg: total_mass,
            com_world_m: com_world,
            linear_world_kg_m_s: lin_mom,
            angular_about_com_world_nms: ang_mom,
        })
    }

    /// Spatial inertia of the whole model about the root origin, in root
    /// axes, with every joint held at its coordinate in `state`.
    pub fn locked_root_spatial_inertia(
        &self,
        state: &ModelState,
        work: &mut Workspace,
    ) -> Result<SpatialMat6, MultibodyError> {
        self.update_frames(state.as_slice(), work)?;
        let total = &mut work.aba.ia;
        total.clear();
        total.extend(self.links.iter().map(|l| l.i_local));
        for (idx, (link, frame)) in self.links.iter().zip(&work.frames).enumerate().rev() {
            let (Some(p_idx), Some(&child_i)) = (link.parent_order, total.get(idx)) else {
                continue;
            };
            let shifted = child_i.transform_to_parent(&frame.x_parent_from_child);
            if let Some(p_slot) = total.get_mut(p_idx) {
                *p_slot += shifted;
            }
        }
        total
            .first()
            .copied()
            .ok_or(MultibodyError::SingularMassMatrix)
    }
}
