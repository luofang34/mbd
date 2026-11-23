use alloc::vec;
use alloc::vec::Vec;

use glam::{DMat3, DVec3};
use mbd_core::{ComponentTree, DynamicMassContribution, JointKind, WrenchAtSite};

use super::chain::{LinkData, build_link_chain, validate_inputs};
use super::{FloatingBaseState, JointAccel, JointDriveInput, MultiBodyAccel, SystemMomentum};
use crate::MultibodyError;
use crate::spatial::{SpatialForce, SpatialMat6, SpatialMotion};

/// Floating-base Featherstone Articulated-Body Algorithm (ABA) with automatic
/// zero-DoF [`RigidFastPath`](MultiBodyAccel::used_rigid_fast_path) dispatch.
pub struct MultiBodyDynamics;

impl MultiBodyDynamics {
    /// Solves exact forward dynamics $\ddot{\mathbf{q}}$ and floating-base
    /// acceleration for `tree` at `base_state`.
    pub fn forward_dynamics(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
        base_state: &FloatingBaseState,
        gravity_world_m_s2: DVec3,
        joint_drives: &[JointDriveInput],
        external_wrenches: &[WrenchAtSite],
    ) -> Result<MultiBodyAccel, MultibodyError> {
        validate_inputs(base_state, gravity_world_m_s2, dynamic_masses)?;
        let links = build_link_chain(tree, dynamic_masses, joint_drives, external_wrenches)?;
        if links.iter().all(|l| matches!(l.kind, JointKind::Fixed)) {
            return solve_rigid_fast_path(&links, base_state, gravity_world_m_s2);
        }
        solve_aba(&links, base_state, gravity_world_m_s2)
    }

    /// Computes total world mass, CoM, linear momentum, and angular momentum
    /// about the instantaneous system CoM across all attached links.
    pub fn system_momentum(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
        base_state: &FloatingBaseState,
    ) -> Result<SystemMomentum, MultibodyError> {
        validate_inputs(base_state, DVec3::ZERO, dynamic_masses)?;
        let links = build_link_chain(tree, dynamic_masses, &[], &[])?;
        let n = links.len();
        let mut r_world_list = vec![DMat3::IDENTITY; n];
        let mut p_world_list = vec![DVec3::ZERO; n];
        let mut v_local_list = vec![SpatialMotion::ZERO; n];

        let r0 = DMat3::from_quat(base_state.rotation_world_from_root);
        let p0 = base_state.position_world_m;
        let v0 = base_state.spatial_velocity_root();

        for idx in 0..n {
            let Some(link) = links.get(idx) else {
                break;
            };
            if let Some(p_idx) = link.parent_order {
                let (Some(&rp), Some(&pp), Some(&vp)) = (
                    r_world_list.get(p_idx),
                    p_world_list.get(p_idx),
                    v_local_list.get(p_idx),
                ) else {
                    break;
                };
                let r_i = rp * link.x_parent_from_child.r_parent_from_child;
                let p_i = pp + rp * link.x_parent_from_child.p_child_in_parent;
                let v_i = link.x_parent_from_child.apply_motion_to_child(vp) + link.v_j_child;
                assign_slot(&mut r_world_list, idx, r_i);
                assign_slot(&mut p_world_list, idx, p_i);
                assign_slot(&mut v_local_list, idx, v_i);
            } else {
                assign_slot(&mut r_world_list, idx, r0);
                assign_slot(&mut p_world_list, idx, p0);
                assign_slot(&mut v_local_list, idx, v0);
            }
        }
        accumulate_world_momentum(&links, &r_world_list, &p_world_list, &v_local_list)
    }

    /// Computes total rigid-body spatial inertia in root axes assuming
    /// joints are momentarily frozen, used by mechanisms and Generalized Jacobians.
    pub fn locked_root_spatial_inertia(
        tree: &ComponentTree,
        dynamic_masses: &[DynamicMassContribution],
    ) -> Result<SpatialMat6, MultibodyError> {
        let links = build_link_chain(tree, dynamic_masses, &[], &[])?;
        let mut total = vec![SpatialMat6::ZERO; links.len()];
        for idx in 0..links.len() {
            if let (Some(slot), Some(link)) = (total.get_mut(idx), links.get(idx)) {
                *slot = link.i_local;
            }
        }
        for idx in (1..links.len()).rev() {
            let (Some(child), Some(&child_i)) = (links.get(idx), total.get(idx)) else {
                continue;
            };
            let Some(p_idx) = child.parent_order else {
                continue;
            };
            let shifted = child_i.transform_to_parent(&child.x_parent_from_child);
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

fn accumulate_world_momentum(
    links: &[LinkData],
    r_world: &[DMat3],
    p_world: &[DVec3],
    v_local: &[SpatialMotion],
) -> Result<SystemMomentum, MultibodyError> {
    let mut total_mass = 0.0;
    let mut first_moment = DVec3::ZERO;
    for idx in 0..links.len() {
        let (Some(link), Some(&r), Some(&p)) = (links.get(idx), r_world.get(idx), p_world.get(idx))
        else {
            continue;
        };
        let cm_world = p + r * link.cg_local_m;
        total_mass += link.mass_kg;
        first_moment += cm_world * link.mass_kg;
    }
    if total_mass <= 0.0 {
        return Err(MultibodyError::SingularMassMatrix);
    }
    let com_world = first_moment / total_mass;
    let mut lin_mom = DVec3::ZERO;
    let mut ang_mom = DVec3::ZERO;
    for idx in 0..links.len() {
        let (Some(link), Some(&r), Some(&p), Some(&v)) = (
            links.get(idx),
            r_world.get(idx),
            p_world.get(idx),
            v_local.get(idx),
        ) else {
            continue;
        };
        let cm_world = p + r * link.cg_local_m;
        let w_world = r * v.ang;
        let v_cm_world = r * (v.lin + v.ang.cross(link.cg_local_m));
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

fn solve_rigid_fast_path(
    links: &[LinkData],
    base_state: &FloatingBaseState,
    gravity_world: DVec3,
) -> Result<MultiBodyAccel, MultibodyError> {
    let n = links.len();
    let mut inertias = vec![SpatialMat6::ZERO; n];
    let mut wrenches = vec![SpatialForce::ZERO; n];
    let r0 = DMat3::from_quat(base_state.rotation_world_from_root);
    let g_root = r0.transpose() * gravity_world;

    for idx in 0..n {
        if let Some(link) = links.get(idx) {
            assign_slot(&mut inertias, idx, link.i_local);
            assign_slot(&mut wrenches, idx, link.ext_force_local);
        }
    }
    for idx in (1..n).rev() {
        let (Some(link), Some(&i_c), Some(&f_c)) =
            (links.get(idx), inertias.get(idx), wrenches.get(idx))
        else {
            continue;
        };
        let Some(p_idx) = link.parent_order else {
            continue;
        };
        let i_shift = i_c.transform_to_parent(&link.x_parent_from_child);
        let f_shift = link.x_parent_from_child.apply_force_to_parent(f_c);
        if let Some(i_p) = inertias.get_mut(p_idx) {
            *i_p += i_shift;
        }
        if let Some(f_p) = wrenches.get_mut(p_idx) {
            *f_p += f_shift;
        }
    }

    let i_total = inertias
        .first()
        .copied()
        .ok_or(MultibodyError::SingularMassMatrix)?;
    let f_ext = wrenches.first().copied().unwrap_or(SpatialForce::ZERO);
    let v0 = base_state.spatial_velocity_root();
    let f_grav = i_total.mul_motion(SpatialMotion::new(DVec3::ZERO, g_root));
    let bias = v0.cross_force(i_total.mul_motion(v0));
    let accel = i_total
        .solve(f_ext + f_grav - bias)
        .ok_or(MultibodyError::SingularMassMatrix)?;

    Ok(MultiBodyAccel {
        base_spatial_accel_root: accel,
        joint_accels: Vec::new(),
        used_rigid_fast_path: true,
    })
}

fn solve_aba(
    links: &[LinkData],
    base_state: &FloatingBaseState,
    gravity_world: DVec3,
) -> Result<MultiBodyAccel, MultibodyError> {
    let n = links.len();
    let mut v = vec![SpatialMotion::ZERO; n];
    let mut c = vec![SpatialMotion::ZERO; n];
    let mut r_w = vec![DMat3::IDENTITY; n];
    let mut ia = vec![SpatialMat6::ZERO; n];
    let mut pa = vec![SpatialForce::ZERO; n];

    aba_pass1_kinematics(
        links,
        base_state,
        gravity_world,
        &mut r_w,
        &mut v,
        &mut c,
        &mut ia,
        &mut pa,
    );

    let mut d_1dof = vec![1.0_f64; n];
    let mut u_1dof = vec![SpatialForce::ZERO; n];
    let mut u_sc = vec![0.0_f64; n];
    let mut aa_3d_inv = vec![DMat3::IDENTITY; n];
    let mut u_3d = vec![DVec3::ZERO; n];

    aba_pass2_inward(
        links,
        &mut ia,
        &mut pa,
        &c,
        &mut d_1dof,
        &mut u_1dof,
        &mut u_sc,
        &mut aa_3d_inv,
        &mut u_3d,
    );

    let i0 = ia
        .first()
        .copied()
        .ok_or(MultibodyError::SingularMassMatrix)?;
    let p0 = pa.first().copied().unwrap_or(SpatialForce::ZERO);
    let a0 = i0.solve(-p0).ok_or(MultibodyError::SingularMassMatrix)?;

    let mut a = vec![SpatialMotion::ZERO; n];
    assign_slot(&mut a, 0, a0);
    let joint_accels = aba_pass3_outward(
        links, &c, &pa, &d_1dof, &u_1dof, &u_sc, &aa_3d_inv, &u_3d, &mut a,
    );

    Ok(MultiBodyAccel {
        base_spatial_accel_root: a0,
        joint_accels,
        used_rigid_fast_path: false,
    })
}

#[allow(clippy::too_many_arguments)]
fn aba_pass1_kinematics(
    links: &[LinkData],
    base_state: &FloatingBaseState,
    gravity_world: DVec3,
    r_w: &mut [DMat3],
    v: &mut [SpatialMotion],
    c: &mut [SpatialMotion],
    ia: &mut [SpatialMat6],
    pa: &mut [SpatialForce],
) {
    let r0 = DMat3::from_quat(base_state.rotation_world_from_root);
    let v0 = base_state.spatial_velocity_root();
    assign_slot(r_w, 0, r0);
    assign_slot(v, 0, v0);
    for idx in 0..links.len() {
        let Some(link) = links.get(idx) else {
            continue;
        };
        let (r_i, v_i, c_i) = if let Some(p_idx) = link.parent_order {
            let rp = r_w.get(p_idx).copied().unwrap_or(DMat3::IDENTITY);
            let vp = v.get(p_idx).copied().unwrap_or(SpatialMotion::ZERO);
            let ri = rp * link.x_parent_from_child.r_parent_from_child;
            let vi = link.x_parent_from_child.apply_motion_to_child(vp) + link.v_j_child;
            let ci = vi.cross_motion(link.v_j_child);
            (ri, vi, ci)
        } else {
            (r0, v0, SpatialMotion::ZERO)
        };
        assign_slot(r_w, idx, r_i);
        assign_slot(v, idx, v_i);
        assign_slot(c, idx, c_i);
        assign_slot(ia, idx, link.i_local);
        let g_local = r_i.transpose() * gravity_world;
        let f_grav = link
            .i_local
            .mul_motion(SpatialMotion::new(DVec3::ZERO, g_local));
        let bias = v_i.cross_force(link.i_local.mul_motion(v_i));
        assign_slot(pa, idx, bias - link.ext_force_local - f_grav);
    }
}

#[allow(clippy::too_many_arguments)]
fn aba_pass2_inward(
    links: &[LinkData],
    ia: &mut [SpatialMat6],
    pa: &mut [SpatialForce],
    c: &[SpatialMotion],
    d_1dof: &mut [f64],
    u_1dof: &mut [SpatialForce],
    u_sc: &mut [f64],
    aa_3d_inv: &mut [DMat3],
    u_3d: &mut [DVec3],
) {
    for idx in (1..links.len()).rev() {
        let (Some(link), Some(&ia_i), Some(&pa_i), Some(&c_i)) =
            (links.get(idx), ia.get(idx), pa.get(idx), c.get(idx))
        else {
            continue;
        };
        let Some(p_idx) = link.parent_order else {
            continue;
        };
        let (i_small, p_small) = match link.kind {
            JointKind::Fixed => (ia_i, pa_i),
            JointKind::Revolute { .. } | JointKind::Prismatic { .. } => {
                let u_vec = ia_i.mul_motion(link.s_1dof);
                let d_val = u_vec.dot(link.s_1dof);
                let d_inv = if d_val.abs() > 1e-12 {
                    1.0 / d_val
                } else {
                    0.0
                };
                let u_val = link.tau_1dof - pa_i.dot(link.s_1dof);
                assign_slot(d_1dof, idx, d_val);
                assign_slot(u_1dof, idx, u_vec);
                assign_slot(u_sc, idx, u_val);
                let i_a = ia_i - SpatialMat6::outer_force(u_vec, u_vec) * d_inv;
                let p_a = pa_i + i_a.mul_motion(c_i) + u_vec * (u_val * d_inv);
                (i_a, p_a)
            }
            JointKind::Spherical { .. } => {
                let aa_inv = if ia_i.aa.determinant().abs() > 1e-18 {
                    ia_i.aa.inverse()
                } else {
                    DMat3::ZERO
                };
                let u_v = link.tau_3d_child - pa_i.torque;
                assign_slot(aa_3d_inv, idx, aa_inv);
                assign_slot(u_3d, idx, u_v);
                let i_a = SpatialMat6 {
                    aa: DMat3::ZERO,
                    al: DMat3::ZERO,
                    la: DMat3::ZERO,
                    ll: ia_i.ll - ia_i.la * aa_inv * ia_i.al,
                };
                let p_a = SpatialForce {
                    torque: link.tau_3d_child,
                    force: pa_i.force + ia_i.la * (aa_inv * u_v),
                } + i_a.mul_motion(c_i);
                (i_a, p_a)
            }
            JointKind::Compliant6Dof { .. } => {
                let tau_6 = SpatialForce::new(link.tau_3d_child, link.force_3d_child);
                (SpatialMat6::ZERO, tau_6)
            }
        };
        let i_shift = i_small.transform_to_parent(&link.x_parent_from_child);
        let p_shift = link.x_parent_from_child.apply_force_to_parent(p_small);
        if let Some(ip) = ia.get_mut(p_idx) {
            *ip += i_shift;
        }
        if let Some(pp) = pa.get_mut(p_idx) {
            *pp += p_shift;
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn aba_pass3_outward(
    links: &[LinkData],
    c: &[SpatialMotion],
    pa: &[SpatialForce],
    d_1dof: &[f64],
    u_1dof: &[SpatialForce],
    u_sc: &[f64],
    aa_3d_inv: &[DMat3],
    u_3d: &[DVec3],
    a: &mut [SpatialMotion],
) -> Vec<JointAccel> {
    let mut accels = Vec::new();
    for idx in 1..links.len() {
        let Some(link) = links.get(idx) else {
            continue;
        };
        let a_parent = link
            .parent_order
            .and_then(|p| a.get(p).copied())
            .unwrap_or(SpatialMotion::ZERO);
        let c_i = c.get(idx).copied().unwrap_or(SpatialMotion::ZERO);
        let a_prime = link.x_parent_from_child.apply_motion_to_child(a_parent) + c_i;
        match link.kind {
            JointKind::Fixed => assign_slot(a, idx, a_prime),
            JointKind::Revolute { .. } | JointKind::Prismatic { .. } => {
                let d_val = d_1dof.get(idx).copied().unwrap_or(0.0);
                let u_vec = u_1dof.get(idx).copied().unwrap_or(SpatialForce::ZERO);
                let u_val = u_sc.get(idx).copied().unwrap_or(0.0);
                let qdd = if d_val.abs() > 1e-12 {
                    (u_val - u_vec.dot(a_prime)) / d_val
                } else {
                    0.0
                };
                assign_slot(a, idx, a_prime + link.s_1dof * qdd);
                accels.push(JointAccel {
                    component: link.id,
                    q_accel_scalar: qdd,
                    linear_accel_m_s2: DVec3::ZERO,
                    angular_accel_rad_s2: DVec3::ZERO,
                });
            }
            JointKind::Spherical { .. } => {
                let aa_inv = aa_3d_inv.get(idx).copied().unwrap_or(DMat3::ZERO);
                let u_v = u_3d.get(idx).copied().unwrap_or(DVec3::ZERO);
                let w_dd_child = aa_inv * (u_v - link.i_local.al * a_prime.lin);
                assign_slot(
                    a,
                    idx,
                    a_prime + SpatialMotion::new(w_dd_child, DVec3::ZERO),
                );
                accels.push(JointAccel {
                    component: link.id,
                    q_accel_scalar: 0.0,
                    linear_accel_m_s2: DVec3::ZERO,
                    angular_accel_rad_s2: link.q_j_rot * w_dd_child,
                });
            }
            JointKind::Compliant6Dof { .. } => {
                let pa_i = pa.get(idx).copied().unwrap_or(SpatialForce::ZERO);
                let tau_6 = SpatialForce::new(link.tau_3d_child, link.force_3d_child);
                let a_i = link
                    .i_local
                    .solve(tau_6 - pa_i)
                    .unwrap_or(SpatialMotion::ZERO);
                let a_rel_child = a_i - a_prime;
                assign_slot(a, idx, a_i);
                let lin_rot = link.v_j_child.ang.cross(link.v_j_child.lin);
                accels.push(JointAccel {
                    component: link.id,
                    q_accel_scalar: 0.0,
                    linear_accel_m_s2: link.q_j_rot * (a_rel_child.lin + lin_rot),
                    angular_accel_rad_s2: link.q_j_rot * a_rel_child.ang,
                });
            }
        }
    }
    accels
}

fn assign_slot<T>(slice: &mut [T], idx: usize, val: T) {
    if let Some(slot) = slice.get_mut(idx) {
        *slot = val;
    }
}
