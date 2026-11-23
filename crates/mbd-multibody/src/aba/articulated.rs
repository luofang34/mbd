//! Featherstone's three-pass articulated-body recursion over a link chain
//! with at least one moving joint.

use alloc::vec;
use alloc::vec::Vec;

use glam::{DMat3, DVec3};

use super::chain::LinkData;
use super::{FloatingBaseState, JointAccel, MultiBodyAccel};
use crate::MultibodyError;
use crate::spatial::ldlt::{PIVOT_RATIO_MIN, SpatialLdlt};
use crate::spatial::{SpatialForce, SpatialMat6, SpatialMotion};

mod inward;

use inward::aba_pass2_inward;

/// Outcome of projecting one joint out of its articulated inertia, decided
/// once in the inward pass and consumed unchanged by the outward pass.
#[derive(Debug, Clone, Copy)]
enum JointSolve {
    /// Zero-DoF attachment: the link moves with its parent.
    Rigid,
    /// The subtree carried by the joint has exactly zero articulated inertia
    /// and the joint carries exactly zero net effort. Its acceleration is
    /// indeterminate and nothing depends on it: the joint transmits no wrench
    /// beyond the subtree's bias, and its rates are left unchanged (zero
    /// derivative). Any nonzero effort on such a joint is a singular-joint
    /// error instead, since it would demand infinite acceleration.
    Inert,
    /// 1-DoF joint: `D = Sᵀ Iᴬ S`, `U = Iᴬ S`, `u = τ − Sᵀ pᴬ`.
    Scalar { d: f64, u_vec: SpatialForce, u: f64 },
    /// Spherical joint: `D = Iᴬ_aa`, `Uᵀ = [Iᴬ_aa  Iᴬ_al]`, `u = τ − pᴬ_torque`.
    Ball {
        aa_inv: DMat3,
        aa: DMat3,
        al: DMat3,
        u: DVec3,
    },
    /// 6-DoF joint: the link's absolute acceleration is `Iᴬ⁻¹ (τ − pᴬ)`.
    Free {
        factor: SpatialLdlt,
        rhs: SpatialForce,
    },
}

/// Velocity-product bias, articulated inertia and articulated bias per link.
struct ArticulatedState {
    c: Vec<SpatialMotion>,
    ia: Vec<SpatialMat6>,
    pa: Vec<SpatialForce>,
}

pub(super) fn solve_aba(
    links: &[LinkData],
    base_state: &FloatingBaseState,
    gravity_world: DVec3,
) -> Result<MultiBodyAccel, MultibodyError> {
    let mut st = aba_pass1_kinematics(links, base_state, gravity_world);
    let solves = aba_pass2_inward(links, &mut st.ia, &mut st.pa, &st.c)?;
    let i0 = st
        .ia
        .first()
        .copied()
        .ok_or(MultibodyError::SingularMassMatrix)?;
    let p0 = st.pa.first().copied().unwrap_or(SpatialForce::ZERO);
    let a0 = i0.solve(-p0).ok_or(MultibodyError::SingularMassMatrix)?;
    let joint_accels = aba_pass3_outward(links, &st.c, &solves, a0);
    Ok(MultiBodyAccel {
        base_spatial_accel_root: a0,
        joint_accels,
        used_rigid_fast_path: false,
    })
}

fn aba_pass1_kinematics(
    links: &[LinkData],
    base_state: &FloatingBaseState,
    gravity_world: DVec3,
) -> ArticulatedState {
    let n = links.len();
    let mut r_w = vec![DMat3::IDENTITY; n];
    let mut v = vec![SpatialMotion::ZERO; n];
    let mut st = ArticulatedState {
        c: vec![SpatialMotion::ZERO; n],
        ia: vec![SpatialMat6::ZERO; n],
        pa: vec![SpatialForce::ZERO; n],
    };
    let r0 = DMat3::from_quat(base_state.rotation_world_from_root);
    let v0 = base_state.spatial_velocity_root();
    for (idx, link) in links.iter().enumerate() {
        let (r_i, v_i, c_i) = if let Some(p_idx) = link.parent_order {
            let rp = r_w.get(p_idx).copied().unwrap_or(DMat3::IDENTITY);
            let vp = v.get(p_idx).copied().unwrap_or(SpatialMotion::ZERO);
            let ri = rp * link.x_parent_from_child.r_parent_from_child;
            let vi = link.x_parent_from_child.apply_motion_to_child(vp) + link.v_j_child;
            (ri, vi, vi.cross_motion(link.v_j_child))
        } else {
            (r0, v0, SpatialMotion::ZERO)
        };
        assign_slot(&mut r_w, idx, r_i);
        assign_slot(&mut v, idx, v_i);
        assign_slot(&mut st.c, idx, c_i);
        assign_slot(&mut st.ia, idx, link.i_local);
        let g_local = r_i.transpose() * gravity_world;
        let f_grav = link
            .i_local
            .mul_motion(SpatialMotion::new(DVec3::ZERO, g_local));
        let bias = v_i.cross_force(link.i_local.mul_motion(v_i));
        assign_slot(&mut st.pa, idx, bias - link.ext_force_local - f_grav);
    }
    st
}

fn aba_pass3_outward(
    links: &[LinkData],
    c: &[SpatialMotion],
    solves: &[JointSolve],
    a0: SpatialMotion,
) -> Vec<JointAccel> {
    let mut a = vec![SpatialMotion::ZERO; links.len()];
    assign_slot(&mut a, 0, a0);
    let mut accels = Vec::new();
    for (idx, link) in links.iter().enumerate().skip(1) {
        let a_parent = link
            .parent_order
            .and_then(|p| a.get(p).copied())
            .unwrap_or(SpatialMotion::ZERO);
        let c_i = c.get(idx).copied().unwrap_or(SpatialMotion::ZERO);
        let a_prime = link.x_parent_from_child.apply_motion_to_child(a_parent) + c_i;
        let solve = solves.get(idx).copied().unwrap_or(JointSolve::Rigid);
        let (a_i, joint) = outward_joint(link, solve, a_prime);
        assign_slot(&mut a, idx, a_i);
        accels.extend(joint);
    }
    accels
}

fn outward_joint(
    link: &LinkData,
    solve: JointSolve,
    a_prime: SpatialMotion,
) -> (SpatialMotion, Option<JointAccel>) {
    let report = |q_accel_scalar, linear_accel_m_s2, angular_accel_rad_s2| JointAccel {
        component: link.id,
        q_accel_scalar,
        linear_accel_m_s2,
        angular_accel_rad_s2,
    };
    match solve {
        JointSolve::Rigid => (a_prime, None),
        JointSolve::Inert => (a_prime, Some(report(0.0, DVec3::ZERO, DVec3::ZERO))),
        JointSolve::Scalar { d, u_vec, u } => {
            let qdd = (u - u_vec.dot(a_prime)) / d;
            let joint = report(qdd, DVec3::ZERO, DVec3::ZERO);
            (a_prime + link.s_1dof * qdd, Some(joint))
        }
        JointSolve::Ball { aa_inv, aa, al, u } => {
            // Relative acceleration D⁻¹(u − Uᵀa′), in child axes.
            let w_dd = aa_inv * (u - aa * a_prime.ang - al * a_prime.lin);
            let joint = report(0.0, DVec3::ZERO, link.q_j_rot * w_dd);
            (a_prime + SpatialMotion::new(w_dd, DVec3::ZERO), Some(joint))
        }
        JointSolve::Free { factor, rhs } => {
            let a_i = factor.solve(rhs);
            let a_rel = a_i - a_prime;
            let lin_rot = link.v_j_child.ang.cross(link.v_j_child.lin);
            let joint = report(
                0.0,
                link.q_j_rot * (a_rel.lin + lin_rot),
                link.q_j_rot * a_rel.ang,
            );
            (a_i, Some(joint))
        }
    }
}

/// Singularity scale for a 1-DoF joint: the trace of the articulated block
/// sharing the axis's units (kg m² for a hinge, kg for a slide).
fn scalar_reference(ia: &SpatialMat6, s: SpatialMotion) -> f64 {
    let tr = |m: DMat3| m.x_axis.x + m.y_axis.y + m.z_axis.z;
    s.ang.length_squared() * tr(ia.aa) + s.lin.length_squared() * tr(ia.ll)
}

fn scalar_is_regular(d: f64, reference: f64) -> bool {
    d.is_finite() && reference.is_finite() && d > PIVOT_RATIO_MIN * reference
}

pub(super) fn assign_slot<T>(slice: &mut [T], idx: usize, val: T) {
    if let Some(slot) = slice.get_mut(idx) {
        *slot = val;
    }
}
