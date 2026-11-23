//! Inward (tip-to-root) pass: articulated inertias, bias forces and the
//! per-joint projections the outward pass reuses.

use glam::{DMat3, DVec3};
use mbd_core::JointKind;

use super::{JointSolve, scalar_is_regular, scalar_reference};
use crate::MultibodyError;
use crate::aba::chain::LinkData;
use crate::spatial::ldlt::SymLdlt3;
use crate::spatial::{SpatialForce, SpatialMat6, SpatialMotion};

/// Articulated inertia and bias a link passes to its parent, before the
/// change of frame.
type Transmitted = (SpatialMat6, SpatialForce);

pub(super) fn aba_pass2_inward(
    links: &[LinkData],
    ia: &mut [SpatialMat6],
    pa: &mut [SpatialForce],
    c: &[SpatialMotion],
    solves: &mut [JointSolve],
) -> Result<(), MultibodyError> {
    for (idx, link) in links.iter().enumerate().skip(1).rev() {
        let (Some(&ia_i), Some(&pa_i), Some(&c_i)) = (ia.get(idx), pa.get(idx), c.get(idx)) else {
            continue;
        };
        let Some(p_idx) = link.parent_order else {
            continue;
        };
        let (solve, (i_small, p_small)) = match link.kind {
            JointKind::Fixed => (JointSolve::Rigid, (ia_i, pa_i)),
            JointKind::Revolute { .. } | JointKind::Prismatic { .. } => {
                project_scalar(link, &ia_i, pa_i, c_i)?
            }
            JointKind::Spherical { .. } => project_ball(link, &ia_i, pa_i, c_i)?,
            JointKind::Compliant6Dof { .. } => project_free(link, &ia_i, pa_i)?,
        };
        if let Some(slot) = solves.get_mut(idx) {
            *slot = solve;
        }
        let i_shift = i_small.transform_to_parent(&link.x_parent_from_child);
        let p_shift = link.x_parent_from_child.apply_force_to_parent(p_small);
        if let Some(ip) = ia.get_mut(p_idx) {
            *ip += i_shift;
        }
        if let Some(pp) = pa.get_mut(p_idx) {
            *pp += p_shift;
        }
    }
    Ok(())
}

fn singular(link: &LinkData) -> MultibodyError {
    MultibodyError::SingularJointInertia { id: link.id }
}

fn project_scalar(
    link: &LinkData,
    ia_i: &SpatialMat6,
    pa_i: SpatialForce,
    c_i: SpatialMotion,
) -> Result<(JointSolve, Transmitted), MultibodyError> {
    let u_vec = ia_i.mul_motion(link.s_1dof);
    let d = u_vec.dot(link.s_1dof);
    // The travel stop is an acceleration demand scaled by the inertia the
    // joint actually drives, so its response is the same at every scale.
    let tau = link.tau_1dof + d * link.stop_accel_1dof;
    let u = tau - pa_i.dot(link.s_1dof);
    if ia_i.is_exactly_zero() && u == 0.0 {
        return Ok((JointSolve::Inert, (SpatialMat6::ZERO, pa_i)));
    }
    if !scalar_is_regular(d, scalar_reference(ia_i, link.s_1dof)) {
        return Err(singular(link));
    }
    let i_a = *ia_i - SpatialMat6::outer_force(u_vec, u_vec) * (1.0 / d);
    let p_a = pa_i + i_a.mul_motion(c_i) + u_vec * (u / d);
    Ok((JointSolve::Scalar { d, u_vec, u }, (i_a, p_a)))
}

fn project_ball(
    link: &LinkData,
    ia_i: &SpatialMat6,
    pa_i: SpatialForce,
    c_i: SpatialMotion,
) -> Result<(JointSolve, Transmitted), MultibodyError> {
    let tau = link.tau_3d_child + ia_i.aa * link.stop_accel_child;
    let u = tau - pa_i.torque;
    if ia_i.is_exactly_zero() && u == DVec3::ZERO {
        return Ok((JointSolve::Inert, (SpatialMat6::ZERO, pa_i)));
    }
    let aa_inv = SymLdlt3::of(ia_i.aa)
        .ok_or_else(|| singular(link))?
        .inverse();
    let i_a = SpatialMat6 {
        aa: DMat3::ZERO,
        al: DMat3::ZERO,
        la: DMat3::ZERO,
        ll: ia_i.ll - ia_i.la * aa_inv * ia_i.al,
    };
    let p_a = SpatialForce {
        torque: tau,
        force: pa_i.force + ia_i.la * (aa_inv * u),
    } + i_a.mul_motion(c_i);
    let solve = JointSolve::Ball {
        aa_inv,
        aa: ia_i.aa,
        al: ia_i.al,
        u,
    };
    Ok((solve, (i_a, p_a)))
}

fn project_free(
    link: &LinkData,
    ia_i: &SpatialMat6,
    pa_i: SpatialForce,
) -> Result<(JointSolve, Transmitted), MultibodyError> {
    let tau = SpatialForce::new(link.tau_3d_child, link.force_3d_child);
    let rhs = tau - pa_i;
    if ia_i.is_exactly_zero() && rhs == SpatialForce::ZERO {
        return Ok((JointSolve::Inert, (SpatialMat6::ZERO, pa_i)));
    }
    let factor = ia_i.factor().ok_or_else(|| singular(link))?;
    // A 6-DoF joint passes no inertia to its parent, only its own effort.
    Ok((JointSolve::Free { factor, rhs }, (SpatialMat6::ZERO, tau)))
}
