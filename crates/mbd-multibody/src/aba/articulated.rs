//! Featherstone's three-pass articulated-body recursion over a link chain.

use alloc::vec::Vec;

use glam::{DMat3, DVec3};

use super::JointAccel;
use super::chain::{LinkData, LinkFrame};
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

/// Scratch of the three passes, sized to the chain and reused across solves.
#[derive(Debug, Clone, Default)]
pub(crate) struct AbaBuffers {
    /// Velocity-product bias per link.
    pub(crate) c: Vec<SpatialMotion>,
    /// Articulated inertia per link.
    pub(crate) ia: Vec<SpatialMat6>,
    /// Articulated bias force per link.
    pub(crate) pa: Vec<SpatialForce>,
    solves: Vec<JointSolve>,
    a: Vec<SpatialMotion>,
}

impl AbaBuffers {
    /// Clears every buffer to `n` zeroed links; allocates only when the
    /// chain is longer than any solved before.
    pub(crate) fn reset(&mut self, n: usize) {
        self.c.clear();
        self.c.resize(n, SpatialMotion::ZERO);
        self.ia.clear();
        self.ia.resize(n, SpatialMat6::ZERO);
        self.pa.clear();
        self.pa.resize(n, SpatialForce::ZERO);
        self.solves.clear();
        self.solves.resize(n, JointSolve::Rigid);
        self.a.clear();
        self.a.resize(n, SpatialMotion::ZERO);
    }
}

/// Solves the chain for the base acceleration, appending one entry per
/// moving joint to `joint_accels`.
pub(crate) fn solve_aba(
    links: &[LinkData],
    frames: &[LinkFrame],
    gravity_world: DVec3,
    buf: &mut AbaBuffers,
    joint_accels: &mut Vec<JointAccel>,
) -> Result<SpatialMotion, MultibodyError> {
    buf.reset(links.len());
    aba_pass1_kinematics(links, frames, gravity_world, buf);
    aba_pass2_inward(links, &mut buf.ia, &mut buf.pa, &buf.c, &mut buf.solves)?;
    let i0 = buf
        .ia
        .first()
        .copied()
        .ok_or(MultibodyError::SingularMassMatrix)?;
    let p0 = buf.pa.first().copied().unwrap_or(SpatialForce::ZERO);
    let a0 = i0.solve(-p0).ok_or(MultibodyError::SingularMassMatrix)?;
    aba_pass3_outward(links, &buf.c, &buf.solves, a0, &mut buf.a, joint_accels);
    Ok(a0)
}

fn aba_pass1_kinematics(
    links: &[LinkData],
    frames: &[LinkFrame],
    gravity_world: DVec3,
    buf: &mut AbaBuffers,
) {
    for (idx, (link, frame)) in links.iter().zip(frames).enumerate() {
        let c_i = if link.parent_order.is_some() {
            frame.v.cross_motion(link.v_j_child)
        } else {
            SpatialMotion::ZERO
        };
        assign_slot(&mut buf.c, idx, c_i);
        assign_slot(&mut buf.ia, idx, link.i_inertial);
        let g_local = frame.r_world.transpose() * gravity_world;
        let f_grav = link
            .i_local
            .mul_motion(SpatialMotion::new(DVec3::ZERO, g_local));
        let bias = frame.v.cross_force(link.i_inertial.mul_motion(frame.v));
        assign_slot(&mut buf.pa, idx, bias - link.ext_force_local - f_grav);
    }
}

fn aba_pass3_outward(
    links: &[LinkData],
    c: &[SpatialMotion],
    solves: &[JointSolve],
    a0: SpatialMotion,
    a: &mut [SpatialMotion],
    accels: &mut Vec<JointAccel>,
) {
    assign_slot(a, 0, a0);
    for (idx, link) in links.iter().enumerate().skip(1) {
        let a_parent = link
            .parent_order
            .and_then(|p| a.get(p).copied())
            .unwrap_or(SpatialMotion::ZERO);
        let c_i = c.get(idx).copied().unwrap_or(SpatialMotion::ZERO);
        let a_prime = link.x_parent_from_child.apply_motion_to_child(a_parent) + c_i;
        let solve = solves.get(idx).copied().unwrap_or(JointSolve::Rigid);
        let (a_i, joint) = outward_joint(link, solve, a_prime);
        assign_slot(a, idx, a_i);
        accels.extend(joint);
    }
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

pub(crate) fn assign_slot<T>(slice: &mut [T], idx: usize, val: T) {
    if let Some(slot) = slice.get_mut(idx) {
        *slot = val;
    }
}
