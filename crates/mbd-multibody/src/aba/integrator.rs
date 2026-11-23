use alloc::vec::Vec;

use glam::{DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentState, ComponentTree, DynamicMassContribution, JointKind, JointState,
    WrenchAtSite,
};

use super::{FloatingBaseState, JointDriveInput, MultiBodyAccel, MultiBodyDynamics};
use crate::MultibodyError;

#[derive(Clone)]
struct PhaseState {
    base: FloatingBaseState,
    joint_states: Vec<JointState>,
}

#[derive(Clone)]
struct PhaseDeriv {
    pos_dot: DVec3,
    quat_dot: DQuat,
    vel_dot: DVec3,
    ang_vel_dot: DVec3,
    joint_dots: Vec<JointDot>,
}

#[derive(Clone, Copy)]
struct JointDot {
    id: ComponentId,
    q_dot: f64,
    q_ddot: f64,
    trans_dot: DVec3,
    vel_dot: DVec3,
    quat_dot: DQuat,
    omega_dot: DVec3,
}

/// Integrates the full floating-base + articulated joint state over `dt_s`
/// using classical 4th-order Runge-Kutta (`RK4`), updating `tree` in place
/// and returning the updated [`FloatingBaseState`].
pub fn step_rk4(
    tree: &mut ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    base_state: &FloatingBaseState,
    gravity_world_m_s2: DVec3,
    joint_drives: &[JointDriveInput],
    external_wrenches: &[WrenchAtSite],
    dt_s: f64,
) -> Result<FloatingBaseState, MultibodyError> {
    step_rk4_with_forces(tree, dynamic_masses, base_state, dt_s, |_, _, _| {
        Ok((
            gravity_world_m_s2,
            joint_drives.to_vec(),
            external_wrenches.to_vec(),
        ))
    })
}

/// Integrates floating-base and articulated joint states over `dt_s` via classical
/// 4th-order Runge-Kutta (`RK4`), evaluating stage-dependent gravity, drives, and
/// wrenches at `(tree, stage_base, stage_dt_s)` across `0`, `0.5 * dt_s`, `0.5 * dt_s`,
/// and `dt_s`. Restores `tree` on failure.
pub fn step_rk4_with_forces<E, F>(
    tree: &mut ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    base_state: &FloatingBaseState,
    dt_s: f64,
    mut eval_forces: F,
) -> Result<FloatingBaseState, E>
where
    E: From<MultibodyError>,
    F: FnMut(
        &ComponentTree,
        &FloatingBaseState,
        f64,
    ) -> Result<(DVec3, Vec<JointDriveInput>, Vec<WrenchAtSite>), E>,
{
    if !dt_s.is_finite() || dt_s < 0.0 {
        return Err(E::from(MultibodyError::InvalidParameter {
            field: "dt_s",
            value: dt_s,
        }));
    }
    if dt_s == 0.0 {
        return Ok(*base_state);
    }
    let y0 = snapshot_phase(tree, base_state);
    let saved_tree = tree.states().to_vec();
    match integrate_rk4_stages(tree, dynamic_masses, &y0, dt_s, &mut eval_forces) {
        Ok(base) => Ok(base),
        Err(err) => {
            tree.restore_states(&saved_tree)
                .map_err(MultibodyError::from)?;
            Err(err)
        }
    }
}

fn integrate_rk4_stages<E, F>(
    tree: &mut ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    y0: &PhaseState,
    dt_s: f64,
    eval_forces: &mut F,
) -> Result<FloatingBaseState, E>
where
    E: From<MultibodyError>,
    F: FnMut(
        &ComponentTree,
        &FloatingBaseState,
        f64,
    ) -> Result<(DVec3, Vec<JointDriveInput>, Vec<WrenchAtSite>), E>,
{
    let mut eval_stage = |y: &PhaseState, stage_dt_s: f64| -> Result<PhaseDeriv, E> {
        apply_phase(tree, y)?;
        let (gravity_world_m_s2, drives, wrenches) = eval_forces(tree, &y.base, stage_dt_s)?;
        let accel = MultiBodyDynamics::forward_dynamics(
            tree,
            dynamic_masses,
            &y.base,
            gravity_world_m_s2,
            &drives,
            &wrenches,
        )?;
        Ok(build_deriv(tree, &y.base, &accel))
    };

    let k1 = eval_stage(y0, 0.0)?;
    let k2 = eval_stage(&advance_phase(y0, &k1, 0.5 * dt_s), 0.5 * dt_s)?;
    let k3 = eval_stage(&advance_phase(y0, &k2, 0.5 * dt_s), 0.5 * dt_s)?;
    let k4 = eval_stage(&advance_phase(y0, &k3, dt_s), dt_s)?;

    let final_y = combine_rk4(y0, &k1, &k2, &k3, &k4, dt_s);
    apply_phase(tree, &final_y)?;
    Ok(final_y.base)
}

fn snapshot_phase(tree: &ComponentTree, base: &FloatingBaseState) -> PhaseState {
    PhaseState {
        base: *base,
        joint_states: tree.states().iter().map(|s| s.joint).collect(),
    }
}

fn apply_phase(tree: &mut ComponentTree, phase: &PhaseState) -> Result<(), MultibodyError> {
    let updated: Vec<ComponentState> = tree
        .states()
        .iter()
        .zip(&phase.joint_states)
        .map(|(st, j)| ComponentState { joint: *j, ..*st })
        .collect();
    tree.restore_states(&updated)?;
    Ok(())
}

fn build_deriv(
    tree: &ComponentTree,
    base: &FloatingBaseState,
    accel: &MultiBodyAccel,
) -> PhaseDeriv {
    let pos_dot = base.linear_vel_world_m_s();
    let w_q = DQuat::from_xyzw(
        base.angular_vel_root_rad_s.x,
        base.angular_vel_root_rad_s.y,
        base.angular_vel_root_rad_s.z,
        0.0,
    );
    let quat_dot = (base.rotation_world_from_root * w_q) * 0.5;
    let mut joint_dots = Vec::new();
    for ja in &accel.joint_accels {
        let Ok(st) = tree.state(ja.component) else {
            continue;
        };
        let j = st.joint;
        let qj = j.rotation;
        let wj = DQuat::from_xyzw(
            j.angular_vel_rad_s.x,
            j.angular_vel_rad_s.y,
            j.angular_vel_rad_s.z,
            0.0,
        );
        // Angular velocity is in placed frame -> left multiply quaternion derivative.
        let j_qdot = (wj * qj) * 0.5;
        joint_dots.push(JointDot {
            id: ja.component,
            q_dot: j.q_vel_scalar,
            q_ddot: ja.q_accel_scalar,
            trans_dot: j.linear_vel_m_s,
            vel_dot: ja.linear_accel_m_s2,
            quat_dot: j_qdot,
            omega_dot: ja.angular_accel_rad_s2,
        });
    }
    PhaseDeriv {
        pos_dot,
        quat_dot,
        vel_dot: accel.base_spatial_accel_root.lin,
        ang_vel_dot: accel.base_spatial_accel_root.ang,
        joint_dots,
    }
}

fn advance_phase(y0: &PhaseState, k: &PhaseDeriv, step: f64) -> PhaseState {
    let q_base = (y0.base.rotation_world_from_root + k.quat_dot * step).normalize();
    let base = FloatingBaseState {
        position_world_m: y0.base.position_world_m + k.pos_dot * step,
        rotation_world_from_root: q_base,
        linear_vel_root_m_s: y0.base.linear_vel_root_m_s + k.vel_dot * step,
        angular_vel_root_rad_s: y0.base.angular_vel_root_rad_s + k.ang_vel_dot * step,
    };
    let mut joint_states = y0.joint_states.clone();
    for jd in &k.joint_dots {
        if let Some(js) = joint_states.get_mut(jd.id.0 as usize) {
            js.q_scalar += jd.q_dot * step;
            js.q_vel_scalar += jd.q_ddot * step;
            js.translation_m += jd.trans_dot * step;
            js.linear_vel_m_s += jd.vel_dot * step;
            js.rotation = (js.rotation + jd.quat_dot * step).normalize();
            js.angular_vel_rad_s += jd.omega_dot * step;
        }
    }
    PhaseState { base, joint_states }
}

fn combine_rk4(
    y0: &PhaseState,
    k1: &PhaseDeriv,
    k2: &PhaseDeriv,
    k3: &PhaseDeriv,
    k4: &PhaseDeriv,
    dt: f64,
) -> PhaseState {
    let w = dt / 6.0;
    let q_sum = y0.base.rotation_world_from_root
        + (k1.quat_dot + k2.quat_dot * 2.0 + k3.quat_dot * 2.0 + k4.quat_dot) * w;
    let base = FloatingBaseState {
        position_world_m: y0.base.position_world_m
            + (k1.pos_dot + k2.pos_dot * 2.0 + k3.pos_dot * 2.0 + k4.pos_dot) * w,
        rotation_world_from_root: q_sum.normalize(),
        linear_vel_root_m_s: y0.base.linear_vel_root_m_s
            + (k1.vel_dot + k2.vel_dot * 2.0 + k3.vel_dot * 2.0 + k4.vel_dot) * w,
        angular_vel_root_rad_s: y0.base.angular_vel_root_rad_s
            + (k1.ang_vel_dot + k2.ang_vel_dot * 2.0 + k3.ang_vel_dot * 2.0 + k4.ang_vel_dot) * w,
    };
    let mut joint_states = y0.joint_states.clone();
    for (((d1, d2), d3), d4) in k1
        .joint_dots
        .iter()
        .zip(&k2.joint_dots)
        .zip(&k3.joint_dots)
        .zip(&k4.joint_dots)
    {
        if let Some(js) = joint_states.get_mut(d1.id.0 as usize) {
            js.q_scalar += (d1.q_dot + 2.0 * d2.q_dot + 2.0 * d3.q_dot + d4.q_dot) * w;
            js.q_vel_scalar += (d1.q_ddot + 2.0 * d2.q_ddot + 2.0 * d3.q_ddot + d4.q_ddot) * w;
            js.translation_m +=
                (d1.trans_dot + d2.trans_dot * 2.0 + d3.trans_dot * 2.0 + d4.trans_dot) * w;
            js.linear_vel_m_s +=
                (d1.vel_dot + d2.vel_dot * 2.0 + d3.vel_dot * 2.0 + d4.vel_dot) * w;
            js.rotation = (js.rotation
                + (d1.quat_dot + d2.quat_dot * 2.0 + d3.quat_dot * 2.0 + d4.quat_dot) * w)
                .normalize();
            js.angular_vel_rad_s +=
                (d1.omega_dot + d2.omega_dot * 2.0 + d3.omega_dot * 2.0 + d4.omega_dot) * w;
        }
    }
    let _ = JointKind::Fixed;
    PhaseState { base, joint_states }
}
