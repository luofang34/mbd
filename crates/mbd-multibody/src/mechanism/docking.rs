use alloc::vec::Vec;

use glam::{DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentTree, DynamicMassContribution, JointSpec, JointState, Placement,
};

mod capture;

pub use capture::{
    DockingCapture, DockingEvaluation, DockingPhase, DockingSpec, DockingState, DockingTransition,
    PortAnchor, PortKinematics,
};

use super::release::{ImpulseReleaseResult, execute_impulse_release, remap_masses};
use crate::MultibodyError;
use crate::aba::{FloatingBaseState, MultiBodyDynamics};
use crate::spatial::SpatialForce;

/// Merged tree and exact momentum-conserving base state from [`hard_mate_vessels`].
#[derive(Debug, Clone, PartialEq)]
pub struct HardMateResult {
    /// Merged multibody component tree rooted at chaser (`tree_a`'s root).
    pub tree: ComponentTree,
    /// Exact momentum- & angular-momentum-conserving base state of `tree`.
    pub base_state: FloatingBaseState,
    /// Combined remapped dynamic mass contributions.
    pub dynamic_masses: Vec<DynamicMassContribution>,
    /// Mapping from `tree_a` original IDs to merged IDs.
    pub a_id_map: Vec<(ComponentId, ComponentId)>,
    /// Mapping from `tree_b` original IDs to merged IDs.
    pub b_id_map: Vec<(ComponentId, ComponentId)>,
}

/// Envelope specification for a robotic end-effector rigid grapple (`GrappleLock`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GrappleLockSpec {
    /// Radial fixture capture envelope, m.
    pub capture_radius_m: f64,
    /// Angular fixture alignment tolerance, rad.
    pub capture_angle_rad: f64,
    /// Maximum relative speed for safe rigid grapple, m/s.
    pub max_relative_speed_m_s: f64,
}

/// Rigidly mates `tree_b` onto `port_a` of `tree_a` (re-rooting `tree_b` to
/// `port_b` first if `port_b` is not already root) and solves the exact merged
/// base velocity conserving total world linear and angular momentum.
#[allow(clippy::too_many_arguments)]
pub fn hard_mate_vessels(
    tree_a: &ComponentTree,
    masses_a: &[DynamicMassContribution],
    state_a: &FloatingBaseState,
    port_a: ComponentId,
    tree_b: &ComponentTree,
    masses_b: &[DynamicMassContribution],
    state_b: &FloatingBaseState,
    port_b: ComponentId,
    mate_placement: Placement,
) -> Result<HardMateResult, MultibodyError> {
    let mom_a = MultiBodyDynamics::system_momentum(tree_a, masses_a, state_a)?;
    let mom_b = MultiBodyDynamics::system_momentum(tree_b, masses_b, state_b)?;

    let (tree_b_rooted, b_reroot_map) = if tree_b.spec(port_b)?.placement.parent.is_some() {
        let rr = tree_b.reroot(port_b)?;
        (rr.tree, rr.id_map)
    } else {
        (tree_b.clone(), tree_b.ids().map(|id| (id, id)).collect())
    };
    let masses_b_rerooted = remap_masses(masses_b, &b_reroot_map);

    let grafted = tree_a.graft_subtree(
        &tree_b_rooted,
        port_a,
        mate_placement,
        JointSpec::FIXED,
        JointState::FIXED,
    )?;
    let a_id_map = grafted.base_id_map;
    let b_id_map: Vec<_> = b_reroot_map
        .iter()
        .filter_map(|&(orig_b, rr_b)| {
            grafted
                .grafted_id_map
                .iter()
                .find(|&&(k, _)| k == rr_b)
                .map(|&(_, final_b)| (orig_b, final_b))
        })
        .collect();
    let mut dynamic_masses = remap_masses(masses_a, &a_id_map);
    dynamic_masses.extend(remap_masses(&masses_b_rerooted, &grafted.grafted_id_map));
    let base_state =
        solve_merged_base_state(&grafted.tree, &dynamic_masses, state_a, mom_a, mom_b)?;

    Ok(HardMateResult {
        tree: grafted.tree,
        base_state,
        dynamic_masses,
        a_id_map,
        b_id_map,
    })
}

fn solve_merged_base_state(
    merged_tree: &ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    state_a: &FloatingBaseState,
    mom_a: crate::aba::SystemMomentum,
    mom_b: crate::aba::SystemMomentum,
) -> Result<FloatingBaseState, MultibodyError> {
    let total_mass = mom_a.total_mass_kg + mom_b.total_mass_kg;
    let com_tot = (mom_a.com_world_m * mom_a.total_mass_kg
        + mom_b.com_world_m * mom_b.total_mass_kg)
        / total_mass;
    let p_tot = mom_a.linear_world_kg_m_s + mom_b.linear_world_kg_m_s;
    let l_tot_com = mom_a.angular_about_com_world_nms
        + (mom_a.com_world_m - com_tot).cross(mom_a.linear_world_kg_m_s)
        + mom_b.angular_about_com_world_nms
        + (mom_b.com_world_m - com_tot).cross(mom_b.linear_world_kg_m_s);

    let l_root_world = l_tot_com + (com_tot - state_a.position_world_m).cross(p_tot);
    let r_a_inv = state_a.rotation_world_from_root.conjugate();
    let h_desired_root = SpatialForce::new(r_a_inv * l_root_world, r_a_inv * p_tot);

    let zero_base = FloatingBaseState {
        position_world_m: state_a.position_world_m,
        rotation_world_from_root: state_a.rotation_world_from_root,
        linear_vel_world_m_s: DVec3::ZERO,
        angular_vel_root_rad_s: DVec3::ZERO,
    };
    let mom_joints = MultiBodyDynamics::system_momentum(merged_tree, dynamic_masses, &zero_base)?;
    let l_j_world = mom_joints.angular_about_com_world_nms
        + (mom_joints.com_world_m - state_a.position_world_m).cross(mom_joints.linear_world_kg_m_s);
    let h_joints_root = SpatialForce::new(
        r_a_inv * l_j_world,
        r_a_inv * mom_joints.linear_world_kg_m_s,
    );

    let h_base = MultiBodyDynamics::locked_root_spatial_inertia(merged_tree, dynamic_masses)?;
    let v0 = h_base
        .solve(h_desired_root - h_joints_root)
        .ok_or(MultibodyError::SingularMassMatrix)?;

    Ok(FloatingBaseState {
        position_world_m: state_a.position_world_m,
        rotation_world_from_root: state_a.rotation_world_from_root,
        linear_vel_world_m_s: state_a.rotation_world_from_root * v0.lin,
        angular_vel_root_rad_s: v0.ang,
    })
}

/// Separates a docked or grappled target subtree via [`execute_impulse_release`]
/// (preserves exact high-rate spin like Gemini 8 60 rpm centrifugal undocking).
pub fn undock_vessel(
    tree: &ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    base_state: &FloatingBaseState,
    child_root: ComponentId,
    impulse_ns: f64,
    axis_in_child_frame: DVec3,
) -> Result<ImpulseReleaseResult, MultibodyError> {
    execute_impulse_release(
        tree,
        dynamic_masses,
        base_state,
        child_root,
        impulse_ns,
        axis_in_child_frame,
    )
}

/// Validates robotic grapple capture envelope and rigidly mates payload onto arm end effector.
#[allow(clippy::too_many_arguments)]
pub fn grapple_lock(
    spec: &GrappleLockSpec,
    ee_world_pos: DVec3,
    ee_world_rot: DQuat,
    ee_world_vel: DVec3,
    fixture_world_pos: DVec3,
    fixture_world_rot: DQuat,
    fixture_world_vel: DVec3,
    tree_servicer: &ComponentTree,
    masses_servicer: &[DynamicMassContribution],
    state_servicer: &FloatingBaseState,
    ee_component: ComponentId,
    tree_payload: &ComponentTree,
    masses_payload: &[DynamicMassContribution],
    state_payload: &FloatingBaseState,
    fixture_component: ComponentId,
    mate_placement: Placement,
) -> Result<HardMateResult, MultibodyError> {
    let dist = (fixture_world_pos - ee_world_pos).length();
    let (_, ang) = (ee_world_rot.conjugate() * fixture_world_rot).to_axis_angle();
    let speed = (fixture_world_vel - ee_world_vel).length();
    if dist > spec.capture_radius_m
        || ang > spec.capture_angle_rad
        || speed > spec.max_relative_speed_m_s
    {
        return Err(MultibodyError::InvalidMechanismState {
            detail: alloc::format!(
                "grapple target outside envelope (dist={dist:.3} m, ang={ang:.3} rad, speed={speed:.3} m/s)"
            ),
        });
    }
    hard_mate_vessels(
        tree_servicer,
        masses_servicer,
        state_servicer,
        ee_component,
        tree_payload,
        masses_payload,
        state_payload,
        fixture_component,
        mate_placement,
    )
}
