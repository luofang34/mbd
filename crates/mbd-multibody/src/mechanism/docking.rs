use alloc::vec::Vec;

use glam::{DQuat, DVec3};
use mbd_core::{
    ComponentId, ComponentTree, DynamicMassContribution, JointSpec, JointState, Placement,
};

use super::release::{ImpulseReleaseResult, execute_impulse_release, remap_masses};
use crate::MultibodyError;
use crate::aba::{FloatingBaseState, MultiBodyDynamics};
use crate::spatial::SpatialForce;

/// Phases of an orbital rendezvous & docking / capture mechanism (`DockingCapture`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DockingPhase {
    /// Mechanisms open and ready outside contact envelope.
    Open,
    /// Soft-capture ring in compliant 6-DoF spring-damper contact.
    SoftContact,
    /// Relative motion settled below threshold; ready for probe retraction.
    SoftDamped,
    /// Retracting capture ring to bring structural docking latches flush.
    Retracting,
    /// Structurally hard-mated (`graft_subtree` locked into single multibody tree).
    HardLatched,
    /// Unlatched and pushing apart under undocking spring impulse.
    Undocking,
}

/// Configuration for compliant soft-to-hard spacecraft docking capture.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DockingSpec {
    /// Lateral capture envelope radius, m.
    pub capture_radius_m: f64,
    /// Angular alignment tolerance inside capture envelope, rad.
    pub capture_angle_rad: f64,
    /// Maximum axial closing speed accepted for capture, m/s.
    pub max_closing_speed_m_s: f64,
    /// Velocity threshold below which soft capture enters [`DockingPhase::SoftDamped`], m/s.
    pub damping_vel_threshold_m_s: f64,
    /// Angular rate threshold for [`DockingPhase::SoftDamped`], rad/s.
    pub damping_rate_threshold_rad_s: f64,
    /// Retraction speed along docking normal during [`DockingPhase::Retracting`], m/s.
    pub retraction_speed_m_s: f64,
    /// Linear soft-capture ring compliance stiffness, N/m.
    pub linear_stiffness_n_m: f64,
    /// Linear soft-capture viscous damping, N·s/m.
    pub linear_damping_n_s_m: f64,
    /// Rotational soft-capture ring compliance stiffness, N·m/rad.
    pub angular_stiffness_nm_rad: f64,
    /// Rotational soft-capture damping, N·m·s/rad.
    pub angular_damping_nm_s_rad: f64,
    /// Separation spring impulse when undocking, N·s.
    pub undock_impulse_ns: f64,
}

/// Mutable state of a [`DockingCapture`](DockingSpec) port pair.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DockingState {
    /// Current mechanism phase.
    pub phase: DockingPhase,
    /// Current target retraction axial stand-off distance, m.
    pub standoff_m: f64,
    /// Accumulated dwell time in `SoftDamped`, s.
    pub dwell_s: f64,
}

impl Default for DockingState {
    fn default() -> Self {
        Self {
            phase: DockingPhase::Open,
            standoff_m: 0.0,
            dwell_s: 0.0,
        }
    }
}

/// Equal-and-opposite world wrenches and hard-mate trigger produced by [`step_docking_capture`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DockingStepOutput {
    /// Updated phase.
    pub phase: DockingPhase,
    /// Soft-capture spatial wrench acting ON chaser (`A`) in world axes about port A.
    pub world_wrench_on_a: SpatialForce,
    /// Soft-capture spatial wrench acting ON target (`B`) in world axes about port B.
    pub world_wrench_on_b: SpatialForce,
    /// True once retraction completes and [`hard_mate_vessels`] should lock the trees.
    pub ready_for_hard_mate: bool,
}

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

/// Evaluates world port poses `(pos, rot, vel, omega)` of chaser port `A` and
/// target port `B` through the soft-capture state machine, generating strictly
/// balanced equal-and-opposite soft-capture ring wrenches in world coordinates.
#[allow(clippy::too_many_arguments)]
pub fn step_docking_capture(
    spec: &DockingSpec,
    state: &mut DockingState,
    pos_a_world: DVec3,
    rot_a_world: DQuat,
    vel_a_world: DVec3,
    omega_a_world: DVec3,
    pos_b_world: DVec3,
    rot_b_world: DQuat,
    vel_b_world: DVec3,
    omega_b_world: DVec3,
    axis_a_local: DVec3,
    dt_s: f64,
) -> DockingStepOutput {
    let delta_pos = pos_b_world - pos_a_world;
    let delta_vel = vel_b_world - vel_a_world;
    let dist = delta_pos.length();
    let rel_rot = rot_a_world.conjugate() * rot_b_world;
    let (_, align_angle) = rel_rot.to_axis_angle();
    let rel_omega = omega_b_world - omega_a_world;
    let axis_world = (rot_a_world * axis_a_local).normalize_or(DVec3::X);

    match state.phase {
        DockingPhase::Open => {
            if dist <= spec.capture_radius_m
                && align_angle <= spec.capture_angle_rad
                && delta_vel.length() <= spec.max_closing_speed_m_s
            {
                state.phase = DockingPhase::SoftContact;
                state.standoff_m = delta_pos.dot(axis_world).max(0.05);
                state.dwell_s = 0.0;
            }
            zero_dock_out(state.phase)
        }
        DockingPhase::SoftContact | DockingPhase::SoftDamped | DockingPhase::Retracting => {
            if state.phase == DockingPhase::SoftContact
                && delta_vel.length() <= spec.damping_vel_threshold_m_s
                && rel_omega.length() <= spec.damping_rate_threshold_rad_s
            {
                state.dwell_s += dt_s;
                if state.dwell_s >= 0.05 {
                    state.phase = DockingPhase::Retracting;
                }
            }
            if state.phase == DockingPhase::Retracting {
                state.standoff_m = (state.standoff_m - spec.retraction_speed_m_s * dt_s).max(0.0);
            }
            let target_pos_b = pos_a_world + axis_world * state.standoff_m;
            let f_on_b = (target_pos_b - pos_b_world) * spec.linear_stiffness_n_m
                + (vel_a_world - vel_b_world) * spec.linear_damping_n_s_m;
            let (ax, ang) = (rot_a_world * rel_rot.to_scaled_axis(), align_angle);
            let _ = ang;
            let t_on_b = (-ax) * spec.angular_stiffness_nm_rad
                + (omega_a_world - omega_b_world) * spec.angular_damping_nm_s_rad;
            let w_b = SpatialForce::new(t_on_b, f_on_b);
            // Action-reaction about A and B: F_A = -F_B, and tau_A = -tau_B - (B - A) x F_B.
            let w_a = SpatialForce::new(-t_on_b - delta_pos.cross(f_on_b), -f_on_b);
            let ready =
                state.phase == DockingPhase::Retracting && state.standoff_m <= 1e-4 && dist <= 0.05;
            if ready {
                state.phase = DockingPhase::HardLatched;
            }
            DockingStepOutput {
                phase: state.phase,
                world_wrench_on_a: w_a,
                world_wrench_on_b: w_b,
                ready_for_hard_mate: ready,
            }
        }
        DockingPhase::HardLatched | DockingPhase::Undocking => zero_dock_out(state.phase),
    }
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
        linear_vel_root_m_s: DVec3::ZERO,
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
        linear_vel_root_m_s: v0.lin,
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

fn zero_dock_out(phase: DockingPhase) -> DockingStepOutput {
    DockingStepOutput {
        phase,
        world_wrench_on_a: SpatialForce::ZERO,
        world_wrench_on_b: SpatialForce::ZERO,
        ready_for_hard_mate: false,
    }
}
