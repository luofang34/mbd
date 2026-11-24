use alloc::vec::Vec;

use glam::DVec3;
use mbd_core::{ComponentId, ComponentTree, DynamicMassContribution};

use crate::MultibodyError;
use crate::aba::{FloatingBaseState, MultiBodyDynamics};
use crate::spatial::{SpatialForce, SpatialTransform};

mod element;

pub use element::{HingeRelease, RailSlide};

/// Output of separating a child subtree under pyrotechnic/spring impulse (`ImpulseRelease`).
#[derive(Debug, Clone, PartialEq)]
pub struct ImpulseReleaseResult {
    /// Remaining parent vehicle tree.
    pub parent_tree: ComponentTree,
    /// Updated parent floating-base state after recoil impulse.
    pub parent_state: FloatingBaseState,
    /// Dynamic mass contributions remapped into `parent_tree`.
    pub parent_masses: Vec<DynamicMassContribution>,
    /// Parent ID mapping `(old_id, new_parent_id)`.
    pub parent_id_map: Vec<(ComponentId, ComponentId)>,
    /// Newly separated free-flying child vehicle tree.
    pub child_tree: ComponentTree,
    /// Free-body floating-base state of the separated child root after impulse.
    pub child_state: FloatingBaseState,
    /// Dynamic mass contributions remapped into `child_tree`.
    pub child_masses: Vec<DynamicMassContribution>,
    /// Child ID mapping `(old_id, new_child_id)`.
    pub child_id_map: Vec<(ComponentId, ComponentId)>,
}

/// Configuration for a guide rail slide separation mechanism (`RailSlideRelease`,
/// e.g., Vostok-1 SK-1 ejection seat or Tu-160 K-36DM seat).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RailSlideSpec {
    /// Component owning the [`mbd_core::JointKind::Prismatic`] guide rail joint.
    pub component: ComponentId,
    /// Distance from zero where the carriage leaves the guide rails, m.
    pub rail_length_m: f64,
    /// Net propellant/catapult actuator force along the rail axis, N.
    pub catapult_force_n: f64,
    /// Sliding guide friction magnitude opposing motion along the rail, N.
    pub guide_friction_n: f64,
}

/// Discrete state of a rail slide, held in its element's two modes.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RailSlideState {
    /// Whether firing has been initiated.
    pub fired: bool,
    /// Whether travel has reached the rail's end, so the host should
    /// separate the body with [`release_from_rail`].
    pub ready_to_release: bool,
}

/// Configuration for a hinged swing-open separation mechanism (`HingeSwingRelease`,
/// e.g., Apollo SLA panels or payload fairing petals).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HingeSwingSpec {
    /// Component owning the [`mbd_core::JointKind::Revolute`] hinge joint.
    pub component: ComponentId,
    /// Opening angle at which the structural hinge pins unhook cleanly, rad.
    pub release_angle_rad: f64,
    /// Positive spring actuator torque pushing toward `release_angle_rad`, N·m.
    pub spring_torque_nm: f64,
    /// Viscous hinge damper, N·m·s/rad.
    pub damper_nm_s_rad: f64,
}

/// Discrete state of a hinged panel, held in its element's two modes.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct HingeSwingState {
    /// Whether pyro bolts have cut to let the panel swing open.
    pub unlatched: bool,
    /// Whether the panel has reached its release angle, so the host should
    /// separate it with [`execute_impulse_release`].
    pub ready_to_release: bool,
}

/// Separates `child_root` from `tree`, preserving exact pre-split link
/// velocities and transferring exact equal-and-opposite linear/angular momentum
/// impulse `(±impulse_ns * axis_in_child_frame)` across the split interface.
pub fn execute_impulse_release(
    tree: &ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    base_state: &FloatingBaseState,
    child_root: ComponentId,
    impulse_ns: f64,
    axis_in_child_frame: DVec3,
) -> Result<ImpulseReleaseResult, MultibodyError> {
    if !impulse_ns.is_finite() || !axis_in_child_frame.is_finite() {
        return Err(MultibodyError::InvalidParameter {
            field: "impulse",
            value: impulse_ns,
        });
    }
    let axis_c = if impulse_ns == 0.0 {
        DVec3::X
    } else if let Some(axis) = axis_in_child_frame.try_normalize() {
        axis
    } else {
        return Err(MultibodyError::InvalidParameter {
            field: "axis_in_child_frame",
            value: axis_in_child_frame.length(),
        });
    };

    let poses = tree.evaluate_pose_table()?;
    let c_pose = poses.get(child_root).copied()?;
    let split = tree.split_subtree(child_root)?;
    let parent_masses = remap_masses(dynamic_masses, &split.parent_id_map);
    let child_masses = remap_masses(dynamic_masses, &split.child_id_map);

    let r_w_from_p = base_state.rotation_world_from_root;
    let c_pos_world = base_state.position_world_m + r_w_from_p * c_pose.position_root_m;
    let c_rot_world = (r_w_from_p * c_pose.rotation_root_from_link).normalize();
    // Velocity of the child root relative to the parent root origin's, in
    // parent root axes.
    let v_c_from_p = base_state
        .angular_vel_root_rad_s
        .cross(c_pose.position_root_m)
        + c_pose.linear_vel_root_m_s;
    let w_c_in_p = base_state.angular_vel_root_rad_s + c_pose.angular_vel_root_rad_s;
    let r_c_from_p = c_pose.rotation_root_from_link.conjugate();

    let mut parent_state = *base_state;
    let mut child_state = FloatingBaseState {
        position_world_m: c_pos_world,
        rotation_world_from_root: c_rot_world,
        linear_vel_world_m_s: base_state.linear_vel_world_m_s + r_w_from_p * v_c_from_p,
        angular_vel_root_rad_s: r_c_from_p * w_c_in_p,
    };

    if impulse_ns != 0.0 {
        let j_c = SpatialForce::new(DVec3::ZERO, axis_c * impulse_ns);
        let x_p_from_c =
            SpatialTransform::from_pos_quat(c_pose.position_root_m, c_pose.rotation_root_from_link);
        let j_p = -x_p_from_c.apply_force_to_parent(j_c);

        let i_p = MultiBodyDynamics::locked_root_spatial_inertia(&split.parent, &parent_masses)?;
        let i_c = MultiBodyDynamics::locked_root_spatial_inertia(&split.child, &child_masses)?;
        let dv_p = i_p.solve(j_p).ok_or(MultibodyError::SingularMassMatrix)?;
        let dv_c = i_c.solve(j_c).ok_or(MultibodyError::SingularMassMatrix)?;

        parent_state.angular_vel_root_rad_s += dv_p.ang;
        parent_state.linear_vel_world_m_s += r_w_from_p * dv_p.lin;
        child_state.angular_vel_root_rad_s += dv_c.ang;
        child_state.linear_vel_world_m_s += c_rot_world * dv_c.lin;
    }

    Ok(ImpulseReleaseResult {
        parent_tree: split.parent,
        parent_state,
        parent_masses,
        parent_id_map: split.parent_id_map,
        child_tree: split.child,
        child_state,
        child_masses,
        child_id_map: split.child_id_map,
    })
}

/// Completes a [`RailSlideSpec`] ejection by cleanly transitioning the sliding
/// subtree to free 6-DoF flight preserving exact guide-exit velocity.
pub fn release_from_rail(
    spec: &RailSlideSpec,
    tree: &ComponentTree,
    dynamic_masses: &[DynamicMassContribution],
    base_state: &FloatingBaseState,
) -> Result<ImpulseReleaseResult, MultibodyError> {
    execute_impulse_release(
        tree,
        dynamic_masses,
        base_state,
        spec.component,
        0.0,
        DVec3::X,
    )
}

pub(crate) fn remap_masses(
    masses: &[DynamicMassContribution],
    map: &[(ComponentId, ComponentId)],
) -> Vec<DynamicMassContribution> {
    masses
        .iter()
        .filter_map(|m| {
            map.iter()
                .find(|&&(old, _)| old == m.component)
                .map(|&(_, new_id)| DynamicMassContribution {
                    component: new_id,
                    ..*m
                })
        })
        .collect()
}
