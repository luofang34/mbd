use alloc::vec::Vec;

use glam::{DMat3, DQuat, DVec3};

use crate::{ComponentId, MbdError};

/// World/root-frame kinematics of one attached or detached component link in
/// [`crate::ComponentTree`], evaluated in a single O(N) root-to-leaves pass.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkPose {
    /// Component id in its tree.
    pub id: ComponentId,
    /// Link origin in the vehicle root frame, metres.
    pub position_root_m: DVec3,
    /// Link-to-root orientation quaternion.
    pub rotation_root_from_link: DQuat,
    /// Linear velocity of the link origin relative to the root frame, m/s.
    pub linear_vel_root_m_s: DVec3,
    /// Angular velocity of the link relative to the root frame, rad/s.
    pub angular_vel_root_rad_s: DVec3,
    /// Whether this link and all its ancestors are attached.
    pub attached: bool,
}

/// Snapshot of every link pose in a [`crate::ComponentTree`], indexed by
/// [`ComponentId`] and consumed identically by aerodynamics, propulsion,
/// sensors, and 3D rendering (`NearFieldUsdPass`).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PoseTable {
    poses: Vec<LinkPose>,
}

impl PoseTable {
    /// Wraps a contiguous vector ordered by `ComponentId(0..len)`.
    pub(crate) fn from_vec(poses: Vec<LinkPose>) -> Self {
        Self { poses }
    }

    /// Pose for `id`, failing closed if `id` is out of range.
    pub fn get(&self, id: ComponentId) -> Result<&LinkPose, MbdError> {
        self.poses
            .get(id.0 as usize)
            .ok_or(MbdError::UnknownComponent { id })
    }

    /// Iterates over all component poses in `ComponentId` order.
    pub fn iter(&self) -> impl Iterator<Item = &LinkPose> {
        self.poses.iter()
    }

    /// Number of links in the table.
    pub fn len(&self) -> usize {
        self.poses.len()
    }

    /// Whether the pose table has zero links.
    pub fn is_empty(&self) -> bool {
        self.poses.is_empty()
    }
}

/// Force and couple applied at a point inside one component's local frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WrenchAtSite {
    /// Component owning this site.
    pub site: ComponentId,
    /// Application point in `site`'s local axes, metres.
    pub point_local_m: DVec3,
    /// Force in `site`'s local axes, N.
    pub force_local_n: DVec3,
    /// Pure couple about `point_local_m` in `site`'s local axes, N·m.
    pub torque_local_nm: DVec3,
}

impl WrenchAtSite {
    /// Zero wrench at a component's origin.
    pub const fn zero(site: ComponentId) -> Self {
        Self {
            site,
            point_local_m: DVec3::ZERO,
            force_local_n: DVec3::ZERO,
            torque_local_nm: DVec3::ZERO,
        }
    }

    /// Transfers this wrench into the vehicle root frame as `(force_root_n, moment_about_cg_root_nm)`.
    pub fn to_root_wrench(&self, pose: &LinkPose, cg_root_m: DVec3) -> (DVec3, DVec3) {
        if !pose.attached {
            return (DVec3::ZERO, DVec3::ZERO);
        }
        let force_root = pose.rotation_root_from_link * self.force_local_n;
        let torque_root = pose.rotation_root_from_link * self.torque_local_nm;
        let app_root = pose.position_root_m + pose.rotation_root_from_link * self.point_local_m;
        let moment_root = torque_root + (app_root - cg_root_m).cross(force_root);
        (force_root, moment_root)
    }
}

/// Live dynamic mass/CG/inertia contribution from a subsystem (such as a partially
/// drained geometric fluid tank, movable ballast, or transferred payload) added
/// to a component's structural mass properties.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DynamicMassContribution {
    /// Host component to which this distributed mass belongs.
    pub component: ComponentId,
    /// Dynamic mass, kg (nonnegative).
    pub mass_kg: f64,
    /// Dynamic CG location in the host component's local frame, metres.
    pub cg_local_m: DVec3,
    /// Dynamic inertia tensor about `cg_local_m` in component axes, kg·m².
    pub inertia_cg_local: DMat3,
}

impl DynamicMassContribution {
    /// Checks finiteness, nonnegativity, and physical inertia symmetry/triangle bounds.
    pub fn validate(&self) -> Result<(), MbdError> {
        if !self.mass_kg.is_finite() || self.mass_kg < 0.0 {
            return Err(MbdError::InvalidMassContribution {
                component: self.component,
                field: "mass_kg",
            });
        }
        if !self.cg_local_m.is_finite() {
            return Err(MbdError::InvalidMassContribution {
                component: self.component,
                field: "cg_local_m",
            });
        }
        if crate::component::inertia_defect(&self.inertia_cg_local).is_some() {
            return Err(MbdError::InvalidMassContribution {
                component: self.component,
                field: "inertia_cg_local",
            });
        }
        Ok(())
    }
}

/// Conserved thermal power exchange attached to a component node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalLoad {
    /// Target component id.
    pub component: ComponentId,
    /// Heat input (positive heats) or rejection (negative cools), W.
    pub heat_rate_w: f64,
}

/// Prioritized electrical load or source attached to a component node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElectricalLoad {
    /// Target component id.
    pub component: ComponentId,
    /// Power demand (> 0) or supply (< 0), W.
    pub power_w: f64,
    /// Shedding priority: `0` = critical/non-sheddable, higher numbers shed first on brownout.
    pub priority: u8,
}
