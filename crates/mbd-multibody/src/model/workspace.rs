//! Caller-owned scratch of the dynamics evaluation.

use alloc::boxed::Box;
use alloc::vec::Vec;

use glam::DVec3;

use crate::aba::JointAccel;
use crate::aba::articulated::AbaBuffers;
use crate::aba::chain::{LinkData, LinkFrame, LinkLoads};
use crate::spatial::{SpatialMat6, SpatialMotion};

/// Buffers a [`crate::MultibodyModel`] evaluates into. One workspace serves
/// any number of evaluations of models up to the largest it has seen; after
/// the first evaluation of a model, none allocates. It carries no
/// information between evaluations.
#[derive(Debug, Clone, Default)]
pub struct Workspace {
    pub(crate) frames: Vec<LinkFrame>,
    pub(crate) link_loads: Vec<LinkLoads>,
    pub(crate) gravity: Option<DVec3>,
    pub(crate) links: Vec<LinkData>,
    pub(crate) aba: AbaBuffers,
    pub(crate) joint_accels: Vec<JointAccel>,
    /// Base acceleration of the latest solve in root axes: the rate of the
    /// angular velocity's root components, and the inertial acceleration of
    /// the root origin.
    pub(crate) base_accel: SpatialMotion,
    pub(crate) base_linear_accel_world: DVec3,
    pub(crate) used_rigid_fast_path: bool,
    /// Added inertia per link of the evaluation in progress, for the
    /// operators of the resolution phase.
    pub(crate) added: Vec<Option<SpatialMat6>>,
    /// Second set of buffers, used by the operators of the resolution phase
    /// while the first holds the evaluation in progress. Created by the
    /// first evaluation that has a resolving element.
    pub(crate) scratch: Option<Box<Workspace>>,
    /// Flat state in root-origin coordinates, rebuilt from the
    /// centre-of-mass coordinates an integrator advances.
    pub(crate) root_state: Vec<f64>,
    /// State and modes at the start of a step, restored if the step fails.
    pub(crate) backup: (Vec<f64>, Vec<f64>),
}

impl Workspace {
    /// An empty workspace; it sizes itself on first use.
    pub fn new() -> Self {
        Self::default()
    }

    /// Joint accelerations of the latest forward-dynamics solve, in chain
    /// order.
    pub fn joint_accels(&self) -> &[JointAccel] {
        &self.joint_accels
    }

    /// Inertial acceleration of the root origin from the latest
    /// forward-dynamics solve, in world axes, m/s²: the rate of the base
    /// linear velocity. After a step the latest solve is the integrator's
    /// last stage; evaluate [`crate::MultibodyModel::derivative`] at a
    /// state for the acceleration at that state.
    pub const fn base_linear_accel_world_m_s2(&self) -> DVec3 {
        self.base_linear_accel_world
    }

    /// Rate of the base angular velocity's root-axis components from the
    /// latest forward-dynamics solve, rad/s².
    pub const fn base_angular_accel_root_rad_s2(&self) -> DVec3 {
        self.base_accel.ang
    }

    /// Whether the latest forward-dynamics solve took the composite rigid
    /// body shortcut: every joint fixed and no element adding inertia.
    pub const fn used_rigid_fast_path(&self) -> bool {
        self.used_rigid_fast_path
    }
}
