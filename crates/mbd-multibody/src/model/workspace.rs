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
    pub(crate) base_accel: SpatialMotion,
    pub(crate) used_rigid_fast_path: bool,
    /// Added inertia per link of the evaluation in progress, for the
    /// operators of the resolution phase.
    pub(crate) added: Vec<Option<SpatialMat6>>,
    /// Second set of buffers, used by the operators of the resolution phase
    /// while the first holds the evaluation in progress. Created by the
    /// first evaluation that has a resolving element.
    pub(crate) scratch: Option<Box<Workspace>>,
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

    /// Base spatial acceleration of the latest forward-dynamics solve, in
    /// rotating root axes.
    pub const fn base_accel(&self) -> SpatialMotion {
        self.base_accel
    }
}
