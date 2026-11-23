//! Unified floating-base Featherstone multibody dynamics (`RigidFastPath` +
//! full ABA + RK4), robotic manipulator kinematics and Generalized Jacobian IK,
//! closed-loop constraints, the six unified separation/docking/winch mechanisms,
//! solid contact/wheels, and spherical-planet ocean hydrodynamics.

#![cfg_attr(not(test), no_std)]

extern crate alloc;

mod aba;
mod contact;
mod error;
mod hydrodynamics;
mod loops;
mod manipulator;
mod mechanism;
mod spatial;
mod wheel;

pub use aba::{
    FloatingBaseState, JointAccel, JointDriveInput, JointLoadInput, MultiBodyAccel,
    MultiBodyDynamics, StageForces, SystemMomentum, step_rk4, step_rk4_with_forces,
    step_rk4_with_loads,
};
pub use contact::{
    ContactPointSpec, ContactPointState, ContactStepOutput, PinnedSurfaceRest, SurfaceMedium,
    SurfaceSample, evaluate_contact_point, step_pinned_surface_rest,
};
pub use error::MultibodyError;
pub use hydrodynamics::{HullStation, HydrodynamicsModel, HydrodynamicsOutput, WaterState};
pub use loops::{LoopConstraint, LoopOutput, evaluate_loop_constraints};
pub use manipulator::{
    AdmittanceController, LinkCapsule, ManipulatorChain, minimum_capsule_clearance,
};
pub use mechanism::{
    DockingPhase, DockingSpec, DockingState, DockingStepOutput, GrappleLockSpec, HardMateResult,
    HingeSwingSpec, HingeSwingState, ImpulseReleaseResult, RailDriveOutput, RailSlideSpec,
    RailSlideState, TetherCutReason, TetherStepOutput, WinchTetherSpec, WinchTetherState,
    execute_impulse_release, grapple_lock, hard_mate_vessels, release_from_rail,
    step_docking_capture, step_hinge_release, step_rail_drive, step_tether, undock_vessel,
};
pub use spatial::{SpatialForce, SpatialMat6, SpatialMotion, SpatialTransform};
pub use wheel::{WheelCommand, WheelForceOutput, WheelSpec, WheelState, evaluate_wheel};
