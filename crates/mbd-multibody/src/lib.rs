//! Floating-base articulated multibody dynamics with an explicit model/solver
//! seam.
//!
//! A [`MultibodyModel`] is a component tree prepared once; its state is an
//! explicit [`ModelState`] value with a flat view for integrators, and
//! [`MultibodyModel::derivative`] is a pure function of time, state and
//! inputs. Everything that loads the model is a [`ForceElement`]: contact
//! points, wheels, tethers, docking rings, loop constraints and hulls carry
//! their own continuous states and discrete modes inside the state value.
//! Any [`Integrator`] ([`Rk4`], [`DormandPrince54`], [`Rosenbrock23`])
//! advances the whole system over an exact interval.
//!
//! Also here: manipulator kinematics and Generalized Jacobian IK, and the
//! separation, grapple and hard-mate operations that restructure trees.

#![cfg_attr(not(test), no_std)]

extern crate alloc;

mod aba;
mod contact;
mod element;
mod error;
mod hydrodynamics;
mod integrate;
mod loops;
mod manipulator;
mod mechanism;
mod model;
mod spatial;
mod wheel;

pub use aba::{
    FloatingBaseState, JointAccel, JointDriveInput, JointLoadInput, MultiBodyAccel,
    MultiBodyDynamics, SystemMomentum,
};
pub use contact::{
    ContactEvaluation, ContactPointSpec, ContactPointState, PinnedRestInputs, PinnedRestSpec,
    PinnedSurfaceRest, SupportedLoad, SurfaceMedium, SurfaceQuery, SurfaceSample,
    step_pinned_surface_rest,
};
pub use element::{BoxedElement, ElementContext, ForceElement, ForceFn, UniformGravity};
pub use error::{EnvironmentError, MultibodyError};
pub use hydrodynamics::{HullStation, HullTotals, HydrodynamicsModel, WaterQuery, WaterState};
pub use integrate::{
    DormandPrince54, Integrator, OdeSystem, Rk4, Rosenbrock23, StepReport, Tolerance, Tolerances,
};
pub use loops::LoopConstraint;
pub use manipulator::{
    AdmittanceController, LinkCapsule, ManipulatorChain, minimum_capsule_clearance,
};
pub use mechanism::{
    DockingCapture, DockingEvaluation, DockingPhase, DockingSpec, DockingState, DockingTransition,
    GrappleLockSpec, HardMateResult, HingeRelease, HingeSwingSpec, HingeSwingState,
    ImpulseReleaseResult, PortAnchor, PortKinematics, RailSlide, RailSlideSpec, RailSlideState,
    Tether, TetherAnchor, TetherCommand, TetherCutReason, TetherEnds, TetherEvaluation,
    WinchTetherSpec, WinchTetherState, execute_impulse_release, grapple_lock, hard_mate_vessels,
    release_from_rail, undock_vessel,
};
pub use model::{
    CentreOfMass, CentreOfMassOde, Kinematics, Loads, MediumMotion, ModelScale, ModelState,
    MultibodyModel, PointKinematics, Resolution, StateKind, StateLayout, TimeScale, Workspace,
};
pub use spatial::{SpatialForce, SpatialMat6, SpatialMotion, SpatialTransform};
pub use wheel::{TireModelSpec, Wheel, WheelCommand, WheelEvaluation, WheelSpec, WheelState};
