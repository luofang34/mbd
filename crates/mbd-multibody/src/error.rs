use alloc::string::String;
use mbd_core::{ComponentId, MbdError};

/// Errors produced by multibody dynamics, manipulators, constraints, mechanisms, or contact solvers.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum MultibodyError {
    /// Underlying component tree, joint, or flow error from `mbd-core`.
    #[error(transparent)]
    Mbd(#[from] MbdError),
    /// Invalid scalar/vector configuration or runtime input.
    #[error("multibody parameter {field} is invalid: {value}")]
    InvalidParameter {
        /// Offending parameter name.
        field: &'static str,
        /// Non-finite or out-of-range value.
        value: f64,
    },
    /// Articulated or rigid base mass matrix cannot be inverted safely.
    #[error("multibody base spatial inertia matrix is singular")]
    SingularMassMatrix,
    /// The articulated inertia about a joint's free axes is singular relative
    /// to its own scale (for example a massless link driven by a nonzero
    /// effort), so the joint acceleration is undefined.
    #[error("articulated inertia about the joint of component {id} is singular")]
    SingularJointInertia {
        /// Component owning the joint.
        id: ComponentId,
    },
    /// A non-fixed joint is declared on the root component, which has no
    /// parent body for the joint to move against.
    #[error("root component {id} declares a non-fixed joint")]
    NonFixedRootJoint {
        /// Root component id.
        id: ComponentId,
    },
    /// Kinematic chain is malformed, detached, or contains unsupported joint multiplicity.
    #[error("invalid kinematic chain: {detail}")]
    InvalidChain {
        /// Reason the chain cannot be evaluated.
        detail: String,
    },
    /// Mechanism or capture/release state violates preconditions.
    #[error("invalid mechanism state: {detail}")]
    InvalidMechanismState {
        /// Reason the mechanism action was rejected.
        detail: String,
    },
    /// Closed-loop or mechanism operation referenced an unexpected component id.
    #[error("component {id} does not satisfy joint requirements")]
    WrongJointKind {
        /// Component id.
        id: ComponentId,
    },
}
