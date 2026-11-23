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
