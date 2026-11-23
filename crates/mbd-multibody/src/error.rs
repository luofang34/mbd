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
    /// A characteristic length, mass, time or acceleration of a model scale
    /// is not finite and positive, given or derived (a tree with no extent
    /// or no mass has no scale to derive).
    #[error("model scale {field} must be finite and positive, got {value}")]
    InvalidScale {
        /// Offending part of the scale.
        field: &'static str,
        /// Value given or derived.
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
    /// A component named by an element or a query is not part of the
    /// attached chain the model was prepared from.
    #[error("component {id} is detached from the model's chain")]
    DetachedComponent {
        /// Component id.
        id: ComponentId,
    },
    /// A flat state, rate or generalized vector has the wrong length for
    /// the model and element set it is used with.
    #[error("state vector has {actual} components, expected {expected}")]
    StateLengthMismatch {
        /// Length the model and elements define.
        expected: usize,
        /// Length supplied.
        actual: usize,
    },
    /// Error control cannot meet its tolerance with any representable step.
    #[error("step size underflow at t = {time_s} s (step {step_s} s)")]
    StepSizeUnderflow {
        /// Time the integrator had reached, s.
        time_s: f64,
        /// Step it would have had to take, s.
        step_s: f64,
    },
    /// An interval needed more internal steps than the integrator's limit.
    #[error("interval needs more than {limit} internal steps (reached t = {time_s} s)")]
    StepLimitExceeded {
        /// Configured limit of internal steps per interval.
        limit: u32,
        /// Time the integrator had reached, s.
        time_s: f64,
    },
    /// The iteration matrix `I − γ h J` of an implicit step is singular.
    #[error("implicit iteration matrix is singular at t = {time_s} s")]
    SingularIterationMatrix {
        /// Time of the failed step, s.
        time_s: f64,
    },
}
