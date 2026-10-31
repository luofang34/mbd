use alloc::string::String;

use crate::ComponentId;

/// Why a model-based design query or conversion could not be answered.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum MbdError {
    /// A component id, or a parent reference, names no component in the tree.
    #[error("component {id} is not in the tree")]
    UnknownComponent {
        /// The id that resolved to nothing.
        id: ComponentId,
    },
    /// Following parent links from a component never reached a root.
    #[error("parent chain of component {id} contains a cycle")]
    HierarchyCycle {
        /// The component whose chain was being resolved.
        id: ComponentId,
    },
    /// A design-time or runtime component value is not physical.
    #[error("component {name:?}: {field}: {detail}")]
    InvalidComponent {
        /// Component name.
        name: String,
        /// Offending field.
        field: &'static str,
        /// What was wrong.
        detail: String,
    },
    /// The site belongs to a separated stage, so it has no lever arm to the
    /// remaining vehicle.
    #[error("component {id} is on a detached stage")]
    DetachedComponent {
        /// The detached site.
        id: ComponentId,
    },
    /// Nothing with mass remains attached, so there is no centre of gravity.
    #[error("no attached mass in the component tree")]
    NoAttachedMass,
    /// A state slice does not cover the tree's components one to one.
    #[error("expected {expected} component states, got {actual}")]
    StateCountMismatch {
        /// Components in the tree.
        expected: usize,
        /// States supplied.
        actual: usize,
    },
    /// A flow input is non-finite or outside what is physical.
    #[error("flow field {field} is invalid: {value}")]
    InvalidFlow {
        /// Offending field.
        field: &'static str,
        /// Offending value.
        value: f64,
    },
    /// An aerodynamic model parameter is non-finite or outside what is
    /// physical.
    #[error("aero parameter {field} is invalid: {value}")]
    InvalidAeroParameter {
        /// Offending field.
        field: &'static str,
        /// Offending value.
        value: f64,
    },
    /// A value does not fit the bits of the field it is packed into.
    #[error("{field} value {value} is outside {min}..={max}")]
    FieldOutOfRange {
        /// Offending field.
        field: &'static str,
        /// Offending value.
        value: i64,
        /// Smallest representable value.
        min: i64,
        /// Largest representable value.
        max: i64,
    },
    /// A value headed for a bus has no numeric encoding.
    #[error("{field} value is not finite")]
    NonFiniteSignal {
        /// Offending field.
        field: &'static str,
    },
    /// An ARINC 429 word arrived with even parity.
    #[error("ARINC 429 word {word:#010x} failed odd parity")]
    Arinc429Parity {
        /// The received word.
        word: u32,
    },
    /// An ARINC 429 word carries a status other than normal operation.
    #[error("ARINC 429 label {label:#o} reports status {ssm:#04b}")]
    Arinc429Status {
        /// Word label.
        label: u8,
        /// Received sign/status matrix.
        ssm: u8,
    },
    /// An ARINC 429 BCD field holds a nibble above 9.
    #[error("ARINC 429 data {data:#07x} is not valid BCD")]
    Arinc429Bcd {
        /// The 19-bit data field.
        data: u32,
    },
    /// A frame was delivered to the wrong ARINC 429 receiver.
    #[error("ARINC 429 frame on channel {channel} label {label:#o} was not expected")]
    Arinc429Routing {
        /// Channel the frame arrived on.
        channel: u8,
        /// Label the frame carries.
        label: u8,
    },
    /// A resolver signal set does not determine a shaft angle.
    #[error("resolver signal lost: {detail}")]
    ResolverSignalLost {
        /// What was wrong.
        detail: String,
    },
    /// A port value is a different variant from the one the operation reads.
    #[error("expected a {expected} port value")]
    WrongPortKind {
        /// The variant the operation reads.
        expected: &'static str,
    },
}
