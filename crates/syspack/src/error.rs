use mbd_core::MbdError;

/// Why a system package could not be read, verified, or built.
#[derive(Debug, thiserror::Error)]
pub enum SyspackError {
    /// `syspack.json` could not be read.
    #[error("cannot read the package manifest {path}")]
    ManifestRead {
        /// Where the manifest was expected.
        path: String,
        /// What the operating system reported.
        #[source]
        source: std::io::Error,
    },
    /// `syspack.json` is not a well-formed manifest.
    #[error("the package manifest is malformed")]
    ManifestParse(#[source] serde_json::Error),
    /// The manifest was written for a schema this build does not read.
    #[error("schema version {found} is not supported; expected {expected}")]
    UnsupportedSchema {
        /// The version the manifest declares.
        found: u32,
        /// The version this build reads.
        expected: u32,
    },
    /// The package id is not a lowercase slug.
    #[error("package id {id:?} must be lowercase ASCII alphanumerics, '-' or '_'")]
    InvalidPackageId {
        /// The offending id.
        id: String,
    },
    /// A required text field is empty.
    #[error("{field} must not be empty")]
    EmptyField {
        /// The offending field.
        field: &'static str,
    },
    /// An asset path could escape the package or is not canonical POSIX.
    #[error("asset path {path:?} is unsafe: {reason}")]
    UnsafeAssetPath {
        /// The offending path.
        path: String,
        /// The rule it breaks.
        reason: &'static str,
    },
    /// A declared digest is not 64 hexadecimal digits.
    #[error("asset {path} declares malformed SHA-256 {sha256:?}")]
    MalformedDigest {
        /// The asset.
        path: String,
        /// The declared digest.
        sha256: String,
    },
    /// A declared asset is not in the source.
    #[error("asset {path} is missing")]
    AssetMissing {
        /// The asset.
        path: String,
    },
    /// A declared asset could not be read.
    #[error("cannot read asset {path}")]
    AssetRead {
        /// The asset.
        path: String,
        /// What the operating system reported.
        #[source]
        source: std::io::Error,
    },
    /// An asset does not have its declared length.
    #[error("asset {path} has {actual} bytes; the manifest declares {declared}")]
    AssetSizeMismatch {
        /// The asset.
        path: String,
        /// Declared length.
        declared: u64,
        /// Length read.
        actual: u64,
    },
    /// An asset does not have its declared digest.
    #[error("asset {path} has SHA-256 {actual}; the manifest declares {declared}")]
    AssetDigestMismatch {
        /// The asset.
        path: String,
        /// Declared digest.
        declared: String,
        /// Digest of the bytes read.
        actual: String,
    },
    /// The dynamics refer to a file that `assets` does not declare, so it
    /// would escape verification.
    #[error("dynamics reference undeclared asset {path}")]
    UndeclaredAsset {
        /// The reference.
        path: String,
    },
    /// A JSBSim virtual path is not where JSBSim will look for the aircraft.
    #[error("JSBSim entry document is at {actual}, but JSBSim will read {expected}")]
    JsbsimEntryPath {
        /// Where `aircraft_dir` and `model_name` make JSBSim look.
        expected: String,
        /// The declared `virtual_path`.
        actual: String,
    },
    /// The package declares no components.
    #[error("the package declares no components")]
    NoComponents,
    /// Two components share a key.
    #[error("component key {key:?} is declared twice")]
    DuplicateComponent {
        /// The repeated key.
        key: String,
    },
    /// A component's parent is not in the package.
    #[error("component {component:?} names unknown parent {parent:?}")]
    UnknownParent {
        /// The child.
        component: String,
        /// The unresolved parent key.
        parent: String,
    },
    /// A component's parent chain never reaches a root.
    #[error("the parent chain of component {key:?} is a cycle")]
    ComponentCycle {
        /// A component on or leading to the cycle.
        key: String,
    },
    /// A component lookup named no component.
    #[error("no component has key {key:?}")]
    UnknownComponent {
        /// The unresolved key.
        key: String,
    },
    /// The component tree holds a non-physical value.
    #[error("the component tree is invalid")]
    InvalidTree(#[source] MbdError),
    /// The aerodynamic parameters are not physical.
    #[error("the aerodynamic parameters are invalid")]
    InvalidAero(#[source] MbdError),
    /// Two devices share an id.
    #[error("device id {id:?} is declared twice")]
    DuplicateDevice {
        /// The repeated id.
        id: String,
    },
    /// A device is mounted on a component the package does not have.
    #[error("device {device:?} is mounted on unknown component {site:?}")]
    UnknownDeviceSite {
        /// The device.
        device: String,
        /// The unresolved site key.
        site: String,
    },
    /// A device's parameters are out of range.
    #[error("device {device:?}: {field}: {detail}")]
    InvalidDevice {
        /// The device.
        device: String,
        /// The offending field.
        field: &'static str,
        /// What is wrong with it.
        detail: String,
    },
    /// The operation needs a different dynamics kind from the package's.
    #[error("{operation} needs {needed} dynamics; the package declares {declared}")]
    DynamicsMismatch {
        /// What was asked for.
        operation: &'static str,
        /// The dynamics it needs.
        needed: &'static str,
        /// The dynamics the manifest declares.
        declared: &'static str,
    },
}
