//! Universal system packages. A vehicle, a ground station and a benchtop rig
//! are the same thing here: a `syspack.json` manifest plus SHA-256-verified
//! asset files, read through a [`PackageSource`] and verified by
//! [`VerifiedSyspack`].
//!
//! Loading fails closed: a malformed manifest, an unsafe asset path, a digest
//! or size mismatch, or a component graph that does not resolve is a typed
//! [`SyspackError`], and nothing of the package is usable.
//!
//! The crate knows no simulator, flight model or design tool. [`DynamicsSpec`]
//! only names how a system moves, and a consumer builds what it needs from the
//! verified data.
//!
//! Vehicles reuse other packages by pin: an instrument pack through
//! [`InstrumentPackRef`], and parts such as engines and propellers through
//! [`PartRef`], each naming the exact digest of the package's `syspack.json`.

mod error;
mod instrument_pack;
mod manifest;
mod source;
mod verified;

pub use error::SyspackError;
pub use instrument_pack::{
    BezelSize, DialUnitSpec, INSTRUMENTS_ASSET, InstrumentKindSpec, InstrumentMechanism,
    InstrumentPackDocument, InstrumentSpec, KollsmanSpec, LayoutSlotSpec, MarkingColour,
    MarkingSpec, NeedleSpec, ScalePoint, SourceBinding,
};
pub use manifest::{
    AssetDigest, ComponentRecord, ControlsSpec, DeviceKindSpec, DynamicsSpec,
    ENGINE_DOCUMENT_ASSET, InstrumentPackRef, MountedDeviceSpec, PROPELLER_DOCUMENT_ASSET, PartRef,
    SCHEMA_VERSION, SyspackManifest, SystemKind,
};
#[cfg(not(target_arch = "wasm32"))]
pub use source::DirectoryPackageSource;
pub use source::{MANIFEST_FILE, MemoryPackageSource, PackageSource};
pub use verified::{INITIAL_COMPONENT_TEMPERATURE_K, VerifiedSyspack, sha256_hex};
