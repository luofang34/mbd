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

mod error;
mod instrument_pack;
mod manifest;
mod source;
mod verified;

pub use error::SyspackError;
pub use instrument_pack::{
    BezelSize, DialFaceSpec, DialUnitSpec, FailureSpec, GraduationSpec, INSTRUMENTS_ASSET,
    InstrumentKindSpec, InstrumentMechanism, InstrumentPackDocument, InstrumentSpec, KollsmanSpec,
    LayoutSlotSpec, LegendSpec, MarkingColour, MarkingSpec, NeedleSpec, NumeralSpec,
    PointerFaceSpec, PointerShapeSpec, ScalePoint, SourceBinding,
};
pub use manifest::{
    AssetDigest, ComponentRecord, ControlsSpec, DeviceKindSpec, DynamicsSpec, InstrumentPackRef,
    MountedDeviceSpec, SCHEMA_VERSION, SyspackManifest, SystemKind,
};
#[cfg(not(target_arch = "wasm32"))]
pub use source::DirectoryPackageSource;
pub use source::{MANIFEST_FILE, MemoryPackageSource, PackageSource};
pub use verified::{INITIAL_COMPONENT_TEMPERATURE_K, VerifiedSyspack, sha256_hex};
