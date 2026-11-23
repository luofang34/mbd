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
mod typeface;
mod verified;

pub use error::SyspackError;
pub use instrument_pack::{
    BezelSize, DialFaceSpec, DialUnitSpec, DirectInstrument, FaceStyleSpec, FailureSpec,
    GraduationSpec, INSTRUMENTS_ASSET, InstrumentKindSpec, InstrumentMechanism,
    InstrumentPackDocument, InstrumentSpec, KollsmanSpec, LayoutSlotSpec, LegendSpec,
    LetteringSpec, MarkingColour, MarkingSpec, NeedleSpec, NumeralSpec, PointerChannelSpec,
    PointerFaceSpec, PointerShapeSpec, ScalePoint, SourceBinding,
};
pub use manifest::{
    AbepThrusterRecord, AeroElementKindRecord, AeroElementRecord, AirbreathingEngineRecord,
    AirbreathingKindRecord, AoCoatingRecord, AssetDigest, BatteryRecord, ComponentRecord,
    ControlsSpec, DeviceKindSpec, DynamicsSpec, ENGINE_DOCUMENT_ASSET, ElectricThrusterRecord,
    FeedRegimeRecord, FluidSpeciesRecord, InstrumentPackRef, JointRecord, JointStopRecord,
    LiquidRocketRecord, MIN_SCHEMA_VERSION, ManipulatorRecord, MechanismKindRecord,
    MechanismRecord, MountedDeviceSpec, PROPELLER_DOCUMENT_ASSET, PartRef,
    PowerGeneratorKindRecord, PowerGeneratorRecord, SCHEMA_VERSION, SloshModeRecord,
    SolarArrayRecord, SolidMotorRecord, SubsystemManifest, SyspackManifest, SystemKind,
    TankGeometryRecord, TankRecord, ThermalLinkRecord, ThermalNodeRecord, TpsShieldRecord,
    TransferLineRecord, WheelRecord,
};
#[cfg(not(target_arch = "wasm32"))]
pub use source::DirectoryPackageSource;
pub use source::{MANIFEST_FILE, MemoryPackageSource, PackageSource};
pub use typeface::{TypefacePin, TypefaceSpec, UpstreamSource};
pub use verified::{INITIAL_COMPONENT_TEMPERATURE_K, VerifiedSyspack, sha256_hex};
