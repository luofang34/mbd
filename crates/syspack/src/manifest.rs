use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::SyspackError;

pub(crate) mod validate;

#[cfg(test)]
mod tests;

/// The only `schema_version` this build reads.
pub const SCHEMA_VERSION: u32 = 1;

/// What kind of system a package holds. Vehicles and stationary systems share
/// one schema; [`DynamicsSpec::None`] marks a system that does not move.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SystemKind {
    /// An aeroplane.
    FixedWing,
    /// A helicopter or other aircraft lifted by one main rotor.
    Rotorcraft,
    /// A rotorcraft with several lifting rotors.
    Multirotor,
    /// A powered lunar lander.
    LunarLander,
    /// A capsule built for atmospheric entry.
    ReentryCapsule,
    /// Any other spacecraft.
    Spacecraft,
    /// A fixed installation operating vehicles remotely.
    GroundStation,
    /// A vehicle that drives or sails on a surface.
    SurfaceVehicle,
    /// Equipment under test on a bench, such as a circuit board or a rig.
    TestBench,
    /// Anything else with components and devices.
    GenericSystem,
    /// A shareable set of cockpit instruments, described by an
    /// `instruments.json` asset, that vehicles pin with an
    /// [`InstrumentPackRef`]. It does not move.
    InstrumentPack,
    /// A lettering face and its provenance, described by the manifest's
    /// [`crate::TypefaceSpec`], that instrument packs pin with a
    /// [`crate::TypefacePin`]. It has no components and does not move.
    TypefacePack,
}

/// The expected content of one asset file.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AssetDigest {
    /// SHA-256 of the file, hexadecimal.
    pub sha256: String,
    /// Length of the file, bytes.
    pub size_bytes: u64,
}

fn identity_rotation() -> [f64; 4] {
    [0.0, 0.0, 0.0, 1.0]
}

/// One component of the system's [`mbd_core::ComponentTree`]. Its
/// [`mbd_core::ComponentId`] is its position in
/// [`SyspackManifest::components`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentRecord {
    /// Symbolic name other records refer to.
    pub key: String,
    /// Human-readable name.
    pub name: String,
    /// Key of the parent component; `None` places it in the root frame.
    pub parent: Option<String>,
    /// Component origin in parent axes, metres.
    pub translation_m: [f64; 3],
    /// Component-to-parent rotation as `[x, y, z, w]`.
    #[serde(default = "identity_rotation")]
    pub rotation_xyzw: [f64; 4],
    /// Structural mass, kg.
    pub dry_mass_kg: f64,
    /// Symmetric structural inertia about the origin in component axes, kg·m².
    pub dry_inertia_body: [[f64; 3]; 3],
    /// Fluid loaded at the start, kg.
    pub initial_fluid_mass_kg: f64,
    /// Payload loaded at the start, kg.
    pub initial_payload_mass_kg: f64,
    /// Whether the component starts joined to its parent.
    pub attached: bool,
    /// Lumped heat capacity, J/K.
    pub thermal_capacity_j_k: f64,
    /// Nominal electrical draw, W; negative for a source.
    pub nominal_power_w: f64,
}

/// How the system moves.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum DynamicsSpec {
    /// It does not: a ground station, a bench, or a circuit board.
    None,
    /// A JSBSim aircraft read entirely from verified package assets.
    JsbsimXml {
        /// Directory of aircraft inside the resolver.
        aircraft_dir: String,
        /// The aircraft's model name.
        model_name: String,
        /// Package asset holding the aircraft document.
        entry_asset: String,
        /// Where JSBSim reads `entry_asset`: always
        /// `<aircraft_dir>/<model_name>/<model_name>.xml`.
        virtual_path: String,
        /// Further documents as `(package asset, JSBSim virtual path)`.
        additional_assets: Vec<(String, String)>,
    },
    /// Rigid-body flight with no aerodynamics, as [`mbd_core::VacuumAero`].
    RigidBodyVacuum,
    /// Rigid-body flight on a [`mbd_core::BluntBodyCapsuleAero`].
    RigidBodyBluntCapsule {
        /// Maximum heat-shield radius, metres.
        base_radius_m: f64,
        /// Spherical heat-shield radius, metres.
        nose_radius_m: f64,
        /// Heat shield's centre of curvature, body axes, metres.
        reference_point_body_m: [f64; 3],
        /// `s` in `CN = s sin α cos α`.
        normal_force_slope: f64,
        /// `CA0` in `CA = CA0 cos² α`.
        axial_drag_coeff: f64,
        /// Pitch-damping derivative `Cmq`; negative damps.
        pitch_damping_coeff: f64,
        /// Sutton-Graves heating constant.
        sutton_graves_k: f64,
    },
}

impl DynamicsSpec {
    /// The manifest name of this dynamics kind.
    pub fn kind(&self) -> &'static str {
        match self {
            Self::None => "None",
            Self::JsbsimXml { .. } => "JsbsimXml",
            Self::RigidBodyVacuum => "RigidBodyVacuum",
            Self::RigidBodyBluntCapsule { .. } => "RigidBodyBluntCapsule",
        }
    }
}

/// Which operator controls the system has. A piston aeroplane declares its
/// magnetos, starter and mixture; an electric trainer, a lunar lander or a
/// ground station omits what it lacks, so an operator panel can be built from
/// the manifest alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ControlsSpec {
    /// An ignition switch selecting left, right or both magnetos.
    pub has_magnetos: bool,
    /// An engine starter.
    pub has_starter: bool,
    /// A mixture control.
    pub has_mixture: bool,
    /// A propeller pitch control.
    pub has_prop_pitch: bool,
    /// A way to cut the fuel to the engine.
    pub has_fuel_cutoff: bool,
    /// Wing flaps.
    pub has_flaps: bool,
    /// Landing gear the operator can raise.
    pub has_retractable_gear: bool,
    /// Wheel brakes.
    pub has_wheel_brakes: bool,
}

/// What a mounted device is.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum DeviceKindSpec {
    /// TDK ICM-42688 IMU at its datasheet noise.
    Icm42688 {
        /// Seed of the noise stream.
        seed: u64,
    },
    /// Bosch BMP390 barometer.
    Bmp390 {
        /// I2C bus instance.
        bus_id: u8,
        /// 7-bit I2C address.
        i2c_addr: u8,
    },
    /// One ARINC 429 channel. Channel numbers follow manifest order.
    Arinc429Channel {
        /// 100 kbps rather than 12.5 kbps.
        high_speed: bool,
        /// Idle bits between words.
        gap_bits: u32,
    },
    /// A pinhole camera.
    PinholeCamera {
        /// Camera position in root axes, metres.
        offset_body_m: [f64; 3],
        /// Body-to-camera rotation as `[x, y, z, w]`, with camera `+X` right,
        /// `+Y` up, looking along `-Z`.
        body_to_camera_xyzw: [f64; 4],
        /// Image width, pixels.
        width: u32,
        /// Image height, pixels.
        height: u32,
        /// Horizontal focal length, pixels.
        fx: f64,
        /// Vertical focal length, pixels.
        fy: f64,
        /// Principal point, horizontal, pixels.
        cx: f64,
        /// Principal point, vertical, pixels.
        cy: f64,
        /// Capture rate, which must divide the master clock, Hz.
        rate_hz: u64,
    },
    /// A pitot-static system: a total-pressure probe and static ports, each
    /// feeding the instruments through a pneumatic line.
    PitotStatic {
        /// Pitot probe tip position in root axes, metres.
        pitot_body_m: [f64; 3],
        /// Static port position in root axes, metres.
        static_body_m: [f64; 3],
        /// The probe installation, such as `kollsman_d2_pitot_static_head`
        /// or `boom_pitot_fuselage_static`, for consumers that model its
        /// position error or icing.
        probe: String,
        /// Time constant of the total-pressure line to the instruments, s.
        total_line_tau_s: f64,
        /// Time constant of the static line to the instruments, s.
        static_line_tau_s: f64,
    },
    /// A device this crate cannot build: the crate that claims `type_id`
    /// interprets `parameters`.
    Custom {
        /// Namespaced device type, such as `apollo.pgncs_imu`.
        type_id: String,
        /// Parameters in the device type's own schema.
        parameters: serde_json::Value,
    },
}

/// A device and where it is mounted.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MountedDeviceSpec {
    /// Unique name of the device in the system.
    pub id: String,
    /// Key of the component it is mounted on.
    pub site: String,
    /// What the device is.
    pub kind: DeviceKindSpec,
}

/// A vehicle's pin on a shared instrument pack. The vehicle's manifest is
/// the trust root, so naming the pack's exact `syspack.json` digest makes the
/// pack as trusted as the vehicle's own assets.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InstrumentPackRef {
    /// The pack's package id.
    pub id: String,
    /// The pack's package version.
    pub version: String,
    /// SHA-256 of the pack's `syspack.json` bytes, hexadecimal.
    pub manifest_sha256: String,
}

/// The content of a system's `syspack.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SyspackManifest {
    /// Manifest format; must be [`SCHEMA_VERSION`].
    pub schema_version: u32,
    /// Lowercase slug of the package.
    pub id: String,
    /// Human-readable name.
    pub name: String,
    /// Version of the package's content.
    pub version: String,
    /// What kind of system this is.
    pub kind: SystemKind,
    /// Every file of the package by relative POSIX path.
    pub assets: BTreeMap<String, AssetDigest>,
    /// The component tree, in [`mbd_core::ComponentId`] order.
    pub components: Vec<ComponentRecord>,
    /// How the system moves.
    pub dynamics: DynamicsSpec,
    /// The operator controls it has; none when omitted.
    #[serde(default)]
    pub controls: ControlsSpec,
    /// Everything mounted on the system.
    pub devices: Vec<MountedDeviceSpec>,
    /// The shared instrument pack the system's cockpit uses, if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub instrument_pack: Option<InstrumentPackRef>,
    /// What a `TypefacePack` holds; nothing for any other kind.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub typeface: Option<crate::TypefaceSpec>,
}

impl SyspackManifest {
    /// Parses and validates `syspack.json` content. Asset contents are not
    /// read here; [`crate::VerifiedSyspack::load`] verifies them.
    pub fn parse(bytes: &[u8]) -> Result<Self, SyspackError> {
        let manifest: Self = serde_json::from_slice(bytes).map_err(SyspackError::ManifestParse)?;
        manifest.validate()?;
        Ok(manifest)
    }
}
