use mbd_systems::{FeedRegime, FluidSpecies, SloshModeSpec, TankGeometry};
use serde::{Deserialize, Serialize};

/// Fluid chemical species serialized inside a `syspack.json` manifest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FluidSpeciesRecord {
    /// Aviation turbine kerosene (Jet-A / JP-8).
    JetA1,
    /// Aviation gasoline 100LL.
    AvGas100Ll,
    /// Rocket kerosene RP-1.
    Rp1,
    /// Liquid hydrogen.
    LiquidHydrogen,
    /// Liquid oxygen.
    LiquidOxygen,
    /// Liquid methane.
    LiquidMethane,
    /// Hydrazine.
    Hydrazine,
    /// Unsymmetrical dimethylhydrazine.
    Udmh,
    /// Monomethylhydrazine.
    Mmh,
    /// Dinitrogen tetroxide.
    N2O4,
    /// MON-3 oxidizer.
    Mon3,
    /// Hydrogen peroxide.
    HydrogenPeroxide,
    /// Compressed/supercritical xenon.
    Xenon,
    /// Compressed/supercritical krypton.
    Krypton,
    /// Cold-gas nitrogen.
    ColdGasNitrogen,
    /// Pressurant helium.
    ColdGasHelium,
    /// ABEP collected atmospheric air.
    AirCollected,
    /// Liquid water.
    Water,
    /// Dense trim/ballast fluid.
    BallastFluid,
}

impl FluidSpeciesRecord {
    /// Maps this manifest species record to `mbd_systems::FluidSpecies`.
    pub const fn to_systems(self) -> FluidSpecies {
        match self {
            Self::JetA1 => FluidSpecies::JetA1,
            Self::AvGas100Ll => FluidSpecies::AvGas100Ll,
            Self::Rp1 => FluidSpecies::Rp1,
            Self::LiquidHydrogen => FluidSpecies::LiquidHydrogen,
            Self::LiquidOxygen => FluidSpecies::LiquidOxygen,
            Self::LiquidMethane => FluidSpecies::LiquidMethane,
            Self::Hydrazine => FluidSpecies::Hydrazine,
            Self::Udmh => FluidSpecies::Udmh,
            Self::Mmh => FluidSpecies::Mmh,
            Self::N2O4 => FluidSpecies::N2O4,
            Self::Mon3 => FluidSpecies::Mon3,
            Self::HydrogenPeroxide => FluidSpecies::HydrogenPeroxide,
            Self::Xenon => FluidSpecies::Xenon,
            Self::Krypton => FluidSpecies::Krypton,
            Self::ColdGasNitrogen => FluidSpecies::ColdGasNitrogen,
            Self::ColdGasHelium => FluidSpecies::ColdGasHelium,
            Self::AirCollected => FluidSpecies::AirCollected,
            Self::Water => FluidSpecies::Water,
            Self::BallastFluid => FluidSpecies::BallastFluid,
        }
    }
}

/// Serialized internal geometry of a fluid tank.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum TankGeometryRecord {
    /// Sphere tank.
    Sphere {
        /// Radius, m.
        radius_m: f64,
    },
    /// Right circular cylinder tank.
    Cylinder {
        /// Radius, m.
        radius_m: f64,
        /// Length, m.
        length_m: f64,
        /// Symmetry axis in host component frame.
        axis_local: [f64; 3],
    },
    /// Ellipsoid tank.
    Ellipsoid {
        /// Positive semi-axes `(a, b, c)`, m.
        semi_axes_m: [f64; 3],
    },
    /// Rectangular box tank.
    Box {
        /// Positive half-extents `(hx, hy, hz)`, m.
        half_extents_m: [f64; 3],
    },
}

impl TankGeometryRecord {
    /// Converts to `mbd_systems::TankGeometry`.
    pub const fn to_systems(self) -> TankGeometry {
        match self {
            Self::Sphere { radius_m } => TankGeometry::Sphere { radius_m },
            Self::Cylinder {
                radius_m,
                length_m,
                axis_local,
            } => TankGeometry::Cylinder {
                radius_m,
                length_m,
                axis_local: glam::DVec3::from_array(axis_local),
            },
            Self::Ellipsoid { semi_axes_m } => TankGeometry::Ellipsoid {
                semi_axes_m: glam::DVec3::from_array(semi_axes_m),
            },
            Self::Box { half_extents_m } => TankGeometry::Box {
                half_extents_m: glam::DVec3::from_array(half_extents_m),
            },
        }
    }
}

/// Expulsion / settling law serialized on a tank record.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum FeedRegimeRecord {
    /// Positive expulsion diaphragm or bladder.
    BladderOrDiaphragm,
    /// Capillary surface-tension PMD.
    CapillaryPmd {
        /// Pickup axis direction in local frame.
        pickup_axis_local: [f64; 3],
        /// Minimum residual acceleration along pickup, m/s².
        min_settling_accel_m_s2: f64,
    },
    /// Free-surface acceleration-settled tank (wing tanks or cryo ullage stages).
    Settled {
        /// Unit pickup axis in local frame.
        pickup_axis_local: [f64; 3],
        /// Minimum settling acceleration, m/s².
        min_settling_accel_m_s2: f64,
        /// Dwell duration above threshold before opening, s.
        settle_time_s: f64,
    },
    /// Baffled negative-G collector / header tank.
    HeaderTank,
}

impl FeedRegimeRecord {
    /// Converts to `mbd_systems::FeedRegime`.
    pub const fn to_systems(self) -> FeedRegime {
        match self {
            Self::BladderOrDiaphragm => FeedRegime::BladderOrDiaphragm,
            Self::CapillaryPmd {
                pickup_axis_local,
                min_settling_accel_m_s2,
            } => FeedRegime::CapillaryPmd {
                pickup_axis_local: glam::DVec3::from_array(pickup_axis_local),
                min_settling_accel_m_s2,
            },
            Self::Settled {
                pickup_axis_local,
                min_settling_accel_m_s2,
                settle_time_s,
            } => FeedRegime::Settled {
                pickup_axis_local: glam::DVec3::from_array(pickup_axis_local),
                min_settling_accel_m_s2,
                settle_time_s,
            },
            Self::HeaderTank => FeedRegime::HeaderTank,
        }
    }
}

/// Optional liquid slosh mode parameters.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SloshModeRecord {
    /// Fraction of liquid mass participating in slosh.
    pub mass_fraction: f64,
    /// 1-g natural frequency, Hz.
    pub frequency_1g_hz: f64,
    /// Viscous damping ratio.
    pub damping_ratio: f64,
    /// Maximum lateral displacement amplitude, m.
    pub max_displacement_m: f64,
}

impl SloshModeRecord {
    /// Converts to `mbd_systems::SloshModeSpec`.
    pub const fn to_systems(self) -> SloshModeSpec {
        SloshModeSpec {
            mass_fraction: self.mass_fraction,
            frequency_1g_hz: self.frequency_1g_hz,
            damping_ratio: self.damping_ratio,
            max_displacement_m: self.max_displacement_m,
        }
    }
}

/// Serialized fluid tank specification inside `SubsystemManifest::tanks`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TankRecord {
    /// Numeric tank id unique within this manifest.
    pub id: u16,
    /// Host component key in `SyspackManifest::components`.
    pub site: String,
    /// Tank center position in host component local frame, m.
    pub center_local_m: [f64; 3],
    /// Geometric tank shape and orientation.
    pub geometry: TankGeometryRecord,
    /// Fluid species.
    pub species: FluidSpeciesRecord,
    /// Fluid density, kg/m³ (`None` uses nominal density of `species`).
    #[serde(default)]
    pub fluid_density_kg_m3: Option<f64>,
    /// Maximum capacity, kg.
    pub capacity_mass_kg: f64,
    /// Initial loaded fluid mass, kg.
    pub initial_mass_kg: f64,
    /// Unusable trapped residual mass, kg.
    #[serde(default)]
    pub minimum_unusable_kg: f64,
    /// Initial internal tank pressure, Pa.
    #[serde(default = "default_pressure_pa")]
    pub initial_pressure_pa: f64,
    /// Initial fluid temperature, K.
    #[serde(default = "default_temp_k")]
    pub initial_temperature_k: f64,
    /// Expulsion / settling regime.
    pub feed_regime: FeedRegimeRecord,
    /// Optional slosh pendulum parameters.
    #[serde(default)]
    pub slosh: Option<SloshModeRecord>,
    /// Passive environmental heat leak, W.
    #[serde(default)]
    pub passive_heat_leak_w: f64,
}

fn default_pressure_pa() -> f64 {
    200_000.0
}

fn default_temp_k() -> f64 {
    293.15
}

/// Inter-tank fluid transfer line / boost pump specification.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TransferLineRecord {
    /// Numeric line id.
    pub id: u16,
    /// Nominal source tank id.
    pub from_tank: u16,
    /// Nominal destination tank id.
    pub to_tank: u16,
    /// Maximum pump/valve mass flow, kg/s.
    pub max_flow_rate_kg_s: f64,
    /// Electrical power required per unit mass flow, W/(kg/s).
    #[serde(default)]
    pub power_w_per_kg_s: f64,
    /// Whether flow can reverse.
    #[serde(default)]
    pub bidirectional: bool,
    /// Whether both endpoint components must remain attached in `ComponentTree`.
    #[serde(default = "default_true")]
    pub requires_attachment: bool,
    /// Shedding priority (`0` = critical).
    #[serde(default)]
    pub priority: u8,
}

fn default_true() -> bool {
    true
}
