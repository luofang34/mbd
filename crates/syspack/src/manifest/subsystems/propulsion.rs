use mbd_systems::AirbreathingEngineKind;
use serde::{Deserialize, Serialize};

/// Liquid, bipropellant, monopropellant, or cold-gas rocket/RCS thruster record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LiquidRocketRecord {
    /// Component key mounting this thruster.
    pub site: String,
    /// Nozzle throat offset in `site` local axes, m.
    pub mount_offset_local_m: [f64; 3],
    /// Unit thrust vector acting on vehicle in `site` local axes.
    pub thrust_axis_local: [f64; 3],
    /// Maximum 2-axis gimbal angle, rad.
    #[serde(default)]
    pub gimbal_max_rad: f64,
    /// Rated full vacuum thrust, N.
    pub max_vacuum_thrust_n: f64,
    /// Vacuum specific impulse, s.
    pub isp_vac_s: f64,
    /// Sea-level specific impulse, s.
    pub isp_sl_s: f64,
    /// Optional fuel tank id.
    #[serde(default)]
    pub fuel_tank: Option<u16>,
    /// Optional oxidizer tank id.
    #[serde(default)]
    pub oxidizer_tank: Option<u16>,
    /// Oxidizer-to-fuel mixture ratio O/F.
    #[serde(default)]
    pub mixture_ratio_o_f: f64,
    /// Minimum active throttle `[0, 1]`.
    #[serde(default)]
    pub min_throttle: f64,
    /// Startup spool time constant, s.
    #[serde(default)]
    pub startup_tau_s: f64,
    /// Shutdown tail-off time constant, s.
    #[serde(default)]
    pub shutdown_tau_s: f64,
    /// Nominal nozzle/chamber heat rejected into `site`, W.
    #[serde(default)]
    pub nominal_heat_to_site_w: f64,
}

/// Solid rocket motor record (e.g., ejection seat, retro motor, ullage, SRB).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SolidMotorRecord {
    /// Component key mounting this motor.
    pub site: String,
    /// Nozzle offset in `site` axes, m.
    pub mount_offset_local_m: [f64; 3],
    /// Grain center of mass in `site` axes, m.
    pub grain_cg_local_m: [f64; 3],
    /// Unit thrust vector in `site` axes.
    pub thrust_axis_local: [f64; 3],
    /// Initial propellant grain mass, kg.
    pub propellant_mass_kg: f64,
    /// Vacuum specific impulse, s.
    pub isp_vac_s: f64,
    /// Nozzle exit area, m².
    #[serde(default)]
    pub nozzle_exit_area_m2: f64,
    /// Active burn duration, s.
    pub burn_time_s: f64,
    /// Linear tail-off fraction `[0.0, 0.5]`.
    #[serde(default = "default_tail_off")]
    pub tail_off_fraction: f64,
    /// Cylindrical grain radius, m.
    pub grain_radius_m: f64,
    /// Cylindrical grain length, m.
    pub grain_length_m: f64,
}

fn default_tail_off() -> f64 {
    0.1
}

/// Power-limited electric ion / Hall thruster record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ElectricThrusterRecord {
    /// Component key mounting this thruster.
    pub site: String,
    /// Thruster offset in `site` axes, m.
    pub mount_offset_local_m: [f64; 3],
    /// Unit thrust vector in `site` axes.
    pub thrust_axis_local: [f64; 3],
    /// Propellant tank id.
    pub propellant_tank: u16,
    /// Specific impulse, s.
    pub isp_s: f64,
    /// Total electrical-to-jet efficiency `(0, 1]`.
    pub total_efficiency: f64,
    /// Rated maximum electrical power, W.
    pub max_power_w: f64,
    /// Bus shedding priority.
    #[serde(default)]
    pub priority: u8,
}

/// VLEO Atmosphere-Breathing Electric Propulsion (ABEP) intake + thruster record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AbepThrusterRecord {
    /// Component key mounting intake and thruster.
    pub site: String,
    /// Thrust application point in `site` axes, m.
    pub mount_offset_local_m: [f64; 3],
    /// Unit thrust direction in `site` axes.
    pub thrust_axis_local: [f64; 3],
    /// Effective intake capture area, m².
    pub intake_area_m2: f64,
    /// Collection efficiency on incident `AtomicO`.
    pub atomic_o_collection_efficiency: f64,
    /// Collection efficiency on incident `N2`.
    pub n2_collection_efficiency: f64,
    /// Specific impulse, s.
    pub isp_s: f64,
    /// Total efficiency `(0, 1]`.
    pub total_efficiency: f64,
    /// Maximum electrical power input, W.
    pub max_power_w: f64,
    /// Optional buffer tank id (`FluidSpecies::AirCollected`).
    #[serde(default)]
    pub buffer_tank: Option<u16>,
    /// Bus shedding priority.
    #[serde(default)]
    pub priority: u8,
}

/// Airbreathing engine class serialized inside `AirbreathingEngineRecord`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AirbreathingKindRecord {
    /// Subsonic or dry supersonic turbofan/turbojet.
    TurbojetOrTurbofan,
    /// Afterburning supersonic turbojet/turbofan (Concorde, Tu-160).
    AfterburningTurbofan,
    /// Turboprop or helicopter turboshaft.
    TurbopropOrTurboshaft,
    /// Piston engine.
    Piston,
}

impl AirbreathingKindRecord {
    /// Converts to `mbd_systems::AirbreathingEngineKind`.
    pub const fn to_systems(self) -> AirbreathingEngineKind {
        match self {
            Self::TurbojetOrTurbofan => AirbreathingEngineKind::TurbojetOrTurbofan,
            Self::AfterburningTurbofan => AirbreathingEngineKind::AfterburningTurbofan,
            Self::TurbopropOrTurboshaft => AirbreathingEngineKind::TurbopropOrTurboshaft,
            Self::Piston => AirbreathingEngineKind::Piston,
        }
    }
}

/// Airbreathing engine record coupled to `AmbientGas` oxidizer/humidity and fuel tank.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AirbreathingEngineRecord {
    /// Component key mounting this engine.
    pub site: String,
    /// Thrust line point in `site` axes, m.
    pub mount_offset_local_m: [f64; 3],
    /// Unit forward thrust axis in `site` axes.
    pub thrust_axis_local: [f64; 3],
    /// Engine architectural type.
    pub kind: AirbreathingKindRecord,
    /// Sea-level static dry max thrust, N.
    pub sl_static_dry_thrust_n: f64,
    /// Dry TSFC, kg/(N·s).
    pub dry_tsfc_kg_n_s: f64,
    /// Reheat thrust multiplier (`>= 1.0`).
    #[serde(default = "default_one")]
    pub reheat_thrust_multiplier: f64,
    /// Reheat TSFC multiplier (`>= 1.0`).
    #[serde(default = "default_one")]
    pub reheat_tsfc_multiplier: f64,
    /// Max sea-level shaft power, W.
    #[serde(default)]
    pub sl_max_shaft_power_w: f64,
    /// Shaft BSFC, kg/(W·s).
    #[serde(default)]
    pub shaft_bsfc_kg_w_s: f64,
    /// Maximum operating Mach limit.
    pub max_operating_mach: f64,
    /// Density lapse exponent.
    #[serde(default = "default_density_exp")]
    pub density_lapse_exponent: f64,
    /// Minimum ambient oxidizer mass fraction for combustion.
    #[serde(default = "default_min_ox")]
    pub min_oxidizer_mass_fraction: f64,
    /// Fuel tank id feeding this engine.
    pub fuel_tank: u16,
    /// Spool time constant, s.
    #[serde(default)]
    pub spool_tau_s: f64,
}

fn default_one() -> f64 {
    1.0
}

fn default_density_exp() -> f64 {
    0.75
}

fn default_min_ox() -> f64 {
    0.10
}
