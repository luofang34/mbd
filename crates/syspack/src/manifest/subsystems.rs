mod fluid;
mod mechanisms;
mod power_thermal;
mod propulsion;
mod validate;

pub use fluid::{
    FeedRegimeRecord, FluidSpeciesRecord, SloshModeRecord, TankGeometryRecord, TankRecord,
    TransferLineRecord,
};
pub use mechanisms::{ManipulatorRecord, MechanismKindRecord, MechanismRecord, WheelRecord};
pub use power_thermal::{
    AoCoatingRecord, BatteryRecord, PowerGeneratorKindRecord, PowerGeneratorRecord,
    SolarArrayRecord, ThermalLinkRecord, ThermalNodeRecord, TpsShieldRecord,
};
pub use propulsion::{
    AbepThrusterRecord, AirbreathingEngineRecord, AirbreathingKindRecord, ElectricThrusterRecord,
    LiquidRocketRecord, SolidMotorRecord,
};

use serde::{Deserialize, Serialize};

/// Declarative physical subsystems hosted on a vehicle's [`mbd_core::ComponentTree`].
///
/// Empty by default in `schema_version = 1` packages; populated in `schema_version = 2`
/// packages for full multi-tank fluid/trim, propulsion, electrical, thermal, TPS,
/// mechanism, robotic manipulator, and wheel simulation.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SubsystemManifest {
    /// Nominal DC electrical bus voltage setpoint, V (`None` when no electrical bus is modeled).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nominal_bus_voltage_v: Option<f64>,
    /// Fluid tanks (fuel, oxidizer, pressurant, xenon, water, ballast).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tanks: Vec<TankRecord>,
    /// Inter-tank crossfeed and CG trim transfer lines.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub transfer_lines: Vec<TransferLineRecord>,
    /// Liquid, hypergolic, monopropellant, or cold-gas thrusters and main engines.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub liquid_rockets: Vec<LiquidRocketRecord>,
    /// Solid rocket motors (ejection, retro, ullage, SRBs).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub solid_motors: Vec<SolidMotorRecord>,
    /// Power-limited electric ion and Hall thrusters.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub electric_thrusters: Vec<ElectricThrusterRecord>,
    /// VLEO Atmosphere-Breathing Electric Propulsion (ABEP) units.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub abep_thrusters: Vec<AbepThrusterRecord>,
    /// Airbreathing jet, afterburning, turboprop, or piston engines.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub airbreathing_engines: Vec<AirbreathingEngineRecord>,
    /// Solar photovoltaic wings/panels.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub solar_arrays: Vec<SolarArrayRecord>,
    /// Electrochemical batteries (primary or rechargeable).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub batteries: Vec<BatteryRecord>,
    /// RTG, fuel cell, or shaft alternator power generators.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub generators: Vec<PowerGeneratorRecord>,
    /// Thermal lumped-capacitance nodes.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub thermal_nodes: Vec<ThermalNodeRecord>,
    /// Thermal conductive or pumped-coolant exchange links.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub thermal_links: Vec<ThermalLinkRecord>,
    /// Ablative TPS entry heatshields.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tps_shields: Vec<TpsShieldRecord>,
    /// VLEO atomic-oxygen exposed surface coatings.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub ao_coatings: Vec<AoCoatingRecord>,
    /// Discrete / compliant mechanisms (staging, docking, rail, hinge, winch, grapple).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub mechanisms: Vec<MechanismRecord>,
    /// Articulated robotic manipulators mapped onto component joints.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub manipulators: Vec<ManipulatorRecord>,
    /// Driven, braked, and steered wheels.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub wheels: Vec<WheelRecord>,
    /// Relative path to an optional JSON `mbd_provenance::SourceTable` declared in `assets`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub provenance_asset: Option<String>,
}

impl SubsystemManifest {
    /// Whether no subsystem, mechanism, manipulator, wheel, or provenance entry is declared.
    pub fn is_empty(&self) -> bool {
        self.nominal_bus_voltage_v.is_none()
            && self.tanks.is_empty()
            && self.transfer_lines.is_empty()
            && self.liquid_rockets.is_empty()
            && self.solid_motors.is_empty()
            && self.electric_thrusters.is_empty()
            && self.abep_thrusters.is_empty()
            && self.airbreathing_engines.is_empty()
            && self.solar_arrays.is_empty()
            && self.batteries.is_empty()
            && self.generators.is_empty()
            && self.thermal_nodes.is_empty()
            && self.thermal_links.is_empty()
            && self.tps_shields.is_empty()
            && self.ao_coatings.is_empty()
            && self.mechanisms.is_empty()
            && self.manipulators.is_empty()
            && self.wheels.is_empty()
            && self.provenance_asset.is_none()
    }
}
