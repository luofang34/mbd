use serde::{Deserialize, Serialize};

/// Photovoltaic wing/panel definition serialized inside `SubsystemManifest`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SolarArrayRecord {
    /// Host component key.
    pub site: String,
    /// Gross photovoltaic cell area, m².
    pub area_m2: f64,
    /// Cell outward normal in host local frame.
    pub normal_local: [f64; 3],
    /// BOL conversion efficiency `(0..=1)`.
    pub efficiency: f64,
    /// Active packing factor `(0..=1)`.
    pub packing_factor: f64,
    /// Annual radiation degradation `[0..=0.5]`.
    #[serde(default)]
    pub annual_degradation: f64,
    /// Nominal operating bus voltage, V.
    pub nominal_voltage_v: f64,
    /// Initial deployment fraction `[0..=1]`.
    #[serde(default = "default_one")]
    pub initial_deployed_fraction: f64,
    /// Mission age at package start, years.
    #[serde(default)]
    pub initial_age_years: f64,
}

fn default_one() -> f64 {
    1.0
}

/// Primary or rechargeable electrochemical battery pack record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BatteryRecord {
    /// Host component key.
    pub site: String,
    /// Total nameplate energy capacity, Wh.
    pub capacity_wh: f64,
    /// Initial stored energy, Wh (`None` starts full at `capacity_wh`).
    #[serde(default)]
    pub initial_energy_wh: Option<f64>,
    /// Open-circuit nominal voltage, V.
    pub nominal_voltage_v: f64,
    /// Maximum charge power, W (`0.0` for primary batteries).
    #[serde(default)]
    pub max_charge_w: f64,
    /// Maximum discharge power, W.
    pub max_discharge_w: f64,
    /// Charge efficiency `(0, 1]`.
    #[serde(default = "default_eff")]
    pub charge_efficiency: f64,
    /// Discharge efficiency `(0, 1]`.
    #[serde(default = "default_eff")]
    pub discharge_efficiency: f64,
    /// Minimum protected SOC fraction `[0, 1)`.
    #[serde(default)]
    pub min_soc: f64,
    /// Equivalent series resistance, Ohm.
    #[serde(default)]
    pub internal_resistance_ohm: f64,
}

fn default_eff() -> f64 {
    0.95
}

/// Non-solar generator type serialized inside `PowerGeneratorRecord`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum PowerGeneratorKindRecord {
    /// Radioisotope Thermoelectric Generator (RTG / MMRTG).
    Rtg {
        /// Beginning-of-life electrical output, W.
        bol_electrical_w: f64,
        /// Beginning-of-life thermal output, W.
        bol_thermal_w: f64,
        /// Isotope half-life, years.
        half_life_years: f64,
    },
    /// Hydrogen/oxygen fuel cell.
    FuelCell {
        /// Rated continuous power, W.
        max_power_w: f64,
        /// Waste thermal heat ratio relative to electrical output.
        waste_heat_ratio: f64,
        /// Reactant mass consumption rate, kg/Wh.
        specific_reactant_kg_wh: f64,
        /// Optional H2 reactant tank id.
        #[serde(default)]
        h2_tank: Option<u16>,
        /// Optional O2 reactant tank id.
        #[serde(default)]
        o2_tank: Option<u16>,
    },
    /// Engine shaft-driven alternator / IDG.
    ShaftAlternator {
        /// Rated max output, W.
        max_power_w: f64,
        /// Minimum gearbox shaft power, W.
        min_shaft_power_w: f64,
        /// Mechanical-to-electrical efficiency.
        efficiency: f64,
    },
}

/// Power generator record mounted on a host component.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PowerGeneratorRecord {
    /// Host component key.
    pub site: String,
    /// Physical generator model.
    pub kind: PowerGeneratorKindRecord,
    /// Nominal output voltage, V.
    pub nominal_voltage_v: f64,
    /// Initial enabled breaker state.
    #[serde(default = "default_true")]
    pub enabled: bool,
}

fn default_true() -> bool {
    true
}

/// Thermal capacitance node bound to a component key.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThermalNodeRecord {
    /// Component key represented by this thermal node.
    pub site: String,
    /// Lumped heat capacity $m c_p$, J/K.
    pub heat_capacity_j_k: f64,
    /// Initial node temperature, K.
    #[serde(default = "default_temp_k")]
    pub initial_temp_k: f64,
    /// Radiative external area, m².
    #[serde(default)]
    pub radiative_area_m2: f64,
    /// Surface IR emissivity `[0, 1]`.
    #[serde(default)]
    pub emissivity: f64,
    /// Solar optical absorptivity `[0, 1]`.
    #[serde(default)]
    pub solar_absorptivity: f64,
    /// Projected solar illumination area, m².
    #[serde(default)]
    pub solar_area_m2: f64,
    /// Atmospheric convective wetted area, m².
    #[serde(default)]
    pub convective_area_m2: f64,
    /// Minimum safe temperature, K.
    #[serde(default = "default_min_temp_k")]
    pub min_operating_temp_k: f64,
    /// Maximum safe temperature, K.
    #[serde(default = "default_max_temp_k")]
    pub max_operating_temp_k: f64,
}

fn default_temp_k() -> f64 {
    293.15
}

fn default_min_temp_k() -> f64 {
    200.0
}

fn default_max_temp_k() -> f64 {
    400.0
}

/// Conductive or pumped-coolant thermal link between two component keys.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ThermalLinkRecord {
    /// First endpoint component key.
    pub node_a: String,
    /// Second endpoint component key.
    pub node_b: String,
    /// Thermal conductance, W/K.
    pub conductance_w_k: f64,
    /// Whether this link models an actively pumped coolant loop.
    #[serde(default)]
    pub pumped_loop: bool,
    /// Pump electrical draw when active, W.
    #[serde(default)]
    pub pump_power_w: f64,
    /// Whether link stops on component detachment/staging.
    #[serde(default = "default_true")]
    pub requires_attachment: bool,
    /// Shedding priority.
    #[serde(default)]
    pub priority: u8,
}

/// Ablative Thermal Protection System (TPS) heatshield record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TpsShieldRecord {
    /// Heatshield component key.
    pub site: String,
    /// Effective nose stagnation radius, m.
    pub nose_radius_m: f64,
    /// Exposed ablation area, m².
    pub shield_area_m2: f64,
    /// Ablating char surface temperature, K.
    pub char_surface_temp_k: f64,
    /// Surface IR emissivity `[0, 1]`.
    pub emissivity: f64,
    /// Thermochemical heat of ablation, J/kg.
    pub heat_of_ablation_j_kg: f64,
    /// Residual backface soak fraction `[0, 1]`.
    pub backface_soak_fraction: f64,
    /// Initial ablator mass, kg.
    pub initial_ablator_mass_kg: f64,
    /// Radiative heating scale coefficient.
    #[serde(default)]
    pub radiative_heating_coeff: f64,
}

/// VLEO atomic-oxygen surface coating erosion record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AoCoatingRecord {
    /// Exposed component key.
    pub site: String,
    /// Exposed surface area, m².
    pub area_m2: f64,
    /// Volumetric erosion yield per incident oxygen atom, m³/atom.
    pub erosion_yield_m3_per_atom: f64,
    /// Coating bulk density, kg/m³.
    pub coating_density_kg_m3: f64,
    /// Initial coating thickness, m.
    pub initial_thickness_m: f64,
}
