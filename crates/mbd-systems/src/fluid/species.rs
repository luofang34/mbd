/// Chemical fluid species held in tanks and transfer networks.
///
/// Covers aviation fuels, cryogenic and hypergolic rocket propellants,
/// cold-gas RCS fluids, electric-propulsion noble gases, atmospheric
/// collected gas (ABEP), water, and movable liquid ballast.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FluidSpecies {
    /// Aviation turbine kerosene (Jet A / Jet A-1 / JP-8).
    JetA1,
    /// Aviation gasoline (AvGas 100LL).
    AvGas100Ll,
    /// Refined rocket kerosene (RP-1 / RG-1).
    Rp1,
    /// Liquid hydrogen ($\text{LH}_2$).
    LiquidHydrogen,
    /// Liquid oxygen ($\text{LOX}$).
    LiquidOxygen,
    /// Liquid methane ($\text{LCH}_4$).
    LiquidMethane,
    /// Hydrazine monopropellant / bipropellant ($\text{N}_2\text{H}_4$).
    Hydrazine,
    /// Unsymmetrical dimethylhydrazine (UDMH / heptyl).
    Udmh,
    /// Monomethylhydrazine (MMH, e.g. Gemini OAMS / Apollo SPS).
    Mmh,
    /// Dinitrogen tetroxide ($\text{N}_2\text{O}_4$ / NTO).
    N2O4,
    /// Mixed oxides of nitrogen ($\text{MON}-3$).
    Mon3,
    /// High-test hydrogen peroxide ($\text{H}_2\text{O}_2$).
    HydrogenPeroxide,
    /// Compressed/supercritical xenon ($\text{Xe}$) for ion/Hall thrusters.
    Xenon,
    /// Compressed/supercritical krypton ($\text{Kr}$).
    Krypton,
    /// Cold-gas nitrogen ($\text{N}_2$).
    ColdGasNitrogen,
    /// Pressurant helium ($\text{He}$).
    ColdGasHelium,
    /// Collected atmospheric air/thermospheric gas (ABEP).
    AirCollected,
    /// Liquid water (cooling/potable/ballast).
    Water,
    /// Generic high-density trim or ballast fluid.
    BallastFluid,
}

impl FluidSpecies {
    /// Default nominal bulk storage density, kg/m³.
    ///
    /// Pressurized gas tanks may override this via [`super::TankSpec::fluid_density_kg_m3`]
    /// to match tank pressure and temperature.
    pub const fn nominal_density_kg_m3(self) -> f64 {
        match self {
            Self::JetA1 => 804.0,
            Self::AvGas100Ll => 715.0,
            Self::Rp1 => 810.0,
            Self::LiquidHydrogen => 70.85,
            Self::LiquidOxygen => 1141.0,
            Self::LiquidMethane => 422.6,
            Self::Hydrazine => 1004.0,
            Self::Udmh => 791.0,
            Self::Mmh => 878.0,
            Self::N2O4 => 1442.0,
            Self::Mon3 => 1440.0,
            Self::HydrogenPeroxide => 1450.0,
            Self::Xenon => 1600.0,
            Self::Krypton => 850.0,
            Self::ColdGasNitrogen => 280.0,
            Self::ColdGasHelium => 45.0,
            Self::AirCollected => 300.0,
            Self::Water => 998.0,
            Self::BallastFluid => 1500.0,
        }
    }

    /// Whether the species is stored cryogenically and exhibits boil-off under heat leak.
    pub const fn is_cryogenic(self) -> bool {
        matches!(
            self,
            Self::LiquidHydrogen | Self::LiquidOxygen | Self::LiquidMethane
        )
    }

    /// Latent heat of vaporization for cryogenic boil-off estimation, J/kg (`0.0` if non-cryogenic).
    pub const fn latent_heat_vaporization_j_kg(self) -> f64 {
        match self {
            Self::LiquidHydrogen => 445_600.0,
            Self::LiquidOxygen => 213_000.0,
            Self::LiquidMethane => 510_000.0,
            Self::ColdGasNitrogen => 199_200.0,
            Self::ColdGasHelium => 20_700.0,
            _ => 0.0,
        }
    }

    /// Nominal boiling temperature at 1 atm, K (`None` for non-cryogenic or supercritical storage).
    pub const fn normal_boiling_point_k(self) -> Option<f64> {
        match self {
            Self::LiquidHydrogen => Some(20.28),
            Self::LiquidOxygen => Some(90.19),
            Self::LiquidMethane => Some(111.66),
            _ => None,
        }
    }

    /// Whether this species is a liquid hypergolic oxidizer.
    pub const fn is_hypergolic_oxidizer(self) -> bool {
        matches!(self, Self::N2O4 | Self::Mon3 | Self::HydrogenPeroxide)
    }

    /// Whether this species is a liquid hypergolic fuel.
    pub const fn is_hypergolic_fuel(self) -> bool {
        matches!(self, Self::Hydrazine | Self::Udmh | Self::Mmh)
    }

    /// Approximate molar mass, kg/mol.
    pub const fn molar_mass_kg_mol(self) -> f64 {
        match self {
            Self::JetA1 => 0.168,
            Self::AvGas100Ll => 0.114,
            Self::Rp1 => 0.172,
            Self::LiquidHydrogen => 0.002_015_88,
            Self::LiquidOxygen => 0.031_998_8,
            Self::LiquidMethane => 0.016_042_5,
            Self::Hydrazine => 0.032_045_2,
            Self::Udmh => 0.060_098_6,
            Self::Mmh => 0.046_072,
            Self::N2O4 => 0.092_011,
            Self::Mon3 => 0.090_5,
            Self::HydrogenPeroxide => 0.034_014_7,
            Self::Xenon => 0.131_293,
            Self::Krypton => 0.083_798,
            Self::ColdGasNitrogen => 0.028_013_4,
            Self::ColdGasHelium => 0.004_002_6,
            Self::AirCollected => 0.028_965,
            Self::Water => 0.018_015_28,
            Self::BallastFluid => 0.100,
        }
    }
}
