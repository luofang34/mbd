use super::{AVOGADRO_CONSTANT_MOL, SUTTON_GRAVES_K_MARS};

/// Chemical species supported in a planetary or atmospheric gas mixture.
///
/// The thermophysical constants allow aerodynamic, airbreathing propulsion,
/// thermospheric VLEO/ABEP (including atomic oxygen `O`), and entry heating
/// models to share one unambiguous description of ambient gas.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GasSpecies {
    /// Molecular nitrogen ($\text{N}_2$).
    N2,
    /// Molecular oxygen ($\text{O}_2$).
    O2,
    /// Atomic oxygen ($\text{O}$), dominant in Earth VLEO/LEO thermosphere.
    AtomicO,
    /// Argon ($\text{Ar}$).
    Ar,
    /// Carbon dioxide ($\text{CO}_2$).
    CO2,
    /// Carbon monoxide ($\text{CO}$).
    CO,
    /// Water vapour ($\text{H}_2\text{O}$).
    H2O,
    /// Helium ($\text{He}$).
    He,
    /// Atomic hydrogen ($\text{H}$).
    H,
    /// Molecular hydrogen ($\text{H}_2$).
    H2,
    /// Methane ($\text{CH}_4$).
    CH4,
    /// Atomic nitrogen ($\text{N}$).
    AtomicN,
    /// Nitric oxide ($\text{NO}$).
    NO,
    /// Ammonia ($\text{NH}_3$).
    NH3,
    /// Sulphur dioxide ($\text{SO}_2$).
    SO2,
}

impl GasSpecies {
    /// Standard IUPAC molar mass, kg/mol.
    pub const fn molar_mass_kg_mol(self) -> f64 {
        match self {
            Self::N2 => 0.028_013_4,
            Self::O2 => 0.031_998_8,
            Self::AtomicO => 0.015_999_4,
            Self::Ar => 0.039_948,
            Self::CO2 => 0.044_009_5,
            Self::CO => 0.028_010_1,
            Self::H2O => 0.018_015_28,
            Self::He => 0.004_002_602,
            Self::H => 0.001_007_94,
            Self::H2 => 0.002_015_88,
            Self::CH4 => 0.016_042_5,
            Self::AtomicN => 0.014_006_7,
            Self::NO => 0.030_006_1,
            Self::NH3 => 0.017_030_52,
            Self::SO2 => 0.064_064_8,
        }
    }

    /// Mass of a single molecule or atom, kg.
    pub const fn molecular_mass_kg(self) -> f64 {
        self.molar_mass_kg_mol() / AVOGADRO_CONSTANT_MOL
    }

    /// Kinetic collision diameter, metres.
    pub const fn collision_diameter_m(self) -> f64 {
        match self {
            Self::N2 => 3.64e-10,
            Self::O2 => 3.46e-10,
            Self::AtomicO => 3.00e-10,
            Self::Ar => 3.40e-10,
            Self::CO2 => 3.30e-10,
            Self::CO => 3.76e-10,
            Self::H2O => 2.65e-10,
            Self::He => 2.60e-10,
            Self::H => 2.50e-10,
            Self::H2 => 2.89e-10,
            Self::CH4 => 3.80e-10,
            Self::AtomicN => 3.00e-10,
            Self::NO => 3.17e-10,
            Self::NH3 => 2.90e-10,
            Self::SO2 => 4.11e-10,
        }
    }

    /// Whether this species participates as an atmospheric oxidizer or ABEP propellant feed.
    pub const fn is_oxidizer(self) -> bool {
        matches!(self, Self::O2 | Self::AtomicO)
    }

    /// Dimensionless constant-pressure molar heat capacity $c_p / R$ at `temperature_k`.
    pub fn cp_over_r(self, temperature_k: f64) -> f64 {
        let t = temperature_k.max(1.0);
        match self {
            Self::AtomicO | Self::Ar | Self::He | Self::H | Self::AtomicN => 2.5,
            Self::N2 => 3.5 + einstein_mode(3_374.0, t),
            Self::O2 => 3.5 + einstein_mode(2_256.0, t),
            Self::CO => 3.5 + einstein_mode(3_122.0, t),
            Self::NO => 3.5 + einstein_mode(2_740.0, t),
            Self::H2 => 3.5 + einstein_mode(6_332.0, t),
            Self::CO2 => 3.5 + 2.0 * einstein_mode(960.0, t) + einstein_mode(2_000.0, t),
            Self::H2O => 4.0 + einstein_mode(2_295.0, t),
            Self::CH4 => 4.0 + 3.0 * einstein_mode(1_900.0, t),
            Self::NH3 => 4.0 + 2.0 * einstein_mode(1_450.0, t),
            Self::SO2 => 4.0 + 2.0 * einstein_mode(750.0, t),
        }
    }

    /// Ratio of specific heats $\gamma = c_p / c_v$ at `temperature_k`.
    pub fn adiabatic_index(self, temperature_k: f64) -> f64 {
        let cp = self.cp_over_r(temperature_k);
        cp / (cp - 1.0)
    }

    /// Dynamic viscosity via Sutherland's law, Pa·s, at `temperature_k`.
    pub fn sutherland_viscosity_pa_s(self, temperature_k: f64) -> f64 {
        let t = temperature_k.max(1.0);
        let (mu0, t0, s) = match self {
            Self::N2 => (1.781e-5, 300.55, 111.0),
            Self::O2 => (2.018e-5, 292.25, 127.0),
            Self::AtomicO => (1.85e-5, 300.0, 100.0),
            Self::Ar => (2.125e-5, 273.15, 144.0),
            Self::CO2 => (1.48e-5, 293.15, 240.0),
            Self::CO => (1.72e-5, 288.15, 118.0),
            Self::H2O => (1.12e-5, 350.0, 1_064.0),
            Self::He => (1.90e-5, 273.15, 79.4),
            Self::H => (6.5e-6, 300.0, 72.0),
            Self::H2 => (8.76e-6, 293.85, 72.0),
            Self::CH4 => (1.09e-5, 293.15, 169.0),
            Self::AtomicN => (1.70e-5, 300.0, 100.0),
            Self::NO => (1.88e-5, 293.15, 128.0),
            Self::NH3 => (9.82e-6, 293.15, 370.0),
            Self::SO2 => (1.25e-5, 293.15, 416.0),
        };
        let ratio = t / t0;
        mu0 * ratio * libm::sqrt(ratio) * (t0 + s) / (t + s)
    }

    /// Effective Sutton-Graves stagnation-heating coefficient contribution,
    /// $\text{kg}^{1/2}/\text{m}$, blended by mass fraction in [`super::GasComposition`].
    pub const fn sutton_graves_k(self) -> f64 {
        match self {
            Self::N2 => 1.7248e-4,
            Self::O2 | Self::AtomicO | Self::NO => 1.785e-4,
            Self::Ar | Self::AtomicN => 1.93e-4,
            Self::CO2 | Self::CO | Self::SO2 => SUTTON_GRAVES_K_MARS,
            Self::H2O | Self::NH3 => 1.65e-4,
            Self::He | Self::H2 => 1.25e-4,
            Self::H => 1.15e-4,
            Self::CH4 => 1.60e-4,
        }
    }

    /// Standard gas-surface name matching common atmospheric tables.
    pub const fn symbol(self) -> &'static str {
        match self {
            Self::N2 => "N2",
            Self::O2 => "O2",
            Self::AtomicO => "O",
            Self::Ar => "Ar",
            Self::CO2 => "CO2",
            Self::CO => "CO",
            Self::H2O => "H2O",
            Self::He => "He",
            Self::H => "H",
            Self::H2 => "H2",
            Self::CH4 => "CH4",
            Self::AtomicN => "N",
            Self::NO => "NO",
            Self::NH3 => "NH3",
            Self::SO2 => "SO2",
        }
    }
}

fn einstein_mode(theta_k: f64, temperature_k: f64) -> f64 {
    let x = theta_k / temperature_k;
    if x > 40.0 {
        return 0.0;
    }
    let exp_neg = libm::exp(-x);
    let denom = 1.0 - exp_neg;
    (x * x * exp_neg) / (denom * denom)
}
