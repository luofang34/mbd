use core::f64::consts::{PI, SQRT_2};

use crate::MbdError;

mod mixture;
mod species;

#[cfg(test)]
mod tests;

pub use mixture::{GasComposition, SpeciesFraction};
pub use species::GasSpecies;

/// Universal gas constant, J/(mol·K).
pub const UNIVERSAL_GAS_CONSTANT_J_MOL_K: f64 = 8.314_462_618;

/// Boltzmann constant, J/K.
pub const BOLTZMANN_CONSTANT_J_K: f64 = 1.380_649e-23;

/// Avogadro constant, $\text{mol}^{-1}$.
pub const AVOGADRO_CONSTANT_MOL: f64 = 6.022_140_76e23;

/// Sutton-Graves stagnation-heating constant for Earth air, in SI units with
/// flux in $\text{W/m}^2$: `k` in $q = k \sqrt{\rho / R_n} V^3$.
pub const SUTTON_GRAVES_K_EARTH: f64 = 1.7415e-4;

/// Sutton-Graves stagnation-heating constant for Martian $\text{CO}_2$ entry.
pub const SUTTON_GRAVES_K_MARS: f64 = 1.9027e-4;

/// Maximum distinct chemical species tracked simultaneously in a fixed-size [`GasComposition`].
pub const MAX_GAS_SPECIES: usize = 8;

/// Undisturbed ambient gas around the vehicle, unified across continuum
/// aerodynamics, supersonic/rarefied regimes, airbreathing engines, thermosphere
/// drag, atomic-oxygen ABEP intakes, and Martian/extraterrestrial atmospheres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AmbientGas {
    /// Total static pressure, Pa.
    pub pressure_pa: f64,
    /// Kinetic/static temperature, K.
    pub temperature_k: f64,
    /// Bulk mass density, kg/m³.
    pub density_kg_m3: f64,
    /// Speed of sound in the mixture, m/s.
    pub sound_speed_m_s: f64,
    /// Mixture dynamic viscosity, Pa·s.
    pub dynamic_viscosity_pa_s: f64,
    /// Molecular mean free path, m (`0.0` in pure vacuum).
    pub mean_free_path_m: f64,
    /// Chemical species mole fractions.
    pub composition: GasComposition,
}

impl Default for AmbientGas {
    fn default() -> Self {
        Self::VACUUM
    }
}

impl AmbientGas {
    /// Hard vacuum: zero pressure, density, viscosity, and composition.
    pub const VACUUM: Self = Self {
        pressure_pa: 0.0,
        temperature_k: 0.0,
        density_kg_m3: 0.0,
        sound_speed_m_s: 0.0,
        dynamic_viscosity_pa_s: 0.0,
        mean_free_path_m: 0.0,
        composition: GasComposition::VACUUM,
    };

    /// Builds an ideal-gas ambient state from bulk static pressure, static
    /// temperature, and chemical [`GasComposition`].
    pub fn from_state_and_composition(
        pressure_pa: f64,
        temperature_k: f64,
        composition: GasComposition,
    ) -> Result<Self, MbdError> {
        check_nonnegative("pressure_pa", pressure_pa)?;
        check_nonnegative("temperature_k", temperature_k)?;
        if pressure_pa == 0.0 || temperature_k == 0.0 || composition.is_empty() {
            return Ok(Self::VACUUM);
        }
        let molar_mass = composition.molar_mass_kg_mol();
        let r_specific = UNIVERSAL_GAS_CONSTANT_J_MOL_K / molar_mass;
        let density_kg_m3 = pressure_pa / (r_specific * temperature_k);
        let gamma = composition.adiabatic_index(temperature_k);
        let sound_speed_m_s = libm::sqrt(gamma * r_specific * temperature_k);
        let dynamic_viscosity_pa_s = composition.dynamic_viscosity_pa_s(temperature_k);
        let d_eff = composition.mean_collision_diameter_m();
        let mean_free_path_m = if d_eff > 0.0 {
            BOLTZMANN_CONSTANT_J_K * temperature_k / (SQRT_2 * PI * d_eff * d_eff * pressure_pa)
        } else {
            0.0
        };
        Ok(Self {
            pressure_pa,
            temperature_k,
            density_kg_m3,
            sound_speed_m_s,
            dynamic_viscosity_pa_s,
            mean_free_path_m,
            composition,
        })
    }

    /// Builds an ambient gas state directly from partial number densities
    /// $n_i$ ($\text{m}^{-3}$) and kinetic temperature $T$ (K), such as an
    /// NRLMSISE thermosphere / VLEO atomic-oxygen profile.
    pub fn from_partial_number_densities(
        temperature_k: f64,
        number_densities_m3: &[(GasSpecies, f64)],
    ) -> Result<Self, MbdError> {
        check_nonnegative("temperature_k", temperature_k)?;
        if temperature_k == 0.0 {
            return Ok(Self::VACUUM);
        }
        let mut total_n = 0.0;
        let mut weighted = [(GasSpecies::N2, 0.0); MAX_GAS_SPECIES];
        let mut count = 0usize;
        for &(species, n_i) in number_densities_m3 {
            check_nonnegative("number_density_m3", n_i)?;
            if n_i == 0.0 {
                continue;
            }
            total_n += n_i;
            accumulate_species(&mut weighted, &mut count, species, n_i)?;
        }
        if total_n <= 0.0 {
            return Ok(Self::VACUUM);
        }
        let pairs = weighted.get(..count).unwrap_or(&[]);
        let composition = GasComposition::from_mole_fractions(pairs)?;
        let pressure_pa = total_n * BOLTZMANN_CONSTANT_J_K * temperature_k;
        Self::from_state_and_composition(pressure_pa, temperature_k, composition)
    }

    /// Convenience constructor for standard dry Earth air.
    pub fn earth_air(pressure_pa: f64, temperature_k: f64) -> Result<Self, MbdError> {
        Self::from_state_and_composition(
            pressure_pa,
            temperature_k,
            GasComposition::earth_dry_air(),
        )
    }

    /// Convenience constructor for Martian $\text{CO}_2$-rich air.
    pub fn mars_air(pressure_pa: f64, temperature_k: f64) -> Result<Self, MbdError> {
        Self::from_state_and_composition(pressure_pa, temperature_k, GasComposition::mars_air())
    }

    /// Whether no appreciable gas is present (`density_kg_m3 == 0.0`).
    pub const fn is_vacuum(&self) -> bool {
        self.density_kg_m3 == 0.0
    }

    /// Validates all scalar fields against non-finite or physically impossible negative values.
    pub fn validate(&self) -> Result<(), MbdError> {
        let checks = [
            ("ambient.pressure_pa", self.pressure_pa),
            ("ambient.temperature_k", self.temperature_k),
            ("ambient.density_kg_m3", self.density_kg_m3),
            ("ambient.sound_speed_m_s", self.sound_speed_m_s),
            (
                "ambient.dynamic_viscosity_pa_s",
                self.dynamic_viscosity_pa_s,
            ),
            ("ambient.mean_free_path_m", self.mean_free_path_m),
        ];
        for (field, value) in checks {
            if !value.is_finite() || value < 0.0 {
                return Err(MbdError::InvalidFlow { field, value });
            }
        }
        Ok(())
    }

    /// Mole fraction of `species` (`0.0` if absent).
    pub fn mole_fraction(&self, species: GasSpecies) -> f64 {
        self.composition.mole_fraction(species)
    }

    /// Mass fraction of `species` (`0.0` if absent).
    pub fn mass_fraction(&self, species: GasSpecies) -> f64 {
        self.composition.mass_fraction(species)
    }

    /// Partial pressure $p_i = \chi_i p$ of `species`, Pa.
    pub fn partial_pressure_pa(&self, species: GasSpecies) -> f64 {
        self.pressure_pa * self.mole_fraction(species)
    }

    /// Total molecular/atomic number density $n = p / (k_B T)$, $\text{m}^{-3}$.
    pub fn total_number_density_m3(&self) -> f64 {
        if self.temperature_k <= 0.0 || self.pressure_pa <= 0.0 {
            return 0.0;
        }
        self.pressure_pa / (BOLTZMANN_CONSTANT_J_K * self.temperature_k)
    }

    /// Partial number density $n_i = \chi_i n$ of `species`, $\text{m}^{-3}$.
    pub fn number_density_m3(&self, species: GasSpecies) -> f64 {
        self.total_number_density_m3() * self.mole_fraction(species)
    }

    /// Partial mass density $\rho_i = Y_i \rho$ of `species`, kg/m³.
    pub fn mass_density_of(&self, species: GasSpecies) -> f64 {
        self.density_kg_m3 * self.mass_fraction(species)
    }

    /// Mixture molar mass, kg/mol.
    pub fn molar_mass_kg_mol(&self) -> f64 {
        self.composition.molar_mass_kg_mol()
    }

    /// Mixture specific gas constant $R_{\text{mix}} = R_u / M_{\text{mix}}$, J/(kg·K).
    pub fn specific_gas_constant_j_kg_k(&self) -> f64 {
        let molar = self.molar_mass_kg_mol();
        if molar <= 0.0 {
            0.0
        } else {
            UNIVERSAL_GAS_CONSTANT_J_MOL_K / molar
        }
    }

    /// Mixture ratio of specific heats $\gamma = c_p / c_v$.
    pub fn adiabatic_index(&self) -> f64 {
        if self.temperature_k <= 0.0 {
            0.0
        } else {
            self.composition.adiabatic_index(self.temperature_k)
        }
    }

    /// Knudsen number $\text{Kn} = \lambda / L$ at reference length `ref_length_m`.
    pub fn knudsen_number(&self, ref_length_m: f64) -> f64 {
        if self.is_vacuum() || ref_length_m <= 0.0 {
            f64::INFINITY
        } else {
            self.mean_free_path_m / ref_length_m
        }
    }

    /// Mixture Sutton-Graves stagnation-heating coefficient $k$, $\text{kg}^{1/2}/\text{m}$.
    pub fn sutton_graves_k(&self) -> f64 {
        self.composition.sutton_graves_k()
    }

    /// Mass fraction of atmospheric oxidizers ($\text{O}_2 + \text{O}$).
    pub fn oxidizer_mass_fraction(&self) -> f64 {
        self.composition.oxidizer_mass_fraction()
    }

    /// Water-vapour mass fraction ($\text{H}_2\text{O}$).
    pub fn humidity_mass_fraction(&self) -> f64 {
        self.mass_fraction(GasSpecies::H2O)
    }
}

fn check_nonnegative(field: &'static str, value: f64) -> Result<(), MbdError> {
    if value.is_finite() && value >= 0.0 {
        return Ok(());
    }
    Err(MbdError::InvalidGas { field, value })
}

fn accumulate_species(
    slots: &mut [(GasSpecies, f64); MAX_GAS_SPECIES],
    count: &mut usize,
    species: GasSpecies,
    value: f64,
) -> Result<(), MbdError> {
    if let Some((_, sum)) = slots.iter_mut().take(*count).find(|(s, _)| *s == species) {
        *sum += value;
        return Ok(());
    }
    let Some(slot) = slots.get_mut(*count) else {
        return Err(MbdError::InvalidGas {
            field: "species_count",
            value: (*count).wrapping_add(1) as f64,
        });
    };
    *slot = (species, value);
    *count = (*count).wrapping_add(1);
    Ok(())
}
