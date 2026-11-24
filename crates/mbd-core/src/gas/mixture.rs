use crate::MbdError;

use super::{GasSpecies, MAX_GAS_SPECIES};

/// Relative normalization tolerance used when checking mole fractions.
const MIN_POSITIVE_MOLE_TOTAL: f64 = 1e-30;

/// One constituent species and its mole (volume) fraction inside a gas mixture.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeciesFraction {
    /// Chemical species.
    pub species: GasSpecies,
    /// Dimensionless mole fraction in `(0.0..=1.0]`.
    pub mole_fraction: f64,
}

/// Fixed-capacity `Copy` description of the chemical makeup of a gas mixture.
///
/// Up to [`MAX_GAS_SPECIES`] nonzero species are stored with mole fractions
/// summing to `1.0`, or zero species for [`GasComposition::VACUUM`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GasComposition {
    entries: [SpeciesFraction; MAX_GAS_SPECIES],
    len: u8,
}

impl Default for GasComposition {
    fn default() -> Self {
        Self::VACUUM
    }
}

impl GasComposition {
    /// Empty gas composition representing hard vacuum.
    pub const VACUUM: Self = Self {
        entries: [SpeciesFraction {
            species: GasSpecies::N2,
            mole_fraction: 0.0,
        }; MAX_GAS_SPECIES],
        len: 0,
    };

    /// Builds a normalized composition from `(species, mole_fraction)` pairs.
    ///
    /// Duplicate species are summed. Rejects non-finite or negative fractions
    /// and mixtures exceeding [`MAX_GAS_SPECIES`] distinct nonzero species. An
    /// empty or all-zero slice produces [`Self::VACUUM`].
    pub fn from_mole_fractions(entries: &[(GasSpecies, f64)]) -> Result<Self, MbdError> {
        let mut combined = [SpeciesFraction {
            species: GasSpecies::N2,
            mole_fraction: 0.0,
        }; MAX_GAS_SPECIES];
        let mut count = 0usize;
        for &(species, fraction) in entries {
            if !fraction.is_finite() || fraction < 0.0 {
                return Err(MbdError::InvalidGas {
                    field: "mole_fraction",
                    value: fraction,
                });
            }
            if fraction == 0.0 {
                continue;
            }
            add_or_insert(&mut combined, &mut count, species, fraction)?;
        }
        normalize_entries(&mut combined, count)
    }

    /// Standard dry Earth homosphere air (sea level through ~85 km).
    pub const fn earth_dry_air() -> Self {
        Self::from_normalized_4(
            (GasSpecies::N2, 0.780_84),
            (GasSpecies::O2, 0.209_46),
            (GasSpecies::Ar, 0.009_34),
            (GasSpecies::CO2, 0.000_36),
        )
    }

    /// Representative Earth VLEO gas composition near 200 km altitude under
    /// moderate solar activity, containing both molecular air and atomic oxygen.
    pub const fn earth_vleo_200km() -> Self {
        Self::from_normalized_5(
            (GasSpecies::AtomicO, 0.48),
            (GasSpecies::N2, 0.46),
            (GasSpecies::O2, 0.05),
            (GasSpecies::Ar, 0.005),
            (GasSpecies::He, 0.005),
        )
    }

    /// Representative Earth LEO thermosphere near 350 km altitude, dominated by atomic oxygen.
    pub const fn earth_vleo_350km() -> Self {
        Self::from_normalized_4(
            (GasSpecies::AtomicO, 0.82),
            (GasSpecies::N2, 0.14),
            (GasSpecies::He, 0.035),
            (GasSpecies::O2, 0.005),
        )
    }

    /// Earth tropospheric air blended with water vapour from `relative_humidity` (`0.0..=1.0`).
    pub fn earth_humid_air(
        relative_humidity: f64,
        temperature_k: f64,
        pressure_pa: f64,
    ) -> Result<Self, MbdError> {
        if !relative_humidity.is_finite() || !(0.0..=1.0).contains(&relative_humidity) {
            return Err(MbdError::InvalidGas {
                field: "relative_humidity",
                value: relative_humidity,
            });
        }
        if !temperature_k.is_finite() || temperature_k <= 0.0 {
            return Err(MbdError::InvalidGas {
                field: "temperature_k",
                value: temperature_k,
            });
        }
        if !pressure_pa.is_finite() || pressure_pa <= 0.0 {
            return Err(MbdError::InvalidGas {
                field: "pressure_pa",
                value: pressure_pa,
            });
        }
        if relative_humidity == 0.0 {
            return Ok(Self::earth_dry_air());
        }
        let x_h2o = (relative_humidity * saturation_vapor_pressure_pa(temperature_k) / pressure_pa)
            .clamp(0.0, 0.99);
        let dry_scale = 1.0 - x_h2o;
        Self::from_mole_fractions(&[
            (GasSpecies::N2, 0.780_84 * dry_scale),
            (GasSpecies::O2, 0.209_46 * dry_scale),
            (GasSpecies::Ar, 0.009_34 * dry_scale),
            (GasSpecies::CO2, 0.000_36 * dry_scale),
            (GasSpecies::H2O, x_h2o),
        ])
    }

    /// Mars atmospheric composition ($\text{CO}_2$-dominated).
    pub const fn mars_air() -> Self {
        Self::from_normalized_5(
            (GasSpecies::CO2, 0.953_2 / 0.998_3),
            (GasSpecies::N2, 0.027_0 / 0.998_3),
            (GasSpecies::Ar, 0.016_0 / 0.998_3),
            (GasSpecies::O2, 0.001_3 / 0.998_3),
            (GasSpecies::CO, 0.000_8 / 0.998_3),
        )
    }

    /// Venusian lower/upper atmosphere ($\text{CO}_2$ + $\text{N}_2$).
    pub const fn venus_air() -> Self {
        Self::from_normalized_2((GasSpecies::CO2, 0.965), (GasSpecies::N2, 0.035))
    }

    /// Whether the composition has no species (vacuum).
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Active nonzero species entries, summing in mole fraction to `1.0` when non-empty.
    pub fn as_slice(&self) -> &[SpeciesFraction] {
        self.entries.get(..usize::from(self.len)).unwrap_or(&[])
    }

    /// Mole fraction of `species` (`0.0` if absent).
    pub fn mole_fraction(&self, species: GasSpecies) -> f64 {
        self.as_slice()
            .iter()
            .find(|entry| entry.species == species)
            .map_or(0.0, |entry| entry.mole_fraction)
    }

    /// Effective mixture molar mass, kg/mol (`0.0` for vacuum).
    pub fn molar_mass_kg_mol(&self) -> f64 {
        self.as_slice()
            .iter()
            .map(|entry| entry.mole_fraction * entry.species.molar_mass_kg_mol())
            .sum()
    }

    /// Mass fraction $Y_i$ of `species` (`0.0` if absent or vacuum).
    pub fn mass_fraction(&self, species: GasSpecies) -> f64 {
        let total = self.molar_mass_kg_mol();
        if total <= 0.0 {
            return 0.0;
        }
        self.mole_fraction(species) * species.molar_mass_kg_mol() / total
    }

    /// Mixture ratio of specific heats $\gamma = c_p / c_v$ at `temperature_k`.
    pub fn adiabatic_index(&self, temperature_k: f64) -> f64 {
        if self.is_empty() {
            return 0.0;
        }
        let cp_over_r: f64 = self
            .as_slice()
            .iter()
            .map(|entry| entry.mole_fraction * entry.species.cp_over_r(temperature_k))
            .sum();
        if cp_over_r <= 1.0 {
            return 0.0;
        }
        cp_over_r / (cp_over_r - 1.0)
    }

    /// Effective hard-sphere collision diameter squared weighted by mole fraction, metres.
    pub fn mean_collision_diameter_m(&self) -> f64 {
        let d_sq: f64 = self
            .as_slice()
            .iter()
            .map(|entry| {
                let d = entry.species.collision_diameter_m();
                entry.mole_fraction * d * d
            })
            .sum();
        libm::sqrt(d_sq)
    }

    /// Mixture dynamic viscosity via Herning-Zipperer mixing rule, Pa·s.
    pub fn dynamic_viscosity_pa_s(&self, temperature_k: f64) -> f64 {
        let mut numer = 0.0;
        let mut denom = 0.0;
        for entry in self.as_slice() {
            let root_m = libm::sqrt(entry.species.molar_mass_kg_mol());
            let w = entry.mole_fraction * root_m;
            numer += w * entry.species.sutherland_viscosity_pa_s(temperature_k);
            denom += w;
        }
        if denom <= 0.0 { 0.0 } else { numer / denom }
    }

    /// Effective Sutton-Graves stagnation-heating constant $k$, $\text{kg}^{1/2}/\text{m}$.
    pub fn sutton_graves_k(&self) -> f64 {
        let molar = self.molar_mass_kg_mol();
        if molar <= 0.0 {
            return 0.0;
        }
        self.as_slice()
            .iter()
            .map(|entry| {
                let y = entry.mole_fraction * entry.species.molar_mass_kg_mol() / molar;
                y * entry.species.sutton_graves_k()
            })
            .sum()
    }

    /// Sum of mass fractions of oxidizing species ($\text{O}_2$ and $\text{O}$).
    pub fn oxidizer_mass_fraction(&self) -> f64 {
        let molar = self.molar_mass_kg_mol();
        if molar <= 0.0 {
            return 0.0;
        }
        self.as_slice()
            .iter()
            .filter(|entry| entry.species.is_oxidizer())
            .map(|entry| entry.mole_fraction * entry.species.molar_mass_kg_mol() / molar)
            .sum()
    }

    const fn from_normalized_2(a: (GasSpecies, f64), b: (GasSpecies, f64)) -> Self {
        let empty = SpeciesFraction {
            species: GasSpecies::N2,
            mole_fraction: 0.0,
        };
        Self {
            entries: [
                SpeciesFraction {
                    species: a.0,
                    mole_fraction: a.1,
                },
                SpeciesFraction {
                    species: b.0,
                    mole_fraction: b.1,
                },
                empty,
                empty,
                empty,
                empty,
                empty,
                empty,
            ],
            len: 2,
        }
    }

    const fn from_normalized_4(
        a: (GasSpecies, f64),
        b: (GasSpecies, f64),
        c: (GasSpecies, f64),
        d: (GasSpecies, f64),
    ) -> Self {
        let empty = SpeciesFraction {
            species: GasSpecies::N2,
            mole_fraction: 0.0,
        };
        Self {
            entries: [
                SpeciesFraction {
                    species: a.0,
                    mole_fraction: a.1,
                },
                SpeciesFraction {
                    species: b.0,
                    mole_fraction: b.1,
                },
                SpeciesFraction {
                    species: c.0,
                    mole_fraction: c.1,
                },
                SpeciesFraction {
                    species: d.0,
                    mole_fraction: d.1,
                },
                empty,
                empty,
                empty,
                empty,
            ],
            len: 4,
        }
    }

    const fn from_normalized_5(
        a: (GasSpecies, f64),
        b: (GasSpecies, f64),
        c: (GasSpecies, f64),
        d: (GasSpecies, f64),
        e: (GasSpecies, f64),
    ) -> Self {
        let empty = SpeciesFraction {
            species: GasSpecies::N2,
            mole_fraction: 0.0,
        };
        Self {
            entries: [
                SpeciesFraction {
                    species: a.0,
                    mole_fraction: a.1,
                },
                SpeciesFraction {
                    species: b.0,
                    mole_fraction: b.1,
                },
                SpeciesFraction {
                    species: c.0,
                    mole_fraction: c.1,
                },
                SpeciesFraction {
                    species: d.0,
                    mole_fraction: d.1,
                },
                SpeciesFraction {
                    species: e.0,
                    mole_fraction: e.1,
                },
                empty,
                empty,
                empty,
            ],
            len: 5,
        }
    }
}

fn add_or_insert(
    combined: &mut [SpeciesFraction; MAX_GAS_SPECIES],
    count: &mut usize,
    species: GasSpecies,
    fraction: f64,
) -> Result<(), MbdError> {
    if let Some(existing) = combined
        .iter_mut()
        .take(*count)
        .find(|entry| entry.species == species)
    {
        existing.mole_fraction += fraction;
        return Ok(());
    }
    let Some(slot) = combined.get_mut(*count) else {
        return Err(MbdError::InvalidGas {
            field: "species_count",
            value: (*count).wrapping_add(1) as f64,
        });
    };
    *slot = SpeciesFraction {
        species,
        mole_fraction: fraction,
    };
    *count = (*count).wrapping_add(1);
    Ok(())
}

fn normalize_entries(
    combined: &mut [SpeciesFraction; MAX_GAS_SPECIES],
    count: usize,
) -> Result<GasComposition, MbdError> {
    if count == 0 {
        return Ok(GasComposition::VACUUM);
    }
    let total: f64 = combined
        .iter()
        .take(count)
        .map(|entry| entry.mole_fraction)
        .sum();
    if !total.is_finite() || total <= MIN_POSITIVE_MOLE_TOTAL {
        return Ok(GasComposition::VACUUM);
    }
    for entry in combined.iter_mut().take(count) {
        entry.mole_fraction /= total;
    }
    Ok(GasComposition {
        entries: *combined,
        len: count as u8,
    })
}

/// Buck / Tetens saturation vapour pressure over liquid water, Pa.
fn saturation_vapor_pressure_pa(temperature_k: f64) -> f64 {
    let t_c = temperature_k - 273.15;
    if t_c >= 0.0 {
        611.21 * libm::exp((18.678 - t_c / 234.5) * (t_c / (257.14 + t_c)))
    } else {
        611.15 * libm::exp((23.036 - t_c / 333.7) * (t_c / (279.82 + t_c)))
    }
}
