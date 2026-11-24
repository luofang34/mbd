use super::*;

#[test]
fn earth_isa_sea_level_matches_standard_values() {
    let air = AmbientGas::earth_air(101_325.0, 288.15).unwrap();
    assert!((air.molar_mass_kg_mol() - 0.028_965).abs() < 2e-6);
    assert!((air.density_kg_m3 - 1.225).abs() < 1e-3);
    assert!((air.sound_speed_m_s - 340.29).abs() < 0.2);
    assert!((air.adiabatic_index() - 1.400).abs() < 2e-3);
    assert!((air.dynamic_viscosity_pa_s - 1.789e-5).abs() < 2e-7);
    assert!((air.sutton_graves_k() - SUTTON_GRAVES_K_EARTH).abs() < 1e-6);
    assert!(air.knudsen_number(1.0) < 1e-6);
}

#[test]
fn mars_co2_air_matches_low_sound_speed_and_heating_constant() {
    let mars = AmbientGas::mars_air(610.0, 215.0).unwrap();
    assert!(mars.mole_fraction(GasSpecies::CO2) > 0.95);
    assert!((mars.molar_mass_kg_mol() - 0.043_34).abs() < 2e-4);
    assert!((mars.density_kg_m3 - 0.0148).abs() < 1e-3);
    assert!((225.0..240.0).contains(&mars.sound_speed_m_s));
    assert!((mars.sutton_graves_k() - SUTTON_GRAVES_K_MARS).abs() < 2e-6);
    assert!(mars.oxidizer_mass_fraction() < 0.002);
}

#[test]
fn vleo_partial_number_densities_recover_exact_atomic_oxygen_species() {
    let n_o = 2.0e15_f64;
    let n_n2 = 4.0e14_f64;
    let n_he = 1.0e14_f64;
    let gas = AmbientGas::from_partial_number_densities(
        900.0,
        &[
            (GasSpecies::AtomicO, n_o),
            (GasSpecies::N2, n_n2),
            (GasSpecies::He, n_he),
        ],
    )
    .unwrap();
    assert!((gas.total_number_density_m3() - (n_o + n_n2 + n_he)).abs() < 1.0);
    assert!((gas.number_density_m3(GasSpecies::AtomicO) - n_o).abs() < 10.0);
    let expected_rho_o = n_o * GasSpecies::AtomicO.molecular_mass_kg();
    assert!((gas.mass_density_of(GasSpecies::AtomicO) - expected_rho_o).abs() < 1e-18);
    assert!(gas.knudsen_number(2.0) > 10.0);
    assert!(gas.oxidizer_mass_fraction() > 0.6);
}

#[test]
fn humid_air_lowers_density_relative_to_dry_air() {
    let dry = AmbientGas::earth_air(101_325.0, 303.15).unwrap();
    let humid_comp = GasComposition::earth_humid_air(0.8, 303.15, 101_325.0).unwrap();
    let humid = AmbientGas::from_state_and_composition(101_325.0, 303.15, humid_comp).unwrap();
    assert!(humid.humidity_mass_fraction() > 0.01);
    assert!(humid.density_kg_m3 < dry.density_kg_m3);
}

#[test]
fn vacuum_and_duplicate_species_behave_cleanly() {
    assert!(AmbientGas::VACUUM.is_vacuum());
    assert_eq!(AmbientGas::VACUUM.knudsen_number(1.0), f64::INFINITY);
    let comp = GasComposition::from_mole_fractions(&[(GasSpecies::N2, 3.0), (GasSpecies::N2, 1.0)])
        .unwrap();
    assert_eq!(comp.as_slice().len(), 1);
    assert!((comp.mole_fraction(GasSpecies::N2) - 1.0).abs() < 1e-12);

    assert!(GasComposition::from_mole_fractions(&[(GasSpecies::N2, -0.1)]).is_err());
    assert!(AmbientGas::earth_air(-1.0, 288.0).is_err());
}
