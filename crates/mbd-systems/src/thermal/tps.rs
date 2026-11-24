use mbd_core::{AmbientGas, ComponentId, ComponentTree, GasSpecies, ThermalLoad};

use crate::error::{SystemsError, check_non_negative, check_positive, check_unit_interval};

const STEFAN_BOLTZMANN_W_M2_K4: f64 = 5.670_374_419e-8;

/// Ablative Thermal Protection System (TPS) heatshield model supporting multi-species entry
/// heating (Earth Air, Mars $\text{CO}_2$, etc.) and RF plasma blackout telemetry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TpsAblationSpec {
    /// Heatshield component identifier.
    pub component: ComponentId,
    /// Effective stagnation nose radius $R_n$, m.
    pub nose_radius_m: f64,
    /// Exposed heatshield area participating in ablation, m².
    pub shield_area_m2: f64,
    /// Ablating char surface radiating temperature, K.
    pub char_surface_temp_k: f64,
    /// Surface infrared emissivity $\epsilon \in [0, 1]$.
    pub emissivity: f64,
    /// Effective thermochemical heat of ablation $Q^*$, J/kg.
    pub heat_of_ablation_j_kg: f64,
    /// Fraction of net un-ablated surface flux Conducted into vehicle structure (`0.0..=1.0`).
    pub backface_soak_fraction: f64,
    /// Initial virgin + pyrolyzing ablator mass loaded on heatshield, kg.
    pub initial_ablator_mass_kg: f64,
    /// Radiative shock-layer heating scale factor (`0.0` to disable, `~1e-7` Tauber-Sutton approximation).
    pub radiative_heating_coeff: f64,
}

impl TpsAblationSpec {
    /// Validates heatshield TPS properties.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_positive("tps.nose_radius_m", self.nose_radius_m)?;
        check_positive("tps.shield_area_m2", self.shield_area_m2)?;
        check_positive("tps.char_surface_temp_k", self.char_surface_temp_k)?;
        check_unit_interval("tps.emissivity", self.emissivity)?;
        check_positive("tps.heat_of_ablation_j_kg", self.heat_of_ablation_j_kg)?;
        check_unit_interval("tps.backface_soak_fraction", self.backface_soak_fraction)?;
        check_positive("tps.initial_ablator_mass_kg", self.initial_ablator_mass_kg)?;
        check_non_negative("tps.radiative_heating_coeff", self.radiative_heating_coeff)?;
        Ok(())
    }
}

/// Dynamic state of a [`TpsAblationSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TpsAblationState {
    /// Remaining unablated heatshield mass, kg.
    pub remaining_ablator_mass_kg: f64,
    /// Cumulative mass ablated since start of entry, kg (`ComponentState.dry_mass_delta_kg = -cumulative_ablated_kg`).
    pub cumulative_ablated_kg: f64,
    /// Peak stagnation cold-wall heat flux recorded, W/m².
    pub peak_heat_flux_w_m2: f64,
    /// Whether heatshield ablator has been completely burned through (`remaining <= 0`).
    pub burned_through: bool,
}

impl TpsAblationState {
    /// Initializes an un-ablated heatshield loaded with `spec.initial_ablator_mass_kg`.
    pub const fn fresh(spec: &TpsAblationSpec) -> Self {
        Self {
            remaining_ablator_mass_kg: spec.initial_ablator_mass_kg,
            cumulative_ablated_kg: 0.0,
            peak_heat_flux_w_m2: 0.0,
            burned_through: false,
        }
    }
}

/// Heating, mass-loss, and RF plasma blackout telemetry from [`step_tps_ablation`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TpsAblationOutput {
    /// Convective Sutton-Graves stagnation heat flux $q_{\text{conv}}$, W/m².
    pub convective_heat_flux_w_m2: f64,
    /// Shock-layer radiative heat flux $q_{\text{rad}}$, W/m².
    pub radiative_heat_flux_w_m2: f64,
    /// Mass ablated during this step, kg.
    pub ablated_mass_kg: f64,
    /// Residual soak-through thermal load conducting into `spec.component`.
    pub backface_thermal_load: ThermalLoad,
    /// Approximate S-band/VHF RF plasma attenuation during ionized hypersonic entry, dB.
    pub rf_attenuation_db: f64,
    /// True during heavy plasma blackout (`rf_attenuation_db >= 20.0` dB).
    pub comm_blackout: bool,
}

/// Evaluates multi-species Sutton-Graves entry heating, TPS ablation mass loss, and
/// hypersonic plasma RF attenuation over `dt_s`.
pub fn step_tps_ablation(
    spec: &TpsAblationSpec,
    state: &mut TpsAblationState,
    dt_s: f64,
    ambient: &AmbientGas,
    airspeed_m_s: f64,
) -> Result<TpsAblationOutput, SystemsError> {
    spec.validate()?;
    ambient.validate()?;
    check_non_negative("tps.dt_s", dt_s)?;
    check_non_negative("tps.airspeed_m_s", airspeed_m_s)?;

    let k_sg = ambient.sutton_graves_k();
    let v3 = airspeed_m_s * airspeed_m_s * airspeed_m_s;
    let convective_heat_flux_w_m2 = if ambient.density_kg_m3 > 0.0 {
        k_sg * libm::sqrt(ambient.density_kg_m3 / spec.nose_radius_m) * v3
    } else {
        0.0
    };

    let radiative_heat_flux_w_m2 = if airspeed_m_s > 6500.0 && ambient.density_kg_m3 > 0.0 {
        spec.radiative_heating_coeff
            * libm::pow(ambient.density_kg_m3, 1.2)
            * libm::pow(spec.nose_radius_m, 0.5)
            * libm::pow(airspeed_m_s / 1000.0, 8.5)
    } else {
        0.0
    };

    let q_total_w_m2 = convective_heat_flux_w_m2 + radiative_heat_flux_w_m2;
    state.peak_heat_flux_w_m2 = state.peak_heat_flux_w_m2.max(q_total_w_m2);

    let q_rerad_w_m2 =
        spec.emissivity * STEFAN_BOLTZMANN_W_M2_K4 * libm::pow(spec.char_surface_temp_k, 4.0);
    let q_net_ablate_w_m2 = (q_total_w_m2 - q_rerad_w_m2).max(0.0);

    let mut ablated_mass_kg = 0.0;
    if q_net_ablate_w_m2 > 0.0 && dt_s > 0.0 && state.remaining_ablator_mass_kg > 0.0 {
        let req_dm = (q_net_ablate_w_m2 * spec.shield_area_m2 * dt_s) / spec.heat_of_ablation_j_kg;
        ablated_mass_kg = req_dm.min(state.remaining_ablator_mass_kg);
        state.remaining_ablator_mass_kg -= ablated_mass_kg;
        state.cumulative_ablated_kg += ablated_mass_kg;
        if state.remaining_ablator_mass_kg <= 1.0e-9 {
            state.burned_through = true;
        }
    }

    let backface_flux = if state.burned_through {
        q_total_w_m2
    } else {
        (q_total_w_m2 - q_net_ablate_w_m2 * 0.75).max(0.0) * spec.backface_soak_fraction
    };

    // Plasma blackout indicator scaled from hypersonic kinetic energy flux 0.5 * rho * V^3
    let ke_flux = 0.5 * ambient.density_kg_m3 * v3;
    let rf_attenuation_db = if airspeed_m_s > 2800.0 && ke_flux > 1.0e4 {
        (12.0 * libm::log10(ke_flux / 1.0e4)).clamp(0.0, 80.0)
    } else {
        0.0
    };

    Ok(TpsAblationOutput {
        convective_heat_flux_w_m2,
        radiative_heat_flux_w_m2,
        ablated_mass_kg,
        backface_thermal_load: ThermalLoad {
            component: spec.component,
            heat_rate_w: backface_flux * spec.shield_area_m2,
        },
        rf_attenuation_db,
        comm_blackout: rf_attenuation_db >= 20.0,
    })
}

/// Applies cumulative TPS ablation mass loss to the host component's `dry_mass_delta_kg`
/// in `tree` (`dry_mass_delta_kg = -state.cumulative_ablated_kg`).
pub fn apply_tps_ablation_to_tree(
    spec: &TpsAblationSpec,
    state: &TpsAblationState,
    tree: &mut ComponentTree,
) -> Result<(), SystemsError> {
    let mut comp_state = *tree.state(spec.component)?;
    comp_state.dry_mass_delta_kg = -state.cumulative_ablated_kg;
    tree.set_state(spec.component, comp_state)?;
    Ok(())
}

/// VLEO Atomic Oxygen (`AtomicO`) surface chemical erosion model (e.g. Kapton blankets, silver interconnects).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AtomicOxygenErosionSpec {
    /// Component bearing the exposed surface coating.
    pub component: ComponentId,
    /// Exposed surface area, m².
    pub area_m2: f64,
    /// Volumetric erosion yield per incident oxygen atom, m³/atom (e.g. Kapton ≈ `3.0e-30`).
    pub erosion_yield_m3_per_atom: f64,
    /// Coating bulk density, kg/m³ (e.g. Kapton ≈ `1420.0`).
    pub coating_density_kg_m3: f64,
    /// Initial protective coating thickness, m.
    pub initial_thickness_m: f64,
}

impl AtomicOxygenErosionSpec {
    /// Validates atomic-oxygen surface erosion configuration.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_positive("ao.area_m2", self.area_m2)?;
        check_non_negative(
            "ao.erosion_yield_m3_per_atom",
            self.erosion_yield_m3_per_atom,
        )?;
        check_positive("ao.coating_density_kg_m3", self.coating_density_kg_m3)?;
        check_positive("ao.initial_thickness_m", self.initial_thickness_m)?;
        Ok(())
    }
}

/// Cumulative AO erosion state on a surface coating.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AtomicOxygenErosionState {
    /// Cumulative incident atomic-oxygen fluence, atoms/m².
    pub fluence_atoms_m2: f64,
    /// Eroded coating thickness, m.
    pub thickness_lost_m: f64,
    /// Eroded surface mass, kg.
    pub mass_lost_kg: f64,
}

/// Output emitted by [`step_atomic_oxygen_erosion`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AtomicOxygenErosionOutput {
    /// Instantaneous incident atomic-oxygen flux onto surface, atoms/(m²·s).
    pub ao_flux_atoms_m2_s: f64,
    /// Incremental surface mass eroded over `dt_s`, kg.
    pub mass_lost_step_kg: f64,
    /// Normalized remaining coating integrity fraction (`1.0` fresh -> `0.0` completely stripped).
    pub remaining_fraction: f64,
}

/// Steps atomic-oxygen surface erosion driven by `ambient.number_density_m3(GasSpecies::AtomicO)`.
pub fn step_atomic_oxygen_erosion(
    spec: &AtomicOxygenErosionSpec,
    state: &mut AtomicOxygenErosionState,
    dt_s: f64,
    ambient: &AmbientGas,
    airspeed_m_s: f64,
    ram_cos_incidence: f64,
) -> Result<AtomicOxygenErosionOutput, SystemsError> {
    spec.validate()?;
    ambient.validate()?;
    check_non_negative("ao.dt_s", dt_s)?;
    check_non_negative("ao.airspeed_m_s", airspeed_m_s)?;

    let cos_inc = ram_cos_incidence.clamp(0.0, 1.0);
    let n_ao = ambient.number_density_m3(GasSpecies::AtomicO);
    let ao_flux_atoms_m2_s = n_ao * airspeed_m_s * cos_inc;

    let mut mass_lost_step_kg = 0.0;
    if ao_flux_atoms_m2_s > 0.0 && dt_s > 0.0 {
        let d_fluence = ao_flux_atoms_m2_s * dt_s;
        state.fluence_atoms_m2 += d_fluence;
        let rem_thickness = (spec.initial_thickness_m - state.thickness_lost_m).max(0.0);
        let d_thickness = (d_fluence * spec.erosion_yield_m3_per_atom).min(rem_thickness);
        state.thickness_lost_m += d_thickness;
        mass_lost_step_kg = d_thickness * spec.area_m2 * spec.coating_density_kg_m3;
        state.mass_lost_kg += mass_lost_step_kg;
    }

    let remaining_fraction = ((spec.initial_thickness_m - state.thickness_lost_m)
        / spec.initial_thickness_m)
        .clamp(0.0, 1.0);

    Ok(AtomicOxygenErosionOutput {
        ao_flux_atoms_m2_s,
        mass_lost_step_kg,
        remaining_fraction,
    })
}
