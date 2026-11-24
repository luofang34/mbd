use glam::DVec3;
use mbd_systems::{
    AbepThrusterSpec, AirbreathingEngineSpec, AtomicOxygenErosionSpec, ElectricThrusterSpec,
    LiquidRocketSpec, SolidMotorSpec, SystemsError, TankId, TpsAblationSpec,
};

use super::super::VerifiedSyspack;
use crate::SyspackError;

fn map_sys_err(err: SystemsError) -> SyspackError {
    SyspackError::InvalidSubsystem {
        detail: err.to_string(),
    }
}

impl VerifiedSyspack {
    /// Builds and validates all declared [`LiquidRocketSpec`] engines/thrusters.
    pub fn build_liquid_rockets(&self) -> Result<Vec<LiquidRocketSpec>, SyspackError> {
        let mut out = Vec::with_capacity(self.manifest.subsystems.liquid_rockets.len());
        for r in &self.manifest.subsystems.liquid_rockets {
            let spec = LiquidRocketSpec {
                site: self.component_id(&r.site)?,
                mount_offset_local_m: DVec3::from_array(r.mount_offset_local_m),
                thrust_axis_local: DVec3::from_array(r.thrust_axis_local),
                gimbal_max_rad: r.gimbal_max_rad,
                max_vacuum_thrust_n: r.max_vacuum_thrust_n,
                isp_vac_s: r.isp_vac_s,
                isp_sl_s: r.isp_sl_s,
                fuel_tank: r.fuel_tank.map(TankId),
                oxidizer_tank: r.oxidizer_tank.map(TankId),
                mixture_ratio_o_f: r.mixture_ratio_o_f,
                min_throttle: r.min_throttle,
                startup_tau_s: r.startup_tau_s,
                shutdown_tau_s: r.shutdown_tau_s,
                nominal_heat_to_site_w: r.nominal_heat_to_site_w,
            };
            spec.validate().map_err(map_sys_err)?;
            out.push(spec);
        }
        Ok(out)
    }

    /// Builds and validates all declared [`SolidMotorSpec`] motors.
    pub fn build_solid_motors(&self) -> Result<Vec<SolidMotorSpec>, SyspackError> {
        let mut out = Vec::with_capacity(self.manifest.subsystems.solid_motors.len());
        for m in &self.manifest.subsystems.solid_motors {
            let spec = SolidMotorSpec {
                site: self.component_id(&m.site)?,
                mount_offset_local_m: DVec3::from_array(m.mount_offset_local_m),
                grain_cg_local_m: DVec3::from_array(m.grain_cg_local_m),
                thrust_axis_local: DVec3::from_array(m.thrust_axis_local),
                propellant_mass_kg: m.propellant_mass_kg,
                isp_vac_s: m.isp_vac_s,
                nozzle_exit_area_m2: m.nozzle_exit_area_m2,
                burn_time_s: m.burn_time_s,
                tail_off_fraction: m.tail_off_fraction,
                grain_radius_m: m.grain_radius_m,
                grain_length_m: m.grain_length_m,
            };
            spec.validate().map_err(map_sys_err)?;
            out.push(spec);
        }
        Ok(out)
    }

    /// Builds and validates all declared [`ElectricThrusterSpec`] units.
    pub fn build_electric_thrusters(&self) -> Result<Vec<ElectricThrusterSpec>, SyspackError> {
        let mut out = Vec::with_capacity(self.manifest.subsystems.electric_thrusters.len());
        for e in &self.manifest.subsystems.electric_thrusters {
            let spec = ElectricThrusterSpec {
                site: self.component_id(&e.site)?,
                mount_offset_local_m: DVec3::from_array(e.mount_offset_local_m),
                thrust_axis_local: DVec3::from_array(e.thrust_axis_local),
                propellant_tank: TankId(e.propellant_tank),
                isp_s: e.isp_s,
                total_efficiency: e.total_efficiency,
                max_power_w: e.max_power_w,
                priority: e.priority,
            };
            spec.validate().map_err(map_sys_err)?;
            out.push(spec);
        }
        Ok(out)
    }

    /// Builds and validates all declared [`AbepThrusterSpec`] intakes/thrusters.
    pub fn build_abep_thrusters(&self) -> Result<Vec<AbepThrusterSpec>, SyspackError> {
        let mut out = Vec::with_capacity(self.manifest.subsystems.abep_thrusters.len());
        for a in &self.manifest.subsystems.abep_thrusters {
            let spec = AbepThrusterSpec {
                site: self.component_id(&a.site)?,
                mount_offset_local_m: DVec3::from_array(a.mount_offset_local_m),
                thrust_axis_local: DVec3::from_array(a.thrust_axis_local),
                intake_area_m2: a.intake_area_m2,
                atomic_o_collection_efficiency: a.atomic_o_collection_efficiency,
                n2_collection_efficiency: a.n2_collection_efficiency,
                isp_s: a.isp_s,
                total_efficiency: a.total_efficiency,
                max_power_w: a.max_power_w,
                buffer_tank: a.buffer_tank.map(TankId),
                priority: a.priority,
            };
            spec.validate().map_err(map_sys_err)?;
            out.push(spec);
        }
        Ok(out)
    }

    /// Builds and validates all declared [`AirbreathingEngineSpec`] engines.
    pub fn build_airbreathing_engines(&self) -> Result<Vec<AirbreathingEngineSpec>, SyspackError> {
        let mut out = Vec::with_capacity(self.manifest.subsystems.airbreathing_engines.len());
        for j in &self.manifest.subsystems.airbreathing_engines {
            let spec = AirbreathingEngineSpec {
                site: self.component_id(&j.site)?,
                mount_offset_local_m: DVec3::from_array(j.mount_offset_local_m),
                thrust_axis_local: DVec3::from_array(j.thrust_axis_local),
                kind: j.kind.to_systems(),
                sl_static_dry_thrust_n: j.sl_static_dry_thrust_n,
                dry_tsfc_kg_n_s: j.dry_tsfc_kg_n_s,
                reheat_thrust_multiplier: j.reheat_thrust_multiplier,
                reheat_tsfc_multiplier: j.reheat_tsfc_multiplier,
                sl_max_shaft_power_w: j.sl_max_shaft_power_w,
                shaft_bsfc_kg_w_s: j.shaft_bsfc_kg_w_s,
                max_operating_mach: j.max_operating_mach,
                density_lapse_exponent: j.density_lapse_exponent,
                min_oxidizer_mass_fraction: j.min_oxidizer_mass_fraction,
                fuel_tank: TankId(j.fuel_tank),
                spool_tau_s: j.spool_tau_s,
            };
            spec.validate().map_err(map_sys_err)?;
            out.push(spec);
        }
        Ok(out)
    }

    /// Builds and validates all declared [`TpsAblationSpec`] thermal protection shields.
    pub fn build_tps_shields(&self) -> Result<Vec<TpsAblationSpec>, SyspackError> {
        let mut out = Vec::with_capacity(self.manifest.subsystems.tps_shields.len());
        for tps in &self.manifest.subsystems.tps_shields {
            let spec = TpsAblationSpec {
                component: self.component_id(&tps.site)?,
                nose_radius_m: tps.nose_radius_m,
                shield_area_m2: tps.shield_area_m2,
                char_surface_temp_k: tps.char_surface_temp_k,
                emissivity: tps.emissivity,
                heat_of_ablation_j_kg: tps.heat_of_ablation_j_kg,
                backface_soak_fraction: tps.backface_soak_fraction,
                initial_ablator_mass_kg: tps.initial_ablator_mass_kg,
                radiative_heating_coeff: tps.radiative_heating_coeff,
            };
            spec.validate().map_err(map_sys_err)?;
            out.push(spec);
        }
        Ok(out)
    }

    /// Builds and validates all declared [`AtomicOxygenErosionSpec`] surface coatings.
    pub fn build_ao_coatings(&self) -> Result<Vec<AtomicOxygenErosionSpec>, SyspackError> {
        let mut out = Vec::with_capacity(self.manifest.subsystems.ao_coatings.len());
        for ao in &self.manifest.subsystems.ao_coatings {
            let spec = AtomicOxygenErosionSpec {
                component: self.component_id(&ao.site)?,
                area_m2: ao.area_m2,
                erosion_yield_m3_per_atom: ao.erosion_yield_m3_per_atom,
                coating_density_kg_m3: ao.coating_density_kg_m3,
                initial_thickness_m: ao.initial_thickness_m,
            };
            spec.validate().map_err(map_sys_err)?;
            out.push(spec);
        }
        Ok(out)
    }
}
