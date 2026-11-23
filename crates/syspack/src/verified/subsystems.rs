mod builders;

use glam::DVec3;
use mbd_aero::CompositeAeroModel;
use mbd_multibody::{ManipulatorChain, MultibodyError, WheelSpec};
use mbd_systems::{
    BatterySpec, BatteryState, ElectricalBus, FluidNetwork, PowerGeneratorKind, PowerGeneratorSpec,
    PowerGeneratorState, SolarArraySpec, SolarArrayState, SystemsError, TankId, TankSpec,
    TankState, ThermalLinkSpec, ThermalNetwork, ThermalNodeSpec, TransferLineSpec,
    TransferLineState,
};

use super::VerifiedSyspack;
use crate::{DynamicsSpec, PowerGeneratorKindRecord, SyspackError};

fn map_sys_err(err: SystemsError) -> SyspackError {
    SyspackError::InvalidSubsystem {
        detail: err.to_string(),
    }
}

fn map_mb_err(err: MultibodyError) -> SyspackError {
    SyspackError::InvalidMultibody {
        detail: err.to_string(),
    }
}

impl VerifiedSyspack {
    /// Builds and validates a [`FluidNetwork`] populated from `manifest.subsystems.tanks`
    /// and `manifest.subsystems.transfer_lines`.
    pub fn build_fluid_network(&self) -> Result<FluidNetwork, SyspackError> {
        let mut net = FluidNetwork::new();
        for tank in &self.manifest.subsystems.tanks {
            let component = self.component_id(&tank.site)?;
            let species = tank.species.to_systems();
            let density = tank
                .fluid_density_kg_m3
                .unwrap_or_else(|| species.nominal_density_kg_m3());
            let spec = TankSpec {
                id: TankId(tank.id),
                component,
                center_local_m: DVec3::from_array(tank.center_local_m),
                geometry: tank.geometry.to_systems(),
                species,
                fluid_density_kg_m3: density,
                capacity_mass_kg: tank.capacity_mass_kg,
                minimum_unusable_kg: tank.minimum_unusable_kg,
                feed_regime: tank.feed_regime.to_systems(),
                slosh: tank.slosh.map(|s| s.to_systems()),
                passive_heat_leak_w: tank.passive_heat_leak_w,
            };
            let state = TankState::nominal(
                tank.initial_mass_kg,
                tank.initial_pressure_pa,
                tank.initial_temperature_k,
            );
            net.add_tank(spec, state).map_err(map_sys_err)?;
        }
        for line in &self.manifest.subsystems.transfer_lines {
            let spec = TransferLineSpec {
                id: line.id,
                from_tank: TankId(line.from_tank),
                to_tank: TankId(line.to_tank),
                max_flow_rate_kg_s: line.max_flow_rate_kg_s,
                power_w_per_kg_s: line.power_w_per_kg_s,
                bidirectional: line.bidirectional,
                requires_attachment: line.requires_attachment,
                priority: line.priority,
            };
            net.add_transfer_line(spec, TransferLineState::default())
                .map_err(map_sys_err)?;
        }
        Ok(net)
    }

    /// Builds and validates an [`ElectricalBus`] populated from solar arrays, batteries,
    /// and generators in `manifest.subsystems`.
    pub fn build_electrical_bus(&self) -> Result<ElectricalBus, SyspackError> {
        let bus_v = self
            .manifest
            .subsystems
            .nominal_bus_voltage_v
            .unwrap_or(28.0);
        let mut bus = ElectricalBus::new(bus_v).map_err(map_sys_err)?;
        for arr in &self.manifest.subsystems.solar_arrays {
            let component = self.component_id(&arr.site)?;
            bus.add_solar_array(
                SolarArraySpec {
                    component,
                    area_m2: arr.area_m2,
                    normal_local: DVec3::from_array(arr.normal_local),
                    efficiency: arr.efficiency,
                    packing_factor: arr.packing_factor,
                    annual_degradation: arr.annual_degradation,
                    nominal_voltage_v: arr.nominal_voltage_v,
                },
                SolarArrayState {
                    deployed_fraction: arr.initial_deployed_fraction,
                    age_years: arr.initial_age_years,
                    ..SolarArrayState::default()
                },
            )
            .map_err(map_sys_err)?;
        }
        for bat in &self.manifest.subsystems.batteries {
            let component = self.component_id(&bat.site)?;
            let spec = BatterySpec {
                component,
                capacity_wh: bat.capacity_wh,
                nominal_voltage_v: bat.nominal_voltage_v,
                max_charge_w: bat.max_charge_w,
                max_discharge_w: bat.max_discharge_w,
                charge_efficiency: bat.charge_efficiency,
                discharge_efficiency: bat.discharge_efficiency,
                min_soc: bat.min_soc,
                internal_resistance_ohm: bat.internal_resistance_ohm,
            };
            let energy_wh = bat.initial_energy_wh.unwrap_or(bat.capacity_wh);
            bus.add_battery(
                spec,
                BatteryState {
                    energy_wh,
                    ..BatteryState::full(&spec)
                },
            )
            .map_err(map_sys_err)?;
        }
        for generator in &self.manifest.subsystems.generators {
            self.add_generator_to_bus(&mut bus, generator)?;
        }
        Ok(bus)
    }

    fn add_generator_to_bus(
        &self,
        bus: &mut ElectricalBus,
        generator: &crate::PowerGeneratorRecord,
    ) -> Result<(), SyspackError> {
        let component = self.component_id(&generator.site)?;
        let kind = match generator.kind {
            PowerGeneratorKindRecord::Rtg {
                bol_electrical_w,
                bol_thermal_w,
                half_life_years,
            } => PowerGeneratorKind::Rtg {
                bol_electrical_w,
                bol_thermal_w,
                half_life_years,
            },
            PowerGeneratorKindRecord::FuelCell {
                max_power_w,
                waste_heat_ratio,
                specific_reactant_kg_wh,
                h2_tank,
                o2_tank,
            } => PowerGeneratorKind::FuelCell {
                max_power_w,
                waste_heat_ratio,
                specific_reactant_kg_wh,
                h2_tank: h2_tank.map(TankId),
                o2_tank: o2_tank.map(TankId),
            },
            PowerGeneratorKindRecord::ShaftAlternator {
                max_power_w,
                min_shaft_power_w,
                efficiency,
            } => PowerGeneratorKind::ShaftAlternator {
                max_power_w,
                min_shaft_power_w,
                efficiency,
            },
        };
        bus.add_generator(
            PowerGeneratorSpec {
                component,
                kind,
                nominal_voltage_v: generator.nominal_voltage_v,
            },
            PowerGeneratorState {
                enabled: generator.enabled,
                ..PowerGeneratorState::default()
            },
        )
        .map_err(map_sys_err)
    }

    /// Builds and validates a [`ThermalNetwork`] from `thermal_nodes` and `thermal_links`.
    pub fn build_thermal_network(&self) -> Result<ThermalNetwork, SyspackError> {
        let mut net = ThermalNetwork::new();
        for node in &self.manifest.subsystems.thermal_nodes {
            let component = self.component_id(&node.site)?;
            net.add_node(
                ThermalNodeSpec {
                    component,
                    heat_capacity_j_k: node.heat_capacity_j_k,
                    radiative_area_m2: node.radiative_area_m2,
                    emissivity: node.emissivity,
                    solar_absorptivity: node.solar_absorptivity,
                    solar_area_m2: node.solar_area_m2,
                    convective_area_m2: node.convective_area_m2,
                    min_operating_temp_k: node.min_operating_temp_k,
                    max_operating_temp_k: node.max_operating_temp_k,
                },
                node.initial_temp_k,
            )
            .map_err(map_sys_err)?;
        }
        for link in &self.manifest.subsystems.thermal_links {
            net.add_link(ThermalLinkSpec {
                node_a: self.component_id(&link.node_a)?,
                node_b: self.component_id(&link.node_b)?,
                conductance_w_k: link.conductance_w_k,
                pumped_loop: link.pumped_loop,
                pump_power_w: link.pump_power_w,
                requires_attachment: link.requires_attachment,
                priority: link.priority,
            })
            .map_err(map_sys_err)?;
        }
        Ok(net)
    }

    /// Builds a validated [`CompositeAeroModel`] when `dynamics` is [`DynamicsSpec::MultibodyComposed`].
    pub fn build_composite_aero(&self) -> Result<CompositeAeroModel, SyspackError> {
        let DynamicsSpec::MultibodyComposed { aero_elements } = &self.manifest.dynamics else {
            return Err(SyspackError::DynamicsMismatch {
                operation: "build_composite_aero",
                needed: "MultibodyComposed",
                declared: self.manifest.dynamics.kind(),
            });
        };
        let mut model = CompositeAeroModel::default();
        for elem in aero_elements {
            let comp = self.component_id(&elem.site)?;
            model.push(comp, elem.kind.build()?);
        }
        Ok(model)
    }

    /// Builds and validates all declared robotic manipulator kinematic chains against `self.tree()`.
    pub fn build_manipulators(&self) -> Result<Vec<(String, ManipulatorChain)>, SyspackError> {
        let mut out = Vec::with_capacity(self.manifest.subsystems.manipulators.len());
        for arm in &self.manifest.subsystems.manipulators {
            let _base = self.component_id(&arm.base_site)?;
            let mut joint_ids = Vec::with_capacity(arm.joint_sites.len());
            for key in &arm.joint_sites {
                joint_ids.push(self.component_id(key)?);
            }
            let Some(&ee_comp) = joint_ids.last() else {
                return Err(SyspackError::InvalidMultibody {
                    detail: format!("manipulator {:?} has no joint sites", arm.name),
                });
            };
            let chain = ManipulatorChain::new(
                &self.tree,
                joint_ids,
                ee_comp,
                DVec3::from_array(arm.end_effector_offset_m),
            )
            .map_err(map_mb_err)?;
            out.push((arm.name.clone(), chain));
        }
        Ok(out)
    }

    /// Converts declared wheel records into validated host [`WheelSpec`] structures.
    pub fn build_wheel_specs(&self) -> Result<Vec<WheelSpec>, SyspackError> {
        let mut out = Vec::with_capacity(self.manifest.subsystems.wheels.len());
        for w in &self.manifest.subsystems.wheels {
            let spin = DVec3::from_array(w.spin_axis_local);
            let steer = DVec3::from_array(w.steer_axis_local);
            let fwd = steer.cross(spin).normalize_or_zero();
            if !w.radius_m.is_finite() || w.radius_m <= 0.0 || fwd.length_squared() <= 1.0e-12 {
                return Err(SyspackError::InvalidMultibody {
                    detail: format!("wheel on site {:?} has invalid radius or axes", w.site),
                });
            }
            out.push(WheelSpec {
                component: self.component_id(&w.site)?,
                hub_offset_local_m: DVec3::from_array(w.hub_offset_local_m),
                wheel_radius_m: w.radius_m,
                spring_n_m: 15_000.0,
                damper_n_s_m: 1_500.0,
                forward_axis_local: fwd,
                axle_axis_local: spin.normalize(),
                peak_mu: 0.8,
                cornering_stiffness_n_rad: w.cornering_stiffness_n_rad,
                rolling_resistance_coeff: 0.02,
                longitudinal_stiffness_n: 5_000.0,
            });
        }
        Ok(out)
    }

    pub(super) fn validate_all_subsystems(&self) -> Result<(), SyspackError> {
        drop(self.build_fluid_network()?);
        drop(self.build_electrical_bus()?);
        drop(self.build_thermal_network()?);
        drop(self.build_manipulators()?);
        drop(self.build_wheel_specs()?);
        drop(self.build_liquid_rockets()?);
        drop(self.build_solid_motors()?);
        drop(self.build_electric_thrusters()?);
        drop(self.build_abep_thrusters()?);
        drop(self.build_airbreathing_engines()?);
        drop(self.build_tps_shields()?);
        drop(self.build_ao_coatings()?);
        if matches!(
            self.manifest.dynamics,
            DynamicsSpec::MultibodyComposed { .. }
        ) {
            drop(self.build_composite_aero()?);
        }
        Ok(())
    }
}
