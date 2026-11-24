use std::collections::BTreeSet;

use super::SubsystemManifest;
use crate::SyspackError;

impl SubsystemManifest {
    /// Validates every component key reference, tank reference, mechanism target,
    /// manipulator chain, and provenance asset path declared in subsystems.
    pub(crate) fn validate_refs(
        &self,
        components: &BTreeSet<&str>,
        check_declared: &dyn Fn(&str) -> Result<(), SyspackError>,
    ) -> Result<(), SyspackError> {
        let mut tank_ids = BTreeSet::new();
        for tank in &self.tanks {
            require_site("tank", &tank.site, components)?;
            if !tank_ids.insert(tank.id) {
                return Err(SyspackError::InvalidSubsystem {
                    detail: format!("duplicate tank id {}", tank.id),
                });
            }
        }
        for line in &self.transfer_lines {
            require_tank("transfer line", line.from_tank, &tank_ids)?;
            require_tank("transfer line", line.to_tank, &tank_ids)?;
        }
        for engine in &self.liquid_rockets {
            require_site("liquid rocket", &engine.site, components)?;
            if let Some(tid) = engine.fuel_tank {
                require_tank("liquid rocket fuel", tid, &tank_ids)?;
            }
            if let Some(tid) = engine.oxidizer_tank {
                require_tank("liquid rocket oxidizer", tid, &tank_ids)?;
            }
        }
        for motor in &self.solid_motors {
            require_site("solid motor", &motor.site, components)?;
        }
        for thruster in &self.electric_thrusters {
            require_site("electric thruster", &thruster.site, components)?;
            require_tank("electric thruster", thruster.propellant_tank, &tank_ids)?;
        }
        for abep in &self.abep_thrusters {
            require_site("abep thruster", &abep.site, components)?;
            if let Some(tid) = abep.buffer_tank {
                require_tank("abep buffer", tid, &tank_ids)?;
            }
        }
        for engine in &self.airbreathing_engines {
            require_site("airbreathing engine", &engine.site, components)?;
            require_tank("airbreathing fuel", engine.fuel_tank, &tank_ids)?;
        }
        self.validate_power_thermal_and_mech(components, &tank_ids, check_declared)
    }

    fn validate_power_thermal_and_mech(
        &self,
        components: &BTreeSet<&str>,
        tank_ids: &BTreeSet<u16>,
        check_declared: &dyn Fn(&str) -> Result<(), SyspackError>,
    ) -> Result<(), SyspackError> {
        for arr in &self.solar_arrays {
            require_site("solar array", &arr.site, components)?;
        }
        for bat in &self.batteries {
            require_site("battery", &bat.site, components)?;
        }
        for generator in &self.generators {
            require_site("power generator", &generator.site, components)?;
            if let super::PowerGeneratorKindRecord::FuelCell {
                h2_tank, o2_tank, ..
            } = generator.kind
            {
                if let Some(tid) = h2_tank {
                    require_tank("fuel cell h2", tid, tank_ids)?;
                }
                if let Some(tid) = o2_tank {
                    require_tank("fuel cell o2", tid, tank_ids)?;
                }
            }
        }
        self.validate_thermal_and_tps(components)?;
        self.validate_mechanisms_and_wheels(components, check_declared)
    }

    fn validate_thermal_and_tps(&self, components: &BTreeSet<&str>) -> Result<(), SyspackError> {
        let mut thermal_sites = BTreeSet::new();
        for node in &self.thermal_nodes {
            require_site("thermal node", &node.site, components)?;
            if !thermal_sites.insert(node.site.as_str()) {
                return Err(SyspackError::InvalidSubsystem {
                    detail: format!("duplicate thermal node site {:?}", node.site),
                });
            }
        }
        for link in &self.thermal_links {
            require_thermal_node(&link.node_a, &thermal_sites)?;
            require_thermal_node(&link.node_b, &thermal_sites)?;
        }
        for tps in &self.tps_shields {
            require_site("tps shield", &tps.site, components)?;
        }
        for ao in &self.ao_coatings {
            require_site("ao coating", &ao.site, components)?;
        }
        Ok(())
    }

    fn validate_mechanisms_and_wheels(
        &self,
        components: &BTreeSet<&str>,
        check_declared: &dyn Fn(&str) -> Result<(), SyspackError>,
    ) -> Result<(), SyspackError> {
        let mut mech_ids = BTreeSet::new();
        for mech in &self.mechanisms {
            if mech.id.is_empty() || !mech_ids.insert(mech.id.as_str()) {
                return Err(SyspackError::InvalidMultibody {
                    detail: format!("empty or duplicate mechanism id {:?}", mech.id),
                });
            }
            require_site("mechanism", &mech.site, components)?;
            if let super::MechanismKindRecord::ImpulseRelease {
                target_child: Some(child),
                ..
            } = &mech.kind
            {
                require_site("mechanism target_child", child, components)?;
            }
        }
        for arm in &self.manipulators {
            if arm.name.is_empty() || arm.joint_sites.is_empty() {
                return Err(SyspackError::InvalidMultibody {
                    detail: format!(
                        "manipulator {:?} must name links and non-empty name",
                        arm.name
                    ),
                });
            }
            require_site("manipulator base_site", &arm.base_site, components)?;
            for site in &arm.joint_sites {
                require_site("manipulator joint_site", site, components)?;
            }
        }
        for wheel in &self.wheels {
            require_site("wheel", &wheel.site, components)?;
        }
        if let Some(asset) = &self.provenance_asset {
            check_declared(asset)?;
        }
        Ok(())
    }
}

fn require_site(
    kind: &'static str,
    site: &str,
    components: &BTreeSet<&str>,
) -> Result<(), SyspackError> {
    if components.contains(site) {
        return Ok(());
    }
    Err(SyspackError::UnknownComponent {
        key: format!("{kind} references unknown component {site:?}"),
    })
}

fn require_tank(
    kind: &'static str,
    tank_id: u16,
    tanks: &BTreeSet<u16>,
) -> Result<(), SyspackError> {
    if tanks.contains(&tank_id) {
        return Ok(());
    }
    Err(SyspackError::InvalidSubsystem {
        detail: format!("{kind} references unknown tank id {tank_id}"),
    })
}

fn require_thermal_node(site: &str, nodes: &BTreeSet<&str>) -> Result<(), SyspackError> {
    if nodes.contains(site) {
        return Ok(());
    }
    Err(SyspackError::InvalidSubsystem {
        detail: format!("thermal link references unregistered thermal node {site:?}"),
    })
}
