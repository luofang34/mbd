use serde::{Deserialize, Serialize};
use syspack::{DynamicsSpec, VerifiedSyspack};

use crate::error::DesignError;

#[cfg(test)]
mod tests;

/// Severity level emitted by [`audit_verified_syspack`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FindingSeverity {
    /// Physical invariant or engineering requirement satisfied.
    Pass,
    /// Non-fatal design advisory (e.g. a vehicle without a provenance table).
    Warning,
    /// Physical impossibility or hard contract failure.
    Fail,
}

/// One structured finding produced while auditing a [`VerifiedSyspack`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DesignFinding {
    /// Machine-readable diagnostic code (e.g. `"MASS_PROPERTIES"`, `"TANK_GEOMETRY_CAPACITY"`).
    pub code: String,
    /// Finding outcome level.
    pub severity: FindingSeverity,
    /// Detailed explanation including measured and expected bounds.
    pub detail: String,
}

/// Complete design-contract audit summary for a loaded package.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DesignAuditReport {
    /// Package slug from `manifest().id`.
    pub package_id: String,
    /// Ordered audit findings.
    pub findings: Vec<DesignFinding>,
}

impl DesignAuditReport {
    /// Returns `true` when zero findings have [`FindingSeverity::Fail`].
    pub fn passed(&self) -> bool {
        !self
            .findings
            .iter()
            .any(|f| f.severity == FindingSeverity::Fail)
    }
}

/// Performs cross-domain MBD verification on a [`VerifiedSyspack`]: mass properties,
/// tank geometric volume feasibility, subsystem builder consistency, and provenance coverage.
pub fn audit_verified_syspack(pack: &VerifiedSyspack) -> Result<DesignAuditReport, DesignError> {
    let manifest = pack.manifest();
    let mut findings = Vec::new();

    audit_mass_properties(pack, &mut findings);
    audit_tank_volumes(pack, &mut findings)?;
    audit_subsystem_builders(pack, &mut findings)?;
    audit_provenance(pack, &mut findings);

    Ok(DesignAuditReport {
        package_id: manifest.id.clone(),
        findings,
    })
}

fn audit_mass_properties(pack: &VerifiedSyspack, findings: &mut Vec<DesignFinding>) {
    if pack.manifest().dynamics == DynamicsSpec::None {
        findings.push(DesignFinding {
            code: "MASS_PROPERTIES".into(),
            severity: FindingSeverity::Pass,
            detail: "stationary system exempt from positive attached mass check".into(),
        });
        return;
    }
    match pack.tree().mass_properties() {
        Ok(props) if props.total_mass_kg > 0.0 => findings.push(DesignFinding {
            code: "MASS_PROPERTIES".into(),
            severity: FindingSeverity::Pass,
            detail: format!("positive attached mass {:.3} kg", props.total_mass_kg),
        }),
        Ok(props) => findings.push(DesignFinding {
            code: "MASS_PROPERTIES".into(),
            severity: FindingSeverity::Fail,
            detail: format!("non-positive vehicle mass {:.3} kg", props.total_mass_kg),
        }),
        Err(err) => findings.push(DesignFinding {
            code: "MASS_PROPERTIES".into(),
            severity: FindingSeverity::Fail,
            detail: format!("invalid mass properties: {err}"),
        }),
    }
}

fn audit_tank_volumes(
    pack: &VerifiedSyspack,
    findings: &mut Vec<DesignFinding>,
) -> Result<(), DesignError> {
    let fluids = pack.build_fluid_network()?;
    for tank_rec in &pack.manifest().subsystems.tanks {
        let geom = tank_rec.geometry.to_systems();
        let volume_m3 = geom.volume_m3();
        let density = tank_rec
            .fluid_density_kg_m3
            .unwrap_or_else(|| tank_rec.species.to_systems().nominal_density_kg_m3());
        let max_physical_kg = volume_m3 * density * 1.05;
        if tank_rec.capacity_mass_kg > max_physical_kg {
            findings.push(DesignFinding {
                code: "TANK_GEOMETRY_CAPACITY".into(),
                severity: FindingSeverity::Fail,
                detail: format!(
                    "tank {} capacity {:.2} kg exceeds 105% geometric fluid capacity {:.2} kg (vol={:.4} m3, rho={:.1} kg/m3)",
                    tank_rec.id, tank_rec.capacity_mass_kg, max_physical_kg, volume_m3, density
                ),
            });
        } else {
            findings.push(DesignFinding {
                code: "TANK_GEOMETRY_CAPACITY".into(),
                severity: FindingSeverity::Pass,
                detail: format!(
                    "tank {} fits geometry ({:.2} <= {:.2} kg)",
                    tank_rec.id, tank_rec.capacity_mass_kg, max_physical_kg
                ),
            });
        }
    }
    drop(fluids);
    Ok(())
}

fn audit_subsystem_builders(
    pack: &VerifiedSyspack,
    findings: &mut Vec<DesignFinding>,
) -> Result<(), DesignError> {
    let bus = pack.build_electrical_bus()?;
    let thermal = pack.build_thermal_network()?;
    let manipulators = pack.build_manipulators()?;
    let wheels = pack.build_wheel_specs()?;
    findings.push(DesignFinding {
        code: "SUBSYSTEM_BUILDERS".into(),
        severity: FindingSeverity::Pass,
        detail: format!(
            "verified {} battery(s), {} thermal node(s)/env, {} manipulator(s), {} wheel(s)",
            bus.batteries().len(),
            pack.manifest().subsystems.thermal_nodes.len(),
            manipulators.len(),
            wheels.len()
        ),
    });
    drop(thermal);
    Ok(())
}

fn audit_provenance(pack: &VerifiedSyspack, findings: &mut Vec<DesignFinding>) {
    match pack.provenance_table() {
        Some(table) => {
            let count = table.iter().count();
            let severity = if count > 0 {
                FindingSeverity::Pass
            } else {
                FindingSeverity::Warning
            };
            findings.push(DesignFinding {
                code: "PROVENANCE_TABLE".into(),
                severity,
                detail: format!("loaded provenance source table ({count} entries)"),
            });
        }
        None => findings.push(DesignFinding {
            code: "PROVENANCE_TABLE".into(),
            severity: FindingSeverity::Warning,
            detail: "vehicle declares no provenance_asset source table".into(),
        }),
    }
}
