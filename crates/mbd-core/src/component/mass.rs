use glam::{DMat3, DQuat, DVec3};

use super::{ComponentId, ComponentTree};
use crate::{DynamicMassContribution, MbdError, PoseTable};

/// Instantaneous mass, centre of gravity and inertia of everything attached.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassProperties {
    /// Total attached mass, kg.
    pub total_mass_kg: f64,
    /// Centre of gravity in root axes, metres.
    pub cg_body_m: DVec3,
    /// Inertia about the centre of gravity in root axes, kg·m².
    pub inertia_cg_body: DMat3,
}

/// One distributed or lumped mass in root axes.
struct Lump {
    mass_kg: f64,
    position_m: DVec3,
    rotation: DQuat,
    inertia_cg_local: DMat3,
}

impl Lump {
    /// `R I Rᵀ + m(|d|²E - d dᵀ)`: the tensor rotated into root axes, then
    /// transferred to the centre of gravity by the parallel-axis theorem.
    fn inertia_about(&self, cg_m: DVec3) -> DMat3 {
        let rotation = DMat3::from_quat(self.rotation);
        let d = self.position_m - cg_m;
        let transfer = DMat3::from_diagonal(DVec3::splat(d.length_squared()))
            - DMat3::from_cols(d * d.x, d * d.y, d * d.z);
        rotation * self.inertia_cg_local * rotation.transpose() + transfer * self.mass_kg
    }
}

impl ComponentTree {
    /// Aggregates every component that is attached along with all its
    /// ancestors, so a detached stage takes its whole subtree with it.
    pub fn mass_properties(&self) -> Result<MassProperties, MbdError> {
        self.mass_properties_with_contributions(&[])
    }

    /// Aggregates attached components together with live subsystem mass
    /// distributions (such as fluid tanks with dynamic fill-level CG shifts
    /// or shifted internal payloads).
    pub fn mass_properties_with_contributions(
        &self,
        extras: &[DynamicMassContribution],
    ) -> Result<MassProperties, MbdError> {
        let poses = self.evaluate_pose_table()?;
        let (base_mass, base_first) = self.sum_first_moments(&poses)?;
        let (extra_mass, extra_first) = sum_extra_first_moments(extras, &poses)?;
        let total_mass_kg = base_mass + extra_mass;
        if total_mass_kg <= 0.0 {
            return Err(MbdError::NoAttachedMass);
        }
        let cg_body_m = (base_first + extra_first) / total_mass_kg;
        let inertia_cg_body = self.sum_inertia_about(&poses, cg_body_m)?
            + sum_extra_inertia(extras, &poses, cg_body_m)?;
        Ok(MassProperties {
            total_mass_kg,
            cg_body_m,
            inertia_cg_body,
        })
    }

    /// `r_site - r_cg` in root axes for the current mass distribution. A site
    /// on a detached stage has no such arm and is an error.
    pub fn lever_arm_from_cg(&self, site: ComponentId) -> Result<DVec3, MbdError> {
        self.lever_arm_from_cg_with_contributions(site, &[])
    }

    /// `r_site - r_cg` accounting for dynamic subsystem mass contributions.
    pub fn lever_arm_from_cg_with_contributions(
        &self,
        site: ComponentId,
        extras: &[DynamicMassContribution],
    ) -> Result<DVec3, MbdError> {
        let pose = self.chain_pose(site)?;
        if !pose.attached {
            return Err(MbdError::DetachedComponent { id: site });
        }
        let cg = self.mass_properties_with_contributions(extras)?.cg_body_m;
        Ok(pose.translation - cg)
    }

    fn sum_first_moments(&self, poses: &PoseTable) -> Result<(f64, DVec3), MbdError> {
        let mut total_mass = 0.0;
        let mut first_moment = DVec3::ZERO;
        for id in self.ids() {
            if let Some(lump) = self.lump_from_pose(id, poses)? {
                total_mass += lump.mass_kg;
                first_moment += lump.position_m * lump.mass_kg;
            }
        }
        Ok((total_mass, first_moment))
    }

    fn sum_inertia_about(&self, poses: &PoseTable, cg_m: DVec3) -> Result<DMat3, MbdError> {
        let mut inertia = DMat3::ZERO;
        for id in self.ids() {
            if let Some(lump) = self.lump_from_pose(id, poses)? {
                inertia += lump.inertia_about(cg_m);
            }
        }
        Ok(inertia)
    }

    fn lump_from_pose(&self, id: ComponentId, poses: &PoseTable) -> Result<Option<Lump>, MbdError> {
        let pose = poses.get(id)?;
        if !pose.attached {
            return Ok(None);
        }
        let spec = self.spec(id)?;
        Ok(Some(Lump {
            mass_kg: self.state(id)?.mass_kg(spec),
            position_m: pose.position_root_m,
            rotation: pose.rotation_root_from_link,
            inertia_cg_local: spec.dry_inertia_body,
        }))
    }
}

fn sum_extra_first_moments(
    extras: &[DynamicMassContribution],
    poses: &PoseTable,
) -> Result<(f64, DVec3), MbdError> {
    let mut total_mass = 0.0;
    let mut first_moment = DVec3::ZERO;
    for extra in extras {
        extra.validate()?;
        let pose = poses.get(extra.component)?;
        if !pose.attached {
            continue;
        }
        let pos_root = pose.position_root_m + pose.rotation_root_from_link * extra.cg_local_m;
        total_mass += extra.mass_kg;
        first_moment += pos_root * extra.mass_kg;
    }
    Ok((total_mass, first_moment))
}

fn sum_extra_inertia(
    extras: &[DynamicMassContribution],
    poses: &PoseTable,
    cg_m: DVec3,
) -> Result<DMat3, MbdError> {
    let mut inertia = DMat3::ZERO;
    for extra in extras {
        let pose = poses.get(extra.component)?;
        if !pose.attached {
            continue;
        }
        let lump = Lump {
            mass_kg: extra.mass_kg,
            position_m: pose.position_root_m + pose.rotation_root_from_link * extra.cg_local_m,
            rotation: pose.rotation_root_from_link,
            inertia_cg_local: extra.inertia_cg_local,
        };
        inertia += lump.inertia_about(cg_m);
    }
    Ok(inertia)
}
