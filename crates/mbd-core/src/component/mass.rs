use glam::{DMat3, DQuat, DVec3};

use super::{ComponentId, ComponentTree};
use crate::MbdError;

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

/// One component's mass concentrated at its origin, with its dry inertia.
struct Lump {
    mass_kg: f64,
    position_m: DVec3,
    rotation: DQuat,
    dry_inertia: DMat3,
}

impl Lump {
    /// `R I Rᵀ + m(|d|²E - d dᵀ)`: the tensor rotated into root axes, then
    /// transferred to the centre of gravity by the parallel-axis theorem.
    fn inertia_about(&self, cg_m: DVec3) -> DMat3 {
        let rotation = DMat3::from_quat(self.rotation);
        let d = self.position_m - cg_m;
        let transfer = DMat3::from_diagonal(DVec3::splat(d.length_squared()))
            - DMat3::from_cols(d * d.x, d * d.y, d * d.z);
        rotation * self.dry_inertia * rotation.transpose() + transfer * self.mass_kg
    }
}

impl ComponentTree {
    /// Aggregates every component that is attached along with all its
    /// ancestors, so a detached stage takes its whole subtree with it.
    pub fn mass_properties(&self) -> Result<MassProperties, MbdError> {
        let mut total_mass_kg = 0.0;
        let mut first_moment = DVec3::ZERO;
        for id in self.ids() {
            if let Some(lump) = self.lump(id)? {
                total_mass_kg += lump.mass_kg;
                first_moment += lump.position_m * lump.mass_kg;
            }
        }
        if total_mass_kg <= 0.0 {
            return Err(MbdError::NoAttachedMass);
        }
        let cg_body_m = first_moment / total_mass_kg;
        let mut inertia_cg_body = DMat3::ZERO;
        for id in self.ids() {
            if let Some(lump) = self.lump(id)? {
                inertia_cg_body += lump.inertia_about(cg_body_m);
            }
        }
        Ok(MassProperties {
            total_mass_kg,
            cg_body_m,
            inertia_cg_body,
        })
    }

    /// `r_site - r_cg` in root axes for the current mass distribution. A site
    /// on a detached stage has no such arm and is an error.
    pub fn lever_arm_from_cg(&self, site: ComponentId) -> Result<DVec3, MbdError> {
        let pose = self.chain_pose(site)?;
        if !pose.attached {
            return Err(MbdError::DetachedComponent { id: site });
        }
        Ok(pose.translation - self.mass_properties()?.cg_body_m)
    }

    fn lump(&self, id: ComponentId) -> Result<Option<Lump>, MbdError> {
        let pose = self.chain_pose(id)?;
        if !pose.attached {
            return Ok(None);
        }
        let spec = self.spec(id)?;
        Ok(Some(Lump {
            mass_kg: self.state(id)?.mass_kg(spec),
            position_m: pose.translation,
            rotation: pose.rotation,
            dry_inertia: spec.dry_inertia_body,
        }))
    }
}
