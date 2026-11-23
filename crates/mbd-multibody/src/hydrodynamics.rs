use alloc::vec::Vec;

use glam::DVec3;
use mbd_core::{ComponentId, WrenchAtSite};

use crate::MultibodyError;
use crate::aba::FloatingBaseState;

#[cfg(test)]
mod tests;

/// Planar-local tangent approximation to a planetary ocean / liquid surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WaterState {
    /// Local mean water surface point directly beneath the vehicle, world coords, m.
    pub surface_origin_world_m: DVec3,
    /// Unit outward surface normal pointing out of water into air, world coords.
    pub outward_up_world: DVec3,
    /// Local gravitational acceleration magnitude, m/s².
    pub gravity_accel_m_s2: f64,
    /// Fluid density, kg/m³ (~`1025.0` for Earth seawater).
    pub fluid_density_kg_m3: f64,
    /// Surface water current velocity in world coords, m/s.
    pub fluid_vel_world_m_s: DVec3,
}

/// Submerged hull volume and slamming station attached to the capsule or vessel root.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HullStation {
    /// Station centroid in host component local frame, m.
    pub offset_local_m: DVec3,
    /// Projected cross-sectional area, m².
    pub cross_area_m2: f64,
    /// Vertical thickness over which volume submerges smoothly, m.
    pub thickness_m: f64,
    /// Hydrodynamic quadratic drag coefficient.
    pub drag_coeff: f64,
    /// Water-entry von Kármán impact slamming coefficient.
    pub slamming_coeff: f64,
}

/// Result of evaluating buoyancy, entry slamming deceleration, and water drag
/// across all hull stations (e.g., Gemini 8 and Apollo CM ocean splashdown).
#[derive(Debug, Clone, PartialEq)]
pub struct HydrodynamicsOutput {
    /// Local wrenches ready for multibody forward dynamics.
    pub wrenches: Vec<WrenchAtSite>,
    /// Total upward buoyant force magnitude, N.
    pub total_buoyancy_n: f64,
    /// Total hydrodynamic impact/viscous drag magnitude, N.
    pub total_drag_n: f64,
}

/// Discretized station hydrodynamics model supporting spherical planet up normals
/// (`outward_up_world`), Archimedes buoyancy, slamming impact, and drag/righting.
#[derive(Debug, Clone, PartialEq)]
pub struct HydrodynamicsModel {
    component: ComponentId,
    stations: Vec<HullStation>,
}

impl HydrodynamicsModel {
    /// Builds and validates a discretized multi-station splashdown hull model.
    pub fn new(component: ComponentId, stations: Vec<HullStation>) -> Result<Self, MultibodyError> {
        for s in &stations {
            if !s.offset_local_m.is_finite()
                || s.cross_area_m2 <= 0.0
                || s.thickness_m <= 0.0
                || s.drag_coeff < 0.0
                || s.slamming_coeff < 0.0
            {
                return Err(MultibodyError::InvalidParameter {
                    field: "hull_station",
                    value: s.cross_area_m2,
                });
            }
        }
        Ok(Self {
            component,
            stations,
        })
    }

    /// Host component carrying these hull buoyancy/slamming stations.
    pub const fn component(&self) -> ComponentId {
        self.component
    }

    /// Discretized hull stations.
    pub fn stations(&self) -> &[HullStation] {
        &self.stations
    }

    /// Evaluates buoyancy, slamming, and quadratic water drag at `base_state`.
    pub fn evaluate(
        &self,
        base_state: &FloatingBaseState,
        water: &WaterState,
    ) -> Result<HydrodynamicsOutput, MultibodyError> {
        if water.fluid_density_kg_m3 <= 0.0 || water.gravity_accel_m_s2 < 0.0 {
            return Err(MultibodyError::InvalidParameter {
                field: "water_state",
                value: water.fluid_density_kg_m3,
            });
        }
        let up = water.outward_up_world.normalize_or(DVec3::Z);
        let rot = base_state.rotation_world_from_root;
        let v_root_world = base_state.linear_vel_world_m_s();
        let w_root_world = base_state.angular_vel_world_rad_s();

        let mut wrenches = Vec::with_capacity(self.stations.len());
        let mut total_buoyancy_n = 0.0;
        let mut total_drag_n = 0.0;

        for st in &self.stations {
            let r_world = rot * st.offset_local_m;
            let pos_world = base_state.position_world_m + r_world;
            let depth = (water.surface_origin_world_m - pos_world).dot(up);
            if depth <= 0.0 {
                continue;
            }
            let frac = (depth / st.thickness_m).clamp(0.0, 1.0);
            let vol = st.cross_area_m2 * st.thickness_m * frac;
            let f_buoy = up * (water.fluid_density_kg_m3 * water.gravity_accel_m_s2 * vol);
            total_buoyancy_n += f_buoy.length();

            let v_station_world = v_root_world + w_root_world.cross(r_world);
            let v_rel = v_station_world - water.fluid_vel_world_m_s;
            let speed = v_rel.length();
            let area_eff = st.cross_area_m2 * frac;
            let f_drag =
                -v_rel * (0.5 * water.fluid_density_kg_m3 * st.drag_coeff * area_eff * speed);

            // Water-entry slamming impulse pressure peaks during partial crossing (`frac < 1`).
            let v_down = (-v_rel.dot(up)).max(0.0);
            let slam_window = (4.0 * frac * (1.0 - frac)).max(0.0);
            let f_slam = up
                * (0.5
                    * water.fluid_density_kg_m3
                    * st.slamming_coeff
                    * st.cross_area_m2
                    * v_down
                    * v_down
                    * slam_window);

            let f_hydro = f_drag + f_slam;
            total_drag_n += f_hydro.length();

            wrenches.push(WrenchAtSite {
                site: self.component,
                point_local_m: st.offset_local_m,
                force_local_n: rot.conjugate() * (f_buoy + f_hydro),
                torque_local_nm: DVec3::ZERO,
            });
        }

        Ok(HydrodynamicsOutput {
            wrenches,
            total_buoyancy_n,
            total_drag_n,
        })
    }
}
