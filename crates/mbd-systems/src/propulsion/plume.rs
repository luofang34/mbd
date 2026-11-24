use alloc::vec::Vec;
use core::f64::consts::PI;

use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, ThermalLoad, WrenchAtSite};

use crate::error::{
    SystemsError, check_non_negative, check_positive, check_unit_interval, check_vec_finite,
    normalize_axis,
};

/// Thruster exhaust plume definition in root/world frame for impingement & ground-effect checks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlumeImpingementSpec {
    /// Thruster nozzle exit position in common reference frame (root or world), m.
    pub nozzle_origin_m: DVec3,
    /// Unit exhaust flow direction (opposite thrust vector), unitless.
    pub exhaust_dir: DVec3,
    /// Nozzle exit radius, m.
    pub nozzle_radius_m: f64,
    /// Effective plume divergence half-angle, rad (in `(0, pi/2)`).
    pub cone_half_angle_rad: f64,
    /// Current realized thruster force magnitude, N.
    pub thrust_n: f64,
    /// Total exhaust jet thermal power carried in plume, W.
    pub exhaust_thermal_power_w: f64,
}

impl PlumeImpingementSpec {
    /// Validates plume geometry and energy bounds.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_vec_finite("plume.nozzle_origin_m", self.nozzle_origin_m)?;
        normalize_axis("plume.exhaust_dir", self.exhaust_dir)?;
        check_positive("plume.nozzle_radius_m", self.nozzle_radius_m)?;
        if !self.cone_half_angle_rad.is_finite()
            || self.cone_half_angle_rad <= 0.0
            || self.cone_half_angle_rad >= 0.5 * PI
        {
            return Err(SystemsError::InvalidParameter {
                field: "plume.cone_half_angle_rad",
                value: self.cone_half_angle_rad,
            });
        }
        check_non_negative("plume.thrust_n", self.thrust_n)?;
        check_non_negative(
            "plume.exhaust_thermal_power_w",
            self.exhaust_thermal_power_w,
        )?;
        Ok(())
    }
}

/// Target surface element potentially struck by a thruster plume (e.g. solar array, docked module, rover deck).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlumeSurfaceTarget {
    /// Component receiving impingement force & thermal heating.
    pub target_site: ComponentId,
    /// Surface centroid in same reference frame as `PlumeImpingementSpec::nozzle_origin_m`, m.
    pub surface_center_m: DVec3,
    /// Application point in `target_site`'s local coordinates for emitted `WrenchAtSite`, m.
    pub point_local_m: DVec3,
    /// Unit quaternion rotating `target_site` local axes into the plume reference frame.
    pub rotation_root_from_target: DQuat,
    /// Unit outward surface normal in the common reference frame.
    pub outward_normal: DVec3,
    /// Surface exposed area, m².
    pub area_m2: f64,
    /// Thermal absorptivity fraction `0.0..=1.0`.
    pub thermal_absorptivity: f64,
}

impl PlumeSurfaceTarget {
    /// Validates surface target parameters.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_vec_finite("plume_target.surface_center_m", self.surface_center_m)?;
        check_vec_finite("plume_target.point_local_m", self.point_local_m)?;
        if !self.rotation_root_from_target.is_finite()
            || (self.rotation_root_from_target.length_squared() - 1.0).abs() > 1.0e-9
        {
            return Err(SystemsError::InvalidParameter {
                field: "plume_target.rotation_root_from_target",
                value: self.rotation_root_from_target.length_squared(),
            });
        }
        normalize_axis("plume_target.outward_normal", self.outward_normal)?;
        check_positive("plume_target.area_m2", self.area_m2)?;
        check_unit_interval(
            "plume_target.thermal_absorptivity",
            self.thermal_absorptivity,
        )?;
        Ok(())
    }
}

/// Aggregated structural impingement wrenches, thermal loads, and ground-effect cushion force.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct PlumeImpingementResult {
    /// Disturbance forces exerted on struck spacecraft targets (in common frame / target normal sense).
    pub target_wrenches: Vec<WrenchAtSite>,
    /// Heating rates deposited onto struck spacecraft surfaces.
    pub target_thermal_loads: Vec<ThermalLoad>,
    /// Ground-effect upward thrust augmentation factor when firing close to planetary surface, N.
    pub ground_cushion_force_n: f64,
    /// Backscattered ground-plume thermal load estimate onto lander base, W.
    pub ground_backscatter_heat_w: f64,
}

/// Evaluates conical thruster plume impingement against nearby spacecraft surfaces
/// and optional planetary surface ground-effect proximity (`altitude_agl_m`).
pub fn evaluate_plume_impingement(
    plume: &PlumeImpingementSpec,
    targets: &[PlumeSurfaceTarget],
    altitude_agl_m: Option<f64>,
) -> Result<PlumeImpingementResult, SystemsError> {
    plume.validate()?;
    if let Some(agl) = altitude_agl_m {
        check_non_negative("plume.altitude_agl_m", agl)?;
    }
    let exhaust = plume.exhaust_dir.normalize();
    let tan_half = libm::tan(plume.cone_half_angle_rad);
    let mut result = PlumeImpingementResult::default();

    if plume.thrust_n <= 0.0 {
        return Ok(result);
    }

    for target in targets {
        target.validate()?;
        let rel = target.surface_center_m - plume.nozzle_origin_m;
        let axial = rel.dot(exhaust);
        if axial <= 0.0 {
            continue;
        }
        let radial_vec = rel - exhaust * axial;
        let plume_radius = plume.nozzle_radius_m + axial * tan_half;
        let radial_dist = radial_vec.length();
        if radial_dist > plume_radius {
            continue;
        }
        let ray_dir = rel.normalize();
        let incidence = (-target.outward_normal.normalize()).dot(ray_dir);
        if incidence <= 0.0 {
            continue;
        }

        let radial_atten = 1.0 - (radial_dist / plume_radius) * (radial_dist / plume_radius);
        let footprint_area = PI * plume_radius * plume_radius;
        let dynamic_pressure = (plume.thrust_n / footprint_area) * 2.0 * radial_atten;
        let force_mag = dynamic_pressure * target.area_m2 * incidence;

        let force_local_n = target.rotation_root_from_target.inverse() * (ray_dir * force_mag);
        result.target_wrenches.push(WrenchAtSite {
            site: target.target_site,
            point_local_m: target.point_local_m,
            force_local_n,
            torque_local_nm: DVec3::ZERO,
        });
        let heat_flux = (plume.exhaust_thermal_power_w / footprint_area) * radial_atten * incidence;
        result.target_thermal_loads.push(ThermalLoad {
            component: target.target_site,
            heat_rate_w: heat_flux * target.area_m2 * target.thermal_absorptivity,
        });
    }

    if let Some(agl) = altitude_agl_m {
        let range = plume.nozzle_radius_m * 30.0;
        if agl < range {
            let ratio = 1.0 - (agl / range);
            result.ground_cushion_force_n = 0.22 * plume.thrust_n * ratio * ratio;
            result.ground_backscatter_heat_w = 0.05 * plume.exhaust_thermal_power_w * ratio * ratio;
        }
    }

    Ok(result)
}
