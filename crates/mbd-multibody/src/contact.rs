use glam::{DQuat, DVec3};
use mbd_core::{ComponentId, WrenchAtSite};

use crate::MultibodyError;

#[cfg(test)]
mod tests;

/// Physical medium under a planetary surface query (`Solid` ground vs `Liquid` ocean).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SurfaceMedium {
    /// Solid rock, regolith, runway, or pad surface.
    #[default]
    Solid,
    /// Water or liquid hydrocarbon ocean/lake surface.
    Liquid,
}

/// Terrain or surface elevation query result at a body contact point or wheel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceSample {
    /// Closest ground point in world axes, m.
    pub surface_point_world_m: DVec3,
    /// Outward surface normal unit vector in world axes.
    pub outward_normal_world: DVec3,
    /// Surface material class (`Solid` vs `Liquid`).
    pub medium: SurfaceMedium,
    /// Optional surface-provided static friction override.
    pub mu_static_override: Option<f64>,
    /// Optional surface-provided kinetic friction override.
    pub mu_dynamic_override: Option<f64>,
}

/// Specification of one deformable solid contact skid, footpad, or crash point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactPointSpec {
    /// Host component owning this contact point.
    pub component: ComponentId,
    /// Point offset in `component` local frame, m.
    pub offset_local_m: DVec3,
    /// Normal penalty contact stiffness, N/m.
    pub stiffness_n_m: f64,
    /// Normal Hunt-Crossley / Kelvin damping coefficient, N·s/m.
    pub damping_n_s_m: f64,
    /// Static Coulomb friction coefficient.
    pub mu_static: f64,
    /// Dynamic kinetic Coulomb friction coefficient.
    pub mu_dynamic: f64,
    /// Tangential bristle stiction spring stiffness, N/m.
    pub bristle_stiffness_n_m: f64,
    /// Tangential bristle viscous damping, N·s/m.
    pub bristle_damping_n_s_m: f64,
}

/// Runtime stick-slip state of a [`ContactPointSpec`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ContactPointState {
    /// Whether the point is currently loaded against solid terrain.
    pub in_contact: bool,
    /// Integrated tangential bristle deflection in world coordinates, m.
    pub tangent_deflection_world_m: DVec3,
}

/// Output of one contact step in both world and local host axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactStepOutput {
    /// Reaction wrench ready for multibody dynamics.
    pub wrench: WrenchAtSite,
    /// Normal load magnitude, N (`0.0` when off ground).
    pub normal_force_n: f64,
    /// Penetration depth below ground surface, m (`0.0` if airborne).
    pub penetration_m: f64,
    /// True if tangential reaction is held in static friction stick mode.
    pub is_sticking: bool,
}

/// State machine latching a stationary landed spacecraft/lander cleanly on terrain
/// (`Free` → `Settling` → `Pinned`) and releasing immediately on liftoff thrust.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum PinnedSurfaceRest {
    /// Unpinned free-body motion.
    #[default]
    Free,
    /// Settled near rest, accumulating dwell time before pinning.
    Settling {
        /// Elapsed continuous settle duration, s.
        elapsed_s: f64,
    },
    /// Mechanically rested/pinned at `world_pos_m` and `world_rot` against micro-jitter.
    Pinned {
        /// Locked world root position, m.
        world_pos_m: DVec3,
        /// Locked world root orientation.
        world_rot: DQuat,
    },
}

/// Evaluates unilateral Hunt-Crossley normal contact and LuGre/bristle stick-slip
/// friction at one body point against `surface`.
pub fn evaluate_contact_point(
    spec: &ContactPointSpec,
    state: &mut ContactPointState,
    point_world_m: DVec3,
    vel_world_m_s: DVec3,
    surface: &SurfaceSample,
    rot_world_from_link: DQuat,
    dt_s: f64,
) -> Result<ContactStepOutput, MultibodyError> {
    if !dt_s.is_finite() || dt_s < 0.0 {
        return Err(MultibodyError::InvalidParameter {
            field: "dt_s",
            value: dt_s,
        });
    }
    let normal = surface.outward_normal_world.normalize_or(DVec3::Z);
    let penetration = (surface.surface_point_world_m - point_world_m).dot(normal);
    if surface.medium != SurfaceMedium::Solid || penetration <= 0.0 {
        state.in_contact = false;
        state.tangent_deflection_world_m = DVec3::ZERO;
        return Ok(ContactStepOutput {
            wrench: WrenchAtSite::zero(spec.component),
            normal_force_n: 0.0,
            penetration_m: 0.0,
            is_sticking: false,
        });
    }

    state.in_contact = true;
    let v_n = vel_world_m_s.dot(normal);
    let ramp = (penetration * 100.0).min(1.0);
    let fn_mag = (spec.stiffness_n_m * penetration - spec.damping_n_s_m * v_n * ramp).max(0.0);

    let v_tan = vel_world_m_s - normal * v_n;
    let z = state.tangent_deflection_world_m + v_tan * dt_s;
    let z_tan = z - normal * z.dot(normal);
    let f_t_trial = -(z_tan * spec.bristle_stiffness_n_m + v_tan * spec.bristle_damping_n_s_m);

    let mu_s = surface.mu_static_override.unwrap_or(spec.mu_static);
    let mu_d = surface.mu_dynamic_override.unwrap_or(spec.mu_dynamic);
    let f_t_mag = f_t_trial.length();
    let max_static = mu_s * fn_mag;
    let (f_t, sticking) = if f_t_mag <= max_static || f_t_mag == 0.0 {
        state.tangent_deflection_world_m = z_tan;
        (f_t_trial, true)
    } else {
        let max_kinetic = mu_d * fn_mag;
        let dir = f_t_trial / f_t_mag;
        if spec.bristle_stiffness_n_m > 0.0 {
            state.tangent_deflection_world_m = -dir * (max_kinetic / spec.bristle_stiffness_n_m);
        }
        (dir * max_kinetic, false)
    };

    let f_world = normal * fn_mag + f_t;
    Ok(ContactStepOutput {
        wrench: WrenchAtSite {
            site: spec.component,
            point_local_m: spec.offset_local_m,
            force_local_n: rot_world_from_link.conjugate() * f_world,
            torque_local_nm: DVec3::ZERO,
        },
        normal_force_n: fn_mag,
        penetration_m: penetration,
        is_sticking: sticking,
    })
}

/// Updates [`PinnedSurfaceRest`] latch for stationary lunar/planetary landers.
#[allow(clippy::too_many_arguments)]
pub fn step_pinned_surface_rest(
    rest: &mut PinnedSurfaceRest,
    any_contact: bool,
    speed_m_s: f64,
    ang_rate_rad_s: f64,
    world_pos_m: DVec3,
    world_rot: DQuat,
    net_non_grav_upward_n: f64,
    breakout_threshold_n: f64,
    dwell_required_s: f64,
    dt_s: f64,
) {
    match *rest {
        PinnedSurfaceRest::Free => {
            if any_contact
                && speed_m_s < 0.03
                && ang_rate_rad_s < 0.02
                && net_non_grav_upward_n < breakout_threshold_n
            {
                *rest = PinnedSurfaceRest::Settling { elapsed_s: 0.0 };
            }
        }
        PinnedSurfaceRest::Settling { elapsed_s } => {
            if !any_contact || speed_m_s > 0.08 || net_non_grav_upward_n >= breakout_threshold_n {
                *rest = PinnedSurfaceRest::Free;
            } else if elapsed_s + dt_s >= dwell_required_s {
                *rest = PinnedSurfaceRest::Pinned {
                    world_pos_m,
                    world_rot,
                };
            } else {
                *rest = PinnedSurfaceRest::Settling {
                    elapsed_s: elapsed_s + dt_s,
                };
            }
        }
        PinnedSurfaceRest::Pinned { .. } => {
            if net_non_grav_upward_n >= breakout_threshold_n {
                *rest = PinnedSurfaceRest::Free;
            }
        }
    }
}
