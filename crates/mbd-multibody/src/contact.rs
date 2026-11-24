//! Penalty contact of a body point against a surface: unilateral normal
//! spring-damper and bristle stick-slip friction.
//!
//! The bristle is a spring `k_b` and damper `c_b` in parallel carrying the
//! tangential force, `f = −(k_b z + c_b ż)`. While the Coulomb bound is not
//! reached the bristle follows the surface (`ż = v_tan`); once it is, the
//! force is held on the bound and the same relation gives the rate at which
//! the bristle relaxes onto it. Which bound applies, static or kinetic, is a
//! latch that changes only at step boundaries, so the force is a smooth
//! function of the state within a step.

use glam::DVec3;
use mbd_core::{ComponentId, WrenchAtSite};

use crate::error::EnvironmentError;
use crate::model::PointKinematics;

mod element;
mod rest;

#[cfg(test)]
mod tests;

pub use rest::{PinnedRestInputs, PinnedRestSpec, PinnedSurfaceRest, step_pinned_surface_rest};

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
    /// Inertial velocity of the surface material at that point in world
    /// axes, m/s: the ground of a spinning planet, a deck, a conveyor.
    /// Penetration rate and slip are measured against it.
    pub velocity_world_m_s: DVec3,
    /// Surface material class (`Solid` vs `Liquid`).
    pub medium: SurfaceMedium,
    /// Optional surface-provided static friction override.
    pub mu_static_override: Option<f64>,
    /// Optional surface-provided kinetic friction override.
    pub mu_dynamic_override: Option<f64>,
}

/// Host input answering "what surface lies under this point at this
/// time". It is queried at every integrator stage with the stage's own
/// time, so the surface follows the point as it moves within a step and a
/// moving surface is where it is at that instant.
pub trait SurfaceQuery {
    /// Surface nearest to `point_world_m` at `time_s`. A failure aborts
    /// the evaluation as [`crate::MultibodyError::Environment`].
    fn surface_at(
        &self,
        point_world_m: DVec3,
        time_s: f64,
    ) -> Result<SurfaceSample, EnvironmentError>;
}

/// A single plane is its own answer everywhere and always.
impl SurfaceQuery for SurfaceSample {
    fn surface_at(
        &self,
        _point_world_m: DVec3,
        _time_s: f64,
    ) -> Result<SurfaceSample, EnvironmentError> {
        Ok(*self)
    }
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
    /// Penetration over which normal damping ramps in from zero, m, so the
    /// damper cannot pull on a point that is only grazing the surface. It
    /// should be of the order of the static penetration;
    /// [`ContactPointSpec::DEFAULT_DAMPING_RAMP_DEPTH_M`] suits a vehicle
    /// that sinks centimetres into its contacts.
    pub damping_ramp_depth_m: f64,
}

/// A supported body, from which a contact's stiffness and damping follow.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SupportedLoad {
    /// Mass resting on the contact point, kg.
    pub mass_kg: f64,
    /// Size of the body, m.
    pub size_m: f64,
    /// Gravitational acceleration pressing it down, m/s².
    pub gravity_m_s2: f64,
    /// Static penetration as a fraction of `size_m`.
    pub penetration_fraction: f64,
    /// Damping ratio of the normal spring against `mass_kg`.
    pub damping_ratio: f64,
}

impl ContactPointSpec {
    /// Damping ramp depth of a vehicle-scale contact, m.
    pub const DEFAULT_DAMPING_RAMP_DEPTH_M: f64 = 0.01;

    /// A contact sized from the body it supports: the stiffness gives the
    /// stated static penetration under the body's weight, the damping the
    /// stated ratio against its mass, the bristle shares both, and the
    /// damping ramp spans half the static penetration, so a body at rest is
    /// fully damped. Nothing in it is a dimensional constant, so the
    /// response is the same at any scale.
    pub fn sized_for(
        component: ComponentId,
        offset_local_m: DVec3,
        load: &SupportedLoad,
        mu_static: f64,
        mu_dynamic: f64,
    ) -> Self {
        let depth = load.penetration_fraction * load.size_m;
        let stiffness = load.mass_kg * load.gravity_m_s2 / depth;
        let damping = 2.0 * load.damping_ratio * libm::sqrt(stiffness * load.mass_kg);
        Self {
            component,
            offset_local_m,
            stiffness_n_m: stiffness,
            damping_n_s_m: damping,
            mu_static,
            mu_dynamic,
            bristle_stiffness_n_m: stiffness,
            bristle_damping_n_s_m: damping,
            damping_ramp_depth_m: 0.5 * depth,
        }
    }
}

/// Stick-slip state of a [`ContactPointSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactPointState {
    /// Friction latch: the static bound applies while set, the kinetic bound
    /// while clear. Changed only by [`ContactPointSpec::transition`].
    pub sticking: bool,
    /// Tangential bristle deflection in world coordinates, m.
    pub tangent_deflection_world_m: DVec3,
}

impl Default for ContactPointState {
    fn default() -> Self {
        Self {
            sticking: true,
            tangent_deflection_world_m: DVec3::ZERO,
        }
    }
}

/// Loads and state rate of one contact point at one state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactEvaluation {
    /// Reaction force on the body point in world axes, N.
    pub force_world_n: DVec3,
    /// Normal load magnitude, N (`0.0` when off ground).
    pub normal_force_n: f64,
    /// Penetration depth below ground surface, m (`0.0` if airborne).
    pub penetration_m: f64,
    /// True while the tangential force is below the Coulomb bound in force.
    pub is_sticking: bool,
    /// Rate of the bristle deflection in world axes, m/s.
    pub deflection_rate_world_m_s: DVec3,
}

impl ContactEvaluation {
    /// The reaction as a wrench at the contact point of `spec`.
    pub fn wrench(&self, spec: &ContactPointSpec, point: &PointKinematics) -> WrenchAtSite {
        WrenchAtSite {
            site: spec.component,
            point_local_m: spec.offset_local_m,
            force_local_n: point.to_link(self.force_world_n),
            torque_local_nm: DVec3::ZERO,
        }
    }
}

/// Normal and tangential quantities shared by evaluation and transition.
struct Engagement {
    normal: DVec3,
    penetration: f64,
    normal_force: f64,
    v_tan: DVec3,
    z_tan: DVec3,
    /// Bristle force before the Coulomb bound.
    f_trial: DVec3,
    mu_static: f64,
    mu_dynamic: f64,
}

impl ContactPointSpec {
    fn engage(
        &self,
        state: &ContactPointState,
        point: &PointKinematics,
        surface: &SurfaceSample,
    ) -> Option<Engagement> {
        let normal = surface.outward_normal_world.normalize_or(DVec3::Z);
        let penetration = (surface.surface_point_world_m - point.position_world_m).dot(normal);
        if surface.medium != SurfaceMedium::Solid || penetration <= 0.0 {
            return None;
        }
        let v_rel = point.velocity_world_m_s - surface.velocity_world_m_s;
        let v_n = v_rel.dot(normal);
        let ramp = if self.damping_ramp_depth_m > 0.0 {
            (penetration / self.damping_ramp_depth_m).min(1.0)
        } else {
            1.0
        };
        let normal_force =
            (self.stiffness_n_m * penetration - self.damping_n_s_m * v_n * ramp).max(0.0);
        let v_tan = v_rel - normal * v_n;
        let z = state.tangent_deflection_world_m;
        let z_tan = z - normal * z.dot(normal);
        Some(Engagement {
            normal,
            penetration,
            normal_force,
            v_tan,
            z_tan,
            f_trial: -(z_tan * self.bristle_stiffness_n_m + v_tan * self.bristle_damping_n_s_m),
            mu_static: surface.mu_static_override.unwrap_or(self.mu_static),
            mu_dynamic: surface.mu_dynamic_override.unwrap_or(self.mu_dynamic),
        })
    }

    /// Reaction force and bristle rate at one state: a pure function of its
    /// arguments, valid at any integrator stage.
    pub fn evaluate(
        &self,
        state: &ContactPointState,
        point: &PointKinematics,
        surface: &SurfaceSample,
    ) -> ContactEvaluation {
        let Some(e) = self.engage(state, point, surface) else {
            return ContactEvaluation {
                force_world_n: DVec3::ZERO,
                normal_force_n: 0.0,
                penetration_m: 0.0,
                is_sticking: false,
                deflection_rate_world_m_s: DVec3::ZERO,
            };
        };
        let mu = if state.sticking {
            e.mu_static
        } else {
            e.mu_dynamic
        };
        let bound = mu * e.normal_force;
        let trial = e.f_trial.length();
        let (f_tan, rate, sticking) = if trial <= bound || trial == 0.0 {
            (e.f_trial, e.v_tan, true)
        } else {
            let dir = e.f_trial / trial;
            let f_tan = dir * bound;
            let rate = if self.bristle_damping_n_s_m > 0.0 {
                -(f_tan + e.z_tan * self.bristle_stiffness_n_m) / self.bristle_damping_n_s_m
            } else {
                // An undamped bristle has no relaxation rate; it stops
                // stretching against the bound and the transition clamps it.
                e.v_tan + dir * (-e.v_tan.dot(dir)).max(0.0)
            };
            (f_tan, rate, false)
        };
        ContactEvaluation {
            force_world_n: e.normal * e.normal_force + f_tan,
            normal_force_n: e.normal_force,
            penetration_m: e.penetration,
            is_sticking: sticking,
            deflection_rate_world_m_s: rate,
        }
    }

    /// Applies the discrete events of a contact at an accepted step
    /// boundary: lift-off clears the bristle, exceeding the static bound
    /// releases the latch, and falling back under the kinetic bound sets it.
    /// Returns whether anything changed.
    pub fn transition(
        &self,
        state: &mut ContactPointState,
        point: &PointKinematics,
        surface: &SurfaceSample,
    ) -> bool {
        let before = *state;
        match self.engage(state, point, surface) {
            None => *state = ContactPointState::default(),
            Some(e) => {
                let trial = e.f_trial.length();
                state.tangent_deflection_world_m = e.z_tan;
                if state.sticking {
                    state.sticking = trial <= e.mu_static * e.normal_force;
                } else {
                    let bound = e.mu_dynamic * e.normal_force;
                    state.sticking = trial <= bound;
                    let k = self.bristle_stiffness_n_m;
                    if !state.sticking && self.bristle_damping_n_s_m <= 0.0 && k > 0.0 {
                        state.tangent_deflection_world_m = e.z_tan.clamp_length_max(bound / k);
                    }
                }
            }
        }
        *state != before
    }
}
