//! Characteristic length, mass and time of a model, and the magnitudes the
//! integrators and elements build from them.

use super::PreparedLink;
use crate::MultibodyError;

/// Characteristic length, mass and time of a model.
///
/// Every magnitude an integrator weighs error against is built from these
/// three, so the same [`crate::Tolerances`] mean the same thing for a 1 mg
/// insect and a 10 t aircraft:
///
/// - **Model states** ([`crate::MultibodyModel`]'s own block): translations
///   against `L`, translational rates against `L/T`, rotational rates
///   against `1/T`; attitude quaternions and hinge angles are already
///   dimensionless and weigh against 1. The absolute tolerance of a group is
///   its [`crate::Tolerance::abs`] times that magnitude.
/// - **Element states**: each [`crate::ForceElement::state_scale`] receives
///   this scale and writes one magnitude per state in the state's own unit.
///   An element takes it from its own parameters where they define one (a
///   contact's ramp depth, a tether's length) and otherwise from this scale:
///   [`ModelScale::length_m`], [`ModelScale::time_s`],
///   [`ModelScale::angular_rate_rad_s`] or [`ModelScale::energy_j`]
///   (`M L²/T²`). Dimensionless states weigh against 1.
///
/// The length and mass follow from the tree
/// ([`crate::MultibodyModel::with_derived_scale`]); the time does not,
/// because it belongs to the motion of interest (a wingbeat, a pendulum
/// swing, an orbit), which is why no default exists.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModelScale {
    length_m: f64,
    mass_kg: f64,
    time_s: f64,
}

/// How the characteristic time of a model is given.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TimeScale {
    /// The time itself, s: a period or time constant of the motion.
    Seconds(f64),
    /// A characteristic acceleration `a`, m/s², such as local gravity or the
    /// peak specific force; the time is `√(L/a)`, the time to cover the
    /// characteristic length from rest at about that acceleration.
    Acceleration(f64),
}

impl ModelScale {
    /// A scale from its three parts. Each must be finite and positive.
    pub fn new(length_m: f64, mass_kg: f64, time_s: f64) -> Result<Self, MultibodyError> {
        let scale = Self {
            length_m: positive("length_m", length_m)?,
            mass_kg: positive("mass_kg", mass_kg)?,
            time_s: positive("time_s", time_s)?,
        };
        Ok(scale)
    }

    /// A scale whose time follows from `length_m` and `time`.
    pub fn with_time(length_m: f64, mass_kg: f64, time: TimeScale) -> Result<Self, MultibodyError> {
        let length_m = positive("length_m", length_m)?;
        let time_s = match time {
            TimeScale::Seconds(t) => t,
            TimeScale::Acceleration(a) => libm::sqrt(length_m / positive("acceleration_m_s2", a)?),
        };
        Self::new(length_m, mass_kg, time_s)
    }

    /// Length and mass of the attached links: the largest link offset,
    /// centre-of-mass offset or radius of gyration, and the total mass.
    pub(crate) fn of_links(
        links: &[PreparedLink],
        time: TimeScale,
    ) -> Result<Self, MultibodyError> {
        let length_m = links.iter().fold(0.0_f64, |acc, l| {
            let gyration = if l.mass_kg > 0.0 {
                let i = l.inertia_cg_local;
                libm::sqrt((i.x_axis.x + i.y_axis.y + i.z_axis.z).max(0.0) / l.mass_kg)
            } else {
                0.0
            };
            acc.max(l.placement.translation_m.length())
                .max(l.cg_local_m.length())
                .max(gyration)
        });
        let mass_kg = links.iter().map(|l| l.mass_kg).sum();
        Self::with_time(length_m, mass_kg, time)
    }

    /// Scale of a model that is evaluated once and never integrated; no
    /// part of it is read.
    pub(crate) const UNREAD: Self = Self {
        length_m: 1.0,
        mass_kg: 1.0,
        time_s: 1.0,
    };

    /// Characteristic length `L`, m.
    pub const fn length_m(&self) -> f64 {
        self.length_m
    }

    /// Characteristic mass `M`, kg.
    pub const fn mass_kg(&self) -> f64 {
        self.mass_kg
    }

    /// Characteristic time `T` of the motion of interest, s.
    pub const fn time_s(&self) -> f64 {
        self.time_s
    }

    /// Characteristic speed `L/T`, m/s.
    pub fn speed_m_s(&self) -> f64 {
        self.length_m / self.time_s
    }

    /// Characteristic rotational rate `1/T`, rad/s.
    pub fn angular_rate_rad_s(&self) -> f64 {
        1.0 / self.time_s
    }

    /// Characteristic energy `M L²/T²`, J.
    pub fn energy_j(&self) -> f64 {
        let v = self.speed_m_s();
        self.mass_kg * v * v
    }
}

fn positive(field: &'static str, value: f64) -> Result<f64, MultibodyError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(MultibodyError::InvalidScale { field, value })
    }
}
