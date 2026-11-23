//! Rest latch of a landed vehicle: a discrete state stepped by the host at
//! accepted step boundaries.

use glam::{DQuat, DVec3};

/// Thresholds of a [`PinnedSurfaceRest`] latch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PinnedRestSpec {
    /// Speed below which a vehicle in contact starts settling, m/s.
    pub settle_speed_m_s: f64,
    /// Angular rate below which it starts settling, rad/s.
    pub settle_rate_rad_s: f64,
    /// Speed above which settling is abandoned, m/s; above
    /// `settle_speed_m_s` so that sensor-level jitter does not restart the
    /// dwell.
    pub release_speed_m_s: f64,
    /// Net non-gravitational upward force that frees the vehicle, N.
    pub breakout_threshold_n: f64,
    /// Continuous settled time required before pinning, s.
    pub dwell_required_s: f64,
}

impl PinnedRestSpec {
    /// Settle speed of a metre-scale lander, m/s.
    pub const DEFAULT_SETTLE_SPEED_M_S: f64 = 0.03;
    /// Settle angular rate of a metre-scale lander, rad/s.
    pub const DEFAULT_SETTLE_RATE_RAD_S: f64 = 0.02;
    /// Release speed of a metre-scale lander, m/s.
    pub const DEFAULT_RELEASE_SPEED_M_S: f64 = 0.08;

    /// Metre-scale lander thresholds with the given breakout force and dwell.
    pub const fn lander(breakout_threshold_n: f64, dwell_required_s: f64) -> Self {
        Self {
            settle_speed_m_s: Self::DEFAULT_SETTLE_SPEED_M_S,
            settle_rate_rad_s: Self::DEFAULT_SETTLE_RATE_RAD_S,
            release_speed_m_s: Self::DEFAULT_RELEASE_SPEED_M_S,
            breakout_threshold_n,
            dwell_required_s,
        }
    }

    /// Thresholds derived from the body: it counts as settled when it moves
    /// less than `fraction` of its size, or turns less than `fraction` of a
    /// radian, per natural period of its contact, and is released at
    /// `8/3` of the settle speed (the ratio of the lander defaults).
    pub fn for_body(
        size_m: f64,
        contact_period_s: f64,
        fraction: f64,
        breakout_threshold_n: f64,
        dwell_required_s: f64,
    ) -> Self {
        let settle_speed_m_s = fraction * size_m / contact_period_s;
        Self {
            settle_speed_m_s,
            settle_rate_rad_s: fraction / contact_period_s,
            release_speed_m_s: settle_speed_m_s * (8.0 / 3.0),
            breakout_threshold_n,
            dwell_required_s,
        }
    }
}

/// What the latch observes at a step boundary.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PinnedRestInputs {
    /// Whether any contact point carries load.
    pub any_contact: bool,
    /// Speed of the vehicle relative to the surface, m/s.
    pub speed_m_s: f64,
    /// Angular rate relative to the surface, rad/s.
    pub ang_rate_rad_s: f64,
    /// Root position in world axes, m.
    pub world_pos_m: DVec3,
    /// Root orientation in world axes.
    pub world_rot: DQuat,
    /// Net non-gravitational upward force, N.
    pub net_non_grav_upward_n: f64,
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

/// Advances the [`PinnedSurfaceRest`] latch over an accepted step of `dt_s`.
pub fn step_pinned_surface_rest(
    rest: &mut PinnedSurfaceRest,
    spec: &PinnedRestSpec,
    inputs: &PinnedRestInputs,
    dt_s: f64,
) {
    let breakout = inputs.net_non_grav_upward_n >= spec.breakout_threshold_n;
    match *rest {
        PinnedSurfaceRest::Free => {
            if inputs.any_contact
                && inputs.speed_m_s < spec.settle_speed_m_s
                && inputs.ang_rate_rad_s < spec.settle_rate_rad_s
                && !breakout
            {
                *rest = PinnedSurfaceRest::Settling { elapsed_s: 0.0 };
            }
        }
        PinnedSurfaceRest::Settling { elapsed_s } => {
            if !inputs.any_contact || inputs.speed_m_s > spec.release_speed_m_s || breakout {
                *rest = PinnedSurfaceRest::Free;
            } else if elapsed_s + dt_s >= spec.dwell_required_s {
                *rest = PinnedSurfaceRest::Pinned {
                    world_pos_m: inputs.world_pos_m,
                    world_rot: inputs.world_rot,
                };
            } else {
                *rest = PinnedSurfaceRest::Settling {
                    elapsed_s: elapsed_s + dt_s,
                };
            }
        }
        PinnedSurfaceRest::Pinned { .. } => {
            if breakout {
                *rest = PinnedSurfaceRest::Free;
            }
        }
    }
}
