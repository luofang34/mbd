//! What an aerodynamic element sees and what it returns: the flow at its own
//! link, the air the host samples for it, and the wrench it applies.

use glam::DVec3;
use mbd_core::{AmbientGas, MbdError};

/// Undisturbed air at one point, as the host samples it from its atmosphere
/// and wind models.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirSample {
    /// Multi-species ambient gas.
    pub ambient: AmbientGas,
    /// Velocity of the air in world axes, m/s.
    pub wind_world_m_s: DVec3,
}

/// Host input to aerodynamic elements. Each element samples the air at its
/// own reference point, so a host with a spatially varying atmosphere or
/// wind field answers per point; one with a single sample answers the same
/// everywhere.
pub trait AeroInputs {
    /// Air at `position_world_m`.
    fn air_at(&self, position_world_m: DVec3) -> AirSample;

    /// Control channels in the order elements name them: surface
    /// deflections, rad; rotor speeds, rad/s; parachute commands. A channel
    /// beyond the slice reads as zero.
    fn controls(&self) -> &[f64];
}

/// One air sample everywhere, with one set of control channels: the common
/// host input.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroEnvironment<'a> {
    /// Air at every point.
    pub air: AirSample,
    /// Control channels.
    pub controls: &'a [f64],
}

impl AeroInputs for AeroEnvironment<'_> {
    fn air_at(&self, _position_world_m: DVec3) -> AirSample {
        self.air
    }

    fn controls(&self) -> &[f64] {
        self.controls
    }
}

/// The flow at an element's link: the motion of the link origin relative to
/// the local air, in link axes. Joint motion and placement are part of it,
/// so an element on a flapping, swinging or spinning link sees its own
/// flow.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LinkFlow<'a> {
    /// Velocity of the link origin relative to the air, link axes, m/s.
    pub velocity_air_link_m_s: DVec3,
    /// Angular rate of the link relative to the air, link axes, rad/s.
    pub angular_rate_link_rad_s: DVec3,
    /// Undisturbed ambient gas at the element.
    pub ambient: AmbientGas,
    /// Control channels; elements define which slot is which.
    pub controls: &'a [f64],
}

impl LinkFlow<'_> {
    /// Rejects a flow a model cannot evaluate: non-finite velocities,
    /// non-finite controls, or invalid ambient gas properties.
    pub fn validate(&self) -> Result<(), MbdError> {
        let invalid = |field, value| Err(MbdError::InvalidFlow { field, value });
        if !self.velocity_air_link_m_s.is_finite() {
            return invalid("velocity_air_link_m_s", self.velocity_air_link_m_s.length());
        }
        if !self.angular_rate_link_rad_s.is_finite() {
            return invalid(
                "angular_rate_link_rad_s",
                self.angular_rate_link_rad_s.length(),
            );
        }
        self.ambient.validate()?;
        for &value in self.controls {
            if !value.is_finite() {
                return invalid("controls", value);
            }
        }
        Ok(())
    }

    /// Value of control `channel`; zero for no channel or one beyond the
    /// slice.
    pub fn control(&self, channel: Option<usize>) -> f64 {
        channel
            .and_then(|idx| self.controls.get(idx).copied())
            .unwrap_or(0.0)
    }
}

/// Aerodynamic load an element applies to its link, at a reference station
/// in link axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroWrench {
    /// Reference station, link axes, m.
    pub reference_point_link_m: DVec3,
    /// Force, link axes, N.
    pub force_link_n: DVec3,
    /// Moment about the reference station, link axes, N·m.
    pub moment_ref_link_nm: DVec3,
    /// Stagnation-point convective heat flux, W/m².
    pub stagnation_heat_flux_w_m2: f64,
}

impl AeroWrench {
    /// No load at all, referenced to the link origin.
    pub const ZERO: Self = Self {
        reference_point_link_m: DVec3::ZERO,
        force_link_n: DVec3::ZERO,
        moment_ref_link_nm: DVec3::ZERO,
        stagnation_heat_flux_w_m2: 0.0,
    };

    /// No load, referenced to `reference_point_link_m`.
    pub const fn zero_at(reference_point_link_m: DVec3) -> Self {
        Self {
            reference_point_link_m,
            ..Self::ZERO
        }
    }

    /// The moment about another point of the link, N·m.
    pub fn moment_about(&self, point_link_m: DVec3) -> DVec3 {
        self.moment_ref_link_nm
            + (self.reference_point_link_m - point_link_m).cross(self.force_link_n)
    }
}
