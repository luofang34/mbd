use glam::{DQuat, DVec3};

#[cfg(test)]
mod tests;

/// Frame that a [`LocalSiteKinematics`] position, velocity and attitude are
/// expressed in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KinematicsFrame {
    /// Relative to the vehicle centre of gravity, in body axes.
    Body,
    /// Absolute, in ECEF axes.
    Ecef,
}

/// Motion of one mounting site on the vehicle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalSiteKinematics {
    /// Frame of `position_m`, `velocity_m_s` and `attitude`.
    pub frame: KinematicsFrame,
    /// Site position, metres.
    pub position_m: DVec3,
    /// Site velocity, m/s.
    pub velocity_m_s: DVec3,
    /// Site-to-frame rotation.
    pub attitude: DQuat,
    /// Specific force an accelerometer at the site senses, in site axes, m/s².
    pub specific_force_site_m_s2: DVec3,
    /// Angular rate a gyroscope at the site senses, in site axes, rad/s.
    pub angular_rate_site_rad_s: DVec3,
}

impl LocalSiteKinematics {
    /// The same site in ECEF, given the vehicle's centre-of-gravity position
    /// and velocity and its body-to-ECEF rotation. Sensed quantities are in
    /// site axes and do not change. Applied to a site already in ECEF, it
    /// returns the site unchanged.
    pub fn in_ecef(
        self,
        cg_position_ecef_m: DVec3,
        cg_velocity_ecef_m_s: DVec3,
        ecef_from_body: DQuat,
    ) -> Self {
        if self.frame == KinematicsFrame::Ecef {
            return self;
        }
        Self {
            frame: KinematicsFrame::Ecef,
            position_m: cg_position_ecef_m + ecef_from_body * self.position_m,
            velocity_m_s: cg_velocity_ecef_m_s + ecef_from_body * self.velocity_m_s,
            attitude: ecef_from_body * self.attitude,
            ..self
        }
    }
}

/// What an ideal inertial sensor at a rigid mounting site reads, given the
/// vehicle's motion at its centre of gravity.
///
/// `a_spec_cg_body` is specific force at the centre of gravity, and
/// `omega_body` and `alpha_body` are inertial angular rate and acceleration,
/// all in body axes. `site` is the site's `(position, site-to-body rotation)`,
/// as [`crate::ComponentTree::world_transform_in_root`] returns it, and
/// `cg_body` the instantaneous centre of gravity in the same frame. The result
/// is in [`KinematicsFrame::Body`].
pub fn compute_site_kinematics(
    a_spec_cg_body: DVec3,
    omega_body: DVec3,
    alpha_body: DVec3,
    site: (DVec3, DQuat),
    cg_body: DVec3,
) -> LocalSiteKinematics {
    let (r_site_body, q_body_from_site) = site;
    let dr = r_site_body - cg_body;
    let site_from_body = q_body_from_site.conjugate();
    let tangential = alpha_body.cross(dr);
    let centripetal = omega_body.cross(omega_body.cross(dr));
    LocalSiteKinematics {
        frame: KinematicsFrame::Body,
        position_m: dr,
        velocity_m_s: omega_body.cross(dr),
        attitude: q_body_from_site,
        specific_force_site_m_s2: site_from_body * (a_spec_cg_body + tangential + centripetal),
        angular_rate_site_rad_s: site_from_body * omega_body,
    }
}

/// Electrical supply at a site.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowerRailState {
    /// Mean rail voltage, V.
    pub voltage_v: f64,
    /// Ripple on the rail, V RMS.
    pub ripple_v_rms: f64,
    /// Whether the rail is switched on.
    pub enabled: bool,
}

impl PowerRailState {
    /// Clean, enabled rail at `voltage_v`.
    pub const fn nominal(voltage_v: f64) -> Self {
        Self {
            voltage_v,
            ripple_v_rms: 0.0,
            enabled: true,
        }
    }

    /// Nominal 28 V DC aircraft and spacecraft bus.
    pub const fn nominal_28v() -> Self {
        Self::nominal(28.0)
    }

    /// Nominal 5 V logic rail.
    pub const fn nominal_5v() -> Self {
        Self::nominal(5.0)
    }

    /// Nominal 3.3 V logic rail.
    pub const fn nominal_3v3() -> Self {
        Self::nominal(3.3)
    }
}

/// Everything a device feels at its mounting site.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SiteEnvironment {
    /// Site motion.
    pub kinematics: LocalSiteKinematics,
    /// Broadband vibration per site axis, m/s² RMS.
    pub vibration_rms_m_s2: DVec3,
    /// Local temperature, K.
    pub temperature_k: f64,
    /// Supply rail feeding the site.
    pub power_rail: PowerRailState,
    /// Stray magnetic field in site axes, tesla.
    pub emi_field_t: DVec3,
}

impl SiteEnvironment {
    /// Benign bench conditions at `kinematics`: 293.15 K on a clean 28 V rail,
    /// with no vibration or stray field.
    pub const fn ideal(kinematics: LocalSiteKinematics) -> Self {
        Self {
            kinematics,
            vibration_rms_m_s2: DVec3::ZERO,
            temperature_k: 293.15,
            power_rail: PowerRailState::nominal_28v(),
            emi_field_t: DVec3::ZERO,
        }
    }
}
