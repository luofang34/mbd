use core::f64::consts::PI;

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, DynamicMassContribution, FaultState, WrenchAtSite};

use super::FluidSpecies;
use super::geometry::{TankGeometry, fluid_inertia_tensor};
use crate::error::{
    SystemsError, check_finite, check_non_negative, check_positive, check_vec_finite,
    normalize_axis,
};

const MIN_G_ACCEL: f64 = 1.0e-5;

/// Unique identifier for a fluid tank in a [`super::FluidNetwork`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TankId(pub u16);

/// Unified propellant/fuel feed mechanism shared across rockets, spacecraft, and aircraft.
///
/// Physical settling (`UllageSettled`, `GravitySettled`, and collector `HeaderTank`)
/// shares the single accelerometer specific-force law `settled_accel = (-specific_force) · pickup_axis_local`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FeedRegime {
    /// Positive expulsion via bladder, diaphragm, or piston regardless of acceleration sign.
    BladderOrDiaphragm,
    /// Capillary surface-tension PMD that holds fluid down to `min_settling_accel_m_s2`.
    CapillaryPmd {
        /// Direction toward the tank outlet sump in host component local frame.
        pickup_axis_local: DVec3,
        /// Minimum residual acceleration along pickup (negative values denote adverse tolerance), m/s².
        min_settling_accel_m_s2: f64,
    },
    /// Free-surface settled tank (both aircraft wing/fuselage tanks under positive/negative G
    /// and cryogenic rocket stages requiring ullage motors to settle before restart).
    Settled {
        /// Unit vector pointing from tank centre toward outlet pickup sump in local axes.
        pickup_axis_local: DVec3,
        /// Minimum effective acceleration along `pickup_axis_local` required to keep pickup covered, m/s².
        min_settling_accel_m_s2: f64,
        /// Continuous dwell duration above threshold before settled feed opens after unporting, s.
        settle_time_s: f64,
    },
    /// Small baffled collector / negative-G header tank whose internal trap feeds engines
    /// reliably until completely depleted, even while main wing tanks unport.
    HeaderTank,
}

impl FeedRegime {
    /// Validates direction vectors and non-negative timers.
    pub fn validate(&self) -> Result<(), SystemsError> {
        match *self {
            Self::BladderOrDiaphragm | Self::HeaderTank => Ok(()),
            Self::CapillaryPmd {
                pickup_axis_local,
                min_settling_accel_m_s2,
            } => {
                normalize_axis("feed.pickup_axis_local", pickup_axis_local)?;
                check_finite("feed.min_settling_accel_m_s2", min_settling_accel_m_s2)?;
                Ok(())
            }
            Self::Settled {
                pickup_axis_local,
                min_settling_accel_m_s2,
                settle_time_s,
            } => {
                normalize_axis("feed.pickup_axis_local", pickup_axis_local)?;
                check_finite("feed.min_settling_accel_m_s2", min_settling_accel_m_s2)?;
                check_non_negative("feed.settle_time_s", settle_time_s)?;
                Ok(())
            }
        }
    }
}

/// Optional equivalent mechanical pendulum mode for liquid slosh dynamics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SloshModeSpec {
    /// Fraction of resident liquid mass participating in lateral slosh (`0.0..=0.8`).
    pub mass_fraction: f64,
    /// Natural frequency scale at 1-g, Hz (`omega = 2*pi*f * sqrt(g_eff / 9.80665)`).
    pub frequency_1g_hz: f64,
    /// Viscous damping ratio due to baffles (`0.0..=1.0`).
    pub damping_ratio: f64,
    /// Maximum lateral displacement amplitude limited by tank walls, m.
    pub max_displacement_m: f64,
}

impl SloshModeSpec {
    /// Validates slosh parameters.
    pub fn validate(&self) -> Result<(), SystemsError> {
        if !self.mass_fraction.is_finite() || !(0.0..=0.8).contains(&self.mass_fraction) {
            return Err(SystemsError::InvalidParameter {
                field: "slosh.mass_fraction",
                value: self.mass_fraction,
            });
        }
        check_positive("slosh.frequency_1g_hz", self.frequency_1g_hz)?;
        if !self.damping_ratio.is_finite() || !(0.0..=1.0).contains(&self.damping_ratio) {
            return Err(SystemsError::InvalidParameter {
                field: "slosh.damping_ratio",
                value: self.damping_ratio,
            });
        }
        check_positive("slosh.max_displacement_m", self.max_displacement_m)?;
        Ok(())
    }
}

/// Static configuration of one fluid tank mounted on a `ComponentTree` component.
#[derive(Debug, Clone, PartialEq)]
pub struct TankSpec {
    /// Unique tank identifier.
    pub id: TankId,
    /// Host component carrying the fluid mass and receiving slosh wrenches.
    pub component: ComponentId,
    /// Tank centre position in `component`'s local frame, m.
    pub center_local_m: DVec3,
    /// Internal geometry in `component`'s local axes.
    pub geometry: TankGeometry,
    /// Chemical species stored in this tank.
    pub species: FluidSpecies,
    /// Bulk fluid density in kg/m³ (typically `species.nominal_density_kg_m3()`).
    pub fluid_density_kg_m3: f64,
    /// Maximum usable + trapped propellant mass capacity, kg.
    pub capacity_mass_kg: f64,
    /// Residual trapped/unusable mass that cannot be pumped out, kg.
    pub minimum_unusable_kg: f64,
    /// Expulsion / settling regime governing feed continuity under maneuvering or 0-g.
    pub feed_regime: FeedRegime,
    /// Optional lateral slosh mode.
    pub slosh: Option<SloshModeSpec>,
    /// Passive heat leak rate into tank when thermal network does not supply explicit flux, W.
    pub passive_heat_leak_w: f64,
}

impl TankSpec {
    /// Validates all fields of `TankSpec`.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_vec_finite("tank.center_local_m", self.center_local_m)?;
        self.geometry.validate()?;
        check_positive("tank.fluid_density_kg_m3", self.fluid_density_kg_m3)?;
        check_non_negative("tank.capacity_mass_kg", self.capacity_mass_kg)?;
        check_non_negative("tank.minimum_unusable_kg", self.minimum_unusable_kg)?;
        if self.minimum_unusable_kg > self.capacity_mass_kg {
            return Err(SystemsError::InvalidConfig {
                detail: alloc::format!(
                    "tank {} minimum_unusable_kg ({}) exceeds capacity_mass_kg ({})",
                    self.id.0,
                    self.minimum_unusable_kg,
                    self.capacity_mass_kg
                ),
            });
        }
        self.feed_regime.validate()?;
        if let Some(slosh) = self.slosh {
            slosh.validate()?;
        }
        check_finite("tank.passive_heat_leak_w", self.passive_heat_leak_w)?;
        Ok(())
    }

    /// Fill fraction in `0.0..=1.0`.
    pub fn fill_fraction(&self, mass_kg: f64) -> f64 {
        if self.capacity_mass_kg <= 0.0 {
            return 0.0;
        }
        (mass_kg / self.capacity_mass_kg).clamp(0.0, 1.0)
    }

    /// Computes live CG and inertia tensor contribution from resident liquid, accounting
    /// for free-surface settling along `last_settled_dir_local` and active slosh displacement.
    pub fn mass_contribution(
        &self,
        state: &TankState,
    ) -> Result<DynamicMassContribution, SystemsError> {
        let m = state.mass_kg.max(0.0);
        if m <= 0.0 {
            return Ok(DynamicMassContribution {
                component: self.component,
                mass_kg: 0.0,
                cg_local_m: self.center_local_m,
                inertia_cg_local: DMat3::ZERO,
            });
        }
        let eta = self.fill_fraction(m);
        let dir = state.last_settled_dir_local.normalize_or_zero();
        let shift_mag = if matches!(self.feed_regime, FeedRegime::BladderOrDiaphragm) {
            0.0
        } else {
            self.geometry.extent_along_unit_axis(dir) * 0.45 * (1.0 - eta)
        };
        let cg_local_m = self.center_local_m + dir * shift_mag + state.slosh_disp_local_m;
        let inertia_cg_local = fluid_inertia_tensor(self.geometry, m, eta);
        let contribution = DynamicMassContribution {
            component: self.component,
            mass_kg: m,
            cg_local_m,
            inertia_cg_local,
        };
        contribution.validate()?;
        Ok(contribution)
    }
}

/// Instantaneous feed availability at a tank outlet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeedAvailability {
    /// Positive feed available.
    Available,
    /// Feed interrupted because acceleration is adverse or below ullage threshold.
    UnportedOrUnsettled,
    /// Tank only contains trapped/unusable residual mass or is completely dry.
    Depleted,
    /// Blocked or inoperable due to an injected valve/sensor fault.
    Faulted,
}

/// Dynamic runtime state of a single fluid tank.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TankState {
    /// Current fluid mass inside tank, kg.
    pub mass_kg: f64,
    /// Internal tank pressure, Pa.
    pub pressure_pa: f64,
    /// Bulk fluid temperature, K.
    pub temperature_k: f64,
    /// Cumulative elapsed time spent continuously above settling threshold, s.
    pub settled_dwell_s: f64,
    /// Current latched feed readiness.
    pub availability: FeedAvailability,
    /// Last unit settling direction in host local frame (used for continuous CG tracking).
    pub last_settled_dir_local: DVec3,
    /// Slosh center-of-mass lateral offset in host local frame, m.
    pub slosh_disp_local_m: DVec3,
    /// Slosh velocity in host local frame, m/s.
    pub slosh_vel_local_m_s: DVec3,
    /// Active valve/tank fault mode.
    pub fault: FaultState,
}

impl TankState {
    /// Builds a nominal tank state at `mass_kg`, `pressure_pa`, and `temperature_k`.
    pub fn nominal(mass_kg: f64, pressure_pa: f64, temperature_k: f64) -> Self {
        Self {
            mass_kg,
            pressure_pa,
            temperature_k,
            settled_dwell_s: 0.0,
            availability: FeedAvailability::Available,
            last_settled_dir_local: DVec3::NEG_Z,
            slosh_disp_local_m: DVec3::ZERO,
            slosh_vel_local_m_s: DVec3::ZERO,
            fault: FaultState::Nominal,
        }
    }
}

/// Inputs to advance one tank by `dt_s`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TankStepInput {
    /// Time step duration, s.
    pub dt_s: f64,
    /// Accelerometer specific force sensed at the tank site in host local axes, m/s².
    ///
    /// Heavy fluid settles toward `-specific_force_site_m_s2`.
    pub specific_force_site_m_s2: DVec3,
    /// External thermal heat input rate into tank fluid, W (`None` uses `spec.passive_heat_leak_w`).
    pub heat_input_w: Option<f64>,
    /// Pressurant relief or ullage pressure setpoint, Pa (`None` keeps existing pressure).
    pub relief_pressure_pa: Option<f64>,
}

/// Mass accounting and structural wrench emitted by a single tank step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TankStepOutput {
    /// Current feed availability after settling, depletion, and fault checks.
    pub availability: FeedAvailability,
    /// Mass vented via cryogenic boil-off during this step, kg.
    pub boiloff_mass_kg: f64,
    /// Mass lost via active [`FaultState::Leak`] during this step, kg.
    pub leaked_mass_kg: f64,
    /// Slosh reaction wrench applied to the host component in local frame.
    pub slosh_wrench_local: WrenchAtSite,
}

/// Advances settling/unporting state, cryogenic boil-off, leak loss, and slosh dynamics
/// over `input.dt_s`, strictly conserving mass.
pub fn step_tank(
    spec: &TankSpec,
    state: &mut TankState,
    input: TankStepInput,
) -> Result<TankStepOutput, SystemsError> {
    spec.validate()?;
    state.fault.validate()?;
    check_non_negative("tank_step.dt_s", input.dt_s)?;
    check_vec_finite(
        "tank_step.specific_force_site_m_s2",
        input.specific_force_site_m_s2,
    )?;

    if let Some(p_relief) = input.relief_pressure_pa {
        check_non_negative("tank_step.relief_pressure_pa", p_relief)?;
        state.pressure_pa = state.pressure_pa.min(p_relief);
    }

    let settled_dir = if input.specific_force_site_m_s2.length_squared() > MIN_G_ACCEL * MIN_G_ACCEL
    {
        (-input.specific_force_site_m_s2).normalize()
    } else {
        state.last_settled_dir_local
    };
    state.last_settled_dir_local = settled_dir;

    let mut boiloff_mass_kg = 0.0;
    let heat_w = input.heat_input_w.unwrap_or(spec.passive_heat_leak_w);
    check_finite("tank_step.heat_input_w", heat_w)?;
    if spec.species.is_cryogenic() && heat_w > 0.0 && input.dt_s > 0.0 && state.mass_kg > 0.0 {
        let h_vap = spec.species.latent_heat_vaporization_j_kg();
        if h_vap > 0.0 {
            let max_boil = (heat_w * input.dt_s) / h_vap;
            boiloff_mass_kg = max_boil.min(state.mass_kg);
            state.mass_kg -= boiloff_mass_kg;
        }
    }

    let mut leaked_mass_kg = 0.0;
    if let FaultState::Leak { rate_kg_s } = state.fault
        && input.dt_s > 0.0
        && state.mass_kg > 0.0
    {
        leaked_mass_kg = (rate_kg_s * input.dt_s).min(state.mass_kg);
        state.mass_kg -= leaked_mass_kg;
    }

    let slosh_wrench_local =
        update_slosh_mode(spec, state, input.dt_s, input.specific_force_site_m_s2);
    state.availability =
        evaluate_availability(spec, state, input.dt_s, input.specific_force_site_m_s2);

    Ok(TankStepOutput {
        availability: state.availability,
        boiloff_mass_kg,
        leaked_mass_kg,
        slosh_wrench_local,
    })
}

fn evaluate_availability(
    spec: &TankSpec,
    state: &mut TankState,
    dt_s: f64,
    specific_force_site_m_s2: DVec3,
) -> FeedAvailability {
    if matches!(state.fault, FaultState::StuckOff | FaultState::Jammed) {
        return FeedAvailability::Faulted;
    }
    if state.mass_kg <= spec.minimum_unusable_kg {
        state.settled_dwell_s = 0.0;
        return FeedAvailability::Depleted;
    }
    match spec.feed_regime {
        FeedRegime::BladderOrDiaphragm | FeedRegime::HeaderTank => {
            state.settled_dwell_s += dt_s;
            FeedAvailability::Available
        }
        FeedRegime::CapillaryPmd {
            pickup_axis_local,
            min_settling_accel_m_s2,
        } => {
            let pickup = pickup_axis_local.normalize_or_zero();
            let settled_accel = (-specific_force_site_m_s2).dot(pickup);
            if settled_accel >= min_settling_accel_m_s2 {
                FeedAvailability::Available
            } else {
                FeedAvailability::UnportedOrUnsettled
            }
        }
        FeedRegime::Settled {
            pickup_axis_local,
            min_settling_accel_m_s2,
            settle_time_s,
        } => {
            let pickup = pickup_axis_local.normalize_or_zero();
            let settled_accel = (-specific_force_site_m_s2).dot(pickup);
            if settled_accel >= min_settling_accel_m_s2 {
                state.settled_dwell_s += dt_s;
                if state.settled_dwell_s >= settle_time_s {
                    FeedAvailability::Available
                } else {
                    FeedAvailability::UnportedOrUnsettled
                }
            } else {
                state.settled_dwell_s = 0.0;
                FeedAvailability::UnportedOrUnsettled
            }
        }
    }
}

fn update_slosh_mode(
    spec: &TankSpec,
    state: &mut TankState,
    dt_s: f64,
    specific_force_site_m_s2: DVec3,
) -> WrenchAtSite {
    let Some(slosh) = spec.slosh else {
        return WrenchAtSite::zero(spec.component);
    };
    if dt_s <= 0.0 || state.mass_kg <= 0.0 {
        return WrenchAtSite::zero(spec.component);
    }
    let m_slosh = state.mass_kg * slosh.mass_fraction;
    let f_mag = specific_force_site_m_s2.length();
    let omega_1g = 2.0 * PI * slosh.frequency_1g_hz;
    let g_scale = libm::sqrt((f_mag / 9.806_65).max(0.01));
    let omega = omega_1g * g_scale;
    let settle_dir = state.last_settled_dir_local.normalize_or_zero();
    let accel_drive = -specific_force_site_m_s2;
    let lat_drive = accel_drive - settle_dir * accel_drive.dot(settle_dir);
    let acc = lat_drive
        - (omega * omega) * state.slosh_disp_local_m
        - (2.0 * slosh.damping_ratio * omega) * state.slosh_vel_local_m_s;

    state.slosh_vel_local_m_s += acc * dt_s;
    state.slosh_disp_local_m += state.slosh_vel_local_m_s * dt_s;
    let disp_norm = state.slosh_disp_local_m.length();
    if disp_norm > slosh.max_displacement_m {
        let normal = state.slosh_disp_local_m / disp_norm;
        state.slosh_disp_local_m = normal * slosh.max_displacement_m;
        let radial_v = state.slosh_vel_local_m_s.dot(normal);
        if radial_v > 0.0 {
            state.slosh_vel_local_m_s -= normal * radial_v;
        }
    }
    let spring_force = m_slosh
        * ((omega * omega) * state.slosh_disp_local_m
            + (2.0 * slosh.damping_ratio * omega) * state.slosh_vel_local_m_s);
    WrenchAtSite {
        site: spec.component,
        point_local_m: spec.center_local_m,
        force_local_n: spring_force,
        torque_local_nm: DVec3::ZERO,
    }
}
