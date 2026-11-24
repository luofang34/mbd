use glam::{DMat3, DVec3};
use mbd_core::{AmbientGas, ComponentId, DynamicMassContribution, FaultState, WrenchAtSite};

use crate::error::{
    SystemsError, check_non_negative, check_positive, check_vec_finite, normalize_axis,
};

const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;

/// Solid rocket motor specification (e.g. Vostok seat ejection / retro, SRBs, ullage motors).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolidMotorSpec {
    /// Host component carrying grain mass and receiving thrust wrench.
    pub site: ComponentId,
    /// Nozzle throat position in `site`'s local axes, m.
    pub mount_offset_local_m: DVec3,
    /// Grain center of mass in `site`'s local axes, m.
    pub grain_cg_local_m: DVec3,
    /// Unit thrust direction acting on the vehicle in `site`'s local axes.
    pub thrust_axis_local: DVec3,
    /// Initial loaded propellant grain mass, kg.
    pub propellant_mass_kg: f64,
    /// Vacuum specific impulse, s.
    pub isp_vac_s: f64,
    /// Nozzle exit area for ambient pressure thrust correction $F = F_{\text{vac}} - p_a A_e$, m².
    pub nozzle_exit_area_m2: f64,
    /// Total active burn duration from ignition to burnout, s.
    pub burn_time_s: f64,
    /// Linear tail-off fraction at end of burn (`0.0..=0.5`).
    pub tail_off_fraction: f64,
    /// Cylindrical grain radius for grain inertia reduction as propellant burns, m.
    pub grain_radius_m: f64,
    /// Cylindrical grain length, m.
    pub grain_length_m: f64,
}

impl SolidMotorSpec {
    /// Validates solid motor geometry and propellant properties.
    pub fn validate(&self) -> Result<(), SystemsError> {
        check_vec_finite(
            "solid_motor.mount_offset_local_m",
            self.mount_offset_local_m,
        )?;
        check_vec_finite("solid_motor.grain_cg_local_m", self.grain_cg_local_m)?;
        normalize_axis("solid_motor.thrust_axis_local", self.thrust_axis_local)?;
        check_positive("solid_motor.propellant_mass_kg", self.propellant_mass_kg)?;
        check_positive("solid_motor.isp_vac_s", self.isp_vac_s)?;
        check_non_negative("solid_motor.nozzle_exit_area_m2", self.nozzle_exit_area_m2)?;
        check_positive("solid_motor.burn_time_s", self.burn_time_s)?;
        if !self.tail_off_fraction.is_finite() || !(0.0..=0.5).contains(&self.tail_off_fraction) {
            return Err(SystemsError::InvalidParameter {
                field: "solid_motor.tail_off_fraction",
                value: self.tail_off_fraction,
            });
        }
        check_positive("solid_motor.grain_radius_m", self.grain_radius_m)?;
        check_positive("solid_motor.grain_length_m", self.grain_length_m)?;
        Ok(())
    }
}

/// Runtime state of a one-shot [`SolidMotorSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolidMotorState {
    /// Whether the igniter has fired.
    pub ignited: bool,
    /// Elapsed time since ignition, s.
    pub burn_elapsed_s: f64,
    /// Remaining solid propellant grain mass, kg.
    pub remaining_propellant_kg: f64,
    /// Whether burnout is complete.
    pub burned_out: bool,
    /// Optional fault injection (e.g. `StuckOff` prevents ignition).
    pub fault: FaultState,
}

impl SolidMotorState {
    /// Initializes an unfired solid rocket motor loaded with `spec.propellant_mass_kg`.
    pub const fn loaded(spec: &SolidMotorSpec) -> Self {
        Self {
            ignited: false,
            burn_elapsed_s: 0.0,
            remaining_propellant_kg: spec.propellant_mass_kg,
            burned_out: false,
            fault: FaultState::Nominal,
        }
    }
}

/// Output emitted by stepping a [`SolidMotorSpec`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SolidMotorStepOutput {
    /// Thrust wrench applied to `spec.site` in its local frame.
    pub wrench_local: WrenchAtSite,
    /// Dynamic mass/CG/inertia contribution of the unburned propellant grain.
    pub grain_mass_contribution: DynamicMassContribution,
    /// Mass consumed during this step, kg.
    pub burned_mass_kg: f64,
    /// Realized thrust magnitude, N.
    pub thrust_n: f64,
}

/// Advances a solid rocket motor over `dt_s`, returning its thrust wrench and updated grain inertia.
pub fn step_solid_motor(
    spec: &SolidMotorSpec,
    state: &mut SolidMotorState,
    dt_s: f64,
    ambient: &AmbientGas,
    ignite_cmd: bool,
) -> Result<SolidMotorStepOutput, SystemsError> {
    spec.validate()?;
    state.fault.validate()?;
    ambient.validate()?;
    check_non_negative("solid_motor.dt_s", dt_s)?;

    if ignite_cmd
        && !state.burned_out
        && !matches!(state.fault, FaultState::StuckOff | FaultState::Jammed)
    {
        state.ignited = true;
    }

    let mut burned_mass_kg = 0.0;
    let mut thrust_n = 0.0;

    if state.ignited && !state.burned_out && dt_s > 0.0 && state.remaining_propellant_kg > 0.0 {
        let shape_integral = 1.0 - 0.5 * spec.tail_off_fraction;
        let peak_mdot = spec.propellant_mass_kg / (spec.burn_time_s * shape_integral);
        let t_mid = (state.burn_elapsed_s + 0.5 * dt_s).min(spec.burn_time_s);
        let u = t_mid / spec.burn_time_s;
        let shape = if u <= 1.0 - spec.tail_off_fraction {
            1.0
        } else if spec.tail_off_fraction > 0.0 {
            ((1.0 - u) / spec.tail_off_fraction).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let eff = match state.fault {
            FaultState::Degraded { efficiency } => efficiency,
            FaultState::StuckOff | FaultState::Jammed => 0.0,
            _ => 1.0,
        };
        burned_mass_kg = (peak_mdot * shape * eff * dt_s).min(state.remaining_propellant_kg);
        state.remaining_propellant_kg -= burned_mass_kg;
        state.burn_elapsed_s += dt_s;
        if state.burn_elapsed_s >= spec.burn_time_s || state.remaining_propellant_kg <= 1.0e-9 {
            state.burned_out = true;
        }
        let realized_mdot = burned_mass_kg / dt_s;
        let f_vac = realized_mdot * spec.isp_vac_s * STANDARD_GRAVITY_M_S2;
        thrust_n = (f_vac - ambient.pressure_pa * spec.nozzle_exit_area_m2).max(0.0);
    }

    let grain_mass_contribution = solid_grain_contribution(spec, state.remaining_propellant_kg);
    grain_mass_contribution.validate()?;

    let dir = spec.thrust_axis_local.normalize_or(DVec3::X);
    Ok(SolidMotorStepOutput {
        wrench_local: WrenchAtSite {
            site: spec.site,
            point_local_m: spec.mount_offset_local_m,
            force_local_n: dir * thrust_n,
            torque_local_nm: DVec3::ZERO,
        },
        grain_mass_contribution,
        burned_mass_kg,
        thrust_n,
    })
}

fn solid_grain_contribution(spec: &SolidMotorSpec, mass_kg: f64) -> DynamicMassContribution {
    if mass_kg <= 0.0 {
        return DynamicMassContribution {
            component: spec.site,
            mass_kg: 0.0,
            cg_local_m: spec.grain_cg_local_m,
            inertia_cg_local: DMat3::ZERO,
        };
    }
    let eta = (mass_kg / spec.propellant_mass_kg).clamp(0.0, 1.0);
    let r2 = spec.grain_radius_m * spec.grain_radius_m;
    let r_inner2 = r2 * (1.0 - eta);
    let i_ax = 0.5 * mass_kg * (r2 + r_inner2);
    let l2 = spec.grain_length_m * spec.grain_length_m;
    let i_tr = mass_kg * (3.0 * (r2 + r_inner2) + l2) / 12.0;
    let e_ax = spec.thrust_axis_local.normalize_or(DVec3::X);
    let ref_up = if e_ax.z.abs() < 0.9 {
        DVec3::Z
    } else {
        DVec3::Y
    };
    let e_u = e_ax.cross(ref_up).normalize();
    let e_v = e_ax.cross(e_u);
    let rot = DMat3::from_cols(e_ax, e_u, e_v);
    let inertia_cg_local =
        rot * DMat3::from_diagonal(DVec3::new(i_ax, i_tr, i_tr)) * rot.transpose();

    DynamicMassContribution {
        component: spec.site,
        mass_kg,
        cg_local_m: spec.grain_cg_local_m,
        inertia_cg_local,
    }
}
