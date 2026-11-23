use glam::DVec3;

use crate::MultibodyError;

/// Cause of tether / umbilical severing reported by [`WinchTetherState`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TetherCutReason {
    /// Ground/flight computer commanded pyrotechnic cutter activation.
    Commanded,
    /// Axial tension exceeded `breaking_tension_n`.
    OverloadSnap,
    /// Aerothermal heating integrated above `burn_through_energy_j` (e.g., Vostok-1 cable).
    ThermalBurnThrough,
    /// Load tension dropped below `auto_cut_low_tension_n` for `auto_cut_dwell_s`
    /// (e.g., Perseverance Sky Crane rover touchdown detection).
    TouchdownAutoCut,
}

/// Configuration for a variable-length winch, Sky Crane bridle, or umbilical
/// tether (`WinchTether`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WinchTetherSpec {
    /// Axial tensile stiffness when taut, N/m.
    pub stiffness_n_m: f64,
    /// Axial viscous damping when taut, N·s/m.
    pub damping_n_s_m: f64,
    /// Hard upper limit of spooled length, m.
    pub max_length_m: f64,
    /// Ultimate tensile failure limit, N.
    pub breaking_tension_n: f64,
    /// Optional cumulative absorbed thermal energy to burn through umbilical, J.
    pub burn_through_energy_j: Option<f64>,
    /// Optional low-tension threshold detecting touchdown weight offloading, N.
    pub auto_cut_low_tension_n: Option<f64>,
    /// Required low-tension dwell time before auto-cut fires, s.
    pub auto_cut_dwell_s: f64,
}

/// Mutable state of a [`WinchTetherSpec`] tether or winch line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WinchTetherState {
    /// Whether the physical line is still intact and capable of carrying tension.
    pub intact: bool,
    /// Current paid-out tether length, m.
    pub deployed_length_m: f64,
    /// Active winch spool rate (`> 0` pays out, `< 0` reels in), m/s.
    pub payout_rate_m_s: f64,
    /// Cumulative thermal energy absorbed by umbilical jacket, J.
    pub absorbed_thermal_j: f64,
    /// Elapsed dwell below `auto_cut_low_tension_n`, s.
    pub low_tension_timer_s: f64,
    /// Last resolved tensile magnitude, N.
    pub last_tension_n: f64,
    /// Cause of severing once `intact == false`.
    pub cut_reason: Option<TetherCutReason>,
}

impl WinchTetherState {
    /// Creates an intact tether initialized at `deployed_length_m`.
    pub const fn intact_at(deployed_length_m: f64) -> Self {
        Self {
            intact: true,
            deployed_length_m,
            payout_rate_m_s: 0.0,
            absorbed_thermal_j: 0.0,
            low_tension_timer_s: 0.0,
            last_tension_n: 0.0,
            cut_reason: None,
        }
    }
}

/// Equal-and-opposite world forces and tether telemetry from [`step_tether`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TetherStepOutput {
    /// Tension force acting ON anchor A toward anchor B in world coordinates, N.
    pub force_on_a_world_n: DVec3,
    /// Tension force acting ON anchor B toward anchor A in world coordinates, N.
    pub force_on_b_world_n: DVec3,
    /// Scalar tether tension, N (`0.0` when slack or severed).
    pub tension_n: f64,
    /// Current separation distance between world anchors, m.
    pub separation_m: f64,
    /// True only on the step where the tether transitioned from intact to severed.
    pub severed_this_step: bool,
}

/// Advances a winch/tether/umbilical over `dt_s`, paying out spool length,
/// computing unilateral tensile forces, and checking thermal burn-through,
/// overload snap, and low-tension touchdown auto-cut.
#[allow(clippy::too_many_arguments)]
pub fn step_tether(
    spec: &WinchTetherSpec,
    state: &mut WinchTetherState,
    anchor_a_world_m: DVec3,
    vel_a_world_m_s: DVec3,
    anchor_b_world_m: DVec3,
    vel_b_world_m_s: DVec3,
    heat_rate_w: f64,
    dt_s: f64,
) -> Result<TetherStepOutput, MultibodyError> {
    if !dt_s.is_finite() || dt_s < 0.0 {
        return Err(MultibodyError::InvalidParameter {
            field: "dt_s",
            value: dt_s,
        });
    }
    let delta = anchor_b_world_m - anchor_a_world_m;
    let sep = delta.length();
    if !state.intact {
        state.last_tension_n = 0.0;
        return Ok(slack_or_severed(sep, false));
    }

    state.deployed_length_m =
        (state.deployed_length_m + state.payout_rate_m_s * dt_s).clamp(0.0, spec.max_length_m);

    if let Some(burn_j) = spec.burn_through_energy_j {
        state.absorbed_thermal_j += heat_rate_w.max(0.0) * dt_s;
        if state.absorbed_thermal_j >= burn_j {
            return Ok(sever(state, TetherCutReason::ThermalBurnThrough, sep));
        }
    }

    let tension = compute_tether_tension(spec, state, delta, sep, vel_a_world_m_s, vel_b_world_m_s);
    if tension >= spec.breaking_tension_n {
        return Ok(sever(state, TetherCutReason::OverloadSnap, sep));
    }

    if let Some(low_thresh) = spec.auto_cut_low_tension_n {
        if tension <= low_thresh {
            state.low_tension_timer_s += dt_s;
            if state.low_tension_timer_s >= spec.auto_cut_dwell_s {
                return Ok(sever(state, TetherCutReason::TouchdownAutoCut, sep));
            }
        } else {
            state.low_tension_timer_s = 0.0;
        }
    }

    state.last_tension_n = tension;
    let f_a = if sep > 0.0 && tension > 0.0 {
        (delta / sep) * tension
    } else {
        DVec3::ZERO
    };
    Ok(TetherStepOutput {
        force_on_a_world_n: f_a,
        force_on_b_world_n: -f_a,
        tension_n: tension,
        separation_m: sep,
        severed_this_step: false,
    })
}

fn compute_tether_tension(
    spec: &WinchTetherSpec,
    state: &WinchTetherState,
    delta: DVec3,
    sep: f64,
    vel_a: DVec3,
    vel_b: DVec3,
) -> f64 {
    if sep <= state.deployed_length_m || sep == 0.0 {
        return 0.0;
    }
    let dir = delta / sep;
    let rel_speed = (vel_b - vel_a).dot(dir) - state.payout_rate_m_s;
    ((sep - state.deployed_length_m) * spec.stiffness_n_m + rel_speed * spec.damping_n_s_m).max(0.0)
}

fn sever(state: &mut WinchTetherState, reason: TetherCutReason, sep: f64) -> TetherStepOutput {
    state.intact = false;
    state.cut_reason = Some(reason);
    state.last_tension_n = 0.0;
    slack_or_severed(sep, true)
}

fn slack_or_severed(separation_m: f64, severed_this_step: bool) -> TetherStepOutput {
    TetherStepOutput {
        force_on_a_world_n: DVec3::ZERO,
        force_on_b_world_n: DVec3::ZERO,
        tension_n: 0.0,
        separation_m,
        severed_this_step,
    }
}
