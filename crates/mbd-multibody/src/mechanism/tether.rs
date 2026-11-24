//! Variable-length winch line, bridle or umbilical: a unilateral axial
//! spring-damper whose paid-out length, absorbed heat and low-tension dwell
//! are continuous states, and whose severing is a discrete event.

use glam::DVec3;

mod element;

pub use element::{Tether, TetherAnchor};

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

/// What the host commands a tether to do over a step.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct TetherCommand {
    /// Winch spool rate (`> 0` pays out, `< 0` reels in), m/s.
    pub payout_rate_m_s: f64,
    /// Heating power absorbed by the line jacket, W.
    pub heat_rate_w: f64,
    /// Fires the cutter at the next step boundary.
    pub cut: bool,
}

/// State of a [`WinchTetherSpec`] tether or winch line.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WinchTetherState {
    /// Whether the physical line is still intact and capable of carrying tension.
    pub intact: bool,
    /// Current paid-out tether length, m.
    pub deployed_length_m: f64,
    /// Cumulative thermal energy absorbed by umbilical jacket, J.
    pub absorbed_thermal_j: f64,
    /// Elapsed dwell below `auto_cut_low_tension_n`, s.
    pub low_tension_timer_s: f64,
    /// Cause of severing once `intact == false`.
    pub cut_reason: Option<TetherCutReason>,
}

impl WinchTetherState {
    /// Creates an intact tether initialized at `deployed_length_m`.
    pub const fn intact_at(deployed_length_m: f64) -> Self {
        Self {
            intact: true,
            deployed_length_m,
            absorbed_thermal_j: 0.0,
            low_tension_timer_s: 0.0,
            cut_reason: None,
        }
    }
}

/// World position and velocity of the two ends of a tether.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TetherEnds {
    /// Anchor A in world axes, m.
    pub anchor_a_world_m: DVec3,
    /// Velocity of anchor A in world axes, m/s.
    pub vel_a_world_m_s: DVec3,
    /// Anchor B in world axes, m.
    pub anchor_b_world_m: DVec3,
    /// Velocity of anchor B in world axes, m/s.
    pub vel_b_world_m_s: DVec3,
}

/// Forces, telemetry and continuous-state rates of a tether at one state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TetherEvaluation {
    /// Tension force acting ON anchor A toward anchor B in world coordinates, N.
    pub force_on_a_world_n: DVec3,
    /// Tension force acting ON anchor B toward anchor A in world coordinates, N.
    pub force_on_b_world_n: DVec3,
    /// Scalar tether tension, N (`0.0` when slack or severed).
    pub tension_n: f64,
    /// Current separation distance between world anchors, m.
    pub separation_m: f64,
    /// Rate of the paid-out length, m/s.
    pub deployed_length_rate_m_s: f64,
    /// Rate of the absorbed thermal energy, W.
    pub absorbed_thermal_rate_w: f64,
    /// Rate of the low-tension dwell timer: one while tension is at or
    /// below the auto-cut threshold, zero otherwise.
    pub low_tension_timer_rate: f64,
}

impl WinchTetherSpec {
    fn tension(&self, state: &WinchTetherState, cmd: &TetherCommand, ends: &TetherEnds) -> f64 {
        let delta = ends.anchor_b_world_m - ends.anchor_a_world_m;
        let sep = delta.length();
        if !state.intact || sep <= state.deployed_length_m || sep == 0.0 {
            return 0.0;
        }
        let dir = delta / sep;
        let rel_speed =
            (ends.vel_b_world_m_s - ends.vel_a_world_m_s).dot(dir) - cmd.payout_rate_m_s;
        ((sep - state.deployed_length_m) * self.stiffness_n_m + rel_speed * self.damping_n_s_m)
            .max(0.0)
    }

    /// Tension forces and state rates at one state: a pure function of its
    /// arguments, valid at any integrator stage.
    pub fn evaluate(
        &self,
        state: &WinchTetherState,
        cmd: &TetherCommand,
        ends: &TetherEnds,
    ) -> TetherEvaluation {
        let delta = ends.anchor_b_world_m - ends.anchor_a_world_m;
        let sep = delta.length();
        let tension = self.tension(state, cmd, ends);
        let f_a = if sep > 0.0 && tension > 0.0 {
            (delta / sep) * tension
        } else {
            DVec3::ZERO
        };
        // The spool stops at either end of its travel.
        let length = state.deployed_length_m;
        let rate = cmd.payout_rate_m_s;
        let at_stop = (length <= 0.0 && rate < 0.0) || (length >= self.max_length_m && rate > 0.0);
        let low = self
            .auto_cut_low_tension_n
            .is_some_and(|threshold| tension <= threshold);
        let live = |value: f64| if state.intact { value } else { 0.0 };
        TetherEvaluation {
            force_on_a_world_n: f_a,
            force_on_b_world_n: -f_a,
            tension_n: tension,
            separation_m: sep,
            deployed_length_rate_m_s: live(if at_stop { 0.0 } else { rate }),
            absorbed_thermal_rate_w: live(if self.burn_through_energy_j.is_some() {
                cmd.heat_rate_w.max(0.0)
            } else {
                0.0
            }),
            low_tension_timer_rate: live(if low { 1.0 } else { 0.0 }),
        }
    }

    /// Applies the discrete events of a tether at an accepted step boundary:
    /// the spool's travel limits, the commanded cutter, thermal
    /// burn-through, overload snap, and the low-tension auto-cut with its
    /// timer reset. Returns the reason when the line is severed here.
    pub fn transition(
        &self,
        state: &mut WinchTetherState,
        cmd: &TetherCommand,
        ends: &TetherEnds,
    ) -> Option<TetherCutReason> {
        if !state.intact {
            return None;
        }
        state.deployed_length_m = state.deployed_length_m.clamp(0.0, self.max_length_m);
        let tension = self.tension(state, cmd, ends);
        let burnt = self
            .burn_through_energy_j
            .is_some_and(|limit| state.absorbed_thermal_j >= limit);
        let mut reason = if cmd.cut {
            Some(TetherCutReason::Commanded)
        } else if burnt {
            Some(TetherCutReason::ThermalBurnThrough)
        } else if tension >= self.breaking_tension_n {
            Some(TetherCutReason::OverloadSnap)
        } else {
            None
        };
        if let (None, Some(threshold)) = (reason, self.auto_cut_low_tension_n) {
            if tension > threshold {
                state.low_tension_timer_s = 0.0;
            } else if state.low_tension_timer_s >= self.auto_cut_dwell_s {
                reason = Some(TetherCutReason::TouchdownAutoCut);
            }
        }
        if reason.is_some() {
            state.intact = false;
            state.cut_reason = reason;
        }
        reason
    }
}
