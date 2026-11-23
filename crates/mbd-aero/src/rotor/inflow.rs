//! Induced velocity of one blade annulus, where blade-element and momentum
//! thrust agree, and the numerical settings of that solve.

use core::f64::consts::PI;

use mbd_core::{AmbientGas, MbdError};

use super::{RotorBemParams, element_loads};
use crate::validation::{check_nonnegative, check_positive};

/// Numerical settings of a [`super::RotorBemAero`]: speeds, in m/s, that
/// bound the induced-velocity solve and the blade sections' tangential
/// speed. They are not derived from the rotor because the solve's result
/// depends on them, and the defaults are the values the element's
/// regression cases were recorded with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RotorBemNumerics {
    /// Added to `0.6 Ωr + |V|` to form the upper end of the bracket that
    /// the induced velocity is bisected in, m/s. A root above the bracket
    /// returns its upper end.
    pub inflow_bracket_pad_m_s: f64,
    /// How far below the momentum-theory edge of the windmill-brake state,
    /// `v_i = −V/2` for a climb speed `V`, the induced velocity may go,
    /// m/s. A root below it returns `−(V/2 + margin)`.
    pub windmill_inflow_margin_m_s: f64,
    /// Floor on a blade section's tangential speed, m/s. Reverse flow over
    /// the retreating blade is not modelled; a section whose tangential
    /// speed would fall below the floor sees the floor instead.
    pub min_tangential_speed_m_s: f64,
}

impl RotorBemNumerics {
    /// 25 m/s of bracket pad, 10 m/s of windmill margin and a 0.01 m/s
    /// tangential floor.
    pub const DEFAULT: Self = Self {
        inflow_bracket_pad_m_s: 25.0,
        windmill_inflow_margin_m_s: 10.0,
        min_tangential_speed_m_s: 0.01,
    };

    pub(super) fn validate(&self) -> Result<(), MbdError> {
        check_positive("inflow_bracket_pad_m_s", self.inflow_bracket_pad_m_s)?;
        check_nonnegative(
            "windmill_inflow_margin_m_s",
            self.windmill_inflow_margin_m_s,
        )?;
        check_nonnegative("min_tangential_speed_m_s", self.min_tangential_speed_m_s)
    }
}

impl Default for RotorBemNumerics {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Bisection steps of the inflow solve.
const BISECTION_STEPS: u32 = 22;

/// Induced velocity at radius `r`: the root, in `[lo, bound]`, of
/// blade-element minus momentum thrust, or the end of the bracket the root
/// lies beyond.
#[allow(clippy::too_many_arguments)]
pub(super) fn solve_induced_inflow(
    p: &RotorBemParams,
    ambient: &AmbientGas,
    omega_r: f64,
    theta: f64,
    v_axial: f64,
    v_edge: f64,
    r: f64,
    dr: f64,
    bound: f64,
) -> f64 {
    let margin = p.numerics.windmill_inflow_margin_m_s;
    let mut lo = -bound.min(v_axial.max(0.0) * 0.5 + margin);
    let mut hi = bound;
    let res = |vi: f64| {
        let u_p = v_axial + p.coaxial_interference_factor * vi;
        let (dt_bet, _, _, _) = element_loads(p, ambient, omega_r, u_p, theta, r, dr);
        // `omega_r > 0` here (the caller returns early at zero speed and
        // `r ≥ hub_radius_m > 0`), so `atan2` is defined; the floor on
        // `sin φ` keeps the Prandtl exponent finite in pure edgewise flow.
        let phi = libm::atan2(u_p.abs(), omega_r);
        let prandtl_arg =
            (0.5 * f64::from(p.blade_count) * (p.radius_m - r)) / (r * libm::sin(phi).max(0.02));
        let f_tip = (2.0 / PI) * libm::acos(libm::exp(-prandtl_arg.min(50.0)).min(1.0));
        let v_disk = libm::sqrt(v_edge * v_edge + u_p * u_p);
        let dt_mom = 4.0 * PI * r * ambient.density_kg_m3 * f_tip.max(0.1) * vi * v_disk * dr;
        dt_bet - dt_mom
    };
    if res(lo) <= 0.0 {
        return lo;
    }
    if res(hi) >= 0.0 {
        return hi;
    }
    for _ in 0..BISECTION_STEPS {
        let mid = 0.5 * (lo + hi);
        if res(mid) > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}
