//! Pitot-static pressure laws for air with `γ = 1.4`.
//!
//! A pitot tube brings the flow isentropically to rest, so below Mach 1 it
//! senses the total pressure; above Mach 1 a normal shock stands ahead of it
//! and it senses the total pressure behind that shock (Rayleigh's pitot
//! formula). The impact pressure `qc` is that sensed pressure minus static.
//! Calibrated airspeed is defined by applying the same law at sea-level
//! standard conditions, which is how every airspeed indicator is calibrated.

use crate::isa::{ISA_P0_PA, ISA_RHO0_KG_M3, ISA_SPEED_OF_SOUND0_M_S, speed_of_sound_m_s};

#[cfg(test)]
mod tests;

/// `((γ+1)/2)^(γ/(γ−1))` = `1.2^3.5`: the pitot-to-static ratio at Mach 1.
const SONIC_TOTAL_RATIO: f64 = 1.892_929_158_737_854;
/// `K = 1.2^3.5 · 6^2.5` in Rayleigh's pitot formula for `γ = 1.4` written as
/// `pt2/p = K · M⁷ / (7M² − 1)^2.5`, which meets the isentropic law at Mach 1.
const RAYLEIGH_K: f64 = 166.921_580_093_168_27;
/// Iterations of the supersonic Mach solve. The fixed-point map contracts
/// quickly above Mach 1, so this many reach f64 precision up to Mach 10.
const SUPERSONIC_ITERATIONS: usize = 40;

/// The ratio of pitot pressure to static pressure at `mach`.
fn pitot_ratio(mach: f64) -> f64 {
    let m2 = mach * mach;
    if mach <= 1.0 {
        libm::pow(1.0 + 0.2 * m2, 3.5)
    } else {
        RAYLEIGH_K * libm::pow(mach, 7.0) / libm::pow(7.0 * m2 - 1.0, 2.5)
    }
}

/// Impact pressure `qc = p_pitot − p_static`, Pa, at static pressure
/// `static_pa` and flight Mach number `mach`. Negative Mach reads as zero.
pub fn impact_pressure_pa(static_pa: f64, mach: f64) -> f64 {
    static_pa * (pitot_ratio(mach.max(0.0)) - 1.0)
}

/// The flight Mach number that produces impact pressure `qc_pa` at static
/// pressure `static_pa`. Closed form below Mach 1; above it, a fixed number of
/// iterations of the rearranged Rayleigh formula, so it always terminates.
/// Non-positive `qc_pa` or `static_pa` reads as Mach 0.
pub fn mach_from_impact_pressure(qc_pa: f64, static_pa: f64) -> f64 {
    if !(qc_pa > 0.0 && static_pa > 0.0) {
        return 0.0;
    }
    let ratio = qc_pa / static_pa + 1.0;
    if ratio <= SONIC_TOTAL_RATIO {
        return libm::sqrt(5.0 * (libm::pow(ratio, 1.0 / 3.5) - 1.0));
    }
    // M² = ratio · (7 − 1/M²)^2.5 / K, iterated from Mach 1.
    let mut mach: f64 = 1.0;
    for _ in 0..SUPERSONIC_ITERATIONS {
        let m2 = mach * mach;
        mach = libm::sqrt(ratio * libm::pow(7.0 - 1.0 / m2, 2.5) / RAYLEIGH_K);
    }
    mach
}

/// Impact pressure, Pa, that an airspeed indicator converts to calibrated
/// airspeed `cas_m_s`: the pitot law at sea-level standard pressure and speed
/// of sound.
pub fn impact_pressure_from_cas_pa(cas_m_s: f64) -> f64 {
    impact_pressure_pa(ISA_P0_PA, cas_m_s / ISA_SPEED_OF_SOUND0_M_S)
}

/// Calibrated airspeed, m/s, for impact pressure `qc_pa`: the inverse of
/// [`impact_pressure_from_cas_pa`], and what a perfect airspeed indicator
/// shows.
pub fn cas_from_impact_pressure_m_s(qc_pa: f64) -> f64 {
    ISA_SPEED_OF_SOUND0_M_S * mach_from_impact_pressure(qc_pa, ISA_P0_PA)
}

/// True airspeed, m/s, at `mach` in air at static `temperature_k`.
pub fn tas_m_s(mach: f64, temperature_k: f64) -> f64 {
    mach * speed_of_sound_m_s(temperature_k)
}

/// Equivalent airspeed, m/s: the sea-level speed with the same dynamic
/// pressure `½ρV² = ½γpM²`.
pub fn eas_m_s(mach: f64, static_pa: f64) -> f64 {
    let dynamic_pa = 0.7 * static_pa * mach * mach;
    libm::sqrt(2.0 * dynamic_pa.max(0.0) / ISA_RHO0_KG_M3)
}
