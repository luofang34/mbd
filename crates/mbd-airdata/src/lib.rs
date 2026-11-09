//! Air data from first principles and the mechanical instruments that display
//! it: the ICAO / US-1976 standard atmosphere, pitot-static pressure laws,
//! pneumatic line lag, static-source position error, and deterministic models
//! of the capsules, aneroids, capillary leaks, gear trains and needles inside
//! real analog instruments.
//!
//! An airspeed indicator here is a diaphragm capsule reading total minus
//! static pressure and calibrated to the sea-level CAS law, so it shows
//! compressibility and position error the way the real one does; an altimeter
//! is an aneroid with a Kollsman subscale; a vertical speed indicator is a
//! capsule whose case breathes through a calibrated leak.
//!
//! The crate builds without `std` or an allocator. Tables have fixed capacity,
//! every mechanism's state is `Copy` plain data a simulator can snapshot and
//! rewind, every step is a fixed amount of work, and transcendental functions
//! come from `libm`, so results are bit-identical across platforms. All
//! quantities are SI unless a name says otherwise.

#![cfg_attr(not(test), no_std)]

mod error;
mod isa;
mod mechanism;
mod pitot;
mod pneumatic;
mod position_error;
mod table;
mod units;

pub use error::AirdataError;
pub use isa::{
    Atmosphere, GAMMA_AIR, ISA_G0_M_S2, ISA_LAPSE_RATE_K_M, ISA_P0_PA, ISA_RHO0_KG_M3,
    ISA_SPEED_OF_SOUND0_M_S, ISA_T0_K, R_AIR_J_KG_K, geopotential_altitude_m, isa_atmosphere,
    pressure_altitude_m, pressure_lapse_pa_m, speed_of_sound_m_s,
};
pub use mechanism::{
    AirspeedConfig, AirspeedIndicator, Altimeter, AltimeterConfig, AltimeterReading, DialReading,
    DialScale, KollsmanWindow, NeedleConfig, NeedleDynamics, PressureGauge, PressureGaugeConfig,
    Tachometer, TachometerConfig, VerticalSpeedConfig, VerticalSpeedIndicator,
};
pub use pitot::{
    cas_from_impact_pressure_m_s, eas_m_s, impact_pressure_from_cas_pa, impact_pressure_pa,
    mach_from_impact_pressure, tas_m_s,
};
pub use pneumatic::PneumaticLag;
pub use position_error::{PositionErrorCurve, PositionErrorTable};
pub use table::{PiecewiseLinear, TABLE_CAPACITY};
pub use units::DialUnit;
