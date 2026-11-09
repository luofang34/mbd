//! Deterministic models of analog instrument mechanisms. Each takes the
//! pressures or shaft speed at the instrument's connections and returns what
//! its dial shows, through the same physical stages as the real instrument:
//! capsule or aneroid, linkage calibration, needle inertia and damping, and a
//! non-linear dial face.

mod airspeed;
mod altimeter;
mod dial;
mod gauge;
mod needle;
mod vertical_speed;

pub use airspeed::{AirspeedConfig, AirspeedIndicator};
pub use altimeter::{Altimeter, AltimeterConfig, AltimeterReading, KollsmanWindow};
pub use dial::{DialReading, DialScale};
pub use gauge::{PressureGauge, PressureGaugeConfig, Tachometer, TachometerConfig};
pub use needle::{NeedleConfig, NeedleDynamics};
pub use vertical_speed::{VerticalSpeedConfig, VerticalSpeedIndicator};
