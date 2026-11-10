//! Model-based design contracts shared by simulators and design tools: the live component and frame tree with its mass properties,
//! the physics at each mounting site, ports at three levels of fidelity,
//! pluggable aerodynamics, and the station frames positions are stated in.
//!
//! The crate has no I/O and no dependency on a simulation scheduler, so a
//! design tool can evaluate the same system a simulation block steps.
//!
//! Vehicle axes follow the aerospace convention (`x` forward, `y` right, `z`
//! down); all quantities are SI.
//!
//! Everything outside the crate's tests builds without `std`, for embedded and
//! browser targets; floating-point functions come from `glam`'s `libm` backing,
//! so results do not depend on the platform's math library.

#![cfg_attr(not(test), no_std)]

extern crate alloc;

mod aero;
mod component;
mod environment;
mod error;
mod math;
mod ports;
mod station;

pub use aero::{
    AeroFlowState, AeroModel, AeroWrench, AmbientFlow, BluntBodyCapsuleAero,
    BluntBodyCapsuleParams, SUTTON_GRAVES_K_EARTH, VacuumAero,
};
pub use component::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, MassProperties, Placement,
};
pub use environment::{
    KinematicsFrame, LocalSiteKinematics, PowerRailState, SiteEnvironment, compute_site_kinematics,
};
pub use error::MbdError;
pub use ports::{
    Arinc429BnrTransducer, Arinc429Fields, DigitalBusFrame, ElectricalPinState, EngineeringValue,
    ResolverTransducer, Transducer,
};
pub use station::{METRES_PER_INCH, MassStation, StationAxes, StationFrame};
