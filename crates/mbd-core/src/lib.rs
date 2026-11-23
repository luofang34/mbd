//! Model-based design contracts shared by simulators and design tools: the live
//! component and frame tree with kinematic joints and topology transformations,
//! multi-species ambient gas, mounting-site physics, conserved flows, faults,
//! layered ports, and pluggable aerodynamics.
//!
//! Vehicle axes follow the aerospace convention (`x` forward, `y` right, `z`
//! down); all quantities are SI. Everything outside tests builds without `std`.

#![cfg_attr(not(test), no_std)]

extern crate alloc;

mod aero;
mod component;
mod environment;
mod error;
mod fault;
mod flow;
mod gas;
mod joint;
mod math;
mod ports;

pub use aero::{AeroFlowState, AeroModel, AeroWrench, VacuumAero};
pub use component::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, GraftedTree, MassProperties,
    Placement, RerootedTree, SplitTrees,
};
pub use environment::{
    KinematicsFrame, LocalSiteKinematics, PowerRailState, SiteEnvironment, compute_site_kinematics,
};
pub use error::MbdError;
pub use fault::FaultState;
pub use flow::{
    DynamicMassContribution, ElectricalLoad, LinkPose, PoseTable, ThermalLoad, WrenchAtSite,
};
pub use gas::{
    AVOGADRO_CONSTANT_MOL, AmbientGas, BOLTZMANN_CONSTANT_J_K, GasComposition, GasSpecies,
    MAX_GAS_SPECIES, SUTTON_GRAVES_K_EARTH, SUTTON_GRAVES_K_MARS, SpeciesFraction,
    UNIVERSAL_GAS_CONSTANT_J_MOL_K,
};
pub use joint::{JointActuatorSpec, JointKind, JointSpec, JointState, JointStopSpec};
pub use ports::{
    Arinc429BnrTransducer, Arinc429Fields, DigitalBusFrame, ElectricalPinState, EngineeringValue,
    ResolverTransducer, Transducer,
};
