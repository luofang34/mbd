//! Host environment queries as an element sees them: each is asked at the
//! time of the integrator stage being evaluated, and a query that fails
//! stops the step with the host's own error and leaves the state untouched.

#![cfg(test)]

use std::cell::RefCell;

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_multibody::{
    ContactPointSpec, EnvironmentError, FloatingBaseState, HullStation, HydrodynamicsModel,
    ModelScale, MultibodyError, MultibodyModel, Rk4, SurfaceMedium, SurfaceQuery, SurfaceSample,
    UniformGravity, WaterQuery, WaterState, Workspace,
};

fn body() -> ComponentTree {
    let spec = ComponentSpec {
        name: "body".to_owned(),
        placement: Placement::ROOT,
        dry_mass_kg: 10.0,
        dry_inertia_body: DMat3::from_diagonal(DVec3::splat(2.0)),
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint: JointSpec::FIXED,
    };
    ComponentTree::new([(spec, ComponentState::at_temperature(293.15))]).unwrap()
}

fn foot() -> ContactPointSpec {
    ContactPointSpec {
        component: ComponentId(0),
        offset_local_m: DVec3::ZERO,
        stiffness_n_m: 1.0e4,
        damping_n_s_m: 2.0e2,
        mu_static: 0.9,
        mu_dynamic: 0.7,
        bristle_stiffness_n_m: 1.0e4,
        bristle_damping_n_s_m: 2.0e2,
        damping_ramp_depth_m: ContactPointSpec::DEFAULT_DAMPING_RAMP_DEPTH_M,
    }
}

fn flat_ground() -> SurfaceSample {
    SurfaceSample {
        surface_point_world_m: DVec3::ZERO,
        outward_normal_world: DVec3::Z,
        velocity_world_m_s: DVec3::ZERO,
        medium: SurfaceMedium::Solid,
        mu_static_override: None,
        mu_dynamic_override: None,
    }
}

/// The failure a host's terrain raises.
#[derive(Debug, PartialEq, thiserror::Error)]
#[error("terrain tile {tile} is not loaded")]
struct TileMissing {
    tile: u32,
}

/// Ground that records when it is asked and fails once time passes
/// `fails_after_s`.
struct Terrain {
    asked_at_s: RefCell<Vec<f64>>,
    fails_after_s: f64,
}

impl SurfaceQuery for Terrain {
    fn surface_at(&self, _point: DVec3, time_s: f64) -> Result<SurfaceSample, EnvironmentError> {
        self.asked_at_s.borrow_mut().push(time_s);
        if time_s > self.fails_after_s {
            return Err(EnvironmentError::new(TileMissing { tile: 7 }));
        }
        Ok(flat_ground())
    }
}

const T0_S: f64 = 2.0;
const STEP_S: f64 = 0.01;

fn step(
    terrain: &Terrain,
) -> (
    Result<(), MultibodyError>,
    mbd_multibody::ModelState,
    mbd_multibody::ModelState,
) {
    let tree = body();
    let model = MultibodyModel::new(&tree, &[], ModelScale::new(1.0, 10.0, 1.0).unwrap()).unwrap();
    let elements = (UniformGravity(DVec3::new(0.0, 0.0, -9.81)), foot());
    let base = FloatingBaseState {
        position_world_m: DVec3::new(0.0, 0.0, -0.005),
        linear_vel_world_m_s: DVec3::new(0.3, 0.0, -0.1),
        ..FloatingBaseState::AT_REST
    };
    let before = model.state(&tree, &base, &elements, terrain).unwrap();
    let mut state = before.clone();
    let outcome = model
        .advance(
            &mut Rk4::new(),
            &elements,
            terrain,
            &mut state,
            T0_S,
            STEP_S,
            &mut Workspace::new(),
        )
        .map(|_| ());
    (outcome, before, state)
}

#[test]
fn surface_is_queried_at_every_stage_time() {
    let terrain = Terrain {
        asked_at_s: RefCell::new(Vec::new()),
        fails_after_s: f64::INFINITY,
    };
    let (outcome, before, after) = step(&terrain);
    outcome.unwrap();
    assert_ne!(before, after);
    // Four RK4 stages, then the accepted boundary.
    let asked = terrain.asked_at_s.borrow();
    let expected = [
        T0_S,
        T0_S + 0.5 * STEP_S,
        T0_S + 0.5 * STEP_S,
        T0_S + STEP_S,
        T0_S + STEP_S,
    ];
    assert_eq!(asked.as_slice(), expected);
}

#[test]
fn failed_query_stops_the_step_with_the_host_error_and_restores_the_state() {
    let terrain = Terrain {
        asked_at_s: RefCell::new(Vec::new()),
        fails_after_s: T0_S + 0.25 * STEP_S,
    };
    let (outcome, before, after) = step(&terrain);
    let MultibodyError::Environment(error) = outcome.unwrap_err() else {
        panic!("not an environment error");
    };
    assert_eq!(
        error.downcast_ref::<TileMissing>(),
        Some(&TileMissing { tile: 7 })
    );
    assert_eq!(error.to_string(), "terrain tile 7 is not loaded");
    // The typed variant names the host's error as its source.
    let wrapped = MultibodyError::from(error);
    let source = std::error::Error::source(&wrapped).unwrap();
    assert_eq!(source.to_string(), "terrain tile 7 is not loaded");
    // First stage answered, second failed, nothing kept.
    assert_eq!(terrain.asked_at_s.borrow().len(), 2);
    assert_eq!(before, after);
}

/// A tide: the water surface rises with time, and beyond `closes_at_s` the
/// host has no data.
struct Tide {
    closes_at_s: f64,
}

impl WaterQuery for Tide {
    fn water_at(&self, _at: DVec3, time_s: f64) -> Result<Option<WaterState>, EnvironmentError> {
        if time_s >= self.closes_at_s {
            return Err(EnvironmentError::new(TileMissing { tile: 3 }));
        }
        Ok(Some(WaterState {
            surface_origin_world_m: DVec3::new(0.0, 0.0, 0.1 * time_s),
            outward_up_world: DVec3::Z,
            fluid_density_kg_m3: 1_000.0,
            fluid_vel_world_m_s: DVec3::ZERO,
            gravity_accel_m_s2: 9.81,
        }))
    }
}

#[test]
fn water_is_sampled_at_the_time_asked_for() {
    let tree = body();
    let model = MultibodyModel::new(&tree, &[], ModelScale::new(1.0, 10.0, 1.0).unwrap()).unwrap();
    let hull = HydrodynamicsModel::new(
        ComponentId(0),
        vec![HullStation {
            offset_local_m: DVec3::ZERO,
            cross_area_m2: 0.5,
            thickness_m: 1.0,
            drag_coeff: 1.0,
            slamming_coeff: 0.0,
        }],
    )
    .unwrap();
    let tide = Tide { closes_at_s: 10.0 };
    let state = model
        .state(&tree, &FloatingBaseState::AT_REST, &hull, &tide)
        .unwrap();
    let mut work = Workspace::new();
    let kin = model.kinematics(&state, &mut work).unwrap();
    // The station sits at the origin: dry at t = 0, 0.2 m deep at t = 2.
    assert_eq!(hull.totals(&kin, &tide, 0.0).unwrap().total_buoyancy_n, 0.0);
    let wet = hull.totals(&kin, &tide, 2.0).unwrap().total_buoyancy_n;
    assert!((wet - 1_000.0 * 9.81 * 0.5 * 0.2).abs() < 1e-9);
    let closed = hull.totals(&kin, &tide, 10.0).unwrap_err();
    assert!(matches!(closed, MultibodyError::Environment(_)));
}
