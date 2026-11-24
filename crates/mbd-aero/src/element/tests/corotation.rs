//! An atmosphere turning with its planet: a vehicle that turns with it,
//! every point moving at the local air velocity, has no motion relative to
//! the air and feels no aerodynamic load.

use glam::DQuat;
use mbd_core::ComponentId;
use mbd_multibody::{EnvironmentError, FloatingBaseState, MultibodyModel, TimeScale, Workspace};

use super::heavy::heavy_tree;
use super::*;
use crate::{AeroInputs, AirSample};

/// Air turning rigidly about the world origin at `spin`, reporting
/// `reported_spin` as its angular velocity.
struct TurningAir {
    spin: DVec3,
    reported_spin: DVec3,
}

impl AeroInputs for TurningAir {
    fn air_at(&self, position_world_m: DVec3, _time_s: f64) -> Result<AirSample, EnvironmentError> {
        Ok(AirSample {
            ambient: sea(),
            wind_world_m_s: self.spin.cross(position_world_m),
            angular_velocity_world_rad_s: self.reported_spin,
        })
    }

    fn controls(&self) -> &[f64] {
        &[]
    }
}

/// One of every element on the links of the heavy vehicle, the rotor at
/// rest with its controls at zero.
fn elements() -> Vec<AeroElement> {
    [
        (0, body_fin()),
        (0, capsule()),
        (1, wing()),
        (2, wing()),
        (3, chute()),
        (3, panel()),
        (4, rotor()),
        (4, dummy()),
    ]
    .into_iter()
    .map(|(c, kind)| AeroElement::new(ComponentId(c), kind))
    .collect()
}

/// Largest force and largest moment about the reference station over all
/// elements, for the vehicle turning at `spin` about the world origin in
/// `air`.
fn worst_loads(spin: DVec3, air: &TurningAir) -> (f64, f64) {
    let tree = heavy_tree(0.3, 0.0);
    let model =
        MultibodyModel::with_derived_scale(&tree, &[], TimeScale::Acceleration(9.81)).unwrap();
    let rotation = DQuat::from_axis_angle(DVec3::new(0.3, -0.5, 0.8).normalize(), 0.7);
    let position = DVec3::new(2_000.0, -500.0, 300.0);
    let base = FloatingBaseState {
        position_world_m: position,
        rotation_world_from_root: rotation,
        linear_vel_world_m_s: spin.cross(position),
        angular_vel_root_rad_s: rotation.conjugate() * spin,
    };
    let state = model.state(&tree, &base, &elements(), air).unwrap();
    let mut work = Workspace::new();
    let kin = model.kinematics(&state, &mut work).unwrap();
    let (mut force, mut moment) = (0.0_f64, 0.0_f64);
    let (mut aux, mut modes) = (state.auxiliary(), state.modes());
    for e in elements() {
        let n = ForceElement::<TurningAir>::state_len(&e);
        let (s, m) = (&aux[..n], &modes[..n]);
        (aux, modes) = (&aux[n..], &modes[n..]);
        let flow = e.flow(&kin, air, 0.0).unwrap();
        let w = e.loads(&flow, s, m).unwrap().wrench;
        force = force.max(w.force_link_n.length());
        moment = moment.max(w.moment_ref_link_nm.length());
    }
    (force, moment)
}

#[test]
fn vehicle_turning_with_the_air_feels_no_aerodynamic_load() {
    let spin = DVec3::new(0.02, -0.03, 0.05);
    let still = TurningAir {
        spin: DVec3::ZERO,
        reported_spin: DVec3::ZERO,
    };
    let turning = TurningAir {
        spin,
        reported_spin: spin,
    };
    // The loads the same motion meets in still air set the scale.
    let (f_still, m_still) = worst_loads(spin, &still);
    assert!(f_still > 1.0e4 && m_still > 1.0e4);
    // Relative velocities are rounding of ~100 m/s, 1e-14 m/s, and no load
    // grows faster than linearly from rest.
    let (force, moment) = worst_loads(spin, &turning);
    assert!(
        force < 1.0e-12 * f_still && moment < 1.0e-12 * m_still,
        "co-rotating force {force:e} N of {f_still:e}, moment {moment:e} N·m of {m_still:e}"
    );
    // Air that moves as the turning mass but is reported as not turning
    // leaves the vehicle spinning relative to it: the rate terms act.
    let unreported = TurningAir {
        spin,
        reported_spin: DVec3::ZERO,
    };
    let (_, m_spurious) = worst_loads(spin, &unreported);
    assert!(
        m_spurious > 1.0e6 * moment.max(f64::MIN_POSITIVE),
        "{m_spurious:e}"
    );
}

/// A large flapping-wing panel on a 5 kg body with spherical inertia: its
/// entrained air is a tenth of the body's mass.
fn winged_ball() -> (mbd_core::ComponentTree, AeroElement) {
    let spec = mbd_core::ComponentSpec {
        name: "ball".to_owned(),
        placement: mbd_core::Placement::ROOT,
        dry_mass_kg: 5.0,
        dry_inertia_body: glam::DMat3::from_diagonal(DVec3::splat(2.0)),
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint: mbd_core::JointSpec::FIXED,
    };
    let state = mbd_core::ComponentState::at_temperature(293.15);
    let wing = FlappingWingAero::new(crate::FlappingWingParams {
        root_link_m: DVec3::new(0.1, 0.0, 0.0),
        span_axis_link: DVec3::Y,
        chord_axis_link: DVec3::X,
        length_m: 2.0,
        chords_m: vec![0.5; 4],
        rotation_axis_chord_fraction: 0.3,
        center_of_pressure_chord_fraction: 0.25,
    })
    .unwrap();
    (
        mbd_core::ComponentTree::new([(spec, state)]).unwrap(),
        AeroElement::new(ComponentId(0), AeroElementKind::FlappingWing(wing)),
    )
}

#[test]
fn entrained_air_of_a_wing_turns_with_the_air() {
    let spin = DVec3::new(0.02, -0.03, 0.05);
    let position = DVec3::new(2_000.0, -500.0, 300.0);
    let rotation = DQuat::from_axis_angle(DVec3::new(0.3, -0.5, 0.8).normalize(), 0.7);
    let (tree, wing) = winged_ball();
    let model =
        MultibodyModel::with_derived_scale(&tree, &[], TimeScale::Acceleration(9.81)).unwrap();
    let base = FloatingBaseState {
        position_world_m: position,
        rotation_world_from_root: rotation,
        linear_vel_world_m_s: spin.cross(position),
        angular_vel_root_rad_s: rotation.conjugate() * spin,
    };
    // The force that keeps the ball on its circle with the air.
    let centripetal = spin.cross(spin.cross(position));
    let support = mbd_core::WrenchAtSite {
        site: ComponentId(0),
        point_local_m: DVec3::ZERO,
        force_local_n: rotation.conjugate() * centripetal * 5.0,
        torque_local_nm: DVec3::ZERO,
    };
    let accel = |with_wing: bool, air: &TurningAir| {
        let elements = (support, with_wing.then(|| wing.clone()));
        let state = model.state(&tree, &base, &elements, air).unwrap();
        let (mut work, mut rates) = (Workspace::new(), vec![0.0; state.as_slice().len()]);
        let x = state.as_slice();
        model
            .derivative(0.0, x, state.modes(), &elements, air, &mut work, &mut rates)
            .unwrap();
        [
            work.base_linear_accel_world_m_s2(),
            work.base_angular_accel_root_rad_s2(),
        ]
    };
    let turning = TurningAir {
        spin,
        reported_spin: spin,
    };
    let (bare, winged) = (accel(false, &turning), accel(true, &turning));
    let scale = centripetal.length();
    assert!(
        (winged[0] - bare[0]).length() < 1.0e-12 * scale
            && (winged[1] - bare[1]).length() < 1.0e-12 * scale,
        "{winged:?} vs {bare:?}"
    );
    // Through still air the same wing is loaded and its entrained air
    // resists.
    let still = TurningAir {
        spin: DVec3::ZERO,
        reported_spin: DVec3::ZERO,
    };
    assert!((accel(true, &still)[0] - bare[0]).length() > scale);
}
