//! Composing element sets and reaching mechanism modes: a struct of
//! elements forwards through a tuple of references, and the latches of the
//! rail and hinge mechanisms read and write as typed values.

#![cfg(test)]

use glam::{DMat3, DVec3};
use mbd_core::{
    ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement, WrenchAtSite,
};
use mbd_multibody::{
    FloatingBaseState, ForceElement, HingeRelease, HingeSwingSpec, HingeSwingState, ModelScale,
    MultibodyModel, RailSlide, RailSlideSpec, RailSlideState, SurfaceMedium, SurfaceQuery,
    SurfaceSample, UniformGravity, Wheel, WheelCommand, WheelSpec, Workspace,
    forward_force_element,
};

fn part(placement: Placement, joint: JointSpec) -> ComponentSpec {
    ComponentSpec {
        name: "part".to_owned(),
        placement,
        dry_mass_kg: 40.0,
        dry_inertia_body: DMat3::from_diagonal(DVec3::splat(3.0)),
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint,
    }
}

fn tree() -> ComponentTree {
    let idle = ComponentState::at_temperature(293.15);
    ComponentTree::new([
        (part(Placement::ROOT, JointSpec::FIXED), idle),
        (
            part(
                Placement::child_of(ComponentId(0), DVec3::X),
                JointSpec::prismatic(DVec3::X, -1.0, 5.0),
            ),
            idle,
        ),
        (
            part(
                Placement::child_of(ComponentId(0), DVec3::Y),
                JointSpec::revolute(DVec3::Z, -3.0, 3.0),
            ),
            idle,
        ),
    ])
    .unwrap()
}

fn wheel() -> Wheel {
    Wheel {
        spec: WheelSpec {
            component: ComponentId(0),
            hub_offset_local_m: DVec3::ZERO,
            wheel_radius_m: 0.3,
            spring_n_m: 2.0e4,
            damper_n_s_m: 5.0e2,
            forward_axis_local: DVec3::X,
            axle_axis_local: DVec3::Y,
            peak_mu: 0.8,
            cornering_stiffness_n_rad: 2.0e3,
            rolling_resistance_coeff: 0.02,
            longitudinal_stiffness_n: 4.0e3,
            tire: mbd_multibody::TireModelSpec::DEFAULT,
        },
        spin_inertia_kg_m2: 0.5,
        command: WheelCommand {
            steer_angle_rad: 0.1,
            drive_torque_nm: 20.0,
            brake_torque_nm: 0.0,
        },
    }
}

fn rail() -> RailSlide {
    RailSlide {
        spec: RailSlideSpec {
            component: ComponentId(1),
            rail_length_m: 1.0,
            catapult_force_n: 900.0,
            guide_friction_n: 30.0,
        },
        fire: false,
    }
}

fn hinge() -> HingeRelease {
    HingeRelease {
        spec: HingeSwingSpec {
            component: ComponentId(2),
            release_angle_rad: 1.0,
            spring_torque_nm: 15.0,
            damper_nm_s_rad: 0.5,
        },
        unlatch: false,
    }
}

/// A host's own grouping of the elements of one vehicle.
struct Vehicle {
    gravity: UniformGravity,
    thrust: Option<WrenchAtSite>,
    gear: Vec<Wheel>,
    rail: RailSlide,
    hinge: HingeRelease,
}

impl<I: SurfaceQuery + ?Sized> ForceElement<I> for Vehicle {
    forward_force_element!(I, self => (
        &self.gravity,
        &self.thrust,
        &self.gear,
        &self.rail,
        &self.hinge,
    ));
}

fn vehicle() -> Vehicle {
    Vehicle {
        gravity: UniformGravity(DVec3::new(0.0, 0.0, -9.81)),
        thrust: Some(WrenchAtSite {
            site: ComponentId(0),
            point_local_m: DVec3::new(0.0, 0.1, 0.0),
            force_local_n: DVec3::new(50.0, 0.0, 10.0),
            torque_local_nm: DVec3::ZERO,
        }),
        gear: vec![wheel(), wheel()],
        rail: rail(),
        hinge: hinge(),
    }
}

fn ground() -> SurfaceSample {
    SurfaceSample {
        surface_point_world_m: DVec3::ZERO,
        outward_normal_world: DVec3::Z,
        velocity_world_m_s: DVec3::ZERO,
        medium: SurfaceMedium::Solid,
        mu_static_override: None,
        mu_dynamic_override: None,
    }
}

fn base() -> FloatingBaseState {
    FloatingBaseState {
        position_world_m: DVec3::new(0.0, 0.0, 0.28),
        linear_vel_world_m_s: DVec3::new(3.0, 0.2, 0.0),
        ..FloatingBaseState::AT_REST
    }
}

#[test]
fn struct_of_elements_evaluates_as_the_tuple_of_its_fields() {
    let (tree, v) = (tree(), vehicle());
    let model = MultibodyModel::new(&tree, &[], ModelScale::new(1.0, 120.0, 1.0).unwrap()).unwrap();
    let tuple = (v.gravity, v.thrust, v.gear.clone(), v.rail, v.hinge);
    let ground = ground();
    let mut by_struct = model.state(&tree, &base(), &v, &ground).unwrap();
    let by_tuple = model.state(&tree, &base(), &tuple, &ground).unwrap();
    assert_eq!(by_struct, by_tuple);
    // Two wheel spins, two rail latches, two hinge latches.
    assert_eq!(by_struct.auxiliary().len(), 2);
    assert_eq!(by_struct.modes().len(), 4);
    // Arm both mechanisms through their typed states.
    let armed_rail = RailSlideState {
        fired: true,
        ready_to_release: false,
    };
    let armed_hinge = HingeSwingState {
        unlatched: true,
        ready_to_release: false,
    };
    armed_rail.write_modes(&mut by_struct.modes_mut()[..2]);
    armed_hinge.write_modes(&mut by_struct.modes_mut()[2..]);
    assert_eq!(
        RailSlideState::from_modes(&by_struct.modes()[..2]),
        armed_rail
    );
    assert_eq!(
        HingeSwingState::from_modes(&by_struct.modes()[2..]),
        armed_hinge
    );
    let rates = |elements: &dyn ForceElement<SurfaceSample>| {
        let mut out = vec![0.0; by_struct.as_slice().len()];
        let mut work = Workspace::new();
        model
            .derivative(
                0.0,
                by_struct.as_slice(),
                by_struct.modes(),
                elements,
                &ground,
                &mut work,
                &mut out,
            )
            .unwrap();
        assert!(!work.used_rigid_fast_path());
        out
    };
    let (a, b) = (rates(&v), rates(&tuple));
    assert_eq!(a, b);
    // The armed catapult and the hinge spring both act.
    let nq = model.layout().position_len();
    assert!(a[nq + 6] > 1.0 && a[nq + 7] > 1.0, "{:?}", &a[nq..]);
}

#[test]
fn all_fixed_tree_reports_the_rigid_fast_path() {
    let tree = ComponentTree::new([
        (
            part(Placement::ROOT, JointSpec::FIXED),
            ComponentState::at_temperature(293.15),
        ),
        (
            part(
                Placement::child_of(ComponentId(0), DVec3::X),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap();
    let model = MultibodyModel::new(&tree, &[], ModelScale::new(1.0, 80.0, 1.0).unwrap()).unwrap();
    let gravity = UniformGravity(DVec3::new(0.0, 0.0, -9.81));
    let state = model.state(&tree, &base(), &gravity, &()).unwrap();
    let (mut work, mut rates) = (Workspace::new(), vec![0.0; state.as_slice().len()]);
    assert!(!work.used_rigid_fast_path());
    model
        .derivative(
            0.0,
            state.as_slice(),
            state.modes(),
            &gravity,
            &(),
            &mut work,
            &mut rates,
        )
        .unwrap();
    assert!(work.used_rigid_fast_path());
}
