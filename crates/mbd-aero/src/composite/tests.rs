use std::f64::consts::FRAC_PI_2;

use glam::DMat3;
use mbd_core::{
    AmbientGas, ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, JointState,
    Placement,
};

use super::*;
use crate::{ParachuteParams, ParachuteState, WingSurfaceParams};

fn wing_spec(name: &str, placement: Placement, joint: JointSpec) -> ComponentSpec {
    ComponentSpec {
        name: name.to_owned(),
        placement,
        dry_mass_kg: 100.0,
        dry_inertia_body: DMat3::from_diagonal(DVec3::splat(10.0)),
        thermal_capacity_j_k: 100.0,
        nominal_power_w: 0.0,
        joint,
    }
}

fn fixture_wing() -> WingSurfaceAero {
    WingSurfaceAero::new(WingSurfaceParams {
        area_m2: 20.0,
        mean_chord_m: 2.0,
        span_m: 10.0,
        aspect_ratio: 5.0,
        sweep_quarter_chord_rad: 0.0,
        sweep_control_channel: None,
        cl_alpha_incompressible: 5.0,
        cl0: 0.3,
        cl_max: 1.5,
        alpha_stall_rad: 0.3,
        cd0: 0.02,
        oswald_efficiency: 0.85,
        wave_drag_factor: 0.0,
        critical_mach: 0.8,
        cm0: 0.0,
        ac_position_body_m: DVec3::ZERO,
        chord_direction_body: DVec3::X,
        lift_normal_body: -DVec3::Z,
        flap_channel: None,
        flap_dcl: 0.0,
        flap_dcd: 0.0,
        aileron_channel: None,
        aileron_dcl: 0.0,
    })
    .unwrap()
}

fn fixture_chute() -> ParachuteCanopyAero {
    ParachuteCanopyAero::new(
        ParachuteParams {
            nominal_area_m2: 50.0,
            cd_subsonic: 0.8,
            cd_supersonic: 0.8,
            cn_alpha: 0.0,
            riser_attach_body_m: DVec3::ZERO,
            canopy_axis_body: -DVec3::X,
            inflation_time_constant_s: 1.0,
            wake_velocity_ratio: 1.0,
            rotational_damping_coeff: 0.0,
        },
        ParachuteState {
            deployed: true,
            inflation_fraction: 1.0,
            reefing_area_fraction: 1.0,
        },
    )
    .unwrap()
}

#[test]
fn detaching_chute_removes_drag_and_joint_rotation_steers_wing_force() {
    let mut tree = ComponentTree::new([
        (
            wing_spec("core", Placement::ROOT, JointSpec::FIXED),
            ComponentState::at_temperature(293.15),
        ),
        (
            wing_spec(
                "pivoting_wing",
                Placement::child_of(ComponentId(0), DVec3::new(0.0, 2.0, 0.0)),
                JointSpec::revolute(DVec3::Z, -std::f64::consts::PI, std::f64::consts::PI),
            ),
            ComponentState::at_temperature(293.15),
        ),
        (
            wing_spec(
                "chute_riser",
                Placement::child_of(ComponentId(0), DVec3::new(-3.0, 0.0, 0.0)),
                JointSpec::FIXED,
            ),
            ComponentState::at_temperature(293.15),
        ),
    ])
    .unwrap();

    let mut comp = CompositeAeroModel::default();
    comp.push(ComponentId(1), AeroElement::WingSurface(fixture_wing()));
    comp.push(
        ComponentId(2),
        AeroElement::ParachuteCanopy(fixture_chute()),
    );

    let air = AmbientGas::earth_air(101_325.0, 288.15).unwrap();
    let flow = AeroFlowState {
        velocity_air_body_m_s: DVec3::X * 60.0,
        angular_rate_body_rad_s: DVec3::ZERO,
        ambient: air,
        control_surfaces_rad: [0.0; 8],
    };

    let attached_w = comp
        .evaluate_with_poses(&flow, &tree.evaluate_pose_table().unwrap())
        .unwrap();
    tree.set_state(
        ComponentId(2),
        ComponentState {
            attached: false,
            ..ComponentState::at_temperature(293.15)
        },
    )
    .unwrap();
    let wing_only = comp
        .evaluate_with_poses(&flow, &tree.evaluate_pose_table().unwrap())
        .unwrap();
    assert!((-attached_w.force_body_n.x) > (-wing_only.force_body_n.x) + 10_000.0);
    assert!(wing_only.force_body_n.z < 0.0);
    assert!(wing_only.moment_ref_body_nm.x < 0.0);

    tree.set_state(
        ComponentId(1),
        ComponentState {
            joint: JointState::revolute(FRAC_PI_2, 0.0),
            ..ComponentState::at_temperature(293.15)
        },
    )
    .unwrap();
    let turned = comp
        .evaluate_with_poses(&flow, &tree.evaluate_pose_table().unwrap())
        .unwrap();
    assert!(turned.force_body_n.z.abs() < 1e-6);
}
