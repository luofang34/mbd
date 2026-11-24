use core::f64::consts::FRAC_PI_2;

use glam::{DQuat, DVec3};

use super::*;

#[test]
fn joint_dof_and_validation_reject_bad_axes_and_bounds() {
    assert_eq!(JointSpec::FIXED.dof(), 0);
    assert_eq!(JointSpec::revolute(DVec3::Y, -1.0, 1.0).dof(), 1);
    assert_eq!(JointSpec::prismatic(DVec3::Z, 0.0, 2.0).dof(), 1);
    assert_eq!(JointSpec::spherical(0.5).dof(), 3);
    assert_eq!(
        JointSpec::compliant_6dof(DVec3::ONE, DVec3::ONE, DVec3::ONE, DVec3::ONE).dof(),
        6
    );

    assert!(
        JointSpec::revolute(DVec3::Y * 2.0, -1.0, 1.0)
            .validate("bad_axis")
            .is_err()
    );
    assert!(
        JointSpec::prismatic(DVec3::Z, 2.0, 1.0)
            .validate("bad_limits")
            .is_err()
    );
    assert!(JointSpec::spherical(4.0).validate("bad_cone").is_err());
    assert!(
        JointSpec::FIXED
            .with_actuator(JointActuatorSpec {
                max_effort: -1.0,
                max_velocity: 1.0,
                damping: 0.0,
                stiffness: 0.0,
                coulomb_friction: 0.0,
                backdrivable: true,
            })
            .validate("bad_act")
            .is_err()
    );
}

#[test]
fn revolute_and_prismatic_transforms_and_velocities_match_definitions() {
    let rev = JointSpec::revolute(DVec3::Z, -FRAC_PI_2, FRAC_PI_2);
    let r_state = JointState::revolute(FRAC_PI_2, 0.4);
    let (t, q) = rev.local_transform(&r_state).unwrap();
    assert_eq!(t, DVec3::ZERO);
    assert!((q * DVec3::X - DVec3::Y).length() < 1e-12);
    assert_eq!(rev.local_velocity(&r_state), (DVec3::ZERO, DVec3::Z * 0.4));

    let pris = JointSpec::prismatic(DVec3::X, 0.0, 5.0);
    let p_state = JointState::prismatic(2.5, -0.3);
    let (t, q) = pris.local_transform(&p_state).unwrap();
    assert_eq!(t, DVec3::X * 2.5);
    assert_eq!(q, DQuat::IDENTITY);
    assert_eq!(
        pris.local_velocity(&p_state),
        (DVec3::X * -0.3, DVec3::ZERO)
    );
}
