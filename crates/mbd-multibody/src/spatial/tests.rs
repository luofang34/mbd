use glam::{DMat3, DQuat, DVec3};

use super::*;

mod small_scale;

#[test]
fn cross_products_and_transforms_preserve_virtual_power_and_duality() {
    let a = SpatialMotion::new(DVec3::new(0.4, -1.2, 0.7), DVec3::new(2.0, -0.5, 1.5));
    let b = SpatialMotion::new(DVec3::new(-0.3, 0.9, 1.1), DVec3::new(-1.0, 3.0, 0.2));
    let f = SpatialForce::new(DVec3::new(5.0, -2.0, 3.0), DVec3::new(10.0, 4.0, -7.0));

    let lhs = f.dot(a.cross_motion(b));
    let rhs = -(a.cross_force(f)).dot(b);
    assert!((lhs - rhs).abs() < 1e-12);

    let x = SpatialTransform::from_pos_quat(
        DVec3::new(1.5, -0.7, 2.3),
        DQuat::from_euler(glam::EulerRot::YXZ, 0.4, -0.7, 1.1),
    );
    let m_c = x.apply_motion_to_child(a);
    let a_round = x.apply_motion_to_parent(m_c);
    assert!((a_round.ang - a.ang).length() < 1e-12);
    assert!((a_round.lin - a.lin).length() < 1e-12);

    let f_p = x.apply_force_to_parent(f);
    assert!((f_p.dot(a) - f.dot(m_c)).abs() < 1e-12);
    let f_round = x.apply_force_to_child(f_p);
    assert!((f_round.torque - f.torque).length() < 1e-12);
    assert!((f_round.force - f.force).length() < 1e-12);
}

#[test]
fn spatial_inertia_transform_matches_parallel_axis_and_solve_inverts_exactly() {
    let mass = 25.0;
    let cg_local = DVec3::new(0.4, -0.2, 0.6);
    let i_cm = DMat3::from_diagonal(DVec3::new(4.0, 7.0, 9.0));
    let i_child = SpatialMat6::from_rigid_inertia(mass, cg_local, i_cm);

    let x = SpatialTransform::from_pos_quat(
        DVec3::new(2.0, 1.0, -0.5),
        DQuat::from_axis_angle(DVec3::new(1.0, 2.0, 3.0).normalize(), 0.8),
    );
    let i_parent = i_child.transform_to_parent(&x);
    let v_p = SpatialMotion::new(DVec3::new(0.5, -0.3, 0.8), DVec3::new(3.0, -2.0, 1.0));
    let v_c = x.apply_motion_to_child(v_p);
    let ke_parent = (i_parent.mul_motion(v_p)).dot(v_p);
    let ke_child = (i_child.mul_motion(v_c)).dot(v_c);
    assert!((ke_parent - ke_child).abs() < 1e-11);

    let rhs = i_parent.mul_motion(v_p);
    let recovered = i_parent.solve(rhs).unwrap();
    assert!((recovered.ang - v_p.ang).length() < 1e-12);
    assert!((recovered.lin - v_p.lin).length() < 1e-12);
}
