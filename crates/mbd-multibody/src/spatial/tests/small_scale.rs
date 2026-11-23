//! `SpatialMat6::solve` on a physically valid but small rigid-body inertia.

use glam::DVec3;

use super::super::{SpatialMat6, SpatialMotion};

/// A 1 mg body of 1 mm size: inertia ~1e-12 kg m^2, so a 3x3 block
/// determinant is ~1e-36. The matrix is perfectly conditioned once scaled.
#[test]
fn solve_inverts_milligram_millimetre_inertia() {
    let (m, l) = (1.0e-6, 1.0e-3);
    let inertia = SpatialMat6::from_rigid_inertia(
        m,
        DVec3::new(0.1, -0.05, 0.02) * l,
        glam::DMat3::from_diagonal(DVec3::new(1.0, 2.0, 3.0) * (m * l * l)),
    );
    let motion = SpatialMotion::new(DVec3::new(0.3, -0.2, 0.5), DVec3::new(1.0, 2.0, -1.0) * l);
    let rhs = inertia.mul_motion(motion);
    let Some(solved) = inertia.solve(rhs) else {
        panic!(
            "solve returned None for a positive-definite inertia (det aa = {:e}, det ll = {:e})",
            inertia.aa.determinant(),
            inertia.ll.determinant()
        );
    };
    // Scaled condition number is O(10); 1e-10 relative is a rounding bound.
    let ang = (solved.ang - motion.ang).length() / motion.ang.length();
    let lin = (solved.lin - motion.lin).length() / motion.lin.length();
    assert!(
        ang < 1e-10 && lin < 1e-10,
        "ang rel {ang:e}, lin rel {lin:e}"
    );
}
