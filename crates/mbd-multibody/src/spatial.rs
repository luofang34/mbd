use core::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

use glam::{DMat3, DQuat, DVec3};

pub(crate) mod ldlt;

#[cfg(test)]
mod tests;

/// 6D spatial motion vector (angular velocity/acceleration top, linear bottom)
/// expressed in a specific reference frame.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SpatialMotion {
    /// Angular component (rad/s or rad/s²).
    pub ang: DVec3,
    /// Linear component at the frame origin (m/s or m/s²).
    pub lin: DVec3,
}

impl SpatialMotion {
    /// Zero spatial twist or acceleration.
    pub const ZERO: Self = Self {
        ang: DVec3::ZERO,
        lin: DVec3::ZERO,
    };

    /// Creates a spatial motion from angular and linear components.
    pub const fn new(ang: DVec3, lin: DVec3) -> Self {
        Self { ang, lin }
    }

    /// Spatial motion Lie bracket $\text{self} \times \text{other}$.
    pub fn cross_motion(self, other: Self) -> Self {
        Self {
            ang: self.ang.cross(other.ang),
            lin: self.ang.cross(other.lin) + self.lin.cross(other.ang),
        }
    }

    /// Dual spatial force cross product $\text{self} \times^* f$.
    pub fn cross_force(self, f: SpatialForce) -> SpatialForce {
        SpatialForce {
            torque: self.ang.cross(f.torque) + self.lin.cross(f.force),
            force: self.ang.cross(f.force),
        }
    }

    /// True if all components are finite.
    pub fn is_finite(self) -> bool {
        self.ang.is_finite() && self.lin.is_finite()
    }
}

impl Add for SpatialMotion {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            ang: self.ang + rhs.ang,
            lin: self.lin + rhs.lin,
        }
    }
}

impl AddAssign for SpatialMotion {
    fn add_assign(&mut self, rhs: Self) {
        self.ang += rhs.ang;
        self.lin += rhs.lin;
    }
}

impl Sub for SpatialMotion {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            ang: self.ang - rhs.ang,
            lin: self.lin - rhs.lin,
        }
    }
}

impl SubAssign for SpatialMotion {
    fn sub_assign(&mut self, rhs: Self) {
        self.ang -= rhs.ang;
        self.lin -= rhs.lin;
    }
}

impl Neg for SpatialMotion {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            ang: -self.ang,
            lin: -self.lin,
        }
    }
}

impl Mul<f64> for SpatialMotion {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            ang: self.ang * rhs,
            lin: self.lin * rhs,
        }
    }
}

/// 6D spatial wrench vector (torque about frame origin top, linear force bottom).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct SpatialForce {
    /// Moment about the frame origin, N·m.
    pub torque: DVec3,
    /// Linear force, N.
    pub force: DVec3,
}

impl SpatialForce {
    /// Zero spatial wrench.
    pub const ZERO: Self = Self {
        torque: DVec3::ZERO,
        force: DVec3::ZERO,
    };

    /// Creates a spatial force from torque and force vectors.
    pub const fn new(torque: DVec3, force: DVec3) -> Self {
        Self { torque, force }
    }

    /// Virtual power dual pairing `torque · ang + force · lin`.
    pub fn dot(self, motion: SpatialMotion) -> f64 {
        self.torque.dot(motion.ang) + self.force.dot(motion.lin)
    }

    /// Pure point force applied at `point_local_m` inside the same frame.
    pub fn force_at_point(force: DVec3, point_local_m: DVec3) -> Self {
        Self {
            torque: point_local_m.cross(force),
            force,
        }
    }

    /// True if all components are finite.
    pub fn is_finite(self) -> bool {
        self.torque.is_finite() && self.force.is_finite()
    }
}

impl Add for SpatialForce {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            torque: self.torque + rhs.torque,
            force: self.force + rhs.force,
        }
    }
}

impl AddAssign for SpatialForce {
    fn add_assign(&mut self, rhs: Self) {
        self.torque += rhs.torque;
        self.force += rhs.force;
    }
}

impl Sub for SpatialForce {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            torque: self.torque - rhs.torque,
            force: self.force - rhs.force,
        }
    }
}

impl SubAssign for SpatialForce {
    fn sub_assign(&mut self, rhs: Self) {
        self.torque -= rhs.torque;
        self.force -= rhs.force;
    }
}

impl Neg for SpatialForce {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            torque: -self.torque,
            force: -self.force,
        }
    }
}

impl Mul<f64> for SpatialForce {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            torque: self.torque * rhs,
            force: self.force * rhs,
        }
    }
}

/// Rigid SE(3) spatial transform between a parent frame and a child frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpatialTransform {
    /// Parent-from-child rotation matrix.
    pub r_parent_from_child: DMat3,
    /// Origin of child expressed in parent axes, m.
    pub p_child_in_parent: DVec3,
}

impl SpatialTransform {
    /// Identity SE(3) transform.
    pub const IDENTITY: Self = Self {
        r_parent_from_child: DMat3::IDENTITY,
        p_child_in_parent: DVec3::ZERO,
    };

    /// Builds from child origin in parent axes and child-to-parent quaternion.
    pub fn from_pos_quat(p_child_in_parent: DVec3, q_parent_from_child: DQuat) -> Self {
        Self {
            r_parent_from_child: DMat3::from_quat(q_parent_from_child),
            p_child_in_parent,
        }
    }

    /// Transforms a parent spatial motion vector into child axes ($X_{c \leftarrow p} m_p$).
    pub fn apply_motion_to_child(&self, m_parent: SpatialMotion) -> SpatialMotion {
        let e = self.r_parent_from_child.transpose();
        SpatialMotion {
            ang: e * m_parent.ang,
            lin: e * (m_parent.lin + m_parent.ang.cross(self.p_child_in_parent)),
        }
    }

    /// Transforms a child spatial motion vector into parent axes ($X_{p \leftarrow c} m_c$).
    pub fn apply_motion_to_parent(&self, m_child: SpatialMotion) -> SpatialMotion {
        let ang = self.r_parent_from_child * m_child.ang;
        SpatialMotion {
            ang,
            lin: self.r_parent_from_child * m_child.lin + self.p_child_in_parent.cross(ang),
        }
    }

    /// Transforms a child spatial wrench into parent axes ($X^*_{p \leftarrow c} f_c$).
    pub fn apply_force_to_parent(&self, f_child: SpatialForce) -> SpatialForce {
        let force = self.r_parent_from_child * f_child.force;
        SpatialForce {
            torque: self.r_parent_from_child * f_child.torque + self.p_child_in_parent.cross(force),
            force,
        }
    }

    /// Transforms a parent spatial wrench into child axes ($X^*_{c \leftarrow p} f_p$).
    pub fn apply_force_to_child(&self, f_parent: SpatialForce) -> SpatialForce {
        let e = self.r_parent_from_child.transpose();
        SpatialForce {
            torque: e * (f_parent.torque - self.p_child_in_parent.cross(f_parent.force)),
            force: e * f_parent.force,
        }
    }
}

/// 6×6 symmetric spatial inertia / articulated-body inertia matrix mapping
/// [`SpatialMotion`] to [`SpatialForce`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpatialMat6 {
    /// Angular-angular upper-left 3×3 block.
    pub aa: DMat3,
    /// Angular-linear upper-right 3×3 block.
    pub al: DMat3,
    /// Linear-angular lower-left 3×3 block.
    pub la: DMat3,
    /// Linear-linear lower-right 3×3 block.
    pub ll: DMat3,
}

impl Default for SpatialMat6 {
    fn default() -> Self {
        Self::ZERO
    }
}

impl SpatialMat6 {
    /// Zero 6×6 matrix.
    pub const ZERO: Self = Self {
        aa: DMat3::ZERO,
        al: DMat3::ZERO,
        la: DMat3::ZERO,
        ll: DMat3::ZERO,
    };

    /// 6×6 identity-scaled matrix.
    pub fn scaled_identity(s: f64) -> Self {
        let eye = DMat3::from_diagonal(DVec3::splat(s));
        Self {
            aa: eye,
            al: DMat3::ZERO,
            la: DMat3::ZERO,
            ll: eye,
        }
    }

    /// Spatial rigid-body inertia about the frame origin given `mass_kg`,
    /// centre-of-mass `cg_local_m` in frame axes, and inertia about the CoM.
    pub fn from_rigid_inertia(mass_kg: f64, cg_local_m: DVec3, inertia_cg_local: DMat3) -> Self {
        let parallel = DMat3::from_diagonal(DVec3::splat(cg_local_m.length_squared()))
            - outer3(cg_local_m, cg_local_m);
        let aa = inertia_cg_local + parallel * mass_kg;
        let mc_skew = skew(cg_local_m) * mass_kg;
        Self {
            aa,
            al: mc_skew,
            la: mc_skew.transpose(),
            ll: DMat3::from_diagonal(DVec3::splat(mass_kg)),
        }
    }

    /// Applies `self * motion`.
    pub fn mul_motion(&self, m: SpatialMotion) -> SpatialForce {
        SpatialForce {
            torque: self.aa * m.ang + self.al * m.lin,
            force: self.la * m.ang + self.ll * m.lin,
        }
    }

    /// Outer product $a \, b^T$ of two spatial force vectors.
    pub fn outer_force(a: SpatialForce, b: SpatialForce) -> Self {
        Self {
            aa: outer3(a.torque, b.torque),
            al: outer3(a.torque, b.force),
            la: outer3(a.force, b.torque),
            ll: outer3(a.force, b.force),
        }
    }

    /// Congruence transform into the parent frame: $X^T I X$.
    pub fn transform_to_parent(&self, x: &SpatialTransform) -> Self {
        let col =
            |m: SpatialMotion| x.apply_force_to_parent(self.mul_motion(x.apply_motion_to_child(m)));
        let ax = col(SpatialMotion::new(DVec3::X, DVec3::ZERO));
        let ay = col(SpatialMotion::new(DVec3::Y, DVec3::ZERO));
        let az = col(SpatialMotion::new(DVec3::Z, DVec3::ZERO));
        let lx = col(SpatialMotion::new(DVec3::ZERO, DVec3::X));
        let ly = col(SpatialMotion::new(DVec3::ZERO, DVec3::Y));
        let lz = col(SpatialMotion::new(DVec3::ZERO, DVec3::Z));
        Self {
            aa: DMat3::from_cols(ax.torque, ay.torque, az.torque),
            al: DMat3::from_cols(lx.torque, ly.torque, lz.torque),
            la: DMat3::from_cols(ax.force, ay.force, az.force),
            ll: DMat3::from_cols(lx.force, ly.force, lz.force),
        }
    }

    /// Solves `self * x = rhs` for a symmetric positive-definite spatial
    /// inertia by block LDLᵀ; `None` when a pivot falls below a fixed
    /// rounding-level fraction of the diagonal entry it came from. Pivot and
    /// diagonal share a unit, so the test holds at any body scale.
    pub fn solve(&self, rhs: SpatialForce) -> Option<SpatialMotion> {
        self.factor().map(|f| f.solve(rhs))
    }

    /// Block LDLᵀ factorisation reused across several right-hand sides.
    pub(crate) fn factor(&self) -> Option<ldlt::SpatialLdlt> {
        ldlt::SpatialLdlt::new(self)
    }

    /// True when every entry is exactly zero: a subtree with no mass at all.
    pub(crate) fn is_exactly_zero(&self) -> bool {
        *self == Self::ZERO
    }
}

impl Add for SpatialMat6 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            aa: self.aa + rhs.aa,
            al: self.al + rhs.al,
            la: self.la + rhs.la,
            ll: self.ll + rhs.ll,
        }
    }
}

impl AddAssign for SpatialMat6 {
    fn add_assign(&mut self, rhs: Self) {
        self.aa += rhs.aa;
        self.al += rhs.al;
        self.la += rhs.la;
        self.ll += rhs.ll;
    }
}

impl Sub for SpatialMat6 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            aa: self.aa - rhs.aa,
            al: self.al - rhs.al,
            la: self.la - rhs.la,
            ll: self.ll - rhs.ll,
        }
    }
}

impl Mul<f64> for SpatialMat6 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            aa: self.aa * rhs,
            al: self.al * rhs,
            la: self.la * rhs,
            ll: self.ll * rhs,
        }
    }
}

/// Matrix `[v]_×` satisfying `skew(v) * w == v.cross(w)`.
pub(crate) fn skew(v: DVec3) -> DMat3 {
    DMat3::from_cols(
        DVec3::new(0.0, v.z, -v.y),
        DVec3::new(-v.z, 0.0, v.x),
        DVec3::new(v.y, -v.x, 0.0),
    )
}

/// Outer product `a * b^T`.
pub(crate) fn outer3(a: DVec3, b: DVec3) -> DMat3 {
    DMat3::from_cols(a * b.x, a * b.y, a * b.z)
}
