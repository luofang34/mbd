//! Symmetric positive-definite LDLᵀ factorisations for 3×3 blocks and 6×6
//! spatial inertias, with a scale-free singularity test.
//!
//! A pivot is accepted only when it keeps at least [`PIVOT_RATIO_MIN`] of the
//! original diagonal entry it was formed from. Each pivot and its diagonal
//! carry the same unit, so the test is invariant under any choice of mass,
//! length or time unit, unlike a cut-off on a determinant or a raw pivot.

use glam::{DMat3, DVec3};

use super::{SpatialForce, SpatialMat6, SpatialMotion};

/// Smallest accepted ratio of an LDLᵀ pivot to its original diagonal entry.
///
/// Rounding perturbs a pivot by a few ulp of its diagonal (~1e-16); a pivot
/// below 1e-12 of its diagonal is cancellation noise, so the matrix is treated
/// as singular rather than amplifying that noise by more than 1e12.
pub(crate) const PIVOT_RATIO_MIN: f64 = 1.0e-12;

/// LDLᵀ factors of a symmetric positive-definite 3×3 matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SymLdlt3 {
    l10: f64,
    l20: f64,
    l21: f64,
    d: DVec3,
}

impl SymLdlt3 {
    /// Factors the symmetric part of `m`; each pivot is tested against the
    /// matching entry of `reference_diag` (the diagonal it was derived from).
    pub(crate) fn new(m: DMat3, reference_diag: DVec3) -> Option<Self> {
        let a01 = 0.5 * (m.y_axis.x + m.x_axis.y);
        let a02 = 0.5 * (m.z_axis.x + m.x_axis.z);
        let a12 = 0.5 * (m.z_axis.y + m.y_axis.z);
        let d0 = m.x_axis.x;
        accept_pivot(d0, reference_diag.x)?;
        let l10 = a01 / d0;
        let l20 = a02 / d0;
        let d1 = m.y_axis.y - l10 * l10 * d0;
        accept_pivot(d1, reference_diag.y)?;
        let l21 = (a12 - l20 * l10 * d0) / d1;
        let d2 = m.z_axis.z - l20 * l20 * d0 - l21 * l21 * d1;
        accept_pivot(d2, reference_diag.z)?;
        Some(Self {
            l10,
            l20,
            l21,
            d: DVec3::new(d0, d1, d2),
        })
    }

    /// Factors `m` testing each pivot against `m`'s own diagonal.
    pub(crate) fn of(m: DMat3) -> Option<Self> {
        Self::new(m, diagonal(m))
    }

    /// Solves `m x = b`.
    pub(crate) fn solve(&self, b: DVec3) -> DVec3 {
        let y0 = b.x;
        let y1 = b.y - self.l10 * y0;
        let y2 = b.z - self.l20 * y0 - self.l21 * y1;
        let z = DVec3::new(y0, y1, y2) / self.d;
        let x2 = z.z;
        let x1 = z.y - self.l21 * x2;
        let x0 = z.x - self.l10 * x1 - self.l20 * x2;
        DVec3::new(x0, x1, x2)
    }

    /// Explicit inverse `m⁻¹`.
    pub(crate) fn inverse(&self) -> DMat3 {
        DMat3::from_cols(
            self.solve(DVec3::X),
            self.solve(DVec3::Y),
            self.solve(DVec3::Z),
        )
    }
}

/// Block LDLᵀ of a symmetric positive-definite [`SpatialMat6`]: the angular
/// block first, then its Schur complement in the linear block.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SpatialLdlt {
    aa: SymLdlt3,
    schur: SymLdlt3,
    al: DMat3,
    la: DMat3,
}

impl SpatialLdlt {
    /// Factors `m`, or `None` when a pivot falls below [`PIVOT_RATIO_MIN`] of
    /// its diagonal entry (or any entry is non-finite).
    pub(crate) fn new(m: &SpatialMat6) -> Option<Self> {
        let aa = SymLdlt3::of(m.aa)?;
        let schur_m = m.ll - m.la * aa.inverse() * m.al;
        let schur = SymLdlt3::new(schur_m, diagonal(m.ll))?;
        Some(Self {
            aa,
            schur,
            al: m.al,
            la: m.la,
        })
    }

    /// Solves `m x = rhs`.
    pub(crate) fn solve(&self, rhs: SpatialForce) -> SpatialMotion {
        let y = self.aa.solve(rhs.torque);
        let lin = self.schur.solve(rhs.force - self.la * y);
        let ang = self.aa.solve(rhs.torque - self.al * lin);
        SpatialMotion { ang, lin }
    }
}

fn accept_pivot(pivot: f64, reference: f64) -> Option<()> {
    let ok = pivot.is_finite()
        && reference.is_finite()
        && reference > 0.0
        && pivot > PIVOT_RATIO_MIN * reference;
    ok.then_some(())
}

fn diagonal(m: DMat3) -> DVec3 {
    DVec3::new(m.x_axis.x, m.y_axis.y, m.z_axis.z)
}
