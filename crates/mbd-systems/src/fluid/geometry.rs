use core::f64::consts::PI;

use glam::{DMat3, DVec3};
use libm::pow;

use crate::error::{SystemsError, check_positive, normalize_axis};

/// Physical geometry of a fluid tank expressed in its host component's local frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TankGeometry {
    /// Spherical tank with uniform radius.
    Sphere {
        /// Internal sphere radius, m.
        radius_m: f64,
    },
    /// Right circular cylinder oriented along `axis_local`.
    Cylinder {
        /// Internal cylinder radius, m.
        radius_m: f64,
        /// Internal cylinder length, m.
        length_m: f64,
        /// Unit cylinder symmetry axis in host component local frame.
        axis_local: DVec3,
    },
    /// Tri-axial ellipsoid aligned with host component local axes.
    Ellipsoid {
        /// Positive semi-axes `(a, b, c)` in host component local frame, m.
        semi_axes_m: DVec3,
    },
    /// Rectangular/prismatic box tank (conformal aircraft/wing/trim tanks).
    Box {
        /// Positive half-extents `(hx, hy, hz)` in host component local frame, m.
        half_extents_m: DVec3,
    },
}

impl TankGeometry {
    /// Validates finite positive dimensions and cylinder axis direction.
    pub fn validate(&self) -> Result<(), SystemsError> {
        match *self {
            Self::Sphere { radius_m } => {
                check_positive("tank.geometry.radius_m", radius_m)?;
            }
            Self::Cylinder {
                radius_m,
                length_m,
                axis_local,
            } => {
                check_positive("tank.geometry.radius_m", radius_m)?;
                check_positive("tank.geometry.length_m", length_m)?;
                normalize_axis("tank.geometry.axis_local", axis_local)?;
            }
            Self::Ellipsoid { semi_axes_m } => {
                check_positive("tank.geometry.semi_axes_m.x", semi_axes_m.x)?;
                check_positive("tank.geometry.semi_axes_m.y", semi_axes_m.y)?;
                check_positive("tank.geometry.semi_axes_m.z", semi_axes_m.z)?;
            }
            Self::Box { half_extents_m } => {
                check_positive("tank.geometry.half_extents_m.x", half_extents_m.x)?;
                check_positive("tank.geometry.half_extents_m.y", half_extents_m.y)?;
                check_positive("tank.geometry.half_extents_m.z", half_extents_m.z)?;
            }
        }
        Ok(())
    }

    /// Internal geometric volume, m³.
    pub fn volume_m3(&self) -> f64 {
        match *self {
            Self::Sphere { radius_m } => (4.0 / 3.0) * PI * radius_m * radius_m * radius_m,
            Self::Cylinder {
                radius_m, length_m, ..
            } => PI * radius_m * radius_m * length_m,
            Self::Ellipsoid { semi_axes_m } => {
                (4.0 / 3.0) * PI * semi_axes_m.x * semi_axes_m.y * semi_axes_m.z
            }
            Self::Box { half_extents_m } => {
                8.0 * half_extents_m.x * half_extents_m.y * half_extents_m.z
            }
        }
    }

    pub(crate) fn extent_along_unit_axis(&self, unit_dir: DVec3) -> f64 {
        let d = unit_dir.abs();
        match *self {
            Self::Sphere { radius_m } => radius_m,
            Self::Cylinder {
                radius_m,
                length_m,
                axis_local,
            } => {
                let axis = axis_local.normalize_or_zero();
                let cos_a = d.dot(axis.abs()).min(1.0);
                let sin_a = libm::sqrt((1.0 - cos_a * cos_a).max(0.0));
                0.5 * length_m * cos_a + radius_m * sin_a
            }
            Self::Ellipsoid { semi_axes_m } => libm::sqrt(
                d.x * d.x * semi_axes_m.x * semi_axes_m.x
                    + d.y * d.y * semi_axes_m.y * semi_axes_m.y
                    + d.z * d.z * semi_axes_m.z * semi_axes_m.z,
            ),
            Self::Box { half_extents_m } => d.dot(half_extents_m),
        }
    }
}

pub(crate) fn fluid_inertia_tensor(
    geometry: TankGeometry,
    mass_kg: f64,
    fill_fraction: f64,
) -> DMat3 {
    let eta = fill_fraction.clamp(0.0, 1.0);
    if mass_kg <= 0.0 || eta <= 0.0 {
        return DMat3::ZERO;
    }
    match geometry {
        TankGeometry::Sphere { radius_m } => {
            let scale = pow(eta, 2.0 / 3.0);
            let i_iso = 0.4 * mass_kg * radius_m * radius_m * scale;
            DMat3::from_diagonal(DVec3::splat(i_iso))
        }
        TankGeometry::Cylinder {
            radius_m,
            length_m,
            axis_local,
        } => {
            let h = (length_m * eta).max(1.0e-6);
            let i_ax = 0.5 * mass_kg * radius_m * radius_m;
            let i_tr = mass_kg * (3.0 * radius_m * radius_m + h * h) / 12.0;
            let e_ax = axis_local.normalize_or(DVec3::Z);
            let ref_up = if e_ax.z.abs() < 0.9 {
                DVec3::Z
            } else {
                DVec3::X
            };
            let e_u = e_ax.cross(ref_up).normalize();
            let e_v = e_ax.cross(e_u);
            let rot = DMat3::from_cols(e_ax, e_u, e_v);
            rot * DMat3::from_diagonal(DVec3::new(i_ax, i_tr, i_tr)) * rot.transpose()
        }
        TankGeometry::Ellipsoid { semi_axes_m } => {
            let scale = pow(eta, 2.0 / 3.0);
            let a2 = semi_axes_m.x * semi_axes_m.x * scale;
            let b2 = semi_axes_m.y * semi_axes_m.y * scale;
            let c2 = semi_axes_m.z * semi_axes_m.z * scale;
            DMat3::from_diagonal(DVec3::new(
                0.2 * mass_kg * (b2 + c2),
                0.2 * mass_kg * (a2 + c2),
                0.2 * mass_kg * (a2 + b2),
            ))
        }
        TankGeometry::Box { half_extents_m } => {
            let scale = pow(eta, 2.0 / 3.0);
            let hx2 = half_extents_m.x * half_extents_m.x * scale;
            let hy2 = half_extents_m.y * half_extents_m.y * scale;
            let hz2 = half_extents_m.z * half_extents_m.z * scale;
            DMat3::from_diagonal(DVec3::new(
                mass_kg * (hy2 + hz2) / 3.0,
                mass_kg * (hx2 + hz2) / 3.0,
                mass_kg * (hx2 + hy2) / 3.0,
            ))
        }
    }
}
