use glam::DVec2;

/// `√x`.
pub(crate) fn sqrt(x: f64) -> f64 {
    DVec2::splat(x).sqrt().x
}

/// `(sin x, cos x)`.
pub(crate) fn sin_cos(x: f64) -> (f64, f64) {
    let unit = DVec2::from_angle(x);
    (unit.y, unit.x)
}

/// The angle of the point `(x, y)` from the `x` axis, in `-π..=π`.
pub(crate) fn atan2(y: f64, x: f64) -> f64 {
    DVec2::new(x, y).to_angle()
}

/// `x` rounded to the nearest integer, halves away from zero.
pub(crate) fn round(x: f64) -> f64 {
    DVec2::splat(x).round().x
}
