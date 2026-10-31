use alloc::borrow::ToOwned;
use alloc::format;
use alloc::string::String;

use glam::{DMat3, DQuat, DVec3};

use crate::MbdError;

mod mass;
mod tree;

pub use mass::MassProperties;
pub use tree::ComponentTree;

#[cfg(test)]
mod tests;

/// Largest departure of a rotation's squared norm from one. Rotations are
/// checked rather than renormalised so a wrong quaternion is reported at its
/// source instead of quietly changing a lever arm.
const UNIT_QUATERNION_TOLERANCE: f64 = 1e-9;

/// Relative tolerance for inertia symmetry and semidefiniteness, covering the
/// rounding of a tensor rotated or transferred before it was entered.
const INERTIA_TOLERANCE: f64 = 1e-12;

/// Position of a component in its [`ComponentTree`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ComponentId(pub u32);

impl core::fmt::Display for ComponentId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "#{}", self.0)
    }
}

/// Where a component sits relative to its parent's frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Placement {
    /// Parent component; `None` places the component in the vehicle root frame.
    pub parent: Option<ComponentId>,
    /// Component origin in parent axes, metres.
    pub translation_m: DVec3,
    /// Component-to-parent rotation.
    pub rotation: DQuat,
}

impl Placement {
    /// The vehicle root frame itself.
    pub const ROOT: Self = Self {
        parent: None,
        translation_m: DVec3::ZERO,
        rotation: DQuat::IDENTITY,
    };

    /// Placed at `translation_m` in `parent`, with axes aligned to it.
    pub fn child_of(parent: ComponentId, translation_m: DVec3) -> Self {
        Self {
            parent: Some(parent),
            translation_m,
            rotation: DQuat::IDENTITY,
        }
    }
}

/// Immutable design-time baseline of a component.
#[derive(Debug, Clone, PartialEq)]
pub struct ComponentSpec {
    /// Human-readable name, carried into error messages.
    pub name: String,
    /// Frame of this component within the tree.
    pub placement: Placement,
    /// Structural mass, located at the component origin, kg.
    pub dry_mass_kg: f64,
    /// Structural inertia about the component origin in component axes,
    /// kg·m². Must be realisable by a nonnegative mass distribution: symmetric
    /// with nonnegative principal moments obeying the triangle inequalities.
    pub dry_inertia_body: DMat3,
    /// Lumped heat capacity, J/K.
    pub thermal_capacity_j_k: f64,
    /// Nominal electrical draw, W; negative for a source.
    pub nominal_power_w: f64,
}

impl ComponentSpec {
    fn validate(&self) -> Result<(), MbdError> {
        let invalid = |field, detail: String| MbdError::InvalidComponent {
            name: self.name.clone(),
            field,
            detail,
        };
        if !self.placement.translation_m.is_finite() {
            let detail = format!("{} is not finite", self.placement.translation_m);
            return Err(invalid("placement.translation_m", detail));
        }
        check_unit_rotation(&self.name, "placement.rotation", self.placement.rotation)?;
        check_nonnegative(&self.name, "dry_mass_kg", self.dry_mass_kg)?;
        check_nonnegative(
            &self.name,
            "thermal_capacity_j_k",
            self.thermal_capacity_j_k,
        )?;
        if !self.nominal_power_w.is_finite() {
            let detail = format!("{} is not finite", self.nominal_power_w);
            return Err(invalid("nominal_power_w", detail));
        }
        inertia_defect(&self.dry_inertia_body).map_or(Ok(()), |defect| {
            Err(invalid("dry_inertia_body", defect.to_owned()))
        })
    }
}

/// Mutable runtime state of a component: everything a simulation block must
/// snapshot to resume the tree exactly.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ComponentState {
    /// Propellant or other fluid, a point mass at the component origin, kg.
    pub fluid_mass_kg: f64,
    /// Carried payload, a point mass at the component origin, kg.
    pub payload_mass_kg: f64,
    /// Whether this component is still joined to its parent. Clearing it
    /// separates the component and its whole subtree, as a staging event does.
    pub attached: bool,
    /// Gimbal or antenna rotation about the component origin, applied in the
    /// component's placed frame; it carries the component's children with it.
    pub articulation: DQuat,
    /// Lumped temperature, K.
    pub temperature_k: f64,
}

impl ComponentState {
    /// Attached, unarticulated and empty, at `temperature_k`.
    pub fn at_temperature(temperature_k: f64) -> Self {
        Self {
            fluid_mass_kg: 0.0,
            payload_mass_kg: 0.0,
            attached: true,
            articulation: DQuat::IDENTITY,
            temperature_k,
        }
    }

    fn validate(&self, name: &str) -> Result<(), MbdError> {
        check_nonnegative(name, "fluid_mass_kg", self.fluid_mass_kg)?;
        check_nonnegative(name, "payload_mass_kg", self.payload_mass_kg)?;
        check_unit_rotation(name, "articulation", self.articulation)?;
        if self.temperature_k.is_finite() && self.temperature_k > 0.0 {
            return Ok(());
        }
        Err(MbdError::InvalidComponent {
            name: name.to_owned(),
            field: "temperature_k",
            detail: format!("{} is not a positive temperature", self.temperature_k),
        })
    }

    fn mass_kg(&self, spec: &ComponentSpec) -> f64 {
        spec.dry_mass_kg + self.fluid_mass_kg + self.payload_mass_kg
    }
}

fn check_nonnegative(name: &str, field: &'static str, value: f64) -> Result<(), MbdError> {
    if value.is_finite() && value >= 0.0 {
        return Ok(());
    }
    Err(MbdError::InvalidComponent {
        name: name.to_owned(),
        field,
        detail: format!("{value} is not a nonnegative finite value"),
    })
}

fn check_unit_rotation(name: &str, field: &'static str, rotation: DQuat) -> Result<(), MbdError> {
    if rotation.is_finite() && (rotation.length_squared() - 1.0).abs() <= UNIT_QUATERNION_TOLERANCE
    {
        return Ok(());
    }
    Err(MbdError::InvalidComponent {
        name: name.to_owned(),
        field,
        detail: format!("{rotation} is not a unit quaternion"),
    })
}

/// Why `inertia` cannot be a mass distribution's tensor, if it cannot.
///
/// `tr(I)/2·E - I` is the second-moment matrix of the distribution, and it
/// being positive semidefinite is exactly the condition for `I` to have
/// nonnegative principal moments that obey the triangle inequalities.
fn inertia_defect(inertia: &DMat3) -> Option<&'static str> {
    if !inertia.is_finite() {
        return Some("tensor is not finite");
    }
    let [xx, xy, xz, yx, yy, yz, zx, zy, zz] = inertia.to_cols_array();
    let scale = inertia
        .abs()
        .to_cols_array()
        .into_iter()
        .fold(0.0, f64::max);
    if scale == 0.0 {
        return None;
    }
    let tolerance = INERTIA_TOLERANCE * scale;
    if (xy - yx).abs() > tolerance || (xz - zx).abs() > tolerance || (yz - zy).abs() > tolerance {
        return Some("tensor is not symmetric");
    }
    let half_trace = (xx + yy + zz) / 2.0;
    let moment = (DMat3::from_diagonal(DVec3::splat(half_trace)) - *inertia) * scale.recip();
    (!is_positive_semidefinite(&moment))
        .then_some("principal moments are negative or break the triangle inequality")
}

/// All principal minors nonnegative, the test for a symmetric matrix; leading
/// minors alone would pass a matrix with a zero pivot hiding a negative one.
fn is_positive_semidefinite(m: &DMat3) -> bool {
    let [xx, xy, xz, _, yy, yz, _, _, zz] = m.to_cols_array();
    let minors = [
        xx,
        yy,
        zz,
        xx * yy - xy * xy,
        xx * zz - xz * xz,
        yy * zz - yz * yz,
        m.determinant(),
    ];
    minors.into_iter().all(|minor| minor >= -INERTIA_TOLERANCE)
}
