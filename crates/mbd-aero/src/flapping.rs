//! Quasi-steady blade-element model of a flapping insect wing at Reynolds
//! numbers of about 10² to 10⁴.
//!
//! The wing is a rigid flat plate split into spanwise strips. Each strip
//! carries three terms of the revised quasi-steady model of Sane &
//! Dickinson (2002), J. Exp. Biol. 205:1087, eq. 2:
//!
//! - **Translational force** from the lift and drag coefficients measured on
//!   a dynamically scaled robotic *Drosophila* wing (Dickinson, Lehmann &
//!   Sane 1999, Science 284:1954):
//!   `C_L(α) = 0.225 + 1.58 sin(2.13α − 7.20°)`,
//!   `C_D(α) = 1.92 − 1.55 cos(2.04α − 9.82°)`, α in degrees over the
//!   measured 0°–90°.
//! - **Rotational circulation force** normal to the plate,
//!   `F′_rot = C_rot ρ U ω c²` with the theoretical coefficient
//!   `C_rot = π(0.75 − x̂₀)` for a rotation axis at chord fraction `x̂₀`
//!   from the leading edge (Sane & Dickinson 2002, eqs 8–11, after Fung
//!   1969). The same paper measures a coefficient that also rises with the
//!   non-dimensional rotation rate; that dependence is not modelled.
//! - **Added mass** of the two-dimensional flat plate: `ρπc²/4` per unit
//!   span normal to the plate at mid-chord and `ρπc⁴/128` per unit span in
//!   rotation about mid-chord: the potential-flow added masses of an
//!   ellipse, `m₂₂ = ρπa²` and `m₆₆ = ρπ(a² − b²)²/8`, with semi-chord
//!   `a = c/2` and thickness `b → 0` (Newman 1977, *Marine Hydrodynamics*;
//!   Sedov 1965, the source Sane & Dickinson cite for eq. 3). It is supplied to the
//!   solver as added inertia, so it reacts to the plate's acceleration
//!   inside the articulated-body solve and its velocity-product terms come
//!   from the solver's bias force. For rotation about the quarter chord this
//!   reproduces the `π/4` and `π/16` coefficients of Sane & Dickinson 2002
//!   eq. 3.
//!   The solver forms those velocity-product terms from the link's
//!   inertial velocity, which is exact in still air; under wind the
//!   added-mass part ignores the air's own motion.
//!
//! The circulatory forces act at a fixed chord fraction, the quarter chord
//! of thin-airfoil theory by default; any migration of the centre of
//! pressure with incidence is not modelled.
//!
//! Left out, because no closed form with sourced coefficients exists for
//! them: wake capture (the fourth term of Sane & Dickinson eq. 2), the
//! reduction of translational force by the wing's own induced downwash
//! (about 10 %, Birch & Dickinson 2003, as cited by Fry et al. 2005), spanwise flow, and any Reynolds
//! dependence of the coefficients. The coefficients were measured at
//! Re ≈ 10²; revolving-wing measurements show them changing only modestly
//! up to Re of order 10⁴ (Lentink & Dickinson 2009, J. Exp. Biol.
//! 212:2705), which bounds where the model is meant to be used.
//! [`FlappingWingLoads::mean_reynolds`] reports the operating point.
//!
//! The coefficients are stroke means normalised by the speed at each
//! radius (Sane & Dickinson 2001, J. Exp. Biol. 204:2607, eq. 3: the
//! blade-element integral over the wing's second moment of area), so they
//! apply element by element with the element's own speed, as here.
//!
//! Known over-prediction for idealised kinematics: with a horizontal stroke
//! plane, no stroke deviation and about 45° incidence through mid-stroke,
//! the model gives a mean lift coefficient near 1.7, while freely hovering
//! *Drosophila* need 1.36 ± 0.21 (Fry, Sayaman & Dickinson 2005, J. Exp.
//! Biol. 208:2303, Table 1), so such a wing carries more than the weight.
//! The likely source is the idealisation, not the coefficients: a robotic
//! wing flapping in a horizontal plane without deviation at 45–50°
//! incidence measures 1.39–1.90 (Sane & Dickinson 2001, Figs 3 and 6), and
//! ±25° of stroke deviation, as real flies have, lowers it by 14–32 %
//! (ibid., Fig. 8). [`FlappingWingLoads`] reports lift, drag and
//! rotational force separately so a host can see which term carries the
//! load.

use alloc::vec::Vec;
use core::f64::consts::PI;

use glam::{DMat3, DVec3};
use mbd_core::MbdError;
use mbd_multibody::SpatialMat6;

use crate::flow::{AeroWrench, LinkFlow};
use crate::validation::{check_finite_vec, check_positive, check_unit_vec, invalid};

#[cfg(test)]
mod tests;

/// Geometry of a flapping wing in the axes of the link that carries it.
#[derive(Debug, Clone, PartialEq)]
pub struct FlappingWingParams {
    /// Root end of the wing's rotation axis, link axes, m.
    pub root_link_m: DVec3,
    /// Unit vector along the rotation axis from root to tip.
    pub span_axis_link: DVec3,
    /// Unit vector in the wing plane, perpendicular to the span, from the
    /// trailing edge toward the leading edge.
    pub chord_axis_link: DVec3,
    /// Root-to-tip length `R`, m.
    pub length_m: f64,
    /// Chord of each blade element, root to tip, m. The elements divide
    /// the span into equal widths; each chord applies at its element's
    /// mid-span.
    pub chords_m: Vec<f64>,
    /// Chord fraction `x̂₀` of the rotation axis behind the leading edge,
    /// `0..=1`.
    pub rotation_axis_chord_fraction: f64,
    /// Chord fraction behind the leading edge where the translational and
    /// rotational forces act, `0..=1`; `0.25` is the thin-airfoil centre.
    pub center_of_pressure_chord_fraction: f64,
}

/// One blade element, prepared once.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Strip {
    /// Point on the rotation axis at mid-span, link axes, m.
    axis_point: DVec3,
    chord: f64,
    width: f64,
}

/// What a flapping wing applies at one instant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlappingWingLoads {
    /// Translational and rotational forces about the link origin.
    pub wrench: AeroWrench,
    /// Translational force alone, link axes, N: the sum of
    /// `translational_lift_link_n` and `translational_drag_link_n`.
    pub translational_force_link_n: DVec3,
    /// Lift part of the translational force, perpendicular to each
    /// element's motion through the air, link axes, N.
    pub translational_lift_link_n: DVec3,
    /// Drag part of the translational force, against each element's motion
    /// through the air, link axes, N.
    pub translational_drag_link_n: DVec3,
    /// Rotational force alone, link axes, N.
    pub rotational_force_link_n: DVec3,
    /// Added inertia of the entrained air about the link origin, link axes.
    pub added_inertia: SpatialMat6,
    /// Power the translational and rotational forces take from the wing,
    /// W: `−Σ F·v` at each force's point of action.
    pub aerodynamic_power_w: f64,
    /// Area-weighted mean of the elements' chord Reynolds numbers.
    pub mean_reynolds: f64,
}

/// Quasi-steady flapping-wing element.
#[derive(Debug, Clone, PartialEq)]
pub struct FlappingWingAero {
    params: FlappingWingParams,
    normal: DVec3,
    strips: Vec<Strip>,
    area_m2: f64,
    /// Added inertia for unit fluid density.
    added_inertia_per_density: SpatialMat6,
}

impl FlappingWingAero {
    /// Validates the geometry and prepares the blade elements.
    pub fn new(params: FlappingWingParams) -> Result<Self, MbdError> {
        check_finite_vec("root_link_m", params.root_link_m)?;
        let span = check_unit_vec("span_axis_link", params.span_axis_link)?;
        let chord = check_unit_vec("chord_axis_link", params.chord_axis_link)?;
        if span.dot(chord).abs() > 1e-6 {
            return Err(invalid("chord_axis_link", span.dot(chord)));
        }
        check_positive("length_m", params.length_m)?;
        if params.chords_m.is_empty() {
            return Err(invalid("chords_m", 0.0));
        }
        for &c in &params.chords_m {
            check_positive("chords_m", c)?;
        }
        for (field, value) in [
            (
                "rotation_axis_chord_fraction",
                params.rotation_axis_chord_fraction,
            ),
            (
                "center_of_pressure_chord_fraction",
                params.center_of_pressure_chord_fraction,
            ),
        ] {
            if !value.is_finite() || !(0.0..=1.0).contains(&value) {
                return Err(invalid(field, value));
            }
        }
        let normal = chord.cross(span);
        let width = params.length_m / params.chords_m.len() as f64;
        let strips: Vec<Strip> = params
            .chords_m
            .iter()
            .enumerate()
            .map(|(k, &c)| Strip {
                axis_point: params.root_link_m + span * ((k as f64 + 0.5) * width),
                chord: c,
                width,
            })
            .collect();
        let area_m2 = strips.iter().map(|s| s.chord * s.width).sum();
        let x0 = params.rotation_axis_chord_fraction;
        let added_inertia_per_density = strips.iter().fold(SpatialMat6::ZERO, |acc, s| {
            let mid_chord = s.axis_point + chord * ((x0 - 0.5) * s.chord);
            acc + plate_added_inertia(s, mid_chord, normal, span)
        });
        Ok(Self {
            params: FlappingWingParams {
                span_axis_link: span,
                chord_axis_link: chord,
                ..params
            },
            normal,
            strips,
            area_m2,
            added_inertia_per_density,
        })
    }

    /// Validated geometry.
    pub const fn params(&self) -> &FlappingWingParams {
        &self.params
    }

    /// Planform area, m².
    pub const fn area_m2(&self) -> f64 {
        self.area_m2
    }

    /// Unit plate normal, `chord × span`, link axes.
    pub const fn normal_link(&self) -> DVec3 {
        self.normal
    }

    /// Loads for the flow at the link.
    pub fn evaluate(&self, flow: &LinkFlow<'_>) -> Result<FlappingWingLoads, MbdError> {
        flow.validate()?;
        let rho = flow.ambient.density_kg_m3;
        let mu = flow.ambient.dynamic_viscosity_pa_s;
        let mut acc = Accum::default();
        for strip in &self.strips {
            self.strip_loads(strip, flow, &mut acc);
        }
        Ok(FlappingWingLoads {
            wrench: AeroWrench {
                reference_point_link_m: DVec3::ZERO,
                force_link_n: (acc.lift + acc.drag + acc.rotational) * rho,
                moment_ref_link_nm: acc.moment * rho,
                stagnation_heat_flux_w_m2: 0.0,
            },
            translational_force_link_n: (acc.lift + acc.drag) * rho,
            translational_lift_link_n: acc.lift * rho,
            translational_drag_link_n: acc.drag * rho,
            rotational_force_link_n: acc.rotational * rho,
            added_inertia: self.added_inertia_per_density * rho,
            aerodynamic_power_w: acc.power * rho,
            mean_reynolds: if mu > 0.0 {
                rho * acc.speed_chord_area / (mu * self.area_m2)
            } else {
                f64::INFINITY
            },
        })
    }

    /// Adds one element's forces per unit density into `acc`.
    fn strip_loads(&self, s: &Strip, flow: &LinkFlow<'_>, acc: &mut Accum) {
        let (span, chord, n) = (
            self.params.span_axis_link,
            self.params.chord_axis_link,
            self.normal,
        );
        let w = flow.angular_rate_link_rad_s;
        let v_axis = flow.velocity_air_link_m_s + w.cross(s.axis_point);
        let u = v_axis - span * v_axis.dot(span);
        let speed = u.length();
        acc.speed_chord_area += speed * s.chord * s.chord * s.width;
        if speed == 0.0 {
            return;
        }
        let x0 = self.params.rotation_axis_chord_fraction;
        let cp =
            s.axis_point + chord * ((x0 - self.params.center_of_pressure_chord_fraction) * s.chord);
        let dir = u / speed;
        let u_n = u.dot(n);
        let alpha_deg = libm::atan2(u_n.abs(), u.dot(chord).abs()).to_degrees();
        let (cl, cd) = translational_coefficients(alpha_deg);
        let q_area = 0.5 * speed * speed * s.chord * s.width;
        let f_lift = lift_direction(dir, span, n, u_n) * (cl * q_area);
        let f_drag = -dir * (cd * q_area);
        let f_trans = f_lift + f_drag;
        // Kutta condition at the three-quarter chord: the circulation is
        // π c times the plate's own normal velocity there from rotation,
        // and the force opposes that motion.
        let to_three_quarter = chord * ((x0 - 0.75) * s.chord);
        let rotation_normal_speed = w.cross(to_three_quarter).dot(n);
        let f_rot = -n * (PI * speed * s.chord * rotation_normal_speed * s.width);
        let v_cp = flow.velocity_air_link_m_s + w.cross(cp);
        acc.lift += f_lift;
        acc.drag += f_drag;
        acc.rotational += f_rot;
        acc.moment += cp.cross(f_trans + f_rot);
        acc.power -= (f_trans + f_rot).dot(v_cp);
    }
}

/// Per-unit-density sums over the blade elements.
#[derive(Debug, Default)]
struct Accum {
    lift: DVec3,
    drag: DVec3,
    rotational: DVec3,
    moment: DVec3,
    power: f64,
    speed_chord_area: f64,
}

/// Dickinson, Lehmann & Sane (1999) fits, α in degrees.
pub fn translational_coefficients(alpha_deg: f64) -> (f64, f64) {
    let cl = 0.225 + 1.58 * libm::sin((2.13 * alpha_deg - 7.20).to_radians());
    let cd = 1.92 - 1.55 * libm::cos((2.04 * alpha_deg - 9.82).to_radians());
    (cl, cd)
}

/// Unit lift direction: perpendicular to the plate's motion `dir` in the
/// chord–normal plane, on the side the plate's normal motion `u_n` pushes
/// the air away from. Zero when the plate moves edgewise or broadside,
/// where the side is undefined.
fn lift_direction(dir: DVec3, span: DVec3, n: DVec3, u_n: f64) -> DVec3 {
    let perpendicular = span.cross(dir);
    let side = perpendicular.dot(n) * u_n;
    if side > 0.0 {
        -perpendicular
    } else if side < 0.0 {
        perpendicular
    } else {
        DVec3::ZERO
    }
}

/// Added inertia of one element for unit density about the link origin:
/// mass `πc²/4·w` along the normal at the mid-chord point and rotational
/// inertia `πc⁴/128·w` about the span through it.
fn plate_added_inertia(s: &Strip, mid_chord: DVec3, n: DVec3, span: DVec3) -> SpatialMat6 {
    let mass = PI * s.chord * s.chord / 4.0 * s.width;
    let c2 = s.chord * s.chord;
    let spin = PI * c2 * c2 / 128.0 * s.width;
    let m_t = outer(n, n) * mass;
    let p = skew(mid_chord);
    let al = p * m_t;
    SpatialMat6 {
        aa: outer(span, span) * spin - p * m_t * p,
        al,
        la: al.transpose(),
        ll: m_t,
    }
}

fn outer(a: DVec3, b: DVec3) -> DMat3 {
    DMat3::from_cols(a * b.x, a * b.y, a * b.z)
}

fn skew(v: DVec3) -> DMat3 {
    DMat3::from_cols(
        DVec3::new(0.0, v.z, -v.y),
        DVec3::new(-v.z, 0.0, v.x),
        DVec3::new(v.y, -v.x, 0.0),
    )
}
