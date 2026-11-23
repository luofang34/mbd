//! A Lagrange-multiplier constraint written as a force element against the
//! public interface only: a point of a free rigid body is pinned by solving
//! `J M⁻¹ Jᵀ λ = −(J v̇_free + J̇ v)` in the resolution phase. The pinned body
//! is a compound pendulum, whose motion is known in closed form.

#![cfg(test)]

use glam::{DMat3, DVec3};
use mbd_core::{ComponentId, ComponentSpec, ComponentState, ComponentTree, JointSpec, Placement};
use mbd_multibody::{
    ElementContext, FloatingBaseState, ForceElement, Loads, ModelState, MultibodyError,
    MultibodyModel, Resolution, Rk4, UniformGravity, Workspace,
};

const MASS_KG: f64 = 3.0;
/// Inertia about the centre of mass, the same on every axis.
const INERTIA_KG_M2: f64 = 0.5;
/// Distance from the pin to the centre of mass.
const ARM_M: f64 = 0.4;
const G: f64 = 9.81;
/// The pinned point, in body axes.
const PIN: DVec3 = DVec3::new(-ARM_M, 0.0, 0.0);

/// Holds the world acceleration of a body point at zero.
struct Pin {
    component: ComponentId,
    offset_local_m: DVec3,
}

impl<I: ?Sized> ForceElement<I> for Pin {
    fn has_resolution(&self) -> bool {
        true
    }

    fn evaluate(
        &self,
        _ctx: &ElementContext<'_, I>,
        _state: &[f64],
        _modes: &[f64],
        _loads: &mut Loads<'_>,
        _state_dot: &mut [f64],
    ) -> Result<(), MultibodyError> {
        Ok(())
    }

    fn resolve(
        &self,
        ctx: &ElementContext<'_, I>,
        _state: &[f64],
        _modes: &[f64],
        resolution: &mut Resolution<'_>,
        loads: &mut Loads<'_>,
    ) -> Result<(), MultibodyError> {
        let (body, at) = (self.component, self.offset_local_m);
        let nv = ctx.model.layout().velocity_len();
        let free = resolution.free_acceleration().to_vec();
        let a_free = resolution.point_acceleration(&free, body, at)?;
        // Delassus matrix, one column per unit multiplier.
        let (mut tau, mut response) = (vec![0.0; nv], vec![0.0; nv]);
        let mut columns = [DVec3::ZERO; 3];
        for (column, axis) in columns.iter_mut().zip([DVec3::X, DVec3::Y, DVec3::Z]) {
            resolution.point_force_to_generalized(body, at, axis, &mut tau)?;
            resolution.apply_inverse_mass(&tau, &mut response)?;
            *column = resolution.point_velocity(&response, body, at)?;
        }
        let lambda = DMat3::from_cols_array_2d(&columns.map(|c| c.to_array())).inverse() * -a_free;
        let point = ctx.kinematics.point(body, at)?;
        loads.add_force(body, at, point.to_link(lambda))
    }
}

type Elements = (UniformGravity, Pin);

fn pendulum() -> (MultibodyModel, ModelState, Elements) {
    let spec = ComponentSpec {
        name: "rod".to_owned(),
        placement: Placement::ROOT,
        dry_mass_kg: MASS_KG,
        dry_inertia_body: DMat3::from_diagonal(DVec3::splat(INERTIA_KG_M2)),
        thermal_capacity_j_k: 1.0,
        nominal_power_w: 0.0,
        joint: JointSpec::FIXED,
    };
    let tree = ComponentTree::new([(spec, ComponentState::at_temperature(293.15))]).unwrap();
    let model = MultibodyModel::new(&tree, &[]).unwrap();
    let pin = Pin {
        component: ComponentId(0),
        offset_local_m: PIN,
    };
    let elements = (UniformGravity(DVec3::new(0.0, 0.0, -G)), pin);
    // Released at rest with the arm horizontal; the pin sits at the origin.
    let base = FloatingBaseState {
        position_world_m: -PIN,
        ..FloatingBaseState::AT_REST
    };
    let state = model.state(&tree, &base, &elements, &()).unwrap();
    (model, state, elements)
}

/// Kinetic plus potential energy of the rod.
fn energy(state: &ModelState) -> f64 {
    let base = state.base();
    0.5 * MASS_KG * base.linear_vel_root_m_s.length_squared()
        + 0.5 * INERTIA_KG_M2 * base.angular_vel_root_rad_s.length_squared()
        + MASS_KG * G * base.position_world_m.z
}

#[test]
fn pinned_rod_accelerates_as_a_compound_pendulum() {
    let (model, state, elements) = pendulum();
    let mut work = Workspace::new();
    let mut rates = vec![0.0; state.as_slice().len()];
    model
        .derivative(
            0.0,
            state.as_slice(),
            state.modes(),
            &elements,
            &(),
            &mut work,
            &mut rates,
        )
        .unwrap();
    let accel = work.base_accel();
    // About the pin: weight m g at arm d against inertia I + m d^2.
    let alpha = MASS_KG * G * ARM_M / (INERTIA_KG_M2 + MASS_KG * ARM_M * ARM_M);
    assert!(
        (accel.ang - DVec3::new(0.0, alpha, 0.0)).length() < 1e-12,
        "{:?}",
        accel.ang
    );
    assert!((accel.lin - DVec3::new(0.0, 0.0, -alpha * ARM_M)).length() < 1e-12);
    // The constrained point does not accelerate.
    let v_dot = &rates[model.layout().position_len()..];
    let pinned = model
        .point_acceleration(state.as_slice(), v_dot, ComponentId(0), PIN, &mut work)
        .unwrap();
    assert!(pinned.length() < 1e-12 * G, "pin acceleration {pinned:?}");
}

#[test]
fn pinned_rod_swings_about_its_pin_and_conserves_energy() {
    let (model, mut state, elements) = pendulum();
    let mut work = Workspace::new();
    let mut rk4 = Rk4::new();
    let e0 = energy(&state);
    let (h, steps) = (1.0e-3, 600);
    let mut lowest = f64::INFINITY;
    for i in 0..steps {
        model
            .advance(
                &mut rk4,
                &elements,
                &(),
                &mut state,
                f64::from(i) * h,
                h,
                &mut work,
            )
            .unwrap();
        lowest = lowest.min(state.base().position_world_m.z);
    }
    // An acceleration-level constraint drifts only by truncation: the pin
    // stays put to a millionth of the arm over most of a swing.
    let kinematics = model.kinematics(&state, &mut work).unwrap();
    let pin = kinematics
        .point(ComponentId(0), PIN)
        .unwrap()
        .position_world_m;
    assert!(pin.length() < 1.0e-6 * ARM_M, "pin drifted to {pin:?}");
    assert!((energy(&state) - e0).abs() < 1.0e-8 * MASS_KG * G * ARM_M);
    // The rod passed through the bottom of its swing.
    assert!(lowest < -0.99 * ARM_M, "lowest centre of mass {lowest}");
}
