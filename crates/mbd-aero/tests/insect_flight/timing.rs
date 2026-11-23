//! Cost of flying the fly: RK4 at a fixed 20 kHz step, the whole model
//! (two flapping wings, servo, gravity) per step. Run in a release build:
//! `cargo test -p mbd-aero --release --test insect_flight -- --ignored`.

use std::io::Write;
use std::time::Instant;

use mbd_multibody::{FloatingBaseState, Rk4, Workspace};

use super::fly::{Fly, FlySpec, FlyWorld};
use super::probe;

#[test]
#[ignore = "timing; meaningful only in a release build"]
fn wall_clock_per_simulated_second_at_20_khz() {
    let spec = FlySpec::drosophila();
    let fly = Fly::new(spec);
    let model = probe::model(&fly);
    let elements = probe::elements(&fly, true, Vec::new());
    let world = FlyWorld::new(&spec);
    let base = FloatingBaseState::AT_REST;
    let mut state = probe::state_at(&fly, &model, &elements, &world, &base, 0.0);
    let mut work = Workspace::new();
    let mut rk4 = Rk4::new();
    let h = 1.0 / 20_000.0;
    let steps = 10_000_u32;
    let start = Instant::now();
    for k in 0..steps {
        model
            .advance(
                &mut rk4,
                &elements,
                &world,
                &mut state,
                f64::from(k) * h,
                h,
                &mut work,
            )
            .unwrap();
    }
    let wall = start.elapsed().as_secs_f64();
    let simulated = f64::from(steps) * h;
    writeln!(
        std::io::stderr(),
        "timing: {:.3} s wall per simulated second ({:.2} µs per step, {} steps)",
        wall / simulated,
        wall / f64::from(steps) * 1e6,
        steps,
    )
    .ok();
    assert!(state.base().position_world_m.is_finite());
}
