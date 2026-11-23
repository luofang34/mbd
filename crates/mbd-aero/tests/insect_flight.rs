//! Insect-scale proof of the per-link aerodynamic elements: a fruit-fly-like
//! body with flapping wings on stroke and pitch hinges, hovering, flying
//! free, reacting to its own wing inertia, landing, and scaled up a
//! hundredfold.

#![cfg(test)]

/// Modules live in `insect_flight/`, beside this file.
mod insect_flight {
    mod fly;
    mod free_flight;
    mod hover;
    mod inertial;
    mod landing;
    mod probe;
    mod scale;
    mod timing;
}
