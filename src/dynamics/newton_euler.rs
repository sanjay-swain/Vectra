use crate::{
    dynamics::forces::{compute_forces, compute_torqes},
    system::{body::Body, state::StateDerivative},
};

pub fn newton_euler(body: &Body, time: &f64) -> StateDerivative {
    // First we need to calculate the Resultant Forces and Torques.
    let resultant_force = compute_forces(body);
    let resultant_torque = compute_torqes(body);
    let x_dot = StateDerivative {
        velocity: body.state.velocity,
        acceleration: resultant_force.force / body.mass,
        angular_velocity: body.state.angular_velocity,
        angular_acceleration: body.inertia.inverse()
            * (resultant_torque.torque
                - body
                    .state
                    .angular_velocity
                    .cross(body.inertia * body.state.angular_velocity)),
    };

    x_dot
}
