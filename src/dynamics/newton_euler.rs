use crate::{
    dynamics::forces::compute_resultant,
    system::{body::Body, state::StateDerivative},
};

pub fn newton_euler(body: &Body) -> StateDerivative {
    // First we need to calculate the Resultant Forces and Torques.
    let (resultant_force, resultant_torque) = compute_resultant(body);

    let x_dot = StateDerivative {
        velocity: body.state.velocity,
        // The resultant force we get after computation is in local frame,
        // we need to convert it into global frame of reference
        acceleration: resultant_force.to_global(body.state.orientation) / body.mass,
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
