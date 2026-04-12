use crate::{
    dynamics::forces::compute_resultant,
    system::{body::Body, state::StateDerivative},
};

pub fn newton_euler(body: &mut Body) {
    // First we need to calculate the Resultant Forces and Torques.
    let (resultant_force, resultant_torque) =
        compute_resultant(&body.forces, &body.torques, body.state.orientation);

    body.state_derivative = StateDerivative {
        velocity: body.state.velocity,
        // The resultant force we get after computation is in local frame,
        // we need to convert it into global frame of reference
        acceleration: resultant_force.to_global(body.state.orientation) * body.mass_inv,
        angular_velocity: body.state.angular_velocity,
        angular_acceleration: body.inertia_inv
            * (resultant_torque.torque
                - body
                    .state
                    .angular_velocity
                    .cross(body.inertia * body.state.angular_velocity)),
    };
}

#[cfg(test)]
mod tests {
    use glam::{DMat3, DVec3};

    use crate::system::interactions::{Force, Frame, Torque};

    use super::*;

    #[test]
    fn translation() {
        let mut body: Body = Body {
            id: 0,
            mass: 5.0,
            ..Default::default()
        };

        body.apply_force(Force::new(
            DVec3::new(50.0, 0.0, 0.0),
            DVec3::ZERO,
            Frame::Local,
        ));

        newton_euler(&mut body);

        assert!((body.state_derivative.acceleration.x - 10.0).abs() < 1e-6);
        assert!((body.state_derivative.acceleration.y).abs() < 1e-6);
        assert!((body.state_derivative.acceleration.z).abs() < 1e-6);

        assert!((body.state_derivative.angular_acceleration.x).abs() < 1e-6);
        assert!((body.state_derivative.angular_acceleration.y).abs() < 1e-6);
        assert!((body.state_derivative.angular_acceleration.z).abs() < 1e-6);
    }

    #[test]
    fn rotation() {
        let mut body: Body = Body {
            id: 0,
            mass: 1.0,
            inertia: DMat3::from_diagonal(DVec3 {
                x: 10.0,
                y: 20.0,
                z: 30.0,
            }),
            ..Default::default()
        };

        body.apply_torque(Torque::new(DVec3::new(0.0, 20.0, 0.0), Frame::Local));

        newton_euler(&mut body);

        assert!((body.state_derivative.acceleration.x).abs() < 1e-6);
        assert!((body.state_derivative.acceleration.y).abs() < 1e-6);
        assert!((body.state_derivative.acceleration.z).abs() < 1e-6);

        assert!((body.state_derivative.angular_acceleration.x).abs() < 1e-6);
        assert!((body.state_derivative.angular_acceleration.y - 1.0).abs() < 1e-6);
        assert!((body.state_derivative.angular_acceleration.z).abs() < 1e-6);
    }

    #[test]
    fn gyroscope() {
        let mut body: Body = Body {
            id: 0,
            mass: 1.0,
            inertia: DMat3::from_diagonal(DVec3 {
                x: 10.0,
                y: 20.0,
                z: 30.0,
            }),
            ..Default::default()
        };

        body.state.angular_velocity = DVec3 {
            x: 1.0,
            y: 0.0,
            z: 1.0,
        };

        newton_euler(&mut body);

        assert!((body.state_derivative.angular_acceleration.x).abs() < 1e-6);
        assert!((body.state_derivative.angular_acceleration.y - 1.0).abs() < 1e-6);
        assert!((body.state_derivative.angular_acceleration.z).abs() < 1e-6);
    }
}
