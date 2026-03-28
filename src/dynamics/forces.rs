use glam::DVec3;

use crate::system::{
    body::Body,
    interactions::{Force, Torque},
};

pub fn compute_resultant(body: &Body) -> (Force, Torque) {
    let mut resultant_force = Force::ZERO_LOCAL;
    let mut resultant_torque = Torque::ZERO_LOCAL;

    let mut tmp: DVec3;

    for force in &body.forces {
        tmp = force.to_local(body.state.orientation);
        resultant_force.force += tmp;
        resultant_torque.torque += force.position.cross(tmp);
    }

    for torque in &body.torques {
        resultant_torque.torque += torque.to_local(body.state.orientation);
    }

    return (resultant_force, resultant_torque);
}
