use glam::DVec3;

use crate::system::{
    body::Body,
    interactions::{Force, Torque},
};

pub fn compute_forces(body: &Body) -> Force {
    let mut resultant_force: Force = Force {
        force: DVec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        position: DVec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    };

    for force in &body.forces {
        resultant_force.force += force.force;
    }

    return resultant_force;
}

pub fn compute_torqes(body: &Body) -> Torque {
    let mut resultant_torque: Torque = Torque {
        torque: DVec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    };

    for torque in &body.torques {
        resultant_torque.torque += torque.torque;
    }

    for force in &body.forces {
        resultant_torque.torque += force.position.cross(force.force);
    }

    return resultant_torque;
}
