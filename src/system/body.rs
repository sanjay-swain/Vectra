use glam::DMat3;

use crate::system::{
    interactions::{Force, Torque},
    state::{State, StateDerivative},
};

pub struct Body {
    pub id: usize,

    pub mass: f64,
    pub mass_inv: f64,

    pub inertia: DMat3,
    pub inertia_inv: DMat3,

    pub state: State,
    pub state_derivative: StateDerivative,

    pub forces: Vec<Force>,
    pub torques: Vec<Torque>,

    pub is_static: bool,
}

impl Body {
    pub fn apply_force(&mut self, force: Force) {
        self.forces.push(force);
    }
    pub fn apply_torque(&mut self, torque: Torque) {
        self.torques.push(torque);
    }

    pub fn clear_forces(&mut self) {
        self.forces = vec![];
    }
    pub fn clear_torques(&mut self) {
        self.torques = vec![];
    }

    pub fn inertia_inv_world(&self) -> DMat3 {
        let r = DMat3::from_quat(self.state.orientation);
        (r * self.inertia_inv) * r.transpose()
    }

    pub fn kinetic_energy(&self) -> f64 {
        return 0.5 * self.mass * self.state.velocity.length() * self.state.velocity.length()
            + 0.5
                * (self
                    .state
                    .angular_velocity
                    .dot(self.inertia * self.state.angular_velocity));
    }
}

impl Default for Body {
    fn default() -> Self {
        Self {
            id: 0,
            mass: 1.0,
            mass_inv: 1.0,
            inertia: DMat3::IDENTITY,
            inertia_inv: DMat3::IDENTITY,
            state: State::ZERO,
            state_derivative: StateDerivative::ZERO,
            forces: vec![],
            torques: vec![],
            is_static: false,
        }
    }
}
