use glam::DMat3;

use crate::system::body::{Body, State};

pub struct World {
    pub bodies: Vec<Body>,
    pub enable_gravity: bool,
    pub gravity: f64,
    next_id: u32,
}

impl World {
    pub fn create_body(&mut self, mass: f64, inertia: DMat3, initial_state: State) {
        self.bodies.push(Body {
            id: self.next_id,
            mass: mass,
            inertia: inertia,
            state: initial_state,
            forces: vec![],
            torques: vec![],
        });
        self.next_id += 1;
    }
}
