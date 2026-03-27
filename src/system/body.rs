use glam::DMat3;

use crate::system::{
    interactions::{Force, Torque},
    state::State,
};

pub struct Body {
    pub id: u32,
    pub mass: f64,
    pub inertia: DMat3,
    pub state: State,
    pub forces: Vec<Force>,
    pub torques: Vec<Torque>,
}
