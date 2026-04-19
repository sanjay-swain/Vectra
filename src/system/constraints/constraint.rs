use glam::DVec3;

use crate::system::constraints::joints::{Jacobian, Joint};

pub struct Constraint {
    pub body_a_index: usize,
    pub body_b_index: usize,

    pub body_a_anchor: DVec3,
    pub body_b_anchor: DVec3,

    pub joint: Box<dyn Joint>,

    pub jacobian: Jacobian,
    pub velocity_bias: [f64; 6],

    pub constraint_forces: [f64; 12],

    pub lagrange_multiplier: [f64; 6],
}

impl Constraint {
    pub fn new(
        body_a_index: usize,
        body_b_index: usize,
        body_a_anchor: DVec3,
        body_b_anchor: DVec3,
        joint: Box<dyn Joint>,
    ) -> Self {
        Self {
            body_a_index,
            body_b_index,
            body_a_anchor,
            body_b_anchor,
            joint,
            jacobian: Jacobian::ZERO,
            velocity_bias: [0.0; 6],

            constraint_forces: [0.0; 12],

            lagrange_multiplier: [0.0; 6],
        }
    }
}
