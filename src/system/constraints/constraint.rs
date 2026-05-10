use glam::{DQuat, DVec3};

use crate::system::constraints::joints::{Jacobian, JointType};

pub struct Constraint {
    pub body_a_index: usize,
    pub body_b_index: usize,

    pub anchor_a: DVec3,
    pub anchor_b: DVec3,

    pub joint_frame_a: DQuat,
    pub joint_frame_b: DQuat,

    pub joint: JointType,

    pub jacobian: Jacobian,
    pub velocity_bias: [f64; 6],

    pub constraint_forces: [f64; 12],

    pub lagrange_multiplier: [f64; 6],
}

impl Constraint {
    pub fn new(
        body_a_index: usize,
        body_b_index: usize,
        anchor_a: DVec3,
        anchor_b: DVec3,
        joint_frame_a: DQuat,
        joint_frame_b: DQuat,
        joint: JointType,
    ) -> Self {
        Self {
            body_a_index,
            body_b_index,
            anchor_a,
            anchor_b,
            joint_frame_a,
            joint_frame_b,
            joint,
            jacobian: Jacobian::ZERO,
            velocity_bias: [0.0; 6],

            constraint_forces: [0.0; 12],

            lagrange_multiplier: [0.0; 6],
        }
    }
}
