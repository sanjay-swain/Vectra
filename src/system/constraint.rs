use glam::DVec3;

use crate::system::joints::{JacobianRow, Joint};

pub struct Constraint<JointType: Joint> {
    pub body_a_index: usize,
    pub body_a_id: usize,
    pub body_b_index: usize,
    pub body_b_id: usize,

    pub body_a_anchor: DVec3,
    pub body_b_anchor: DVec3,

    pub joint: JointType,

    pub jacobian: Vec<JacobianRow>,
    pub velocity_bias: Vec<f64>,
}
