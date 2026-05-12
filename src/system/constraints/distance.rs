use glam::DVec3;

use crate::system::{
    constraints::joints::{Jacobian, Joint},
    state::State,
};

pub struct DistanceJoint {
    length: f64,
}

impl DistanceJoint {
    pub fn new(length: f64) -> Self {
        Self { length }
    }
}

impl Joint for DistanceJoint {
    fn restricted_dof(&self) -> usize {
        return 1;
    }

    fn calculate_jacobian(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        jacobian: &mut Jacobian,
    ) {
        let r_a = state_a.orientation * anchor_a;
        let r_b = state_b.orientation * anchor_b;

        let d = (state_a.position + r_a) - (state_b.position + r_b);

        let r_a_d = r_a.cross(d);
        let r_b_d = r_b.cross(d);

        jacobian.j[0][0] = d.x;
        jacobian.j[0][1] = d.y;
        jacobian.j[0][2] = d.z;

        jacobian.j[0][3] = r_a_d.x;
        jacobian.j[0][4] = r_a_d.y;
        jacobian.j[0][5] = r_a_d.z;

        jacobian.j[0][6] = -d.x;
        jacobian.j[0][7] = -d.y;
        jacobian.j[0][8] = -d.z;

        jacobian.j[0][9] = -r_b_d.x;
        jacobian.j[0][10] = -r_b_d.y;
        jacobian.j[0][11] = -r_b_d.z;
    }

    fn calculate_velocity_bias(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        velocity_bias: &mut [f64; 6],
    ) {
        let r_a = state_a.orientation * anchor_a;
        let r_b = state_b.orientation * anchor_b;

        let d = (state_a.position + r_a) - (state_b.position + r_b);

        let r_a_d = r_a.cross(d);
        let r_b_d = r_b.cross(d);

        let w_a = state_a.orientation * state_a.angular_velocity;
        let w_b = state_b.orientation * state_b.angular_velocity;

        let d_dot = state_a.velocity + w_a.cross(r_a) - state_b.velocity - w_b.cross(r_b);

        velocity_bias[0] = d_dot.dot(state_a.velocity) + w_a.cross(r_a_d).dot(w_a)
            - d_dot.dot(state_b.velocity)
            - w_b.cross(r_b_d).dot(w_b);
    }

    fn calculate_joint_error(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
    ) -> [f64; 6] {
        [
            self.length
                - (state_a.position + state_a.orientation * anchor_a - state_b.position
                    + state_b.orientation * anchor_b)
                    .length(),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
    }

    fn calculate_joint_velocity_error(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
    ) -> [f64; 6] {
        let w_a = state_a.orientation * state_a.angular_velocity;
        let w_b = state_b.orientation * state_b.angular_velocity;
        [
            -(state_a.position + state_a.orientation * anchor_a - state_b.position
                + state_b.orientation * anchor_b)
                .dot(
                    state_b.velocity
                        + w_b.cross(state_b.orientation * anchor_b)
                        + (state_a.velocity + w_a.cross(state_a.orientation * anchor_a)),
                ),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
    }
}
