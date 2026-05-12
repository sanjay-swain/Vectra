use glam::{DMat3, DQuat, DVec3};

use crate::system::{
    constraints::joints::{Jacobian, Joint},
    state::State,
};

pub struct FixedJoint {
    joint_frame_a: DQuat,
    joint_frame_b: DQuat,
}

impl FixedJoint {
    pub fn new(joint_frame_a: DQuat, joint_frame_b: DQuat) -> Self {
        Self {
            joint_frame_a,
            joint_frame_b,
        }
    }
}

impl Joint for FixedJoint {
    fn restricted_dof(&self) -> usize {
        return 5;
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

        let x_a = state_a.orientation * (self.joint_frame_a * DVec3::X);
        let y_a = state_a.orientation * (self.joint_frame_a * DVec3::Y);

        let y_b = state_b.orientation * (self.joint_frame_b * DVec3::Y);
        let z_b = state_b.orientation * (self.joint_frame_b * DVec3::Z);

        let x_cross_z = x_a.cross(z_b);
        let y_cross_z = y_a.cross(z_b);

        let x_cross_y = x_a.cross(y_b);

        jacobian.j[0] = [
            //    v_a     |       w_a         |      v_b     |       w_b         |
            -1.0, 0.0, 0.0, 0.0, -r_a.z, r_a.y, 1.0, 0.0, 0.0, 0.0, r_b.z, -r_b.y,
        ];

        jacobian.j[1] = [
            //    v_a     |       w_a         |      v_b     |       w_b         |
            0.0, -1.0, 0.0, r_a.z, 0.0, -r_a.x, 0.0, 1.0, 0.0, -r_b.z, 0.0, r_b.x,
        ];

        jacobian.j[2] = [
            //    v_a     |       w_a         |      v_b     |       w_b         |
            0.0, 0.0, -1.0, -r_a.y, r_a.x, 0.0, 0.0, 0.0, 1.0, r_b.y, -r_b.x, 0.0,
        ];

        jacobian.j[3] = [
            //      v_a
            0.0,
            0.0,
            0.0,
            //      w_a
            x_cross_z.x,
            x_cross_z.y,
            x_cross_z.z,
            //      v_b
            0.0,
            0.0,
            0.0,
            //      w_b
            -x_cross_z.x,
            -x_cross_z.y,
            -x_cross_z.z,
        ];

        jacobian.j[4] = [
            //      v_a
            0.0,
            0.0,
            0.0,
            //      w_a
            y_cross_z.x,
            y_cross_z.y,
            y_cross_z.z,
            //      v_b
            0.0,
            0.0,
            0.0,
            //      w_b
            -y_cross_z.x,
            -y_cross_z.y,
            -y_cross_z.z,
        ];

        jacobian.j[5] = [
            //      v_a
            0.0,
            0.0,
            0.0,
            //      w_a
            x_cross_y.x,
            x_cross_y.y,
            x_cross_y.z,
            //      v_b
            0.0,
            0.0,
            0.0,
            //      w_b
            -x_cross_y.x,
            -x_cross_y.y,
            -x_cross_y.z,
        ];
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

        // The angular velocity within the state property of the body is with respect to local coordinates
        // First we need to convert them to global coordinates
        let w_a = state_a.orientation * state_a.angular_velocity;
        let w_b = state_b.orientation * state_b.angular_velocity;

        let x_a = state_a.orientation * (self.joint_frame_a * DVec3::X);
        let y_a = state_a.orientation * (self.joint_frame_a * DVec3::Y);

        let y_b = state_b.orientation * (self.joint_frame_b * DVec3::Y);
        let z_b = state_b.orientation * (self.joint_frame_b * DVec3::Z);

        let x_a_dot = w_a.cross(x_a);
        let z_b_dot = w_b.cross(z_b);
        let y_a_dot = w_a.cross(y_a);

        let y_b_dot = w_b.cross(y_b);

        let omega_b = DMat3 {
            x_axis: DVec3::new(0.0, -w_b.z, w_b.y),
            y_axis: DVec3::new(w_b.z, 0.0, -w_b.x),
            z_axis: DVec3::new(-w_b.y, w_b.x, 0.0),
        };

        let omega_a = DMat3 {
            x_axis: DVec3::new(0.0, -w_a.z, w_a.y),
            y_axis: DVec3::new(w_a.z, 0.0, -w_a.x),
            z_axis: DVec3::new(-w_a.y, w_a.x, 0.0),
        };

        let r_a_mat = DMat3 {
            x_axis: DVec3::new(0.0, -r_a.z, r_a.y),
            y_axis: DVec3::new(r_a.z, 0.0, -r_a.x),
            z_axis: DVec3::new(-r_a.y, r_a.x, 0.0),
        };

        let r_b_mat = DMat3 {
            x_axis: DVec3::new(0.0, -r_b.z, r_b.y),
            y_axis: DVec3::new(r_b.z, 0.0, -r_b.x),
            z_axis: DVec3::new(-r_b.y, r_b.x, 0.0),
        };

        let res =
            (omega_a.mul_mat3(&r_a_mat)).mul_vec3(w_a) - (omega_b.mul_mat3(&r_b_mat)).mul_vec3(w_b);

        velocity_bias[0] = res.x;
        velocity_bias[1] = res.y;
        velocity_bias[2] = res.z;

        velocity_bias[3] = w_a.cross(x_a_dot).dot(z_b)
            + 2.0 * (x_a_dot.dot(z_b_dot))
            + x_a.dot(w_b.cross(z_b_dot));
        velocity_bias[4] = w_a.cross(y_a_dot).dot(z_b)
            + 2.0 * (y_a_dot.dot(z_b_dot))
            + y_a.dot(w_b.cross(z_b_dot));
        velocity_bias[5] = w_a.cross(x_a_dot).dot(y_b)
            + 2.0 * (x_a_dot.dot(y_b_dot))
            + x_a.dot(w_b.cross(y_b_dot));
    }

    fn calculate_joint_error(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
    ) -> [f64; 6] {
        let e = state_b.position + state_b.orientation * anchor_b
            - (state_a.position + state_a.orientation * anchor_a);

        [
            e.x,
            e.y,
            e.z,
            (state_a.orientation * (self.joint_frame_a * DVec3::X))
                .dot(state_b.orientation * (self.joint_frame_b * DVec3::Z)),
            (state_a.orientation * (self.joint_frame_a * DVec3::Y))
                .dot(state_b.orientation * (self.joint_frame_b * DVec3::Z)),
            (state_a.orientation * (self.joint_frame_a * DVec3::X))
                .dot(state_b.orientation * (self.joint_frame_b * DVec3::Y)),
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

        let e = state_b.velocity + w_b.cross(state_b.orientation * anchor_b)
            - (state_a.velocity + w_a.cross(state_a.orientation * anchor_a));

        let x_a = state_a.orientation * (self.joint_frame_a * DVec3::X);
        let y_a = state_a.orientation * (self.joint_frame_a * DVec3::Y);

        let y_b = state_b.orientation * (self.joint_frame_b * DVec3::Y);
        let z_b = state_b.orientation * (self.joint_frame_b * DVec3::Z);

        let x_a_dot = w_a.cross(x_a);
        let z_b_dot = w_b.cross(z_b);
        let y_a_dot = w_a.cross(y_a);

        let y_b_dot = w_b.cross(y_b);

        [
            e.x,
            e.y,
            e.z,
            (x_a_dot.dot(z_b) + x_a.dot(z_b_dot)),
            (y_a_dot.dot(z_b) + y_a.dot(z_b_dot)),
            (x_a_dot.dot(y_b) + x_a.dot(y_b_dot)),
        ]
    }
}
