use glam::{DMat3, DVec3};

use crate::system::{
    constraints::joints::{Jacobian, Joint},
    state::State,
};

pub struct SphericalJoint {}

impl SphericalJoint {
    pub fn new() -> Self {
        Self {}
    }
}

impl Joint for SphericalJoint {
    fn restricted_dof(&self) -> usize {
        return 3;
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

        [e.x, e.y, e.z, 0.0, 0.0, 0.0]
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

        [e.x, e.y, e.z, 0.0, 0.0, 0.0]
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use glam::DQuat;

    use super::*;

    #[test]
    fn jacobian_check_basic() {
        let body_a_state = State::ZERO;
        let body_b_state = State {
            position: DVec3::new(0.0, 10.0, 0.0),
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
            velocity: DVec3::ZERO,
        };

        let anchor_a = DVec3::new(0.0, 4.0, 0.0);
        let anchor_b = DVec3::new(0.0, -6.0, 0.0);

        let mut jacobian = Jacobian::ZERO;

        let joint = SphericalJoint::new();

        joint.calculate_jacobian(
            &body_a_state,
            &body_b_state,
            anchor_a,
            anchor_b,
            &mut jacobian,
        );

        let mut expected = Jacobian::ZERO;
        expected.j[0][0] = -1.0;
        expected.j[0][6] = 1.0;
        expected.j[0][5] = 4.0;
        expected.j[0][11] = 6.0;

        expected.j[1][1] = -1.0;
        expected.j[1][7] = 1.0;

        expected.j[2][2] = -1.0;
        expected.j[2][8] = 1.0;
        expected.j[2][3] = -4.0;
        expected.j[2][9] = -6.0;

        for i in 0..6 {
            assert!((jacobian.j[i][0] - expected.j[i][0]).abs() < 1e-6);
            assert!((jacobian.j[i][1] - expected.j[i][1]).abs() < 1e-6);
            assert!((jacobian.j[i][2] - expected.j[i][2]).abs() < 1e-6);

            assert!((jacobian.j[i][3] - expected.j[i][3]).abs() < 1e-6);
            assert!((jacobian.j[i][4] - expected.j[i][4]).abs() < 1e-6);
            assert!((jacobian.j[i][5] - expected.j[i][5]).abs() < 1e-6);

            assert!((jacobian.j[i][6] - expected.j[i][6]).abs() < 1e-6);
            assert!((jacobian.j[i][7] - expected.j[i][7]).abs() < 1e-6);
            assert!((jacobian.j[i][8] - expected.j[i][8]).abs() < 1e-6);

            assert!((jacobian.j[i][9] - expected.j[i][9]).abs() < 1e-6);
            assert!((jacobian.j[i][10] - expected.j[i][10]).abs() < 1e-6);
            assert!((jacobian.j[i][11] - expected.j[i][11]).abs() < 1e-6);
        }
    }

    #[test]
    fn jacobian_test_inter() {
        let body_a_state = State {
            position: DVec3::new(1.0, 1.0, 1.0),
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
        };
        let body_b_state = State {
            position: DVec3::new(3.0, 4.0, 5.0),
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
            velocity: DVec3::ZERO,
        };

        let anchor_a = DVec3::new(1.0, 1.0, 1.0);
        let anchor_b = DVec3::new(-1.0, -2.0, -3.0);

        let mut jacobian = Jacobian::ZERO;

        let joint = SphericalJoint::new();

        joint.calculate_jacobian(
            &body_a_state,
            &body_b_state,
            anchor_a,
            anchor_b,
            &mut jacobian,
        );

        let mut expected = Jacobian::ZERO;
        expected.j = [
            [
                -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 1.0, 0.0, 0.0, 0.0, -3.0, 2.0,
            ],
            [
                0.0, -1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, 3.0, 0.0, -1.0,
            ],
            [
                0.0, 0.0, -1.0, -1.0, 1.0, 0.0, 0.0, 0.0, 1.0, -2.0, 1.0, 0.0,
            ],
            [0.0; 12],
            [0.0; 12],
            [0.0; 12],
        ];

        for i in 0..6 {
            assert!((jacobian.j[i][0] - expected.j[i][0]).abs() < 1e-6);
            assert!((jacobian.j[i][1] - expected.j[i][1]).abs() < 1e-6);
            assert!((jacobian.j[i][2] - expected.j[i][2]).abs() < 1e-6);

            assert!((jacobian.j[i][3] - expected.j[i][3]).abs() < 1e-6);
            assert!((jacobian.j[i][4] - expected.j[i][4]).abs() < 1e-6);
            assert!((jacobian.j[i][5] - expected.j[i][5]).abs() < 1e-6);

            assert!((jacobian.j[i][6] - expected.j[i][6]).abs() < 1e-6);
            assert!((jacobian.j[i][7] - expected.j[i][7]).abs() < 1e-6);
            assert!((jacobian.j[i][8] - expected.j[i][8]).abs() < 1e-6);

            assert!((jacobian.j[i][9] - expected.j[i][9]).abs() < 1e-6);
            assert!((jacobian.j[i][10] - expected.j[i][10]).abs() < 1e-6);
            assert!((jacobian.j[i][11] - expected.j[i][11]).abs() < 1e-6);
        }
    }

    #[test]
    fn velocity_bias_test() {
        let body_a_state = State::ZERO;
        let body_b_state = State {
            position: DVec3::ZERO,
            orientation: DQuat::from_axis_angle(DVec3::Z, PI / 2.0),
            angular_velocity: DVec3::new(0.0, 0.0, 3.0),
            velocity: DVec3::ZERO,
        };

        let anchor_a = DVec3::new(0.0, 2.0, 0.0);
        let anchor_b = DVec3::new(2.0, 0.0, 0.0);

        let joint = SphericalJoint::new();

        let mut vel_bias = [0.0; 6];
        let expected_bias = [0.0, -18.0, 0.0, 0.0, 0.0, 0.0];

        joint.calculate_velocity_bias(
            &body_a_state,
            &body_b_state,
            anchor_a,
            anchor_b,
            &mut vel_bias,
        );

        for i in 0..6 {
            assert!(
                (vel_bias[i] - expected_bias[i]).abs() < 1e-6,
                "{} expected {}",
                vel_bias[i],
                expected_bias[i]
            );
        }
    }
}
