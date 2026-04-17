use glam::DVec3;

use crate::system::{
    constraints::joints::{JacobianRow, Joint},
    state::State,
};

pub struct SphericalJoint {}

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
        jacobian: &mut [JacobianRow; 6],
    ) {
        let r_a = state_a.orientation.mul_vec3(anchor_a);
        let r_b = state_b.orientation.mul_vec3(anchor_b);

        jacobian[0] = JacobianRow {
            v_a: DVec3::new(-1.0, 0.0, 0.0),
            w_a: DVec3::new(0.0, -r_a.z, r_a.y),
            v_b: DVec3::new(1.0, 0.0, 0.0),
            w_b: DVec3::new(0.0, r_b.z, -r_b.y),
        };

        jacobian[1] = JacobianRow {
            v_a: DVec3::new(0.0, -1.0, 0.0),
            w_a: DVec3::new(r_a.z, 0.0, -r_a.x),
            v_b: DVec3::new(0.0, 1.0, 0.0),
            w_b: DVec3::new(-r_b.z, 0.0, r_b.x),
        };

        jacobian[2] = JacobianRow {
            v_a: DVec3::new(0.0, 0.0, -1.0),
            w_a: DVec3::new(-r_a.y, r_a.x, 0.0),
            v_b: DVec3::new(0.0, 0.0, 1.0),
            w_b: DVec3::new(r_b.y, -r_b.x, 0.0),
        };
    }

    fn calculate_velocity_bias(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        velocity_bias: &mut [f64; 6],
    ) {
        let r_a = state_a.orientation.mul_vec3(anchor_a);
        let r_b = state_b.orientation.mul_vec3(anchor_b);

        // The angular velocity within the state property of the body is with respect to local coordinates
        // First we need to convert them to global coordinates
        let w_a = state_a.orientation * state_a.angular_velocity;
        let w_b = state_b.orientation * state_b.angular_velocity;

        let dot_wr_a = w_a.x * r_a.x + w_a.y * r_a.y + w_a.z * r_a.z;
        let dot_ww_a = w_a.x * w_a.x + w_a.y * w_a.y + w_a.z * w_a.z;

        let res_x_a = w_a.x * dot_wr_a - r_a.x * dot_ww_a;
        let res_y_a = w_a.y * dot_wr_a - r_a.y * dot_ww_a;
        let res_z_a = w_a.z * dot_wr_a - r_a.z * dot_ww_a;

        let dot_wr_b = w_b.x * r_b.x + w_b.y * r_b.y + w_b.z * r_b.z;
        let dot_ww_b = w_b.x * w_b.x + w_b.y * w_b.y + w_b.z * w_b.z;

        let res_x_b = w_b.x * dot_wr_b - r_b.x * dot_ww_b;
        let res_y_b = w_b.y * dot_wr_b - r_b.y * dot_ww_b;
        let res_z_b = w_b.z * dot_wr_b - r_b.z * dot_ww_b;

        velocity_bias[0] = res_x_b - res_x_a;
        velocity_bias[1] = res_y_b - res_y_a;
        velocity_bias[2] = res_z_b - res_z_a;
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

        let mut jacobian = [JacobianRow::ZERO; 6];

        let joint = SphericalJoint {};

        joint.calculate_jacobian(
            &body_a_state,
            &body_b_state,
            anchor_a,
            anchor_b,
            &mut jacobian,
        );

        let mut expected = [JacobianRow::ZERO; 6];
        expected[0].v_a.x = -1.0;
        expected[0].v_b.x = 1.0;
        expected[0].w_a.z = 4.0;
        expected[0].w_b.z = 6.0;

        expected[1].v_a.y = -1.0;
        expected[1].v_b.y = 1.0;

        expected[2].v_a.z = -1.0;
        expected[2].v_b.z = 1.0;
        expected[2].w_a.x = -4.0;
        expected[2].w_b.x = -6.0;

        for i in 0..6 {
            assert!((jacobian[i].v_a.x - expected[i].v_a.x).abs() < 1e-6);
            assert!((jacobian[i].v_a.y - expected[i].v_a.y).abs() < 1e-6);
            assert!((jacobian[i].v_a.z - expected[i].v_a.z).abs() < 1e-6);

            assert!((jacobian[i].w_a.x - expected[i].w_a.x).abs() < 1e-6);
            assert!((jacobian[i].w_a.y - expected[i].w_a.y).abs() < 1e-6);
            assert!((jacobian[i].w_a.z - expected[i].w_a.z).abs() < 1e-6);

            assert!((jacobian[i].v_b.x - expected[i].v_b.x).abs() < 1e-6);
            assert!((jacobian[i].v_b.y - expected[i].v_b.y).abs() < 1e-6);
            assert!((jacobian[i].v_b.z - expected[i].v_b.z).abs() < 1e-6);

            assert!((jacobian[i].w_b.x - expected[i].w_b.x).abs() < 1e-6);
            assert!((jacobian[i].w_b.y - expected[i].w_b.y).abs() < 1e-6);
            assert!((jacobian[i].w_b.z - expected[i].w_b.z).abs() < 1e-6);
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

        let mut jacobian = [JacobianRow::ZERO; 6];

        let joint = SphericalJoint {};

        joint.calculate_jacobian(
            &body_a_state,
            &body_b_state,
            anchor_a,
            anchor_b,
            &mut jacobian,
        );

        let mut expected = [JacobianRow::ZERO; 6];
        expected[0].v_a.x = -1.0;
        expected[0].v_b.x = 1.0;
        expected[0].w_a.y = -1.0;
        expected[0].w_b.y = -3.0;
        expected[0].w_a.z = 1.0;
        expected[0].w_b.z = 2.0;

        expected[1].v_a.y = -1.0;
        expected[1].v_b.y = 1.0;
        expected[1].w_a.x = 1.0;
        expected[1].w_b.x = 3.0;
        expected[1].w_a.z = -1.0;
        expected[1].w_b.z = -1.0;

        expected[2].v_a.z = -1.0;
        expected[2].v_b.z = 1.0;
        expected[2].w_a.x = -1.0;
        expected[2].w_b.x = -2.0;
        expected[2].w_a.y = 1.0;
        expected[2].w_b.y = 1.0;

        for i in 0..6 {
            assert!((jacobian[i].v_a.x - expected[i].v_a.x).abs() < 1e-6);
            assert!((jacobian[i].v_a.y - expected[i].v_a.y).abs() < 1e-6);
            assert!((jacobian[i].v_a.z - expected[i].v_a.z).abs() < 1e-6);

            assert!((jacobian[i].w_a.x - expected[i].w_a.x).abs() < 1e-6);
            assert!((jacobian[i].w_a.y - expected[i].w_a.y).abs() < 1e-6);
            assert!((jacobian[i].w_a.z - expected[i].w_a.z).abs() < 1e-6);

            assert!((jacobian[i].v_b.x - expected[i].v_b.x).abs() < 1e-6);
            assert!((jacobian[i].v_b.y - expected[i].v_b.y).abs() < 1e-6);
            assert!((jacobian[i].v_b.z - expected[i].v_b.z).abs() < 1e-6);

            assert!((jacobian[i].w_b.x - expected[i].w_b.x).abs() < 1e-6);
            assert!((jacobian[i].w_b.y - expected[i].w_b.y).abs() < 1e-6);
            assert!((jacobian[i].w_b.z - expected[i].w_b.z).abs() < 1e-6);
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

        let joint = SphericalJoint {};

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
