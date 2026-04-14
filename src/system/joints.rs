use glam::DVec3;

use crate::system::state::State;

#[derive(Clone, Copy)]
pub struct JacobianRow {
    pub v_a: DVec3,
    pub w_a: DVec3,
    pub v_b: DVec3,
    pub w_b: DVec3,
}

impl JacobianRow {
    pub const ZERO: Self = Self {
        v_a: DVec3::ZERO,
        w_a: DVec3::ZERO,
        v_b: DVec3::ZERO,
        w_b: DVec3::ZERO,
    };
}

pub trait Joint {
    fn restricted_dof(&self) -> usize;

    fn calculate_jacobian(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        jacobian: &mut Vec<JacobianRow>,
    );

    fn calculate_velocity_bias(
        &self,
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        velocity_bias: &mut Vec<f64>,
    );
}

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
        jacobian: &mut Vec<JacobianRow>,
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
        velocity_bias: &mut Vec<f64>,
    ) {
        let r_a = state_a.orientation.mul_vec3(anchor_a);
        let r_b = state_b.orientation.mul_vec3(anchor_b);
        let omega_a = state_a.angular_velocity;
        let omega_b = state_b.angular_velocity;
        velocity_bias[0] = (-omega_a.x * (omega_a.z * r_a.z + omega_a.y * r_a.y)
            + omega_a.y * omega_a.y * r_a.x
            + omega_a.z * omega_a.z * r_a.x)
            - (-omega_b.x * (omega_b.z * r_b.z + omega_b.y * r_b.y)
                + omega_b.y * omega_b.y * r_b.x
                + omega_b.z * omega_b.z * r_b.x);

        velocity_bias[1] = (omega_a.x * omega_a.x * r_a.y
            - omega_a.y * (omega_a.z * r_a.z + omega_a.x * r_a.x)
            + omega_a.z * omega_a.z * r_a.x)
            - (omega_b.x * omega_b.x * r_b.y - omega_b.y * (omega_b.z * r_b.z + omega_b.x * r_b.x)
                + omega_b.z * omega_b.z * r_b.x);

        velocity_bias[2] = (omega_a.x * omega_a.x * r_a.z + omega_a.y * omega_a.y * r_a.z
            - omega_a.z * (omega_a.y * r_a.y + omega_a.x * r_a.x))
            - (omega_b.x * omega_b.x * r_b.z + omega_b.y * omega_b.y * r_b.z
                - omega_b.z * (omega_b.y * r_b.y + omega_b.x * r_b.x));
    }
}
