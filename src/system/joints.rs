use glam::DVec3;

use crate::system::state::State;

pub struct JacobianRow {
    pub v_a: DVec3,
    pub w_a: DVec3,
    pub v_b: DVec3,
    pub w_b: DVec3,
}

pub trait Joint {
    fn calculate_jacobian(
        state_a: &State,
        state_b: &State,
        anchor_a: DVec3,
        anchor_b: DVec3,
        jacobian: &mut Vec<JacobianRow>,
    );
}

pub struct SphericalJoint {}

impl Joint for SphericalJoint {
    fn calculate_jacobian(
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
}
