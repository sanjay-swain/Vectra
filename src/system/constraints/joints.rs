use crate::system::{
    constraints::{distance::DistanceJoint, spherical::SphericalJoint},
    state::State,
};

use enum_dispatch::enum_dispatch;

#[derive(Clone, Copy)]
pub struct Jacobian {
    pub j: [[f64; 12]; 6],
}

impl Jacobian {
    pub const ZERO: Self = Self { j: [[0.0; 12]; 6] };

    pub fn dot(&self, row_index: usize, jacobian_row: &[f64; 12]) -> f64 {
        let mut sum = 0.0;

        for i in 0..12 {
            sum += self.j[row_index][i] * jacobian_row[i];
        }

        return sum;
    }
}

#[enum_dispatch]
pub trait Joint {
    fn restricted_dof(&self) -> usize;

    fn calculate_jacobian(&self, state_a: &State, state_b: &State, jacobian: &mut Jacobian);

    fn calculate_velocity_bias(
        &self,
        state_a: &State,
        state_b: &State,
        velocity_bias: &mut [f64; 6],
    );

    fn calculate_joint_error(&self, state_a: &State, state_b: &State) -> f64;
}

#[enum_dispatch(Joint)]
pub enum JointType {
    SphericalJoint,
    DistanceJoint,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dot_product_tests() {
        let zero_jacobian = Jacobian::ZERO;
        let ones_jacobian = Jacobian { j: [[1.0; 12]; 6] };
        let complex_jacobian_1 = Jacobian {
            j: [
                [
                    1.0, -2.0, 3.0, 0.0, 5.0, -1.0, 0.5, 2.0, -3.0, 1.0, 0.0, -4.0,
                ],
                [0.0; 12],
                [0.0; 12],
                [0.0; 12],
                [0.0; 12],
                [0.0; 12],
            ],
        };

        let complex_jacobian_2 = Jacobian {
            j: [
                [2.0, 3.0, -1.0, 5.0, 0.0, 4.0, 2.0, -1.0, 0.0, 3.0, 6.0, 1.0],
                [0.0; 12],
                [0.0; 12],
                [0.0; 12],
                [0.0; 12],
                [0.0; 12],
            ],
        };

        assert!((zero_jacobian.dot(0, &zero_jacobian.j[0])).abs() < 1e-6);
        assert!((ones_jacobian.dot(0, &zero_jacobian.j[0])).abs() < 1e-6);
        assert!((ones_jacobian.dot(0, &ones_jacobian.j[0]) - 12.0).abs() < 1e-6);
        assert!((complex_jacobian_1.dot(0, &complex_jacobian_2.j[0]) - (-13.0)).abs() < 1e-6);
    }
}
