use glam::DMat3;

use crate::{
    dynamics::{forces::compute_resultant, gauss_seidel::gauss_sediel},
    system::{
        body::Body,
        constraints::{constraint::Constraint, joints::JacobianRow},
        interactions::{Force, Torque},
    },
};

pub trait ConstraintSolver {
    fn solve(&self, constraint: &mut Constraint, bodies: &Vec<Body>);
}

pub struct AccelerationConstraint {}

impl ConstraintSolver for AccelerationConstraint {
    fn solve(&self, constraint: &mut Constraint, bodies: &Vec<Body>) {
        let n = constraint.joint.restricted_dof();

        let mut k_matrix: Vec<Vec<f64>> = vec![vec![0.0; n]; n];
        let mut rhs: Vec<f64> = vec![0.0; n];

        let body_a_orient = bodies[constraint.body_a_index].state.orientation;
        let body_b_orient = bodies[constraint.body_b_index].state.orientation;

        let r_a = DMat3::from_quat(body_a_orient);
        let r_b = DMat3::from_quat(body_b_orient);

        let m_a = bodies[constraint.body_a_index].mass_inv;
        let i_a = r_a * bodies[constraint.body_a_index].inertia_inv * r_a.transpose();
        let m_b = bodies[constraint.body_b_index].mass_inv;
        let i_b = r_b * bodies[constraint.body_b_index].inertia_inv * r_b.transpose();

        let f_a_ext: Force;
        let f_b_ext: Force;
        let t_a_ext: Torque;
        let t_b_ext: Torque;

        (f_a_ext, t_a_ext) = compute_resultant(
            &bodies[constraint.body_a_index].forces,
            &bodies[constraint.body_a_index].torques,
            body_a_orient,
        );

        (f_b_ext, t_b_ext) = compute_resultant(
            &bodies[constraint.body_b_index].forces,
            &bodies[constraint.body_b_index].torques,
            body_b_orient,
        );

        let mut j_m: Vec<JacobianRow> = vec![JacobianRow::ZERO; n];

        let mut i: usize = 0;

        while i < n {
            j_m[i] = JacobianRow {
                v_a: constraint.jacobian[i].v_a * m_a,
                w_a: i_a * constraint.jacobian[i].w_a,
                v_b: constraint.jacobian[i].v_b * m_b,
                w_b: i_b * constraint.jacobian[i].w_b,
            };

            let mut j = 0;

            while j < n {
                k_matrix[i][j] = j_m[i].dot(&constraint.jacobian[j]);
                j = j + 1;
            }

            // RHS side calculation now
            rhs[i] = -1.0 * constraint.velocity_bias[i]
                - (j_m[i].v_a.dot(f_a_ext.to_global(body_a_orient))
                    + j_m[i].w_a.dot(t_a_ext.to_global(body_a_orient))
                    + j_m[i].v_b.dot(f_b_ext.to_global(body_b_orient))
                    + j_m[i].w_b.dot(t_b_ext.to_global(body_b_orient)));

            i = i + 1;
        }

        gauss_sediel(&k_matrix, &rhs, &mut constraint.lagrange_multiplier, 10);

        i = 0;

        while i < n {
            constraint.constraint_forces.f_a = constraint.constraint_forces.f_a
                + constraint.jacobian[i].v_a * constraint.lagrange_multiplier[i];

            constraint.constraint_forces.t_a = constraint.constraint_forces.t_a
                + constraint.jacobian[i].w_a * constraint.lagrange_multiplier[i];

            constraint.constraint_forces.f_b = constraint.constraint_forces.f_b
                + constraint.jacobian[i].v_b * constraint.lagrange_multiplier[i];

            constraint.constraint_forces.t_b = constraint.constraint_forces.t_b
                + constraint.jacobian[i].w_b * constraint.lagrange_multiplier[i];

            i = i + 1;
        }
    }
}
