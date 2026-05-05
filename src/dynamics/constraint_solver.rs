use crate::{
    dynamics::{forces::compute_resultant, gauss_seidel::gauss_sediel},
    system::{
        body::Body,
        constraints::{constraint::Constraint, joints::Jacobian},
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

        let m_a = bodies[constraint.body_a_index].mass_inv;
        let i_a = bodies[constraint.body_a_index]
            .inertia_inv_world()
            .to_cols_array_2d();

        let m_b = bodies[constraint.body_b_index].mass_inv;
        let i_b = bodies[constraint.body_b_index]
            .inertia_inv_world()
            .to_cols_array_2d();

        let mut f_a_ext: Force;
        let mut f_b_ext: Force;
        let mut t_a_ext: Torque;
        let mut t_b_ext: Torque;

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

        f_a_ext.force = f_a_ext.to_global(body_a_orient);
        f_b_ext.force = f_b_ext.to_global(body_b_orient);
        t_a_ext.torque = t_a_ext.to_global(body_a_orient);
        t_b_ext.torque = t_b_ext.to_global(body_b_orient);

        let f_ext: [f64; 12] = [
            f_a_ext.force.x,
            f_a_ext.force.y,
            f_a_ext.force.z,
            t_a_ext.torque.x,
            t_a_ext.torque.y,
            t_a_ext.torque.z,
            f_b_ext.force.x,
            f_b_ext.force.y,
            f_b_ext.force.z,
            t_b_ext.torque.x,
            t_b_ext.torque.y,
            t_b_ext.torque.z,
        ];

        constraint.joint.calculate_jacobian(
            &bodies[constraint.body_a_index].state,
            &bodies[constraint.body_b_index].state,
            constraint.body_a_anchor,
            constraint.body_b_anchor,
            &mut constraint.jacobian,
        );

        constraint.joint.calculate_velocity_bias(
            &bodies[constraint.body_a_index].state,
            &bodies[constraint.body_b_index].state,
            constraint.body_a_anchor,
            constraint.body_b_anchor,
            &mut constraint.velocity_bias,
        );

        let mut j_m = Jacobian::ZERO;

        // Set constraint force to zero
        constraint.constraint_forces = [0.0; 12];

        for i in 0..n {
            j_m.j[i][i] = constraint.jacobian.j[i][i] * m_a;

            j_m.j[i][3] = constraint.jacobian.j[i][3] * i_a[0][0]
                + constraint.jacobian.j[i][4] * i_a[1][0]
                + constraint.jacobian.j[i][5] * i_a[2][0];

            j_m.j[i][4] = constraint.jacobian.j[i][3] * i_a[0][1]
                + constraint.jacobian.j[i][4] * i_a[1][1]
                + constraint.jacobian.j[i][5] * i_a[2][1];

            j_m.j[i][5] = constraint.jacobian.j[i][3] * i_a[0][2]
                + constraint.jacobian.j[i][4] * i_a[1][2]
                + constraint.jacobian.j[i][5] * i_a[2][2];

            j_m.j[i][i + 6] = constraint.jacobian.j[i][i + 6] * m_b;

            j_m.j[i][9] = constraint.jacobian.j[i][9] * i_b[0][0]
                + constraint.jacobian.j[i][10] * i_b[1][0]
                + constraint.jacobian.j[i][11] * i_b[2][0];

            j_m.j[i][10] = constraint.jacobian.j[i][9] * i_b[0][1]
                + constraint.jacobian.j[i][10] * i_b[1][1]
                + constraint.jacobian.j[i][11] * i_b[2][1];

            j_m.j[i][11] = constraint.jacobian.j[i][9] * i_b[0][2]
                + constraint.jacobian.j[i][10] * i_b[1][2]
                + constraint.jacobian.j[i][11] * i_b[2][2];

            for j in 0..n {
                k_matrix[i][j] = j_m.dot(i, &constraint.jacobian.j[j]);
            }

            // RHS side calculation now
            rhs[i] = (-1.0 * constraint.velocity_bias[i]) - (j_m.dot(i, &f_ext));
        }

        gauss_sediel(&k_matrix, &rhs, &mut constraint.lagrange_multiplier, 50);

        for i in 0..12 {
            let mut sum = 0.0;
            for j in 0..n {
                sum = sum + constraint.jacobian.j[j][i] * constraint.lagrange_multiplier[j];
            }
            constraint.constraint_forces[i] = sum;
        }
    }
}
