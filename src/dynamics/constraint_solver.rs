use glam::DVec3;

use crate::{
    dynamics::{forces::compute_resultant, gauss_seidel::gauss_sediel},
    system::{
        body::Body,
        constraints::{constraint::Constraint, joints::Jacobian},
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
        let i_a = bodies[constraint.body_a_index].inertia_inv_world();

        let m_b = bodies[constraint.body_b_index].mass_inv;
        let i_b = bodies[constraint.body_b_index].inertia_inv_world();

        let mut j_m = Jacobian::ZERO;

        // Set constraint force to zero
        constraint.constraint_forces = [0.0; 12];

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

        // println!("{:#?}", constraint.velocity_bias);

        for i in 0..n {
            let w_a = DVec3::new(
                constraint.jacobian.j[i][3],
                constraint.jacobian.j[i][4],
                constraint.jacobian.j[i][5],
            );
            let w_b = DVec3::new(
                constraint.jacobian.j[i][9],
                constraint.jacobian.j[i][10],
                constraint.jacobian.j[i][11],
            );
            j_m.j[i][0] = m_a * constraint.jacobian.j[i][0];
            j_m.j[i][1] = m_a * constraint.jacobian.j[i][1];
            j_m.j[i][2] = m_a * constraint.jacobian.j[i][2];

            j_m.j[i][3] = i_a.x_axis.dot(w_a);
            j_m.j[i][4] = i_a.y_axis.dot(w_a);
            j_m.j[i][5] = i_a.z_axis.dot(w_a);

            j_m.j[i][6] = m_b * constraint.jacobian.j[i][6];
            j_m.j[i][7] = m_b * constraint.jacobian.j[i][7];
            j_m.j[i][8] = m_b * constraint.jacobian.j[i][8];

            j_m.j[i][9] = i_b.x_axis.dot(w_b);
            j_m.j[i][10] = i_b.y_axis.dot(w_b);
            j_m.j[i][11] = i_b.z_axis.dot(w_b);
        }

        for i in 0..n {
            for j in 0..n {
                k_matrix[i][j] = j_m.dot(i, &constraint.jacobian.j[j]);
            }
        }

        let (fa_ext, ta_ext) = compute_resultant(
            &bodies[constraint.body_a_index].forces,
            &bodies[constraint.body_a_index].torques,
            body_a_orient,
        );

        let (fb_ext, tb_ext) = compute_resultant(
            &bodies[constraint.body_b_index].forces,
            &bodies[constraint.body_b_index].torques,
            body_b_orient,
        );

        let fa_ext = fa_ext.to_global(body_a_orient);
        let fb_ext = fb_ext.to_global(body_b_orient);
        let ta_ext = ta_ext.to_global(body_a_orient);
        let tb_ext = tb_ext.to_global(body_b_orient);

        let f_ext = [
            fa_ext.x, fa_ext.y, fa_ext.z, ta_ext.x, ta_ext.y, ta_ext.z, fb_ext.x, fb_ext.y,
            fb_ext.z, tb_ext.x, tb_ext.y, tb_ext.z,
        ];

        for i in 0..n {
            rhs[i] = -1.0 * constraint.velocity_bias[i] - j_m.dot(i, &f_ext);
        }

        gauss_sediel(&k_matrix, &rhs, &mut constraint.lagrange_multiplier, 100);

        for i in 0..12 {
            let mut sum = 0.0;
            for j in 0..n {
                sum = sum + constraint.jacobian.j[j][i] * constraint.lagrange_multiplier[j];
            }
            constraint.constraint_forces[i] = sum;
        }
    }
}
