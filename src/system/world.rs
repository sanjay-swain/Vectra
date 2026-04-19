use glam::{DMat3, DVec3};

use crate::{
    dynamics::{constraint_solver::ConstraintSolver, forces::ForceSolver},
    integrator::integrator::Integrator,
    system::{
        body::Body,
        constraints::{constraint::Constraint, joints::Joint},
        interactions::{Force, Frame, Torque},
        state::State,
    },
};

pub struct World<F, C, I>
where
    F: ForceSolver,
    C: ConstraintSolver,
    I: Integrator,
{
    pub bodies: Vec<Body>,
    pub constraints: Vec<Constraint>,

    pub force_solver: F,
    pub constraint_solver: C,
    pub integrator: I,

    pub enable_gravity: bool,
    pub gravity: Force,
    pub step_size: f64,
    next_id: usize,
}

impl<F, C, I> World<F, C, I>
where
    F: ForceSolver,
    C: ConstraintSolver,
    I: Integrator,
{
    pub fn new(force_solver: F, constraint_solver: C, integrator: I, step_size: f64) -> Self {
        Self {
            bodies: Vec::new(),
            constraints: Vec::new(),
            force_solver,
            constraint_solver,
            integrator,
            enable_gravity: true,
            gravity: Force {
                force: DVec3::new(0.0, 0.0, -9.81),
                position: DVec3::ZERO,
                frame: Frame::Global,
            },
            step_size,
            next_id: 0,
        }
    }

    pub fn create_body(
        &mut self,
        mass: f64,
        inertia: DMat3,
        initial_state: State,
        is_static: bool,
    ) -> usize {
        let id = self.next_id;
        if is_static {
            self.bodies.push(Body {
                id: self.next_id,
                mass: mass,
                mass_inv: 0.0,
                inertia: inertia,
                inertia_inv: DMat3::ZERO,
                is_static: true,
                ..Default::default()
            });
        } else {
            self.bodies.push(Body {
                id: self.next_id,
                mass: mass,
                mass_inv: 1.0 / mass,
                inertia: inertia,
                inertia_inv: inertia.inverse(),
                state: initial_state,
                is_static: false,
                ..Default::default()
            });
        }

        self.next_id += 1;

        return id;
    }

    pub fn add_ground(&mut self) -> usize {
        self.create_body(1.0, DMat3::IDENTITY, State::ZERO, true)
    }

    pub fn create_constraint(
        &mut self,
        body_a_id: usize,
        body_b_id: usize,
        anchor_a: DVec3,
        anchor_b: DVec3,
        joint: Box<dyn Joint>,
    ) {
        let mut body_a_index = 0;
        let mut body_b_index = 0;
        let mut i = 0;
        while i < self.bodies.len() {
            if self.bodies[i].id == body_a_id {
                body_a_index = i;
            }
            if self.bodies[i].id == body_b_id {
                body_b_index = i;
            }
            i = i + 1;
        }
        self.constraints.push(Constraint::new(
            body_a_index,
            body_b_index,
            anchor_a,
            anchor_b,
            joint,
        ));
    }

    pub fn set_gravity(&mut self, g: DVec3) {
        self.gravity = Force::new(g, DVec3::ZERO, Frame::Global);
    }

    pub fn apply_gravity_force(&mut self) {
        if self.enable_gravity {
            for body in &mut self.bodies {
                body.apply_force(self.gravity);
            }
        }
    }

    pub fn apply_constraint_forces(&mut self) {
        for constraint in &self.constraints {
            self.bodies[constraint.body_a_index].apply_force(Force {
                force: DVec3::new(
                    constraint.constraint_forces[0],
                    constraint.constraint_forces[1],
                    constraint.constraint_forces[2],
                ),
                position: constraint.body_a_anchor,
                frame: Frame::Global,
            });

            self.bodies[constraint.body_a_index].apply_torque(Torque {
                torque: DVec3::new(
                    constraint.constraint_forces[3],
                    constraint.constraint_forces[4],
                    constraint.constraint_forces[5],
                ),
                frame: Frame::Global,
            });

            self.bodies[constraint.body_b_index].apply_force(Force {
                force: DVec3::new(
                    constraint.constraint_forces[6],
                    constraint.constraint_forces[7],
                    constraint.constraint_forces[8],
                ),
                position: constraint.body_b_anchor,
                frame: Frame::Global,
            });

            self.bodies[constraint.body_b_index].apply_torque(Torque {
                torque: DVec3::new(
                    constraint.constraint_forces[9],
                    constraint.constraint_forces[10],
                    constraint.constraint_forces[11],
                ),
                frame: Frame::Global,
            });
        }
    }

    pub fn clear_forces_and_torques(&mut self) {
        for body in &mut self.bodies {
            body.clear_forces();
            body.clear_torques();
        }
    }
}
