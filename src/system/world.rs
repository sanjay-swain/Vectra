use glam::{DMat3, DVec3};

use crate::{
    dynamics::{constraint_solver::ConstraintSolver, forces::ForceSolver},
    error::PhysicsError,
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
    pub fn new(
        force_solver: F,
        constraint_solver: C,
        integrator: I,
        step_size: f64,
    ) -> Result<Self, PhysicsError> {
        if step_size > 0.0 {
            Ok(Self {
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
            })
        } else {
            Err(PhysicsError::InvalidStepSize(step_size))
        }
    }

    pub fn create_body(
        &mut self,
        mass: f64,
        inertia: DMat3,
        initial_state: State,
        is_static: bool,
    ) -> Result<usize, PhysicsError> {
        let id = self.next_id;

        let mut mass_inv = 0.0;
        let mut inertia_inv = DMat3::ZERO;

        if !(mass > 0.0) {
            return Err(PhysicsError::InvalidMass(mass));
        }

        // Validate the inertia matrix
        // 1. All diagonal elements must be greater than zero
        let diagonal = inertia.diagonal();
        if (diagonal.x <= 0.0) || (diagonal.y <= 0.0) || (diagonal.z <= 0.0) {
            return Err(PhysicsError::InvalidInertia(inertia));
        }

        // 2. The product of inertia of any two product of inertia must be
        // greater than the third product of inertia
        if inertia.x_axis.y + inertia.x_axis.z < inertia.y_axis.z
            || (inertia.x_axis.y + inertia.y_axis.z < inertia.x_axis.z)
            || (inertia.y_axis.z + inertia.x_axis.z < inertia.x_axis.y)
        {
            return Err(PhysicsError::InvalidInertia(inertia));
        }

        // 3. Inertia matrix must be symmetric
        if !((inertia.x_axis.y - inertia.y_axis.x).abs() < 1e-6
            && (inertia.x_axis.z - inertia.z_axis.x).abs() < 1e-6
            && (inertia.z_axis.y - inertia.y_axis.z).abs() < 1e-6)
        {
            return Err(PhysicsError::InvalidInertia(inertia));
        }

        if !is_static {
            mass_inv = 1.0 / mass;
            inertia_inv = inertia.inverse();
        }

        self.bodies.push(Body {
            id: self.next_id,
            mass: mass,
            mass_inv: mass_inv,
            inertia: inertia,
            inertia_inv: inertia_inv,
            state: initial_state,
            is_static: is_static,
            ..Default::default()
        });

        self.next_id += 1;

        return Ok(id);
    }

    pub fn add_ground(&mut self) -> Result<usize, PhysicsError> {
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
                position: DVec3::ZERO,
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
                position: DVec3::ZERO,
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
