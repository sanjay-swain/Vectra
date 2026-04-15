use glam::{DMat3, DVec3};

use crate::{
    dynamics::forces::ForceSolver,
    integrator::integrator::Integrator,
    system::{
        body::Body,
        interactions::{Force, Frame},
        state::State,
    },
};

pub struct World<F, I>
where
    F: ForceSolver,
    I: Integrator,
{
    pub bodies: Vec<Body>,

    pub force_solver: F,
    pub integrator: I,

    pub enable_gravity: bool,
    pub gravity: Force,
    pub step_size: f64,
    next_id: usize,
}

impl<F, I> World<F, I>
where
    F: ForceSolver,
    I: Integrator,
{
    pub fn new(force_solver: F, integrator: I, step_size: f64) -> Self {
        Self {
            bodies: Vec::new(),
            force_solver,
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
    ) {
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
    }

    pub fn add_ground(&mut self) {
        self.create_body(1.0, DMat3::IDENTITY, State::ZERO, true);
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

    pub fn clear_forces_and_torques(&mut self) {
        for body in &mut self.bodies {
            body.clear_forces();
            body.clear_torques();
        }
    }
}
