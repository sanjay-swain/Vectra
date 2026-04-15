use glam::{DMat3, DVec3};
use kite_core::{
    dynamics::{forces::ForceSolver, newton_euler::NewtonEuler},
    integrator::{euler::SemiImplicitEuler, integrator::Integrator},
    system::{
        interactions::{Frame, Torque},
        state::State,
        world::World,
    },
};

fn main() {
    println!("Starting");
    let force_solver = NewtonEuler {};
    let integration = SemiImplicitEuler {};
    let mut world = World::new(force_solver, integration, 1e-5);

    world.enable_gravity = false;

    world.create_body(3.0, DMat3::from_diagonal(DVec3::ONE), State::ZERO, false);

    let mut t: f64 = 0.0;

    while t < 10.0 {
        // Apply forces
        world.apply_gravity_force();
        world.bodies[0].apply_torque(Torque::new(
            DVec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Frame::Local,
        ));

        // Update the state_derivative of each body
        world.force_solver.solve(&mut world.bodies);

        // increase the time
        world.integrator.step(&mut world.bodies, world.step_size);
        t += world.step_size;

        // Clear all the forces at the end
        world.clear_forces_and_torques();
    }
    let (axis, angle) = world.bodies[0].state.orientation.to_axis_angle();
    println!("{} {}", axis, angle);
    println!("Finished");
}
