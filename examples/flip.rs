use glam::{DMat3, DQuat, DVec3};
use kite_core::{
    dynamics::{
        constraint_solver::AccelerationConstraint, forces::ForceSolver, newton_euler::NewtonEuler,
    },
    integrator::{euler::SemiImplicitEuler, integrator::Integrator},
    system::{state::State, world::World},
};

fn main() {
    println!("Starting");
    let force_solver = NewtonEuler {};
    let constraint_solver = AccelerationConstraint {};
    let integration = SemiImplicitEuler {};
    let mut world = match World::new(force_solver, constraint_solver, integration, 1e-3) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    world.enable_gravity = false;

    match world.create_body(
        3.0,
        DMat3::from_diagonal(DVec3::new(1.0, 2.0, 3.0)),
        State {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::new(0.0, 10.0, 0.01),
        },
        false,
    ) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let mut t: f64 = 0.0;

    while t < 10.0 {
        // Update the state_derivative of each body
        world.force_solver.solve(&mut world.bodies);

        // increase the time
        world.integrator.step(&mut world.bodies, world.step_size);

        t += world.step_size;

        // Clear all the forces at the end
        world.clear_forces_and_torques();
    }
    println!("Finished");
    println!(
        "{}",
        (world.bodies[0].inertia * world.bodies[0].state.angular_velocity).length()
    );
}
