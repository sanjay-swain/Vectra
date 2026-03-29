use glam::{DMat3, DQuat, DVec3};
use kite_core::{
    dynamics::newton_euler::newton_euler,
    integrator::euler::explicit_euler_step,
    system::{state::State, world::World},
};

fn main() {
    println!("Starting");
    let mut world = World::default();

    world.enable_gravity = false;

    world.create_body(
        3.0,
        DMat3::from_diagonal(DVec3::new(1.0, 2.0, 3.0)),
        State {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::new(0.0, 10.0, 0.01),
        },
    );

    let mut t: f64 = 0.0;

    while t < 10.0 {
        // Update the state_derivative of each body
        for body in &mut world.bodies {
            newton_euler(body);
        }

        // increase the time
        explicit_euler_step(&mut world);
        t += world.step_size;

        // Clear all the forces at the end
        world.clear_forces_and_torques();
    }
    println!("Finished");
    println!(
        "{}",
        (world.bodies[0].inertia * world.bodies[0].state.angular_velocity).length()
    )
}
