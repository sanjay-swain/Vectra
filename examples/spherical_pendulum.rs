use glam::{DMat3, DQuat, DVec3};
use kite_core::{
    dynamics::{
        constraint_solver::{AccelerationConstraint, ConstraintSolver},
        forces::ForceSolver,
        newton_euler::NewtonEuler,
    },
    integrator::{euler::SemiImplicitEuler, integrator::Integrator},
    plots::PhysicsLog,
    system::{constraints::spherical::SphericalJoint, state::State, world::World},
};

use std::error::Error;
use std::fs::File;

fn main() -> Result<(), Box<dyn Error>> {
    let file = File::create("simulation_log_2.csv")?;
    let mut wtr = csv::Writer::from_writer(file);

    println!("Starting");
    let force_solver = NewtonEuler {};
    let constraint_solver = AccelerationConstraint {};
    let integration = SemiImplicitEuler {};
    let mut world = World::new(force_solver, constraint_solver, integration, 1e-5);

    let gr = world.add_ground();

    let b1 = world.create_body(
        1.0,
        DMat3::from_diagonal(DVec3::new(0.004, 0.004, 0.004)),
        State {
            position: DVec3::new(1.0, 0.0, 5.0),
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
        },
        false,
    );

    world.create_constraint(
        gr,
        b1,
        DVec3::new(0.0, 0.0, 5.0),
        DVec3::new(-1.0, 0.0, 0.0),
        Box::new(SphericalJoint {}),
    );

    let mut t = 0.0;

    while t < 5.0 {
        world.apply_gravity_force();

        for constraint in &mut world.constraints {
            world.constraint_solver.solve(constraint, &world.bodies);
        }

        world.apply_constraint_forces();

        world.force_solver.solve(&mut world.bodies);

        world.integrator.step(&mut world.bodies, world.step_size);

        let log = PhysicsLog {
            time: t,
            pos_x: world.bodies[b1].state.position.x,
            pos_y: world.bodies[b1].state.position.y,
            pos_z: world.bodies[b1].state.position.z,

            vel_x: world.bodies[b1].state.velocity.x,
            vel_y: world.bodies[b1].state.velocity.y,
            vel_z: world.bodies[b1].state.velocity.z,

            constraint_error: world.constraints[0].joint.calculate_joint_error(
                &world.bodies[0].state,
                &world.bodies[1].state,
                world.constraints[0].body_a_anchor,
                world.constraints[0].body_b_anchor,
            ),

            force_x: world.constraints[0].constraint_forces[9],
            force_y: world.constraints[0].constraint_forces[10],
            force_z: world.constraints[0].constraint_forces[11],
        };

        wtr.serialize(log)?;

        t += world.step_size;

        world.clear_forces_and_torques();
    }
    println!("{}", world.bodies[1].state.position);
    wtr.flush()?;
    Ok(())
}
