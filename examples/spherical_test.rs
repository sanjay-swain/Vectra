use glam::{DMat3, DQuat, DVec3};
use kite_core::{
    dynamics::{
        constraint_solver::{AccelerationConstraint, ConstraintSolver},
        forces::ForceSolver,
        newton_euler::NewtonEuler,
    },
    integrator::{euler::SemiImplicitEuler, integrator::Integrator},
    plots::PhysicsLog,
    system::{
        constraints::spherical::SphericalJoint, interactions::Force, state::State, world::World,
    },
};

use std::error::Error;
use std::fs::File;

fn main() -> Result<(), Box<dyn Error>> {
    let file = File::create("simulation_log_3.csv")?;
    let mut wtr = csv::Writer::from_writer(file);

    println!("Starting");
    let constraint_solver = AccelerationConstraint {};
    let integration = SemiImplicitEuler {};
    let mut world = match World::new(NewtonEuler {}, constraint_solver, integration, 1e-5) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let b1 = match world.create_body(
        1.0,
        DMat3::from_diagonal(DVec3::new(0.004, 0.004, 0.004)),
        State {
            position: DVec3::new(1.0, 0.0, 0.0),
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
        },
        false,
    ) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let b2 = match world.create_body(
        1.0,
        DMat3::from_diagonal(DVec3::new(0.004, 0.004, 0.004)),
        State {
            position: DVec3::new(-1.0, 0.0, 0.0),
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
        },
        false,
    ) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    world.create_constraint(
        b1,
        b2,
        DVec3::new(-1.0, 0.0, 0.0),
        DVec3::new(1.0, 0.0, 0.0),
        Box::new(SphericalJoint {}),
    );

    let mut t = 0.0;

    while t < 5.0 {
        // world.apply_gravity_force();

        world.bodies[b1].apply_force(Force::new(
            DVec3::X,
            DVec3::ZERO,
            kite_core::system::interactions::Frame::Global,
        ));

        for constraint in &mut world.constraints {
            world.constraint_solver.solve(constraint, &world.bodies);
        }

        world.apply_constraint_forces();

        world.force_solver.solve(&mut world.bodies);

        world.integrator.step(&mut world.bodies, world.step_size);

        let mut log = PhysicsLog::ZERO;

        log.update(&world.bodies[b1], &world.constraints[0], t);

        wtr.serialize(log)?;

        t += world.step_size;

        world.clear_forces_and_torques();
    }
    println!(
        "{} {}",
        world.bodies[0].state.position, world.bodies[1].state.position
    );
    wtr.flush()?;
    Ok(())
}
