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
        constraints::{distance::DistanceJoint, joints::JointType},
        state::State,
        world::World,
    },
};

use std::error::Error;
use std::fs::File;

fn main() -> Result<(), Box<dyn Error>> {
    let file = File::create("pendulum.csv")?;
    let mut wtr = csv::Writer::from_writer(file);

    println!("Starting");
    let force_solver = NewtonEuler {};
    let constraint_solver = AccelerationConstraint {};
    let integration = SemiImplicitEuler {};
    let mut world = match World::new(force_solver, constraint_solver, integration, 1e-3) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let gr = match world.add_ground() {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let b1 = match world.create_body(
        1.0,
        DMat3::from_diagonal(DVec3::new(0.004, 0.004, 0.004)),
        State {
            position: DVec3::new(1.0, 0.0, 1.0),
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
        },
        false,
    ) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let dist = DistanceJoint::new(1.0);

    world.create_constraint(
        gr,
        b1,
        DVec3::new(0.0, 0.0, 1.0),
        DVec3::new(0.0, 0.0, 0.0),
        DQuat::IDENTITY,
        DQuat::IDENTITY,
        JointType::DistanceJoint(dist),
    );

    let mut t: f64 = 0.0;

    let mut step: u64 = 0;

    let log_enable = true;

    while t < 10.0 {
        world.apply_gravity_force();

        for constraint in &mut world.constraints {
            world.constraint_solver.solve(constraint, &world.bodies);
        }

        world.apply_constraint_forces();

        world.force_solver.solve(&mut world.bodies);

        world.integrator.step(&mut world.bodies, world.step_size);

        if log_enable && (step % 10 == 0) {
            let mut log = PhysicsLog::ZERO;

            log.update(&world.bodies[1], &world.constraints[0], t);

            log.energy = world.bodies[b1].mass
                * (world.gravity.force.length() * world.bodies[b1].state.position.z)
                + world.bodies[b1].kinetic_energy();

            // log.constraint_error = world.constraints[0].joint.calculate_joint_error(
            //     &world.bodies[0].state,
            //     &world.bodies[1].state,
            //     world.constraints[0].anchor_a,
            //     world.constraints[0].anchor_b,
            // );

            wtr.serialize(log)?;
        }

        t += world.step_size;

        world.clear_forces_and_torques();
        step += 1;
    }
    println!("{}", world.bodies[1].state.position);
    wtr.flush()?;
    Ok(())
}
