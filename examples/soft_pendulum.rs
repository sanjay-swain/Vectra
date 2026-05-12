use glam::{DMat3, DQuat, DVec3};
use kite_core::{
    dynamics::{
        constraint_solver::ConstraintSolver, forces::ForceSolver, newton_euler::NewtonEuler,
        soft_constraint::SoftConstraint,
    },
    integrator::{euler::SemiImplicitEuler, integrator::Integrator},
    system::{
        constraints::{
            joints::{Joint, JointType},
            spherical::SphericalJoint,
        },
        state::State,
        world::World,
    },
};

fn main() {
    println!("Starting");
    let force_solver = NewtonEuler {};
    let constraint_solver = SoftConstraint::new(0.01, 1e-5);
    let integration = SemiImplicitEuler {};
    let mut world = match World::new(force_solver, constraint_solver, integration, 1e-6) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let gr = match world.add_ground() {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let b1 = match world.create_body(
        1.0,
        DMat3::from_diagonal(DVec3::new(1.0, 1.0, 1.0)),
        State {
            position: DVec3::new(0.0, 0.0, 0.0),
            velocity: DVec3::new(3.0, 0.0, 0.0),
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::new(0.0, 3.0, 0.0),
        },
        false,
    ) {
        Ok(it) => it,
        Err(_err) => panic!(),
    };

    let spherical_joint = SphericalJoint::new();

    world.create_constraint(
        gr,
        b1,
        DVec3::new(0.0, 0.0, 1.0),
        DVec3::new(0.0, 0.0, 1.0),
        DQuat::IDENTITY,
        DQuat::IDENTITY,
        JointType::SphericalJoint(spherical_joint),
    );

    let mut t: f64 = 0.0;

    while t < 100.0 {
        world.apply_gravity_force();

        for constraint in &mut world.constraints {
            world.constraint_solver.solve(constraint, &world.bodies);
        }

        world.apply_constraint_forces();

        world.force_solver.solve(&mut world.bodies);

        world.integrator.step(&mut world.bodies, world.step_size);

        t += world.step_size;

        world.clear_forces_and_torques();
    }
    println!(
        "{}",
        world.bodies[b1].mass * (world.gravity.force.length() * world.bodies[b1].state.position.z)
            + world.bodies[b1].kinetic_energy()
    );
}
