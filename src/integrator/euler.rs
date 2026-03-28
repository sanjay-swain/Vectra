use glam::DQuat;

use crate::system::{state::StateDerivative, world::World};

pub fn explict_euler_step(world: &mut World, state_derivative: StateDerivative) {
    for body in &mut world.bodies {
        body.state.position += state_derivative.velocity * world.step_size;
        body.state.velocity += state_derivative.acceleration * world.step_size;

        let q = body.state.orientation;
        let w = body.state.angular_velocity;
        let dq_dt = DQuat::from_xyzw(
            0.5 * (w.x * q.x + w.y * q.z - w.z * q.y),
            0.5 * (-w.x * q.z + w.y * q.w + w.z * q.x),
            0.5 * (w.x * q.y - w.y * q.x + w.z * q.w),
            0.5 * (-w.x * q.x - w.y * q.y - w.z * q.z),
        );

        body.state.orientation = (q + dq_dt * world.step_size).normalize();
        body.state.angular_velocity += state_derivative.angular_acceleration * world.step_size;
    }
}

pub fn semi_implict_euler_step(world: &mut World, state_derivative: StateDerivative) {
    for body in &mut world.bodies {
        body.state.velocity += state_derivative.acceleration * world.step_size;
        body.state.position += body.state.velocity * world.step_size;

        body.state.angular_velocity += state_derivative.angular_acceleration * world.step_size;

        let angle = body.state.angular_velocity.length() * world.step_size;
        if angle > 1e-6 {
            let axis = body.state.angular_velocity.normalize();
            let dq = DQuat::from_axis_angle(axis, angle);

            body.state.orientation = (body.state.orientation * dq).normalize();
        }
    }
}
