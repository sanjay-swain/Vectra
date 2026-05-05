use glam::DQuat;

use crate::{integrator::integrator::Integrator, system::body::Body};

pub struct ExplicitEuler {}

impl Integrator for ExplicitEuler {
    fn step(&self, bodies: &mut Vec<Body>, step_size: f64) {
        for body in bodies {
            body.state.position += body.state.velocity * step_size;
            body.state.velocity += body.state_derivative.acceleration * step_size;

            let q = body.state.orientation;
            let w = body.state.angular_velocity;
            let dq_dt = DQuat::from_xyzw(
                0.5 * (w.x * q.x + w.y * q.z - w.z * q.y),
                0.5 * (-w.x * q.z + w.y * q.w + w.z * q.x),
                0.5 * (w.x * q.y - w.y * q.x + w.z * q.w),
                0.5 * (-w.x * q.x - w.y * q.y - w.z * q.z),
            );

            body.state.orientation = (q + dq_dt * step_size).normalize();
            body.state.angular_velocity += body.state_derivative.angular_acceleration * step_size;
        }
    }
}

pub struct SemiImplicitEuler {}

impl Integrator for SemiImplicitEuler {
    fn step(&self, bodies: &mut Vec<Body>, step_size: f64) {
        for body in bodies {
            body.state.velocity += body.state_derivative.acceleration * step_size;
            body.state.position += body.state.velocity * step_size;

            body.state.angular_velocity += body.state_derivative.angular_acceleration * step_size;

            let angle = body.state.angular_velocity.length() * step_size;
            if angle > 1e-6 {
                let axis = body.state.angular_velocity.normalize();
                let dq = DQuat::from_axis_angle(axis, angle);

                body.state.orientation = (body.state.orientation * dq).normalize();
            }
        }
    }
}
