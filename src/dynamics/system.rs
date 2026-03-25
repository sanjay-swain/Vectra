use glam::DMat3;
use glam::DQuat;
use glam::DVec3;

pub struct State {
    position: DVec3,
    velocity: DVec3,
    quaternion: DQuat,
    angular_velocity: DVec3,
}

pub struct Body {
    id: u32,
    mass: f64,
    inertia: DMat3,
    state: State,
}

pub struct World {
    bodies: Vec<Body>,
    enable_gravity: bool,
    gravity: f64,
    next_id: u32,
}

impl World {
    pub fn create_body(&mut self, mass: f64, inertia: DMat3) {
        self.bodies.push(Body {
            id: self.next_id,
            mass: mass,
            inertia: inertia,
            state: State {
                position: DVec3::ZERO,
                velocity: DVec3::ZERO,
                quaternion: DQuat::IDENTITY,
                angular_velocity: DVec3::ZERO,
            },
        });
        self.next_id += 1;
    }
}
