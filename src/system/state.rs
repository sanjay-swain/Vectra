use glam::{DQuat, DVec3};

pub struct State {
    pub position: DVec3,
    pub velocity: DVec3,
    pub orientation: DQuat,
    pub angular_velocity: DVec3,
}

pub struct StateDerivative {
    pub velocity: DVec3,
    pub acceleration: DVec3,
    pub angular_velocity: DVec3,
    pub angular_acceleration: DVec3,
}
