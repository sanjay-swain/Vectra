use glam::DMat3;

pub enum PhysicsError {
    InvalidMass(f64),
    InvalidInertia(DMat3),
    InvalidStepSize(f64),
}
