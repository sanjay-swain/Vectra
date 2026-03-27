use glam::DVec3;

#[derive(Clone, Copy)]
pub enum Frame {
    Global,
    Local,
}

#[derive(Clone, Copy)]
pub struct Force {
    pub force: DVec3,
    pub position: DVec3,
    pub frame: Frame,
}

pub struct Torque {
    pub torque: DVec3,
}
