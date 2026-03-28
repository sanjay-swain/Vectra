use glam::{DQuat, DVec3};

/// Specifies the reference frame of a vector
#[derive(Clone, Copy)]
pub enum Frame {
    /// The vector is defined relative to the world's origin
    Global,
    /// The vector is defined relative to the body center of mass.
    Local,
}

#[derive(Clone, Copy)]
pub struct Force {
    pub force: DVec3,
    /// The point where force is applied.
    /// This is ALWAYS with respect to local frame of reference regardless the frame assigned is Local or Global
    pub position: DVec3,
    /// This defines whether the force is being applied is oriented according to local frame of reference or global
    pub frame: Frame,
}

impl Force {
    pub const ZERO_LOCAL: Self = Self {
        force: DVec3::ZERO,
        position: DVec3::ZERO,
        frame: Frame::Local,
    };

    pub const ZERO_GLOBAL: Self = Self {
        force: DVec3::ZERO,
        position: DVec3::ZERO,
        frame: Frame::Global,
    };

    pub fn new(force: DVec3, position: DVec3, frame: Frame) -> Self {
        Self {
            force: force,
            position: position,
            frame: frame,
        }
    }

    pub fn to_global(&self, orientation: DQuat) -> DVec3 {
        return match self.frame {
            Frame::Global => self.force,
            Frame::Local => orientation * self.force,
        };
    }

    pub fn to_local(&self, orientation: DQuat) -> DVec3 {
        return match self.frame {
            Frame::Global => orientation.inverse() * self.force,
            Frame::Local => self.force,
        };
    }
}

#[derive(Clone, Copy)]
pub struct Torque {
    pub torque: DVec3,
    pub frame: Frame,
}

impl Torque {
    pub const ZERO_LOCAL: Self = Self {
        torque: DVec3::ZERO,
        frame: Frame::Local,
    };

    pub const ZERO_GLOBAL: Self = Self {
        torque: DVec3::ZERO,
        frame: Frame::Global,
    };

    pub fn new(torque: DVec3, frame: Frame) -> Self {
        Self {
            torque: torque,
            frame: frame,
        }
    }

    pub fn to_global(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::Global => self.torque,
            Frame::Local => orientation * self.torque,
        }
    }

    pub fn to_local(&self, orientation: DQuat) -> DVec3 {
        match self.frame {
            Frame::Global => orientation.inverse() * self.torque,
            Frame::Local => self.torque,
        }
    }
}
