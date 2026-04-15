use glam::{DQuat, DVec3};

use crate::system::{
    body::Body,
    interactions::{Force, Torque},
};

pub fn compute_resultant(
    forces: &Vec<Force>,
    torques: &Vec<Torque>,
    orientation: DQuat,
) -> (Force, Torque) {
    let mut resultant_force = Force::ZERO_LOCAL;
    let mut resultant_torque = Torque::ZERO_LOCAL;

    let mut tmp: DVec3;

    for force in forces {
        tmp = force.to_local(orientation);
        resultant_force.force += tmp;
        resultant_torque.torque += force.position.cross(tmp);
    }

    for torque in torques {
        resultant_torque.torque += torque.to_local(orientation);
    }

    return (resultant_force, resultant_torque);
}

pub trait ForceSolver {
    fn solve(&self, bodies: &mut Vec<Body>);
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use glam::DQuat;

    use crate::system::interactions::Frame;

    use super::*;

    #[test]
    fn translation() {
        let mut forces: Vec<Force> = vec![];
        let torques: Vec<Torque> = vec![];

        forces.push(Force {
            force: DVec3 {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
            position: DVec3::ZERO,
            frame: Frame::Local,
        });

        forces.push(Force {
            force: DVec3 {
                x: 0.0,
                y: 20.0,
                z: 0.0,
            },
            position: DVec3::ZERO,
            frame: Frame::Local,
        });

        let (f, tau) = compute_resultant(&forces, &torques, DQuat::IDENTITY);

        assert!((f.force.x - 10.0).abs() < 1e-6);
        assert!((f.force.y - 20.0).abs() < 1e-6);
        assert!((f.force.z).abs() < 1e-6);

        assert!((tau.torque.x).abs() < 1e-6);
        assert!((tau.torque.y).abs() < 1e-6);
        assert!((tau.torque.z).abs() < 1e-6);
    }

    #[test]
    fn pure_couple() {
        let mut forces: Vec<Force> = vec![];
        let torques: Vec<Torque> = vec![];

        forces.push(Force {
            force: DVec3 {
                x: 0.0,
                y: 10.0,
                z: 0.0,
            },
            position: DVec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            frame: Frame::Local,
        });

        forces.push(Force {
            force: DVec3 {
                x: 0.0,
                y: -10.0,
                z: 0.0,
            },
            position: DVec3 {
                x: -1.0,
                y: 0.0,
                z: 0.0,
            },
            frame: Frame::Local,
        });

        let (f, tau) = compute_resultant(&forces, &torques, DQuat::IDENTITY);

        assert!((f.force.x).abs() < 1e-6);
        assert!((f.force.y).abs() < 1e-6);
        assert!((f.force.z).abs() < 1e-6);

        assert!((tau.torque.x).abs() < 1e-6);
        assert!((tau.torque.y).abs() < 1e-6);
        assert!((tau.torque.z - 20.0).abs() < 1e-6);
    }

    #[test]
    fn force_with_torque() {
        let mut forces: Vec<Force> = vec![];
        let torques: Vec<Torque> = vec![];

        let orientation = DQuat::from_axis_angle(DVec3::Z, PI / 2.0);

        forces.push(Force {
            force: DVec3 {
                x: 0.0,
                y: 10.0,
                z: 0.0,
            },
            position: DVec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            frame: Frame::Local,
        });

        let (f, tau) = compute_resultant(&forces, &torques, orientation);

        assert!((f.to_global(orientation).x + 10.0) < 1e-6);
        assert!((f.to_global(orientation).y) < 1e-6);
        assert!((f.to_global(orientation).x) < 1e-6);

        assert!((tau.to_global(orientation).x) < 1e-6);
        assert!((tau.to_global(orientation).y) < 1e-6);
        assert!((tau.to_global(orientation).z - 10.0) < 1e-6);
    }

    #[test]
    fn symmetry() {
        let mut forces: Vec<Force> = vec![];
        let torques: Vec<Torque> = vec![];

        let orientation = DQuat::from_axis_angle(DVec3::Y, PI);

        forces.push(Force {
            force: DVec3 {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
            position: DVec3 {
                x: 0.0,
                y: 0.0,
                z: 1.0,
            },
            frame: Frame::Local,
        });

        let (f, tau) = compute_resultant(&forces, &torques, orientation);

        assert!((f.to_global(orientation).x + 10.0) < 1e-6);
        assert!((f.to_global(orientation).y) < 1e-6);
        assert!((f.to_global(orientation).x) < 1e-6);

        assert!((tau.to_global(orientation).x) < 1e-6);
        assert!((tau.to_global(orientation).y - 10.0) < 1e-6);
        assert!((tau.to_global(orientation).z) < 1e-6);
    }
}
