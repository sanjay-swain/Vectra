use glam::DVec3;

use crate::system::{
    body::Body,
    interactions::{Force, Torque},
};

pub fn compute_resultant(body: &Body) -> (Force, Torque) {
    let mut resultant_force = Force::ZERO_LOCAL;
    let mut resultant_torque = Torque::ZERO_LOCAL;

    let mut tmp: DVec3;

    for force in &body.forces {
        tmp = force.to_local(body.state.orientation);
        resultant_force.force += tmp;
        resultant_torque.torque += force.position.cross(tmp);
    }

    for torque in &body.torques {
        resultant_torque.torque += torque.to_local(body.state.orientation);
    }

    return (resultant_force, resultant_torque);
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use glam::DQuat;

    use crate::system::interactions::Frame;

    use super::*;

    #[test]
    fn translation() {
        let mut body: Body = Body {
            mass: 2.0,
            ..Default::default()
        };

        body.apply_force(Force {
            force: DVec3 {
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
            position: DVec3::ZERO,
            frame: Frame::Local,
        });

        body.apply_force(Force {
            force: DVec3 {
                x: 0.0,
                y: 20.0,
                z: 0.0,
            },
            position: DVec3::ZERO,
            frame: Frame::Local,
        });

        let (f, tau) = compute_resultant(&body);

        assert!((f.force.x - 10.0).abs() < 1e-6);
        assert!((f.force.y - 20.0).abs() < 1e-6);
        assert!((f.force.z).abs() < 1e-6);

        assert!((tau.torque.x).abs() < 1e-6);
        assert!((tau.torque.y).abs() < 1e-6);
        assert!((tau.torque.z).abs() < 1e-6);
    }

    #[test]
    fn pure_couple() {
        let mut body: Body = Body {
            mass: 2.0,
            ..Default::default()
        };

        body.apply_force(Force {
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

        body.apply_force(Force {
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
        let (f, tau) = compute_resultant(&body);

        assert!((f.force.x).abs() < 1e-6);
        assert!((f.force.y).abs() < 1e-6);
        assert!((f.force.z).abs() < 1e-6);

        assert!((tau.torque.x).abs() < 1e-6);
        assert!((tau.torque.y).abs() < 1e-6);
        assert!((tau.torque.z - 20.0).abs() < 1e-6);
    }

    #[test]
    fn force_with_torque() {
        let mut body: Body = Body {
            mass: 2.0,
            ..Default::default()
        };

        body.state.orientation = DQuat::from_axis_angle(DVec3::Z, PI / 2.0);

        body.apply_force(Force {
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
        let (f, tau) = compute_resultant(&body);

        assert!((f.to_global(body.state.orientation).x + 10.0) < 1e-6);
        assert!((f.to_global(body.state.orientation).y) < 1e-6);
        assert!((f.to_global(body.state.orientation).x) < 1e-6);

        assert!((tau.to_global(body.state.orientation).x) < 1e-6);
        assert!((tau.to_global(body.state.orientation).y) < 1e-6);
        assert!((tau.to_global(body.state.orientation).z - 10.0) < 1e-6);
    }

    #[test]
    fn symmetry() {
        let mut body: Body = Body {
            mass: 2.0,
            ..Default::default()
        };

        body.state.orientation = DQuat::from_axis_angle(DVec3::Y, PI);

        body.apply_force(Force {
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
        let (f, tau) = compute_resultant(&body);

        assert!((f.to_global(body.state.orientation).x + 10.0) < 1e-6);
        assert!((f.to_global(body.state.orientation).y) < 1e-6);
        assert!((f.to_global(body.state.orientation).x) < 1e-6);

        assert!((tau.to_global(body.state.orientation).x) < 1e-6);
        assert!((tau.to_global(body.state.orientation).y - 10.0) < 1e-6);
        assert!((tau.to_global(body.state.orientation).z) < 1e-6);
    }
}
