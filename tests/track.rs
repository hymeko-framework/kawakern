//! A rational circle as a closed road, built from plain data.

use hymeko_kinematics::Indexed;
use kawakern::track::{ObstacleSpec, Track, TrackControl, TrackSpec};

const W: f64 = std::f64::consts::FRAC_1_SQRT_2;

fn circle() -> TrackSpec<Indexed> {
    let points = [
        ([100.0, 0.0], 1.0),
        ([100.0, 100.0], W),
        ([0.0, 100.0], 1.0),
        ([-100.0, 100.0], W),
        ([-100.0, 0.0], 1.0),
        ([-100.0, -100.0], W),
        ([0.0, -100.0], 1.0),
        ([100.0, -100.0], W),
        ([100.0, 0.0], 1.0),
    ];
    TrackSpec {
        id: 0,
        width: 16.0,
        samples: 2048,
        degree: 2,
        knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0],
        controls: points
            .iter()
            .enumerate()
            .map(|(k, &(point, weight))| TrackControl { id: k + 1, point, weight })
            .collect(),
        obstacles: vec![ObstacleSpec { id: 10, fraction: 0.25, offset: 3.0, radius: 0.5, height: 1.0 }],
    }
}

#[test]
fn a_rational_circle_closes_with_exact_frames() {
    let road = Track::new(circle()).expect("a valid road");
    assert!((road.length() - std::f64::consts::TAU * 100.0).abs() < 0.001);
    for s in road.samples() {
        assert!((s.position[0].hypot(s.position[1]) - 100.0).abs() < 1e-10);
        assert!((s.curvature - 0.01).abs() < 1e-12);
    }
    let obstacle = &road.obstacles[0];
    assert_eq!(obstacle.id, 10);
    assert!(obstacle.position[0].abs() < 1e-9);
    assert!((obstacle.position[1] - 97.0).abs() < 1e-9);
    let p = road.project([0.0, 102.0], None).expect("a projection");
    assert!((p.lateral + 2.0).abs() < 0.001, "{p:?}");
    assert!(road.sample(f64::NAN).is_err());
    assert!(road.project([f64::INFINITY, 0.0], None).is_none());
    assert_eq!(road.id, 0);
}

#[test]
fn malformed_or_unbounded_roads_fail_before_use() {
    let cases: [fn(&mut TrackSpec<Indexed>); 9] = [
        |s| s.samples = 8193,
        |s| s.width = 0.0,
        |s| s.degree = 6,
        |s| s.controls[1].weight = 0.0,
        |s| s.controls[8].point = [101.0, 0.0],
        |s| s.obstacles[0].fraction = 1.0,
        |s| s.obstacles[0].radius = -1.0,
        |s| s.obstacles[0].offset = 100.0,
        |s| s.knots[3] = 2.0,
    ];
    for (i, break_it) in cases.iter().enumerate() {
        let mut spec = circle();
        break_it(&mut spec);
        assert!(Track::new(spec).is_err(), "case {i} accepted");
    }
}
