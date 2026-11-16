//! The box prune of `Classifier::classify` against the answer over every
//! face (`classify_linear`), in a debug run: points near the faces, edges
//! and vertices of the sample bodies, and of a box whose vertices carry a
//! tolerance far above its edges' and faces'.

use arris_check::classify::{Classification, Classifier};
use arris_debug::sample;
use arris_math::Point3;
use arris_topo::{Body, Model};

/// Probes around `centre`: on it and offsets of `step` in each of the
/// 26 directions at three scales.
fn around(centre: Point3, step: f64) -> Vec<Point3> {
    let mut points = vec![centre];
    for scale in [0.25, 0.9, 1.5] {
        for i in -1i32..=1 {
            for j in -1i32..=1 {
                for k in -1i32..=1 {
                    if (i, j, k) != (0, 0, 0) {
                        let d = step * scale;
                        points.push(
                            centre
                                + arris_math::Vec3::new(
                                    f64::from(i) * d,
                                    f64::from(j) * d,
                                    f64::from(k) * d,
                                ),
                        );
                    }
                }
            }
        }
    }
    points
}

fn assert_prune_equals_linear(m: &Model, body: Body, centres: &[Point3], step: f64) -> usize {
    let classifier = Classifier::of_body(m, body).expect("the body resolves");
    let mut on = 0;
    for &c in centres {
        for p in around(c, step) {
            let linear = classifier.classify_linear(p);
            if linear.is_ok() {
                assert_eq!(classifier.classify(p), linear, "{p:?}");
            }
            on += usize::from(matches!(linear, Ok(Classification::On(_))));
        }
    }
    on
}

#[test]
fn the_prune_holds_on_the_sample_bodies() {
    let mut m = Model::default();
    let cuboid = sample::cuboid(&mut m, Point3::origin(), Point3::new(4.0, 3.0, 2.0)).unwrap();
    let corners: Vec<Point3> = m
        .closure(cuboid)
        .unwrap()
        .vertices
        .iter()
        .map(|&v| m.vertex(v).unwrap().point())
        .collect();
    let mut centres = corners.clone();
    centres.push(Point3::new(2.0, 0.0, 1.0));
    centres.push(Point3::new(2.0, 1.5, 0.0));
    centres.push(Point3::new(2.0, 1.5, 1.0));
    let on = assert_prune_equals_linear(&m, cuboid, &centres, 1e-4);
    assert!(on > 0);

    let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let centres = [
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(4.0, 0.0, 12.0),
        Point3::new(0.0, 4.0, 6.0),
        Point3::new(0.0, 0.0, 12.0),
        Point3::new(1.0, 1.0, 6.0),
    ];
    assert_prune_equals_linear(&m, cylinder, &centres, 1e-4);
}

/// A vertex's tolerance above its edges' and faces' put points on the
/// vertex outside every adjacent face's box, unless the box holds the
/// vertex's ball.
#[test]
fn the_prune_holds_near_a_vertex_looser_than_its_edges_and_faces() {
    let mut m = Model::default();
    let tol = 0.05;
    let body =
        sample::cuboid_loose_vertices(&mut m, Point3::origin(), Point3::new(4.0, 3.0, 2.0), tol)
            .unwrap();
    let classifier = Classifier::of_body(&m, body).unwrap();
    let corners: Vec<Point3> = m
        .closure(body)
        .unwrap()
        .vertices
        .iter()
        .map(|&v| m.vertex(v).unwrap().point())
        .collect();
    // Diagonally outward from a corner, within the vertex's ball but
    // beyond the edges' and faces' own tolerances.
    let mut found = 0;
    for &c in &corners {
        let outward = Point3::new(
            if c.x < 2.0 { -1.0 } else { 1.0 },
            if c.y < 1.5 { -1.0 } else { 1.0 },
            if c.z < 1.0 { -1.0 } else { 1.0 },
        );
        let p = c + outward.coords * (0.9 * tol / 3.0_f64.sqrt());
        let linear = classifier.classify_linear(p).unwrap();
        found += usize::from(matches!(linear, Classification::On(_)));
        assert_eq!(classifier.classify(p), Ok(linear), "{p:?}");
    }
    assert_eq!(found, 8, "each is on its own vertex");
    let on = assert_prune_equals_linear(&m, body, &corners, tol / 3.0);
    assert!(on > 0);
}
