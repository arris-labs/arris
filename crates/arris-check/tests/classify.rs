//! `classify_point` against the closed forms of the bodies it is run on,
//! and against the entity a point is on (ADR-0004): B1's ray cast
//! made public and complete.

use arris_check::classify::{Classification, Classifier, ClassifyError, classify_point};
use arris_debug::prop::{DEFAULT_SCALE, check as prop_check, point_in_box, radius};
use arris_debug::sample;
use arris_geom::{GeomError, Surface};
use arris_math::{Frame, Point2, Point3};
use arris_topo::{Body, EntityKind, Model};
use proptest::prelude::*;

/// How far from the boundary a random probe has to be for the closed form
/// to be unambiguous: far above the model's default tolerance, far below
/// the bodies' own size.
const BAND: f64 = 1e-6;

/// `classify_point`, held to agree with a `Classifier` built once for the
/// body and asked twice: every point these tests classify checks both.
fn classify(m: &Model, body: Body, point: Point3) -> Result<Classification, ClassifyError> {
    let once = classify_point(m, body, point);
    match Classifier::of_body(m, body) {
        Ok(classifier) => {
            assert_eq!(classifier.classify(point), once, "{point:?}");
            assert_eq!(classifier.classify(point), once, "{point:?}, asked again");
        }
        Err(e) => assert_eq!(Err(e), once, "{point:?}"),
    }
    once
}

fn kind_of(c: Classification) -> Option<EntityKind> {
    match c {
        Classification::On(shape) => Some(shape.kind()),
        _ => None,
    }
}

#[test]
fn random_points_against_a_boxs_closed_form() {
    prop_check(
        (
            point_in_box(DEFAULT_SCALE),
            radius(1.0..=20.0),
            radius(1.0..=20.0),
            radius(1.0..=20.0),
            point_in_box(2.0 * DEFAULT_SCALE),
        ),
        |(min, dx, dy, dz, probe)| {
            let mut m = Model::default();
            let max = Point3::new(min.x + dx, min.y + dy, min.z + dz);
            let body =
                sample::cuboid(&mut m, min, max).map_err(|e| TestCaseError::fail(e.to_string()))?;
            // The signed distance to the box: negative inside.
            let outside: f64 = (0..3)
                .map(|k| (min[k] - probe[k]).max(probe[k] - max[k]))
                .fold(f64::NEG_INFINITY, f64::max);
            if outside.abs() <= BAND {
                // Too near the boundary for the closed form to say; the
                // deliberate `On` cases below are where that is tested.
                return Ok(());
            }
            let got = classify(&m, body, probe).map_err(|e| TestCaseError::fail(e.to_string()))?;
            let expected = if outside < 0.0 {
                Classification::Inside
            } else {
                Classification::Outside
            };
            prop_assert_eq!(got, expected, "{:?} against {:?}..{:?}", probe, min, max);
            prop_assert_eq!(classify(&m, body, probe).ok(), Some(got), "two runs differ");
            Ok(())
        },
    );
}

#[test]
fn random_points_against_a_cylinders_closed_form() {
    prop_check(
        (
            radius(1.0..=20.0),
            radius(1.0..=20.0),
            point_in_box(2.0 * DEFAULT_SCALE),
        ),
        |(r, height, probe)| {
            let mut m = Model::default();
            let body = sample::cylinder(&mut m, r, height)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            // The cylinder stands on the world origin along +Z.
            let radial = probe.x.hypot(probe.y) - r;
            let axial = (-probe.z).max(probe.z - height);
            let outside = radial.max(axial);
            if radial.abs() <= BAND || axial.abs() <= BAND {
                return Ok(());
            }
            let got = classify(&m, body, probe).map_err(|e| TestCaseError::fail(e.to_string()))?;
            let expected = if outside < 0.0 {
                Classification::Inside
            } else {
                Classification::Outside
            };
            prop_assert_eq!(got, expected, "{:?} against r {} h {}", probe, r, height);
            Ok(())
        },
    );
}

#[test]
fn a_point_on_a_face_an_edge_a_seam_or_a_vertex_names_the_entity() {
    let mut m = Model::default();
    let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let at = |p: Point3| classify(&m, cylinder, p).unwrap();
    // The wall, away from the seam and the rims.
    assert_eq!(
        kind_of(at(Point3::new(0.0, 4.0, 6.0))),
        Some(EntityKind::Face)
    );
    // The seam runs up the wall at u = 0; the edge answers before the
    // face it lies on.
    assert_eq!(
        kind_of(at(Point3::new(4.0, 0.0, 6.0))),
        Some(EntityKind::Edge)
    );
    // The bottom circle.
    assert_eq!(
        kind_of(at(Point3::new(0.0, -4.0, 0.0))),
        Some(EntityKind::Edge)
    );
    // Where the seam meets the bottom circle: the vertex, not either edge.
    assert_eq!(
        kind_of(at(Point3::new(4.0, 0.0, 0.0))),
        Some(EntityKind::Vertex)
    );
    // A cap, and its centre — inside the loop, not on it.
    assert_eq!(
        kind_of(at(Point3::new(1.0, 1.0, 12.0))),
        Some(EntityKind::Face)
    );
    assert_eq!(
        kind_of(at(Point3::new(0.0, 0.0, 0.0))),
        Some(EntityKind::Face)
    );

    let mut m = Model::default();
    let cuboid = sample::cuboid(&mut m, Point3::origin(), Point3::new(4.0, 3.0, 2.0)).unwrap();
    let at = |p: Point3| classify(&m, cuboid, p).unwrap();
    assert_eq!(
        kind_of(at(Point3::new(2.0, 1.5, 2.0))),
        Some(EntityKind::Face)
    );
    assert_eq!(
        kind_of(at(Point3::new(2.0, 0.0, 0.0))),
        Some(EntityKind::Edge)
    );
    assert_eq!(
        kind_of(at(Point3::new(0.0, 0.0, 0.0))),
        Some(EntityKind::Vertex)
    );
    assert_eq!(at(Point3::new(2.0, 1.5, 1.0)), Classification::Inside);
}

#[test]
fn a_point_in_a_hole_of_the_frame_is_outside() {
    let mut m = Model::default();
    let body = sample::frame(
        &mut m,
        Point3::origin(),
        Point3::new(40.0, 30.0, 10.0),
        Point2::new(10.0, 10.0),
        Point2::new(30.0, 20.0),
    )
    .unwrap();
    // The middle of the window, at every height through it.
    for z in [0.5, 5.0, 9.5] {
        assert_eq!(
            classify(&m, body, Point3::new(20.0, 15.0, z)).unwrap(),
            Classification::Outside,
            "the window at z = {z}"
        );
    }
    // The material either side of it, and above and below the window is
    // material too — the frame is a through window, so only the wall.
    assert_eq!(
        classify(&m, body, Point3::new(5.0, 15.0, 5.0)).unwrap(),
        Classification::Inside
    );
    assert_eq!(
        classify(&m, body, Point3::new(35.0, 15.0, 5.0)).unwrap(),
        Classification::Inside
    );
    // On the window's own wall.
    assert_eq!(
        kind_of(classify(&m, body, Point3::new(10.0, 15.0, 5.0)).unwrap()),
        Some(EntityKind::Face)
    );
}

#[test]
fn a_surface_a_ray_has_no_closed_form_against_is_a_typed_error() {
    // A NURBS face: far from the box, so the boundary test says nothing
    // and the ray cast is reached.
    let mut n = Model::default();
    let nurbs = sample::cuboid_nurbs(&mut n, Point3::origin(), Point3::new(2.0, 3.0, 4.0)).unwrap();
    match classify(&n, nurbs, Point3::new(50.0, 1.0, 2.0)) {
        Err(ClassifyError::Geometry(GeomError::Unsupported { .. })) => {}
        other => panic!("expected an unsupported ray–NURBS pair, got {other:?}"),
    }
    // A sphere has a closed form against a ray: its inside and outside
    // are cast for.
    let mut m = Model::default();
    let body = sample::sphere(&mut m, Point3::origin(), 3.0).unwrap();
    assert_eq!(
        classify(&m, body, Point3::new(50.0, 0.0, 0.0)).unwrap(),
        Classification::Outside
    );
    assert_eq!(
        classify(&m, body, Point3::new(0.5, -0.25, 1.0)).unwrap(),
        Classification::Inside
    );
    // And a point on it is still `On`, since that test never casts: the
    // seam meridian runs through `(R, 0, 0)`, and the poles are the
    // sphere sample's two vertices.
    assert_eq!(
        kind_of(classify(&m, body, Point3::new(3.0, 0.0, 0.0)).unwrap()),
        Some(EntityKind::Edge),
        "on the seam"
    );
    assert_eq!(
        kind_of(classify(&m, body, Point3::new(0.0, 0.0, -3.0)).unwrap()),
        Some(EntityKind::Vertex),
        "the south pole"
    );
    assert_eq!(
        kind_of(classify(&m, body, Point3::new(0.0, 3.0, 0.0)).unwrap()),
        Some(EntityKind::Face),
        "the surface away from the seam"
    );
}

#[test]
fn a_patch_is_not_a_solid_and_its_points_are_outside_or_on() {
    // A sheet has no inside; the ray cast crosses its one face an even
    // number of times, so every point off it is `Outside`.
    let mut m = Model::default();
    let plane = Surface::Plane {
        frame: Frame::world(),
    };
    let patch = sample::patch(
        &mut m,
        plane,
        arris_math::Interval::new(0.0, 4.0).unwrap(),
        arris_math::Interval::new(0.0, 3.0).unwrap(),
    )
    .unwrap();
    assert_eq!(
        kind_of(classify(&m, patch, Point3::new(2.0, 1.5, 0.0)).unwrap()),
        Some(EntityKind::Face)
    );
    assert_eq!(
        classify(&m, patch, Point3::new(2.0, 1.5, 1.0)).unwrap(),
        Classification::Outside
    );
}

#[test]
fn the_classification_of_a_body_that_does_not_resolve_is_an_error() {
    let m = Model::default();
    let ghost = arris_topo::Body::forward(arris_topo::BodyId::new(3, 0));
    assert!(matches!(
        classify(&m, ghost, Point3::origin()),
        Err(ClassifyError::Unresolved(_))
    ));
}

#[test]
fn a_probe_lands_where_the_corpus_fixtures_say() {
    // The two `primitive/*` fixtures' probe points, which the corpus
    // runner's probe stage holds against the oracle: repeated here so a
    // regression names the classifier rather than the runner.
    let mut m = Model::default();
    let body = sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let at = |p: [f64; 3]| classify(&m, body, Point3::new(p[0], p[1], p[2])).unwrap();
    assert_eq!(at([20.0, 15.0, 5.0]), Classification::Inside);
    assert_eq!(at([50.0, 15.0, 5.0]), Classification::Outside);
    assert_eq!(kind_of(at([20.0, 15.0, 10.0])), Some(EntityKind::Face));
    assert_eq!(kind_of(at([40.0, 15.0, 10.0])), Some(EntityKind::Edge));
    assert_eq!(kind_of(at([40.0, 30.0, 10.0])), Some(EntityKind::Vertex));
}
