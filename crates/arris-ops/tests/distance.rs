//! `query::distance` from a point or a vertex (ADR-0054): the closest point
//! on a vertex, an edge, a face and a solid, each naming the most specific
//! entity it lands on, and the refusals. The oracle's comparison is the
//! `query/distance-*` corpus; these are the closed forms.

use arris_debug::sample;
use arris_debug::unmetered::{primitive_box, revolve};
use arris_geom::{Profile, ProfileLoop, ProfileSegment};
use arris_math::{Axis, Control, Frame, Point2, Point3, Vec3};
use arris_ops::query::{Distance, FootParam, Operand, distance};
use arris_ops::{OpError, QueryReason, Reason};
use arris_topo::{Body, EntityId, Model, Orientation, Shape};
use core::f64::consts::TAU;

const EXACT: f64 = 1e-12;

/// The box `[0, 4] × [0, 3] × [0, 2]`.
fn cuboid() -> (Model, Body) {
    let mut m = Model::default();
    let (body, _) = primitive_box(&mut m, Point3::origin(), Point3::new(4.0, 3.0, 2.0)).unwrap();
    (m, body)
}

fn whole(b: Body) -> Operand {
    Operand::Shape(Shape::new(b.id, b.orientation))
}

fn from(m: &Model, p: [f64; 3], to: Operand) -> Distance {
    distance(
        m,
        Operand::Point(Point3::new(p[0], p[1], p[2])),
        to,
        &Control::NONE,
    )
    .unwrap()
}

#[test]
fn a_point_over_a_face_lands_in_its_middle_and_past_a_corner_on_the_vertex() {
    let (m, body) = cuboid();
    let d = from(&m, [1.0, 1.0, 5.0], whole(body));
    assert!((d.distance - 3.0).abs() < EXACT);
    assert!(matches!(d.b.shape.unwrap().id, EntityId::Face(_)));
    assert!(matches!(d.b.param, FootParam::Face(_)));
    assert!((d.b.point - Point3::new(1.0, 1.0, 2.0)).norm() < EXACT);
    assert!(d.unique && !d.contained && d.a.shape.is_none());

    let d = from(&m, [5.0, 4.0, 3.0], whole(body));
    assert!((d.distance - 3f64.sqrt()).abs() < EXACT);
    assert!(matches!(d.b.shape.unwrap().id, EntityId::Vertex(_)));
    assert!((d.b.point - Point3::new(4.0, 3.0, 2.0)).norm() < EXACT);

    // Beside an edge: on the edge, at its parameter.
    let d = from(&m, [2.0, 4.0, 3.0], whole(body));
    assert!((d.distance - 2f64.sqrt()).abs() < EXACT);
    assert!(matches!(d.b.shape.unwrap().id, EntityId::Edge(_)));
    assert!(matches!(d.b.param, FootParam::Edge(_)));
}

#[test]
fn a_point_inside_a_solid_is_contained_at_zero_and_on_its_wall_is_not() {
    let (m, body) = cuboid();
    let d = from(&m, [1.0, 1.0, 1.0], whole(body));
    assert!(d.contained && d.distance == 0.0 && d.unique);
    let d = from(&m, [1.0, 1.0, 2.0], whole(body));
    assert!(!d.contained && d.distance < 1e-9);
}

#[test]
fn a_point_against_an_entity_alone_is_that_entitys_closest() {
    let (m, body) = cuboid();
    let faces = m.closure(body).unwrap();
    let top = faces
        .faces
        .iter()
        .copied()
        .find(|&f| {
            let d = from(
                &m,
                [1.0, 1.0, 5.0],
                Operand::Shape(Shape::new(f, Orientation::Forward)),
            );
            (d.distance - 3.0).abs() < 1e-9
        })
        .expect("a face 3 under the point");
    // The same face, from a point beside it: the foot falls outside the
    // trim, so it is on an edge.
    let d = from(
        &m,
        [6.0, 1.0, 5.0],
        Operand::Shape(Shape::new(top, Orientation::Forward)),
    );
    assert!((d.distance - (4.0f64 + 9.0).sqrt()).abs() < EXACT);
    assert!(matches!(d.b.shape.unwrap().id, EntityId::Edge(_)));

    let edge = faces.edges[0];
    let e = m.edge(edge).unwrap();
    let (start, end) = (
        m.vertex(e.start()).unwrap().point(),
        m.vertex(e.end()).unwrap().point(),
    );
    let mid = Point3::from((start.coords + end.coords) / 2.0);
    let d = distance(
        &m,
        Operand::Point(mid),
        Operand::Shape(Shape::new(edge, Orientation::Forward)),
        &Control::NONE,
    )
    .unwrap();
    assert!(d.distance < EXACT);
    assert!(matches!(d.b.param, FootParam::Edge(_)));

    // A vertex against a vertex.
    let v0 = faces.vertices[0];
    let d = distance(
        &m,
        Operand::Shape(Shape::new(v0, Orientation::Forward)),
        Operand::Point(Point3::new(0.0, 0.0, 10.0)),
        &Control::NONE,
    )
    .unwrap();
    assert!(d.a.shape.is_some() && d.b.shape.is_none());
}

#[test]
fn a_point_on_a_cylinders_axis_is_not_unique_against_its_wall() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 2.0, 6.0).unwrap();
    let wall = m
        .closure(body)
        .unwrap()
        .faces
        .into_iter()
        .find(|&f| {
            let d = from(
                &m,
                [0.0, 0.0, 3.0],
                Operand::Shape(Shape::new(f, Orientation::Forward)),
            );
            (d.distance - 2.0).abs() < 1e-9
        })
        .expect("the wall is 2 from the axis");
    let d = from(
        &m,
        [0.0, 0.0, 3.0],
        Operand::Shape(Shape::new(wall, Orientation::Forward)),
    );
    assert!(!d.unique, "every ruling is as near");
    // Inside the solid it is contained; above its cap it is on the cap.
    assert!(from(&m, [0.0, 0.0, 3.0], whole(body)).contained);
    let d = from(&m, [0.0, 0.0, 9.0], whole(body));
    assert!((d.distance - 3.0).abs() < 1e-9 && d.unique);
}

#[test]
fn a_ball_is_measured_from_outside_and_from_its_centre() {
    let mut m = Model::default();
    let body = sample::sphere(&mut m, Point3::new(1.0, 2.0, 3.0), 2.0).unwrap();
    let d = from(&m, [1.0, 2.0, 8.0], whole(body));
    assert!((d.distance - 3.0).abs() < 1e-9);
    assert!(d.unique);
    let d = from(&m, [1.0, 2.0, 3.0], whole(body));
    assert!(d.contained && d.distance == 0.0);
}

#[test]
fn a_shell_and_a_non_finite_point_are_refused() {
    let (m, body) = cuboid();
    let shell = m.body(body.id).unwrap().shells()[0].id;
    let e = distance(
        &m,
        Operand::Point(Point3::origin()),
        Operand::Shape(Shape::new(shell, Orientation::Forward)),
        &Control::NONE,
    )
    .unwrap_err();
    assert!(matches!(
        e,
        OpError::Degenerate {
            reason: Reason::Query(QueryReason::NotMeasurable),
            ..
        }
    ));
    let e = distance(
        &m,
        Operand::Point(Point3::new(f64::NAN, 0.0, 0.0)),
        whole(body),
        &Control::NONE,
    )
    .unwrap_err();
    assert!(matches!(e, OpError::Degenerate { .. }));
}

#[test]
fn a_stopped_control_interrupts() {
    let (m, body) = cuboid();
    let e = distance(
        &m,
        Operand::Point(Point3::new(1.0, 1.0, 5.0)),
        whole(body),
        &Control::budget(0),
    )
    .unwrap_err();
    assert!(matches!(e, OpError::Interrupted(_)));
}

/// The entity of `kind` nearest `at` among a body's, by the distance query
/// itself.
fn nearest_entity(m: &Model, body: Body, at: [f64; 3], kind: &str) -> Shape {
    let closure = m.closure(body).unwrap();
    let shapes: Vec<Shape> = match kind {
        "face" => closure
            .faces
            .iter()
            .map(|&f| Shape::new(f, Orientation::Forward))
            .collect(),
        "edge" => closure
            .edges
            .iter()
            .map(|&e| Shape::new(e, Orientation::Forward))
            .collect(),
        _ => closure
            .vertices
            .iter()
            .map(|&v| Shape::new(v, Orientation::Forward))
            .collect(),
    };
    let p = Operand::Point(Point3::new(at[0], at[1], at[2]));
    shapes
        .into_iter()
        .min_by(|x, y| {
            let d = |s: &Shape| {
                distance(m, p, Operand::Shape(*s), &Control::NONE)
                    .unwrap()
                    .distance
            };
            d(x).total_cmp(&d(y))
        })
        .unwrap()
}

fn between(m: &Model, a: Shape, b: Shape) -> Distance {
    distance(m, Operand::Shape(a), Operand::Shape(b), &Control::NONE).unwrap()
}

fn two_boxes(a: ([f64; 3], [f64; 3]), b: ([f64; 3], [f64; 3])) -> (Model, Body, Body) {
    let mut m = Model::default();
    let p = |v: [f64; 3]| Point3::new(v[0], v[1], v[2]);
    let (x, _) = primitive_box(&mut m, p(a.0), p(a.1)).unwrap();
    let (y, _) = primitive_box(&mut m, p(b.0), p(b.1)).unwrap();
    (m, x, y)
}

#[test]
fn skew_edges_meet_at_their_interior_feet_and_parallel_ones_are_a_continuum() {
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([1.0, 1.5, 4.0], [6.0, 8.0, 6.0]),
    );
    // A's edge along y at x = 4, z = 2 against B's along x at y = 1.5, z = 4.
    let ea = nearest_entity(&m, a, [4.0, 1.5, 2.0], "edge");
    let eb = nearest_entity(&m, b, [3.5, 1.5, 4.0], "edge");
    let d = between(&m, ea, eb);
    assert!((d.distance - 2.0).abs() < EXACT);
    assert!((d.a.point - Point3::new(4.0, 1.5, 2.0)).norm() < EXACT);
    assert!((d.b.point - Point3::new(4.0, 1.5, 4.0)).norm() < EXACT);
    assert!(d.unique);
    assert!(matches!(d.a.param, FootParam::Edge(_)) && matches!(d.b.param, FootParam::Edge(_)));
    let back = between(&m, eb, ea);
    assert!((back.distance - 2.0).abs() < EXACT && (back.a.point - d.b.point).norm() < EXACT);

    // Two parallel top edges, 5 apart in y.
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([0.0, 8.0, 0.0], [4.0, 9.0, 2.0]),
    );
    let ea = nearest_entity(&m, a, [2.0, 3.0, 2.0], "edge");
    let eb = nearest_entity(&m, b, [2.0, 8.0, 2.0], "edge");
    let d = between(&m, ea, eb);
    assert!((d.distance - 5.0).abs() < EXACT);
    assert!(!d.unique, "every pair of facing points is as near");
}

#[test]
fn facing_plates_are_a_continuum_and_offset_ones_meet_at_an_edge() {
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([6.0, 0.0, 0.0], [9.0, 3.0, 2.0]),
    );
    let fa = nearest_entity(&m, a, [4.0, 1.5, 1.0], "face");
    let fb = nearest_entity(&m, b, [6.0, 1.5, 1.0], "face");
    let d = between(&m, fa, fb);
    assert!((d.distance - 2.0).abs() < EXACT);
    assert!(!d.unique);

    // Offset along x and z: the nearest points are the facing edges.
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([6.0, 0.0, 5.0], [9.0, 3.0, 7.0]),
    );
    let top = nearest_entity(&m, a, [2.0, 1.5, 2.0], "face");
    let floor = nearest_entity(&m, b, [7.0, 1.5, 5.0], "face");
    let d = between(&m, top, floor);
    assert!((d.distance - 13f64.sqrt()).abs() < EXACT);
    // The whole facing edge is as near; the pick is its lowest end, the
    // most specific entity there.
    assert!(matches!(d.a.shape.unwrap().id, EntityId::Vertex(_)));
    assert!(matches!(d.b.shape.unwrap().id, EntityId::Vertex(_)));
    assert!(!d.unique, "the whole facing edge");
    let back = between(&m, floor, top);
    assert!((back.distance - d.distance).abs() < EXACT);
}

#[test]
fn an_edge_against_a_face_alongside_and_across() {
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([1.0, 1.0, 5.0], [3.0, 2.0, 7.0]),
    );
    // B's bottom edge along x at y = 1, z = 5 hovers 3 above A's top face.
    let eb = nearest_entity(&m, b, [2.0, 1.0, 5.0], "edge");
    let top = nearest_entity(&m, a, [2.0, 1.5, 2.0], "face");
    let d = between(&m, eb, top);
    assert!((d.distance - 3.0).abs() < EXACT);
    assert!(!d.unique);
    let back = between(&m, top, eb);
    assert!((back.distance - 3.0).abs() < EXACT);
    assert!(matches!(back.a.shape.unwrap().id, EntityId::Face(_)));
}

#[test]
fn a_ball_resting_on_a_plate_touches_it_at_one_point() {
    let mut m = Model::default();
    let ball = sample::sphere(&mut m, Point3::new(0.0, 0.0, 1.0), 1.0).unwrap();
    let (plate, _) = primitive_box(
        &mut m,
        Point3::new(-5.0, -5.0, -1.0),
        Point3::new(5.0, 5.0, 0.0),
    )
    .unwrap();
    let surface = nearest_entity(&m, ball, [0.0, 0.0, 0.0], "face");
    let top = nearest_entity(&m, plate, [0.0, 0.0, 0.0], "face");
    let d = between(&m, surface, top);
    assert!(d.distance < 1e-9, "the contact is inside both faces");
    assert!((d.a.point - Point3::origin()).norm() < 1e-6);
}

#[test]
fn a_ball_above_a_plate_is_a_ball_radius_closer_than_its_centre() {
    let mut m = Model::default();
    let ball = sample::sphere(&mut m, Point3::new(0.0, 0.0, 4.0), 1.0).unwrap();
    let (plate, _) = primitive_box(
        &mut m,
        Point3::new(-5.0, -5.0, -1.0),
        Point3::new(5.0, 5.0, 0.0),
    )
    .unwrap();
    let surface = nearest_entity(&m, ball, [0.0, 0.0, 3.0], "face");
    let top = nearest_entity(&m, plate, [0.0, 0.0, 0.0], "face");
    let d = between(&m, surface, top);
    assert!((d.distance - 3.0).abs() < 1e-9);
    assert!(d.unique);
}

fn solids(m: &Model, a: Body, b: Body) -> Distance {
    between(
        m,
        Shape::new(a.id, a.orientation),
        Shape::new(b.id, b.orientation),
    )
}

#[test]
fn two_boxes_apart_are_at_their_gap_and_touching_ones_at_zero() {
    // Apart in x and z: the nearest are the facing edges, √(3² + 4²).
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([7.0, 0.0, 6.0], [9.0, 3.0, 8.0]),
    );
    let d = solids(&m, a, b);
    assert!((d.distance - 5.0).abs() < EXACT);
    assert!(!d.contained);
    assert!((d.a.point - Point3::new(4.0, d.a.point.y, 2.0)).norm() < EXACT);
    assert!((d.b.point - Point3::new(7.0, d.b.point.y, 6.0)).norm() < EXACT);
    assert!(
        !d.unique,
        "a whole edge faces an edge: a continuum of nearest pairs"
    );
    // The same answer with the operands swapped, feet swapped with them.
    let e = solids(&m, b, a);
    assert!((e.distance - d.distance).abs() < EXACT);
    assert!((e.a.point - d.b.point).norm() < EXACT && (e.b.point - d.a.point).norm() < EXACT);

    // Apart along one axis only: a face over a face.
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([1.0, 1.0, 5.0], [3.0, 2.0, 8.0]),
    );
    let d = solids(&m, a, b);
    assert!((d.distance - 3.0).abs() < EXACT);
    assert!(matches!(d.a.shape.unwrap().id, EntityId::Face(_)));

    // Face against face, touching.
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([4.0, 0.0, 0.0], [6.0, 3.0, 2.0]),
    );
    let d = solids(&m, a, b);
    assert!(d.distance < 1e-9 && !d.contained);
}

#[test]
fn crossing_boxes_are_at_zero_and_one_inside_another_is_contained() {
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([3.0, 1.0, 1.0], [7.0, 2.0, 5.0]),
    );
    // A vertex of one inside the other: at zero, and contained (Open
    // Cascade's `InnerSolution`).
    let d = solids(&m, a, b);
    assert!(d.distance == 0.0 && d.contained);

    // Crossing like a plus with no vertex inside either: at zero by
    // contact, not contained.
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [10.0, 2.0, 2.0]),
        ([4.0, -3.0, -3.0], [6.0, 5.0, 5.0]),
    );
    let d = solids(&m, a, b);
    assert!(d.distance < 1e-9 && !d.contained);

    // Wholly inside: the same, in either order, and the inner operand's
    // point is the foot.
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]),
        ([2.0, 3.0, 4.0], [5.0, 6.0, 7.0]),
    );
    for (x, y) in [(a, b), (b, a)] {
        let d = solids(&m, x, y);
        assert!(d.contained && d.distance == 0.0);
        assert!((d.a.point - d.b.point).norm() < EXACT);
    }
    let d = solids(&m, a, b);
    assert!(d.a.shape.is_none() && d.b.shape.is_some());
}

#[test]
fn a_solid_against_an_edge_or_a_face_of_another() {
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([7.0, 1.0, 1.0], [9.0, 2.0, 5.0]),
    );
    let face = nearest_entity(&m, b, [7.0, 1.5, 3.0], "face");
    let d = distance(
        &m,
        Operand::Shape(Shape::new(a.id, a.orientation)),
        Operand::Shape(face),
        &Control::NONE,
    )
    .unwrap();
    assert!((d.distance - 3.0).abs() < EXACT);
    // Facing faces are a continuum, answered on the most specific entity
    // of the one that is smaller.
    assert!(d.b.shape.is_some() && !d.unique);
    // And the other way round.
    let e = distance(
        &m,
        Operand::Shape(face),
        Operand::Shape(Shape::new(a.id, a.orientation)),
        &Control::NONE,
    )
    .unwrap();
    assert!((e.distance - 3.0).abs() < EXACT);
    assert!(e.a.shape.is_some() && !e.unique);

    // A face of the second box inside the first is contained.
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]),
        ([2.0, 3.0, 4.0], [5.0, 6.0, 7.0]),
    );
    let inner = nearest_entity(&m, b, [3.0, 4.0, 7.0], "edge");
    let d = distance(
        &m,
        Operand::Shape(Shape::new(a.id, a.orientation)),
        Operand::Shape(inner),
        &Control::NONE,
    )
    .unwrap();
    assert!(d.contained && d.distance == 0.0);
}

#[test]
fn a_ball_against_a_cylinder_and_a_box() {
    let mut m = Model::default();
    let ball = sample::sphere(&mut m, Point3::new(10.0, 0.0, 3.0), 1.5).unwrap();
    let pin = sample::cylinder(&mut m, 2.0, 6.0).unwrap();
    let d = solids(&m, ball, pin);
    // The pin's wall is 2 from its axis, the ball's centre 10 from it.
    assert!((d.distance - (10.0 - 2.0 - 1.5)).abs() < 1e-9);
    assert!(!d.contained);
}

#[test]
fn a_body_far_from_the_other_is_walked_in_a_handful_of_pairs() {
    // Offset on every axis, only the three faces at each nearest corner
    // can reach the answer: 9 of the 36 face pairs, about 1900 ticks, where
    // all of them cost about 6300. The budget runs out on the full set.
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([40.0, 30.0, 20.0], [44.0, 33.0, 22.0]),
    );
    let d = distance(
        &m,
        Operand::Shape(Shape::new(a.id, a.orientation)),
        Operand::Shape(Shape::new(b.id, b.orientation)),
        &Control::budget(3000),
    )
    .unwrap();
    let corner = (Point3::new(4.0, 3.0, 2.0) - Point3::new(40.0, 30.0, 20.0)).norm();
    assert!((d.distance - corner).abs() < EXACT);
    assert!(d.unique && matches!(d.a.shape.unwrap().id, EntityId::Vertex(_)));
}

#[test]
fn a_stopped_control_interrupts_a_body_pair() {
    let (m, a, b) = two_boxes(
        ([0.0, 0.0, 0.0], [4.0, 3.0, 2.0]),
        ([7.0, 0.0, 6.0], [9.0, 3.0, 8.0]),
    );
    let e = distance(
        &m,
        Operand::Shape(Shape::new(a.id, a.orientation)),
        Operand::Shape(Shape::new(b.id, b.orientation)),
        &Control::budget(0),
    )
    .unwrap_err();
    assert!(matches!(e, OpError::Interrupted(_)));
}

/// The frustum of `sweep/revolve-frustum`, about the axis through `at`.
fn frustum(m: &mut Model, at: Point3) -> Body {
    let p = |u, v| Point2::new(u, v);
    let profile = Profile {
        plane: Frame::new(at, -Vec3::y(), Vec3::x()).unwrap(),
        outer: ProfileLoop::Path {
            start: p(1.0, -1.0),
            segments: vec![
                ProfileSegment::LineTo(p(4.0, -1.0)),
                ProfileSegment::LineTo(p(2.0, 1.0)),
                ProfileSegment::LineTo(p(1.0, 1.0)),
                ProfileSegment::LineTo(p(1.0, -1.0)),
            ],
        },
        holes: Vec::new(),
    };
    revolve(m, &profile, Axis::z_at(at), TAU).unwrap().0
}

#[test]
fn two_cones_off_their_common_axis_are_refused_naming_both_faces() {
    let mut m = Model::default();
    let a = frustum(&mut m, Point3::origin());
    let b = frustum(&mut m, Point3::new(20.0, 0.0, 0.0));
    let e = distance(
        &m,
        Operand::Shape(Shape::new(a.id, a.orientation)),
        Operand::Shape(Shape::new(b.id, b.orientation)),
        &Control::NONE,
    )
    .unwrap_err();
    let OpError::Unsupported {
        a: (_, x),
        b: (_, y),
    } = e
    else {
        panic!("expected Unsupported, got {e:?}");
    };
    assert!(matches!(x.id, EntityId::Face(_)) && matches!(y.id, EntityId::Face(_)));
    // Against a box the same frustum answers: cone against plane.
    let (cube, _) = primitive_box(
        &mut m,
        Point3::new(10.0, -5.0, -1.0),
        Point3::new(12.0, 5.0, 1.0),
    )
    .unwrap();
    let d = solids(&m, a, cube);
    assert!((d.distance - 6.0).abs() < 1e-9, "{}", d.distance);
}

#[test]
fn a_ball_wholly_inside_a_block_is_contained_and_one_beside_it_is_not() {
    let mut m = Model::default();
    let (block, _) = primitive_box(
        &mut m,
        Point3::new(-5.0, -5.0, -5.0),
        Point3::new(5.0, 5.0, 5.0),
    )
    .unwrap();
    let inner = sample::sphere(&mut m, Point3::origin(), 2.0).unwrap();
    let outer = sample::sphere(&mut m, Point3::new(10.0, 0.0, 0.0), 2.0).unwrap();
    // No vertex of the ball to classify: its closest point does.
    for (x, y) in [(block, inner), (inner, block)] {
        let d = solids(&m, x, y);
        assert!(d.contained && d.distance == 0.0);
    }
    let d = solids(&m, block, outer);
    assert!(!d.contained && (d.distance - 3.0).abs() < 1e-9);
}
