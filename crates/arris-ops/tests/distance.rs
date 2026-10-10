//! `query::distance` from a point or a vertex (ADR-0054): the closest point
//! on a vertex, an edge, a face and a solid, each naming the most specific
//! entity it lands on, and the refusals. The oracle's comparison is the
//! `query/distance-*` corpus; these are the closed forms.

use arris_debug::sample;
use arris_debug::unmetered::primitive_box;
use arris_math::{Control, Point3};
use arris_ops::query::{Distance, FootParam, Operand, distance};
use arris_ops::{OpError, QueryReason, Reason};
use arris_topo::{Body, EntityId, Model, Orientation, Shape};

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
