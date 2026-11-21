//! The public tight-bounds query spends its control on edge subdivision.

use core::sync::atomic::{AtomicU64, Ordering};

use arris_check::{Level, check};
use arris_geom::{Curve, NurbsCurve};
use arris_math::{Aabb, Control, Interrupted, Interval, Point3, Stop};
use arris_ops::{OpError, query::tight_bounds};
use arris_topo::entity::{Body as BodyEntity, BodyKind, Edge, EdgeGeometry, Vertex};
use arris_topo::{Body, Model};

#[test]
fn tight_bounds_meters_a_nurbs_edge_split() {
    let mut m = Model::default();
    let range = Interval::UNIT;
    let a = Point3::origin();
    let b = Point3::new(1.0, 0.0, 0.0);
    let curve = m.add_curve(Curve::Nurbs(
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![a, Point3::new(0.5, 2.0, 0.0), b],
            vec![1.0; 3],
        )
        .unwrap(),
    ));
    let tol = m.precision().default_tolerance;
    let mut raw = m.raw();
    let start = raw.add_vertex(Vertex::new(a, tol));
    let end = raw.add_vertex(Vertex::new(b, tol));
    let edge = raw.add_edge(Edge::new(
        EdgeGeometry::Curve { curve, range },
        start,
        end,
        tol,
    ));
    let body = Body::forward(raw.add_body(BodyEntity::new(
        BodyKind::Wire,
        vec![],
        vec![arris_topo::Edge::forward(edge)],
        vec![],
    )));
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");

    // Two vertices and the edge spend three steps. The hull rises to y = 2,
    // so it takes one split to certify the true maximum y = 1 at t = 1/2.
    let query = |control: &Control<'_>| tight_bounds(&m, edge.into(), control);
    let steps = 3;
    assert_eq!(
        query(&Control::budget(steps)),
        Err(OpError::Interrupted(Interrupted {
            by: Stop::Budget,
            steps
        }))
    );
    let calls = AtomicU64::new(0);
    let poll = || calls.fetch_add(1, Ordering::Relaxed) >= steps;
    assert_eq!(
        query(&Control::poll(&poll)),
        Err(OpError::Interrupted(Interrupted {
            by: Stop::Poll,
            steps
        }))
    );
    assert_eq!(calls.load(Ordering::Relaxed), steps + 1);
    let expected = Aabb::of_point(a).union(Aabb::of_point(Point3::new(1.0, 1.0, 0.0)));
    assert_eq!(query(&Control::budget(steps + 1)), Ok(Some(expected)));
}
