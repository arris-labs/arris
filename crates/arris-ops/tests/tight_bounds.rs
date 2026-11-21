//! Certification and cancellation through the public tight-bounds query.

use core::sync::atomic::{AtomicU64, Ordering};

use arris_check::{Level, check};
use arris_debug::sample;
use arris_geom::{Curve, Curve2, NurbsCurve, NurbsCurve2, NurbsSurface, Surface};
use arris_math::{
    Aabb, Control, Frame, Interrupted, Interval, Point2, Point3, Precision, Stop, Vec3, real,
};
use arris_ops::{OpError, query::tight_bounds};
use arris_topo::entity::{
    Body as BodyEntity, BodyKind, Coedge, Edge, EdgeGeometry, Face, Loop, Shell, Vertex,
};
use arris_topo::{Body, Face as FaceHandle, Model, Orientation, Shell as ShellHandle};

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

#[test]
fn tight_bounds_keep_peak_inside_a_sharply_parameterized_rational_trim() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let range = Interval::UNIT;
    // z = v(u(1-u) - v) vanishes on both edges of the trim. Its interior
    // maximum is 1/64, at (u, v) = (1/2, 1/8).
    let points = [0.0, 0.5, 0.0]
        .into_iter()
        .enumerate()
        .flat_map(|(i, a)| {
            [0.0, 0.5, 1.0].into_iter().enumerate().map(move |(j, v)| {
                let v_squared = if j == 2 { 1.0 } else { 0.0 };
                Point3::new(i as f64 / 2.0, v, a * v - v_squared)
            })
        })
        .collect();
    let surface = m.add_surface(Surface::Nurbs(
        NurbsSurface::new(
            [2, 2],
            [(); 2].map(|()| vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]),
            points,
            vec![1.0; 9],
        )
        .unwrap(),
    ));

    // The upper arc is (s, s(1-s)), where s = t³/(d + (1-d)t³).
    // Most of it is traversed between t=0 and the first uniform sample.
    // Express W², t³W and t³(W-t³) in the degree-six Bernstein basis.
    let d = 1e-10;
    let t3 = [0.0, 0.0, 0.0, 1.0 / 20.0, 4.0 / 20.0, 10.0 / 20.0, 1.0];
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for (i, t3) in t3.into_iter().enumerate() {
        let t6 = if i == 6 { 1.0 } else { 0.0 };
        let w = d * d + 2.0 * d * (1.0 - d) * t3 + (1.0 - d) * (1.0 - d) * t6;
        points.push(Point2::new(
            (d * t3 + (1.0 - d) * t6) / w,
            d * (t3 - t6) / w,
        ));
        weights.push(w);
    }
    let knots = [[0.0; 7], [1.0; 7]].concat();
    let arc = m.add_curve(Curve::Nurbs(
        NurbsCurve::new(
            6,
            knots.clone(),
            points.iter().map(|p| Point3::new(p.x, p.y, 0.0)).collect(),
            weights.clone(),
        )
        .unwrap(),
    ));
    let arc_uv = m.add_curve2(Curve2::Nurbs(
        NurbsCurve2::new(6, knots, points, weights).unwrap(),
    ));
    let line = m.add_curve(Curve::Line {
        origin: Point3::origin(),
        direction: Vec3::x_axis(),
    });
    let line_uv = m.add_curve2(Curve2::along_u(0.0));
    let mut raw = m.raw();
    let a = raw.add_vertex(Vertex::new(Point3::origin(), tol));
    let b = raw.add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), tol));
    let line = raw.add_edge(Edge::new(
        EdgeGeometry::Curve { curve: line, range },
        a,
        b,
        tol,
    ));
    let arc = raw.add_edge(Edge::new(
        EdgeGeometry::Curve { curve: arc, range },
        a,
        b,
        tol,
    ));
    let face = raw.add_face(Face::new(
        surface,
        vec![Loop::new(vec![
            Coedge::new(line, Orientation::Forward, line_uv),
            Coedge::new(arc, Orientation::Reversed, arc_uv),
        ])],
        tol,
    ));
    let shell = raw.add_shell(Shell::new(vec![FaceHandle::forward(face)]));
    let body = Body::forward(raw.add_body(BodyEntity::new(
        BodyKind::Sheet,
        vec![ShellHandle::forward(shell)],
        vec![],
        vec![],
    )));
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");

    let b = tight_bounds(&m, body.id.into(), &Control::NONE)
        .unwrap()
        .unwrap();
    assert!((1.0 / 64.0..=1.0 / 64.0 + tol).contains(&b.max[2]), "{b:?}");
}

#[test]
fn tight_bounds_exclude_a_sphere_stationary_point_in_the_old_boundary_band() {
    let precision = Precision {
        parametric_tolerance: 0.1,
        ..Precision::DEFAULT
    };
    let mut m = Model::new(precision).unwrap();
    let u_min = 0.025;
    let body = sample::patch(
        &mut m,
        Surface::Sphere {
            frame: Frame::world(),
            radius: 1.0,
        },
        Interval::new(u_min, 0.3).unwrap(),
        Interval::new(-0.3, 0.3).unwrap(),
    )
    .unwrap();
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");

    // The removed stationary point at (u, v) = (0, 0) has x = 1.
    // The face only reaches x = cos(u_min), along its first meridian.
    let b = tight_bounds(&m, body.id.into(), &Control::NONE)
        .unwrap()
        .unwrap();
    let expected = real::cos(u_min);
    assert!(
        b.max[0] >= expected && b.max[0] <= expected + precision.default_tolerance,
        "{b:?}"
    );
}
