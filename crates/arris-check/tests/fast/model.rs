//! M1–M3 and the clean samples: the sample bodies are clean, a missing body is one M1 line, and the report is the same on two builds.

use arris_check::{Level, Quantity, Reference, Violation, check};
use arris_debug::sample;
use arris_debug::testing::cylinder;
use arris_geom::{Curve, Curve2, Surface};
use arris_math::{Frame, Frame2, Interval, Point3};
use arris_topo::entity::{
    Body as BodyEntity, BodyKind, Coedge, Edge, EdgeGeometry, Face, Loop, Shell, Vertex,
};
use arris_topo::{
    Body, BodyId, CurveId, EdgeId, Face as FaceHandle, Model, Orientation, Shell as ShellHandle,
};

use crate::support::*;

#[test]
fn the_sample_bodies_are_clean() {
    let mut m = Model::default();
    let bodies = [
        sample::unit_box(&mut m).unwrap(),
        box40(&mut m),
        cylinder(&mut m),
        sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap(),
    ];
    for body in bodies {
        for level in [Level::Fast, Level::Full] {
            let report = check(&m, body, level);
            assert!(report.is_ok(), "{body} at {level:?}:\n{report}");
        }
    }
}

#[test]
fn a_missing_body_is_one_m1_line() {
    let m = Model::default();
    let body = Body::forward(BodyId::new(3, 0));
    let report = check(&m, body, Level::Fast);
    assert_eq!(
        report.violations(),
        [Violation::Unresolved {
            from: body.id.into(),
            to: Reference::Entity(body.id.into()),
        }]
    );
}

#[test]
fn m1_a_dangling_curve_id() {
    let mut m = Model::default();
    let c = cylinder(&mut m);
    let ghost = CurveId::new(999, 0);
    let r = rebuild(&mut m, c, &keep_vertex, &keep_edge, &keep_face);
    // The seam with a curve that does not resolve: every geometric row
    // skips it, M1 names it.
    let r2 = rebuild(
        &mut m,
        r.body,
        &keep_vertex,
        &|i, e| {
            if i == 1 {
                edit_edge(
                    &e,
                    EdgeGeometry::Curve {
                        curve: ghost,
                        range: e.range(),
                    },
                )
            } else {
                e
            }
        },
        &keep_face,
    );
    let report = check(&m, r2.body, Level::Fast);
    assert_lines(&report, &[("M1", r2.edges[1].to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::Unresolved {
            from: r2.edges[1].into(),
            to: Reference::Geometry(ghost.into()),
        }
    );
}

#[test]
fn m1_a_dangling_face_in_a_shell_and_a_dangling_pcurve() {
    let mut m = Model::default();
    let c = cylinder(&mut m);
    let ghost_pcurve = arris_topo::Curve2Id::new(999, 0);
    let r = rebuild(&mut m, c, &keep_vertex, &keep_edge, &|i, f| {
        if i == 1 {
            let coedges = f.loops()[0]
                .coedges()
                .iter()
                .map(|c| Coedge::new(c.edge(), c.orientation(), ghost_pcurve))
                .collect();
            Face::new(f.surface(), vec![Loop::new(coedges)], f.tolerance())
        } else {
            f
        }
    });
    let report = check(&m, r.body, Level::Fast);
    assert_lines(&report, &[("M1", r.faces[1].to_string())]);
}

/// M2: a face appended before the edge its coedge names is not in the
/// edge's use index even once the edge exists.
#[test]
fn m2_a_forward_reference_is_not_indexed() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let plane = m.add_surface(Surface::Plane {
        frame: Frame::world(),
    });
    let circle = m.add_curve(Curve::Circle {
        frame: Frame::world(),
        radius: 2.0,
    });
    let pcurve = m.add_curve2(Curve2::Circle {
        frame: Frame2::identity(),
        radius: 2.0,
    });
    let v = m
        .raw()
        .add_vertex(Vertex::new(Point3::new(2.0, 0.0, 0.0), tol));
    let future_edge = EdgeId::new(0, 0);
    let face = m.raw().add_face(Face::new(
        plane,
        vec![Loop::new(vec![Coedge::new(
            future_edge,
            Orientation::Forward,
            pcurve,
        )])],
        tol,
    ));
    let edge = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: circle,
            range: Interval::TURN,
        },
        v,
        v,
        tol,
    ));
    assert_eq!(edge, future_edge);
    let shell = m
        .raw()
        .add_shell(Shell::new(vec![FaceHandle::forward(face)]));
    let body = m.raw().add_body(BodyEntity::new(
        BodyKind::Sheet,
        vec![ShellHandle::forward(shell)],
        Vec::new(),
        Vec::new(),
    ));
    let report = check(&m, Body::forward(body), Level::Fast);
    assert_eq!(
        report.violations(),
        [Violation::NotIndexed {
            entity: edge.into(),
            referenced_by: face.into(),
        }],
        "{report}"
    );
}

#[test]
fn m3_a_nan_coordinate_and_a_nan_tolerance() {
    let mut m = Model::default();
    let b = box40(&mut m);
    let r = rebuild(
        &mut m,
        b,
        &|i, v| match i {
            0 => Vertex::new(Point3::new(f64::NAN, 0.0, 0.0), v.tolerance()),
            1 => Vertex::new(v.point(), f64::INFINITY),
            _ => v,
        },
        &keep_edge,
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    assert_lines(
        &report,
        &[
            ("M3", r.vertices[0].to_string()),
            ("M3", r.vertices[1].to_string()),
        ],
    );
    assert!(matches!(
        report.violations()[0],
        Violation::NonFinite {
            quantity: Quantity::Coordinate,
            ..
        }
    ));
    assert!(matches!(
        report.violations()[1],
        Violation::NonFinite {
            quantity: Quantity::Tolerance,
            ..
        }
    ));
}

#[test]
fn the_report_is_the_same_on_two_builds() {
    let build = |m: &mut Model| {
        let c = cylinder(m);
        let r = rebuild(
            m,
            c,
            &|i, v| {
                if i == 1 {
                    Vertex::new(
                        v.point() + arris_math::Vec3::new(0.0, 0.0, 1.0),
                        v.tolerance(),
                    )
                } else {
                    v
                }
            },
            &keep_edge,
            &keep_face,
        );
        check(m, r.body, Level::Fast).to_string()
    };
    let (mut a, mut b) = (Model::default(), Model::default());
    let text = build(&mut a);
    assert_eq!(text, build(&mut b));
    assert!(text.lines().count() >= 3, "{text}");
}

#[test]
fn a_plate_with_a_hole_is_clean_and_so_are_the_samples() {
    let mut m = Model::default();
    let (body, _, _) = plate(
        &mut m,
        &[
            rect(0.0, 0.0, 10.0, 10.0, true),
            rect(2.0, 2.0, 8.0, 8.0, false),
        ],
    );
    let report = check(&m, body, Level::Fast);
    assert!(report.is_ok(), "{report}");
    for body in [
        sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap(),
        sample::cylinder(&mut m, 4.0, 12.0).unwrap(),
        sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap(),
    ] {
        let report = check(&m, body, Level::Fast);
        assert!(report.is_ok(), "{body}:\n{report}");
    }
}
