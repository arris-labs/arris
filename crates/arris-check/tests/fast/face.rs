//! F1–F2.

use arris_check::{FaceFault, Level, ToleranceBound, Violation, check};
use arris_geom::{Curve, Curve2, Surface};
use arris_math::{Frame, Interval, Point2, Point3, UnitVec2, Vec3, real};
use arris_topo::entity::{Coedge, Edge, EdgeGeometry, Loop, Vertex};
use arris_topo::{Model, Orientation};

use crate::support::*;

#[test]
fn f1_a_face_without_loops() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let (body, face, _) = one_face(&mut m, surface, Vec::new(), tol);
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("F1", face.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::FaceMalformed {
            face,
            fault: FaceFault::NoLoops,
        }
    );
}

/// F1: the same plane as a NURBS patch of knot range `[0, 1]²`, with a
/// loop that walks a square of side two. The patch extrapolates to the
/// same points, so E4 and V3 hold; what is wrong is that the face claims
/// (u, v) its surface does not have.
#[test]
fn f1_a_pcurve_outside_the_surfaces_domain() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground_patch(&mut m);
    let r = ring(&mut m, &rect(0.0, 0.0, 2.0, 2.0, true));
    let (body, face, _) = one_face(&mut m, surface, vec![Loop::new(r.coedges.clone())], tol);
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("F1", face.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::FaceMalformed {
            face,
            fault: FaceFault::PcurveOutsideDomain {
                loop_index: 0,
                coedge: 0,
            },
        }
    );
}

#[test]
fn f2_a_face_tolerance_below_the_floor() {
    let mut m = Model::default();
    let precision = m.precision();
    let surface = ground(&mut m);
    let r = ring(&mut m, &rect(0.0, 0.0, 10.0, 10.0, true));
    let (body, face, _) = one_face(
        &mut m,
        surface,
        vec![Loop::new(r.coedges.clone())],
        precision.min_tolerance / 10.0,
    );
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("F2", face.to_string())]);
    assert!(matches!(
        report.violations()[0],
        Violation::FaceTolerance {
            bound: ToleranceBound::BelowMinimum,
            ..
        }
    ));
}

/// F2 and E5 are the same ordering read from the two sides, so a face
/// coarser than its edges is reported on the face and on every edge.
#[test]
fn f2_a_face_tolerance_above_its_edges() {
    let mut m = Model::default();
    let precision = m.precision();
    let surface = ground(&mut m);
    let r = ring(&mut m, &[p2(0.0, 0.0), p2(10.0, 0.0), p2(0.0, 10.0)]);
    let (body, face, _) = one_face(
        &mut m,
        surface,
        vec![Loop::new(r.coedges.clone())],
        precision.default_tolerance * 10.0,
    );
    let report = check(&m, body, Level::Fast);
    let f = face.to_string();
    assert_lines(
        &report,
        &[
            ("E5", r.edges[0].to_string()),
            ("E5", r.edges[1].to_string()),
            ("E5", r.edges[2].to_string()),
            ("F2", f.clone()),
            ("F2", f.clone()),
            ("F2", f),
        ],
    );
    for v in &report.violations()[3..] {
        assert!(matches!(
            v,
            Violation::FaceTolerance {
                bound: ToleranceBound::Neighbour { .. },
                ..
            }
        ));
    }
}

/// F1: a horn torus (`R = r`) face whose one pcurve runs along `v` from
/// 0 to `v_end`. At `v_end = π` the surface point is on the axis, past the
/// regular band (ADR-0080); short of it the face is on the band.
fn horn_meridian_face(v_end: f64) -> (Model, arris_check::Report, arris_topo::FaceId) {
    horn_meridian_span(0.0, v_end)
}

/// The same face over `v` from `v_start` to `v_end`.
fn horn_meridian_span(
    v_start: f64,
    v_end: f64,
) -> (Model, arris_check::Report, arris_topo::FaceId) {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = m.add_surface(Surface::Torus {
        frame: Frame::world(),
        major_radius: 1.0,
        minor_radius: 1.0,
    });
    let at = |v: f64| Point3::new(1.0 + real::cos(v), 0.0, real::sin(v));
    let a = m.raw().add_vertex(Vertex::new(at(v_start), tol));
    let b = m.raw().add_vertex(Vertex::new(at(v_end), tol));
    let curve = m.add_curve(Curve::Circle {
        frame: Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        radius: 1.0,
    });
    let edge = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve,
            range: Interval::new(v_start, v_end).unwrap(),
        },
        a,
        b,
        tol,
    ));
    let pcurve = m.add_curve2(Curve2::Line {
        origin: Point2::new(0.0, 0.0),
        direction: UnitVec2::new_normalize(arris_math::Vec2::new(0.0, 1.0)),
    });
    let coedge = Coedge::new(edge, Orientation::Forward, pcurve);
    let (body, face, _) = one_face(&mut m, surface, vec![Loop::new(vec![coedge])], tol);
    let report = check(&m, body, Level::Fast);
    (m, report, face)
}

#[test]
fn f1_a_torus_face_over_its_axis_point() {
    let (_m, report, face) = horn_meridian_face(std::f64::consts::PI);
    assert!(
        report.violations().contains(&Violation::FaceMalformed {
            face,
            fault: FaceFault::OffTorusBand {
                loop_index: 0,
                coedge: 0,
            },
        }),
        "report:\n{report}"
    );
}

#[test]
fn f1_a_torus_face_short_of_its_axis_point_is_on_the_band() {
    let (_m, report, _face) = horn_meridian_face(std::f64::consts::FRAC_PI_2);
    assert!(
        !report.violations().iter().any(|v| v.code() == "F1"),
        "report:\n{report}"
    );
}

/// A pcurve whose ends are both short of the axis point but which runs
/// across it between two samples is off the band all the same.
#[test]
fn f1_a_torus_face_across_its_axis_point_between_samples() {
    use std::f64::consts::PI;
    let (_m, report, face) = horn_meridian_span(PI - 0.3, PI + 0.5);
    assert!(
        report.violations().contains(&Violation::FaceMalformed {
            face,
            fault: FaceFault::OffTorusBand {
                loop_index: 0,
                coedge: 0,
            },
        }),
        "report:\n{report}"
    );
}

/// A spindle face between the circles `v = v_lo` and `v = v_hi`, each a
/// closed `u` loop clear of the axis; the face covers the `v` between.
fn spindle_annulus(v_lo: f64, v_hi: f64) -> (Model, arris_check::Report, arris_topo::FaceId) {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let (major, minor) = (0.5, 1.5);
    let surface = m.add_surface(Surface::Torus {
        frame: Frame::world(),
        major_radius: major,
        minor_radius: minor,
    });
    let mut loops = Vec::new();
    for v in [v_lo, v_hi] {
        let radius = major + minor * real::cos(v);
        let start = Point3::new(radius, 0.0, minor * real::sin(v));
        let vertex = m.raw().add_vertex(Vertex::new(start, tol));
        let curve = m.add_curve(Curve::Circle {
            frame: Frame::new(
                Point3::new(0.0, 0.0, start.z),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
            radius,
        });
        let edge = m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve {
                curve,
                range: Interval::new(0.0, std::f64::consts::TAU).unwrap(),
            },
            vertex,
            vertex,
            tol,
        ));
        let pcurve = m.add_curve2(Curve2::along_u(v));
        loops.push(Loop::new(vec![Coedge::new(
            edge,
            Orientation::Forward,
            pcurve,
        )]));
    }
    let (body, face, _) = one_face(&mut m, surface, loops, tol);
    let report = check(&m, body, Level::Fast);
    (m, report, face)
}

/// Neither loop reaches the axis points (`cos v = -1/3` there, `v` about
/// 1.91), but between `v = 1.8` and `v = 2π − 1.8` the face crosses the
/// axis and the lemon.
#[test]
fn f1_a_spindle_face_between_two_loops_across_its_axis() {
    let (_m, report, face) = spindle_annulus(1.8, std::f64::consts::TAU - 1.8);
    assert!(
        report.violations().contains(&Violation::FaceMalformed {
            face,
            fault: FaceFault::OffTorusBand {
                loop_index: 0,
                coedge: 0,
            },
        }),
        "report:\n{report}"
    );
}

/// The same two circles given as `v = -1.8` and `v = 1.8`: the face is the
/// outer part, on the band.
#[test]
fn f1_a_spindle_face_between_two_loops_on_the_band() {
    let (_m, report, _face) = spindle_annulus(-1.8, 1.8);
    assert!(
        !report.violations().iter().any(|v| v.code() == "F1"),
        "report:\n{report}"
    );
}
