//! L1–L4.

use arris_check::{Level, LoopBreak, NestingFault, Violation, check};
use arris_geom::{Curve, Curve2};
use arris_math::{Frame, Interval, Point2, Point3, Precision, Vec2, Vec3};
use arris_topo::entity::{Coedge, Edge, EdgeGeometry, Loop, Vertex};
use arris_topo::{Body, FaceId, Model, Orientation};

use crate::support::*;

/// L1: the last coedge of a ring ends at a *twin* of the first vertex —
/// a second vertex at the same point, so every geometric row still holds
/// and only the loop's closure is broken.
#[test]
fn l1_a_loop_that_does_not_close() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let r = ring(&mut m, &rect(0.0, 0.0, 10.0, 10.0, true));
    let twin = m.raw().add_vertex(Vertex::new(Point3::origin(), tol));
    let last = *r.coedges.last().unwrap();
    let old = *m.edge(last.edge()).unwrap();
    let edge = m.raw().add_edge(Edge::new(
        old.geometry(),
        old.start(),
        twin,
        old.tolerance(),
    ));
    let mut coedges = r.coedges.clone();
    let n = coedges.len();
    coedges[n - 1] = Coedge::new(edge, last.orientation(), last.pcurve());
    let (body, face, _) = one_face(&mut m, surface, vec![Loop::new(coedges)], tol);
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("L1", face.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::LoopOpen {
            face,
            loop_index: 0,
            fault: LoopBreak::Between { coedge: 3 },
        }
    );
}

#[test]
fn l1_an_empty_loop() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let r = ring(&mut m, &rect(0.0, 0.0, 10.0, 10.0, true));
    let loops = vec![Loop::new(r.coedges.clone()), Loop::new(Vec::new())];
    let (body, face, _) = one_face(&mut m, surface, loops, tol);
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("L1", face.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::LoopOpen {
            face,
            loop_index: 1,
            fault: LoopBreak::Empty,
        }
    );
}

/// L2: a pcurve moved in (u, v) by more than the model's parametric
/// tolerance but less than its linear one, so the 3D rows (V2, V3, E4)
/// still hold and the junctions at both ends of the moved coedge are the
/// only thing wrong.
#[test]
fn l2_a_junction_gap_above_the_parametric_tolerance() {
    let mut m = Model::new(Precision {
        parametric_tolerance: 1e-9,
        ..Precision::DEFAULT
    })
    .unwrap();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let r = ring(&mut m, &rect(0.0, 0.0, 10.0, 10.0, true));
    let c0 = r.coedges[0];
    let Curve2::Line { origin, direction } = *m.curve2(c0.pcurve()).unwrap() else {
        panic!("a ring's pcurves are lines")
    };
    let moved = m.add_curve2(Curve2::Line {
        origin: origin + Vec2::new(1e-8, 0.0),
        direction,
    });
    let mut coedges = r.coedges.clone();
    coedges[0] = Coedge::new(c0.edge(), c0.orientation(), moved);
    let (body, face, _) = one_face(&mut m, surface, vec![Loop::new(coedges)], tol);
    let report = check(&m, body, Level::Fast);
    assert_lines(
        &report,
        &[("L2", face.to_string()), ("L2", face.to_string())],
    );
    for (violation, coedge) in report.violations().iter().zip([0, 3]) {
        assert!(
            matches!(
                violation,
                Violation::PcurveGap { face: f, loop_index: 0, coedge: c, gap }
                    if *f == face && *c == coedge && (gap - 1e-8).abs() < 1e-14
            ),
            "{violation}"
        );
    }
}

/// L3: a plane has no period, so an edge used twice by one of its loops
/// is no seam — and the sliver the two uses bound encloses nothing, which
/// is L4's `ZeroArea`.
#[test]
fn l3_an_edge_used_twice_by_one_loop_of_a_plane() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let v0 = m.raw().add_vertex(Vertex::new(Point3::origin(), tol));
    let v1 = m
        .raw()
        .add_vertex(Vertex::new(Point3::new(10.0, 0.0, 0.0), tol));
    let curve = m.add_curve(Curve::Line {
        origin: Point3::origin(),
        direction: Vec3::x_axis(),
    });
    let edge = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve,
            range: Interval::new(0.0, 10.0).unwrap(),
        },
        v0,
        v1,
        tol,
    ));
    let pcurve = m.add_curve2(Curve2::Line {
        origin: Point2::origin(),
        direction: Vec2::x_axis(),
    });
    let loops = vec![Loop::new(vec![
        Coedge::new(edge, Orientation::Forward, pcurve),
        Coedge::new(edge, Orientation::Reversed, pcurve),
    ])];
    let (body, face, _) = one_face(&mut m, surface, loops, tol);
    let report = check(&m, body, Level::Fast);
    assert_lines(
        &report,
        &[("L3", face.to_string()), ("L4", face.to_string())],
    );
    assert_eq!(
        report.violations()[0],
        Violation::EdgeReusedInFace { face, edge }
    );
    assert_eq!(
        report.violations()[1],
        Violation::LoopNesting {
            face,
            fault: NestingFault::ZeroArea { loop_index: 0 },
        }
    );
}

#[test]
fn l4_two_outer_loops() {
    let mut m = Model::default();
    let (body, face, _) = plate(
        &mut m,
        &[
            rect(0.0, 0.0, 10.0, 10.0, true),
            rect(2.0, 2.0, 8.0, 8.0, true),
        ],
    );
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("L4", face.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::LoopNesting {
            face,
            fault: NestingFault::MultipleOuter { loops: vec![0, 1] },
        }
    );
}

#[test]
fn l4_a_hole_outside_its_outer_loop() {
    let mut m = Model::default();
    let (body, face, _) = plate(
        &mut m,
        &[
            rect(0.0, 0.0, 10.0, 10.0, true),
            rect(20.0, 20.0, 30.0, 30.0, false),
        ],
    );
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("L4", face.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::LoopNesting {
            face,
            fault: NestingFault::HoleOutside { loop_index: 1 },
        }
    );
}

/// A plate whose outer loop is the circle of radius 10 about the origin,
/// one closed edge, and whose hole is the clockwise square of side 0.1
/// centred `r` from the origin at 22.5°: midway between two vertices of
/// the circle's octagon, the polygon L4 decides on first.
fn disc_with_a_square_hole(m: &mut Model, r: f64) -> (Body, FaceId) {
    let tol = m.precision().default_tolerance;
    let surface = ground(m);
    let circle = m.add_curve(Curve::Circle {
        frame: Frame::world(),
        radius: 10.0,
    });
    let pcurve = m.add_curve2(Curve2::Circle {
        frame: arris_math::Frame2::identity(),
        radius: 10.0,
    });
    let v = m
        .raw()
        .add_vertex(Vertex::new(Point3::new(10.0, 0.0, 0.0), tol));
    let edge = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: circle,
            range: Interval::TURN,
        },
        v,
        v,
        tol,
    ));
    let angle = core::f64::consts::PI / 8.0;
    let (x, y) = (
        r * arris_math::real::cos(angle),
        r * arris_math::real::sin(angle),
    );
    let hole = ring(m, &rect(x - 0.05, y - 0.05, x + 0.05, y + 0.05, false));
    let loops = vec![
        Loop::new(vec![Coedge::new(edge, Orientation::Forward, pcurve)]),
        Loop::new(hole.coedges),
    ];
    let (body, face, _) = one_face(m, surface, loops, tol);
    (body, face)
}

/// L4 decides on coarse polygons first. A hole between the circle and
/// its octagon, 0.36 inside the rim and 0.3 outside the octagon, is
/// inside the face: the coarse answer is within the octagon's deviation,
/// so the loops are read again at the fine chord.
#[test]
fn l4_a_hole_between_a_circle_and_its_octagon_is_inside() {
    let mut m = Model::default();
    let (body, _) = disc_with_a_square_hole(&mut m, 9.6);
    let report = check(&m, body, Level::Fast);
    assert!(report.is_ok(), "{report}");
}

/// The same hole just past the rim is outside, whichever polygon says so.
#[test]
fn l4_a_hole_just_past_a_circles_rim_is_outside() {
    let mut m = Model::default();
    let (body, face) = disc_with_a_square_hole(&mut m, 10.1);
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("L4", face.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::LoopNesting {
            face,
            fault: NestingFault::HoleOutside { loop_index: 1 },
        }
    );
}

#[test]
fn l4_a_face_whose_only_loop_turns_the_wrong_way() {
    let mut m = Model::default();
    let (body, face, _) = plate(&mut m, &[rect(0.0, 0.0, 10.0, 10.0, false)]);
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("L4", face.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::LoopNesting {
            face,
            fault: NestingFault::NoOuter,
        }
    );
}
