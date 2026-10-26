//! E1–E7, with the three rows that measure one triangle from different corners (V2, V3, E4) named in each test.

use arris_check::{
    DegenerateFault, EndMismatch, Level, SeamFault, ToleranceBound, Violation, check,
};
use arris_debug::testing::cylinder;
use arris_geom::{Curve, Curve2, NurbsCurve2, Surface};
use arris_math::{Frame, Frame2, Interval, Point2, Point3, Precision, Vec2};
use arris_topo::entity::{
    Body as BodyEntity, BodyKind, Coedge, Edge, EdgeGeometry, Face, Loop, Shell, Vertex,
};
use arris_topo::{
    Body, EdgeId, Face as FaceHandle, Model, Orientation, Shell as ShellHandle, VertexId,
};
use core::f64::consts::{PI, TAU};

use crate::support::*;

#[test]
fn e1_a_range_longer_than_the_period() {
    let mut m = Model::default();
    let c = cylinder(&mut m);
    let r = rebuild(
        &mut m,
        c,
        &keep_vertex,
        &|i, e| {
            if i == 0 {
                let (curve, _) = e.curve().unwrap();
                edit_edge(
                    &e,
                    EdgeGeometry::Curve {
                        curve,
                        range: Interval::new(0.0, 2.0 * TAU).unwrap(),
                    },
                )
            } else {
                e
            }
        },
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    assert_eq!(
        report.violations(),
        [Violation::EdgeRange { edge: r.edges[0] }],
        "{report}"
    );
    // And an empty range on a line, which is also outside no domain.
    let b = box40(&mut m);
    let r = rebuild(
        &mut m,
        b,
        &keep_vertex,
        &|i, e| {
            if i == 0 {
                let (curve, _) = e.curve().unwrap();
                edit_edge(
                    &e,
                    EdgeGeometry::Curve {
                        curve,
                        range: Interval::new(0.0, 0.0).unwrap(),
                    },
                )
            } else {
                e
            }
        },
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    assert!(
        report
            .violations()
            .contains(&Violation::EdgeRange { edge: r.edges[0] }),
        "{report}"
    );
}

#[test]
fn e2_a_closed_curve_with_two_vertices_and_an_open_one_with_one() {
    let mut m = Model::default();
    let c = cylinder(&mut m);
    // The bottom circle ending at the top vertex, the copy appended right
    // after the bottom one.
    let r = rebuild(
        &mut m,
        c,
        &keep_vertex,
        &|i, e| {
            if i == 0 {
                let top = VertexId::new(e.start().index() + 1, 0);
                Edge::new(e.geometry(), e.start(), top, e.tolerance())
            } else {
                e
            }
        },
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    let v1 = r.vertices[1].to_string();
    // Both loops that use the circle now break where its end vertex moved.
    assert_lines(
        &report,
        &[
            ("V2", v1.clone()),
            ("V3", v1.clone()),
            ("V3", v1),
            ("E2", r.edges[0].to_string()),
            ("L1", r.faces[0].to_string()),
            ("L1", r.faces[1].to_string()),
        ],
    );
    assert!(matches!(
        report.violations()[3],
        Violation::EdgeEnds {
            fault: EndMismatch::ClosedWithTwoVertices { .. },
            ..
        }
    ));
    // A box edge naming its start vertex at both ends.
    let b = box40(&mut m);
    let r = rebuild(
        &mut m,
        b,
        &keep_vertex,
        &|i, e| {
            if i == 0 {
                Edge::new(e.geometry(), e.start(), e.start(), e.tolerance())
            } else {
                e
            }
        },
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    let v0 = r.vertices[0].to_string();
    let (fa, fb) = faces_of(&m, r.edges[0], &r);
    assert_lines(
        &report,
        &[
            ("V2", v0.clone()),
            ("V3", v0.clone()),
            ("V3", v0),
            ("E2", r.edges[0].to_string()),
            ("L1", fa),
            ("L1", fb),
        ],
    );
    assert!(matches!(
        report.violations()[3],
        Violation::EdgeEnds {
            fault: EndMismatch::OpenWithOneVertex { gap, .. },
            ..
        } if (gap - 30.0).abs() < 1e-12
    ));
}

#[test]
fn e3_an_edge_of_a_solid_used_by_no_coedge() {
    let mut m = Model::default();
    let b = box40(&mut m);
    let tol = m.precision().default_tolerance;
    let old = m.body(b.id).unwrap().clone();
    let (v0, v7) = (VertexId::new(0, 0), VertexId::new(7, 0));
    let diagonal = m.add_curve(Curve::Line {
        origin: Point3::origin(),
        direction: arris_math::UnitVec3::new_normalize(
            m.vertex(v7).unwrap().point() - Point3::origin(),
        ),
    });
    let length = (m.vertex(v7).unwrap().point() - Point3::origin()).norm();
    let stray = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: diagonal,
            range: Interval::new(0.0, length).unwrap(),
        },
        v0,
        v7,
        tol,
    ));
    let body = m.raw().add_body(BodyEntity::new(
        BodyKind::Solid,
        old.shells().to_vec(),
        vec![arris_topo::Edge::forward(stray)],
        Vec::new(),
    ));
    let report = check(&m, Body::forward(body), Level::Fast);
    assert_eq!(
        report.violations(),
        [Violation::EdgeUnused { edge: stray }],
        "{report}"
    );
    // The same free edge in a wire body is allowed by E3.
    let wire = m.raw().add_body(BodyEntity::new(
        BodyKind::Wire,
        Vec::new(),
        vec![arris_topo::Edge::forward(stray)],
        Vec::new(),
    ));
    assert!(check(&m, Body::forward(wire), Level::Fast).is_ok());
}

#[test]
fn e4_a_pcurve_that_leaves_the_curve_between_its_ends() {
    let mut m = Model::default();
    let c = cylinder(&mut m);
    // Same ends as the wall's bottom line, one unit up at the middle.
    let bulge = m.add_curve2(Curve2::Nurbs(
        NurbsCurve2::new(
            2,
            vec![0.0, 0.0, 0.0, TAU, TAU, TAU],
            vec![
                Point2::new(0.0, 0.0),
                Point2::new(PI, 2.0),
                Point2::new(TAU, 0.0),
            ],
            vec![1.0; 3],
        )
        .unwrap(),
    ));
    let r = rebuild(&mut m, c, &keep_vertex, &keep_edge, &|i, f| {
        if i == 0 {
            let coedges = f.loops()[0]
                .coedges()
                .iter()
                .enumerate()
                .map(|(k, c)| {
                    Coedge::new(
                        c.edge(),
                        c.orientation(),
                        if k == 0 { bulge } else { c.pcurve() },
                    )
                })
                .collect();
            Face::new(f.surface(), vec![Loop::new(coedges)], f.tolerance())
        } else {
            f
        }
    });
    let report = check(&m, r.body, Level::Fast);
    assert_lines(&report, &[("E4", r.edges[0].to_string())]);
    assert!(matches!(
        report.violations()[0],
        Violation::PcurveOffCurve { face, parameter, distance, .. }
            if face == r.faces[0] && (parameter - PI).abs() < 1e-9 && (distance - 1.0).abs() < 1e-9
    ));
}

#[test]
fn e5_an_edge_tolerance_below_its_faces_and_above_its_vertices() {
    let p = Precision::DEFAULT;
    let mut m = Model::default();
    let b = box40(&mut m);
    let r = rebuild(
        &mut m,
        b,
        &keep_vertex,
        &|i, e| {
            if i == 0 {
                Edge::new(e.geometry(), e.start(), e.end(), p.default_tolerance / 10.0)
            } else {
                e
            }
        },
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    let e0 = r.edges[0].to_string();
    // F2 is the same ordering read from the face's side, on each of the
    // two faces the edge bounds.
    let (fa, fb) = faces_of(&m, r.edges[0], &r);
    assert_lines(
        &report,
        &[("E5", e0.clone()), ("E5", e0), ("F2", fa), ("F2", fb)],
    );
    for v in report.violations() {
        assert!(matches!(
            v,
            Violation::EdgeTolerance {
                bound: ToleranceBound::Neighbour {
                    entity: arris_topo::EntityId::Face(_),
                    ..
                },
                ..
            } | Violation::FaceTolerance {
                bound: ToleranceBound::Neighbour {
                    entity: arris_topo::EntityId::Edge(_),
                    ..
                },
                ..
            }
        ));
    }
    let b = box40(&mut m);
    let r = rebuild(
        &mut m,
        b,
        &keep_vertex,
        &|i, e| {
            if i == 0 {
                Edge::new(e.geometry(), e.start(), e.end(), p.default_tolerance * 10.0)
            } else {
                e
            }
        },
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    let e0 = r.edges[0].to_string();
    assert_lines(&report, &[("E5", e0.clone()), ("E5", e0)]);
    for v in report.violations() {
        assert!(matches!(
            v,
            Violation::EdgeTolerance {
                bound: ToleranceBound::Neighbour {
                    entity: arris_topo::EntityId::Vertex(_),
                    ..
                },
                ..
            }
        ));
    }
}

/// A sheet of one planar face whose one loop is a degenerate edge with a
/// circular pcurve: the plane is not singular anywhere, so the edge's
/// image is a circle, not a point.
fn degenerate_on_a_plane(m: &mut Model, second_vertex: bool) -> (Body, EdgeId) {
    let tol = m.precision().default_tolerance;
    let plane = m.add_surface(Surface::Plane {
        frame: Frame::world(),
    });
    let pcurve = m.add_curve2(Curve2::Circle {
        frame: Frame2::identity(),
        radius: 1.0,
    });
    let v = m
        .raw()
        .add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), tol));
    let end = if second_vertex {
        m.raw()
            .add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), tol))
    } else {
        v
    };
    let edge = m.raw().add_edge(Edge::new(
        EdgeGeometry::Degenerate {
            range: Interval::TURN,
        },
        v,
        end,
        tol,
    ));
    let face = m.raw().add_face(Face::new(
        plane,
        vec![Loop::new(vec![Coedge::new(
            edge,
            Orientation::Forward,
            pcurve,
        )])],
        tol,
    ));
    let shell = m
        .raw()
        .add_shell(Shell::new(vec![FaceHandle::forward(face)]));
    let body = m.raw().add_body(BodyEntity::new(
        BodyKind::Sheet,
        vec![ShellHandle::forward(shell)],
        Vec::new(),
        Vec::new(),
    ));
    (Body::forward(body), edge)
}

#[test]
fn e6_a_degenerate_edge_where_the_surface_is_not_singular_or_with_two_vertices() {
    let mut m = Model::default();
    let (body, edge) = degenerate_on_a_plane(&mut m, false);
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("E6", edge.to_string())]);
    assert!(matches!(
        report.violations()[0],
        Violation::DegenerateEdge {
            fault: DegenerateFault::NotSingular { extent, .. },
            ..
        } if (extent - 2.0).abs() < 1e-9
    ));
    let (body, edge) = degenerate_on_a_plane(&mut m, true);
    let report = check(&m, body, Level::Fast);
    // Two vertices on the one coedge of the loop is also an open loop.
    let face = m.faces(body).unwrap()[0].id.to_string();
    assert_lines(
        &report,
        &[
            ("E6", edge.to_string()),
            ("E6", edge.to_string()),
            ("L1", face),
        ],
    );
    assert!(report.violations().iter().any(|v| matches!(
        v,
        Violation::DegenerateEdge {
            fault: DegenerateFault::TwoVertices,
            ..
        }
    )));
}

#[test]
fn e7_a_seam_with_both_uses_forward_or_pcurves_not_a_period_apart() {
    let mut m = Model::default();
    let c = cylinder(&mut m);
    let r = rebuild(&mut m, c, &keep_vertex, &keep_edge, &|i, f| {
        if i == 0 {
            let coedges = f.loops()[0]
                .coedges()
                .iter()
                .map(|c| Coedge::new(c.edge(), Orientation::Forward, c.pcurve()))
                .collect();
            Face::new(f.surface(), vec![Loop::new(coedges)], f.tolerance())
        } else {
            f
        }
    });
    let report = check(&m, r.body, Level::Fast);
    // Turning the wall's uses all forward takes the loop's closure, its
    // (u, v) junctions and the pairing of two edges' uses with it.
    let f0 = r.faces[0].to_string();
    let s0 = m.shells(r.body).unwrap()[0].id.to_string();
    assert_lines(
        &report,
        &[
            ("E7", r.edges[1].to_string()),
            ("L1", f0.clone()),
            ("L2", f0.clone()),
            ("L2", f0),
            ("S2", s0.clone()),
            ("S2", s0),
        ],
    );
    assert_eq!(
        report.violations()[0],
        Violation::Seam {
            edge: r.edges[1],
            face: r.faces[0],
            fault: SeamFault::SameOrientation,
        },
        "{report}"
    );
    // The seam's up pcurve at u = π instead of 2π: half a period apart,
    // and the surface along it is the far side of the cylinder (E4, V3).
    let c = cylinder(&mut m);
    let half = m.add_curve2(Curve2::Line {
        origin: Point2::new(PI, 0.0),
        direction: Vec2::y_axis(),
    });
    let r = rebuild(&mut m, c, &keep_vertex, &keep_edge, &|i, f| {
        if i == 0 {
            let coedges = f.loops()[0]
                .coedges()
                .iter()
                .enumerate()
                .map(|(k, c)| {
                    Coedge::new(
                        c.edge(),
                        c.orientation(),
                        if k == 1 { half } else { c.pcurve() },
                    )
                })
                .collect();
            Face::new(f.surface(), vec![Loop::new(coedges)], f.tolerance())
        } else {
            f
        }
    });
    let report = check(&m, r.body, Level::Fast);
    // Half a period is a jump at both ends of the seam's forward use.
    let f0 = r.faces[0].to_string();
    assert_lines(
        &report,
        &[
            ("V3", r.vertices[0].to_string()),
            ("V3", r.vertices[1].to_string()),
            ("E4", r.edges[1].to_string()),
            ("E7", r.edges[1].to_string()),
            ("L2", f0.clone()),
            ("L2", f0),
        ],
    );
    assert!(matches!(
        report.violations()[3],
        Violation::Seam {
            fault: SeamFault::PeriodMismatch { offset, period },
            ..
        } if (offset + PI).abs() < 1e-12 && period == TAU
    ));
}
