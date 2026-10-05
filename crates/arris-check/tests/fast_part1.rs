//! Checker `Fast`, part 1 (`docs/DATA-MODEL.md` §Invariants): the sample
//! bodies are clean; for each of the rows M1–M3, V1–V3 and E1–E7 a sample
//! body broken through the raw insert API reports that row on that entity,
//! together with exactly the rows its definition implies and nothing else;
//! the report is the same on two fresh builds.
//!
//! Three rows measure the same triangle from different corners — V2 (vertex
//! against the curve's end), V3 (vertex against the surface at the
//! pcurve's end), E4 (curve against the surface along the pcurve) — so a
//! moved range end or a shifted pcurve is reported under two of them; each
//! test names its full set.

use core::f64::consts::{PI, TAU};
use std::collections::BTreeMap;

use arris_check::{
    DegenerateFault, EndMismatch, Level, Quantity, Reference, Report, SeamFault, ToleranceBound,
    Violation, check,
};
use arris_debug::sample;
use arris_geom::{Curve, Curve2, NurbsCurve2, Surface};
use arris_math::{Frame, Frame2, Interval, Point2, Point3, Precision, Vec2};
use arris_topo::entity::{
    Body as BodyEntity, BodyKind, Coedge, Edge, EdgeGeometry, Face, Loop, Shell, Vertex,
};
use arris_topo::{
    Body, BodyId, CurveId, EdgeId, Face as FaceHandle, FaceId, Model, Orientation,
    Shell as ShellHandle, VertexId,
};

/// A copy of `body` appended through the raw API in iteration order, each
/// entity passed through its edit with its index in that order; references
/// are remapped to the copies, an id the edit made dangling is kept. The
/// new ids, by index.
struct Rebuilt {
    body: Body,
    vertices: Vec<VertexId>,
    edges: Vec<EdgeId>,
    faces: Vec<FaceId>,
}

fn rebuild(
    m: &mut Model,
    body: Body,
    vertex: &dyn Fn(usize, Vertex) -> Vertex,
    edge: &dyn Fn(usize, Edge) -> Edge,
    face: &dyn Fn(usize, Face) -> Face,
) -> Rebuilt {
    let old = m.body(body.id).unwrap().clone();
    let old_vertices = m.vertices(body).unwrap();
    let old_edges = m.edges(body).unwrap();
    let old_faces = m.faces(body).unwrap();
    let old_shells = m.shells(body).unwrap();

    let mut vmap = BTreeMap::new();
    let mut vertices = Vec::new();
    for (i, v) in old_vertices.iter().enumerate() {
        let v0 = *m.vertex(v.id).unwrap();
        let id = m.raw().add_vertex(vertex(i, v0));
        vmap.insert(v.id, id);
        vertices.push(id);
    }
    let mut emap = BTreeMap::new();
    let mut edges = Vec::new();
    for (i, e) in old_edges.iter().enumerate() {
        let e0 = *m.edge(e.id).unwrap();
        let remapped = Edge::new(
            e0.geometry(),
            vmap.get(&e0.start()).copied().unwrap_or(e0.start()),
            vmap.get(&e0.end()).copied().unwrap_or(e0.end()),
            e0.tolerance(),
        );
        let id = m.raw().add_edge(edge(i, remapped));
        emap.insert(e.id, id);
        edges.push(id);
    }
    let mut fmap = BTreeMap::new();
    let mut faces = Vec::new();
    for (i, f) in old_faces.iter().enumerate() {
        let f0 = m.face(f.id).unwrap().clone();
        let loops = f0
            .loops()
            .iter()
            .map(|l| {
                Loop::new(
                    l.coedges()
                        .iter()
                        .map(|c| {
                            Coedge::new(
                                emap.get(&c.edge()).copied().unwrap_or(c.edge()),
                                c.orientation(),
                                c.pcurve(),
                            )
                        })
                        .collect(),
                )
            })
            .collect();
        let id = m
            .raw()
            .add_face(face(i, Face::new(f0.surface(), loops, f0.tolerance())));
        fmap.insert(f.id, id);
        faces.push(id);
    }
    let mut smap = BTreeMap::new();
    for s in &old_shells {
        let s0 = m.shell(s.id).unwrap().clone();
        let uses = s0
            .faces()
            .iter()
            .map(|f| FaceHandle::new(fmap.get(&f.id).copied().unwrap_or(f.id), f.orientation))
            .collect();
        smap.insert(s.id, m.raw().add_shell(Shell::new(uses)));
    }
    let shells = old
        .shells()
        .iter()
        .map(|s| ShellHandle::new(smap.get(&s.id).copied().unwrap_or(s.id), s.orientation))
        .collect();
    let free_edges = old
        .free_edges()
        .iter()
        .map(|e| arris_topo::Edge::new(emap.get(&e.id).copied().unwrap_or(e.id), e.orientation))
        .collect();
    let free_vertices = old
        .free_vertices()
        .iter()
        .map(|v| vmap.get(v).copied().unwrap_or(*v))
        .collect();
    let id = m.raw().add_body(BodyEntity::new(
        old.kind(),
        shells,
        free_edges,
        free_vertices,
    ));
    Rebuilt {
        body: Body::new(id, body.orientation),
        vertices,
        edges,
        faces,
    }
}

fn keep_vertex(_: usize, v: Vertex) -> Vertex {
    v
}
fn keep_edge(_: usize, e: Edge) -> Edge {
    e
}
fn keep_face(_: usize, f: Face) -> Face {
    f
}

/// `(code, entity)` per line of the report.
fn lines(report: &Report) -> Vec<(String, String)> {
    report
        .violations()
        .iter()
        .map(|v| (v.code().to_string(), v.entity().to_string()))
        .collect()
}

/// The report holds exactly `expected` as `(code, entity)` lines, in
/// report order.
fn assert_lines(report: &Report, expected: &[(&str, String)]) {
    let want: Vec<(String, String)> = expected
        .iter()
        .map(|(c, e)| (c.to_string(), e.clone()))
        .collect();
    assert_eq!(lines(report), want, "report:\n{report}");
}

/// The two faces of the rebuilt body that use `edge`, ascending.
fn faces_of(m: &Model, edge: EdgeId, r: &Rebuilt) -> (String, String) {
    let mut faces: Vec<FaceId> = r
        .faces
        .iter()
        .copied()
        .filter(|&f| {
            m.face(f)
                .unwrap()
                .loops()
                .iter()
                .flat_map(|l| l.coedges())
                .any(|c| c.edge() == edge)
        })
        .collect();
    faces.sort();
    (faces[0].to_string(), faces[1].to_string())
}

fn cylinder(m: &mut Model) -> Body {
    sample::cylinder(m, 4.0, 12.0).unwrap()
}

fn box40(m: &mut Model) -> Body {
    sample::cuboid(m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap()
}

/// The sample cylinder's entities in iteration order: v0 at the base of
/// the seam, v1 at its top; e0 the bottom circle, e1 the seam, e2 the top
/// circle; f0 the wall, f1 the bottom cap, f2 the top cap.
fn edit_edge(e: &Edge, geometry: EdgeGeometry) -> Edge {
    Edge::new(geometry, e.start(), e.end(), e.tolerance())
}

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
fn v1_a_vertex_tolerance_above_the_ceiling_and_below_the_floor() {
    let p = Precision::DEFAULT;
    let mut m = Model::default();
    let b = box40(&mut m);
    let r = rebuild(
        &mut m,
        b,
        &|i, v| {
            if i == 0 {
                Vertex::new(v.point(), 10.0 * p.max_tolerance)
            } else {
                v
            }
        },
        &keep_edge,
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    assert_eq!(
        report.violations(),
        [Violation::VertexTolerance {
            vertex: r.vertices[0],
            tolerance: 10.0 * p.max_tolerance,
            bound: ToleranceBound::AboveMaximum,
        }],
        "{report}"
    );
    // Below the floor: every edge at the vertex is now above it too (E5).
    let b = box40(&mut m);
    let r = rebuild(
        &mut m,
        b,
        &|i, v| {
            if i == 0 {
                Vertex::new(v.point(), p.min_tolerance / 10.0)
            } else {
                v
            }
        },
        &keep_edge,
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    let v0 = r.vertices[0].to_string();
    let mut expected = vec![("V1", v0)];
    for e in m.vertex_edges(r.vertices[0]).unwrap() {
        expected.push(("E5", e.to_string()));
    }
    assert_lines(&report, &expected);
}

#[test]
fn v2_a_range_end_off_its_vertex() {
    let mut m = Model::default();
    let c = cylinder(&mut m);
    let r = rebuild(
        &mut m,
        c,
        &keep_vertex,
        &|i, e| {
            if i == 1 {
                let (curve, _) = e.curve().unwrap();
                edit_edge(
                    &e,
                    EdgeGeometry::Curve {
                        curve,
                        range: Interval::new(0.5, 12.0).unwrap(),
                    },
                )
            } else {
                e
            }
        },
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    let v0 = r.vertices[0].to_string();
    // The moved range end takes the seam's pcurve end with it, so the
    // wall's loop no longer meets itself in (u, v) at either side of it.
    let f0 = r.faces[0].to_string();
    assert_lines(
        &report,
        &[
            ("V2", v0.clone()),
            ("V3", v0),
            ("L2", f0.clone()),
            ("L2", f0),
        ],
    );
    assert!(matches!(
        report.violations()[0],
        Violation::VertexOffEdge { edge, distance, .. } if edge == r.edges[1] && (distance - 0.5).abs() < 1e-12
    ));
    assert!(matches!(
        report.violations()[1],
        Violation::VertexOffFace { face, .. } if face == r.faces[0]
    ));
}

#[test]
fn v3_a_pcurve_whose_end_misses_the_vertex() {
    let mut m = Model::default();
    let c = cylinder(&mut m);
    let shifted = m.add_curve2(Curve2::Circle {
        frame: Frame2::new(
            Point2::new(1.0, 0.0),
            Vec2::x(),
            arris_math::Handedness::Right,
        )
        .unwrap(),
        radius: 4.0,
    });
    let r = rebuild(&mut m, c, &keep_vertex, &keep_edge, &|i, f| {
        if i == 1 {
            let c = f.loops()[0].coedges()[0];
            Face::new(
                f.surface(),
                vec![Loop::new(vec![Coedge::new(
                    c.edge(),
                    c.orientation(),
                    shifted,
                )])],
                f.tolerance(),
            )
        } else {
            f
        }
    });
    let report = check(&m, r.body, Level::Fast);
    assert_lines(
        &report,
        &[
            ("V3", r.vertices[0].to_string()),
            ("E4", r.edges[0].to_string()),
        ],
    );
    assert!(matches!(
        report.violations()[0],
        Violation::VertexOffFace { face, distance, .. } if face == r.faces[1] && (distance - 1.0).abs() < 1e-12
    ));
}

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
