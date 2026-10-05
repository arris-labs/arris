//! Checker `Fast`, part 2 (`docs/DATA-MODEL.md` §Invariants): one
//! violation test per row of L1–L4, F1–F2, S1–S4 and B3, and the
//! Euler–Poincaré line every report carries.
//!
//! Most of these rows are about a face's own (u, v), so most tests start
//! from a *plate* — a sheet body of one planar face whose loops are rings
//! of line edges, its (u, v) the world's `x` and `y`. A plate is clean at
//! `Fast`, so what a test breaks is the only thing the report holds,
//! except where one row's definition implies another's; each test names
//! its full set.

use arris_check::{
    EdgeUseFault, FaceFault, Level, LoopBreak, NestingFault, Report, ToleranceBound, Violation,
    WireFault, check,
};
use arris_debug::sample;
use arris_geom::{Curve, Curve2, NurbsSurface, Surface};
use arris_math::{Frame, Interval, Point2, Point3, Precision, UnitVec2, UnitVec3, Vec2, Vec3};
use arris_topo::entity::{
    Body as BodyEntity, BodyKind, Coedge, Edge, EdgeGeometry, Face, Loop, Shell, Vertex,
};
use arris_topo::{
    Body, Edge as EdgeHandle, EdgeId, Face as FaceHandle, FaceId, Model, Orientation,
    Shell as ShellHandle, ShellId, SurfaceId,
};

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

fn p2(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

/// The corners of an axis-aligned rectangle, counter-clockwise in (u, v)
/// when `ccw`, clockwise otherwise.
fn rect(x0: f64, y0: f64, x1: f64, y1: f64, ccw: bool) -> Vec<Point2> {
    let mut corners = vec![p2(x0, y0), p2(x1, y0), p2(x1, y1), p2(x0, y1)];
    if !ccw {
        corners.reverse();
    }
    corners
}

/// The plane `z = 0`, whose own (u, v) is the world's `x` and `y`.
fn ground(m: &mut Model) -> SurfaceId {
    m.add_surface(Surface::Plane {
        frame: Frame::world(),
    })
}

/// The bilinear NURBS patch over `[0, 1]²` that is the same plane at the
/// same parameter, extrapolated outside its knot range — a surface with
/// a *bounded* domain, which F1 is the only row to care about.
fn ground_patch(m: &mut Model) -> SurfaceId {
    let corner = |u: f64, v: f64| Point3::new(u, v, 0.0);
    m.add_surface(Surface::Nurbs(
        NurbsSurface::new(
            [1, 1],
            [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
            vec![
                corner(0.0, 0.0),
                corner(0.0, 1.0),
                corner(1.0, 0.0),
                corner(1.0, 1.0),
            ],
            vec![1.0; 4],
        )
        .unwrap(),
    ))
}

/// One ring of the ground plane: a vertex per corner, a line edge per
/// side from corner `k` to corner `k + 1`, and a forward coedge per edge
/// whose pcurve is that side in (u, v) at the edge's own parameter.
struct Ring {
    edges: Vec<EdgeId>,
    coedges: Vec<Coedge>,
}

fn ring(m: &mut Model, corners: &[Point2]) -> Ring {
    let tol = m.precision().default_tolerance;
    let n = corners.len();
    let at = |p: Point2| Point3::new(p.x, p.y, 0.0);
    let vertices: Vec<_> = corners
        .iter()
        .map(|&c| m.raw().add_vertex(Vertex::new(at(c), tol)))
        .collect();
    let mut edges = Vec::with_capacity(n);
    let mut coedges = Vec::with_capacity(n);
    for k in 0..n {
        let (a, b) = (corners[k], corners[(k + 1) % n]);
        let d = b - a;
        let range = Interval::new(0.0, d.norm()).unwrap();
        let curve = m.add_curve(Curve::Line {
            origin: at(a),
            direction: UnitVec3::new_normalize(Vec3::new(d.x, d.y, 0.0)),
        });
        let edge = m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve { curve, range },
            vertices[k],
            vertices[(k + 1) % n],
            tol,
        ));
        let pcurve = m.add_curve2(Curve2::Line {
            origin: a,
            direction: UnitVec2::new_normalize(d),
        });
        edges.push(edge);
        coedges.push(Coedge::new(edge, Orientation::Forward, pcurve));
    }
    Ring { edges, coedges }
}

/// A body of one shell over `uses`, of `kind`.
fn body_of(m: &mut Model, uses: Vec<FaceHandle>, kind: BodyKind) -> (Body, ShellId) {
    let shell = m.raw().add_shell(Shell::new(uses));
    let body = m.raw().add_body(BodyEntity::new(
        kind,
        vec![ShellHandle::forward(shell)],
        Vec::new(),
        Vec::new(),
    ));
    (Body::forward(body), shell)
}

/// A sheet body of one face on `surface` with `loops`, at `tolerance`.
fn one_face(
    m: &mut Model,
    surface: SurfaceId,
    loops: Vec<Loop>,
    tolerance: f64,
) -> (Body, FaceId, ShellId) {
    let face = m.raw().add_face(Face::new(surface, loops, tolerance));
    let (body, shell) = body_of(m, vec![FaceHandle::forward(face)], BodyKind::Sheet);
    (body, face, shell)
}

/// A sheet body of one planar face whose loops are `rings`, in order.
fn plate(m: &mut Model, rings: &[Vec<Point2>]) -> (Body, FaceId, Vec<Ring>) {
    let tol = m.precision().default_tolerance;
    let surface = ground(m);
    let rings: Vec<Ring> = rings.iter().map(|c| ring(m, c)).collect();
    let loops = rings.iter().map(|r| Loop::new(r.coedges.clone())).collect();
    let (body, face, _) = one_face(m, surface, loops, tol);
    (body, face, rings)
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

#[test]
fn the_euler_line_of_each_sample_closes_and_a_sheets_does_not() {
    let mut m = Model::default();
    let box40 = sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let line = |body| check(&m, body, Level::Full).euler().unwrap();
    assert_eq!(line(box40).to_string(), "8/12/6/6/1 g0 = 0");
    assert_eq!(line(cylinder).to_string(), "2/3/3/3/1 g0 = 0");
    assert_eq!(line(box40).genus, 0);
    assert!(line(box40).closes() && line(cylinder).closes());
    // A sheet is not a closed orientable surface, and the counts say so:
    // no genus takes the residual to zero.
    let (sheet, _, _) = plate(&mut m, &[rect(0.0, 0.0, 10.0, 10.0, true)]);
    let report = check(&m, sheet, Level::Fast);
    let line = report.euler().unwrap();
    assert_eq!(line.to_string(), "4/4/1/1/1 g1 = 1");
    assert!(!line.closes());
    // The line is a line, never a violation.
    assert!(report.is_ok(), "{report}");
    // A body that does not resolve has no counts to take.
    assert_eq!(
        check(
            &m,
            Body::forward(arris_topo::BodyId::new(99, 0)),
            Level::Fast
        )
        .euler(),
        None
    );
}

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

#[test]
fn s1_a_face_used_twice_by_a_shell() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let r = ring(&mut m, &rect(0.0, 0.0, 10.0, 10.0, true));
    let face = m
        .raw()
        .add_face(Face::new(surface, vec![Loop::new(r.coedges.clone())], tol));
    // The second use is reversed, so the uses of every edge still pair
    // up and S1 is the only thing wrong.
    let (body, shell) = body_of(
        &mut m,
        vec![
            FaceHandle::forward(face),
            FaceHandle::new(face, Orientation::Reversed),
        ],
        BodyKind::Sheet,
    );
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("S1", shell.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::FaceUsedTwice { shell, face }
    );
}

/// S2: one face of a box used the other way round, so its four edges
/// have two uses that agree about which side the material is on.
#[test]
fn s2_a_solids_face_use_flipped() {
    let mut m = Model::default();
    let b = sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let faces = m.faces(b).unwrap();
    let mut edges: Vec<EdgeId> = m.face(faces[0].id).unwrap().loops()[0]
        .coedges()
        .iter()
        .map(|c| c.edge())
        .collect();
    edges.sort();
    let uses: Vec<FaceHandle> = faces
        .iter()
        .enumerate()
        .map(|(i, f)| {
            if i == 0 {
                FaceHandle::new(f.id, f.orientation.flipped())
            } else {
                *f
            }
        })
        .collect();
    let (body, shell) = body_of(&mut m, uses, BodyKind::Solid);
    let report = check(&m, body, Level::Fast);
    let expected: Vec<(&str, String)> = edges.iter().map(|_| ("S2", shell.to_string())).collect();
    assert_lines(&report, &expected);
    for (v, edge) in report.violations().iter().zip(edges) {
        assert_eq!(
            *v,
            Violation::EdgeUses {
                shell,
                edge,
                fault: EdgeUseFault::Orientation,
            }
        );
    }
}

#[test]
fn s3_two_disconnected_face_sets_in_one_shell() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let near = ring(&mut m, &rect(0.0, 0.0, 10.0, 10.0, true));
    let far = ring(&mut m, &rect(20.0, 20.0, 30.0, 30.0, true));
    let uses: Vec<FaceHandle> = [near, far]
        .iter()
        .map(|r| {
            FaceHandle::forward(m.raw().add_face(Face::new(
                surface,
                vec![Loop::new(r.coedges.clone())],
                tol,
            )))
        })
        .collect();
    let (body, shell) = body_of(&mut m, uses, BodyKind::Sheet);
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("S3", shell.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::ShellDisconnected {
            shell,
            components: 2,
        }
    );
}

/// S4: the sample cylinder without its top cap. The top circle is left
/// with one coedge, which is both the wrong count for a solid (S2) and
/// the hole that leaves the shell open (S4).
#[test]
fn s4_a_solids_edge_with_one_coedge() {
    let mut m = Model::default();
    let c = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let faces = m.faces(c).unwrap();
    let top_circle = m.edges(c).unwrap()[2].id;
    let (body, shell) = body_of(&mut m, vec![faces[0], faces[1]], BodyKind::Solid);
    let report = check(&m, body, Level::Fast);
    assert_lines(
        &report,
        &[("S2", shell.to_string()), ("S4", shell.to_string())],
    );
    assert_eq!(
        report.violations()[0],
        Violation::EdgeUses {
            shell,
            edge: top_circle,
            fault: EdgeUseFault::Count { coedges: 1 },
        }
    );
    assert_eq!(
        report.violations()[1],
        Violation::ShellOpen {
            shell,
            edge: top_circle,
        }
    );
}

#[test]
fn b3_a_wire_with_a_shell() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let r = ring(&mut m, &rect(0.0, 0.0, 10.0, 10.0, true));
    let face = m
        .raw()
        .add_face(Face::new(surface, vec![Loop::new(r.coedges.clone())], tol));
    let shell = m
        .raw()
        .add_shell(Shell::new(vec![FaceHandle::forward(face)]));
    let id = m.raw().add_body(BodyEntity::new(
        BodyKind::Wire,
        vec![ShellHandle::forward(shell)],
        Vec::new(),
        Vec::new(),
    ));
    let report = check(&m, Body::forward(id), Level::Fast);
    assert_lines(&report, &[("B3", id.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::WireMalformed {
            body: id,
            fault: WireFault::HasShell { shell },
        }
    );
}

#[test]
fn b3_a_wire_whose_free_edges_meet_three_at_a_vertex() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let hub = m.raw().add_vertex(Vertex::new(Point3::origin(), tol));
    let mut free = Vec::new();
    for direction in [Vec3::x_axis(), Vec3::y_axis(), Vec3::z_axis()] {
        let tip = m
            .raw()
            .add_vertex(Vertex::new(Point3::origin() + direction.scale(10.0), tol));
        let curve = m.add_curve(Curve::Line {
            origin: Point3::origin(),
            direction,
        });
        free.push(EdgeHandle::forward(m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve {
                curve,
                range: Interval::new(0.0, 10.0).unwrap(),
            },
            hub,
            tip,
            tol,
        ))));
    }
    let id = m.raw().add_body(BodyEntity::new(
        BodyKind::Wire,
        Vec::new(),
        free,
        Vec::new(),
    ));
    let report = check(&m, Body::forward(id), Level::Fast);
    assert_lines(&report, &[("B3", id.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::WireMalformed {
            body: id,
            fault: WireFault::VertexOverused {
                vertex: hub,
                edges: 3,
            },
        }
    );
}

/// The rows of this step run at `Fast`, so `Level::Full` reports them
/// too and no `Full` row of step 8 is claimed yet.
#[test]
fn the_fast_rows_also_run_at_full() {
    let mut m = Model::default();
    let (body, face, _) = plate(&mut m, &[rect(0.0, 0.0, 10.0, 10.0, false)]);
    let report = check(&m, body, Level::Full);
    assert_lines(&report, &[("L4", face.to_string())]);
}
