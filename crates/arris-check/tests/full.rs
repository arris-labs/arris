//! Checker `Full` (`docs/DATA-MODEL.md` §Invariants): one violation
//! test per row of E8, L5, S5, B1 and B2, the sample bodies clean at
//! `Full`, and a pair the geometry kernel does not decide landing under
//! `Report::unchecked` rather than passing or failing. The rows over a
//! torus face are in `torus.rs`.

use core::f64::consts::{PI, TAU};

use arris_check::classify::{Classification, classify_point};
use arris_check::{
    Level, Lump, LumpError, Report, ShellNestingFault, Unchecked, Violation, check, lumps,
};
use arris_debug::sample;
use arris_geom::{Curve, Curve2, NurbsCurve, Surface, SurfaceKind};
use arris_math::{
    Frame, Frame2, Handedness, Interval, Point2, Point3, Precision, UnitVec2, UnitVec3, Vec2, Vec3,
};
use arris_topo::entity::{
    Body as BodyEntity, BodyKind, Coedge, Edge, EdgeGeometry, Face, Loop, Shell, Vertex,
};
use arris_topo::{
    Body, Curve2Id, Edge as EdgeHandle, EdgeId, Face as FaceHandle, FaceId, Model, Orientation,
    Shell as ShellHandle, ShellId, SurfaceId, VertexId,
};

/// `(code, entity)` per line of the report.
fn lines(report: &Report) -> Vec<(String, String)> {
    report
        .violations()
        .iter()
        .map(|v| (v.code().to_string(), v.entity().to_string()))
        .collect()
}

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

fn ground(m: &mut Model) -> SurfaceId {
    m.add_surface(Surface::Plane {
        frame: Frame::world(),
    })
}

/// A ring of the ground plane: a vertex per corner, a line edge per side
/// and a forward coedge whose pcurve is that side at the edge's own
/// parameter. The same helper `fast_part2.rs` builds its plates from.
struct Ring {
    edges: Vec<EdgeId>,
    coedges: Vec<Coedge>,
}

fn ring(m: &mut Model, corners: &[Point2]) -> Ring {
    let world: Vec<Point3> = corners.iter().map(|p| Point3::new(p.x, p.y, 0.0)).collect();
    ring_on(m, &Frame::world(), &world)
}

/// [`ring`] on any plane: the corners are world points on it and each
/// pcurve is the side in that plane's own (u, v), at the edge's own
/// parameter.
fn ring_on(m: &mut Model, frame: &Frame, corners: &[Point3]) -> Ring {
    let tol = m.precision().default_tolerance;
    ring_at(m, frame, corners, tol)
}

/// [`ring_on`] with every vertex and edge at `tol`.
fn ring_at(m: &mut Model, frame: &Frame, corners: &[Point3], tol: f64) -> Ring {
    let vertices: Vec<_> = corners
        .iter()
        .map(|&c| m.raw().add_vertex(Vertex::new(c, tol)))
        .collect();
    ring_through(m, frame, corners, tol, vertices)
}

/// [`ring_at`] through the given vertices, one per corner — some of them
/// vertices another face already reaches.
fn ring_through(
    m: &mut Model,
    frame: &Frame,
    corners: &[Point3],
    tol: f64,
    vertices: Vec<VertexId>,
) -> Ring {
    let n = corners.len();
    let at = |p: Point3| {
        let local = frame.to_local(p);
        Point2::new(local.x, local.y)
    };
    let mut edges = Vec::with_capacity(n);
    let mut coedges = Vec::with_capacity(n);
    for k in 0..n {
        let (a, b) = (corners[k], corners[(k + 1) % n]);
        let d = b - a;
        let range = Interval::new(0.0, d.norm()).unwrap();
        let curve = m.add_curve(Curve::Line {
            origin: a,
            direction: UnitVec3::new_normalize(d),
        });
        let edge = m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve { curve, range },
            vertices[k],
            vertices[(k + 1) % n],
            tol,
        ));
        let pcurve = m.add_curve2(Curve2::Line {
            origin: at(a),
            direction: UnitVec2::new_normalize(at(b) - at(a)),
        });
        edges.push(edge);
        coedges.push(Coedge::new(edge, Orientation::Forward, pcurve));
    }
    Ring { edges, coedges }
}

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

/// A sheet body of one planar face whose one loop walks `corners`.
fn plate(m: &mut Model, corners: &[Point2]) -> (Body, FaceId) {
    let tol = m.precision().default_tolerance;
    let surface = ground(m);
    let r = ring(m, corners);
    let face = m
        .raw()
        .add_face(Face::new(surface, vec![Loop::new(r.coedges)], tol));
    let (body, _) = body_of(m, vec![FaceHandle::forward(face)], BodyKind::Sheet);
    (body, face)
}

/// The shell of `body`, with every face use turned the other way: the
/// same surfaces with their normals into the material, so the volume it
/// encloses is the negative of the original's.
fn inverted_shell(m: &mut Model, body: Body) -> ShellId {
    let uses: Vec<FaceHandle> = m
        .faces(body)
        .unwrap()
        .iter()
        .map(|f| FaceHandle::new(f.id, f.orientation.flipped()))
        .collect();
    m.raw().add_shell(Shell::new(uses))
}

/// A solid body over `shells`, as they are.
fn solid_of(m: &mut Model, shells: Vec<ShellId>) -> Body {
    let uses = shells.into_iter().map(ShellHandle::forward).collect();
    Body::forward(m.raw().add_body(BodyEntity::solid(uses)))
}

#[test]
fn the_sample_bodies_are_clean_at_full_and_decide_every_row() {
    let mut m = Model::default();
    let bodies = [
        sample::unit_box(&mut m).unwrap(),
        sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap(),
        sample::cylinder(&mut m, 4.0, 12.0).unwrap(),
    ];
    for body in bodies {
        let report = check(&m, body, Level::Full);
        assert!(report.is_ok(), "{body}:\n{report}");
        assert!(report.unchecked().is_empty(), "{body}:\n{report}");
    }
}

/// The B-spline probe box: the intersector has no closed form for a plane
/// against a NURBS surface, so the four pairs its NURBS face shares an
/// edge with are listed as undecided — not passed, and not a violation.
/// The fifth pair, with the opposite face, is decided by the boxes: they
/// are apart, so the faces share no point and no intersector is asked.
#[test]
fn a_nurbs_face_pair_is_unchecked_and_not_a_violation() {
    let mut m = Model::default();
    let body =
        sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert_eq!(report.unchecked().len(), 4, "{report}");
    for u in report.unchecked() {
        assert_eq!(u.code(), "S5");
        let Unchecked::FacePair { kinds, .. } = *u else {
            panic!("{u}")
        };
        assert!(
            kinds.0 == SurfaceKind::Nurbs || kinds.1 == SurfaceKind::Nurbs,
            "{u}"
        );
    }
    assert!(report.to_string().contains("S5? "), "{report}");
    // `Fast` neither runs the row nor claims it decided anything.
    assert!(check(&m, body, Level::Fast).unchecked().is_empty());
}

/// E8: a degree-1 NURBS whose control polygon crosses itself, on the one
/// free edge of a wire body. The crossing is off the sample grid, so one
/// pair of polyline segments meets and no neighbour of it does.
#[test]
fn e8_a_nurbs_edge_that_crosses_itself() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let corners = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 0.0),
        Point3::new(12.0, 0.0, 0.0),
        Point3::new(0.0, 7.0, 0.0),
    ];
    let curve = m.add_curve(Curve::Nurbs(
        NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 2.0, 3.0, 3.0],
            corners.to_vec(),
            vec![1.0; 4],
        )
        .unwrap(),
    ));
    let start = m.raw().add_vertex(Vertex::new(corners[0], tol));
    let end = m.raw().add_vertex(Vertex::new(corners[3], tol));
    let edge = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve,
            range: Interval::new(0.0, 3.0).unwrap(),
        },
        start,
        end,
        tol,
    ));
    let id = m.raw().add_body(BodyEntity::new(
        BodyKind::Wire,
        Vec::new(),
        vec![EdgeHandle::forward(edge)],
        Vec::new(),
    ));
    let body = Body::forward(id);
    // The row is `Full`: nothing is claimed at `Fast`.
    assert!(check(&m, body, Level::Fast).is_ok());
    let report = check(&m, body, Level::Full);
    assert_lines(&report, &[("E8", edge.to_string())]);
    let Violation::EdgeSelfIntersects { t0, t1, .. } = report.violations()[0] else {
        panic!("{report}")
    };
    // The crossing is at (84/19, 84/19): four fifths of the way along the
    // first leg, and two thirds back along the third.
    assert!(
        (0.4 < t0 && t0 < 0.5) && (2.6 < t1 && t1 < 2.7),
        "{t0} {t1}"
    );
}

/// L5: a loop that crosses itself in (u, v) but still turns positively,
/// so L4 is happy and L5 alone reports it.
#[test]
fn l5_a_loop_that_crosses_itself() {
    let mut m = Model::default();
    let (body, face) = plate(
        &mut m,
        &[p2(0.0, 0.0), p2(0.0, 8.0), p2(12.0, 0.0), p2(10.0, 10.0)],
    );
    assert!(check(&m, body, Level::Fast).is_ok());
    let report = check(&m, body, Level::Full);
    assert_lines(&report, &[("L5", face.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::LoopsIntersect {
            face,
            loop_a: 0,
            loop_b: 0,
        }
    );
}

/// S5: a zero-thickness sheet — the same square as two faces, one on the
/// plane seen from above and one from below, bounded by the same four
/// edges. Every row but S5 is satisfied, and the two faces coincide
/// everywhere rather than meeting along their shared edges.
#[test]
fn s5_two_coincident_faces_in_one_shell() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let up = ground(&mut m);
    let down = m.add_surface(Surface::Plane {
        frame: Frame::new(Point3::origin(), -Vec3::z(), Vec3::x()).unwrap(),
    });
    let corners = [p2(0.0, 0.0), p2(10.0, 0.0), p2(10.0, 10.0), p2(0.0, 10.0)];
    let r = ring(&mut m, &corners);
    // The same ring walked backwards, in the lower face's (u, v), which
    // is `(x, −y)`.
    let mut back = Vec::with_capacity(corners.len());
    for k in (0..corners.len()).rev() {
        let (a, b) = (corners[k], corners[(k + 1) % corners.len()]);
        let d = b - a;
        let pcurve = m.add_curve2(Curve2::Line {
            origin: p2(a.x, -a.y),
            direction: UnitVec2::new_normalize(Vec2::new(d.x, -d.y)),
        });
        back.push(Coedge::new(r.edges[k], Orientation::Reversed, pcurve));
    }
    let face_up = m
        .raw()
        .add_face(Face::new(up, vec![Loop::new(r.coedges)], tol));
    let face_down = m
        .raw()
        .add_face(Face::new(down, vec![Loop::new(back)], tol));
    let (body, shell) = body_of(
        &mut m,
        vec![FaceHandle::forward(face_up), FaceHandle::forward(face_down)],
        BodyKind::Sheet,
    );
    assert!(check(&m, body, Level::Fast).is_ok());
    let report = check(&m, body, Level::Full);
    assert_lines(&report, &[("S5", shell.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::FacesIntersect {
            shell,
            face_a: face_up,
            face_b: face_down,
        }
    );
}

/// S5 through the transversal arm: a vertical face standing through the
/// middle of a horizontal one. Their planes cross along a line interior
/// to both faces, and the two share no edge to excuse it — nor any edge
/// at all, which is what S3 says alongside.
#[test]
fn s5_two_faces_that_cross_along_the_line_of_their_planes() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let ground_surface = ground(&mut m);
    let flat = ring(
        &mut m,
        &[p2(0.0, 0.0), p2(10.0, 0.0), p2(10.0, 10.0), p2(0.0, 10.0)],
    );
    // The plane `y = 5`, its (u, v) the world's `x` and `z + 5`.
    let frame = Frame::new(Point3::new(0.0, 5.0, -5.0), -Vec3::y(), Vec3::x()).unwrap();
    let upright_surface = m.add_surface(Surface::Plane { frame });
    let upright = ring_on(
        &mut m,
        &frame,
        &[
            Point3::new(0.0, 5.0, -5.0),
            Point3::new(10.0, 5.0, -5.0),
            Point3::new(10.0, 5.0, 5.0),
            Point3::new(0.0, 5.0, 5.0),
        ],
    );
    let face_flat = m.raw().add_face(Face::new(
        ground_surface,
        vec![Loop::new(flat.coedges)],
        tol,
    ));
    let face_upright = m.raw().add_face(Face::new(
        upright_surface,
        vec![Loop::new(upright.coedges)],
        tol,
    ));
    let (body, shell) = body_of(
        &mut m,
        vec![
            FaceHandle::forward(face_flat),
            FaceHandle::forward(face_upright),
        ],
        BodyKind::Sheet,
    );
    let s = shell.to_string();
    // S3: two faces sharing no edge are two components, which is the
    // price of a pair that meets nowhere it is allowed to.
    assert_lines(
        &check(&m, body, Level::Full),
        &[("S3", s.clone()), ("S5", s)],
    );
    assert_eq!(
        check(&m, body, Level::Full).violations()[1],
        Violation::FacesIntersect {
            shell,
            face_a: face_flat,
            face_b: face_upright,
        }
    );
}

/// S5 through a meeting in points alone (ADR-0008): a plane face touching a sphere
/// face at one point interior to both — the sphere's equator at `u = π/2`,
/// off its seam and its poles — with nothing shared to excuse it. S3
/// says alongside that two faces sharing no edge are two components.
#[test]
fn s5_a_plane_touching_a_sphere_inside_both_faces() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let ball = sample::sphere(&mut m, Point3::origin(), 3.0).unwrap();
    let face_ball = m.faces(ball).unwrap()[0].id;
    // The plane `y = 3`, its (u, v) the world's `x` and `−z`.
    let frame = Frame::new(Point3::new(0.0, 3.0, 0.0), Vec3::y(), Vec3::x()).unwrap();
    let surface = m.add_surface(Surface::Plane { frame });
    let r = ring_on(
        &mut m,
        &frame,
        &[
            Point3::new(-2.0, 3.0, 2.0),
            Point3::new(2.0, 3.0, 2.0),
            Point3::new(2.0, 3.0, -2.0),
            Point3::new(-2.0, 3.0, -2.0),
        ],
    );
    let face_plane = m
        .raw()
        .add_face(Face::new(surface, vec![Loop::new(r.coedges)], tol));
    let (body, shell) = body_of(
        &mut m,
        vec![
            FaceHandle::forward(face_ball),
            FaceHandle::forward(face_plane),
        ],
        BodyKind::Sheet,
    );
    let report = check(&m, body, Level::Full);
    let s = shell.to_string();
    assert_lines(&report, &[("S3", s.clone()), ("S5", s)]);
    assert_eq!(
        report.violations()[1],
        Violation::FacesIntersect {
            shell,
            face_a: face_ball,
            face_b: face_plane,
        }
    );
    assert!(report.unchecked().is_empty(), "{report}");
}

/// The same touch at a vertex both faces reach: the plane `z = −3`
/// touches the sphere at its south pole, and the plane face's loop goes
/// through the pole's own vertex — shared through no edge, as two cones
/// closing on one apex share theirs — so S5 has nothing to report.
#[test]
fn s5_a_touch_at_a_vertex_both_faces_reach_passes() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let ball = sample::sphere(&mut m, Point3::origin(), 3.0).unwrap();
    let face_ball = m.faces(ball).unwrap()[0].id;
    let south = m
        .edges(ball)
        .unwrap()
        .iter()
        .map(|e| m.edge(e.id).unwrap().start())
        .find(|&v| m.vertex(v).unwrap().point().z < 0.0)
        .expect("the south pole");
    let pole = m.vertex(south).unwrap().point();
    let frame = Frame::new(pole, -Vec3::z(), Vec3::x()).unwrap();
    let surface = m.add_surface(Surface::Plane { frame });
    let corners = [
        pole,
        Point3::new(0.0, 4.0, -3.0),
        Point3::new(4.0, 4.0, -3.0),
        Point3::new(4.0, 0.0, -3.0),
    ];
    let mut vertices = vec![south];
    vertices.extend(
        corners[1..]
            .iter()
            .map(|&c| m.raw().add_vertex(Vertex::new(c, tol))),
    );
    let r = ring_through(&mut m, &frame, &corners, tol, vertices);
    let face_plane = m
        .raw()
        .add_face(Face::new(surface, vec![Loop::new(r.coedges)], tol));
    let (body, shell) = body_of(
        &mut m,
        vec![
            FaceHandle::forward(face_ball),
            FaceHandle::forward(face_plane),
        ],
        BodyKind::Sheet,
    );
    let report = check(&m, body, Level::Full);
    assert_lines(&report, &[("S3", shell.to_string())]);
    assert!(report.unchecked().is_empty(), "{report}");
}

/// The tolerance the loose tests' entities are at: ten times the model's
/// default.
const LOOSE: f64 = 1e-6;

/// S5 by the faces' own tolerance: the zero-thickness sheet of
/// `s5_two_coincident_faces_in_one_shell` with the lower face's plane
/// lifted 5e-7 — five times the model's default tolerance, half the
/// faces'. Every entity is at `LOOSE`, so the lift is within the edges'
/// and vertices' tolerance and the faces coincide by theirs; by the
/// model's default the planes would be apart and S5 silent.
#[test]
fn s5_two_faces_that_coincide_within_their_own_tolerance() {
    let mut m = Model::default();
    let lift = 5e-7;
    let up = ground(&mut m);
    let down = m.add_surface(Surface::Plane {
        frame: Frame::new(Point3::new(0.0, 0.0, lift), -Vec3::z(), Vec3::x()).unwrap(),
    });
    let corners = [p2(0.0, 0.0), p2(10.0, 0.0), p2(10.0, 10.0), p2(0.0, 10.0)];
    let world: Vec<Point3> = corners.iter().map(|p| Point3::new(p.x, p.y, 0.0)).collect();
    let r = ring_at(&mut m, &Frame::world(), &world, LOOSE);
    // The same ring walked backwards, in the lower face's (u, v), which
    // is `(x, −y)`.
    let mut back = Vec::with_capacity(corners.len());
    for k in (0..corners.len()).rev() {
        let (a, b) = (corners[k], corners[(k + 1) % corners.len()]);
        let d = b - a;
        let pcurve = m.add_curve2(Curve2::Line {
            origin: p2(a.x, -a.y),
            direction: UnitVec2::new_normalize(Vec2::new(d.x, -d.y)),
        });
        back.push(Coedge::new(r.edges[k], Orientation::Reversed, pcurve));
    }
    let face_up = m
        .raw()
        .add_face(Face::new(up, vec![Loop::new(r.coedges)], LOOSE));
    let face_down = m
        .raw()
        .add_face(Face::new(down, vec![Loop::new(back)], LOOSE));
    let (body, shell) = body_of(
        &mut m,
        vec![FaceHandle::forward(face_up), FaceHandle::forward(face_down)],
        BodyKind::Sheet,
    );
    let fast = check(&m, body, Level::Fast);
    assert!(fast.is_ok(), "{fast}");
    assert_lines(&check(&m, body, Level::Full), &[("S5", shell.to_string())]);
}

/// B1 by the faces' own tolerance: a hollow box whose cavity's top is
/// 5e-7 under the outer top — a wall thinner than its faces' tolerance —
/// built in a model whose default is `LOOSE` and imported, tolerances
/// and all, into one whose default is ten times finer. By the faces'
/// tolerance the void's top meets the outer top, an overlap; by the
/// model's default the two planes are apart and the nesting passes.
/// `classify_point` decides the same body by the same tolerances: a
/// point in the wall is on the outer top, one in the cavity is outside
/// and one in the material inside.
#[test]
fn b1_a_wall_thinner_than_its_faces_tolerance_is_an_overlap() {
    let mut loose = Model::new(Precision {
        default_tolerance: LOOSE,
        ..Precision::DEFAULT
    })
    .unwrap();
    let outer =
        sample::cuboid(&mut loose, Point3::origin(), Point3::new(10.0, 10.0, 10.0)).unwrap();
    let cavity = sample::cuboid(
        &mut loose,
        Point3::new(2.0, 2.0, 2.0),
        Point3::new(8.0, 8.0, 10.0 - 5e-7),
    )
    .unwrap();
    let mut m = Model::default();
    let (outer, _) = m.import(&loose, outer).unwrap();
    let (cavity, _) = m.import(&loose, cavity).unwrap();
    let outer_shell = m.shells(outer).unwrap()[0].id;
    let void = inverted_shell(&mut m, cavity);
    let body = solid_of(&mut m, vec![outer_shell, void]);
    let fast = check(&m, body, Level::Fast);
    assert!(fast.is_ok(), "{fast}");
    let full = check(&m, body, Level::Full);
    assert_eq!(
        only_nesting_fault(&full),
        ShellNestingFault::Overlap {
            shells: [outer_shell, void]
        },
        "{full}"
    );
    let at = |x, y, z| classify_point(&m, body, Point3::new(x, y, z)).unwrap();
    assert!(matches!(at(5.0, 5.0, 10.0 - 2.5e-7), Classification::On(_)));
    assert_eq!(at(5.0, 5.0, 5.0), Classification::Outside);
    assert_eq!(at(1.0, 5.0, 5.0), Classification::Inside);
}

/// A (u, v) line through `(u, v)`, along `u` or along `v`, at unit speed.
fn uv_line(m: &mut Model, u: f64, v: f64, along_u: bool) -> Curve2Id {
    m.add_curve2(Curve2::Line {
        origin: p2(u, v),
        direction: if along_u {
            Vec2::x_axis()
        } else {
            Vec2::y_axis()
        },
    })
}

/// S5 on a periodic surface: two faces on one cylinder, the same patch
/// `[0.5, 1.5] × [0, 10]` written a period along, in `u + 2π`, the second
/// used the other way. A point of one face projected onto the surface
/// comes back in `[0, 2π)`, a period from the other face's loops, so only
/// a side test that tries the period sees the two coincide.
#[test]
fn s5_two_coincident_faces_whose_loops_are_a_period_along() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let (r, h, a, b) = (4.0, 10.0, 0.5, 1.5);
    let surface = m.add_surface(Surface::Cylinder {
        frame: Frame::world(),
        radius: r,
    });
    let at = |angle: f64, z: f64| Point3::new(r * angle.cos(), r * angle.sin(), z);
    let [v0, v1, v2, v3] = [at(a, 0.0), at(b, 0.0), at(b, h), at(a, h)]
        .map(|p| m.raw().add_vertex(Vertex::new(p, tol)));
    let arc = |m: &mut Model, z: f64, from, to| {
        let curve = m.add_curve(Curve::Circle {
            frame: Frame::world().with_origin(Point3::new(0.0, 0.0, z)),
            radius: r,
        });
        let range = Interval::new(a, b).unwrap();
        m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve { curve, range },
            from,
            to,
            tol,
        ))
    };
    let ruling = |m: &mut Model, angle: f64, from, to| {
        let curve = m.add_curve(Curve::Line {
            origin: at(angle, 0.0),
            direction: Vec3::z_axis(),
        });
        let range = Interval::new(0.0, h).unwrap();
        m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve { curve, range },
            from,
            to,
            tol,
        ))
    };
    let bottom = arc(&mut m, 0.0, v0, v1);
    let right = ruling(&mut m, b, v1, v2);
    let top = arc(&mut m, h, v3, v2);
    let left = ruling(&mut m, a, v0, v3);
    let coedges = vec![
        Coedge::new(
            bottom,
            Orientation::Forward,
            uv_line(&mut m, TAU, 0.0, true),
        ),
        Coedge::new(
            right,
            Orientation::Forward,
            uv_line(&mut m, TAU + b, 0.0, false),
        ),
        Coedge::new(top, Orientation::Reversed, uv_line(&mut m, TAU, h, true)),
        Coedge::new(
            left,
            Orientation::Reversed,
            uv_line(&mut m, TAU + a, 0.0, false),
        ),
    ];
    let face_a = m
        .raw()
        .add_face(Face::new(surface, vec![Loop::new(coedges.clone())], tol));
    let face_b = m
        .raw()
        .add_face(Face::new(surface, vec![Loop::new(coedges)], tol));
    let (body, shell) = body_of(
        &mut m,
        vec![
            FaceHandle::forward(face_a),
            FaceHandle::new(face_b, Orientation::Reversed),
        ],
        BodyKind::Sheet,
    );
    assert!(check(&m, body, Level::Fast).is_ok());
    let report = check(&m, body, Level::Full);
    assert_lines(&report, &[("S5", shell.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::FacesIntersect {
            shell,
            face_a,
            face_b,
        }
    );
}

/// The shell of a solid cylinder of radius 4 about the z axis from `z0`
/// to `z0 + height`, as `sample::cylinder` builds it but with the wall's
/// loop written a period along, in `u ∈ [2π, 4π]`.
fn cylinder_a_period_along(m: &mut Model, z0: f64, height: f64) -> ShellId {
    let tol = m.precision().default_tolerance;
    let r = 4.0;
    let base = Frame::world().with_origin(Point3::new(0.0, 0.0, z0));
    let top = Frame::world().with_origin(Point3::new(0.0, 0.0, z0 + height));
    let wall = m.add_surface(Surface::Cylinder {
        frame: Frame::world(),
        radius: r,
    });
    let bottom_plane = m.add_surface(Surface::Plane { frame: base });
    let top_plane = m.add_surface(Surface::Plane { frame: top });
    let v0 = m
        .raw()
        .add_vertex(Vertex::new(Point3::new(r, 0.0, z0), tol));
    let v1 = m
        .raw()
        .add_vertex(Vertex::new(Point3::new(r, 0.0, z0 + height), tol));
    let circle = |m: &mut Model, frame: Frame, v| {
        let curve = m.add_curve(Curve::Circle { frame, radius: r });
        let range = Interval::TURN;
        m.raw()
            .add_edge(Edge::new(EdgeGeometry::Curve { curve, range }, v, v, tol))
    };
    let e_bottom = circle(m, base, v0);
    let e_top = circle(m, top, v1);
    let seam_curve = m.add_curve(Curve::Line {
        origin: Point3::new(r, 0.0, z0),
        direction: Vec3::z_axis(),
    });
    let e_seam = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: seam_curve,
            range: Interval::new(0.0, height).unwrap(),
        },
        v0,
        v1,
        tol,
    ));
    let wall_loop = vec![
        Coedge::new(e_bottom, Orientation::Forward, uv_line(m, TAU, z0, true)),
        Coedge::new(
            e_seam,
            Orientation::Forward,
            uv_line(m, 2.0 * TAU, z0, false),
        ),
        Coedge::new(
            e_top,
            Orientation::Reversed,
            uv_line(m, TAU, z0 + height, true),
        ),
        Coedge::new(e_seam, Orientation::Reversed, uv_line(m, TAU, z0, false)),
    ];
    let wall_face = m
        .raw()
        .add_face(Face::new(wall, vec![Loop::new(wall_loop)], tol));
    let cap = |m: &mut Model, plane, edge| {
        let pcurve = m.add_curve2(Curve2::Circle {
            frame: Frame2::identity(),
            radius: r,
        });
        let coedge = Coedge::new(edge, Orientation::Forward, pcurve);
        m.raw()
            .add_face(Face::new(plane, vec![Loop::new(vec![coedge])], tol))
    };
    let bottom_face = cap(m, bottom_plane, e_bottom);
    let top_face = cap(m, top_plane, e_top);
    m.raw().add_shell(Shell::new(vec![
        FaceHandle::forward(wall_face),
        FaceHandle::new(bottom_face, Orientation::Reversed),
        FaceHandle::forward(top_face),
    ]))
}

/// B1 on a periodic surface: two solid cylinders of one radius on one
/// axis, overlapping over `z ∈ [6, 12]`, their walls' loops written a
/// period along. Only the two walls share interior — every other pair of
/// faces meets on a cap's boundary or not at all — so the overlap is seen
/// only by a side test that tries the period.
#[test]
fn b1_two_cylinders_overlapping_on_walls_written_a_period_along() {
    let mut m = Model::default();
    let a = cylinder_a_period_along(&mut m, 0.0, 12.0);
    let b = cylinder_a_period_along(&mut m, 6.0, 12.0);
    for shell in [a, b] {
        let one = solid_of(&mut m, vec![shell]);
        let report = check(&m, one, Level::Full);
        assert!(report.is_ok(), "{report}");
        assert!(report.unchecked().is_empty(), "{report}");
    }
    let body = solid_of(&mut m, vec![a, b]);
    let report = check(&m, body, Level::Full);
    assert_eq!(
        only_nesting_fault(&report),
        ShellNestingFault::Overlap { shells: [a, b] },
        "{report}"
    );
}

/// S5's shared-boundary excuse on a closed edge stored a period along. A
/// cup's wall and bottom share the rim, a circle turned a half turn and
/// stored over `[π, 3π]` at a tolerance of 1e-3; the bottom's plane is
/// 5e-4 above the rim, so it crosses the wall in a whole circle inside
/// both faces and within the rim's tolerance all round. A point of that
/// circle projects onto the rim's curve in `[0, 2π)`, and half of those
/// parameters are a period from the edge's range: they are on the edge
/// only when the period is tried.
#[test]
fn s5_a_rim_stored_a_period_along_excuses_the_faces_it_bounds() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let (r, h, lift, loose) = (4.0, 12.0, 5e-4, 1e-3);
    let half_turn = |z: f64| Frame::new(Point3::new(0.0, 0.0, z), Vec3::z(), -Vec3::x()).unwrap();
    let wall = m.add_surface(Surface::Cylinder {
        frame: Frame::world(),
        radius: r,
    });
    let bottom = m.add_surface(Surface::Plane {
        frame: Frame::world().with_origin(Point3::new(0.0, 0.0, lift)),
    });
    let v0 = m
        .raw()
        .add_vertex(Vertex::new(Point3::new(r, 0.0, 0.0), loose));
    let v1 = m.raw().add_vertex(Vertex::new(Point3::new(r, 0.0, h), tol));
    let range = Interval::new(PI, 3.0 * PI).unwrap();
    let rim_curve = m.add_curve(Curve::Circle {
        frame: half_turn(0.0),
        radius: r,
    });
    let top_curve = m.add_curve(Curve::Circle {
        frame: half_turn(h),
        radius: r,
    });
    let seam_curve = m.add_curve(Curve::Line {
        origin: Point3::new(r, 0.0, 0.0),
        direction: Vec3::z_axis(),
    });
    let rim = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: rim_curve,
            range,
        },
        v0,
        v0,
        loose,
    ));
    let seam = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: seam_curve,
            range: Interval::new(0.0, h).unwrap(),
        },
        v0,
        v1,
        tol,
    ));
    let top = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: top_curve,
            range,
        },
        v1,
        v1,
        tol,
    ));
    // The wall's loop is in `[0, 2π]`: the circles' parameter `t` is the
    // angle `t − π`.
    let wall_loop = vec![
        Coedge::new(rim, Orientation::Forward, uv_line(&mut m, -PI, 0.0, true)),
        Coedge::new(seam, Orientation::Forward, uv_line(&mut m, TAU, 0.0, false)),
        Coedge::new(top, Orientation::Reversed, uv_line(&mut m, -PI, h, true)),
        Coedge::new(
            seam,
            Orientation::Reversed,
            uv_line(&mut m, 0.0, 0.0, false),
        ),
    ];
    let wall_face = m
        .raw()
        .add_face(Face::new(wall, vec![Loop::new(wall_loop)], tol));
    // The bottom's loop is the rim widened by the lift, at the same
    // parameter.
    let widened = m.add_curve2(Curve2::Circle {
        frame: Frame2::new(Point2::origin(), -Vec2::x(), Handedness::Right).unwrap(),
        radius: r + lift,
    });
    let bottom_face = m.raw().add_face(Face::new(
        bottom,
        vec![Loop::new(vec![Coedge::new(
            rim,
            Orientation::Forward,
            widened,
        )])],
        tol,
    ));
    let (body, _) = body_of(
        &mut m,
        vec![
            FaceHandle::forward(wall_face),
            FaceHandle::new(bottom_face, Orientation::Reversed),
        ],
        BodyKind::Sheet,
    );
    assert!(check(&m, body, Level::Fast).is_ok());
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
}

/// B2, and B1's `NoOuter` with it: a cylinder whose every face use is
/// turned inwards encloses the negative of its volume, so it is no
/// solid at all.
#[test]
fn b2_an_inside_out_cylinder_encloses_negative_volume() {
    let mut m = Model::default();
    let c = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let shell = inverted_shell(&mut m, c);
    let body = solid_of(&mut m, vec![shell]);
    assert!(check(&m, body, Level::Fast).is_ok());
    let report = check(&m, body, Level::Full);
    let id = body.id.to_string();
    assert_lines(&report, &[("B1", id.clone()), ("B2", id)]);
    let Violation::NonPositiveVolume { volume, .. } = report.violations()[1] else {
        panic!("{report}")
    };
    let expected = -core::f64::consts::PI * 16.0 * 12.0;
    assert!(
        (volume - expected).abs() < 1e-9 * expected.abs(),
        "{volume} vs {expected}"
    );
    assert_eq!(
        report.violations()[0],
        Violation::ShellNesting {
            body: body.id,
            fault: ShellNestingFault::NoOuter,
        }
    );
}

/// B2's value is the Gauss volume the fixtures assert: the sample box's
/// is its extents, the cylinder's is `πr²h`.
#[test]
fn the_gauss_volume_of_a_clean_solid_is_its_volume() {
    let mut m = Model::default();
    for (body, expected) in [
        (
            sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap(),
            12000.0,
        ),
        (
            sample::cylinder(&mut m, 4.0, 12.0).unwrap(),
            core::f64::consts::PI * 16.0 * 12.0,
        ),
    ] {
        // An inverted copy reports the value through B2, which is how the
        // sign test reads it back.
        let shell = inverted_shell(&mut m, body);
        let inverted = solid_of(&mut m, vec![shell]);
        let report = check(&m, inverted, Level::Full);
        let Some(Violation::NonPositiveVolume { volume, .. }) = report
            .violations()
            .iter()
            .find(|v| v.code() == "B2")
            .cloned()
        else {
            panic!("{report}")
        };
        assert!(
            (-volume - expected).abs() < 1e-9 * expected,
            "{volume} vs {expected}"
        );
    }
}

/// The shell of a sample cuboid from `min` to `max`, its normals out of
/// it (`void` false) or turned into it (`void` true).
fn cuboid_shell(m: &mut Model, min: [f64; 3], max: [f64; 3], void: bool) -> ShellId {
    let body = sample::cuboid(
        m,
        Point3::new(min[0], min[1], min[2]),
        Point3::new(max[0], max[1], max[2]),
    )
    .unwrap();
    if void {
        inverted_shell(m, body)
    } else {
        m.shells(body).unwrap()[0].id
    }
}

/// The B1 line of `report`, the only line it has.
fn only_nesting_fault(report: &Report) -> ShellNestingFault {
    let [Violation::ShellNesting { fault, .. }] = report.violations() else {
        panic!("{report}")
    };
    fault.clone()
}

/// B1: two shells enclosing positive volume apart from each other are two
/// lumps of one solid (ADR-0006), each with no void.
#[test]
fn b1_two_disjoint_outer_shells_are_two_lumps() {
    let mut m = Model::default();
    let near = cuboid_shell(&mut m, [0.0; 3], [10.0; 3], false);
    let far = cuboid_shell(&mut m, [100.0; 3], [110.0; 3], false);
    let body = solid_of(&mut m, vec![near, far]);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
    assert_eq!(
        lumps(&m, body).unwrap(),
        [
            Lump {
                outer: ShellHandle::forward(near),
                voids: Vec::new(),
            },
            Lump {
                outer: ShellHandle::forward(far),
                voids: Vec::new(),
            },
        ]
    );
}

/// B1: two outer shells that overlap — the faces of one cross the faces
/// of the other — are no lumps, whichever vertex each would be placed by.
#[test]
fn b1_two_overlapping_outer_shells_meet() {
    let mut m = Model::default();
    let a = cuboid_shell(&mut m, [0.0; 3], [10.0; 3], false);
    let b = cuboid_shell(&mut m, [5.0; 3], [15.0; 3], false);
    let body = solid_of(&mut m, vec![a, b]);
    let report = check(&m, body, Level::Full);
    assert_eq!(
        only_nesting_fault(&report),
        ShellNestingFault::Overlap { shells: [a, b] }
    );
    assert!(matches!(
        lumps(&m, body),
        Err(LumpError::Nesting {
            fault: ShellNestingFault::Overlap { .. },
            ..
        })
    ));
}

/// B1: a hollow box — an outer shell with a void inside it — and a box
/// sitting in the cavity, clear of its walls, are two lumps: the hollow
/// box with its void, and the box inside, whose innermost container is
/// the void.
#[test]
fn b1_a_box_in_the_cavity_of_a_hollow_box_is_two_lumps() {
    let mut m = Model::default();
    let outer = cuboid_shell(&mut m, [0.0; 3], [10.0; 3], false);
    let void = cuboid_shell(&mut m, [2.0; 3], [8.0; 3], true);
    let inside = cuboid_shell(&mut m, [4.0; 3], [6.0; 3], false);
    let body = solid_of(&mut m, vec![outer, void, inside]);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
    assert_eq!(
        lumps(&m, body).unwrap(),
        [
            Lump {
                outer: ShellHandle::forward(outer),
                voids: vec![ShellHandle::forward(void)],
            },
            Lump {
                outer: ShellHandle::forward(inside),
                voids: Vec::new(),
            },
        ]
    );
}

/// B1: a void whose innermost container is another void — a cavity
/// inside a cavity with no material between — is no lump's.
#[test]
fn b1_a_void_inside_a_void() {
    let mut m = Model::default();
    let outer = cuboid_shell(&mut m, [0.0; 3], [10.0; 3], false);
    let void = cuboid_shell(&mut m, [1.0; 3], [9.0; 3], true);
    let inner = cuboid_shell(&mut m, [3.0; 3], [7.0; 3], true);
    let body = solid_of(&mut m, vec![outer, void, inner]);
    assert_eq!(
        only_nesting_fault(&check(&m, body, Level::Full)),
        ShellNestingFault::VoidInVoid {
            shell: inner,
            container: void,
        }
    );
}

/// B1: an outer shell whose innermost container is another outer shell —
/// material inside material with no cavity between.
#[test]
fn b1_an_outer_shell_inside_another() {
    let mut m = Model::default();
    let big = cuboid_shell(&mut m, [0.0; 3], [10.0; 3], false);
    let small = cuboid_shell(&mut m, [3.0; 3], [7.0; 3], false);
    let body = solid_of(&mut m, vec![big, small]);
    assert_eq!(
        only_nesting_fault(&check(&m, body, Level::Full)),
        ShellNestingFault::OuterInOuter {
            shell: small,
            container: big,
        }
    );
}

/// B1: a void inside the B-spline probe box. No ray has a closed form
/// against its NURBS face, so where the void lies is not known: an
/// unchecked row, never a pass and never a violation, and `lumps` says
/// the same.
#[test]
fn b1_a_shell_no_ray_classifies_is_unchecked() {
    let mut m = Model::default();
    let probe =
        sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let outer = m.shells(probe).unwrap()[0].id;
    let void = cuboid_shell(&mut m, [10.0, 10.0, 2.0], [20.0, 20.0, 8.0], true);
    let body = solid_of(&mut m, vec![outer, void]);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(
        report.unchecked().iter().any(|u| u.code() == "B1"),
        "{report}"
    );
    assert!(matches!(lumps(&m, body), Err(LumpError::Undecided { .. })));
}

/// `lumps` of what is not a solid, and of a solid B1 faults, is the
/// reason.
#[test]
fn lumps_of_a_sheet_and_of_a_hollow_solid_turned_inside_out() {
    let mut m = Model::default();
    let (sheet, _) = plate(&mut m, &[p2(0.0, 0.0), p2(1.0, 0.0), p2(1.0, 1.0)]);
    assert!(matches!(
        lumps(&m, sheet),
        Err(LumpError::NotSolid {
            kind: BodyKind::Sheet,
            ..
        })
    ));
    let c = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let inverted = inverted_shell(&mut m, c);
    let body = solid_of(&mut m, vec![inverted]);
    assert!(matches!(
        lumps(&m, body),
        Err(LumpError::Nesting {
            fault: ShellNestingFault::NoOuter,
            ..
        })
    ));
}

/// B1: an inward-facing shell is a void, but only where it is inside the
/// outer shell. This one is nowhere near it, and the ray cast says so.
#[test]
fn b1_a_void_shell_outside_its_outer() {
    let mut m = Model::default();
    let outer = sample::cuboid(
        &mut m,
        Point3::new(100.0, 100.0, 100.0),
        Point3::new(140.0, 130.0, 110.0),
    )
    .unwrap();
    let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let void = inverted_shell(&mut m, cylinder);
    let outer_shell = m.shells(outer).unwrap()[0].id;
    let body = solid_of(&mut m, vec![outer_shell, void]);
    let report = check(&m, body, Level::Full);
    assert_lines(&report, &[("B1", body.id.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::ShellNesting {
            body: body.id,
            fault: ShellNestingFault::VoidOutside { shell: void },
        }
    );
    assert!(report.unchecked().is_empty(), "{report}");
}

/// A solid with no shell at all is B1's first fault, and the volume it
/// does not enclose is B2's.
#[test]
fn b1_a_solid_without_shells() {
    let mut m = Model::default();
    let id = m.raw().add_body(BodyEntity::solid(Vec::new()));
    let body = Body::forward(id);
    let report = check(&m, body, Level::Full);
    assert_lines(&report, &[("B1", id.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::ShellNesting {
            body: id,
            fault: ShellNestingFault::NoShells,
        }
    );
}

#[test]
fn the_report_is_the_same_on_two_builds_at_full() {
    let build = |m: &mut Model| {
        let c = sample::cylinder(m, 4.0, 12.0).unwrap();
        let shell = inverted_shell(m, c);
        let body = solid_of(m, vec![shell]);
        check(m, body, Level::Full).to_string()
    };
    let (mut a, mut b) = (Model::default(), Model::default());
    let text = build(&mut a);
    assert_eq!(text, build(&mut b));
    assert!(text.lines().count() >= 2, "{text}");
}
