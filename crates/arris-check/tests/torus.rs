//! Checker `Full` over a torus face — S5 and B1 of `docs/DATA-MODEL.md`
//! §Invariants. Every pair of analytic surfaces is decided now that the
//! intersector traces a torus in any pose (ADR-0019), so a face pair with
//! a torus in it is reported or passed here, never listed under
//! `Report::unchecked`. Each test asserts that list empty, which is the
//! point of the rows: before ADR-0019 every one of these pairs was
//! undecided.

use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, TAU};

use arris_check::{Level, Lump, Report, ShellNestingFault, Violation, check, lumps};
use arris_debug::sample;
use arris_geom::{Curve, Curve2, Surface};
use arris_math::{Frame, Frame2, Interval, Point2, Point3, UnitVec2, UnitVec3, Vec2, Vec3};
use arris_topo::entity::{
    Body as BodyEntity, BodyKind, Coedge, Edge, EdgeGeometry, Face, Loop, Shell, Vertex,
};
use arris_topo::{
    Body, Curve2Id, EdgeId, Face as FaceHandle, FaceId, Model, Orientation, Shell as ShellHandle,
    ShellId, VertexId,
};

/// The ring every test here is built around: a metre-scale torus at an
/// `R/r` well away from 1, the size `geom/c3-torus-pairs` poses its
/// partners at.
const R: f64 = 5.0;
const MINOR: f64 = 2.0;

/// `(code, entity)` per violation of the report.
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

/// A (u, v) line through `(u, v)`, along `u` or along `v`, at unit speed:
/// the pcurve of a curve the surface's own parametrisation carries at the
/// curve's own parameter.
fn uv_line(m: &mut Model, u: f64, v: f64, along_u: bool) -> Curve2Id {
    m.add_curve2(Curve2::Line {
        origin: Point2::new(u, v),
        direction: if along_u {
            Vec2::x_axis()
        } else {
            Vec2::y_axis()
        },
    })
}

/// The frame of the ring's tube circle at `u`: its origin the centre
/// circle's point there, its `X` the radial direction and its `Y` the
/// ring's axis, so `P(t)` is the torus at `(u, t)`. Its `Z` is the centre
/// circle's tangent reversed — the axis the elbow's pipe runs along.
fn tube_frame(u: f64) -> Frame {
    let radial = Vec3::new(u.cos(), u.sin(), 0.0);
    Frame::from_orthonormal(
        Point3::new(R * u.cos(), R * u.sin(), 0.0),
        radial,
        Vec3::z(),
        radial.cross(&Vec3::z()),
    )
    .unwrap()
}

/// A closed circle edge of `radius` on `frame`, over `[0, 2π]`, with a
/// vertex of its own where `frame`'s `X` points.
fn circle_edge(m: &mut Model, frame: Frame, radius: f64) -> EdgeId {
    let tol = m.precision().default_tolerance;
    let curve = m.add_curve(Curve::Circle { frame, radius });
    let corner = m.raw().add_vertex(Vertex::new(
        frame.origin() + frame.x().into_inner() * radius,
        tol,
    ));
    m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve,
            range: Interval::TURN,
        },
        corner,
        corner,
        tol,
    ))
}

/// A band of the ring, `[u0, u1] × [0, 2π]` in its own (u, v): a bend of
/// pipe. Its four coedges are the ones `sample::torus` gives the whole
/// ring over the shorter `u` — the `v = 0` parallel used at `v = 0` and
/// at `v = 2π`, and a tube circle at each end — so the face's normal is
/// the ring's own, out of the tube. The tube circle at `u0` is `shared`
/// when another face already carries it; the face and that edge come
/// back.
fn torus_band(m: &mut Model, u0: f64, u1: f64, shared: Option<EdgeId>) -> (FaceId, EdgeId) {
    let tol = m.precision().default_tolerance;
    let surface = m.add_surface(Surface::Torus {
        frame: Frame::world(),
        major_radius: R,
        minor_radius: MINOR,
    });
    let e_start = shared.unwrap_or_else(|| circle_edge(m, tube_frame(u0), MINOR));
    let e_end = circle_edge(m, tube_frame(u1), MINOR);
    let (v0, v1) = (
        m.edge(e_start).unwrap().start(),
        m.edge(e_end).unwrap().start(),
    );
    let parallel = m.add_curve(Curve::Circle {
        frame: Frame::world(),
        radius: R + MINOR,
    });
    let e_parallel = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: parallel,
            range: Interval::new(u0, u1).unwrap(),
        },
        v0,
        v1,
        tol,
    ));
    let p_low = uv_line(m, 0.0, 0.0, true);
    let p_end = uv_line(m, u1, 0.0, false);
    let p_high = uv_line(m, 0.0, TAU, true);
    let p_start = uv_line(m, u0, 0.0, false);
    let face = m.raw().add_face(Face::new(
        surface,
        vec![Loop::new(vec![
            Coedge::new(e_parallel, Orientation::Forward, p_low),
            Coedge::new(e_end, Orientation::Forward, p_end),
            Coedge::new(e_parallel, Orientation::Reversed, p_high),
            Coedge::new(e_start, Orientation::Reversed, p_start),
        ])],
        tol,
    ));
    (face, e_start)
}

/// The elbow's straight pipe, the pose ADR-0019's tube circle is about: the
/// cylinder of the tube's radius about the centre circle's tangent at
/// `u0`, over `v ∈ [v0, v1]` of its own parametrisation — which is
/// [`tube_frame`]'s, so the cylinder at `(t, 0)` is the ring at `(u0, t)`
/// and the two are tangent along that tube circle. `shared` is the ring's
/// own edge on it where both faces carry it, and then `v0` is 0.
fn elbow_pipe(m: &mut Model, u0: f64, v0: f64, v1: f64, shared: Option<EdgeId>) -> FaceId {
    let tol = m.precision().default_tolerance;
    let frame = tube_frame(u0);
    let surface = m.add_surface(Surface::Cylinder {
        frame,
        radius: MINOR,
    });
    let axis = frame.z().into_inner();
    let at = |v: f64| frame.with_origin(frame.origin() + axis * v);
    let e_low = shared.unwrap_or_else(|| circle_edge(m, at(v0), MINOR));
    let e_high = circle_edge(m, at(v1), MINOR);
    let (start, end) = (
        m.edge(e_low).unwrap().start(),
        m.edge(e_high).unwrap().start(),
    );
    let seam = m.add_curve(Curve::Line {
        origin: frame.origin() + frame.x().into_inner() * MINOR,
        direction: frame.z(),
    });
    let e_seam = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: seam,
            range: Interval::new(v0, v1).unwrap(),
        },
        start,
        end,
        tol,
    ));
    let p_low = uv_line(m, 0.0, v0, true);
    let p_up = uv_line(m, TAU, 0.0, false);
    let p_high = uv_line(m, 0.0, v1, true);
    let p_down = uv_line(m, 0.0, 0.0, false);
    m.raw().add_face(Face::new(
        surface,
        vec![Loop::new(vec![
            Coedge::new(e_low, Orientation::Forward, p_low),
            Coedge::new(e_seam, Orientation::Forward, p_up),
            Coedge::new(e_high, Orientation::Reversed, p_high),
            Coedge::new(e_seam, Orientation::Reversed, p_down),
        ])],
        tol,
    ))
}

/// A sheet body of `faces`, as they are, and its shell.
fn sheet(m: &mut Model, faces: Vec<FaceId>) -> (Body, ShellId) {
    let uses = faces.into_iter().map(FaceHandle::forward).collect();
    let shell = m.raw().add_shell(Shell::new(uses));
    let body = m.raw().add_body(BodyEntity::new(
        BodyKind::Sheet,
        vec![ShellHandle::forward(shell)],
        Vec::new(),
        Vec::new(),
    ));
    (Body::forward(body), shell)
}

/// A solid body over `shells`, as they are.
fn solid_of(m: &mut Model, shells: Vec<ShellId>) -> Body {
    let uses = shells.into_iter().map(ShellHandle::forward).collect();
    Body::forward(m.raw().add_body(BodyEntity::solid(uses)))
}

/// The shell of a solid cylinder of `radius` and `height` on `base`'s `Z`
/// with its base at `base`'s origin and its seam where `base`'s `X`
/// points: `sample::cylinder` on a frame of its own, which is what a pin
/// tilted against the ring's axis needs.
fn pin_shell(m: &mut Model, base: Frame, radius: f64, height: f64) -> ShellId {
    let tol = m.precision().default_tolerance;
    let top = base.with_origin(base.origin() + base.z().into_inner() * height);
    let wall = m.add_surface(Surface::Cylinder {
        frame: base,
        radius,
    });
    let bottom_plane = m.add_surface(Surface::Plane { frame: base });
    let top_plane = m.add_surface(Surface::Plane { frame: top });
    let e_bottom = circle_edge(m, base, radius);
    let e_top = circle_edge(m, top, radius);
    let seam = m.add_curve(Curve::Line {
        origin: base.origin() + base.x().into_inner() * radius,
        direction: base.z(),
    });
    let (start, end) = (
        m.edge(e_bottom).unwrap().start(),
        m.edge(e_top).unwrap().start(),
    );
    let e_seam = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: seam,
            range: Interval::new(0.0, height).unwrap(),
        },
        start,
        end,
        tol,
    ));
    let p_bottom = uv_line(m, 0.0, 0.0, true);
    let p_up = uv_line(m, TAU, 0.0, false);
    let p_top = uv_line(m, 0.0, height, true);
    let p_down = uv_line(m, 0.0, 0.0, false);
    let wall_face = m.raw().add_face(Face::new(
        wall,
        vec![Loop::new(vec![
            Coedge::new(e_bottom, Orientation::Forward, p_bottom),
            Coedge::new(e_seam, Orientation::Forward, p_up),
            Coedge::new(e_top, Orientation::Reversed, p_top),
            Coedge::new(e_seam, Orientation::Reversed, p_down),
        ])],
        tol,
    ));
    let cap = |m: &mut Model, plane, edge| {
        let pcurve = m.add_curve2(Curve2::Circle {
            frame: Frame2::identity(),
            radius,
        });
        m.raw().add_face(Face::new(
            plane,
            vec![Loop::new(vec![Coedge::new(
                edge,
                Orientation::Forward,
                pcurve,
            )])],
            tol,
        ))
    };
    let bottom_face = cap(m, bottom_plane, e_bottom);
    let top_face = cap(m, top_plane, e_top);
    m.raw().add_shell(Shell::new(vec![
        FaceHandle::forward(wall_face),
        FaceHandle::new(bottom_face, Orientation::Reversed),
        FaceHandle::forward(top_face),
    ]))
}

/// The frame of a pin tilted `tilt` from the ring's axis in the `y–z`
/// plane, its axis through `through` and its base half of `height` back
/// along that axis.
fn pin_frame(through: Point3, tilt: f64, height: f64) -> Frame {
    let axis = Vec3::new(0.0, tilt.sin(), tilt.cos());
    Frame::new(through - axis * (height / 2.0), axis, Vec3::x()).unwrap()
}

/// An oblique plane: the one through the origin holding the `x` axis and
/// tilted `tilt` from `z = 0`. It shares no axis with the ring and does
/// not hold the ring's axis, so `intersect_surfaces` traces the pair
/// (ADR-0019) instead of taking ADR-0008's closed form.
fn oblique_plane(tilt: f64) -> Frame {
    Frame::new(
        Point3::origin(),
        Vec3::new(0.0, -tilt.sin(), tilt.cos()),
        Vec3::x(),
    )
    .unwrap()
}

/// A square face of `plane`, `half` across in each direction from its
/// (u, v) origin.
fn plane_patch(m: &mut Model, plane: Frame, half: f64) -> FaceId {
    let tol = m.precision().default_tolerance;
    let surface = m.add_surface(Surface::Plane { frame: plane });
    let corners = [
        Point2::new(-half, -half),
        Point2::new(half, -half),
        Point2::new(half, half),
        Point2::new(-half, half),
    ];
    let world: Vec<Point3> = corners
        .iter()
        .map(|p| plane.to_world(Point3::new(p.x, p.y, 0.0)))
        .collect();
    let vertices: Vec<VertexId> = world
        .iter()
        .map(|&p| m.raw().add_vertex(Vertex::new(p, tol)))
        .collect();
    let mut coedges = Vec::with_capacity(4);
    for k in 0..4 {
        let (a, b) = (world[k], world[(k + 1) % 4]);
        let d = b - a;
        let curve = m.add_curve(Curve::Line {
            origin: a,
            direction: UnitVec3::new_normalize(d),
        });
        let edge = m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve {
                curve,
                range: Interval::new(0.0, d.norm()).unwrap(),
            },
            vertices[k],
            vertices[(k + 1) % 4],
            tol,
        ));
        let pcurve = m.add_curve2(Curve2::Line {
            origin: corners[k],
            direction: UnitVec2::new_normalize(corners[(k + 1) % 4] - corners[k]),
        });
        coedges.push(Coedge::new(edge, Orientation::Forward, pcurve));
    }
    m.raw()
        .add_face(Face::new(surface, vec![Loop::new(coedges)], tol))
}

/// The ring on its own is clean at `Full` and decides every row: one
/// face, so S5 has no pair to ask about, and one shell, so B1 has one
/// lump.
#[test]
fn the_sample_ring_is_clean_at_full() {
    let mut m = Model::default();
    let body = sample::torus(&mut m, Point3::origin(), R, MINOR).unwrap();
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
}

/// B1 over a traced pair: the ring and a thin pin tilted against its
/// axis, through the hole and clear of the tube. Their boxes overlap, so
/// the pin's wall against the ring goes to the intersector — a cylinder
/// on no axis of the torus, which only ADR-0019's tracer decides — and it
/// finds nothing, so the two shells are two lumps of one solid (ADR-0006)
/// with nothing left unchecked.
#[test]
fn b1_a_tilted_pin_through_the_hole_is_two_lumps() {
    let mut m = Model::default();
    let ring = sample::torus(&mut m, Point3::origin(), R, MINOR).unwrap();
    let ring_shell = m.shells(ring).unwrap()[0].id;
    // Tilted 20°, its axis through the centre: the pin reaches 1.3 from
    // the ring's axis where the ring's nearest point is `R − r` = 3 away.
    let pin = pin_shell(
        &mut m,
        pin_frame(Point3::origin(), 20f64.to_radians(), 12.0),
        0.5,
        12.0,
    );
    let body = solid_of(&mut m, vec![ring_shell, pin]);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
    assert_eq!(
        lumps(&m, body).unwrap(),
        [
            Lump {
                outer: ShellHandle::forward(ring_shell),
                voids: Vec::new(),
            },
            Lump {
                outer: ShellHandle::forward(pin),
                voids: Vec::new(),
            },
        ]
    );
}

/// B1 the other way: the same pin drilled through the tube. The pair the
/// hole's pose left empty now meets, so the two shells overlap and are no
/// lumps — decided, not unchecked.
#[test]
fn b1_a_tilted_pin_through_the_tube_meets_the_ring() {
    let mut m = Model::default();
    let ring = sample::torus(&mut m, Point3::origin(), R, MINOR).unwrap();
    let ring_shell = m.shells(ring).unwrap()[0].id;
    let pin = pin_shell(
        &mut m,
        pin_frame(Point3::new(R, 0.0, 0.0), 20f64.to_radians(), 12.0),
        0.5,
        12.0,
    );
    let body = solid_of(&mut m, vec![ring_shell, pin]);
    let report = check(&m, body, Level::Full);
    assert!(report.unchecked().is_empty(), "{report}");
    let [Violation::ShellNesting { fault, .. }] = report.violations() else {
        panic!("{report}")
    };
    assert_eq!(
        *fault,
        ShellNestingFault::Overlap {
            shells: [ring_shell.min(pin), ring_shell.max(pin)],
        },
        "{report}"
    );
}

/// S5 over a traced and fitted pair: the ring and an oblique plane face
/// that cuts it. Their section is no conic — the plane holds neither the
/// ring's axis nor a normal to it — so it is ADR-0019's trace fitted to
/// NURBS (ADR-0018), and S5 finds it interior to both faces with nothing
/// shared to excuse it. S3 says alongside that two faces sharing no edge
/// are two components.
#[test]
fn s5_an_oblique_plane_that_cuts_the_ring() {
    let mut m = Model::default();
    let ring = sample::torus(&mut m, Point3::origin(), R, MINOR).unwrap();
    let face_ring = m.faces(ring).unwrap()[0].id;
    let face_plane = plane_patch(&mut m, oblique_plane(40f64.to_radians()), 10.0);
    let (body, shell) = sheet(&mut m, vec![face_ring, face_plane]);
    let report = check(&m, body, Level::Full);
    let s = shell.to_string();
    assert_lines(&report, &[("S3", s.clone()), ("S5", s)]);
    assert_eq!(
        report.violations()[1],
        Violation::FacesIntersect {
            shell,
            face_a: face_ring,
            face_b: face_plane,
        }
    );
    assert!(report.unchecked().is_empty(), "{report}");
}

/// The same plane trimmed to the ring's hole: the pair still meets in the
/// same curve, but no point of it is interior to the plane's face, so S5
/// has nothing to report. The boxes overlap, so the tracer is asked and
/// its answer is what decides the row.
#[test]
fn s5_an_oblique_plane_trimmed_to_the_hole_passes() {
    let mut m = Model::default();
    let ring = sample::torus(&mut m, Point3::origin(), R, MINOR).unwrap();
    let face_ring = m.faces(ring).unwrap()[0].id;
    // Every point of the patch is within 2 of the centre, where the
    // ring's nearest point is `R − r` = 3 away.
    let face_plane = plane_patch(&mut m, oblique_plane(40f64.to_radians()), 1.4);
    let (body, shell) = sheet(&mut m, vec![face_ring, face_plane]);
    let report = check(&m, body, Level::Full);
    assert_lines(&report, &[("S3", shell.to_string())]);
    assert!(report.unchecked().is_empty(), "{report}");
}

/// S5 over the elbow of step 3: a bend of pipe and the straight pipe it
/// runs into, tangent along the tube circle they share as an edge. The
/// pair meets in the exact `Curve::Circle` the tracer takes out as a
/// factor, every point of it is within the shared edge's tolerance, and
/// the loop the pose also meets in lies at a `u` the bend does not cover
/// — so S5 passes, with nothing unchecked.
#[test]
fn s5_the_elbow_shares_its_tube_circle_and_passes() {
    let mut m = Model::default();
    let (bend, shared) = torus_band(&mut m, 0.0, FRAC_PI_2, None);
    let pipe = elbow_pipe(&mut m, 0.0, 0.0, 3.0, Some(shared));
    let (body, _) = sheet(&mut m, vec![bend, pipe]);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
}

/// The same tangency with nothing shared: the straight pipe laid across
/// the middle of a bend, touching it along a tube circle interior to both
/// faces. A touch is held to S5's rule as a crossing is
/// (`docs/DATA-MODEL.md` §Invariants), so the circle is a violation —
/// reported, not unchecked.
#[test]
fn s5_a_pipe_tangent_inside_a_bend_is_a_violation() {
    let mut m = Model::default();
    let (bend, _) = torus_band(&mut m, 0.0, FRAC_PI_2, None);
    let pipe = elbow_pipe(&mut m, FRAC_PI_4, -1.5, 1.5, None);
    let (body, shell) = sheet(&mut m, vec![bend, pipe]);
    let report = check(&m, body, Level::Full);
    let s = shell.to_string();
    assert_lines(&report, &[("S3", s.clone()), ("S5", s)]);
    assert_eq!(
        report.violations()[1],
        Violation::FacesIntersect {
            shell,
            face_a: bend,
            face_b: pipe,
        }
    );
    assert!(report.unchecked().is_empty(), "{report}");
}
