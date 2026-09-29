//! `query::project_to_plane` (`docs/ARCHITECTURE.md` §Operations,
//! `docs/DATA-MODEL.md` §Pcurves): every curve kind, in a random pose,
//! projected onto a random plane over a random piece, is the 3D piece
//! projected point for point — the range carried through the projected
//! curve's own parameter, which for an oblique conic is a phase shift a
//! copied range would miss — and the typed refusals, each naming the
//! shape.

use arris_debug::unmetered::primitive_box;
use core::f64::consts::{FRAC_PI_2, FRAC_PI_3, TAU};

use arris_debug::prop::geom::{circle, ellipse, line, nurbs_curve};
use arris_debug::sample;
use arris_debug::unmetered::cut;
use arris_debug::{prop, prop_shards};
use arris_ops::arris_check::arris_topo::arris_geom::region2::Side;
use arris_ops::arris_check::arris_topo::arris_geom::{Curve, Curve2, Surface};
use arris_ops::arris_check::arris_topo::arris_math::{Frame, Interval, Point2, Point3, Vec3};
use arris_ops::arris_check::arris_topo::entity::{Edge, EdgeGeometry, Vertex};
use arris_ops::arris_check::arris_topo::{
    AnyId, EdgeId, Face, Model, Orientation, Shape, VertexId,
};

use arris_ops::arris_check::classify::{Classification, classify_point};
use arris_ops::arris_check::domain::FaceDomain;
use arris_ops::query::{Projection, face_frame, frame_at, project_to_plane};
use arris_ops::{OpError, Reason};
use proptest::prelude::*;

/// How far a projected point may be from the projection of the 3D point,
/// relative to the coordinates involved: the two paths are a handful of
/// products and one `atan2` apart.
const EXACT: f64 = 1e-12;

/// The edge tolerance the hand-built edges carry.
const EDGE_TOLERANCE: f64 = 1e-7;

/// `p` in `plane`'s `(x, y)`.
fn in_plane(plane: &Frame, p: Point3) -> Point2 {
    let q = plane.to_local(p);
    Point2::new(q.x, q.y)
}

/// An edge over `curve` and `range`, with its vertices at the ends.
fn add_edge(m: &mut Model, curve: Curve, range: Interval) -> (EdgeId, VertexId, VertexId) {
    let (a, b) = (curve.point(range.lo()), curve.point(range.hi()));
    let curve = m.add_curve(curve);
    let mut raw = m.raw();
    let start = raw.add_vertex(Vertex::new(a, EDGE_TOLERANCE));
    let end = raw.add_vertex(Vertex::new(b, EDGE_TOLERANCE));
    let edge = raw.add_edge(Edge::new(
        EdgeGeometry::Curve { curve, range },
        start,
        end,
        EDGE_TOLERANCE,
    ));
    (edge, start, end)
}

fn forward(id: impl Into<arris_ops::arris_check::arris_topo::EntityId>) -> Shape {
    Shape::new(id, Orientation::Forward)
}

/// The curve kinds, each equally likely, with a piece of each: a line's
/// anywhere in the default box, a conic's starting anywhere on the turn
/// and up to a whole turn long, a NURBS's anywhere in its knot range.
fn piece() -> impl Strategy<Value = (Curve, Interval)> {
    let conic = |c: BoxedStrategy<Curve>| {
        (c, prop::finite_f64(0.0..=TAU), prop::finite_f64(0.01..=TAU))
            .prop_map(|(c, lo, len)| (c, Interval::new(lo, lo + len).unwrap()))
    };
    prop_oneof![
        (
            line(),
            prop::finite_f64(-prop::DEFAULT_SCALE..=prop::DEFAULT_SCALE),
            prop::finite_f64(0.01..=prop::DEFAULT_SCALE),
        )
            .prop_map(|(c, lo, len)| (c, Interval::new(lo, lo + len).unwrap())),
        conic(circle().boxed()),
        conic(ellipse().boxed()),
        (
            nurbs_curve(),
            prop::finite_f64(0.0..=0.9),
            prop::finite_f64(0.01..=1.0),
        )
            .prop_map(|(c, a, len)| {
                let d = c.domain();
                let (lo, hi) = (d.lerp(a), d.lerp((a + len).min(1.0)));
                (Curve::Nurbs(c), Interval::new(lo, hi).unwrap())
            }),
    ]
}

prop_shards! {
    /// The projected piece at `range.lerp(s)` is the 3D piece at
    /// `edge.range().lerp(s)` projected, at both ends and inside, so it
    /// covers exactly the edge; its length is the edge's (a line's
    /// scaled by the cosine to the plane); the vertices project to its
    /// ends; and the projections come back in the order asked.
    every_curve_kind_projects_over_its_own_piece [shard_0 shard_1 shard_2 shard_3]
        (((curve, range), plane)) = (piece(), prop::frame()) => {
            let mut m = Model::default();
            let (edge, start, end) = add_edge(&mut m, curve.clone(), range);
            let shapes = [forward(start), forward(edge), forward(end)];
            let projected = match project_to_plane(&m, &shapes, &plane) {
                Ok(p) => p,
                Err(OpError::Degenerate { reason: Reason::ProjectionCollapses, entities }) => {
                    // Only a curve seen edge-on collapses, and a random
                    // pose is that only to rounding.
                    prop_assert_eq!(entities, vec![forward(edge)]);
                    return Ok(());
                }
                Err(e) => {
                    prop_assert!(false, "{e}");
                    unreachable!()
                }
            };
            let [
                Projection::Vertex { vertex: v0, point: p0 },
                Projection::Edge { edge: e, curve: c2, range: r2 },
                Projection::Vertex { vertex: v1, point: p1 },
            ] = projected.as_slice()
            else {
                prop_assert!(false, "{projected:?}");
                unreachable!()
            };
            prop_assert_eq!((*v0, *e, *v1), (start, edge, end));
            let scale = |p: Point3| p.coords.norm() + plane.origin().coords.norm() + 1.0;
            for (vertex, at) in [(p0, range.lo()), (p1, range.hi())] {
                let p = curve.point(at);
                prop_assert!((vertex - in_plane(&plane, p)).norm() <= EXACT * scale(p));
            }
            const STEPS: usize = 16;
            for i in 0..=STEPS {
                let s = i as f64 / STEPS as f64;
                let p = curve.point(range.lerp(s));
                let found = c2.point(r2.lerp(s));
                let expected = in_plane(&plane, p);
                prop_assert!(
                    (found - expected).norm() <= EXACT * scale(p),
                    "at s = {s}: {found} vs {expected}, off by {:e}\n{c2:?} over {r2:?}",
                    (found - expected).norm()
                );
            }
            let expected_length = match curve {
                Curve::Line { direction, .. } => {
                    let d = plane.vec_to_local(direction.into_inner());
                    range.length() * d.x.hypot(d.y)
                }
                Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_) => range.length(),
            };
            prop_assert!((r2.length() - expected_length).abs() <= EXACT * range.length().max(1.0));
            if c2.period().is_some() {
                prop_assert!((0.0..TAU).contains(&r2.lo()), "{r2:?}");
            }
            Ok(())
        }
}

/// The case the carried range retires: a quarter of a circle tilted
/// about an axis that is not its `X` projects to an ellipse whose major
/// axis is not the image of the circle's `X`, so the quarter starts at a
/// phase. The copied range draws the wrong arc; the carried one the right.
#[test]
fn an_oblique_arc_is_shifted_by_the_ellipse_phase() {
    let tilted = Frame::new(
        Point3::new(1.0, 2.0, 3.0),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let circle = Curve::Circle {
        frame: tilted,
        radius: 2.0,
    };
    let quarter = Interval::new(FRAC_PI_3, FRAC_PI_3 + FRAC_PI_2).unwrap();
    let mut m = Model::default();
    let (edge, _, _) = add_edge(&mut m, circle.clone(), quarter);
    let plane = Frame::world();
    let p = project_to_plane(&m, &[forward(edge)], &plane).unwrap();
    let [
        Projection::Edge {
            curve: ellipse @ Curve2::Ellipse { .. },
            range,
            ..
        },
    ] = p.as_slice()
    else {
        panic!("{p:?}")
    };
    let start = in_plane(&plane, circle.point(quarter.lo()));
    let end = in_plane(&plane, circle.point(quarter.hi()));
    assert!((ellipse.point(range.lo()) - start).norm() < 1e-14);
    assert!((ellipse.point(range.hi()) - end).norm() < 1e-14);
    assert!(
        (ellipse.point(quarter.lo()) - start).norm() > 0.1,
        "the phase is not trivial here, so the copied range is wrong"
    );
    assert!((range.length() - quarter.length()).abs() < 1e-15);
}

#[test]
fn refusals_name_the_shape() {
    let plane = Frame::world();
    let mut m = Model::default();
    let collapses = |m: &Model, shape: Shape| {
        assert_eq!(
            project_to_plane(m, &[shape], &plane),
            Err(OpError::Degenerate {
                entities: vec![shape],
                reason: Reason::ProjectionCollapses,
            })
        );
    };

    // A line along the plane's normal, a circle seen edge-on.
    let (vertical, _, _) = add_edge(
        &mut m,
        Curve::Line {
            origin: Point3::origin(),
            direction: Vec3::z_axis(),
        },
        Interval::UNIT,
    );
    collapses(&m, forward(vertical));
    let (edge_on, _, _) = add_edge(
        &mut m,
        Curve::Circle {
            frame: Frame::from_z(Point3::origin(), Vec3::x()).unwrap(),
            radius: 1.0,
        },
        Interval::TURN,
    );
    // The handle's orientation is carried into the error as given.
    collapses(&m, Shape::new(edge_on, Orientation::Reversed));

    // A degenerate edge has no curve to project.
    let apex = m
        .raw()
        .add_vertex(Vertex::new(Point3::origin(), EDGE_TOLERANCE));
    let degenerate = m.raw().add_edge(Edge::new(
        EdgeGeometry::Degenerate {
            range: Interval::TURN,
        },
        apex,
        apex,
        EDGE_TOLERANCE,
    ));
    assert_eq!(
        project_to_plane(&m, &[forward(degenerate)], &plane),
        Err(OpError::Degenerate {
            entities: vec![forward(degenerate)],
            reason: Reason::DegenerateEdge,
        })
    );

    // A face and a body are not expanded into their edges.
    let (body, _) = primitive_box(&mut m, Point3::origin(), Point3::new(1.0, 1.0, 1.0)).unwrap();
    let face = m.faces(body).unwrap()[0];
    for shape in [face.shape(), body.shape()] {
        assert_eq!(
            project_to_plane(&m, &[shape], &plane),
            Err(OpError::Degenerate {
                entities: vec![shape],
                reason: Reason::NotProjectable,
            })
        );
    }

    // An id from another model's arena, past the end of this one's.
    let stale = EdgeId::new(10_000, 0);
    assert_eq!(
        project_to_plane(&m, &[forward(stale)], &plane),
        Err(OpError::NotFound(AnyId::from(stale)))
    );

    // The first refusal in the list is the one returned.
    assert!(matches!(
        project_to_plane(
            &m,
            &[forward(apex), forward(vertical), body.shape()],
            &plane
        ),
        Err(OpError::Degenerate {
            reason: Reason::ProjectionCollapses,
            ..
        })
    ));
}

/// The outward normal `face_frame` and `frame_at` must agree on: the
/// surface's own normal there, negated when `use_` is `Reversed`.
fn outward(surface: &Surface, u: f64, v: f64, use_: Orientation) -> Vec3 {
    let n = surface.normal(u, v).unwrap().into_inner();
    if use_ == Orientation::Reversed { -n } else { n }
}

/// `face_frame`'s `Z` is the outward normal of every planar face of the
/// cuboid and of a boolean result — a plate with a through hole, whose
/// cut faces are plane, cylinder and plane again — and its axes are
/// right-handed by construction.
#[test]
fn face_frame_z_is_the_outward_normal() {
    let mut m = Model::default();
    let cuboid = primitive_box(&mut m, Point3::origin(), Point3::new(4.0, 3.0, 2.0))
        .unwrap()
        .0;
    let plate = primitive_box(&mut m, Point3::origin(), Point3::new(4.0, 4.0, 1.0))
        .unwrap()
        .0;
    let punch = primitive_box(
        &mut m,
        Point3::new(1.0, 1.0, -1.0),
        Point3::new(2.0, 2.0, 2.0),
    )
    .unwrap()
    .0;
    let plate = cut(&mut m, plate, punch).unwrap().0;

    for body in [cuboid, plate] {
        for f in m.faces(body).unwrap() {
            let entity = m.face(f.id).unwrap();
            let Surface::Plane { frame: raw } = m.surface(entity.surface()).unwrap() else {
                continue;
            };
            let expected = outward(&Surface::Plane { frame: *raw }, 0.0, 0.0, f.orientation);
            let frame = face_frame(&m, f).unwrap();
            assert!(
                (frame.z().into_inner() - expected).norm() < EXACT,
                "{}",
                f.shape()
            );
            assert!(
                (frame.x().cross(&frame.y()) - frame.z().into_inner()).norm() < EXACT,
                "{}: not right-handed",
                f.shape()
            );
        }
    }
}

/// `face_frame`'s frame agrees with `frame_at`'s at another `(u, v)` of
/// the same planar face, and a point offset along its `Z` by a few
/// multiples of the face's tolerance classifies `Outside`, the opposite
/// offset `Inside` (`classify_point`).
#[test]
fn face_frame_agrees_with_frame_at_and_classifies_either_side() {
    let mut m = Model::default();
    let (body, _) = primitive_box(&mut m, Point3::origin(), Point3::new(4.0, 3.0, 2.0)).unwrap();
    let top: Face = m
        .faces(body)
        .unwrap()
        .into_iter()
        .find(|f| {
            let entity = m.face(f.id).unwrap();
            matches!(
                m.surface(entity.surface()).unwrap(),
                Surface::Plane { frame } if frame.z().z.abs() > 0.5
            )
        })
        .unwrap();
    let frame = face_frame(&m, top).unwrap();

    let tolerance = m.face(top.id).unwrap().tolerance();
    let domain = FaceDomain::of(&m, top.id, tolerance).unwrap();
    let [du, dv] = domain.uv_box().unwrap();
    let uv = Point2::new(du.midpoint(), dv.midpoint());
    assert_eq!(domain.side(uv).0, Side::Inside);
    let at = frame_at(&m, top, uv).unwrap();
    assert!((at.x().into_inner() - frame.x().into_inner()).norm() < EXACT);
    assert!((at.y().into_inner() - frame.y().into_inner()).norm() < EXACT);
    assert!((at.z().into_inner() - frame.z().into_inner()).norm() < EXACT);

    let point = at.origin();
    let step = frame.z().into_inner() * (4.0 * tolerance);
    assert_eq!(
        classify_point(&m, body, point + step).unwrap(),
        Classification::Outside
    );
    assert_eq!(
        classify_point(&m, body, point - step).unwrap(),
        Classification::Inside
    );
}

/// `face_frame` refuses a face whose surface is not a plane.
#[test]
fn face_frame_refuses_a_curved_face() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let wall = m
        .faces(body)
        .unwrap()
        .into_iter()
        .find(|f| {
            matches!(
                m.surface(m.face(f.id).unwrap().surface()).unwrap(),
                Surface::Cylinder { .. }
            )
        })
        .unwrap();
    assert_eq!(
        face_frame(&m, wall),
        Err(OpError::Degenerate {
            entities: vec![wall.shape()],
            reason: Reason::NotPlanar,
        })
    );
}

/// `frame_at`'s `Z` agrees with `Surface::normal` composed with the
/// face's use, over a cylinder, a sphere and a torus.
#[test]
fn frame_at_agrees_with_surface_normal() {
    let mut m = Model::default();
    let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let wall = m
        .faces(cylinder)
        .unwrap()
        .into_iter()
        .find(|f| {
            matches!(
                m.surface(m.face(f.id).unwrap().surface()).unwrap(),
                Surface::Cylinder { .. }
            )
        })
        .unwrap();
    let sphere = sample::sphere(&mut m, Point3::new(1.0, -2.0, 0.5), 3.0).unwrap();
    let sphere_face = m.faces(sphere).unwrap()[0];
    let torus = sample::torus(&mut m, Point3::origin(), 5.0, 2.0).unwrap();
    let torus_face = m.faces(torus).unwrap()[0];

    for (face, u, v) in [
        (wall, 1.0, 6.0),
        (sphere_face, 0.3, 0.2),
        (torus_face, 0.5, 0.5),
    ] {
        let entity = m.face(face.id).unwrap();
        let surface = m.surface(entity.surface()).unwrap();
        let expected = outward(surface, u, v, face.orientation);
        let frame = frame_at(&m, face, Point2::new(u, v)).unwrap();
        assert!(
            (frame.z().into_inner() - expected).norm() < EXACT,
            "{}",
            face.shape()
        );
    }
}

/// `frame_at` is refused at a pole, where the sphere's parametrisation
/// is singular, and off a face's own domain.
#[test]
fn frame_at_refuses_a_pole_and_off_domain() {
    use core::f64::consts::FRAC_PI_2;

    let mut m = Model::default();
    let body = sample::sphere(&mut m, Point3::origin(), 3.0).unwrap();
    let face = m.faces(body).unwrap()[0];

    assert_eq!(
        frame_at(&m, face, Point2::new(0.0, FRAC_PI_2)),
        Err(OpError::Degenerate {
            entities: vec![face.shape()],
            reason: Reason::Singular,
        })
    );
    assert_eq!(
        frame_at(&m, face, Point2::new(0.0, 10.0)),
        Err(OpError::Degenerate {
            entities: vec![face.shape()],
            reason: Reason::OutOfDomain,
        })
    );
}
