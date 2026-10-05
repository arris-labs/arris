//! `FaceDomain` on periodic surfaces (`docs/ARCHITECTURE.md` §The
//! checker): the translate of the domain a face's loops are written in
//! changes nothing about the side a point is on.

use core::f64::consts::TAU;

use arris_check::domain::{FaceDomain, band, shifts};
use arris_debug::prop::finite_f64;
use arris_geom::region2::{Side, point_side};
use arris_geom::{Curve, Curve2, Surface};
use arris_math::{Frame, Interval, Point2, Point3, Vec2, Vec3};
use arris_topo::entity::{Coedge, Edge, EdgeGeometry, Face, Loop, Vertex};
use arris_topo::{FaceId, Model, NotFound, Orientation};
use proptest::prelude::*;

const MAJOR: f64 = 5.0;
const MINOR: f64 = 2.0;

/// The patch `[u0, u0 + du] × [v0, v0 + dv]` of a cylinder of radius
/// `MINOR` about the z axis, or of the torus of radii `MAJOR` and `MINOR`
/// about it, as a face bounded by its four iso-curves: two circles and
/// two rulings on the cylinder, four circles on the torus. Each edge runs
/// over its range in the first translate and each pcurve adds `shift`, so
/// the loop is written in the translate `shift` names.
fn patch(m: &mut Model, torus: bool, [u0, du, v0, dv]: [f64; 4], shift: Vec2) -> FaceId {
    let tol = m.precision().default_tolerance;
    let frame = Frame::world();
    let surface = if torus {
        Surface::Torus {
            frame,
            major_radius: MAJOR,
            minor_radius: MINOR,
        }
    } else {
        Surface::Cylinder {
            frame,
            radius: MINOR,
        }
    };
    let (u1, v1) = (u0 + du, v0 + dv);
    let corners = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
        .map(|(u, v)| m.raw().add_vertex(Vertex::new(surface.point(u, v), tol)));
    // The iso-curve along `u` at `v`: a circle whose parameter is `u`.
    let along_u = |v: f64| {
        let (rho, z) = if torus {
            (MAJOR + MINOR * v.cos(), MINOR * v.sin())
        } else {
            (MINOR, v)
        };
        Curve::Circle {
            frame: Frame::world().with_origin(Point3::new(0.0, 0.0, z)),
            radius: rho,
        }
    };
    // The iso-curve along `v` at `u`, whose parameter is `v`: a ruling on
    // the cylinder, the circle of the tube's section on the torus.
    let along_v = |u: f64| {
        let radial = Vec3::new(u.cos(), u.sin(), 0.0);
        if torus {
            Curve::Circle {
                frame: Frame::new(
                    Point3::origin() + MAJOR * radial,
                    radial.cross(&Vec3::z()),
                    radial,
                )
                .unwrap(),
                radius: MINOR,
            }
        } else {
            Curve::Line {
                origin: Point3::origin() + MINOR * radial,
                direction: Vec3::z_axis(),
            }
        }
    };
    let edge = |m: &mut Model, curve: Curve, lo: f64, hi: f64, from, to| {
        let curve = m.add_curve(curve);
        let range = Interval::new(lo, hi).unwrap();
        m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve { curve, range },
            from,
            to,
            tol,
        ))
    };
    let bottom = edge(m, along_u(v0), u0, u1, corners[0], corners[1]);
    let right = edge(m, along_v(u1), v0, v1, corners[1], corners[2]);
    let top = edge(m, along_u(v1), u0, u1, corners[3], corners[2]);
    let left = edge(m, along_v(u0), v0, v1, corners[0], corners[3]);
    let line = |m: &mut Model, origin: Point2, u_wise: bool| {
        m.add_curve2(Curve2::Line {
            origin: origin + shift,
            direction: if u_wise {
                Vec2::x_axis()
            } else {
                Vec2::y_axis()
            },
        })
    };
    let coedges = vec![
        Coedge::new(
            bottom,
            Orientation::Forward,
            line(m, Point2::new(0.0, v0), true),
        ),
        Coedge::new(
            right,
            Orientation::Forward,
            line(m, Point2::new(u1, 0.0), false),
        ),
        Coedge::new(
            top,
            Orientation::Reversed,
            line(m, Point2::new(0.0, v1), true),
        ),
        Coedge::new(
            left,
            Orientation::Reversed,
            line(m, Point2::new(u0, 0.0), false),
        ),
    ];
    let surface = m.add_surface(surface);
    m.raw()
        .add_face(Face::new(surface, vec![Loop::new(coedges)], tol))
}

arris_debug::prop_shards! {
    /// A point inside a patch written in the first translate is inside the
    /// same patch written a period along in `u`, in `v` on the torus, or
    /// both; `side` returns the shift that takes it into the loops'
    /// translate, and the loops wind around it either way.
    a_face_a_period_along_answers_alike [shard_0 shard_1 shard_2 shard_3]
        ((torus, u0, du, v0, dv, s, t, ku, kv)) = (
            any::<bool>(),
            finite_f64(0.0..=TAU),
            finite_f64(0.1..=3.0),
            finite_f64(-3.0..=3.0),
            finite_f64(0.1..=3.0),
            finite_f64(0.05..=0.95),
            finite_f64(0.05..=0.95),
            -1i32..=1,
            -1i32..=1,
        ) => {
            let tolerance = Model::default().precision().parametric_tolerance;
            let rect = [u0, du, v0, dv];
            let uv = Point2::new(u0 + s * du, v0 + t * dv);
            // A cylinder has no period in `v`.
            let along = Vec2::new(
                f64::from(ku) * TAU,
                if torus { f64::from(kv) * TAU } else { 0.0 },
            );
            let fail = |e: NotFound| TestCaseError::fail(e.to_string());
            let mut first = Model::default();
            let face = patch(&mut first, torus, rect, Vec2::zeros());
            let a = FaceDomain::of(&first, face, tolerance).map_err(fail)?;
            let mut moved = Model::default();
            let moved_face = patch(&mut moved, torus, rect, along);
            let b = FaceDomain::of(&moved, moved_face, tolerance).map_err(fail)?;
            prop_assert_eq!(a.side(uv), (Side::Inside, Vec2::zeros()), "{:?}", rect);
            prop_assert_eq!(b.side(uv), (Side::Inside, along), "{:?} a period along {:?}", rect, along);
            prop_assert!(a.winds_around(uv) && b.winds_around(uv), "{:?}", rect);
            Ok(())
        }
}

/// What `FaceDomain::side` answered before it read an index: the walk
/// over every segment in every translate, `point_side` at the band.
fn walk(d: &FaceDomain, uv: Point2) -> (Side, Vec2) {
    let near = band(d.surface(), uv, d.tolerance());
    let periods = d.surface().period();
    let mut best = (Side::Outside, Vec2::zeros());
    for du in shifts(periods[0]) {
        for dv in shifts(periods[1]) {
            let shift = Vec2::new(du, dv);
            match point_side(d.polygons(), uv + shift, near) {
                Side::Inside => return (Side::Inside, shift),
                Side::Boundary => {
                    if best.0 == Side::Outside {
                        best = (Side::Boundary, shift);
                    }
                }
                Side::Outside => {}
            }
        }
    }
    best
}

arris_debug::prop_shards! {
    /// `FaceDomain::side`, which reads an index, answers as the walk over
    /// every segment: a cylinder or torus patch written in any translate,
    /// at points across and around its (u, v) box and on its loops'
    /// vertices (docs/ARCHITECTURE.md §The checker).
    the_indexed_side_is_the_walk [shard_0 shard_1 shard_2 shard_3]
        ((torus, u0, du, v0, dv, k, spots)) = (
            any::<bool>(),
            finite_f64(0.0..=TAU),
            finite_f64(0.1..=3.0),
            finite_f64(-3.0..=3.0),
            finite_f64(0.1..=3.0),
            -1i32..=1,
            proptest::collection::vec((finite_f64(-0.2..=1.2), finite_f64(-0.2..=1.2)), 16),
        ) => {
            let tolerance = Model::default().precision().parametric_tolerance;
            let mut m = Model::default();
            let along = Vec2::new(f64::from(k) * TAU, 0.0);
            let face = patch(&mut m, torus, [u0, du, v0, dv], along);
            let d = FaceDomain::of(&m, face, tolerance)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            let mut points: Vec<Point2> = spots
                .iter()
                .map(|&(s, t)| Point2::new(u0 + s * du, v0 + t * dv))
                .collect();
            points.extend(
                d.polygons()
                    .iter()
                    .flat_map(|p| p.points().iter().copied())
                    .step_by(7),
            );
            for uv in points {
                prop_assert_eq!(d.side(uv), walk(&d, uv), "{:?}", uv);
            }
            Ok(())
        }
}
