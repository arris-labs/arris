//! `intersect_surfaces` over a NURBS surface and an analytic one, in both
//! orders (ADR-0061): the exact twin of a cylinder met by each of the six
//! kinds, every fitted point on both surfaces within the tolerance and the
//! argument order not reaching the result, a NURBS twin of a pole met at
//! the pole as a point, a meridian met beside its pole, a saddle a hair
//! past the domain's edge, and two NURBS surfaces still `Unsupported`.
//! `tests/oracle.rs` holds the same section to Open CASCADE's.

use core::f64::consts::{FRAC_PI_2, PI};

use arris_geom::{
    Curve, GeomError, MeetKind, NurbsSurface, Surface, SurfaceIntersection, intersect_surfaces,
};
use arris_math::{Aabb, Frame, Interval, Meter, Point3, Precision, Tolerance, Vec3};

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

fn frame(origin: [f64; 3], z: [f64; 3]) -> Frame {
    Frame::from_z(Point3::from(origin), Vec3::from(z)).unwrap()
}

fn within() -> Aabb {
    Aabb {
        min: [-10.0; 3],
        max: [10.0; 3],
    }
}

fn meet(a: &Surface, b: &Surface) -> Result<SurfaceIntersection, GeomError> {
    intersect_surfaces(a, b, &within(), tol(), &mut Meter::default())
}

fn wall() -> Surface {
    let pipe = Surface::Cylinder {
        frame: Frame::world(),
        radius: 1.0,
    };
    let nurbs = pipe
        .to_nurbs([Interval::TURN, Interval::new(-1.0, 1.0).unwrap()])
        .unwrap();
    Surface::Nurbs(nurbs)
}

fn distance(surface: &Surface, p: Point3) -> f64 {
    surface.project(p).map_or(f64::INFINITY, |on| on.distance)
}

#[test]
fn a_cylinder_twin_meets_each_analytic_kind_in_both_orders() {
    let wall = wall();
    let sideways = frame([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    let others = [
        (
            "plane",
            Surface::Plane {
                frame: frame([0.0, 0.0, 0.2], [0.1, 0.2, 1.0]),
            },
        ),
        (
            "cylinder",
            Surface::Cylinder {
                frame: sideways,
                radius: 0.5,
            },
        ),
        (
            "elliptic cylinder",
            Surface::EllipticCylinder {
                frame: sideways,
                major_radius: 0.6,
                minor_radius: 0.4,
            },
        ),
        (
            "cone",
            Surface::Cone {
                frame: sideways,
                radius: 0.3,
                half_angle: 0.17,
            },
        ),
        (
            "sphere",
            Surface::Sphere {
                frame: frame([0.9, 0.0, 0.0], [0.0, 0.0, 1.0]),
                radius: 0.5,
            },
        ),
        (
            "torus",
            Surface::Torus {
                frame: sideways,
                major_radius: 0.8,
                minor_radius: 0.5,
            },
        ),
    ];
    for (name, other) in &others {
        let hit = meet(&wall, other).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(!hit.curves().is_empty(), "{name}: {hit:?}");
        for m in hit.curves() {
            let Curve::Nurbs(c) = &m.curve else {
                panic!("{name}: {m:?}")
            };
            let d = c.domain();
            for k in 0..=40 {
                let p = c.eval(d.lerp(k as f64 / 40.0)).point;
                assert!(distance(&wall, p) <= tol().linear, "{name}: off the wall");
                assert!(distance(other, p) <= tol().linear, "{name}: off the other");
            }
        }
        let back = meet(other, &wall).unwrap_or_else(|e| panic!("{name} swapped: {e}"));
        assert_eq!(hit, back, "{name}: the order reaches the result");
    }
}

#[test]
fn a_hemisphere_twin_meets_the_plane_tangent_at_its_pole_in_the_pole() {
    let globe = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let half = [Interval::TURN, Interval::new(0.0, FRAC_PI_2).unwrap()];
    let dome = Surface::Nurbs(globe.to_nurbs(half).unwrap());
    let cap = Surface::Plane {
        frame: frame([0.0, 0.0, 2.0], [0.0, 0.0, 1.0]),
    };
    for (a, b) in [(&dome, &cap), (&cap, &dome)] {
        let hit = meet(a, b).unwrap();
        assert!(hit.curves().is_empty(), "{hit:?}");
        let [point] = hit.points() else {
            panic!("{hit:?}")
        };
        assert_eq!(point.kind, MeetKind::Touch);
        assert!((point.point - Point3::new(0.0, 0.0, 2.0)).norm() <= tol().linear);
    }
}

/// The half dome's twin against the plane through its pole and a
/// meridian: the section is the meridian, an isoline of `u`, and the pole,
/// a collapsed row of `v` the plane crosses, so isolines run both ways.
/// The pole is divided out beside the meridian and comes back as a point
/// (ADR-0061 §3, amended at nurbs-face-blend step 9b; it was
/// refused as tube circles both ways), as a walked blend's pole meets the
/// faces it narrows between.
#[test]
fn a_half_dome_twin_meets_the_plane_through_its_pole_in_a_meridian_and_the_pole() {
    let globe = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let half = [
        Interval::new(0.0, PI).unwrap(),
        Interval::new(0.0, FRAC_PI_2).unwrap(),
    ];
    let dome = Surface::Nurbs(globe.to_nurbs(half).unwrap());
    let upright = Surface::Plane {
        frame: frame([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
    };
    for (a, b) in [(&dome, &upright), (&upright, &dome)] {
        let hit = meet(a, b).unwrap();
        let [meridian] = hit.curves() else {
            panic!("{hit:?}")
        };
        assert_eq!(meridian.kind, MeetKind::Crossing);
        let d = meridian.curve.domain();
        for k in 0..=40 {
            let p = meridian.curve.eval(d.lerp(k as f64 / 40.0)).point;
            assert!(p.x.abs() <= tol().linear, "off the plane: {p:?}");
            assert!(
                (p.coords.norm() - 2.0).abs() <= tol().linear,
                "off the dome: {p:?}"
            );
        }
        for point in hit.points() {
            assert!((point.point - Point3::new(0.0, 0.0, 2.0)).norm() <= tol().linear);
        }
    }
}

/// `z = (u − v)(u + 2v) + c` over `u ∈ [ε, 1]`, `v ∈ [−0.8, 1.3]`, a
/// quadratic Bézier patch: its saddle at the origin is `ε` past the edge
/// `u = ε`, and the plane `z = 0` misses it by `c`, so the section's near
/// miss turns inside the domain beside it, and leaves it across `u = 1`
/// clear of the corners. Within the tolerance the edge does not
/// tell the saddle from one on it, and the section is traced with the
/// saddle put on the edge (ADR-0061 §3, amended at nurbs-face-blend
/// step 9b, where a walked blend's contact edge met the face across a
/// junction so: its turning points were refused as not told apart).
#[test]
fn a_saddle_a_hair_past_the_domains_edge_is_the_sections_singular_point_on_it() {
    let (epsilon, c) = (1e-8, -5e-10);
    let (u, v) = ([epsilon, 1.0], [-0.8, 1.3]);
    // The polar forms of `u`, `v` and `z`, at the ends each control point
    // takes of `[u₀, u₁]` and `[v₀, v₁]`.
    let ends = |x: [f64; 2], k: usize| [[x[0], x[0]], [x[0], x[1]], [x[1], x[1]]][k];
    let mut points = Vec::with_capacity(9);
    for i in 0..3 {
        for j in 0..3 {
            let ([u1, u2], [v1, v2]) = (ends(u, i), ends(v, j));
            let (um, vm) = (0.5 * (u1 + u2), 0.5 * (v1 + v2));
            points.push(Point3::new(um, vm, u1 * u2 + um * vm - 2.0 * v1 * v2 + c));
        }
    }
    let knots = |x: [f64; 2]| vec![x[0], x[0], x[0], x[1], x[1], x[1]];
    let patch = NurbsSurface::new([2, 2], [knots(u), knots(v)], points, vec![1.0; 9]).unwrap();
    let saddle = Surface::Nurbs(patch);
    let flat = Surface::Plane {
        frame: Frame::world(),
    };
    for (a, b) in [(&saddle, &flat), (&flat, &saddle)] {
        let hit = meet(a, b).unwrap();
        assert!(!hit.curves().is_empty(), "{hit:?}");
        let mut nearest = f64::INFINITY;
        for m in hit.curves() {
            let d = m.curve.domain();
            for k in 0..=400 {
                let p = m.curve.eval(d.lerp(k as f64 / 400.0)).point;
                assert!(p.z.abs() <= tol().linear, "off the plane: {p:?}");
                assert!(distance(&saddle, p) <= tol().linear, "off the patch: {p:?}");
                assert!(p.x >= epsilon - tol().linear, "past the edge: {p:?}");
                nearest = nearest.min(p.coords.xy().norm());
            }
        }
        // The arm round `+u` turns `√(8|c|/9)` from the saddle, which is
        // the section's singular point on the edge, within the tolerance
        // of both surfaces there.
        assert!(nearest <= 1e-3, "{nearest}");
        let [on_edge] = hit.points() else {
            panic!("{hit:?}")
        };
        assert!((on_edge.point - Point3::new(epsilon, 0.0, 0.0)).norm() <= tol().linear);
    }
}

/// `z = 4v² − u²` over `[0, 1]²`, a quadratic Bézier patch, against the
/// plane `z = 0`: the section's saddle is the patch's corner at the origin,
/// on an edge of `u` and of `v` at once, and of its four arms the one
/// inside the quadrant, `v = u/2`, runs out across `u = 1`. The saddle's
/// cell is cut at both edges, in either order of the surfaces (plans/
/// nurbs-face-blend step 10c, where a walked blend's patch met cylinders
/// at its corners).
#[test]
fn a_saddle_at_a_corner_of_the_domain_keeps_the_one_arm_inside_it() {
    let (u, v) = ([0.0, 1.0], [0.0, 1.0]);
    let ends = |x: [f64; 2], k: usize| [[x[0], x[0]], [x[0], x[1]], [x[1], x[1]]][k];
    let mut points = Vec::with_capacity(9);
    for i in 0..3 {
        for j in 0..3 {
            let ([u1, u2], [v1, v2]) = (ends(u, i), ends(v, j));
            let (um, vm) = (0.5 * (u1 + u2), 0.5 * (v1 + v2));
            points.push(Point3::new(um, vm, 4.0 * v1 * v2 - u1 * u2));
        }
    }
    let knots = |x: [f64; 2]| vec![x[0], x[0], x[0], x[1], x[1], x[1]];
    let patch = NurbsSurface::new([2, 2], [knots(u), knots(v)], points, vec![1.0; 9]).unwrap();
    let saddle = Surface::Nurbs(patch);
    let flat = Surface::Plane {
        frame: Frame::world(),
    };
    for (a, b) in [(&saddle, &flat), (&flat, &saddle)] {
        let hit = meet(a, b).unwrap();
        let [arm] = hit.curves() else {
            panic!("{hit:?}")
        };
        let d = arm.curve.domain();
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for k in 0..=200 {
            let p = arm.curve.eval(d.lerp(k as f64 / 200.0)).point;
            assert!(p.z.abs() <= tol().linear, "off the plane: {p:?}");
            assert!(
                (p.y - 0.5 * p.x).abs() <= tol().linear,
                "off the arm: {p:?}"
            );
            lo = f64::min(lo, p.x);
            hi = f64::max(hi, p.x);
        }
        assert!(lo <= tol().linear && hi >= 1.0 - tol().linear, "{lo} {hi}");
    }
}

#[test]
fn a_nurbs_surface_against_itself_is_flush_and_unsupported() {
    let a = wall();
    assert!(matches!(meet(&a, &a), Err(GeomError::Unsupported { .. })));
}
