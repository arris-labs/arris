//! `intersect_surfaces` over a NURBS surface and an analytic one, in both
//! orders (ADR-0061): the exact twin of a cylinder met by each of the six
//! kinds, every fitted point on both surfaces within the tolerance and the
//! argument order not reaching the result, a NURBS twin of a pole met at
//! the pole as a point, and two NURBS surfaces still `Unsupported`.
//! `tests/oracle.rs` holds the same section to Open CASCADE's.

use core::f64::consts::FRAC_PI_2;

use arris_geom::{Curve, GeomError, MeetKind, Surface, SurfaceIntersection, intersect_surfaces};
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

#[test]
fn two_nurbs_surfaces_are_still_unsupported() {
    let a = wall();
    assert!(matches!(meet(&a, &a), Err(GeomError::Unsupported { .. })));
}
