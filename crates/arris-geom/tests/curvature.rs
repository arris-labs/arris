//! `Surface::normal_curvature` against the closed forms of every
//! analytic kind and of a hand-built NURBS saddle, its refusals, and the
//! fact the boolean's curvature rule leans on for two cylinders: a
//! tangent pair never has equal curvatures across its ruling
//! (docs/ARCHITECTURE.md §Operations, the curvature rule).

use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use arris_geom::{Curve, MeetKind, NurbsSurface, Surface, SurfaceIntersection, intersect_surfaces};
use arris_math::{Frame, Point3, Precision, Tolerance, Vec3};

/// Closed form against the second fundamental form, at unit scale.
const EXACT: f64 = 1e-12;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

/// A region every traced section of these tests lies in; the closed
/// forms ignore it.
fn within() -> arris_math::Aabb {
    arris_math::Aabb {
        min: [-100.0; 3],
        max: [100.0; 3],
    }
}

/// A frame off every axis, so no closed form is helped by the world's.
fn posed() -> Frame {
    Frame::from_z(Point3::new(1.0, -2.0, 0.5), Vec3::new(1.0, 2.0, 3.0)).unwrap()
}

fn close(found: Option<f64>, wanted: f64) {
    let found = found.expect("a curvature");
    assert!((found - wanted).abs() <= EXACT, "{found} vs {wanted}");
}

#[test]
fn a_plane_is_flat_in_every_direction() {
    let plane = Surface::Plane { frame: posed() };
    let e = plane.eval(0.3, -0.7);
    for w in [e.du, e.dv, e.du + 2.0 * e.dv, e.du - e.dv] {
        close(plane.normal_curvature(0.3, -0.7, w, tol()), 0.0);
    }
}

/// `−1/R` across the rulings, `0` along them, and Euler's `−cos²θ/R` at
/// an angle `θ` from the circumferential direction between.
#[test]
fn a_cylinder_bends_away_from_its_normal_across_its_rulings() {
    let r = 2.5;
    let wall = Surface::Cylinder {
        frame: posed(),
        radius: r,
    };
    for u in [0.0, 1.0, PI, 5.5] {
        let e = wall.eval(u, 0.4);
        let (across, along) = (e.du.normalize(), e.dv.normalize());
        close(wall.normal_curvature(u, 0.4, across, tol()), -1.0 / r);
        close(wall.normal_curvature(u, 0.4, along, tol()), 0.0);
        for theta in [FRAC_PI_4, 1.2] {
            let w = theta.cos() * across + theta.sin() * along;
            close(
                wall.normal_curvature(u, 0.4, 3.0 * w, tol()),
                -theta.cos().powi(2) / r,
            );
        }
        // A bore is the same surface: the sign follows the normal, not
        // the face's orientation.
        close(wall.normal_curvature(u, 0.4, -across, tol()), -1.0 / r);
    }
}

#[test]
fn a_sphere_is_umbilic() {
    let r = 1.5;
    let ball = Surface::Sphere {
        frame: posed(),
        radius: r,
    };
    for (u, v) in [(0.0, 0.0), (2.0, 0.7), (4.0, -1.1)] {
        let e = ball.eval(u, v);
        for w in [e.du, e.dv, e.du + e.dv, 2.0 * e.du - 0.5 * e.dv] {
            close(ball.normal_curvature(u, v, w, tol()), -1.0 / r);
        }
    }
    // The pole has no normal, so no curvature either.
    let e = ball.eval(0.0, FRAC_PI_2);
    assert_eq!(
        ball.normal_curvature(0.0, FRAC_PI_2, e.du + e.dv, tol()),
        None
    );
}

/// Along a ruling `0`; along the parallel circle of radius `ρ`, whose
/// principal normal makes the angle `α` with the surface normal,
/// `−cos α / ρ`.
#[test]
fn a_cone_is_flat_along_its_rulings() {
    let (r, alpha) = (2.0, 0.5f64);
    let cone = Surface::Cone {
        frame: posed(),
        radius: r,
        half_angle: alpha,
    };
    for (u, v) in [(0.0, 0.0), (1.5, 1.0), (3.0, -2.0)] {
        let e = cone.eval(u, v);
        let rho = r + v * alpha.sin();
        close(cone.normal_curvature(u, v, e.dv, tol()), 0.0);
        close(cone.normal_curvature(u, v, e.du, tol()), -alpha.cos() / rho);
    }
}

/// Along the tube `−1/r`; along the parallel circle at `v`,
/// `−cos v / (R + r cos v)` — positive on the inner equator.
#[test]
fn a_torus_is_elliptic_outside_and_hyperbolic_inside() {
    let (big, small) = (3.0, 1.0);
    let torus = Surface::Torus {
        frame: posed(),
        major_radius: big,
        minor_radius: small,
    };
    for (u, v) in [(0.0, 0.0), (1.0, 1.0), (2.0, PI), (4.0, 2.5)] {
        let e = torus.eval(u, v);
        close(torus.normal_curvature(u, v, e.dv, tol()), -1.0 / small);
        close(
            torus.normal_curvature(u, v, e.du, tol()),
            -v.cos() / (big + small * v.cos()),
        );
    }
}

/// The bilinear NURBS `P(u, v) = (u, v, u v)` over `[0, 1]²`: the graph
/// of `z = x y`, whose normal is `(−v, −u, 1)` normalised, `L = N = 0`
/// and `M = 1/√(1 + u² + v²)`. Flat along both parameter lines; along
/// `∂P/∂u ± ∂P/∂v` the form is `±2M` over `E ± 2F + G`.
#[test]
fn the_saddle_bends_both_ways() {
    let saddle = Surface::Nurbs(
        NurbsSurface::new(
            [1, 1],
            [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 1.0),
            ],
            vec![1.0; 4],
        )
        .unwrap(),
    );
    for (u, v) in [(0.5, 0.5), (0.2, 0.7), (0.9, 0.1)] {
        let e = saddle.eval(u, v);
        assert!((e.point - Point3::new(u, v, u * v)).norm() <= EXACT);
        let (ee, ff, gg) = (1.0 + v * v, u * v, 1.0 + u * u);
        let m = 1.0 / (1.0 + u * u + v * v).sqrt();
        close(saddle.normal_curvature(u, v, e.du, tol()), 0.0);
        close(saddle.normal_curvature(u, v, e.dv, tol()), 0.0);
        close(
            saddle.normal_curvature(u, v, e.du + e.dv, tol()),
            2.0 * m / (ee + 2.0 * ff + gg),
        );
        close(
            saddle.normal_curvature(u, v, e.du - e.dv, tol()),
            -2.0 * m / (ee - 2.0 * ff + gg),
        );
    }
}

#[test]
fn a_direction_off_the_tangent_plane_or_of_no_length_has_no_curvature() {
    let wall = Surface::Cylinder {
        frame: Frame::world(),
        radius: 1.0,
    };
    let e = wall.eval(0.0, 0.0);
    let n = wall.normal(0.0, 0.0).unwrap().into_inner();
    assert_eq!(wall.normal_curvature(0.0, 0.0, n, tol()), None);
    assert_eq!(
        wall.normal_curvature(0.0, 0.0, e.du + 1e-6 * n, tol()),
        None
    );
    assert_eq!(wall.normal_curvature(0.0, 0.0, Vec3::zeros(), tol()), None);
    assert_eq!(
        wall.normal_curvature(0.0, 0.0, Vec3::new(f64::NAN, 0.0, 0.0), tol()),
        None
    );
}

/// The curvature rule's tie — equal curvatures across the contact — is
/// never reached by two cylinders. Across a tangent ruling, signed
/// against one normal, an outside touch is `−1/R₁` against `+1/R₂` and an
/// inside touch `−1/R₁` against `−1/R₂` with `R₁ ≠ R₂`; equal radii
/// touching inside are the same axis, which is `Coincident`, not a
/// touch.
#[test]
fn a_tangent_cylinder_pair_never_ties_its_curvatures() {
    let cylinder = |x: f64, radius: f64| Surface::Cylinder {
        frame: Frame::from_z(Point3::new(x, 0.0, 0.0), Vec3::z()).unwrap(),
        radius,
    };
    for (a, b) in [
        (cylinder(0.0, 1.0), cylinder(2.0, 1.0)),
        (cylinder(0.0, 1.0), cylinder(3.0, 2.0)),
        (cylinder(0.0, 2.0), cylinder(1.0, 1.0)),
    ] {
        let r = intersect_surfaces(&a, &b, &within(), tol(), &mut arris_math::Meter::default())
            .unwrap();
        let [meet] = r.curves() else {
            panic!("one ruling: {r:?}");
        };
        assert_eq!(meet.kind, MeetKind::Touch, "a tangent pair");
        let Curve::Line { origin, direction } = &meet.curve else {
            panic!("one ruling: {r:?}");
        };
        let on_a = a.project(*origin).unwrap().uv;
        let on_b = b.project(*origin).unwrap().uv;
        let n = a.normal(on_a.x, on_a.y).unwrap().into_inner();
        let across = n.cross(&direction.into_inner());
        let ka = a.normal_curvature(on_a.x, on_a.y, across, tol()).unwrap();
        let nb = b.normal(on_b.x, on_b.y).unwrap().into_inner();
        let kb = b.normal_curvature(on_b.x, on_b.y, across, tol()).unwrap() * nb.dot(&n).signum();
        assert_ne!(ka, kb, "{a:?} and {b:?}");
    }
    assert_eq!(
        intersect_surfaces(
            &cylinder(0.0, 1.0),
            &cylinder(0.0, 1.0),
            &within(),
            tol(),
            &mut arris_math::Meter::default()
        )
        .unwrap(),
        SurfaceIntersection::Coincident
    );
}
