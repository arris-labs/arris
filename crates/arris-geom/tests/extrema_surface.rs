//! Extrema of the distance between a curve and a surface, and the feet of
//! a point on a surface: the closed forms and the continua by name; and,
//! for random curves and surfaces in random poses, every pair stationary
//! and on both operands, every local minimum a dense sample of the curve
//! leads to among the answer, and the nearest foot of a point the
//! projection's.

use core::f64::consts::{FRAC_PI_2, PI};

use arris_debug::prop::geom::{curve, nurbs_curve, surface};
use arris_debug::prop::{DEFAULT_SCALE, point_in_box};
use arris_geom::{
    Curve, CurveSurfaceExtrema, GeomError, NurbsSurface, Surface, SurfaceKind,
    extrema_curve_surface,
};
use arris_math::{Frame, Meter, Point3, Precision, Tolerance, UnitVec3, Vec3};
use proptest::prelude::*;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

fn p(x: [f64; 3]) -> Point3 {
    Point3::new(x[0], x[1], x[2])
}

fn v(x: [f64; 3]) -> Vec3 {
    Vec3::new(x[0], x[1], x[2])
}

fn frame(o: [f64; 3], z: [f64; 3], x: [f64; 3]) -> Frame {
    Frame::new(p(o), v(z), v(x)).unwrap()
}

fn line(o: [f64; 3], d: [f64; 3]) -> Curve {
    Curve::Line {
        origin: p(o),
        direction: UnitVec3::new_normalize(v(d)),
    }
}

fn circle(o: [f64; 3], z: [f64; 3], radius: f64) -> Curve {
    Curve::Circle {
        frame: frame(o, z, [1.0, 0.0, 0.0]),
        radius,
    }
}

fn extrema(c: &Curve, s: &Surface) -> CurveSurfaceExtrema {
    extrema_curve_surface(c, s, tol(), &mut Meter::default())
        .unwrap_or_else(|e| panic!("{c:?} vs {s:?}: {e}"))
}

fn distances(e: &CurveSurfaceExtrema) -> Vec<f64> {
    let mut d: Vec<f64> = e.isolated.iter().map(|x| x.distance).collect();
    d.sort_by(f64::total_cmp);
    d
}

fn continua(e: &CurveSurfaceExtrema) -> Vec<f64> {
    e.continua.iter().map(|x| x.distance).collect()
}

/// Every pair lies on both operands at its parameters and is stationary:
/// the offset has no component along the curve's tangent beyond the
/// tolerance and is along the surface's normal.
fn well_formed(c: &Curve, s: &Surface, e: &CurveSurfaceExtrema) -> Result<(), String> {
    for x in e.isolated.iter().chain(&e.continua) {
        let ec = c.eval(x.t);
        let scale = 1e-12 * DEFAULT_SCALE.max(x.curve_point.coords.norm());
        if (ec.point - x.curve_point).norm() > scale
            || (s.point(x.uv.x, x.uv.y) - x.surface_point).norm() > scale
        {
            return Err(format!("{x:?} is off its operands"));
        }
        if ((x.curve_point - x.surface_point).norm() - x.distance).abs() > scale {
            return Err(format!("{x:?}: the distance is not the points'"));
        }
        let r = x.curve_point - x.surface_point;
        let n = ec.d1.norm();
        if n > 0.0 && (r.dot(&ec.d1) / n).abs() > tol().linear {
            return Err(format!("{x:?} is not stationary on the curve"));
        }
        if let Some(normal) = s.normal(x.uv.x, x.uv.y) {
            if x.distance > tol().linear && r.cross(&normal).norm() > tol().linear {
                return Err(format!("{x:?} is not along the surface's normal"));
            }
        }
    }
    Ok(())
}

// --- by name --------------------------------------------------------------------

#[test]
fn a_tilted_circle_over_a_plane_has_its_height_extrema() {
    // The circle's plane makes 30° with the plane z = 0 and its centre is
    // 5 above: heights 5 ± 2 sin 30°, no crossing.
    let plane = Surface::Plane {
        frame: Frame::world(),
    };
    let ring = Curve::Circle {
        frame: frame(
            [1.0, 2.0, 5.0],
            [0.0, 0.5, 3f64.sqrt() / 2.0],
            [1.0, 0.0, 0.0],
        ),
        radius: 2.0,
    };
    let e = extrema(&ring, &plane);
    assert!(e.continua.is_empty(), "{e:?}");
    let d = distances(&e);
    assert_eq!(d.len(), 2, "{e:?}");
    assert!(
        (d[0] - 4.0).abs() < 1e-12 && (d[1] - 6.0).abs() < 1e-12,
        "{d:?}"
    );
    well_formed(&ring, &plane, &e).unwrap();
    // Lowered to cross, it also meets the plane twice, at distance 0.
    let low = Curve::Circle {
        frame: frame(
            [1.0, 2.0, 0.5],
            [0.0, 0.5, 3f64.sqrt() / 2.0],
            [1.0, 0.0, 0.0],
        ),
        radius: 2.0,
    };
    let e = extrema(&low, &plane);
    let d = distances(&e);
    assert_eq!(d.len(), 4, "{e:?}");
    assert!(d[0] < 1e-12 && d[1] < 1e-12, "{d:?}");
    well_formed(&low, &plane, &e).unwrap();
}

#[test]
fn a_circle_parallel_to_a_plane_and_a_line_parallel_to_it_are_continua() {
    let plane = Surface::Plane {
        frame: Frame::world(),
    };
    let ring = circle([0.0, 0.0, 3.0], [0.0, 0.0, 1.0], 1.0);
    let e = extrema(&ring, &plane);
    assert!(e.isolated.is_empty(), "{e:?}");
    assert!((continua(&e)[0] - 3.0).abs() < 1e-12);
    let rod = line([0.0, 0.0, -2.0], [1.0, 1.0, 0.0]);
    let e = extrema(&rod, &plane);
    assert!(
        e.isolated.is_empty() && (continua(&e)[0] - 2.0).abs() < 1e-12,
        "{e:?}"
    );
    // A slanted line meets it once.
    let skew = line([0.0, 0.0, -2.0], [1.0, 0.0, 1.0]);
    let e = extrema(&skew, &plane);
    assert!(e.continua.is_empty());
    assert_eq!(e.isolated.len(), 1);
    assert!(e.isolated[0].distance < 1e-12 && (e.isolated[0].t - 2f64.sqrt() * 2.0).abs() < 1e-12);
    // A circle lying in the plane is a continuum at 0.
    let flat = circle([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 1.0);
    let e = extrema(&flat, &plane);
    assert!(continua(&e).iter().all(|d| d.abs() < 1e-12), "{e:?}");
}

#[test]
fn a_line_past_a_sphere_has_its_near_and_far_points() {
    let ball = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let rod = line([5.0, 3.0, -4.0], [0.0, 0.0, 1.0]);
    let e = extrema(&rod, &ball);
    let d = distances(&e);
    // The foot of the centre on the rod is 5.83 away: 5.83 − 2 and + 2.
    let rho = 34f64.sqrt();
    assert_eq!(d.len(), 2, "{e:?}");
    assert!((d[0] - (rho - 2.0)).abs() < 1e-12 && (d[1] - (rho + 2.0)).abs() < 1e-12);
    well_formed(&rod, &ball, &e).unwrap();
    // Through it, the rod meets the sphere twice.
    let through = line([1.0, 0.0, -4.0], [0.0, 0.0, 1.0]);
    let e = extrema(&through, &ball);
    assert!(
        distances(&e).iter().filter(|d| **d < 1e-12).count() == 2,
        "{e:?}"
    );
    well_formed(&through, &ball, &e).unwrap();
}

#[test]
fn a_circle_round_a_sphere_s_axis_is_a_continuum_near_and_one_far() {
    let ball = Surface::Sphere {
        frame: Frame::world(),
        radius: 1.0,
    };
    let ring = circle([0.0, 0.0, 3.0], [0.0, 0.0, 1.0], 4.0);
    let e = extrema(&ring, &ball);
    assert!(e.isolated.is_empty(), "{e:?}");
    let c = continua(&e);
    assert!(
        (c[0] - 4.0).abs() < 1e-12 && (c[1] - 6.0).abs() < 1e-12,
        "{c:?}"
    );
    well_formed(&ring, &ball, &e).unwrap();
}

#[test]
fn a_line_beside_a_cylinder_is_a_continuum_near_and_one_far() {
    let pipe = Surface::Cylinder {
        frame: Frame::world(),
        radius: 2.0,
    };
    let rod = line([5.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
    let e = extrema(&rod, &pipe);
    assert!(e.isolated.is_empty(), "{e:?}");
    let c = continua(&e);
    assert!(
        (c[0] - 3.0).abs() < 1e-12 && (c[1] - 7.0).abs() < 1e-12,
        "{c:?}"
    );
    well_formed(&rod, &pipe, &e).unwrap();
    // A skew line has the common normal's pairs.
    let skew = line([5.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    let e = extrema(&skew, &pipe);
    let d = distances(&e);
    assert_eq!(d.len(), 2, "{e:?}");
    assert!(
        (d[0] - 3.0).abs() < 1e-12 && (d[1] - 7.0).abs() < 1e-12,
        "{d:?}"
    );
    well_formed(&skew, &pipe, &e).unwrap();
}

#[test]
fn a_ring_on_a_cylinder_is_a_continuum_at_zero_and_a_ring_about_it_is_not() {
    let pipe = Surface::Cylinder {
        frame: Frame::world(),
        radius: 2.0,
    };
    let on = circle([0.0, 0.0, 1.0], [0.0, 0.0, 1.0], 2.0);
    let e = extrema(&on, &pipe);
    assert!(continua(&e).iter().any(|d| d.abs() < 1e-12), "{e:?}");
    // A circle through the axis, slanted: crossing the surface twice and
    // no stationary pair on the axis it passes through.
    let slant = circle([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], 3.0);
    let e = extrema(&slant, &pipe);
    well_formed(&slant, &pipe, &e).unwrap();
}

#[test]
fn a_line_along_a_cylinder_s_axis_is_a_continuum_at_its_radius() {
    let pipe = Surface::Cylinder {
        frame: Frame::world(),
        radius: 2.0,
    };
    let axis = line([0.0, 0.0, -3.0], [0.0, 0.0, 1.0]);
    let e = extrema(&axis, &pipe);
    assert!(e.isolated.is_empty(), "{e:?}");
    assert!((continua(&e)[0] - 2.0).abs() < 1e-12);
    // Half a tolerance off it still is; ten tolerances off, it is not.
    let near = line([0.25 * tol().linear, 0.0, -3.0], [0.0, 0.0, 1.0]);
    assert_eq!(extrema(&near, &pipe).continua.len(), 1);
    let off = line([10.0 * tol().linear, 0.0, -3.0], [0.0, 0.0, 1.0]);
    let e = extrema(&off, &pipe);
    assert_eq!(e.continua.len(), 2, "{e:?}");
}

#[test]
fn a_circle_against_a_torus_lifts_the_centre_circle_s_pairs() {
    let ring = Surface::Torus {
        frame: Frame::world(),
        major_radius: 5.0,
        minor_radius: 1.0,
    };
    // Coaxial and above: every point of the circle is as far from the tube.
    let above = circle([0.0, 0.0, 3.0], [0.0, 0.0, 1.0], 5.0);
    let e = extrema(&above, &ring);
    assert!(e.isolated.is_empty(), "{e:?}");
    let c = continua(&e);
    assert!(
        (c[0] - 2.0).abs() < 1e-12 && (c[1] - 4.0).abs() < 1e-12,
        "{c:?}"
    );
    well_formed(&above, &ring, &e).unwrap();
    // The centre circle itself: a continuum at the tube's radius.
    let centre = circle([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], 5.0);
    let e = extrema(&centre, &ring);
    assert!(
        continua(&e).iter().any(|d| (d - 1.0).abs() < 1e-12),
        "{e:?}"
    );
    // A line through the hole along the axis: the tube is 4 from it at
    // the nearest and 6 at the farthest (a family).
    let axis = line([0.0, 0.0, -2.0], [0.0, 0.0, 1.0]);
    let e = extrema(&axis, &ring);
    let c = continua(&e);
    assert!(
        (c[0] - 4.0).abs() < 1e-12 && (c[1] - 6.0).abs() < 1e-12,
        "{c:?}"
    );
}

#[test]
fn a_line_against_a_cone_meets_it_and_a_coaxial_circle_is_a_continuum() {
    let half = 0.5f64;
    let cone = Surface::Cone {
        frame: Frame::world(),
        radius: 1.0,
        half_angle: half,
    };
    // Parallel to the axis: one crossing with each nappe, no other
    // stationary pair.
    let rod = line([3.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
    let e = extrema(&rod, &cone);
    assert_eq!(e.isolated.len(), 2, "{e:?}");
    assert!(e.isolated.iter().all(|x| x.distance < 1e-12));
    well_formed(&rod, &cone, &e).unwrap();
    // A coaxial circle: the distance to each line in the meridian plane.
    let ring = circle([0.0, 0.0, 2.0], [0.0, 0.0, 1.0], 4.0);
    let e = extrema(&ring, &cone);
    assert!(e.isolated.is_empty(), "{e:?}");
    // At height 2 the cone's radius is 1 + 2 tan 0.5 = 2.0926.
    let (sa, ca) = half.sin_cos();
    let apex = -ca / sa;
    let s_plus = 4.0 * ca - (2.0 - apex) * sa;
    let s_minus = -4.0 * ca - (2.0 - apex) * sa;
    let mut want = [s_plus.abs(), s_minus.abs()];
    want.sort_by(f64::total_cmp);
    let c = continua(&e);
    assert_eq!(c.len(), 2, "{e:?}");
    assert!(
        (c[0] - want[0]).abs() < 1e-12 && (c[1] - want[1]).abs() < 1e-12,
        "{c:?} vs {want:?}"
    );
    well_formed(&ring, &cone, &e).unwrap();
}

#[test]
fn a_circle_against_a_cone_in_an_oblique_pose_is_stationary() {
    let cone = Surface::Cone {
        frame: frame([0.5, -1.0, 0.0], [0.2, 0.1, 1.0], [1.0, 0.0, 0.0]),
        radius: 1.0,
        half_angle: 0.7,
    };
    let ring = Curve::Circle {
        frame: frame([2.0, 1.0, 3.0], [0.3, 1.0, 0.2], [1.0, 0.0, 0.0]),
        radius: 2.5,
    };
    let e = extrema(&ring, &cone);
    assert!(!e.isolated.is_empty(), "{e:?}");
    well_formed(&ring, &cone, &e).unwrap();
    minima_held(&ring, &cone, &e).unwrap();
}

#[test]
fn an_ellipse_against_an_elliptic_cylinder_is_the_flat_pair_s() {
    let tube = Surface::EllipticCylinder {
        frame: Frame::world(),
        major_radius: 3.0,
        minor_radius: 1.0,
    };
    // A line along the axis, off it: the ellipse's feet of the point,
    // each a family along the line.
    let rod = line([5.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
    let e = extrema(&rod, &tube);
    assert!(e.isolated.is_empty(), "{e:?}");
    let c = continua(&e);
    assert_eq!(c.len(), 2, "{e:?}");
    assert!(
        (c[0] - 2.0).abs() < 1e-12 && (c[1] - 8.0).abs() < 1e-12,
        "{c:?}"
    );
    well_formed(&rod, &tube, &e).unwrap();
    // A circle about the axis, tilted: stationary pairs against a dense
    // sample's.
    let ring = Curve::Circle {
        frame: frame([0.5, 0.2, 0.0], [0.1, 0.2, 1.0], [1.0, 0.0, 0.0]),
        radius: 6.0,
    };
    let e = extrema(&ring, &tube);
    well_formed(&ring, &tube, &e).unwrap();
    minima_held(&ring, &tube, &e).unwrap();
}

#[test]
fn a_nurbs_curve_against_a_plane_and_an_elliptic_cylinder() {
    let wave = arris_geom::NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0],
        vec![
            p([0.0, 0.0, 1.0]),
            p([1.0, 2.0, 3.0]),
            p([2.0, -1.0, -2.0]),
            p([3.0, 1.0, 4.0]),
            p([4.0, 0.0, 1.0]),
        ],
        vec![1.0, 1.2, 0.9, 1.0, 1.1],
    )
    .unwrap();
    let wave = Curve::Nurbs(wave);
    let plane = Surface::Plane {
        frame: frame([0.0, 0.0, 0.5], [0.1, 0.2, 1.0], [1.0, 0.0, 0.0]),
    };
    let e = extrema(&wave, &plane);
    assert!(!e.isolated.is_empty(), "{e:?}");
    well_formed(&wave, &plane, &e).unwrap();
    let tube = Surface::EllipticCylinder {
        frame: frame([2.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
        major_radius: 3.0,
        minor_radius: 1.5,
    };
    let e = extrema(&wave, &tube);
    assert!(!e.isolated.is_empty(), "{e:?}");
    well_formed(&wave, &tube, &e).unwrap();
}

#[test]
fn a_curve_against_a_nurbs_surface_is_unsupported() {
    let patch = NurbsSurface::new(
        [1, 1],
        [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
        vec![
            p([0.0, 0.0, 0.0]),
            p([0.0, 1.0, 0.0]),
            p([1.0, 0.0, 0.0]),
            p([1.0, 1.0, 0.0]),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    let e = extrema_curve_surface(
        &line([0.0, 0.0, 1.0], [0.0, 0.0, 1.0]),
        &Surface::Nurbs(patch),
        tol(),
        &mut Meter::default(),
    );
    assert!(
        matches!(&e, Err(GeomError::Unsupported { b, .. }) if *b == SurfaceKind::Nurbs.into()),
        "{e:?}"
    );
}

// --- the feet of a point --------------------------------------------------------

#[test]
fn feet_of_a_point_on_each_surface() {
    let mut meter = Meter::default();
    let t = tol();
    let plane = Surface::Plane {
        frame: Frame::world(),
    };
    let f = plane.feet(p([1.0, 2.0, 3.0]), t, &mut meter).unwrap();
    assert_eq!(f.isolated.len(), 1);
    assert_eq!(f.isolated[0].distance, 3.0);
    let ball = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let f = ball.feet(p([0.0, 0.0, 5.0]), t, &mut meter).unwrap();
    assert_eq!(f.isolated.len(), 2);
    assert!(
        (f.isolated[0].distance - 3.0).abs() < 1e-15
            || (f.isolated[1].distance - 3.0).abs() < 1e-15
    );
    let f = ball.feet(p([0.0, 0.0, 0.0]), t, &mut meter).unwrap();
    assert!(f.isolated.is_empty() && (f.continua[0].distance - 2.0).abs() < 1e-15);
    let ring = Surface::Torus {
        frame: Frame::world(),
        major_radius: 5.0,
        minor_radius: 1.0,
    };
    let f = ring.feet(p([7.0, 0.0, 0.0]), t, &mut meter).unwrap();
    let mut d: Vec<f64> = f.isolated.iter().map(|x| x.distance).collect();
    d.sort_by(f64::total_cmp);
    // Near and far side of the tube at the near foot, and of the far one's.
    assert_eq!(d.len(), 4);
    let want = [1.0, 3.0, 11.0, 13.0];
    assert!(
        d.iter().zip(want).all(|(a, b)| (a - b).abs() < 1e-12),
        "{d:?}"
    );
    // On the centre circle: the tube's meridian circle, all at r.
    let f = ring.feet(p([5.0, 0.0, 0.0]), t, &mut meter).unwrap();
    assert!(
        f.continua.iter().any(|x| (x.distance - 1.0).abs() < 1e-12),
        "{f:?}"
    );
    let cone = Surface::Cone {
        frame: Frame::world(),
        radius: 1.0,
        half_angle: FRAC_PI_2 / 2.0,
    };
    let f = cone.feet(p([3.0, 0.0, 0.0]), t, &mut meter).unwrap();
    assert_eq!(f.isolated.len(), 2);
    let near = cone.project(p([3.0, 0.0, 0.0])).unwrap();
    let best = f
        .isolated
        .iter()
        .map(|x| x.distance)
        .fold(f64::INFINITY, f64::min);
    assert!((near.distance - best).abs() < 1e-12);
    let _ = PI;
}

// --- at random ------------------------------------------------------------------

/// Where a curve is sampled.
fn sampled(c: &Curve) -> (f64, f64) {
    let d = c.domain();
    if d.is_bounded() {
        (d.lo(), d.hi())
    } else {
        (-4.0 * DEFAULT_SCALE, 4.0 * DEFAULT_SCALE)
    }
}

const SAMPLES: usize = 2000;

/// The distance from the curve's point at `t` to the surface (the
/// closed-form projection's), `None` where it is ambiguous.
fn gap(c: &Curve, s: &Surface, t: f64) -> Option<f64> {
    s.project(c.point(t)).ok().map(|x| x.distance)
}

/// Every local minimum of the distance a dense sample of the curve finds,
/// the surface eliminated by its projection, is a pair of the answer or on
/// a continuum at its distance.
fn minima_held(c: &Curve, s: &Surface, e: &CurveSurfaceExtrema) -> Result<(), String> {
    let (t0, t1) = sampled(c);
    let periodic = c.period().is_some();
    let ts: Vec<f64> = (0..=SAMPLES)
        .map(|i| t0 + (t1 - t0) * i as f64 / SAMPLES as f64)
        .collect();
    let g: Vec<Option<f64>> = ts.iter().map(|&t| gap(c, s, t)).collect();
    let last = SAMPLES;
    for i in 0..=last {
        let (prev, next) = if periodic {
            (g[(i + last - 1) % last], g[(i + 1) % last])
        } else if i == 0 || i == last {
            continue;
        } else {
            (g[i - 1], g[i + 1])
        };
        let (Some(here), Some(a), Some(b)) = (g[i], prev, next) else {
            continue;
        };
        // A minimum at a knot where the curve is only C⁰ is a kink of the
        // distance, no stationary pair: the topology's (a knot there is
        // where an edge would be split).
        if let Curve::Nurbs(n) = c {
            let step = (t1 - t0) / SAMPLES as f64;
            if n.knots().iter().any(|k| (k - ts[i]).abs() <= 2.0 * step) {
                continue;
            }
        }
        if !(here <= a && here <= b) {
            continue;
        }
        // Golden section around the sample.
        let step = (t1 - t0) / SAMPLES as f64;
        let (mut lo, mut hi) = (ts[i] - step, ts[i] + step);
        let phi = (5f64.sqrt() - 1.0) / 2.0;
        let f = |t: f64| gap(c, s, t).unwrap_or(f64::INFINITY);
        for _ in 0..80 {
            let (x1, x2) = (hi - phi * (hi - lo), lo + phi * (hi - lo));
            if f(x1) < f(x2) {
                hi = x2;
            } else {
                lo = x1;
            }
        }
        let t = 0.5 * (lo + hi);
        let d = f(t);
        let at = c.point(t);
        let isolated = e.isolated.iter().any(|x| {
            (x.curve_point - at).norm() <= 1e-4 * DEFAULT_SCALE.max(at.coords.norm())
                || (x.distance - d).abs() <= tol().linear && x.distance <= d
        });
        let on_continuum = e
            .continua
            .iter()
            .any(|x| (x.distance - d).abs() <= tol().linear);
        if !(isolated || on_continuum) {
            return Err(format!(
                "the sample's local minimum at t = {t} ({d}) is not in {e:?}"
            ));
        }
    }
    Ok(())
}

arris_debug::prop_shards! {
    /// Random curves against random analytic surfaces.
    random_pairs [s0 s1 s2 s3] ((c, s)) = (curve(), surface()) => {
        let e = extrema(&c, &s);
        well_formed(&c, &s, &e).map_err(TestCaseError::fail)?;
        minima_held(&c, &s, &e).map_err(TestCaseError::fail)?;
        Ok(())
    }
}

arris_debug::prop_shards! {
    /// Random NURBS curves against random analytic surfaces: an open
    /// curve's sampled local minima are interior and so stationary.
    random_nurbs_pairs [n0 n1 n2 n3] ((c, s)) = (nurbs_curve().prop_map(Curve::Nurbs), surface()) => {
        let e = extrema(&c, &s);
        well_formed(&c, &s, &e).map_err(TestCaseError::fail)?;
        minima_held(&c, &s, &e).map_err(TestCaseError::fail)?;
        Ok(())
    }
}

arris_debug::prop_shards! {
    /// Random points on random analytic surfaces: every foot is on the
    /// surface and along its normal, and the nearest is the projection's.
    random_feet [f0 f1] ((s, q)) = (surface(), point_in_box(DEFAULT_SCALE)) => {
        let feet = s.feet(q, tol(), &mut Meter::default()).unwrap();
        for f in feet.isolated.iter().chain(&feet.continua) {
            prop_assert!((s.point(f.uv.x, f.uv.y) - f.point).norm() <= 1e-9, "{f:?}");
            if let Some(n) = s.normal(f.uv.x, f.uv.y) {
                let r = q - f.point;
                prop_assert!(r.cross(&n).norm() <= tol().linear, "{f:?}");
            }
        }
        if let Ok(nearest) = s.project(q) {
            let best = feet.isolated.iter().chain(&feet.continua).map(|f| f.distance).fold(f64::INFINITY, f64::min);
            prop_assert!(best <= nearest.distance + tol().linear, "{best} vs {}", nearest.distance);
        }
        Ok(())
    }
}
