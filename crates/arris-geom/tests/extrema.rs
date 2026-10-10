//! Extrema of the distance between two curves and from a point to one:
//! the closed forms and the continua by name; and, for random pairs in
//! random poses, every stationary pair a dense grid's local minima lead
//! Newton to is among the answer, every pair of the answer is stationary
//! and on both curves, and swapping the operands swaps the answer.

use core::f64::consts::{FRAC_PI_2, PI};

use arris_debug::prop::DEFAULT_SCALE;
use arris_debug::prop::geom::{curve, nurbs_curve};
use arris_debug::prop::point_in_box;
use arris_geom::{Curve, Extrema, ExtremumPair, extrema_curve_curve};
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

fn circle(o: [f64; 3], z: [f64; 3], x: [f64; 3], radius: f64) -> Curve {
    Curve::Circle {
        frame: Frame::new(p(o), v(z), v(x)).unwrap(),
        radius,
    }
}

fn ellipse(o: [f64; 3], z: [f64; 3], x: [f64; 3], a: f64, b: f64) -> Curve {
    Curve::Ellipse {
        frame: Frame::new(p(o), v(z), v(x)).unwrap(),
        major_radius: a,
        minor_radius: b,
    }
}

fn line(o: [f64; 3], d: [f64; 3]) -> Curve {
    Curve::Line {
        origin: p(o),
        direction: UnitVec3::new_normalize(v(d)),
    }
}

fn extrema(a: &Curve, b: &Curve) -> Extrema {
    extrema_curve_curve(a, b, tol(), &mut Meter::default())
        .unwrap_or_else(|e| panic!("{a:?} vs {b:?}: {e}"))
}

/// `|a − b|` modulo the period of a periodic curve.
fn param_gap(c: &Curve, a: f64, b: f64) -> f64 {
    match c.period() {
        Some(per) => {
            let d = (a - b).rem_euclid(per);
            d.min(per - d)
        }
        None => (a - b).abs(),
    }
}

/// Every pair lies on both curves at its parameters and is stationary:
/// the offset has no component along either tangent beyond the tolerance.
fn well_formed(a: &Curve, b: &Curve, e: &Extrema) -> Result<(), String> {
    for x in e.isolated.iter().chain(&e.continua) {
        let (ea, eb) = (a.eval(x.ta), b.eval(x.tb));
        let scale = 1e-12 * DEFAULT_SCALE.max(x.a.coords.norm());
        if (ea.point - x.a).norm() > scale || (eb.point - x.b).norm() > scale {
            return Err(format!("{x:?} is off its curves"));
        }
        if ((x.a - x.b).norm() - x.distance).abs() > scale {
            return Err(format!("{x:?}: the distance is not the points'"));
        }
        let r = x.a - x.b;
        for d in [ea.d1, eb.d1] {
            let n = d.norm();
            if n > 0.0 && (r.dot(&d) / n).abs() > tol().linear {
                return Err(format!("{x:?} is not stationary: {}", r.dot(&d) / n));
            }
        }
    }
    Ok(())
}

/// The answer with the operands swapped is the answer swapped: the same
/// pairs, to rounding, and the same continua.
fn swaps(a: &Curve, b: &Curve, e: &Extrema) -> Result<(), String> {
    let f = extrema(b, a);
    if f.isolated.len() != e.isolated.len() || f.continua.len() != e.continua.len() {
        return Err(format!("{e:?}\nswapped\n{f:?}"));
    }
    for x in &e.isolated {
        let found = f
            .isolated
            .iter()
            .any(|y| (x.a - y.b).norm() <= tol().linear && (x.b - y.a).norm() <= tol().linear);
        if !found {
            return Err(format!("{x:?} is not among the swapped {:?}", f.isolated));
        }
    }
    for (x, y) in e.continua.iter().zip(&f.continua) {
        if (x.distance - y.distance).abs() > tol().linear {
            return Err(format!(
                "continua {:?} vs swapped {:?}",
                e.continua, f.continua
            ));
        }
    }
    Ok(())
}

// --- by name --------------------------------------------------------------------

#[test]
fn two_skew_lines_have_one_pair_at_their_common_normal() {
    let (a, b) = (
        line([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        line([3.0, 2.0, 5.0], [0.0, 1.0, 0.0]),
    );
    let e = extrema(&a, &b);
    assert!(e.continua.is_empty());
    let [x] = e.isolated[..] else { panic!("{e:?}") };
    assert_eq!((x.ta, x.tb, x.distance), (3.0, -2.0, 5.0));
}

#[test]
fn parallel_lines_are_a_continuum_at_their_gap() {
    let (a, b) = (
        line([0.0, 0.0, 0.0], [1.0, 1.0, 0.0]),
        line([0.0, 0.0, 4.0], [-1.0, -1.0, 0.0]),
    );
    let e = extrema(&a, &b);
    assert!(e.isolated.is_empty());
    let [x] = e.continua[..] else { panic!("{e:?}") };
    assert_eq!(x.ta, 0.0);
    assert!((x.distance - 4.0).abs() < 1e-15);
}

#[test]
fn a_line_along_a_circles_axis_is_a_continuum() {
    let ring = circle([1.0, 2.0, 3.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 2.0);
    let axis = line([1.0, 2.0, -7.0], [0.0, 0.0, 1.0]);
    let e = extrema(&axis, &ring);
    assert!(e.isolated.is_empty(), "{e:?}");
    assert!((e.continua[0].distance - 2.0).abs() < 1e-15);
    swaps(&axis, &ring, &e).unwrap();
    // Half a tolerance off the axis is still one; ten past it is not, and
    // the line then has its four pairs with the circle.
    let near = line([1.0 + 0.25 * tol().linear, 2.0, -7.0], [0.0, 0.0, 1.0]);
    assert_eq!(extrema(&near, &ring).continua.len(), 1);
    let off = line([1.0 + 10.0 * tol().linear, 2.0, -7.0], [0.0, 0.0, 1.0]);
    let e = extrema(&off, &ring);
    assert!(e.continua.is_empty(), "{e:?}");
    assert_eq!(e.isolated.len(), 2, "{e:?}");
    well_formed(&off, &ring, &e).unwrap();
}

#[test]
fn two_coaxial_circles_are_a_continuum_near_and_one_far() {
    let a = circle([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 2.0);
    let b = circle([0.0, 0.0, 1.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0], 3.0);
    let e = extrema(&a, &b);
    assert!(e.isolated.is_empty());
    let d: Vec<f64> = e.continua.iter().map(|x| x.distance).collect();
    assert!(
        (d[0] - 2f64.sqrt()).abs() < 1e-15 && (d[1] - 26f64.sqrt()).abs() < 1e-14,
        "{d:?}"
    );
    swaps(&a, &b, &e).unwrap();
}

#[test]
fn a_circle_on_the_torus_about_another_is_a_continuum_with_no_common_axis() {
    // The second circle runs round the tube of radius 2 about the first:
    // every one of its points is 2 from it.
    let a = circle([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 2.0);
    let b = circle([2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0], 2.0);
    let e = extrema(&a, &b);
    let near = e.nearest().unwrap();
    assert!((near.distance - 2.0).abs() < 1e-12, "{e:?}");
    assert!(
        e.continua.iter().any(|x| (x.distance - 2.0).abs() < 1e-12),
        "{e:?}"
    );
    well_formed(&a, &b, &e).unwrap();
}

#[test]
fn an_ellipse_and_a_circle_in_a_symmetric_pose_have_their_pairs_at_the_vertices() {
    // The circle stands in the plane y = 0 over the ellipse's centre: the
    // stationary pairs are at the ellipse's four vertices, where both of
    // the squared system's factors vanish at once.
    let a = ellipse([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 3.0, 1.0);
    let b = circle([0.0, 0.0, 2.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0], 1.0);
    let e = extrema(&a, &b);
    assert_eq!(e.isolated.len(), 8, "{e:?}");
    let near = e.nearest().unwrap();
    assert!((near.distance - 2f64.sqrt()).abs() < 1e-14);
    assert!(param_gap(&a, near.ta, FRAC_PI_2) < 1e-12);
    well_formed(&a, &b, &e).unwrap();
    swaps(&a, &b, &e).unwrap();
    // The grid the random pairs are held to finds the two nearest pairs.
    let grid = grid_minima(&a, &b);
    assert!(grid.len() >= 2, "{grid:?}");
    holds_grid(&a, &b, &e).unwrap();
}

#[test]
fn nearly_coaxial_circles_have_four_pairs_and_within_the_tolerance_are_two_continua() {
    let a = circle([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 10.0);
    for tilt in [1e-9, 1e-8, 1e-7, 1e-6, 1e-4, 1e-2] {
        let b = circle([0.0, 0.0, 1.0], [0.0, tilt, 1.0], [1.0, 0.0, 0.0], 7.0);
        let e = extrema(&a, &b);
        well_formed(&a, &b, &e).unwrap();
        // The near distance varies along its family by `14·tilt/√10`, the
        // far by `14·tilt/√290`: within the tolerance up to a tilt of 1e-8.
        if tilt <= 1e-8 {
            assert!(
                e.isolated.is_empty() && e.continua.len() == 2,
                "{tilt}: {e:?}"
            );
        } else {
            assert_eq!(e.isolated.len(), 4, "{tilt}: {e:?}");
            let near = e.nearest().unwrap();
            assert!(param_gap(&a, near.ta, FRAC_PI_2) < 1e-6, "{tilt}: {near:?}");
        }
    }
}

#[test]
fn two_stacked_ellipses_are_a_continuum_and_tilted_have_six_pairs() {
    // One ellipse over another of its shape: every point is 1 above its
    // twin. Tilted off that, the pairs are isolated, and the nearest is at
    // the end of the minor axis, which the tilt about the major axis
    // brings nearer.
    let a = ellipse([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 5.0, 3.0);
    let stacked = ellipse([0.0, 0.0, 1.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 5.0, 3.0);
    let e = extrema(&a, &stacked);
    assert!(
        e.continua.iter().any(|x| (x.distance - 1.0).abs() < 1e-12),
        "{e:?}"
    );
    well_formed(&a, &stacked, &e).unwrap();
    for tilt in [1e-5, 1e-3] {
        let b = ellipse([0.0, 0.0, 1.0], [0.0, tilt, 1.0], [1.0, 0.0, 0.0], 5.0, 3.0);
        let e = extrema(&a, &b);
        assert!(e.continua.is_empty(), "{tilt}: {e:?}");
        well_formed(&a, &b, &e).unwrap();
        holds_grid(&a, &b, &e).unwrap();
        let near = e.nearest().unwrap();
        let drop = 3.0 * (tilt / (1.0 + tilt * tilt).sqrt());
        assert!(
            (near.distance - (1.0 - drop) / (1.0 + tilt * tilt).sqrt().recip().max(1.0)).abs()
                < 1e-6,
            "{tilt}: {near:?}"
        );
    }
}

#[test]
fn a_nurbs_arc_coaxial_with_a_circle_is_a_continuum() {
    let frame = Frame::new(p([1.0, 1.0, 1.0]), v([0.0, 1.0, 0.0]), v([0.0, 0.0, 1.0])).unwrap();
    let arc =
        arris_geom::NurbsCurve::circle(&frame, 3.0, arris_math::Interval::new(0.3, 2.9).unwrap())
            .unwrap();
    let ring = Curve::Circle {
        frame: frame.with_origin(p([1.0, 3.0, 1.0])),
        radius: 1.0,
    };
    let e = extrema(&Curve::Nurbs(arc), &ring);
    let near = e.nearest().unwrap();
    assert!((near.distance - 8f64.sqrt()).abs() < 1e-12, "{e:?}");
    assert!(!e.continua.is_empty(), "{e:?}");
}

#[test]
fn a_far_pair_has_the_answer_of_the_pair_at_the_origin() {
    let shift = Vec3::new(3.0e5, -2.0e5, 7.0e4);
    let a = ellipse([0.0, 0.0, 0.0], [0.2, 0.1, 1.0], [1.0, 0.0, 0.0], 3.0, 1.0);
    let b = circle([1.0, 1.0, 1.0], [0.3, 1.0, 0.2], [1.0, 0.0, 0.0], 2.0);
    let moved = |c: &Curve| c.transformed(&arris_math::Isometry::from_translation(shift));
    let (e, f) = (extrema(&a, &b), extrema(&moved(&a), &moved(&b)));
    assert_eq!(e.isolated.len(), f.isolated.len());
    let (x, y) = (e.nearest().unwrap(), f.nearest().unwrap());
    assert!((x.distance - y.distance).abs() < 1e-9, "{x:?} vs {y:?}");
}

#[test]
fn feet_of_a_point_on_a_curve() {
    let tol = tol();
    let mut meter = Meter::default();
    let ring = circle([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 2.0);
    let f = ring.feet(p([0.0, 3.0, 0.0]), tol, &mut meter).unwrap();
    assert_eq!(f.isolated.len(), 2);
    assert!(
        (f.isolated[0].t - FRAC_PI_2).abs() < 1e-15 && (f.isolated[1].t - 1.5 * PI).abs() < 1e-15
    );
    // An ellipse seen from its centre: the four vertices.
    let e = ellipse([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], 3.0, 1.0);
    let f = e.feet(p([0.0, 0.0, 0.5]), tol, &mut meter).unwrap();
    assert_eq!(f.isolated.len(), 4, "{f:?}");
    for (foot, t) in f.isolated.iter().zip([0.0, FRAC_PI_2, PI, 1.5 * PI]) {
        assert!((foot.t - t).abs() < 1e-12, "{f:?}");
    }
    // A point on a line has one foot, itself.
    let l = line([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
    let f = l.feet(p([1.0, 5.0, 0.0]), tol, &mut meter).unwrap();
    assert_eq!(f.isolated.len(), 1);
    assert_eq!(f.isolated[0].distance, 0.0);
}

// --- at random ------------------------------------------------------------------

/// Where a curve is sampled: a line a stretch either side of its origin
/// that reaches past the default box, a closed curve its whole domain.
fn sampled(c: &Curve) -> (f64, f64) {
    let d = c.domain();
    if d.is_bounded() {
        (d.lo(), d.hi())
    } else {
        (-4.0 * DEFAULT_SCALE, 4.0 * DEFAULT_SCALE)
    }
}

const GRID: usize = 160;

/// Newton on `(A − B)·A′ = (A − B)·B′ = 0` from a grid point, to a
/// stationary pair or nothing.
fn newton(a: &Curve, b: &Curve, mut ta: f64, mut tb: f64) -> Option<(f64, f64)> {
    for _ in 0..60 {
        let (ea, eb) = (a.eval(ta), b.eval(tb));
        let r = ea.point - eb.point;
        let f = [r.dot(&ea.d1), r.dot(&eb.d1)];
        let j = [
            [ea.d1.norm_squared() + r.dot(&ea.d2), -ea.d1.dot(&eb.d1)],
            [ea.d1.dot(&eb.d1), -eb.d1.norm_squared() + r.dot(&eb.d2)],
        ];
        let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
        if det == 0.0 || !det.is_finite() {
            return None;
        }
        ta -= (f[0] * j[1][1] - f[1] * j[0][1]) / det;
        tb -= (j[0][0] * f[1] - j[1][0] * f[0]) / det;
    }
    let (ea, eb) = (a.eval(ta), b.eval(tb));
    let r = ea.point - eb.point;
    let stationary = [ea.d1, eb.d1]
        .iter()
        .all(|d| d.norm() == 0.0 || (r.dot(d) / d.norm()).abs() <= 1e-3 * tol().linear);
    let inside = |c: &Curve, t: f64| c.period().is_some() || c.domain().contains(t);
    (stationary && inside(a, ta) && inside(b, tb)).then_some((ta, tb))
}

/// The stationary pairs Newton reaches from the grid's interior local
/// minima of the distance.
#[allow(clippy::needless_range_loop)] // a grid's neighbours, by index
fn grid_minima(a: &Curve, b: &Curve) -> Vec<(f64, f64)> {
    let ((a0, a1), (b0, b1)) = (sampled(a), sampled(b));
    let ta: Vec<f64> = (0..=GRID)
        .map(|i| a0 + (a1 - a0) * i as f64 / GRID as f64)
        .collect();
    let tb: Vec<f64> = (0..=GRID)
        .map(|i| b0 + (b1 - b0) * i as f64 / GRID as f64)
        .collect();
    let pa: Vec<Point3> = ta.iter().map(|&t| a.point(t)).collect();
    let pb: Vec<Point3> = tb.iter().map(|&t| b.point(t)).collect();
    let d = |i: usize, j: usize| (pa[i] - pb[j]).norm();
    let mut out = Vec::new();
    for i in 1..GRID {
        for j in 1..GRID {
            let here = d(i, j);
            let lowest = (-1i64..=1).all(|di| {
                (-1i64..=1).all(|dj| here <= d((i as i64 + di) as usize, (j as i64 + dj) as usize))
            });
            if lowest {
                out.extend(newton(a, b, ta[i], tb[j]));
            }
        }
    }
    out
}

/// Every stationary pair the grid leads to is an isolated pair of the
/// answer, or on one of its continua at that distance.
fn holds_grid(a: &Curve, b: &Curve, e: &Extrema) -> Result<(), String> {
    for (ta, tb) in grid_minima(a, b) {
        let (pa, pb) = (a.point(ta), b.point(tb));
        let d = (pa - pb).norm();
        let isolated = e.isolated.iter().any(|x: &ExtremumPair| {
            (x.a - pa).norm() <= 1e3 * tol().linear && (x.b - pb).norm() <= 1e3 * tol().linear
        });
        let on_continuum = e
            .continua
            .iter()
            .any(|x| (x.distance - d).abs() <= tol().linear);
        if !(isolated || on_continuum) {
            return Err(format!(
                "the grid's stationary pair ta = {ta}, tb = {tb} at {d} is not in {e:?}"
            ));
        }
    }
    Ok(())
}

fn any_curve() -> impl Strategy<Value = Curve> {
    prop_oneof![3 => curve(), 1 => nurbs_curve().prop_map(Curve::Nurbs)]
}

arris_debug::prop_shards! {
    /// Random pairs of lines, circles, ellipses and NURBS curves.
    random_pairs [s0 s1 s2 s3] ((a, b)) = (any_curve(), any_curve()) => {
        let e = extrema(&a, &b);
        well_formed(&a, &b, &e).map_err(TestCaseError::fail)?;
        holds_grid(&a, &b, &e).map_err(TestCaseError::fail)?;
        swaps(&a, &b, &e).map_err(TestCaseError::fail)?;
        Ok(())
    }
}

/// A curve and its twin moved along its plane's normal by up to its own
/// size and tilted about an axis in its plane by up to a thousandth of a
/// radian: the near-continua the isolation can give up on.
fn near_twin() -> impl Strategy<Value = (Curve, Curve)> {
    use arris_debug::prop::{finite_f64, frame};
    (
        frame(),
        0usize..3,
        finite_f64(1.0..=20.0),
        finite_f64(0.2..=1.0),
        finite_f64(0.0..=1.0),
        finite_f64(-1e-3..=1e-3),
        finite_f64(0.0..=core::f64::consts::TAU),
    )
        .prop_map(|(f, kind, r, ratio, lift, tilt, axis)| {
            let make = |frame: Frame| match kind {
                0 => Curve::Circle { frame, radius: r },
                1 => Curve::Ellipse {
                    frame,
                    major_radius: r,
                    minor_radius: r * ratio,
                },
                _ => Curve::Nurbs(
                    arris_geom::NurbsCurve::ellipse(
                        &frame,
                        r,
                        r * ratio,
                        arris_math::Interval::new(0.3, 5.0).unwrap(),
                    )
                    .unwrap(),
                ),
            };
            let (x, y, z) = (f.x().into_inner(), f.y().into_inner(), f.z().into_inner());
            let about = axis.cos() * x + axis.sin() * y;
            let turned = arris_math::nalgebra::UnitQuaternion::from_axis_angle(
                &UnitVec3::new_normalize(about),
                tilt,
            );
            let twin = Frame::new(f.origin() + lift * r * z, turned * z, turned * x).unwrap();
            (make(f), make(twin))
        })
}

arris_debug::prop_shards! {
    /// Near-continua: every pair stationary, the grid's pairs among them,
    /// and the answer swapping with its operands.
    near_twins [n0 n1] ((a, b)) = near_twin() => {
        let e = extrema(&a, &b);
        well_formed(&a, &b, &e).map_err(TestCaseError::fail)?;
        holds_grid(&a, &b, &e).map_err(TestCaseError::fail)?;
        swaps(&a, &b, &e).map_err(TestCaseError::fail)?;
        Ok(())
    }
}

arris_debug::prop_shards! {
    /// Random points against random curves: every foot is stationary and
    /// the nearest is no farther than a dense sample's nearest.
    random_feet [f0 f1] ((c, q)) = (any_curve(), point_in_box(DEFAULT_SCALE)) => {
        let feet = c.feet(q, tol(), &mut Meter::default()).unwrap();
        for f in feet.isolated.iter().chain(&feet.continua) {
            let e = c.eval(f.t);
            let n = e.d1.norm();
            prop_assert!((e.point - f.point).norm() <= 1e-9);
            prop_assert!(n == 0.0 || ((e.point - q).dot(&e.d1) / n).abs() <= tol().linear, "{f:?}");
        }
        let nearest = feet.isolated.iter().chain(&feet.continua).map(|f| f.distance).fold(f64::INFINITY, f64::min);
        let (t0, t1) = sampled(&c);
        let sample = (0..=4000).map(|i| (c.point(t0 + (t1 - t0) * i as f64 / 4000.0) - q).norm()).fold(f64::INFINITY, f64::min);
        // The nearest point of a NURBS curve may be an end, which is no
        // foot; anything else is stationary.
        let interior = !matches!(c, Curve::Nurbs(_));
        if interior {
            prop_assert!(nearest <= sample + tol().linear, "{nearest} vs the sample's {sample}: {feet:?}");
        }
        Ok(())
    }
}
