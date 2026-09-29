//! A `Curve::Nurbs` against a line, a circle and an ellipse (ADR-0018):
//! an ellipse as a NURBS — exact and rational, or fitted and periodic —
//! meets each where the ellipse itself does, hit for hit, through the
//! other conic's plane or, in the ellipse's own plane, through its
//! cylinder; a line through a point of a random NURBS, or a circle
//! through one, hits it there; a NURBS on a line or a conic is
//! `Coincident`, and `curves_coincide` says so; two NURBS curves are the
//! same spline, apart, or `Unsupported`.

use core::f64::consts::{FRAC_1_SQRT_2, TAU};

use arris_debug::prop::geom::{circle, ellipse, nurbs_curve};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, frame, radius, unit_vec3};
use arris_geom::{
    Curve, CurveCurveHit, CurveIntersection, CurveSurfaceIntersection, GeomError, GeomKind,
    NurbsCurve, Surface, curves_coincide, fit_curve_periodic, intersect_curve_surface,
    intersect_curves,
};
use arris_math::{Frame, Interval, Isometry, Point3, Precision, Tolerance, Vec3};
use proptest::prelude::*;

/// Hit points against both operands and against the exact arms.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;
/// What a fitted ellipse is held to against the ellipse.
const FIT: f64 = 1e-10;
/// How far a crossing of the fitted ellipse may be from the ellipse's
/// own: [`FIT`] over the sine of the crossing angle, which [`STEEP`]
/// keeps above 1e-3.
const FITTED: f64 = 1e-6;
/// The smallest sine of the angle a built crossing is allowed to make
/// with the plane it is found through.
const STEEP: f64 = 1e-3;
/// Where two touches are compared: an extremum's parameter is a root of
/// the distance's derivative, found to about the square root of the
/// rounding by one arm and to rounding by another.
const TOUCH: f64 = 1e-6;

/// The angle of the circle the rational ellipse's seam is at.
const SEAM: f64 = 0.3;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

fn fail<T>(msg: String) -> Result<T, TestCaseError> {
    Err(TestCaseError::fail(msg))
}

fn hits_of(r: &CurveIntersection) -> &[CurveCurveHit] {
    match r {
        CurveIntersection::Points(h) => h,
        CurveIntersection::Coincident => &[],
    }
}

/// The ellipse of `frame` reaching `a` along `X` and `b` along `Y` as
/// four rational quadratic arcs: exact, clamped, closed, its parameter
/// not the ellipse's angle. The arcs are a circle's quarters turned by
/// [`SEAM`] and stretched, so the seam — two ends, each a hit where it
/// lies on the other curve (`intersect_curve_surface`) — is off the
/// vertices the tests touch at.
fn rational_ellipse(frame: &Frame, a: f64, b: f64) -> NurbsCurve {
    let (sin, cos) = SEAM.sin_cos();
    let at = |x: f64, y: f64| {
        frame.to_world(Point3::new(
            a * (x * cos - y * sin),
            b * (x * sin + y * cos),
            0.0,
        ))
    };
    let w = FRAC_1_SQRT_2;
    NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0],
        vec![
            at(1.0, 0.0),
            at(1.0, 1.0),
            at(0.0, 1.0),
            at(-1.0, 1.0),
            at(-1.0, 0.0),
            at(-1.0, -1.0),
            at(0.0, -1.0),
            at(1.0, -1.0),
            at(1.0, 0.0),
        ],
        vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0],
    )
    .unwrap()
}

/// A conic's frame and its two radii.
fn conic_parts(c: &Curve) -> (Frame, f64, f64) {
    match *c {
        Curve::Circle { frame, radius } => (frame, radius, radius),
        Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => (frame, major_radius, minor_radius),
        Curve::Line { .. } | Curve::Nurbs(_) => unreachable!("the strategies draw conics"),
    }
}

/// A conic as a periodic quintic fitted at the conic's own angle.
fn fitted(conic: &Curve) -> NurbsCurve {
    fit_curve_periodic(
        |t| conic.point(t),
        Interval::new(0.0, TAU).unwrap(),
        5,
        |t, q| (q - conic.point(t)).norm(),
        FIT,
        &mut arris_math::Meter::default(),
    )
    .unwrap()
}

/// What every result owes whatever its case: each parameter in its
/// curve's domain — a periodic NURBS's below its end, a conic's in `[0,
/// 2π)` — each hit's point the first curve's and on the second within
/// the tolerance, sorted by `ta`, and a second run the same bit for bit.
/// A swapped pair keeps the point it computed, which is the second
/// curve's (`intersect_curves`), so the first curve is held to the
/// tolerance there instead.
fn common_properties(a: &Curve, b: &Curve) -> Result<CurveIntersection, TestCaseError> {
    let r = intersect_curves(a, b, tol(), &mut arris_math::Meter::default())
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
    for h in hits_of(&r) {
        for (c, t) in [(a, h.ta), (b, h.tb)] {
            prop_assert!(c.domain().contains(t), "{c:?}: t = {t} outside the domain");
            if c.period().is_some() {
                prop_assert!(t < c.domain().hi(), "{c:?}: t = {t} not below the period");
            }
        }
        let [on_a, on_b] = [(a, h.ta), (b, h.tb)].map(|(c, t)| (c.point(t) - h.point).norm());
        prop_assert!(
            on_a.min(on_b) <= EXACT && on_a.max(on_b) <= tol().linear + EXACT,
            "{a:?} vs {b:?}: {h:?} is {on_a} off the first curve and {on_b} off the second"
        );
    }
    prop_assert!(
        hits_of(&r).windows(2).all(|w| w[0].ta <= w[1].ta),
        "unsorted: {:?}",
        hits_of(&r)
    );
    let again = intersect_curves(a, b, tol(), &mut arris_math::Meter::default())
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
    prop_assert_eq!(&again, &r, "two runs differ");
    Ok(r)
}

/// `got` is `expected` one for one: a hit at each expected point within
/// `bound` — [`TOUCH`] for a touch — of the same kind.
fn expect_hits(
    what: &str,
    got: &[CurveCurveHit],
    expected: &[(Point3, bool)],
    bound: f64,
) -> Result<(), TestCaseError> {
    prop_assert_eq!(
        got.len(),
        expected.len(),
        "{}: {:?} against {:?}",
        what,
        got,
        expected
    );
    for &(p, tangent) in expected {
        let within = if tangent { TOUCH } else { bound };
        let Some(h) = got.iter().find(|h| (h.point - p).norm() <= within) else {
            return fail(format!("{what}: no hit at {p} among {got:?}"));
        };
        prop_assert_eq!(h.tangent, tangent, "{}: {:?} at {}", what, h, p);
    }
    Ok(())
}

/// The ellipse's exact hits against `other`, and the same from its
/// rational form both ways round and, when `fit` is set, its fitted
/// periodic form: the same points, the same kinds.
fn as_the_ellipse_does(conic: &Curve, other: &Curve, fit: bool) -> Result<(), TestCaseError> {
    let exact = common_properties(conic, other)?;
    let expected: Vec<(Point3, bool)> = hits_of(&exact)
        .iter()
        .map(|h| (h.point, h.tangent))
        .collect();
    let (frame, a, b) = conic_parts(conic);
    let rational = Curve::Nurbs(rational_ellipse(&frame, a, b));
    let r = common_properties(&rational, other)?;
    expect_hits("the rational ellipse", hits_of(&r), &expected, EXACT)?;
    let r = common_properties(other, &rational)?;
    expect_hits("the rational ellipse second", hits_of(&r), &expected, EXACT)?;
    if fit {
        let fitted = Curve::Nurbs(fitted(conic));
        let r = common_properties(&fitted, other)?;
        expect_hits("the fitted ellipse", hits_of(&r), &expected, FITTED)?;
        // The fit keeps the ellipse's own parameter. Each hit against the
        // exact hit at its point, not the one at its index: ascending by
        // `ta`, a hit at the seam sorts first on one side and last on the
        // other (a circle through the line at t = 0, on a nightly seed).
        for h in hits_of(&r) {
            let Some(e) = hits_of(&exact).iter().min_by(|x, y| {
                (x.point - h.point)
                    .norm()
                    .total_cmp(&(y.point - h.point).norm())
            }) else {
                return fail(format!("{h:?} has no exact hit"));
            };
            let d = (h.ta - e.ta).rem_euclid(TAU);
            let speed = conic.eval(e.ta).d1.norm();
            prop_assert!(d.min(TAU - d) * speed <= FITTED, "{h:?} against {e:?}");
        }
    }
    Ok(())
}

/// A conic and a point of it at `t`; the unit tangent and the conic's
/// normal there.
fn at(conic: &Curve, t: f64) -> (Point3, Vec3, Vec3) {
    let (frame, _, _) = conic_parts(conic);
    (
        conic.point(t),
        conic.eval(t).d1.normalize(),
        frame.z().into_inner(),
    )
}

#[test]
fn an_ellipse_as_a_nurbs_meets_a_line_where_the_ellipse_does() {
    check(
        (
            prop_oneof![circle(), ellipse()],
            finite_f64(0.0..=TAU),
            unit_vec3(),
            finite_f64(0.05..=3.0),
        ),
        |(conic, t, direction, turn)| {
            let (on, tangent, normal) = at(&conic, t);
            // Through the point in a direction off the plane: one crossing.
            prop_assume!(direction.dot(&normal).abs() >= STEEP);
            let across = Curve::Line {
                origin: on - 1.5 * direction.into_inner(),
                direction,
            };
            as_the_ellipse_does(&conic, &across, true)?;
            // In the plane, turned off the tangent: a chord, two crossings.
            let chord = normal.cross(&tangent) * turn.sin() + tangent * turn.cos();
            let chord = Curve::Line {
                origin: on + 0.7 * chord,
                direction: arris_math::UnitVec3::new_normalize(chord),
            };
            as_the_ellipse_does(&conic, &chord, true)?;
            // Along the tangent: one touch — away from the rational
            // ellipse's seam, whose two ends are each a hit wherever it
            // lies on the other curve ([`rational_ellipse`]): a touch
            // within its reach, `√(2·tol·a)` along the curve, of the seam
            // is those hits besides (t = 0.30006 on a nightly seed).
            let (frame, a, b) = conic_parts(&conic);
            let seam = rational_ellipse(&frame, a, b).eval(0.0).point;
            prop_assume!((on - seam).norm() > 4.0 * (2.0 * tol().linear * a).sqrt());
            let touch = Curve::Line {
                origin: on - 2.0 * tangent,
                direction: arris_math::UnitVec3::new_normalize(tangent),
            };
            as_the_ellipse_does(&conic, &touch, false)
        },
    );
}

#[test]
fn an_ellipse_as_a_nurbs_meets_a_conic_across_its_plane_where_the_ellipse_does() {
    check(
        (
            prop_oneof![circle(), ellipse()],
            finite_f64(0.0..=TAU),
            prop_oneof![circle(), ellipse()],
            finite_f64(0.0..=TAU),
        ),
        |(conic, t, other, s)| {
            // The second conic moved to pass through the first's point at
            // `t`, its plane as drawn.
            let (on, tangent, _) = at(&conic, t);
            let other = other.transformed(&Isometry::from_translation(on - other.point(s)));
            let (plane, _, _) = conic_parts(&other);
            prop_assume!(tangent.dot(&plane.z()).abs() >= STEEP);
            as_the_ellipse_does(&conic, &other, true)
        },
    );
}

#[test]
fn an_ellipse_as_a_nurbs_meets_a_circle_in_its_plane_where_the_quartic_does() {
    check((ellipse(), finite_f64(0.05..=0.95)), |(conic, between)| {
        let (frame, a, b) = conic_parts(&conic);
        prop_assume!(a - b >= 0.1 * b);
        let rational = Curve::Nurbs(rational_ellipse(&frame, a, b));
        let wall = Surface::EllipticCylinder {
            frame,
            major_radius: a,
            minor_radius: b,
        };
        // A concentric circle between the radii: four crossings; at
        // either radius: two touches, at the vertices.
        for (r, touches) in [(b + between * (a - b), false), (a, true), (b, true)] {
            let ring = Curve::Circle { frame, radius: r };
            let CurveSurfaceIntersection::Points(quartic) =
                intersect_curve_surface(&ring, &wall, tol(), &mut arris_math::Meter::default())
                    .map_err(|e| TestCaseError::fail(e.to_string()))?
            else {
                return fail(format!("{ring:?} on {wall:?}"));
            };
            let expected: Vec<(Point3, bool)> =
                quartic.iter().map(|h| (h.point, h.tangent)).collect();
            prop_assert_eq!(expected.len(), if touches { 2 } else { 4 });
            prop_assert!(expected.iter().all(|&(_, tangent)| tangent == touches));
            let r = common_properties(&rational, &ring)?;
            expect_hits("the rational ellipse", hits_of(&r), &expected, EXACT)?;
            let r = common_properties(&ring, &rational)?;
            expect_hits("the rational ellipse second", hits_of(&r), &expected, EXACT)?;
        }
        Ok(())
    });
}

#[test]
fn a_line_or_a_circle_through_a_point_of_a_nurbs_hits_it_there() {
    check(
        (
            nurbs_curve(),
            finite_f64(0.0..=1.0),
            unit_vec3(),
            frame(),
            radius(0.5..=3.0),
            finite_f64(0.0..=TAU),
        ),
        |(n, along, direction, f, r, s)| {
            let t = n.domain().lerp(along);
            let d1 = n.eval(t).d1;
            prop_assume!(d1.norm() > 0.0);
            let tangent = d1.normalize();
            let on = n.eval(t).point;
            let c = Curve::Nurbs(n);
            let found = |r: &CurveIntersection| {
                hits_of(r)
                    .iter()
                    .any(|h| (h.point - on).norm() <= 1e-9 * DEFAULT_SCALE)
            };
            // A line through it not along the curve.
            prop_assume!(tangent.cross(&direction).norm() >= STEEP);
            let line = Curve::Line {
                origin: on + 2.0 * direction.into_inner(),
                direction,
            };
            for (x, y) in [(&c, &line), (&line, &c)] {
                let r = common_properties(x, y)?;
                prop_assert!(found(&r), "{x:?} vs {y:?}: nothing at {on} in {r:?}");
            }
            // A circle through it whose plane the curve crosses.
            let ring = Curve::Circle {
                frame: f,
                radius: r,
            };
            let ring = ring.transformed(&Isometry::from_translation(on - ring.point(s)));
            prop_assume!(tangent.dot(&f.z()).abs() >= STEEP);
            for (x, y) in [(&c, &ring), (&ring, &c)] {
                let r = common_properties(x, y)?;
                prop_assert!(found(&r), "{x:?} vs {y:?}: nothing at {on} in {r:?}");
            }
            Ok(())
        },
    );
}

/// A segment of a line as a NURBS of degree one from `lo` to `hi`.
fn segment(origin: Point3, direction: Vec3, lo: f64, hi: f64) -> NurbsCurve {
    NurbsCurve::new(
        1,
        vec![lo, lo, hi, hi],
        vec![origin + lo * direction, origin + hi * direction],
        vec![1.0; 2],
    )
    .unwrap()
}

#[test]
fn a_nurbs_on_a_line_or_a_conic_is_coincident_and_one_beside_it_is_not() {
    check(
        (prop_oneof![circle(), ellipse()], unit_vec3()),
        |(conic, away)| {
            let (frame, a, b) = conic_parts(&conic);
            let lift = Isometry::from_translation(10.0 * tol().linear * frame.z().into_inner());
            let on = Curve::Nurbs(rational_ellipse(&frame, a, b));
            let beside = on.transformed(&lift);
            let line = Curve::Line {
                origin: frame.origin(),
                direction: frame.x(),
            };
            let straight = Curve::Nurbs(segment(frame.origin(), frame.x().into_inner(), -3.0, 2.0));
            let across = away.into_inner() - away.dot(&frame.x()) * frame.x().into_inner();
            prop_assume!(across.norm() > 0.5);
            let off_line = straight.transformed(&Isometry::from_translation(
                10.0 * tol().linear * across.normalize(),
            ));
            for (x, y, same) in [
                (&on, &conic, true),
                (&straight, &line, true),
                (&beside, &conic, false),
                (&off_line, &line, false),
            ] {
                for (p, q) in [(x, y), (y, x)] {
                    let r = common_properties(p, q)?;
                    prop_assert_eq!(
                        r == CurveIntersection::Coincident,
                        same,
                        "{:?} vs {:?}: {:?}",
                        p,
                        q,
                        r
                    );
                    let verdict = curves_coincide(p, q, tol(), &mut arris_math::Meter::default())
                        .map_err(|e| TestCaseError::fail(e.to_string()))?;
                    prop_assert_eq!(
                        verdict,
                        same,
                        "curves_coincide({:?}, {:?}, &mut arris_math::Meter::default())",
                        p,
                        q
                    );
                }
            }
            Ok(())
        },
    );
}

#[test]
fn two_nurbs_curves_are_the_same_spline_apart_or_unsupported() {
    check(nurbs_curve(), |n| {
        let c = Curve::Nurbs(n.clone());
        let unsupported = |r: Result<_, GeomError>| {
            matches!(
                r,
                Err(GeomError::Unsupported { a, b })
                    if a == GeomKind::Curve(c.kind()) && b == GeomKind::Curve(c.kind())
            )
        };
        prop_assert!(unsupported(
            intersect_curves(&c, &c, tol(), &mut arris_math::Meter::default()).map(|_| ())
        ));
        let same =
            |x: &Curve, y: &Curve| curves_coincide(x, y, tol(), &mut arris_math::Meter::default());
        prop_assert_eq!(same(&c, &c), Ok(true));
        // Within the tolerance of itself: every control point moved less.
        let nudged = c.transformed(&Isometry::from_translation(Vec3::new(
            0.3 * tol().linear,
            0.0,
            0.0,
        )));
        prop_assert_eq!(same(&c, &nudged), Ok(true));
        let apart = c.transformed(&Isometry::from_translation(Vec3::new(1e-3, 0.0, 0.0)));
        prop_assert_eq!(same(&c, &apart), Ok(false));
        prop_assert_eq!(same(&apart, &c), Ok(false));
        // The same curve over one more knot: only a marcher could say.
        let refined = Curve::Nurbs(
            n.insert_knot(n.domain().lerp(0.37), 1)
                .map_err(|e| TestCaseError::fail(e.to_string()))?,
        );
        prop_assert!(unsupported(same(&c, &refined).map(|_| ())));
        Ok(())
    });
}
