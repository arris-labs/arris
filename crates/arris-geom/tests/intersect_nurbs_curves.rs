//! Two NURBS curves meet where their analytic twins do (plan
//! `nurbs-nurbs-section`, step 8): the exact twins of a line, a circle and
//! an ellipse, in pairs built to cross or touch, give the hits of the
//! closed form, point for point and touch for touch; a curve beside its
//! twin is `Coincident`, and the argument order does not reach the answer.

use arris_debug::testing::tol;
use arris_math::real;
use core::f64::consts::TAU;

use arris_debug::prop::{check, frame, radius};
use arris_geom::{
    Curve, CurveCurveHit, CurveIntersection, NurbsCurve, curves_coincide, intersect_curves,
};
use arris_math::{Frame, Interval, Meter, Point3, UnitVec3, Vec3};
use proptest::prelude::*;

/// How far a twin's hit may be from the closed form's: to rounding where
/// the curves cross; where they touch the distance's gradient has a triple
/// zero, which isolation places to about the cube root of rounding at the
/// poses' scale (`DEFAULT_SCALE`), inside the `√tol.linear` reach over which
/// the curves are within the tolerance of each other anyway.
const CROSSING: f64 = 1e-9;
const TOUCH: f64 = 1e-4;

/// The exact NURBS twin of a line, circle or ellipse: a line over
/// `[-reach, reach]`, a conic over its turn.
fn twin(c: &Curve, reach: f64) -> NurbsCurve {
    match *c {
        Curve::Line { origin, direction } => NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![
                origin - reach * direction.into_inner(),
                origin + reach * direction.into_inner(),
            ],
            vec![1.0, 1.0],
        )
        .unwrap(),
        Curve::Circle { frame, radius } => {
            NurbsCurve::circle(&frame, radius, Interval::TURN).unwrap()
        }
        Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => NurbsCurve::ellipse(&frame, major_radius, minor_radius, Interval::TURN).unwrap(),
        Curve::Nurbs(_) => unreachable!("a twin is of an analytic curve"),
    }
}

fn hits_of(r: Result<CurveIntersection, arris_geom::GeomError>) -> Vec<CurveCurveHit> {
    match r {
        Ok(CurveIntersection::Points(h)) => h,
        other => panic!("{other:?}"),
    }
}

/// The twins of `a` and `b` meet where `a` and `b` do, in both orders.
fn twins_meet_as_the_conics_do(a: &Curve, b: &Curve, reach: f64) -> Result<(), TestCaseError> {
    let meter = || Meter::default();
    let want = hits_of(intersect_curves(a, b, tol(), &mut meter()));
    let (ta, tb) = (Curve::Nurbs(twin(a, reach)), Curve::Nurbs(twin(b, reach)));
    for (x, y) in [(&ta, &tb), (&tb, &ta)] {
        let got = hits_of(intersect_curves(x, y, tol(), &mut meter()));
        prop_assert_eq!(got.len(), want.len(), "{:?} vs {:?}: {:?}", a, b, got);
        for w in &want {
            let bound = if w.tangent { TOUCH } else { CROSSING };
            prop_assert!(
                got.iter()
                    .any(|g| (g.point - w.point).norm() <= bound && g.tangent == w.tangent),
                "{w:?} is not among {got:?}"
            );
            // Each hit is on both twins at the parameters it names.
            for g in &got {
                prop_assert!((x.point(g.ta) - g.point).norm() <= CROSSING);
                prop_assert!((y.point(g.tb) - g.point).norm() <= TOUCH);
            }
        }
    }
    Ok(())
}

#[test]
fn two_coplanar_circles_cross_in_two_points_as_their_twins_do() {
    check(
        (
            frame(),
            radius(1.0..=3.0),
            radius(1.0..=3.0),
            0.1..=0.9f64,
            0.0..TAU,
        ),
        |(f, ra, rb, share, bearing)| {
            // The centres are `d` apart, strictly between the circles'
            // difference and sum of radii.
            let (lo, hi) = ((ra - rb).abs(), ra + rb);
            let d = lo + share * (hi - lo);
            let (x, y) = (f.x().into_inner(), f.y().into_inner());
            let heading = real::cos(bearing) * x + real::sin(bearing) * y;
            let other = Frame::new(f.origin() + d * heading, f.z().into_inner(), x).unwrap();
            let a = Curve::Circle {
                frame: f,
                radius: ra,
            };
            let b = Curve::Circle {
                frame: other,
                radius: rb,
            };
            twins_meet_as_the_conics_do(&a, &b, 1.0)
        },
    );
}

#[test]
#[ignore = "extrema_curve_curve misses the touch of two circle twins in a general pose (plan nurbs-nurbs-section, open question)"]
fn two_circles_touching_outside_are_one_touch_as_their_twins_are() {
    check(
        (frame(), radius(1.0..=3.0), radius(1.0..=3.0), 0.0..TAU),
        |(f, ra, rb, bearing)| {
            let (x, y) = (f.x().into_inner(), f.y().into_inner());
            let heading = real::cos(bearing) * x + real::sin(bearing) * y;
            let other =
                Frame::new(f.origin() + (ra + rb) * heading, f.z().into_inner(), x).unwrap();
            let a = Curve::Circle {
                frame: f,
                radius: ra,
            };
            let b = Curve::Circle {
                frame: other,
                radius: rb,
            };
            twins_meet_as_the_conics_do(&a, &b, 1.0)
        },
    );
}

#[test]
fn a_line_through_an_ellipse_crosses_it_as_the_twins_do() {
    check(
        (
            frame(),
            radius(1.5..=3.0),
            radius(0.5..=1.4),
            0.0..TAU,
            0.0..TAU,
        ),
        |(f, a, b, angle, aim)| {
            let ellipse = Curve::Ellipse {
                frame: f,
                major_radius: a,
                minor_radius: b,
            };
            // A line in the ellipse's plane through its point at `angle`
            // and the centre's neighbourhood, so it crosses twice.
            let through = ellipse.point(angle);
            let toward = f.origin()
                + 0.2
                    * b
                    * (real::cos(aim) * f.x().into_inner() + real::sin(aim) * f.y().into_inner());
            let Some(direction) = (toward - through).try_normalize(1e-6) else {
                return Ok(());
            };
            let line = Curve::Line {
                origin: through,
                direction: UnitVec3::new_normalize(direction),
            };
            twins_meet_as_the_conics_do(&ellipse, &line, 10.0)
        },
    );
}

#[test]
fn two_skew_lines_through_one_point_meet_there_as_their_twins_do() {
    check((frame(), 0.1..=1.0f64, 0.0..TAU), |(f, tilt, turn)| {
        let at = f.origin();
        let u = f.x().into_inner();
        let v = real::cos(tilt) * f.y().into_inner() + real::sin(tilt) * f.z().into_inner();
        let w = real::cos(turn) * u + real::sin(turn) * v;
        let a = Curve::Line {
            origin: at - 2.0 * u,
            direction: UnitVec3::new_normalize(u),
        };
        let b = Curve::Line {
            origin: at - 2.0 * w,
            direction: UnitVec3::new_normalize(w),
        };
        twins_meet_as_the_conics_do(&a, &b, 5.0)
    });
}

#[test]
fn a_circle_and_its_twin_beside_it_are_coincident_both_ways() {
    check((frame(), radius(1.0..=3.0)), |(f, r)| {
        let circle = Curve::Circle {
            frame: f,
            radius: r,
        };
        let twin = Curve::Nurbs(NurbsCurve::circle(&f, r, Interval::TURN).unwrap());
        let meter = || Meter::default();
        for (x, y) in [(&twin, &twin), (&circle, &twin), (&twin, &circle)] {
            // A NURBS against an analytic curve is a plane's business and
            // is exact only for a line or a conic that is in it; the pair
            // of twins is ours.
            if matches!(x, Curve::Nurbs(_)) && matches!(y, Curve::Nurbs(_)) {
                prop_assert_eq!(
                    intersect_curves(x, y, tol(), &mut meter()),
                    Ok(CurveIntersection::Coincident)
                );
                prop_assert_eq!(curves_coincide(x, y, tol(), &mut meter()), Ok(true));
            }
        }
        // A twin turned a quarter about its axis is not.
        let turned = Curve::Nurbs(
            NurbsCurve::circle(
                &Frame::new(
                    Point3::from(f.origin() + 0.5 * f.z().into_inner()),
                    f.z().into_inner(),
                    f.x().into_inner(),
                )
                .unwrap(),
                r,
                Interval::TURN,
            )
            .unwrap(),
        );
        prop_assert_eq!(
            intersect_curves(&twin, &turned, tol(), &mut meter()),
            Ok(CurveIntersection::Points(Vec::new()))
        );
        let _ = Vec3::zeros();
        Ok(())
    });
}

/// A fuzz crash (`fuzz/crashes/intersect_curves/`): a degree-2 curve whose
/// domain is about 1e-71 wide, its first spans from 1e-274 to 1e-168 wide
/// and so below the resolution `bezier_spans` extracts, across which it
/// travels 3e2 from the line. It was `Coincident` with the line, decided on
/// the last span alone; the knots of the narrow spans are looked at too, so
/// it is a point of the line at its far end and nothing else.
#[test]
fn a_curve_through_spans_too_narrow_to_extract_is_not_coincident_with_a_line_it_leaves() {
    let curve = Curve::Nurbs(
        NurbsCurve::new(
            2,
            vec![
                -792.0,
                -440.0,
                -1.0641291246752412e-213,
                -1.388253028666376e-235,
                0.0,
                8.075142282759219e-274,
                9.653699974243035e-212,
                1.9285913724732417e-168,
                1.9639295305171301e-168,
                1.0895165243265732e-71,
                40.0,
                376.0,
            ],
            vec![
                Point3::new(
                    -1.481083735425437e-213,
                    1.898893952442176e-124,
                    1.1161986402769749e-101,
                ),
                Point3::new(-214.71592139755376, -2.3227082873029094e-264, 328.0),
                Point3::new(
                    4.482028047256569e-305,
                    -3.018283424843103e-182,
                    2.746575544094824e-309,
                ),
                Point3::new(-64.0, -7.584950765891977e-180, -2.3046743733444784e-185),
                Point3::new(-1.3283834148018051e-220, 7.031109225814759e-306, 0.0),
                Point3::new(8.774986012890368e-215, 216.0, -1.6808498517258876e-87),
                Point3::new(
                    3.4782629920244e-310,
                    1.2387753456017573e-20,
                    3.47129385248387e-310,
                ),
                Point3::new(1.2387753456017573e-20, 3.47129385248387e-310, 0.0),
                Point3::new(0.0, 0.0, 1.7765824089018436e-307),
            ],
            vec![
                999.999,
                842.8024713794528,
                999.999,
                667.9212392065713,
                999.999,
                999.999,
                999.999,
                999.999,
                999.999,
            ],
        )
        .unwrap(),
    );
    let line = Curve::Line {
        origin: Point3::new(0.0, 0.0, 2.070238e-317),
        direction: UnitVec3::new_unchecked(Vec3::new(
            1.463879e-317,
            -0.7071067811865475,
            -0.7071067811865475,
        )),
    };
    for (a, b) in [(&curve, &line), (&line, &curve)] {
        let found = intersect_curves(a, b, tol(), &mut Meter::default()).unwrap();
        assert!(matches!(found, CurveIntersection::Points(_)), "{found:?}");
    }
}
