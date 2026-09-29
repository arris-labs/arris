//! Pcurves: `Curve2` circles of both handedness evaluate and project,
//! and `fit_curve2` approximates the ellipse-on-cylinder sinusoid at the
//! caller's parameter within the tolerance, or says it cannot.

use core::f64::consts::TAU;

use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, radius};
use arris_geom::{Curve2, FitError, GeomError, MAX_FIT_SPANS, fit_curve2};
use arris_math::{Frame2, Handedness, Interval, Point2, Vec2};
use proptest::prelude::*;

/// Closed form against evaluation.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;
/// The tolerance the sinusoid is fitted to: the order of a model's edge
/// tolerance on features at the default scale.
const FIT_TOL: f64 = 1e-7;
/// Parameters the fitted curve is checked at, both ends included.
const CHECKS: usize = 1000;

fn frame2() -> impl Strategy<Value = Frame2> {
    (
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        finite_f64(0.0..=TAU),
        any::<bool>(),
    )
        .prop_map(|(u, v, angle, right)| {
            let handedness = if right {
                Handedness::Right
            } else {
                Handedness::Left
            };
            Frame2::new(
                Point2::new(u, v),
                Vec2::new(angle.cos(), angle.sin()),
                handedness,
            )
            .unwrap()
        })
}

#[test]
fn circles_of_both_handedness_evaluate_and_project() {
    check(
        (
            frame2(),
            radius(0.1..=10.0),
            finite_f64(-1.0..=TAU + 1.0),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        ),
        |(f, r, t, pu, pv)| {
            let c = Curve2::Circle {
                frame: f,
                radius: r,
            };
            let e = c.eval(t);
            // The closed form in the frame's own axes.
            let expected =
                f.origin() + r * (t.cos() * f.x().into_inner() + t.sin() * f.y().into_inner());
            prop_assert!((e.point - expected).norm() <= EXACT);
            // Traversal: the tangent turns from the radial by +90° in a
            // right-handed frame and −90° in a left-handed one.
            let radial = e.point - f.origin();
            let turn = radial.perp(&e.d1);
            let turns_with_the_frame = if f.is_right_handed() {
                turn > 0.0
            } else {
                turn < 0.0
            };
            prop_assert!(turns_with_the_frame, "turn {turn}");
            prop_assert!((e.d2 + radial).norm() <= EXACT);
            prop_assert!((c.point(t + TAU) - e.point).norm() <= EXACT);
            // Projection: the nearest point is where the ray from the
            // centre meets the circle, at the right parameter.
            let p = Point2::new(pu, pv);
            let Ok(proj) = c.project(p) else {
                prop_assert!((p - f.origin()).norm() <= 1e-12 * DEFAULT_SCALE);
                return Ok(());
            };
            let d = p - f.origin();
            let expected = f.origin() + r * d.normalize();
            prop_assert!(
                (proj.point - expected).norm() <= EXACT,
                "{} vs {expected}",
                proj.point
            );
            prop_assert!((proj.distance - (d.norm() - r).abs()).abs() <= EXACT);
            prop_assert!((c.point(proj.t) - proj.point).norm() <= EXACT);
            prop_assert!((0.0..TAU).contains(&proj.t));
            let again = c.project(proj.point).unwrap();
            prop_assert!(again.distance <= EXACT && (again.t - proj.t).abs() <= 1e-12);
            Ok(())
        },
    );
}

/// `u = t`, `v = A + B cos t + C sin t`: the pcurve of an oblique plane
/// section on a cylinder, in the cylinder's own `u`.
fn sinusoid(a: f64, b: f64, c: f64) -> impl Fn(f64) -> Point2 + Copy {
    move |t: f64| Point2::new(t, a + b * t.cos() + c * t.sin())
}

#[test]
fn the_sinusoid_is_fitted_within_tolerance_with_a_bounded_knot_count() {
    check(
        (
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-10.0..=10.0),
            finite_f64(-10.0..=10.0),
            finite_f64(0.0..=TAU),
            finite_f64(0.5..=TAU),
            3..=5usize,
        ),
        |(a, b, c, start, length, degree)| {
            let f = sinusoid(a, b, c);
            let range = Interval::new(start, start + length).unwrap();
            let deviation = |t: f64, q: Point2| (q - f(t)).norm();
            let fit = fit_curve2(
                f,
                range,
                degree,
                deviation,
                FIT_TOL,
                &mut arris_math::Meter::default(),
            )
            .map_err(|e| TestCaseError::fail(format!("{e} for A={a} B={b} C={c} on {range:?}")))?;
            prop_assert_eq!(fit.domain(), range);
            prop_assert_eq!(fit.degree(), degree);
            let spans = fit.knots().len() - 2 * (degree + 1) + 1;
            prop_assert!(spans <= MAX_FIT_SPANS, "{spans} spans");
            let mut worst = 0.0f64;
            for i in 0..=CHECKS {
                let t = range.lerp(i as f64 / CHECKS as f64);
                worst = worst.max(deviation(t, fit.eval(t).point));
            }
            prop_assert!(
                worst <= FIT_TOL,
                "worst deviation {worst} over {spans} spans for A={a} B={b} C={c} degree {degree}"
            );
            // Both ends are interpolated exactly.
            prop_assert!((fit.eval(range.lo()).point - f(range.lo())).norm() <= EXACT);
            prop_assert!((fit.eval(range.hi()).point - f(range.hi())).norm() <= EXACT);
            // Same parameter by construction: `u` is `t` itself.
            let t = range.lerp(0.37);
            prop_assert!((fit.eval(t).point.x - t).abs() <= FIT_TOL);
            Ok(())
        },
    );
}

#[test]
fn a_tolerance_below_the_sampling_noise_diverges() {
    // The deviation carries noise the fit can never get under, so the
    // refinement runs to the bound and stops there.
    let f = sinusoid(1.0, 2.0, 0.5);
    let range = Interval::new(0.0, TAU).unwrap();
    let noise = 1e-6;
    let deviation = |t: f64, q: Point2| (q - f(t)).norm() + noise * (1000.0 * t).sin().abs();
    let err = fit_curve2(
        f,
        range,
        3,
        deviation,
        1e-9,
        &mut arris_math::Meter::default(),
    )
    .unwrap_err();
    let FitError::Diverged {
        spans,
        deviation: worst,
    } = err
    else {
        panic!("expected Diverged, got {err}");
    };
    assert!(spans <= MAX_FIT_SPANS, "{spans}");
    assert!(worst >= 1e-9 * 0.5, "{worst}");
    let wrapped: GeomError = err.into();
    assert!(matches!(wrapped, GeomError::Fit(FitError::Diverged { .. })));
    // The same request with a tolerance above the noise succeeds.
    assert!(
        fit_curve2(
            f,
            range,
            3,
            deviation,
            1e-5,
            &mut arris_math::Meter::default()
        )
        .is_ok()
    );
}

#[test]
fn a_fitted_curve_is_a_curve2_that_projects() {
    let f = sinusoid(0.0, 1.0, 0.0);
    let range = Interval::new(0.0, TAU).unwrap();
    let fit = fit_curve2(
        f,
        range,
        3,
        |t, q| (q - f(t)).norm(),
        1e-9,
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    let c = Curve2::Nurbs(fit);
    let p = Point2::new(1.0, 5.0);
    let proj = c.project(p).unwrap();
    // The nearest point of `v = cos u` to (1, 5): the residual `(P − q)·P'`
    // vanishes there.
    let e = c.eval(proj.t);
    assert!((e.point - p).dot(&e.d1).abs() <= 1e-9 * (e.point - p).norm() * e.d1.norm());
    assert!(c.domain().contains(proj.t));
}
