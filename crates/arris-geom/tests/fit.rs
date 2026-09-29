//! 3D fits: `fit_curve` approximates an open space curve at the caller's
//! parameter within the tolerance, `fit_curve_periodic` a closed one with
//! a wrapping knot vector, and both say why when they cannot.

use core::f64::consts::TAU;

use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, pose, radius};
use arris_geom::{FitError, GeomError, MAX_FIT_SPANS, NurbsCurve, fit_curve, fit_curve_periodic};
use arris_math::{Interval, Point3};
use proptest::prelude::*;

/// Closed form against evaluation.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;
/// The tolerance the curves are fitted to: the order of a model's edge
/// tolerance on features at the default scale.
const FIT_TOL: f64 = 1e-7;
/// Parameters a fitted curve is checked at, both ends included — far
/// denser than the fit's own checks, so "between samples" is tested.
const CHECKS: usize = 2000;

/// The worst distance from `fit(t)` to `f(t)` over `CHECKS + 1` even
/// parameters of `range`.
fn worst_deviation(fit: &NurbsCurve, f: impl Fn(f64) -> Point3, range: Interval) -> f64 {
    (0..=CHECKS)
        .map(|i| {
            let t = range.lerp(i as f64 / CHECKS as f64);
            (fit.eval(t).point - f(t)).norm()
        })
        .fold(0.0, f64::max)
}

fn spans(fit: &NurbsCurve) -> usize {
    fit.control_points().len() - fit.degree()
}

/// Viviani's curve, the section of a sphere of radius 2 and a cylinder
/// of radius 1 through its centre: closed over `[0, 4π]`.
fn viviani(t: f64) -> Point3 {
    Point3::new(1.0 + t.cos(), t.sin(), 2.0 * (0.5 * t).sin())
}

#[test]
fn a_helix_in_any_pose_is_fitted_within_tolerance() {
    check(
        (
            pose(),
            radius(0.1..=10.0),
            finite_f64(-3.0..=3.0),
            finite_f64(-TAU..=TAU),
            finite_f64(0.5..=2.0 * TAU),
            3..=5usize,
        ),
        |(motion, r, pitch, start, length, degree)| {
            let f = move |t: f64| motion.apply(Point3::new(r * t.cos(), r * t.sin(), pitch * t));
            let range = Interval::new(start, start + length).unwrap();
            let deviation = |t: f64, q: Point3| (q - f(t)).norm();
            let fit = fit_curve(
                f,
                range,
                degree,
                deviation,
                FIT_TOL,
                &mut arris_math::Meter::default(),
            )
            .map_err(|e| {
                TestCaseError::fail(format!("{e} for r={r} pitch={pitch} on {range:?}"))
            })?;
            prop_assert_eq!(fit.domain(), range);
            prop_assert_eq!(fit.degree(), degree);
            prop_assert_eq!(fit.period(), None);
            prop_assert!(spans(&fit) <= MAX_FIT_SPANS);
            let worst = worst_deviation(&fit, f, range);
            prop_assert!(
                worst <= FIT_TOL,
                "worst deviation {worst} over {} spans, degree {degree}",
                spans(&fit)
            );
            // Both ends are interpolated.
            prop_assert!((fit.eval(range.lo()).point - f(range.lo())).norm() <= EXACT);
            prop_assert!((fit.eval(range.hi()).point - f(range.hi())).norm() <= EXACT);
            Ok(())
        },
    );
}

#[test]
fn a_closed_curve_in_any_pose_is_fitted_periodically() {
    // A closed space curve with three harmonics of random amplitude: an
    // ellipse wobbling out of its plane, as a section loop does.
    check(
        (
            pose(),
            radius(0.1..=10.0),
            radius(0.1..=10.0),
            finite_f64(-3.0..=3.0),
            finite_f64(-TAU..=TAU),
            3..=5usize,
        ),
        |(motion, a, b, c, start, degree)| {
            let f = move |t: f64| {
                motion.apply(Point3::new(
                    a * t.cos(),
                    b * t.sin(),
                    c * (2.0 * t).sin() + 0.3 * c * (3.0 * t).cos(),
                ))
            };
            let range = Interval::new(start, start + TAU).unwrap();
            let deviation = |t: f64, q: Point3| (q - f(t)).norm();
            let fit = fit_curve_periodic(
                f,
                range,
                degree,
                deviation,
                FIT_TOL,
                &mut arris_math::Meter::default(),
            )
            .map_err(|e| TestCaseError::fail(format!("{e} for a={a} b={b} c={c}")))?;
            prop_assert_eq!(fit.domain(), range);
            prop_assert_eq!(fit.degree(), degree);
            prop_assert!(fit.period().is_some_and(|p| (p - TAU).abs() <= 1e-12 * TAU));
            let worst = worst_deviation(&fit, f, range);
            prop_assert!(
                worst <= FIT_TOL,
                "worst deviation {worst} over {} spans, degree {degree}",
                spans(&fit)
            );
            // Closed, and smooth across the seam, to rounding.
            let (lo, hi) = (fit.eval(range.lo()), fit.eval(range.hi()));
            prop_assert!((lo.point - hi.point).norm() <= EXACT);
            let d1_scale = lo.d1.norm().max(1.0);
            prop_assert!((lo.d1 - hi.d1).norm() <= 1e-12 * d1_scale);
            // Just either side of the seam agrees with the curve too.
            let eps = 1e-9 * TAU;
            prop_assert!(
                (fit.eval(range.hi() - eps).point - f(range.hi() - eps)).norm() <= FIT_TOL
            );
            prop_assert!(
                (fit.eval(range.lo() + eps).point - f(range.lo() + eps)).norm() <= FIT_TOL
            );
            Ok(())
        },
    );
}

/// Viviani's curve fitted as one periodic loop is held to its closed
/// form between samples at every degree the section fit may choose.
#[test]
fn vivianis_curve_is_one_periodic_fit_within_tolerance() {
    let range = Interval::new(0.0, 2.0 * TAU).unwrap();
    let deviation = |t: f64, q: Point3| (q - viviani(t)).norm();
    for degree in 3..=5 {
        let fit = fit_curve_periodic(
            viviani,
            range,
            degree,
            deviation,
            FIT_TOL,
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert_eq!(fit.period(), Some(2.0 * TAU));
        let worst = worst_deviation(&fit, viviani, range);
        assert!(
            worst <= FIT_TOL,
            "degree {degree}: {worst} over {} spans",
            spans(&fit)
        );
        // A second period evaluates the same loop.
        for i in 0..50 {
            let t = range.lerp(i as f64 / 50.0);
            let (a, b) = (fit.eval(t).point, fit.eval(t + 2.0 * TAU).point);
            assert!((a - b).norm() <= 1e-12, "t = {t}");
        }
    }
    // The same loop fitted open is clamped and closed only because both
    // ends are interpolated.
    let open = fit_curve(
        viviani,
        range,
        3,
        deviation,
        FIT_TOL,
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!(open.period(), None);
    assert!((open.eval(0.0).point - open.eval(2.0 * TAU).point).norm() <= EXACT);
}

/// Fewer spans than the degree: the wrap puts two of a span's functions
/// on one free control point, and the fit still closes.
#[test]
fn a_periodic_fit_with_fewer_spans_than_its_degree_wraps() {
    let circle = |t: f64| Point3::new(t.cos(), t.sin(), 0.0);
    let range = Interval::new(0.0, TAU).unwrap();
    let deviation = |t: f64, q: Point3| (q - circle(t)).norm();
    let fit = fit_curve_periodic(
        circle,
        range,
        5,
        deviation,
        1e-2,
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert!(spans(&fit) < 5, "{} spans", spans(&fit));
    assert_eq!(fit.period(), Some(TAU));
    assert!(worst_deviation(&fit, circle, range) <= 1e-2);
    assert!((fit.eval(0.0).point - fit.eval(TAU).point).norm() <= EXACT);
}

#[test]
fn fits_are_deterministic() {
    let range = Interval::new(0.0, 2.0 * TAU).unwrap();
    let deviation = |t: f64, q: Point3| (q - viviani(t)).norm();
    let a = fit_curve_periodic(
        viviani,
        range,
        4,
        deviation,
        FIT_TOL,
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    let b = fit_curve_periodic(
        viviani,
        range,
        4,
        deviation,
        FIT_TOL,
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!(a, b);
    let range = Interval::new(0.3, 5.0).unwrap();
    let a = fit_curve(
        viviani,
        range,
        3,
        deviation,
        FIT_TOL,
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    let b = fit_curve(
        viviani,
        range,
        3,
        deviation,
        FIT_TOL,
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!(a, b);
}

#[test]
fn bad_requests_are_named() {
    let f = |t: f64| Point3::new(t, 0.0, 0.0);
    let loop_ = |t: f64| Point3::new(t.cos(), t.sin(), 0.0);
    let range = Interval::new(0.0, TAU).unwrap();
    let exact = |t: f64, q: Point3| (q - loop_(t)).norm();
    type Fit = fn(
        &dyn Fn(f64) -> Point3,
        Interval,
        usize,
        &dyn Fn(f64, Point3) -> f64,
        f64,
    ) -> Result<NurbsCurve, FitError>;
    let fits: [Fit; 2] = [
        |f, range, degree, dev, tol| {
            fit_curve(
                f,
                range,
                degree,
                dev,
                tol,
                &mut arris_math::Meter::default(),
            )
        },
        |f, range, degree, dev, tol| {
            fit_curve_periodic(
                f,
                range,
                degree,
                dev,
                tol,
                &mut arris_math::Meter::default(),
            )
        },
    ];
    for fit in fits {
        assert!(matches!(
            fit(&loop_, range, 0, &exact, 1e-6),
            Err(FitError::Degenerate(_))
        ));
        assert!(matches!(
            fit(&loop_, Interval::REAL, 3, &exact, 1e-6),
            Err(FitError::Degenerate(_))
        ));
        assert!(matches!(
            fit(&loop_, range, 3, &exact, f64::NAN),
            Err(FitError::InvalidTolerance(t)) if t.is_nan()
        ));
        assert_eq!(
            fit(&loop_, range, 3, &exact, 0.0),
            Err(FitError::InvalidTolerance(0.0))
        );
        let hole = |t: f64| {
            if t > 1.0 && t < 2.0 {
                Point3::new(f64::NAN, 0.0, 0.0)
            } else {
                loop_(t)
            }
        };
        assert!(matches!(
            fit(&hole, range, 3, &|_, _| 0.0, 1e-6),
            Err(FitError::NonFinite { t }) if t > 1.0 && t < 2.0
        ));
        // A deviation the fit can never get under, everywhere but at the
        // seam: the refinement runs to the bound and stops there.
        let noise = |t: f64, _: Point3| if t == TAU { 0.0 } else { 1.0 };
        let e = fit(&loop_, range, 3, &noise, 1e-6).unwrap_err();
        assert!(
            matches!(e, FitError::Diverged { spans, deviation }
                if spans <= MAX_FIT_SPANS && deviation == 1.0),
            "{e}"
        );
    }
    // A periodic fit of a curve that does not close says so, up front.
    let e = fit_curve_periodic(
        f,
        range,
        3,
        |t: f64, q: Point3| (q - f(t)).norm(),
        1e-6,
        &mut arris_math::Meter::default(),
    )
    .unwrap_err();
    assert!(
        matches!(&e, FitError::Degenerate(reason) if reason.contains("does not close")),
        "{e}"
    );
    let wrapped: GeomError = e.into();
    assert!(matches!(wrapped, GeomError::Fit(FitError::Degenerate(_))));
    // The open fit of the same curve is fine.
    assert!(
        fit_curve(
            f,
            range,
            3,
            |t: f64, q: Point3| (q - f(t)).norm(),
            1e-6,
            &mut arris_math::Meter::default()
        )
        .is_ok()
    );
}
