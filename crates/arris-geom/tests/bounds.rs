//! `Curve::bounds` and `Surface::bounds` contain what they bound in any
//! pose over any finite range, are tight where the geometry is affine,
//! and refuse a range that is not finite: the cheap reject a boolean's
//! face pairs go through (ADR-0004).

use arris_debug::prop::geom::{curve, nurbs_curve, nurbs_surface, surface};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64};
use arris_geom::{Curve, Surface};
use arris_math::{Aabb, Frame, Interval, Point3, UnitVec3, Vec3};
use proptest::prelude::*;

/// Rounding at the scale: a box holds its own geometry to this.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;
/// Parameters sampled along a curve and per direction on a surface.
const SAMPLES: usize = 33;

/// `true` when `p` is in `b` allowing `EXACT` of rounding on every side.
fn holds(b: &Aabb, p: Point3) -> bool {
    (0..3).all(|k| p[k] >= b.min[k] - EXACT && p[k] <= b.max[k] + EXACT)
}

/// A sub-range of `domain`, `[lo + a·len, lo + b·len]` with `a < b`.
fn sub_range(domain: Interval, a: f64, b: f64) -> Option<Interval> {
    let (lo, len) = (domain.lo(), domain.length());
    let (lo, len) = if len.is_finite() {
        (lo, len)
    } else {
        (-DEFAULT_SCALE, 2.0 * DEFAULT_SCALE)
    };
    Interval::new(lo + a.min(b) * len, lo + a.max(b) * len).ok()
}

#[test]
fn every_curve_stays_inside_its_bounds() {
    check(
        (curve(), finite_f64(0.0..=1.0), finite_f64(0.0..=1.0)),
        |(c, a, b)| {
            let Some(range) = sub_range(c.domain(), a, b) else {
                return Ok(());
            };
            let Some(bounds) = c.bounds(range) else {
                return Err(TestCaseError::fail(format!(
                    "{c:?}: no bounds for {range:?}"
                )));
            };
            for i in 0..=SAMPLES {
                let t = range.lo() + range.length() * i as f64 / SAMPLES as f64;
                let p = c.point(t);
                prop_assert!(
                    holds(&bounds, p),
                    "{c:?}: {p:?} is outside {bounds:?} at {t}"
                );
            }
            prop_assert_eq!(c.bounds(range), c.bounds(range), "two runs differ");
            Ok(())
        },
    );
}

#[test]
fn a_nurbs_curves_control_hull_holds_it() {
    check(
        (nurbs_curve(), finite_f64(0.0..=1.0), finite_f64(0.0..=1.0)),
        |(n, a, b)| {
            let c = Curve::Nurbs(n);
            let Some(range) = sub_range(c.domain(), a, b) else {
                return Ok(());
            };
            let Some(bounds) = c.bounds(range) else {
                return Err(TestCaseError::fail("no bounds".to_string()));
            };
            for i in 0..=SAMPLES {
                let t = range.lo() + range.length() * i as f64 / SAMPLES as f64;
                prop_assert!(
                    holds(&bounds, c.point(t)),
                    "{:?} outside {bounds:?}",
                    c.point(t)
                );
            }
            Ok(())
        },
    );
}

#[test]
fn a_lines_bounds_are_its_two_endpoints() {
    check(
        (
            arris_debug::prop::point_in_box(DEFAULT_SCALE),
            arris_debug::prop::unit_vec3(),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        ),
        |(origin, direction, a, b)| {
            prop_assume!((a - b).abs() > 1e-6);
            let line = Curve::Line { origin, direction };
            let range = Interval::new(a.min(b), a.max(b))
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            let bounds = line.bounds(range).expect("a finite range");
            let ends = Aabb::of_points(&[
                [
                    line.point(range.lo()).x,
                    line.point(range.lo()).y,
                    line.point(range.lo()).z,
                ],
                [
                    line.point(range.hi()).x,
                    line.point(range.hi()).y,
                    line.point(range.hi()).z,
                ],
            ])
            .expect("two points");
            prop_assert_eq!(bounds, ends, "a line's box is exactly its endpoints'");
            Ok(())
        },
    );
}

#[test]
fn an_unbounded_range_has_no_box() {
    let line = Curve::Line {
        origin: Point3::origin(),
        direction: UnitVec3::new_normalize(Vec3::x()),
    };
    assert_eq!(line.bounds(Interval::REAL), None);
    let plane = Surface::Plane {
        frame: Frame::world(),
    };
    assert_eq!(plane.bounds([Interval::REAL, Interval::UNIT]), None);
    assert_eq!(plane.bounds([Interval::UNIT, Interval::REAL]), None);
}

#[test]
fn every_surface_stays_inside_its_bounds() {
    check(
        (
            surface(),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
        ),
        |(s, ua, ub, va, vb)| {
            let domain = s.domain();
            let (Some(u), Some(v)) = (sub_range(domain[0], ua, ub), sub_range(domain[1], va, vb))
            else {
                return Ok(());
            };
            let Some(bounds) = s.bounds([u, v]) else {
                return Err(TestCaseError::fail(format!("{s:?}: no bounds")));
            };
            for i in 0..=SAMPLES {
                for j in 0..=SAMPLES {
                    let (a, b) = (
                        u.lo() + u.length() * i as f64 / SAMPLES as f64,
                        v.lo() + v.length() * j as f64 / SAMPLES as f64,
                    );
                    let p = s.point(a, b);
                    prop_assert!(
                        holds(&bounds, p),
                        "{s:?}: {p:?} outside {bounds:?} at ({a}, {b})"
                    );
                }
            }
            prop_assert_eq!(s.bounds([u, v]), s.bounds([u, v]), "two runs differ");
            Ok(())
        },
    );
}

#[test]
fn a_nurbs_surfaces_control_hull_holds_it() {
    check(nurbs_surface(), |n| {
        let s = Surface::Nurbs(n);
        let [u, v] = s.domain();
        let bounds = s.bounds([u, v]).expect("a finite domain");
        for i in 0..=SAMPLES {
            for j in 0..=SAMPLES {
                let (a, b) = (
                    u.lo() + u.length() * i as f64 / SAMPLES as f64,
                    v.lo() + v.length() * j as f64 / SAMPLES as f64,
                );
                prop_assert!(holds(&bounds, s.point(a, b)));
            }
        }
        Ok(())
    });
}

#[test]
fn a_plane_rectangles_bounds_are_its_four_corners() {
    check(
        (
            arris_debug::prop::frame(),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        ),
        |(frame, ua, ub, va, vb)| {
            prop_assume!((ua - ub).abs() > 1e-6 && (va - vb).abs() > 1e-6);
            let plane = Surface::Plane { frame };
            let u = Interval::new(ua.min(ub), ua.max(ub))
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            let v = Interval::new(va.min(vb), va.max(vb))
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            let bounds = plane.bounds([u, v]).expect("a finite rectangle");
            // Tight: some sampled point reaches every side of the box.
            for k in 0..3 {
                let mut lowest = f64::INFINITY;
                let mut highest = f64::NEG_INFINITY;
                for a in [u.lo(), u.hi()] {
                    for b in [v.lo(), v.hi()] {
                        let p = plane.point(a, b);
                        lowest = lowest.min(p[k]);
                        highest = highest.max(p[k]);
                    }
                }
                prop_assert!((bounds.min[k] - lowest).abs() <= EXACT);
                prop_assert!((bounds.max[k] - highest).abs() <= EXACT);
            }
            Ok(())
        },
    );
}

/// The nightly of 2026-10-01: a closed section curve whose edge starts
/// near the end of its domain and runs on into the next period, as the
/// edge of a result that was cut at a seam does. The box held only the
/// few control points beside the end of the knots, so a boolean's face
/// pair was rejected by it and a crossing of the edge was never found.
#[test]
fn a_periodic_nurbs_over_a_range_past_its_domain_is_bounded_by_the_whole_curve() {
    use arris_geom::fit_curve_periodic;
    use arris_math::Meter;
    use core::f64::consts::{PI, TAU};

    let f = |t: f64| Point3::new(2.0 * t.cos(), t.sin(), 0.5 * t.sin());
    let fit = fit_curve_periodic(
        f,
        Interval::new(0.0, TAU).unwrap(),
        5,
        |t, q| (q - f(t)).norm(),
        1e-9,
        &mut Meter::default(),
    )
    .unwrap();
    let c = Curve::Nurbs(fit);
    // From just before the end of the domain a whole turn on, from the
    // middle of it a whole turn on, and from before its start.
    for (lo, hi) in [
        (TAU - 0.02, 2.0 * TAU - 0.02),
        (PI, 3.0 * PI),
        (-1.0, TAU - 1.0),
        (TAU - 0.5, TAU + 0.5),
        (1.0, 1.0 + 3.0 * TAU),
    ] {
        let range = Interval::new(lo, hi).unwrap();
        let bounds = c.bounds(range).expect("a finite range");
        for i in 0..=SAMPLES {
            let t = lo + (hi - lo) * i as f64 / SAMPLES as f64;
            let p = f(t);
            assert!(
                (0..3).all(|k| p[k] >= bounds.min[k] - 1e-8 && p[k] <= bounds.max[k] + 1e-8),
                "{p:?} outside {bounds:?} at {t} of [{lo}, {hi}]"
            );
        }
    }
}
