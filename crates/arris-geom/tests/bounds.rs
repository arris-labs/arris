//! `Curve::bounds` and `Surface::bounds` contain what they bound in any
//! pose over any finite range, are tight where the geometry is affine,
//! and refuse a range that is not finite: the cheap reject a boolean's
//! face pairs go through (ADR-0004).

use arris_debug::prop::geom::{NURBS_WEIGHT_RANGE, curve, nurbs_curve, nurbs_surface, surface};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, point_in_box};
use arris_debug::testing::EXACT;
use arris_geom::{Curve, NurbsCurve, Surface};
use arris_math::real;
use arris_math::roots::golden_section;
use arris_math::{Aabb, Frame, Interval, Point3, UnitVec3, Vec3};
use proptest::prelude::*;

/// Rounding at the scale: a box holds its own geometry to this.
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

const TIGHT_EPS: f64 = 1e-6;

/// Verify containment at every sample, then polish the best sample for
/// each side per knot span. Every polished point is also on the curve,
/// so finding one within eps of a returned side certifies its tightness.
fn check_tight_curve(n: NurbsCurve, range: Interval) -> Result<(), TestCaseError> {
    // Refine over one complete knot sequence when the range covers it
    // more than once. Check containment over the original range as well.
    let witnesses = if n.period().is_some_and(|period| range.length() >= period) {
        n.domain()
    } else {
        range
    };
    let mut ends = vec![witnesses.lo()];
    ends.extend(n.breaks_within(witnesses));
    ends.push(witnesses.hi());
    let c = Curve::Nurbs(n);
    let Some(tight) = c.tight_bounds(range, TIGHT_EPS) else {
        return Err(TestCaseError::fail(format!(
            "no tight bounds over {range:?}"
        )));
    };
    if witnesses != range {
        for t in range.samples(SAMPLES) {
            prop_assert!(holds(&tight, c.point(t)), "point outside {tight:?} at {t}");
        }
    }
    let mut reached = Aabb::of_point(c.point(range.lo()));
    for span in ends.windows(2) {
        let piece = Interval::new(span[0], span[1]).unwrap();
        let samples: Vec<_> = piece
            .samples(SAMPLES)
            .into_iter()
            .map(|t| (t, c.point(t)))
            .collect();
        for &(_, p) in &samples {
            prop_assert!(holds(&tight, p), "{p:?} outside {tight:?} over {range:?}");
            reached = reached.union(Aabb::of_point(p));
        }
        for axis in 0..3 {
            for sign in [-1.0, 1.0] {
                let best = (0..samples.len())
                    .min_by(|&a, &b| {
                        (sign * samples[a].1[axis]).total_cmp(&(sign * samples[b].1[axis]))
                    })
                    .unwrap();
                let lo = samples[best.saturating_sub(1)].0;
                let hi = samples[(best + 1).min(samples.len() - 1)].0;
                let b = golden_section(|t| sign * c.point(t)[axis], lo, hi, 32);
                for t in [b.low_probe.0, b.high_probe.0] {
                    let p = c.point(t);
                    prop_assert!(holds(&tight, p), "{p:?} outside {tight:?} at {t}");
                    reached = reached.union(Aabb::of_point(p));
                }
            }
        }
    }
    for axis in 0..3 {
        prop_assert!(
            reached.min[axis] - tight.min[axis] <= TIGHT_EPS + EXACT
                && tight.max[axis] - reached.max[axis] <= TIGHT_EPS + EXACT,
            "{tight:?} is loose on axis {axis}; curve reaches {reached:?} over {range:?}"
        );
    }
    Ok(())
}

#[test]
fn tight_bounds_of_rational_curves_hold_on_trimmed_ranges() {
    check(
        (nurbs_curve(), finite_f64(0.0..=1.0), finite_f64(0.0..=1.0)),
        |(n, a, b)| {
            let range = sub_range(n.domain(), a, b).unwrap();
            check_tight_curve(n, range)
        },
    );
}

#[test]
fn tight_bounds_of_periodic_curves_hold_across_the_seam() {
    const DEGREE: usize = 3;
    const SPANS: usize = 5;
    check(
        (
            proptest::collection::vec(point_in_box(DEFAULT_SCALE), SPANS),
            proptest::collection::vec(finite_f64(NURBS_WEIGHT_RANGE), SPANS),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
        ),
        |(mut points, mut weights, a, b)| {
            points.extend_from_within(..DEGREE);
            weights.extend_from_within(..DEGREE);
            let knots = (0..=SPANS + 2 * DEGREE).map(|i| i as f64).collect();
            let n = NurbsCurve::new(DEGREE, knots, points, weights).unwrap();
            let period = SPANS as f64;
            prop_assert_eq!(n.period(), Some(period));
            let seam = n.domain().hi();
            // Include the stored seam and a seam before the stored knots.
            for seam in [seam, seam - 3.0 * period] {
                let range = Interval::new(seam - (1.0 + a), seam + (1.0 + b)).unwrap();
                check_tight_curve(n.clone(), range)?;
            }
            check_tight_curve(n, Interval::new(seam + a, seam + a + 2.0 * period).unwrap())
        },
    );
}

#[test]
fn tight_bounds_of_lines_and_conics_equal_bounds() {
    let frame = Frame::world();
    let curves = [
        Curve::Line {
            origin: frame.origin(),
            direction: frame.x(),
        },
        Curve::Circle { frame, radius: 2.0 },
        Curve::Ellipse {
            frame,
            major_radius: 3.0,
            minor_radius: 2.0,
        },
    ];
    for c in curves {
        for range in [
            Interval::new(-0.5, 2.0).unwrap(),
            Interval::TURN,
            Interval::REAL,
        ] {
            assert_eq!(c.tight_bounds(range, TIGHT_EPS), c.bounds(range));
        }
    }
}

#[test]
fn tight_bounds_include_unclamped_domain_ends() {
    let c = Curve::Nurbs(
        NurbsCurve::new(
            2,
            (0..=5).map(f64::from).collect(),
            vec![
                Point3::origin(),
                Point3::new(1.0, 2.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
            ],
            vec![1.0; 3],
        )
        .unwrap(),
    );
    // Over its single span, x = 0.5 + u and y = 1 + 2u(1 - u).
    let tight = c.tight_bounds(c.domain(), TIGHT_EPS).unwrap();
    assert_eq!(tight.min, [0.5, 1.0, 0.0]);
    assert_eq!(tight.max, [1.5, 1.5, 0.0]);
    let at_mid = Interval::new(2.5, 2.5).unwrap();
    assert_eq!(
        c.tight_bounds(at_mid, TIGHT_EPS),
        Some(Aabb::of_point(Point3::new(1.0, 1.5, 0.0)))
    );
    assert_eq!(c.tight_bounds(Interval::UNIT, TIGHT_EPS), None);
    for eps in [0.0, -TIGHT_EPS, f64::INFINITY, f64::NAN] {
        assert_eq!(c.tight_bounds(c.domain(), eps), None);
    }
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

    let f = |t: f64| Point3::new(2.0 * real::cos(t), real::sin(t), 0.5 * real::sin(t));
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

/// The box of a NURBS over part of its domain is the clipped patch's
/// control hull, not the hull of every span the range touches: a quarter
/// of a one-span quadratic holds the quarter's own points only.
#[test]
fn a_nurbs_over_a_part_of_one_span_is_clipped_to_it() {
    use arris_geom::{NurbsCurve, NurbsSurface};
    let arch = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ],
        vec![1.0; 3],
    )
    .unwrap();
    let c = Curve::Nurbs(arch);
    let quarter = c.bounds(Interval::new(0.0, 0.25).unwrap()).unwrap();
    // The curve at 0.25 is (0.5, 0.75): the control point (1, 2) is out.
    assert!(quarter.max[1] < 1.0, "{quarter:?}");
    assert!(quarter.max[0] < 1.0, "{quarter:?}");
    let whole = c.bounds(Interval::new(0.0, 1.0).unwrap()).unwrap();
    assert_eq!(whole.max, [2.0, 2.0, 0.0]);

    // A bilinear patch over a sub-rectangle: its four corners.
    let patch = NurbsSurface::new(
        [1, 1],
        [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(4.0, 4.0, 2.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    let s = Surface::Nurbs(patch);
    let part = s
        .bounds([
            Interval::new(0.0, 0.5).unwrap(),
            Interval::new(0.25, 0.5).unwrap(),
        ])
        .unwrap();
    assert!(
        part.max[0] <= 2.0 + EXACT && part.max[1] <= 2.0 + EXACT,
        "{part:?}"
    );
    assert!(part.max[2] <= 0.5 + EXACT, "{part:?}");
}
