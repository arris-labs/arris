//! Curve–curve intersection follows the case table in any pose: every hit
//! lies on both curves at the curves' own parameters, two lines meet at
//! the closed form or not at all, a line and a coplanar conic meet where
//! the 2D closed form says, two coplanar circles meet on the radical
//! line, a coplanar pair with an ellipse among them meets where the
//! conics' quartic says, a curve against itself is `Coincident`, and the
//! one pair without a form — two NURBS curves — is `Unsupported` naming
//! both operands (ADR-0004).

use core::f64::consts::{PI, TAU};

use arris_debug::prop::geom::{circle, curve, ellipse, line, nurbs_curve};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, point_in_box, unit_vec3};
use arris_geom::{
    Curve, CurveCurveHit, CurveIntersection, GeomError, GeomKind, curves_coincide, intersect_curves,
};
use arris_math::{Frame, Point3, Precision, Tolerance, Vec3};
use proptest::prelude::*;

/// Hit points against both operands and against the closed forms.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;

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

/// The checks every result passes whatever its case: each parameter in
/// its curve's domain, each hit on both curves, sorted by `ta`, and the
/// same answer on a second run.
fn common_properties(a: &Curve, b: &Curve) -> Result<CurveIntersection, TestCaseError> {
    let r = intersect_curves(a, b, tol(), &mut arris_math::Meter::default())
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
    for h in hits_of(&r) {
        for (c, t) in [(a, h.ta), (b, h.tb)] {
            prop_assert!(c.domain().contains(t), "{c:?}: t = {t} outside the domain");
            if c.period().is_some() {
                prop_assert!((0.0..TAU).contains(&t), "{c:?}: t = {t} not in [0, 2π)");
            }
        }
        prop_assert!(
            (a.point(h.ta) - h.point).norm() <= EXACT,
            "{a:?}: the hit is not the first curve's point at ta"
        );
        let off = (b.point(h.tb) - h.point).norm();
        prop_assert!(
            off <= tol().linear,
            "{a:?} vs {b:?}: the hit is {off} off the second curve"
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

// --- line–line ------------------------------------------------------------

/// Two lines through one point, in random directions.
fn crossing_lines() -> impl Strategy<Value = (Curve, Curve, Point3)> {
    (point_in_box(DEFAULT_SCALE), unit_vec3(), unit_vec3()).prop_filter_map(
        "two lines that are not parallel",
        |(meet, da, db)| {
            if da.cross(&db).norm().atan2(da.dot(&db).abs()) <= 1e-3 {
                return None;
            }
            Some((
                Curve::Line {
                    origin: meet - 2.0 * da.into_inner(),
                    direction: da,
                },
                Curve::Line {
                    origin: meet + 3.0 * db.into_inner(),
                    direction: db,
                },
                meet,
            ))
        },
    )
}

#[test]
fn two_lines_through_one_point_meet_there_at_the_arc_lengths() {
    check(crossing_lines(), |(a, b, meet)| {
        let r = common_properties(&a, &b)?;
        let hits = hits_of(&r);
        prop_assert_eq!(hits.len(), 1, "{:?}", r);
        prop_assert!((hits[0].point - meet).norm() <= EXACT);
        // A line's parameter is arc length from its origin.
        prop_assert!((hits[0].ta - 2.0).abs() <= EXACT);
        prop_assert!((hits[0].tb + 3.0).abs() <= EXACT);
        prop_assert!(!hits[0].tangent);
        Ok(())
    });
}

#[test]
fn skew_lines_never_meet() {
    check(
        (
            crossing_lines(),
            finite_f64(1e-3..=DEFAULT_SCALE),
            unit_vec3(),
        ),
        |((a, b, _), lift, away)| {
            let Curve::Line { origin, direction } = b else {
                return fail("not a line".to_string());
            };
            // Moved off the meeting point along something the second line
            // does not run along.
            let across = away.into_inner() - away.dot(&direction) * direction.into_inner();
            prop_assume!(across.norm() > 0.5);
            let moved = Curve::Line {
                origin: origin + lift * across.normalize(),
                direction,
            };
            let r = common_properties(&a, &moved)?;
            // Skew unless the shift happened to land the second line on
            // the first again, which a general direction does not.
            if lift > tol().linear {
                prop_assert_eq!(hits_of(&r).len(), 0, "{:?}", r);
            }
            Ok(())
        },
    );
}

#[test]
fn a_line_against_itself_is_coincident_and_a_parallel_one_is_empty() {
    check(
        (line(), finite_f64(0.1..=DEFAULT_SCALE), unit_vec3()),
        |(a, gap, away)| {
            let Curve::Line { origin, direction } = a else {
                return fail("not a line".to_string());
            };
            prop_assert_eq!(
                intersect_curves(&a, &a, tol(), &mut arris_math::Meter::default())
                    .map_err(|e| TestCaseError::fail(e.to_string()))?,
                CurveIntersection::Coincident
            );
            // The same line walked from another point, and the other way.
            let slid = Curve::Line {
                origin: origin + 3.0 * direction.into_inner(),
                direction: -direction,
            };
            prop_assert_eq!(
                intersect_curves(&a, &slid, tol(), &mut arris_math::Meter::default())
                    .map_err(|e| TestCaseError::fail(e.to_string()))?,
                CurveIntersection::Coincident
            );
            let across = away.into_inner() - away.dot(&direction) * direction.into_inner();
            prop_assume!(across.norm() > 0.5);
            let apart = Curve::Line {
                origin: origin + gap * across.normalize(),
                direction,
            };
            prop_assert_eq!(
                intersect_curves(&a, &apart, tol(), &mut arris_math::Meter::default())
                    .map_err(|e| TestCaseError::fail(e.to_string()))?,
                CurveIntersection::Points(Vec::new())
            );
            Ok(())
        },
    );
}

// --- a line and a coplanar conic ------------------------------------------

/// A circle, and a line in its plane whose distance from the centre is
/// `offset`: two hits below the radius, one touch at it, none above.
fn line_across_a_circle() -> impl Strategy<Value = (Curve, Curve, f64, f64)> {
    (circle(), finite_f64(0.0..=2.5), finite_f64(-3.0..=3.0)).prop_filter_map(
        "a line in the circle's plane",
        |(c, factor, slide)| {
            let (frame, radius) = match c {
                Curve::Circle { frame, radius } => (frame, radius),
                _ => return None,
            };
            let offset = factor * radius;
            let origin =
                frame.origin() + offset * frame.x().into_inner() + slide * frame.y().into_inner();
            Some((
                c,
                Curve::Line {
                    origin,
                    direction: frame.y(),
                },
                offset,
                radius,
            ))
        },
    )
}

#[test]
fn a_line_in_a_circles_plane_meets_it_at_the_2d_closed_form() {
    check(line_across_a_circle(), |(c, l, offset, radius)| {
        let r = common_properties(&l, &c)?;
        let hits = hits_of(&r);
        if (offset - radius).abs() <= tol().linear {
            prop_assert_eq!(hits.len(), 1, "a touch at the radius: {:?}", r);
            prop_assert!(hits[0].tangent);
            return Ok(());
        }
        let expected = usize::from(offset < radius) * 2;
        prop_assert_eq!(
            hits.len(),
            expected,
            "offset {} r {}: {:?}",
            offset,
            radius,
            r
        );
        // The half-chord of the closed form, either side of the line's
        // own origin.
        let half = (radius - offset).max(0.0).sqrt() * (radius + offset).sqrt();
        for h in hits {
            prop_assert!(!h.tangent);
            let Curve::Line { origin, .. } = l else {
                return fail("not a line".to_string());
            };
            prop_assert!(
                ((h.point - origin).norm_squared() - h.ta * h.ta).abs() <= EXACT,
                "the hit is not on the line at ta"
            );
            let Curve::Circle { frame, .. } = c else {
                return fail("not a circle".to_string());
            };
            let local = frame.to_local(h.point);
            prop_assert!((local.x - offset).abs() <= EXACT, "off the chord");
            prop_assert!((local.y.abs() - half).abs() <= EXACT, "not the half-chord");
        }
        Ok(())
    });
}

// --- two coplanar circles -------------------------------------------------

/// Two circles in one plane, the second's centre `apart` from the first's.
fn coplanar_circles() -> impl Strategy<Value = (Curve, Curve, f64, f64, f64)> {
    (circle(), finite_f64(0.05..=3.0), finite_f64(0.0..=3.0)).prop_filter_map(
        "a second circle in the plane",
        |(a, factor, gap)| {
            let (frame, ra) = match a {
                Curve::Circle { frame, radius } => (frame, radius),
                _ => return None,
            };
            let rb = factor * ra;
            let apart = gap * (ra + rb);
            let b = Curve::Circle {
                frame: frame.with_origin(frame.origin() + apart * frame.x().into_inner()),
                radius: rb,
            };
            Some((a, b, apart, ra, rb))
        },
    )
}

#[test]
fn two_coplanar_circles_meet_on_the_radical_line() {
    check(coplanar_circles(), |(a, b, apart, ra, rb)| {
        let r = common_properties(&a, &b)?;
        let (sum, difference) = (ra + rb, (ra - rb).abs());
        if apart <= tol().linear {
            let expected = if (ra - rb).abs() <= tol().linear {
                CurveIntersection::Coincident
            } else {
                CurveIntersection::Points(Vec::new())
            };
            prop_assert_eq!(r, expected);
            return Ok(());
        }
        let hits = hits_of(&r);
        if (apart - sum).abs() <= tol().linear || (apart - difference).abs() <= tol().linear {
            prop_assert_eq!(hits.len(), 1, "a touch: {:?}", r);
            prop_assert!(hits[0].tangent);
        } else if apart > sum || apart < difference {
            prop_assert_eq!(hits.len(), 0, "apart {}: {:?}", apart, r);
        } else {
            prop_assert_eq!(hits.len(), 2, "apart {}: {:?}", apart, r);
            prop_assert!(hits.iter().all(|h| !h.tangent));
        }
        // Whatever the case, every hit is at both radii from both centres.
        for h in hits {
            for (c, radius) in [(&a, ra), (&b, rb)] {
                let Curve::Circle { frame, .. } = c else {
                    return fail("not a circle".to_string());
                };
                prop_assert!(
                    ((h.point - frame.origin()).norm() - radius).abs() <= EXACT,
                    "off a circle by {}",
                    (h.point - frame.origin()).norm() - radius
                );
            }
        }
        Ok(())
    });
}

#[test]
fn a_circle_against_itself_is_coincident() {
    check(circle(), |c| {
        prop_assert_eq!(
            intersect_curves(&c, &c, tol(), &mut arris_math::Meter::default())
                .map_err(|e| TestCaseError::fail(e.to_string()))?,
            CurveIntersection::Coincident
        );
        Ok(())
    });
}

// --- through the conic's plane --------------------------------------------

/// A conic, and a line through one of its points in a random direction:
/// whatever the pose, the point is a hit.
fn line_through_a_conic() -> impl Strategy<Value = (Curve, Curve, f64)> {
    (
        prop_oneof![circle(), ellipse()],
        finite_f64(0.0..=TAU),
        unit_vec3(),
    )
        .prop_map(|(c, t, direction)| {
            let on = c.point(t);
            (
                Curve::Line {
                    origin: on - 1.5 * direction.into_inner(),
                    direction,
                },
                c,
                t,
            )
        })
}

#[test]
fn a_line_through_a_conics_point_hits_it_there() {
    check(line_through_a_conic(), |(l, c, t)| {
        let r = common_properties(&l, &c)?;
        let hits = hits_of(&r);
        let on = c.point(t);
        prop_assert!(
            hits.iter().any(|h| (h.point - on).norm() <= tol().linear),
            "the built point is not among {hits:?}"
        );
        // And with the operands the other way round, with the parameters
        // swapped and the hits still ordered by the first curve's.
        let swapped = common_properties(&c, &l)?;
        prop_assert_eq!(hits_of(&swapped).len(), hits.len());
        prop_assert!(
            hits_of(&swapped)
                .iter()
                .any(|h| (h.point - on).norm() <= tol().linear)
        );
        Ok(())
    });
}

/// Two circles whose planes cross, sharing one point of the first.
#[test]
fn two_circles_in_crossing_planes_meet_where_they_were_built_to() {
    check(
        (
            circle(),
            finite_f64(0.0..=TAU),
            finite_f64(0.3..=1.2),
            unit_vec3(),
        ),
        |(a, t, tilt, hint)| {
            let Curve::Circle { frame, radius } = a else {
                return fail("not a circle".to_string());
            };
            let on = a.point(t);
            // A second circle of the same radius through `on`, in a plane
            // tilted about the tangent there.
            let radial = (on - frame.origin()).normalize();
            let tangent = frame.z().cross(&radial);
            let normal = tilt.cos() * frame.z().into_inner() + tilt.sin() * tangent;
            prop_assume!(hint.cross(&normal).norm() > 0.1);
            let b_frame = Frame::from_z(on + radius * radial, normal)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            let b = Curve::Circle {
                frame: b_frame,
                radius,
            };
            let r = common_properties(&a, &b)?;
            prop_assert!(
                hits_of(&r)
                    .iter()
                    .any(|h| (h.point - on).norm() <= 1e-9 * DEFAULT_SCALE),
                "the shared point is not among {:?}",
                r
            );
            Ok(())
        },
    );
}

// --- two coplanar conics with an ellipse among them ------------------------

/// An ellipse, and a conic in its plane built to cross it at one of its
/// points: whatever the pose, that point is a hit of the quartic.
/// `second` is a circle for half the cases and an ellipse for the other
/// half. A pair built *tangent* there is dropped — a touch is located
/// only to the square root of the tolerance over the two curvatures'
/// difference, so the point they were built to share says nothing about
/// where the touch is reported; that case is
/// `the_same_coplanar_ellipse_is_coincident_and_its_major_circle_touches`'s
/// and `geom/c3-conic-hits`'.
fn coplanar_conic_through_an_ellipse() -> impl Strategy<Value = (Curve, Curve, Point3)> {
    (
        ellipse(),
        finite_f64(0.0..=TAU),
        finite_f64(0.2..=1.5),
        finite_f64(0.1..=1.0),
        finite_f64(0.0..=TAU),
        any::<bool>(),
    )
        .prop_filter_map(
            "a conic crossing the ellipse's plane at a point of it",
            |(e, t, scale, squash, phase, round)| {
                let Curve::Ellipse {
                    frame,
                    major_radius,
                    minor_radius,
                } = e
                else {
                    return None;
                };
                let on = e.point(t);
                let (x, y) = (frame.x().into_inner(), frame.y().into_inner());
                let a = scale * DEFAULT_SCALE * 0.25;
                let b = if round { a } else { squash * a };
                // The centre that puts `on` at the second conic's own
                // angle `phase`.
                let centre = on - (a * phase.cos()) * x - (b * phase.sin()) * y;
                // Both tangents there, in the plane's own axes: the two
                // must cross, not graze.
                let first_d = -(major_radius * t.sin()) * x + (minor_radius * t.cos()) * y;
                let second_d = -(a * phase.sin()) * x + (b * phase.cos()) * y;
                let angle = first_d
                    .cross(&second_d)
                    .norm()
                    .atan2(first_d.dot(&second_d).abs());
                if angle <= 1e-2 {
                    return None;
                }
                let placed = frame.with_origin(centre);
                let second = if round {
                    Curve::Circle {
                        frame: placed,
                        radius: a,
                    }
                } else {
                    Curve::Ellipse {
                        frame: placed,
                        major_radius: a,
                        minor_radius: b,
                    }
                };
                Some((e, second, on))
            },
        )
}

/// A coplanar pair with an ellipse among them meets where it was built
/// to: the quartic of `conic2` finds the shared point, whichever way
/// round the pair goes — or the two are one conic, and every point is
/// shared (docs/DATA-MODEL.md §Curves).
#[test]
fn a_coplanar_pair_with_an_ellipse_meets_where_it_was_built_to() {
    check(coplanar_conic_through_an_ellipse(), |(a, b, on)| {
        for (first, second) in [(&a, &b), (&b, &a)] {
            let r = common_properties(first, second)?;
            if r == CurveIntersection::Coincident {
                continue;
            }
            prop_assert!(
                hits_of(&r)
                    .iter()
                    .any(|h| (h.point - on).norm() <= tol().linear),
                "{first:?} vs {second:?}: the built point {on:?} is not among {:?}",
                hits_of(&r)
            );
        }
        Ok(())
    });
}

/// The same ellipse twice — in its own frame, in the frame turned half
/// a turn about its normal, and with the two radii and axes swapped —
/// is `Coincident`; a circle of its major radius about its centre
/// touches it at the two major vertices, where the ellipse reaches the
/// circle from inside without crossing it.
#[test]
fn the_same_coplanar_ellipse_is_coincident_and_its_major_circle_touches() {
    check(ellipse(), |e| {
        let Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } = e
        else {
            return fail("not an ellipse".to_string());
        };
        let turned = Frame::new(
            frame.origin(),
            frame.z().into_inner(),
            -frame.x().into_inner(),
        )
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
        let swapped = Frame::new(
            frame.origin(),
            frame.z().into_inner(),
            frame.y().into_inner(),
        )
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
        let twins = [
            e.clone(),
            Curve::Ellipse {
                frame: turned,
                major_radius,
                minor_radius,
            },
            Curve::Ellipse {
                frame: swapped,
                major_radius: minor_radius,
                minor_radius: major_radius,
            },
        ];
        for twin in &twins {
            prop_assert_eq!(
                intersect_curves(&e, twin, tol(), &mut arris_math::Meter::default())
                    .map_err(|e| TestCaseError::fail(e.to_string()))?,
                CurveIntersection::Coincident,
                "{:?} vs {:?}",
                e,
                twin
            );
        }
        let circle = Curve::Circle {
            frame,
            radius: major_radius,
        };
        let r = common_properties(&e, &circle)?;
        let flatness = major_radius - minor_radius;
        if flatness <= tol().linear {
            // Round within the tolerance: the circle *is* the ellipse.
            prop_assert_eq!(r, CurveIntersection::Coincident, "{:?}", e);
        } else if flatness > 2.0 * tol().linear {
            // Clear of round: the two major vertices, each a touch.
            let hits = hits_of(&r);
            prop_assert_eq!(hits.len(), 2, "{:?} vs its major circle: {:?}", e, r);
            prop_assert!(hits.iter().all(|h| h.tangent), "{:?}", hits);
            // At the two vertices, modulo the turn: `t = 0` is the
            // seam, and a root the polish nudges a rounding below it
            // comes back at the far end of the domain, the same point.
            let turn_from = |t: f64, from: f64| {
                let apart = (t - from).rem_euclid(TAU);
                apart.min(TAU - apart)
            };
            for h in hits {
                prop_assert!(
                    turn_from(h.ta, 0.0).min(turn_from(h.ta, PI)) <= tol().linear,
                    "{:?} is at neither vertex",
                    hits
                );
            }
            prop_assert!(
                (turn_from(hits[0].ta, hits[1].ta) - PI).abs() <= tol().linear,
                "{:?} are not half a turn apart",
                hits
            );
        }
        // Between one tolerance and two the two verdicts meet, and
        // either is honest: the ellipse is within the tolerance of the
        // circle over an arc that reaches the minor vertices.
        Ok(())
    });
}

// --- what has no closed form ----------------------------------------------

/// Two NURBS curves are the only arm left without a form: every pair
/// with an analytic curve in it answers, and a NURBS pair is
/// `Unsupported` naming both operands.
#[test]
fn only_two_nurbs_curves_are_unsupported() {
    check((curve(), nurbs_curve(), nurbs_curve()), |(c, m, n)| {
        let (m, n) = (Curve::Nurbs(m), Curve::Nurbs(n));
        for (a, b) in [(&c, &m), (&m, &c), (&c, &c)] {
            match intersect_curves(a, b, tol(), &mut arris_math::Meter::default()) {
                Ok(_) => {}
                other => return fail(format!("{a:?} vs {b:?}: {other:?}")),
            }
        }
        for (a, b) in [(&m, &n), (&n, &m), (&n, &n)] {
            match intersect_curves(a, b, tol(), &mut arris_math::Meter::default()) {
                Err(GeomError::Unsupported { a: ka, b: kb }) => {
                    prop_assert_eq!(ka, GeomKind::Curve(a.kind()));
                    prop_assert_eq!(kb, GeomKind::Curve(b.kind()));
                }
                other => return fail(format!("{a:?} vs {b:?}: {other:?}")),
            }
        }
        Ok(())
    });
}

#[test]
fn an_inconsistent_tolerance_is_an_error() {
    let a = Curve::Line {
        origin: Point3::origin(),
        direction: Vec3::x_axis(),
    };
    assert!(matches!(
        intersect_curves(
            &a,
            &a,
            Tolerance::new(1e-7, 0.0),
            &mut arris_math::Meter::default()
        ),
        Err(GeomError::InvalidTolerance(_))
    ));
}

#[test]
fn every_analytic_pair_passes_the_common_properties() {
    check((curve(), curve()), |(a, b)| {
        common_properties(&a, &b)?;
        Ok(())
    });
}

/// `curves_coincide` is `intersect_curves`' `Coincident` verdict, pair
/// for pair: over random analytic pairs, and over the coplanar pairs a
/// random pair almost never is — an ellipse's twins, a circle in its
/// plane and a second ellipse in it (docs/DATA-MODEL.md §Curves). Two
/// NURBS curves, which `intersect_curves` refuses, are the same curve,
/// apart, or `Unsupported` naming the pair (`intersect_spline_curves.rs`).
#[test]
fn curves_coincide_is_the_coincident_verdict() {
    check((curve(), curve()), |(a, b)| {
        let found = intersect_curves(&a, &b, tol(), &mut arris_math::Meter::default())
            .map_err(|e| TestCaseError::fail(e.to_string()))?;
        prop_assert_eq!(
            curves_coincide(&a, &b, tol(), &mut arris_math::Meter::default())
                .map_err(|e| TestCaseError::fail(e.to_string()))?,
            found == CurveIntersection::Coincident,
            "{:?} vs {:?}",
            a,
            b
        );
        Ok(())
    });
    check((ellipse(), finite_f64(0.1..=2.0)), |(e, factor)| {
        let Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } = e
        else {
            return fail("not an ellipse".to_string());
        };
        let swapped = Frame::new(
            frame.origin(),
            frame.z().into_inner(),
            frame.y().into_inner(),
        )
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
        let twin = Curve::Ellipse {
            frame: swapped,
            major_radius: minor_radius,
            minor_radius: major_radius,
        };
        let circle = Curve::Circle {
            frame,
            radius: factor * major_radius,
        };
        let other = Curve::Ellipse {
            frame,
            major_radius: factor * major_radius + minor_radius,
            minor_radius,
        };
        let same = |x: &Curve, y: &Curve| {
            curves_coincide(x, y, tol(), &mut arris_math::Meter::default())
                .map_err(|e| TestCaseError::fail(e.to_string()))
        };
        prop_assert!(same(&e, &twin)?);
        for (x, y) in [(&e, &circle), (&circle, &e), (&e, &other)] {
            let found = intersect_curves(x, y, tol(), &mut arris_math::Meter::default())
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(
                same(x, y)?,
                found == CurveIntersection::Coincident,
                "{:?} vs {:?}: {:?}",
                x,
                y,
                found
            );
        }
        Ok(())
    });
}
