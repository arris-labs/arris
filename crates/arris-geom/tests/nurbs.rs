//! NURBS curves and surfaces evaluate, wrap, refine and project as *The
//! NURBS Book* says they do (`docs/DATA-MODEL.md` §NURBS), and the
//! analytic dispatches treat the `Nurbs` variants exhaustively.

use core::f64::consts::FRAC_1_SQRT_2;

use arris_debug::prop::geom::{nurbs_curve, nurbs_surface, plane};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, frame, point_in_box, pose, radius};
use arris_debug::testing::{central_differences_curve, central_differences_surface};
use arris_geom::{
    Curve, CurveKind, CurveSurfaceIntersection, GeomError, GeomKind, NurbsCurve, NurbsSurface,
    Surface, SurfaceKind, intersect_curve_surface, intersect_surfaces,
};
use arris_math::{Frame, Interval, Point3, Precision, Vec3};
use proptest::prelude::*;

/// Closed form against evaluation.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;
/// Central differences against the analytic derivatives. The shared
/// stencils (`arris_debug::testing`) are fourth order, because a
/// rational piece whose weights differ by a factor of four has higher
/// derivatives that grow like that factor's powers, and the
/// second-order stencil's truncation error at any usable step is above
/// this for them.
const DIFFERENCE: f64 = 1e-6 * DEFAULT_SCALE;

/// The full circle of `frame` and `radius` as the rational quadratic
/// B-spline of nine control points on the circumscribed square, weights
/// `1, √2/2` alternating, double interior knots at the quarters.
fn nurbs_circle(frame: &Frame, radius: f64) -> NurbsCurve {
    let (o, x, y) = (
        frame.origin(),
        frame.x().into_inner(),
        frame.y().into_inner(),
    );
    let at = |cx: f64, cy: f64| o + radius * (cx * x + cy * y);
    let points = vec![
        at(1.0, 0.0),
        at(1.0, 1.0),
        at(0.0, 1.0),
        at(-1.0, 1.0),
        at(-1.0, 0.0),
        at(-1.0, -1.0),
        at(0.0, -1.0),
        at(1.0, -1.0),
        at(1.0, 0.0),
    ];
    let w = FRAC_1_SQRT_2;
    let weights = vec![1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0];
    let knots = vec![
        0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
    ];
    NurbsCurve::new(2, knots, points, weights).unwrap()
}

#[test]
fn a_rational_quadratic_circle_is_the_analytic_circle() {
    check(
        (frame(), radius(0.1..=10.0), finite_f64(0.0..=1.0)),
        |(f, r, t)| {
            let nurbs = nurbs_circle(&f, r);
            let circle = Curve::Circle {
                frame: f,
                radius: r,
            };
            let e = nurbs.eval(t);
            // On the circle: at distance R from the centre, in its plane.
            let d = e.point - f.origin();
            prop_assert!(
                (d.norm() - r).abs() <= EXACT,
                "radius off by {}",
                d.norm() - r
            );
            prop_assert!(d.dot(&f.z()).abs() <= EXACT);
            // The analytic circle's projection of the point finds it.
            let proj = circle.project(e.point).unwrap();
            prop_assert!(proj.distance <= EXACT);
            // Traversed counter-clockwise about `Z`: the tangent is `Z × radial`.
            let expected_tangent = f.z().cross(&d).normalize();
            prop_assert!((e.d1.normalize() - expected_tangent).norm() <= 1e-9);
            // The quarters land on the axes exactly.
            for (q, (cx, cy)) in [
                (0.0, (1.0, 0.0)),
                (0.25, (0.0, 1.0)),
                (0.5, (-1.0, 0.0)),
                (0.75, (0.0, -1.0)),
                (1.0, (1.0, 0.0)),
            ] {
                let expected = f.origin() + r * (cx * f.x().into_inner() + cy * f.y().into_inner());
                prop_assert!(
                    (nurbs.eval(q).point - expected).norm() <= EXACT,
                    "quarter {q}"
                );
            }
            Ok(())
        },
    );
}

#[test]
fn knot_insertion_leaves_the_curve_unchanged() {
    check(
        (
            nurbs_curve(),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
            1..=5usize,
        ),
        |(c, where_, at, times)| {
            let domain = c.domain();
            // An interior parameter that is not already a knot (the
            // strategy's knots are integers; a random real never is one).
            let t_knot = domain.lerp(where_).clamp(domain.lo(), domain.hi() - 1e-9);
            let existing = c.knots().iter().filter(|&&k| k == t_knot).count();
            let times = times.min(c.degree() - existing);
            if times == 0 {
                return Ok(());
            }
            let refined = c.insert_knot(t_knot, times).unwrap();
            prop_assert_eq!(
                refined.control_points().len(),
                c.control_points().len() + times
            );
            prop_assert_eq!(refined.knots().len(), c.knots().len() + times);
            prop_assert_eq!(refined.domain(), domain);
            let t = domain.lerp(at);
            let (a, b) = (c.eval(t), refined.eval(t));
            prop_assert!(
                (a.point - b.point).norm() <= EXACT,
                "point moved by {}",
                (a.point - b.point).norm()
            );
            prop_assert!(
                (a.d1 - b.d1).norm() <= EXACT * 10.0,
                "d1 moved by {}",
                (a.d1 - b.d1).norm()
            );
            prop_assert!(
                (a.d2 - b.d2).norm() <= EXACT * 100.0,
                "d2 moved by {}",
                (a.d2 - b.d2).norm()
            );
            Ok(())
        },
    );
}

#[test]
fn curve_derivatives_match_central_differences() {
    check((nurbs_curve(), finite_f64(0.0..=1.0)), |(c, at)| {
        // Away from the ends and the knots, where the differences would
        // straddle a piece boundary.
        let domain = c.domain();
        let t = domain.lerp(at);
        let nearest_knot = c
            .knots()
            .iter()
            .map(|k| (k - t).abs())
            .fold(f64::MAX, f64::min);
        if nearest_knot <= 3.0 * arris_debug::testing::FD_STEP {
            return Ok(());
        }
        let (e, d) = (c.eval(t), central_differences_curve(|t| c.eval(t).point, t));
        prop_assert!(
            (e.d1 - d.d1).norm() <= DIFFERENCE,
            "d1 of {c:?} at {t}: {} vs {}",
            e.d1,
            d.d1
        );
        prop_assert!(
            (e.d2 - d.d2).norm() <= DIFFERENCE,
            "d2 of {c:?} at {t}: {} vs {}",
            e.d2,
            d.d2
        );
        Ok(())
    });
}

#[test]
fn surface_derivatives_match_central_differences() {
    check(
        (
            nurbs_surface(),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
        ),
        |(s, au, av)| {
            let [du, dv] = s.domain();
            let (u, v) = (du.lerp(au), dv.lerp(av));
            let [ku, kv] = s.knots();
            let near = |knots: &[f64], t: f64| {
                knots.iter().map(|k| (k - t).abs()).fold(f64::MAX, f64::min)
                    <= 3.0 * arris_debug::testing::FD_STEP
            };
            if near(ku, u) || near(kv, v) {
                return Ok(());
            }
            let (e, d) = (
                s.eval(u, v),
                central_differences_surface(|u, v| s.eval(u, v).point, u, v),
            );
            for (name, a, b) in [
                ("du", e.du, d.du),
                ("dv", e.dv, d.dv),
                ("duu", e.duu, d.duu),
                ("duv", e.duv, d.duv),
                ("dvv", e.dvv, d.dvv),
            ] {
                let err = (a - b).norm();
                prop_assert!(
                    err <= DIFFERENCE,
                    "{name} at ({u}, {v}): {a} vs {b}, off by {err}"
                );
            }
            // The normal, where it exists, is unit and orthogonal to both.
            if let Some(n) = s.normal(u, v) {
                prop_assert!((n.norm() - 1.0).abs() <= 1e-14);
                prop_assert!(n.dot(&e.du.normalize()).abs() <= 1e-12);
                prop_assert!(n.dot(&e.dv.normalize()).abs() <= 1e-12);
            }
            Ok(())
        },
    );
}

#[test]
fn a_bilinear_patch_evaluates_as_its_plane() {
    check(
        (
            frame(),
            radius(0.1..=DEFAULT_SCALE),
            radius(0.1..=DEFAULT_SCALE),
            finite_f64(-1.0..=2.0),
            finite_f64(-1.0..=2.0),
        ),
        |(f, a, b, u, v)| {
            let (o, x, y) = (f.origin(), f.x().into_inner(), f.y().into_inner());
            let patch = NurbsSurface::new(
                [1, 1],
                [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
                vec![o, o + b * y, o + a * x, o + a * x + b * y],
                vec![1.0; 4],
            )
            .unwrap();
            let plane = Surface::Plane { frame: f };
            let e = Surface::Nurbs(patch.clone()).eval(u, v);
            // The same plane, with `u` and `v` scaled by the patch's sides;
            // outside [0, 1] the clamped patch extrapolates its one piece.
            let p = plane.eval(a * u, b * v);
            prop_assert!(
                (e.point - p.point).norm() <= EXACT,
                "{} vs {}",
                e.point,
                p.point
            );
            prop_assert!((e.du - a * x).norm() <= EXACT && (e.dv - b * y).norm() <= EXACT);
            prop_assert!(e.duu.norm() <= EXACT && e.dvv.norm() <= EXACT && e.duv.norm() <= EXACT);
            // The sides are differences of points at the box's scale, so
            // their directions are known to rounding over the side's length.
            let n = Surface::Nurbs(patch).normal(u, v).unwrap();
            prop_assert!((n.into_inner() - f.z().into_inner()).norm() <= EXACT / a.min(b));
            Ok(())
        },
    );
}

/// A uniform periodic B-spline of `degree` through `distinct` control
/// points wrapped `degree` times, with unit knot spacing.
fn periodic(degree: usize, points: &[Point3]) -> NurbsCurve {
    let m = points.len();
    let n = m + degree;
    let knots: Vec<f64> = (0..n + degree + 1).map(|i| i as f64).collect();
    let wrapped: Vec<Point3> = (0..n).map(|i| points[i % m]).collect();
    NurbsCurve::new(degree, knots, wrapped, vec![1.0; n]).unwrap()
}

#[test]
fn a_periodic_knot_vector_wraps() {
    check(
        (
            1..=5usize,
            proptest::collection::vec(point_in_box(DEFAULT_SCALE), 3..=8),
            finite_f64(-3.0..=3.0),
            1..=3i32,
        ),
        |(degree, points, at, turns)| {
            let c = periodic(degree, &points);
            let m = points.len() as f64;
            prop_assert_eq!(c.period(), Some(m));
            prop_assert_eq!(
                c.domain(),
                Interval::new(degree as f64, degree as f64 + m).unwrap()
            );
            let t = c.domain().lerp(at);
            let (a, b) = (c.eval(t), c.eval(t + f64::from(turns) * m));
            prop_assert!(
                (a.point - b.point).norm() <= EXACT,
                "{} vs {}",
                a.point,
                b.point
            );
            prop_assert!((a.d1 - b.d1).norm() <= EXACT * 10.0);
            prop_assert!((a.d2 - b.d2).norm() <= EXACT * 100.0);
            // Through the enum too, and the junction is continuous.
            let curve = Curve::Nurbs(c.clone());
            prop_assert_eq!(curve.period(), Some(m));
            let (lo, hi) = (curve.eval(c.domain().lo()), curve.eval(c.domain().hi()));
            prop_assert!((lo.point - hi.point).norm() <= EXACT);
            if degree >= 2 {
                prop_assert!((lo.d1 - hi.d1).norm() <= EXACT * 10.0);
            }
            Ok(())
        },
    );
}

#[test]
fn projection_onto_the_nurbs_circle_agrees_with_the_analytic_circle() {
    check(
        (frame(), radius(0.1..=10.0), point_in_box(DEFAULT_SCALE)),
        |(f, r, p)| {
            let circle = Curve::Circle {
                frame: f,
                radius: r,
            };
            let Ok(analytic) = circle.project(p) else {
                // On the axis: the analytic projection is ambiguous, the
                // NURBS one is some point of the circle at the right distance.
                let proj = Curve::Nurbs(nurbs_circle(&f, r)).project(p).unwrap();
                let d = (proj.point - f.origin()).norm();
                prop_assert!((d - r).abs() <= EXACT);
                return Ok(());
            };
            let nurbs = Curve::Nurbs(nurbs_circle(&f, r));
            let proj = nurbs.project(p).unwrap();
            prop_assert!(
                (proj.point - analytic.point).norm() <= EXACT,
                "nearest points differ by {} for {p} on {circle:?}",
                (proj.point - analytic.point).norm()
            );
            prop_assert!((proj.distance - analytic.distance).abs() <= EXACT);
            prop_assert!(nurbs.domain().contains(proj.t));
            // Idempotent.
            let again = nurbs.project(proj.point).unwrap();
            prop_assert!((again.point - proj.point).norm() <= EXACT && again.distance <= EXACT);
            Ok(())
        },
    );
}

/// A piece of a curve is the curve over its range, clamped there: a
/// random clamped curve over a random range inside its domain, and a
/// periodic curve over a range of up to a period starting anywhere —
/// past its knots' end too — in the curve's own parameter throughout.
#[test]
fn a_segment_is_the_curve_over_its_range() {
    check(
        (
            nurbs_curve(),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
            frame(),
            radius(0.5..=3.0),
            finite_f64(-10.0..=10.0),
            finite_f64(0.05..=1.0),
        ),
        |(c, x, y, f, r, start, share)| {
            let domain = c.domain();
            let (lo, hi) = (domain.lerp(x.min(y)), domain.lerp(x.max(y)));
            prop_assume!(hi - lo > 1e-6 * domain.length());
            let range = Interval::new(lo, hi).unwrap();
            let piece = c.segment(range).unwrap();
            prop_assert_eq!((piece.domain().lo(), piece.domain().hi()), (lo, hi));
            prop_assert!(piece.period().is_none());
            for i in 0..=20 {
                let t = range.lerp(f64::from(i) / 20.0);
                let off = (piece.eval(t).point - c.eval(t).point).norm();
                prop_assert!(off <= EXACT, "{off} at t = {t}");
            }
            // A periodic curve: a circle fitted as a periodic quintic, over
            // a range of `share` of a period from anywhere.
            let circle = Curve::Circle {
                frame: f,
                radius: r,
            };
            let loop_ = arris_geom::fit_curve_periodic(
                |t| circle.point(t),
                Interval::new(0.0, core::f64::consts::TAU).unwrap(),
                5,
                |t, q| (q - circle.point(t)).norm(),
                1e-9,
                &mut arris_math::Meter::default(),
            )
            .unwrap();
            let period = loop_.period().unwrap();
            let range = Interval::new(start, start + share * period).unwrap();
            let piece = loop_.segment(range).unwrap();
            prop_assert_eq!(
                (piece.domain().lo(), piece.domain().hi()),
                (range.lo(), range.hi())
            );
            for i in 0..=20 {
                let t = range.lerp(f64::from(i) / 20.0);
                let off = (piece.eval(t).point - loop_.eval(t).point).norm();
                prop_assert!(off <= EXACT, "{off} at t = {t}");
            }
            prop_assert!(
                loop_
                    .segment(Interval::new(0.0, 1.5 * period).unwrap())
                    .is_err()
            );
            Ok(())
        },
    );
}

/// A polyline's corner is a kink of the distance's derivative: a point
/// on a leg, beside the corner and nearest a sample on the other side
/// of it, projects onto itself — the bracket that ends on the knot is
/// read on its own leg's piece, where the sign change is.
#[test]
fn a_point_beside_a_polylines_corner_projects_onto_its_own_leg() {
    let corner = |t: f64| {
        let legs = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 2.0, 2.0],
            vec![
                Point3::origin(),
                Point3::new(0.0, 0.0, -87.5),
                Point3::new(0.0, 87.8, 0.0),
            ],
            vec![0.5; 3],
        )
        .unwrap();
        let p = legs.eval(t).point;
        let back = legs.project_parameter(p);
        (back, (legs.eval(back).point - p).norm())
    };
    for t in [0.87, 0.95, 1.05, 1.13] {
        let (back, off) = corner(t);
        assert!(
            (back - t).abs() <= 1e-12 && off <= EXACT,
            "{t} projects to {back}, {off} away"
        );
    }
}

#[test]
fn a_moved_nurbs_curve_is_the_curve_moved() {
    check(
        (nurbs_curve(), pose(), finite_f64(0.0..=1.0)),
        |(c, m, at)| {
            let t = c.domain().lerp(at);
            let moved = Curve::Nurbs(c.clone()).transformed(&m);
            let (a, b) = (moved.eval(t), c.eval(t));
            prop_assert!((a.point - m.apply(b.point)).norm() <= EXACT);
            prop_assert!((a.d1 - m.apply_vec(b.d1)).norm() <= EXACT * 10.0);
            prop_assert_eq!(moved.kind(), CurveKind::Nurbs);
            prop_assert_eq!(moved.domain(), c.domain());
            Ok(())
        },
    );
}

#[test]
fn every_cycle_one_query_on_a_nurbs_is_unsupported_by_name() {
    check((nurbs_curve(), nurbs_surface(), plane()), |(c, s, p)| {
        let tol = Precision::DEFAULT.tolerance();
        let (curve, surface) = (Curve::Nurbs(c), Surface::Nurbs(s));
        prop_assert_eq!(surface.kind(), SurfaceKind::Nurbs);
        prop_assert!(surface.frame().is_none());
        prop_assert_eq!(
            intersect_surfaces(
                &surface,
                &p,
                &within(),
                tol,
                &mut arris_math::Meter::default()
            ),
            Err(GeomError::Unsupported {
                a: GeomKind::Surface(SurfaceKind::Nurbs),
                b: GeomKind::Surface(SurfaceKind::Plane),
            })
        );
        prop_assert_eq!(
            intersect_surfaces(
                &p,
                &surface,
                &within(),
                tol,
                &mut arris_math::Meter::default()
            )
            .unwrap_err(),
            GeomError::Unsupported {
                a: GeomKind::Surface(SurfaceKind::Plane),
                b: GeomKind::Surface(SurfaceKind::Nurbs),
            }
        );
        // A NURBS curve meets every analytic surface (ADR-0018); it is
        // the NURBS surface that no curve meets yet.
        prop_assert_eq!(
            intersect_curve_surface(&curve, &surface, tol, &mut arris_math::Meter::default()),
            Err(GeomError::Unsupported {
                a: GeomKind::Curve(CurveKind::Nurbs),
                b: GeomKind::Surface(SurfaceKind::Nurbs),
            })
        );
        let curve_on_plane =
            intersect_curve_surface(&curve, &p, tol, &mut arris_math::Meter::default());
        prop_assert!(curve_on_plane.is_ok(), "{curve_on_plane:?}");
        let line = Curve::Line {
            origin: Point3::origin(),
            direction: Vec3::z_axis(),
        };
        let line_on_nurbs =
            intersect_curve_surface(&line, &surface, tol, &mut arris_math::Meter::default());
        let unsupported = matches!(line_on_nurbs, Err(GeomError::Unsupported { .. }));
        prop_assert!(unsupported, "{line_on_nurbs:?}");
        let line_on_plane =
            intersect_curve_surface(&line, &p, tol, &mut arris_math::Meter::default()).unwrap();
        let has_points = matches!(line_on_plane, CurveSurfaceIntersection::Points(_));
        prop_assert!(has_points, "{line_on_plane:?}");
        // A projection is a search since ADR-0025: answered, or a tie
        // named, never `Unsupported`.
        let projected = surface.project(Point3::origin());
        let answered = matches!(projected, Ok(_) | Err(GeomError::Ambiguous { .. }));
        prop_assert!(answered, "{projected:?}");
        // The NURBS surface's domain and period come through the enum.
        let [du, dv] = surface.domain();
        prop_assert!(du.is_bounded() && dv.is_bounded());
        prop_assert_eq!(surface.period(), [None, None]);
        prop_assert!(curve.domain().length() >= 1.0);
        Ok(())
    });
}

/// A region every traced section of these tests lies in; the closed
/// forms ignore it.
fn within() -> arris_math::Aabb {
    arris_math::Aabb {
        min: [-100.0; 3],
        max: [100.0; 3],
    }
}
