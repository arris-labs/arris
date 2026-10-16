//! Every analytic surface, the line and the circle project a point to
//! their nearest point: the result is on the target, idempotent, at the
//! closed-form distance, recovers a displaced point's parameters, and the
//! ambiguous loci are reported rather than guessed.

use arris_math::real;
use core::f64::consts::{FRAC_PI_2, TAU};

use arris_debug::prop::geom::{
    RADIUS_RANGE, circle, cone, cylinder, ellipse, elliptic_cylinder, line, plane, sphere, surface,
    torus,
};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, point_in_box};
use arris_geom::{AmbiguousLocus, Curve, GeomError, GeomKind, Surface, SurfaceProjection};
use arris_math::Point3;
use proptest::prelude::*;

/// Distances, and parameter errors measured in length units.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;

/// `|a − b|` on a circle of period `period`, or plainly when there is
/// none.
fn param_diff(period: Option<f64>, a: f64, b: f64) -> f64 {
    match period {
        Some(p) => {
            let d = (a - b).rem_euclid(p);
            d.min(p - d)
        }
        None => (a - b).abs(),
    }
}

/// The parameter error between two `(u, v)` on `s`, in length units:
/// each difference scaled by the parametric speed at `expected`, so a
/// meaningless `u` at a pole costs nothing and a plane's `u` is a length.
fn param_error(s: &Surface, got: (f64, f64), expected: (f64, f64)) -> f64 {
    let e = s.eval(expected.0, expected.1);
    let [pu, pv] = s.period();
    let du = param_diff(pu, got.0, expected.0) * e.du.norm();
    let dv = param_diff(pv, got.1, expected.1) * e.dv.norm();
    du.max(dv)
}

/// The distance from `p` to the surface by its implicit form, written
/// independently of `Surface::project`: both nappes of the cone count.
fn implicit_distance(s: &Surface, p: Point3) -> f64 {
    let q = s.frame().unwrap().to_local(p);
    let rho = real::hypot(q.x, q.y);
    match s {
        Surface::Plane { .. } => q.z.abs(),
        &Surface::Cylinder { radius, .. } => (rho - radius).abs(),
        &Surface::EllipticCylinder {
            major_radius,
            minor_radius,
            ..
        } => sampled_ellipse_distance(major_radius, minor_radius, q.x, q.y),
        &Surface::Cone {
            radius, half_angle, ..
        } => {
            let (sa, ca) = real::sin_cos(half_angle);
            let main = (ca * (rho - radius) - sa * q.z).abs();
            let other = (ca * (rho + radius) + sa * q.z).abs();
            main.min(other)
        }
        &Surface::Sphere { radius, .. } => (q.coords.norm() - radius).abs(),
        &Surface::Torus {
            major_radius,
            minor_radius,
            ..
        } => (real::hypot(rho - major_radius, q.z) - minor_radius).abs(),
        Surface::Nurbs(_) => unreachable!("the analytic strategies yield no NURBS"),
    }
}

/// The distance from `(x, y)` to the ellipse `(a cos t, b sin t)` in its
/// plane, independently of the kernel's quartic: the nearest of
/// [`DISTANCE_SAMPLES`] points, then Newton on the stationarity
/// `(E − p) · E′ = 0` from it, which converges to rounding from a sample
/// within half a degree.
fn sampled_ellipse_distance(a: f64, b: f64, x: f64, y: f64) -> f64 {
    let d = |t: f64| real::hypot(a * real::cos(t) - x, b * real::sin(t) - y);
    let mut best = (0..DISTANCE_SAMPLES)
        .map(|i| TAU * i as f64 / DISTANCE_SAMPLES as f64)
        .min_by(|&s, &t| d(s).total_cmp(&d(t)))
        .unwrap_or(0.0);
    let e = b * b - a * a;
    for _ in 0..NEWTON_STEPS {
        let (st, ct) = real::sin_cos(best);
        let g = e * st * ct + a * x * st - b * y * ct;
        let dg = e * real::cos(2.0 * best) + a * x * ct + b * y * st;
        if dg == 0.0 {
            break;
        }
        let next = best - g / dg;
        if d(next) > d(best) {
            break;
        }
        best = next;
    }
    d(best)
}

/// Samples of [`sampled_ellipse_distance`]: half a degree apart.
const DISTANCE_SAMPLES: usize = 720;
/// Newton steps that polish the nearest sample to rounding.
const NEWTON_STEPS: usize = 20;

fn in_domain(s: &Surface, proj: &SurfaceProjection) -> bool {
    let [du, dv] = s.domain();
    let u_ok = du.contains(proj.uv.x) && (s.period()[0].is_none() || proj.uv.x < TAU);
    let v_ok = dv.contains(proj.uv.y) && (s.period()[1].is_none() || proj.uv.y < TAU);
    u_ok && v_ok
}

fn unwrap_projection<T>(r: Result<T, GeomError>) -> Result<T, TestCaseError> {
    r.map_err(|e| TestCaseError::fail(format!("unexpected error: {e}")))
}

/// The projection of a random point onto a random surface of `strategy`
/// is on the surface, is the evaluation at its `(u, v)` inside the
/// domain, is at the closed-form distance, and projects to itself.
fn projection_properties(strategy: impl Strategy<Value = Surface>) {
    check((strategy, point_in_box(DEFAULT_SCALE)), |(s, p)| {
        let proj = unwrap_projection(s.project(p))?;
        let (u, v) = (proj.uv.x, proj.uv.y);
        prop_assert!(in_domain(&s, &proj), "{s:?}: ({u}, {v}) outside the domain");
        prop_assert!(
            implicit_distance(&s, proj.point) <= EXACT,
            "{s:?}: projection of {p} is off the surface by {}",
            implicit_distance(&s, proj.point)
        );
        prop_assert!((s.point(u, v) - proj.point).norm() <= EXACT);
        let closed = implicit_distance(&s, p);
        prop_assert!(
            (proj.distance - closed).abs() <= EXACT,
            "{s:?}: distance {} vs closed form {closed}",
            proj.distance
        );
        prop_assert!((proj.distance - (p - proj.point).norm()).abs() <= EXACT);
        let again = unwrap_projection(s.project(proj.point))?;
        prop_assert!(again.distance <= EXACT);
        prop_assert!((again.point - proj.point).norm() <= EXACT);
        let err = param_error(&s, (again.uv.x, again.uv.y), (u, v));
        prop_assert!(err <= EXACT, "{s:?}: re-projection moved (u, v) by {err}");
        Ok(())
    });
}

#[test]
fn plane_projection_properties() {
    projection_properties(plane());
}

#[test]
fn cylinder_projection_properties() {
    projection_properties(cylinder());
}

#[test]
fn cone_projection_properties() {
    projection_properties(cone());
}

#[test]
fn sphere_projection_properties() {
    projection_properties(sphere());
}

#[test]
fn torus_projection_properties() {
    projection_properties(torus());
}

#[test]
fn elliptic_cylinder_projection_properties() {
    projection_properties(elliptic_cylinder());
}

/// How far a point at `(u, v)` may move along the normal and keep that
/// point as its nearest: half the distance to the nearest ambiguous
/// locus, scaled by `raw ∈ [−1, 1]`.
fn safe_offset(s: &Surface, v: f64, raw: f64) -> f64 {
    match s {
        Surface::Plane { .. } => raw * DEFAULT_SCALE,
        &Surface::Cylinder { radius, .. } | &Surface::Sphere { radius, .. } => raw * radius / 2.0,
        // The evolute is nearest at the major vertices, a radius of
        // curvature b²/a in.
        &Surface::EllipticCylinder {
            major_radius,
            minor_radius,
            ..
        } => raw * minor_radius * minor_radius / major_radius / 2.0,
        &Surface::Cone {
            radius, half_angle, ..
        } => {
            // Height above the plane through the apex, in units of the
            // normal's vertical component sin α.
            let (sa, ca) = real::sin_cos(half_angle);
            let above = (v * ca + radius * ca / sa) / sa;
            raw * (radius / 2.0).min(above / 2.0)
        }
        &Surface::Torus {
            major_radius,
            minor_radius,
            ..
        } => raw * (minor_radius / 2.0).min((major_radius - minor_radius) / 2.0),
        Surface::Nurbs(_) => unreachable!("the analytic strategies yield no NURBS"),
    }
}

/// A point built at `(u, v)` and displaced along the normal projects back
/// to `(u, v)`, at the displacement's distance.
fn displacement_properties(
    strategy: impl Strategy<Value = Surface>,
    v_range: core::ops::RangeInclusive<f64>,
) {
    check(
        (
            strategy,
            finite_f64(-1.0..=TAU + 1.0),
            finite_f64(v_range),
            finite_f64(-1.0..=1.0),
        ),
        |(s, u, v, raw)| {
            let n = s
                .normal(u, v)
                .ok_or_else(|| TestCaseError::fail(format!("{s:?} singular at ({u}, {v})")))?;
            let d = safe_offset(&s, v, raw);
            let base = s.point(u, v);
            let p = base + d * n.into_inner();
            let proj = unwrap_projection(s.project(p))?;
            let err = param_error(&s, (proj.uv.x, proj.uv.y), (u, v));
            prop_assert!(
                err <= EXACT,
                "{s:?}: ({u}, {v}) + {d}·n came back as {:?}, off by {err}",
                proj.uv
            );
            prop_assert!((proj.point - base).norm() <= EXACT);
            prop_assert!((proj.distance - d.abs()).abs() <= EXACT);
            Ok(())
        },
    );
}

#[test]
fn plane_recovers_a_displaced_point() {
    displacement_properties(plane(), -DEFAULT_SCALE..=DEFAULT_SCALE);
}

#[test]
fn cylinder_recovers_a_displaced_point() {
    displacement_properties(cylinder(), -DEFAULT_SCALE..=DEFAULT_SCALE);
}

#[test]
fn cone_recovers_a_displaced_point() {
    displacement_properties(cone(), 0.0..=*RADIUS_RANGE.end());
}

#[test]
fn sphere_recovers_a_displaced_point() {
    displacement_properties(sphere(), -1.4..=1.4);
}

#[test]
fn torus_recovers_a_displaced_point() {
    displacement_properties(torus(), -1.0..=TAU + 1.0);
}

#[test]
fn elliptic_cylinder_recovers_a_displaced_point() {
    displacement_properties(elliptic_cylinder(), -DEFAULT_SCALE..=DEFAULT_SCALE);
}

fn assert_ambiguous(
    r: Result<SurfaceProjection, GeomError>,
    expected: AmbiguousLocus,
) -> Result<(), TestCaseError> {
    match r {
        Err(GeomError::Ambiguous { locus, .. }) if locus == expected => Ok(()),
        other => Err(TestCaseError::fail(format!(
            "expected ambiguous on {expected}, got {other:?}"
        ))),
    }
}

/// The axis of a cylinder, cone or torus, wherever along it.
fn on_axis(s: &Surface, t: f64) -> Point3 {
    s.frame().unwrap().origin() + t * s.frame().unwrap().z().into_inner()
}

#[test]
fn the_cylinder_axis_is_ambiguous() {
    check(
        (cylinder(), finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE)),
        |(s, t)| assert_ambiguous(s.project(on_axis(&s, t)), AmbiguousLocus::Axis),
    );
}

#[test]
fn the_cone_axis_and_apex_plane_are_ambiguous() {
    check(
        (
            cone(),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(0.0..=TAU),
            finite_f64(0.1..=DEFAULT_SCALE),
        ),
        |(s, t, angle, rho)| {
            assert_ambiguous(s.project(on_axis(&s, t)), AmbiguousLocus::Axis)?;
            let Surface::Cone {
                frame,
                radius,
                half_angle,
            } = s
            else {
                unreachable!()
            };
            let apex = on_axis(&s, -radius * real::cos(half_angle) / real::sin(half_angle));
            let radial = real::cos(angle) * frame.x().into_inner()
                + real::sin(angle) * frame.y().into_inner();
            assert_ambiguous(s.project(apex + rho * radial), AmbiguousLocus::ApexPlane)
        },
    );
}

#[test]
fn the_sphere_centre_is_ambiguous_and_its_axis_projects_to_a_pole() {
    check(
        (sphere(), finite_f64(0.1..=DEFAULT_SCALE), any::<bool>()),
        |(s, t, north)| {
            assert_ambiguous(
                s.project(s.frame().unwrap().origin()),
                AmbiguousLocus::Centre,
            )?;
            let t = if north { t } else { -t };
            let proj = unwrap_projection(s.project(on_axis(&s, t)))?;
            let v = if north { FRAC_PI_2 } else { -FRAC_PI_2 };
            prop_assert_eq!(proj.uv.x, 0.0);
            prop_assert_eq!(proj.uv.y, v);
            prop_assert!((proj.point - s.point(0.0, v)).norm() <= EXACT);
            Ok(())
        },
    );
}

#[test]
fn the_torus_axis_and_centre_circle_are_ambiguous() {
    check(
        (
            torus(),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(0.0..=TAU),
        ),
        |(s, t, angle)| {
            assert_ambiguous(s.project(on_axis(&s, t)), AmbiguousLocus::Axis)?;
            let Surface::Torus {
                frame,
                major_radius,
                ..
            } = s
            else {
                unreachable!()
            };
            let radial = real::cos(angle) * frame.x().into_inner()
                + real::sin(angle) * frame.y().into_inner();
            assert_ambiguous(
                s.project(frame.origin() + major_radius * radial),
                AmbiguousLocus::CentreCircle,
            )
        },
    );
}

#[test]
fn the_plane_is_never_ambiguous_and_every_variant_reports_its_kind() {
    check((surface(), point_in_box(DEFAULT_SCALE)), |(s, p)| {
        match s.project(p) {
            Ok(_) => {}
            Err(GeomError::Ambiguous { kind, .. }) => {
                prop_assert_ne!(s.kind(), arris_geom::SurfaceKind::Plane);
                prop_assert_eq!(kind, GeomKind::Surface(s.kind()));
            }
            Err(e) => prop_assert!(false, "{e}"),
        }
        Ok(())
    });
}

#[test]
fn line_projection_is_the_foot_of_the_perpendicular() {
    check((line(), point_in_box(DEFAULT_SCALE)), |(c, p)| {
        let proj = unwrap_projection(c.project(p))?;
        let Curve::Line { direction, .. } = c else {
            unreachable!()
        };
        prop_assert!((c.point(proj.t) - proj.point).norm() <= EXACT);
        prop_assert!((p - proj.point).dot(&direction).abs() <= EXACT);
        prop_assert!((proj.distance - (p - proj.point).norm()).abs() <= EXACT);
        let again = unwrap_projection(c.project(proj.point))?;
        prop_assert!((again.t - proj.t).abs() <= EXACT && again.distance <= EXACT);
        Ok(())
    });
}

#[test]
fn circle_projection_properties() {
    check((circle(), point_in_box(DEFAULT_SCALE)), |(c, p)| {
        let proj = unwrap_projection(c.project(p))?;
        let Curve::Circle { frame, radius } = c else {
            unreachable!()
        };
        prop_assert!((0.0..TAU).contains(&proj.t));
        prop_assert!((c.point(proj.t) - proj.point).norm() <= EXACT);
        let q = frame.to_local(p);
        let closed = real::hypot(real::hypot(q.x, q.y) - radius, q.z);
        prop_assert!((proj.distance - closed).abs() <= EXACT);
        prop_assert!((proj.distance - (p - proj.point).norm()).abs() <= EXACT);
        let again = unwrap_projection(c.project(proj.point))?;
        prop_assert!(param_diff(Some(TAU), again.t, proj.t) * radius <= EXACT);
        prop_assert!(again.distance <= EXACT);
        Ok(())
    });
}

#[test]
fn circle_recovers_a_displaced_point() {
    check(
        (
            circle(),
            finite_f64(-1.0..=TAU + 1.0),
            finite_f64(-0.5..=DEFAULT_SCALE / 10.0),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        ),
        |(c, t, radial_raw, axial)| {
            let Curve::Circle { frame, radius } = c else {
                unreachable!()
            };
            // Inward by at most half the radius, outward freely.
            let radial = if radial_raw < 0.0 {
                radial_raw * radius
            } else {
                radial_raw
            };
            let base = c.point(t);
            let out = (base - frame.origin()) / radius;
            let p = base + radial * out + axial * frame.z().into_inner();
            let proj = unwrap_projection(c.project(p))?;
            prop_assert!(param_diff(Some(TAU), proj.t, t) * radius <= EXACT);
            prop_assert!((proj.point - base).norm() <= EXACT);
            prop_assert!((proj.distance - real::hypot(radial, axial)).abs() <= EXACT);
            Ok(())
        },
    );
}

#[test]
fn the_circle_axis_is_ambiguous() {
    check(
        (circle(), finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE)),
        |(c, t)| {
            let Curve::Circle { frame, .. } = c else {
                unreachable!()
            };
            match c.project(frame.origin() + t * frame.z().into_inner()) {
                Err(GeomError::Ambiguous {
                    locus: AmbiguousLocus::Axis,
                    kind,
                    ..
                }) => {
                    prop_assert_eq!(kind, GeomKind::Curve(arris_geom::CurveKind::Circle));
                    Ok(())
                }
                other => Err(TestCaseError::fail(format!("{other:?}"))),
            }
        },
    );
}

/// The residual `(p − C(t)) · C′(t)`, a length squared.
const RESIDUAL: f64 = 1e-12 * DEFAULT_SCALE * DEFAULT_SCALE;
/// Parameters sampled around an ellipse to bound its true minimum
/// distance from above.
const ELLIPSE_SAMPLES: usize = 720;

#[test]
fn ellipse_projection_properties() {
    check((ellipse(), point_in_box(DEFAULT_SCALE)), |(c, p)| {
        let proj = unwrap_projection(c.project(p))?;
        prop_assert!((0.0..TAU).contains(&proj.t));
        let e = c.eval(proj.t);
        prop_assert!((e.point - proj.point).norm() <= EXACT);
        prop_assert!((proj.distance - (p - proj.point).norm()).abs() <= EXACT);
        let residual = (p - proj.point).dot(&e.d1).abs();
        prop_assert!(residual <= RESIDUAL, "{c:?} at {p}: residual {residual}");
        // A global minimum: no sample around the ellipse is nearer.
        let sampled = (0..ELLIPSE_SAMPLES)
            .map(|i| (p - c.point(TAU * i as f64 / ELLIPSE_SAMPLES as f64)).norm())
            .fold(f64::INFINITY, f64::min);
        prop_assert!(
            proj.distance <= sampled + EXACT,
            "{c:?} at {p}: {} but a sample is at {sampled}",
            proj.distance
        );
        let again = unwrap_projection(c.project(proj.point))?;
        prop_assert!(param_diff(Some(TAU), again.t, proj.t) * e.d1.norm() <= EXACT);
        prop_assert!(again.distance <= EXACT);
        Ok(())
    });
}

#[test]
fn ellipse_recovers_a_displaced_point() {
    check(
        (
            ellipse(),
            finite_f64(-1.0..=TAU + 1.0),
            finite_f64(0.0..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        ),
        |(c, t, outward, axial)| {
            let Curve::Ellipse {
                frame,
                major_radius: a,
                minor_radius: b,
            } = c
            else {
                unreachable!()
            };
            // The in-plane normal is the gradient of (x/a)² + (y/b)²;
            // outward it never crosses the evolute, so the nearest point
            // stays at t.
            let (st, ct) = real::sin_cos(t);
            let n = (b * ct * frame.x().into_inner() + a * st * frame.y().into_inner()).normalize();
            let base = c.point(t);
            let p = base + outward * n + axial * frame.z().into_inner();
            let proj = unwrap_projection(c.project(p))?;
            let speed = c.eval(t).d1.norm();
            prop_assert!(
                param_diff(Some(TAU), proj.t, t) * speed <= EXACT,
                "{c:?}: t = {t} came back as {}",
                proj.t
            );
            prop_assert!((proj.point - base).norm() <= EXACT);
            prop_assert!((proj.distance - real::hypot(outward, axial)).abs() <= EXACT);
            Ok(())
        },
    );
}

#[test]
fn the_ellipse_centre_and_major_axis_inside_the_evolute_are_ambiguous() {
    check(
        (
            ellipse(),
            finite_f64(0.1..=0.9),
            finite_f64(1.1..=2.0),
            any::<bool>(),
        ),
        |(c, inside, beyond, flip)| {
            let Curve::Ellipse {
                frame,
                major_radius: a,
                minor_radius: b,
            } = c
            else {
                unreachable!()
            };
            // The centre, of a circle-like ellipse too.
            match c.project(frame.origin()) {
                Err(GeomError::Ambiguous {
                    locus: AmbiguousLocus::Centre,
                    ..
                }) => {}
                other => return Err(TestCaseError::fail(format!("centre gave {other:?}"))),
            }
            // The evolute meets the major axis at ±(a² − b²)/a.
            let cusp = (a * a - b * b) / a;
            if cusp < 0.1 {
                return Ok(());
            }
            let sign = if flip { -1.0 } else { 1.0 };
            let x = frame.x().into_inner();
            match c.project(frame.origin() + sign * inside * cusp * x) {
                Err(GeomError::Ambiguous {
                    locus: AmbiguousLocus::MajorAxis,
                    ..
                }) => {}
                other => {
                    return Err(TestCaseError::fail(format!(
                        "inside the evolute gave {other:?}"
                    )));
                }
            }
            // Beyond the cusp the vertex is the unique nearest point.
            let vertex_t = if flip { core::f64::consts::PI } else { 0.0 };
            let proj = unwrap_projection(c.project(frame.origin() + sign * beyond * cusp * x))?;
            prop_assert!(param_diff(Some(TAU), proj.t, vertex_t) * b <= EXACT);
            Ok(())
        },
    );
}

#[test]
fn the_error_message_names_the_locus() {
    let s = Surface::Sphere {
        frame: arris_math::Frame::world(),
        radius: 1.0,
    };
    let e = s.project(Point3::origin()).unwrap_err();
    assert_eq!(
        e.to_string(),
        "projection of {0, 0, 0} onto a sphere surface is ambiguous: the point is on the centre"
    );
}
