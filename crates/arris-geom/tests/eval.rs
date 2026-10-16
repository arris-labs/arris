//! Every analytic surface and curve evaluates as its closed form in any
//! pose, its derivatives are the derivatives, its normal is the normal,
//! its periods are periods and its singular loci are reported
//! (`docs/DATA-MODEL.md` §Geometry).

use arris_math::real;
use core::f64::consts::{FRAC_PI_2, TAU};

use arris_debug::prop::geom::{cone, curve, sphere, surface};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, frame, unit_vec3};
use arris_debug::testing::{central_differences_curve, central_differences_surface};
use arris_geom::{Curve, Surface};
use arris_math::{Frame, Point3, Vec3};
use proptest::prelude::*;

/// Closed form against evaluation, and evaluation against itself one
/// period later.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;
/// Central differences against the analytic derivatives.
const DIFFERENCE: f64 = 1e-6 * DEFAULT_SCALE;
/// A unit vector's length and two orthogonal unit vectors' dot product.
const UNIT: f64 = 1e-14;

/// Parameters for a surface: `u` over a turn plus a bit, `v` over the
/// sphere's latitude range, which is also a sensible range for the others.
fn params() -> impl Strategy<Value = (f64, f64)> {
    (
        finite_f64(-1.0..=TAU + 1.0),
        finite_f64(-FRAC_PI_2..=FRAC_PI_2),
    )
}

/// The closed forms of `docs/DATA-MODEL.md` §Surfaces written out in
/// the world frame, independently of `Surface::eval`.
fn local_surface(s: &Surface, u: f64, v: f64) -> Point3 {
    let (su, cu) = real::sin_cos(u);
    let (sv, cv) = real::sin_cos(v);
    match s {
        Surface::Plane { .. } => Point3::new(u, v, 0.0),
        &Surface::Cylinder { radius: r, .. } => Point3::new(r * cu, r * su, v),
        &Surface::EllipticCylinder {
            major_radius: a,
            minor_radius: b,
            ..
        } => Point3::new(a * cu, b * su, v),
        &Surface::Cone {
            radius: r,
            half_angle: a,
            ..
        } => {
            let rho = r + v * real::sin(a);
            Point3::new(rho * cu, rho * su, v * real::cos(a))
        }
        &Surface::Sphere { radius: r, .. } => Point3::new(r * cv * cu, r * cv * su, r * sv),
        &Surface::Torus {
            major_radius: big,
            minor_radius: small,
            ..
        } => {
            let rho = big + small * cv;
            Point3::new(rho * cu, rho * su, small * sv)
        }
        Surface::Nurbs(_) => unreachable!("the analytic strategies yield no NURBS"),
    }
}

fn local_curve(c: &Curve, t: f64) -> Point3 {
    let (st, ct) = real::sin_cos(t);
    match c {
        Curve::Line { .. } => Point3::new(0.0, 0.0, t),
        &Curve::Circle { radius: r, .. } => Point3::new(r * ct, r * st, 0.0),
        &Curve::Ellipse {
            major_radius: a,
            minor_radius: b,
            ..
        } => Point3::new(a * ct, b * st, 0.0),
        Curve::Nurbs(_) => unreachable!("the analytic strategies yield no NURBS"),
    }
}

fn with_frame(s: &Surface, frame: Frame) -> Surface {
    match s {
        Surface::Plane { .. } => Surface::Plane { frame },
        &Surface::Cylinder { radius, .. } => Surface::Cylinder { frame, radius },
        &Surface::EllipticCylinder {
            major_radius,
            minor_radius,
            ..
        } => Surface::EllipticCylinder {
            frame,
            major_radius,
            minor_radius,
        },
        &Surface::Cone {
            radius, half_angle, ..
        } => Surface::Cone {
            frame,
            radius,
            half_angle,
        },
        &Surface::Sphere { radius, .. } => Surface::Sphere { frame, radius },
        &Surface::Torus {
            major_radius,
            minor_radius,
            ..
        } => Surface::Torus {
            frame,
            major_radius,
            minor_radius,
        },
        Surface::Nurbs(_) => unreachable!("the analytic strategies yield no NURBS"),
    }
}

/// The frame a curve is posed by: a line's is the frame whose `z` is its
/// direction through its origin.
fn curve_frame(c: &Curve) -> Frame {
    match c {
        &Curve::Line { origin, direction } => {
            Frame::from_z(origin, direction.into_inner()).unwrap()
        }
        &Curve::Circle { frame, .. } | &Curve::Ellipse { frame, .. } => frame,
        Curve::Nurbs(_) => unreachable!("the analytic strategies yield no NURBS"),
    }
}

#[test]
fn posed_surface_equals_the_local_closed_form_moved_by_the_pose() {
    check((surface(), params()), |(s, (u, v))| {
        let pose = s.frame().unwrap().as_isometry();
        let expected = pose.apply(local_surface(&s, u, v));
        let got = s.eval(u, v).point;
        prop_assert!(
            (got - expected).norm() <= EXACT,
            "{s:?} at ({u}, {v}): {got} vs {expected}"
        );
        // The same surface in the world frame, moved by the pose, is this one.
        let moved = with_frame(&s, Frame::world()).transformed(&pose);
        let m = moved.eval(u, v);
        let e = s.eval(u, v);
        prop_assert!((m.point - e.point).norm() <= EXACT);
        prop_assert!((m.du - e.du).norm() <= EXACT && (m.dv - e.dv).norm() <= EXACT);
        prop_assert!((m.duu - e.duu).norm() <= EXACT && (m.dvv - e.dvv).norm() <= EXACT);
        prop_assert_eq!(moved.kind(), s.kind());
        Ok(())
    });
}

#[test]
fn posed_curve_equals_the_local_closed_form_moved_by_the_pose() {
    check((curve(), finite_f64(-1.0..=TAU + 1.0)), |(c, t)| {
        let pose = curve_frame(&c).as_isometry();
        let expected = pose.apply(local_curve(&c, t));
        let got = c.eval(t).point;
        prop_assert!(
            (got - expected).norm() <= EXACT,
            "{c:?} at {t}: {got} vs {expected}"
        );
        let moved = c.transformed(&pose.inverse()).transformed(&pose);
        let (m, e) = (moved.eval(t), c.eval(t));
        prop_assert!((m.point - e.point).norm() <= EXACT);
        prop_assert!((m.d1 - e.d1).norm() <= EXACT && (m.d2 - e.d2).norm() <= EXACT);
        Ok(())
    });
}

#[test]
fn surface_derivatives_match_central_differences() {
    check((surface(), params()), |(s, (u, v))| {
        let e = s.eval(u, v);
        let d = central_differences_surface(|u, v| s.point(u, v), u, v);
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
                "{name} of {s:?} at ({u}, {v}): {a} vs {b}, off by {err}"
            );
        }
        Ok(())
    });
}

#[test]
fn curve_derivatives_match_central_differences() {
    check((curve(), finite_f64(-1.0..=TAU + 1.0)), |(c, t)| {
        let e = c.eval(t);
        let d = central_differences_curve(|t| c.point(t), t);
        prop_assert!((e.d1 - d.d1).norm() <= DIFFERENCE, "d1 of {c:?} at {t}");
        prop_assert!((e.d2 - d.d2).norm() <= DIFFERENCE, "d2 of {c:?} at {t}");
        Ok(())
    });
}

#[test]
fn normal_is_unit_and_orthogonal_to_both_derivatives() {
    check((surface(), params()), |(s, (u, v))| {
        let e = s.eval(u, v);
        let Some(n) = s.normal(u, v) else {
            // Only the loci the table names are singular, and `params` reaches
            // the sphere's poles only at exactly ±π/2.
            prop_assert!(
                matches!(s, Surface::Sphere { .. }) && v.abs() == FRAC_PI_2
                    || matches!(s, Surface::Cone { .. }),
                "{s:?} has no normal at ({u}, {v})"
            );
            return Ok(());
        };
        prop_assert!((n.norm() - 1.0).abs() <= UNIT);
        let (du, dv) = (e.du.normalize(), e.dv.normalize());
        prop_assert!(n.dot(&du).abs() <= UNIT, "n·du = {}", n.dot(&du));
        prop_assert!(n.dot(&dv).abs() <= UNIT, "n·dv = {}", n.dot(&dv));
        // Outward: away from the axis for the cylinder, the elliptic
        // cylinder (whose normal is never more than a right angle from
        // the radial) and the cone, from the
        // centre for the sphere, from the tube's centre circle for the torus.
        let f = s.frame().unwrap();
        let outward = match s {
            Surface::Plane { .. } => f.z().into_inner(),
            Surface::Cylinder { .. } | Surface::EllipticCylinder { .. } | Surface::Cone { .. } => {
                let d = e.point - f.origin();
                d - d.dot(&f.z()) * f.z().into_inner()
            }
            Surface::Sphere { .. } => e.point - f.origin(),
            Surface::Torus { major_radius, .. } => {
                let d = e.point - f.origin();
                let radial = (d - d.dot(&f.z()) * f.z().into_inner()).normalize();
                e.point - (f.origin() + major_radius * radial)
            }
            Surface::Nurbs(_) => unreachable!("the analytic strategies yield no NURBS"),
        };
        prop_assert!(
            n.dot(&outward) > 0.0,
            "{s:?} normal at ({u}, {v}) points inward"
        );
        Ok(())
    });
}

#[test]
fn periodic_directions_repeat_after_one_period() {
    check((surface(), curve(), params()), |(s, c, (u, v))| {
        let e = s.eval(u, v);
        let [pu, pv] = s.period();
        if let Some(p) = pu {
            let w = s.eval(u + p, v);
            prop_assert!((w.point - e.point).norm() <= EXACT && (w.du - e.du).norm() <= EXACT);
        }
        if let Some(p) = pv {
            let w = s.eval(u, v + p);
            prop_assert!((w.point - e.point).norm() <= EXACT && (w.dv - e.dv).norm() <= EXACT);
        }
        prop_assert_eq!(pu.is_some(), s.domain()[0] == arris_math::Interval::TURN);
        if let Some(p) = c.period() {
            let (a, b) = (c.eval(u), c.eval(u + p));
            prop_assert!((a.point - b.point).norm() <= EXACT && (a.d1 - b.d1).norm() <= EXACT);
            prop_assert!(c.domain().length() == p);
        } else {
            prop_assert!(!c.domain().is_bounded());
        }
        Ok(())
    });
}

#[test]
fn the_singular_loci_have_no_normal_and_their_neighbours_do() {
    check((sphere(), cone(), finite_f64(0.0..=TAU)), |(s, k, u)| {
        prop_assert!(s.normal(u, FRAC_PI_2).is_none(), "{s:?} north pole");
        prop_assert!(s.normal(u, -FRAC_PI_2).is_none(), "{s:?} south pole");
        prop_assert!(s.normal(u, FRAC_PI_2 - 1e-6).is_some());
        let Surface::Cone {
            radius, half_angle, ..
        } = k
        else {
            unreachable!("the cone strategy yields cones")
        };
        let apex = -radius / real::sin(half_angle);
        prop_assert!(k.normal(u, apex).is_none(), "{k:?} apex at v = {apex}");
        prop_assert!(k.normal(u, apex + 1e-6).is_some() && k.normal(u, 0.0).is_some());
        // The apex is one point whatever `u` is.
        let a = k.point(u, apex);
        let b = k.point(u + 1.0, apex);
        prop_assert!((a - b).norm() <= EXACT, "apex moved with u: {a} vs {b}");
        Ok(())
    });
}

#[test]
fn frames_and_kinds_survive_a_transform() {
    check(
        (frame(), unit_vec3(), finite_f64(0.1..=10.0)),
        |(f, d, r)| {
            let s = Surface::Sphere {
                frame: f,
                radius: r,
            };
            let m = f.as_isometry().inverse();
            let back = s.transformed(&m);
            prop_assert!((back.frame().unwrap().origin() - Point3::origin()).norm() <= EXACT);
            prop_assert!((back.frame().unwrap().z().into_inner() - Vec3::z()).norm() <= 1e-14);
            let l = Curve::Line {
                origin: f.origin(),
                direction: d,
            };
            let Curve::Line { direction, .. } = &l.transformed(&m) else {
                unreachable!("a moved line is a line")
            };
            prop_assert!((direction.norm() - 1.0).abs() <= UNIT);
            Ok(())
        },
    );
}
