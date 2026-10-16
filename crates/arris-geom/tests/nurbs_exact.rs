//! The exact rational forms (`docs/DATA-MODEL.md` §NURBS): a conic arc, an
//! extrusion, a revolution and the twin `Surface::to_nurbs` gives an
//! analytic surface are the same *point set* as what they stand for, to
//! rounding, with the parametrisation the design says: linear where the
//! analytic one is linear, and an angle at every arc end otherwise.
//!
//! A rational quadratic is not parametrised by its angle, so "evaluates
//! onto its twin" is not `twin.eval(u, v) == surface.eval(u, v)`. It is
//! four facts, checked here at every sampled `(u, v)`:
//!
//! 1. every twin point lies on the surface (its implicit residual);
//! 2. the twin's corners are the surface's corners of `bounds`;
//! 3. the angle a twin point makes about the axis is continuous, monotone
//!    and runs from the range's start to its end, so the twin covers the
//!    range and nothing outside it;
//! 4. where the analytic parameter is linear the twin's is the same.
//!
//! Together 1–3 are an onto map of the range: a continuous monotone
//! parameter whose ends are the range's ends, on the surface.

use arris_math::real;
use core::f64::consts::{FRAC_PI_2, TAU};

mod common;

use arris_debug::prop::geom::nurbs_curve;
use arris_debug::prop::{check, finite_f64, frame, radius, unit_vec3};
use arris_geom::{NurbsCurve, NurbsSurface, Surface};
use arris_math::{Frame, Interval, Point3, Vec3};
use common::*;
use proptest::prelude::*;

/// An angle recovered from a point through `atan2` carries the point's
/// error over the radius it turns about, and the strategies keep that at
/// least a few hundredths of a unit.
const ANGLE: f64 = 1e-6;
/// Samples along a direction of a twin.
const SAMPLES: usize = 121;

/// The angles `angles` run through, unwrapped and required non-decreasing:
/// they must start at `start` (mod a turn) and add up to `sweep`.
fn assert_monotone_over(
    angles: &[f64],
    start: f64,
    sweep: f64,
    what: &str,
) -> Result<(), TestCaseError> {
    prop_assert!(
        wrap(angles[0] - start).abs() <= ANGLE,
        "{what}: starts at {} not {start}",
        angles[0]
    );
    let mut total = 0.0;
    for w in angles.windows(2) {
        let d = wrap(w[1] - w[0]);
        prop_assert!(d >= -ANGLE, "{what}: turns back by {d}");
        total += d;
    }
    prop_assert!(
        (total - sweep).abs() <= ANGLE,
        "{what}: covers {total}, the range is {sweep}"
    );
    Ok(())
}

/// The twin of `surface` over `bounds` is the surface's part over `bounds`.
fn assert_twin(surface: &Surface, bounds: [Interval; 2]) -> Result<(), TestCaseError> {
    let twin = surface.to_nurbs(bounds).expect("finite bounds, radii > 0");
    let [du, dv] = twin.domain();
    let grid = |n: usize| (0..=n).map(move |i| i as f64 / n as f64);

    // 1. On the surface.
    for s in grid(16) {
        for t in grid(16) {
            let p = twin.eval(du.lerp(s), dv.lerp(t)).point;
            let r = locate(surface, p).residual;
            prop_assert!(r <= EXACT, "({s}, {t}) is {r} off the surface");
        }
    }
    // 2. The corners of `bounds`.
    for (a, b, c, d) in [
        (du.lo(), dv.lo(), bounds[0].lo(), bounds[1].lo()),
        (du.lo(), dv.hi(), bounds[0].lo(), bounds[1].hi()),
        (du.hi(), dv.lo(), bounds[0].hi(), bounds[1].lo()),
        (du.hi(), dv.hi(), bounds[0].hi(), bounds[1].hi()),
    ] {
        let gap = (twin.eval(a, b).point - surface.point(c, d)).norm();
        prop_assert!(gap <= EXACT, "a corner is {gap} from the surface's");
    }
    // 4. Linear parameters agree.
    let linear_u = matches!(surface, Surface::Plane { .. });
    let linear_v = matches!(
        surface,
        Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
    );
    if linear_u {
        prop_assert_eq!(du, bounds[0]);
    }
    if linear_v {
        prop_assert_eq!(dv, bounds[1]);
        for t in grid(SAMPLES - 1) {
            let v = dv.lerp(t);
            let got = locate(surface, twin.eval(du.lerp(0.5), v).point).v;
            prop_assert!((got - v).abs() <= EXACT, "v = {v} lands at {got}");
        }
    }
    // 3. The angle of `u` and, where `v` is an angle, of `v`.
    if !linear_u {
        // The parameter `v` at which `u` is probed: the cone's end farthest
        // from the apex, where the radius is not small, else the middle.
        let probe = match *surface {
            Surface::Cone {
                radius, half_angle, ..
            } => {
                let reach = |v: f64| (radius + v * real::sin(half_angle)).abs();
                if reach(bounds[1].lo()) > reach(bounds[1].hi()) {
                    bounds[1].lo()
                } else {
                    bounds[1].hi()
                }
            }
            _ => dv.lerp(0.5),
        };
        let angles: Vec<f64> = (0..SAMPLES)
            .map(|i| {
                let u = du.lerp(i as f64 / (SAMPLES - 1) as f64);
                locate(surface, twin.eval(u, probe).point).angle
            })
            .collect();
        assert_monotone_over(&angles, bounds[0].lo(), bounds[0].length(), "u")?;
    }
    if !linear_v {
        let angles: Vec<f64> = (0..SAMPLES)
            .map(|i| {
                let v = dv.lerp(i as f64 / (SAMPLES - 1) as f64);
                locate(surface, twin.eval(du.lerp(0.5), v).point).v
            })
            .collect();
        assert_monotone_over(&angles, bounds[1].lo(), bounds[1].length(), "v")?;
    }
    // A full turn closes the net; a sphere's pole rows collapse.
    let [n, m] = twin.counts();
    if !linear_u && bounds[0].length() >= TAU * (1.0 - 1e-12) {
        for j in 0..m {
            prop_assert_eq!(twin.control_point(0, j), twin.control_point(n - 1, j));
        }
    }
    if let Surface::Sphere { .. } = surface {
        for (j, pole) in [
            (0, bounds[1].lo() == -FRAC_PI_2),
            (m - 1, bounds[1].hi() == FRAC_PI_2),
        ] {
            if pole {
                let first = twin.control_point(0, j);
                prop_assert!(
                    (0..n).all(|i| twin.control_point(i, j) == first),
                    "row {j} is not one point"
                );
            }
        }
    }
    Ok(())
}

#[test]
fn a_plane_is_a_bilinear_patch_of_itself() {
    check((frame(), line_range(), line_range()), |(f, u, v)| {
        let plane = Surface::Plane { frame: f };
        assert_twin(&plane, [u, v])?;
        let twin = plane.to_nurbs([u, v]).unwrap();
        for (s, t) in [(0.1, 0.9), (0.5, 0.5), (1.0, 0.0)] {
            let (a, b) = (u.lerp(s), v.lerp(t));
            let gap = (twin.eval(a, b).point - plane.point(a, b)).norm();
            prop_assert!(gap <= EXACT, "({a}, {b}) is {gap} apart");
        }
        Ok(())
    });
}

#[test]
fn a_cylinder_is_the_extrusion_of_its_circle() {
    check(
        (frame(), radius(0.1..=10.0), angle_range(), line_range()),
        |(f, r, u, v)| {
            assert_twin(
                &Surface::Cylinder {
                    frame: f,
                    radius: r,
                },
                [u, v],
            )
        },
    );
}

#[test]
fn an_elliptic_cylinder_is_the_extrusion_of_its_ellipse() {
    check(
        (
            frame(),
            radius(0.1..=10.0),
            finite_f64(0.1..=10.0),
            angle_range(),
            line_range(),
        ),
        |(f, b, extra, u, v)| {
            let s = Surface::EllipticCylinder {
                frame: f,
                major_radius: b + extra,
                minor_radius: b,
            };
            assert_twin(&s, [u, v])
        },
    );
}

#[test]
fn a_cone_is_the_revolution_of_its_ruling() {
    check(
        (
            frame(),
            radius(0.1..=10.0),
            finite_f64(0.05..=1.5),
            angle_range(),
            line_range(),
        ),
        |(f, r, alpha, u, v)| {
            let s = Surface::Cone {
                frame: f,
                radius: r,
                half_angle: alpha,
            };
            assert_twin(&s, [u, v])
        },
    );
}

#[test]
fn a_cone_through_its_apex_has_a_collapsed_row() {
    check(
        (
            frame(),
            radius(0.1..=10.0),
            finite_f64(0.05..=1.5),
            angle_range(),
        ),
        |(f, r, alpha, u)| {
            let cone = Surface::Cone {
                frame: f,
                radius: r,
                half_angle: alpha,
            };
            let apex = -r / real::sin(alpha);
            let v = Interval::new(apex, apex + 5.0 * r).unwrap();
            let twin = cone.to_nurbs([u, v]).unwrap();
            let [n, _] = twin.counts();
            let first = twin.control_point(0, 0);
            prop_assert!(
                (0..n).all(|i| twin.control_point(i, 0) == first),
                "the apex row is not one point"
            );
            assert_twin(&cone, [u, v])
        },
    );
}

#[test]
fn a_sphere_is_the_revolution_of_its_meridian() {
    check(
        (frame(), radius(0.1..=10.0), angle_range(), latitude_range()),
        |(f, r, u, v)| {
            assert_twin(
                &Surface::Sphere {
                    frame: f,
                    radius: r,
                },
                [u, v],
            )
        },
    );
}

#[test]
fn a_torus_is_the_revolution_of_its_tube() {
    check(
        (
            frame(),
            radius(0.1..=10.0),
            radius(0.1..=10.0),
            angle_range(),
            angle_range(),
        ),
        |(f, minor, extra, u, v)| {
            let s = Surface::Torus {
                frame: f,
                major_radius: minor + extra,
                minor_radius: minor,
            };
            assert_twin(&s, [u, v])
        },
    );
}

#[test]
fn a_nurbs_is_its_own_twin_and_unbounded_bounds_are_refused() {
    let f = Frame::world();
    let cylinder = Surface::Cylinder {
        frame: f,
        radius: 1.0,
    };
    let twin = cylinder.to_nurbs([Interval::TURN, Interval::UNIT]).unwrap();
    let again = Surface::Nurbs(twin.clone())
        .to_nurbs([Interval::UNIT, Interval::UNIT])
        .unwrap();
    assert_eq!(again, twin);
    for bad in [
        cylinder.to_nurbs([Interval::TURN, Interval::REAL]),
        cylinder.to_nurbs([Interval::new(0.0, 7.0).unwrap(), Interval::UNIT]),
        cylinder.to_nurbs([Interval::new(1.0, 1.0).unwrap(), Interval::UNIT]),
        Surface::Cylinder {
            frame: f,
            radius: 0.0,
        }
        .to_nurbs([Interval::TURN, Interval::UNIT]),
    ] {
        assert!(bad.is_err());
    }
}

#[test]
fn the_circle_and_the_ellipse_are_their_curves() {
    check(
        (
            frame(),
            radius(0.1..=10.0),
            finite_f64(0.1..=10.0),
            angle_range(),
        ),
        |(f, b, extra, range)| {
            let a = b + extra;
            let arcs = [
                (NurbsCurve::circle(&f, b, range).unwrap(), b, b),
                (NurbsCurve::ellipse(&f, a, b, range).unwrap(), a, b),
            ];
            for (arc, a, b) in arcs {
                let d = arc.domain();
                // At the ends, exactly the conic's points.
                for (t, angle) in [(d.lo(), range.lo()), (d.hi(), range.hi())] {
                    let want = f.origin()
                        + a * real::cos(angle) * f.x().into_inner()
                        + b * real::sin(angle) * f.y().into_inner();
                    let gap = (arc.eval(t).point - want).norm();
                    prop_assert!(gap <= EXACT, "an end is {gap} off");
                }
                // On it, monotone in its eccentric anomaly, over the range.
                let mut angles = Vec::new();
                for i in 0..SAMPLES {
                    let q = f.to_local(arc.eval(d.lerp(i as f64 / (SAMPLES - 1) as f64)).point);
                    let r = (real::hypot(q.x / a, q.y / b) - 1.0).abs() * b;
                    prop_assert!(r <= EXACT && q.z.abs() <= EXACT, "off the conic by {r}");
                    angles.push(real::atan2(q.y / b, q.x / a));
                }
                assert_monotone_over(&angles, range.lo(), range.length(), "t")?;
            }
            Ok(())
        },
    );
}

#[test]
fn the_parabola_is_evaluated_by_its_own_parameter() {
    check(
        (
            frame(),
            radius(0.05..=10.0),
            finite_f64(-5.0..=5.0),
            finite_f64(0.1..=6.0),
        ),
        |(f, focal, lo, len)| {
            let range = Interval::new(lo, lo + len).unwrap();
            let arc = NurbsCurve::parabola(&f, focal, range).unwrap();
            prop_assert_eq!(arc.domain(), range);
            for i in 0..=20 {
                let t = range.lerp(f64::from(i) / 20.0);
                let want = f.origin()
                    + (focal * t * t) * f.x().into_inner()
                    + (2.0 * focal * t) * f.y().into_inner();
                let gap = (arc.eval(t).point - want).norm();
                prop_assert!(gap <= EXACT, "t = {t} is {gap} off");
            }
            Ok(())
        },
    );
}

#[test]
fn the_hyperbola_is_on_its_branch_from_end_to_end() {
    check(
        (
            frame(),
            radius(0.1..=10.0),
            radius(0.1..=10.0),
            finite_f64(-3.0..=3.0),
            finite_f64(0.1..=4.0),
        ),
        |(f, a, b, lo, len)| {
            let range = Interval::new(lo, lo + len).unwrap();
            let arc = NurbsCurve::hyperbola(&f, a, b, range).unwrap();
            let d = arc.domain();
            for (t, param) in [(d.lo(), range.lo()), (d.hi(), range.hi())] {
                let want = f.origin()
                    + (a * real::cosh(param)) * f.x().into_inner()
                    + (b * real::sinh(param)) * f.y().into_inner();
                let gap = (arc.eval(t).point - want).norm();
                prop_assert!(
                    gap <= EXACT * (1.0 + real::cosh(param)),
                    "an end is {gap} off"
                );
            }
            let mut last = f64::NEG_INFINITY;
            for i in 0..SAMPLES {
                let q = f.to_local(arc.eval(d.lerp(i as f64 / (SAMPLES - 1) as f64)).point);
                let r = (real::powi(q.x / a, 2) - real::powi(q.y / b, 2) - 1.0).abs();
                prop_assert!(
                    r <= 1e-9 * (1.0 + real::powi(q.x / a, 2)),
                    "off the branch by {r}"
                );
                prop_assert!(q.z.abs() <= EXACT * 10.0);
                prop_assert!(q.y >= last - EXACT, "y turns back at sample {i}");
                last = q.y;
            }
            Ok(())
        },
    );
}

#[test]
fn an_extrusion_is_the_curve_swept_along_the_direction() {
    check(
        (
            nurbs_curve(),
            unit_vec3(),
            finite_f64(0.1..=10.0),
            line_range(),
        ),
        |(c, d, scale, range)| {
            let dir: Vec3 = scale * d.into_inner();
            let s = NurbsSurface::extrusion(&c, dir, range).unwrap();
            prop_assert_eq!(s.domain()[0], c.domain());
            prop_assert_eq!(s.domain()[1], range);
            for i in 0..=8 {
                for j in 0..=8 {
                    let (u, v) = (
                        c.domain().lerp(f64::from(i) / 8.0),
                        range.lerp(f64::from(j) / 8.0),
                    );
                    let gap = (s.eval(u, v).point - (c.eval(u).point + v * dir)).norm();
                    prop_assert!(gap <= EXACT, "({u}, {v}) is {gap} off");
                }
            }
            Ok(())
        },
    );
}

#[test]
fn a_revolution_turns_the_curve_about_the_axis() {
    check((nurbs_curve(), frame(), angle_range()), |(c, f, angle)| {
        let (o, a) = (f.origin(), f.z().into_inner());
        let s = NurbsSurface::revolution(&c, o, f.z(), angle).unwrap();
        prop_assert_eq!(s.domain()[0], angle);
        prop_assert_eq!(s.domain()[1], c.domain());
        let split = |p: Point3| {
            let axial = (p - o).dot(&a);
            (axial, p - o - axial * a)
        };
        for i in 0..=12 {
            for j in 0..=12 {
                let (u, v) = (
                    angle.lerp(f64::from(i) / 12.0),
                    c.domain().lerp(f64::from(j) / 12.0),
                );
                let (axial, spoke) = split(s.eval(u, v).point);
                let (want_axial, want_spoke) = split(c.eval(v).point);
                // Turning keeps the height and the distance from the axis.
                prop_assert!((axial - want_axial).abs() <= EXACT, "the height moved");
                prop_assert!(
                    (spoke.norm() - want_spoke.norm()).abs() <= EXACT,
                    "the radius moved"
                );
                // At an arc end the turn is exactly Rodrigues' rotation by `u`.
                if i == 0 || i == 12 {
                    let (su, cu) = real::sin_cos(u);
                    let turned = o + axial * a + cu * want_spoke + su * a.cross(&want_spoke);
                    let gap = (s.eval(u, v).point - turned).norm();
                    prop_assert!(gap <= EXACT, "({u}, {v}) is {gap} from the turn");
                }
            }
        }
        Ok(())
    });
}
