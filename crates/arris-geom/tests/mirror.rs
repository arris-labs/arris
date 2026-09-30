//! The geometry of a mirror (ADR-0031 §2–3): a curve keeps its parameter,
//! a quadric reflects `u ↦ 2π − u`, a plane and a NURBS surface keep
//! their parameters and turn their normal, every frame stays
//! right-handed, and a pcurve reflected by the map lands on the mirror
//! image of the original's 3D point.

use core::f64::consts::{FRAC_PI_2, TAU};

use arris_debug::prop::geom::{curve, nurbs_curve, nurbs_surface, surface};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, point_in_box, radius, unit_vec3};
use arris_geom::{Curve, Curve2, ParamMap, Surface, SurfaceKind};
use arris_math::{Frame, Frame2, Handedness, Point2, Point3, Reflection, Vec2, Vec3};
use proptest::prelude::*;

/// Rounding slack at the strategies' scale: a mirror is a handful of
/// multiplications per coordinate.
const SLACK: f64 = 1e-11;

fn reflection() -> impl Strategy<Value = Reflection> {
    (point_in_box(DEFAULT_SCALE), unit_vec3())
        .prop_map(|(o, n)| Reflection::new(o, n.into_inner()).unwrap())
}

fn right_handed_and_accepted(f: &Frame) -> bool {
    Frame::from_orthonormal(
        f.origin(),
        f.x().into_inner(),
        f.y().into_inner(),
        f.z().into_inner(),
    )
    .is_ok()
        && (f.x().cross(&f.y()) - f.z().into_inner()).norm() < 1e-14
}

fn frame_of(s: &Surface) -> Option<Frame> {
    match s {
        Surface::Plane { frame }
        | Surface::Cylinder { frame, .. }
        | Surface::EllipticCylinder { frame, .. }
        | Surface::Cone { frame, .. }
        | Surface::Sphere { frame, .. }
        | Surface::Torus { frame, .. } => Some(*frame),
        Surface::Nurbs(_) => None,
    }
}

/// Parameters inside every analytic surface's useful range.
fn uv() -> impl Strategy<Value = (f64, f64)> {
    (finite_f64(0.0..=TAU), finite_f64(-1.2..=1.2))
}

#[test]
fn a_mirrored_surface_is_the_image_at_the_mapped_parameters() {
    check((surface(), reflection(), uv()), |(s, r, (u, v))| {
        let (image, map) = s.mirrored(&r);
        let (mu, mv) = map.apply(u, v);
        let (a, b) = (image.eval(mu, mv).point, r.apply(s.eval(u, v).point));
        prop_assert!((a - b).norm() < SLACK, "{a:?} vs {b:?}");
        prop_assert_eq!(
            map == ParamMap::Identity,
            matches!(s.kind(), SurfaceKind::Plane | SurfaceKind::Nurbs)
        );
        Ok(())
    });
}

#[test]
fn a_mirrored_nurbs_surface_is_the_image_and_its_normal_turns() {
    check((nurbs_surface(), reflection()), |(s, r)| {
        let surf = Surface::Nurbs(s);
        let (image, map) = surf.mirrored(&r);
        prop_assert_eq!(map, ParamMap::Identity);
        let d = surf.domain();
        for (i, j) in [(0.2, 0.3), (0.5, 0.5), (0.9, 0.1)] {
            let (u, v) = (d[0].lerp(i), d[1].lerp(j));
            let (a, b) = (image.eval(u, v).point, r.apply(surf.eval(u, v).point));
            prop_assert!((a - b).norm() < SLACK);
            if let (Some(n), Some(m)) = (surf.normal(u, v), image.normal(u, v)) {
                let expected = -r.apply_vec(n.into_inner());
                prop_assert!((m.into_inner() - expected).norm() < 1e-9);
            }
        }
        prop_assert_eq!(image.domain(), surf.domain());
        Ok(())
    });
}

#[test]
fn the_normal_is_the_images_for_a_quadric_and_opposite_for_a_plane() {
    check((surface(), reflection(), uv()), |(s, r, (u, v))| {
        let (image, map) = s.mirrored(&r);
        let (mu, mv) = map.apply(u, v);
        let (Some(n), Some(m)) = (s.normal(u, v), image.normal(mu, mv)) else {
            return Ok(());
        };
        let sign = if map == ParamMap::Identity { -1.0 } else { 1.0 };
        let expected = sign * r.apply_vec(n.into_inner());
        prop_assert!(
            (m.into_inner() - expected).norm() < 1e-9,
            "{:?}: {m:?} vs {expected:?}",
            s.kind()
        );
        Ok(())
    });
}

#[test]
fn a_mirrored_frame_is_right_handed_and_a_frame() {
    check((surface(), reflection()), |(s, r)| {
        let (image, _) = s.mirrored(&r);
        let f = frame_of(&image).unwrap();
        prop_assert!(right_handed_and_accepted(&f), "{f:?}");
        Ok(())
    });
}

#[test]
fn a_periodic_range_keeps_its_convention() {
    check((surface(), reflection()), |(s, r)| {
        let (image, _) = s.mirrored(&r);
        prop_assert_eq!(image.domain(), s.domain());
        prop_assert_eq!(image.period(), s.period());
        Ok(())
    });
}

#[test]
fn a_curve_keeps_its_parameter() {
    check(
        (curve(), reflection(), finite_f64(0.0..=TAU)),
        |(c, r, t)| {
            let image = c.mirrored(&r);
            let (a, b) = (image.point(t), r.apply(c.point(t)));
            prop_assert!((a - b).norm() < SLACK);
            let (da, db) = (image.eval(t).d1, r.apply_vec(c.eval(t).d1));
            prop_assert!((da - db).norm() < SLACK);
            prop_assert_eq!(image.domain(), c.domain());
            prop_assert_eq!(image.period(), c.period());
            if let Curve::Circle { .. } | Curve::Ellipse { .. } = image {
                let frame = match image {
                    Curve::Circle { frame, .. } | Curve::Ellipse { frame, .. } => frame,
                    _ => unreachable!(),
                };
                prop_assert!(right_handed_and_accepted(&frame));
            }
            Ok(())
        },
    );
    check(
        (nurbs_curve(), reflection(), finite_f64(0.0..=1.0)),
        |(c, r, t)| {
            let t = c.domain().lerp(t);
            let image = Curve::Nurbs(c.clone()).mirrored(&r);
            prop_assert!((image.point(t) - r.apply(c.eval(t).point)).norm() < SLACK);
            Ok(())
        },
    );
}

/// A pcurve of each kind, either handedness, anywhere in the (u, v) plane.
fn pcurve() -> impl Strategy<Value = Curve2> {
    fn frame2() -> impl Strategy<Value = Frame2> {
        (
            finite_f64(-1.0..=8.0),
            finite_f64(-1.0..=1.0),
            finite_f64(0.0..=TAU),
            any::<bool>(),
        )
            .prop_map(|(u, v, a, left)| {
                Frame2::new(
                    Point2::new(u, v),
                    Vec2::new(a.cos(), a.sin()),
                    if left {
                        Handedness::Left
                    } else {
                        Handedness::Right
                    },
                )
                .unwrap()
            })
    }
    prop_oneof![
        (frame2(), radius(0.1..=2.0)).prop_map(|(frame, radius)| Curve2::Circle { frame, radius }),
        (frame2(), radius(0.5..=2.0), 0.2..=0.9f64).prop_map(|(frame, major_radius, k)| {
            Curve2::Ellipse {
                frame,
                major_radius,
                minor_radius: major_radius * k,
            }
        }),
        (
            finite_f64(-1.0..=8.0),
            finite_f64(-1.0..=1.0),
            finite_f64(0.0..=TAU)
        )
            .prop_map(|(u, v, a)| {
                Curve2::Line {
                    origin: Point2::new(u, v),
                    direction: arris_math::UnitVec2::new_normalize(Vec2::new(a.cos(), a.sin())),
                }
            }),
    ]
}

#[test]
fn a_reflected_pcurve_on_the_mirrored_quadric_is_the_mirror_of_the_original() {
    check(
        (surface(), reflection(), pcurve(), finite_f64(0.0..=TAU)),
        |(s, r, p, t)| {
            let (image, map) = s.mirrored(&r);
            if map == ParamMap::Identity {
                // A plane's pcurve is reused as it stands.
                let q = p.point(t);
                let (a, b) = (image.eval(q.x, q.y).point, r.apply(s.eval(q.x, q.y).point));
                prop_assert!((a - b).norm() < SLACK);
                return Ok(());
            }
            let q = p.point(t);
            let reflected = p.reflected().point(t);
            prop_assert!((reflected - Point2::new(TAU - q.x, q.y)).norm() < 1e-14);
            let (a, b) = (
                image.eval(reflected.x, reflected.y).point,
                r.apply(s.eval(q.x, q.y).point),
            );
            prop_assert!((a - b).norm() < SLACK, "{a:?} vs {b:?}");
            Ok(())
        },
    );
}

#[test]
fn a_reflected_pcurve_is_an_involution_on_the_period() {
    check((pcurve(), finite_f64(0.0..=TAU)), |(p, t)| {
        let twice = p.reflected().reflected();
        prop_assert!((twice.point(t) - p.point(t)).norm() < 1e-13);
        Ok(())
    });
}

fn cylinder() -> Surface {
    Surface::Cylinder {
        frame: Frame::world(),
        radius: 2.0,
    }
}

fn holds(s: &Surface, r: &Reflection) {
    let (image, map) = s.mirrored(r);
    for &(u, v) in &[(0.0, 0.0), (1.0, 0.5), (TAU, 1.0), (3.0, -0.4)] {
        let (mu, mv) = map.apply(u, v);
        let d = image.eval(mu, mv).point - r.apply(s.eval(u, v).point);
        assert!(d.norm() < SLACK, "{s:?} at ({u}, {v}): {d:?}");
    }
}

#[test]
fn a_plane_through_the_axis() {
    let r = Reflection::new(Point3::origin(), Vec3::x()).unwrap();
    holds(&cylinder(), &r);
    let (image, _) = cylinder().mirrored(&r);
    // The cylinder maps onto itself; the seam point (2, 0, 0) goes to
    // (−2, 0, 0), and the image's seam is there.
    assert!((image.eval(0.0, 0.0).point - Point3::new(-2.0, 0.0, 0.0)).norm() < SLACK);
}

#[test]
fn a_plane_parallel_to_the_axis() {
    let r = Reflection::new(Point3::new(5.0, 0.0, 0.0), Vec3::x()).unwrap();
    holds(&cylinder(), &r);
}

#[test]
fn a_plane_normal_to_the_axis() {
    let r = Reflection::new(Point3::new(0.0, 0.0, 1.0), Vec3::z()).unwrap();
    holds(&cylinder(), &r);
    let (image, map) = cylinder().mirrored(&r);
    assert_eq!(map, ParamMap::ReflectU);
    // Z′ = R Z, which this plane turns to −Z: `v` runs along the image of
    // the original axis, not along the original direction.
    assert!((frame_of(&image).unwrap().z().into_inner() + Vec3::z()).norm() < 1e-15);
}

#[test]
fn a_sphere_keeps_its_poles_and_a_torus_its_seam() {
    let sphere = Surface::Sphere {
        frame: Frame::world(),
        radius: 3.0,
    };
    let torus = Surface::Torus {
        frame: Frame::world(),
        major_radius: 4.0,
        minor_radius: 1.0,
    };
    for r in [
        Reflection::new(Point3::origin(), Vec3::z()).unwrap(),
        Reflection::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(1.0, 1.0, 0.3)).unwrap(),
    ] {
        let (image, map) = sphere.mirrored(&r);
        for pole in [FRAC_PI_2, -FRAC_PI_2] {
            for u in [0.0, 1.0, TAU] {
                let (mu, mv) = map.apply(u, pole);
                let d = image.eval(mu, mv).point - r.apply(sphere.eval(u, pole).point);
                assert!(d.norm() < SLACK);
            }
        }
        holds(&sphere, &r);
        holds(&torus, &r);
        // The seam u = 0 and u = 2π stay on the image of the seam.
        let (image, map) = torus.mirrored(&r);
        for (u, v) in [(0.0, 0.3), (TAU, 0.3)] {
            let (mu, mv) = map.apply(u, v);
            assert!((image.eval(mu, mv).point - r.apply(torus.eval(u, v).point)).norm() < SLACK);
        }
    }
}

#[test]
fn a_degenerate_normal_is_refused() {
    assert!(Reflection::new(Point3::origin(), Vec3::zeros()).is_err());
    assert!(Reflection::new(Point3::origin(), Vec3::new(f64::NAN, 0.0, 1.0)).is_err());
    assert!(Reflection::new(Point3::new(f64::INFINITY, 0.0, 0.0), Vec3::z()).is_err());
    assert!(Reflection::new(Point3::origin(), Vec3::new(1e-300, 0.0, 0.0)).is_ok());
}
