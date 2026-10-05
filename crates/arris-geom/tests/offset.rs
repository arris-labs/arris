//! `Surface::offset` moves a surface along its own normal and stays the
//! same kind (`docs/DATA-MODEL.md` §Surfaces): a point `d` along the
//! normal from any sampled point lies on the offset, whose own normal
//! there is the same one, and every collapse is `None`.

use core::f64::consts::{FRAC_PI_2, TAU};

use arris_debug::prop::geom::{
    ELLIPTIC_GAP, HALF_ANGLE_RANGE, RADIUS_RANGE, cone, cylinder, elliptic_cylinder, nurbs_surface,
    plane, sphere, torus,
};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64};
use arris_geom::{Surface, SurfaceKind};
use proptest::prelude::*;

/// A moved point against the offset's closed form, and its normal against
/// the original's: a few multiples of rounding at the strategies' scale.
const EXACT: f64 = 1e-9;
/// Distances reach past every radius of the strategies, so collapses are drawn.
const REACH: f64 = 2.0 * *RADIUS_RANGE.end();

fn params() -> impl Strategy<Value = (f64, f64)> {
    (
        finite_f64(-1.0..=TAU + 1.0),
        finite_f64(-FRAC_PI_2..=FRAC_PI_2),
    )
}

/// Whether `(u, v)` is on the nappe a cone's normal is outward on, and
/// stays there once moved `d` along it (the move does not cross the axis).
fn on_the_main_nappe(s: &Surface, v: f64, d: f64) -> bool {
    match *s {
        Surface::Cone {
            radius, half_angle, ..
        } => {
            radius + v * half_angle.sin() > 0.0
                && radius + v * half_angle.sin() + d * half_angle.cos() > 0.0
        }
        _ => true,
    }
}

/// The sampled points of `s` moved by `d` along its normal lie on `offset`
/// with the same normal, at distance `|d|` from where they started.
fn moved_points_lie_on(
    s: &Surface,
    offset: &Surface,
    d: f64,
    uv: (f64, f64),
) -> Result<(), TestCaseError> {
    let (u, v) = uv;
    prop_assume!(on_the_main_nappe(s, v, d));
    let Some(n) = s.normal(u, v) else {
        return Ok(());
    };
    let p = s.point(u, v);
    let q = p + d * n.into_inner();
    prop_assert!(((q - p).norm() - d.abs()).abs() <= EXACT);
    // A point on an axis has no nearest point to speak of: not the offset's fault.
    let Ok(on) = offset.project(q) else {
        return Ok(());
    };
    prop_assert!(
        on.distance <= EXACT,
        "{q:?} is {} from the offset",
        on.distance
    );
    if let Some(m) = offset.normal(on.uv.x, on.uv.y) {
        prop_assert!(
            (m.into_inner() - n.into_inner()).norm() <= 1e-6,
            "the offset's normal {m:?} is not the original's {n:?}"
        );
    }
    Ok(())
}

fn same_kind(s: &Surface, offset: &Surface) -> bool {
    s.kind() == offset.kind()
}

#[test]
fn a_plane_moves_along_its_normal_with_its_parameters() {
    check(
        (plane(), finite_f64(-REACH..=REACH), params()),
        |(s, d, uv)| {
            let o = s.offset(d).expect("a plane always offsets");
            prop_assert!(same_kind(&s, &o));
            // `(u, v)` unchanged: the point is the original's plus d·normal.
            let n = s.normal(uv.0, uv.1).unwrap();
            prop_assert!(
                (o.point(uv.0, uv.1) - (s.point(uv.0, uv.1) + d * n.into_inner())).norm() <= EXACT
            );
            moved_points_lie_on(&s, &o, d, uv)
        },
    );
}

#[test]
fn a_cylinder_is_coaxial_with_radius_plus_d() {
    check(
        (cylinder(), finite_f64(-REACH..=REACH), params()),
        |(s, d, uv)| {
            let Surface::Cylinder { radius, .. } = s else {
                unreachable!()
            };
            let moved = radius + d;
            prop_assume!(moved.abs() > EXACT);
            match s.offset(d) {
                Some(o) => {
                    prop_assert!(moved > 0.0);
                    prop_assert!(same_kind(&s, &o));
                    moved_points_lie_on(&s, &o, d, uv)?;
                }
                None => prop_assert!(moved <= 0.0),
            }
            Ok(())
        },
    );
}

#[test]
fn a_cone_keeps_its_axis_and_half_angle_and_grows_by_d_over_cos() {
    check(
        (cone(), finite_f64(-REACH..=REACH), params()),
        |(s, d, uv)| {
            let o = s.offset(d).expect("a cone always offsets");
            prop_assert!(same_kind(&s, &o));
            let (
                Surface::Cone {
                    radius, half_angle, ..
                },
                Surface::Cone {
                    radius: moved,
                    half_angle: same,
                    ..
                },
            ) = (&s, &o)
            else {
                unreachable!()
            };
            prop_assert_eq!(half_angle, same);
            prop_assert!((moved - (radius + d / half_angle.cos())).abs() <= EXACT);
            moved_points_lie_on(&s, &o, d, uv)
        },
    );
}

#[test]
fn a_sphere_is_concentric_with_radius_plus_d() {
    check(
        (sphere(), finite_f64(-REACH..=REACH), params()),
        |(s, d, uv)| {
            let Surface::Sphere { radius, .. } = s else {
                unreachable!()
            };
            let moved = radius + d;
            prop_assume!(moved.abs() > EXACT);
            match s.offset(d) {
                Some(o) => {
                    prop_assert!(moved > 0.0);
                    moved_points_lie_on(&s, &o, d, uv)?;
                }
                None => prop_assert!(moved <= 0.0),
            }
            Ok(())
        },
    );
}

#[test]
fn a_torus_keeps_its_major_radius_and_its_tube_grows_by_d() {
    check(
        (torus(), finite_f64(-REACH..=REACH), params()),
        |(s, d, uv)| {
            let Surface::Torus {
                major_radius,
                minor_radius,
                ..
            } = s
            else {
                unreachable!()
            };
            let moved = minor_radius + d;
            prop_assume!(moved.abs() > EXACT && (major_radius - moved).abs() > EXACT);
            match s.offset(d) {
                Some(o) => {
                    prop_assert!(0.0 < moved && moved < major_radius);
                    let Surface::Torus {
                        major_radius: big,
                        minor_radius: small,
                        ..
                    } = o
                    else {
                        panic!("a torus offsets to a torus, got {o:?}");
                    };
                    prop_assert_eq!(big, major_radius);
                    prop_assert!((small - moved).abs() <= EXACT);
                    moved_points_lie_on(&s, &o, d, uv)?;
                }
                None => prop_assert!(moved <= 0.0 || moved >= major_radius),
            }
            Ok(())
        },
    );
}

#[test]
fn elliptic_cylinders_nurbs_surfaces_and_non_finite_distances_have_no_offset() {
    check(
        (elliptic_cylinder(), finite_f64(-REACH..=REACH)),
        |(s, d)| {
            prop_assert!(s.offset(d).is_none());
            Ok(())
        },
    );
    check((nurbs_surface(), finite_f64(-REACH..=REACH)), |(s, d)| {
        let s = Surface::Nurbs(s);
        prop_assert_eq!(s.kind(), SurfaceKind::Nurbs);
        prop_assert!(s.offset(d).is_none());
        Ok(())
    });
    check(cylinder(), |s| {
        for d in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            prop_assert!(s.offset(d).is_none());
        }
        Ok(())
    });
}

#[test]
fn a_radius_driven_exactly_to_zero_is_a_collapse() {
    use arris_math::Frame;
    let f = Frame::world();
    assert!(
        Surface::Cylinder {
            frame: f,
            radius: 2.0
        }
        .offset(-2.0)
        .is_none()
    );
    assert!(
        Surface::Sphere {
            frame: f,
            radius: 2.0
        }
        .offset(-2.0)
        .is_none()
    );
    assert!(
        Surface::Torus {
            frame: f,
            major_radius: 3.0,
            minor_radius: 2.0
        }
        .offset(1.0)
        .is_none()
    );
    assert!(
        Surface::Torus {
            frame: f,
            major_radius: 3.0,
            minor_radius: 2.0
        }
        .offset(-2.0)
        .is_none()
    );
    // Just short of a collapse is an offset.
    assert!(
        Surface::Torus {
            frame: f,
            major_radius: 3.0,
            minor_radius: 2.0
        }
        .offset(0.5)
        .is_some()
    );
}

/// The ranges the strategies draw from, so a change to them is noticed here.
#[test]
fn the_strategies_ranges_leave_room_for_the_reach() {
    assert!(*RADIUS_RANGE.start() > 0.0 && REACH > *RADIUS_RANGE.end());
    assert!(*HALF_ANGLE_RANGE.end() < FRAC_PI_2 && ELLIPTIC_GAP > 0.0 && DEFAULT_SCALE > REACH);
}
