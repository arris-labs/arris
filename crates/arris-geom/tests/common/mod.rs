//! Shared by the exact-form tests: the ranges the twins are built over and
//! where a point is on an analytic surface, in closed form.
#![allow(dead_code)] // each test crate uses some of it

use arris_math::real;
use core::f64::consts::{FRAC_PI_2, PI, TAU};

use arris_debug::prop::finite_f64;
use arris_geom::Surface;
use arris_math::{Interval, Point3};
use proptest::prelude::*;

/// Closed form against evaluation, as in `tests/nurbs.rs`.
pub const EXACT: f64 = 1e-12 * arris_debug::prop::DEFAULT_SCALE;

/// A range of angle: any start, a sweep from a few hundredths of a radian
/// to a full turn, one time in five exactly a turn.
pub fn angle_range() -> impl Strategy<Value = Interval> {
    (
        finite_f64(-7.0..=7.0),
        prop_oneof![4 => finite_f64(0.05..=TAU), 1 => Just(TAU)],
    )
        .prop_map(|(lo, sweep)| Interval::new(lo, lo + sweep).unwrap())
}

/// A range along a linear parameter, a unit or more long.
pub fn line_range() -> impl Strategy<Value = Interval> {
    (finite_f64(-20.0..=20.0), finite_f64(1.0..=30.0))
        .prop_map(|(lo, len)| Interval::new(lo, lo + len).unwrap())
}

/// A range of the sphere's latitude: at least a fifth of a radian, one time
/// in three reaching a pole exactly.
pub fn latitude_range() -> impl Strategy<Value = Interval> {
    (
        prop_oneof![2 => finite_f64(-FRAC_PI_2..=FRAC_PI_2 - 0.2), 1 => Just(-FRAC_PI_2)],
        finite_f64(0.2..=PI),
        any::<bool>(),
    )
        .prop_map(|(lo, len, to_pole)| {
            let hi = if to_pole {
                FRAC_PI_2
            } else {
                (lo + len).min(FRAC_PI_2)
            };
            Interval::new(lo.min(hi - 0.2), hi).unwrap()
        })
}

/// Where a point is on a surface: the distance from it (scaled to a
/// length), and its angle about the axis and its `v`, in the analytic
/// parametrisation. Closed forms in the surface's own frame.
pub struct Located {
    pub residual: f64,
    pub angle: f64,
    pub v: f64,
}

pub fn locate(surface: &Surface, p: Point3) -> Located {
    let frame = surface.frame().expect("an analytic surface");
    let q = frame.to_local(p);
    let rho = real::hypot(q.x, q.y);
    match *surface {
        Surface::Plane { .. } => Located {
            residual: q.z.abs(),
            angle: q.x,
            v: q.y,
        },
        Surface::Cylinder { radius, .. } => Located {
            residual: (rho - radius).abs(),
            angle: real::atan2(q.y, q.x),
            v: q.z,
        },
        Surface::EllipticCylinder {
            major_radius: a,
            minor_radius: b,
            ..
        } => Located {
            residual: (real::hypot(q.x / a, q.y / b) - 1.0).abs() * b,
            angle: real::atan2(q.y / b, q.x / a),
            v: q.z,
        },
        Surface::Cone {
            radius, half_angle, ..
        } => {
            let signed = radius + q.z * real::tan(half_angle);
            let side = signed.signum();
            Located {
                residual: (rho - signed.abs()).abs() * real::cos(half_angle),
                angle: real::atan2(side * q.y, side * q.x),
                v: q.z / real::cos(half_angle),
            }
        }
        Surface::Sphere { radius, .. } => Located {
            residual: (q.coords.norm() - radius).abs(),
            angle: real::atan2(q.y, q.x),
            v: real::atan2(q.z, rho),
        },
        Surface::Torus {
            major_radius,
            minor_radius,
            ..
        } => {
            let across = rho - major_radius;
            Located {
                residual: (real::hypot(across, q.z) - minor_radius).abs(),
                angle: real::atan2(q.y, q.x),
                v: real::atan2(q.z, across),
            }
        }
        Surface::Nurbs(_) => unreachable!("the twins of analytic surfaces only"),
    }
}

/// `x` moved into `(−π, π]`.
pub fn wrap(x: f64) -> f64 {
    let w = (x + PI).rem_euclid(TAU) - PI;
    if w <= -PI { w + TAU } else { w }
}
