//! Circular arcs on horn and spindle meridians, swept about the axis
//! in random poses. The closed forms use the sketch's dimensions.

use std::f64::consts::{PI, TAU};

use arris_check::{Level, check};
use arris_debug::fixtures::Tolerances;
use arris_debug::testing::{close_to, fail};
use arris_debug::unmetered::{mass_properties, revolve};
use arris_debug::{dump_text, prop, prop_shards};
use arris_geom::{Profile, ProfileLoop, ProfileSegment};
use arris_math::{Axis, Frame, Point2, real};
use arris_ops::{OpError, Reason, SweepReason};
use arris_topo::Model;
use proptest::prelude::*;

#[derive(Clone, Copy, Debug)]
enum ArcEnd {
    Clear(f64),
    Pole,
}

fn point(major: f64, radius: f64, t: f64) -> Point2 {
    let (s, c) = real::sin_cos(t);
    Point2::new(major + radius * c, radius * s)
}

fn axis(plane: &Frame) -> Axis {
    Axis {
        origin: plane.origin(),
        direction: plane.y(),
    }
}

prop_shards! {
    /// The arc stays on the outer sheet and may end on a pole. Its sweep
    /// is clean at Full and agrees with Pappus for volume and area.
    arcs_beside_or_ending_on_the_axis_sweep_to_pappus
        [shard_0 shard_1 shard_2 shard_3]
        ((radius, ratio, start_fraction, end, from_end, angle, plane)) = (
            prop::radius(1.0..=8.0),
            prop_oneof![3 => prop::finite_f64(0.05..=0.95), 1 => Just(1.0)],
            prop::finite_f64(0.1..=0.35),
            prop_oneof![prop::finite_f64(0.6..=0.9).prop_map(ArcEnd::Clear), Just(ArcEnd::Pole)],
            any::<bool>(),
            prop_oneof![4 => prop::finite_f64(0.25..=TAU - 0.25), 1 => Just(TAU)],
            prop::frame(),
        ) => {
            let major = radius * ratio;
            let pole = real::acos(-ratio);
            let a = pole * start_fraction;
            let left_angle = match end {
                ArcEnd::Clear(fraction) => pole * fraction,
                ArcEnd::Pole => pole,
            };
            let right = point(major, radius, a);
            let mut left = point(major, radius, left_angle);
            if matches!(end, ArcEnd::Pole) {
                left.x = 0.0;
            }
            let baseline = -radius / 2.0;
            let right_base = Point2::new(right.x, baseline);
            let left_base = Point2::new(left.x, baseline);
            let (start, finish, finish_base, start_base) = if from_end {
                (left, right, right_base, left_base)
            } else {
                (right, left, left_base, right_base)
            };
            let profile = Profile {
                plane,
                outer: ProfileLoop::Path {
                    start,
                    segments: vec![
                        ProfileSegment::ArcTo {
                            via: point(major, radius, (a + left_angle) / 2.0),
                            to: finish,
                        },
                        ProfileSegment::LineTo(finish_base),
                        ProfileSegment::LineTo(start_base),
                        ProfileSegment::LineTo(start),
                    ],
                },
                holes: Vec::new(),
            };
            let mut model = Model::default();
            let (body, _) = revolve(&mut model, &profile, axis(&plane), angle).map_err(fail)?;
            let report = check(&model, body, Level::Full);
            prop_assert!(
                report.is_ok() && report.unchecked().is_empty(),
                "{report}\n{}", dump_text(&model, body).map_err(fail)?
            );

            // For x = R + r cos t, y = r sin t, integrate (y − baseline)
            // and x(y − baseline) against −dx from a to left_angle.
            let b = left_angle;
            let (sin_a, cos_a) = real::sin_cos(a);
            let (sin_b, cos_b) = real::sin_cos(b);
            let delta = b - a;
            let section_area = real::powi(radius, 2)
                * (delta - (sin_b * cos_b - sin_a * cos_a)) / 2.0
                - baseline * radius * (cos_a - cos_b);
            let moment = major * section_area
                + real::powi(radius, 3) * (real::powi(sin_b, 3) - real::powi(sin_a, 3)) / 3.0
                - baseline * real::powi(radius, 2) * (real::powi(sin_b, 2) - real::powi(sin_a, 2)) / 2.0;
            let volume = angle * moment;

            // The integral of x ds on the arc, both vertical sides and
            // the baseline gives the lateral area per radian.
            let boundary_moment = radius * (major * delta + radius * (sin_b - sin_a))
                + right.x * (right.y - baseline)
                + left.x * (left.y - baseline)
                + (real::powi(right.x, 2) - real::powi(left.x, 2)) / 2.0;
            let ends = if angle == TAU { 0.0 } else { 2.0 * section_area };
            let area = angle * boundary_moment + ends;
            let props = mass_properties(&model, body).map_err(fail)?;
            let tol = Tolerances::default();
            prop_assert!(
                close_to(props.volume, volume, 0.0, tol.volume_rel),
                "volume {} vs Pappus {}", props.volume, volume
            );
            prop_assert!(
                close_to(props.area, area, 0.0, tol.area_rel),
                "area {} vs Pappus {}", props.area, area
            );
            Ok(())
        }
}

prop_shards! {
    /// Both arc endpoints are clear of the axis. Its interior crosses
    /// the axis twice, so the revolve must refuse it.
    arcs_crossing_the_axis_are_refused
        [shard_0 shard_1 shard_2 shard_3]
        ((radius, ratio, offset, angle, plane)) = (
            prop::radius(1.0..=8.0),
            prop::finite_f64(0.05..=0.95),
            prop::finite_f64(0.1..=0.7),
            prop::finite_f64(0.25..=TAU),
            prop::frame(),
        ) => {
            let major = radius * ratio;
            let a = real::acos(-ratio) - offset;
            let start = point(major, radius, a);
            let profile = Profile {
                plane,
                outer: ProfileLoop::Path {
                    start,
                    segments: vec![
                        ProfileSegment::ArcTo {
                            via: point(major, radius, PI),
                            to: point(major, radius, -a),
                        },
                        ProfileSegment::LineTo(start),
                    ],
                },
                holes: Vec::new(),
            };
            let result = revolve(&mut Model::default(), &profile, axis(&plane), angle);
            prop_assert!(
                matches!(
                    result,
                    Err(OpError::Degenerate {
                        reason: Reason::Sweep(SweepReason::ProfileCrossesAxis),
                        ..
                    })
                ),
                "expected ProfileCrossesAxis, got {result:?}"
            );
            Ok(())
        }
}
