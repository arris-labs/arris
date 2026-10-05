//! The periodic-parameter toolkit: the boundaries of each helper, and
//! the properties every caller relies on (seeded).

use arris_debug::prop::check;
use arris_math::{
    Interval, period_end, shift_into_range, shift_nearest, wrap_angle, wrap_into, wrap_offset,
    wrap_signed,
};
use core::f64::consts::{PI, TAU};
use proptest::prelude::*;

#[test]
fn a_value_at_the_end_of_the_period_starts_the_next() {
    assert_eq!(wrap_into(TAU, 0.0, TAU), 0.0);
    assert_eq!(wrap_into(5.0, 2.0, 3.0), 2.0);
    assert_eq!(wrap_into(2.0, 2.0, 3.0), 2.0);
}

#[test]
fn a_tiny_negative_value_does_not_leave_the_range() {
    assert_eq!(wrap_into(-1e-300, 0.0, TAU), 0.0);
    assert_eq!(wrap_into(-1e-300, 0.0, 1.5), 0.0);
    // Off zero: one step below `lo`, whose distance from `lo` the period
    // swallows, still lands inside the range.
    let lo = 11.0_f64;
    let below = f64::from_bits(lo.to_bits() - 1);
    let w = wrap_into(below, lo, 1.0e17);
    assert!(w >= lo && w - lo < 1.0e17, "{w}");
}

#[test]
fn a_period_that_is_not_a_turn_is_respected() {
    assert_eq!(wrap_into(7.5, 0.0, 2.5), 0.0);
    assert_eq!(wrap_into(-0.5, 0.0, 2.5), 2.0);
    assert_eq!(wrap_offset(2.0, 2.5), -0.5);
    assert_eq!(shift_nearest(0.0, 7.0, 2.5), 7.5);
}

#[test]
fn the_angle_wrap_is_the_toolkit_at_zero_and_a_turn() {
    for t in [
        -7.0,
        -TAU,
        -1.0,
        -0.0,
        0.0,
        1.0,
        PI,
        TAU - 1e-15,
        TAU,
        9.0,
        1e300,
    ] {
        assert_eq!(
            wrap_angle(t).to_bits(),
            wrap_into(t, 0.0, TAU).to_bits(),
            "{t}"
        );
    }
    assert!(wrap_angle(f64::NAN).is_nan());
    assert_eq!(wrap_angle(f64::INFINITY), f64::INFINITY);
}

#[test]
fn a_value_already_in_range_is_returned_bit_for_bit() {
    // `rem_euclid` of a value in range need not return it: the fast path
    // keeps the value, and the wrap of `x` about a shifted `lo` does not
    // add and subtract it.
    let (lo, period) = (0.1_f64 + 0.2, 0.7_f64);
    for k in 0..1000 {
        let t = lo + period * (k as f64 / 1000.0);
        if t - lo < period {
            assert_eq!(wrap_into(t, lo, period).to_bits(), t.to_bits());
        }
    }
}

#[test]
fn the_offset_wrap_and_the_signed_wrap_differ_in_rounding_and_at_a_half() {
    // Already in range: the signed wrap subtracts a zero shift and is
    // exact; the offset wrap adds and subtracts half a period.
    let x = 0.1;
    assert_eq!(wrap_signed(x, TAU).to_bits(), x.to_bits());
    assert_ne!(wrap_offset(x, TAU).to_bits(), x.to_bits());
    // A half period: the offset wrap sends it to the lower end, the
    // signed wrap to the other side of zero by the sign of the input.
    assert_eq!(wrap_offset(PI, TAU), -PI);
    assert_eq!(wrap_offset(-PI, TAU), -PI);
    assert_eq!(wrap_signed(PI, TAU), -PI);
    assert_eq!(wrap_signed(-PI, TAU), PI);
}

#[test]
fn a_range_is_reached_from_the_first_translate_at_or_above_its_start() {
    let range = Interval::new(1.0, 2.0).unwrap();
    assert_eq!(shift_into_range(range, 1.0, Some(3.0)), Some(1.0));
    assert_eq!(shift_into_range(range, 2.0 - 3.0, Some(3.0)), Some(2.0));
    assert_eq!(shift_into_range(range, 2.0 + 1e-9, Some(3.0)), None);
    assert_eq!(shift_into_range(range, 1.5 - 30.0, Some(3.0)), Some(1.5));
    assert_eq!(shift_into_range(range, 2.0, None), Some(2.0));
    assert_eq!(shift_into_range(range, 0.5, None), None);
}

#[test]
fn a_period_end_is_never_more_than_one_period_from_its_start() {
    let lo = f64::from_bits(TAU.to_bits() - 1);
    assert!(period_end(lo, TAU) - lo <= TAU);
    assert!(period_end(lo, 3.0) - lo <= 3.0);
}

fn parameters() -> impl Strategy<Value = (f64, f64, f64)> {
    (-1e3..1e3f64, -1e3..1e3f64, 0.1..20.0f64)
}

#[test]
fn a_wrapped_value_is_in_range_and_a_whole_number_of_periods_away() {
    check(parameters(), |(t, lo, period)| {
        let w = wrap_into(t, lo, period);
        prop_assert!(w >= lo && w - lo < period, "{w} outside [{lo}, +{period})");
        let k = (t - w) / period;
        prop_assert!((k - k.round()).abs() < 1e-9, "{t} -> {w} is {k} periods");
        prop_assert_eq!(wrap_into(w, lo, period).to_bits(), w.to_bits());
        Ok(())
    });
}

#[test]
fn a_shift_brings_a_value_within_half_a_period_of_its_target() {
    check(parameters(), |(x, target, period)| {
        let by = shift_nearest(x, target, period);
        prop_assert!(
            (x + by - target).abs() <= 0.5 * period * (1.0 + 1e-12) + 1e-9,
            "{x} + {by} is not near {target} (period {period})"
        );
        Ok(())
    });
}

#[test]
fn signed_wraps_stay_within_half_a_period() {
    check(parameters(), |(d, _, period)| {
        let slack = 0.5 * period * (1.0 + 1e-12) + 1e-9;
        prop_assert!(wrap_signed(d, period).abs() <= slack);
        prop_assert!(wrap_offset(d, period).abs() <= slack);
        Ok(())
    });
}

#[test]
fn a_surface_shift_rounds_only_the_periodic_parameters() {
    use arris_math::{Vec2, shift_nearest_uv};
    let by = shift_nearest_uv(Vec2::new(TAU + 0.1, 9.0), [Some(TAU), None]);
    assert_eq!(by, Vec2::new(TAU, 0.0));
    let by = shift_nearest_uv(Vec2::new(-0.1, -3.0 * 2.5 - 0.2), [Some(TAU), Some(2.5)]);
    assert_eq!(by, Vec2::new(0.0, -7.5));
}
