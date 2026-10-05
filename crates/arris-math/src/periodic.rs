//! Periodic parameters: how a parameter of a closed curve or surface is
//! wrapped, shifted and differenced. The period always comes from the
//! entity (`Curve::period`, `Surface::period`); nothing here assumes `2π`
//! except [`wrap_angle`], which says so (`docs/DATA-MODEL.md` §Geometry).

use crate::{Interval, Vec2};

/// `t` moved by whole periods into `[lo, lo + period)`. A value already in
/// that range comes back unchanged, bit for bit (no add-and-subtract of
/// `lo`), and a result that rounds up to `lo + period` becomes `lo` — the
/// same point on the circle, and inside the range. A non-finite `t` comes
/// back unchanged.
///
/// ```
/// use arris_math::wrap_into;
///
/// assert_eq!(wrap_into(7.0, 2.0, 3.0), 4.0);
/// // Exactly at `lo + period` is the start of the next period.
/// assert_eq!(wrap_into(5.0, 2.0, 3.0), 2.0);
/// // A period that is not a turn.
/// assert_eq!(wrap_into(-0.5, 0.0, 1.5), 1.0);
/// // A tiny negative value whose sum with the period rounds up to it.
/// assert_eq!(wrap_into(-1e-300, 0.0, 3.0), 0.0);
/// ```
pub fn wrap_into(t: f64, lo: f64, period: f64) -> f64 {
    if !t.is_finite() {
        return t;
    }
    let w = if t >= lo && t - lo < period {
        t
    } else {
        lo + (t - lo).rem_euclid(period)
    };
    if w - lo >= period { lo } else { w }
}

/// An angle moved into `[0, 2π)`: what a periodic curve's or surface's
/// parameter is reported in (`docs/DATA-MODEL.md` §Conventions). A
/// negative angle whose sum with `2π` rounds up to `2π` becomes `0` —
/// the same point on the circle, and inside the domain. A non-finite
/// angle comes back unchanged.
///
/// ```
/// use arris_math::wrap_angle;
/// use core::f64::consts::TAU;
///
/// assert_eq!(wrap_angle(0.0), 0.0);
/// assert_eq!(wrap_angle(-1.0), TAU - 1.0);
/// assert_eq!(wrap_angle(-1e-300), 0.0);
/// assert_eq!(wrap_angle(TAU + 1.0), 1.0);
/// ```
pub fn wrap_angle(t: f64) -> f64 {
    wrap_into(t, 0.0, core::f64::consts::TAU)
}

/// A difference moved into `[−period / 2, period / 2)`, by wrapping
/// `x + period / 2` into `[0, period)` and moving back. Unlike
/// [`wrap_signed`] it can return a value one rounding from `x` when `x`
/// is already in range, because of the add and subtract of the half
/// period; callers that were written against it keep that rounding.
///
/// ```
/// use arris_math::wrap_offset;
/// use core::f64::consts::{PI, TAU};
///
/// assert_eq!(wrap_offset(0.0, TAU), 0.0);
/// // Exactly half a period goes to the lower end.
/// assert_eq!(wrap_offset(PI, TAU), -PI);
/// assert!((wrap_offset(TAU + 0.25, TAU) - 0.25).abs() < 1e-15);
/// // A period that is not a turn.
/// assert_eq!(wrap_offset(1.0, 1.0), 0.0);
/// ```
pub fn wrap_offset(x: f64, period: f64) -> f64 {
    let half = 0.5 * period;
    wrap_into(x + half, 0.0, period) - half
}

/// The whole-period shift that moves `x` nearest to `target`: `k·period`
/// with `k` the integer nearest `(target − x) / period` (halves away from
/// zero). Adding it to `x` is the translate of `x` that lies closest to
/// `target`; it is `0.0` when `x` already is.
///
/// ```
/// use arris_math::shift_nearest;
/// use core::f64::consts::TAU;
///
/// assert_eq!(shift_nearest(0.5, 0.5 + TAU, TAU), TAU);
/// assert_eq!(shift_nearest(0.5, 0.5 - 2.0 * TAU, TAU), -2.0 * TAU);
/// assert_eq!(shift_nearest(1.0, 1.4, TAU), 0.0);
/// // A period that is not a turn.
/// assert_eq!(shift_nearest(0.0, 7.0, 3.0), 6.0);
/// ```
pub fn shift_nearest(x: f64, target: f64, period: f64) -> f64 {
    ((target - x) / period).round() * period
}

/// `d` rounded to whole periods in each periodic parameter of a surface
/// (`Surface::period`), zero in the others: the translate in (u, v) that
/// carries a point by `d` back to the nearest copy of itself.
///
/// ```
/// use arris_math::{Vec2, shift_nearest_uv};
/// use core::f64::consts::TAU;
///
/// let by = shift_nearest_uv(Vec2::new(TAU + 0.1, 5.0), [Some(TAU), None]);
/// assert_eq!(by, Vec2::new(TAU, 0.0));
/// ```
pub fn shift_nearest_uv(d: Vec2, period: [Option<f64>; 2]) -> Vec2 {
    let round = |x: f64, p: Option<f64>| p.map_or(0.0, |p| shift_nearest(0.0, x, p));
    Vec2::new(round(d.x, period[0]), round(d.y, period[1]))
}

/// A difference moved into `[−period / 2, period / 2]`: `d` less the
/// whole periods nearest it. A half period goes to whichever end the
/// rounding of [`shift_nearest`] sends it, so the ends are not a promise.
///
/// ```
/// use arris_math::wrap_signed;
/// use core::f64::consts::TAU;
///
/// assert_eq!(wrap_signed(0.5, TAU), 0.5);
/// assert!((wrap_signed(TAU - 0.5, TAU) + 0.5).abs() < 1e-15);
/// assert_eq!(wrap_signed(-4.0, 3.0), -1.0);
/// ```
pub fn wrap_signed(d: f64, period: f64) -> f64 {
    d - shift_nearest(0.0, d, period)
}

/// `t` moved by whole periods into `range` when it can be; a parameter of
/// a non-periodic entity (`period` of `None`) only if it already lies in
/// `range`. The shift is the smallest one that brings `t` to or above
/// `range.lo()`, so the answer is the first translate at or after the
/// start, and `None` when that overshoots `range.hi()`.
///
/// ```
/// use arris_math::{Interval, shift_into_range};
///
/// let range = Interval::new(1.0, 2.0).unwrap();
/// assert_eq!(shift_into_range(range, 1.5 + 6.0, Some(3.0)), Some(1.5));
/// assert_eq!(shift_into_range(range, 0.5, Some(3.0)), None);
/// assert_eq!(shift_into_range(range, 1.5, None), Some(1.5));
/// assert_eq!(shift_into_range(range, 2.5, None), None);
/// ```
pub fn shift_into_range(range: Interval, t: f64, period: Option<f64>) -> Option<f64> {
    match period {
        Some(p) => {
            let k = ((range.lo() - t) / p).ceil();
            let shifted = t + k * p;
            (shifted <= range.hi()).then_some(shifted)
        }
        None => range.contains(t).then_some(t),
    }
}

/// The end of one whole period from `lo`: `lo + period`, stepped down to
/// the representable value below when that sum rounds up, so that
/// `end - lo <= period` holds exactly. A closed edge spans one period and
/// no more (`docs/DATA-MODEL.md` §Invariants, E1), and for a
/// `lo` that is not a small multiple of the period the sum can round to
/// one unit in the last place too far; this is the range's construction,
/// not a tolerance. A non-finite argument, or a `period` that is not
/// positive, comes back as `lo + period`.
///
/// ```
/// use arris_math::period_end;
/// use core::f64::consts::TAU;
///
/// assert_eq!(period_end(0.0, TAU), TAU);
/// // A pave one unit in the last place below a full turn: the sum
/// // rounds up, and the end is stepped back to keep the turn one turn.
/// let lo = f64::from_bits(TAU.to_bits() - 1);
/// assert!(lo + TAU - lo > TAU);
/// assert!(period_end(lo, TAU) - lo <= TAU);
/// ```
pub fn period_end(lo: f64, period: f64) -> f64 {
    let mut end = lo + period;
    if !(end.is_finite() && period > 0.0) {
        return end;
    }
    while end - lo > period {
        end = f64::from_bits(end.to_bits() - 1);
    }
    end
}
