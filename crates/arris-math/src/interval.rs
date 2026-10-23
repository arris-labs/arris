//! Closed parameter intervals.

use core::fmt;

/// A closed interval `[lo, hi]` with `lo ≤ hi`, neither end NaN. Either end
/// may be infinite: a line's domain is [`Interval::REAL`]. An interval may
/// be longer than a curve's period — an edge range on a periodic curve may
/// cross the period (`docs/DATA-MODEL.md` §Topology).
///
/// ```
/// use arris_math::Interval;
///
/// let i = Interval::new(1.0, 3.0).unwrap();
/// assert!(i.contains(3.0));
/// assert_eq!(i.clamp(5.0), 3.0);
/// assert_eq!(i.lerp(0.5), 2.0);
/// assert!(i.overlaps(&Interval::new(3.0, 4.0).unwrap()));
/// assert!(Interval::new(3.0, 1.0).is_err());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(try_from = "IntervalRepr"))]
pub struct Interval {
    lo: f64,
    hi: f64,
}

/// The wire form of an [`Interval`]: the two ends, validated by
/// [`Interval::new`] on the way in so a decoded interval is one that could
/// have been built.
#[cfg(feature = "serde")]
#[derive(serde::Deserialize)]
struct IntervalRepr {
    lo: f64,
    hi: f64,
}

#[cfg(feature = "serde")]
impl TryFrom<IntervalRepr> for Interval {
    type Error = IntervalError;

    fn try_from(r: IntervalRepr) -> Result<Self, IntervalError> {
        Interval::new(r.lo, r.hi)
    }
}

/// Why two numbers are not an [`Interval`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IntervalError {
    /// `lo > hi`.
    Reversed {
        /// The lower end given.
        lo: f64,
        /// The upper end given.
        hi: f64,
    },
    /// An end is NaN.
    NotANumber,
}

impl fmt::Display for IntervalError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IntervalError::Reversed { lo, hi } => write!(f, "interval [{lo}, {hi}] is reversed"),
            IntervalError::NotANumber => write!(f, "interval end is NaN"),
        }
    }
}

impl std::error::Error for IntervalError {}

impl Interval {
    /// The whole real line, `[−∞, +∞]`: the domain of a line or a plane's
    /// parameter.
    pub const REAL: Interval = Interval {
        lo: f64::NEG_INFINITY,
        hi: f64::INFINITY,
    };
    /// `[0, 1]`.
    pub const UNIT: Interval = Interval { lo: 0.0, hi: 1.0 };
    /// `[0, 2π]`: one turn of an angular parameter, both ends included —
    /// the closed fundamental interval of every periodic direction.
    pub const TURN: Interval = Interval {
        lo: 0.0,
        hi: core::f64::consts::TAU,
    };

    /// `[lo, hi]`; an error when `lo > hi` or either is NaN.
    pub fn new(lo: f64, hi: f64) -> Result<Self, IntervalError> {
        if lo.is_nan() || hi.is_nan() {
            return Err(IntervalError::NotANumber);
        }
        if lo > hi {
            return Err(IntervalError::Reversed { lo, hi });
        }
        Ok(Interval { lo, hi })
    }

    /// The lower end.
    pub const fn lo(&self) -> f64 {
        self.lo
    }

    /// The upper end.
    pub const fn hi(&self) -> f64 {
        self.hi
    }

    /// `hi − lo`; infinite for an unbounded interval, zero for a point.
    pub fn length(&self) -> f64 {
        self.hi - self.lo
    }

    /// `true` when both ends are finite.
    pub fn is_bounded(&self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
    }

    /// `(lo + hi) / 2`; NaN for [`Interval::REAL`], where no midpoint
    /// exists.
    pub fn midpoint(&self) -> f64 {
        self.lerp(0.5)
    }

    /// `lo ≤ t ≤ hi`. Both ends are inside; NaN is not.
    pub fn contains(&self, t: f64) -> bool {
        self.lo <= t && t <= self.hi
    }

    /// The nearest parameter inside: `t` itself when it is contained, the
    /// nearer end otherwise. NaN stays NaN.
    pub fn clamp(&self, t: f64) -> f64 {
        if t < self.lo {
            self.lo
        } else if t > self.hi {
            self.hi
        } else {
            t
        }
    }

    /// `lo` at `s = 0`, `hi` at `s = 1`, both exactly, linear between and
    /// beyond. Bounded intervals only; an infinite end yields an infinite
    /// or NaN result.
    pub fn lerp(&self, s: f64) -> f64 {
        (1.0 - s) * self.lo + s * self.hi
    }

    /// `n` parameters evenly over the interval, both ends included; the
    /// midpoint alone when `n` is one or zero. Bounded intervals only.
    ///
    /// ```
    /// use arris_math::Interval;
    ///
    /// let i = Interval::new(0.0, 1.0).unwrap();
    /// assert_eq!(i.samples(3), vec![0.0, 0.5, 1.0]);
    /// assert_eq!(i.samples(1), vec![0.5]);
    /// ```
    pub fn samples(&self, n: usize) -> Vec<f64> {
        if n <= 1 {
            return vec![self.midpoint()];
        }
        (0..n)
            .map(|i| self.lerp(i as f64 / (n - 1) as f64))
            .collect()
    }

    /// `true` when the two closed intervals share at least one point:
    /// touching at an end counts.
    pub fn overlaps(&self, other: &Interval) -> bool {
        self.lo <= other.hi && other.lo <= self.hi
    }

    /// The common part, or `None` when they do not [`overlap`](Self::overlaps).
    pub fn intersection(&self, other: &Interval) -> Option<Interval> {
        self.overlaps(other).then(|| Interval {
            lo: self.lo.max(other.lo),
            hi: self.hi.min(other.hi),
        })
    }

    /// The smallest interval containing both.
    pub fn hull(&self, other: &Interval) -> Interval {
        Interval {
            lo: self.lo.min(other.lo),
            hi: self.hi.max(other.hi),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn iv(lo: f64, hi: f64) -> Interval {
        Interval::new(lo, hi).unwrap()
    }

    #[test]
    fn construction_rejects_reversed_and_nan() {
        assert_eq!(
            Interval::new(2.0, 1.0),
            Err(IntervalError::Reversed { lo: 2.0, hi: 1.0 })
        );
        assert_eq!(Interval::new(f64::NAN, 1.0), Err(IntervalError::NotANumber));
        assert!(Interval::new(1.0, 1.0).is_ok());
        assert!(Interval::new(f64::NEG_INFINITY, 0.0).is_ok());
    }

    #[test]
    fn clamp_and_contains() {
        let i = iv(-1.0, 2.0);
        assert!(i.contains(-1.0) && i.contains(2.0) && i.contains(0.5));
        assert!(!i.contains(-1.0000001) && !i.contains(f64::NAN));
        assert_eq!(i.clamp(-5.0), -1.0);
        assert_eq!(i.clamp(5.0), 2.0);
        assert_eq!(i.clamp(0.25), 0.25);
        assert!(i.clamp(f64::NAN).is_nan());
        assert_eq!(Interval::REAL.clamp(1e300), 1e300);
        assert!(Interval::REAL.contains(f64::INFINITY));
    }

    #[test]
    fn lerp_hits_the_ends_exactly() {
        let i = iv(0.1, 0.7);
        assert_eq!(i.lerp(0.0), 0.1);
        assert_eq!(i.lerp(1.0), 0.7);
        assert!((i.midpoint() - 0.4).abs() <= f64::EPSILON);
        assert_eq!(i.length(), 0.7 - 0.1);
        assert!(i.is_bounded() && !Interval::REAL.is_bounded());
        assert!(Interval::REAL.midpoint().is_nan());
    }

    #[test]
    fn overlaps_intersection_hull() {
        let a = iv(0.0, 1.0);
        let b = iv(1.0, 2.0);
        let c = iv(1.5, 3.0);
        assert!(a.overlaps(&b) && b.overlaps(&a));
        assert!(!a.overlaps(&c) && !c.overlaps(&a));
        assert_eq!(a.intersection(&b), Some(iv(1.0, 1.0)));
        assert_eq!(a.intersection(&c), None);
        assert_eq!(b.intersection(&c), Some(iv(1.5, 2.0)));
        assert_eq!(a.hull(&c), iv(0.0, 3.0));
        assert_eq!(Interval::REAL.intersection(&c), Some(c));
        assert_eq!(Interval::UNIT, a);
        assert_eq!(Interval::TURN.hi(), core::f64::consts::TAU);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_round_trips_and_rejects_a_reversed_interval() {
        let i = iv(-1.5, 2.25);
        let text = serde_json::to_string(&i).unwrap();
        assert_eq!(text, r#"{"lo":-1.5,"hi":2.25}"#);
        let back: Interval = serde_json::from_str(&text).unwrap();
        assert_eq!(back, i);
        assert!(serde_json::from_str::<Interval>(r#"{"lo":2.0,"hi":1.0}"#).is_err());
    }
}
