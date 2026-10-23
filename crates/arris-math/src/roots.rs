//! Real roots of polynomials of degree at most four, with multiplicity,
//! and a Newton iteration that never leaves its bracket.
//!
//! The method is real-root isolation by the derivative: the real roots of
//! `p'` split the line into intervals on which `p` is monotone, so each
//! holds at most one root, found by bracketed Newton when the signs at
//! its ends differ; a critical point where `p` vanishes to rounding is a
//! multiple root, of multiplicity one more than its multiplicity in `p'`.
//! No discriminant is compared against zero, no complex arithmetic is
//! rounded back to the real line, and a double root is located as the
//! *simple* root of `p'` it is — to full precision, where a closed form
//! would lose half the digits.

use core::fmt;
use core::ops::Deref;

use crate::Interval;

/// Rounding slack for deciding that a polynomial *vanishes* at a point:
/// `|p(x)| ≤ POLYNOMIAL_ROUNDING · Σ|aₖ||x|ᵏ` is zero in floating point.
/// Horner's evaluation error is under `8ε` times that sum for degree four,
/// the coefficients a caller expanded from factors carry about as much
/// again, amplified by up to `2⁴` where terms of opposite sign cancelled,
/// and the rest is slack. Like [`crate::RELATIVE_ROUNDING`] it is a statement
/// about `f64`, never a geometric tolerance: two roots closer than the
/// rounding of their polynomial allows are one root of multiplicity two,
/// because nothing in `f64` can tell them apart.
pub const POLYNOMIAL_ROUNDING: f64 = 256.0 * f64::EPSILON;

/// The most iterations a bracketed Newton runs before returning its
/// current bracket: bisection alone halves a bracket of `2^1024` width to
/// an ulp in fewer, and Newton inside a bracket only narrows faster.
const MAX_ITERATIONS: usize = 1100;

/// A real root and how many times it is one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Root {
    /// Where.
    pub value: f64,
    /// How many times: `1` for a simple root, `2` where `p` and `p'`
    /// vanish together, and so on.
    pub multiplicity: usize,
}

/// The real roots of a polynomial of degree at most four, ascending by
/// value, each once with its multiplicity. Derefs to a slice.
///
/// ```
/// use arris_math::roots::quadratic;
///
/// let r = quadratic(1.0, -3.0, 2.0).unwrap(); // (x − 1)(x − 2)
/// assert_eq!(r.len(), 2);
/// assert!((r[0].value - 1.0).abs() < 1e-15 && (r[1].value - 2.0).abs() < 1e-15);
/// let double = quadratic(1.0, -2.0, 1.0).unwrap(); // (x − 1)²
/// assert_eq!(double.len(), 1);
/// assert_eq!(double[0].multiplicity, 2);
/// assert!(quadratic(1.0, 0.0, 1.0).unwrap().is_empty()); // x² + 1
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Roots {
    len: usize,
    items: [Root; 4],
}

impl Roots {
    const EMPTY: Roots = Roots {
        len: 0,
        items: [Root {
            value: 0.0,
            multiplicity: 0,
        }; 4],
    };

    /// Appends a root. Total multiplicity never exceeds the degree, so
    /// the array never fills; a fifth root would be a bug and is dropped
    /// rather than panicked on.
    fn push(&mut self, root: Root) {
        if self.len < self.items.len() {
            self.items[self.len] = root;
            self.len += 1;
        }
    }

    /// The roots as a slice.
    pub fn as_slice(&self) -> &[Root] {
        &self.items[..self.len]
    }

    /// The sum of the multiplicities: how many roots there are counted
    /// with multiplicity.
    pub fn total_multiplicity(&self) -> usize {
        self.as_slice().iter().map(|r| r.multiplicity).sum()
    }
}

impl Default for Roots {
    fn default() -> Self {
        Roots::EMPTY
    }
}

impl Deref for Roots {
    type Target = [Root];

    fn deref(&self) -> &[Root] {
        self.as_slice()
    }
}

/// Why a root query has no answer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RootError {
    /// A coefficient, a bracket end, a tolerance or a function value is
    /// not finite (a negative tolerance counts), or the coefficients are
    /// so far apart that the root bound overflows `f64`.
    NonFinite,
    /// Every coefficient is zero: every `x` is a root.
    Zero,
    /// The function has the same sign at both ends of the bracket, so the
    /// bracket is no bracket.
    NoSignChange,
}

impl fmt::Display for RootError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            RootError::NonFinite => "root finding on a non-finite input",
            RootError::Zero => "every coefficient is zero: every point is a root",
            RootError::NoSignChange => "the function has the same sign at both bracket ends",
        })
    }
}

impl std::error::Error for RootError {}

/// The real roots of `a x² + b x + c`, ascending, with multiplicity. A
/// zero `a` lowers the degree; a double root is reported once with
/// multiplicity two.
///
/// Errors: [`RootError::NonFinite`] for a non-finite coefficient,
/// [`RootError::Zero`] when all three are zero.
pub fn quadratic(a: f64, b: f64, c: f64) -> Result<Roots, RootError> {
    real_roots(&[c, b, a])
}

/// The real roots of `a x³ + b x² + c x + d`, ascending, with
/// multiplicity. A zero `a` lowers the degree. Errors as [`quadratic`].
///
/// ```
/// use arris_math::roots::cubic;
///
/// let r = cubic(1.0, -6.0, 11.0, -6.0).unwrap(); // (x − 1)(x − 2)(x − 3)
/// let values: Vec<f64> = r.iter().map(|r| r.value).collect();
/// assert!(values.iter().zip([1.0, 2.0, 3.0]).all(|(a, b)| (a - b).abs() < 1e-14));
/// ```
pub fn cubic(a: f64, b: f64, c: f64, d: f64) -> Result<Roots, RootError> {
    real_roots(&[d, c, b, a])
}

/// The real roots of `a x⁴ + b x³ + c x² + d x + e`, ascending, with
/// multiplicity. A zero `a` lowers the degree. Errors as [`quadratic`].
///
/// ```
/// use arris_math::roots::quartic;
///
/// // (x² − 1)(x² + 1): two real roots, one complex pair.
/// let r = quartic(1.0, 0.0, 0.0, 0.0, -1.0).unwrap();
/// assert_eq!(r.len(), 2);
/// assert!((r[0].value + 1.0).abs() < 1e-15 && (r[1].value - 1.0).abs() < 1e-15);
/// ```
pub fn quartic(a: f64, b: f64, c: f64, d: f64, e: f64) -> Result<Roots, RootError> {
    real_roots(&[e, d, c, b, a])
}

/// A root of `f` inside `bracket`, by Newton steps that fall back to
/// bisection whenever a step would leave the part of the bracket that
/// still holds the sign change; the result never leaves the bracket, and
/// the iteration converges for any continuous `f` that changes sign.
/// Stops when the bracket is narrower than `tol` — or, for a zero `tol`,
/// than rounding — or when `f` is exactly zero. `df` is the derivative;
/// where it is zero or the step misbehaves, the step is a bisection.
///
/// Errors: [`RootError::NoSignChange`] when `f` has the same sign at both
/// ends (an end where `f` is exactly zero is returned as the root),
/// [`RootError::NonFinite`] for a non-finite end, tolerance or value.
///
/// ```
/// use arris_math::Interval;
/// use arris_math::roots::newton_in_interval;
///
/// let f = |x: f64| x * x * x - 2.0;
/// let df = |x: f64| 3.0 * x * x;
/// let root = newton_in_interval(f, df, Interval::new(0.0, 4.0).unwrap(), 0.0).unwrap();
/// assert!((root - 2f64.cbrt()).abs() < 1e-15);
/// ```
pub fn newton_in_interval(
    f: impl Fn(f64) -> f64,
    df: impl Fn(f64) -> f64,
    bracket: Interval,
    tol: f64,
) -> Result<f64, RootError> {
    if !(bracket.is_bounded() && tol.is_finite() && tol >= 0.0) {
        return Err(RootError::NonFinite);
    }
    let (lo, hi) = (bracket.lo(), bracket.hi());
    let (f_lo, f_hi) = (f(lo), f(hi));
    if !(f_lo.is_finite() && f_hi.is_finite()) {
        return Err(RootError::NonFinite);
    }
    if f_lo == 0.0 {
        return Ok(lo);
    }
    if f_hi == 0.0 {
        return Ok(hi);
    }
    if (f_lo < 0.0) == (f_hi < 0.0) {
        return Err(RootError::NoSignChange);
    }
    bracketed_newton(&f, &df, lo, hi, f_lo < 0.0, tol)
}

/// Newton with a bisection guard on `[lo, hi]`, where `f(lo)` is negative
/// iff `lo_negative` and `f(hi)` has the other sign.
fn bracketed_newton(
    f: &dyn Fn(f64) -> f64,
    df: &dyn Fn(f64) -> f64,
    mut lo: f64,
    mut hi: f64,
    lo_negative: bool,
    tol: f64,
) -> Result<f64, RootError> {
    let mut x = 0.5 * (lo + hi);
    for _ in 0..MAX_ITERATIONS {
        let fx = f(x);
        if !fx.is_finite() {
            return Err(RootError::NonFinite);
        }
        if fx == 0.0 {
            return Ok(x);
        }
        if (fx < 0.0) == lo_negative {
            lo = x;
        } else {
            hi = x;
        }
        // A zero `tol` runs to the ulp: a bracket one rounding step wide
        // holds nothing Newton could still improve.
        let width = hi - lo;
        if width <= tol.max(f64::EPSILON * lo.abs().max(hi.abs())) {
            return Ok(x);
        }
        let d = df(x);
        let newton = x - fx / d;
        let next = if d != 0.0 && newton > lo && newton < hi {
            newton
        } else {
            0.5 * (lo + hi)
        };
        if next == x {
            return Ok(x);
        }
        x = next;
    }
    Ok(x)
}

/// Horner's evaluation of `c[0] + c[1] x + …`.
fn eval(c: &[f64], x: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, &k| acc * x + k)
}

/// `Σ |cₖ| |x|ᵏ`: the magnitude against which `p(x)` is zero to rounding.
fn eval_abs(c: &[f64], x: f64) -> f64 {
    let x = x.abs();
    c.iter().rev().fold(0.0, |acc, &k| acc * x + k.abs())
}

/// `1 + max |cₖ / cₙ|`: every real root has a smaller magnitude, and the
/// leading term decides the sign of `p` beyond it.
fn cauchy_bound(c: &[f64]) -> Result<f64, RootError> {
    let lead = c[c.len() - 1];
    let ratio = c[..c.len() - 1]
        .iter()
        .map(|k| (k / lead).abs())
        .fold(0.0, f64::max);
    let bound = 1.0 + ratio;
    if bound.is_finite() && eval_abs(c, bound).is_finite() {
        Ok(bound)
    } else {
        Err(RootError::NonFinite)
    }
}

/// The real roots of the polynomial with ascending coefficients `c`, of
/// degree at most four; leading zeros lower the degree.
fn real_roots(c: &[f64]) -> Result<Roots, RootError> {
    if c.iter().any(|k| !k.is_finite()) {
        return Err(RootError::NonFinite);
    }
    let degree = match c.iter().rposition(|&k| k != 0.0) {
        None => return Err(RootError::Zero),
        Some(0) => return Ok(Roots::EMPTY),
        Some(n) => n,
    };
    let c = &c[..=degree];
    let mut out = Roots::EMPTY;
    if degree == 1 {
        out.push(Root {
            value: -c[0] / c[1],
            multiplicity: 1,
        });
        return Ok(out);
    }
    let mut derivative = [0.0; 4];
    for (k, d) in derivative.iter_mut().enumerate().take(degree) {
        *d = c[k + 1] * (k + 1) as f64;
    }
    let critical = real_roots(&derivative[..degree])?;
    let bound = cauchy_bound(c)?;
    let lead_negative = c[degree] < 0.0;
    // The sign of p at −bound is the leading term's, flipped for odd degree.
    let mut lo = -bound;
    let mut lo_negative = lead_negative != (degree % 2 == 1);
    // After a multiple root the neighbouring monotone interval holds only
    // that root's rounding twin, so it is not searched.
    let mut skip = false;
    let f = |x| eval(c, x);
    let df = |x| eval(&derivative[..degree], x);
    for cp in critical.iter() {
        let x = cp.value;
        if x <= -bound || x >= bound {
            continue;
        }
        let fx = f(x);
        if fx.abs() <= POLYNOMIAL_ROUNDING * eval_abs(c, x) {
            out.push(Root {
                value: x,
                multiplicity: cp.multiplicity + 1,
            });
            lo = x;
            skip = true;
            continue;
        }
        let negative = fx < 0.0;
        if !skip && negative != lo_negative {
            out.push(Root {
                value: bracketed_newton(&f, &df, lo, x, lo_negative, 0.0)?,
                multiplicity: 1,
            });
        }
        lo = x;
        lo_negative = negative;
        skip = false;
    }
    if !skip && lead_negative != lo_negative {
        out.push(Root {
            value: bracketed_newton(&f, &df, lo, bound, lo_negative, 0.0)?,
            multiplicity: 1,
        });
    }
    Ok(out)
}

/// Where a [`golden_section`] search ended: the bracket it narrowed to and
/// its two interior probes, each with the value of the function there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bracket {
    /// The bracket's low end.
    pub lo: f64,
    /// The bracket's high end.
    pub hi: f64,
    /// The lower interior probe and the function's value at it.
    pub low_probe: (f64, f64),
    /// The upper interior probe and the function's value at it.
    pub high_probe: (f64, f64),
}

/// The bracket `[a, b]` narrowed by `steps` golden-section steps toward the
/// minimum of `f`, which the caller guarantees is unimodal there. One
/// evaluation per step: the surviving interior probe is reused. A tie
/// keeps the lower side. Each step shrinks the bracket by `0.618`, so
/// `steps` is an iteration count, never a tolerance; the caller picks its
/// ending (the midpoint, the better probe, the better of those and the
/// ends). A maximum is a minimum of `-f`.
///
/// ```
/// use arris_math::roots::golden_section;
///
/// let end = golden_section(|x| (x - 0.3) * (x - 0.3), -1.0, 2.0, 60);
/// assert!((0.5 * (end.lo + end.hi) - 0.3).abs() < 1e-7);
/// assert!(end.lo <= end.low_probe.0 && end.high_probe.0 <= end.hi);
/// ```
pub fn golden_section(f: impl Fn(f64) -> f64, mut a: f64, mut b: f64, steps: usize) -> Bracket {
    let ratio = 0.5 * (5f64.sqrt() - 1.0);
    let (mut x1, mut x2) = (b - ratio * (b - a), a + ratio * (b - a));
    let (mut f1, mut f2) = (f(x1), f(x2));
    for _ in 0..steps {
        if f1 <= f2 {
            (b, x2, f2) = (x2, x1, f1);
            x1 = b - ratio * (b - a);
            f1 = f(x1);
        } else {
            (a, x1, f1) = (x1, x2, f2);
            x2 = a + ratio * (b - a);
            f2 = f(x2);
        }
    }
    Bracket {
        lo: a,
        hi: b,
        low_probe: (x1, f1),
        high_probe: (x2, f2),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn golden_section_finds_a_minimum_and_keeps_the_lower_side_on_a_tie() {
        let end = golden_section(|x| (x - 0.3).abs(), -1.0, 2.0, 80);
        assert!((0.5 * (end.lo + end.hi) - 0.3).abs() < 1e-12);
        assert_eq!(end.low_probe.1, (end.low_probe.0 - 0.3).abs());
        // A flat function ties every comparison: the high end is dropped.
        let flat = golden_section(|_| 1.0, 0.0, 1.0, 5);
        assert_eq!(flat.lo, 0.0);
        assert!(flat.hi < 0.2);
        // Zero steps are the two first probes.
        let none = golden_section(|x| x, 0.0, 1.0, 0);
        assert_eq!((none.lo, none.hi), (0.0, 1.0));
    }

    #[test]
    fn leading_zeros_lower_the_degree() {
        let r = quartic(0.0, 0.0, 1.0, 0.0, -4.0).unwrap();
        assert_eq!(r.len(), 2);
        assert_eq!((r[0].value, r[1].value), (-2.0, 2.0));
        let linear = cubic(0.0, 0.0, 2.0, -1.0).unwrap();
        assert_eq!(
            linear.as_slice(),
            &[Root {
                value: 0.5,
                multiplicity: 1
            }]
        );
        assert!(quadratic(0.0, 0.0, 3.0).unwrap().is_empty());
    }

    #[test]
    fn degenerate_inputs_are_errors() {
        assert_eq!(quadratic(0.0, 0.0, 0.0), Err(RootError::Zero));
        assert_eq!(quadratic(f64::NAN, 1.0, 0.0), Err(RootError::NonFinite));
        assert_eq!(
            quartic(1e-300, 0.0, 0.0, 0.0, 1e300),
            Err(RootError::NonFinite)
        );
        let f = |x: f64| x * x + 1.0;
        assert_eq!(
            newton_in_interval(f, |x| 2.0 * x, Interval::UNIT, 0.0),
            Err(RootError::NoSignChange)
        );
        assert_eq!(
            newton_in_interval(f, |x| 2.0 * x, Interval::REAL, 0.0),
            Err(RootError::NonFinite)
        );
    }

    #[test]
    fn a_triple_root_is_found_once() {
        // (x − 2)³ = x³ − 6x² + 12x − 8
        let r = cubic(1.0, -6.0, 12.0, -8.0).unwrap();
        assert_eq!(r.len(), 1);
        assert_eq!(r[0].multiplicity, 3);
        assert!((r[0].value - 2.0).abs() < 1e-14);
        assert_eq!(r.total_multiplicity(), 3);
    }

    #[test]
    fn newton_returns_an_end_that_is_exactly_a_root() {
        let f = |x: f64| x - 1.0;
        let bracket = Interval::new(1.0, 3.0).unwrap();
        assert_eq!(newton_in_interval(f, |_| 1.0, bracket, 0.0), Ok(1.0));
        let bracket = Interval::new(-1.0, 1.0).unwrap();
        assert_eq!(newton_in_interval(f, |_| 1.0, bracket, 0.0), Ok(1.0));
    }
}
