//! The transcendental functions, one implementation on every target.
//!
//! `f64::sin` and its kin are the platform's: glibc natively, wasi-libc or
//! Rust's `libm` port on `wasm32`. They disagree in the last place for some
//! inputs (`atan2(1.9422128270990565e-6, 1.271166962267458e-5)` is
//! `1.516171868280155e-1` under glibc and `1.5161718682801548e-1` under the
//! port), and a last-place difference in an arc's sweep is a different body:
//! other bytes, other measures. These functions all call the `libm` crate,
//! pure Rust and pinned exactly, so the same input gives the same bits on
//! every target and a determinism digest means the same on all of them.
//!
//! The workspace's `clippy.toml` bans the `std` methods, so a new call
//! cannot slip back (ADR-0064). Not wrapped, because IEEE 754 makes them
//! exact or correctly rounded everywhere: `sqrt`, `mul_add`, `abs`,
//! `floor`, `ceil`, `round`, `trunc`, `rem_euclid`, `min`, `max`,
//! `to_radians`, and float parsing and printing.
//!
//! ```
//! use arris_math::real;
//!
//! let (s, c) = real::sin_cos(0.0);
//! assert_eq!((s, c), (0.0, 1.0));
//! assert_eq!(
//!     real::atan2(1.9422128270990565e-6, 1.271166962267458e-5),
//!     1.5161718682801548e-1, // `libm`'s, on every target
//! );
//! ```

macro_rules! unary {
    ($($(#[$m:meta])* $name:ident => $f:ident;)*) => {$(
        $(#[$m])*
        #[inline]
        #[must_use]
        pub fn $name(x: f64) -> f64 {
            libm::$f(x)
        }
    )*};
}

unary! {
    /// Sine of `x` radians.
    sin => sin;
    /// Cosine of `x` radians.
    cos => cos;
    /// Tangent of `x` radians.
    tan => tan;
    /// Arcsine, in radians; NaN outside `[-1, 1]`.
    asin => asin;
    /// Arccosine, in radians; NaN outside `[-1, 1]`.
    acos => acos;
    /// Arctangent, in radians.
    atan => atan;
    /// Hyperbolic sine.
    sinh => sinh;
    /// Hyperbolic cosine.
    cosh => cosh;
    /// Hyperbolic tangent.
    tanh => tanh;
    /// Inverse hyperbolic sine.
    asinh => asinh;
    /// Inverse hyperbolic cosine; NaN below 1.
    acosh => acosh;
    /// Inverse hyperbolic tangent; NaN outside `(-1, 1)`.
    atanh => atanh;
    /// `e` to the power `x`.
    exp => exp;
    /// Two to the power `x`.
    exp2 => exp2;
    /// `e^x - 1`, accurate for small `x`.
    exp_m1 => expm1;
    /// Natural logarithm.
    ln => log;
    /// `ln(1 + x)`, accurate for small `x`.
    ln_1p => log1p;
    /// Base-2 logarithm.
    log2 => log2;
    /// Base-10 logarithm.
    log10 => log10;
    /// Cube root; defined for negative `x`.
    cbrt => cbrt;
}

/// Sine and cosine of `x` radians, as `(sin, cos)`.
#[inline]
#[must_use]
pub fn sin_cos(x: f64) -> (f64, f64) {
    libm::sincos(x)
}

/// The angle of the point `(x, y)` from the positive x axis, in `(-π, π]`.
/// Note the argument order: `y` first, as `f64::atan2`.
#[inline]
#[must_use]
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}

/// `sqrt(x² + y²)` without intermediate overflow.
#[inline]
#[must_use]
pub fn hypot(x: f64, y: f64) -> f64 {
    libm::hypot(x, y)
}

/// `x` to the power `y`.
#[inline]
#[must_use]
pub fn powf(x: f64, y: f64) -> f64 {
    libm::pow(x, y)
}

/// The logarithm of `x` in `base`, `ln(x) / ln(base)`.
#[inline]
#[must_use]
pub fn log(x: f64, base: f64) -> f64 {
    libm::log(x) / libm::log(base)
}

/// The largest `f64` below the finite `x`: `f64::next_down`, which is
/// stable from Rust 1.86 where the workspace supports 1.85. Below `0.0` is
/// the smallest negative subnormal.
///
/// ```
/// use arris_math::real;
///
/// assert!(real::next_down(1.0) < 1.0);
/// assert_eq!(real::next_down(1.0), 1.0 - f64::EPSILON / 2.0);
/// assert_eq!(real::next_down(0.0), -f64::from_bits(1));
/// assert!(real::next_down(-1.0) < -1.0);
/// ```
#[inline]
#[must_use]
pub fn next_down(x: f64) -> f64 {
    if x == 0.0 {
        return -f64::from_bits(1);
    }
    let bits = x.to_bits();
    f64::from_bits(if x > 0.0 { bits - 1 } else { bits + 1 })
}

/// `x` to the integer power `n` by repeated squaring in a fixed order, so
/// the rounding is the same on every target and `powi(x, 2)` is `x * x`
/// bit for bit. A negative `n` is the reciprocal of the positive power.
#[inline]
#[must_use]
pub fn powi(x: f64, n: i32) -> f64 {
    let mut base = x;
    let mut rest = n.unsigned_abs();
    let mut acc = 1.0;
    loop {
        if rest & 1 == 1 {
            acc *= base;
        }
        rest >>= 1;
        if rest == 0 {
            break;
        }
        base *= base;
    }
    if n < 0 { 1.0 / acc } else { acc }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The backlog's one call (`arc-sweep-differs-on-wasm32`): glibc gives
    /// `1.516171868280155e-1` here, `libm` `1.5161718682801548e-1`, and
    /// this assertion holds the latter on every target.
    #[test]
    fn atan2_of_the_gear_arc_has_the_same_bits_everywhere() {
        let v = atan2(1.9422128270990565e-6, 1.271166962267458e-5);
        assert_eq!(v, 1.5161718682801548e-1);
    }

    #[test]
    fn powi_squares_exactly_and_inverts_negative_powers() {
        let x = 1.0000000000000002;
        assert_eq!(powi(x, 0), 1.0);
        assert_eq!(powi(x, 1), x);
        assert_eq!(powi(x, 2).to_bits(), (x * x).to_bits());
        assert_eq!(powi(x, 3).to_bits(), (x * (x * x)).to_bits());
        assert_eq!(powi(2.0, -3), 0.125);
        assert_eq!(powi(0.0, 0), 1.0);
        assert_eq!(powi(-2.0, i32::MIN + 1), 0.0);
    }
}
