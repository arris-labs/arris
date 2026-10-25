//! Tensor-product polynomials in Bernstein form on `[0, 1]²`: the
//! product, the partial derivatives, de Casteljau's evaluation and
//! subdivision in either direction, and the isolation of the common
//! zeros of two of them, each certified alone in its box. A rational
//! surface patch substituted into a surface's implicit polynomial is one
//! of these (`crate::implicit`, `crate::trace_torus`), and what
//! `crate::bernstein` says of one variable holds of two: every operation
//! is a convex combination or a sum of products with positive factors,
//! and the coefficients bound the polynomial over the box.
//!
//! The isolation is subdivision with three tests on a box. It is
//! **excluded** when one polynomial's coefficients keep one sign beyond
//! their rounding floor: the polynomial has no zero in the closed box.
//! It is **certified** by the Krawczyk operator on the box grown by
//! [`MARGIN`] of its width on every side: with `[J]` the ranges of the
//! partial derivatives over the grown box `X`, `c` its middle, `Y` the
//! inverse of `[J]`'s middle and `K = c − Y·F(c) + (I − Y·[J])(X − c)`,
//! every zero in `X` is in `K`, and `K` inside `X` puts exactly one
//! there, a simple one (Krawczyk 1969; Moore, *A test for existence of
//! solutions to nonlinear systems*, SIAM J. Numer. Anal. 14, 1977;
//! Sherbrooke and Patrikalakis, *Computation of the solutions of
//! nonlinear polynomial systems*, CAGD 10, 1993, for the Bernstein
//! setting). Otherwise it is halved, along the direction the system
//! varies more along. The margin is what finds a zero
//! on the line between two boxes — a symmetric pose puts them there as a
//! rule — at the depth its neighbourhood is certified at, rather than at
//! the bottom: it is interior to both grown boxes, reported by both, and
//! two certified zeros one of which lies in the other's box are one zero
//! (`crate::bernstein_n::merge`).
//!
//! The asymmetry of `crate::bernstein` is kept: the isolation may return
//! a candidate too many and never a zero too few. What it cannot certify
//! — a zero that is not simple, a stretch where a polynomial is zero as
//! far as `f64` can tell, a cluster that `MAX_DEPTH` halvings each way
//! do not separate — comes back as an **uncertified** box that holds whatever
//! is there, touching ones merged into their hull. Zeros that are not
//! isolated at all, a whole curve of them, are `Continuum`: the boxes
//! alive at one depth double with every level instead of settling, and
//! [`MAX_FRONT`] ends it.

/// How far beyond each side a box is grown before it is certified, as a
/// fraction of its width. A ratio, not a tolerance: large enough that a
/// zero on a box's edge is well inside the grown box, small enough that
/// the de Casteljau extrapolation it takes — no longer a convex
/// combination — amplifies a coefficient's rounding by no more than
/// `(1 + 2·MARGIN)ⁿ`, under ten at degree ten.
pub(crate) const MARGIN: f64 = 0.125;

/// How many boxes may be alive at one depth. Isolated zeros keep a
/// handful of boxes each, whatever the depth; a curve of common zeros
/// doubles them with every level. A structural bound, not a tolerance:
/// no torus section measured has more than 110 alive on a patch, a tube
/// circle lying on the other surface among them (`crate::trace_torus`).
pub(crate) const MAX_FRONT: usize = 1024;

/// A polynomial is taken for zero over a box where its coefficients are
/// all within this many floors, while it takes one floor to exclude a
/// box. Any factor above one would do: with the same bound for both, a
/// box whose coefficients straddle the floor itself is neither, however
/// small it gets, and the boxes along the curve where the polynomial
/// *equals* its floor would be halved to the bottom for nothing. A
/// ratio, not a tolerance.
pub(crate) const FLAT: f64 = 2.0;

/// How many Newton steps polish a certified zero. Newton from inside a
/// certified box converges quadratically, so the count is never reached
/// by a step that still improves; it only ends a cycle of rounding.
pub(crate) const POLISH_STEPS: usize = 12;

/// The value of a one-variable polynomial at `s` by de Casteljau.
pub(crate) fn casteljau(c: &[f64], s: f64) -> f64 {
    let mut row = c.to_vec();
    for level in 1..row.len() {
        for i in 0..row.len() - level {
            row[i] = (1.0 - s) * row[i] + s * row[i + 1];
        }
    }
    row.first().copied().unwrap_or(0.0)
}

/// The two parts of a one-variable polynomial either side of `at`, the
/// sides of de Casteljau's triangle.
pub(crate) fn split(c: &[f64], at: f64) -> (Vec<f64>, Vec<f64>) {
    let n = c.len();
    let mut row = c.to_vec();
    let mut lower = Vec::with_capacity(n);
    let mut upper = vec![0.0; n];
    for level in 0..n {
        lower.push(row[0]);
        upper[n - 1 - level] = row[n - 1 - level];
        for i in 0..n - 1 - level {
            row[i] = (1.0 - at) * row[i] + at * row[i + 1];
        }
    }
    (lower, upper)
}

/// A closed range of reals, for the certificate. Its own rounding is not
/// directed outwards: it is a few `ε` of quantities the callers have
/// already widened by their rounding floors, which are hundreds of `ε`
/// of larger ones.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Range {
    pub(crate) lo: f64,
    pub(crate) hi: f64,
}

impl Range {
    pub(crate) fn of(values: impl Iterator<Item = f64>) -> Range {
        values.fold(
            Range {
                lo: f64::INFINITY,
                hi: f64::NEG_INFINITY,
            },
            |r, v| Range {
                lo: r.lo.min(v),
                hi: r.hi.max(v),
            },
        )
    }

    pub(crate) fn around(value: f64, by: f64) -> Range {
        Range {
            lo: value - by,
            hi: value + by,
        }
    }

    pub(crate) fn widened(self, by: f64) -> Range {
        Range {
            lo: self.lo - by,
            hi: self.hi + by,
        }
    }

    pub(crate) fn middle(self) -> f64 {
        0.5 * (self.lo + self.hi)
    }

    pub(crate) fn magnitude(self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }

    pub(crate) fn scaled(self, k: f64) -> Range {
        Range::of([k * self.lo, k * self.hi].into_iter())
    }

    pub(crate) fn plus(self, o: Range) -> Range {
        Range {
            lo: self.lo + o.lo,
            hi: self.hi + o.hi,
        }
    }

    pub(crate) fn minus(self, o: Range) -> Range {
        Range {
            lo: self.lo - o.hi,
            hi: self.hi - o.lo,
        }
    }
}
