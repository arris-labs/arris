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
//! ([`merge`]).
//!
//! The asymmetry of `crate::bernstein` is kept: the isolation may return
//! a candidate too many and never a zero too few. What it cannot certify
//! — a zero that is not simple, a stretch where a polynomial is zero as
//! far as `f64` can tell, a cluster that [`MAX_DEPTH`] halvings each way
//! do not separate — comes back as an **uncertified** box that holds whatever
//! is there, touching ones merged into their hull. Zeros that are not
//! isolated at all, a whole curve of them, are [`Continuum`]: the boxes
//! alive at one depth double with every level instead of settling, and
//! [`MAX_FRONT`] ends it.

use arris_math::Meter;

use crate::bernstein::{BernsteinAlgebra, Binomials, MAX_DEPTH};
use crate::halt::Halt;

/// How far beyond each side a box is grown before it is certified, as a
/// fraction of its width. A ratio, not a tolerance: large enough that a
/// zero on a box's edge is well inside the grown box, small enough that
/// the de Casteljau extrapolation it takes — no longer a convex
/// combination — amplifies a coefficient's rounding by no more than
/// `(1 + 2·MARGIN)ⁿ`, under ten at degree ten.
const MARGIN: f64 = 0.125;

/// How many boxes may be alive at one depth. Isolated zeros keep a
/// handful of boxes each, whatever the depth; a curve of common zeros
/// doubles them with every level. A structural bound, not a tolerance:
/// no torus section measured has more than 110 alive on a patch, a tube
/// circle lying on the other surface among them (`crate::trace_torus`).
const MAX_FRONT: usize = 1024;

/// A polynomial is taken for zero over a box where its coefficients are
/// all within this many floors, while it takes one floor to exclude a
/// box. Any factor above one would do: with the same bound for both, a
/// box whose coefficients straddle the floor itself is neither, however
/// small it gets, and the boxes along the curve where the polynomial
/// *equals* its floor would be halved to the bottom for nothing. A
/// ratio, not a tolerance.
const FLAT: f64 = 2.0;

/// How many Newton steps polish a certified zero. Newton from inside a
/// certified box converges quadratically, so the count is never reached
/// by a step that still improves; it only ends a cycle of rounding.
const POLISH_STEPS: usize = 12;

/// A polynomial `Σ c[i][j] Bᵢᵐ(s) Bⱼⁿ(t)` on `[0, 1]²`.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Poly2 {
    /// The degrees `[m, n]` in `s` and in `t`.
    degree: [usize; 2],
    /// Row-major, `s` the row: `c[i · (n + 1) + j]`.
    c: Vec<f64>,
}

impl Poly2 {
    /// `a(s) · b(t)` from two polynomials in one variable.
    pub(crate) fn outer(a: &[f64], b: &[f64]) -> Self {
        let degree = [a.len().saturating_sub(1), b.len().saturating_sub(1)];
        let c = a
            .iter()
            .flat_map(|ai| b.iter().map(move |bj| ai * bj))
            .collect();
        Poly2 { degree, c }
    }

    /// The degrees in `s` and in `t`.
    pub(crate) fn degree(&self) -> [usize; 2] {
        self.degree
    }

    /// The coefficients, row-major with `s` the row.
    pub(crate) fn coefficients(&self) -> &[f64] {
        &self.c
    }

    fn stride(&self) -> usize {
        self.degree[1] + 1
    }

    fn rows(&self) -> impl Iterator<Item = &[f64]> {
        self.c.chunks(self.stride())
    }

    /// The product, of the degrees' sums: the one-variable formula in
    /// each direction.
    pub(crate) fn mul(&self, other: &Poly2, binomials: &Binomials) -> Poly2 {
        let ([m1, n1], [m2, n2]) = (self.degree, other.degree);
        let degree = [m1 + m2, n1 + n2];
        let stride = degree[1] + 1;
        let ratio = |a: usize, i: usize, b: usize, k: usize| {
            binomials.get(a, i) * binomials.get(b, k) / binomials.get(a + b, i + k)
        };
        let mut c = vec![0.0; (degree[0] + 1) * stride];
        for (i, row_a) in self.rows().enumerate() {
            for (k, row_b) in other.rows().enumerate() {
                let across = ratio(m1, i, m2, k);
                for (j, a) in row_a.iter().enumerate() {
                    for (l, b) in row_b.iter().enumerate() {
                        c[(i + k) * stride + j + l] += across * ratio(n1, j, n2, l) * a * b;
                    }
                }
            }
        }
        Poly2 { degree, c }
    }

    /// `Σ kᵢ · pᵢ` over polynomials of one pair of degrees.
    pub(crate) fn combine(terms: &[(f64, &Poly2)]) -> Poly2 {
        let degree = terms.first().map_or([0, 0], |(_, p)| p.degree);
        let c = (0..(degree[0] + 1) * (degree[1] + 1))
            .map(|i| {
                terms
                    .iter()
                    .map(|(k, p)| k * p.c.get(i).copied().unwrap_or(0.0))
                    .sum()
            })
            .collect();
        Poly2 { degree, c }
    }

    /// `∂/∂t`, of degree `n − 1` in `t`; a constant's is the constant
    /// zero.
    pub(crate) fn dv(&self) -> Poly2 {
        let [m, n] = self.degree;
        if n == 0 {
            return Poly2 {
                degree: [m, 0],
                c: vec![0.0; m + 1],
            };
        }
        let c = self
            .rows()
            .flat_map(|row| row.windows(2).map(move |w| n as f64 * (w[1] - w[0])))
            .collect();
        Poly2 {
            degree: [m, n - 1],
            c,
        }
    }

    /// `∂/∂s`, of degree `m − 1` in `s`.
    pub(crate) fn du(&self) -> Poly2 {
        let [m, n] = self.degree;
        if m == 0 {
            return Poly2 {
                degree: [0, n],
                c: vec![0.0; n + 1],
            };
        }
        let stride = self.stride();
        let c = (self.c.iter().zip(&self.c[stride..]))
            .map(|(below, above)| m as f64 * (above - below))
            .collect();
        Poly2 {
            degree: [m - 1, n],
            c,
        }
    }

    /// The value at `(s, t)` by de Casteljau's triangle along `t` in
    /// every row and then along `s`: convex combinations only inside
    /// the unit square.
    pub(crate) fn eval(&self, s: f64, t: f64) -> f64 {
        let column: Vec<f64> = self.rows().map(|row| casteljau(row, t)).collect();
        casteljau(&column, s)
    }

    /// The polynomial in `t` along the line `s = at`, by de Casteljau
    /// down every column.
    pub(crate) fn across(&self, at: f64) -> Vec<f64> {
        let [m, n] = self.degree;
        (0..=n)
            .map(|j| {
                let column: Vec<f64> = (0..=m).map(|i| self.c[i * (n + 1) + j]).collect();
                casteljau(&column, at)
            })
            .collect()
    }

    /// The polynomial in `s` along the line `t = at`.
    pub(crate) fn along(&self, at: f64) -> Vec<f64> {
        self.rows().map(|row| casteljau(row, at)).collect()
    }

    /// `self / (s − root)`, of one degree less in `s`, for a polynomial
    /// that vanishes along the line `s = root` — inside the unit square
    /// or outside it — and the rounding of the quotient's coefficients
    /// where `floor` is that of `self`'s. In the scaled basis `aᵢ·C(m, i)`
    /// the product with `(1 − root)·s − root·(1 − s)` is the two-term
    /// recurrence `ãᵢ = (1 − root)·c̃ᵢ₋₁ − root·c̃ᵢ`; it is solved forwards
    /// from `ã₀` and backwards from `ãₘ`, each as far as the basis function
    /// that peaks at the root, which is as far as either keeps what it
    /// carries in check — all the way for a root outside the square. One
    /// equation is left over between the two. For a polynomial
    /// that does vanish on the line it holds to rounding; otherwise what
    /// it is off by is dropped, a multiple of the one basis function that
    /// peaks nearest the root.
    pub(crate) fn over_linear(&self, root: f64, floor: f64, binomials: &Binomials) -> (Poly2, f64) {
        let [m, n] = self.degree;
        if m == 0 {
            return (self.clone(), floor);
        }
        let stride = n + 1;
        // Quotient coefficients below `meet` come from the forward
        // recurrence, the others from the backward one.
        let meet = (root * m as f64).round().clamp(0.0, m as f64) as usize;
        let mut c = vec![0.0; m * stride];
        let mut rounding = vec![0.0; m];
        for j in 0..=n {
            let scaled = |i: usize| binomials.get(m, i) * self.c[i * stride + j];
            let mut carried = 0.0;
            for i in 0..meet {
                carried = ((1.0 - root) * carried - scaled(i)) / root;
                c[i * stride + j] = carried / binomials.get(m - 1, i);
            }
            let mut carried = 0.0;
            for i in (meet..m).rev() {
                carried = (scaled(i + 1) + root * carried) / (1.0 - root);
                c[i * stride + j] = carried / binomials.get(m - 1, i);
            }
        }
        // The same recurrences on the magnitudes of what each step adds.
        let mut carried = 0.0;
        for (i, slot) in rounding.iter_mut().enumerate().take(meet) {
            carried = ((1.0 - root).abs() * carried + binomials.get(m, i) * floor) / root.abs();
            *slot = carried / binomials.get(m - 1, i);
        }
        let mut carried = 0.0;
        for i in (meet..m).rev() {
            carried = (binomials.get(m, i + 1) * floor + root.abs() * carried) / (1.0 - root).abs();
            rounding[i] = carried / binomials.get(m - 1, i);
        }
        let worst = rounding.iter().copied().fold(floor, f64::max);
        (
            Poly2 {
                degree: [m - 1, n],
                c,
            },
            worst,
        )
    }

    /// The same polynomial with `s` and `t` exchanged.
    fn transposed(&self) -> Poly2 {
        let [m, n] = self.degree;
        let mut c = Vec::with_capacity(self.c.len());
        for j in 0..=n {
            for i in 0..=m {
                c.push(self.c[i * (n + 1) + j]);
            }
        }
        Poly2 { degree: [n, m], c }
    }

    /// The two parts either side of `t = at`, each on its own `[0, 1]`:
    /// the halves of the unit square for `at = ½`, and for `at` outside
    /// `[0, 1]` the extrapolation to `[0, at]` and what is left.
    fn split_v(&self, at: f64) -> (Poly2, Poly2) {
        let mut lower = Vec::with_capacity(self.c.len());
        let mut upper = Vec::with_capacity(self.c.len());
        for row in self.rows() {
            let (l, u) = split(row, at);
            lower.extend(l);
            upper.extend(u);
        }
        let part = |c| Poly2 {
            degree: self.degree,
            c,
        };
        (part(lower), part(upper))
    }

    /// [`Self::split_v`] along `s`.
    fn split_u(&self, at: f64) -> (Poly2, Poly2) {
        let (lower, upper) = self.transposed().split_v(at);
        (lower.transposed(), upper.transposed())
    }

    /// The polynomial over the unit square grown by [`MARGIN`] on every
    /// side, on its own `[0, 1]²`.
    fn grown(&self) -> Poly2 {
        let (hi, inner) = (1.0 + MARGIN, -MARGIN / (1.0 + MARGIN));
        let wide = self.split_u(hi).0.split_u(inner).1;
        wide.split_v(hi).0.split_v(inner).1
    }

    /// The polynomial over the box `[s₀, s₁] × [t₀, t₁]` of the unit
    /// square, on its own `[0, 1]²`: convex combinations only. A box with
    /// no width is a line of the square, and its polynomial is constant
    /// across it.
    pub(crate) fn restricted(&self, s: [f64; 2], t: [f64; 2]) -> Poly2 {
        // The part below the upper end, and of that the part above the
        // lower end, which is at `lo / hi` of it.
        let inner = |[lo, hi]: [f64; 2]| if hi > 0.0 { (lo / hi).min(1.0) } else { 0.0 };
        let along_s = self.split_u(s[1]).0.split_u(inner(s)).1;
        along_s.split_v(t[1]).0.split_v(inner(t)).1
    }

    /// `Some(true)` where every coefficient is above `floor`,
    /// `Some(false)` where every one is below `-floor`: the polynomial's
    /// sign over the closed unit square, `None` where the coefficients do
    /// not say.
    pub(crate) fn sign(&self, floor: f64) -> Option<bool> {
        let range = self.range();
        if range.lo > floor {
            Some(true)
        } else if range.hi < -floor {
            Some(false)
        } else {
            None
        }
    }

    /// The least and greatest coefficient: bounds on the polynomial over
    /// the unit square.
    fn range(&self) -> Range {
        Range::of(self.c.iter().copied())
    }

    /// Every coefficient on one side of zero by more than `floor`.
    fn keeps_sign(&self, floor: f64) -> bool {
        let range = self.range();
        range.lo > floor || range.hi < -floor
    }

    /// Every coefficient within `floor` of zero: zero over the box, as
    /// far as `f64` can tell.
    fn is_flat(&self, floor: f64) -> bool {
        let range = self.range();
        range.lo >= -floor && range.hi <= floor
    }
}

impl BernsteinAlgebra<Poly2> for Binomials {
    fn mul(&self, a: &Poly2, b: &Poly2) -> Poly2 {
        a.mul(b, self)
    }

    fn combine(&self, terms: &[(f64, &Poly2)]) -> Poly2 {
        Poly2::combine(terms)
    }
}

/// The value of a one-variable polynomial at `s` by de Casteljau.
fn casteljau(c: &[f64], s: f64) -> f64 {
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
fn split(c: &[f64], at: f64) -> (Vec<f64>, Vec<f64>) {
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
struct Range {
    lo: f64,
    hi: f64,
}

impl Range {
    fn of(values: impl Iterator<Item = f64>) -> Range {
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

    fn around(value: f64, by: f64) -> Range {
        Range {
            lo: value - by,
            hi: value + by,
        }
    }

    fn widened(self, by: f64) -> Range {
        Range {
            lo: self.lo - by,
            hi: self.hi + by,
        }
    }

    fn middle(self) -> f64 {
        0.5 * (self.lo + self.hi)
    }

    fn magnitude(self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }

    fn scaled(self, k: f64) -> Range {
        Range::of([k * self.lo, k * self.hi].into_iter())
    }

    fn plus(self, o: Range) -> Range {
        Range {
            lo: self.lo + o.lo,
            hi: self.hi + o.hi,
        }
    }

    fn minus(self, o: Range) -> Range {
        Range {
            lo: self.lo - o.hi,
            hi: self.hi - o.lo,
        }
    }
}

/// A common zero of two polynomials, or a box that may hold some.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Zero2 {
    /// The zero, polished; the middle of the box of one not certified.
    pub(crate) at: [f64; 2],
    /// The low corner of the box.
    pub(crate) lo: [f64; 2],
    /// The high corner of the box.
    pub(crate) hi: [f64; 2],
    /// The box holds exactly one common zero, a simple one, and `at` is
    /// it. Otherwise the box holds whatever common zeros there are near
    /// it — none, one that is not simple, several — and the isolation
    /// says no more.
    pub(crate) certified: bool,
}

impl Zero2 {
    fn holds(&self, p: [f64; 2], shift: [f64; 2]) -> bool {
        (0..2).all(|k| self.lo[k] + shift[k] <= p[k] && p[k] <= self.hi[k] + shift[k])
    }

    fn area(&self) -> f64 {
        (self.hi[0] - self.lo[0]) * (self.hi[1] - self.lo[1])
    }
}

/// What an isolation found and what it took.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Isolation {
    /// The zeros and the uncertified boxes, ascending by `at`.
    pub(crate) zeros: Vec<Zero2>,
    /// The deepest level a box was looked at on.
    pub(crate) depth: usize,
    /// How many boxes were looked at.
    pub(crate) boxes: usize,
}

/// The common zeros are not isolated: more than [`MAX_FRONT`] boxes that
/// can neither be excluded nor certified at one depth, which a curve of
/// common zeros makes and a finite set does not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Continuum {
    /// The depth the front outgrew the bound at.
    pub(crate) depth: usize,
}

/// A polynomial that only excludes: a box where its coefficients keep
/// one sign beyond `floor` is dropped, and so is a certified zero where
/// its value is beyond `floor`; it takes no other part. With a `floor`
/// above the rounding it keeps the zeros of the system where it is
/// merely small.
pub(crate) struct Gate<'a> {
    /// The polynomial.
    pub(crate) poly: &'a Poly2,
    /// What its coefficients have to clear to exclude a box.
    pub(crate) floor: f64,
}

struct Cell {
    system: [Poly2; 2],
    gate: Option<Poly2>,
    lo: [f64; 2],
    width: [f64; 2],
    /// How many times the box was halved along `s` and along `t`.
    halved: [usize; 2],
}

impl Cell {
    /// The direction to halve along, `None` once neither can be halved
    /// again. There are two reasons to halve a box, and each has its
    /// direction.
    ///
    /// While the certificate is out of reach the box is halved to be
    /// **excluded**: along the direction the system varies more along,
    /// the largest first difference of each polynomial's coefficients
    /// against its largest coefficient, since that is the halving that
    /// narrows a range of coefficients most. A section that is thin one
    /// way — a drill through a tube a hundredth of the ring across — is
    /// resolved that way first, instead of by squares the size of its
    /// thin side all along it.
    ///
    /// `within_reach`, the box passed [`krawczyk`]'s first look and
    /// failed only on the grown box: it is halved to be **certified**.
    /// What keeps the test from deciding is the spread of `Y·[J]`, and a
    /// range of `[J]` is as wide as the second derivatives make it over
    /// the box, so each direction is scored by the second derivatives
    /// along it weighted by `|Y|`. The first rule alone settles at the
    /// shape where a polynomial varies as much either way, which for a
    /// zero at a box's corner is a shape the test never passes.
    fn direction(&self, within_reach: bool) -> Option<usize> {
        let size = |p: &Poly2| p.range().magnitude();
        let first = self.system.each_ref().map(|p| [p.du(), p.dv()]);
        let mut score = [0.0f64; 2];
        let y = within_reach
            .then(|| preconditioned(&jacobian(&self.system, [0.0; 2])))
            .flatten();
        match y {
            Some((y, _)) => {
                for (j, [p_s, p_t]) in first.iter().enumerate() {
                    let weight = y[0][j].abs() + y[1][j].abs();
                    let cross = size(&p_s.dv());
                    score[0] += weight * (size(&p_s.du()) + cross);
                    score[1] += weight * (size(&p_t.dv()) + cross);
                }
            }
            None => {
                for (p, [p_s, p_t]) in self.system.iter().zip(&first) {
                    let scale = size(p);
                    if scale > 0.0 {
                        score[0] = score[0].max(size(p_s) / scale);
                        score[1] = score[1].max(size(p_t) / scale);
                    }
                }
            }
        }
        let first = if score[1] > score[0] { 1 } else { 0 };
        [first, 1 - first]
            .into_iter()
            .find(|&k| self.halved[k] < MAX_DEPTH)
    }

    fn halves(&self, along: usize) -> [Cell; 2] {
        let split = |p: &Poly2| {
            if along == 0 {
                p.split_u(0.5)
            } else {
                p.split_v(0.5)
            }
        };
        let (a, b) = (split(&self.system[0]), split(&self.system[1]));
        let gate = self.gate.as_ref().map(split);
        let (gate_lo, gate_hi) = match gate {
            Some((lo, hi)) => (Some(lo), Some(hi)),
            None => (None, None),
        };
        let mut width = self.width;
        width[along] *= 0.5;
        let mut halved = self.halved;
        halved[along] += 1;
        let mut upper = self.lo;
        upper[along] += width[along];
        [
            Cell {
                system: [a.0, b.0],
                gate: gate_lo,
                lo: self.lo,
                width,
                halved,
            },
            Cell {
                system: [a.1, b.1],
                gate: gate_hi,
                lo: upper,
                width,
                halved,
            },
        ]
    }
}

/// The common zeros of `system` in `[0, 1]²`: every one is `at` a
/// certified [`Zero2`] or inside an uncertified one's box, against
/// `floors`, the rounding of each polynomial's coefficients, the
/// caller's to state. A certified zero may lie outside the square, within
/// [`MARGIN`] of it, and not every zero there is found: a caller tiling
/// a domain with squares finds it from the neighbour whose square it is
/// in, and merges the two where both did ([`merge`]).
pub(crate) fn common_zeros(
    system: [&Poly2; 2],
    floors: [f64; 2],
    gate: Option<Gate<'_>>,
    meter: &mut Meter,
) -> Result<Isolation, Halt<Continuum>> {
    let partials = [
        [system[0].du(), system[0].dv()],
        [system[1].du(), system[1].dv()],
    ];
    let mut zeros = Vec::new();
    let mut boxes = 0;
    let mut front = vec![Cell {
        system: [system[0].clone(), system[1].clone()],
        gate: gate.as_ref().map(|g| g.poly.clone()),
        lo: [0.0, 0.0],
        width: [1.0, 1.0],
        halved: [0, 0],
    }];
    let mut depth = 0;
    loop {
        let mut next = Vec::new();
        for cell in &front {
            meter.tick()?;
            boxes += 1;
            let excluded = cell.system.iter().zip(floors).any(|(p, f)| p.keeps_sign(f))
                || cell
                    .gate
                    .as_ref()
                    .zip(gate.as_ref())
                    .is_some_and(|(p, g)| p.keeps_sign(g.floor));
            if excluded {
                continue;
            }
            let uncertified = Zero2 {
                at: [0, 1].map(|k| cell.lo[k] + 0.5 * cell.width[k]),
                lo: cell.lo,
                hi: [0, 1].map(|k| cell.lo[k] + cell.width[k]),
                certified: false,
            };
            if cell
                .system
                .iter()
                .zip(floors)
                .any(|(p, f)| p.is_flat(FLAT * f))
            {
                zeros.push(uncertified);
                continue;
            }
            match krawczyk(&cell.system, floors) {
                Verdict::One(inside) => {
                    let grown_lo = [0, 1].map(|k| cell.lo[k] - MARGIN * cell.width[k]);
                    let grown = [0, 1].map(|k| (1.0 + 2.0 * MARGIN) * cell.width[k]);
                    let to_square = |p: [f64; 2]| [0, 1].map(|k| grown_lo[k] + grown[k] * p[k]);
                    let at = polish(
                        system,
                        &partials,
                        to_square(inside.lo),
                        to_square(inside.hi),
                    );
                    // The one zero of the grown box: where the gate is
                    // clear of zero there is none on its zero set.
                    let gated = gate
                        .as_ref()
                        .is_some_and(|g| g.poly.eval(at[0], at[1]).abs() > g.floor);
                    if !gated {
                        zeros.push(Zero2 {
                            at,
                            lo: grown_lo,
                            hi: to_square([1.0, 1.0]),
                            certified: true,
                        });
                    }
                }
                Verdict::None => {}
                Verdict::Unknown { within_reach } => match cell.direction(within_reach) {
                    Some(along) => next.extend(cell.halves(along)),
                    None => zeros.push(uncertified),
                },
            }
        }
        if next.is_empty() {
            break;
        }
        depth += 1;
        if next.len() > MAX_FRONT {
            return Err(Halt::Fault(Continuum { depth }));
        }
        front = next;
    }
    Ok(Isolation {
        zeros: merge(zeros, None),
        depth,
        boxes,
    })
}

/// The box `K` of the Krawczyk operator, in the grown box's own unit
/// square.
struct Inside {
    lo: [f64; 2],
    hi: [f64; 2],
}

enum Verdict {
    /// Exactly one common zero in the grown box, a simple one, inside
    /// this part of it.
    One(Inside),
    /// No common zero in the grown box.
    None,
    /// Neither shown: `within_reach` where the box itself would pass
    /// and the grown box did not.
    Unknown { within_reach: bool },
}

/// The ranges of the four partial derivatives over a box, from the
/// differences of its coefficients, widened by what two coefficients
/// `floor` wrong make of a difference.
fn jacobian(system: &[Poly2; 2], floors: [f64; 2]) -> [[Range; 2]; 2] {
    [0, 1].map(|k| {
        let p = &system[k];
        let [m, n] = p.degree();
        [
            p.du().range().widened(2.0 * m as f64 * floors[k]),
            p.dv().range().widened(2.0 * n as f64 * floors[k]),
        ]
    })
}

/// `Y`, the inverse of the middle of `[J]`, and the magnitudes of
/// `I − Y·[J]` summed along each row: how far the preconditioned system
/// is from the identity over the box, `None` where the middle has no
/// inverse.
fn preconditioned(j: &[[Range; 2]; 2]) -> Option<([[f64; 2]; 2], [f64; 2])> {
    let mid = j.map(|row| row.map(Range::middle));
    let det = mid[0][0] * mid[1][1] - mid[0][1] * mid[1][0];
    let y = [
        [mid[1][1] / det, -mid[0][1] / det],
        [-mid[1][0] / det, mid[0][0] / det],
    ];
    if !y.iter().flatten().all(|v| v.is_finite()) {
        return None;
    }
    let spread = [0, 1].map(|i| {
        (0..2)
            .map(|k| {
                let identity = if i == k { 1.0 } else { 0.0 };
                let entry = j[0][k].scaled(y[i][0]).plus(j[1][k].scaled(y[i][1]));
                Range::around(identity, 0.0).minus(entry).magnitude()
            })
            .sum::<f64>()
    });
    Some((y, spread))
}

/// The Krawczyk test of one box, on the box grown by [`MARGIN`]: `X` the
/// grown box, `c` its middle, `[J]` the ranges of the partial derivatives
/// over it and `Y` the inverse of their middle,
/// `K = c − Y·F(c) + (I − Y·[J])(X − c)`, with `F(c)` widened by the
/// floors. The preconditioning is what lets `[J]`'s entries be taken one
/// by one: two gradients that are far from parallel but whose components
/// overlap would never pass a determinant of ranges. The grown box is
/// only built where the box itself is within reach of the test, which
/// most boxes that are halved are not.
fn krawczyk(system: &[Poly2; 2], floors: [f64; 2]) -> Verdict {
    match preconditioned(&jacobian(system, floors)) {
        Some((_, spread)) if spread.iter().all(|s| *s < 1.0) => {}
        _ => {
            return Verdict::Unknown {
                within_reach: false,
            };
        }
    }
    let grown = [system[0].grown(), system[1].grown()];
    let Some((y, spread)) = preconditioned(&jacobian(&grown, floors)) else {
        return Verdict::Unknown { within_reach: true };
    };
    let f = [0, 1].map(|k| Range::around(grown[k].eval(0.5, 0.5), floors[k]));
    let mut inside = Inside {
        lo: [0.0; 2],
        hi: [0.0; 2],
    };
    for i in 0..2 {
        let step = f[0].scaled(y[i][0]).plus(f[1].scaled(y[i][1]));
        inside.lo[i] = 0.5 - step.hi - 0.5 * spread[i];
        inside.hi[i] = 0.5 - step.lo + 0.5 * spread[i];
    }
    let Inside { lo, hi } = inside;
    if (0..2).any(|k| hi[k] < 0.0 || lo[k] > 1.0) {
        Verdict::None
    } else if (0..2).all(|k| lo[k] > 0.0 && hi[k] < 1.0) {
        Verdict::One(inside)
    } else {
        // A NaN end comes here as well.
        Verdict::Unknown { within_reach: true }
    }
}

/// Newton on the polynomials themselves — not on a box's, which carry
/// the rounding of the subdivisions that made them — from the middle of
/// the certified box `[lo, hi]`, and never out of it. It stops at the
/// first step that does not shrink.
fn polish(system: [&Poly2; 2], partials: &[[Poly2; 2]; 2], lo: [f64; 2], hi: [f64; 2]) -> [f64; 2] {
    let mut at = [0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])];
    let mut last = f64::INFINITY;
    for _ in 0..POLISH_STEPS {
        let [s, t] = at;
        let f = [system[0].eval(s, t), system[1].eval(s, t)];
        let j = partials
            .each_ref()
            .map(|row| row.each_ref().map(|p| p.eval(s, t)));
        let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
        let step = [
            (j[1][1] * f[0] - j[0][1] * f[1]) / det,
            (j[0][0] * f[1] - j[1][0] * f[0]) / det,
        ];
        // A step that is no number, from a Jacobian with no inverse at
        // the iterate, ends it as well.
        let size = step[0].abs().max(step[1].abs());
        if !step.iter().all(|v| v.is_finite()) || size >= last {
            break;
        }
        last = size;
        at = [
            (s - step[0]).clamp(lo[0], hi[0]),
            (t - step[1]).clamp(lo[1], hi[1]),
        ];
    }
    at
}

/// One entry for every zero found more than once, ascending by `at`. Two
/// certified zeros are one where either lies in the other's box, which
/// holds a single zero; the smaller box stands for both. Uncertified
/// boxes that touch, within `touch`, are one box, their hull; one that
/// lies inside a certified box, kept or not, adds nothing to it and is
/// dropped.
///
/// With a `period` the coordinates are angles: boxes are compared a
/// period either way as well, and every `at` comes back in `[0, period)`
/// with its box around it. `touch` is then the rounding of the map that
/// made the angles, which two boxes sharing an edge may differ by.
pub(crate) fn merge(zeros: Vec<Zero2>, wrap: Option<(f64, f64)>) -> Vec<Zero2> {
    let (period, touch) = wrap.unwrap_or((0.0, 0.0));
    let shifts: Vec<[f64; 2]> = if wrap.is_some() {
        let turns = [-period, 0.0, period];
        turns
            .iter()
            .flat_map(|&a| turns.iter().map(move |&b| [a, b]))
            .collect()
    } else {
        vec![[0.0, 0.0]]
    };
    let order = |a: &Zero2, b: &Zero2| {
        (a.at[0].total_cmp(&b.at[0]))
            .then(a.at[1].total_cmp(&b.at[1]))
            .then(a.area().total_cmp(&b.area()))
    };
    let (mut certified, mut open): (Vec<Zero2>, Vec<Zero2>) =
        zeros.into_iter().partition(|z| z.certified);

    // Smallest boxes first, so the one kept is the tightest.
    let every = certified.clone();
    certified.sort_by(|a, b| a.area().total_cmp(&b.area()).then(order(a, b)));
    let mut kept: Vec<Zero2> = Vec::new();
    for z in certified {
        let same = kept.iter().any(|k| {
            shifts
                .iter()
                .any(|&shift| k.holds(z.at, shift) || z.holds(k.at, [-shift[0], -shift[1]]))
        });
        if !same {
            kept.push(z);
        }
    }

    // Hulls of touching boxes, to a fixed point.
    open.sort_by(order);
    loop {
        let mut merged = false;
        let mut hulls: Vec<Zero2> = Vec::new();
        for z in open {
            let touching = hulls.iter_mut().find_map(|h| {
                shifts
                    .iter()
                    .find(|shift| {
                        (0..2).all(|k| {
                            z.lo[k] + shift[k] <= h.hi[k] + touch
                                && h.lo[k] <= z.hi[k] + shift[k] + touch
                        })
                    })
                    .map(|&shift| (h, shift))
            });
            match touching {
                Some((h, shift)) => {
                    for (k, by) in shift.into_iter().enumerate() {
                        h.lo[k] = h.lo[k].min(z.lo[k] + by);
                        h.hi[k] = h.hi[k].max(z.hi[k] + by);
                        h.at[k] = 0.5 * (h.lo[k] + h.hi[k]);
                    }
                    merged = true;
                }
                None => hulls.push(z),
            }
        }
        open = hulls;
        if !merged {
            break;
        }
    }
    open.retain(|z| {
        !every.iter().any(|k| {
            shifts
                .iter()
                .any(|&shift| k.holds(z.lo, shift) && k.holds(z.hi, shift))
        })
    });

    kept.extend(open);
    if wrap.is_some() {
        for z in &mut kept {
            for k in 0..2 {
                let mut turn = period * (z.at[k] / period).floor();
                if z.at[k] - turn >= period {
                    // Rounded up to the period from just below zero.
                    turn += period;
                    z.at[k] = turn;
                }
                z.at[k] -= turn;
                z.lo[k] -= turn;
                z.hi[k] -= turn;
            }
        }
    }
    kept.sort_by(order);
    kept
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `a₀ + aₛ·s + aₜ·t`.
    fn linear(a0: f64, a_s: f64, a_t: f64) -> Poly2 {
        Poly2 {
            degree: [1, 1],
            c: vec![a0, a0 + a_t, a0 + a_s, a0 + a_s + a_t],
        }
    }

    /// `(s − a)² + (t − b)² − ρ²`.
    fn circle(a: f64, b: f64, rho: f64, binomials: &Binomials) -> Poly2 {
        let (ds, dt) = (linear(-a, 1.0, 0.0), linear(-b, 0.0, 1.0));
        let one = Poly2::outer(&[1.0; 3], &[1.0; 3]);
        Poly2::combine(&[
            (1.0, &ds.mul(&ds, binomials)),
            (1.0, &dt.mul(&dt, binomials)),
            (-rho * rho, &one),
        ])
    }

    fn certified(found: &Isolation) -> Vec<[f64; 2]> {
        found
            .zeros
            .iter()
            .filter(|z| z.certified)
            .map(|z| z.at)
            .collect()
    }

    fn assert_near(found: &[[f64; 2]], expected: &[[f64; 2]]) {
        assert_eq!(found.len(), expected.len(), "{found:?} vs {expected:?}");
        for (f, e) in found.iter().zip(expected) {
            assert!(
                (f[0] - e[0]).abs() < 1e-13 && (f[1] - e[1]).abs() < 1e-13,
                "{found:?} vs {expected:?}"
            );
        }
    }

    #[test]
    fn the_arithmetic_is_the_polynomials_own() {
        let b = Binomials::new(8);
        let p = circle(0.3, 0.6, 0.25, &b);
        let q = linear(0.5, -2.0, 1.5);
        let pq = p.mul(&q, &b);
        assert_eq!(pq.degree(), [3, 3]);
        let (ps, pt) = (p.du(), p.dv());
        assert_eq!((ps.degree(), pt.degree()), ([1, 2], [2, 1]));
        for i in 0..=10 {
            for j in 0..=10 {
                let (s, t) = (i as f64 / 10.0, j as f64 / 10.0);
                let value = (s - 0.3).powi(2) + (t - 0.6).powi(2) - 0.0625;
                assert!((p.eval(s, t) - value).abs() < 1e-15);
                assert!((pq.eval(s, t) - value * q.eval(s, t)).abs() < 1e-14);
                assert!((ps.eval(s, t) - 2.0 * (s - 0.3)).abs() < 1e-14);
                assert!((pt.eval(s, t) - 2.0 * (t - 0.6)).abs() < 1e-14);
            }
        }
        assert_eq!(linear(1.0, 0.0, 0.0).du().dv().coefficients(), [0.0]);
    }

    #[test]
    fn a_polynomial_over_its_linear_factor_is_the_other_factor() {
        let binomials = Binomials::new(16);
        // A quotient of bidegree (5, 3) with nothing special about it.
        let quotient = Poly2 {
            degree: [5, 3],
            c: (0..24).map(|k| ((k * 7 + 3) % 11) as f64 - 4.5).collect(),
        };
        // Roots inside the square, on its edges and outside it.
        for root in [0.0, 0.3, 0.5, 0.83, 1.0, -0.4, 1.5, 3.4] {
            let factor = Poly2::outer(&[-root, 1.0 - root], &[1.0]);
            let product = factor.mul(&quotient, &binomials);
            let floor = 8.0 * f64::EPSILON * 5.5;
            let (found, rounding) = product.over_linear(root, floor, &binomials);
            assert_eq!(found.degree(), [5, 3]);
            for (a, b) in found.coefficients().iter().zip(quotient.coefficients()) {
                assert!((a - b).abs() <= rounding, "{root}: {a} vs {b} ± {rounding}");
            }
            // The rounding carried is a small multiple of what went in.
            assert!(rounding <= 64.0 * floor, "{root}: {rounding} from {floor}");
            // Twice over, for a factor squared.
            let (again, _) = (factor.mul(&product, &binomials))
                .over_linear(root, floor, &binomials)
                .0
                .over_linear(root, floor, &binomials);
            for (a, b) in again.coefficients().iter().zip(quotient.coefficients()) {
                assert!((a - b).abs() <= 1e-12, "{root}: {a} vs {b}");
            }
        }
        // The lines of a polynomial are the polynomial along them.
        let (s, t) = (0.37, 0.81);
        let value = quotient.eval(s, t);
        assert!((casteljau(&quotient.across(s), t) - value).abs() < 1e-14);
        assert!((casteljau(&quotient.along(t), s) - value).abs() < 1e-14);
    }

    #[test]
    fn the_parts_of_a_box_are_the_polynomial_on_them() {
        let b = Binomials::new(8);
        let p = circle(0.3, 0.6, 0.25, &b).mul(&linear(0.5, -2.0, 1.5), &b);
        let (lo, hi) = p.split_u(0.5);
        let ((a, b), (c, d)) = (lo.split_v(0.5), hi.split_v(0.5));
        let quarters = [a, b, c, d];
        let grown = p.grown();
        for i in 0..=4 {
            for j in 0..=4 {
                let (s, t) = (i as f64 / 4.0, j as f64 / 4.0);
                for (k, part) in quarters.iter().enumerate() {
                    let at = [0.5 * ((k / 2) as f64 + s), 0.5 * ((k % 2) as f64 + t)];
                    assert!((part.eval(s, t) - p.eval(at[0], at[1])).abs() < 1e-15);
                }
                let at = [
                    -MARGIN + (1.0 + 2.0 * MARGIN) * s,
                    -MARGIN + (1.0 + 2.0 * MARGIN) * t,
                ];
                assert!((grown.eval(s, t) - p.eval(at[0], at[1])).abs() < 1e-14);
            }
        }
    }

    #[test]
    fn the_turning_points_of_a_circle_and_its_crossings_with_a_line() {
        let b = Binomials::new(8);
        let f = circle(0.4, 0.55, 0.3, &b);
        let turning = common_zeros([&f, &f.dv()], [1e-15; 2], None, &mut Meter::default()).unwrap();
        assert_near(&certified(&turning), &[[0.1, 0.55], [0.7, 0.55]]);
        assert_eq!(turning.zeros.len(), 2);
        // s + t = 0.95 through the centre: (0.4 ∓ 0.3/√2, 0.55 ± 0.3/√2).
        let line = linear(-0.95, 1.0, 1.0);
        let d = 0.3 * core::f64::consts::FRAC_1_SQRT_2;
        let crossings = common_zeros([&f, &line], [1e-15; 2], None, &mut Meter::default()).unwrap();
        assert_near(
            &certified(&crossings),
            &[[0.4 - d, 0.55 + d], [0.4 + d, 0.55 - d]],
        );
        // A product of known factors: the zeros of each against the line.
        let g = f.mul(&circle(0.4, 0.55, 0.1, &b), &b);
        let e = 0.1 * core::f64::consts::FRAC_1_SQRT_2;
        let crossings = common_zeros([&g, &line], [1e-15; 2], None, &mut Meter::default()).unwrap();
        assert_near(
            &certified(&crossings),
            &[
                [0.4 - d, 0.55 + d],
                [0.4 - e, 0.55 + e],
                [0.4 + e, 0.55 - e],
                [0.4 + d, 0.55 - d],
            ],
        );
        assert!(crossings.depth < 16, "{}", crossings.depth);
    }

    #[test]
    fn a_zero_on_a_line_between_boxes_or_on_the_edge_is_found_once() {
        let b = Binomials::new(8);
        // Turning points on s = ¼ and s = ¾, both on t = ½: corners of
        // boxes at every depth from the second.
        let f = circle(0.5, 0.5, 0.25, &b);
        let found = common_zeros([&f, &f.dv()], [1e-15; 2], None, &mut Meter::default()).unwrap();
        assert_near(&certified(&found), &[[0.25, 0.5], [0.75, 0.5]]);
        assert_eq!(found.zeros.len(), 2);
        assert!(found.depth < 16, "{}", found.depth);
        // On the square's own edge and corner, and one just outside it,
        // which the margin holds.
        let f = circle(0.25, 0.0, 0.25, &b);
        let found = common_zeros([&f, &f.dv()], [1e-15; 2], None, &mut Meter::default()).unwrap();
        assert_near(&certified(&found), &[[0.0, 0.0], [0.5, 0.0]]);
        // A zero outside the square is the neighbouring square's: found
        // from here where a margin happens to certify it, and no fault
        // where none does.
        let f = circle(0.249, 0.3, 0.25, &b);
        let found = certified(
            &common_zeros([&f, &f.dv()], [1e-15; 2], None, &mut Meter::default()).unwrap(),
        );
        let inside = found.iter().filter(|z| z[0] > 0.0).copied();
        assert_near(&inside.collect::<Vec<_>>(), &[[0.499, 0.3]]);
        let outside = found.iter().filter(|z| z[0] <= 0.0).copied();
        let outside: Vec<[f64; 2]> = outside.collect();
        assert!(outside.is_empty() || outside.len() == 1);
        assert_near(&outside, &vec![[-0.001, 0.3]; outside.len()]);
        let f = circle(-0.26, 0.3, 0.25, &b);
        let found = common_zeros([&f, &f.dv()], [1e-15; 2], None, &mut Meter::default()).unwrap();
        assert!(found.zeros.is_empty());
    }

    #[test]
    fn a_singular_point_is_a_critical_point_certified_and_a_turning_point_not() {
        let b = Binomials::new(8);
        // (s − 0.3)² − (t − 0.6)²: two lines crossing.
        let (ds, dt) = (linear(-0.3, 1.0, 0.0), linear(-0.6, 0.0, 1.0));
        let f = Poly2::combine(&[(1.0, &ds.mul(&ds, &b)), (-1.0, &dt.mul(&dt, &b))]);
        let critical = common_zeros(
            [&f.du(), &f.dv()],
            [1e-15; 2],
            Some(Gate {
                poly: &f,
                floor: 1e-15,
            }),
            &mut Meter::default(),
        )
        .unwrap();
        assert_near(&certified(&critical), &[[0.3, 0.6]]);
        // As a zero of (f, f_t) it is not simple: a box, not a zero, and
        // the point is in it.
        let turning = common_zeros([&f, &f.dv()], [1e-15; 2], None, &mut Meter::default()).unwrap();
        assert!(certified(&turning).is_empty());
        assert_eq!(turning.zeros.len(), 1, "{:?}", turning.zeros);
        assert!(turning.zeros[0].holds([0.3, 0.6], [0.0, 0.0]));
        assert!(turning.zeros[0].area() < 1e-12);
        // The gate drops a critical point off the curve, and with a
        // floor above the value there keeps it.
        let lifted = Poly2::combine(&[(1.0, &f), (1e-3, &Poly2::outer(&[1.0; 3], &[1.0; 3]))]);
        let gated = |floor: f64| {
            common_zeros(
                [&lifted.du(), &lifted.dv()],
                [1e-15; 2],
                Some(Gate {
                    poly: &lifted,
                    floor,
                }),
                &mut Meter::default(),
            )
            .unwrap()
        };
        assert!(gated(1e-15).zeros.is_empty());
        assert_near(&certified(&gated(2e-3)), &[[0.3, 0.6]]);
    }

    #[test]
    fn a_curve_of_common_zeros_is_a_continuum() {
        let b = Binomials::new(8);
        // (s − 0.3)(1 + t²): f and f_t both vanish along s = 0.3.
        let t = linear(0.0, 0.0, 1.0);
        let one = Poly2::outer(&[1.0; 3], &[1.0; 3]);
        let bowl = Poly2::combine(&[(1.0, &one), (1.0, &t.mul(&t, &b))]);
        let f = linear(-0.3, 1.0, 0.0).mul(&bowl, &b);
        let found = common_zeros([&f, &f.dv()], [1e-15; 2], None, &mut Meter::default());
        assert!(
            matches!(found, Err(Halt::Fault(Continuum { .. }))),
            "{found:?}"
        );
        // The zero polynomial is flat at once: one box, the square.
        let zero = Poly2::outer(&[0.0; 3], &[0.0; 3]);
        let found = common_zeros([&zero, &f], [1e-15; 2], None, &mut Meter::default()).unwrap();
        assert_eq!(found.zeros.len(), 1);
        assert!(!found.zeros[0].certified);
        assert_eq!(found.boxes, 1);
    }

    #[test]
    fn zeros_found_twice_across_a_seam_are_one() {
        let zero = |at: [f64; 2], half: f64, certified: bool| Zero2 {
            at,
            lo: [at[0] - half, at[1] - half],
            hi: [at[0] + half, at[1] + half],
            certified,
        };
        let merged = merge(
            vec![
                zero([6.25, 1.0], 0.1, true),
                zero([6.25 - 6.0, 1.0], 0.05, true),
                zero([-0.01, 3.0], 0.2, true),
                zero([5.99, 3.0], 0.05, true),
                zero([5.9, 3.05], 0.01, false),
                zero([0.05, 2.0], 0.05, false),
                zero([5.95, 2.0], 0.05, false),
            ],
            Some((6.0, 1e-12)),
        );
        let at: Vec<[f64; 2]> = merged.iter().map(|z| z.at).collect();
        assert_eq!(merged.len(), 3, "{merged:?}");
        assert!((at[0][0] - 0.0).abs() < 1e-12 && at[0][1] == 2.0, "{at:?}");
        assert!(!merged[0].certified);
        assert!((merged[0].hi[0] - merged[0].lo[0] - 0.2).abs() < 1e-12);
        assert!((at[1][0] - 0.25).abs() < 1e-12 && at[1][1] == 1.0, "{at:?}");
        assert_eq!(at[2], [5.99, 3.0]);
    }
}
