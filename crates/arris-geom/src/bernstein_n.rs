//! Tensor-product polynomials in Bernstein form on `[0, 1]ᴺ`, and the
//! isolation of the common zeros of `N` of them in `N` unknowns: what
//! `crate::bernstein2` does for two, carried to three and four. The
//! section of two rational patches is three of these in the four
//! parameters of the pair, `w₂·P₁ − w₁·P₂`, and an isocurve of one
//! against the other three in three; a fourth equation closes the first
//! system on the points a walk has to start from (ADR-0068).
//!
//! The arithmetic is `crate::bernstein2`'s one direction at a time: the
//! product, the partial derivative along any variable, de Casteljau's
//! evaluation and subdivision along any variable. Every operation is a
//! convex combination or a sum of products with positive factors, and
//! the coefficients bound the polynomial over the box.
//!
//! The isolation is `crate::bernstein2`'s with its three tests on a box
//! in `N` dimensions. A box is **excluded** where one polynomial's
//! coefficients keep one sign beyond its floor, or one row of `Y·F`'s
//! does, `Y` the inverse of the box's middle Jacobian: what
//! `crate::bernstein2` does without, and four unknowns cannot
//! (`excluded_together`). It is **certified** by
//! the Krawczyk operator on the box grown by `MARGIN` of its width along
//! every axis, `K = c − Y·F(c) + (I − Y·[J])(X − c)` with `Y` the inverse
//! of the middle of `[J]`, an `N × N` matrix now; `K` inside the grown
//! box `X` puts exactly one zero there, a simple one. Otherwise it is
//! halved, along the axis whose worse half comes nearer the certificate
//! when it is within reach, along the one the system varies most along
//! when it is not (`Cell::direction`). The asymmetry is kept: a
//! candidate too many, never a zero too few. A zero that is not simple,
//! a box where a polynomial is zero as far as `f64` can tell, a cluster
//! that [`MAX_DEPTH`] halvings along every axis do not separate, comes
//! back as an **uncertified** box; zeros that are not isolated at all are
//! a [`Continuum`], the front of live boxes outgrowing `MAX_FRONT`.

use arris_math::Meter;

use crate::bernstein::{BernsteinAlgebra, Binomials, MAX_DEPTH};
use crate::bernstein2::{FLAT, MARGIN, MAX_FRONT, POLISH_STEPS, Range, casteljau, split};
use crate::halt::Halt;

/// A polynomial `Σ c[i] Πₖ B_{iₖ}^{nₖ}(xₖ)` on `[0, 1]ᴺ`.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct PolyN<const N: usize> {
    /// The degree `nₖ` in each variable.
    degree: [usize; N],
    /// Row-major, the last variable fastest: the multi-index `i` is at
    /// `Σₖ iₖ · Πⱼ₍ⱼ>ₖ₎ (nⱼ + 1)`.
    c: Vec<f64>,
}

/// How many coefficients a polynomial of `degree` has.
fn count<const N: usize>(degree: [usize; N]) -> usize {
    degree.iter().map(|d| d + 1).product()
}

/// The step in the coefficients between neighbours along each variable.
fn strides<const N: usize>(degree: [usize; N]) -> [usize; N] {
    core::array::from_fn(|k| count_after(&degree, k))
}

fn count_after(degree: &[usize], k: usize) -> usize {
    degree.iter().skip(k + 1).map(|d| d + 1).product()
}

/// Every multi-index of `degree`, in the order of the coefficients.
fn indices<const N: usize>(degree: [usize; N]) -> Vec<[usize; N]> {
    let stride = strides(degree);
    (0..count(degree))
        .map(|flat| core::array::from_fn(|k| (flat / stride[k]) % (degree[k] + 1)))
        .collect()
}

impl PolyN<3> {
    /// `a(x₀)·b(x₁, x₂)`: a polynomial in one variable times one in two
    /// others, of degree `a`'s then `b`'s.
    pub(crate) fn tensor3(a: &PolyN<1>, b: &PolyN<2>) -> PolyN<3> {
        let c =
            a.c.iter()
                .flat_map(|x| b.c.iter().map(move |y| x * y))
                .collect();
        PolyN {
            degree: [a.degree[0], b.degree[0], b.degree[1]],
            c,
        }
    }
}

impl PolyN<4> {
    /// `a(x₀, x₁)·b(x₂, x₃)`: a polynomial in two variables times one in
    /// two others, of degree `a`'s then `b`'s.
    pub(crate) fn tensor(a: &PolyN<2>, b: &PolyN<2>) -> PolyN<4> {
        let [da, db] = [a.degree, b.degree];
        let c =
            a.c.iter()
                .flat_map(|x| b.c.iter().map(move |y| x * y))
                .collect();
        PolyN {
            degree: [da[0], da[1], db[0], db[1]],
            c,
        }
    }
}

impl<const N: usize> PolyN<N> {
    /// The polynomial of `degree` with coefficients `c`, row-major with
    /// the last variable fastest, as many as the degree has: the
    /// caller's invariant.
    pub(crate) fn new(degree: [usize; N], c: Vec<f64>) -> Self {
        debug_assert_eq!(c.len(), count(degree), "coefficients for {degree:?}");
        PolyN { degree, c }
    }

    /// `Πₖ fₖ(xₖ)` from one polynomial in each variable.
    pub(crate) fn outer(factors: [&[f64]; N]) -> Self {
        let degree = factors.map(|f| f.len().saturating_sub(1));
        let c = indices(degree)
            .iter()
            .map(|i| (0..N).map(|k| factors[k][i[k]]).product())
            .collect();
        PolyN { degree, c }
    }

    /// The degree in each variable.
    pub(crate) fn degree(&self) -> [usize; N] {
        self.degree
    }

    /// The coefficients, row-major with the last variable fastest.
    pub(crate) fn coefficients(&self) -> &[f64] {
        &self.c
    }

    /// The product, of the degrees' sums: the one-variable formula along
    /// every variable.
    pub(crate) fn mul(&self, other: &PolyN<N>, binomials: &Binomials) -> PolyN<N> {
        let degree: [usize; N] = core::array::from_fn(|k| self.degree[k] + other.degree[k]);
        let stride = strides(degree);
        let mut c = vec![0.0; count(degree)];
        let theirs = indices(other.degree);
        for (i, a) in indices(self.degree).iter().zip(&self.c) {
            for (j, b) in theirs.iter().zip(&other.c) {
                let mut term = a * b;
                let mut at = 0;
                for k in 0..N {
                    let (m, n) = (self.degree[k], other.degree[k]);
                    term *= binomials.get(m, i[k]) * binomials.get(n, j[k])
                        / binomials.get(m + n, i[k] + j[k]);
                    at += (i[k] + j[k]) * stride[k];
                }
                c[at] += term;
            }
        }
        PolyN { degree, c }
    }

    /// `Σ kᵢ · pᵢ` over polynomials of one degree.
    pub(crate) fn combine(terms: &[(f64, &PolyN<N>)]) -> PolyN<N> {
        let degree = terms.first().map_or([0; N], |(_, p)| p.degree);
        let c = (0..count(degree))
            .map(|i| {
                terms
                    .iter()
                    .map(|(k, p)| k * p.c.get(i).copied().unwrap_or(0.0))
                    .sum()
            })
            .collect();
        PolyN { degree, c }
    }

    /// `∂/∂xₖ`, of degree `nₖ − 1` in `xₖ`; a constant's is the constant
    /// zero.
    pub(crate) fn partial(&self, k: usize) -> PolyN<N> {
        let n = self.degree[k];
        if n == 0 {
            return PolyN {
                degree: self.degree,
                c: vec![0.0; self.c.len()],
            };
        }
        let mut degree = self.degree;
        degree[k] = n - 1;
        let stride = count_after(&self.degree, k);
        let c = (0..count(degree))
            .map(|i| {
                // The block of every index before `k` is `n` strides wide
                // here and `n + 1` in `self`.
                let old = (i / (n * stride)) * (n + 1) * stride + i % (n * stride);
                n as f64 * (self.c[old + stride] - self.c[old])
            })
            .collect();
        PolyN { degree, c }
    }

    /// The value at `x` by de Casteljau's triangle along the last
    /// variable in every fibre, then along the one before: convex
    /// combinations only inside the unit cube.
    pub(crate) fn eval(&self, x: [f64; N]) -> f64 {
        let mut c = self.c.clone();
        for k in (0..N).rev() {
            c = c
                .chunks(self.degree[k] + 1)
                .map(|fibre| casteljau(fibre, x[k]))
                .collect();
        }
        c.first().copied().unwrap_or(0.0)
    }

    /// The two parts either side of `xₖ = at`, each on its own `[0, 1]`:
    /// the halves of the cube for `at = ½`, and for `at` outside `[0, 1]`
    /// the extrapolation to `[0, at]` and what is left.
    pub(crate) fn split(&self, k: usize, at: f64) -> (PolyN<N>, PolyN<N>) {
        let (width, stride) = (self.degree[k] + 1, count_after(&self.degree, k));
        let mut lower = vec![0.0; self.c.len()];
        let mut upper = vec![0.0; self.c.len()];
        let mut fibre = Vec::with_capacity(width);
        for block in 0..self.c.len() / (width * stride) {
            for inner in 0..stride {
                let base = block * width * stride + inner;
                fibre.clear();
                fibre.extend((0..width).map(|i| self.c[base + i * stride]));
                let (l, u) = split(&fibre, at);
                for (i, (l, u)) in l.into_iter().zip(u).enumerate() {
                    lower[base + i * stride] = l;
                    upper[base + i * stride] = u;
                }
            }
        }
        let part = |c| PolyN {
            degree: self.degree,
            c,
        };
        (part(lower), part(upper))
    }

    /// The same polynomial of `degree`, at least its own along every
    /// axis: elevated one degree at a time, each a convex combination of
    /// neighbouring coefficients, `c'ᵢ = i/(n+1)·cᵢ₋₁ + (1 − i/(n+1))·cᵢ`.
    pub(crate) fn elevated(&self, degree: [usize; N]) -> PolyN<N> {
        let mut p = self.clone();
        for k in 0..N {
            while p.degree[k] < degree[k] {
                let (n, stride) = (p.degree[k], count_after(&p.degree, k));
                let mut raised = p.degree;
                raised[k] = n + 1;
                let c = (0..count(raised))
                    .map(|i| {
                        // The index along `k`, and the coefficient at
                        // index 0 along `k` in `p` for the same others.
                        let (block, rest) = (i / ((n + 2) * stride), i % ((n + 2) * stride));
                        let (along, inner) = (rest / stride, rest % stride);
                        let base = block * (n + 1) * stride + inner;
                        let t = along as f64 / (n + 1) as f64;
                        let below = if along > 0 {
                            p.c[base + (along - 1) * stride]
                        } else {
                            0.0
                        };
                        let here = if along <= n {
                            p.c[base + along * stride]
                        } else {
                            0.0
                        };
                        t * below + (1.0 - t) * here
                    })
                    .collect();
                p = PolyN { degree: raised, c };
            }
        }
        p
    }

    /// The polynomial over the unit cube grown by [`MARGIN`] along every
    /// axis, on its own `[0, 1]ᴺ`.
    fn grown(&self) -> PolyN<N> {
        let (hi, inner) = (1.0 + MARGIN, -MARGIN / (1.0 + MARGIN));
        (0..N).fold(self.clone(), |p, k| p.split(k, hi).0.split(k, inner).1)
    }

    /// The least and greatest coefficient: bounds on the polynomial over
    /// the unit cube.
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

impl<const N: usize> BernsteinAlgebra<PolyN<N>> for Binomials {
    fn mul(&self, a: &PolyN<N>, b: &PolyN<N>) -> PolyN<N> {
        a.mul(b, self)
    }

    fn combine(&self, terms: &[(f64, &PolyN<N>)]) -> PolyN<N> {
        PolyN::combine(terms)
    }
}

/// A common zero of `N` polynomials, or a box that may hold some.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Zero<const N: usize> {
    /// The zero, polished; the middle of the box of one not certified.
    pub(crate) at: [f64; N],
    /// The low corner of the box.
    pub(crate) lo: [f64; N],
    /// The high corner of the box.
    pub(crate) hi: [f64; N],
    /// The box holds exactly one common zero, a simple one, and `at` is
    /// it. Otherwise the box holds whatever common zeros there are near
    /// it — none, one that is not simple, several — and the isolation
    /// says no more.
    pub(crate) certified: bool,
}

impl<const N: usize> Zero<N> {
    /// Whether `p` is in the box moved by `shift`.
    pub(crate) fn holds(&self, p: [f64; N], shift: [f64; N]) -> bool {
        (0..N).all(|k| self.lo[k] + shift[k] <= p[k] && p[k] <= self.hi[k] + shift[k])
    }

    fn volume(&self) -> f64 {
        (0..N).map(|k| self.hi[k] - self.lo[k]).product()
    }

    fn of_box(lo: [f64; N], width: [f64; N]) -> Self {
        Zero {
            at: core::array::from_fn(|k| lo[k] + 0.5 * width[k]),
            lo,
            hi: core::array::from_fn(|k| lo[k] + width[k]),
            certified: false,
        }
    }
}

/// What an isolation found and what it took.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Isolation<const N: usize> {
    /// The zeros and the uncertified boxes, ascending by `at`.
    pub(crate) zeros: Vec<Zero<N>>,
    /// The deepest level a box was looked at on.
    pub(crate) depth: usize,
    /// How many boxes were looked at.
    pub(crate) boxes: usize,
}

/// The common zeros are not isolated: more than `MAX_FRONT` boxes that
/// can neither be excluded nor certified at one depth, which a curve of
/// common zeros makes and a finite set does not.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Continuum<const N: usize> {
    /// The depth the front outgrew the bound at.
    pub(crate) depth: usize,
    /// The zeros and boxes found before the front outgrew the bound.
    pub(crate) found: Vec<Zero<N>>,
    /// The boxes alive at that depth, uncertified: every common zero in
    /// the unit cube that is not in `found` is in one of them.
    pub(crate) front: Vec<Zero<N>>,
}

/// A polynomial that only excludes, as `crate::bernstein2::Gate` is: a
/// box where its coefficients are all above `floor` is dropped, and so is
/// a certified zero where its value is; it takes no other part. It keeps
/// the zeros of the system where it is at most `floor`, however far
/// below.
pub(crate) struct Gate<'a, const N: usize> {
    /// The polynomial.
    pub(crate) poly: &'a PolyN<N>,
    /// What its coefficients have to clear to exclude a box.
    pub(crate) floor: f64,
}

struct Cell<const N: usize> {
    system: [PolyN<N>; N],
    /// The gates' parts over the box, in the gates' order.
    gates: Vec<PolyN<N>>,
    lo: [f64; N],
    width: [f64; N],
    /// How many times the box was halved along each axis.
    halved: [usize; N],
}

impl<const N: usize> Cell<N> {
    /// The axis to halve along, `None` once none can be halved again.
    /// `crate::bernstein2`'s two reasons, each with its direction.
    ///
    /// While the certificate is out of reach the box is halved to be
    /// **excluded**: along the axis the system varies most along, the
    /// largest first difference of each polynomial's coefficients against
    /// its largest coefficient.
    ///
    /// `within_reach`, the box passed [`krawczyk`]'s first look and failed
    /// only on the grown box: it is halved to be **certified**. Where
    /// every axis is open, the axis whose worse half has the smallest
    /// spread of `I − Y·[J]` is taken, as `crate::bernstein2`'s
    /// look-ahead halving does: a range of `[J]` is as wide as the cross
    /// derivatives make it across the other axes, which a score of one
    /// axis alone does not see. Where no axis is strictly best, each is
    /// scored by the second derivatives along it and across it weighted
    /// by `|Y|`.
    fn direction(&self, within_reach: bool) -> Option<usize> {
        if within_reach && self.halved.iter().all(|&h| h < MAX_DEPTH) {
            let worst = |along: usize| {
                let halves = self.system.each_ref().map(|p| p.split(along, 0.5));
                let lower = halves.each_ref().map(|h| h.0.clone());
                [lower, halves.map(|h| h.1)]
                    .iter()
                    .map(|half| {
                        preconditioned(&jacobian(half, [0.0; N])).map_or(f64::INFINITY, |(_, s)| {
                            s.iter().copied().fold(0.0, f64::max)
                        })
                    })
                    .fold(0.0, f64::max)
            };
            let spread: [f64; N] = core::array::from_fn(worst);
            let best = spread.iter().copied().fold(f64::INFINITY, f64::min);
            let mut at_best = (0..N).filter(|&k| spread[k] == best);
            if let (Some(k), None) = (at_best.next(), at_best.next()) {
                return Some(k);
            }
        }
        let size = |p: &PolyN<N>| p.range().magnitude();
        let first = self
            .system
            .each_ref()
            .map(|p| core::array::from_fn::<_, N, _>(|k| p.partial(k)));
        let mut score = [0.0f64; N];
        let y = within_reach
            .then(|| preconditioned(&jacobian(&self.system, [0.0; N])))
            .flatten();
        match y {
            Some((y, _)) => {
                for (j, partials) in first.iter().enumerate() {
                    let weight: f64 = (0..N).map(|i| y[i][j].abs()).sum();
                    for (k, p_k) in partials.iter().enumerate() {
                        let along: f64 = (0..N).map(|l| size(&p_k.partial(l))).sum();
                        score[k] += weight * along;
                    }
                }
            }
            None => {
                for (p, partials) in self.system.iter().zip(&first) {
                    let scale = size(p);
                    if scale > 0.0 {
                        for (k, p_k) in partials.iter().enumerate() {
                            score[k] = score[k].max(size(p_k) / scale);
                        }
                    }
                }
            }
        }
        let mut order: [usize; N] = core::array::from_fn(|k| k);
        // Stable: of two axes scored alike the first is halved first.
        order.sort_by(|&a, &b| score[b].total_cmp(&score[a]));
        order.into_iter().find(|&k| self.halved[k] < MAX_DEPTH)
    }

    fn halves(&self, along: usize) -> [Cell<N>; 2] {
        let parts = self.system.each_ref().map(|p| p.split(along, 0.5));
        let mut width = self.width;
        width[along] *= 0.5;
        let mut halved = self.halved;
        halved[along] += 1;
        let mut upper = self.lo;
        upper[along] += width[along];
        let (lower_system, upper_system) =
            (parts.each_ref().map(|p| p.0.clone()), parts.map(|p| p.1));
        let (lower_gates, upper_gates) = self.gates.iter().map(|g| g.split(along, 0.5)).unzip();
        [
            Cell {
                system: lower_system,
                gates: lower_gates,
                lo: self.lo,
                width,
                halved,
            },
            Cell {
                system: upper_system,
                gates: upper_gates,
                lo: upper,
                width,
                halved,
            },
        ]
    }
}

/// The common zeros of `system` in `[0, 1]ᴺ`: every one is `at` a
/// certified [`Zero`] or inside an uncertified one's box, against
/// `floors`, the rounding of each polynomial's coefficients, the
/// caller's to state. A certified zero may lie outside the cube, within
/// [`MARGIN`] of it, and not every zero there is found: a caller tiling
/// a domain with cubes finds it from the neighbour whose cube it is in,
/// and merges the two where both did ([`merge`]).
pub(crate) fn common_zeros<const N: usize>(
    system: [&PolyN<N>; N],
    floors: [f64; N],
    meter: &mut Meter,
) -> Result<Isolation<N>, Halt<Continuum<N>>> {
    common_zeros_with(system, floors, &[], meter)
}

/// [`common_zeros`] of the zeros `gates` keep: a box or a zero a gate
/// drops is not looked at again ([`Gate`]).
pub(crate) fn common_zeros_with<const N: usize>(
    system: [&PolyN<N>; N],
    floors: [f64; N],
    gates: &[Gate<'_, N>],
    meter: &mut Meter,
) -> Result<Isolation<N>, Halt<Continuum<N>>> {
    let partials: [[PolyN<N>; N]; N] = system.map(|p| core::array::from_fn(|k| p.partial(k)));
    // One degree for all, so that any combination of them is one more
    // polynomial of it (`excluded_together`).
    let degree: [usize; N] =
        core::array::from_fn(|k| system.iter().map(|p| p.degree[k]).max().unwrap_or(0));
    let mut zeros = Vec::new();
    let mut boxes = 0;
    let mut front = vec![Cell {
        system: system.map(|p| p.elevated(degree)),
        gates: gates.iter().map(|g| g.poly.clone()).collect(),
        lo: [0.0; N],
        width: [1.0; N],
        halved: [0; N],
    }];
    let mut depth = 0;
    loop {
        let mut next = Vec::new();
        for cell in &front {
            meter.tick()?;
            boxes += 1;
            if cell.system.iter().zip(floors).any(|(p, f)| p.keeps_sign(f))
                || cell
                    .gates
                    .iter()
                    .zip(gates)
                    .any(|(p, g)| p.range().lo > g.floor)
            {
                continue;
            }
            if cell
                .system
                .iter()
                .zip(floors)
                .any(|(p, f)| p.is_flat(FLAT * f))
            {
                zeros.push(Zero::of_box(cell.lo, cell.width));
                continue;
            }
            let first_look = preconditioned(&jacobian(&cell.system, floors));
            if first_look
                .as_ref()
                .is_some_and(|(y, _)| excluded_together(&cell.system, floors, y))
            {
                continue;
            }
            match krawczyk(&cell.system, floors, first_look) {
                Verdict::One(inside) => {
                    let grown_lo: [f64; N] =
                        core::array::from_fn(|k| cell.lo[k] - MARGIN * cell.width[k]);
                    let grown: [f64; N] =
                        core::array::from_fn(|k| (1.0 + 2.0 * MARGIN) * cell.width[k]);
                    let to_cube =
                        |p: [f64; N]| core::array::from_fn(|k| grown_lo[k] + grown[k] * p[k]);
                    let at = polish(system, &partials, to_cube(inside.lo), to_cube(inside.hi));
                    // The one zero of the grown box: where a gate drops
                    // it, the box holds none the gates keep.
                    if gates.iter().all(|g| g.poly.eval(at) <= g.floor) {
                        zeros.push(Zero {
                            at,
                            lo: grown_lo,
                            hi: to_cube([1.0; N]),
                            certified: true,
                        });
                    }
                }
                Verdict::None => {}
                Verdict::Unknown { within_reach } => match cell.direction(within_reach) {
                    Some(along) => next.extend(cell.halves(along)),
                    None => zeros.push(Zero::of_box(cell.lo, cell.width)),
                },
            }
        }
        if next.is_empty() {
            break;
        }
        depth += 1;
        if next.len() > MAX_FRONT {
            let front = next
                .iter()
                .map(|cell| Zero::of_box(cell.lo, cell.width))
                .collect();
            return Err(Halt::Fault(Continuum {
                depth,
                found: zeros,
                front,
            }));
        }
        front = next;
    }
    Ok(Isolation {
        zeros: merge(zeros, &Wrap::open()),
        depth,
        boxes,
    })
}

/// The box `K` of the Krawczyk operator, in the grown box's own unit
/// cube.
struct Inside<const N: usize> {
    lo: [f64; N],
    hi: [f64; N],
}

enum Verdict<const N: usize> {
    /// Exactly one common zero in the grown box, a simple one, inside
    /// this part of it.
    One(Inside<N>),
    /// No common zero in the grown box.
    None,
    /// Neither shown: `within_reach` where the box itself would pass
    /// and the grown box did not.
    Unknown { within_reach: bool },
}

/// The ranges of the `N × N` partial derivatives over a box, row `j` the
/// gradient of polynomial `j`, from the differences of its coefficients
/// widened by what two coefficients `floor` wrong make of a difference.
fn jacobian<const N: usize>(system: &[PolyN<N>; N], floors: [f64; N]) -> [[Range; N]; N] {
    core::array::from_fn(|j| {
        let p = &system[j];
        core::array::from_fn(|k| {
            (p.partial(k).range()).widened(2.0 * p.degree[k] as f64 * floors[j])
        })
    })
}

/// An `N × N` matrix, row by row.
type Matrix<const N: usize> = [[f64; N]; N];

/// The inverse of `m` by Gauss–Jordan elimination with partial pivoting,
/// `None` where a pivot is zero or the result is not finite.
fn inverse<const N: usize>(m: Matrix<N>) -> Option<Matrix<N>> {
    let mut a = m;
    let mut inv: Matrix<N> =
        core::array::from_fn(|i| core::array::from_fn(|k| if i == k { 1.0 } else { 0.0 }));
    for col in 0..N {
        let pivot = (col..N).max_by(|&r, &s| a[r][col].abs().total_cmp(&a[s][col].abs()))?;
        if a[pivot][col] == 0.0 {
            return None;
        }
        a.swap(col, pivot);
        inv.swap(col, pivot);
        let scale = 1.0 / a[col][col];
        for k in 0..N {
            a[col][k] *= scale;
            inv[col][k] *= scale;
        }
        for row in 0..N {
            if row == col {
                continue;
            }
            let factor = a[row][col];
            for k in 0..N {
                a[row][k] -= factor * a[col][k];
                inv[row][k] -= factor * inv[col][k];
            }
        }
    }
    inv.iter().flatten().all(|v| v.is_finite()).then_some(inv)
}

/// `Y`, the inverse of the middle of `[J]`, and the magnitudes of
/// `I − Y·[J]` summed along each row: how far the preconditioned system
/// is from the identity over the box, `None` where the middle has no
/// inverse.
fn preconditioned<const N: usize>(j: &[[Range; N]; N]) -> Option<(Matrix<N>, [f64; N])> {
    let y = inverse(j.map(|row| row.map(Range::middle)))?;
    let spread = core::array::from_fn(|i| {
        (0..N)
            .map(|k| {
                let identity = if i == k { 1.0 } else { 0.0 };
                let entry = (0..N).fold(Range::around(0.0, 0.0), |sum, l| {
                    sum.plus(j[l][k].scaled(y[i][l]))
                });
                Range::around(identity, 0.0).minus(entry).magnitude()
            })
            .sum::<f64>()
    });
    Some((y, spread))
}

/// Whether `Y·F` excludes the box: some combination `Σⱼ yᵢⱼ·Fⱼ`, a
/// polynomial with no zero where the system has none, keeps one sign
/// beyond `Σⱼ |yᵢⱼ|·floorⱼ`. With `Y` the inverse of the box's middle
/// Jacobian these are near `xᵢ − rᵢ`, whose coefficients exclude every
/// box but those within a few widths of the root along each axis;
/// the polynomials one by one leave a slab of the condition number's
/// widths around it along each, and the boxes alive near a zero would
/// grow as its `N`th power (Mourrain and Pavone, *Subdivision methods
/// for solving polynomial equations*, J. Symb. Comput. 44, 2009, for the
/// preconditioning). The system is of one degree (`common_zeros`
/// elevates it), so every combination is one polynomial of it.
fn excluded_together<const N: usize>(
    system: &[PolyN<N>; N],
    floors: [f64; N],
    y: &Matrix<N>,
) -> bool {
    y.iter().any(|row| {
        let terms: Vec<(f64, &PolyN<N>)> = row.iter().copied().zip(system).collect();
        let floor = (0..N).map(|j| row[j].abs() * floors[j]).sum();
        PolyN::combine(&terms).keeps_sign(floor)
    })
}

/// The Krawczyk test of one box, on the box grown by [`MARGIN`]:
/// `crate::bernstein2`'s in `N` dimensions, from `first_look`, the
/// preconditioner of the box itself and its spread. The grown box is
/// only built where the box itself is within reach of the test.
fn krawczyk<const N: usize>(
    system: &[PolyN<N>; N],
    floors: [f64; N],
    first_look: Option<(Matrix<N>, [f64; N])>,
) -> Verdict<N> {
    match first_look {
        Some((_, spread)) if spread.iter().all(|s| *s < 1.0) => {}
        _ => {
            return Verdict::Unknown {
                within_reach: false,
            };
        }
    }
    let grown = system.each_ref().map(PolyN::grown);
    let Some((y, spread)) = preconditioned(&jacobian(&grown, floors)) else {
        return Verdict::Unknown { within_reach: true };
    };
    let f: [Range; N] = core::array::from_fn(|j| Range::around(grown[j].eval([0.5; N]), floors[j]));
    let mut inside = Inside {
        lo: [0.0; N],
        hi: [0.0; N],
    };
    for i in 0..N {
        let step = (0..N).fold(Range::around(0.0, 0.0), |sum, j| {
            sum.plus(f[j].scaled(y[i][j]))
        });
        inside.lo[i] = 0.5 - step.hi - 0.5 * spread[i];
        inside.hi[i] = 0.5 - step.lo + 0.5 * spread[i];
    }
    let Inside { lo, hi } = &inside;
    if (0..N).any(|k| hi[k] < 0.0 || lo[k] > 1.0) {
        Verdict::None
    } else if (0..N).all(|k| lo[k] > 0.0 && hi[k] < 1.0) {
        Verdict::One(inside)
    } else {
        // A NaN end comes here as well.
        Verdict::Unknown { within_reach: true }
    }
}

/// Newton on the polynomials themselves from the middle of the certified
/// box `[lo, hi]`, never out of it; it stops at the first step that does
/// not shrink.
fn polish<const N: usize>(
    system: [&PolyN<N>; N],
    partials: &[[PolyN<N>; N]; N],
    lo: [f64; N],
    hi: [f64; N],
) -> [f64; N] {
    let mut at: [f64; N] = core::array::from_fn(|k| 0.5 * (lo[k] + hi[k]));
    let mut last = f64::INFINITY;
    for _ in 0..POLISH_STEPS {
        let f = system.map(|p| p.eval(at));
        let j = partials
            .each_ref()
            .map(|row| row.each_ref().map(|p| p.eval(at)));
        // A Jacobian with no inverse at the iterate ends it.
        let Some(y) = inverse(j) else {
            break;
        };
        let step: [f64; N] = core::array::from_fn(|i| (0..N).map(|l| y[i][l] * f[l]).sum());
        let size = step.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        if !step.iter().all(|v| v.is_finite()) || size >= last {
            break;
        }
        last = size;
        at = core::array::from_fn(|k| (at[k] - step[k]).clamp(lo[k], hi[k]));
    }
    at
}

/// How [`merge`] compares the boxes of a parameter space that closes
/// along some axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Wrap<const N: usize> {
    /// The length each axis closes over, `None` where it is open.
    pub(crate) period: [Option<f64>; N],
    /// Where each closed axis's range starts: an `at` comes back in
    /// `[origin, origin + period)`.
    pub(crate) origin: [f64; N],
    /// The rounding of the map that made the coordinates, which two
    /// boxes sharing a face may differ by.
    pub(crate) touch: f64,
}

impl<const N: usize> Wrap<N> {
    /// Every axis open, boxes touching only where they share a face
    /// exactly.
    pub(crate) fn open() -> Self {
        Wrap {
            period: [None; N],
            origin: [0.0; N],
            touch: 0.0,
        }
    }
}

/// One entry for every zero found more than once, ascending by `at`:
/// `crate::bernstein2::merge_wrapped` in `N` dimensions. Two certified
/// zeros are one where either lies in the other's box, the smaller box
/// standing for both; uncertified boxes that touch, within `touch`, are
/// their hull, and one inside a certified box is dropped. Along a closed
/// axis boxes are compared a period either way as well, and `at` comes
/// back in that axis's range with its box around it.
pub(crate) fn merge<const N: usize>(zeros: Vec<Zero<N>>, wrap: &Wrap<N>) -> Vec<Zero<N>> {
    let touch = wrap.touch;
    let mut shifts: Vec<[f64; N]> = vec![[0.0; N]];
    for k in 0..N {
        if let Some(period) = wrap.period[k] {
            shifts = shifts
                .into_iter()
                .flat_map(|s| {
                    [-period, 0.0, period].map(|by| {
                        let mut s = s;
                        s[k] = by;
                        s
                    })
                })
                .collect();
        }
    }
    let order = |a: &Zero<N>, b: &Zero<N>| {
        (0..N)
            .map(|k| a.at[k].total_cmp(&b.at[k]))
            .fold(core::cmp::Ordering::Equal, core::cmp::Ordering::then)
            .then(a.volume().total_cmp(&b.volume()))
    };
    let (mut certified, mut open): (Vec<Zero<N>>, Vec<Zero<N>>) =
        zeros.into_iter().partition(|z| z.certified);

    // Smallest boxes first, so the one kept is the tightest.
    let every = certified.clone();
    certified.sort_by(|a, b| a.volume().total_cmp(&b.volume()).then(order(a, b)));
    let mut kept: Vec<Zero<N>> = Vec::new();
    for z in certified {
        let same = kept.iter().any(|k| {
            shifts
                .iter()
                .any(|&shift| k.holds(z.at, shift) || z.holds(k.at, shift.map(|s| -s)))
        });
        if !same {
            kept.push(z);
        }
    }

    // Hulls of touching boxes, to a fixed point.
    open.sort_by(order);
    loop {
        let mut merged = false;
        let mut hulls: Vec<Zero<N>> = Vec::new();
        for z in open {
            let touching = hulls.iter_mut().find_map(|h| {
                shifts
                    .iter()
                    .find(|shift| {
                        (0..N).all(|k| {
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
    for z in &mut kept {
        for k in 0..N {
            let Some(period) = wrap.period[k] else {
                continue;
            };
            let origin = wrap.origin[k];
            let mut turn = period * ((z.at[k] - origin) / period).floor();
            if z.at[k] - origin - turn >= period {
                // Rounded up to the period from just below the origin.
                turn += period;
                z.at[k] = origin + turn;
            }
            z.at[k] -= turn;
            z.lo[k] -= turn;
            z.hi[k] -= turn;
        }
    }
    kept.sort_by(order);
    kept
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::{Control, Interrupted, Stop, real};

    /// What the measured cost of the isolation is held to, with room to
    /// spare, over the posed systems of degree three in four unknowns
    /// (`bernstein_n_every_simple_zero_of_a_posed_system_is_certified_alone`).
    /// The depth, in halvings over all axes: 23 to 44, growing with the
    /// zeros' condition number (10 to 860), since the certificate wants
    /// a box whose Jacobian varies by less than its inverse's size. The
    /// boxes: 989 at the most. Without the preconditioned exclusion
    /// (`excluded_together`) the same systems took up to 10 263 boxes,
    /// and seven of 31 outgrew the front.
    const MEASURED_DEPTH: usize = 56;
    const MEASURED_BOXES: usize = 2000;

    /// `(x − a)²` in Bernstein form of degree two on `[0, 1]`.
    fn square(a: f64) -> [f64; 3] {
        [a * a, a * a - a, (1.0 - a) * (1.0 - a)]
    }

    /// `|x − c|² − ρ²` on `[0, 1]ᴺ`, of degree two along every axis.
    fn sphere<const N: usize>(c: [f64; N], rho: f64) -> PolyN<N> {
        let ones = [1.0; 3];
        let terms: Vec<PolyN<N>> = (0..N)
            .map(|k| {
                let sq = square(c[k]);
                PolyN::outer(core::array::from_fn(|j| {
                    if j == k { &sq[..] } else { &ones[..] }
                }))
            })
            .chain([PolyN::outer([&ones[..]; N])])
            .collect();
        let weights: Vec<(f64, &PolyN<N>)> = terms
            .iter()
            .enumerate()
            .map(|(k, p)| (if k < N { 1.0 } else { -rho * rho }, p))
            .collect();
        PolyN::combine(&weights)
    }

    /// `Πₖ (1 + aₖ·xₖ)`, positive on the cube for `aₖ > −1`: what a
    /// rational patch's weight multiplies an equation by.
    fn weight<const N: usize>(a: [f64; N]) -> PolyN<N> {
        let factors: [[f64; 2]; N] = a.map(|a| [1.0, 1.0 + a]);
        PolyN::outer(factors.each_ref().map(|f| &f[..]))
    }

    fn floor<const N: usize>(p: &PolyN<N>) -> f64 {
        64.0 * f64::EPSILON * p.range().magnitude()
    }

    fn dot<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
        (0..N).map(|k| a[k] * b[k]).sum()
    }

    /// `N` spheres through both `p` and `q`, their centres in general
    /// position on the plane halfway between: their common zeros are
    /// exactly `p` and `q`, both simple.
    fn through<const N: usize>(p: [f64; N], q: [f64; N], spread: &[[f64; N]; N]) -> [PolyN<N>; N] {
        let mid: [f64; N] = core::array::from_fn(|k| 0.5 * (p[k] + q[k]));
        let d: [f64; N] = core::array::from_fn(|k| q[k] - p[k]);
        core::array::from_fn(|i| {
            let r = spread[i];
            let along = dot(r, d) / dot(d, d);
            let c: [f64; N] = core::array::from_fn(|k| mid[k] + r[k] - along * d[k]);
            let to_p: [f64; N] = core::array::from_fn(|k| p[k] - c[k]);
            sphere(c, dot(to_p, to_p).sqrt())
        })
    }

    fn isolate<const N: usize>(system: &[PolyN<N>; N]) -> Isolation<N> {
        let floors = system.each_ref().map(floor);
        common_zeros(system.each_ref(), floors, &mut Meter::default()).unwrap()
    }

    fn certified<const N: usize>(found: &Isolation<N>) -> Vec<[f64; N]> {
        let zeros = found.zeros.iter().filter(|z| z.certified);
        zeros.map(|z| z.at).collect()
    }

    fn assert_near<const N: usize>(found: &[[f64; N]], expected: &[[f64; N]], within: f64) {
        assert_eq!(found.len(), expected.len(), "{found:?} vs {expected:?}");
        for (f, e) in found.iter().zip(expected) {
            assert!(
                (0..N).all(|k| (f[k] - e[k]).abs() < within),
                "{found:?} vs {expected:?}"
            );
        }
    }

    /// The expected zeros in the isolation's order.
    fn sorted<const N: usize>(mut zeros: Vec<[f64; N]>) -> Vec<[f64; N]> {
        zeros.sort_by(|a, b| {
            (0..N)
                .map(|k| a[k].total_cmp(&b[k]))
                .fold(core::cmp::Ordering::Equal, core::cmp::Ordering::then)
        });
        zeros
    }

    /// SplitMix64: the seeded sequence the posed systems are drawn from.
    struct Seeded(u64);

    impl Seeded {
        fn next(&mut self) -> f64 {
            self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
            let mut z = self.0;
            z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
            ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
        }

        fn point<const N: usize>(&mut self, lo: f64, hi: f64) -> [f64; N] {
            core::array::from_fn(|_| lo + (hi - lo) * self.next())
        }
    }

    #[test]
    fn bernstein_n_arithmetic_is_the_polynomials_own() {
        let b = Binomials::new(16);
        let c = [0.3, 0.6, 0.45, 0.2];
        let p = sphere(c, 0.25);
        let w = weight([0.5, -0.3, 0.2, 0.7]);
        let pw = p.mul(&w, &b);
        assert_eq!(pw.degree(), [3; 4]);
        let value = |x: [f64; 4]| (0..4).map(|k| real::powi(x[k] - c[k], 2)).sum::<f64>() - 0.0625;
        let weight_at = |x: [f64; 4]| {
            (1.0 + 0.5 * x[0]) * (1.0 - 0.3 * x[1]) * (1.0 + 0.2 * x[2]) * (1.0 + 0.7 * x[3])
        };
        let mut s = Seeded(7);
        for _ in 0..64 {
            let x: [f64; 4] = s.point(0.0, 1.0);
            assert!((p.eval(x) - value(x)).abs() < 1e-15);
            assert!((pw.eval(x) - value(x) * weight_at(x)).abs() < 1e-14);
            for k in 0..4 {
                assert!((p.partial(k).eval(x) - 2.0 * (x[k] - c[k])).abs() < 1e-14);
                // The halves of the cube along `k`, and the grown cube.
                let (lo, hi) = pw.split(k, 0.5);
                let mut at = x;
                at[k] = 0.5 * x[k];
                assert!((lo.eval(x) - pw.eval(at)).abs() < 1e-14);
                at[k] = 0.5 + 0.5 * x[k];
                assert!((hi.eval(x) - pw.eval(at)).abs() < 1e-14);
            }
            let at = x.map(|v| -MARGIN + (1.0 + 2.0 * MARGIN) * v);
            assert!((pw.grown().eval(x) - value(at) * weight_at(at)).abs() < 1e-13);
        }
        let constant = PolyN::<3>::outer([&[2.0], &[1.0, 1.0], &[1.0]]);
        assert_eq!(constant.partial(0).coefficients(), [0.0, 0.0]);
        assert_eq!(constant.partial(1).coefficients(), [0.0]);
    }

    #[test]
    fn bernstein_n_tensor_is_the_product_of_its_factors() {
        let a = PolyN::new([2, 1], vec![1.0, -2.0, 0.5, 3.0, -1.0, 2.0]);
        let b = PolyN::new([1, 3], vec![0.25, 1.0, -1.5, 2.0, 0.0, 1.0, -0.5, 0.75]);
        let ab = PolyN::tensor(&a, &b);
        assert_eq!(ab.degree(), [2, 1, 1, 3]);
        for x in [[0.1, 0.7, 0.3, 0.9], [0.5, 0.2, 0.8, 0.4]] {
            let product = a.eval([x[0], x[1]]) * b.eval([x[2], x[3]]);
            assert!((ab.eval(x) - product).abs() < 1e-14);
        }
    }

    #[test]
    fn bernstein_n_three_spheres_meet_in_their_two_points_each_certified_alone() {
        let (p, q) = ([0.3, 0.7, 0.4], [0.6, 0.35, 0.55]);
        let spread = [[0.1, 0.2, -0.3], [-0.25, 0.1, 0.15], [0.2, -0.15, 0.05]];
        let found = isolate(&through(p, q, &spread));
        assert_eq!(found.zeros.len(), 2, "{:?}", found.zeros);
        assert_near(&certified(&found), &sorted(vec![p, q]), 1e-13);
    }

    #[test]
    fn bernstein_n_every_simple_zero_of_a_posed_system_is_certified_alone() {
        let b = Binomials::new(16);
        let mut s = Seeded(0x0068);
        let (mut depth, mut boxes) = (0, 0);
        for case in 0..32 {
            let p: [f64; 4] = s.point(0.05, 0.95);
            let q: [f64; 4] = s.point(0.05, 0.95);
            let d: [f64; 4] = core::array::from_fn(|k| q[k] - p[k]);
            if dot(d, d) < 0.04 {
                continue;
            }
            let spread: [[f64; 4]; 4] = core::array::from_fn(|_| s.point(-0.4, 0.4));
            let system = through(p, q, &spread).map(|f| f.mul(&weight(s.point(-0.5, 1.0)), &b));
            let found = isolate(&system);
            assert_eq!(found.zeros.len(), 2, "{case}: {:?}", found.zeros);
            assert_near(&certified(&found), &sorted(vec![p, q]), 1e-12);
            depth = depth.max(found.depth);
            boxes = boxes.max(found.boxes);
        }
        assert!(depth <= MEASURED_DEPTH, "depth {depth}");
        assert!(boxes <= MEASURED_BOXES, "boxes {boxes}");
    }

    #[test]
    fn bernstein_n_a_zero_on_the_lines_between_boxes_is_found_once() {
        // The middle of the cube and a quarter point: corners of boxes at
        // every depth.
        let (p, q) = ([0.5; 4], [0.25, 0.75, 0.25, 0.5]);
        let spread = [
            [0.1, 0.2, -0.3, 0.05],
            [-0.25, 0.1, 0.15, 0.2],
            [0.2, -0.15, 0.05, -0.1],
            [0.05, 0.1, 0.2, -0.3],
        ];
        let found = isolate(&through(p, q, &spread));
        assert_eq!(found.zeros.len(), 2, "{:?}", found.zeros);
        assert_near(&certified(&found), &sorted(vec![p, q]), 1e-13);
    }

    #[test]
    fn bernstein_n_a_double_zero_comes_back_uncertified_never_lost() {
        let b = Binomials::new(8);
        // (x − 0.3)(x − 0.7)², y − 0.4, z − 0.6: a simple zero and a
        // double one.
        let linear = |a: f64| [-a, 1.0 - a];
        let (one, two) = ([1.0], [1.0, 1.0]);
        let along_x = |f: &[f64]| PolyN::<3>::outer([f, &one, &one]);
        let x3 = along_x(&linear(0.7))
            .mul(&along_x(&linear(0.7)), &b)
            .mul(&along_x(&linear(0.3)), &b);
        let y = PolyN::outer([&two, &linear(0.4), &one]);
        let z = PolyN::outer([&two, &one, &linear(0.6)]);
        let found = isolate(&[x3, y, z]);
        assert_near(&certified(&found), &[[0.3, 0.4, 0.6]], 1e-14);
        let open: Vec<&Zero<3>> = found.zeros.iter().filter(|z| !z.certified).collect();
        assert_eq!(open.len(), 1, "{:?}", found.zeros);
        assert!(open[0].holds([0.7, 0.4, 0.6], [0.0; 3]), "{:?}", open[0]);
        assert!(open[0].hi[0] - open[0].lo[0] < 1e-6, "{:?}", open[0]);
    }

    #[test]
    fn bernstein_n_a_curve_of_common_zeros_is_a_continuum() {
        let b = Binomials::new(8);
        // x − 0.3, y − 0.4 and (x − 0.3)(1 + z): the line x = 0.3,
        // y = 0.4 is common to all three.
        let linear = |a: f64| [-a, 1.0 - a];
        let one = [1.0, 1.0];
        let x = PolyN::<3>::outer([&linear(0.3), &one, &one]);
        let y = PolyN::outer([&one, &linear(0.4), &one]);
        let xz = x.mul(&PolyN::outer([&[1.0], &[1.0], &[1.0, 2.0]]), &b);
        let found = common_zeros([&x, &y, &xz], [1e-15; 3], &mut Meter::default());
        assert!(
            matches!(found, Err(Halt::Fault(Continuum { .. }))),
            "{found:?}"
        );
        // The zero polynomial is flat at once: one box, the cube.
        let zero = PolyN::<3>::outer([&[0.0; 2], &[0.0; 2], &[0.0; 2]]);
        let found = common_zeros([&zero, &x, &y], [1e-15; 3], &mut Meter::default()).unwrap();
        assert_eq!(found.zeros.len(), 1);
        assert!(!found.zeros[0].certified);
        assert_eq!(found.boxes, 1);
    }

    #[test]
    fn bernstein_n_a_budget_stops_the_isolation() {
        let (p, q) = ([0.3, 0.7, 0.4], [0.6, 0.35, 0.55]);
        let spread = [[0.1, 0.2, -0.3], [-0.25, 0.1, 0.15], [0.2, -0.15, 0.05]];
        let system = through(p, q, &spread);
        let floors = system.each_ref().map(floor);
        let control = Control::budget(5);
        let found = common_zeros(system.each_ref(), floors, &mut Meter::new(&control));
        assert!(
            matches!(
                found,
                Err(Halt::Stopped(Interrupted {
                    by: Stop::Budget,
                    steps: 5,
                    ..
                }))
            ),
            "{found:?}"
        );
    }

    #[test]
    fn bernstein_n_zeros_found_twice_across_a_seam_are_one() {
        let zero = |at: [f64; 3], half: f64, certified: bool| Zero {
            at,
            lo: at.map(|v| v - half),
            hi: at.map(|v| v + half),
            certified,
        };
        // Axes 0 and 2 close over 6 and 4; axis 1 is open.
        let wrap = Wrap {
            period: [Some(6.0), None, Some(4.0)],
            origin: [0.0; 3],
            touch: 1e-12,
        };
        let merged = merge(
            vec![
                zero([6.25, 1.0, 1.0], 0.1, true),
                zero([0.25, 1.0, 1.0], 0.05, true),
                zero([-0.01, 3.0, 3.99], 0.2, true),
                zero([5.99, 3.0, -0.01], 0.05, true),
                zero([0.05, 2.0, 2.0], 0.05, false),
                zero([5.95, 2.0, 2.0], 0.05, false),
                // The same `x` and `z` as the last two, but a period away
                // only along the open axis: not the same box.
                zero([0.05, 8.0, 2.0], 0.05, false),
            ],
            &wrap,
        );
        assert_eq!(merged.len(), 4, "{merged:?}");
        let at: Vec<[f64; 3]> = merged.iter().map(|z| z.at).collect();
        assert!(at[0][0].abs() < 1e-12 && at[0][1] == 2.0, "{at:?}");
        assert!(!merged[0].certified);
        assert!((merged[0].hi[0] - merged[0].lo[0] - 0.2).abs() < 1e-12);
        assert_eq!(at[1], [0.05, 8.0, 2.0]);
        assert!((at[2][0] - 0.25).abs() < 1e-12 && at[2][1] == 1.0, "{at:?}");
        assert!(
            at[3][0] == 5.99 && (at[3][2] - 3.99).abs() < 1e-12,
            "{at:?}"
        );
    }
}
