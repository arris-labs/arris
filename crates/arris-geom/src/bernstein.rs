//! Polynomials in Bernstein form on `[0, 1]`: the product, the
//! derivative, de Casteljau's evaluation and subdivision, and the
//! isolation of sign changes by the variation of the coefficients'
//! signs. A rational B-spline span substituted into a surface's implicit
//! polynomial is one of these (`crate::intersect_spline`), and the
//! Bernstein form is what makes that robust: every operation here is a
//! convex combination or a sum of products with positive factors, so a
//! coefficient's rounding stays a multiple of the operands' own, and the
//! coefficients bound the polynomial over the interval. Farouki and
//! Rajan, *Algorithms for polynomials in Bernstein form* (CAGD 5, 1988),
//! is the reference for the arithmetic; the isolation is the classical
//! subdivision on Descartes' rule in the Bernstein basis (Lane and
//! Riesenfeld, 1981).

use arris_math::Interval;
use arris_math::roots::newton_in_interval;

/// How many times the isolation halves `[0, 1]` before it takes what is
/// left of a cluster of sign changes for one parameter: `2⁻⁴⁸` of a span
/// is a few ulps of any parameter inside it, and no evaluation tells two
/// parameters that close apart. A structural bound, not a tolerance.
pub(crate) const MAX_DEPTH: usize = 48;

/// The arithmetic a substitution into an implicit form
/// (`crate::implicit`) needs of polynomials in Bernstein form: the
/// product and the linear combination, in however many variables. Both
/// are sums of products with positive factors, which is what the
/// rounding floor of the result counts on.
pub(crate) trait BernsteinAlgebra<P> {
    /// The product, of the degrees' sum.
    fn mul(&self, a: &P, b: &P) -> P;

    /// `Σ kᵢ · pᵢ` over polynomials of one degree.
    fn combine(&self, terms: &[(f64, &P)]) -> P;
}

/// Pascal's triangle as `f64`, up to the largest degree a caller
/// multiplies to. `C(100, 50) ≈ 1e29` is far inside `f64`'s range, and a
/// row's rounding is relative, which a ratio of three of them keeps.
pub(crate) struct Binomials {
    rows: Vec<Vec<f64>>,
}

impl Binomials {
    /// The rows `0..=degree`.
    pub(crate) fn new(degree: usize) -> Self {
        let mut rows: Vec<Vec<f64>> = Vec::with_capacity(degree + 1);
        for n in 0..=degree {
            let mut row = vec![1.0; n + 1];
            if let Some(above) = rows.last() {
                for k in 1..n {
                    row[k] = above[k - 1] + above[k];
                }
            }
            rows.push(row);
        }
        Binomials { rows }
    }

    /// `C(n, k)`; zero outside the table, which no caller reaches.
    pub(crate) fn get(&self, n: usize, k: usize) -> f64 {
        self.rows
            .get(n)
            .and_then(|row| row.get(k))
            .copied()
            .unwrap_or(0.0)
    }
}

/// The product of two polynomials in Bernstein form, of degree `m + n`:
/// `c_k = Σ_{i + j = k} C(m, i) C(n, j) / C(m + n, k) · a_i b_j`.
pub(crate) fn mul(a: &[f64], b: &[f64], binomials: &Binomials) -> Vec<f64> {
    let (m, n) = (a.len().saturating_sub(1), b.len().saturating_sub(1));
    let mut c = vec![0.0; m + n + 1];
    for (i, &ai) in a.iter().enumerate() {
        for (j, &bj) in b.iter().enumerate() {
            c[i + j] +=
                binomials.get(m, i) * binomials.get(n, j) / binomials.get(m + n, i + j) * ai * bj;
        }
    }
    c
}

/// `Σ kᵢ · pᵢ` over polynomials of one degree.
pub(crate) fn combine(terms: &[(f64, &[f64])]) -> Vec<f64> {
    let len = terms.first().map_or(0, |(_, p)| p.len());
    (0..len)
        .map(|i| {
            terms
                .iter()
                .map(|(k, p)| k * p.get(i).copied().unwrap_or(0.0))
                .sum()
        })
        .collect()
}

impl BernsteinAlgebra<Vec<f64>> for Binomials {
    fn mul(&self, a: &Vec<f64>, b: &Vec<f64>) -> Vec<f64> {
        mul(a, b, self)
    }

    fn combine(&self, terms: &[(f64, &Vec<f64>)]) -> Vec<f64> {
        let terms: Vec<(f64, &[f64])> = terms.iter().map(|(k, p)| (*k, p.as_slice())).collect();
        combine(&terms)
    }
}

/// The derivative, of degree `n − 1`: `n (c_{i+1} − c_i)`. A constant's
/// is the constant zero.
pub(crate) fn derivative(c: &[f64]) -> Vec<f64> {
    let n = c.len().saturating_sub(1);
    if n == 0 {
        return vec![0.0];
    }
    c.windows(2).map(|w| n as f64 * (w[1] - w[0])).collect()
}

/// The value at `s` by de Casteljau's triangle: convex combinations only
/// for `s` in `[0, 1]`, and the end coefficients themselves at the ends.
fn eval(c: &[f64], s: f64) -> f64 {
    let mut row = c.to_vec();
    for level in 1..row.len() {
        for i in 0..row.len() - level {
            row[i] = (1.0 - s) * row[i] + s * row[i + 1];
        }
    }
    row[0]
}

/// The two halves of `c` at `½`, each in Bernstein form on its own
/// `[0, 1]`: the sides of de Casteljau's triangle.
fn halves(c: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let n = c.len();
    let mut row = c.to_vec();
    let mut left = Vec::with_capacity(n);
    let mut right = vec![0.0; n];
    for level in 0..n {
        left.push(row[0]);
        right[n - 1 - level] = row[n - 1 - level];
        for i in 0..n - 1 - level {
            row[i] = 0.5 * (row[i] + row[i + 1]);
        }
    }
    (left, right)
}

/// How many times the coefficients change sign, those within `floor` of
/// zero left out: by Descartes' rule in the Bernstein basis, no fewer
/// than the polynomial's roots in `(0, 1)` and of the same parity.
fn variations(c: &[f64], floor: f64) -> usize {
    let mut count = 0;
    let mut last: Option<bool> = None;
    for &v in c.iter().filter(|v| v.abs() > floor) {
        let negative = v < 0.0;
        if last.is_some_and(|l| l != negative) {
            count += 1;
        }
        last = Some(negative);
    }
    count
}

/// Parameters in `[0, 1]`, ascending, that hold every sign change of the
/// polynomial `c`: a change of sign by more than `floor` — the rounding
/// of the coefficients, the caller's to state — has one of them within
/// rounding of it. The converse is not promised: a parameter returned
/// need be no root. Where the coefficients are all within `floor` the
/// polynomial is zero as far as `f64` can tell and the middle of that
/// stretch stands for it; a cluster of changes that [`MAX_DEPTH`]
/// halvings do not separate is its middle too; and a half whose end
/// falls within `floor` is remembered by that end, since neither side's
/// variations would see a change exactly there. A caller after the
/// extrema of a function passes its derivative, and loses nothing by a
/// parameter that is none: that only splits a monotone stretch.
///
/// A stretch with one variation whose end coefficients both clear the
/// floor holds exactly one simple root, polished by bracketed Newton on
/// de Casteljau's evaluation.
pub(crate) fn sign_change_candidates(c: &[f64], floor: f64) -> Vec<f64> {
    let mut found = Vec::new();
    let mut stack: Vec<(Vec<f64>, f64, f64, usize)> = vec![(c.to_vec(), 0.0, 1.0, 0)];
    while let Some((c, lo, hi, depth)) = stack.pop() {
        let mid = 0.5 * (lo + hi);
        if c.iter().all(|v| v.abs() <= floor) {
            found.push(mid);
            continue;
        }
        let count = variations(&c, floor);
        if count == 0 {
            continue;
        }
        let clear_ends =
            c.first().is_some_and(|v| v.abs() > floor) && c.last().is_some_and(|v| v.abs() > floor);
        if count == 1 && clear_ends {
            let d = derivative(&c);
            let root = newton_in_interval(|s| eval(&c, s), |s| eval(&d, s), Interval::UNIT, 0.0)
                .unwrap_or(0.5);
            found.push(lo + (hi - lo) * root);
            continue;
        }
        if depth == MAX_DEPTH {
            found.push(mid);
            continue;
        }
        let (left, right) = halves(&c);
        if right.first().is_some_and(|v| v.abs() <= floor) {
            found.push(mid);
        }
        stack.push((right, mid, hi, depth + 1));
        stack.push((left, lo, mid, depth + 1));
    }
    found.sort_by(f64::total_cmp);
    found.dedup();
    found
}

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

#[cfg(test)]
mod tests {
    use super::*;

    /// The Bernstein coefficients of a polynomial given by its power
    /// coefficients, ascending: `c_k = Σ_{i ≤ k} C(k, i) / C(n, i) · a_i`.
    fn from_power(a: &[f64]) -> Vec<f64> {
        let n = a.len() - 1;
        let b = Binomials::new(n);
        (0..=n)
            .map(|k| (0..=k).map(|i| b.get(k, i) / b.get(n, i) * a[i]).sum())
            .collect()
    }

    #[test]
    fn a_product_evaluates_to_the_product() {
        let b = Binomials::new(8);
        let p = from_power(&[1.0, -3.0, 2.0]);
        let q = from_power(&[0.5, 0.0, -1.0, 4.0]);
        let pq = mul(&p, &q, &b);
        assert_eq!(pq.len(), 6);
        for i in 0..=10 {
            let s = i as f64 / 10.0;
            assert!((eval(&pq, s) - eval(&p, s) * eval(&q, s)).abs() < 1e-14);
        }
        let sum = combine(&[(2.0, &p), (-1.0, &p)]);
        assert!((eval(&sum, 0.3) - eval(&p, 0.3)).abs() < 1e-15);
    }

    #[test]
    fn the_derivative_and_the_halves_are_the_polynomials_own() {
        let p = from_power(&[0.0, 1.0, 0.0, -2.0]); // s − 2s³
        let d = derivative(&p); // 1 − 6s²
        for i in 0..=10 {
            let s = i as f64 / 10.0;
            assert!((eval(&d, s) - (1.0 - 6.0 * s * s)).abs() < 1e-14);
        }
        assert_eq!(derivative(&[3.0]), vec![0.0]);
        let (l, r) = halves(&p);
        for i in 0..=10 {
            let s = i as f64 / 10.0;
            assert!((eval(&l, s) - eval(&p, 0.5 * s)).abs() < 1e-15);
            assert!((eval(&r, s) - eval(&p, 0.5 + 0.5 * s)).abs() < 1e-15);
        }
    }

    #[test]
    fn every_simple_root_is_found_and_polished() {
        // (s − 0.1)(s − 0.35)(s − 0.36)(s − 0.9), and a root outside.
        let roots = [0.1, 0.35, 0.36, 0.9];
        let mut a = vec![1.0];
        for r in roots.iter().chain(&[1.5]) {
            let mut next = vec![0.0; a.len() + 1];
            for (i, c) in a.iter().enumerate() {
                next[i] -= r * c;
                next[i + 1] += c;
            }
            a = next;
        }
        let found = sign_change_candidates(&from_power(&a), 1e-15);
        assert_eq!(found.len(), 4, "{found:?}");
        for (f, r) in found.iter().zip(roots) {
            assert!((f - r).abs() < 1e-12, "{f} vs {r}");
        }
    }

    #[test]
    fn a_root_at_the_middle_and_a_zero_polynomial_are_candidates() {
        // (s − ½)(s − ¼)(s − ¾): the first halving lands on a root.
        let p = from_power(&[-0.09375, 0.6875, -1.5, 1.0]);
        let found = sign_change_candidates(&p, 1e-15);
        assert!(found.iter().any(|s| (s - 0.5).abs() < 1e-12), "{found:?}");
        assert!(found.iter().any(|s| (s - 0.25).abs() < 1e-12), "{found:?}");
        assert!(found.iter().any(|s| (s - 0.75).abs() < 1e-12), "{found:?}");
        assert_eq!(sign_change_candidates(&[0.0, 0.0, 0.0], 1e-15), vec![0.5]);
        // No sign change, no candidate: (s − ½)² + 0.01.
        let clear = from_power(&[0.26, -1.0, 1.0]);
        assert!(sign_change_candidates(&clear, 1e-15).is_empty());
        // A double root changes no sign, and whether it is a candidate
        // is not promised; a triple one does, and is.
        let triple = from_power(&[-0.027, 0.27, -0.9, 1.0]); // (s − 0.3)³
        let found = sign_change_candidates(&triple, 1e-15);
        assert!(found.iter().any(|s| (s - 0.3).abs() < 1e-4), "{found:?}");
    }
}
