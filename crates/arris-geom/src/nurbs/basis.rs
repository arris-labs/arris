//! Knot vectors and B-spline basis functions: validation, span search and
//! the basis functions with their derivatives to second order, computed
//! into stack buffers (*The NURBS Book* §2.5, algorithms A2.1–A2.3, written
//! for our bounded degree and our error reporting).

/// The largest degree a NURBS curve or surface may have. It is Open
/// CASCADE's bound (`Geom_BSplineCurve::MaxDegree`), so anything a STEP
/// file carries fits, and it sizes the evaluator's stack buffers, so
/// evaluation never allocates. A structural bound, not a tolerance.
pub const MAX_DEGREE: usize = 25;

/// How many derivative orders the evaluators compute: the point, the
/// first and the second derivative.
pub(crate) const ORDERS: usize = 3;

/// One row of basis values or derivatives for the `p + 1` functions
/// active on a span, indexed from the first active one.
pub(crate) type Row = [f64; MAX_DEGREE + 1];

/// Why `knots` is not a knot vector for `degree` and `n` control points,
/// as the reason of a `GeomError::Degenerate`. The rules: the degree is
/// in `1..=MAX_DEGREE`; there are at least `degree + 1` control points
/// and exactly `n + degree + 1` finite, non-decreasing knots; the domain
/// `[knots[degree], knots[n]]` has positive length and its last span is
/// not empty (so evaluation at the domain's end divides by nothing); a
/// span that is not empty is at least `f64::MIN_POSITIVE`, a normal
/// number, since every derivative divides by it and a subnormal one's
/// reciprocal is past `f64` — a curve over `[0, 5e-315]` evaluated to
/// NaN. Two knots a rounding apart at an ordinary scale stay valid: the
/// kernel's own splits and fits make them; no knot
/// value has multiplicity above `degree + 1`, and none strictly inside
/// the domain has multiplicity above `degree` (a higher one would break
/// the curve).
pub(crate) fn validate(degree: usize, knots: &[f64], n: usize) -> Result<(), String> {
    if degree == 0 || degree > MAX_DEGREE {
        return Err(format!("degree {degree} is not in 1..={MAX_DEGREE}"));
    }
    if n < degree + 1 {
        return Err(format!(
            "{n} control points for degree {degree}: at least {} are needed",
            degree + 1
        ));
    }
    if knots.len() != n + degree + 1 {
        return Err(format!(
            "{} knots for degree {degree} and {n} control points: {} are needed",
            knots.len(),
            n + degree + 1
        ));
    }
    if let Some(i) = knots.iter().position(|k| !k.is_finite()) {
        return Err(format!("knot {i} is not finite"));
    }
    if let Some(i) = knots.windows(2).position(|w| w[0] > w[1]) {
        return Err(format!(
            "knots {i} and {} decrease: {} > {}",
            i + 1,
            knots[i],
            knots[i + 1]
        ));
    }
    if let Some(i) = knots
        .windows(2)
        .position(|w| w[1] > w[0] && w[1] - w[0] < f64::MIN_POSITIVE)
    {
        return Err(format!(
            "knots {i} and {} are a subnormal span apart: {} and {}",
            i + 1,
            knots[i],
            knots[i + 1]
        ));
    }
    let (lo, hi) = (knots[degree], knots[n]);
    if lo >= hi {
        return Err(format!("the domain [{lo}, {hi}] is empty"));
    }
    if knots[n - 1] == hi {
        return Err(format!("the span ending at the domain's end {hi} is empty"));
    }
    let mut i = 0;
    while i < knots.len() {
        let value = knots[i];
        let run = knots[i..].iter().take_while(|&&k| k == value).count();
        if run > degree + 1 {
            return Err(format!(
                "knot {value} has multiplicity {run}, above degree + 1 = {}",
                degree + 1
            ));
        }
        if run > degree && value > lo && value < hi {
            return Err(format!(
                "interior knot {value} has multiplicity {run}, above the degree {degree}"
            ));
        }
        i += run;
    }
    Ok(())
}

/// The span `s` with `knots[s] ≤ t < knots[s + 1]` inside the domain,
/// clamped to `[degree, n − 1]`: a `t` at or past the domain's end lands
/// on the last span, one before its start on the first, so the polynomial
/// piece nearest `t` is the one evaluated. Every span this returns is
/// non-empty when the knots passed [`validate`].
pub(crate) fn span(degree: usize, knots: &[f64], n: usize, t: f64) -> usize {
    let count = knots[degree..=n].partition_point(|&k| k <= t);
    (degree + count).saturating_sub(1).clamp(degree, n - 1)
}

/// The knot spans of `[knots[degree], knots[n]]` whose width is at least
/// [`arris_math::KNOT_RESOLUTION`]: narrower spans are sub-resolution
/// and omitted so ill-conditioned Bézier pieces are not extracted before
/// isolation. If every span is narrower than resolution, the single widest
/// non-empty span is retained.
pub(crate) fn resolvable_spans(degree: usize, knots: &[f64], n: usize) -> Vec<usize> {
    let mut valid: Vec<usize> = (degree..n)
        .filter(|&i| knots[i + 1] - knots[i] >= arris_math::KNOT_RESOLUTION)
        .collect();
    if valid.is_empty() {
        if let Some(i) = (degree..n)
            .max_by(|&a, &b| (knots[a + 1] - knots[a]).total_cmp(&(knots[b + 1] - knots[b])))
        {
            if knots[i + 1] > knots[i] {
                valid.push(i);
            }
        }
    }
    valid
}

/// The `degree + 1` basis functions active on `span` at `t`, with their
/// first and second derivatives: `out[k][j]` is the `k`-th derivative of
/// `N_{span − degree + j}` (A2.3). Derivatives of order above the degree
/// are zero.
// The triangular tables are indexed by two or three loop variables at
// once, as the algorithm is written; an iterator form would hide which
// entry is which.
#[allow(clippy::needless_range_loop)]
pub(crate) fn derivatives(degree: usize, knots: &[f64], span: usize, t: f64) -> [Row; ORDERS] {
    let p = degree;
    // `ndu[j][r]` holds the basis functions of degree `j` in its upper
    // triangle and the knot differences they divide by in its lower one.
    let mut ndu = [[0.0; MAX_DEGREE + 1]; MAX_DEGREE + 1];
    let mut left: Row = [0.0; MAX_DEGREE + 1];
    let mut right: Row = [0.0; MAX_DEGREE + 1];
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = t - knots[span + 1 - j];
        right[j] = knots[span + j] - t;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    let mut out = [[0.0; MAX_DEGREE + 1]; ORDERS];
    for j in 0..=p {
        out[0][j] = ndu[j][p];
    }
    let orders = (ORDERS - 1).min(p);
    let mut a = [[0.0; MAX_DEGREE + 1]; 2];
    for r in 0..=p {
        let (mut s1, mut s2) = (0, 1);
        a[0][0] = 1.0;
        for k in 1..=orders {
            let mut d = 0.0;
            let rk = r as isize - k as isize;
            let pk = p - k;
            if r >= k {
                a[s2][0] = a[s1][0] / ndu[pk + 1][(rk) as usize];
                d = a[s2][0] * ndu[rk as usize][pk];
            }
            let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
            let j2 = if r <= pk + 1 { k - 1 } else { p - r };
            for j in j1..=j2 {
                let idx = (rk + j as isize) as usize;
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][idx];
                d += a[s2][j] * ndu[idx][pk];
            }
            if r <= pk {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            out[k][r] = d;
            core::mem::swap(&mut s1, &mut s2);
        }
    }
    let mut factor = p as f64;
    for k in 1..=orders {
        for value in out[k].iter_mut().take(p + 1) {
            *value *= factor;
        }
        factor *= (p - k) as f64;
    }
    out
}

/// The number of knots equal to `t`, exactly.
pub(crate) fn multiplicity(knots: &[f64], t: f64) -> usize {
    knots.iter().filter(|&&k| k == t).count()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_clamped_cubic_passes_and_the_broken_ones_name_their_fault() {
        let knots = [0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0];
        assert_eq!(validate(3, &knots, 5), Ok(()));
        assert!(validate(0, &knots, 5).unwrap_err().contains("degree 0"));
        assert!(validate(3, &knots, 4).unwrap_err().contains("9 knots"));
        // Unclamped at the far end is still a knot vector.
        assert_eq!(validate(3, &knots[..8], 4), Ok(()));
        let decreasing = [0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 2.0, 2.0, 2.0];
        assert!(
            validate(3, &decreasing, 5)
                .unwrap_err()
                .contains("decrease")
        );
        let heavy = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0];
        assert!(
            validate(3, &heavy, 8)
                .unwrap_err()
                .contains("interior knot 1 has multiplicity 4")
        );
        let empty_end = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0];
        assert!(validate(3, &empty_end, 5).unwrap_err().contains("is empty"));
        let flat = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert!(validate(3, &flat, 4).unwrap_err().contains("is empty"));
        let nan = [0.0, 0.0, f64::NAN, 1.0, 1.0, 1.0];
        assert!(validate(2, &nan, 3).unwrap_err().contains("not finite"));
        // Found by the `intersect_curves` fuzz target (`fuzz/`, ADR-0024
        // §5): a domain of 5e-315 passed, and every derivative over it
        // was infinite.
        let subnormal = [0.0, 0.0, 0.0, 5e-315, 5e-315, 5e-315];
        assert!(
            validate(2, &subnormal, 3)
                .unwrap_err()
                .contains("subnormal span")
        );
        // A rounding apart at an ordinary scale is a knot vector: the
        // kernel's splits and fits make such (the differential at 1000
        // draws of the fixed seed, where refusing it cost two results).
        let rounding = [1.0, 1.0, 1.0, 1.0 + f64::EPSILON, 2.0, 2.0, 2.0];
        assert_eq!(validate(2, &rounding, 4), Ok(()));
        let short = [0.0, 0.0, 0.0, 1e-9, 1e-9, 1e-9];
        assert_eq!(validate(2, &short, 3), Ok(()));
    }

    #[test]
    fn spans_land_inside_the_domain() {
        let knots = [0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 3.0, 3.0, 3.0];
        let n = 6;
        assert_eq!(span(2, &knots, n, -1.0), 2);
        assert_eq!(span(2, &knots, n, 0.0), 2);
        assert_eq!(span(2, &knots, n, 0.5), 2);
        assert_eq!(span(2, &knots, n, 1.0), 3);
        assert_eq!(span(2, &knots, n, 2.0), 5);
        assert_eq!(span(2, &knots, n, 2.5), 5);
        assert_eq!(span(2, &knots, n, 3.0), 5);
        assert_eq!(span(2, &knots, n, 9.0), 5);
    }

    #[test]
    fn basis_functions_sum_to_one_and_their_derivatives_to_zero() {
        let knots = [0.0, 0.0, 0.0, 0.0, 1.0, 2.5, 4.0, 4.0, 4.0, 4.0];
        let n = 6;
        for &t in &[0.0, 0.3, 1.0, 2.0, 3.9, 4.0] {
            let s = span(3, &knots, n, t);
            let d = derivatives(3, &knots, s, t);
            let sum: f64 = d[0][..4].iter().sum();
            assert!((sum - 1.0).abs() < 1e-15, "t = {t}: sum {sum}");
            assert!(d[1][..4].iter().sum::<f64>().abs() < 1e-13);
            assert!(d[2][..4].iter().sum::<f64>().abs() < 1e-12);
            assert!(d[0][..4].iter().all(|&v| v >= 0.0));
        }
    }

    #[test]
    fn derivatives_match_the_quadratic_bernstein_polynomials() {
        // On [0, 0, 0, 1, 1, 1] the basis is Bernstein: (1−t)², 2t(1−t), t².
        let knots = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
        let t = 0.3;
        let d = derivatives(2, &knots, span(2, &knots, 3, t), t);
        let b = [(1.0 - t) * (1.0 - t), 2.0 * t * (1.0 - t), t * t];
        let db = [-2.0 * (1.0 - t), 2.0 - 4.0 * t, 2.0 * t];
        let ddb = [2.0, -4.0, 2.0];
        for i in 0..3 {
            assert!((d[0][i] - b[i]).abs() < 1e-15);
            assert!((d[1][i] - db[i]).abs() < 1e-15);
            assert!((d[2][i] - ddb[i]).abs() < 1e-14);
        }
        // A linear basis has no second derivative.
        let linear = [0.0, 0.0, 1.0, 1.0];
        let d = derivatives(1, &linear, 1, 0.25);
        assert_eq!(d[2][..2], [0.0, 0.0]);
        assert_eq!(d[1][..2], [-1.0, 1.0]);
    }

    #[test]
    fn sub_resolution_spans_are_merged_into_their_neighbours() {
        let knots = [
            0.0,
            0.0,
            0.0,
            0.0,
            3.645561009778199e-304,
            2.0,
            3.0,
            4.0,
            5.0,
            5.0,
            5.0,
            5.0,
        ];
        let n = 8;
        let spans = resolvable_spans(3, &knots, n);
        // Span 3 (width 1e-15 < 1e-7) is omitted; spans 4, 5, 6, 7 are kept.
        assert_eq!(spans, vec![4, 5, 6, 7]);

        let knots_trail = [
            0.0,
            0.0,
            0.0,
            0.0,
            2.0,
            2.0 + 1e-15,
            2.0 + 1e-15,
            2.0 + 1e-15,
            2.0 + 1e-15,
        ];
        let spans_trail = resolvable_spans(3, &knots_trail, 5);
        assert_eq!(spans_trail, vec![3]);
    }
}
