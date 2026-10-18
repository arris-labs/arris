//! The rational B-spline shared by curves in 2D and 3D: validation,
//! evaluation with derivatives to second order, the period its knots
//! imply, knot insertion and projection. Dimension-generic so
//! `NurbsCurve` and `NurbsCurve2` are the same code.

use arris_math::nalgebra::{Const, OPoint, SVector};
use arris_math::roots::newton_in_interval;
use arris_math::{Interval, is_negligible};

use super::basis::{self, ORDERS};

/// A rational B-spline in `D` dimensions: degree, knots, Cartesian control
/// points and their positive weights, and the period the knots imply.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Spline<const D: usize> {
    degree: usize,
    knots: Vec<f64>,
    points: Vec<OPoint<f64, Const<D>>>,
    weights: Vec<f64>,
    period: Option<f64>,
}

/// A point of a spline with its first two derivatives.
pub(crate) struct Derivatives<const D: usize> {
    pub point: OPoint<f64, Const<D>>,
    pub d1: SVector<f64, D>,
    pub d2: SVector<f64, D>,
}

/// One polynomial piece of a spline in Bernstein form over `[lo, hi]`:
/// the homogeneous Bézier control points `(w P, w)`.
pub(crate) struct BezierSpan<const D: usize> {
    pub lo: f64,
    pub hi: f64,
    pub control: Vec<(SVector<f64, D>, f64)>,
}

impl<const D: usize> Spline<D> {
    /// A validated spline; the error is the reason for
    /// `GeomError::Degenerate`.
    pub(crate) fn new(
        degree: usize,
        knots: Vec<f64>,
        points: Vec<OPoint<f64, Const<D>>>,
        weights: Vec<f64>,
    ) -> Result<Self, String> {
        basis::validate(degree, &knots, points.len())?;
        if weights.len() != points.len() {
            return Err(format!(
                "{} weights for {} control points",
                weights.len(),
                points.len()
            ));
        }
        if let Some(i) = points
            .iter()
            .position(|p| !p.coords.iter().all(|c| c.is_finite()))
        {
            return Err(format!("control point {i} is not finite"));
        }
        if let Some(i) = weights.iter().position(|w| !(w.is_finite() && *w > 0.0)) {
            return Err(format!(
                "weight {i} is {}: weights are finite and positive",
                weights[i]
            ));
        }
        let period = detect_period(degree, &knots, points.len(), |a, b| {
            let scale = points.iter().map(|p| p.coords.norm()).fold(0.0, f64::max);
            let wscale = weights.iter().copied().fold(0.0, f64::max);
            is_negligible((points[a] - points[b]).norm(), scale)
                && is_negligible(weights[a] - weights[b], wscale)
        });
        Ok(Spline {
            degree,
            knots,
            points,
            weights,
            period,
        })
    }

    pub(crate) fn degree(&self) -> usize {
        self.degree
    }

    pub(crate) fn knots(&self) -> &[f64] {
        &self.knots
    }

    pub(crate) fn points(&self) -> &[OPoint<f64, Const<D>>] {
        &self.points
    }

    pub(crate) fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// `[knots[p], knots[n]]`; the constructor made it non-empty and
    /// ordered, so the fallback is unreachable.
    pub(crate) fn domain(&self) -> Interval {
        Interval::new(self.knots[self.degree], self.knots[self.points.len()])
            .unwrap_or(Interval::UNIT)
    }

    pub(crate) fn period(&self) -> Option<f64> {
        self.period
    }

    /// The distinct knots strictly inside `range`, ascending: where the
    /// curve's polynomial pieces change over it. A periodic spline takes a
    /// range wherever it lies — an edge's past the knots' end among them —
    /// so beyond the stored knots its domain's knots are repeated by whole
    /// periods; inside them the stored values are returned as they are. A
    /// range longer than two periods is no range of a periodic curve, and
    /// gets the stored knots alone rather than a count that grows with it.
    pub(crate) fn breaks_within(&self, range: Interval) -> Vec<f64> {
        let inside = |k: f64| range.lo() < k && k < range.hi();
        let mut out: Vec<f64> = self.knots.iter().copied().filter(|&k| inside(k)).collect();
        if let (Some(period), true) = (self.period, range.is_bounded()) {
            let (first, last) = (self.knots[0], self.knots[self.knots.len() - 1]);
            let domain = self.domain();
            if range.length() <= 2.0 * period && (range.lo() < first || range.hi() > last) {
                let base: Vec<f64> = self
                    .knots
                    .iter()
                    .copied()
                    .filter(|&k| domain.lo() <= k && k < domain.hi())
                    .collect();
                let mut shift = ((range.lo() - domain.hi()) / period).floor();
                let end = ((range.hi() - domain.lo()) / period).ceil();
                while shift <= end {
                    for &k in &base {
                        let x = k + shift * period;
                        if inside(x) && (x < first || x > last) {
                            out.push(x);
                        }
                    }
                    shift += 1.0;
                }
                out.sort_by(f64::total_cmp);
            }
        }
        out.dedup();
        out
    }

    /// `t` moved into `[knots[p], knots[n])` by the period; unchanged for
    /// a curve without one.
    pub(crate) fn wrap(&self, t: f64) -> f64 {
        match self.period {
            Some(period) => {
                let lo = self.knots[self.degree];
                let w = lo + (t - lo).rem_euclid(period);
                if w >= self.knots[self.points.len()] {
                    lo
                } else {
                    w
                }
            }
            None => t,
        }
    }

    /// The weighted sums `Σ N_i^(k) w_i P_i` and `Σ N_i^(k) w_i` for
    /// `k = 0, 1, 2` on the span holding `t`.
    #[allow(clippy::needless_range_loop)] // `j` indexes the basis rows and the control points
    fn homogeneous(&self, t: f64) -> ([SVector<f64, D>; ORDERS], [f64; ORDERS]) {
        let n = self.points.len();
        self.homogeneous_on(basis::span(self.degree, &self.knots, n, t), t)
    }

    /// [`Self::homogeneous`] of the polynomial piece of `span`, at any
    /// `t`: at a knot, the piece on the chosen side.
    #[allow(clippy::needless_range_loop)] // `j` indexes the basis rows and the control points
    fn homogeneous_on(&self, span: usize, t: f64) -> ([SVector<f64, D>; ORDERS], [f64; ORDERS]) {
        let ders = basis::derivatives(self.degree, &self.knots, span, t);
        let mut a = [SVector::<f64, D>::zeros(); ORDERS];
        let mut w = [0.0; ORDERS];
        let first = span - self.degree;
        for j in 0..=self.degree {
            let i = first + j;
            let wp = self.weights[i] * self.points[i].coords;
            for k in 0..ORDERS {
                a[k] += ders[k][j] * wp;
                w[k] += ders[k][j] * self.weights[i];
            }
        }
        (a, w)
    }

    /// The point and its derivatives at `t`: a periodic spline wraps `t`
    /// first, any other evaluates the polynomial piece nearest `t`.
    pub(crate) fn eval(&self, t: f64) -> Derivatives<D> {
        let (a, w) = self.homogeneous(self.wrap(t));
        Self::quotient(a, w)
    }

    /// The point and its derivatives from the weighted sums, by the
    /// quotient rule.
    fn quotient(a: [SVector<f64, D>; ORDERS], w: [f64; ORDERS]) -> Derivatives<D> {
        let point = a[0] / w[0];
        let d1 = (a[1] - w[1] * point) / w[0];
        let d2 = (a[2] - 2.0 * w[1] * d1 - w[2] * point) / w[0];
        Derivatives {
            point: OPoint::from(point),
            d1,
            d2,
        }
    }

    /// The same spline with `t` inserted `times` more times as a knot
    /// (*The NURBS Book* A5.1 on the homogeneous control points); the
    /// curve is unchanged over its domain. `t` is wrapped into the domain
    /// of a periodic spline; the error is the reason the result would
    /// not be a spline — `t` outside `[knots[p], knots[n])`, or a
    /// multiplicity that would exceed the degree.
    #[allow(clippy::needless_range_loop)] // A5.1's four index passes, as written
    pub(crate) fn insert_knot(&self, t: f64, times: usize) -> Result<Self, String> {
        let p = self.degree;
        let n = self.points.len();
        let t = self.wrap(t);
        if !(t.is_finite() && t >= self.knots[p] && t < self.knots[n]) {
            return Err(format!(
                "knot {t} is outside the domain [{}, {})",
                self.knots[p], self.knots[n]
            ));
        }
        let s = basis::multiplicity(&self.knots, t);
        if times == 0 {
            return Ok(self.clone());
        }
        if s + times > p {
            return Err(format!(
                "inserting {t} {times} more times would give it multiplicity {}, above the degree {p}",
                s + times
            ));
        }
        let k = basis::span(p, &self.knots, n, t);
        let r = times;
        let hom = |i: usize| (self.weights[i] * self.points[i].coords, self.weights[i]);
        let mut knots = Vec::with_capacity(self.knots.len() + r);
        knots.extend_from_slice(&self.knots[..=k]);
        knots.extend(core::iter::repeat_n(t, r));
        knots.extend_from_slice(&self.knots[k + 1..]);
        let mut out: Vec<Option<(SVector<f64, D>, f64)>> = vec![None; n + r];
        for i in 0..=k - p {
            out[i] = Some(hom(i));
        }
        for i in k - s..n {
            out[i + r] = Some(hom(i));
        }
        let mut temp: Vec<(SVector<f64, D>, f64)> = (0..=p - s).map(|i| hom(k - p + i)).collect();
        let mut l = k - p;
        for j in 1..=r {
            l = k - p + j;
            for i in 0..=p - j - s {
                let alpha = (t - self.knots[l + i]) / (self.knots[i + k + 1] - self.knots[l + i]);
                let (q1, w1) = temp[i + 1];
                let (q0, w0) = temp[i];
                temp[i] = (
                    alpha * q1 + (1.0 - alpha) * q0,
                    alpha * w1 + (1.0 - alpha) * w0,
                );
            }
            out[l] = Some(temp[0]);
            out[k + r - j - s] = Some(temp[p - j - s]);
        }
        for i in l + 1..k - s {
            out[i] = Some(temp[i - l]);
        }
        let (points, weights): (Vec<_>, Vec<_>) = out
            .into_iter()
            .map(|slot| {
                // Every slot is written by one of the four passes above
                // (A5.1 covers the indices exactly); an unwritten one would
                // be a programming error, and yields the origin rather
                // than a panic.
                let (q, w) = slot.unwrap_or((SVector::zeros(), 1.0));
                (OPoint::from(q / w), w)
            })
            .unzip();
        Spline::new(p, knots, points, weights)
    }

    /// The spline over exactly `[lo, hi]`, clamped there, in the same
    /// parameter: every knot of the range raised to multiplicity `p`
    /// (A5.1), and the control points and knots between them kept. A
    /// periodic spline takes any range of at most one period, wherever
    /// it starts — the range is wrapped, read on the spline unrolled over
    /// two periods, and the piece's knots shifted back by the whole
    /// periods the wrap took; any other spline a range inside its domain.
    /// The error is the reason.
    pub(crate) fn segment(&self, lo: f64, hi: f64) -> Result<Self, String> {
        let p = self.degree;
        if !(lo.is_finite() && hi.is_finite() && lo < hi) {
            return Err(format!("[{lo}, {hi}] is no range"));
        }
        let (given_lo, given_hi) = (lo, hi);
        let (mut s, lo, hi) = match self.period {
            Some(period) => {
                let length = hi - lo;
                if length > period && !is_negligible(length - period, period) {
                    return Err(format!("[{lo}, {hi}] is longer than the period {period}"));
                }
                let lo = self.wrap(lo);
                (self.unrolled(period)?, lo, lo + length.min(period))
            }
            None => {
                let domain = self.domain();
                if lo < domain.lo() || hi > domain.hi() {
                    return Err(format!(
                        "[{lo}, {hi}] is outside the domain [{}, {}]",
                        domain.lo(),
                        domain.hi()
                    ));
                }
                (self.clone(), lo, hi)
            }
        };
        for t in [lo, hi] {
            let already = basis::multiplicity(&s.knots, t);
            if already < p {
                s = s.insert_knot(t, p - already)?;
            }
        }
        let (Some(last_lo), Some(first_hi)) = (
            s.knots.iter().rposition(|&k| k == lo),
            s.knots.iter().position(|&k| k == hi),
        ) else {
            return Err("a knot the insertion put in is missing".to_string());
        };
        // A knot of multiplicity `p` or more is interpolated: the point at
        // `lo` is the control point `p` before its last copy, the one at
        // `hi` the control point before its first.
        let (a, b) = (last_lo - p, first_hi - 1);
        let shift = given_lo - lo;
        let mut knots = vec![given_lo; p + 1];
        knots.extend(s.knots[last_lo + 1..first_hi].iter().map(|k| k + shift));
        knots.extend(core::iter::repeat_n(given_hi, p + 1));
        Spline::new(
            p,
            knots,
            s.points[a..=b].to_vec(),
            s.weights[a..=b].to_vec(),
        )
    }

    /// A periodic spline over two of its periods, as a spline whose knots
    /// and control points run on past the first period's end: the knot
    /// `m = n − p` places on is the knot plus the period, and the control
    /// point `m` places on the same point, as the periodic sequence is.
    fn unrolled(&self, period: f64) -> Result<Self, String> {
        let p = self.degree;
        let n = self.points.len();
        let m = n - p;
        let mut knots = self.knots.clone();
        for j in n + p + 1..n + p + 1 + m {
            knots.push(knots[j - m] + period);
        }
        let mut points = self.points.clone();
        let mut weights = self.weights.clone();
        for j in n..n + m {
            points.push(points[j - m]);
            weights.push(weights[j - m]);
        }
        Spline::new(p, knots, points, weights)
    }

    /// The spline's polynomial pieces in Bernstein form: for every
    /// non-empty span of the domain, ascending, its two knots and the
    /// `p + 1` homogeneous Bézier control points `(w P, w)` of the piece
    /// over it — the blossom of the span at its own ends, `p − k` times
    /// the first and `k` times the second (de Boor's recurrence with a
    /// different argument at each level), which reads unclamped knots as
    /// it reads clamped ones, so a periodic spline needs no unwrapping.
    /// The image of each piece is the spline's over the span to rounding.
    pub(crate) fn bezier_spans(&self) -> Vec<BezierSpan<D>> {
        let p = self.degree;
        let n = self.points.len();
        let spans = basis::resolvable_spans(p, &self.knots, n);
        let mut out = Vec::with_capacity(spans.len());
        for i in spans {
            let (lo, hi) = (self.knots[i], self.knots[i + 1]);
            let mut control = Vec::with_capacity(p + 1);
            for k in 0..=p {
                let mut d: Vec<(SVector<f64, D>, f64)> = (i - p..=i)
                    .map(|j| (self.weights[j] * self.points[j].coords, self.weights[j]))
                    .collect();
                for r in 1..=p {
                    // The last `k` arguments of the blossom are `hi`.
                    let t = if r > p - k { hi } else { lo };
                    for j in (r..=p).rev() {
                        let g = i - p + j;
                        let alpha =
                            (t - self.knots[g]) / (self.knots[g + p - r + 1] - self.knots[g]);
                        d[j] = (
                            (1.0 - alpha) * d[j - 1].0 + alpha * d[j].0,
                            (1.0 - alpha) * d[j - 1].1 + alpha * d[j].1,
                        );
                    }
                }
                control.push(d[p]);
            }
            out.push(BezierSpan { lo, hi, control });
        }
        out
    }

    /// The spline with every control point mapped through `f`.
    pub(crate) fn map_points(
        &self,
        f: impl Fn(&OPoint<f64, Const<D>>) -> OPoint<f64, Const<D>>,
    ) -> Self {
        Spline {
            degree: self.degree,
            knots: self.knots.clone(),
            points: self.points.iter().map(f).collect(),
            weights: self.weights.clone(),
            period: self.period,
        }
    }

    /// The parameter of the nearest point to `q`: every span is sampled,
    /// and bracketed Newton on the derivative of the squared distance runs
    /// in the brackets around the best sample — the nearest local minimum
    /// from that sample, never an error. A periodic spline's neighbours
    /// wrap; a closed clamped one's (first and last point the same to
    /// rounding) do too, so a minimum just past the seam is found from
    /// the seam.
    pub(crate) fn project(&self, q: &OPoint<f64, Const<D>>) -> f64 {
        let p = self.degree;
        let n = self.points.len();
        let per_span = samples_per_span(p);
        let mut samples: Vec<f64> = Vec::new();
        for i in p..n {
            let (lo, hi) = (self.knots[i], self.knots[i + 1]);
            if lo == hi {
                continue;
            }
            for j in 0..per_span {
                samples.push(lo + (hi - lo) * j as f64 / per_span as f64);
            }
        }
        samples.push(self.knots[n]);
        let last = samples.len() - 1;
        let dist2 = |t: f64| (self.eval(t).point - q).norm_squared();
        let mut best = (samples[0], dist2(samples[0]));
        for &t in &samples[1..] {
            let d = dist2(t);
            if d < best.1 {
                best = (t, d);
            }
        }
        let idx = samples
            .iter()
            .position(|&t| t == best.0)
            .unwrap_or_default();
        let (first, end) = (self.eval(samples[0]).point, self.eval(samples[last]).point);
        let scale = first.coords.norm().max(end.coords.norm());
        let closed = is_negligible((first - end).norm(), scale);
        // The brackets to search: the neighbours' union, each half, and
        // across the seam where there is one.
        let mut brackets: Vec<(f64, f64)> = Vec::with_capacity(5);
        match self.period {
            Some(period) => {
                let prev = if idx == 0 {
                    samples[last - 1] - period
                } else {
                    samples[idx - 1]
                };
                let next = if idx >= last - 1 {
                    samples[0] + period
                } else {
                    samples[idx + 1]
                };
                brackets.extend([(prev, next), (prev, best.0), (best.0, next)]);
            }
            None => {
                if idx > 0 && idx < last {
                    brackets.push((samples[idx - 1], samples[idx + 1]));
                }
                if idx > 0 {
                    brackets.push((samples[idx - 1], samples[idx]));
                }
                if idx < last {
                    brackets.push((samples[idx], samples[idx + 1]));
                }
                if closed && idx == 0 {
                    brackets.push((samples[last - 1], samples[last]));
                }
                if closed && idx == last {
                    brackets.push((samples[0], samples[1]));
                }
            }
        }
        // Newton runs on one polynomial piece: where the curve is only C⁰
        // at a knot, the derivative of the distance jumps there, and a
        // bracket ending on the knot read on the next piece can lose its
        // sign change. So a bracket across a knot is left to its halves,
        // and each is read on the piece its middle is on — wrapped, with
        // the shift that wrapping it took, for a bracket past a seam.
        let (k_lo, k_hi) = (self.knots[p], self.knots[n]);
        let knot_scale = k_lo.abs().max(k_hi.abs());
        let crosses_knot = |a: f64, b: f64| {
            self.knots[p..=n].iter().any(|&k| {
                k > a
                    && k < b
                    && !is_negligible(k - a, knot_scale)
                    && !is_negligible(b - k, knot_scale)
            })
        };
        let mut candidate = best;
        for (lo, hi) in brackets {
            let Ok(interval) = Interval::new(lo, hi) else {
                continue;
            };
            let mid = interval.midpoint();
            let shift = self.wrap(mid) - mid;
            if crosses_knot(lo + shift, hi + shift) {
                continue;
            }
            let span = basis::span(p, &self.knots, n, mid + shift);
            let on_piece = |t: f64| {
                let (a, w) = self.homogeneous_on(span, t + shift);
                Self::quotient(a, w)
            };
            let g = |t: f64| {
                let e = on_piece(t);
                (e.point - q).dot(&e.d1)
            };
            let dg = |t: f64| {
                let e = on_piece(t);
                e.d1.norm_squared() + (e.point - q).dot(&e.d2)
            };
            if let Ok(t) = newton_in_interval(g, dg, interval, 0.0) {
                let d = dist2(t);
                if d < candidate.1 {
                    candidate = (t, d);
                }
            }
        }
        self.wrap(candidate.0)
    }
}

/// How many parameters projection samples on each span before Newton: the
/// squared distance on a span of degree `p` is a rational function whose
/// numerator has degree `2p`, so `2p + 2` samples see every one of its
/// extrema apart by more than a sample. A sampling density, not a
/// tolerance.
fn samples_per_span(degree: usize) -> usize {
    2 * degree + 2
}

/// `Some(T)` when the knots and control points wrap: with `m = n − p`
/// spans in the domain `[k_p, k_n]` and `T` its length, every knot repeats
/// `m` places on shifted by `T` and the last `p` control points repeat
/// the first `p`, all to rounding. A clamped knot vector never wraps.
pub(crate) fn detect_period(
    degree: usize,
    knots: &[f64],
    n: usize,
    same_point: impl Fn(usize, usize) -> bool,
) -> Option<f64> {
    let m = n - degree;
    let period = knots[n] - knots[degree];
    let scale = knots.iter().fold(0.0f64, |acc, k| acc.max(k.abs()));
    let knots_wrap =
        (0..knots.len() - m).all(|i| is_negligible(knots[i + m] - knots[i] - period, scale));
    if !knots_wrap {
        return None;
    }
    (0..degree).all(|i| same_point(i, i + m)).then_some(period)
}
