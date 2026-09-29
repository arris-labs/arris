//! Least-squares approximation of a curve by a B-spline at the caller's
//! own parametrisation (*The NURBS Book* §9.4.1, with the knot vector
//! refined where the caller's deviation says the fit is not good
//! enough): in the (u, v) plane for the pcurves that have no exact
//! `Curve2` variant, in 3D for the section curves that have no exact
//! `Curve` variant, and in 3D over a periodic knot vector for the closed
//! ones.

use core::fmt;

use arris_math::nalgebra::{Const, OPoint, SVector};
use arris_math::{Interrupted, Interval, Meter, Point2, Point3};

use super::basis::{self, MAX_DEGREE};
use super::spline::Spline;
use crate::{NurbsCurve, NurbsCurve2};

/// The most spans a fit may refine to before it gives up with
/// [`FitError::Diverged`]: a structural bound that makes every fit
/// terminate, not a tolerance. A cubic on a thousand spans resolves a
/// sinusoid of amplitude a thousand to `1e-7`; a caller that needs more
/// asks for a higher degree. Four thousand because a torus section is
/// as long as the two tori are big: the loop two metre-scale tori of
/// nearly equal radii share runs 100 to 170 units, and a quintic holds
/// it within `SECTION_FIT_FRACTION` of the default tolerance on 1000 to
/// 1300 spans — the cap measured on the cylinder pairs (1024) cut those
/// off with the fit a factor of two short (ADR-0019). The work per
/// refinement is linear in the spans (a banded solve), so the bound
/// costs nothing where it is not reached.
pub const MAX_FIT_SPANS: usize = 4096;

/// Spans the first fit tries; refinement splits from here.
const INITIAL_SPANS: usize = 4;

/// The fit is accepted where the caller's deviation is at or below this
/// fraction of the tolerance at every check parameter. The residual of a
/// least-squares B-spline fit oscillates about `p + 1` times per span and
/// the check samples it `4p + 4` times per span, so the largest checked
/// value is within `cos(π/8)` of the true maximum; half the tolerance is
/// margin for what the samples miss, so the result meets the tolerance
/// between them too. A ratio between sampling density and the residual's
/// shape, not a tolerance.
const FIT_MARGIN: f64 = 0.5;

/// Why a fit did not produce a curve.
#[derive(Debug, Clone, PartialEq)]
pub enum FitError {
    /// The request cannot be fitted: a degree outside `1..=MAX_DEGREE`,
    /// an unbounded or empty range.
    Degenerate(String),
    /// The tolerance is not finite and positive.
    InvalidTolerance(f64),
    /// The curve or the deviation returned a non-finite value at `t`.
    NonFinite {
        /// Where.
        t: f64,
    },
    /// The deviation still exceeded the tolerance with [`MAX_FIT_SPANS`]
    /// spans: the curve is not one this degree can approximate at the
    /// caller's parametrisation to that tolerance — or the deviation is
    /// noise the fit cannot get under.
    Diverged {
        /// How many spans the last fit had.
        spans: usize,
        /// The worst deviation it left.
        deviation: f64,
    },
    /// The caller stopped the fit ([`arris_math::Control`]).
    Interrupted(Interrupted),
}

impl From<Interrupted> for FitError {
    fn from(stop: Interrupted) -> Self {
        FitError::Interrupted(stop)
    }
}

impl fmt::Display for FitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FitError::Degenerate(reason) => write!(f, "cannot fit: {reason}"),
            FitError::InvalidTolerance(tol) => write!(f, "fit tolerance {tol} is not positive"),
            FitError::NonFinite { t } => {
                write!(f, "the curve or its deviation is not finite at t = {t}")
            }
            FitError::Diverged { spans, deviation } => write!(
                f,
                "the fit still deviates by {deviation} with {spans} spans, the most it may use"
            ),
            FitError::Interrupted(stop) => stop.fmt(f),
        }
    }
}

impl std::error::Error for FitError {}

/// A non-rational B-spline of `degree` over `range` approximating `f` at
/// its own parameter — `fit(t) ≈ f(t)`, same `t`, so the result is
/// same-parameter by construction and interpolates `f` at both ends —
/// with the knots refined until `deviation(t, fit(t))` is within `tol`
/// at every check parameter (the fit's samples and their midpoints, four
/// times `p + 1` per span, both ends included). `deviation` is the
/// caller's measure in the caller's units: the distance in (u, v) for a
/// plain fit, the 3D distance between the surface at the fitted point
/// and the true curve for a pcurve.
///
/// Guarantees: deterministic; terminates, with [`FitError::Diverged`]
/// once [`MAX_FIT_SPANS`] would be exceeded — never a loop; the result's
/// domain is `range` exactly and its control points are finite. `meter`
/// is ticked once per sample of `f` and once per check of `deviation`, so
/// a stop ([`FitError::Interrupted`]) lands within one call of either of
/// the caller's closures.
///
/// ```
/// use arris_geom::fit_curve2;
/// use arris_math::{Interval, Meter, Point2};
///
/// let f = |t: f64| Point2::new(t, t.cos());
/// let range = Interval::new(0.0, 3.0).unwrap();
/// let fit = fit_curve2(f, range, 3, |t, q| (q - f(t)).norm(), 1e-9, &mut Meter::default()).unwrap();
/// assert_eq!(fit.domain(), range);
/// assert!((fit.eval(1.7).point - f(1.7)).norm() <= 1e-9);
/// ```
pub fn fit_curve2(
    f: impl Fn(f64) -> Point2,
    range: Interval,
    degree: usize,
    deviation: impl Fn(f64, Point2) -> f64,
    tol: f64,
    meter: &mut Meter,
) -> Result<NurbsCurve2, FitError> {
    fit(&f, range, degree, &deviation, tol, Ends::Clamped, meter).map(NurbsCurve2::from_spline)
}

/// A non-rational B-spline of `degree` over `range` approximating the 3D
/// curve `f` at its own parameter, with the contract of [`fit_curve2`]
/// in three dimensions: same parameter, both ends interpolated, the
/// caller's `deviation` within `tol` at every check parameter, the
/// domain `range` exactly, deterministic, [`FitError::Diverged`] past
/// [`MAX_FIT_SPANS`]. For an open section curve, whose deviation is the
/// farther of its two surfaces.
///
/// ```
/// use arris_geom::fit_curve;
/// use arris_math::{Interval, Meter, Point3};
///
/// // A helix.
/// let f = |t: f64| Point3::new(t.cos(), t.sin(), 0.2 * t);
/// let range = Interval::new(0.0, 4.0).unwrap();
/// let fit = fit_curve(f, range, 3, |t, q| (q - f(t)).norm(), 1e-9, &mut Meter::default()).unwrap();
/// assert_eq!(fit.domain(), range);
/// assert_eq!(fit.eval(4.0).point, f(4.0));
/// assert!((fit.eval(2.3).point - f(2.3)).norm() <= 1e-9);
/// ```
pub fn fit_curve(
    f: impl Fn(f64) -> Point3,
    range: Interval,
    degree: usize,
    deviation: impl Fn(f64, Point3) -> f64,
    tol: f64,
    meter: &mut Meter,
) -> Result<NurbsCurve, FitError> {
    fit(&f, range, degree, &deviation, tol, Ends::Clamped, meter).map(NurbsCurve::from_spline)
}

/// A periodic non-rational B-spline of `degree` approximating the closed
/// 3D curve `f` at its own parameter over one period `range`: the
/// contract of [`fit_curve`], except that nothing is interpolated — the
/// knots wrap with period `range.length()`, so the result's `period()`
/// is `Some` of it, its domain is `range` exactly, and its image is
/// closed with a continuous derivative of every order below `degree` at
/// the seam, by construction rather than to a tolerance. For a closed
/// section loop, which the pave model cuts at its paves only.
///
/// `f` has to close: [`FitError::Degenerate`] when `deviation` at
/// `range.hi()` of the point `f(range.lo())` is above the fraction of
/// `tol` the fit is held to, since no periodic curve could then meet the
/// tolerance at both ends.
///
/// ```
/// use arris_geom::fit_curve_periodic;
/// use arris_math::{Interval, Meter, Point3};
/// use core::f64::consts::TAU;
///
/// // An ellipse, as a closed loop.
/// let f = |t: f64| Point3::new(2.0 * t.cos(), t.sin(), 0.5 * t.sin());
/// let range = Interval::new(0.0, TAU).unwrap();
/// let fit = fit_curve_periodic(f, range, 3, |t, q| (q - f(t)).norm(), 1e-9, &mut Meter::default()).unwrap();
/// assert_eq!(fit.period(), Some(TAU));
/// assert!((fit.eval(0.0).point - fit.eval(TAU).point).norm() <= 1e-15);
/// assert!((fit.eval(1.1).point - f(1.1)).norm() <= 1e-9);
/// ```
pub fn fit_curve_periodic(
    f: impl Fn(f64) -> Point3,
    range: Interval,
    degree: usize,
    deviation: impl Fn(f64, Point3) -> f64,
    tol: f64,
    meter: &mut Meter,
) -> Result<NurbsCurve, FitError> {
    fit(&f, range, degree, &deviation, tol, Ends::Periodic, meter).map(NurbsCurve::from_spline)
}

/// How a fit's ends are held.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ends {
    /// Clamped knots, the curve interpolating both ends.
    Clamped,
    /// Knots and control points wrapping with the range's length.
    Periodic,
}

/// The fit in `D` dimensions: validation, then least squares and
/// refinement until every check parameter is within the margin.
fn fit<const D: usize>(
    f: &dyn Fn(f64) -> OPoint<f64, Const<D>>,
    range: Interval,
    degree: usize,
    deviation: &dyn Fn(f64, OPoint<f64, Const<D>>) -> f64,
    tol: f64,
    ends: Ends,
    meter: &mut Meter,
) -> Result<Spline<D>, FitError> {
    if degree == 0 || degree > MAX_DEGREE {
        return Err(FitError::Degenerate(format!(
            "degree {degree} is not in 1..={MAX_DEGREE}"
        )));
    }
    if !range.is_bounded() || range.length() <= 0.0 {
        return Err(FitError::Degenerate(format!(
            "range [{}, {}] is not a bounded interval of positive length",
            range.lo(),
            range.hi()
        )));
    }
    if !(tol.is_finite() && tol > 0.0) {
        return Err(FitError::InvalidTolerance(tol));
    }
    if ends == Ends::Periodic {
        let (lo, hi) = (range.lo(), range.hi());
        let start = f(lo);
        if !start.coords.iter().all(|c| c.is_finite()) {
            return Err(FitError::NonFinite { t: lo });
        }
        let gap = deviation(hi, start);
        if !gap.is_finite() {
            return Err(FitError::NonFinite { t: hi });
        }
        if gap > FIT_MARGIN * tol {
            return Err(FitError::Degenerate(format!(
                "the curve does not close: its start deviates by {gap} from its end, \
                 above the fit's {} of the tolerance {tol}",
                FIT_MARGIN
            )));
        }
    }
    let mut breaks: Vec<f64> = (0..=INITIAL_SPANS)
        .map(|i| range.lerp(i as f64 / INITIAL_SPANS as f64))
        .collect();
    loop {
        let samples = sample_parameters(&breaks, degree);
        let curve = match ends {
            Ends::Clamped => least_squares(f, degree, &breaks, &samples, meter)?,
            // The last sample is the first one a period on.
            Ends::Periodic => {
                least_squares_periodic(f, degree, &breaks, &samples[..samples.len() - 1], meter)?
            }
        };
        // Check at the samples and between them; split every span whose
        // worst check exceeds the margin, both spans at a break. A miss on
        // a break charged to the span after it alone, where the one
        // before is what bends the fit there, halves the span after until
        // its knots coincide and the normal equations are singular: a
        // torus sliced a hair off a meridian plane did, at a turning
        // point.
        let mut worst = 0.0f64;
        let mut split: Vec<bool> = vec![false; breaks.len() - 1];
        let mut span = 0;
        for pair in samples.windows(2) {
            for t in [pair[0], 0.5 * (pair[0] + pair[1])] {
                meter.tick()?;
                let d = deviation(t, curve.eval(t).point);
                if !d.is_finite() {
                    return Err(FitError::NonFinite { t });
                }
                worst = worst.max(d);
                while span + 1 < breaks.len() - 1 && t >= breaks[span + 1] {
                    span += 1;
                }
                if d > FIT_MARGIN * tol {
                    split[span] = true;
                    // A check on a break is as much the span before's —
                    // on a loop, the first break is the last span's end.
                    if t == breaks[span] {
                        match (span, ends) {
                            (0, Ends::Periodic) => split[breaks.len() - 2] = true,
                            (0, Ends::Clamped) => {}
                            _ => split[span - 1] = true,
                        }
                    }
                }
            }
        }
        let t_end = range.hi();
        meter.tick()?;
        let d_end = deviation(t_end, curve.eval(t_end).point);
        if !d_end.is_finite() {
            return Err(FitError::NonFinite { t: t_end });
        }
        worst = worst.max(d_end);
        if d_end > FIT_MARGIN * tol {
            split[breaks.len() - 2] = true;
            if matches!(ends, Ends::Periodic) {
                split[0] = true;
            }
        }
        if !split.iter().any(|&s| s) {
            return Ok(curve);
        }
        let spans = breaks.len() - 1;
        let new_spans = spans + split.iter().filter(|&&s| s).count();
        if new_spans > MAX_FIT_SPANS {
            return Err(FitError::Diverged {
                spans,
                deviation: worst,
            });
        }
        let mut refined = Vec::with_capacity(new_spans + 1);
        for (i, w) in breaks.windows(2).enumerate() {
            refined.push(w[0]);
            if split[i] {
                refined.push(0.5 * (w[0] + w[1]));
            }
        }
        refined.push(range.hi());
        breaks = refined;
    }
}

/// The fit's sample parameters: every break, and `2p + 1` parameters
/// evenly inside each span, ascending.
fn sample_parameters(breaks: &[f64], degree: usize) -> Vec<f64> {
    let inside = 2 * degree + 1;
    let mut out = Vec::with_capacity(breaks.len() + (breaks.len() - 1) * inside);
    for w in breaks.windows(2) {
        out.push(w[0]);
        for j in 1..=inside {
            out.push(w[0] + (w[1] - w[0]) * j as f64 / (inside + 1) as f64);
        }
    }
    out.push(breaks[breaks.len() - 1]);
    out
}

/// The clamped knot vector over `breaks`.
fn clamped_knots(breaks: &[f64], degree: usize) -> Vec<f64> {
    let (lo, hi) = (breaks[0], breaks[breaks.len() - 1]);
    let mut knots = vec![lo; degree + 1];
    knots.extend_from_slice(&breaks[1..breaks.len() - 1]);
    knots.extend(core::iter::repeat_n(hi, degree + 1));
    knots
}

/// The periodic knot vector over `breaks`: the `m` spans between them,
/// continued `p` knots on either side by the period, so the domain
/// `[knots[p], knots[m + p]]` is `[breaks[0], breaks[m]]` exactly.
fn periodic_knots(breaks: &[f64], degree: usize) -> Vec<f64> {
    let m = breaks.len() - 1;
    let period = breaks[m] - breaks[0];
    (0..m + 2 * degree + 1)
        .map(|idx| {
            let j = idx as isize - degree as isize;
            if (0..=m as isize).contains(&j) {
                breaks[j as usize]
            } else {
                let turns = j.div_euclid(m as isize);
                breaks[j.rem_euclid(m as isize) as usize] + turns as f64 * period
            }
        })
        .collect()
}

/// `f` at every sample, or the first sample where it is not finite.
fn sample_data<const D: usize>(
    f: &dyn Fn(f64) -> OPoint<f64, Const<D>>,
    samples: &[f64],
    meter: &mut Meter,
) -> Result<Vec<OPoint<f64, Const<D>>>, FitError> {
    let data = samples
        .iter()
        .map(|&t| {
            meter.tick()?;
            Ok(f(t))
        })
        .collect::<Result<Vec<OPoint<f64, Const<D>>>, Interrupted>>()?;
    match data
        .iter()
        .position(|q| !q.coords.iter().all(|c| c.is_finite()))
    {
        Some(i) => Err(FitError::NonFinite { t: samples[i] }),
        None => Ok(data),
    }
}

/// The least-squares B-spline through the end samples and nearest the
/// rest (A9.7's normal equations, solved by a banded Cholesky
/// factorisation since `NᵀN` has bandwidth `p`).
#[allow(clippy::needless_range_loop)] // `j`, `l` index the basis row and the net together
fn least_squares<const D: usize>(
    f: &dyn Fn(f64) -> OPoint<f64, Const<D>>,
    degree: usize,
    breaks: &[f64],
    samples: &[f64],
    meter: &mut Meter,
) -> Result<Spline<D>, FitError> {
    let p = degree;
    let knots = clamped_knots(breaks, p);
    let n = knots.len() - p - 1;
    let m = samples.len() - 1;
    let data = sample_data(f, samples, meter)?;
    let (q0, qm) = (data[0], data[m]);
    // Interior unknowns P_1 .. P_{n-2}.
    let unknowns = n - 2;
    let mut points = vec![OPoint::<f64, Const<D>>::origin(); n];
    points[0] = q0;
    points[n - 1] = qm;
    if unknowns > 0 {
        let mut normal = Envelope::banded(unknowns, p);
        let mut rhs = vec![SVector::<f64, D>::zeros(); unknowns];
        for k in 1..m {
            let t = samples[k];
            let s = basis::span(p, &knots, n, t);
            let row = basis::derivatives(p, &knots, s, t)[0];
            let first = s - p;
            // `row[0]` only weighs `Q_0` when `P_0` is active, and the last
            // active function is `P_{n-1}` only on the last span.
            let r = {
                let mut r = data[k].coords;
                for j in 0..=p {
                    let idx = first + j;
                    if idx == 0 {
                        r -= row[j] * q0.coords;
                    } else if idx == n - 1 {
                        r -= row[j] * qm.coords;
                    }
                }
                r
            };
            for j in 0..=p {
                let a = first + j;
                if a == 0 || a == n - 1 {
                    continue;
                }
                rhs[a - 1] += row[j] * r;
                for l in j..=p {
                    let b = first + l;
                    if b == 0 || b == n - 1 {
                        continue;
                    }
                    normal.add(a - 1, b - 1, row[j] * row[l]);
                }
            }
        }
        let solved = normal
            .solve(&rhs)
            .ok_or_else(|| FitError::Degenerate("the normal equations are singular".to_owned()))?;
        for (i, v) in solved.into_iter().enumerate() {
            points[i + 1] = OPoint::from(v);
        }
    }
    Spline::new(p, knots, points, vec![1.0; n]).map_err(FitError::Degenerate)
}

/// The least-squares periodic B-spline nearest every sample, `samples`
/// covering one period with its end left out: `m` free control points
/// for `m` spans, control point `i` being free point `i mod m`. The
/// normal equations couple each free point to its `p` neighbours on
/// either side cyclically, so the matrix is a band with corners, which
/// the envelope factorisation keeps exact.
fn least_squares_periodic<const D: usize>(
    f: &dyn Fn(f64) -> OPoint<f64, Const<D>>,
    degree: usize,
    breaks: &[f64],
    samples: &[f64],
    meter: &mut Meter,
) -> Result<Spline<D>, FitError> {
    let p = degree;
    let m = breaks.len() - 1;
    let knots = periodic_knots(breaks, p);
    let n = m + p;
    let data = sample_data(f, samples, meter)?;
    // The free points active together on span `s` are `s..=s + p` mod `m`;
    // a row's envelope starts at the lowest of its partners.
    let mut first: Vec<usize> = (0..m).collect();
    for s in 0..m {
        for a in (s..=s + p).map(|c| c % m) {
            for b in (s..=s + p).map(|c| c % m) {
                if b < first[a] {
                    first[a] = b;
                }
            }
        }
    }
    let mut normal = Envelope::new(first);
    let mut rhs = vec![SVector::<f64, D>::zeros(); m];
    // A span's `p + 1` functions land on at most `p + 1` free points, and
    // fewer when `m ≤ p` wraps two of them onto one.
    let mut weights: Vec<(usize, f64)> = Vec::with_capacity(p + 1);
    for (&t, q) in samples.iter().zip(&data) {
        let s = basis::span(p, &knots, n, t);
        let row = basis::derivatives(p, &knots, s, t)[0];
        weights.clear();
        for (j, &value) in row.iter().enumerate().take(p + 1) {
            let free = (s - p + j) % m;
            match weights.iter_mut().find(|(i, _)| *i == free) {
                Some((_, w)) => *w += value,
                None => weights.push((free, value)),
            }
        }
        for &(a, wa) in &weights {
            rhs[a] += wa * q.coords;
            for &(b, wb) in &weights {
                if b <= a {
                    normal.add(a, b, wa * wb);
                }
            }
        }
    }
    let free = normal.solve(&rhs).ok_or_else(|| {
        FitError::Degenerate("the periodic normal equations are singular".to_owned())
    })?;
    let points = (0..n).map(|i| OPoint::from(free[i % m])).collect();
    let spline = Spline::new(p, knots, points, vec![1.0; n]).map_err(FitError::Degenerate)?;
    // The knots and points wrap by construction; a spline that does not
    // see it would evaluate unwrapped, so it is refused rather than kept.
    match spline.period() {
        Some(_) => Ok(spline),
        None => Err(FitError::Degenerate(
            "the periodic knots do not wrap to rounding".to_owned(),
        )),
    }
}

/// A symmetric matrix stored by its envelope: row `i` holds the entries
/// `(i, j)` for `first[i] ≤ j ≤ i`, and every entry left of `first[i]` is
/// zero. A Cholesky factor has the same envelope, so a band (`first[i] =
/// i − w`) factors as a band, and a cyclic band's corner rows only fill
/// in the rows that already reach column zero.
struct Envelope {
    first: Vec<usize>,
    rows: Vec<Vec<f64>>,
}

impl Envelope {
    /// Zero, over the envelope `first` (`first[i] ≤ i`).
    fn new(first: Vec<usize>) -> Self {
        let rows = first
            .iter()
            .enumerate()
            .map(|(i, &f)| vec![0.0; i - f + 1])
            .collect();
        Envelope { first, rows }
    }

    /// Zero, over the band of half-width `w`.
    fn banded(size: usize, w: usize) -> Self {
        Envelope::new((0..size).map(|i| i.saturating_sub(w)).collect())
    }

    /// Adds `v` to `(i, j)` and `(j, i)`, inside the envelope.
    fn add(&mut self, i: usize, j: usize, v: f64) {
        let (lo, hi) = if i <= j { (i, j) } else { (j, i) };
        self.rows[hi][lo - self.first[hi]] += v;
    }

    /// `x` with `A x = rhs` for a positive definite `A`, by a Cholesky
    /// factorisation within the envelope; `None` when a pivot is not
    /// positive.
    fn solve<const D: usize>(&self, rhs: &[SVector<f64, D>]) -> Option<Vec<SVector<f64, D>>> {
        let n = rhs.len();
        let first = &self.first;
        // `l[i][j − first[i]]` holds `L(i, j)`.
        let mut l: Vec<Vec<f64>> = self.rows.iter().map(|r| vec![0.0; r.len()]).collect();
        for i in 0..n {
            for j in first[i]..=i {
                let mut sum = self.rows[i][j - first[i]];
                for k in first[i].max(first[j])..j {
                    sum -= l[i][k - first[i]] * l[j][k - first[j]];
                }
                if i == j {
                    if sum.is_nan() || sum <= 0.0 {
                        return None;
                    }
                    l[i][i - first[i]] = sum.sqrt();
                } else {
                    l[i][j - first[i]] = sum / l[j][j - first[j]];
                }
            }
        }
        let diag = |i: usize| l[i][i - first[i]];
        // Forward: L y = rhs; back: Lᵀ x = y, row by row so each sum runs
        // in ascending order.
        let mut y = vec![SVector::<f64, D>::zeros(); n];
        for i in 0..n {
            let mut v = rhs[i];
            for k in first[i]..i {
                v -= l[i][k - first[i]] * y[k];
            }
            y[i] = v / diag(i);
        }
        let mut x = vec![SVector::<f64, D>::zeros(); n];
        for i in (0..n).rev() {
            let mut v = y[i];
            for k in i + 1..n {
                if first[k] <= i {
                    v -= l[k][i - first[k]] * x[k];
                }
            }
            x[i] = v / diag(i);
        }
        Some(x)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::Vec2;

    #[test]
    fn the_band_solver_agrees_with_a_dense_solution() {
        // A tridiagonal SPD system with a known answer.
        let mut b = Envelope::banded(4, 1);
        for i in 0..4 {
            b.add(i, i, 4.0);
        }
        for i in 0..3 {
            b.add(i, i + 1, 1.0);
        }
        let x_true: Vec<Vec2> = (0..4)
            .map(|i| Vec2::new(i as f64, 1.0 - i as f64))
            .collect();
        let rhs: Vec<Vec2> = (0..4)
            .map(|i| {
                let mut v = 4.0 * x_true[i];
                if i > 0 {
                    v += x_true[i - 1];
                }
                if i < 3 {
                    v += x_true[i + 1];
                }
                v
            })
            .collect();
        let x = b.solve(&rhs).unwrap();
        for (a, e) in x.iter().zip(&x_true) {
            assert!((a - e).norm() < 1e-14);
        }
        let mut bad = Envelope::banded(2, 1);
        bad.add(0, 0, 1.0);
        bad.add(0, 1, 2.0);
        bad.add(1, 1, 1.0);
        assert!(bad.solve(&[Vec2::zeros(); 2]).is_none());
    }

    #[test]
    fn a_line_is_fitted_exactly_by_a_linear_spline() {
        let f = |t: f64| Point2::new(2.0 * t + 1.0, -t);
        let fit = fit_curve2(
            f,
            Interval::new(-1.0, 3.0).unwrap(),
            1,
            |t, q| (q - f(t)).norm(),
            1e-12,
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert_eq!(fit.degree(), 1);
        assert!((fit.eval(0.37).point - f(0.37)).norm() < 1e-13);
        assert_eq!(fit.eval(3.0).point, f(3.0));
    }

    #[test]
    fn bad_requests_are_named() {
        let f = |t: f64| Point2::new(t, 0.0);
        let range = Interval::new(0.0, 1.0).unwrap();
        assert!(matches!(
            fit_curve2(
                f,
                range,
                0,
                |_, _| 0.0,
                1e-6,
                &mut arris_math::Meter::default()
            ),
            Err(FitError::Degenerate(_))
        ));
        assert!(matches!(
            fit_curve2(
                f,
                Interval::REAL,
                2,
                |_, _| 0.0,
                1e-6,
                &mut arris_math::Meter::default()
            ),
            Err(FitError::Degenerate(_))
        ));
        assert_eq!(
            fit_curve2(
                f,
                range,
                2,
                |_, _| 0.0,
                0.0,
                &mut arris_math::Meter::default()
            ),
            Err(FitError::InvalidTolerance(0.0))
        );
        assert!(matches!(
            fit_curve2(
                f,
                range,
                2,
                |_, _| f64::NAN,
                1e-6,
                &mut arris_math::Meter::default()
            ),
            Err(FitError::NonFinite { .. })
        ));
        let e = fit_curve2(
            f,
            range,
            2,
            |_, _| 1.0,
            1e-6,
            &mut arris_math::Meter::default(),
        )
        .unwrap_err();
        assert!(
            matches!(e, FitError::Diverged { spans, .. } if spans <= MAX_FIT_SPANS),
            "{e}"
        );
    }
}
