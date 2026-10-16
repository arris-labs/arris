//! A surface skinned through a family of compatible sections (ADR-0062
//! §1): each section's homogeneous control net fitted along the family's
//! parameter by the curve fit's least squares, the knots along it refined
//! until the surface is within the caller's tolerance of every section it
//! is checked against. The rolling-ball blend skins the ball's exact arcs
//! along its spine. A smooth loft has a few sections, not a family, and
//! interpolates them instead (ADR-0065 §7): [`interpolate_skin`].

use arris_math::nalgebra::{Const, OPoint, Point4};
use arris_math::{Interval, Meter};

use super::basis::{self, MAX_DEGREE};
use super::fit::{
    FIT_MARGIN, FitError, INITIAL_SPANS, MAX_FIT_SPANS, clamped_knots, least_squares_samples,
    sample_parameters,
};
use super::spline::BezierSpan;
use super::{NurbsCurve, NurbsSurface};
use crate::bernstein::{Binomials, mul};
use crate::{CurveKind, GeomError, GeomKind, SurfaceKind};

/// Two weight functions whose weights, each divided by its first, agree
/// this closely relative to their size are one: the spread of the same
/// arc built twice. A structural constant of the matching, not a model
/// tolerance.
const WEIGHT_COINCIDENCE: f64 = 1e-12;

/// A rational B-spline surface through the sections `section(v)` for `v`
/// over `range`: `u` is the sections' own parameter, `v` the family's,
/// and `result.eval(u, v)` is within `tol` of `section(v).eval(u)` at
/// every check — the fit's samples along `v` and the midpoints between
/// them, crossed with `2(p + 1)` parameters inside each of the sections'
/// knot spans and their ends. The sections are *compatible*: one degree
/// `p`, one knot vector and one count of control points, which become the
/// surface's along `u`; their weights may vary with `v`. Each control
/// point's homogeneous coordinates `(w·P, w)` are fitted along `v` by
/// least squares, at `degree`, both ends interpolated, so the surface's
/// isocurves `v = range.lo()` and `v = range.hi()` are the end sections
/// to rounding — a section collapsed to a point gives a collapsed row.
/// The knots along `v` are refined, a span at a time, where a check
/// misses.
///
/// Guarantees: deterministic; terminates, with [`FitError::Diverged`]
/// once [`crate::MAX_FIT_SPANS`] spans along `v` would be exceeded; the
/// result's domain is the sections' domain by `range` exactly. `meter` is
/// ticked once per section and once per check.
///
/// Errors: what `section` returns, as is; and as `E` from
/// [`GeomError`]:
/// - [`GeomError::Fit`]: a degree outside `1..=MAX_DEGREE`, an unbounded
///   or empty `range`, a tolerance that is not finite and positive, or
///   the fit diverging;
/// - [`GeomError::Degenerate`]: a section not compatible with the first,
///   or a fitted weight that is not positive;
/// - [`GeomError::Interrupted`]: the caller stopped it.
///
/// ```
/// use arris_geom::{GeomError, NurbsCurve, fit_skin};
/// use arris_math::{Frame, Interval, Meter, Point3, Vec3};
///
/// // Circles of radius 1 + v/2 at height v: a cone, skinned.
/// let ring = |v: f64| -> Result<NurbsCurve, GeomError> {
///     let frame = Frame::new(Point3::new(0.0, 0.0, v), Vec3::z(), Vec3::x()).unwrap();
///     NurbsCurve::circle(&frame, 1.0 + 0.5 * v, Interval::TURN)
/// };
/// let range = Interval::new(0.0, 2.0).unwrap();
/// let cone = fit_skin(ring, range, 3, 1e-9, &mut Meter::default()).unwrap();
/// let p = cone.eval(1.3, 0.7).point;
/// assert!((p.coords.xy().norm() - 1.35).abs() <= 1e-9);
/// assert!((p.z - 0.7).abs() <= 1e-9);
/// ```
pub fn fit_skin<E: From<GeomError>>(
    section: impl Fn(f64) -> Result<NurbsCurve, E>,
    range: Interval,
    degree: usize,
    tol: f64,
    meter: &mut Meter,
) -> Result<NurbsSurface, E> {
    let fit = |e: FitError| E::from(GeomError::from(e));
    if degree == 0 || degree > MAX_DEGREE {
        return Err(fit(FitError::Degenerate(format!(
            "degree {degree} is not in 1..={MAX_DEGREE}"
        ))));
    }
    if !range.is_bounded() || range.length() <= 0.0 {
        return Err(fit(FitError::Degenerate(format!(
            "range [{}, {}] is not a bounded interval of positive length",
            range.lo(),
            range.hi()
        ))));
    }
    if !(tol.is_finite() && tol > 0.0) {
        return Err(fit(FitError::InvalidTolerance(tol)));
    }
    let tick = |meter: &mut Meter| meter.tick().map_err(|stop| E::from(GeomError::from(stop)));
    tick(meter)?;
    let first = section(range.lo())?;
    let p = first.degree();
    let knots_u = first.knots().to_vec();
    let n = first.control_points().len();
    // The section at `v`, held to the first's degree, knots and count.
    let compatible = |v: f64, meter: &mut Meter| -> Result<NurbsCurve, E> {
        tick(meter)?;
        let c = section(v)?;
        if c.degree() != p || c.knots() != knots_u.as_slice() || c.control_points().len() != n {
            return Err(E::from(GeomError::Degenerate {
                kind: GeomKind::Curve(CurveKind::Nurbs),
                reason: format!(
                    "the section at v = {v} is not compatible with the first: degree {} \
                     and {} knots, where the first has degree {p} and {} knots",
                    c.degree(),
                    c.knots().len(),
                    knots_u.len()
                ),
            }));
        }
        Ok(c)
    };
    // The checks along `u`: `2(p + 1)` inside each knot span and its ends.
    let domain = first.domain();
    let mut checks_u = vec![domain.lo()];
    for w in knots_u.windows(2) {
        let (a, b) = (w[0].max(domain.lo()), w[1].min(domain.hi()));
        if b <= a {
            continue;
        }
        let inside = 2 * (p + 1);
        for j in 1..=inside {
            checks_u.push(a + (b - a) * j as f64 / (inside + 1) as f64);
        }
        checks_u.push(b);
    }
    let mut breaks: Vec<f64> = (0..=INITIAL_SPANS)
        .map(|i| range.lerp(i as f64 / INITIAL_SPANS as f64))
        .collect();
    loop {
        let samples = sample_parameters(&breaks, degree);
        let sections = samples
            .iter()
            .map(|&v| {
                if v == range.lo() {
                    Ok(first.clone())
                } else {
                    compatible(v, meter)
                }
            })
            .collect::<Result<Vec<_>, E>>()?;
        let knots_v = clamped_knots(&breaks, degree);
        // Each control point's homogeneous coordinates along `v`.
        let mut columns: Vec<Vec<OPoint<f64, Const<4>>>> = Vec::with_capacity(n);
        for i in 0..n {
            let data: Vec<OPoint<f64, Const<4>>> = sections
                .iter()
                .map(|c| {
                    let (q, w) = (c.control_points()[i], c.weights()[i]);
                    Point4::new(w * q.x, w * q.y, w * q.z, w)
                })
                .collect();
            let spline =
                least_squares_samples(degree, &knots_v, &samples, &data, meter).map_err(fit)?;
            columns.push(spline.points().to_vec());
        }
        let m = columns.first().map_or(0, Vec::len);
        let mut points = Vec::with_capacity(n * m);
        let mut weights = Vec::with_capacity(n * m);
        for column in &columns {
            for h in column {
                weights.push(h.w);
                points.push(arris_math::Point3::new(h.x / h.w, h.y / h.w, h.z / h.w));
            }
        }
        let surface = NurbsSurface::new([p, degree], [knots_u.clone(), knots_v], points, weights)
            .map_err(E::from)?;
        // Check at the samples and between them; split every span whose
        // worst check exceeds the margin, both spans at a break.
        let mut worst = 0.0f64;
        let mut split = vec![false; breaks.len() - 1];
        let mut span = 0;
        let last = samples.len() - 1;
        for (k, &v) in samples.iter().enumerate() {
            let mut at = vec![(v, None)];
            if k < last {
                at.push((0.5 * (v + samples[k + 1]), Some(())));
            }
            for (v, between) in at {
                let owned;
                let exact = match between {
                    None => &sections[k],
                    Some(()) => {
                        owned = compatible(v, meter)?;
                        &owned
                    }
                };
                let mut d = 0.0f64;
                for &u in &checks_u {
                    tick(meter)?;
                    let e = (surface.eval(u, v).point - exact.eval(u).point).norm();
                    if !e.is_finite() {
                        return Err(fit(FitError::NonFinite { t: v }));
                    }
                    d = d.max(e);
                }
                worst = worst.max(d);
                while span + 1 < breaks.len() - 1 && v >= breaks[span + 1] {
                    span += 1;
                }
                if d > FIT_MARGIN * tol {
                    split[span] = true;
                    if v == breaks[span] && span > 0 {
                        split[span - 1] = true;
                    }
                }
            }
        }
        if !split.iter().any(|&s| s) {
            return Ok(surface);
        }
        let spans = breaks.len() - 1;
        let new_spans = spans + split.iter().filter(|&&s| s).count();
        if new_spans > MAX_FIT_SPANS {
            return Err(fit(FitError::Diverged {
                spans,
                deviation: worst,
            }));
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

/// The surface through the compatible `sections` at `params` along `v`:
/// `u` the sections' own parameter, and the isoline `v = params[k]` the
/// section `k` exactly, up to rounding. The *points* of the sections are
/// interpolated along `v`, `S(u, v) = Σₖ Lₖ(v) Cₖ(u)`, by a B-spline of
/// `degree`, its interior knots the averages of `degree` consecutive
/// parameters (*The NURBS Book* eq. 9.8): at `degree = params.len() − 1`
/// one polynomial through all of them, at `1` the ruled surfaces between
/// consecutive sections. Sections whose weights differ — a line against
/// an arc, arcs of different angles — are first written over one weight
/// function, the product of their distinct ones, which raises the degree
/// along `u` by the sections' degree per distinct weight function and
/// leaves every section's point at every `u` where it was; the control
/// points' homogeneous coordinates are then interpolated, which on one
/// weight function is interpolating the points. Sections that share
/// their weights keep their degree and net. With `closed` the first section is taken again
/// after the last, at `params`' last value, so `params` has one entry
/// more than `sections` and the surface closes along `v`, not
/// periodically.
///
/// Guarantees: deterministic; the surface's `u` is the sections' degree,
/// knots and control-point count, its `v` domain `[params[0],
/// params[last]]`.
///
/// Errors: [`GeomError::Degenerate`] for fewer than two sections, a
/// section not compatible with the first, `params` not one per section
/// (one more when closed) or not strictly increasing, a degree outside
/// `1..params.len()`, an interpolation whose system is singular, or an
/// interpolated weight that is not positive.
///
/// ```
/// use arris_geom::{NurbsCurve, interpolate_skin};
/// use arris_math::{Frame, Interval, Point3, Vec3};
///
/// // Circles of radius 1, 2 and 1 at heights 0, 1 and 2: a quadratic
/// // barrel through all three.
/// let ring = |z: f64, r: f64| {
///     let frame = Frame::new(Point3::new(0.0, 0.0, z), Vec3::z(), Vec3::x()).unwrap();
///     NurbsCurve::circle(&frame, r, Interval::TURN).unwrap()
/// };
/// let sections = [ring(0.0, 1.0), ring(1.0, 2.0), ring(2.0, 1.0)];
/// let barrel = interpolate_skin(&sections, &[0.0, 0.5, 1.0], 2, false).unwrap();
/// let p = barrel.eval(0.3, 0.5).point;
/// assert!((p.coords.xy().norm() - 2.0).abs() <= 1e-12);
/// assert!((p.z - 1.0).abs() <= 1e-12);
/// ```
// The elimination reads the pivot row and the row it reduces by index at
// once, as it is written.
#[allow(clippy::needless_range_loop)]
pub fn interpolate_skin(
    sections: &[NurbsCurve],
    params: &[f64],
    degree: usize,
    closed: bool,
) -> Result<NurbsSurface, GeomError> {
    let degenerate = |reason: String| GeomError::Degenerate {
        kind: GeomKind::Surface(SurfaceKind::Nurbs),
        reason,
    };
    let rows: Vec<&NurbsCurve> = sections
        .iter()
        .chain(sections.first().filter(|_| closed))
        .collect();
    let count = rows.len();
    let Some(first) = rows.first() else {
        return Err(degenerate("a skin through no sections".to_string()));
    };
    if sections.len() < 2 {
        return Err(degenerate(format!(
            "a skin through {} section, at least two are needed",
            sections.len()
        )));
    }
    if params.len() != count {
        return Err(degenerate(format!(
            "{} parameters for {count} sections",
            params.len()
        )));
    }
    if let Some(k) = params.iter().position(|t| !t.is_finite()) {
        return Err(degenerate(format!("parameter {k} is not finite")));
    }
    if let Some(k) = params.windows(2).position(|w| w[0] >= w[1]) {
        return Err(degenerate(format!(
            "parameters {k} and {} do not increase: {} and {}",
            k + 1,
            params[k],
            params[k + 1]
        )));
    }
    if degree == 0 || degree >= count {
        return Err(degenerate(format!(
            "degree {degree} is not in 1..{count} for {count} sections"
        )));
    }
    let p = first.degree();
    let n = first.control_points().len();
    if let Some(k) = rows.iter().position(|c| {
        c.degree() != p || c.knots() != first.knots() || c.control_points().len() != n
    }) {
        return Err(degenerate(format!(
            "section {k} is not compatible with the first"
        )));
    }
    let unified = one_weight(&rows)?;
    let rows: Vec<&NurbsCurve> = unified.iter().collect();
    let first = rows[0];
    let n = first.control_points().len();
    // The knots along `v`: clamped at the ends, the interior by averaging.
    let (lo, hi) = (params[0], params[count - 1]);
    let mut knots_v = vec![lo; degree + 1];
    for j in 1..count - degree {
        knots_v.push(params[j..j + degree].iter().sum::<f64>() / degree as f64);
    }
    knots_v.extend(core::iter::repeat_n(hi, degree + 1));
    // The collocation matrix, `a[k][i]` the basis function `i` at `params[k]`.
    let mut a = vec![vec![0.0; count]; count];
    for (row, &t) in a.iter_mut().zip(params) {
        let s = basis::span(degree, &knots_v, count, t);
        let values = basis::derivatives(degree, &knots_v, s, t)[0];
        for (j, &value) in values.iter().enumerate().take(degree + 1) {
            row[s - degree + j] = value;
        }
    }
    // One right-hand side per control point of the sections: their
    // homogeneous coordinates.
    let mut b: Vec<Vec<Point4<f64>>> = rows
        .iter()
        .map(|c| {
            c.control_points()
                .iter()
                .zip(c.weights())
                .map(|(q, &w)| Point4::new(w * q.x, w * q.y, w * q.z, w))
                .collect()
        })
        .collect();
    // Gaussian elimination with partial pivoting, the pivot the first
    // row of largest magnitude so the choice is deterministic.
    for col in 0..count {
        let pivot = (col..count).fold(col, |best, r| {
            if a[r][col].abs() > a[best][col].abs() {
                r
            } else {
                best
            }
        });
        if a[pivot][col] == 0.0 {
            return Err(degenerate(format!(
                "the interpolation at parameter {} is singular",
                params[col]
            )));
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for r in col + 1..count {
            let f = a[r][col] / a[col][col];
            if f == 0.0 {
                continue;
            }
            for c in col..count {
                a[r][c] -= f * a[col][c];
            }
            for i in 0..n {
                let d = b[col][i].coords * f;
                b[r][i].coords -= d;
            }
        }
    }
    for col in (0..count).rev() {
        for r in col + 1..count {
            let f = a[col][r];
            for i in 0..n {
                let d = b[r][i].coords * f;
                b[col][i].coords -= d;
            }
        }
        let pivot = a[col][col];
        for h in &mut b[col] {
            h.coords /= pivot;
        }
    }
    // Row-major over `u`: control point `i` of the sections, then `v`.
    let mut points = Vec::with_capacity(n * count);
    let mut weights = Vec::with_capacity(n * count);
    for i in 0..n {
        for (k, row) in b.iter().enumerate() {
            let h = row[i];
            if h.w.is_nan() || h.w <= 0.0 {
                return Err(degenerate(format!(
                    "the interpolated weight of control point {i} at row {k} is {}",
                    h.w
                )));
            }
            weights.push(h.w);
            points.push(arris_math::Point3::new(h.x / h.w, h.y / h.w, h.z / h.w));
        }
    }
    NurbsSurface::new(
        [first.degree(), degree],
        [first.knots().to_vec(), knots_v],
        points,
        weights,
    )
}

/// `c` in Bernstein form raised to degree `to`, one degree at a time.
fn elevate(c: &[f64], to: usize) -> Vec<f64> {
    let mut c = c.to_vec();
    while c.len() < to + 1 {
        let n = c.len();
        let mut up = Vec::with_capacity(n + 1);
        up.push(c[0]);
        for k in 1..n {
            let a = k as f64 / n as f64;
            up.push(a * c[k - 1] + (1.0 - a) * c[k]);
        }
        up.push(c[n - 1]);
        c = up;
    }
    c
}

/// The compatible `rows` written over one weight function: each row's
/// weight function divided by its first weight is either constant or one
/// of a few distinct functions `Ŵ_c`; row `k` of class `c` becomes
/// `(ŵ P)ₖ · Π_{c' ≠ c} Ŵ_{c'}` over `D = Π Ŵ_c`, polynomial piece by
/// polynomial piece in Bernstein form, every piece raised to the highest
/// degree and the pieces joined by knots of that multiplicity. Rows that
/// already share one weight function, scale included, come back as they
/// are.
// A piece is read by its index from every row's spans and every class's.
#[allow(clippy::needless_range_loop)]
fn one_weight(rows: &[&NurbsCurve]) -> Result<Vec<NurbsCurve>, GeomError> {
    let spans: Vec<Vec<BezierSpan<3>>> = rows.iter().map(|c| c.bezier_spans()).collect();
    let scale = |k: usize| spans[k].first().map_or(1.0, |s| s.control[0].1);
    let profile = |k: usize| -> Vec<f64> {
        let w0 = scale(k);
        spans[k]
            .iter()
            .flat_map(|s| s.control.iter().map(move |c| c.1 / w0))
            .collect()
    };
    let profiles: Vec<Vec<f64>> = (0..rows.len()).map(profile).collect();
    let same = |a: &[f64], b: &[f64]| {
        a.iter()
            .zip(b)
            .all(|(x, y)| (x - y).abs() <= WEIGHT_COINCIDENCE * x.abs().max(y.abs()))
    };
    // The distinct weight functions, each by the first row that has it;
    // `None` for a constant one.
    let mut classes: Vec<usize> = Vec::new();
    let mut class_of: Vec<Option<usize>> = Vec::with_capacity(rows.len());
    for (k, w) in profiles.iter().enumerate() {
        if w.iter().all(|x| (x - 1.0).abs() <= WEIGHT_COINCIDENCE) {
            class_of.push(None);
        } else if let Some(c) = classes.iter().position(|&r| same(&profiles[r], w)) {
            class_of.push(Some(c));
        } else {
            class_of.push(Some(classes.len()));
            classes.push(k);
        }
    }
    let first_class = class_of[0];
    if class_of.iter().all(|&c| c == first_class)
        && rows.iter().all(|c| c.weights() == rows[0].weights())
    {
        return Ok(rows.iter().map(|&c| c.clone()).collect());
    }
    let p = rows[0].degree();
    let top = p * (classes.len() + 1);
    if top > MAX_DEGREE {
        return Err(GeomError::Degenerate {
            kind: GeomKind::Curve(CurveKind::Nurbs),
            reason: format!(
                "{} distinct weight functions need degree {top}, above {MAX_DEGREE}",
                classes.len()
            ),
        });
    }
    let binomials = Binomials::new(top);
    let pieces = spans[0].len();
    // Per piece: each class's normalised weights, and their product.
    let mut weight_of: Vec<Vec<Vec<f64>>> = Vec::with_capacity(pieces);
    let mut denominator: Vec<Vec<f64>> = Vec::with_capacity(pieces);
    for s in 0..pieces {
        let per_class: Vec<Vec<f64>> = classes
            .iter()
            .map(|&r| {
                let w0 = scale(r);
                spans[r][s].control.iter().map(|c| c.1 / w0).collect()
            })
            .collect();
        let product = per_class
            .iter()
            .fold(vec![1.0], |acc, w| mul(&acc, w, &binomials));
        weight_of.push(per_class);
        denominator.push(product);
    }
    // Each row's numerator per piece, coordinate by coordinate.
    let numerators: Vec<Vec<[Vec<f64>; 3]>> = (0..rows.len())
        .map(|k| {
            let w0 = scale(k);
            (0..pieces)
                .map(|s| {
                    let axis = |a: usize| -> Vec<f64> {
                        let own: Vec<f64> =
                            spans[k][s].control.iter().map(|c| c.0[a] / w0).collect();
                        weight_of[s]
                            .iter()
                            .enumerate()
                            .filter(|(c, _)| Some(*c) != class_of[k])
                            .fold(own, |acc, (_, w)| mul(&acc, w, &binomials))
                    };
                    [axis(0), axis(1), axis(2)]
                })
                .collect()
        })
        .collect();
    let d = numerators
        .iter()
        .flatten()
        .map(|q| q[0].len() - 1)
        .chain(denominator.iter().map(|w| w.len() - 1))
        .max()
        .unwrap_or(p)
        .max(1);
    // One knot vector: the pieces' ends, interior ones of multiplicity `d`.
    let mut knots = Vec::with_capacity(2 * (d + 1) + d * pieces);
    for (s, span) in spans[0].iter().enumerate() {
        let times = if s == 0 { d + 1 } else { d };
        knots.extend(core::iter::repeat_n(span.lo, times));
    }
    if let Some(end) = spans[0].last() {
        knots.extend(core::iter::repeat_n(end.hi, d + 1));
    }
    let raised: Vec<Vec<f64>> = denominator.iter().map(|w| elevate(w, d)).collect();
    let weights: Vec<f64> = raised
        .iter()
        .enumerate()
        .flat_map(|(s, w)| w.iter().copied().skip(usize::from(s > 0)))
        .collect();
    numerators
        .iter()
        .map(|row| {
            let mut points = Vec::with_capacity(weights.len());
            for (s, q) in row.iter().enumerate() {
                let [x, y, z] = [elevate(&q[0], d), elevate(&q[1], d), elevate(&q[2], d)];
                let w = &raised[s];
                for i in usize::from(s > 0)..=d {
                    points.push(arris_math::Point3::new(
                        x[i] / w[i],
                        y[i] / w[i],
                        z[i] / w[i],
                    ));
                }
            }
            NurbsCurve::new(d, knots.clone(), points, weights.clone())
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use arris_math::{Frame, Point3, Vec3, real};

    use super::*;

    fn ring(v: f64) -> Result<NurbsCurve, GeomError> {
        let frame = Frame::new(Point3::new(0.0, 0.0, v), Vec3::z(), Vec3::x()).unwrap();
        NurbsCurve::circle(&frame, 1.0 + 0.5 * real::sin(v), Interval::TURN)
    }

    /// Circles of a radius that swells and shrinks along their axis: the
    /// surface of revolution of `1 + sin(v)/2`, held to the tolerance off
    /// the checks too.
    #[test]
    fn rings_skin_to_their_surface_of_revolution() {
        let range = Interval::new(0.0, 6.0).unwrap();
        let tol = 1e-8;
        let s = fit_skin(ring, range, 3, tol, &mut Meter::default()).unwrap();
        assert_eq!(s.domain()[1], range);
        for i in 0..=97 {
            for j in 0..=101 {
                let (u, v) = (
                    core::f64::consts::TAU * i as f64 / 97.0,
                    range.lerp(j as f64 / 101.0),
                );
                let p = s.eval(u, v).point;
                let off = p.coords.xy().norm() - (1.0 + 0.5 * real::sin(v));
                assert!(
                    off.abs() <= tol && (p.z - v).abs() <= tol,
                    "{u} {v}: {off:e}"
                );
            }
        }
        // The ends are the end sections.
        for u in [0.0, 1.0, 4.0] {
            let end = ring(6.0).unwrap().eval(u).point;
            assert!((s.eval(u, 6.0).point - end).norm() <= 1e-14);
        }
    }

    /// A section that collapses to a point at the family's end gives the
    /// surface a collapsed row there: a dome closing on its pole.
    #[test]
    fn a_section_collapsed_to_a_point_is_a_collapsed_row() {
        let cap = |v: f64| -> Result<NurbsCurve, GeomError> {
            let frame =
                Frame::new(Point3::new(0.0, 0.0, real::sin(v)), Vec3::z(), Vec3::x()).unwrap();
            let c = NurbsCurve::circle(&frame, 1.0, Interval::TURN)?;
            // Scaled toward the axis: a circle of radius cos(v).
            let k = real::cos(v).max(0.0);
            let points = (c.control_points().iter())
                .map(|q| Point3::new(k * q.x, k * q.y, q.z))
                .collect();
            NurbsCurve::new(c.degree(), c.knots().to_vec(), points, c.weights().to_vec())
        };
        let range = Interval::new(0.0, core::f64::consts::FRAC_PI_2).unwrap();
        let s = fit_skin(cap, range, 3, 1e-8, &mut Meter::default()).unwrap();
        let [n, m] = s.counts();
        let pole = s.control_point(0, m - 1).unwrap();
        for i in 0..n {
            assert!((s.control_point(i, m - 1).unwrap() - pole).norm() <= 1e-15);
        }
        assert!((pole - Point3::new(0.0, 0.0, 1.0)).norm() <= 1e-15);
    }

    #[test]
    fn incompatible_sections_and_bad_requests_are_named() {
        let range = Interval::new(0.0, 1.0).unwrap();
        let mixed = |v: f64| -> Result<NurbsCurve, GeomError> {
            let arc = if v < 0.5 {
                Interval::TURN
            } else {
                Interval::new(0.0, 1.0).unwrap()
            };
            NurbsCurve::circle(&Frame::world(), 1.0, arc)
        };
        assert!(matches!(
            fit_skin(mixed, range, 3, 1e-8, &mut Meter::default()),
            Err(GeomError::Degenerate { .. })
        ));
        assert!(matches!(
            fit_skin(ring, range, 0, 1e-8, &mut Meter::default()),
            Err(GeomError::Fit(FitError::Degenerate(_)))
        ));
        assert!(matches!(
            fit_skin(ring, range, 3, 0.0, &mut Meter::default()),
            Err(GeomError::Fit(FitError::InvalidTolerance(_)))
        ));
    }

    /// A section of degree 2 on `[0, 1]`: the arc of radius `r` about
    /// `(0, 0, z)` from angle 0 to `angle`, or the segment between the
    /// same ends with weights one.
    fn arc(z: f64, r: f64, angle: f64) -> NurbsCurve {
        let frame = Frame::new(Point3::new(0.0, 0.0, z), Vec3::z(), Vec3::x()).unwrap();
        let c = NurbsCurve::circle(&frame, r, Interval::new(0.0, angle).unwrap()).unwrap();
        let knots = c.knots().iter().map(|k| k / angle).collect();
        NurbsCurve::new(2, knots, c.control_points().to_vec(), c.weights().to_vec()).unwrap()
    }

    fn segment(z: f64, a: Point3, b: Point3) -> NurbsCurve {
        let lift = |p: Point3| Point3::new(p.x, p.y, z);
        let (a, b) = (lift(a), lift(b));
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![a, a + (b - a) * 0.5, b],
            vec![1.0; 3],
        )
        .unwrap()
    }

    /// The Lagrange polynomials through `params` at `v`.
    fn lagrange(params: &[f64], v: f64) -> Vec<f64> {
        (0..params.len())
            .map(|k| {
                (0..params.len())
                    .filter(|&m| m != k)
                    .map(|m| (v - params[m]) / (params[k] - params[m]))
                    .product()
            })
            .collect()
    }

    /// An arc, the chord under it and an arc of another angle: three weight
    /// functions, one of them constant. The surface interpolates the
    /// sections' points, not their homogeneous nets, and its rows are the
    /// sections with their own parameters.
    #[test]
    fn sections_of_different_weights_are_interpolated_point_for_point() {
        let a = arc(0.0, 2.0, 1.2);
        let ends = (a.eval(0.0).point, a.eval(1.0).point);
        let sections = [a, segment(1.0, ends.0, ends.1), arc(3.0, 1.5, 0.7)];
        let params = [0.0, 0.3, 1.0];
        let s = interpolate_skin(&sections, &params, 2, false).unwrap();
        assert_eq!(s.degree(), [6, 2], "two distinct weights and a constant");
        for i in 0..=20 {
            let u = i as f64 / 20.0;
            for (c, &v) in sections.iter().zip(&params) {
                let e = (s.eval(u, v).point - c.eval(u).point).norm();
                assert!(e <= 1e-13, "row at v = {v}, u = {u}: {e:e}");
            }
            for j in 0..=10 {
                let v = j as f64 / 10.0;
                let want = sections
                    .iter()
                    .zip(lagrange(&params, v))
                    .fold(Point3::origin(), |acc, (c, l)| {
                        acc + c.eval(u).point.coords * l
                    });
                let e = (s.eval(u, v).point - want).norm();
                assert!(e <= 1e-12, "({u}, {v}): {e:e}");
            }
        }
    }

    /// Sections that share their weights keep their degree and net; the
    /// first repeated closes the surface along `v`; at degree one it is the
    /// ruled surface between consecutive sections.
    #[test]
    fn shared_weights_keep_the_net_and_closed_repeats_the_first() {
        let sections = [arc(0.0, 1.0, 1.0), arc(1.0, 2.0, 1.0), arc(2.0, 1.5, 1.0)];
        let closed = interpolate_skin(&sections, &[0.0, 0.3, 0.6, 1.0], 3, true).unwrap();
        assert_eq!(closed.degree(), [2, 3]);
        assert!(closed.closure()[1].is_some());
        for i in 0..=10 {
            let u = i as f64 / 10.0;
            let e = (closed.eval(u, 0.0).point - closed.eval(u, 1.0).point).norm();
            assert!(e <= 1e-14, "{e:e}");
        }
        let ruled = interpolate_skin(&sections, &[0.0, 0.5, 1.0], 1, false).unwrap();
        let (p, q) = (sections[0].eval(0.4).point, sections[1].eval(0.4).point);
        let mid = p + (q - p) * 0.5;
        assert!((ruled.eval(0.4, 0.25).point - mid).norm() <= 1e-14);
    }

    #[test]
    fn bad_skins_are_refused() {
        let sections = [arc(0.0, 1.0, 1.0), arc(1.0, 2.0, 1.0)];
        let refused =
            |r: Result<NurbsSurface, GeomError>| matches!(r, Err(GeomError::Degenerate { .. }));
        assert!(refused(interpolate_skin(&sections[..1], &[0.0], 1, false)));
        assert!(refused(interpolate_skin(&sections, &[0.0], 1, false)));
        assert!(refused(interpolate_skin(&sections, &[0.0, 0.0], 1, false)));
        assert!(refused(interpolate_skin(
            &sections,
            &[0.0, f64::NAN],
            1,
            false
        )));
        assert!(refused(interpolate_skin(&sections, &[0.0, 1.0], 2, false)));
        assert!(refused(interpolate_skin(&sections, &[0.0, 1.0], 0, false)));
        let other = [arc(0.0, 1.0, 1.0), arc(1.0, 1.0, 3.0)];
        assert!(refused(interpolate_skin(&other, &[0.0, 1.0], 1, false)));
    }
}
