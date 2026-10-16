//! A surface skinned through a family of compatible sections (ADR-0062
//! §1): each section's homogeneous control net fitted along the family's
//! parameter by the curve fit's least squares, the knots along it refined
//! until the surface is within the caller's tolerance of every section it
//! is checked against. The rolling-ball blend skins the ball's exact arcs
//! along its spine; loft is the next consumer.

use arris_math::nalgebra::{Const, OPoint, Point4};
use arris_math::{Interval, Meter};

use super::basis::MAX_DEGREE;
use super::fit::{
    FIT_MARGIN, FitError, INITIAL_SPANS, MAX_FIT_SPANS, clamped_knots, least_squares_samples,
    sample_parameters,
};
use super::{NurbsCurve, NurbsSurface};
use crate::{CurveKind, GeomError, GeomKind};

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
}
