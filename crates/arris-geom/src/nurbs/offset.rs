//! The fitted offset of a NURBS surface (ADR-0062): `S + d·N` sampled on a
//! grid over a parameter box, fitted by a tensor-product least-squares
//! B-spline, and refined per direction until a denser grid of checks is
//! within the caller's tolerance.

use arris_math::{Interval, Meter, Point2, Point3, is_negligible};

use super::fit::{
    FIT_MARGIN, FitError, least_squares_grid, least_squares_samples, sample_parameters,
};
use super::surface::NurbsSurface;
use crate::{GeomError, GeomKind, SurfaceKind};

/// The least degree of a fitted offset in each direction; a direction of
/// the surface of higher degree keeps its own. A bicubic net follows the
/// offset's curvature with a residual of fourth order per span, and the
/// rolling-ball blend that consumes it is a cubic surface along its spine.
/// A structural choice, not a tolerance.
pub const OFFSET_FIT_DEGREE: usize = 3;

/// The most spans a fitted offset may refine to in either direction before
/// it gives up with [`FitError::Diverged`]: with [`MAX_OFFSET_PATCHES`], a
/// structural bound that makes every fit terminate, as
/// [`crate::MAX_FIT_SPANS`] does for a curve. One direction may run long
/// where the other stays short: a rational twin of a cylinder of radius 3
/// to 10 takes 512 bicubic spans round a whole turn at `2.5·10⁻⁸`, and 2
/// along its axis, where a radius of 2 takes 256 (ADR-0062 §2).
pub const MAX_OFFSET_SPANS: usize = 2048;

/// The most patches, the product of the two directions' spans, a fitted
/// offset may refine to before it gives up with [`FitError::Diverged`]:
/// the grid it samples and the tracer that walks it grow with the product,
/// so the bound is the square of 256 bicubic spans, two thousand samples a
/// side. A surface folded so tightly that it needs more is one the offset
/// would cusp on anyway.
pub const MAX_OFFSET_PATCHES: usize = 65536;

/// The fewest spans each direction's first fit takes; refinement splits
/// from there.
const INITIAL_SPANS: usize = 2;

/// One direction's breaks, each interior one with the multiplicity its
/// knot takes in the fit.
#[derive(Debug, Clone)]
struct Breaks {
    at: Vec<f64>,
    /// `mult[i]` for `at[i]`; the two ends are clamped and read none.
    mult: Vec<usize>,
}

impl Breaks {
    /// The clamped knot vector of `degree` over the breaks.
    fn knots(&self, degree: usize) -> Vec<f64> {
        let last = self.at.len() - 1;
        let mut knots = vec![self.at[0]; degree + 1];
        for i in 1..last {
            knots.extend(core::iter::repeat_n(self.at[i], self.mult[i]));
        }
        knots.extend(core::iter::repeat_n(self.at[last], degree + 1));
        knots
    }

    /// The span `t` lies in.
    fn span(&self, t: f64) -> usize {
        self.at
            .partition_point(|&b| b <= t)
            .saturating_sub(1)
            .min(self.at.len() - 2)
    }

    fn spans(&self) -> usize {
        self.at.len() - 1
    }

    /// The breaks with every span flagged in `split` halved, the new
    /// breaks simple knots.
    fn refined(&self, split: &[bool]) -> Breaks {
        let mut at = Vec::with_capacity(self.at.len() + split.len());
        let mut mult = Vec::with_capacity(at.capacity());
        for (i, w) in self.at.windows(2).enumerate() {
            at.push(w[0]);
            mult.push(self.mult[i]);
            if split[i] {
                at.push(0.5 * (w[0] + w[1]));
                mult.push(1);
            }
        }
        at.push(self.at[self.at.len() - 1]);
        mult.push(0);
        Breaks { at, mult }
    }
}

/// The principal curvatures of an evaluation against its unit normal `n`,
/// positive where the surface bends toward it; `None` where the first
/// fundamental form is singular.
fn principal_curvatures(e: &crate::SurfaceEval, n: arris_math::Vec3) -> Option<[f64; 2]> {
    let (ee, ff, gg) = (e.du.dot(&e.du), e.du.dot(&e.dv), e.dv.dot(&e.dv));
    let (l, m, nn) = (e.duu.dot(&n), e.duv.dot(&n), e.dvv.dot(&n));
    let det = ee * gg - ff * ff;
    if det.is_nan() || det <= 0.0 {
        return None;
    }
    let gauss = (l * nn - m * m) / det;
    let mean = (ee * nn - 2.0 * ff * m + gg * l) / (2.0 * det);
    let root = (mean * mean - gauss).max(0.0).sqrt();
    Some([mean - root, mean + root])
}

impl NurbsSurface {
    /// The surface's offset by `distance` along its normal (positive the
    /// way `∂P/∂u × ∂P/∂v` points), fitted over the parameter box `over`
    /// as a non-rational B-spline: `result.eval(u, v)` is within `tol` of
    /// `P(u, v) + distance · N(u, v)` at the same `(u, v)`, checked on a
    /// grid of the fit's samples and the midpoints between them, and its
    /// domain is `over` exactly (ADR-0062). The degree in each direction is
    /// [`OFFSET_FIT_DEGREE`] or the surface's own, the higher. Every
    /// interior knot of the surface inside `over` is a knot of the result,
    /// one continuity order lower, since the normal is a derivative. Where
    /// `over` reaches past the surface's domain, the offset is of the end
    /// span's own polynomial carried on, as [`NurbsSurface::eval`] carries
    /// it; a closed direction wraps instead. [`crate::Surface::offset`]
    /// is the exact offset of an analytic surface; this is the NURBS
    /// one's.
    ///
    /// Guarantees: deterministic; terminates, refining no direction past
    /// [`MAX_OFFSET_SPANS`] and the two past [`MAX_OFFSET_PATCHES`];
    /// `meter` is ticked once per sample of the
    /// surface and once per check.
    ///
    /// Errors:
    /// - [`GeomError::OffsetFolds`]: at a sample, `distance` reaches or
    ///   passes a principal radius on the side it moves to, so the offset
    ///   folds there and is no surface.
    /// - [`GeomError::Degenerate`]: a collapsed row of the net (a pole)
    ///   lies in `over`, or the normal is undefined at a sample, so the
    ///   offset has no point there; a non-finite `distance`; a box that is
    ///   not bounded or has an empty side.
    /// - [`GeomError::Fit`]: the tolerance is not finite and positive
    ///   ([`FitError::InvalidTolerance`]), or the fit still misses with
    ///   [`MAX_OFFSET_SPANS`] spans in a direction or
    ///   [`MAX_OFFSET_PATCHES`] patches ([`FitError::Diverged`]) — a crease of
    ///   the surface, whose offset tears, ends here.
    /// - [`GeomError::Interrupted`]: the caller stopped it.
    ///
    /// ```
    /// use arris_geom::Surface;
    /// use arris_math::{Frame, Interval, Meter};
    ///
    /// // The exact B-spline twin of a quarter of a cylinder of radius 2,
    /// // offset 0.5 outward: a quarter of a cylinder of radius 2.5.
    /// let wall = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
    /// let quarter = Interval::new(0.0, core::f64::consts::FRAC_PI_2).unwrap();
    /// let height = Interval::new(0.0, 1.0).unwrap();
    /// let twin = wall.to_nurbs([quarter, height]).unwrap();
    /// let out = twin
    ///     .offset_fitted(0.5, twin.domain(), 1e-7, &mut Meter::default())
    ///     .unwrap();
    /// assert_eq!(out.domain(), twin.domain());
    /// let p = out.eval(0.3, 0.7).point;
    /// assert!((p.coords.xy().norm() - 2.5).abs() <= 1e-7);
    /// ```
    pub fn offset_fitted(
        &self,
        distance: f64,
        over: [Interval; 2],
        tol: f64,
        meter: &mut Meter,
    ) -> Result<NurbsSurface, GeomError> {
        let kind = GeomKind::Surface(SurfaceKind::Nurbs);
        let degenerate = |reason: String| GeomError::Degenerate { kind, reason };
        if !distance.is_finite() {
            return Err(degenerate(format!(
                "the offset distance {distance} is not finite"
            )));
        }
        if !(tol.is_finite() && tol > 0.0) {
            return Err(FitError::InvalidTolerance(tol).into());
        }
        for (k, range) in over.iter().enumerate() {
            if !range.is_bounded() || range.length() <= 0.0 {
                return Err(degenerate(format!(
                    "the offset's {} range [{}, {}] is not a bounded interval of positive length",
                    ["u", "v"][k],
                    range.lo(),
                    range.hi()
                )));
            }
        }
        for (k, value, _) in self.collapsed_rows() {
            if over[k].contains(value) {
                return Err(degenerate(format!(
                    "a collapsed row at {} = {value} lies in the offset's box: the normal is \
                     undefined along it",
                    ["u", "v"][k]
                )));
            }
        }
        let [p, q] = self.degree();
        let degree = [p.max(OFFSET_FIT_DEGREE), q.max(OFFSET_FIT_DEGREE)];
        let mut breaks = [0, 1].map(|k| self.initial_breaks(k, over[k], degree[k]));
        // The offset at `(u, v)`, refused where it has no point or folds.
        let offset = |u: f64, v: f64| -> Result<Point3, GeomError> {
            let e = self.eval(u, v);
            let n = self
                .normal(u, v)
                .ok_or_else(|| degenerate(format!("no normal at ({u}, {v})")))?
                .into_inner();
            let kappa = principal_curvatures(&e, n)
                .ok_or_else(|| degenerate(format!("no normal at ({u}, {v})")))?;
            for k in kappa {
                let left = 1.0 - distance * k;
                if left <= 0.0 || is_negligible(left, 1.0) {
                    return Err(GeomError::OffsetFolds {
                        kind,
                        at: Point2::new(u, v),
                        distance,
                        radius: 1.0 / k.abs(),
                    });
                }
            }
            Ok(e.point + distance * n)
        };
        loop {
            let samples = [0, 1].map(|k| sample_parameters(&breaks[k].at, degree[k]));
            let knots = [0, 1].map(|k| breaks[k].knots(degree[k]));
            // Each direction's checks: its samples and the midpoints between
            // them, the samples at the even places.
            let checks = samples.clone().map(|s| {
                let mut c: Vec<f64> = Vec::with_capacity(2 * s.len());
                for w in s.windows(2) {
                    c.push(w[0]);
                    c.push(0.5 * (w[0] + w[1]));
                }
                c.push(s[s.len() - 1]);
                c
            });
            // The offset along each sample line of one direction at the
            // other's checks: `lines[k][i]` runs along direction `k` at
            // the other direction's `i`-th sample.
            let mut lines: [Vec<Vec<Point3>>; 2] = [Vec::new(), Vec::new()];
            for k in 0..2 {
                for &across in &samples[1 - k] {
                    let mut line = Vec::with_capacity(checks[k].len());
                    for &t in &checks[k] {
                        meter.tick()?;
                        let (u, v) = if k == 0 { (t, across) } else { (across, t) };
                        line.push(offset(u, v)?);
                    }
                    lines[k].push(line);
                }
            }
            // First each direction alone: every sample line fitted along
            // it is held to half the margin at its checks, and a span that
            // misses is that direction's to split.
            let mut split = [0, 1].map(|k| vec![false; breaks[k].spans()]);
            let mut worst = 0.0f64;
            for k in 0..2 {
                for line in &lines[k] {
                    let at_samples: Vec<Point3> = line.iter().step_by(2).copied().collect();
                    let fit = least_squares_samples(
                        degree[k],
                        &knots[k],
                        &samples[k],
                        &at_samples,
                        meter,
                    )?;
                    for (&t, target) in checks[k].iter().zip(line) {
                        meter.tick()?;
                        let d = (fit.eval(t).point - target).norm();
                        if !d.is_finite() {
                            return Err(FitError::NonFinite { t }.into());
                        }
                        worst = worst.max(d);
                        if d > 0.5 * FIT_MARGIN * tol {
                            split[k][breaks[k].span(t)] = true;
                        }
                    }
                }
            }
            if split.iter().all(|s| !s.iter().any(|&x| x)) {
                // Then the surface: the tensor product of the two, held to
                // the margin over the whole grid of checks.
                let data: Vec<Point3> = (0..samples[0].len())
                    .flat_map(|i| lines[1][i].iter().step_by(2).copied())
                    .collect();
                let net = least_squares_grid(
                    degree,
                    [&knots[0], &knots[1]],
                    [&samples[0], &samples[1]],
                    &data,
                    meter,
                )?;
                let weights = vec![1.0; net.len()];
                let fitted = NurbsSurface::new(degree, knots, net, weights)?;
                for (a, &u) in checks[0].iter().enumerate() {
                    for (b, &v) in checks[1].iter().enumerate() {
                        meter.tick()?;
                        // Where either parameter is a sample the offset is
                        // on a line already.
                        let target = match (a % 2, b % 2) {
                            (0, _) => lines[1][a / 2][b],
                            (_, 0) => lines[0][b / 2][a],
                            _ => offset(u, v)?,
                        };
                        let d = (fitted.eval(u, v).point - target).norm();
                        if !d.is_finite() {
                            return Err(FitError::NonFinite { t: u }.into());
                        }
                        worst = worst.max(d);
                        if d > FIT_MARGIN * tol {
                            split[0][breaks[0].span(u)] = true;
                            split[1][breaks[1].span(v)] = true;
                        }
                    }
                }
                if split.iter().all(|s| !s.iter().any(|&x| x)) {
                    return Ok(fitted);
                }
            }
            let spans = [0, 1].map(|k| breaks[k].spans() + split[k].iter().filter(|&&x| x).count());
            for k in 0..2 {
                if spans[k] > MAX_OFFSET_SPANS || spans[0] * spans[1] > MAX_OFFSET_PATCHES {
                    return Err(FitError::Diverged {
                        spans: breaks[k].spans(),
                        deviation: worst,
                    }
                    .into());
                }
            }
            breaks = [0, 1].map(|k| breaks[k].refined(&split[k]));
        }
    }

    /// Direction `k`'s first breaks over `range` for a fit of `degree`:
    /// the range's ends, and every knot of the surface strictly inside it
    /// — a closed direction's seam and a periodic one's knots repeated by
    /// the closure — with the multiplicity that leaves the fit one order
    /// less continuous than the surface there; then halved until there
    /// are [`INITIAL_SPANS`].
    fn initial_breaks(&self, k: usize, range: Interval, degree: usize) -> Breaks {
        let base = self.degree()[k];
        let knots = self.knots()[k];
        let domain = self.domain()[k];
        // The surface's interior knots with their multiplicity, a closed
        // seam as a C0 joint.
        let mut joints: Vec<(f64, usize)> = Vec::new();
        for &t in knots {
            if t <= domain.lo() || t >= domain.hi() {
                continue;
            }
            match joints.last_mut() {
                Some((x, m)) if *x == t => *m += 1,
                _ => joints.push((t, 1)),
            }
        }
        let closure = self.closure()[k];
        if closure.is_some() && self.period()[k].is_none() {
            joints.push((domain.lo(), base));
        }
        let mut found: Vec<(f64, usize)> = Vec::new();
        match closure {
            Some(c) => {
                // Every repeat of a joint by the closure inside the range.
                for &(t, m) in &joints {
                    let first = ((range.lo() - t) / c).ceil() as i64;
                    let last = ((range.hi() - t) / c).floor() as i64;
                    for turn in first..=last {
                        found.push((t + turn as f64 * c, m));
                    }
                }
            }
            None => found.extend(joints.iter().copied()),
        }
        found.retain(|&(t, _)| t > range.lo() && t < range.hi());
        found.sort_by(|a, b| a.0.total_cmp(&b.0));
        found.dedup_by(|a, b| a.0 == b.0);
        let mut at = vec![range.lo()];
        let mut mult = vec![0];
        for (t, m) in found {
            // The surface is C^(base − m) there and its normal one less.
            let normal = base.saturating_sub(m).saturating_sub(1);
            at.push(t);
            mult.push(degree.saturating_sub(normal).clamp(1, degree));
        }
        at.push(range.hi());
        mult.push(0);
        let mut breaks = Breaks { at, mult };
        while breaks.spans() < INITIAL_SPANS {
            let all = vec![true; breaks.spans()];
            breaks = breaks.refined(&all);
        }
        breaks
    }
}
