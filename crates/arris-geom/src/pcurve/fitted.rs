//! The fitted pcurve: singular ends, and the NURBS fitted over the surface's projection.

use arris_math::real;
use arris_math::roots::golden_section;

use arris_math::{
    Interval, Meter, Point2, Point3, Tolerance, Vec3, is_negligible, wrap_angle as wrap_turn,
    wrap_signed,
};

use crate::{Curve, Curve2, GeomError, GeomKind, PcurveFault, Surface, fit_curve2};

use super::*;

/// An end of the range that is on a singular point of the surface.
#[derive(Debug, Clone, Copy)]
pub(super) struct SingularEnd {
    pub(super) point: Point3,
    /// By how much the curve's end misses the point, at most the band.
    pub(super) miss: Vec3,
    /// The pcurve's end: the free parameter the curve arrives with, the
    /// point's own value of the fixed one.
    pub(super) uv: Point2,
    /// The parameter beyond which the curve is left where it is.
    pub(super) exit: f64,
}

/// The parameter in `[a, b]` at which `curve` is nearest `point`, and the
/// distance there, by golden section: the bracket is one the sampling
/// found a single dip in.
pub(super) fn nearest_approach(curve: &Curve, point: Point3, a: f64, b: f64) -> (f64, f64) {
    let d = |t: f64| (curve.point(t) - point).norm();
    let end = golden_section(d, a, b, GOLDEN_STEPS);
    let (a, b) = (end.lo, end.hi);
    let ((x1, d1), (x2, d2)) = (end.low_probe, end.high_probe);
    // An end of the bracket may be nearer than anything inside it.
    [(a, d(a)), (b, d(b)), (x1, d1), (x2, d2)]
        .into_iter()
        .fold(
            (a, f64::INFINITY),
            |best, c| if c.1 < best.1 { c } else { best },
        )
}

/// Golden-section steps of [`nearest_approach`]: each shrinks the bracket
/// by 0.618, so eighty take one sampling interval below the spacing of
/// `f64` parameters. An iteration count, not a tolerance.
pub(super) const GOLDEN_STEPS: usize = 80;

/// How many times the interval between two of the [`PCURVE_SAMPLES`] is
/// halved to follow a periodic parameter that swings faster than a
/// quarter turn between them, which is what `u` does beside a pole or an
/// apex: by `π` over a stretch as long as the miss. Thirty-two halvings
/// of a 256th of the range reach a miss of `1e-12` of the curve's length;
/// a step that is still a quarter turn there is a jump, and is refused. A
/// sampling depth, not a tolerance.
pub(super) const UNWRAP_DEPTH: usize = 32;

/// Where, in units of the band from a singular point a range ends on,
/// the curve is carried onto the point in full and where it is left
/// alone, with a smooth fade between: the end misses the point by at most
/// one unit, so its `u` as seen from the point is wrong by up to a right
/// angle within a few units and by a sixteenth of a radian at the far
/// bound, where the 3D effect of either reading is the same miss. Ratios
/// of the band, not tolerances.
pub(super) const SINGULAR_FADE: [f64; 2] = [4.0, 16.0];

/// `1` at or below `SINGULAR_FADE[0]`, `0` at or above `SINGULAR_FADE[1]`,
/// the C² quintic step between.
pub(super) fn fade(x: f64) -> f64 {
    let [near, far] = SINGULAR_FADE;
    let s = ((x - near) / (far - near)).clamp(0.0, 1.0);
    1.0 - s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)
}

/// The ends of `range` that are within the band of a singular point of
/// the surface ([`PCURVE_SINGULAR_BAND`]), or [`GeomError::ThroughSingularity`] when the curve
/// comes that near one anywhere else: the decision is a distance, taken
/// at the nearest approach inside each sampling interval that dips. The
/// parameters of the nearer approaches that stay outside the band come
/// with them, ascending: beside a singular point `u` swings by `π` and
/// back over a stretch as long as the miss, which a sampling coarser than
/// that never sees.
pub(super) fn singular_ends(
    curve: &Curve,
    range: Interval,
    surface: &Surface,
    tol: Tolerance,
) -> Result<([Option<SingularEnd>; 2], Vec<f64>), GeomError> {
    let kind = GeomKind::Curve(curve.kind());
    let n = PCURVE_SAMPLES;
    let ts: Vec<f64> = (0..=n).map(|i| range.lerp(i as f64 / n as f64)).collect();
    let points: Vec<Point3> = ts.iter().map(|&t| curve.point(t)).collect();
    let band = PCURVE_SINGULAR_BAND * tol.linear;
    let mut ends = [None, None];
    let mut passes = Vec::new();
    for singular in surface.singularities() {
        let d: Vec<f64> = (points.iter())
            .map(|p| (p - singular.point).norm())
            .collect();
        let at_end = [d[0] <= band, d[n] <= band];
        for i in 0..=n {
            let (lo, hi) = (i.saturating_sub(1), (i + 1).min(n));
            let dips = d[i] <= d[lo] && d[i] <= d[hi];
            let reach = band + (points[hi] - points[lo]).norm();
            if !dips || d[i] > reach {
                continue;
            }
            let (t, nearest) = nearest_approach(curve, singular.point, ts[lo], ts[hi]);
            let of_an_end = (at_end[0] && lo == 0) || (at_end[1] && hi == n);
            if nearest <= band && !of_an_end {
                return Err(GeomError::ThroughSingularity {
                    curve: kind,
                    surface: GeomKind::Surface(surface.kind()),
                    t,
                });
            }
            if !of_an_end && range.lo() < t && t < range.hi() {
                passes.push(t);
            }
        }
        for (side, &end) in [0, n].iter().enumerate() {
            if !at_end[side] {
                continue;
            }
            // The free parameter the curve arrives with is its tangent's,
            // read where the tangent leads from the point into the range:
            // as far as the curve's middle on a surface of revolution,
            // where every distance along it reads the same, and on a
            // NURBS just clear of the fade, where the limit is read before
            // the surface bends away from the tangent.
            let into = if side == 0 { 1.0 } else { -1.0 };
            let middle = (curve.point(range.midpoint()) - singular.point).norm();
            let reach = match surface {
                Surface::Nurbs(_) => middle.min(SINGULAR_FADE[1] * band),
                Surface::Plane { .. }
                | Surface::Cylinder { .. }
                | Surface::EllipticCylinder { .. }
                | Surface::Cone { .. }
                | Surface::Sphere { .. }
                | Surface::Torus { .. } => middle,
            };
            let tangent = (into * curve.eval(ts[end]).d1)
                .try_normalize(0.0)
                .filter(|_| reach > 0.0)
                .ok_or_else(|| degenerate(kind, PcurveFault::SingularNoTangent))?;
            let free = surface.project(singular.point + reach * tangent)?.uv[1 - singular.fixed];
            // Carried onto the point until the first sample clear of the
            // fade, or the middle of a range too short to have one.
            let far = SINGULAR_FADE[1] * band;
            let inward: Vec<usize> = if side == 0 {
                (1..=n / 2).collect()
            } else {
                (n / 2..n).rev().collect()
            };
            let exit = (inward.iter().find(|&&i| d[i] >= far)).map_or(range.midpoint(), |&i| ts[i]);
            ends[side] = Some(SingularEnd {
                point: singular.point,
                miss: points[end] - singular.point,
                uv: if singular.fixed == 1 {
                    Point2::new(free, singular.value)
                } else {
                    Point2::new(singular.value, free)
                },
                exit,
            });
        }
    }
    passes.sort_by(f64::total_cmp);
    passes.dedup();
    Ok((ends, passes))
}

/// A NURBS pcurve fitted over the surface's own projection of the curve
/// ([`Surface::project`]), each periodic parameter unwrapped along `t`
/// through a table of [`PCURVE_SAMPLES`] parameters, halved where the
/// parameter swings; the rules by a singular point are [`pcurve_on`]'s.
pub(super) fn fitted_on(
    curve: &Curve,
    range: Interval,
    surface: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Curve2, GeomError> {
    let kind = GeomKind::Curve(curve.kind());
    let (ends, passes) = singular_ends(curve, range, surface, tol)?;
    let origin = surface.frame().map_or(0.0, |f| f.origin().coords.norm());
    let band = PCURVE_SINGULAR_BAND * tol.linear;
    let raw = |t: f64| -> Result<Point2, GeomError> {
        let mut p = curve.point(t);
        // An end is read on its own side of the range only: a closed curve
        // through a pole, cut there, ends on the one point twice, half a
        // turn of `u` apart.
        let carried = |side: usize, end: &SingularEnd| {
            if side == 0 {
                t <= end.exit
            } else {
                t >= end.exit
            }
        };
        for (side, end) in ends.iter().enumerate() {
            let Some(end) = end else { continue };
            if carried(side, end) {
                p -= fade((p - end.point).norm() / band) * end.miss;
            }
        }
        for (side, end) in ends.iter().enumerate() {
            let Some(end) = end else { continue };
            if carried(side, end) && is_negligible((p - end.point).norm(), p.coords.norm() + origin)
            {
                return Ok(end.uv);
            }
        }
        let uv = surface.project(p)?.uv;
        // A cylinder's `u` as `atan2` gives it rather than wrapped into
        // the turn: the unwrapping puts the turns back either way, and
        // wrapping first costs the last bit of a negative angle, which
        // would move every fitted pcurve a cylinder already carries.
        Ok(match surface {
            Surface::Cylinder { frame, .. } => {
                let q = frame.to_local(p);
                Point2::new(real::atan2(q.y, q.x), uv.y)
            }
            Surface::Plane { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_) => uv,
        })
    };
    // A turn on the analytic surfaces; a NURBS surface's closure, over
    // which its projection already reports a parameter inside the domain.
    let periods = surface.period();
    // `q` with each periodic parameter moved the short way round from
    // `near`'s, which is how far along the unwrapping it is.
    let beside = |near: [f64; 2], q: Point2| {
        [0, 1].map(|k| match periods[k] {
            Some(period) => near[k] + wrap_signed(q[k] - near[k], period),
            None => q[k],
        })
    };
    let n = PCURVE_SAMPLES;
    let first = raw(range.lo())?;
    let start = [0, 1].map(|k| match (periods[k], surface) {
        (Some(_), Surface::Nurbs(_)) | (None, _) => first[k],
        (Some(_), _) => wrap_turn(first[k]),
    });
    // The unwrapped parameters along `t`, and where in the table each of
    // the `n + 1` even samples is: the halvings lie between them.
    let mut table: Vec<(f64, [f64; 2])> = Vec::with_capacity(n + 1);
    let mut even: Vec<usize> = Vec::with_capacity(n + 1);
    table.push((range.lo(), start));
    even.push(0);
    for i in 1..=n {
        // The interval's end, after the nearest approach to a singular
        // point inside it, each of which is a sample: the table follows
        // the swing of `u` there, which the even samples step over.
        let end = range.lerp(i as f64 / n as f64);
        let before = table[table.len() - 1].0;
        let targets = passes
            .iter()
            .copied()
            .filter(|&t| before < t && t < end)
            .chain([end]);
        for target in targets {
            // Depth-first over the halvings of this step, nearest first.
            let mut pending = vec![(target, 0usize)];
            while let Some(&(t, depth)) = pending.last() {
                let (t0, last) = table[table.len() - 1];
                let next = beside(last, raw(t)?);
                // The swing as a fraction of the turn.
                let swing = (0..2)
                    .filter_map(|k| periods[k].map(|period| (next[k] - last[k]).abs() / period))
                    .fold(0.0, f64::max);
                // A NaN is not below the bound, and is not halved away either.
                if swing < 0.25 {
                    table.push((t, next));
                    pending.pop();
                } else if depth < UNWRAP_DEPTH && swing.is_finite() {
                    // Both halves are one level down: the rest of the
                    // interval is looked at again once the near half is in.
                    if let Some(rest) = pending.last_mut() {
                        rest.1 = depth + 1;
                    }
                    pending.push((0.5 * (t0 + t), depth + 1));
                } else {
                    return Err(degenerate(
                        kind,
                        PcurveFault::WindsTooFast {
                            swing,
                            samples: n,
                            depth: UNWRAP_DEPTH,
                        },
                    ));
                }
            }
        }
        even.push(table.len() - 1);
    }
    let f = |t: f64| {
        // The nearest even sample, and where an interval beside it was
        // halved, the nearest of what the halving put there.
        let s = ((t - range.lo()) / range.length() * n as f64).round();
        let i = (s.max(0.0) as usize).min(n);
        let (from, to) = (even[i.saturating_sub(1)], even[(i + 1).min(n)]);
        let mut near = table[even[i]];
        if to - from > 2 {
            for &entry in &table[from..=to] {
                if (entry.0 - t).abs() < (near.0 - t).abs() {
                    near = entry;
                }
            }
        }
        // On a NURBS surface, whose projection is a search, the table's
        // own neighbour starts a local one: taken where it lands within
        // the band of the curve — what the curve's own point is at most
        // off a surface that holds it — and not beside a singular end,
        // whose carrying `raw` does.
        if let Surface::Nurbs(nurbs) = surface {
            let beside_an_end = ends.iter().enumerate().any(|(side, end)| {
                end.is_some_and(|end| {
                    if side == 0 {
                        t <= end.exit
                    } else {
                        t >= end.exit
                    }
                })
            });
            if !beside_an_end {
                let local = nurbs.project_from(curve.point(t), Point2::new(near.1[0], near.1[1]));
                if local.distance <= band {
                    let [u, v] = beside(near.1, local.uv);
                    return Point2::new(u, v);
                }
            }
        }
        let Ok(q) = raw(t) else {
            return Point2::new(f64::NAN, f64::NAN);
        };
        let [u, v] = beside(near.1, q);
        Point2::new(u, v)
    };
    let deviation = |t: f64, q: Point2| (surface.point(q.x, q.y) - curve.point(t)).norm();
    let fit = fit_curve2(f, range, PCURVE_FIT_DEGREE, deviation, tol.linear, meter)?;
    Ok(Curve2::Nurbs(fit))
}
