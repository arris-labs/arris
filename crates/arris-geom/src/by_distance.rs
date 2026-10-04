//! The verdict a curve and an analytic surface get from the signed
//! distance along the curve, shared by the arms that can say *where to
//! look*: a NURBS curve's spans put into the surface's implicit
//! polynomial (`crate::intersect_spline`) and a conic put into a
//! quadric's, which is a trigonometric polynomial of degree two
//! (`crate::intersect_curve`).
//!
//! Either arm hands over **splits**: parameters that include every
//! extremum and every kink of the distance, so that between two
//! consecutive ones the distance is monotone and crosses zero at most
//! once. Any others only split a monotone stretch, so a refinement is
//! free. What is decided there is decided in length, as every closed
//! form of the table decides it: on the exact signed
//! [`Implicit::distance`], never on the polynomial, whose scale is the
//! surface's dimensions to the power of its degree.

use arris_math::roots;
use arris_math::{Interval, Tolerance, is_negligible};

use crate::implicit::Implicit;
use crate::intersect_curve::{hit, points};
use crate::{Curve, CurveSurfaceIntersection, GeomError, GeomKind, Surface};

/// A parameter at which the distance is looked at: an extremum of it
/// among its neighbours, or an end of an open curve.
#[derive(Clone, Copy)]
struct Stop {
    t: f64,
    distance: f64,
    /// Within `tol.linear` of the surface.
    touch: bool,
    /// An extremum inside the curve rather than an end of it.
    interior: bool,
}

/// The hits of `curve` on `surface` by the signed distance along it,
/// given `splits` (module docs); `implicit` is the surface's polynomial,
/// which only the distance and its gradient are read from here.
///
/// Of the splits, the **stops** are the ones where the distance is an
/// extremum among its neighbours, and the two ends of a curve that is not
/// periodic, which are extrema of a function on a closed interval. Then:
/// every stop within `tol.linear` is the whole curve within it,
/// `Coincident`; a stop within `tol.linear` is a hit that absorbs the
/// crossings on the stretches beside it, and a run of such stops with no
/// other between them is one hit, at the one nearest the surface; two
/// consecutive other stops of opposite sign hold one crossing, by
/// bracketed Newton on the distance. A hit at a stop is `tangent` when
/// its run holds an extremum inside the curve: a curve that only *ends*
/// within `tol.linear` of the surface meets it there without touching it
/// — that is a section edge ending on a face, which the pave model makes
/// a vertex of, not a graze. A periodic curve's stops go round, and its
/// hits come back in its own domain. So do a *closed* curve's that is not
/// periodic — a clamped B-spline whose two ends are one point to
/// rounding, as a file's closed curve is: its two ends are one parameter,
/// the start, and a hit at the join is one hit there, not one at each
/// end ([`closure`]).
pub(crate) fn hits_by_distance(
    curve: &Curve,
    surface: &Surface,
    implicit: &Implicit<'_>,
    mut splits: Vec<f64>,
    tol: Tolerance,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let domain = curve.domain();
    let period = closure(curve);
    // The join of a closed curve is its start: the end is dropped, and
    // the curve is read past it by wrapping, as a periodic one reads
    // itself.
    let closed = period.is_some() && curve.period().is_none();
    let at = |t: f64| match period {
        Some(period) if closed && t > domain.hi() => t - period,
        _ => t,
    };
    if closed {
        splits.retain(|&t| t < domain.hi());
    }
    splits.retain(|t| t.is_finite());
    splits.sort_by(f64::total_cmp);
    splits.dedup();
    if splits.is_empty() {
        // Nowhere to look: a validated curve has a span and a conic has
        // its whole turn, so only knots that are not finite numbers —
        // which no constructor admits — reach here.
        return Ok(CurveSurfaceIntersection::Points(Vec::new()));
    }

    let distance = |t: f64| implicit.distance(curve.eval(at(t)).point);
    let slope = |t: f64| {
        let e = curve.eval(at(t));
        implicit
            .gradient(e.point)
            .dot(&implicit.frame.vec_to_local(e.d1))
    };
    let values: Vec<f64> = splits.iter().map(|&t| distance(t)).collect();
    let n = splits.len();
    let cyclic = period.is_some();
    let neighbour = |i: usize, step: isize| -> Option<f64> {
        let j = i as isize + step;
        if cyclic {
            Some(values[j.rem_euclid(n as isize) as usize])
        } else if j < 0 || j >= n as isize {
            None
        } else {
            Some(values[j as usize])
        }
    };
    let mut stops: Vec<Stop> = Vec::new();
    for i in 0..n {
        let v = values[i];
        let (before, after) = (neighbour(i, -1), neighbour(i, 1));
        let low = before.is_none_or(|b| b >= v) && after.is_none_or(|a| a >= v);
        let high = before.is_none_or(|b| b <= v) && after.is_none_or(|a| a <= v);
        if low || high {
            stops.push(Stop {
                t: splits[i],
                distance: v,
                touch: v.abs() <= tol.linear,
                interior: before.is_some() && after.is_some(),
            });
        }
    }
    if stops.iter().all(|s| s.touch) {
        return Ok(CurveSurfaceIntersection::Coincident);
    }
    if cyclic {
        // Started at a stop clear of the surface, no run of touches goes
        // round the end of the list.
        let first = stops.iter().position(|s| !s.touch).unwrap_or(0);
        stops.rotate_left(first);
    }
    let mut runs: Vec<Stop> = Vec::with_capacity(stops.len());
    for stop in stops {
        match runs.last_mut() {
            Some(last) if last.touch && stop.touch => {
                let interior = last.interior || stop.interior;
                if stop.distance.abs() < last.distance.abs() {
                    *last = stop;
                }
                last.interior = interior;
            }
            _ => runs.push(stop),
        }
    }

    let crossing = |lo: f64, hi: f64| -> Result<f64, GeomError> {
        let degenerate = |reason: String| GeomError::Degenerate {
            kind: GeomKind::Curve(curve.kind()),
            reason,
        };
        let bracket = Interval::new(lo, hi)
            .map_err(|_| degenerate(format!("crossing bracket [{lo}, {hi}]")))?;
        roots::newton_in_interval(distance, slope, bracket, 0.0)
            .map_err(|e| degenerate(format!("crossing in [{lo}, {hi}]: {e}")))
    };
    let mut hits = Vec::new();
    for (i, stop) in runs.iter().enumerate() {
        if stop.touch {
            hits.push(hit(curve, surface, stop.t, stop.interior)?);
        }
        // The stretch to the next stop; a periodic curve's goes round,
        // past the end of the domain wherever the next stop is behind.
        let next = match (runs.get(i + 1), period) {
            (Some(next), _) => *next,
            (None, Some(_)) if runs.len() > 1 => runs[0],
            (None, _) => continue,
        };
        let next_t = match period {
            Some(period) if next.t <= stop.t => next.t + period,
            _ => next.t,
        };
        if stop.touch || next.touch || (stop.distance < 0.0) == (next.distance < 0.0) {
            continue;
        }
        let mut t = if closed && stop.t < domain.hi() && next_t > domain.hi() {
            // The stretch runs over the join of a closed curve whose two
            // ends are one point, but which is no smoother there than
            // its first span: that span can be narrower than a parameter
            // near the end resolves, so a bracket across the join
            // converges on the jump from the end to the start's far side,
            // not on a root. Each side of the join is looked at alone,
            // the sign at the join (the start's) saying which holds the
            // crossing.
            if (stop.distance < 0.0) != (distance(domain.lo()) < 0.0) {
                crossing(stop.t, domain.hi())?
            } else {
                crossing(domain.lo(), next_t - period.unwrap_or(0.0))?
            }
        } else {
            crossing(stop.t, next_t)?
        };
        if let Some(period) = period {
            if t >= domain.hi() {
                t = (t - period).max(domain.lo());
            }
            // A closed curve's end is its start: a crossing found a
            // rounding short of it, at the scale of the parameters, is
            // there.
            let knots = domain.lo().abs().max(domain.hi().abs());
            if closed && is_negligible(domain.hi() - t, knots) {
                t = domain.lo();
            }
        }
        hits.push(hit(curve, surface, t, false)?);
    }
    Ok(points(hits))
}

/// The length over which `curve` repeats: its period, or the length of
/// its domain where it is *closed* without being periodic — bounded, and
/// its two ends one point to rounding ([`is_negligible`] against their
/// coordinates), which a clamped B-spline in a file is and no curve the
/// kernel makes is (ROADMAP §C4).
pub(crate) fn closure(curve: &Curve) -> Option<f64> {
    curve.period().or_else(|| {
        let domain = curve.domain();
        if !domain.is_bounded() || domain.length() <= 0.0 {
            return None;
        }
        let (a, b) = (curve.point(domain.lo()), curve.point(domain.hi()));
        let scale = a.coords.norm().max(b.coords.norm());
        is_negligible((a - b).norm(), scale).then(|| domain.length())
    })
}
