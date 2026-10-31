//! What of a section lies on a torus's regular band (ADR-0080): a horn's
//! or a spindle's section is traced on its polynomial, whose zero set
//! holds the axis point(s) and a spindle's lemon besides, and only what is
//! off the axis on the band's own sheet is kept.
//!
//! The band's edge is a point of the axis — one on a horn, two on a
//! spindle — and nowhere else does a branch pass from the band to what is
//! not. A section through an axis point has a singular point there: on
//! the walked torus the axis point is a whole row of `v` collapsed, the
//! section's polynomial vanishes along all of it, and a branch crossing it
//! is a saddle with the row; on a torus that is the other surface the
//! polynomial's gradient vanishes at the axis point. Either way the tracer
//! ends branches there (`crate::torus_walk`'s singular points), so each
//! branch lies on the band or off it whole, and is kept or dropped whole;
//! one that runs from the band to the lemon without ending at the axis is
//! refused. The rows themselves, a branch on the axis all along, are off
//! the band.

use super::*;

use std::collections::BTreeMap;

/// How many points of a branch or a circle are looked at to tell which
/// side of the band's edge it lies on: it lies on one side whole, so any
/// point away from its ends would do; the count is there to catch one
/// that does not. A sampling count, not a tolerance.
const BAND_SAMPLES: usize = 32;

/// A torus whose regular band is not the whole of it — a horn or a
/// spindle, or a ring whose hole is within `tol` of closing — in its
/// frame, with its radii.
struct BandTorus<'s> {
    frame: &'s Frame,
    major: f64,
    minor: f64,
}

impl BandTorus<'_> {
    fn of(surface: &Surface, tol: Tolerance) -> Option<BandTorus<'_>> {
        match surface {
            Surface::Torus {
                frame,
                major_radius,
                minor_radius,
            } if major_radius - minor_radius <= tol.linear => Some(BandTorus {
                frame,
                major: *major_radius,
                minor: *minor_radius,
            }),
            _ => None,
        }
    }

    /// The distance of `p` from the axis, signed by the sheet `p` is
    /// nearer: positive on the band's, the tube about the centre circle,
    /// and negative on its mirror across the axis, of which a spindle's
    /// lemon is the part on this side. A point of the section is on one
    /// of the two, and on the band where this is above the tolerance.
    fn signed_rho(&self, p: Point3) -> f64 {
        let q = self.frame.to_local(p);
        let rho = real::hypot(q.x, q.y);
        let band = (real::hypot(rho - self.major, q.z) - self.minor).abs();
        let mirror = (real::hypot(rho + self.major, q.z) - self.minor).abs();
        if band <= mirror { rho } else { -rho }
    }

    /// Whether the samples lie on the band, off it, or on both sides of
    /// its edge: `None` for the last.
    fn side(&self, samples: &[Point3], tol: Tolerance) -> Option<bool> {
        let rho: Vec<f64> = samples.iter().map(|&p| self.signed_rho(p)).collect();
        let on = rho.iter().any(|&r| r > tol.linear);
        let off = rho.iter().any(|&r| r < -tol.linear);
        match (on, off) {
            (true, true) => None,
            (on, _) => Some(on),
        }
    }

    fn on_axis(&self, p: Point3, tol: Tolerance) -> bool {
        let q = self.frame.to_local(p);
        real::hypot(q.x, q.y) <= tol.linear
    }
}

/// `trace` cut down to what lies on the regular band of each torus among
/// `operands` that is not a ring: the branches and circles on it, the
/// points a kept branch ends at, and the isolated points on it. The
/// singular points a kept branch ends at on an axis — each a vertex of
/// the section there, where a branch arrives from the band — are one
/// point per axis point, and the branches ending there are pinned to it.
/// A trace with no such torus comes back as it is.
///
/// Refused: a branch on both sides of the band's edge, as a singular
/// point the tracer did not part ([`SectionFault::CrowdedSingularity`]),
/// and a circle on both ([`SectionFault::TubeCircle`]).
pub(super) fn on_band(
    trace: SectionTrace,
    operands: [&Surface; 2],
    tol: Tolerance,
) -> Result<SectionTrace, SectionFault> {
    let tori: Vec<BandTorus<'_>> = operands
        .iter()
        .filter_map(|s| BandTorus::of(s, tol))
        .collect();
    if tori.is_empty() {
        return Ok(trace);
    }
    // On the band of every such torus; `None` where on both sides.
    let kept = |samples: &[Point3]| {
        tori.iter().try_fold(true, |all, torus| {
            torus.side(samples, tol).map(|on| all && on)
        })
    };
    let mut branches = Vec::with_capacity(trace.branches().len());
    for branch in trace.branches() {
        let domain = branch.domain();
        let samples: Vec<Point3> = (0..BAND_SAMPLES)
            .map(|i| branch.point(domain.lerp((i as f64 + 0.5) / BAND_SAMPLES as f64)))
            .collect();
        match kept(&samples) {
            Some(true) => branches.push(branch),
            Some(false) => {}
            None => return Err(SectionFault::CrowdedSingularity),
        }
    }
    let mut circles = Vec::with_capacity(trace.circles().len());
    for circle in trace.circles() {
        let domain = circle.circle.domain();
        let samples: Vec<Point3> = (0..BAND_SAMPLES)
            .map(|i| (circle.circle).point(domain.lerp((i as f64 + 0.5) / BAND_SAMPLES as f64)))
            .collect();
        match kept(&samples) {
            Some(true) => circles.push(circle.clone()),
            Some(false) => {}
            None => return Err(SectionFault::TubeCircle),
        }
    }
    // The points a kept branch ends at, and the isolated ones on the band.
    let ended: Vec<usize> = (branches.iter())
        .filter_map(|b| b.ends())
        .flatten()
        .filter_map(|end| match end {
            BranchEnd::Singular(i) => Some(i),
            BranchEnd::Clipped => None,
        })
        .collect();
    let on_band = |p: &SectionPoint| {
        tori.iter()
            .all(|torus| torus.signed_rho(p.point) > tol.linear)
    };
    let mut points: Vec<SectionPoint> = Vec::new();
    // Old index to new, and whether the point is another's now.
    let mut index: BTreeMap<usize, (usize, bool)> = BTreeMap::new();
    for (i, p) in trace.points().iter().enumerate() {
        if !(ended.contains(&i) || (p.isolated && on_band(p))) {
            continue;
        }
        let at_axis = |q: &SectionPoint| tori.iter().any(|t| t.on_axis(q.point, tol));
        let same = (at_axis(p))
            .then(|| {
                (points.iter()).position(|q| at_axis(q) && (q.point - p.point).norm() <= tol.linear)
            })
            .flatten();
        match same {
            Some(j) => {
                index.insert(i, (j, true));
            }
            None => {
                index.insert(i, (points.len(), false));
                points.push(*p);
            }
        }
    }
    let branches = (branches.into_iter())
        .map(|b| {
            b.clone().renumbered(|i| {
                let (j, moved) = index.get(&i).copied().unwrap_or((i, false));
                (j, moved.then(|| points[j].point))
            })
        })
        .collect();
    Ok(SectionTrace::new(branches, points).with_circles(circles))
}
