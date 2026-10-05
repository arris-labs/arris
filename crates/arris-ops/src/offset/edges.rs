//! The edges an offset recomputes: every edge at a moved vertex, between
//! its new ends — on the line two planes meet in, or on the branch of the
//! two new surfaces' section nearest the old edge, or on its own curve
//! where neither of its faces moves.

use std::collections::BTreeMap;

use arris_geom::{Curve, GeomKind, MeetKind, Surface, SurfaceIntersection, intersect_surfaces};
use arris_math::{Aabb, Interval, Meter, Point3, Tolerance, shift_into_range};
use arris_topo::{CurveId, EdgeId, Model, VertexId};

use super::Moves;
use super::meet::seam_plane;
use crate::body_view::{BodyView, UseAt};
use crate::error::{Fault, OffsetReason, OpError, Reason, fault_of};
use crate::rebuild::forward;

/// An edge's new geometry: its curve — its own when neither of its faces
/// moves, re-ranged — its range, and its ends in the order the curve runs
/// from one to the other.
#[derive(Debug, Clone, Copy)]
pub(super) struct NewEdge {
    /// `None` for a degenerate edge at a surface's pole, which has no
    /// curve and keeps its range and its pcurves.
    pub(super) curve: Option<CurveId>,
    pub(super) range: Interval,
    /// The vertex at the range's start and the one at its end.
    pub(super) start: VertexId,
    pub(super) end: VertexId,
    /// Whether the new curve runs against the old edge's direction: the
    /// ends are then swapped, and every use of the edge walks the other
    /// way.
    pub(super) reversed: bool,
    /// A parameter of the new curve inside its range, where the old
    /// edge's midpoint is nearest it: a use's new pcurve is placed to
    /// meet the old one there.
    pub(super) mid: f64,
}

/// Every edge at a moved vertex, recomputed. Two planes meet in a line
/// whose direction their offsets keep, so a line between two planes keeps
/// its direction: between two fixed faces the line itself, re-ranged to
/// its moved ends; where a face moved, the line through its new start
/// along that direction. Any other edge lies on its own curve between two
/// fixed faces, re-ranged; on its `carried` curve between two tangent
/// moved faces; and otherwise on the branch of its two new surfaces'
/// section the intersector gives that passes through both new ends — a
/// seam edge's second surface the plane it lies in. An edge whose new ends
/// meet or cross within its tolerance, or that no branch passes through,
/// is [`OffsetReason::Vanishes`] — [`OffsetReason::Gap`] where it lies
/// between a dragged face and one that stays; a section that is only
/// tangent is outside this release, [`OpError::Unsupported`].
pub(super) fn moved_edges(
    m: &mut Model,
    view: &BodyView,
    moves: &Moves,
    carried: &BTreeMap<EdgeId, Curve>,
    points: &BTreeMap<VertexId, Point3>,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<BTreeMap<EdgeId, NewEdge>, OpError> {
    let mut out = BTreeMap::new();
    for (&e, uses) in &view.uses {
        let edge = *m.edge(e)?;
        let (start, end) = (edge.start(), edge.end());
        if !points.contains_key(&start) && !points.contains_key(&end) {
            continue;
        }
        meter.tick()?;
        let at = |v: VertexId| -> Result<Point3, OpError> {
            match points.get(&v) {
                Some(&p) => Ok(p),
                None => Ok(m.vertex(v)?.point()),
            }
        };
        let (ps, pe) = (at(start)?, at(end)?);
        let Some((curve_id, old_range)) = edge.curve() else {
            out.insert(
                e,
                NewEdge {
                    curve: None,
                    range: edge.range(),
                    start,
                    end,
                    reversed: false,
                    mid: edge.range().midpoint(),
                },
            );
            continue;
        };
        let old = m.curve(curve_id)?.clone();
        let tolerance = edge
            .tolerance()
            .max(tol.linear)
            .max(m.vertex(start)?.tolerance())
            .max(m.vertex(end)?.tolerance());
        let moved = uses.iter().any(|u| moves.surfaces.contains_key(&u.face));
        let new = match (&old, moved) {
            (Curve::Line { origin, direction }, false) => {
                let d = direction.into_inner();
                let length = (pe - ps).dot(&d);
                if length <= tolerance {
                    return Err(vanishes(e));
                }
                let lo = (ps - origin).dot(&d);
                NewEdge {
                    curve: Some(curve_id),
                    range: interval(lo, lo + length)?,
                    start,
                    end,
                    reversed: false,
                    mid: lo + length / 2.0,
                }
            }
            (Curve::Line { direction, .. }, true) if both_planes(m, moves, uses)? => {
                let d = direction.into_inner();
                let length = (pe - ps).dot(&d);
                if length <= tolerance {
                    return Err(vanishes(e));
                }
                let curve = m.add_curve(Curve::Line {
                    origin: ps,
                    direction: *direction,
                });
                NewEdge {
                    curve: Some(curve),
                    range: interval(0.0, length)?,
                    start,
                    end,
                    reversed: false,
                    mid: length / 2.0,
                }
            }
            _ => {
                let candidates = match carried.get(&e) {
                    Some(curve) => vec![curve.clone()],
                    None if moved => {
                        section(m, moves, e, uses, [ps, pe], &old, old_range, tol, meter)?
                    }
                    None => vec![old.clone()],
                };
                // No branch through the ends where a dragged face meets one
                // that stays is the chain pulled off its neighbour.
                let missing = match uses.as_slice() {
                    [a, b] if moves.gap_between(a.face, b.face) => OffsetReason::Gap,
                    _ => OffsetReason::Vanishes,
                };
                let (curve, reversed, range, mid) = along(
                    &old,
                    old_range,
                    &candidates,
                    [ps, pe],
                    start == end,
                    tolerance,
                    e,
                    missing,
                )?;
                let (start, end) = if reversed && start != end {
                    (end, start)
                } else {
                    (start, end)
                };
                let curve = if moved { m.add_curve(curve) } else { curve_id };
                NewEdge {
                    curve: Some(curve),
                    range,
                    start,
                    end,
                    reversed,
                    mid,
                }
            }
        };
        out.insert(e, new);
    }
    Ok(out)
}

fn vanishes(e: EdgeId) -> OpError {
    OpError::Degenerate {
        entities: vec![forward(e)],
        reason: Reason::Offset(OffsetReason::Vanishes),
    }
}

/// Whether every face of the edge lies on a plane once the move is made.
fn both_planes(m: &Model, moves: &Moves, uses: &[UseAt]) -> Result<bool, OpError> {
    for u in uses {
        if !matches!(moves.surface_of(m, u.face)?, Surface::Plane { .. }) {
            return Ok(false);
        }
    }
    Ok(true)
}

/// The curves where the edge's two new surfaces cross, in the
/// intersector's order: the faces' own, or the one face's and its seam's
/// plane.
#[allow(clippy::too_many_arguments)]
fn section(
    m: &Model,
    moves: &Moves,
    e: EdgeId,
    uses: &[UseAt],
    [ps, pe]: [Point3; 2],
    old: &Curve,
    old_range: Interval,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<Vec<Curve>, OpError> {
    let [a, b] = uses else {
        return Err(OpError::Internal(Fault::Invariant {
            what: "an edge used by two faces",
        }));
    };
    let first = moves.surface_of(m, a.face)?;
    let second = if a.face == b.face {
        let surface = m.surface(m.face(a.face)?.surface())?;
        let kind = GeomKind::Surface(surface.kind());
        seam_plane(surface, old, old.point(old_range.midpoint()), tol).ok_or(
            OpError::Unsupported {
                a: (kind, forward(a.face)),
                b: (kind, forward(e)),
            },
        )?
    } else {
        moves.surface_of(m, b.face)?
    };
    for (surface, face) in [(&first, a.face), (&second, b.face)] {
        if let Surface::EllipticCylinder { .. } | Surface::Nurbs(_) = surface {
            let kind = GeomKind::Surface(surface.kind());
            return Err(OpError::Unsupported {
                a: (kind, forward(face)),
                b: (kind, forward(e)),
            });
        }
    }
    let reach = (pe - ps).norm() + 2.0 * moves.distance.abs();
    let within = Aabb::of_point(ps)
        .union(Aabb::of_point(pe))
        .union(old.bounds(old_range).unwrap_or(Aabb::of_point(ps)))
        .inflated(reach);
    let cut = intersect_surfaces(&first, &second, &within, tol, meter).map_err(fault_of)?;
    let (mut crossing, mut touching) = (Vec::new(), 0);
    if let SurfaceIntersection::Meets { curves, .. } = cut {
        for c in curves {
            match c.kind {
                MeetKind::Crossing => crossing.push(c.curve),
                MeetKind::Touch => touching += 1,
            }
        }
    }
    if crossing.is_empty() && touching > 0 {
        let kind = GeomKind::Surface(first.kind());
        return Err(OpError::Unsupported {
            a: (kind, forward(a.face)),
            b: (GeomKind::Surface(second.kind()), forward(b.face)),
        });
    }
    Ok(crossing)
}

/// The candidate that passes within `tolerance` of both new ends — the
/// nearest if several — with the range it runs between them, whether it
/// runs against the old edge, and a parameter near the old edge's
/// midpoint. The range starts at the first end the curve reaches going
/// the old edge's way and, on a periodic curve, goes forward a whole turn
/// for a closed edge and to the other end otherwise. `missing` where no
/// candidate passes through both ends, [`OffsetReason::Vanishes`] where
/// the ends meet.
#[allow(clippy::too_many_arguments)]
fn along(
    old: &Curve,
    old_range: Interval,
    candidates: &[Curve],
    [ps, pe]: [Point3; 2],
    closed: bool,
    tolerance: f64,
    e: EdgeId,
    missing: OffsetReason,
) -> Result<(Curve, bool, Interval, f64), OpError> {
    let reach = |c: &Curve| -> Option<f64> {
        let (a, b) = (c.project(ps).ok()?, c.project(pe).ok()?);
        Some(a.distance.max(b.distance))
    };
    let best = candidates
        .iter()
        .filter_map(|c| reach(c).map(|r| (r, c)))
        .filter(|&(r, _)| r <= tolerance)
        .min_by(|x, y| x.0.total_cmp(&y.0))
        .map(|(_, c)| c)
        .ok_or(OpError::Degenerate {
            entities: vec![forward(e)],
            reason: Reason::Offset(missing),
        })?;
    let old_mid = old_range.midpoint();
    let near = best.project(old.point(old_mid)).map_err(fault_of)?;
    let reversed = best.eval(near.t).d1.dot(&old.eval(old_mid).d1) < 0.0;
    let (first, last) = if reversed { (pe, ps) } else { (ps, pe) };
    let t_a = best.project(first).map_err(fault_of)?.t;
    let t_b = best.project(last).map_err(fault_of)?.t;
    let range = match best.period() {
        Some(period) if closed => interval(t_a, whole_turn(t_a, period))?,
        Some(period) => {
            let span = (t_b - t_a).rem_euclid(period);
            if span <= tolerance || period - span <= tolerance {
                return Err(vanishes(e));
            }
            interval(t_a, t_a + span)?
        }
        None => {
            if t_b - t_a <= tolerance {
                return Err(vanishes(e));
            }
            interval(t_a, t_b)?
        }
    };
    let mid = shift_into_range(range, near.t, best.period()).unwrap_or(range.midpoint());
    Ok((best.clone(), reversed, range, mid))
}

/// `lo + period` rounded down until it is no more than a period from `lo`:
/// the checker holds a range to at most one period, and far from zero the
/// sum rounds past it.
fn whole_turn(lo: f64, period: f64) -> f64 {
    let mut hi = lo + period;
    while hi - lo > period {
        // `f64::next_down` is stable from Rust 1.86 and the workspace
        // supports 1.85.
        let bits = hi.to_bits();
        hi = f64::from_bits(if hi > 0.0 { bits - 1 } else { bits + 1 });
    }
    hi
}

fn interval(lo: f64, hi: f64) -> Result<Interval, OpError> {
    Interval::new(lo, hi).map_err(|_| {
        OpError::Internal(Fault::Invariant {
            what: "a finite, increasing range",
        })
    })
}
