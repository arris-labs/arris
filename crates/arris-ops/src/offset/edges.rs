//! The edges an offset recomputes: every edge at a moved vertex, on the
//! line its two planes meet in once moved, between its new ends.

use std::collections::BTreeMap;

use arris_geom::{Curve, GeomKind};
use arris_math::{Interval, Meter, Point3, Tolerance};
use arris_topo::{CurveId, EdgeId, Model, VertexId};

use super::Moves;
use crate::body_view::BodyView;
use crate::error::{Fault, OffsetReason, OpError, Reason};
use crate::rebuild::forward;

/// An edge's new geometry: its curve — its own when neither of its faces
/// moves, re-ranged — and its range from its start to its end.
#[derive(Debug, Clone, Copy)]
pub(super) struct NewEdge {
    pub(super) curve: CurveId,
    pub(super) range: Interval,
}

/// Every edge at a moved vertex, recomputed. Two planes meet in a line
/// whose direction their offsets keep, so each edge keeps its line's
/// direction: between two fixed faces the line itself, re-ranged to its
/// moved ends; where a face moved, the line through its new start along
/// that direction. An edge whose new ends meet or cross within its
/// tolerance is [`OffsetReason::Vanishes`]; a curve other than a line is
/// outside the planar core, [`OpError::Unsupported`].
pub(super) fn moved_edges(
    m: &mut Model,
    view: &BodyView,
    moves: &Moves,
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
        let Some((curve_id, _)) = edge.curve() else {
            return Err(OpError::Internal(Fault::Invariant {
                what: "a curve on an edge between two planes",
            }));
        };
        let curve = m.curve(curve_id)?.clone();
        let Curve::Line { origin, direction } = curve else {
            let face = uses.first().map_or(forward(e), |u| forward(u.face));
            return Err(OpError::Unsupported {
                a: (GeomKind::Curve(curve.kind()), forward(e)),
                b: (GeomKind::Curve(curve.kind()), face),
            });
        };
        let d = direction.into_inner();
        let length = (pe - ps).dot(&d);
        let tolerance = edge.tolerance().max(tol.linear);
        if length <= tolerance {
            return Err(OpError::Degenerate {
                entities: vec![forward(e)],
                reason: Reason::Offset(OffsetReason::Vanishes),
            });
        }
        let moved = uses.iter().any(|u| moves.surfaces.contains_key(&u.face));
        let new = if moved {
            let curve = m.add_curve(Curve::Line {
                origin: ps,
                direction,
            });
            NewEdge {
                curve,
                range: interval(0.0, length)?,
            }
        } else {
            let lo = (ps - origin).dot(&d);
            NewEdge {
                curve: curve_id,
                range: interval(lo, lo + length)?,
            }
        };
        out.insert(e, new);
    }
    Ok(out)
}

fn interval(lo: f64, hi: f64) -> Result<Interval, OpError> {
    Interval::new(lo, hi).map_err(|_| {
        OpError::Internal(Fault::Invariant {
            what: "a finite, increasing range",
        })
    })
}
