//! The faces an offset rewrites: every face with a recomputed edge, its
//! loops checked against turning inside out, each use of a new edge
//! given its pcurve on the face's surface, moved or kept.

use std::collections::BTreeMap;

use arris_geom::{Curve, Surface, pcurve_on};
use arris_math::{Frame, Meter, Point2, Point3, Tolerance, Vec2, shift_nearest_uv};
use arris_topo::builder::EdgeSpec;
use arris_topo::builder::{EdgeKey, VertexKey, VertexSpec};
use arris_topo::entity::EdgeGeometry;
use arris_topo::{EdgeId, FaceId, Model, Orientation, SurfaceId, VertexId};

use super::Moves;
use super::edges::NewEdge;
use crate::error::{Fault, OffsetReason, OpError, Reason, fault_of};
use crate::rebuild::{Rewrite, StoredUse, forward};

/// The rewrite of the body with the new vertices, edges and surfaces: a
/// face with a recomputed edge gets new loops — the same uses, a new
/// edge's pcurve on its surface — and a moved face its offset surface.
/// A loop whose polygon of vertices turns the other way, or to nothing,
/// once moved is [`OffsetReason::Vanishes`] naming its face.
pub(super) fn rewrite_of(
    m: &mut Model,
    faces: &[FaceId],
    moves: &Moves,
    points: &BTreeMap<VertexId, Point3>,
    edges: &BTreeMap<EdgeId, NewEdge>,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<Rewrite, OpError> {
    let mut rw = Rewrite::default();
    let mut vkey: BTreeMap<VertexId, VertexKey> = BTreeMap::new();
    for (&v, &point) in points {
        let tolerance = m.vertex(v)?.tolerance();
        vkey.insert(v, VertexKey::New(rw.vertices.len()));
        rw.vertex_parents.insert(rw.vertices.len(), v);
        rw.vertices.push(VertexSpec::New { point, tolerance });
    }
    let key = |v: VertexId| vkey.get(&v).copied().unwrap_or(VertexKey::Kept(v));
    let mut ekey: BTreeMap<EdgeId, EdgeKey> = BTreeMap::new();
    for (&e, new) in edges {
        let edge = m.edge(e)?;
        ekey.insert(e, EdgeKey::New(rw.edges.len()));
        rw.edges.push((
            EdgeSpec::New {
                geometry: match new.curve {
                    Some(curve) => EdgeGeometry::Curve {
                        curve,
                        range: new.range,
                    },
                    None => EdgeGeometry::Degenerate { range: new.range },
                },
                start: key(new.start),
                end: key(new.end),
                tolerance: edge.tolerance(),
            },
            Some(e),
        ));
    }
    let surface_ids: BTreeMap<FaceId, SurfaceId> = moves
        .surfaces
        .iter()
        .map(|(&f, s)| (f, m.add_surface(s.clone())))
        .collect();
    for &f in faces {
        let entity = m.face(f)?.clone();
        let touched = entity
            .loops()
            .iter()
            .flat_map(|l| l.coedges())
            .any(|c| edges.contains_key(&c.edge()));
        if !touched {
            continue;
        }
        meter.tick()?;
        let old_surface = m.surface(entity.surface())?.clone();
        let surface = moves.surfaces.get(&f).unwrap_or(&old_surface).clone();
        let mut loops = Vec::with_capacity(entity.loops().len());
        for l in entity.loops() {
            let mut before = Vec::with_capacity(l.coedges().len());
            let mut after = Vec::with_capacity(l.coedges().len());
            let mut uses = Vec::with_capacity(l.coedges().len());
            let mut straight = true;
            for c in l.coedges() {
                let edge = m.edge(c.edge())?;
                let shape = match edges.get(&c.edge()) {
                    Some(new) => new.curve,
                    None => edge.curve().map(|(id, _)| id),
                };
                straight &= match shape {
                    Some(id) => matches!(m.curve(id)?, Curve::Line { .. }),
                    None => false,
                };
                let first = match c.orientation() {
                    Orientation::Forward => edge.start(),
                    Orientation::Reversed => edge.end(),
                };
                let old = m.vertex(first)?.point();
                before.push(old);
                after.push(points.get(&first).copied().unwrap_or(old));
                let use_ = match edges.get(&c.edge()) {
                    // A degenerate edge stays at its pole, its pcurve the
                    // same line across it.
                    Some(new) if new.curve.is_none() => StoredUse {
                        edge: ekey[&c.edge()],
                        orientation: c.orientation(),
                        pcurve: c.pcurve(),
                    },
                    Some(new) => {
                        let curve_id = new.curve.ok_or(OpError::Internal(Fault::Invariant {
                            what: "a curve on a recomputed edge",
                        }))?;
                        let curve = m.curve(curve_id)?.clone();
                        let pcurve =
                            pcurve_on(&curve, new.range, &surface, tol, meter).map_err(fault_of)?;
                        // The new pcurve beside the old one, on the same
                        // sheet of a periodic surface: shifted by whole
                        // periods to the old use's `(u, v)` at the edge's
                        // midpoint, which the offset keeps to the period.
                        let old_mid = match m.edge(c.edge())?.curve() {
                            Some((_, range)) => range.midpoint(),
                            None => new.mid,
                        };
                        let target = m.curve2(c.pcurve())?.point(old_mid);
                        let by = shift_nearest_uv(target - pcurve.point(new.mid), surface.period());
                        let pcurve = if by == Vec2::zeros() {
                            pcurve
                        } else {
                            pcurve.translated(by)
                        };
                        let orientation = if new.reversed {
                            c.orientation().flipped()
                        } else {
                            c.orientation()
                        };
                        StoredUse {
                            edge: ekey[&c.edge()],
                            orientation,
                            pcurve: m.add_curve2(pcurve),
                        }
                    }
                    None => StoredUse {
                        edge: EdgeKey::Kept(c.edge()),
                        orientation: c.orientation(),
                        pcurve: c.pcurve(),
                    },
                };
                uses.push(use_);
            }
            // A polygon of straight edges on a plane turns inside out or to
            // nothing when its face vanishes; a curved loop is left to the
            // checker.
            if let (Surface::Plane { frame }, true) = (&old_surface, straight && after.len() >= 3) {
                let (old_area, _) = signed_area(frame, &before);
                let (new_area, perimeter) = signed_area(frame, &after);
                if new_area * old_area.signum() <= tol.linear * perimeter {
                    return Err(OpError::Degenerate {
                        entities: vec![forward(f)],
                        reason: Reason::Offset(OffsetReason::Vanishes),
                    });
                }
            }
            loops.push(uses);
        }
        rw.faces.insert(f, loops);
        if let Some(&id) = surface_ids.get(&f) {
            rw.surfaces.insert(f, id);
        }
    }
    Ok(rw)
}

/// The signed area of the polygon `points` in `frame`'s (x, y), and its
/// perimeter.
fn signed_area(frame: &Frame, points: &[Point3]) -> (f64, f64) {
    let uv: Vec<Point2> = points
        .iter()
        .map(|&p| {
            let q = frame.to_local(p);
            Point2::new(q.x, q.y)
        })
        .collect();
    let mut area = 0.0;
    let mut perimeter = 0.0;
    for (i, a) in uv.iter().enumerate() {
        let b = uv[(i + 1) % uv.len()];
        area += a.x * b.y - b.x * a.y;
        perimeter += (b - a).norm();
    }
    (area / 2.0, perimeter)
}
