//! The faces an offset rewrites: every face with a recomputed edge, its
//! loops checked against turning inside out, each use of a new edge
//! given its pcurve on the face's surface, moved or kept.

use std::collections::BTreeMap;

use arris_geom::{Surface, pcurve_on};
use arris_math::{Frame, Meter, Point2, Point3, Tolerance};
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
                geometry: EdgeGeometry::Curve {
                    curve: new.curve,
                    range: new.range,
                },
                start: key(edge.start()),
                end: key(edge.end()),
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
        let frame = match &old_surface {
            Surface::Plane { frame } => *frame,
            _ => {
                return Err(OpError::Internal(Fault::Invariant {
                    what: "a plane under a rewritten face",
                }));
            }
        };
        let mut loops = Vec::with_capacity(entity.loops().len());
        for l in entity.loops() {
            let mut before = Vec::with_capacity(l.coedges().len());
            let mut after = Vec::with_capacity(l.coedges().len());
            let mut uses = Vec::with_capacity(l.coedges().len());
            for c in l.coedges() {
                let edge = m.edge(c.edge())?;
                let first = match c.orientation() {
                    Orientation::Forward => edge.start(),
                    Orientation::Reversed => edge.end(),
                };
                let old = m.vertex(first)?.point();
                before.push(old);
                after.push(points.get(&first).copied().unwrap_or(old));
                let use_ = match edges.get(&c.edge()) {
                    Some(new) => {
                        let curve = m.curve(new.curve)?.clone();
                        let pcurve =
                            pcurve_on(&curve, new.range, &surface, tol, meter).map_err(fault_of)?;
                        StoredUse {
                            edge: ekey[&c.edge()],
                            orientation: c.orientation(),
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
            let (old_area, _) = signed_area(&frame, &before);
            let (new_area, perimeter) = signed_area(&frame, &after);
            if new_area * old_area.signum() <= tol.linear * perimeter {
                return Err(OpError::Degenerate {
                    entities: vec![forward(f)],
                    reason: Reason::Offset(OffsetReason::Vanishes),
                });
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
