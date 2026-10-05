//! The vertices an offset moves, each the meeting of the planes of the
//! faces around it once the moved ones are offset.

use std::collections::{BTreeMap, BTreeSet};

use arris_geom::{GeomKind, Surface};
use arris_math::{Meter, Point3, Tolerance, Vec3};
use arris_topo::{FaceId, Model, VertexId};

use super::Moves;
use crate::body_view::BodyView;
use crate::error::{OffsetReason, OpError, Reason};
use crate::rebuild::forward;

/// A plane as `n · x = c`, `n` unit.
#[derive(Debug, Clone, Copy)]
pub(super) struct PlaneEq {
    pub(super) normal: Vec3,
    pub(super) offset: f64,
}

/// The plane `face` lies on once the offset is made — its offset when it
/// moves, its own otherwise. A face that is not a plane is
/// [`OpError::Unsupported`] against `beside`, the moved face whose move
/// reaches it: the planar core holds planes alone.
pub(super) fn plane_of(
    m: &Model,
    moves: &Moves,
    face: FaceId,
    beside: FaceId,
) -> Result<PlaneEq, OpError> {
    let surface = match moves.surfaces.get(&face) {
        Some(s) => s,
        None => m.surface(m.face(face)?.surface())?,
    };
    match surface {
        Surface::Plane { frame } => {
            let normal = frame.z().into_inner();
            Ok(PlaneEq {
                normal,
                offset: normal.dot(&frame.origin().coords),
            })
        }
        other => {
            let near = m.surface(m.face(beside)?.surface())?;
            Err(OpError::Unsupported {
                a: (GeomKind::Surface(other.kind()), forward(face)),
                b: (GeomKind::Surface(near.kind()), forward(beside)),
            })
        }
    }
}

/// The faces around `v`, in the body's order of its edges' uses.
pub(super) fn faces_at(view: &BodyView, v: VertexId) -> Vec<FaceId> {
    let mut faces: Vec<FaceId> = Vec::new();
    for e in view.vertex_edges.get(&v).into_iter().flatten() {
        for u in view.uses.get(e).into_iter().flatten() {
            if !faces.contains(&u.face) {
                faces.push(u.face);
            }
        }
    }
    faces
}

/// Every vertex of a moved face at its new point: the point nearest its
/// old one on every plane around it (three independent planes meet in
/// one point; a vertex between two, on a straight run, slides square to
/// their line). A vertex whose planes do not all pass through that point
/// within its tolerance is [`OffsetReason::VertexSplits`].
pub(super) fn moved_vertices(
    m: &Model,
    view: &BodyView,
    moves: &Moves,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<BTreeMap<VertexId, Point3>, OpError> {
    let mut reached: BTreeSet<VertexId> = BTreeSet::new();
    for &f in moves.surfaces.keys() {
        for l in m.face(f)?.loops() {
            for c in l.coedges() {
                let edge = m.edge(c.edge())?;
                reached.insert(edge.start());
                reached.insert(edge.end());
            }
        }
    }
    let mut points = BTreeMap::new();
    for v in reached {
        meter.tick()?;
        let faces = faces_at(view, v);
        let beside = faces
            .iter()
            .copied()
            .find(|f| moves.surfaces.contains_key(f))
            .ok_or(OpError::Internal(crate::error::Fault::Invariant {
                what: "a moved face at a moved vertex",
            }))?;
        let planes = faces
            .iter()
            .map(|&f| plane_of(m, moves, f, beside))
            .collect::<Result<Vec<_>, _>>()?;
        let vertex = m.vertex(v)?;
        let tolerance = vertex.tolerance().max(m.precision().default_tolerance);
        let point = nearest_on(vertex.point(), &planes, tol)
            .filter(|p| {
                planes
                    .iter()
                    .all(|q| (q.normal.dot(&p.coords) - q.offset).abs() <= tolerance)
            })
            .ok_or_else(|| OpError::Degenerate {
                entities: vec![forward(v)],
                reason: Reason::Offset(OffsetReason::VertexSplits),
            })?;
        points.insert(v, point);
    }
    Ok(points)
}

/// The point nearest `old` on the first independent planes of `planes`
/// — at most three, their normals more than `tol.angular` from the span
/// of those before — or `None` where none is.
fn nearest_on(old: Point3, planes: &[PlaneEq], tol: Tolerance) -> Option<Point3> {
    let mut basis: Vec<Vec3> = Vec::new();
    let mut kept: Vec<PlaneEq> = Vec::new();
    for q in planes {
        let mut r = q.normal;
        for b in &basis {
            r -= b * b.dot(&r);
        }
        let sine = r.norm();
        if sine > tol.angular {
            basis.push(r / sine);
            kept.push(*q);
        }
        if kept.len() == 3 {
            break;
        }
    }
    // The residual of each kept plane at `old`: the move is the least
    // one that clears all of them, a combination of their normals.
    let g = |q: &PlaneEq| q.offset - q.normal.dot(&old.coords);
    match kept.as_slice() {
        [a] => Some(old + a.normal * g(a)),
        [a, b] => {
            let k = a.normal.dot(&b.normal);
            let det = 1.0 - k * k;
            let (ga, gb) = (g(a), g(b));
            let x = (ga - k * gb) / det;
            let y = (gb - k * ga) / det;
            Some(old + a.normal * x + b.normal * y)
        }
        [a, b, c] => {
            let det = a.normal.dot(&b.normal.cross(&c.normal));
            let p = (b.normal.cross(&c.normal) * a.offset
                + c.normal.cross(&a.normal) * b.offset
                + a.normal.cross(&b.normal) * c.offset)
                / det;
            Some(Point3::from(p))
        }
        _ => None,
    }
}
