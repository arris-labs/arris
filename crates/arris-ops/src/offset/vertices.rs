//! The vertices an offset moves, each the meeting of the surfaces of the
//! faces around it once the moved ones are offset.

use std::collections::{BTreeMap, BTreeSet};

use arris_geom::{Curve, GeomKind, Surface};
use arris_math::{Meter, Point3, Tolerance, Vec3};
use arris_topo::{EdgeId, FaceId, Model, VertexId};

use super::Moves;
use super::chain::constraints_of;
use super::meet::{Constraint, meet_near, seam_plane};
use crate::body_view::BodyView;
use crate::error::{OffsetReason, OpError, Reason};
use crate::rebuild::forward;

/// A plane as `n · x = c`, `n` unit.
#[derive(Debug, Clone, Copy)]
pub(super) struct PlaneEq {
    pub(super) normal: Vec3,
    pub(super) offset: f64,
}

/// The plane `surface` is, as `n · x = c`, or `None` for another kind.
fn plane_eq(surface: &Surface) -> Option<PlaneEq> {
    match surface {
        Surface::Plane { frame } => {
            let normal = frame.z().into_inner();
            Some(PlaneEq {
                normal,
                offset: normal.dot(&frame.origin().coords),
            })
        }
        Surface::Cylinder { .. }
        | Surface::EllipticCylinder { .. }
        | Surface::Cone { .. }
        | Surface::Sphere { .. }
        | Surface::Torus { .. }
        | Surface::Nurbs(_) => None,
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
/// old one on every surface around it once the move is made (three
/// independent planes meet in one point; a vertex between two, on a
/// straight run, slides square to their line; a vertex on a seam also
/// lies on the plane the seam is in; a vertex on a carried tangent edge
/// lies on its curve). A vertex whose surfaces do not all pass through that
/// point within its tolerance is [`OffsetReason::VertexSplits`], or
/// [`OffsetReason::Gap`] naming the edge where a dragged face and one that
/// stays no longer meet there; a face on an elliptic cylinder or a
/// free-form surface, or a surface with no distance there, is
/// [`OpError::Unsupported`] against the moved face that reaches it.
pub(super) fn moved_vertices(
    m: &Model,
    view: &BodyView,
    moves: &Moves,
    carried: &BTreeMap<EdgeId, Curve>,
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
        // A carried edge's curve first: the vertex is on it exactly, where
        // its two tangent faces alone pin no direction across it.
        let mut constraints = Vec::with_capacity(faces.len() + 2);
        for e in view.vertex_edges.get(&v).into_iter().flatten() {
            if let Some(curve) = carried.get(e) {
                let [a, b] = constraints_of(curve, beside).ok_or(OpError::Internal(
                    crate::error::Fault::Invariant {
                        what: "a carried edge on a line or a circle",
                    },
                ))?;
                constraints.push(a);
                constraints.push(b);
            }
        }
        for &f in &faces {
            let surface = moves.surface_of(m, f)?;
            if let Surface::EllipticCylinder { .. } | Surface::Nurbs(_) = surface {
                let near = m.surface(m.face(beside)?.surface())?;
                let kind = GeomKind::Surface(surface.kind());
                return Err(OpError::Unsupported {
                    a: (kind, forward(f)),
                    b: (GeomKind::Surface(near.kind()), forward(beside)),
                });
            }
            constraints.push(Constraint { surface, face: f });
        }
        let vertex = m.vertex(v)?;
        let old = vertex.point();
        let tolerance = vertex.tolerance().max(m.precision().default_tolerance);
        let planes: Option<Vec<PlaneEq>> =
            constraints.iter().map(|c| plane_eq(&c.surface)).collect();
        let point = match planes {
            Some(planes) => nearest_on(old, &planes, tol).filter(|p| {
                planes
                    .iter()
                    .all(|q| (q.normal.dot(&p.coords) - q.offset).abs() <= tolerance)
            }),
            None => {
                seams_at(m, view, v, old, tol, &mut constraints)?;
                meet_near(old, &constraints, tol, tolerance)?
            }
        };
        let Some(point) = point else {
            return Err(split_or_gap(m, view, moves, v, old, tol, tolerance)?);
        };
        points.insert(v, point);
    }
    Ok(points)
}

/// Why the faces at `v` no longer meet: a [`OffsetReason::Gap`] naming
/// the edge at `v` between a dragged face and one that stays where those
/// two alone no longer meet near the vertex, [`OffsetReason::VertexSplits`]
/// naming the vertex otherwise.
fn split_or_gap(
    m: &Model,
    view: &BodyView,
    moves: &Moves,
    v: VertexId,
    old: Point3,
    tol: Tolerance,
    tolerance: f64,
) -> Result<OpError, OpError> {
    for e in view.vertex_edges.get(&v).into_iter().flatten() {
        let Some(&[a, b]) = view.uses.get(e).map(Vec::as_slice) else {
            continue;
        };
        if !moves.gap_between(a.face, b.face) {
            continue;
        }
        let on = |face: FaceId| -> Result<Constraint, OpError> {
            Ok(Constraint {
                surface: moves.surface_of(m, face)?,
                face,
            })
        };
        if meet_near(old, &[on(a.face)?, on(b.face)?], tol, tolerance)?.is_none() {
            return Ok(OpError::Degenerate {
                entities: vec![forward(*e)],
                reason: Reason::Offset(OffsetReason::Gap),
            });
        }
    }
    Ok(OpError::Degenerate {
        entities: vec![forward(v)],
        reason: Reason::Offset(OffsetReason::VertexSplits),
    })
}

/// The plane of every seam edge at `v`, added to `constraints`: a face
/// that meets itself along the edge ties the vertex to the seam's plane,
/// which no face's own surface says. An edge with no curve, or one in no
/// such plane, is [`OpError::Unsupported`] on its face.
fn seams_at(
    m: &Model,
    view: &BodyView,
    v: VertexId,
    old: Point3,
    tol: Tolerance,
    constraints: &mut Vec<Constraint>,
) -> Result<(), OpError> {
    for e in view.vertex_edges.get(&v).into_iter().flatten() {
        let uses = view.uses.get(e).map(Vec::as_slice).unwrap_or_default();
        let [a, b] = uses else { continue };
        if a.face != b.face {
            continue;
        }
        let surface = m.surface(m.face(a.face)?.surface())?;
        let kind = GeomKind::Surface(surface.kind());
        let unsupported = || OpError::Unsupported {
            a: (kind, forward(a.face)),
            b: (kind, forward(*e)),
        };
        let (curve, _) = m.edge(*e)?.curve().ok_or_else(unsupported)?;
        let plane = seam_plane(surface, m.curve(curve)?, old, tol).ok_or_else(unsupported)?;
        constraints.push(Constraint {
            surface: plane,
            face: a.face,
        });
    }
    Ok(())
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
