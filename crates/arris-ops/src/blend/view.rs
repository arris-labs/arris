//! The body as the blend reads it: effective face orientations, edge uses and the convexity of an edge.

use std::collections::{BTreeMap, BTreeSet};

use arris_math::{Point2, Tolerance, Vec3};
use arris_topo::{Body, Curve2Id, EdgeId, FaceId, Model, Orientation, VertexId};

use super::chain::tangent_normals;
use super::invariant;
use crate::error::{Fault, OpError};

/// One use of an edge by a loop of one of the body's faces, addressed.
#[derive(Debug, Clone, Copy)]
pub(super) struct UseAt {
    pub(super) face: FaceId,
    pub(super) loop_index: usize,
    pub(super) coedge_index: usize,
    /// The stored orientation of the coedge.
    pub(super) orientation: Orientation,
    pub(super) pcurve: Curve2Id,
}

/// The body read once: each face's effective orientation and the shell
/// it belongs to, every edge's uses and every vertex's edges, all in the
/// body's own order.
pub(super) struct View {
    pub(super) orientation: BTreeMap<FaceId, Orientation>,
    pub(super) shell_of: BTreeMap<FaceId, usize>,
    pub(super) faces: Vec<FaceId>,
    pub(super) uses: BTreeMap<EdgeId, Vec<UseAt>>,
    pub(super) vertex_edges: BTreeMap<VertexId, BTreeSet<EdgeId>>,
}

impl View {
    pub(super) fn of(m: &Model, body: Body) -> Result<View, OpError> {
        let mut view = View {
            orientation: BTreeMap::new(),
            shell_of: BTreeMap::new(),
            faces: Vec::new(),
            uses: BTreeMap::new(),
            vertex_edges: BTreeMap::new(),
        };
        for (s, shell) in m.shells(body)?.into_iter().enumerate() {
            for face_use in m.shell(shell.id)?.faces() {
                let face = face_use.oriented_by(shell.orientation);
                if view.orientation.insert(face.id, face.orientation).is_some() {
                    return Err(invariant("a face used once"));
                }
                view.shell_of.insert(face.id, s);
                view.faces.push(face.id);
                let entity = m.face(face.id)?;
                for (loop_index, l) in entity.loops().iter().enumerate() {
                    for (coedge_index, c) in l.coedges().iter().enumerate() {
                        view.uses.entry(c.edge()).or_default().push(UseAt {
                            face: face.id,
                            loop_index,
                            coedge_index,
                            orientation: c.orientation(),
                            pcurve: c.pcurve(),
                        });
                        let edge = m.edge(c.edge())?;
                        for v in [edge.start(), edge.end()] {
                            view.vertex_edges.entry(v).or_default().insert(c.edge());
                        }
                    }
                }
            }
        }
        Ok(view)
    }

    /// The face's outward normal at `uv`, the surface's composed with the
    /// body's use of the face.
    pub(super) fn outward(&self, m: &Model, face: FaceId, uv: Point2) -> Result<Vec3, OpError> {
        let surface = m.surface(m.face(face)?.surface())?;
        let n = surface
            .normal(uv.x, uv.y)
            .ok_or(OpError::Internal(Fault::NoNormal { face }))?;
        Ok(n.into_inner() * self.orientation[&face].sign())
    }
}

/// The tolerance a blend is built to beside the faces `f1` and `f2`: the
/// model's default, or the larger of the faces' own where that is wider.
pub(super) fn faces_tolerance(m: &Model, f1: FaceId, f2: FaceId) -> Result<f64, OpError> {
    Ok(m.precision()
        .default_tolerance
        .max(m.face(f1)?.tolerance())
        .max(m.face(f2)?.tolerance()))
}

/// Whether the two faces of `edge` are tangent at its parameter `t` for a
/// blend of `size`, by `tangent_normals` at their own tolerance; `None` for
/// an edge not used by exactly two faces.
pub(super) fn tangent_at(
    m: &Model,
    view: &View,
    edge: EdgeId,
    t: f64,
    size: f64,
    tol: Tolerance,
) -> Result<Option<bool>, OpError> {
    let Some(&[ua, ub]) = view.uses.get(&edge).map(Vec::as_slice) else {
        return Ok(None);
    };
    let n1 = view.outward(m, ua.face, m.curve2(ua.pcurve)?.point(t))?;
    let n2 = view.outward(m, ub.face, m.curve2(ub.pcurve)?.point(t))?;
    let tolerance = faces_tolerance(m, ua.face, ub.face)?;
    Ok(Some(tangent_normals(n1, n2, size, tolerance, tol)))
}

/// Whether `edge` is convex, read at its midpoint as a stripe or a ring
/// reads it: the direction into its first face against the second face's
/// outward normal. `None` for an edge with no curve or not used by exactly
/// two faces.
pub(super) fn convex_edge(m: &Model, view: &View, edge: EdgeId) -> Result<Option<bool>, OpError> {
    let Some((curve, range)) = m.edge(edge)?.curve() else {
        return Ok(None);
    };
    let Some(&[ua, ub]) = view.uses.get(&edge).map(Vec::as_slice) else {
        return Ok(None);
    };
    let mid = range.midpoint();
    let n1 = view.outward(m, ua.face, m.curve2(ua.pcurve)?.point(mid))?;
    let n2 = view.outward(m, ub.face, m.curve2(ub.pcurve)?.point(mid))?;
    let t1 =
        m.curve(curve)?.eval(mid).d1 * view.orientation[&ua.face].compose(ua.orientation).sign();
    Ok(Some(n1.cross(&t1).dot(&n2) < 0.0))
}
