use std::collections::BTreeMap;

use super::validate::uses_allowed;
use super::{BuildError, Builder, Built, EdgeRef, FaceRef, Position, VertexRef, effective_uses};
use crate::entity::{self, BodyKind, Coedge, EdgeGeometry, Loop};
use crate::handle::{self, Body};
use crate::id::ShellId;
use crate::model::Model;
use crate::orientation::Orientation;

impl Builder {
    /// Freezes the body into `model` as a `Solid` or a `Sheet`: every live
    /// vertex, edge and face appended in slot order — bar the slots
    /// [`Builder::assemble`] marked `Keep` and no operator has touched,
    /// which keep their arena id and append nothing — then one shell per
    /// shell index the faces carry, in index order and each over its faces
    /// in slot order, and the body over the shells, inside a transaction.
    /// A loop of a `Reversed` face is stored backwards with every use
    /// flipped, so every stored loop is counter-clockwise about its
    /// surface's normal. Returns the body, its shells and the slot → id
    /// maps.
    ///
    /// A `Solid`'s edges are used exactly twice, a `Sheet`'s once (its
    /// boundary) or twice — the open shells [`Builder::assemble_sheet`]
    /// proves (`docs/DATA-MODEL.md` §Invariants, S2).
    ///
    /// Errors, each leaving the model untouched: [`BuildError::Kind`] for
    /// a `Wire` or `General` body, which no operation returns;
    /// [`BuildError::Empty`]; an [`BuildError::EmptyLoop`]; a
    /// [`BuildError::MissingPcurve`]; an edge used a number of times its
    /// kind does not allow, or a degenerate one not exactly once
    /// ([`BuildError::EdgeUses`]), or used twice the same way
    /// ([`BuildError::SameDirection`]); a curve, surface, pcurve or kept
    /// entity id that does not resolve ([`BuildError::NotFound`]).
    pub fn finish(self, model: &mut Model, kind: BodyKind) -> Result<Built, BuildError> {
        let open = match kind {
            BodyKind::Solid => false,
            BodyKind::Sheet => true,
            BodyKind::Wire | BodyKind::General => {
                return Err(BuildError::Kind(kind));
            }
        };
        if self.faces.len() == 0 {
            return Err(BuildError::Empty);
        }
        let mut uses: BTreeMap<EdgeRef, Vec<Orientation>> = BTreeMap::new();
        for (fi, face) in self.faces.iter() {
            model.surface(face.surface)?;
            if let Some(id) = face.kept {
                model.face(id)?;
            }
            for (li, lp) in face.loops.iter().enumerate() {
                if lp.uses.is_empty() {
                    return Err(BuildError::EmptyLoop {
                        face: FaceRef(fi),
                        loop_index: li,
                    });
                }
                for (ci, u) in lp.uses.iter().enumerate() {
                    let Some(p) = u.pcurve else {
                        return Err(BuildError::MissingPcurve {
                            position: Position::new(FaceRef(fi), li, ci),
                        });
                    };
                    model.curve2(p)?;
                    self.edge(u.edge)?;
                    uses.entry(u.edge).or_default().push(u.orientation);
                }
            }
        }
        for (ei, e) in self.edges.iter() {
            let edge = EdgeRef(ei);
            let list = uses.get(&edge).map_or(&[][..], Vec::as_slice);
            if !uses_allowed(e, list.len(), open) {
                return Err(BuildError::EdgeUses {
                    edge,
                    uses: list.len(),
                });
            }
            if list.len() == 2 && list[0] == list[1] {
                return Err(BuildError::SameDirection { edge });
            }
            if let EdgeGeometry::Curve { curve, .. } = e.geometry {
                model.curve(curve)?;
            }
            if let Some(id) = e.kept {
                model.edge(id)?;
            }
            self.vertex(e.start)?;
            self.vertex(e.end)?;
        }
        for (_, v) in self.vertices.iter() {
            if let Some(id) = v.kept {
                model.vertex(id)?;
            }
        }
        model.transaction(|m| {
            let mut vertices = BTreeMap::new();
            for (i, v) in self.vertices.iter() {
                let id = match v.kept {
                    Some(id) => id,
                    None => m.push_vertex(entity::Vertex::new(v.point, v.tolerance)),
                };
                vertices.insert(VertexRef(i), id);
            }
            let mut edges = BTreeMap::new();
            for (i, e) in self.edges.iter() {
                let id = match e.kept {
                    Some(id) => id,
                    None => {
                        let start = *vertices
                            .get(&e.start)
                            .ok_or(BuildError::NoVertex(e.start))?;
                        let end = *vertices.get(&e.end).ok_or(BuildError::NoVertex(e.end))?;
                        m.push_edge(entity::Edge::new(e.geometry, start, end, e.tolerance))
                    }
                };
                edges.insert(EdgeRef(i), id);
            }
            let mut faces = BTreeMap::new();
            let mut shell_faces: BTreeMap<usize, Vec<handle::Face>> = BTreeMap::new();
            for (i, f) in self.faces.iter() {
                let id = match f.kept {
                    Some(id) => id,
                    None => {
                        let mut loops = Vec::with_capacity(f.loops.len());
                        for (li, lp) in f.loops.iter().enumerate() {
                            let mut effective = Vec::with_capacity(lp.uses.len());
                            for (ci, u) in lp.uses.iter().enumerate() {
                                let edge = *edges.get(&u.edge).ok_or(BuildError::NoEdge(u.edge))?;
                                let pcurve = u.pcurve.ok_or(BuildError::MissingPcurve {
                                    position: Position::new(FaceRef(i), li, ci),
                                })?;
                                effective.push((edge, u.orientation, pcurve));
                            }
                            let coedges = effective_uses(f.orientation, effective)
                                .into_iter()
                                .map(|(edge, orientation, pcurve)| {
                                    Coedge::new(edge, orientation, pcurve)
                                })
                                .collect();
                            loops.push(Loop::new(coedges));
                        }
                        m.push_face(entity::Face::new(f.surface, loops, f.tolerance))
                    }
                };
                faces.insert(FaceRef(i), id);
                shell_faces
                    .entry(f.shell)
                    .or_default()
                    .push(handle::Face::new(id, f.orientation));
            }
            let shells: Vec<ShellId> = shell_faces
                .into_values()
                .map(|uses| m.push_shell(entity::Shell::new(uses)))
                .collect();
            let body = m.push_body(entity::Body::new(
                kind,
                shells.iter().copied().map(handle::Shell::forward).collect(),
                Vec::new(),
                Vec::new(),
            ));
            Ok(Built {
                body: Body::forward(body),
                shells,
                vertices,
                edges,
                faces,
            })
        })
    }
}
