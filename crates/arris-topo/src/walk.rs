//! Iteration over a body and its closure (`docs/DATA-MODEL.md`
//! §Adjacency and iteration).

use std::collections::BTreeSet;

use crate::NotFound;
use crate::handle::{Body, Edge, Face, Shell, Vertex};
use crate::id::{Curve2Id, CurveId, EdgeId, FaceId, ShellId, SurfaceId, VertexId};
use crate::model::Model;
use crate::orientation::Orientation;

/// Every entity and geometry value reachable from a body, as sorted,
/// duplicate-free id lists per kind. What the checker, `import`, `retain`
/// and the text dump walk. Only ids that resolve are listed: a dangling
/// reference reaches nothing (the checker's M1 row is where it is
/// reported).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Closure {
    /// The shells, ascending.
    pub shells: Vec<ShellId>,
    /// The faces, ascending.
    pub faces: Vec<FaceId>,
    /// The edges, ascending.
    pub edges: Vec<EdgeId>,
    /// The vertices, ascending.
    pub vertices: Vec<VertexId>,
    /// The curves the edges reference, ascending.
    pub curves: Vec<CurveId>,
    /// The surfaces the faces reference, ascending.
    pub surfaces: Vec<SurfaceId>,
    /// The pcurves the coedges reference, ascending.
    pub curve2s: Vec<Curve2Id>,
}

/// The four iteration lists of one walk.
#[derive(Debug, Default)]
struct Walk {
    shells: Vec<Shell>,
    faces: Vec<Face>,
    edges: Vec<Edge>,
    vertices: Vec<Vertex>,
}

/// A first-visit filter over ids.
#[derive(Default)]
struct Seen {
    shells: BTreeSet<ShellId>,
    faces: BTreeSet<FaceId>,
    edges: BTreeSet<EdgeId>,
    vertices: BTreeSet<VertexId>,
}

impl Model {
    /// The body's shells with their effective orientation (the body
    /// handle's composed with each use's), in stored order, each once at
    /// first visit. Errors: the body does not resolve. A shell use that
    /// does not resolve is skipped.
    pub fn shells(&self, body: Body) -> Result<Vec<Shell>, NotFound> {
        Ok(self.walk(body)?.shells)
    }

    /// The body's faces with their effective orientation, depth-first
    /// through the shells in stored order, each once at first visit (a
    /// face used by two shells appears under the first). Errors: the body
    /// does not resolve. A reference that does not resolve is skipped.
    pub fn faces(&self, body: Body) -> Result<Vec<Face>, NotFound> {
        Ok(self.walk(body)?.faces)
    }

    /// The body's edges with their effective orientation — the coedge's
    /// composed down the path through the face use, the shell use and the
    /// body handle — depth-first through shells, faces, loops and coedges
    /// in stored order, then the free edges; each once at first visit, so
    /// a seam edge appears once, with the orientation of its first use.
    /// Errors: the body does not resolve. A reference that does not
    /// resolve is skipped.
    pub fn edges(&self, body: Body) -> Result<Vec<Edge>, NotFound> {
        Ok(self.walk(body)?.edges)
    }

    /// The body's vertices in the order the edge walk reaches them — each
    /// edge's effective start then its effective end — then the free
    /// vertices; each once at first visit. A vertex handle carries the
    /// effective orientation of the edge use it was first reached through
    /// (the body handle's, for a free vertex); it has no geometric meaning
    /// and is there so every handle composes the same way. Errors: the
    /// body does not resolve. A reference that does not resolve is
    /// skipped.
    pub fn vertices(&self, body: Body) -> Result<Vec<Vertex>, NotFound> {
        Ok(self.walk(body)?.vertices)
    }

    /// The closures of `bodies` concatenated, kind by kind: unsorted, with
    /// an entity reachable from two bodies listed twice. The live set
    /// [`Model::retain`] and [`Model::discard`] keep. Errors: a body does
    /// not resolve.
    pub(crate) fn closures(&self, bodies: &[Body]) -> Result<Closure, NotFound> {
        let mut all = Closure::default();
        for &body in bodies {
            let c = self.closure(body)?;
            all.vertices.extend(c.vertices);
            all.edges.extend(c.edges);
            all.faces.extend(c.faces);
            all.shells.extend(c.shells);
            all.curves.extend(c.curves);
            all.surfaces.extend(c.surfaces);
            all.curve2s.extend(c.curve2s);
        }
        Ok(all)
    }

    /// Everything reachable from `body`, as sorted id lists per kind.
    /// Errors: the body does not resolve.
    pub fn closure(&self, body: Body) -> Result<Closure, NotFound> {
        let walk = self.walk(body)?;
        let mut c = Closure {
            shells: walk.shells.iter().map(|s| s.id).collect(),
            faces: walk.faces.iter().map(|f| f.id).collect(),
            edges: walk.edges.iter().map(|e| e.id).collect(),
            vertices: walk.vertices.iter().map(|v| v.id).collect(),
            ..Closure::default()
        };
        let mut curves = BTreeSet::new();
        let mut surfaces = BTreeSet::new();
        let mut curve2s = BTreeSet::new();
        for e in &walk.edges {
            if let Some((curve, _)) = self.edge(e.id).ok().and_then(|e| e.curve()) {
                if self.curve(curve).is_ok() {
                    curves.insert(curve);
                }
            }
        }
        for f in &walk.faces {
            let Ok(face) = self.face(f.id) else { continue };
            if self.surface(face.surface()).is_ok() {
                surfaces.insert(face.surface());
            }
            for coedge in face.loops().iter().flat_map(|l| l.coedges()) {
                if self.curve2(coedge.pcurve()).is_ok() {
                    curve2s.insert(coedge.pcurve());
                }
            }
        }
        c.shells.sort();
        c.faces.sort();
        c.edges.sort();
        c.vertices.sort();
        c.curves = curves.into_iter().collect();
        c.surfaces = surfaces.into_iter().collect();
        c.curve2s = curve2s.into_iter().collect();
        Ok(c)
    }

    fn walk(&self, body: Body) -> Result<Walk, NotFound> {
        let entity = self.body(body.id)?;
        let mut walk = Walk::default();
        let mut seen = Seen::default();
        for shell_use in entity.shells() {
            let shell = shell_use.oriented_by(body.orientation);
            let Ok(shell_entity) = self.shell(shell.id) else {
                continue;
            };
            if !seen.shells.insert(shell.id) {
                continue;
            }
            walk.shells.push(shell);
            for face_use in shell_entity.faces() {
                let face = face_use.oriented_by(shell.orientation);
                let Ok(face_entity) = self.face(face.id) else {
                    continue;
                };
                if !seen.faces.insert(face.id) {
                    continue;
                }
                walk.faces.push(face);
                for coedge in face_entity.loops().iter().flat_map(|l| l.coedges()) {
                    let edge = coedge.edge_use().oriented_by(face.orientation);
                    self.visit_edge(edge, &mut walk, &mut seen);
                }
            }
        }
        for edge_use in entity.free_edges() {
            let edge = edge_use.oriented_by(body.orientation);
            self.visit_edge(edge, &mut walk, &mut seen);
        }
        for &vertex in entity.free_vertices() {
            self.visit_vertex(Vertex::new(vertex, body.orientation), &mut walk, &mut seen);
        }
        Ok(walk)
    }

    fn visit_edge(&self, edge: Edge, walk: &mut Walk, seen: &mut Seen) {
        let Ok(entity) = self.edge(edge.id) else {
            return;
        };
        if !seen.edges.insert(edge.id) {
            return;
        }
        walk.edges.push(edge);
        let (first, second) = match edge.orientation {
            Orientation::Forward => (entity.start(), entity.end()),
            Orientation::Reversed => (entity.end(), entity.start()),
        };
        self.visit_vertex(Vertex::new(first, edge.orientation), walk, seen);
        self.visit_vertex(Vertex::new(second, edge.orientation), walk, seen);
    }

    fn visit_vertex(&self, vertex: Vertex, walk: &mut Walk, seen: &mut Seen) {
        if self.vertex(vertex.id).is_err() || !seen.vertices.insert(vertex.id) {
            return;
        }
        walk.vertices.push(vertex);
    }
}
