//! `Model`'s topology walks: shells, faces, edges and vertices of a body.

use super::*;

#[pymethods]
impl Model {
    /// The shells of `body`, in stored order.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// assert len(model.shells(body)) == 1
    /// ```
    fn shells(&self, body: &Body) -> PyResult<Vec<Shell>> {
        let body = body.resolve(&self.shared)?;
        let shells = self.shared.lock()?.shells(body).map_err(BindError::from)?;
        Ok(shells
            .into_iter()
            .map(|s| Shell::minted_by(self, s))
            .collect())
    }

    /// The kind of `body`: `"solid"`, `"sheet"` (open shells, as a planar
    /// section or a STEP surface model makes), `"wire"` or `"general"`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// assert model.body_kind(body) == "solid"
    /// ```
    fn body_kind(&self, body: &Body) -> PyResult<&'static str> {
        use arris::topo::entity::BodyKind;
        let body = body.resolve(&self.shared)?;
        let kind = self
            .shared
            .lock()?
            .body(body.id)
            .map_err(BindError::from)?
            .kind();
        Ok(match kind {
            BodyKind::Solid => "solid",
            BodyKind::Sheet => "sheet",
            BodyKind::Wire => "wire",
            BodyKind::General => "general",
        })
    }

    /// The faces of `body`, each once, depth-first through its shells.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// assert len(model.faces(body)) == 6
    /// ```
    fn faces(&self, body: &Body) -> PyResult<Vec<Face>> {
        let body = body.resolve(&self.shared)?;
        let faces = self.shared.lock()?.faces(body).map_err(BindError::from)?;
        Ok(faces
            .into_iter()
            .map(|f| Face::minted_by(self, f))
            .collect())
    }

    /// The edges of `body`, each once, in the order its faces' loops reach
    /// them; a seam appears once.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// assert len(model.edges(body)) == 12
    /// ```
    fn edges(&self, body: &Body) -> PyResult<Vec<Edge>> {
        let body = body.resolve(&self.shared)?;
        let edges = self.shared.lock()?.edges(body).map_err(BindError::from)?;
        Ok(edges
            .into_iter()
            .map(|e| Edge::minted_by(self, e))
            .collect())
    }

    /// The vertices of `body`, each once, in the order the edge walk
    /// reaches them.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// assert len(model.vertices(body)) == 8
    /// ```
    fn vertices(&self, body: &Body) -> PyResult<Vec<Vertex>> {
        let body = body.resolve(&self.shared)?;
        let vertices = self
            .shared
            .lock()?
            .vertices(body)
            .map_err(BindError::from)?;
        Ok(vertices
            .into_iter()
            .map(|v| Vertex::minted_by(self, v))
            .collect())
    }

    /// The edges that bound `face`, each once, in the order of its loops
    /// (the outer loop first) and their coedges, with the orientation each
    /// is used in.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// [face, *_] = model.faces(body)
    /// assert len(model.edges_of(face)) == 4
    /// ```
    fn edges_of(&self, face: &Face) -> PyResult<Vec<Edge>> {
        let face = face.resolve(&self.shared)?;
        let kernel = self.shared.lock()?;
        let entity = kernel.face(face.id).map_err(BindError::from)?;
        let mut edges: Vec<topo::Edge> = Vec::new();
        for coedge in entity.loops().iter().flat_map(|l| l.coedges()) {
            let used = coedge.edge_use().oriented_by(face.orientation);
            if !edges.iter().any(|e| e.id == used.id) {
                edges.push(used);
            }
        }
        Ok(edges
            .into_iter()
            .map(|e| Edge::minted_by(self, e))
            .collect())
    }

    /// The vertices at the two ends of `edge`, in the direction it was
    /// reached: a reversed edge starts at its curve's end.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// [edge, *_] = model.edges(body)
    /// start, end = model.vertices_of(edge)
    /// assert start != end
    /// ```
    fn vertices_of(&self, edge: &Edge) -> PyResult<(Vertex, Vertex)> {
        let edge = edge.resolve(&self.shared)?;
        let kernel = self.shared.lock()?;
        let entity = kernel.edge(edge.id).map_err(BindError::from)?;
        let (start, end) = match edge.orientation {
            topo::Orientation::Forward => (entity.start(), entity.end()),
            topo::Orientation::Reversed => (entity.end(), entity.start()),
        };
        Ok((
            Vertex::minted_by(self, topo::Vertex::forward(start)),
            Vertex::minted_by(self, topo::Vertex::forward(end)),
        ))
    }

    /// The faces of `body` that use `edge`: two for an edge between faces
    /// of a solid (one face twice for a seam, listed once).
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// [edge, *_] = model.edges(body)
    /// assert len(model.faces_of(body, edge)) == 2
    /// ```
    fn faces_of(&self, body: &Body, edge: &Edge) -> PyResult<Vec<Face>> {
        let (body, edge) = (body.resolve(&self.shared)?, edge.resolve(&self.shared)?);
        let kernel = self.shared.lock()?;
        let uses = kernel.edge_uses(edge.id).map_err(BindError::from)?;
        let faces = kernel.faces(body).map_err(BindError::from)?;
        Ok(faces
            .into_iter()
            .filter(|f| uses.iter().any(|u| u.face == f.id))
            .map(|f| Face::minted_by(self, f))
            .collect())
    }

    /// The edges of `body` that start or end at `vertex`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// [vertex, *_] = model.vertices(body)
    /// assert len(model.edges_at(body, vertex)) == 3
    /// ```
    fn edges_at(&self, body: &Body, vertex: &Vertex) -> PyResult<Vec<Edge>> {
        let (body, vertex) = (body.resolve(&self.shared)?, vertex.resolve(&self.shared)?);
        let kernel = self.shared.lock()?;
        let at = kernel.vertex_edges(vertex.id).map_err(BindError::from)?;
        let edges = kernel.edges(body).map_err(BindError::from)?;
        Ok(edges
            .into_iter()
            .filter(|e| at.contains(&e.id))
            .map(|e| Edge::minted_by(self, e))
            .collect())
    }
}
