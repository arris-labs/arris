//! `Mesh`: a tessellation as bytes, for a consumer that renders it with
//! its own tools (ADR-0034 §5).
//!
//! The buffers are the kernel's own, unconverted: positions are
//! little-endian `f64` (the tessellation boundary is `f64`, ADR-0011) and
//! indices little-endian `u32`, so `struct`, `memoryview.cast` and
//! `numpy.frombuffer` read them without copying. Nothing here needs numpy;
//! [`Mesh::to_numpy`] imports it only when called.

use std::sync::Arc;

use arris::mesh::TriMesh;
use arris::topo::{self, EdgeId, FaceId};
use pyo3::prelude::*;
use pyo3::types::PyBytes;

use crate::handle::{Edge, Face};
use crate::model::{Model, Shared};

/// A point as Python reads it.
type Point = (f64, f64, f64);

/// The triangle mesh of a body.
///
/// `positions` holds `n_positions` points as three little-endian `f64`
/// each; `triangles` holds `n_triangles` triples of little-endian `u32`
/// indices into them, counter-clockwise seen from outside. `faces` and
/// `edges` say which run of triangles (or of `edge_indices`, a polyline of
/// indices) belongs to which B-Rep face or edge, in the body's iteration
/// order. Positions are shared along edges, so a closed body's mesh is
/// closed (`is_closed()`).
///
/// ```python
/// import arris, struct
///
/// model = arris.Model()
/// body, _ = model.primitive_box((0, 0, 0), (1, 2, 3))
/// mesh = model.tessellate(body, 1e-3)
/// assert mesh.is_closed() and mesh.n_triangles == 12
/// xs = struct.unpack(f"<{3 * mesh.n_positions}d", mesh.positions)
/// assert max(xs[0::3]) == 1.0
/// ```
#[pyclass(frozen, module = "arris")]
#[derive(Clone, Debug)]
pub struct Mesh {
    model: Arc<Shared>,
    kernel: TriMesh,
}

impl Mesh {
    /// `kernel`, the mesh of a body of `model`.
    pub fn minted_by(model: &Model, kernel: TriMesh) -> Mesh {
        Mesh {
            model: Arc::clone(&model.shared),
            kernel,
        }
    }

    fn face_handle(&self, id: FaceId) -> Face {
        Face::minted_by(&Model::sharing(&self.model), topo::Face::forward(id))
    }

    fn edge_handle(&self, id: EdgeId) -> Edge {
        Edge::minted_by(&Model::sharing(&self.model), topo::Edge::forward(id))
    }
}

#[pymethods]
impl Mesh {
    /// The points: `3 * n_positions` little-endian `f64`s, as bytes.
    #[getter]
    fn positions<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        let bytes: Vec<u8> = self
            .kernel
            .positions()
            .iter()
            .flatten()
            .flat_map(|c| c.to_le_bytes())
            .collect();
        PyBytes::new(py, &bytes)
    }

    /// The triangles: `3 * n_triangles` little-endian `u32` indices into
    /// the positions, as bytes.
    #[getter]
    fn triangles<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        let bytes: Vec<u8> = self
            .kernel
            .triangles()
            .iter()
            .flatten()
            .flat_map(|i| i.to_le_bytes())
            .collect();
        PyBytes::new(py, &bytes)
    }

    /// The edge polylines' indices into the positions, as little-endian
    /// `u32` bytes; `edges` says which run is which edge.
    #[getter]
    fn edge_indices<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        let bytes: Vec<u8> = self
            .kernel
            .edge_indices()
            .iter()
            .flat_map(|i| i.to_le_bytes())
            .collect();
        PyBytes::new(py, &bytes)
    }

    /// How many points.
    #[getter]
    fn n_positions(&self) -> usize {
        self.kernel.positions().len()
    }

    /// How many triangles.
    #[getter]
    fn n_triangles(&self) -> usize {
        self.kernel.triangles().len()
    }

    /// The triangles of each face: `(face, start, stop)`, where
    /// `start..stop` indexes the triangle list.
    #[getter]
    fn faces(&self) -> Vec<(Face, usize, usize)> {
        self.kernel
            .faces()
            .iter()
            .map(|f| (self.face_handle(f.face), f.triangles.start, f.triangles.end))
            .collect()
    }

    /// The polyline of each edge: `(edge, start, stop)`, where `start..stop`
    /// indexes `edge_indices`.
    #[getter]
    fn edges(&self) -> Vec<(Edge, usize, usize)> {
        self.kernel
            .edges()
            .iter()
            .map(|e| (self.edge_handle(e.edge), e.indices.start, e.indices.end))
            .collect()
    }

    /// Whether every edge of the mesh is shared by two triangles running
    /// opposite ways: a watertight mesh of a closed body.
    fn is_closed(&self) -> bool {
        self.kernel.is_closed()
    }

    /// The enclosed volume by the divergence theorem, positive for a mesh
    /// whose triangles face outward; `None` when the mesh is not closed.
    fn signed_volume(&self) -> Option<f64> {
        self.kernel.signed_volume()
    }

    /// The total area of the triangles.
    fn area(&self) -> f64 {
        self.kernel.area()
    }

    /// The corners of the box around the points, `(min, max)`, or `None`
    /// for an empty mesh.
    fn bounds(&self) -> Option<(Point, Point)> {
        self.kernel.aabb().map(|b| {
            (
                (b.min[0], b.min[1], b.min[2]),
                (b.max[0], b.max[1], b.max[2]),
            )
        })
    }

    /// `(positions, triangles)` as numpy arrays of shape `(n, 3)` and
    /// dtype `float64` and `uint32`, sharing nothing with the mesh.
    ///
    /// Raises `ImportError` when numpy is not installed; nothing else in
    /// `arris` needs it.
    fn to_numpy<'py>(slf: &Bound<'py, Self>) -> PyResult<Bound<'py, PyAny>> {
        let py = slf.py();
        py.import("arris._numpy")?
            .getattr("mesh_to_numpy")?
            .call1((slf,))
    }

    fn __repr__(&self) -> String {
        format!(
            "Mesh(positions={}, triangles={}, faces={})",
            self.kernel.positions().len(),
            self.kernel.triangles().len(),
            self.kernel.faces().len()
        )
    }
}
