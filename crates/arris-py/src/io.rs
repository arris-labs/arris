//! The formats: what a STEP read returns, what body bytes import as, and
//! the mesh writers.
//!
//! The writers and readers themselves are `Model` methods (`write_step`,
//! `read_step`, `write_body`, `read_body`, `to_native`, …) and the mesh
//! writers are the module functions [`stl_binary`], [`stl_ascii`] and
//! [`obj`]; this file holds the values they return. A read returns handles
//! of the model it read into, and a refusal is a value, not an exception:
//! one refused solid never hides another (ADR-0025).

use std::sync::Arc;

use arris::io::body::Imported as KernelImported;
use arris::io::step::{
    LengthUnit, Occurrence as KernelOccurrence, Read, ReadSolid, Refusal as KernelRefusal, Rgb,
};
use arris::topo;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyString};

use crate::handle::{Body, Face};
use crate::kernel_error::mesh_write_error;
use crate::mesh::Mesh;
use crate::model::{Model, Shared};
use crate::provenance::Provenance;

/// The unit a STEP read converts lengths to, from its Python spelling.
pub(crate) fn length_unit(name: &str) -> PyResult<LengthUnit> {
    Ok(match name {
        "um" | "micrometre" => LengthUnit::Micrometre,
        "mm" | "millimetre" => LengthUnit::Millimetre,
        "cm" | "centimetre" => LengthUnit::Centimetre,
        "m" | "metre" => LengthUnit::Metre,
        "in" | "inch" => LengthUnit::Inch,
        "ft" | "foot" => LengthUnit::Foot,
        other => {
            return Err(PyValueError::new_err(format!(
                "unknown length unit {other:?}: one of \"um\", \"mm\", \"cm\", \"m\", \"in\", \"ft\""
            )));
        }
    })
}

/// Why a solid of a STEP file, or a placement in it, was not read, naming
/// the file entity where the reader stopped.
///
/// `kind` is the refusal's category (`"Offset"`, `"Unsupported"`, `"Gap"`,
/// …, the names the kernel's refusal histogram counts), `entity` the `#id`
/// of the file instance and `message` the kernel's own text.
#[pyclass(frozen, eq, module = "arris")]
#[derive(Clone, Debug, PartialEq)]
pub struct Refusal {
    kernel: KernelRefusal,
}

#[pymethods]
impl Refusal {
    /// The category of the refusal, as the kernel names it.
    #[getter]
    fn kind(&self) -> String {
        format!("{:?}", self.kernel.kind())
    }

    /// The `#id` of the file instance where the reader stopped.
    #[getter]
    fn entity(&self) -> u64 {
        self.kernel.entity()
    }

    /// The kernel's text for it.
    #[getter]
    fn message(&self) -> String {
        self.kernel.to_string()
    }

    fn __str__(&self) -> String {
        self.kernel.to_string()
    }

    fn __repr__(&self) -> String {
        format!("Refusal({:?}, entity={})", self.kind(), self.entity())
    }
}

/// One solid of a STEP file, read or refused.
///
/// Exactly one of `body` and `refusal` is set. `provenance` names, for
/// every shell, face, edge and vertex of `body`, the file entity it was read
/// from (`Role("file", …)`). `uncertainty` is the length uncertainty the
/// file's context claims, in the unit of the read; it is never an entity's
/// tolerance.
#[pyclass(frozen, module = "arris")]
#[derive(Clone, Debug)]
pub struct StepSolid {
    model: Arc<Shared>,
    kernel: ReadSolid,
}

#[pymethods]
impl StepSolid {
    /// The `#id` of the solid's instance in the file.
    #[getter]
    fn file_id(&self) -> u64 {
        self.kernel.entity.id
    }

    /// Which placement of it: an assembly that places one solid several
    /// times reads one body per placement, numbered from 0.
    #[getter]
    fn instance(&self) -> u32 {
        self.kernel.entity.instance
    }

    /// The length uncertainty the file's context claims, or `None`.
    #[getter]
    fn uncertainty(&self) -> Option<f64> {
        self.kernel.uncertainty
    }

    /// The body read, a solid that passes the checker, or `None` when the
    /// solid was refused.
    #[getter]
    fn body(&self) -> Option<Body> {
        let model = Model::sharing(&self.model);
        self.kernel
            .result
            .as_ref()
            .ok()
            .map(|read| Body::minted_by(&model, read.body))
    }

    /// The record of the body, or `None` when the solid was refused.
    #[getter]
    fn provenance(&self) -> Option<Provenance> {
        let model = Model::sharing(&self.model);
        self.kernel
            .result
            .as_ref()
            .ok()
            .map(|read| Provenance::minted_by(&model, read.provenance.clone()))
    }

    /// Why there is no body, or `None` when there is one.
    #[getter]
    fn refusal(&self) -> Option<Refusal> {
        self.kernel
            .result
            .as_ref()
            .err()
            .map(|r| Refusal { kernel: r.clone() })
    }

    /// Whether the solid was read.
    #[getter]
    fn ok(&self) -> bool {
        self.kernel.result.is_ok()
    }

    fn __repr__(&self) -> String {
        match &self.kernel.result {
            Ok(_) => format!("StepSolid(#{}, read)", self.kernel.entity.id),
            Err(refusal) => format!(
                "StepSolid(#{}, refused: {:?})",
                self.kernel.entity.id,
                refusal.kind()
            ),
        }
    }
}

fn rgb(colour: Rgb) -> (f64, f64, f64) {
    (colour.0[0], colour.0[1], colour.0[2])
}

/// One place a product stands in a file's assemblies.
///
/// `placement` is the 4×4 row-major matrix that puts the occurrence in its
/// parent, in the unit of the read (the identity at a root), or `None` when
/// the file's transformation is one the reader refuses (`placement_refusal`
/// says why). `solids` index `StepRead.solids`; `children` are the
/// occurrences placed in this one.
#[pyclass(frozen, module = "arris")]
#[derive(Clone, Debug)]
pub struct Occurrence {
    kernel: KernelOccurrence,
}

#[pymethods]
impl Occurrence {
    /// The `#id` of the `PRODUCT_DEFINITION`, or `None` for a
    /// representation attached to no product.
    #[getter]
    fn product(&self) -> Option<u64> {
        self.kernel.product
    }

    /// The product's name.
    #[getter]
    fn name(&self) -> &str {
        &self.kernel.name
    }

    /// The placement in the parent as four rows of four numbers, or `None`.
    #[getter]
    fn placement(&self) -> Option<[[f64; 4]; 4]> {
        let iso = self.kernel.placement.as_ref().ok()?;
        let r = iso.rotation().to_rotation_matrix();
        let t = iso.translation();
        let mut m = [[0.0, 0.0, 0.0, 0.0]; 4];
        for (i, row) in m.iter_mut().enumerate().take(3) {
            for (j, cell) in row.iter_mut().enumerate().take(3) {
                *cell = r[(i, j)];
            }
            row[3] = t[i];
        }
        m[3][3] = 1.0;
        Some(m)
    }

    /// Why there is no placement, or `None`.
    #[getter]
    fn placement_refusal(&self) -> Option<Refusal> {
        self.kernel
            .placement
            .as_ref()
            .err()
            .map(|r| Refusal { kernel: r.clone() })
    }

    /// The colour of the occurrence's solids, `(r, g, b)` in `0..=1`, if
    /// the file gives one.
    #[getter]
    fn colour(&self) -> Option<(f64, f64, f64)> {
        self.kernel.colour.map(rgb)
    }

    /// The solids this occurrence holds itself, as indices of
    /// `StepRead.solids`.
    #[getter]
    fn solids(&self) -> Vec<usize> {
        self.kernel.solids.clone()
    }

    /// The occurrences placed in this one.
    #[getter]
    fn children(&self) -> Vec<Occurrence> {
        self.kernel
            .children
            .iter()
            .map(|kernel| Occurrence {
                kernel: kernel.clone(),
            })
            .collect()
    }

    fn __repr__(&self) -> String {
        format!(
            "Occurrence({:?}, solids={}, children={})",
            self.kernel.name,
            self.kernel.solids.len(),
            self.kernel.children.len()
        )
    }
}

/// What reading a STEP file returned: one `StepSolid` per solid of the file
/// (at every placement an assembly puts it), the product structure beside
/// them, and the faces the file colours apart from their solids.
///
/// ```python
/// import arris
///
/// model = arris.Model()
/// body, _ = model.primitive_box((0, 0, 0), (1, 2, 3))
/// text = model.write_step([body])
///
/// back = arris.Model()
/// read = back.read_step(text)
/// [solid] = read.solids
/// assert solid.ok and solid.refusal is None
/// assert [root.name for root in read.products] == ["arris"]
/// assert back.mass_properties(solid.body).volume == 6.0
/// ```
#[pyclass(frozen, module = "arris")]
#[derive(Clone, Debug)]
pub struct StepRead {
    model: Arc<Shared>,
    kernel: Read,
}

impl StepRead {
    /// `kernel`, read into `model`.
    pub(crate) fn minted_by(model: &Model, kernel: Read) -> StepRead {
        StepRead {
            model: Arc::clone(&model.shared),
            kernel,
        }
    }
}

#[pymethods]
impl StepRead {
    /// Every solid of the file, ascending by file entity then placement.
    #[getter]
    fn solids(&self) -> Vec<StepSolid> {
        self.kernel
            .solids
            .iter()
            .map(|kernel| StepSolid {
                model: Arc::clone(&self.model),
                kernel: kernel.clone(),
            })
            .collect()
    }

    /// The file's assemblies: each root assembly and each lone part.
    #[getter]
    fn products(&self) -> Vec<Occurrence> {
        self.kernel
            .products
            .roots
            .iter()
            .map(|kernel| Occurrence {
                kernel: kernel.clone(),
            })
            .collect()
    }

    /// The faces the file colours apart from their solids:
    /// `(solid index, face, (r, g, b))`.
    #[getter]
    fn face_colours(&self) -> Vec<(usize, Face, (f64, f64, f64))> {
        let model = Model::sharing(&self.model);
        self.kernel
            .products
            .faces
            .iter()
            .map(|c| {
                (
                    c.solid,
                    Face::minted_by(&model, topo::Face::forward(c.face)),
                    rgb(c.colour),
                )
            })
            .collect()
    }

    fn __repr__(&self) -> String {
        let read = self
            .kernel
            .solids
            .iter()
            .filter(|s| s.result.is_ok())
            .count();
        format!(
            "StepRead(solids={}, read={read}, refused={})",
            self.kernel.solids.len(),
            self.kernel.solids.len() - read
        )
    }
}

/// A body imported from body bytes, and the way back to its record.
///
/// ```python
/// import arris
///
/// a = arris.Model()
/// body, record = a.primitive_box((0, 0, 0), (1, 2, 3))
/// data = a.write_body(body, record)
///
/// b = arris.Model()
/// imported = b.read_body(data)
/// assert b.mass_properties(imported.body).volume == 6.0
/// assert imported.foreign() == []
/// assert imported.translated().generated  # the box's roles, now in `b`
/// ```
#[pyclass(frozen, module = "arris")]
#[derive(Clone, Debug)]
pub struct Imported {
    model: Arc<Shared>,
    kernel: KernelImported,
}

impl Imported {
    /// `kernel`, imported into `model`.
    pub(crate) fn minted_by(model: &Model, kernel: KernelImported) -> Imported {
        Imported {
            model: Arc::clone(&model.shared),
            kernel,
        }
    }
}

#[pymethods]
impl Imported {
    /// The body, now in the reading model.
    #[getter]
    fn body(&self) -> Body {
        Body::minted_by(&Model::sharing(&self.model), self.kernel.body)
    }

    /// The version the data carried, before any migration.
    #[getter]
    fn version(&self) -> u32 {
        self.kernel.version
    }

    /// The entities the written record names that are not the body's (a
    /// boolean's inputs, the faces it deleted), as the writer's ids in
    /// text form: they have no entity in this model.
    fn foreign(&self) -> Vec<String> {
        self.kernel
            .foreign()
            .iter()
            .map(|s| s.to_string())
            .collect()
    }

    /// The record the writer held, in this model's handles.
    ///
    /// Raises `ValueError` when the record names entities that are not the
    /// body's (`foreign()` lists them): they have no handle here, and
    /// guessing one would name a different entity.
    fn translated(&self) -> PyResult<Provenance> {
        let foreign = self.foreign();
        if !foreign.is_empty() {
            return Err(PyValueError::new_err(format!(
                "the record names {} entities outside the body ({}), which have no handle in this model",
                foreign.len(),
                foreign.join(", ")
            )));
        }
        let record = self.kernel.translated(&arris::topo::IdMap::default());
        Ok(Provenance::minted_by(&Model::sharing(&self.model), record))
    }

    fn __repr__(&self) -> String {
        format!("Imported(version={})", self.kernel.version)
    }
}

fn kernel_meshes(meshes: &[PyRef<'_, Mesh>]) -> Vec<arris::mesh::TriMesh> {
    meshes.iter().map(|m| m.kernel().clone()).collect()
}

/// The meshes as binary STL: an 80-byte header carrying `name`, a `u32`
/// count and 50 bytes a triangle, little-endian. Facet coordinates are
/// `f32` because the format says so.
///
/// Raises `MeshTooManyTrianglesError` past `u32::MAX` triangles.
///
/// ```python
/// import arris
///
/// model = arris.Model()
/// body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
/// data = arris.stl_binary([model.tessellate(body, 1e-3)])
/// assert len(data) == 84 + 50 * 12
/// ```
#[pyfunction]
#[pyo3(signature = (meshes, name="arris"))]
pub fn stl_binary<'py>(
    py: Python<'py>,
    meshes: Vec<PyRef<'py, Mesh>>,
    name: &str,
) -> PyResult<Bound<'py, PyBytes>> {
    let meshes = kernel_meshes(&meshes);
    let bytes = arris::io::stl::write_binary(&meshes, name)
        .map_err(|e| mesh_write_error(&e).raise(py, None))?;
    Ok(PyBytes::new(py, &bytes))
}

/// The meshes as ASCII STL, one `solid` named `name`.
///
/// ```python
/// import arris
///
/// model = arris.Model()
/// body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
/// text = arris.stl_ascii([model.tessellate(body, 1e-3)], "cube")
/// assert text.startswith("solid cube") and text.count("facet normal") == 12
/// ```
#[pyfunction]
#[pyo3(signature = (meshes, name="arris"))]
pub fn stl_ascii<'py>(
    py: Python<'py>,
    meshes: Vec<PyRef<'py, Mesh>>,
    name: &str,
) -> PyResult<Bound<'py, PyString>> {
    let meshes = kernel_meshes(&meshes);
    let text = arris::io::stl::write_ascii(&meshes, name)
        .map_err(|e| mesh_write_error(&e).raise(py, None))?;
    Ok(PyString::new(py, &text))
}

/// The meshes as Wavefront OBJ text, one object per mesh.
///
/// ```python
/// import arris
///
/// model = arris.Model()
/// body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
/// text = arris.obj([model.tessellate(body, 1e-3)])
/// assert text.count("f ") >= 12
/// ```
#[pyfunction]
pub fn obj<'py>(py: Python<'py>, meshes: Vec<PyRef<'py, Mesh>>) -> PyResult<Bound<'py, PyString>> {
    let meshes = kernel_meshes(&meshes);
    let text = arris::io::obj::write(&meshes).map_err(|e| mesh_write_error(&e).raise(py, None))?;
    Ok(PyString::new(py, &text))
}
