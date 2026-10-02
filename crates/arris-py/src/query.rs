//! What a body measures and what the checker says of it, as Python values.

use std::sync::Arc;

use arris::check::{self, EulerLine as KernelEuler, Level, Report as KernelReport, Unchecked};
use arris::math::{Frame as KernelFrame, Matrix3, Point3};
use arris::ops::measure::MassProperties as KernelMass;
use arris::topo::EntityId;
use pyo3::IntoPyObjectExt;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::kernel_error::entity;
use crate::model::{Model, Shared};

/// A right-handed orthonormal frame: a point and three unit axes.
///
/// A face's frame has `z` along its outward normal.
///
/// ```python
/// import arris
///
/// model = arris.Model()
/// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
/// top = record.generated_from(arris.Role("box", "Face", "Z", "Max"))[0]
/// frame = model.face_frame(top)
/// assert frame.z == (0.0, 0.0, 1.0)
/// ```
#[pyclass(frozen, eq, module = "arris")]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    kernel: KernelFrame,
}

impl Frame {
    /// The Python value of `kernel`.
    pub fn new(kernel: KernelFrame) -> Frame {
        Frame { kernel }
    }
}

fn triple(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    (x, y, z)
}

#[pymethods]
impl Frame {
    /// The frame's origin.
    #[getter]
    fn origin(&self) -> (f64, f64, f64) {
        let o = self.kernel.origin();
        triple(o.x, o.y, o.z)
    }

    /// The `x` axis, a unit vector.
    #[getter]
    fn x(&self) -> (f64, f64, f64) {
        let v = self.kernel.x();
        triple(v.x, v.y, v.z)
    }

    /// The `y` axis, a unit vector.
    #[getter]
    fn y(&self) -> (f64, f64, f64) {
        let v = self.kernel.y();
        triple(v.x, v.y, v.z)
    }

    /// The `z` axis, a unit vector: `x × y`.
    #[getter]
    fn z(&self) -> (f64, f64, f64) {
        let v = self.kernel.z();
        triple(v.x, v.y, v.z)
    }

    fn __repr__(&self) -> String {
        format!(
            "Frame(origin={:?}, x={:?}, y={:?}, z={:?})",
            self.origin(),
            self.x(),
            self.y(),
            self.z()
        )
    }
}

type Rows = ((f64, f64, f64), (f64, f64, f64), (f64, f64, f64));

fn rows(m: &Matrix3) -> Rows {
    (
        triple(m[(0, 0)], m[(0, 1)], m[(0, 2)]),
        triple(m[(1, 0)], m[(1, 1)], m[(1, 2)]),
        triple(m[(2, 0)], m[(2, 1)], m[(2, 2)]),
    )
}

/// The mass properties of a solid of unit density.
///
/// `volume` is also the mass. `inertia` is the tensor about the centroid in
/// the physical convention (the diagonal holds the moments, the
/// off-diagonal the negated products), as a tuple of three rows.
///
/// ```python
/// import arris
///
/// model = arris.Model()
/// body, _ = model.primitive_box((0, 0, 0), (1, 2, 3))
/// props = model.mass_properties(body)
/// assert abs(props.volume - 6) < 1e-12
/// assert abs(props.area - 22) < 1e-12
/// assert all(abs(c - h) < 1e-12 for c, h in zip(props.centroid, (0.5, 1, 1.5)))
/// ```
#[pyclass(frozen, eq, module = "arris")]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MassProperties {
    kernel: KernelMass,
}

impl MassProperties {
    /// The Python value of `kernel`.
    pub fn new(kernel: KernelMass) -> MassProperties {
        MassProperties { kernel }
    }
}

#[pymethods]
impl MassProperties {
    /// The enclosed volume.
    #[getter]
    fn volume(&self) -> f64 {
        self.kernel.volume
    }

    /// The total area of the body's faces.
    #[getter]
    fn area(&self) -> f64 {
        self.kernel.area
    }

    /// The centre of mass.
    #[getter]
    fn centroid(&self) -> (f64, f64, f64) {
        let c = self.kernel.centroid;
        triple(c.x, c.y, c.z)
    }

    /// The inertia tensor about the centroid, as three rows.
    #[getter]
    fn inertia(&self) -> Rows {
        rows(&self.kernel.inertia)
    }

    /// The inertia tensor about `point` instead, by the parallel-axis
    /// theorem.
    fn inertia_about(&self, point: [f64; 3]) -> Rows {
        rows(&self.kernel.inertia_about(Point3::from(point)))
    }

    fn __repr__(&self) -> String {
        format!(
            "MassProperties(volume={}, area={}, centroid={:?})",
            self.kernel.volume,
            self.kernel.area,
            self.centroid()
        )
    }
}

/// The counts of the Euler–Poincaré relation and the genus they imply.
///
/// A line, never a violation: a body whose line does not close is reported
/// by the rows that are broken, if any.
#[pyclass(frozen, eq, hash, module = "arris")]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct EulerLine {
    kernel: KernelEuler,
}

#[pymethods]
impl EulerLine {
    /// Vertices in the body.
    #[getter]
    fn vertices(&self) -> usize {
        self.kernel.vertices
    }

    /// Edges in the body that are not degenerate.
    #[getter]
    fn edges(&self) -> usize {
        self.kernel.edges
    }

    /// Faces in the body.
    #[getter]
    fn faces(&self) -> usize {
        self.kernel.faces
    }

    /// Loops over those faces.
    #[getter]
    fn loops(&self) -> usize {
        self.kernel.loops
    }

    /// Shells in the body.
    #[getter]
    fn shells(&self) -> usize {
        self.kernel.shells
    }

    /// The genus the counts imply.
    #[getter]
    fn genus(&self) -> i64 {
        self.kernel.genus
    }

    fn __repr__(&self) -> String {
        let k = &self.kernel;
        format!(
            "EulerLine(vertices={}, edges={}, faces={}, loops={}, shells={}, genus={})",
            k.vertices, k.edges, k.faces, k.loops, k.shells, k.genus
        )
    }
}

/// How an entity is named in a report: a handle of the model the check ran
/// on, or the kernel's text (`f3`) when the report has no model.
fn name<'py>(
    py: Python<'py>,
    model: &Option<Arc<Shared>>,
    id: EntityId,
) -> PyResult<Bound<'py, PyAny>> {
    let shape = arris::topo::Shape::new(id, Default::default());
    match model {
        Some(shared) => entity(py, Some(&Model::sharing(shared)), shape),
        None => id.to_string().into_bound_py_any(py),
    }
}

/// One invariant the checker found broken.
///
/// `code` is the row of `docs/DATA-MODEL.md` §Invariants (`"E3"`), `entity`
/// the entity that breaks it (a handle, or its text for a report with no
/// model), `level` `"fast"` or `"full"`, and `str(violation)` the checker's
/// own line with the numbers.
#[pyclass(frozen, module = "arris")]
#[derive(Clone, Debug)]
pub struct Violation {
    model: Option<Arc<Shared>>,
    kernel: check::Violation,
}

#[pymethods]
impl Violation {
    /// The invariant's row: `"M1"`, `"E4"`, …
    #[getter]
    fn code(&self) -> &'static str {
        self.kernel.code()
    }

    /// The entity that violates the invariant.
    #[getter]
    fn entity<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        name(py, &self.model, self.kernel.entity())
    }

    /// When the invariant runs: `"fast"` or `"full"`.
    #[getter]
    fn level(&self) -> &'static str {
        level_name(self.kernel.level())
    }

    fn __str__(&self) -> String {
        self.kernel.to_string()
    }

    fn __repr__(&self) -> String {
        format!("Violation({})", self.kernel)
    }
}

/// A `Full` row the checker could not decide on a body: neither a violation
/// nor a pass.
#[pyclass(frozen, module = "arris")]
#[derive(Clone, Debug)]
pub struct UncheckedRow {
    model: Option<Arc<Shared>>,
    kernel: Unchecked,
}

#[pymethods]
impl UncheckedRow {
    /// The invariant's row.
    #[getter]
    fn code(&self) -> &'static str {
        self.kernel.code()
    }

    /// The entity the row is reported against.
    #[getter]
    fn entity<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        name(py, &self.model, self.kernel.entity())
    }

    fn __str__(&self) -> String {
        self.kernel.to_string()
    }

    fn __repr__(&self) -> String {
        format!("UncheckedRow({})", self.kernel)
    }
}

/// What the checker says of a body.
///
/// Truthy when nothing is violated; `violations` are in the checker's
/// deterministic order (by entity, then row, then content), so two runs
/// over the same model print the same report byte for byte. A `Full` row
/// the kernel could not decide is in `unchecked`, not a silent pass.
///
/// ```python
/// import arris
///
/// model = arris.Model()
/// body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
/// report = model.check(body, "full")
/// assert report and report.violations == [] and report.euler.genus == 0
/// ```
#[pyclass(frozen, module = "arris")]
#[derive(Clone, Debug)]
pub struct Report {
    model: Option<Arc<Shared>>,
    kernel: KernelReport,
}

impl Report {
    /// `kernel`, a report about a body of `model` (`None` for ids that
    /// belong to no model the caller holds).
    pub fn new(model: Option<&Model>, kernel: KernelReport) -> Report {
        Report {
            model: model.map(|m| Arc::clone(&m.shared)),
            kernel,
        }
    }
}

#[pymethods]
impl Report {
    /// Whether nothing was violated.
    #[getter]
    fn ok(&self) -> bool {
        self.kernel.is_ok()
    }

    /// The violations, in report order.
    #[getter]
    fn violations(&self) -> Vec<Violation> {
        self.kernel
            .violations()
            .iter()
            .map(|kernel| Violation {
                model: self.model.clone(),
                kernel: kernel.clone(),
            })
            .collect()
    }

    /// The `Full` rows that could not be decided.
    #[getter]
    fn unchecked(&self) -> Vec<UncheckedRow> {
        self.kernel
            .unchecked()
            .iter()
            .map(|kernel| UncheckedRow {
                model: self.model.clone(),
                kernel: *kernel,
            })
            .collect()
    }

    /// The Euler–Poincaré line, or `None` when the body did not resolve.
    #[getter]
    fn euler(&self) -> Option<EulerLine> {
        self.kernel.euler().map(|kernel| EulerLine { kernel })
    }

    fn __bool__(&self) -> bool {
        self.kernel.is_ok()
    }

    fn __len__(&self) -> usize {
        self.kernel.len()
    }

    fn __str__(&self) -> String {
        self.kernel.to_string()
    }

    fn __repr__(&self) -> String {
        format!(
            "Report(violations={}, unchecked={})",
            self.kernel.len(),
            self.kernel.unchecked().len()
        )
    }
}

fn level_name(level: Level) -> &'static str {
    match level {
        Level::Fast => "fast",
        Level::Full => "full",
    }
}

/// The checker's level named by `text`: `"fast"` or `"full"`.
pub(crate) fn level_of(text: &str) -> PyResult<Level> {
    match text {
        "fast" => Ok(Level::Fast),
        "full" => Ok(Level::Full),
        other => Err(PyValueError::new_err(format!(
            "{other:?} is not a check level (\"fast\" or \"full\")"
        ))),
    }
}
