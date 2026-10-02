//! `Provenance`: what an operation generated, modified and deleted, read
//! from Python.

use std::sync::Arc;

use arris::topo::Shape;
use arris::topo::provenance::{Origin as KernelOrigin, Provenance as Kernel};
use pyo3::IntoPyObjectExt;
use pyo3::prelude::*;
use pyo3::types::{PyList, PyTuple};

use crate::error::BindError;
use crate::handle::AnyHandle;
use crate::kernel_error::entity;
use crate::model::{Model, Shared};
use crate::role::Role;

/// The record an operation returns: for every origin, the entities it
/// generated and the pieces it modified, and the inputs that have no image
/// in the result.
///
/// An origin is an entity of an input body (a handle) or a [`Role`] for an
/// operation that makes a body from nothing. The outputs of one origin are
/// in the kernel's split order, which is a contract: piece `k` of a split
/// face means the same piece after an edit that keeps which entities bound
/// which piece. An entity an operation leaves alone is in no list.
#[pyclass(frozen, eq, module = "arris")]
#[derive(Clone, Debug)]
pub struct Provenance {
    model: Arc<Shared>,
    kernel: Kernel,
}

impl PartialEq for Provenance {
    fn eq(&self, other: &Self) -> bool {
        self.model.serial() == other.model.serial() && self.kernel == other.kernel
    }
}

/// An origin as the calls that take one read it: an entity of the model, or
/// a role.
#[derive(FromPyObject)]
enum AnyOrigin<'py> {
    Role(PyRef<'py, Role>),
    Entity(AnyHandle<'py>),
}

impl Provenance {
    /// `kernel`, a record of operations on `model`.
    pub fn minted_by(model: &Model, kernel: Kernel) -> Self {
        Provenance {
            model: Arc::clone(&model.shared),
            kernel,
        }
    }

    /// The record the kernel holds.
    pub fn kernel(&self) -> &Kernel {
        &self.kernel
    }

    fn origin(&self, origin: &AnyOrigin<'_>) -> Result<KernelOrigin, BindError> {
        Ok(match origin {
            AnyOrigin::Role(role) => KernelOrigin::Role(role.kernel),
            AnyOrigin::Entity(handle) => {
                let (id, orientation) = handle.resolve_shape(&self.model)?;
                KernelOrigin::Entity(Shape::new(id, orientation))
            }
        })
    }

    fn py_origin<'py>(&self, py: Python<'py>, origin: KernelOrigin) -> PyResult<Bound<'py, PyAny>> {
        match origin {
            KernelOrigin::Entity(shape) => self.py_shape(py, shape),
            KernelOrigin::Role(role) => Role::new(role).into_bound_py_any(py),
        }
    }

    fn py_shape<'py>(&self, py: Python<'py>, shape: Shape) -> PyResult<Bound<'py, PyAny>> {
        entity(py, Some(&Model::sharing(&self.model)), shape)
    }

    fn py_shapes<'py>(&self, py: Python<'py>, shapes: &[Shape]) -> PyResult<Bound<'py, PyList>> {
        let items = shapes
            .iter()
            .map(|s| self.py_shape(py, *s))
            .collect::<PyResult<Vec<_>>>()?;
        PyList::new(py, items)
    }

    /// `(origin, outputs)` pairs for `records`, in origin order.
    fn pairs<'py, 'a>(
        &self,
        py: Python<'py>,
        records: impl Iterator<Item = (KernelOrigin, &'a [Shape])>,
    ) -> PyResult<Bound<'py, PyList>> {
        let items = records
            .map(|(origin, outputs)| {
                PyTuple::new(
                    py,
                    [
                        self.py_origin(py, origin)?,
                        self.py_shapes(py, outputs)?.into_any(),
                    ],
                )
            })
            .collect::<PyResult<Vec<_>>>()?;
        PyList::new(py, items)
    }

    fn recorded(&self, generated: bool) -> Vec<(KernelOrigin, &[Shape])> {
        self.kernel
            .origins_recorded()
            .filter_map(|origin| {
                let outputs = if generated {
                    self.kernel.generated_from(origin)
                } else {
                    self.kernel.modified_from(origin)
                };
                (!outputs.is_empty()).then_some((origin, outputs))
            })
            .collect()
    }
}

#[pymethods]
impl Provenance {
    /// What was generated: a list of `(origin, outputs)`, origins in the
    /// kernel's order (entities before roles) and each origin's outputs in
    /// split order.
    #[getter]
    fn generated<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        self.pairs(py, self.recorded(true).into_iter())
    }

    /// What was modified: a list of `(origin, pieces)`, in the same orders.
    #[getter]
    fn modified<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        self.pairs(py, self.recorded(false).into_iter())
    }

    /// The inputs with no image in the result, ascending.
    #[getter]
    fn deleted<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let shapes: Vec<Shape> = self.kernel.deleted().collect();
        self.py_shapes(py, &shapes)
    }

    /// The outputs generated from `origin` (a handle or a `Role`), in split
    /// order; empty when there are none. Raises `ForeignHandleError` for a
    /// handle of another model.
    fn generated_from<'py>(
        &self,
        py: Python<'py>,
        origin: AnyOrigin<'py>,
    ) -> PyResult<Bound<'py, PyList>> {
        let origin = self.origin(&origin)?;
        self.py_shapes(py, self.kernel.generated_from(origin))
    }

    /// The pieces of `origin` (a handle or a `Role`), in split order.
    fn modified_from<'py>(
        &self,
        py: Python<'py>,
        origin: AnyOrigin<'py>,
    ) -> PyResult<Bound<'py, PyList>> {
        let origin = self.origin(&origin)?;
        self.py_shapes(py, self.kernel.modified_from(origin))
    }

    /// Whether `entity` was deleted: it has no image in the result.
    fn is_deleted(&self, entity: AnyHandle<'_>) -> PyResult<bool> {
        let (id, orientation) = entity.resolve_shape(&self.model)?;
        Ok(self.kernel.is_deleted(Shape::new(id, orientation)))
    }

    /// Every `(relation, origin)` that `output` came from, ascending by
    /// origin. `relation` is `"generated"` or `"modified"`.
    fn origins<'py>(
        &self,
        py: Python<'py>,
        output: AnyHandle<'py>,
    ) -> PyResult<Bound<'py, PyList>> {
        let (id, orientation) = output.resolve_shape(&self.model)?;
        let items = self
            .kernel
            .origins(Shape::new(id, orientation))
            .into_iter()
            .map(|(relation, origin)| {
                PyTuple::new(
                    py,
                    [
                        relation.to_string().into_bound_py_any(py)?,
                        self.py_origin(py, origin)?,
                    ],
                )
            })
            .collect::<PyResult<Vec<_>>>()?;
        PyList::new(py, items)
    }

    /// This record followed by `next`, reported against this record's
    /// inputs: a chain of operations reads as one. Raises
    /// `ForeignHandleError` when the records are of different models.
    fn then(&self, next: &Provenance) -> PyResult<Provenance> {
        if self.model.serial() != next.model.serial() {
            return Err(BindError::ForeignRecord {
                owner: next.model.serial(),
                this: self.model.serial(),
            }
            .into());
        }
        Ok(Provenance {
            model: Arc::clone(&self.model),
            kernel: self.kernel.then(&next.kernel),
        })
    }

    /// Whether nothing was recorded.
    fn __bool__(&self) -> bool {
        !self.kernel.is_empty()
    }

    /// One line per record, `<origin> generated <outputs…>`, then
    /// `<input> deleted`.
    fn __str__(&self) -> String {
        self.kernel.to_string()
    }

    fn __repr__(&self) -> String {
        format!(
            "Provenance(model={}, generated={}, modified={}, deleted={})",
            self.model.serial(),
            self.recorded(true).len(),
            self.recorded(false).len(),
            self.kernel.deleted().count()
        )
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use arris::Control;
    use arris::math::{Axis, Point3};
    use arris::topo::provenance::{BoxPart, Coord, CylinderPart, Role as KernelRole, Side};
    use arris::topo::{EntityId, Model as KernelModel};

    use super::*;
    use crate::role::{self, Field};

    /// The bolt-pattern recipe (`boolean/bolt-pattern-8`): a plate and eight
    /// tools on a circle, each cut in turn, and the record of the whole
    /// chain composed against the primitives.
    fn bolt_pattern() -> (Model, arris::topo::Body, Provenance) {
        let mut kernel = KernelModel::default();
        let control = Control::NONE;
        let (plate, mut whole) =
            arris::ops::primitive_box(&mut kernel, [0.0; 3], [100.0, 100.0, 10.0], &control)
                .unwrap();
        let mut tools = Vec::new();
        for i in 0..8 {
            let angle = f64::from(i) * std::f64::consts::FRAC_PI_4;
            let base = Point3::new(50.0 + 35.0 * angle.cos(), 50.0 + 35.0 * angle.sin(), -1.0);
            let (tool, made) =
                arris::ops::primitive_cylinder(&mut kernel, Axis::z_at(base), 3.0, 12.0, &control)
                    .unwrap();
            whole = whole.then(&made);
            tools.push(tool);
        }
        let mut body = plate;
        for tool in tools {
            let (next, cut) = arris::ops::cut(&mut kernel, body, tool, &control).unwrap();
            whole = whole.then(&cut);
            body = next;
        }
        let model = Model::wrapping(kernel);
        let record = Provenance::minted_by(&model, whole);
        (model, body, record)
    }

    #[test]
    fn eight_hole_walls_sit_under_one_origin() {
        let (model, body, record) = bolt_pattern();
        let wall = KernelRole::Cylinder(CylinderPart::Wall);
        let generated = record.recorded(true);
        let (_, outputs) = generated
            .iter()
            .find(|(origin, _)| *origin == KernelOrigin::Role(wall))
            .expect("the cylinder wall role generated the hole walls");
        // The role generates the hole edges too (two seam and rim edges per
        // wall); the walls are its faces.
        let walls: Vec<Shape> = outputs
            .iter()
            .filter(|s| matches!(s.id, EntityId::Face(_)))
            .copied()
            .collect();
        assert_eq!(role::view(wall), ("cylinder", "Wall", vec![]));
        assert_eq!(walls.len(), 8, "{walls:?}");
        let distinct: BTreeSet<_> = walls.iter().map(|s| s.id).collect();
        assert_eq!(distinct.len(), 8);

        // Each is a face of the result, with no origin but that role.
        let kernel = model.shared.lock().unwrap();
        let closure = kernel.closure(body).unwrap();
        for shape in &walls {
            let EntityId::Face(face) = shape.id else {
                panic!("{shape} is not a face");
            };
            assert!(
                closure.faces.contains(&face),
                "{shape} is not in the result"
            );
            assert_eq!(
                record.kernel.origins(*shape),
                [(
                    arris::topo::provenance::Relation::Generated,
                    KernelOrigin::Role(wall)
                )]
            );
        }

        // The plate's top face is a role too, and the roles read as parts.
        let top = KernelRole::Box(BoxPart::Face(Coord::Z, Side::Max));
        assert_eq!(
            role::view(top),
            ("box", "Face", vec![Field::Name("Z"), Field::Name("Max")])
        );
        assert!(!record.kernel.generated_from(top).is_empty());
    }

    #[test]
    fn records_of_different_models_do_not_combine() {
        let (a, _, in_a) = bolt_pattern();
        let (b, _, in_b) = bolt_pattern();
        assert_ne!(a.shared.serial(), b.shared.serial());
        assert_ne!(in_a, in_b, "equal records of different models differ");
        let refused = BindError::ForeignRecord {
            owner: b.shared.serial(),
            this: a.shared.serial(),
        };
        assert_eq!(refused.class(), crate::error::Class::ForeignHandleError);
        assert!(refused.to_string().contains("cannot be combined"));
    }
}
