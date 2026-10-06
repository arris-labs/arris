//! `Model`: the arena behind a lock, and the identity its handles carry.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};

use arris::math::nalgebra::UnitQuaternion;
use arris::math::{Axis, Frame as KernelFrame, Isometry, Point3, Reflection, UnitVec3, Vec3};
use arris::ops::{InputReason, OpError, Reason, ShellSide};
use arris::topo;
use arris::{Control, Interrupted, Stop};
use pyo3::exceptions::PyKeyboardInterrupt;
use pyo3::prelude::*;

use crate::control::{Cancel, Limits};
use crate::error::BindError;
use crate::handle::{AnyHandle, Body, Edge, Face, Shell, Vertex};
use crate::io::{Imported, StepRead, length_unit as length_unit_of};
use crate::kernel_error::{
    Mapped, body_error, mesh_error, native_error, op_error, step_error, step_read_error,
};
use crate::mesh::Mesh;
use crate::profile::Profile;
use crate::provenance::Provenance;
use crate::query::{Frame, MassProperties, Report, level_of};

/// Serial numbers for models, in creation order within the process: what a
/// handle compares and hashes by, and what an error names.
static SERIALS: AtomicU64 = AtomicU64::new(0);

/// The kernel's model, the lock a long operation releases the GIL under,
/// and the serial that tells one model's ids from another's: the kernel's
/// ids are plain `(slot, generation)` and carry none (ADR-0034 §6).
#[derive(Debug)]
pub(crate) struct Shared {
    serial: u64,
    model: Mutex<topo::Model>,
}

impl Shared {
    pub(crate) fn new(model: topo::Model) -> Arc<Shared> {
        Arc::new(Shared {
            serial: SERIALS.fetch_add(1, Ordering::Relaxed),
            model: Mutex::new(model),
        })
    }

    pub(crate) fn serial(&self) -> u64 {
        self.serial
    }

    /// The kernel model, locked. A lock poisoned by a panic is an error: the
    /// panic may have left the arena between two states.
    pub(crate) fn lock(&self) -> Result<MutexGuard<'_, topo::Model>, BindError> {
        self.model
            .lock()
            .map_err(|_| BindError::Poisoned { model: self.serial })
    }
}

/// An arena of geometry and topology, and the operations that append to it.
///
/// Every handle an operation returns carries the model it came from: a
/// handle given to a different model raises `ForeignHandleError` before it
/// reaches the kernel, and one whose entity was freed raises
/// `StaleHandleError`.
///
/// ```python
/// import arris
///
/// a, b = arris.Model(), arris.Model()
/// assert a == a and a != b
/// ```
#[pyclass(frozen, eq, hash, module = "arris")]
#[derive(Clone, Debug)]
pub struct Model {
    pub(crate) shared: Arc<Shared>,
}

impl PartialEq for Model {
    fn eq(&self, other: &Model) -> bool {
        self.shared.serial == other.shared.serial
    }
}

impl std::hash::Hash for Model {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.shared.serial.hash(state);
    }
}

impl Model {
    /// The model that owns `shared`.
    pub(crate) fn sharing(shared: &Arc<Shared>) -> Model {
        Model {
            shared: Arc::clone(shared),
        }
    }

    /// A model around `model`, with a serial of its own.
    pub fn wrapping(model: topo::Model) -> Model {
        Model {
            shared: Shared::new(model),
        }
    }

    /// Whether `entity` resolves in this model: `false` for a freed slot or an
    /// older generation.
    pub(crate) fn holds(&self, entity: topo::EntityId) -> Result<bool, BindError> {
        let model = self.shared.lock()?;
        Ok(match entity {
            topo::EntityId::Vertex(id) => model.vertex(id).is_ok(),
            topo::EntityId::Edge(id) => model.edge(id).is_ok(),
            topo::EntityId::Face(id) => model.face(id).is_ok(),
            topo::EntityId::Shell(id) => model.shell(id).is_ok(),
            topo::EntityId::Body(id) => model.body(id).is_ok(),
        })
    }

    /// Frees everything not reachable from `keep`; the count freed.
    pub(crate) fn retaining(&self, keep: &[Body]) -> Result<usize, BindError> {
        let keep = keep
            .iter()
            .map(|body| body.resolve(&self.shared))
            .collect::<Result<Vec<_>, _>>()?;
        let mut model = self.shared.lock()?;
        Ok(model.retain(&keep)?)
    }
}

/// An operation's result in Python: the body and its record.
pub type Made = (Body, Provenance);

/// Why a call did not produce a body: the binding refused it, or the kernel
/// did.
enum Failure<E> {
    Bind(BindError),
    Kernel(E),
}

impl<E> From<BindError> for Failure<E> {
    fn from(e: BindError) -> Self {
        Failure::Bind(e)
    }
}

/// An argument no kernel call could take, as the kernel's own refusal.
fn refused(reason: Reason) -> OpError {
    OpError::Degenerate {
        entities: Vec::new(),
        reason,
    }
}

fn finite(what: &'static str, v: [f64; 3]) -> Result<(), OpError> {
    if v.iter().all(|c| c.is_finite()) {
        Ok(())
    } else {
        Err(refused(Reason::Input(InputReason::NonFinite { what })))
    }
}

/// A direction from three numbers: finite, and not zero.
fn direction_of(what: &'static str, v: [f64; 3]) -> Result<Vec3, OpError> {
    finite(what, v)?;
    let v = Vec3::from(v);
    if v.norm() > 0.0 {
        Ok(v)
    } else {
        Err(refused(Reason::Input(InputReason::NotPositive {
            what,
            value: 0.0,
        })))
    }
}

impl Model {
    /// Runs one kernel call on this model with the GIL released and
    /// `limits` enforced.
    ///
    /// The model's lock is held for the call, so two threads' calls on one
    /// model run one after the other. A stop leaves the model as it was (the
    /// kernel rolls back); Ctrl-C is a `KeyboardInterrupt` and the other
    /// stops are `Interrupted`.
    fn run<T: Send>(
        &self,
        py: Python<'_>,
        limits: &Limits,
        call: impl FnOnce(&mut topo::Model, &Control<'_>) -> Result<T, OpError> + Send,
    ) -> PyResult<T> {
        self.run_with(py, limits, call, op_error, true, |error| {
            matches!(
                error,
                OpError::Interrupted(Interrupted { by: Stop::Poll, .. })
            )
        })
    }

    /// [`Model::run`] for any kernel error type: `map` says how `E` is
    /// raised, `own_ids` whether the ids it names are this model's (false for
    /// a read of bytes, whose errors name the writer's) and `by_poll`
    /// whether it is a stop by the poll, which Ctrl-C answers.
    fn run_with<T: Send, E: Send>(
        &self,
        py: Python<'_>,
        limits: &Limits,
        call: impl FnOnce(&mut topo::Model, &Control<'_>) -> Result<T, E> + Send,
        map: impl FnOnce(&E) -> Mapped,
        own_ids: bool,
        by_poll: impl FnOnce(&E) -> bool,
    ) -> PyResult<T> {
        let (outcome, signalled) = py.detach(|| {
            limits.run(|control| {
                let mut kernel = self.shared.lock()?;
                call(&mut kernel, control).map_err(Failure::Kernel)
            })
        });
        match outcome {
            Ok(value) => Ok(value),
            Err(Failure::Bind(error)) => Err(error.into()),
            Err(Failure::Kernel(error)) if signalled && by_poll(&error) => {
                Err(PyKeyboardInterrupt::new_err("interrupted by Ctrl-C"))
            }
            Err(Failure::Kernel(error)) => Err(map(&error).raise(py, own_ids.then_some(self))),
        }
    }

    /// Runs one kernel operation as [`Model::run`] does and mints its
    /// result.
    fn operate(
        &self,
        py: Python<'_>,
        limits: &Limits,
        call: impl FnOnce(
            &mut topo::Model,
            &Control<'_>,
        ) -> Result<(topo::Body, arris::topo::provenance::Provenance), OpError>
        + Send,
    ) -> PyResult<Made> {
        let (body, record) = self.run(py, limits, call)?;
        Ok((
            Body::minted_by(self, body),
            Provenance::minted_by(self, record),
        ))
    }

    /// An argument refusal, raised as the kernel's would be.
    fn refuse(&self, py: Python<'_>, error: OpError) -> PyErr {
        op_error(&error).raise(py, Some(self))
    }
}

#[pymethods]
impl Model {
    /// An empty model with the default precision.
    #[new]
    fn new() -> Model {
        Model::wrapping(topo::Model::default())
    }

    /// Whether `handle` still resolves in this model; raises
    /// `ForeignHandleError` for a handle another model minted.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// assert model.contains(body)
    /// try:
    ///     arris.Model().contains(body)
    /// except arris.ForeignHandleError:
    ///     pass
    /// else:
    ///     raise AssertionError("another model refuses the handle")
    /// ```
    fn contains(&self, handle: AnyHandle<'_>) -> PyResult<bool> {
        Ok(self.holds(handle.resolve(&self.shared)?)?)
    }

    /// Frees every entity not reachable from the bodies in `keep`, and
    /// returns how many it freed. A kept entity's handle stays valid; the
    /// others go stale (ids are never renumbered).
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// kept, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    /// dropped, _ = model.primitive_box((5, 5, 5), (6, 6, 6))
    /// assert model.retain([kept]) > 0
    /// assert model.contains(kept) and not model.contains(dropped)
    /// ```
    fn retain(&self, py: Python<'_>, keep: Vec<PyRef<'_, Body>>) -> PyResult<usize> {
        let keep: Vec<Body> = keep.iter().map(|body| (**body).clone()).collect();
        Ok(py.detach(|| self.retaining(&keep))?)
    }

    /// A box with corners `min` and `max`, every face, edge and vertex
    /// named by a `Role` in the returned `Provenance`.
    ///
    /// Raises `OpDegenerateError` for a corner that is not finite or an
    /// extent that is not positive.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// assert abs(model.mass_properties(body).volume - 6) < 1e-12
    /// assert record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    /// ```
    #[pyo3(signature = (min, max, *, cancel=None, budget=None))]
    fn primitive_box(
        &self,
        py: Python<'_>,
        min: [f64; 3],
        max: [f64; 3],
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::primitive_box(m, min, max, control)
        })
    }

    /// A cylinder whose bottom cap is centred on `origin`, rising `height`
    /// along `axis` with the given `radius`.
    ///
    /// Raises `OpDegenerateError` for a radius or height that is not
    /// positive or an axis that is not finite or is zero.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_cylinder((0, 0, 0), (0, 0, 1), 2, 5)
    /// assert abs(model.mass_properties(body).volume - math.pi * 4 * 5) < 1e-9
    /// ```
    #[pyo3(signature = (origin, axis, radius, height, *, cancel=None, budget=None))]
    #[allow(clippy::too_many_arguments)] // the keywords are the Python signature
    fn primitive_cylinder(
        &self,
        py: Python<'_>,
        origin: [f64; 3],
        axis: [f64; 3],
        radius: f64,
        height: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let axis = finite("axis origin", origin)
            .and_then(|()| direction_of("axis direction", axis))
            .and_then(|d| Axis::new(Point3::from(origin), d).map_err(OpError::from))
            .map_err(|e| self.refuse(py, e))?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::primitive_cylinder(m, axis, radius, height, control)
        })
    }

    /// A copy of `body` turned by `angle` radians about the line through
    /// the origin along `axis`, then moved by `translation`.
    ///
    /// Raises `OpDegenerateError` for a number that is not finite or an
    /// axis of zero length; `ForeignHandleError` for a body of another model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// moved, _ = model.transform(body, (10, 0, 0))
    /// assert abs(model.mass_properties(moved).centroid[0] - 10.5) < 1e-12
    /// ```
    #[pyo3(signature = (body, translation=[0.0; 3], *, axis=[0.0, 0.0, 1.0], angle=0.0, cancel=None, budget=None))]
    #[allow(clippy::too_many_arguments)] // the keywords are the Python signature
    fn transform(
        &self,
        py: Python<'_>,
        body: &Body,
        translation: [f64; 3],
        axis: [f64; 3],
        angle: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let body = body.resolve(&self.shared)?;
        let motion = finite("translation", translation)
            .and_then(|()| direction_of("rotation axis", axis))
            .and_then(|a| {
                if angle.is_finite() {
                    let turn = UnitQuaternion::from_axis_angle(&UnitVec3::new_normalize(a), angle);
                    Ok(Isometry::new(turn, Vec3::from(translation)))
                } else {
                    Err(refused(Reason::Input(InputReason::NonFinite {
                        what: "angle",
                    })))
                }
            })
            .map_err(|e| self.refuse(py, e))?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::transform(m, body, &motion, control)
        })
    }

    /// A copy of `body` reflected in the plane through `origin` with the
    /// given `normal`; its faces face outward again.
    ///
    /// Raises `OpDegenerateError` for a coordinate that is not finite or a
    /// normal of zero length.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// mirrored, _ = model.mirror(body, (0, 0, 0), (1, 0, 0))
    /// assert abs(model.mass_properties(mirrored).centroid[0] + 0.5) < 1e-12
    /// assert abs(model.mass_properties(mirrored).volume - 6) < 1e-12
    /// ```
    #[pyo3(signature = (body, origin, normal, *, cancel=None, budget=None))]
    fn mirror(
        &self,
        py: Python<'_>,
        body: &Body,
        origin: [f64; 3],
        normal: [f64; 3],
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let body = body.resolve(&self.shared)?;
        let plane = finite("mirror origin", origin)
            .and_then(|()| direction_of("mirror normal", normal))
            .and_then(|n| {
                Reflection::new(Point3::from(origin), n).map_err(|_| {
                    refused(Reason::Input(InputReason::NonFinite {
                        what: "mirror plane",
                    }))
                })
            })
            .map_err(|e| self.refuse(py, e))?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::mirror(m, body, &plane, control)
        })
    }

    /// `target` with `tool` removed. The tool's faces that bound the new
    /// cavity are generated from; see the returned `Provenance`.
    ///
    /// Raises `OpUnsupportedError` (naming both operands) for a pair of
    /// surfaces with no closed form yet, `OpDegenerateError` when nothing
    /// would remain, and `Interrupted` when `cancel` or `budget` stops it.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// plate, _ = model.primitive_box((0, 0, 0), (10, 10, 10))
    /// tool, _ = model.primitive_cylinder((5, 5, -1), (0, 0, 1), 2, 12)
    /// holed, record = model.cut(plate, tool)
    /// volume = model.mass_properties(holed).volume
    /// assert abs(volume - (1000 - math.pi * 4 * 10)) < 1e-9
    /// assert record.generated
    /// ```
    #[pyo3(signature = (target, tool, *, cancel=None, budget=None))]
    fn cut(
        &self,
        py: Python<'_>,
        target: &Body,
        tool: &Body,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let (target, tool) = (target.resolve(&self.shared)?, tool.resolve(&self.shared)?);
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::cut(m, target, tool, control)
        })
    }

    /// The union of `a` and `b`. Errors as for `cut`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// a, _ = model.primitive_box((0, 0, 0), (2, 2, 2))
    /// b, _ = model.primitive_box((1, 0, 0), (3, 2, 2))
    /// union, _ = model.fuse(a, b)
    /// assert abs(model.mass_properties(union).volume - 12) < 1e-9
    /// ```
    #[pyo3(signature = (a, b, *, cancel=None, budget=None))]
    fn fuse(
        &self,
        py: Python<'_>,
        a: &Body,
        b: &Body,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let (a, b) = (a.resolve(&self.shared)?, b.resolve(&self.shared)?);
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::fuse(m, a, b, control)
        })
    }

    /// `target` with every body of `tools` removed, in one decomposition: the
    /// result of cutting them one after another, made by one pass over all of
    /// them, with one provenance record that names each tool. A pattern of
    /// holes is one call. The tools may overlap one another.
    ///
    /// Raises `OpDegenerateError` for an empty `tools` or a body named twice
    /// among the target and the tools, `OpUnsupportedError` for tools that
    /// touch one another where no result is defined (flush on one face,
    /// tangent), and otherwise as for `cut`.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// plate, _ = model.primitive_box((0, 0, 0), (40, 30, 10))
    /// tools = [
    ///     model.primitive_cylinder((x, 15, -1), (0, 0, 1), 3, 12)[0]
    ///     for x in (10, 20, 30)
    /// ]
    /// holed, record = model.cut_many(plate, tools)
    /// volume = model.mass_properties(holed).volume
    /// assert abs(volume - (12000 - 3 * math.pi * 9 * 10)) < 1e-9 * 12000
    /// assert record.generated
    /// ```
    #[pyo3(signature = (target, tools, *, cancel=None, budget=None))]
    fn cut_many(
        &self,
        py: Python<'_>,
        target: &Body,
        tools: Vec<PyRef<'_, Body>>,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let target = target.resolve(&self.shared)?;
        let tools = tools
            .iter()
            .map(|t| t.resolve(&self.shared))
            .collect::<Result<Vec<_>, _>>()?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::cut_many(m, target, &tools, control)
        })
    }

    /// The union of every body of `bodies`, in one decomposition: the result
    /// of fusing them one after another, with one provenance record that
    /// names each. Errors as for `cut_many`; fewer than two bodies is
    /// `OpDegenerateError`.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// plate, _ = model.primitive_box((0, 0, 0), (60, 30, 10))
    /// bosses = [
    ///     model.primitive_cylinder((x, 15, 5), (0, 0, 1), 4, 15)[0]
    ///     for x in (15, 45)
    /// ]
    /// fused, _ = model.fuse_many([plate, *bosses])
    /// volume = model.mass_properties(fused).volume
    /// assert abs(volume - (18000 + 2 * math.pi * 16 * 10)) < 1e-9 * 18000
    /// ```
    #[pyo3(signature = (bodies, *, cancel=None, budget=None))]
    fn fuse_many(
        &self,
        py: Python<'_>,
        bodies: Vec<PyRef<'_, Body>>,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let bodies = bodies
            .iter()
            .map(|b| b.resolve(&self.shared))
            .collect::<Result<Vec<_>, _>>()?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::fuse_many(m, &bodies, control)
        })
    }

    /// The intersection of `a` and `b`. Errors as for `cut`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// a, _ = model.primitive_box((0, 0, 0), (2, 2, 2))
    /// b, _ = model.primitive_box((1, 0, 0), (3, 2, 2))
    /// overlap, _ = model.common(a, b)
    /// assert abs(model.mass_properties(overlap).volume - 4) < 1e-9
    /// ```
    #[pyo3(signature = (a, b, *, cancel=None, budget=None))]
    fn common(
        &self,
        py: Python<'_>,
        a: &Body,
        b: &Body,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let (a, b) = (a.resolve(&self.shared)?, b.resolve(&self.shared)?);
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::common(m, a, b, control)
        })
    }

    /// `body` cut by the plane through `origin` with the given `normal` into
    /// the solid on each side: `(positive, negative, record)`, `positive`
    /// the one the normal points to. Either may have several lumps. The cut
    /// faces are generated from `Role("split", "Cap", "Positive")` and
    /// `Role("split", "Cap", "Negative")`; their (u, v) are the plane's `x`
    /// direction (any perpendicular of the normal when `x` is `None`) and its
    /// cross product with the normal. Every entity the plane does not touch
    /// keeps its id on its side.
    ///
    /// Raises `OpDegenerateError` for a coordinate that is not finite, a
    /// normal of zero length or an `x` with no part across it, and for a plane that
    /// misses the body or only touches it; `OpUnsupportedError` and
    /// `OpDegenerateError` as for `cut` where the plane meets the body as a
    /// boolean would refuse; `Interrupted` when `cancel` or `budget` stops it.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// block, _ = model.primitive_box((0, 0, 0), (10, 10, 10))
    /// above, below, record = model.split(block, (0, 0, 4), (0, 0, 1))
    /// assert abs(model.mass_properties(above).volume - 600) < 1e-9
    /// assert abs(model.mass_properties(below).volume - 400) < 1e-9
    /// assert record.generated_from(arris.Role("split", "Cap", "Positive"))
    /// ```
    #[pyo3(signature = (body, origin, normal, x=None, *, cancel=None, budget=None))]
    #[allow(clippy::too_many_arguments)] // the keywords are the Python signature
    fn split(
        &self,
        py: Python<'_>,
        body: &Body,
        origin: [f64; 3],
        normal: [f64; 3],
        x: Option<[f64; 3]>,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<(Body, Body, Provenance)> {
        let body = body.resolve(&self.shared)?;
        let plane = finite("split origin", origin)
            .and_then(|()| direction_of("split normal", normal))
            .and_then(|n| match x {
                None => KernelFrame::from_z(Point3::from(origin), n).map_err(|_| {
                    refused(Reason::Input(InputReason::NonFinite {
                        what: "split plane",
                    }))
                }),
                Some(x) => finite("split x", x).and_then(|()| {
                    KernelFrame::new(Point3::from(origin), n, Vec3::from(x)).map_err(|_| {
                        refused(Reason::Input(InputReason::NotPositive {
                            what: "split x's part across the normal",
                            value: 0.0,
                        }))
                    })
                }),
            })
            .map_err(|e| self.refuse(py, e))?;
        let limits = Limits::new(cancel, budget);
        let halves = self.run(py, &limits, move |m, control| {
            arris::ops::split(m, body, &plane, control)
        })?;
        Ok((
            Body::minted_by(self, halves.positive),
            Body::minted_by(self, halves.negative),
            Provenance::minted_by(self, halves.provenance),
        ))
    }

    /// The solid `profile` sweeps along `direction` for `length`: caps from
    /// the profile's face and a side face for each of its segments, every
    /// entity named by the part of the sketch it came from
    /// (`Role("extrude", "Side", loop, segment)`).
    ///
    /// Raises `OpProfileError` for an invalid sketch and `OpDegenerateError`
    /// for a length that is not positive or finite or a direction that is
    /// not along the profile's normal.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// sketch = arris.Profile(arris.Loop.polygon([(0, 0), (4, 0), (4, 3), (0, 3)]))
    /// body, record = model.extrude(sketch, (0, 0, 1), 2)
    /// assert abs(model.mass_properties(body).volume - 24) < 1e-9
    /// assert record.generated_from(arris.Role("extrude", "Side", 0, 0))
    /// ```
    #[pyo3(signature = (profile, direction, length, *, cancel=None, budget=None))]
    fn extrude(
        &self,
        py: Python<'_>,
        profile: &Profile,
        direction: [f64; 3],
        length: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let direction =
            direction_of("extrude direction", direction).map_err(|e| self.refuse(py, e))?;
        let (profile, limits) = (profile.kernel.clone(), Limits::new(cancel, budget));
        self.operate(py, &limits, move |m, control| {
            arris::ops::extrude(m, &profile, direction, length, control)
        })
    }

    /// The solid `profile` sweeps turning by `angle` radians (a full turn
    /// is `2π`) about the line through `origin` along `axis`; the axis must
    /// lie in the profile's plane.
    ///
    /// Raises `OpProfileError` for an invalid sketch and `OpDegenerateError`
    /// for an axis off the plane, a profile across the axis, or an angle
    /// that is not in `(0, 2π]`.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// sketch = arris.Profile(arris.Loop.polygon([(1, 0), (2, 0), (2, 1), (1, 1)]))
    /// body, _ = model.revolve(sketch, (0, 0, 0), (0, 1, 0), 2 * math.pi)
    /// assert abs(model.mass_properties(body).volume - 3 * math.pi) < 1e-9
    /// ```
    #[pyo3(signature = (profile, origin, axis, angle, *, cancel=None, budget=None))]
    #[allow(clippy::too_many_arguments)] // the keywords are the Python signature
    fn revolve(
        &self,
        py: Python<'_>,
        profile: &Profile,
        origin: [f64; 3],
        axis: [f64; 3],
        angle: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let axis = finite("axis origin", origin)
            .and_then(|()| direction_of("axis direction", axis))
            .and_then(|d| Axis::new(Point3::from(origin), d).map_err(OpError::from))
            .map_err(|e| self.refuse(py, e))?;
        let (profile, limits) = (profile.kernel.clone(), Limits::new(cancel, budget));
        self.operate(py, &limits, move |m, control| {
            arris::ops::revolve(m, &profile, axis, angle, control)
        })
    }

    /// `body` with the given `edges` rounded to `radius`.
    ///
    /// Raises `OpDegenerateError`, `OpUnsupportedError` or `OpToleranceError`
    /// for an edge set the blend network cannot build, and
    /// `ForeignHandleError` for an edge of another model.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// cube, record = model.primitive_box((0, 0, 0), (2, 2, 2))
    /// edge = record.generated_from(arris.Role("box", "Edge", "Z", "Max", "Max"))
    /// rounded, _ = model.fillet(cube, [e for e in edge if isinstance(e, arris.Edge)], 0.2)
    /// removed = (1 - math.pi / 4) * 0.2**2 * 2
    /// assert abs(model.mass_properties(rounded).volume - (8 - removed)) < 1e-9
    /// ```
    #[pyo3(signature = (body, edges, radius, *, cancel=None, budget=None))]
    fn fillet(
        &self,
        py: Python<'_>,
        body: &Body,
        edges: Vec<PyRef<'_, Edge>>,
        radius: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let body = body.resolve(&self.shared)?;
        let edges = self.edges_list(&edges)?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::fillet(m, body, &edges, radius, control)
        })
    }

    /// `body` with the given `edges` cut back by `distance` on each side.
    /// Errors as for `fillet`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// cube, record = model.primitive_box((0, 0, 0), (2, 2, 2))
    /// edge = record.generated_from(arris.Role("box", "Edge", "Z", "Max", "Max"))
    /// cut, _ = model.chamfer(cube, [e for e in edge if isinstance(e, arris.Edge)], 0.2)
    /// assert abs(model.mass_properties(cut).volume - (8 - 0.5 * 0.2**2 * 2)) < 1e-9
    /// ```
    #[pyo3(signature = (body, edges, distance, *, cancel=None, budget=None))]
    fn chamfer(
        &self,
        py: Python<'_>,
        body: &Body,
        edges: Vec<PyRef<'_, Edge>>,
        distance: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let body = body.resolve(&self.shared)?;
        let edges = self.edges_list(&edges)?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::chamfer(m, body, &edges, distance, control)
        })
    }

    /// `body` with the given `faces` moved along their outward normals by
    /// `distance`: positive adds material, negative removes it. Each moved
    /// face stays on the offset of its own surface, the faces beside it are
    /// extended or trimmed to meet it, and a face tangent to a moved one
    /// moves with it.
    ///
    /// Raises `OpDegenerateError` with the reason's name in `reason` for a
    /// move the kernel refuses (`NoFaces`, `RepeatedFace`, `FaceNotInBody`,
    /// `Vanishes`, `VertexSplits`, `NoExactOffset`, `SurfaceCollapses`,
    /// `Gap` or `SelfIntersects`) or a zero or non-finite `distance`,
    /// `OpUnsupportedError` for a neighbouring face it cannot place, and
    /// `ForeignHandleError` for a face of another model. The model is left
    /// as it was on every refusal.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// cube, record = model.primitive_box((0, 0, 0), (2, 2, 2))
    /// top = record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    /// taller, _ = model.offset_faces(cube, [f for f in top if isinstance(f, arris.Face)], 0.5)
    /// assert abs(model.mass_properties(taller).volume - 10) < 1e-9
    /// ```
    #[pyo3(signature = (body, faces, distance, *, cancel=None, budget=None))]
    fn offset_faces(
        &self,
        py: Python<'_>,
        body: &Body,
        faces: Vec<PyRef<'_, Face>>,
        distance: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let body = body.resolve(&self.shared)?;
        let faces = faces
            .iter()
            .map(|f| f.resolve(&self.shared))
            .collect::<Result<Vec<topo::Face>, BindError>>()?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::offset_faces(m, body, &faces, distance, control)
        })
    }

    /// `body` hollowed to a wall of `thickness`, open at `openings`. With
    /// `side="inward"` the body's faces stay the outside and the cavity
    /// grows inside them; with `side="outward"` the body's faces become the
    /// cavity and a skin grows outside them. The inner skin is every face
    /// but the openings moved by `thickness`, each on the offset of its own
    /// surface; each opening becomes a rim face between the two skins. No
    /// openings leaves a closed void: a body of two shells. Each skin face
    /// is generated from the face it copies.
    ///
    /// Raises `ValueError` for a `side` other than `"inward"` or
    /// `"outward"`; `OpDegenerateError` with the reason's name in `reason`
    /// for a shell the kernel refuses — the openings themselves
    /// (`RepeatedOpening`, `OpeningNotInBody`, `NoWalls`), an opening
    /// tangent to a wall (`OpeningDragged`), the walls' offset (`Vanishes`,
    /// `VertexSplits`, `NoExactOffset`, `SurfaceCollapses`, `Gap`, or
    /// `SelfIntersects` for a thickness past the thinnest wall) — or a
    /// non-positive or non-finite `thickness`; `OpUnsupportedError` for
    /// openings meeting in a way it cannot splice; and `ForeignHandleError`
    /// for a face of another model. The model is left as it was on every
    /// refusal.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// cube, record = model.primitive_box((0, 0, 0), (10, 10, 10))
    /// top = record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    /// cup, _ = model.shell(cube, [f for f in top if isinstance(f, arris.Face)], 1.0)
    /// assert abs(model.mass_properties(cup).volume - (1000 - 8 * 8 * 9)) < 1e-9
    /// assert len(model.faces(cup)) == 11
    /// ```
    #[pyo3(signature = (body, openings, thickness, side="inward", *, cancel=None, budget=None))]
    #[allow(clippy::too_many_arguments)] // the keywords are the Python signature
    fn shell(
        &self,
        py: Python<'_>,
        body: &Body,
        openings: Vec<PyRef<'_, Face>>,
        thickness: f64,
        side: &str,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let side = shell_side_of(side)?;
        let body = body.resolve(&self.shared)?;
        let openings = openings
            .iter()
            .map(|f| f.resolve(&self.shared))
            .collect::<Result<Vec<topo::Face>, BindError>>()?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::shell(m, body, &openings, thickness, side, control)
        })
    }

    /// The volume, area, centroid and inertia of the solid `body`, at unit
    /// density.
    ///
    /// Raises `OpDegenerateError` for a body that is not a solid and
    /// `Interrupted` when `cancel` or `budget` stops it.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// props = model.mass_properties(body)
    /// assert abs(props.volume - 6) < 1e-12 and abs(props.area - 22) < 1e-12
    /// assert props.centroid == (0.5, 1.0, 1.5)
    /// ```
    #[pyo3(signature = (body, *, cancel=None, budget=None))]
    fn mass_properties(
        &self,
        py: Python<'_>,
        body: &Body,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<MassProperties> {
        let body = body.resolve(&self.shared)?;
        let limits = Limits::new(cancel, budget);
        let kernel = self.run(py, &limits, move |m, control| {
            arris::ops::measure::mass_properties(m, body, control)
        })?;
        Ok(MassProperties::new(kernel))
    }

    /// The frame of a planar `face`: `z` is its outward normal, reversed
    /// for a face reached against its orientation.
    ///
    /// Raises `OpDegenerateError` for a face that is not planar.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// [top] = [f for f in record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    ///          if isinstance(f, arris.Face)]
    /// assert model.face_frame(top).z == (0.0, 0.0, 1.0)
    /// ```
    fn face_frame(&self, py: Python<'_>, face: &Face) -> PyResult<Frame> {
        let face = face.resolve(&self.shared)?;
        let kernel = arris::ops::query::face_frame(&*self.shared.lock()?, face)
            .map_err(|e| self.refuse(py, e))?;
        Ok(Frame::new(kernel))
    }

    /// The outward frame of `face`'s surface at `(u, v)`: `z` is the
    /// normal there, `x` the tangent along `u`.
    ///
    /// Raises `OpDegenerateError` for a point outside the face or at a
    /// singularity of its surface.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// [top] = [f for f in record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    ///          if isinstance(f, arris.Face)]
    /// frame = model.frame_at(top, 0.5, 0.5)
    /// assert frame.z == (0.0, 0.0, 1.0) and frame.origin[2] == 3.0
    /// ```
    fn frame_at(&self, py: Python<'_>, face: &Face, u: f64, v: f64) -> PyResult<Frame> {
        let face = face.resolve(&self.shared)?;
        let uv = arris::math::Point2::new(u, v);
        let kernel = arris::ops::query::frame_at(&*self.shared.lock()?, face, uv)
            .map_err(|e| self.refuse(py, e))?;
        Ok(Frame::new(kernel))
    }

    /// The checker's report on `body`: `level` is `"fast"` (linear in the
    /// body, what every operation asserts of its result in debug builds) or
    /// `"full"` (adds the global rows: face-face intersection, shell
    /// nesting, enclosed volume).
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// report = model.check(body, "full")
    /// assert report and not report.violations and not report.unchecked
    /// ```
    #[pyo3(signature = (body, level="fast"))]
    fn check(&self, py: Python<'_>, body: &Body, level: &str) -> PyResult<Report> {
        let (body, level) = (body.resolve(&self.shared)?, level_of(level)?);
        let report = py.detach(|| {
            let kernel = self.shared.lock()?;
            Ok::<_, BindError>(arris::check::check(&kernel, body, level))
        })?;
        Ok(Report::new(Some(self), report))
    }

    /// The triangle mesh of `body`: every position within `chord` of the
    /// surface it stands on, closed along shared edges, every triangle
    /// counter-clockwise seen from outside. `chord` is the caller's
    /// resolution, not a model tolerance: finite and positive.
    ///
    /// Raises `MeshChordError` for a bad chord, `MeshInvalidInputError` for
    /// a body that fails the checker (debug builds), `MeshFaceError` for a
    /// face that cannot be triangulated, and `Interrupted` when `cancel` or
    /// `budget` stops it.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// mesh = model.tessellate(body, 1e-3)
    /// assert mesh.is_closed() and mesh.n_triangles == 12
    /// assert abs(mesh.signed_volume() - 6) < 1e-9
    /// ```
    #[pyo3(signature = (body, chord, *, cancel=None, budget=None))]
    fn tessellate(
        &self,
        py: Python<'_>,
        body: &Body,
        chord: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Mesh> {
        let body = body.resolve(&self.shared)?;
        let limits = Limits::new(cancel, budget);
        let kernel = self.run_with(
            py,
            &limits,
            move |m, control| arris::mesh::tessellate(m, body, chord, control),
            mesh_error,
            true,
            |error| {
                matches!(
                    error,
                    arris::mesh::MeshError::Interrupted(Interrupted { by: Stop::Poll, .. })
                )
            },
        )?;
        Ok(Mesh::minted_by(self, kernel))
    }

    /// The triangle mesh of some of `body`'s faces, `faces` in any order:
    /// each edge's polyline and each face's triangles are exactly those
    /// `tessellate` gives, so meshes of different faces made at different
    /// times meet bit for bit and `arris.weld` joins them. The mesh holds
    /// the faces asked for, in the body's order, and the edges and vertices
    /// they use.
    ///
    /// Raises `MeshNotInBodyError` for a face the body does not have,
    /// `ForeignHandleError` for a face of another model, and what
    /// `tessellate` raises.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// faces = model.faces(body)
    /// mesh = model.tessellate_faces(body, faces[:2], 1e-3)
    /// assert mesh.n_triangles == 4 and not mesh.is_closed()
    /// ```
    #[pyo3(signature = (body, faces, chord, *, cancel=None, budget=None))]
    fn tessellate_faces(
        &self,
        py: Python<'_>,
        body: &Body,
        faces: Vec<PyRef<'_, Face>>,
        chord: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Mesh> {
        let body = body.resolve(&self.shared)?;
        let faces = faces
            .iter()
            .map(|f| f.resolve(&self.shared).map(|f| f.id))
            .collect::<Result<Vec<_>, BindError>>()?;
        let limits = Limits::new(cancel, budget);
        let kernel = self.run_with(
            py,
            &limits,
            move |m, control| {
                arris::mesh::tessellate_faces(
                    m,
                    body,
                    &faces,
                    &arris::mesh::MeshRequest::new(chord),
                    control,
                )
            },
            mesh_error,
            true,
            |error| {
                matches!(
                    error,
                    arris::mesh::MeshError::Interrupted(Interrupted { by: Stop::Poll, .. })
                )
            },
        )?;
        Ok(Mesh::minted_by(self, kernel))
    }

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

    /// The STEP text of `bodies`, in the order given, as one product with a
    /// solid entity per lump of each body (AP214, lengths in millimetres).
    /// Deterministic: the same bodies write the same text.
    ///
    /// Raises `StepUnsupportedError` for a body that is not a solid or has a
    /// form STEP cannot hold, `StepLumpsError` for shells that do not nest,
    /// `StepNoBodiesError` for an empty list and `ForeignHandleError` for a
    /// body of another model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// text = model.write_step([body])
    /// assert text.startswith("ISO-10303-21;") and "MANIFOLD_SOLID_BREP" in text
    /// ```
    fn write_step(&self, py: Python<'_>, bodies: Vec<PyRef<'_, Body>>) -> PyResult<String> {
        let bodies = bodies
            .iter()
            .map(|b| b.resolve(&self.shared))
            .collect::<Result<Vec<_>, _>>()?;
        py.detach(|| {
            let kernel = self.shared.lock()?;
            Ok::<_, BindError>(arris::io::step::write(&kernel, &bodies))
        })?
        .map_err(|e| step_error(&e).raise(py, Some(self)))
    }

    /// Reads every solid of the STEP file `text` into this model, lengths
    /// converted to `length_unit` (`"mm"` by default; also `"um"`, `"cm"`,
    /// `"m"`, `"in"`, `"ft"`).
    ///
    /// A solid the reader cannot read is a `StepSolid` with a `refusal`
    /// naming the file entity where it stopped and leaves nothing in the
    /// model; one refused solid never hides another. Raises `StepParseError`
    /// when the text is not Part 21 at all, `Interrupted` when `cancel` or
    /// `budget` stops it (the model is as it was) and `ValueError` for an
    /// unknown unit.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// back = arris.Model()
    /// [solid] = back.read_step(model.write_step([body])).solids
    /// assert solid.ok and abs(back.mass_properties(solid.body).volume - 6) < 1e-9
    /// ```
    #[pyo3(signature = (text, *, length_unit="mm", cancel=None, budget=None))]
    fn read_step(
        &self,
        py: Python<'_>,
        text: &str,
        length_unit: &str,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<StepRead> {
        let options = arris::io::step::ReadOptions {
            length_unit: length_unit_of(length_unit)?,
        };
        let limits = Limits::new(cancel, budget);
        let read = self.run_with(
            py,
            &limits,
            |m, control| arris::io::step::read(m, text, &options, control),
            step_read_error,
            true,
            |error| {
                matches!(
                    error,
                    arris::io::step::ReadError::Interrupted(Interrupted { by: Stop::Poll, .. })
                )
            },
        )?;
        Ok(StepRead::minted_by(self, read))
    }

    /// `body` and its `provenance` (empty by default) as body bytes: a
    /// self-contained, versioned record for storage, read by every later
    /// release. The record may name entities outside the body, such as a
    /// boolean's inputs.
    ///
    /// Raises `ForeignHandleError` for a body or record of another model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// data = model.write_body(body, record)
    /// assert data[:8] == b"ARRISBDY"
    /// ```
    #[pyo3(signature = (body, provenance=None))]
    fn write_body<'py>(
        &self,
        py: Python<'py>,
        body: &Body,
        provenance: Option<&Provenance>,
    ) -> PyResult<Bound<'py, pyo3::types::PyBytes>> {
        let bytes = self.write_body_with(py, body, provenance, |m, b, r| {
            arris::io::body::write(m, b, r)
        })?;
        Ok(pyo3::types::PyBytes::new(py, &bytes))
    }

    /// The same body as `write_body`, as one line of JSON text, for diffs
    /// and tests.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// assert model.write_body_json(body, record).startswith('{"magic":"ARRISBDY"')
    /// ```
    #[pyo3(signature = (body, provenance=None))]
    fn write_body_json(
        &self,
        py: Python<'_>,
        body: &Body,
        provenance: Option<&Provenance>,
    ) -> PyResult<String> {
        self.write_body_with(py, body, provenance, |m, b, r| {
            arris::io::body::to_json(m, b, r)
        })
    }

    /// Imports the body in `data` (`write_body`'s bytes) into this model,
    /// checked at the `full` level, and returns it with its record.
    ///
    /// Raises `BodyMagicError` for data that is not body bytes,
    /// `BodyVersionError` for a newer version, `BodyDecodeError` for a
    /// damaged stream, `BodyPrecisionError` for a tolerance this model
    /// cannot hold and `BodyRejectedError` for a body the checker rejects.
    /// On any error the model is as it was.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// imported = arris.Model().read_body(model.write_body(body, record))
    /// assert imported.foreign() == []
    /// ```
    #[pyo3(signature = (data, *, cancel=None, budget=None))]
    fn read_body(
        &self,
        py: Python<'_>,
        data: &[u8],
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Imported> {
        let limits = Limits::new(cancel, budget);
        self.read_body_with(py, &limits, |m, control| {
            arris::io::body::read(m, data, control)
        })
    }

    /// As `read_body`, for `write_body_json`'s text.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// imported = arris.Model().read_body_json(model.write_body_json(body, record))
    /// assert imported.version >= 1
    /// ```
    #[pyo3(signature = (text, *, cancel=None, budget=None))]
    fn read_body_json(
        &self,
        py: Python<'_>,
        text: &str,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Imported> {
        let limits = Limits::new(cancel, budget);
        self.read_body_with(py, &limits, |m, control| {
            arris::io::body::from_json(m, text, control)
        })
    }

    /// The whole model, every entity and freed slot, as native-format
    /// bytes: deterministic, and read back to a model that dumps the same.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// data = model.to_native()
    /// assert arris.Model.from_native(data).to_native() == data
    /// ```
    fn to_native<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, pyo3::types::PyBytes>> {
        let bytes = py
            .detach(|| {
                let kernel = self.shared.lock()?;
                Ok::<_, BindError>(arris::io::native::to_bytes(&kernel))
            })?
            .map_err(|e| native_error(&e).raise(py, None))?;
        Ok(pyo3::types::PyBytes::new(py, &bytes))
    }

    /// The whole model as native-format JSON text, for diffs.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// text = model.to_native_json()
    /// assert arris.Model.from_native_json(text).to_native_json() == text
    /// ```
    fn to_native_json(&self, py: Python<'_>) -> PyResult<String> {
        py.detach(|| {
            let kernel = self.shared.lock()?;
            Ok::<_, BindError>(arris::io::native::to_json(&kernel))
        })?
        .map_err(|e| native_error(&e).raise(py, None))
    }

    /// A new model read from `to_native`'s bytes. It is a model of its own:
    /// no handle of the one that wrote them belongs to it.
    ///
    /// Raises `NativeVersionError` for another version and
    /// `NativeDecodeError` for data that is not a model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// data = model.to_native()
    /// assert arris.Model.from_native(data).to_native() == data
    /// ```
    #[staticmethod]
    fn from_native(py: Python<'_>, data: &[u8]) -> PyResult<Model> {
        let kernel = py
            .detach(|| arris::io::native::from_bytes(data))
            .map_err(|e| native_error(&e).raise(py, None))?;
        Ok(Model::wrapping(kernel))
    }

    /// A new model read from `to_native_json`'s text. Errors as for
    /// `from_native`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// text = model.to_native_json()
    /// assert arris.Model.from_native_json(text).to_native_json() == text
    /// ```
    #[staticmethod]
    fn from_native_json(py: Python<'_>, text: &str) -> PyResult<Model> {
        let kernel = py
            .detach(|| arris::io::native::from_json(text))
            .map_err(|e| native_error(&e).raise(py, None))?;
        Ok(Model::wrapping(kernel))
    }

    fn __repr__(&self) -> String {
        format!("Model({})", self.shared.serial)
    }
}

impl Model {
    /// A body write, with `write` the kernel's encoder: the record must be
    /// this model's.
    fn write_body_with<T: Send>(
        &self,
        py: Python<'_>,
        body: &Body,
        provenance: Option<&Provenance>,
        write: impl FnOnce(
            &topo::Model,
            topo::Body,
            &arris::topo::provenance::Provenance,
        ) -> Result<T, arris::io::body::BodyError>
        + Send,
    ) -> PyResult<T> {
        let body = body.resolve(&self.shared)?;
        let empty = arris::topo::provenance::Provenance::default();
        let record = match provenance {
            Some(record) => record.kernel_in(&self.shared)?,
            None => &empty,
        };
        py.detach(|| {
            let kernel = self.shared.lock()?;
            Ok::<_, BindError>(write(&kernel, body, record))
        })?
        .map_err(|e| body_error(&e).raise(py, Some(self)))
    }

    /// A body read, with `read` the kernel's decoder. Its errors name the
    /// writer's ids, so they are raised with no model.
    fn read_body_with(
        &self,
        py: Python<'_>,
        limits: &Limits,
        read: impl FnOnce(
            &mut topo::Model,
            &Control<'_>,
        ) -> Result<arris::io::body::Imported, arris::io::body::BodyError>
        + Send,
    ) -> PyResult<Imported> {
        let imported = self.run_with(py, limits, read, body_error, false, |error| {
            matches!(
                error,
                arris::io::body::BodyError::Interrupted(Interrupted { by: Stop::Poll, .. })
            )
        })?;
        Ok(Imported::minted_by(self, imported))
    }

    fn edges_list(&self, edges: &[PyRef<'_, Edge>]) -> Result<Vec<topo::Edge>, BindError> {
        edges.iter().map(|e| e.resolve(&self.shared)).collect()
    }
}

/// The side of a shell's wall named by `text`: `"inward"` or `"outward"`.
fn shell_side_of(text: &str) -> PyResult<ShellSide> {
    match text {
        "inward" => Ok(ShellSide::Inward),
        "outward" => Ok(ShellSide::Outward),
        other => Err(pyo3::exceptions::PyValueError::new_err(format!(
            "{other:?} is not a shell side (\"inward\" or \"outward\")"
        ))),
    }
}

#[cfg(test)]
mod tests {
    use arris::Control;
    use arris::topo::{BodyId, EntityId};

    use super::*;
    use crate::handle::Body;

    /// A box in a model of its own, and its handle.
    fn model_with_box() -> (Model, Body) {
        let mut kernel = topo::Model::default();
        let (body, _) =
            arris::ops::primitive_box(&mut kernel, [0.0; 3], [1.0; 3], &Control::NONE).unwrap();
        let model = Model::wrapping(kernel);
        let handle = Body::minted_by(&model, body);
        (model, handle)
    }

    #[test]
    fn a_handle_resolves_in_its_own_model_only() {
        let (a, in_a) = model_with_box();
        let (b, in_b) = model_with_box();
        // The same slot and generation in each, so the kernel alone could
        // not tell them apart: the second model's box is a real body in the
        // first model too.
        assert_eq!(
            in_a.resolve(&a.shared).unwrap(),
            in_b.resolve(&b.shared).unwrap()
        );
        assert!(a.shared.lock().unwrap().body(BodyId::new(0, 0)).is_ok());

        let refused = in_a.resolve(&b.shared).unwrap_err();
        assert_eq!(
            refused,
            BindError::Foreign {
                entity: EntityId::Body(BodyId::new(0, 0)),
                owner: a.shared.serial(),
                this: b.shared.serial(),
            }
        );
        assert!(matches!(
            b.retaining(std::slice::from_ref(&in_a)),
            Err(BindError::Foreign { .. })
        ));
        assert_ne!(
            in_a, in_b,
            "equal slots in different models are different handles"
        );
    }

    #[test]
    fn a_handle_equals_its_copy_and_hashes_with_it() {
        use std::hash::{BuildHasher, RandomState};
        let (model, handle) = model_with_box();
        let again = Body::minted_by(&model, handle.resolve(&model.shared).unwrap().reversed());
        assert_eq!(
            handle, again,
            "the orientation a handle was reached with is not its identity"
        );
        let state = RandomState::new();
        assert_eq!(state.hash_one(&handle), state.hash_one(&again));
        assert_eq!(model.clone(), model);
    }

    #[test]
    fn a_freed_entity_is_stale_and_a_kept_one_stays_valid() {
        let (model, kept) = model_with_box();
        let dropped = {
            let mut kernel = model.shared.lock().unwrap();
            let (second, _) =
                arris::ops::primitive_box(&mut kernel, [5.0; 3], [6.0; 3], &Control::NONE).unwrap();
            Body::minted_by(&model, second)
        };
        let entity = |h: &Body| EntityId::Body(h.resolve(&model.shared).unwrap().id);
        assert!(model.holds(entity(&dropped)).unwrap());

        let freed = model.retaining(std::slice::from_ref(&kept)).unwrap();
        assert!(freed > 0);
        assert!(
            model.holds(entity(&kept)).unwrap(),
            "retain never renumbers (ADR-0010)"
        );
        assert!(!model.holds(entity(&dropped)).unwrap());
        // Keeping the freed body is the kernel's NotFound, surfaced as stale.
        let err = model.retaining(std::slice::from_ref(&dropped)).unwrap_err();
        assert!(matches!(err, BindError::Stale(nf) if nf.id == entity(&dropped).into()));
    }

    #[test]
    fn a_poisoned_lock_is_an_error_naming_the_model() {
        let (model, _) = model_with_box();
        let shared = Arc::clone(&model.shared);
        let _ = std::thread::spawn(move || {
            let _guard = shared.lock().unwrap();
            panic!("a panic inside the lock");
        })
        .join();
        assert_eq!(
            model.shared.lock().unwrap_err(),
            BindError::Poisoned {
                model: model.shared.serial()
            }
        );
    }
}
