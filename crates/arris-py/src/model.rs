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
use crate::geometry::{Corners, CurveInfo, SurfaceInfo};
use crate::handle::{AnyHandle, Body, DistanceOperand, Edge, Face, Shell, Vertex};
use crate::io::{Imported, StepRead, length_unit as length_unit_of};
use crate::kernel_error::{
    Mapped, body_error, mesh_error, native_error, op_error, step_error, step_read_error,
};
use crate::mesh::Mesh;
use crate::path::{FrameLaw, Path};
use crate::profile::Profile;
use crate::provenance::Provenance;
use crate::query::{
    Clash, Distance, Frame, Interference, MassProperties, RayHit, Report, Section,
    SurfaceProperties, level_of,
};

mod features;
mod io;
mod ops;
mod query;
mod topology;

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
pub(crate) fn refused(reason: Reason) -> OpError {
    OpError::Degenerate {
        entities: Vec::new(),
        reason,
    }
}

/// The plane through `origin` with `normal` and, if given, the `x` the
/// sheet or caps are parameterised by; `what` names the operation in a
/// refusal.
fn plane_of(
    what: &'static str,
    origin: [f64; 3],
    normal: [f64; 3],
    x: Option<[f64; 3]>,
) -> Result<KernelFrame, OpError> {
    let refused_plane = || {
        refused(Reason::Input(InputReason::NonFinite {
            what: match what {
                "section" => "section plane",
                _ => "split plane",
            },
        }))
    };
    finite("plane origin", origin)?;
    let n = direction_of("plane normal", normal)?;
    match x {
        None => KernelFrame::from_z(Point3::from(origin), n).map_err(|_| refused_plane()),
        Some(x) => {
            finite("plane x", x)?;
            KernelFrame::new(Point3::from(origin), n, Vec3::from(x)).map_err(|_| {
                refused(Reason::Input(InputReason::NotPositive {
                    what: "the plane x's part across the normal",
                    value: 0.0,
                }))
            })
        }
    }
}

pub(crate) fn finite(what: &'static str, v: [f64; 3]) -> Result<(), OpError> {
    if v.iter().all(|c| c.is_finite()) {
        Ok(())
    } else {
        Err(refused(Reason::Input(InputReason::NonFinite { what })))
    }
}

/// A direction from three numbers: finite, and not zero.
pub(crate) fn direction_of(what: &'static str, v: [f64; 3]) -> Result<Vec3, OpError> {
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

    /// The tolerance this model creates entities with: two points closer
    /// than it are the same point (Open CASCADE's `Precision::Confusion`).
    ///
    /// ```python
    /// import arris
    ///
    /// assert arris.Model().tolerance == 1e-7
    /// ```
    #[getter]
    fn tolerance(&self) -> PyResult<f64> {
        Ok(self.shared.lock()?.precision().default_tolerance)
    }

    /// The angle in radians below which this model holds two directions
    /// parallel and two surfaces tangent.
    ///
    /// ```python
    /// import arris
    ///
    /// assert arris.Model().angular_tolerance == 1e-12
    /// ```
    #[getter]
    fn angular_tolerance(&self) -> PyResult<f64> {
        Ok(self.shared.lock()?.precision().angular_tolerance)
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

    fn __repr__(&self) -> String {
        format!("Model({})", self.shared.serial)
    }
}

impl Model {
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
