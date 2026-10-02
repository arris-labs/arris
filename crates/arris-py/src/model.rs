//! `Model`: the arena behind a lock, and the identity its handles carry.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};

use arris::math::nalgebra::UnitQuaternion;
use arris::math::{Axis, Isometry, Point3, Reflection, UnitVec3, Vec3};
use arris::ops::{OpError, Reason};
use arris::topo;
use arris::{Control, Interrupted, Stop};
use pyo3::exceptions::PyKeyboardInterrupt;
use pyo3::prelude::*;

use crate::control::{Cancel, Limits};
use crate::error::BindError;
use crate::handle::{AnyHandle, Body, Edge, Face, Shell, Vertex};
use crate::kernel_error::{Mapped, mesh_error, op_error};
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
        Err(refused(Reason::NonFinite { what }))
    }
}

/// A direction from three numbers: finite, and not zero.
fn direction_of(what: &'static str, v: [f64; 3]) -> Result<Vec3, OpError> {
    finite(what, v)?;
    let v = Vec3::from(v);
    if v.norm() > 0.0 {
        Ok(v)
    } else {
        Err(refused(Reason::NotPositive { what, value: 0.0 }))
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
        self.run_with(py, limits, call, op_error, |error| {
            matches!(
                error,
                OpError::Interrupted(Interrupted { by: Stop::Poll, .. })
            )
        })
    }

    /// [`Model::run`] for any kernel error type: `map` says how `E` is
    /// raised and `by_poll` whether it is a stop by the poll, which Ctrl-C
    /// answers.
    fn run_with<T: Send, E: Send>(
        &self,
        py: Python<'_>,
        limits: &Limits,
        call: impl FnOnce(&mut topo::Model, &Control<'_>) -> Result<T, E> + Send,
        map: impl FnOnce(&E) -> Mapped,
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
            Err(Failure::Kernel(error)) => Err(map(&error).raise(py, Some(self))),
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
    fn contains(&self, handle: AnyHandle<'_>) -> PyResult<bool> {
        Ok(self.holds(handle.resolve(&self.shared)?)?)
    }

    /// Frees every entity not reachable from the bodies in `keep`, and
    /// returns how many it freed. A kept entity's handle stays valid; the
    /// others go stale (ids are never renumbered).
    fn retain(&self, py: Python<'_>, keep: Vec<PyRef<'_, Body>>) -> PyResult<usize> {
        let keep: Vec<Body> = keep.iter().map(|body| (**body).clone()).collect();
        Ok(py.detach(|| self.retaining(&keep))?)
    }

    /// A box with corners `min` and `max`, every face, edge and vertex
    /// named by a `Role` in the returned `Provenance`.
    ///
    /// Raises `OpDegenerateError` for a corner that is not finite or an
    /// extent that is not positive.
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
                    Err(refused(Reason::NonFinite { what: "angle" }))
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
                    refused(Reason::NonFinite {
                        what: "mirror plane",
                    })
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

    /// The intersection of `a` and `b`. Errors as for `cut`.
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

    /// The solid `profile` sweeps along `direction` for `length`: caps from
    /// the profile's face and a side face for each of its segments, every
    /// entity named by the part of the sketch it came from
    /// (`Role("extrude", "Side", loop, segment)`).
    ///
    /// Raises `OpProfileError` for an invalid sketch and `OpDegenerateError`
    /// for a length that is not positive or finite or a direction that is
    /// not along the profile's normal.
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

    /// The volume, area, centroid and inertia of the solid `body`, at unit
    /// density.
    ///
    /// Raises `OpDegenerateError` for a body that is not a solid and
    /// `Interrupted` when `cancel` or `budget` stops it.
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
    fn shells(&self, body: &Body) -> PyResult<Vec<Shell>> {
        let body = body.resolve(&self.shared)?;
        let shells = self.shared.lock()?.shells(body).map_err(BindError::from)?;
        Ok(shells
            .into_iter()
            .map(|s| Shell::minted_by(self, s))
            .collect())
    }

    /// The faces of `body`, each once, depth-first through its shells.
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

    fn __repr__(&self) -> String {
        format!("Model({})", self.shared.serial)
    }
}

impl Model {
    fn edges_list(&self, edges: &[PyRef<'_, Edge>]) -> Result<Vec<topo::Edge>, BindError> {
        edges.iter().map(|e| e.resolve(&self.shared)).collect()
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
