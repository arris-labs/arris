//! Handles: a kernel id and the model that minted it.

use std::sync::Arc;

use arris::topo::{self, EntityId, Orientation};
use pyo3::prelude::*;

use crate::error::BindError;
use crate::model::{Model, Shared};

macro_rules! handles {
    ($( $(#[$doc:meta])* $name:ident($kernel:ident, $id:ident) => $variant:ident ),* $(,)?) => {$(
        $(#[$doc])*
        ///
        /// Frozen and hashable: two handles are equal when they name the same
        /// slot and generation of the same model. `reversed` is how the entity
        /// was reached and is not part of the identity.
        #[pyclass(frozen, eq, hash, module = "arris")]
        #[derive(Clone, Debug)]
        pub struct $name {
            model: Arc<Shared>,
            kernel: topo::$kernel,
        }

        impl PartialEq for $name {
            fn eq(&self, other: &Self) -> bool {
                self.model.serial() == other.model.serial() && self.kernel.id == other.kernel.id
            }
        }

        impl std::hash::Hash for $name {
            fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
                self.model.serial().hash(state);
                self.kernel.id.hash(state);
            }
        }

        impl $name {
            /// A handle to `kernel`, an entity of `model`.
            pub fn minted_by(model: &Model, kernel: topo::$kernel) -> Self {
                Self { model: Arc::clone(&model.shared), kernel }
            }

            /// The kernel handle, if `owner` is the model that minted this one.
            pub(crate) fn resolve(&self, owner: &Arc<Shared>) -> Result<topo::$kernel, BindError> {
                if self.model.serial() == owner.serial() {
                    Ok(self.kernel)
                } else {
                    Err(BindError::Foreign {
                        entity: EntityId::$variant(self.kernel.id),
                        owner: self.model.serial(),
                        this: owner.serial(),
                    })
                }
            }
        }

        #[pymethods]
        impl $name {
            /// The slot of the entity in its model's arena.
            #[getter]
            fn index(&self) -> u32 {
                self.kernel.id.index()
            }

            /// The slot's generation when this handle was minted.
            #[getter]
            fn generation(&self) -> u32 {
                self.kernel.id.generation()
            }

            /// Whether the entity was reached against its own orientation.
            #[getter]
            fn reversed(&self) -> bool {
                self.kernel.orientation == Orientation::Reversed
            }

            /// The model this handle belongs to.
            #[getter]
            fn model(&self) -> Model {
                Model { shared: Arc::clone(&self.model) }
            }

            fn __repr__(&self) -> String {
                format!(
                    concat!(stringify!($name), "(model={}, index={}, generation={})"),
                    self.model.serial(),
                    self.kernel.id.index(),
                    self.kernel.id.generation(),
                )
            }
        }
    )*};
}

handles! {
    /// A body of a model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    /// handle = body
    /// assert handle == body and hash(handle) == hash(body)
    /// assert handle.model == model and not handle.reversed
    /// ```
    Body(Body, BodyId) => Body,
    /// A shell of a model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    /// handle = model.shells(body)[0]
    /// assert handle == model.shells(body)[0] and hash(handle) == hash(model.shells(body)[0])
    /// assert handle.model == model and not handle.reversed
    /// ```
    Shell(Shell, ShellId) => Shell,
    /// A face of a model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    /// handle = model.faces(body)[0]
    /// assert handle == model.faces(body)[0] and hash(handle) == hash(model.faces(body)[0])
    /// assert handle.model == model and not handle.reversed
    /// ```
    Face(Face, FaceId) => Face,
    /// An edge of a model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    /// handle = model.edges(body)[0]
    /// assert handle == model.edges(body)[0] and hash(handle) == hash(model.edges(body)[0])
    /// assert handle.model == model and isinstance(handle.reversed, bool)
    /// ```
    Edge(Edge, EdgeId) => Edge,
    /// A vertex of a model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_box((0, 0, 0), (1, 1, 1))
    /// handle = model.vertices(body)[0]
    /// assert handle == model.vertices(body)[0] and hash(handle) == hash(model.vertices(body)[0])
    /// assert handle.model == model and isinstance(handle.reversed, bool)
    /// ```
    Vertex(Vertex, VertexId) => Vertex,
}

/// A handle of any kind, for the calls that take one.
#[derive(FromPyObject)]
pub(crate) enum AnyHandle<'py> {
    Body(PyRef<'py, Body>),
    Shell(PyRef<'py, Shell>),
    Face(PyRef<'py, Face>),
    Edge(PyRef<'py, Edge>),
    Vertex(PyRef<'py, Vertex>),
}

impl AnyHandle<'_> {
    /// The entity and the orientation it was reached with, if `owner` minted
    /// this handle.
    pub(crate) fn resolve_shape(
        &self,
        owner: &Arc<Shared>,
    ) -> Result<(EntityId, Orientation), BindError> {
        Ok(match self {
            AnyHandle::Body(h) => {
                let k = h.resolve(owner)?;
                (EntityId::Body(k.id), k.orientation)
            }
            AnyHandle::Shell(h) => {
                let k = h.resolve(owner)?;
                (EntityId::Shell(k.id), k.orientation)
            }
            AnyHandle::Face(h) => {
                let k = h.resolve(owner)?;
                (EntityId::Face(k.id), k.orientation)
            }
            AnyHandle::Edge(h) => {
                let k = h.resolve(owner)?;
                (EntityId::Edge(k.id), k.orientation)
            }
            AnyHandle::Vertex(h) => {
                let k = h.resolve(owner)?;
                (EntityId::Vertex(k.id), k.orientation)
            }
        })
    }

    /// The entity, if `owner` minted this handle.
    pub(crate) fn resolve(&self, owner: &Arc<Shared>) -> Result<EntityId, BindError> {
        Ok(match self {
            AnyHandle::Body(h) => EntityId::Body(h.resolve(owner)?.id),
            AnyHandle::Shell(h) => EntityId::Shell(h.resolve(owner)?.id),
            AnyHandle::Face(h) => EntityId::Face(h.resolve(owner)?.id),
            AnyHandle::Edge(h) => EntityId::Edge(h.resolve(owner)?.id),
            AnyHandle::Vertex(h) => EntityId::Vertex(h.resolve(owner)?.id),
        })
    }
}
