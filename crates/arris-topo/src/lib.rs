//! Topology of the Arris kernel: the `Model` arena, typed generational ids,
//! `Shape` handles as id plus orientation, the entities of
//! `docs/DATA-MODEL.md` §Topology, pcurves, per-entity tolerances, Euler
//! operators, deterministic adjacency and iteration, and `Provenance`.
//!
//! Guarantees: entities are immutable and the arena is append-only; ids are
//! allocated in creation order and iteration order is the same on every
//! platform. The `serde` feature (on by default) derives the native format's
//! encoding. Depends on `arris-geom` and below, both re-exported here so a
//! crate above reaches geometry through this one alone.
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod arena;
pub mod builder;
pub mod entity;
mod error;
pub mod euler;
mod handle;
mod id;
mod idmap;
mod model;
mod orientation;
#[cfg(feature = "serde")]
mod pairs;
pub mod provenance;
mod walk;

pub use arena::CHUNK_SIZE;
pub use builder::Builder;
pub use error::{AnyId, NotFound, TopoError};
pub use handle::{Body, Edge, Face, Shape, Shell, Vertex, WrongKind};
pub use id::{
    BodyId, Curve2Id, CurveId, EdgeId, EntityId, EntityKind, FaceId, GeometryId, ShellId,
    SurfaceId, VertexId,
};
pub use idmap::IdMap;
pub use model::{CoedgeRef, Model, RawInsert};
pub use orientation::Orientation;
pub use provenance::{
    ConsumerKey, FileEntity, Origin, PlaneSide, Provenance, Relation, Role, SplitPart,
};
pub use walk::Closure;
