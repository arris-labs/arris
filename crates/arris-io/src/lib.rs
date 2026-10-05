//! Formats of the Arris kernel: the STEP AP214 Part 21 writer and reader
//! (ADR-0025), the native format (`serde` of the model), body bytes (one
//! body and its record, read by every later release, ADR-0029), and the
//! mesh formats STL and OBJ (ADR-0013).
//!
//! Guarantees: the native format round-trips a model to an identical text
//! dump; STEP carries the B-Rep entity subset with pcurves written out, and
//! reads back every solid of that subset as a body the checker passes or a
//! typed refusal naming the file entity (`docs/ARCHITECTURE.md` §Formats
//! and tools). The `serde` feature (on by default) enables the native format.
//! Depends on `arris-check` and `arris-mesh` and below, both re-exported
//! here so a crate above reaches the checker, the representation and the
//! mesh types through this one; never on `arris-ops`.
#![forbid(unsafe_code)]
#![warn(missing_docs)]

#[cfg(feature = "serde")]
pub mod body;
#[cfg(feature = "serde")]
pub mod native;
pub mod obj;
pub mod step;
pub mod stl;

/// Why a mesh format writer could not write an [`arris_mesh::TriMesh`]:
/// shared by [`stl`] and [`obj`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum MeshWriteError {
    /// Binary STL's triangle count is a `u32`; the mesh has more
    /// triangles than that can hold.
    #[error("{triangles} triangles do not fit binary STL's u32 count")]
    TooManyTriangles {
        /// How many triangles the mesh has.
        triangles: usize,
    },
}
