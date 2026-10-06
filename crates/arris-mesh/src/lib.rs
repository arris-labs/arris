//! Meshes of the Arris kernel: `TriMesh` and `Polyline`, with signed volume,
//! area and closedness, and the tessellation of bodies into them with
//! per-face and per-edge ranges, optionally carrying the render buffer
//! of `Corners` beside the watertight one.
//!
//! Guarantees (`docs/ARCHITECTURE.md` §Tessellation, ADR-0003):
//! positions are `f64` and exact evaluations of the geometry — a topo
//! vertex's point, an edge's curve at a sampled parameter, later a
//! surface at an interior grid point; a topo vertex is one mesh vertex
//! and an edge's samples are one index run shared by every face that
//! uses the edge, so a mesh of a solid is closed by construction; a seam
//! edge is discretised once and its indices appear in the wall's
//! triangles twice, a degenerate edge's (u, v) segment maps to one index
//! and the triangles that collapse are dropped; triangles are
//! counter-clockwise seen from outside, by the face use's effective
//! orientation against the surface normal; `FaceRange`s and `EdgeRange`s
//! are in the body's iteration order; same body, same chord, same mesh
//! on every platform, with the `parallel` feature on or off; an edge's
//! samples are a function of the edge and the chord, so a subset of a body's
//! faces ([`tessellate_faces`]) meshes to the whole's edges and triangles
//! bit for bit, and [`TriMesh::weld`] joins them by their shared edges
//! (ADR-0052). The
//! `parallel` feature reserves `rayon` over faces. Depends on
//! `arris-check` and below; never on `arris-ops` or `arris-io`.
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod cdt;
mod corners;
mod polyline;
mod tessellate;
mod trimesh;

// `Aabb` and `Interval` live in `arris-math` (01 §Crates); re-exported
// so a mesh caller reaches them through this crate as it always has.
pub use arris_math::{Aabb, Interval};
pub use corners::{CornerFace, Corners, NORMAL_UNIT_SLACK};
pub use polyline::Polyline;
pub use tessellate::{
    MAX_INTERIOR_POINTS, MeshRequest, RING_RATIO, tessellate, tessellate_faces, tessellate_with,
};
pub use trimesh::{EdgeRange, FaceRange, MeshError, TriMesh};
