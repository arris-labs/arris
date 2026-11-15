//! Why a mesh could not be built: the one error type of the crate.

use arris_check::Report;
use arris_math::Interrupted;
use arris_topo::{Body, EdgeId, FaceId, NotFound};

use crate::cdt::CdtError;

/// Why a mesh could not be built or extended: by hand through the
/// `push_*` methods, or from a body by [`crate::tessellate`].
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum MeshError {
    /// A triangle or edge index does not name a position.
    #[error("index {index} is out of range for {positions} positions")]
    IndexOutOfRange {
        /// The offending index.
        index: u32,
        /// How many positions the mesh has.
        positions: usize,
    },
    /// A face or edge range does not fit in the list it indexes.
    #[error("range {start}..{end} is out of bounds for a list of {len}")]
    RangeOutOfBounds {
        /// Range start.
        start: usize,
        /// Range end.
        end: usize,
        /// Length of the indexed list.
        len: usize,
    },
    /// A position has a non-finite coordinate.
    #[error("position {index} is not finite")]
    NonFinitePosition {
        /// The position's index.
        index: usize,
    },
    /// The body, or an entity or geometry value it refers to, does not
    /// resolve in the model.
    #[error(transparent)]
    NotFound(#[from] NotFound),
    /// A face asked for by [`crate::tessellate_faces`] is not a face of
    /// the body.
    #[error("{face} is not a face of {body}")]
    NotInBody {
        /// The face.
        face: FaceId,
        /// The body.
        body: Body,
    },
    /// Two meshes offered to [`crate::TriMesh::weld`] discretise `edge`
    /// differently: not the same number of samples, or a sample at
    /// another position. They were meshed at different chords, or of
    /// different states of the model. Nothing is snapped (ADR-0003).
    #[error("{edge} is meshed differently in the parts to weld")]
    WeldMismatch {
        /// The edge.
        edge: EdgeId,
    },
    /// The body fails the checker at `Level::Fast` (checked in debug
    /// builds before tessellation starts, as every operation checks its
    /// input).
    #[error("{body} fails the checker:\n{report}")]
    InvalidInput {
        /// The body.
        body: Body,
        /// What it fails.
        report: Box<Report>,
    },
    /// The chord tolerance asked for is not finite and positive.
    #[error("chord tolerance {0} is not finite and positive")]
    Chord(f64),
    /// A face's domain could not be triangulated: its loops, discretised,
    /// are not the simple nested polygons the checker's L4 and L5 rows
    /// promise.
    #[error("{face}: {source}")]
    Face {
        /// The face.
        face: FaceId,
        /// What the triangulation found.
        source: CdtError,
    },
    /// The interior lattice `chord` asks for on `face` would need more
    /// than [`crate::MAX_INTERIOR_POINTS`] points: a surface whose
    /// curvature varies enormously over its domain (a torus with a
    /// minor radius far smaller than its major one, at a fine chord).
    #[error("{face}'s interior lattice would need {points} points")]
    GridTooLarge {
        /// The face.
        face: FaceId,
        /// How many points it would need.
        points: usize,
    },
    /// A corner block does not fit together, or does not fit the mesh it
    /// was offered to: parallel arrays of different lengths, a face list
    /// that is not the mesh's faces in order, a triangle list that is not
    /// parallel to the mesh's, or a face-local vertex that does not stand
    /// on the mesh vertex its triangle does (ADR-0012).
    #[error("corner block: {0}")]
    Corners(String),
    /// Tessellation's own bookkeeping broke on validated input: never a
    /// property of the body or the chord tolerance, and never the CDT's
    /// own fault, so never a [`MeshError::Face`].
    #[error("kernel bug: {0}")]
    Internal(&'static str),
    /// The caller's poll or budget stopped the tessellation (ADR-0030):
    /// nothing was built, and the same call with a larger budget or a poll
    /// that stays `false` can succeed. The model is only read, so it is
    /// untouched.
    #[error("{0}")]
    Interrupted(Interrupted),
}

impl From<Interrupted> for MeshError {
    fn from(stop: Interrupted) -> Self {
        MeshError::Interrupted(stop)
    }
}
