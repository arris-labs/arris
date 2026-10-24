#[cfg(doc)]
use super::{Assembly, Builder};
use super::{Counts, EdgeRef, FaceRef, Position, VertexRef};
use crate::entity::BodyKind;
use crate::error::NotFound;
use crate::id::{EntityId, EntityKind, SurfaceId};

/// Why an operator, [`Builder::finish`] or [`Builder::assemble`] refused,
/// by the kind of fault; each variant's message is its group's. Every
/// variant names the slot, position or edge it is about; none is a panic.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum BuildError {
    /// A slot, loop, position or spec the call named is not there.
    #[error(transparent)]
    Slot(#[from] SlotError),
    /// An Euler operator's arguments do not fit the builder's state.
    #[error(transparent)]
    Euler(#[from] EulerError),
    /// The staged body does not close as the kind asked for.
    #[error(transparent)]
    Closure(#[from] ClosureError),
    /// A geometry id a slot references does not resolve in the model.
    #[error(transparent)]
    NotFound(#[from] NotFound),
    /// A kill operator's own bookkeeping broke: neither the use it left a
    /// position after, nor the empty loop it seeded, was on `face` — one
    /// of the two always is by construction. Never a property of the
    /// input.
    #[error("{face} has neither the use nor the empty loop a kill left a position after")]
    Invariant {
        /// The face.
        face: FaceRef,
    },
}

/// A builder slot, loop or position a call named is not there, or an assembly key names a spec it does not have.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SlotError {
    /// A vertex slot that is not live.
    #[error("{0} is not a vertex of the builder")]
    NoVertex(VertexRef),
    /// An edge slot that is not live.
    #[error("{0} is not an edge of the builder")]
    NoEdge(EdgeRef),
    /// A face slot that is not live.
    #[error("{0} is not a face of the builder")]
    NoFace(FaceRef),
    /// A loop index past the face's loops.
    #[error("{face} has no loop {loop_index}")]
    NoLoop {
        /// The face.
        face: FaceRef,
        /// The index asked for.
        loop_index: usize,
    },
    /// A coedge index past the loop's junctions.
    #[error("{position} is past the end of a loop of {len} coedge(s)")]
    BadPosition {
        /// The position.
        position: Position,
        /// The loop's length; `len` itself is the last valid junction.
        len: usize,
    },
    /// [`Builder::assemble`]: a key names a spec the assembly does not have.
    #[error("the assembly has no {kind} spec at index {index}")]
    NoSpec {
        /// The kind of spec the key was for.
        kind: EntityKind,
        /// The index it named.
        index: usize,
    },
}

/// An Euler operator refused its arguments: the shape it needs is not what the builder holds.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum EulerError {
    /// `mvfs` on a builder that already holds a body.
    #[error("the builder already holds a body; mvfs starts one from nothing")]
    NotEmpty,
    /// `mef` needs both positions in one loop.
    #[error("{from} and {to} are not in the same loop")]
    NotOneLoop {
        /// The first position.
        from: Position,
        /// The second.
        to: Position,
    },
    /// `mekr` needs two loops of one face.
    #[error("{from} and {to} are not two loops of one face")]
    NotTwoLoops {
        /// The first position.
        from: Position,
        /// The second.
        to: Position,
    },
    /// The vertex is at several junctions of the loop; say which.
    #[error("{vertex} is at junctions {positions:?} of loop {loop_index} of {face}")]
    Ambiguous {
        /// The vertex.
        vertex: VertexRef,
        /// The face.
        face: FaceRef,
        /// The loop.
        loop_index: usize,
        /// Every junction it is at.
        positions: Vec<usize>,
    },
    /// The vertex is not on the loop.
    #[error("{vertex} is not on loop {loop_index} of {face}")]
    NotInLoop {
        /// The vertex.
        vertex: VertexRef,
        /// The face.
        face: FaceRef,
        /// The loop.
        loop_index: usize,
    },
    /// `kvfs` needs exactly the state `mvfs` makes.
    #[error("kvfs needs one vertex, one face with one empty loop and nothing else")]
    NotASeed,
    /// `kev` needs an edge used twice in a row by one loop, away and back,
    /// with distinct vertices.
    #[error("{edge} is not a strut: its two uses are not consecutive in one loop")]
    NotAStrut {
        /// The edge.
        edge: EdgeRef,
    },
    /// The vertex `kev` would kill is an end of another edge.
    #[error("{vertex} is used by {other}")]
    VertexInUse {
        /// The vertex.
        vertex: VertexRef,
        /// The other edge.
        other: EdgeRef,
    },
    /// The vertex `kev` would kill is the seed of an empty loop.
    #[error("{vertex} seeds loop {loop_index} of {face}")]
    VertexSeeds {
        /// The vertex.
        vertex: VertexRef,
        /// The face.
        face: FaceRef,
        /// The loop.
        loop_index: usize,
    },
    /// `kef` needs an edge used once by each of two faces.
    #[error("{edge} does not separate two faces")]
    NotSeparating {
        /// The edge.
        edge: EdgeRef,
    },
    /// `kemr` needs an edge used twice by one loop.
    #[error("{edge} is not used twice by one loop")]
    NotInOneLoop {
        /// The edge.
        edge: EdgeRef,
    },
    /// `kef` and `kfmrh` kill a face of one loop only.
    #[error("{face} has {loops} loops; it must have one")]
    Rings {
        /// The face.
        face: FaceRef,
        /// How many loops it has.
        loops: usize,
    },
    /// `mfkrh` needs a face with a ring to lift out.
    #[error("{face} has no loop {ring} to make a face of, or it is the only one")]
    NoRing {
        /// The face.
        face: FaceRef,
        /// The index asked for.
        ring: usize,
    },
    /// `kfmrh` needs two faces.
    #[error("{face} cannot be joined to itself")]
    OneFace {
        /// The face.
        face: FaceRef,
    },
    /// `kfmrh` joins two faces on one surface.
    #[error("{kill} is on {kill_surface} and {into} on {into_surface}; kfmrh needs one surface")]
    SurfaceMismatch {
        /// The face to kill.
        kill: FaceRef,
        /// Its surface.
        kill_surface: SurfaceId,
        /// The face to keep.
        into: FaceRef,
        /// Its surface.
        into_surface: SurfaceId,
    },
    /// `kfmrh` joins two faces whose outward normals oppose.
    #[error("{kill} and {into} are used with the same orientation; a handle needs opposite ones")]
    SameOrientation {
        /// The face to kill.
        kill: FaceRef,
        /// The face to keep.
        into: FaceRef,
    },
    /// `mfkrh` on a builder with no handle.
    #[error("no handle to remove: the genus is zero")]
    NoHandle,
    /// `mev` with a degenerate edge.
    #[error("a strut has two vertices and needs a curve; a degenerate edge has neither")]
    DegenerateStrut,
    /// A degenerate edge between two vertices.
    #[error("a degenerate edge joins one vertex to itself, not {start} to {end}")]
    DegenerateEnds {
        /// The vertex at `from`.
        start: VertexRef,
        /// The vertex at `to`.
        end: VertexRef,
    },
}

/// The staged body is not one that [`Builder::finish`] or [`Builder::assemble`] can store: it is not closed as the kind asked for, or the assembly describes it inconsistently.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ClosureError {
    /// `finish` on a builder holding nothing.
    #[error("the builder holds no face")]
    Empty,
    /// `finish` makes solids and sheets: no operation returns a wire or a
    /// general body.
    #[error("the builder makes a solid or a sheet, not a {0} body")]
    Kind(BodyKind),
    /// A loop without coedges cannot be stored.
    #[error("loop {loop_index} of {face} has no coedges")]
    EmptyLoop {
        /// The face.
        face: FaceRef,
        /// The loop.
        loop_index: usize,
    },
    /// A coedge the caller never gave a pcurve.
    #[error("the coedge at {position} has no pcurve")]
    MissingPcurve {
        /// The coedge.
        position: Position,
    },
    /// An edge not used by exactly two coedges, which a solid needs — or a
    /// degenerate edge not used by exactly one: the singular point of the
    /// one face that closes on it (`docs/DATA-MODEL.md` §Invariants, S2),
    /// which no surface closes on twice. The operators never make one; the
    /// check is the contract, stated.
    #[error("{edge} is used by {uses} coedge(s); a solid's edge needs two, a degenerate edge one")]
    EdgeUses {
        /// The edge.
        edge: EdgeRef,
        /// Its uses.
        uses: usize,
    },
    /// An edge used twice in the same effective direction.
    #[error("{edge} is used twice in the same direction")]
    SameDirection {
        /// The edge.
        edge: EdgeRef,
    },
    /// [`Builder::assemble`]: one arena entity is kept by two specs, which
    /// would put it in the result twice.
    #[error("{0} is kept twice")]
    Duplicate(EntityId),
    /// [`Builder::assemble`]: consecutive uses of a loop do not meet — the
    /// use at `coedge_index` does not start where its predecessor ended.
    #[error(
        "loop {loop_index} of {face} is open at junction {coedge_index}: {ended} then {starts}"
    )]
    LoopOpen {
        /// The face.
        face: FaceRef,
        /// The loop.
        loop_index: usize,
        /// The junction the two uses fail to share.
        coedge_index: usize,
        /// The vertex the previous use ends at.
        ended: VertexRef,
        /// The vertex this use starts at.
        starts: VertexRef,
    },
    /// [`Builder::assemble`]: the faces of one shell are not one
    /// edge-connected component.
    #[error("{face} is not edge-connected to {from}, the first face of its shell")]
    Disconnected {
        /// A face of another component.
        face: FaceRef,
        /// The face the walk started from.
        from: FaceRef,
    },
    /// [`Builder::assemble`]: the counts of a shell do not close the
    /// Euler–Poincaré line at a whole genus, so its faces are not a closed
    /// surface — or a vertex is on no shell's edge, so the body's do not.
    /// [`Builder::assemble_sheet`] reports a shell's counts with each of its
    /// boundary cycles capped by a face of one loop.
    #[error("{counts} is not a closed surface of whole genus")]
    NotClosed {
        /// The counts of the shell that does not close (`S` = 1), or of
        /// the whole body for a vertex on no edge; the genus reported as
        /// zero.
        counts: Counts,
    },
    /// [`Builder::assemble`]: a shell of the assembly has no faces.
    #[error("shell {shell} of the assembly has no faces")]
    EmptyShell {
        /// The shell's index in [`Assembly::shells`].
        shell: usize,
    },
    /// [`Builder::assemble`]: an edge is used by faces of two shells, so
    /// neither shell is closed on its own.
    #[error("{edge} is used by shells {} and {}", .shells[0], .shells[1])]
    SharedEdge {
        /// The edge.
        edge: EdgeRef,
        /// The two shells, ascending.
        shells: [usize; 2],
    },
    /// [`Builder::assemble`]: a vertex is an end of edges of two shells —
    /// two closed shells touching at a point, which a manifold solid's
    /// shells never do.
    #[error("{vertex} is on edges of shells {} and {}", .shells[0], .shells[1])]
    SharedVertex {
        /// The vertex.
        vertex: VertexRef,
        /// The two shells, ascending.
        shells: [usize; 2],
    },
}
