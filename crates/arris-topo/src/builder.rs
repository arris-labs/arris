//! The builder and the Euler operators (`docs/DATA-MODEL.md` §Euler
//! operators, ADR-0002): the only way an operation makes topology.
//!
//! Entities in the arena are immutable and Euler operators mutate, so the
//! two meet in a [`Builder`]: a staging area holding one body under
//! construction, edited by Mäntylä's ten operators (*An Introduction to
//! Solid Modeling*, ch. 9, adapted to coedges and seams), and frozen into
//! the arena by [`Builder::finish`], which appends every entity in a
//! deterministic order inside a transaction.
//!
//! What the builder guarantees: every operator keeps the Euler–Poincaré
//! line at zero ([`Builder::counts`]); every operator has an inverse that
//! restores the builder byte for byte ([`Builder::dump`]) — a killed slot
//! is reused by the next make, loops are kept in a canonical rotation and
//! order, so the state after `op` then `op⁻¹` is the state before; and
//! `finish` refuses a body that is not closed as the kind asked for, a
//! loop without coedges, a coedge without a pcurve, or a geometry id that
//! does not resolve — each a typed [`BuildError`] — and then the model is
//! exactly as it was.
//!
//! What it does not do: it never computes a pcurve and never evaluates
//! geometry. Every pcurve is the caller's ([`Strut::pcurves`],
//! [`Split::pcurves`], [`Builder::set_pcurve`]), since only the caller
//! knows which use of a seam it is drawing; and the checker
//! (`arris-check`, above this crate) is where the finished body is
//! proven.
//!
//! Orientations inside the builder are *effective*: a [`Use`] is walked
//! as seen from outside the material with the face's outward normal up,
//! whatever the surface's own normal, and each [`StagedFace`] carries the
//! orientation the shell will use it with. `finish` stores a loop of a
//! `Reversed` face backwards with every use flipped, which is what
//! `docs/DATA-MODEL.md` §Orientation says a stored loop is.

use core::fmt;
use std::collections::{BTreeMap, BTreeSet};

use arris_math::Point3;

use crate::entity::{self, BodyKind, Coedge, EdgeGeometry, Loop};
use crate::error::NotFound;
use crate::euler::EulerLine;
use crate::handle::{self, Body};
use crate::id::{
    Curve2Id, CurveId, EdgeId, EntityId, EntityKind, FaceId, ShellId, SurfaceId, VertexId,
};
use crate::model::Model;
use crate::orientation::Orientation;

macro_rules! refs {
    ($( $(#[$doc:meta])* $name:ident => $prefix:literal ),* $(,)?) => {$(
        $(#[$doc])*
        ///
        /// A slot in the builder, not an arena id: the entity gets its
        /// [`VertexId`]/[`EdgeId`]/[`FaceId`] from [`Builder::finish`],
        /// which returns the map. Slots are reused by the next make after
        /// a kill, most recently freed first.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(u32);

        impl $name {
            /// The reference to slot `index`. The operators mint the ones
            /// that resolve; a test that needs a dangling one builds it here.
            pub const fn new(index: u32) -> Self {
                Self(index)
            }

            /// The slot index.
            pub const fn index(self) -> u32 {
                self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}{}", $prefix, self.0)
            }
        }
    )*};
}

refs! {
    /// A vertex under construction.
    VertexRef => "v",
    /// An edge under construction.
    EdgeRef => "e",
    /// A face under construction.
    FaceRef => "f",
}

/// A place in a loop: the junction before coedge `coedge_index` of loop
/// `loop_index` of `face`, which is the effective start vertex of that
/// coedge. `coedge_index` may equal the loop's length, naming the junction
/// after its last coedge — the same vertex as index 0, and the same
/// insertion point for [`Builder::mev`], but a different split for
/// [`Builder::mef`]: the coedges from `to` up to `from` go to the new face,
/// so `to = 0, from = n` moves them all and `to = from` moves none. For
/// the full turn from any other junction `s`, `from` is `s + n` — the one
/// place a position exceeds the length, and only as `mef`'s `from`.
///
/// Operators take positions rather than vertices because a vertex may
/// occur several times in one loop — at a seam, at a closed edge — and
/// only the position says which occurrence is meant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Position {
    /// The face.
    pub face: FaceRef,
    /// Which loop of the face, in the builder's canonical order.
    pub loop_index: usize,
    /// Which junction of the loop, `0..=len`.
    pub coedge_index: usize,
}

impl Position {
    /// The junction before coedge `coedge_index` of loop `loop_index` of
    /// `face`.
    pub const fn new(face: FaceRef, loop_index: usize, coedge_index: usize) -> Self {
        Position {
            face,
            loop_index,
            coedge_index,
        }
    }
}

impl fmt::Display for Position {
    /// `f3/0/2`: face, loop, coedge.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}/{}", self.face, self.loop_index, self.coedge_index)
    }
}

/// One use of an edge by a loop under construction: the edge, the
/// *effective* direction it is walked in (as seen from outside the
/// material), and the pcurve of this use on the face's surface, if the
/// caller has given it yet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Use {
    /// The edge.
    pub edge: EdgeRef,
    /// Along (`Forward`) or against (`Reversed`) the edge's curve, as seen
    /// from outside with the face's outward normal up.
    pub orientation: Orientation,
    /// The pcurve of this use on the face's surface; `None` until
    /// [`Builder::set_pcurve`] or the operator that made the use gave it.
    /// [`Builder::finish`] refuses a `None`.
    pub pcurve: Option<Curve2Id>,
}

impl Use {
    const fn key(&self) -> (EdgeRef, Orientation) {
        (self.edge, self.orientation)
    }
}

/// What a loop sorts by within its face: loops without uses first, by
/// their seed, then the rest by their first use — so a face's loop order
/// is a function of its content alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum LoopKey {
    Seed(VertexRef),
    Use(EdgeRef, Orientation),
}

impl fmt::Display for Use {
    /// `+e3 p7`, or `+e3 ?` without a pcurve.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}{} ", self.orientation, self.edge)?;
        match self.pcurve {
            Some(p) => write!(f, "{p}"),
            None => f.write_str("?"),
        }
    }
}

/// A vertex under construction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StagedVertex {
    point: Point3,
    tolerance: f64,
    kept: Option<VertexId>,
}

impl StagedVertex {
    /// The point.
    pub const fn point(&self) -> Point3 {
        self.point
    }

    /// The tolerance it will be stored with.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// The arena vertex this slot *is*, when [`Builder::assemble`] took it
    /// from the model and no operator has touched it since: [`Builder::finish`]
    /// appends nothing for it and returns this id. `None` for a slot that
    /// will be appended.
    pub const fn kept(&self) -> Option<VertexId> {
        self.kept
    }
}

/// An edge under construction: its geometry and its two vertices, the
/// start at the lower end of the range.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StagedEdge {
    geometry: EdgeGeometry,
    start: VertexRef,
    end: VertexRef,
    tolerance: f64,
    kept: Option<EdgeId>,
}

impl StagedEdge {
    /// The geometry.
    pub const fn geometry(&self) -> EdgeGeometry {
        self.geometry
    }

    /// The vertex at the lower end of the range.
    pub const fn start(&self) -> VertexRef {
        self.start
    }

    /// The vertex at the upper end of the range.
    pub const fn end(&self) -> VertexRef {
        self.end
    }

    /// The tolerance it will be stored with.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// The arena edge this slot *is*, as [`StagedVertex::kept`].
    pub const fn kept(&self) -> Option<EdgeId> {
        self.kept
    }
}

/// A loop under construction: its uses in effective walking order, kept
/// rotated so that the use with the lowest `(edge, orientation)` comes
/// first. A loop with no uses is one vertex, `seed` — what
/// [`Builder::mvfs`] starts from and what a kill leaves behind; `finish`
/// refuses it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StagedLoop {
    uses: Vec<Use>,
    seed: VertexRef,
}

impl StagedLoop {
    /// The uses in walking order.
    pub fn uses(&self) -> &[Use] {
        &self.uses
    }

    /// The vertex a loop without uses stands at.
    pub const fn seed(&self) -> VertexRef {
        self.seed
    }

    fn key(&self) -> LoopKey {
        match self.uses.first() {
            Some(u) => LoopKey::Use(u.edge, u.orientation),
            None => LoopKey::Seed(self.seed),
        }
    }

    fn canonicalise(&mut self) {
        let Some(at) = (0..self.uses.len()).min_by_key(|&i| self.uses[i].key()) else {
            return;
        };
        self.uses.rotate_left(at);
    }
}

/// A face under construction: its surface, the orientation the shell will
/// use it with, the shell it belongs to, and its loops in the builder's
/// canonical order (loops without uses first, by their seed; then by their
/// first use).
#[derive(Debug, Clone, PartialEq)]
pub struct StagedFace {
    surface: SurfaceId,
    orientation: Orientation,
    shell: usize,
    loops: Vec<StagedLoop>,
    tolerance: f64,
    kept: Option<FaceId>,
}

impl StagedFace {
    /// The surface.
    pub const fn surface(&self) -> SurfaceId {
        self.surface
    }

    /// The shell the face belongs to: its index in the
    /// [`Assembly::shells`] the builder was assembled from, and `0` for a
    /// builder of operators, which makes one shell. A face an operator
    /// makes out of another belongs to that face's shell.
    pub const fn shell(&self) -> usize {
        self.shell
    }

    /// `Forward` when the surface's normal is the outward one, `Reversed`
    /// when it points into the material.
    pub const fn orientation(&self) -> Orientation {
        self.orientation
    }

    /// The loops in canonical order.
    pub fn loops(&self) -> &[StagedLoop] {
        &self.loops
    }

    /// The tolerance it will be stored with.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// The arena face this slot *is*, as [`StagedVertex::kept`]. An
    /// operator that changes the face drops the mark, and `finish` then
    /// appends a new face carrying the change.
    pub const fn kept(&self) -> Option<FaceId> {
        self.kept
    }

    fn canonicalise(&mut self) {
        for lp in &mut self.loops {
            lp.canonicalise();
        }
        self.loops.sort_by_key(StagedLoop::key);
    }
}

/// What [`Builder::mvfs`] makes and [`Builder::kvfs`] returns: the first
/// vertex, and the surface and orientation of the first face.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Seed {
    /// The vertex's point.
    pub point: Point3,
    /// The face's surface.
    pub surface: SurfaceId,
    /// The orientation the shell will use the face with.
    pub orientation: Orientation,
}

/// What [`Builder::mev`] makes and [`Builder::kev`] returns: a new vertex
/// and the edge from the position's vertex to it, used twice in a row by
/// the loop — away and back.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Strut {
    /// The new vertex's point.
    pub point: Point3,
    /// The edge's geometry; its range runs from the existing vertex to the
    /// new one. Never `Degenerate`: a strut has two vertices.
    pub geometry: EdgeGeometry,
    /// The pcurves of the two uses on the face's surface: the `Forward`
    /// use away from the position, then the `Reversed` use back. A seam is
    /// exactly a strut whose two pcurves differ by the period.
    pub pcurves: [Option<Curve2Id>; 2],
}

/// What [`Builder::mef`] makes and [`Builder::kef`] returns: an edge
/// across a loop and the face on its left.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Split {
    /// The edge's geometry; its range runs from `from`'s vertex to `to`'s.
    /// `Degenerate` only when both are the same vertex.
    pub geometry: EdgeGeometry,
    /// The new face's surface.
    pub surface: SurfaceId,
    /// The orientation the shell will use the new face with.
    pub orientation: Orientation,
    /// The pcurves of the edge's two uses: `Forward` on the new face,
    /// `Reversed` on the old one.
    pub pcurves: [Option<Curve2Id>; 2],
}

/// What [`Builder::mekr`] makes and [`Builder::kemr`] returns: an edge
/// joining two loops of one face into one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Join {
    /// The edge's geometry; its range runs from `from`'s vertex to `to`'s.
    pub geometry: EdgeGeometry,
    /// The pcurves of the two uses, both on the face's surface: `Forward`
    /// from `from` to `to`, then `Reversed` back.
    pub pcurves: [Option<Curve2Id>; 2],
}

/// The counts of a builder and the genus it has made with
/// [`Builder::kfmrh`]: the Euler–Poincaré line
/// `V − E + F − (L − F) − 2(S − G)`, which every operator keeps at zero.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Counts {
    /// Live vertices.
    pub vertices: usize,
    /// Live edges, less every degenerate edge: a singular point of its
    /// face, which the Euler–Poincaré line does not count.
    pub edges: usize,
    /// Live faces.
    pub faces: usize,
    /// Loops over those faces.
    pub loops: usize,
    /// The shells the live faces belong to: one while a face exists for a
    /// builder of operators, the assembly's shells for an assembled one.
    pub shells: usize,
    /// Handles made by `kfmrh` and not yet removed by `mfkrh`.
    pub genus: usize,
}

impl Counts {
    /// `V − E + F − (L − F) − 2(S − G)`: zero for every state an operator
    /// leaves behind.
    pub const fn euler(&self) -> i64 {
        self.line().at_genus(self.genus as i64)
    }

    /// The Euler line of the counts, its genus derived from them rather
    /// than the handles made.
    pub const fn line(&self) -> EulerLine {
        EulerLine::new(
            self.vertices,
            self.edges,
            self.faces,
            self.loops,
            self.shells,
        )
    }
}

impl fmt::Display for Counts {
    /// `V/E/F/L/S g<G> = <euler>`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}/{}/{}/{}/{} g{} = {}",
            self.vertices,
            self.edges,
            self.faces,
            self.loops,
            self.shells,
            self.genus,
            self.euler()
        )
    }
}

/// What [`Builder::finish`] appended: the body, its shells, and the arena
/// id of every slot — what an operation's provenance is built from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Built {
    /// The body, `Forward`.
    pub body: Body,
    /// Its shells, one per shell index the faces carry, in index order —
    /// the order the body stores them in. One for a builder of operators.
    pub shells: Vec<ShellId>,
    /// Slot → arena id, every live vertex.
    pub vertices: BTreeMap<VertexRef, VertexId>,
    /// Slot → arena id, every live edge.
    pub edges: BTreeMap<EdgeRef, EdgeId>,
    /// Slot → arena id, every live face.
    pub faces: BTreeMap<FaceRef, FaceId>,
}

/// Why an operator or [`Builder::finish`] refused. Every variant names
/// the slot, position or edge it is about; none is a panic.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum BuildError {
    /// `mvfs` on a builder that already holds a body.
    #[error("the builder already holds a body; mvfs starts one from nothing")]
    NotEmpty,
    /// `finish` on a builder holding nothing.
    #[error("the builder holds no face")]
    Empty,
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
    /// `finish` makes solids: Euler operators build closed surfaces, and
    /// no operation returns another kind.
    #[error("the builder makes a solid, not a {0} body")]
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
    /// A geometry id a slot references does not resolve in the model.
    #[error(transparent)]
    NotFound(#[from] NotFound),
    /// [`Builder::assemble`]: a key names a spec the assembly does not have.
    #[error("the assembly has no {kind} spec at index {index}")]
    NoSpec {
        /// The kind of spec the key was for.
        kind: EntityKind,
        /// The index it named.
        index: usize,
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

/// Tombstoned slots with a LIFO free list: a kill leaves a hole and the
/// next make of the same kind fills the most recently freed one, so a
/// kill undone by its make gives the slot back and a make undone by its
/// kill leaves only a hole.
#[derive(Debug, Clone)]
struct Slots<T> {
    items: Vec<Option<T>>,
    free: Vec<u32>,
}

impl<T> Default for Slots<T> {
    fn default() -> Self {
        Slots {
            items: Vec::new(),
            free: Vec::new(),
        }
    }
}

impl<T> Slots<T> {
    fn insert(&mut self, value: T) -> u32 {
        if let Some(i) = self.free.pop() {
            self.items[i as usize] = Some(value);
            return i;
        }
        // Slots are addressed by `u32`, as the arena's are; running out is
        // resource exhaustion, not a geometric condition.
        let i = u32::try_from(self.items.len()).expect("the builder holds at most u32::MAX slots");
        self.items.push(Some(value));
        i
    }

    fn remove(&mut self, i: u32) -> Option<T> {
        let value = self.items.get_mut(i as usize)?.take();
        if value.is_some() {
            self.free.push(i);
        }
        value
    }

    fn get(&self, i: u32) -> Option<&T> {
        self.items.get(i as usize)?.as_ref()
    }

    fn get_mut(&mut self, i: u32) -> Option<&mut T> {
        self.items.get_mut(i as usize)?.as_mut()
    }

    /// Live slots in index order.
    fn iter(&self) -> impl Iterator<Item = (u32, &T)> {
        self.items
            .iter()
            .enumerate()
            .filter_map(|(i, v)| v.as_ref().map(|v| (i as u32, v)))
    }

    fn len(&self) -> usize {
        self.items.iter().filter(|v| v.is_some()).count()
    }

    /// The index the next `insert` will use.
    fn next_slot(&self) -> u32 {
        self.free.last().copied().unwrap_or_else(|| {
            u32::try_from(self.items.len()).expect("the builder holds at most u32::MAX slots")
        })
    }

    fn clear(&mut self) {
        self.items.clear();
        self.free.clear();
    }
}

/// The uses from `from` up to but not including `to`, around the loop:
/// nothing when they are equal, `uses[from..]` then `uses[..to]` when
/// `from` is past `to`.
fn around(uses: &[Use], from: usize, to: usize) -> Vec<Use> {
    let n = uses.len();
    let (from, to) = (from.min(n), to.min(n));
    if from < to {
        uses[from..to].to_vec()
    } else if from > to {
        let mut v = uses[from..].to_vec();
        v.extend_from_slice(&uses[..to]);
        v
    } else {
        Vec::new()
    }
}

/// Which vertex an [`EdgeSpec`] is over: one already in the model, or the
/// one the assembly's `index`-th [`VertexSpec`] stands for — whichever
/// kind that spec is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VertexKey {
    /// The vertex with this arena id, kept.
    Kept(VertexId),
    /// The assembly's `0`-based vertex spec at this index.
    New(usize),
}

/// A vertex of an [`Assembly`]: one the model already holds, kept with its
/// id, or one to append.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VertexSpec {
    /// The vertex with this arena id, kept: its point and tolerance are the
    /// model's and [`Builder::finish`] appends nothing for it.
    Keep(VertexId),
    /// A vertex to append.
    New {
        /// Its point.
        point: Point3,
        /// The tolerance it is stored with.
        tolerance: f64,
    },
}

/// Which edge a [`UseSpec`] is over: one already in the model, or the one
/// the assembly's `index`-th [`EdgeSpec`] stands for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeKey {
    /// The edge with this arena id, kept.
    Kept(EdgeId),
    /// The assembly's `0`-based edge spec at this index.
    New(usize),
}

/// An edge of an [`Assembly`]: one the model already holds, kept with its
/// id, or one to append.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EdgeSpec {
    /// The edge with this arena id, kept: its geometry, vertices and
    /// tolerance are the model's, and its two vertices are kept with it.
    Keep(EdgeId),
    /// An edge to append.
    New {
        /// Its geometry.
        geometry: EdgeGeometry,
        /// The vertex at the lower end of the range.
        start: VertexKey,
        /// The vertex at the upper end.
        end: VertexKey,
        /// The tolerance it is stored with.
        tolerance: f64,
    },
}

/// One use of an edge by a loop of an assembled face, in the *effective*
/// orientation the Euler operators take: the direction the loop is walked
/// in as seen from outside the material, whatever the surface's own normal
/// (`docs/DATA-MODEL.md` §Orientation).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UseSpec {
    /// The edge.
    pub edge: EdgeKey,
    /// Along (`Forward`) or against (`Reversed`) the edge's curve.
    pub orientation: Orientation,
    /// The pcurve of this use on the face's surface.
    pub pcurve: Curve2Id,
}

/// The uses of one loop the other way it can be walked: from stored order
/// to *effective* order (as seen from outside the material, whatever the
/// surface's own normal, `docs/DATA-MODEL.md` §Orientation) or back —
/// composing an orientation with itself and reversing a list are each
/// their own inverse, so one function walks both ways. Every orientation
/// is composed with `face_orientation`, and the whole walk is reversed
/// when `face_orientation` is `Reversed`.
pub fn effective_uses<T>(
    face_orientation: Orientation,
    uses: impl IntoIterator<Item = (T, Orientation, Curve2Id)>,
) -> Vec<(T, Orientation, Curve2Id)> {
    let mut out: Vec<(T, Orientation, Curve2Id)> = uses
        .into_iter()
        .map(|(id, orientation, pcurve)| (id, face_orientation.compose(orientation), pcurve))
        .collect();
    if face_orientation.is_reversed() {
        out.reverse();
    }
    out
}

/// A face of an [`Assembly`]: one the model already holds, kept whole with
/// its id, or one to append.
#[derive(Debug, Clone, PartialEq)]
pub enum FaceSpec {
    /// The face with this arena id, used by its new shell with this
    /// orientation, kept: its surface, loops, pcurves and tolerance are the
    /// model's, and every edge and vertex it names is kept with it.
    Keep(handle::Face),
    /// A face to append.
    New {
        /// Its surface.
        surface: SurfaceId,
        /// `Forward` when the surface's normal is the outward one.
        orientation: Orientation,
        /// Its loops, each a walk in effective orientation. The first is
        /// no more the outer one than any other: a loop's role is the
        /// pcurves' business, not the builder's.
        loops: Vec<Vec<UseSpec>>,
        /// The tolerance it is stored with.
        tolerance: f64,
    },
}

impl FaceSpec {
    /// The [`FaceSpec::New`] that reads `face_use` whole from `model`: its
    /// surface, tolerance, and every loop's uses in effective order, each
    /// edge named through `edge_key` — what an operation that copies or
    /// moves a face whole wants, differing from another only in how it
    /// names an edge. Errors: `face_use` does not resolve.
    pub fn from_face(
        model: &Model,
        face_use: handle::Face,
        mut edge_key: impl FnMut(EdgeId) -> EdgeKey,
    ) -> Result<FaceSpec, NotFound> {
        let entity = model.face(face_use.id)?;
        let loops = entity
            .loops()
            .iter()
            .map(|l| {
                effective_uses(
                    face_use.orientation,
                    l.coedges()
                        .iter()
                        .map(|c| (c.edge(), c.orientation(), c.pcurve())),
                )
                .into_iter()
                .map(|(edge, orientation, pcurve)| UseSpec {
                    edge: edge_key(edge),
                    orientation,
                    pcurve,
                })
                .collect()
            })
            .collect();
        Ok(FaceSpec::New {
            surface: entity.surface(),
            orientation: face_use.orientation,
            loops,
            tolerance: entity.tolerance(),
        })
    }
}

/// The faces of one body, grouped into its shells, and the edges and
/// vertices they are over, each kept from the model or new: what
/// [`Builder::assemble`] takes.
///
/// The lists are addressed positionally by [`VertexKey::New`] and
/// [`EdgeKey::New`], so a caller that has a slot per entity of its
/// operands — which is what a boolean has — writes the table it already
/// holds. A `Keep` spec and a `Kept` key that name one arena entity are
/// one slot; naming an entity by two `Keep` specs is
/// [`BuildError::Duplicate`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Assembly {
    /// The vertices, addressed by [`VertexKey::New`].
    pub vertices: Vec<VertexSpec>,
    /// The edges, addressed by [`EdgeKey::New`].
    pub edges: Vec<EdgeSpec>,
    /// The body's shells in the order the body stores them, each the
    /// faces it uses in order. A shell shares no edge and no vertex with
    /// another (`docs/DATA-MODEL.md` §Entities: a solid's lumps).
    pub shells: Vec<Vec<FaceSpec>>,
}

/// The slot [`Builder::assemble`] gave to each spec of the [`Assembly`] it
/// took, in spec order: a caller that has a slot per entity of its own
/// operands — a boolean, a sweep, a transform — looks its output up as
/// `built.vertices[&slots.vertices[i]]` rather than assuming the builder's
/// slot order matches the spec order, which interleaved `Keep` and `New`
/// specs need not.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct AssemblySlots {
    /// The slot of `assembly.vertices[i]`, at index `i`.
    pub vertices: Vec<VertexRef>,
    /// The slot of `assembly.edges[i]`, at index `i`.
    pub edges: Vec<EdgeRef>,
    /// The slot of `assembly.shells[s][i]`, at `[s][i]`.
    pub faces: Vec<Vec<FaceRef>>,
}

/// How [`Assembly::of_body`] treats a body's geometry as it copies it,
/// each asked once per occurrence: the identity ([`KeepGeometry`]) keeps
/// it; a transform moves it and, for a curve or surface, adds the result
/// (a vertex's point is a value, not an id, and has no arena entry of its
/// own to add).
pub trait GeometryRemap {
    /// The point to give a copied vertex, in place of `p`.
    fn point(&mut self, model: &mut Model, p: Point3) -> Point3;
    /// The curve to describe an edge over `c` with.
    fn curve(&mut self, model: &mut Model, c: CurveId) -> CurveId;
    /// The surface to describe a face over `s` with.
    fn surface(&mut self, model: &mut Model, s: SurfaceId) -> SurfaceId;
    /// The pcurve to give a copied use in place of `p`, which lay on
    /// `surface`, the *original* surface (not the one
    /// [`GeometryRemap::surface`] returned). The default keeps `p`: right
    /// wherever the surface's parametrisation is carried along, as a rigid
    /// motion does.
    fn pcurve(&mut self, _model: &mut Model, p: Curve2Id, _surface: SurfaceId) -> Curve2Id {
        p
    }
    /// What to change about a copied face over `surface`, the original
    /// surface. The default is [`FaceRemap::KEEP`]: the face is copied as
    /// it stands.
    fn face(&mut self, _model: &mut Model, _surface: SurfaceId) -> FaceRemap {
        FaceRemap::KEEP
    }
}

/// What [`Assembly::of_body`] changes about a face it copies, answered by
/// [`GeometryRemap::face`]: the two facts a map that reverses the winding
/// of a surface's parameters (a mirror, ADR-0031) needs said.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FaceRemap {
    /// The copy's use of its surface has the opposite orientation.
    pub toggle_use: bool,
    /// The copy's *stored* loops are the original's walked backwards, each
    /// coedge use's orientation toggled: the winding a stored loop must
    /// have in the `(u, v)` of a surface whose parameters were reflected.
    pub reverse_loops: bool,
}

impl FaceRemap {
    /// Change nothing: what a rigid motion asks.
    pub const KEEP: FaceRemap = FaceRemap {
        toggle_use: false,
        reverse_loops: false,
    };
}

/// A [`GeometryRemap`] that keeps every point, curve and surface as it is:
/// what a caller that copies a body's geometry unchanged passes.
#[derive(Debug, Clone, Copy, Default)]
pub struct KeepGeometry;

impl GeometryRemap for KeepGeometry {
    fn point(&mut self, _model: &mut Model, p: Point3) -> Point3 {
        p
    }

    fn curve(&mut self, _model: &mut Model, c: CurveId) -> CurveId {
        c
    }

    fn surface(&mut self, _model: &mut Model, s: SurfaceId) -> SurfaceId {
        s
    }
}

/// Where [`Assembly::of_body`] put each entity of the body it read: which
/// spec of the [`Assembly`], or, for a face, which shell and which spec
/// within it. What provenance is built from, reading the built id behind
/// a spec out of the matching [`AssemblySlots`].
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct BodyIndex {
    /// The vertex's spec index in [`Assembly::vertices`].
    pub vertices: BTreeMap<VertexId, usize>,
    /// The edge's spec index in [`Assembly::edges`].
    pub edges: BTreeMap<EdgeId, usize>,
    /// The face's shell index and spec index within it, into
    /// [`Assembly::shells`].
    pub faces: BTreeMap<FaceId, (usize, usize)>,
    /// The shell's index into [`Assembly::shells`].
    pub shells: BTreeMap<ShellId, usize>,
}

impl Assembly {
    /// The [`Assembly`] that describes `body` shell by shell with every
    /// entity `New`: a copy of its vertices, edges and faces, each curve
    /// and surface named through `remap` (the identity, [`KeepGeometry`],
    /// for a caller that wants the same geometry). What [`transform`]
    /// builds over, its own work reduced to `remap` and the provenance
    /// [`BodyIndex`] gives.
    ///
    /// Errors: `body` does not resolve.
    ///
    /// [`transform`]: ../../arris_ops/fn.transform.html
    ///
    /// ```
    /// use arris_topo::builder::{Assembly, Builder, KeepGeometry};
    /// use arris_topo::entity::BodyKind;
    /// use arris_topo::Model;
    /// use arris_debug::{dump_text, sample};
    ///
    /// let mut m = Model::default();
    /// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
    /// let before = dump_text(&m, body)?;
    /// let (assembly, _index) = Assembly::of_body(&mut m, body, &mut KeepGeometry)?;
    /// let (b, _slots) = Builder::assemble(&m, m.precision().default_tolerance, assembly)?;
    /// let built = b.finish(&mut m, BodyKind::Solid)?;
    /// assert_ne!(built.body.id, body.id, "every entity is new");
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn of_body(
        model: &mut Model,
        body: Body,
        remap: &mut impl GeometryRemap,
    ) -> Result<(Assembly, BodyIndex), NotFound> {
        let closure = model.closure(body)?;

        let mut curve_of: BTreeMap<CurveId, CurveId> = BTreeMap::new();
        for &c in &closure.curves {
            curve_of.insert(c, remap.curve(model, c));
        }
        let mut surface_of: BTreeMap<SurfaceId, SurfaceId> = BTreeMap::new();
        for &s in &closure.surfaces {
            surface_of.insert(s, remap.surface(model, s));
        }

        let mut vertex_index = BTreeMap::new();
        let mut vertices = Vec::with_capacity(closure.vertices.len());
        for (i, &v) in closure.vertices.iter().enumerate() {
            let old = *model.vertex(v)?;
            vertices.push(VertexSpec::New {
                point: remap.point(model, old.point()),
                tolerance: old.tolerance(),
            });
            vertex_index.insert(v, i);
        }

        let mut edge_index = BTreeMap::new();
        let mut edges = Vec::with_capacity(closure.edges.len());
        for (i, &e) in closure.edges.iter().enumerate() {
            let old = *model.edge(e)?;
            let geometry = match old.geometry() {
                EdgeGeometry::Curve { curve, range } => EdgeGeometry::Curve {
                    curve: curve_of[&curve],
                    range,
                },
                degenerate @ EdgeGeometry::Degenerate { .. } => degenerate,
            };
            edges.push(EdgeSpec::New {
                geometry,
                start: VertexKey::New(vertex_index[&old.start()]),
                end: VertexKey::New(vertex_index[&old.end()]),
                tolerance: old.tolerance(),
            });
            edge_index.insert(e, i);
        }

        let mut face_index: BTreeMap<FaceId, (usize, usize)> = BTreeMap::new();
        let mut shell_index: BTreeMap<ShellId, usize> = BTreeMap::new();
        let mut shells = Vec::new();
        for (si, shell) in model.shells(body)?.into_iter().enumerate() {
            let uses: Vec<handle::Face> = model
                .shell(shell.id)?
                .faces()
                .iter()
                .map(|f| f.oriented_by(shell.orientation))
                .collect();
            let mut face_specs = Vec::with_capacity(uses.len());
            for (fi, f) in uses.iter().enumerate() {
                let old_surface = model.face(f.id)?.surface();
                let mut spec = FaceSpec::from_face(model, *f, |id| EdgeKey::New(edge_index[&id]))?;
                let change = remap.face(model, old_surface);
                if let FaceSpec::New {
                    surface,
                    orientation,
                    loops,
                    ..
                } = &mut spec
                {
                    *surface = surface_of[&old_surface];
                    let old_orientation = *orientation;
                    if change.toggle_use {
                        *orientation = orientation.compose(Orientation::Reversed);
                    }
                    for l in loops.iter_mut() {
                        // Effective order back to stored, the change, and
                        // forward again under the new use orientation.
                        let mut stored = effective_uses(
                            old_orientation,
                            l.iter().map(|u| (u.edge, u.orientation, u.pcurve)),
                        );
                        if change.reverse_loops {
                            stored.reverse();
                            for u in &mut stored {
                                u.1 = u.1.compose(Orientation::Reversed);
                            }
                        }
                        *l = effective_uses(*orientation, stored)
                            .into_iter()
                            .map(|(edge, orientation, pcurve)| UseSpec {
                                edge,
                                orientation,
                                pcurve: remap.pcurve(model, pcurve, old_surface),
                            })
                            .collect();
                    }
                }
                face_specs.push(spec);
                face_index.insert(f.id, (si, fi));
            }
            shell_index.insert(shell.id, si);
            shells.push(face_specs);
        }

        Ok((
            Assembly {
                vertices,
                edges,
                shells,
            },
            BodyIndex {
                vertices: vertex_index,
                edges: edge_index,
                faces: face_index,
                shells: shell_index,
            },
        ))
    }
}

/// ` kept f3` for a slot [`Builder::assemble`] took from the model and no
/// operator has touched since, the empty string for one that will be
/// appended: what tells two [`Builder::dump`]s of the same shape apart.
fn kept(id: Option<impl fmt::Display>) -> String {
    id.map_or_else(String::new, |id| format!(" kept {id}"))
}

/// One body under construction, edited by the Euler operators and frozen
/// into a [`Model`] by [`Builder::finish`]. Module docs for the
/// guarantees.
///
/// The cylinder of `docs/DATA-MODEL.md` §Seams, as the primitive
/// builds it: a seed vertex on the bottom cap, the bottom circle as a
/// closed edge splitting off the wall, the seam as a strut whose two
/// pcurves differ by the period, the top circle splitting off the top
/// cap.
///
/// ```
/// use arris_topo::arris_geom::{Curve, Curve2, Surface};
/// use arris_topo::arris_math::{Frame, Frame2, Interval, Point2, Point3, Vec2};
/// use arris_topo::builder::{Builder, Position, Seed, Split, Strut};
/// use arris_topo::entity::{BodyKind, EdgeGeometry};
/// use arris_topo::{Model, Orientation};
/// use core::f64::consts::TAU;
///
/// let mut m = Model::default();
/// let (r, h) = (4.0, 12.0);
/// let base = Frame::world();
/// let top = base.with_origin(Point3::new(0.0, 0.0, h));
/// let wall = m.add_surface(Surface::Cylinder { frame: base, radius: r });
/// let bottom = m.add_surface(Surface::Plane { frame: base });
/// let top_plane = m.add_surface(Surface::Plane { frame: top });
/// let uv_line = |m: &mut Model, u, v, along_u: bool| m.add_curve2(Curve2::Line {
///     origin: Point2::new(u, v),
///     direction: if along_u { Vec2::x_axis() } else { Vec2::y_axis() },
/// });
/// let cap_circle = |m: &mut Model| m.add_curve2(Curve2::Circle { frame: Frame2::identity(), radius: r });
///
/// let mut b = Builder::new(m.precision().default_tolerance);
/// let (v0, f_bottom) = b.mvfs(Seed { point: Point3::new(r, 0.0, 0.0), surface: bottom, orientation: Orientation::Reversed })?;
/// let at = Position::new(f_bottom, 0, 0);
/// let circle = m.add_curve(Curve::Circle { frame: base, radius: r });
/// let (p_wall_bottom, p_cap_bottom) = (uv_line(&mut m, 0.0, 0.0, true), cap_circle(&mut m));
/// let (_e_bottom, f_wall) = b.mef(at, at, Split {
///     geometry: EdgeGeometry::Curve { curve: circle, range: Interval::TURN },
///     surface: wall,
///     orientation: Orientation::Forward,
///     pcurves: [Some(p_wall_bottom), Some(p_cap_bottom)],
/// })?;
/// let seam = m.add_curve(Curve::Line { origin: Point3::new(r, 0.0, 0.0), direction: arris_topo::arris_math::Vec3::z_axis() });
/// let (up, down) = (uv_line(&mut m, TAU, 0.0, false), uv_line(&mut m, 0.0, 0.0, false));
/// let (v1, _e_seam) = b.mev(b.find_position(f_wall, 0, v0)?, Strut {
///     point: Point3::new(r, 0.0, h),
///     geometry: EdgeGeometry::Curve { curve: seam, range: Interval::new(0.0, h).unwrap() },
///     pcurves: [Some(up), Some(down)],
/// })?;
/// let top_circle = m.add_curve(Curve::Circle { frame: top, radius: r });
/// let (p_wall_top, p_cap_top) = (uv_line(&mut m, 0.0, h, true), cap_circle(&mut m));
/// let at = b.find_position(f_wall, 0, v1)?;
/// let (_e_top, _f_top) = b.mef(at, at, Split {
///     geometry: EdgeGeometry::Curve { curve: top_circle, range: Interval::TURN },
///     surface: top_plane,
///     orientation: Orientation::Forward,
///     pcurves: [Some(p_cap_top), Some(p_wall_top)],
/// })?;
/// assert_eq!(b.counts().to_string(), "2/3/3/3/1 g0 = 0");
/// let built = b.finish(&mut m, BodyKind::Solid)?;
/// assert_eq!(m.faces(built.body)?.len(), 3);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
#[derive(Debug, Clone)]
pub struct Builder {
    tolerance: f64,
    vertices: Slots<StagedVertex>,
    edges: Slots<StagedEdge>,
    faces: Slots<StagedFace>,
    genus: usize,
}

impl Builder {
    /// An empty builder whose every entity will carry `tolerance` — a
    /// primitive passes the model's `default_tolerance`.
    pub fn new(tolerance: f64) -> Self {
        Builder {
            tolerance,
            vertices: Slots::default(),
            edges: Slots::default(),
            faces: Slots::default(),
            genus: 0,
        }
    }

    /// The tolerance every entity is made with.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// `true` before `mvfs` and after `kvfs`.
    pub fn is_empty(&self) -> bool {
        self.faces.len() == 0 && self.edges.len() == 0 && self.vertices.len() == 0
    }

    /// The counts and the Euler line they make. No degenerate edge is
    /// counted — a singular point of its face, not a boundary between two
    /// — by the one rule every Euler line keeps ([`EulerLine`]).
    pub fn counts(&self) -> Counts {
        let loops = self.faces.iter().map(|(_, f)| f.loops.len()).sum();
        let degenerate = self.edges.iter().filter(|&(_, e)| is_degenerate(e)).count();
        let shells = self
            .faces
            .iter()
            .map(|(_, f)| f.shell)
            .collect::<BTreeSet<_>>()
            .len();
        Counts {
            vertices: self.vertices.len(),
            edges: self.edges.len() - degenerate,
            faces: self.faces.len(),
            loops,
            shells,
            genus: self.genus,
        }
    }

    /// The vertex under `v`. Errors: the slot is not live.
    pub fn vertex(&self, v: VertexRef) -> Result<&StagedVertex, BuildError> {
        self.vertices.get(v.0).ok_or(BuildError::NoVertex(v))
    }

    /// The edge under `e`. Errors: the slot is not live.
    pub fn edge(&self, e: EdgeRef) -> Result<&StagedEdge, BuildError> {
        self.edges.get(e.0).ok_or(BuildError::NoEdge(e))
    }

    /// The face under `f`. Errors: the slot is not live.
    pub fn face(&self, f: FaceRef) -> Result<&StagedFace, BuildError> {
        self.faces.get(f.0).ok_or(BuildError::NoFace(f))
    }

    /// Every live vertex, in slot order.
    pub fn vertices(&self) -> impl Iterator<Item = (VertexRef, &StagedVertex)> {
        self.vertices.iter().map(|(i, v)| (VertexRef(i), v))
    }

    /// Every live edge, in slot order.
    pub fn edges(&self) -> impl Iterator<Item = (EdgeRef, &StagedEdge)> {
        self.edges.iter().map(|(i, e)| (EdgeRef(i), e))
    }

    /// Every live face, in slot order.
    pub fn faces(&self) -> impl Iterator<Item = (FaceRef, &StagedFace)> {
        self.faces.iter().map(|(i, f)| (FaceRef(i), f))
    }

    fn face_mut(&mut self, f: FaceRef) -> Result<&mut StagedFace, BuildError> {
        self.faces.get_mut(f.0).ok_or(BuildError::NoFace(f))
    }

    fn loop_ref(&self, f: FaceRef, loop_index: usize) -> Result<&StagedLoop, BuildError> {
        self.face(f)?
            .loops
            .get(loop_index)
            .ok_or(BuildError::NoLoop {
                face: f,
                loop_index,
            })
    }

    fn loop_mut(&mut self, f: FaceRef, loop_index: usize) -> Result<&mut StagedLoop, BuildError> {
        self.face_mut(f)?
            .loops
            .get_mut(loop_index)
            .ok_or(BuildError::NoLoop {
                face: f,
                loop_index,
            })
    }

    /// The effective start vertex of `u`.
    fn use_start(&self, u: &Use) -> Result<VertexRef, BuildError> {
        let e = self.edge(u.edge)?;
        Ok(match u.orientation {
            Orientation::Forward => e.start,
            Orientation::Reversed => e.end,
        })
    }

    /// The vertex at a junction: the effective start of the coedge there,
    /// or the seed of a loop without coedges. Errors: the position does
    /// not resolve or is past the loop's last junction.
    pub fn vertex_at(&self, at: Position) -> Result<VertexRef, BuildError> {
        let lp = self.loop_ref(at.face, at.loop_index)?;
        let n = lp.uses.len();
        if at.coedge_index > n {
            return Err(BuildError::BadPosition {
                position: at,
                len: n,
            });
        }
        if n == 0 {
            return Ok(lp.seed);
        }
        self.use_start(&lp.uses[at.coedge_index % n])
    }

    /// The one junction of the loop where `vertex` stands. Errors: it is
    /// at none, or at several (a seam vertex, a closed edge's vertex) —
    /// then the caller says which through [`Builder::vertex_at`].
    pub fn find_position(
        &self,
        face: FaceRef,
        loop_index: usize,
        vertex: VertexRef,
    ) -> Result<Position, BuildError> {
        let lp = self.loop_ref(face, loop_index)?;
        let n = lp.uses.len();
        let mut positions = Vec::new();
        if n == 0 {
            if lp.seed == vertex {
                positions.push(0);
            }
        } else {
            for (i, u) in lp.uses.iter().enumerate() {
                if self.use_start(u)? == vertex {
                    positions.push(i);
                }
            }
        }
        match positions.as_slice() {
            [i] => Ok(Position::new(face, loop_index, *i)),
            [] => Err(BuildError::NotInLoop {
                vertex,
                face,
                loop_index,
            }),
            _ => Err(BuildError::Ambiguous {
                vertex,
                face,
                loop_index,
                positions,
            }),
        }
    }

    /// Every use of `e`: `(face, loop, coedge, orientation)`, in face slot
    /// order.
    fn uses_of(&self, e: EdgeRef) -> Vec<(FaceRef, usize, usize, Orientation)> {
        let mut out = Vec::new();
        for (fi, face) in self.faces.iter() {
            for (li, lp) in face.loops.iter().enumerate() {
                for (ci, u) in lp.uses.iter().enumerate() {
                    if u.edge == e {
                        out.push((FaceRef(fi), li, ci, u.orientation));
                    }
                }
            }
        }
        out
    }

    /// Where the use `(edge, orientation)` is in `f`, after
    /// canonicalisation.
    fn locate(&self, f: FaceRef, key: (EdgeRef, Orientation)) -> Option<(usize, usize)> {
        let face = self.faces.get(f.0)?;
        for (li, lp) in face.loops.iter().enumerate() {
            if let Some(ci) = lp.uses.iter().position(|u| u.key() == key) {
                return Some((li, ci));
            }
        }
        None
    }

    /// The index of the loop of `f` that has no uses and stands at `seed`.
    fn empty_loop(&self, f: FaceRef, seed: VertexRef) -> Option<usize> {
        self.faces
            .get(f.0)?
            .loops
            .iter()
            .position(|lp| lp.uses.is_empty() && lp.seed == seed)
    }

    /// Puts `f`'s loops back in canonical rotation and order after an
    /// operator changed them, and drops the face's `Keep` mark: a face an
    /// operator has touched is no longer the arena's face, so `finish`
    /// appends it (`docs/DATA-MODEL.md` §Euler operators).
    fn canonicalise(&mut self, f: FaceRef) {
        if let Some(face) = self.faces.get_mut(f.0) {
            face.canonicalise();
            face.kept = None;
        }
    }

    /// The position of the use `(edge, orientation)` in `f`, or of the
    /// empty loop at `seed` when there is no such use. Both exist by
    /// construction after the kills that call this.
    fn position_after(
        &self,
        f: FaceRef,
        key: Option<(EdgeRef, Orientation)>,
        seed: VertexRef,
    ) -> Result<Position, BuildError> {
        let (li, ci) = key
            .and_then(|k| self.locate(f, k))
            .or_else(|| self.empty_loop(f, seed).map(|li| (li, 0)))
            .ok_or(BuildError::Invariant { face: f })?;
        Ok(Position::new(f, li, ci))
    }

    /// **mvfs** — make vertex, face, shell: the first vertex of a body, a
    /// face with one loop of no coedges standing at it, and the shell.
    /// Returns the vertex and the face; the loop is `Position::new(face,
    /// 0, 0)`. Errors: the builder already holds a body.
    pub fn mvfs(&mut self, seed: Seed) -> Result<(VertexRef, FaceRef), BuildError> {
        if !self.is_empty() {
            return Err(BuildError::NotEmpty);
        }
        let v = VertexRef(self.vertices.insert(StagedVertex {
            point: seed.point,
            tolerance: self.tolerance,
            kept: None,
        }));
        let f = FaceRef(self.faces.insert(StagedFace {
            surface: seed.surface,
            orientation: seed.orientation,
            shell: 0,
            loops: vec![StagedLoop {
                uses: Vec::new(),
                seed: v,
            }],
            tolerance: self.tolerance,
            kept: None,
        }));
        Ok((v, f))
    }

    /// **kvfs** — kill vertex, face, shell: the inverse of [`Builder::mvfs`],
    /// leaving the builder empty. Errors: the builder holds anything but
    /// one vertex and one face of one loop of no coedges.
    pub fn kvfs(&mut self) -> Result<Seed, BuildError> {
        let seed = match (
            self.vertices.iter().next(),
            self.faces.iter().next(),
            self.vertices.len(),
            self.edges.len(),
            self.faces.len(),
        ) {
            (Some((_, v)), Some((_, f)), 1, 0, 1)
                if f.loops.len() == 1 && f.loops[0].uses.is_empty() =>
            {
                Seed {
                    point: v.point,
                    surface: f.surface,
                    orientation: f.orientation,
                }
            }
            _ => return Err(BuildError::NotASeed),
        };
        self.vertices.clear();
        self.edges.clear();
        self.faces.clear();
        self.genus = 0;
        Ok(seed)
    }

    /// **mev** — make edge, vertex: a new vertex and the edge from the
    /// vertex at `at` to it, inserted at `at` as two consecutive uses —
    /// `Forward` away, `Reversed` back. Returns the vertex and the edge.
    /// Errors: `at` does not resolve; the strut is degenerate.
    pub fn mev(&mut self, at: Position, strut: Strut) -> Result<(VertexRef, EdgeRef), BuildError> {
        if matches!(strut.geometry, EdgeGeometry::Degenerate { .. }) {
            return Err(BuildError::DegenerateStrut);
        }
        let start = self.vertex_at(at)?;
        let v = VertexRef(self.vertices.insert(StagedVertex {
            point: strut.point,
            tolerance: self.tolerance,
            kept: None,
        }));
        let e = EdgeRef(self.edges.insert(StagedEdge {
            geometry: strut.geometry,
            start,
            end: v,
            tolerance: self.tolerance,
            kept: None,
        }));
        let lp = self.loop_mut(at.face, at.loop_index)?;
        let i = at.coedge_index.min(lp.uses.len());
        lp.uses.splice(
            i..i,
            [
                Use {
                    edge: e,
                    orientation: Orientation::Forward,
                    pcurve: strut.pcurves[0],
                },
                Use {
                    edge: e,
                    orientation: Orientation::Reversed,
                    pcurve: strut.pcurves[1],
                },
            ],
        );
        self.canonicalise(at.face);
        Ok((v, e))
    }

    /// **kev** — kill edge, vertex: the inverse of [`Builder::mev`]. The
    /// edge must be a strut — used twice in a row by one loop, `Forward`
    /// then `Reversed` — and its end vertex on nothing else; both are
    /// removed. Returns the position and the strut that remake them.
    /// Errors: not a strut; the end vertex is used by another edge or
    /// seeds an empty loop.
    pub fn kev(&mut self, edge: EdgeRef) -> Result<(Position, Strut), BuildError> {
        let e = *self.edge(edge)?;
        let uses = self.uses_of(edge);
        let (f, li, i) = match uses.as_slice() {
            [
                (fa, la, i, Orientation::Forward),
                (fb, lb, j, Orientation::Reversed),
            ]
            | [
                (fb, lb, j, Orientation::Reversed),
                (fa, la, i, Orientation::Forward),
            ] if fa == fb && la == lb => {
                let n = self.loop_ref(*fa, *la)?.uses.len();
                if (*i + 1) % n != *j || e.start == e.end {
                    return Err(BuildError::NotAStrut { edge });
                }
                (*fa, *la, *i)
            }
            _ => return Err(BuildError::NotAStrut { edge }),
        };
        let tip = e.end;
        for (k, other) in self.edges.iter() {
            if k != edge.0 && (other.start == tip || other.end == tip) {
                return Err(BuildError::VertexInUse {
                    vertex: tip,
                    other: EdgeRef(k),
                });
            }
        }
        for (fi, face) in self.faces.iter() {
            for (l, lp) in face.loops.iter().enumerate() {
                if lp.uses.is_empty() && lp.seed == tip {
                    return Err(BuildError::VertexSeeds {
                        vertex: tip,
                        face: FaceRef(fi),
                        loop_index: l,
                    });
                }
            }
        }
        let point = self.vertex(tip)?.point;
        let lp = self.loop_mut(f, li)?;
        let n = lp.uses.len();
        let j = (i + 1) % n;
        let following = (n > 2).then(|| lp.uses[(i + 2) % n].key());
        let (p0, p1) = (lp.uses[i].pcurve, lp.uses[j].pcurve);
        let (first, second) = if j > i { (j, i) } else { (i, j) };
        lp.uses.remove(first);
        lp.uses.remove(second);
        if lp.uses.is_empty() {
            lp.seed = e.start;
        }
        self.vertices.remove(tip.0);
        self.edges.remove(edge.0);
        self.canonicalise(f);
        let at = self.position_after(f, following, e.start)?;
        Ok((
            at,
            Strut {
                point,
                geometry: e.geometry,
                pcurves: [p0, p1],
            },
        ))
    }

    /// **mef** — make edge, face: an edge from the vertex at `from` to the
    /// vertex at `to`, both junctions of one loop, splitting it. The new
    /// face is on the edge's left: its loop is the edge `Forward` then the
    /// coedges from `to` around to `from`; the old loop keeps the rest
    /// after the edge `Reversed`. With `from == to` the new face is the
    /// edge alone (a closed edge splitting off a cap); with `from = to +
    /// len` it takes every coedge, the old face keeping the edge alone.
    /// Coedges that move keep their pcurve ids, which are on the old
    /// surface — [`Builder::set_pcurve`] is the caller's next call unless
    /// the surfaces are one. Returns the edge and the new face. Errors:
    /// the positions are not in one loop; `from` is past the loop and not
    /// `to + len`; a degenerate edge between two vertices.
    pub fn mef(
        &mut self,
        from: Position,
        to: Position,
        split: Split,
    ) -> Result<(EdgeRef, FaceRef), BuildError> {
        if from.face != to.face || from.loop_index != to.loop_index {
            return Err(BuildError::NotOneLoop { from, to });
        }
        let n = self.loop_ref(from.face, from.loop_index)?.uses.len();
        let (a, b) = (from.coedge_index, to.coedge_index);
        let full_turn = n > 0 && a == b + n;
        if a > n && !full_turn {
            return Err(BuildError::BadPosition {
                position: from,
                len: n,
            });
        }
        let end = self.vertex_at(to)?;
        let start = if full_turn {
            end
        } else {
            self.vertex_at(from)?
        };
        if matches!(split.geometry, EdgeGeometry::Degenerate { .. }) && start != end {
            return Err(BuildError::DegenerateEnds { start, end });
        }
        let shell = self.face(from.face)?.shell;
        let e = EdgeRef(self.edges.insert(StagedEdge {
            geometry: split.geometry,
            start,
            end,
            tolerance: self.tolerance,
            kept: None,
        }));
        let tolerance = self.tolerance;
        let lp = self.loop_mut(from.face, from.loop_index)?;
        let mut new_uses = vec![Use {
            edge: e,
            orientation: Orientation::Forward,
            pcurve: split.pcurves[0],
        }];
        let mut old_uses = vec![Use {
            edge: e,
            orientation: Orientation::Reversed,
            pcurve: split.pcurves[1],
        }];
        if full_turn {
            // Every coedge moves, the edge going in at the junction.
            let b = b.min(n);
            new_uses.extend_from_slice(&lp.uses[b..]);
            new_uses.extend_from_slice(&lp.uses[..b]);
        } else if a == b {
            // Every coedge stays, the edge going in at the junction.
            let a = a.min(n);
            old_uses.extend_from_slice(&lp.uses[a..]);
            old_uses.extend_from_slice(&lp.uses[..a]);
        } else {
            new_uses.extend(around(&lp.uses, b, a));
            old_uses.extend(around(&lp.uses, a, b));
        }
        lp.uses = old_uses;
        let f = FaceRef(self.faces.insert(StagedFace {
            surface: split.surface,
            orientation: split.orientation,
            shell,
            loops: vec![StagedLoop {
                uses: new_uses,
                seed: end,
            }],
            tolerance,
            kept: None,
        }));
        self.canonicalise(from.face);
        self.canonicalise(f);
        Ok((e, f))
    }

    /// **kef** — kill edge, face: the inverse of [`Builder::mef`]. The
    /// edge must be used once by each of two faces; the face on its left
    /// (the `Forward` use), which must have one loop, is removed and its
    /// other coedges take the edge's place in the other face's loop.
    /// Returns the two positions and the split that remake them. Errors:
    /// the edge does not separate two faces; the face to kill has rings.
    pub fn kef(&mut self, edge: EdgeRef) -> Result<(Position, Position, Split), BuildError> {
        let e = *self.edge(edge)?;
        let uses = self.uses_of(edge);
        let ((fa, la, i), (fb, lb, j)) = match uses.as_slice() {
            [
                (fa, la, i, Orientation::Forward),
                (fb, lb, j, Orientation::Reversed),
            ]
            | [
                (fb, lb, j, Orientation::Reversed),
                (fa, la, i, Orientation::Forward),
            ] if fa != fb => ((*fa, *la, *i), (*fb, *lb, *j)),
            _ => return Err(BuildError::NotSeparating { edge }),
        };
        let loops = self.face(fa)?.loops.len();
        if loops != 1 {
            return Err(BuildError::Rings { face: fa, loops });
        }
        let Some(killed) = self.faces.remove(fa.0) else {
            return Err(BuildError::NoFace(fa));
        };
        let mut spliced = killed.loops[la].uses.clone();
        spliced.rotate_left(i);
        let plus = spliced.remove(0);
        let first_spliced = spliced.first().map(Use::key);
        let count = spliced.len();
        let lp = self.loop_mut(fb, lb)?;
        let n = lp.uses.len();
        let minus = lp.uses[j];
        let following = (n > 1).then(|| lp.uses[(j + 1) % n].key());
        lp.uses.splice(j..=j, spliced);
        if lp.uses.is_empty() {
            lp.seed = e.start;
        }
        self.edges.remove(edge.0);
        self.canonicalise(fb);
        let to = self.position_after(fb, first_spliced.or(following), e.start)?;
        let n = self.loop_ref(fb, to.loop_index)?.uses.len();
        let mut from = to;
        if first_spliced.is_some() {
            if count == n {
                // The spliced block is the whole loop: `from = to + n` is
                // the split that moves every coedge back from there.
                from.coedge_index = to.coedge_index + n;
            } else {
                from.coedge_index = (to.coedge_index + count) % n;
            }
        }
        Ok((
            from,
            to,
            Split {
                geometry: e.geometry,
                surface: killed.surface,
                orientation: killed.orientation,
                pcurves: [plus.pcurve, minus.pcurve],
            },
        ))
    }

    /// **mekr** — make edge, kill ring: an edge from the vertex at `from`
    /// to the vertex at `to`, junctions of two loops of one face, joining
    /// them into one loop — `from`'s loop up to `from`, the edge
    /// `Forward`, all of `to`'s loop from `to` around, the edge
    /// `Reversed`, the rest of `from`'s loop. Returns the edge. Errors: the
    /// positions are not two loops of one face; a degenerate edge between
    /// two vertices.
    pub fn mekr(
        &mut self,
        from: Position,
        to: Position,
        join: Join,
    ) -> Result<EdgeRef, BuildError> {
        if from.face != to.face || from.loop_index == to.loop_index {
            return Err(BuildError::NotTwoLoops { from, to });
        }
        let start = self.vertex_at(from)?;
        let end = self.vertex_at(to)?;
        if matches!(join.geometry, EdgeGeometry::Degenerate { .. }) && start != end {
            return Err(BuildError::DegenerateEnds { start, end });
        }
        let e = EdgeRef(self.edges.insert(StagedEdge {
            geometry: join.geometry,
            start,
            end,
            tolerance: self.tolerance,
            kept: None,
        }));
        let face = self.face_mut(from.face)?;
        let ring = face.loops.remove(to.loop_index);
        let la = if to.loop_index < from.loop_index {
            from.loop_index - 1
        } else {
            from.loop_index
        };
        let lp = &mut face.loops[la];
        let a = from.coedge_index.min(lp.uses.len());
        let b = to.coedge_index.min(ring.uses.len());
        let mut merged = lp.uses[..a].to_vec();
        merged.push(Use {
            edge: e,
            orientation: Orientation::Forward,
            pcurve: join.pcurves[0],
        });
        merged.extend_from_slice(&ring.uses[b..]);
        merged.extend_from_slice(&ring.uses[..b]);
        merged.push(Use {
            edge: e,
            orientation: Orientation::Reversed,
            pcurve: join.pcurves[1],
        });
        merged.extend_from_slice(&lp.uses[a..]);
        lp.uses = merged;
        self.canonicalise(from.face);
        Ok(e)
    }

    /// **kemr** — kill edge, make ring: the inverse of [`Builder::mekr`].
    /// The edge must be used twice by one loop; removing it splits the
    /// loop at its two uses, the coedges between the `Forward` and the
    /// `Reversed` use becoming a second loop of the face (a ring, possibly
    /// a lone vertex). Returns the two positions and the join that remake
    /// them. Errors: the edge is not used twice by one loop.
    pub fn kemr(&mut self, edge: EdgeRef) -> Result<(Position, Position, Join), BuildError> {
        let e = *self.edge(edge)?;
        let uses = self.uses_of(edge);
        let (f, li, i, j) = match uses.as_slice() {
            [
                (fa, la, i, Orientation::Forward),
                (fb, lb, j, Orientation::Reversed),
            ]
            | [
                (fb, lb, j, Orientation::Reversed),
                (fa, la, i, Orientation::Forward),
            ] if fa == fb && la == lb => (*fa, *la, *i, *j),
            _ => return Err(BuildError::NotInOneLoop { edge }),
        };
        let lp = self.loop_mut(f, li)?;
        let n = lp.uses.len();
        let (plus, minus) = (lp.uses[i], lp.uses[j]);
        let ring = around(&lp.uses, (i + 1) % n, j);
        let rest = around(&lp.uses, (j + 1) % n, i);
        let (ring_first, rest_first) = (ring.first().map(Use::key), rest.first().map(Use::key));
        lp.uses = rest;
        if lp.uses.is_empty() {
            lp.seed = e.start;
        }
        self.face_mut(f)?.loops.push(StagedLoop {
            uses: ring,
            seed: e.end,
        });
        self.edges.remove(edge.0);
        self.canonicalise(f);
        let from = self.position_after(f, rest_first, e.start)?;
        let to = self.position_after(f, ring_first, e.end)?;
        Ok((
            from,
            to,
            Join {
                geometry: e.geometry,
                pcurves: [plus.pcurve, minus.pcurve],
            },
        ))
    }

    /// **kfmrh** — kill face, make ring, hole: `kill`, a face of one loop
    /// on the same surface as `into` and used with the opposite
    /// orientation (two coplanar faces with opposing outward normals and
    /// nothing between them), is removed and its loop becomes a ring of
    /// `into`, keeping its uses and pcurves; the body gains a handle.
    /// Returns the ring's index in `into`. Errors: the faces are one; the
    /// surfaces or orientations do not match; `kill` has rings.
    pub fn kfmrh(&mut self, kill: FaceRef, into: FaceRef) -> Result<usize, BuildError> {
        let k = self.face(kill)?;
        let t = self.face(into)?;
        if kill == into {
            return Err(BuildError::OneFace { face: kill });
        }
        if k.surface != t.surface {
            return Err(BuildError::SurfaceMismatch {
                kill,
                kill_surface: k.surface,
                into,
                into_surface: t.surface,
            });
        }
        if k.orientation == t.orientation {
            return Err(BuildError::SameOrientation { kill, into });
        }
        if k.loops.len() != 1 {
            return Err(BuildError::Rings {
                face: kill,
                loops: k.loops.len(),
            });
        }
        let Some(killed) = self.faces.remove(kill.0) else {
            return Err(BuildError::NoFace(kill));
        };
        let ring = killed.loops.into_iter().next().ok_or(BuildError::Rings {
            face: kill,
            loops: 0,
        })?;
        let key = ring.uses.first().map(Use::key);
        let seed = ring.seed;
        self.face_mut(into)?.loops.push(ring);
        self.genus += 1;
        self.canonicalise(into);
        Ok(self.position_after(into, key, seed)?.loop_index)
    }

    /// **mfkrh** — make face, kill ring, hole: the inverse of
    /// [`Builder::kfmrh`]. Loop `ring` of `face` is lifted out as the one
    /// loop of a new face on the same surface with the opposite
    /// orientation; the body loses a handle. Returns the new face. Errors:
    /// `face` has no such ring or no other loop; the genus is zero.
    pub fn mfkrh(&mut self, face: FaceRef, ring: usize) -> Result<FaceRef, BuildError> {
        let f = self.face(face)?;
        if f.loops.len() < 2 || ring >= f.loops.len() {
            return Err(BuildError::NoRing { face, ring });
        }
        if self.genus == 0 {
            return Err(BuildError::NoHandle);
        }
        let (surface, orientation, tolerance, shell) =
            (f.surface, f.orientation, f.tolerance, f.shell);
        let lifted = self.face_mut(face)?.loops.remove(ring);
        let new = FaceRef(self.faces.insert(StagedFace {
            surface,
            orientation: orientation.flipped(),
            shell,
            loops: vec![lifted],
            tolerance,
            kept: None,
        }));
        self.genus -= 1;
        self.canonicalise(face);
        self.canonicalise(new);
        Ok(new)
    }

    /// Gives the coedge at `at` its pcurve on the face's surface,
    /// returning the previous one. Errors: `at` does not name a coedge.
    pub fn set_pcurve(
        &mut self,
        at: Position,
        pcurve: Curve2Id,
    ) -> Result<Option<Curve2Id>, BuildError> {
        let lp = self.loop_mut(at.face, at.loop_index)?;
        let n = lp.uses.len();
        let Some(u) = lp.uses.get_mut(at.coedge_index) else {
            return Err(BuildError::BadPosition {
                position: at,
                len: n,
            });
        };
        let previous = u.pcurve.replace(pcurve);
        if let Some(face) = self.faces.get_mut(at.face.0) {
            face.kept = None;
        }
        Ok(previous)
    }

    /// The builder's state as text: every live slot in order with its
    /// geometry ids — a face's shell index too once there are several —
    /// every loop's uses, the genus and the counts line. Identical for two
    /// builders that went through the same calls, and restored byte for
    /// byte by an operator's inverse.
    pub fn dump(&self) -> String {
        use core::fmt::Write as _;
        let several_shells = self.counts().shells > 1;
        let mut out = String::new();
        let _ = writeln!(
            out,
            "builder tolerance {} genus {}",
            self.tolerance, self.genus
        );
        let _ = writeln!(out, "vertices");
        for (i, v) in self.vertices.iter() {
            let _ = writeln!(
                out,
                "  v{i} ({}, {}, {}) tol {}{}",
                v.point.x,
                v.point.y,
                v.point.z,
                v.tolerance,
                kept(v.kept)
            );
        }
        let _ = writeln!(out, "edges");
        for (i, e) in self.edges.iter() {
            let _ = write!(out, "  e{i} {} -> {} ", e.start, e.end);
            match e.geometry {
                EdgeGeometry::Curve { curve, range } => {
                    let _ = write!(out, "{curve} [{}, {}]", range.lo(), range.hi());
                }
                EdgeGeometry::Degenerate { range } => {
                    let _ = write!(out, "degenerate [{}, {}]", range.lo(), range.hi());
                }
            }
            let _ = writeln!(out, " tol {}{}", e.tolerance, kept(e.kept));
        }
        let _ = writeln!(out, "faces");
        for (i, f) in self.faces.iter() {
            let shell = if several_shells {
                format!(" shell {}", f.shell)
            } else {
                String::new()
            };
            let _ = writeln!(
                out,
                "  f{i} {} {}{shell} tol {}{}",
                f.surface,
                f.orientation,
                f.tolerance,
                kept(f.kept)
            );
            for (li, lp) in f.loops.iter().enumerate() {
                if lp.uses.is_empty() {
                    let _ = writeln!(out, "    loop {li} at {}", lp.seed);
                } else {
                    let _ = writeln!(out, "    loop {li}");
                }
                for u in &lp.uses {
                    let _ = writeln!(out, "      {u}");
                }
            }
        }
        let _ = writeln!(out, "counts {}", self.counts());
        out
    }

    /// Freezes the body into `model` as a `Solid`: every live vertex,
    /// edge and face appended in slot order — bar the slots
    /// [`Builder::assemble`] marked `Keep` and no operator has touched,
    /// which keep their arena id and append nothing — then one shell per
    /// shell index the faces carry, in index order and each over its faces
    /// in slot order, and the body over the shells, inside a transaction.
    /// A loop of a `Reversed` face is stored backwards with every use
    /// flipped, so every stored loop is counter-clockwise about its
    /// surface's normal. Returns the body, its shells and the slot → id
    /// maps.
    ///
    /// Errors, each leaving the model untouched: [`BuildError::Kind`] for
    /// any kind but `Solid` (the operators build closed surfaces, and no
    /// operation returns a sheet, wire or general body);
    /// [`BuildError::Empty`]; an [`BuildError::EmptyLoop`]; a
    /// [`BuildError::MissingPcurve`]; an edge not used exactly twice, or
    /// a degenerate one not exactly once ([`BuildError::EdgeUses`]), or used twice the same way
    /// ([`BuildError::SameDirection`]); a curve, surface, pcurve or kept
    /// entity id that does not resolve ([`BuildError::NotFound`]).
    pub fn finish(self, model: &mut Model, kind: BodyKind) -> Result<Built, BuildError> {
        match kind {
            BodyKind::Solid => {}
            BodyKind::Sheet | BodyKind::Wire | BodyKind::General => {
                return Err(BuildError::Kind(kind));
            }
        }
        if self.faces.len() == 0 {
            return Err(BuildError::Empty);
        }
        let mut uses: BTreeMap<EdgeRef, Vec<Orientation>> = BTreeMap::new();
        for (fi, face) in self.faces.iter() {
            model.surface(face.surface)?;
            if let Some(id) = face.kept {
                model.face(id)?;
            }
            for (li, lp) in face.loops.iter().enumerate() {
                if lp.uses.is_empty() {
                    return Err(BuildError::EmptyLoop {
                        face: FaceRef(fi),
                        loop_index: li,
                    });
                }
                for (ci, u) in lp.uses.iter().enumerate() {
                    let Some(p) = u.pcurve else {
                        return Err(BuildError::MissingPcurve {
                            position: Position::new(FaceRef(fi), li, ci),
                        });
                    };
                    model.curve2(p)?;
                    self.edge(u.edge)?;
                    uses.entry(u.edge).or_default().push(u.orientation);
                }
            }
        }
        for (ei, e) in self.edges.iter() {
            let edge = EdgeRef(ei);
            let list = uses.get(&edge).map_or(&[][..], Vec::as_slice);
            if list.len() != uses_wanted(e) {
                return Err(BuildError::EdgeUses {
                    edge,
                    uses: list.len(),
                });
            }
            if list.len() == 2 && list[0] == list[1] {
                return Err(BuildError::SameDirection { edge });
            }
            if let EdgeGeometry::Curve { curve, .. } = e.geometry {
                model.curve(curve)?;
            }
            if let Some(id) = e.kept {
                model.edge(id)?;
            }
            self.vertex(e.start)?;
            self.vertex(e.end)?;
        }
        for (_, v) in self.vertices.iter() {
            if let Some(id) = v.kept {
                model.vertex(id)?;
            }
        }
        model.transaction(|m| {
            let mut vertices = BTreeMap::new();
            for (i, v) in self.vertices.iter() {
                let id = match v.kept {
                    Some(id) => id,
                    None => m.push_vertex(entity::Vertex::new(v.point, v.tolerance)),
                };
                vertices.insert(VertexRef(i), id);
            }
            let mut edges = BTreeMap::new();
            for (i, e) in self.edges.iter() {
                let id = match e.kept {
                    Some(id) => id,
                    None => {
                        let start = *vertices
                            .get(&e.start)
                            .ok_or(BuildError::NoVertex(e.start))?;
                        let end = *vertices.get(&e.end).ok_or(BuildError::NoVertex(e.end))?;
                        m.push_edge(entity::Edge::new(e.geometry, start, end, e.tolerance))
                    }
                };
                edges.insert(EdgeRef(i), id);
            }
            let mut faces = BTreeMap::new();
            let mut shell_faces: BTreeMap<usize, Vec<handle::Face>> = BTreeMap::new();
            for (i, f) in self.faces.iter() {
                let id = match f.kept {
                    Some(id) => id,
                    None => {
                        let mut loops = Vec::with_capacity(f.loops.len());
                        for (li, lp) in f.loops.iter().enumerate() {
                            let mut effective = Vec::with_capacity(lp.uses.len());
                            for (ci, u) in lp.uses.iter().enumerate() {
                                let edge = *edges.get(&u.edge).ok_or(BuildError::NoEdge(u.edge))?;
                                let pcurve = u.pcurve.ok_or(BuildError::MissingPcurve {
                                    position: Position::new(FaceRef(i), li, ci),
                                })?;
                                effective.push((edge, u.orientation, pcurve));
                            }
                            let coedges = effective_uses(f.orientation, effective)
                                .into_iter()
                                .map(|(edge, orientation, pcurve)| {
                                    Coedge::new(edge, orientation, pcurve)
                                })
                                .collect();
                            loops.push(Loop::new(coedges));
                        }
                        m.push_face(entity::Face::new(f.surface, loops, f.tolerance))
                    }
                };
                faces.insert(FaceRef(i), id);
                shell_faces
                    .entry(f.shell)
                    .or_default()
                    .push(handle::Face::new(id, f.orientation));
            }
            let shells: Vec<ShellId> = shell_faces
                .into_values()
                .map(|uses| m.push_shell(entity::Shell::new(uses)))
                .collect();
            let body = m.push_body(entity::Body::new(
                kind,
                shells.iter().copied().map(handle::Shell::forward).collect(),
                Vec::new(),
                Vec::new(),
            ));
            Ok(Built {
                body: Body::forward(body),
                shells,
                vertices,
                edges,
                faces,
            })
        })
    }
}

/// The tables [`Builder::assemble`] resolves keys through.
#[derive(Default)]
struct Assembled {
    kept_vertices: BTreeMap<VertexId, VertexRef>,
    kept_edges: BTreeMap<EdgeId, EdgeRef>,
    kept_faces: BTreeMap<FaceId, FaceRef>,
    vertex_of: Vec<VertexRef>,
    edge_of: Vec<EdgeRef>,
}

impl Builder {
    /// A builder holding the body `assembly` describes: the builder's
    /// second entry point beside [`Builder::new`] and the operators, and
    /// the one an operation that computes its result's faces outright —
    /// a boolean, a sweep, a transform — uses (`docs/DATA-MODEL.md`
    /// §Euler operators, ADR-0004).
    ///
    /// Every entity is `Keep` or `New`. A `Keep` slot *is* the arena's
    /// entity: its geometry is read from `model`, [`Builder::finish`]
    /// appends nothing for it and returns its id, so an operation that
    /// leaves a face alone shares it with its input and its provenance
    /// records nothing. A kept face is kept whole — its loops, pcurves,
    /// edges and vertices come from the model, and the edges and vertices
    /// are kept with it. Any operator applied to a kept slot drops the
    /// mark, and `finish` appends that slot instead.
    ///
    /// What `assemble` proves, so that the result is a body each of whose
    /// shells the operators could have built: every loop has coedges and
    /// closes through effective vertices; every edge is used exactly twice
    /// and in opposite directions, or once when it is degenerate — a
    /// singular point, which the Euler–Poincaré line below does not count;
    /// no arena entity is kept twice; no shell
    /// is empty; no edge is used by, and no vertex is an end of edges of,
    /// two shells; the faces of each shell are one edge-connected
    /// component; and the Euler–Poincaré line of each shell closes at a
    /// whole genus, their sum becoming the builder's ([`Builder::counts`]).
    /// It does not prove how the shells nest — which one is outer and
    /// which a void inside it is geometry, the checker's B1.
    ///
    /// Errors, leaving nothing behind (the model is only read):
    /// [`BuildError::NoSpec`] for a key past its list;
    /// [`BuildError::NotFound`] for an id that does not resolve;
    /// [`BuildError::Empty`], [`BuildError::EmptyShell`],
    /// [`BuildError::Duplicate`], [`BuildError::EmptyLoop`],
    /// [`BuildError::LoopOpen`], [`BuildError::EdgeUses`],
    /// [`BuildError::SameDirection`], [`BuildError::SharedEdge`],
    /// [`BuildError::SharedVertex`], [`BuildError::Disconnected`],
    /// [`BuildError::NotClosed`].
    ///
    /// ```
    /// use arris_topo::builder::{Assembly, Builder, FaceSpec};
    /// use arris_topo::entity::BodyKind;
    /// use arris_topo::Model;
    /// use arris_debug::{dump_text, sample};
    ///
    /// // Every face of a body kept: the same entities, a new shell.
    /// let mut m = Model::default();
    /// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
    /// let faces = m.faces(body)?.into_iter().map(FaceSpec::Keep).collect();
    /// let assembly = Assembly { shells: vec![faces], ..Assembly::default() };
    /// let (b, _slots) = Builder::assemble(&m, m.precision().default_tolerance, assembly)?;
    /// assert_eq!(b.counts().to_string(), "2/3/3/3/1 g0 = 0");
    /// let again = b.finish(&mut m, BodyKind::Solid)?;
    /// assert_eq!(m.faces(again.body)?, m.faces(body)?, "the same faces, kept");
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn assemble(
        model: &Model,
        tolerance: f64,
        assembly: Assembly,
    ) -> Result<(Builder, AssemblySlots), BuildError> {
        let mut b = Builder::new(tolerance);
        let mut at = Assembled::default();
        for spec in &assembly.vertices {
            let slot = match *spec {
                VertexSpec::Keep(id) => b.keep_vertex(model, &mut at, id, true)?,
                VertexSpec::New { point, tolerance } => {
                    VertexRef(b.vertices.insert(StagedVertex {
                        point,
                        tolerance,
                        kept: None,
                    }))
                }
            };
            at.vertex_of.push(slot);
        }
        for spec in &assembly.edges {
            let slot = match *spec {
                EdgeSpec::Keep(id) => b.keep_edge(model, &mut at, id, true)?,
                EdgeSpec::New {
                    geometry,
                    start,
                    end,
                    tolerance,
                } => {
                    let start = b.vertex_key(model, &mut at, start)?;
                    let end = b.vertex_key(model, &mut at, end)?;
                    EdgeRef(b.edges.insert(StagedEdge {
                        geometry,
                        start,
                        end,
                        tolerance,
                        kept: None,
                    }))
                }
            };
            at.edge_of.push(slot);
        }
        let mut face_slots: Vec<Vec<FaceRef>> = Vec::with_capacity(assembly.shells.len());
        for (shell, specs) in assembly.shells.iter().enumerate() {
            if specs.is_empty() {
                return Err(BuildError::EmptyShell { shell });
            }
            let mut shell_slots = Vec::with_capacity(specs.len());
            for spec in specs {
                let slot = match spec {
                    FaceSpec::Keep(face) => b.keep_face(model, &mut at, *face, shell)?,
                    FaceSpec::New {
                        surface,
                        orientation,
                        loops,
                        tolerance,
                    } => {
                        model.surface(*surface)?;
                        let mut staged = Vec::with_capacity(loops.len());
                        for uses in loops {
                            let mut walk = Vec::with_capacity(uses.len());
                            for u in uses {
                                model.curve2(u.pcurve)?;
                                walk.push(Use {
                                    edge: b.edge_key(model, &mut at, u.edge)?,
                                    orientation: u.orientation,
                                    pcurve: Some(u.pcurve),
                                });
                            }
                            staged.push(b.staged_loop(walk, staged.len())?);
                        }
                        let mut face = StagedFace {
                            surface: *surface,
                            orientation: *orientation,
                            shell,
                            loops: staged,
                            tolerance: *tolerance,
                            kept: None,
                        };
                        face.canonicalise();
                        FaceRef(b.faces.insert(face))
                    }
                };
                shell_slots.push(slot);
            }
            face_slots.push(shell_slots);
        }
        b.genus = b.assembled_genus()?;
        let slots = AssemblySlots {
            vertices: at.vertex_of,
            edges: at.edge_of,
            faces: face_slots,
        };
        Ok((b, slots))
    }

    /// The slot of the kept vertex `id`, made on first mention. `named`
    /// marks the mention that is a [`VertexSpec::Keep`] of its own, which
    /// may not repeat one.
    fn keep_vertex(
        &mut self,
        model: &Model,
        at: &mut Assembled,
        id: VertexId,
        named: bool,
    ) -> Result<VertexRef, BuildError> {
        if let Some(&slot) = at.kept_vertices.get(&id) {
            if named {
                return Err(BuildError::Duplicate(id.into()));
            }
            return Ok(slot);
        }
        let v = model.vertex(id)?;
        let slot = VertexRef(self.vertices.insert(StagedVertex {
            point: v.point(),
            tolerance: v.tolerance(),
            kept: Some(id),
        }));
        at.kept_vertices.insert(id, slot);
        Ok(slot)
    }

    /// The slot of the kept edge `id`, made on first mention together with
    /// its two vertices.
    fn keep_edge(
        &mut self,
        model: &Model,
        at: &mut Assembled,
        id: EdgeId,
        named: bool,
    ) -> Result<EdgeRef, BuildError> {
        if let Some(&slot) = at.kept_edges.get(&id) {
            if named {
                return Err(BuildError::Duplicate(id.into()));
            }
            return Ok(slot);
        }
        let e = *model.edge(id)?;
        let start = self.keep_vertex(model, at, e.start(), false)?;
        let end = self.keep_vertex(model, at, e.end(), false)?;
        let slot = EdgeRef(self.edges.insert(StagedEdge {
            geometry: e.geometry(),
            start,
            end,
            tolerance: e.tolerance(),
            kept: Some(id),
        }));
        at.kept_edges.insert(id, slot);
        Ok(slot)
    }

    /// The slot of the kept face `face` in shell `shell`, with every edge
    /// and vertex it names kept too. A stored loop of a `Reversed` face is
    /// walked backwards with every use flipped — the inverse of what
    /// [`Builder::finish`] stores.
    fn keep_face(
        &mut self,
        model: &Model,
        at: &mut Assembled,
        face: handle::Face,
        shell: usize,
    ) -> Result<FaceRef, BuildError> {
        if at.kept_faces.contains_key(&face.id) {
            return Err(BuildError::Duplicate(face.id.into()));
        }
        let entity = model.face(face.id)?.clone();
        model.surface(entity.surface())?;
        let mut loops = Vec::with_capacity(entity.loops().len());
        for l in entity.loops() {
            let raw = l
                .coedges()
                .iter()
                .map(|c| (c.edge(), c.orientation(), c.pcurve()));
            let mut walk = Vec::with_capacity(l.coedges().len());
            for (edge, orientation, pcurve) in effective_uses(face.orientation, raw) {
                model.curve2(pcurve)?;
                walk.push(Use {
                    edge: self.keep_edge(model, at, edge, false)?,
                    orientation,
                    pcurve: Some(pcurve),
                });
            }
            let index = loops.len();
            loops.push(self.staged_loop(walk, index)?);
        }
        let mut staged = StagedFace {
            surface: entity.surface(),
            orientation: face.orientation,
            shell,
            loops,
            tolerance: entity.tolerance(),
            kept: Some(face.id),
        };
        staged.canonicalise();
        let slot = FaceRef(self.faces.insert(staged));
        at.kept_faces.insert(face.id, slot);
        Ok(slot)
    }

    /// The loop of `walk`, seeded at the effective start of its first use.
    /// `loop_index` is only what an error names, the face being the one
    /// about to be inserted. Errors: the walk is empty — an assembled loop
    /// is a walk, never the bare vertex [`Builder::mvfs`] leaves behind.
    fn staged_loop(&self, walk: Vec<Use>, loop_index: usize) -> Result<StagedLoop, BuildError> {
        let first = walk.first().ok_or(BuildError::EmptyLoop {
            face: FaceRef(self.faces.next_slot()),
            loop_index,
        })?;
        let seed = self.use_start(first)?;
        Ok(StagedLoop { uses: walk, seed })
    }

    fn vertex_key(
        &mut self,
        model: &Model,
        at: &mut Assembled,
        key: VertexKey,
    ) -> Result<VertexRef, BuildError> {
        match key {
            VertexKey::Kept(id) => self.keep_vertex(model, at, id, false),
            VertexKey::New(index) => at.vertex_of.get(index).copied().ok_or(BuildError::NoSpec {
                kind: EntityKind::Vertex,
                index,
            }),
        }
    }

    fn edge_key(
        &mut self,
        model: &Model,
        at: &mut Assembled,
        key: EdgeKey,
    ) -> Result<EdgeRef, BuildError> {
        match key {
            EdgeKey::Kept(id) => self.keep_edge(model, at, id, false),
            EdgeKey::New(index) => at.edge_of.get(index).copied().ok_or(BuildError::NoSpec {
                kind: EntityKind::Edge,
                index,
            }),
        }
    }

    /// The effective end vertex of `u`: the start of the same use walked
    /// the other way.
    fn use_end(&self, u: &Use) -> Result<VertexRef, BuildError> {
        let e = self.edge(u.edge)?;
        Ok(match u.orientation {
            Orientation::Forward => e.end,
            Orientation::Reversed => e.start,
        })
    }

    /// Proves the assembled slots are closed surfaces of whole genus, one
    /// per shell and sharing nothing, and returns the sum of their genera;
    /// the errors are [`Builder::assemble`]'s.
    fn assembled_genus(&self) -> Result<usize, BuildError> {
        let mut uses: BTreeMap<EdgeRef, Vec<(Orientation, FaceRef)>> = BTreeMap::new();
        for (fi, face) in self.faces.iter() {
            let face_ref = FaceRef(fi);
            for (li, lp) in face.loops.iter().enumerate() {
                if lp.uses.is_empty() {
                    return Err(BuildError::EmptyLoop {
                        face: face_ref,
                        loop_index: li,
                    });
                }
                let n = lp.uses.len();
                for (ci, u) in lp.uses.iter().enumerate() {
                    let previous = &lp.uses[(ci + n - 1) % n];
                    let (ended, starts) = (self.use_end(previous)?, self.use_start(u)?);
                    if ended != starts {
                        return Err(BuildError::LoopOpen {
                            face: face_ref,
                            loop_index: li,
                            coedge_index: ci,
                            ended,
                            starts,
                        });
                    }
                    uses.entry(u.edge)
                        .or_default()
                        .push((u.orientation, face_ref));
                }
            }
        }
        for (ei, e) in self.edges.iter() {
            let edge = EdgeRef(ei);
            let list = uses.get(&edge).map_or(&[][..], Vec::as_slice);
            if list.len() != uses_wanted(e) {
                return Err(BuildError::EdgeUses {
                    edge,
                    uses: list.len(),
                });
            }
            if matches!(list, [a, b] if a.0 == b.0) {
                return Err(BuildError::SameDirection { edge });
            }
        }
        if self.faces.len() == 0 {
            return Err(BuildError::Empty);
        }
        let shell_of = |f: FaceRef| self.faces.get(f.0).map(|face| face.shell);
        // Every edge in the shell of its two faces, and every vertex in the
        // shell of its edges: two shells share nothing.
        let mut edge_shell: BTreeMap<EdgeRef, usize> = BTreeMap::new();
        for (&edge, list) in &uses {
            let shells: Vec<usize> = list.iter().filter_map(|&(_, f)| shell_of(f)).collect();
            if let [a, b] = shells[..] {
                if a != b {
                    return Err(BuildError::SharedEdge {
                        edge,
                        shells: [a.min(b), a.max(b)],
                    });
                }
            }
            if let Some(&s) = shells.first() {
                edge_shell.insert(edge, s);
            }
        }
        let mut vertex_shell: BTreeMap<VertexRef, usize> = BTreeMap::new();
        for (ei, e) in self.edges.iter() {
            let Some(&s) = edge_shell.get(&EdgeRef(ei)) else {
                continue;
            };
            for v in [e.start, e.end] {
                match vertex_shell.get(&v) {
                    Some(&t) if t != s => {
                        return Err(BuildError::SharedVertex {
                            vertex: v,
                            shells: [t.min(s), t.max(s)],
                        });
                    }
                    _ => {
                        vertex_shell.insert(v, s);
                    }
                }
            }
        }
        let shells: BTreeSet<usize> = self.faces.iter().map(|(_, f)| f.shell).collect();
        let mut genus = 0;
        for shell in shells {
            let faces: Vec<FaceRef> = self
                .faces
                .iter()
                .filter(|(_, f)| f.shell == shell)
                .map(|(fi, _)| FaceRef(fi))
                .collect();
            let Some(&from) = faces.first() else {
                continue;
            };
            let mut reached: BTreeSet<FaceRef> = BTreeSet::new();
            reached.insert(from);
            let mut front = vec![from];
            while let Some(f) = front.pop() {
                let Some(face) = self.faces.get(f.0) else {
                    continue;
                };
                for u in face.loops.iter().flat_map(|lp| lp.uses.iter()) {
                    for &(_, other) in uses.get(&u.edge).map_or(&[][..], Vec::as_slice) {
                        if reached.insert(other) {
                            front.push(other);
                        }
                    }
                }
            }
            if let Some(&face) = faces.iter().find(|f| !reached.contains(f)) {
                return Err(BuildError::Disconnected { face, from });
            }
            let counts = Counts {
                vertices: vertex_shell.values().filter(|&&s| s == shell).count(),
                edges: edge_shell
                    .iter()
                    .filter(|&(&e, &s)| {
                        s == shell && !self.edges.get(e.0).is_some_and(is_degenerate)
                    })
                    .count(),
                faces: faces.len(),
                loops: faces
                    .iter()
                    .filter_map(|f| self.faces.get(f.0))
                    .map(|f| f.loops.len())
                    .sum(),
                shells: 1,
                genus: 0,
            };
            // The shell's line at S = 1 closes at a whole genus, or the
            // faces are no closed surface.
            let line = counts.line();
            if !line.closes() || line.genus < 0 {
                return Err(BuildError::NotClosed { counts });
            }
            genus += line.genus as usize;
        }
        // A vertex no edge ends at is on no shell, and the body's counts
        // are then not the sum of its closed shells'.
        if vertex_shell.len() != self.vertices.len() {
            return Err(BuildError::NotClosed {
                counts: self.counts(),
            });
        }
        Ok(genus)
    }
}

/// Whether a staged edge is degenerate: the singular point of the one face
/// that closes on it — a cone's apex, a sphere's pole — which is not a
/// boundary between two faces (`docs/DATA-MODEL.md` §Invariants, S2) and
/// which the Euler–Poincaré line does not count.
fn is_degenerate(edge: &StagedEdge) -> bool {
    matches!(edge.geometry, EdgeGeometry::Degenerate { .. })
}

/// The coedges a finished solid's edge is used by: two, and one for a
/// degenerate edge, since no surface closes on one singular point twice.
fn uses_wanted(edge: &StagedEdge) -> usize {
    if is_degenerate(edge) { 1 } else { 2 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slots_reuse_the_most_recently_freed_first() {
        let mut s = Slots::default();
        assert_eq!((s.insert('a'), s.insert('b'), s.insert('c')), (0, 1, 2));
        assert_eq!(s.remove(0), Some('a'));
        assert_eq!(s.remove(2), Some('c'));
        assert_eq!(s.remove(2), None, "already free");
        assert_eq!(s.len(), 1);
        assert_eq!(s.insert('d'), 2, "the last freed slot comes back first");
        assert_eq!(s.insert('e'), 0);
        assert_eq!(s.insert('f'), 3);
        let live: Vec<_> = s.iter().collect();
        assert_eq!(live, [(0, &'e'), (1, &'b'), (2, &'d'), (3, &'f')]);
    }

    #[test]
    fn around_is_the_cyclic_half_open_range() {
        let u = |i: u32| Use {
            edge: EdgeRef(i),
            orientation: Orientation::Forward,
            pcurve: None,
        };
        let uses = [u(0), u(1), u(2), u(3)];
        let edges = |v: Vec<Use>| v.iter().map(|u| u.edge.0).collect::<Vec<_>>();
        assert_eq!(edges(around(&uses, 1, 3)), [1, 2]);
        assert_eq!(edges(around(&uses, 3, 1)), [3, 0]);
        assert_eq!(edges(around(&uses, 2, 2)), Vec::<u32>::new());
        assert_eq!(edges(around(&uses, 0, 4)), [0, 1, 2, 3]);
        assert_eq!(edges(around(&uses, 4, 0)), Vec::<u32>::new());
        assert_eq!(edges(around(&[], 0, 0)), Vec::<u32>::new());
    }
}
