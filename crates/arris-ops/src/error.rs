//! The typed errors of the operations (`docs/ARCHITECTURE.md` §Errors):
//! every variant names the entities involved.

use arris_check::{ClassifyError, LumpError, Report};
use arris_geom::{GeomError, GeomKind, ProfileError};
use arris_math::{FrameError, Interrupted};
use arris_topo::builder::BuildError;
use arris_topo::{AnyId, Body, EdgeId, FaceId, NotFound, Shape};

use crate::build::{BuildSlot, Rejection};

/// Why a requested result has no valid representation, grouped by the
/// operation that raises it. [`Reason::name`] is the stable name of the
/// leaf reason — the one a histogram or a census counts a refusal under —
/// whatever the grouping; `Display` is each leaf's own message.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Reason {
    /// An operand or a parameter no operation can use: the reasons more than one operation raises.
    Input(InputReason),
    /// A revolve or an extrude refused its profile, axis, angle or direction.
    Sweep(SweepReason),
    /// A fuse, cut or common whose result a manifold solid cannot hold.
    Boolean(BooleanReason),
    /// A fillet or a chamfer refused an edge, a chain or a corner.
    Blend(BlendReason),
    /// An offset of faces refused its faces, or a move that would change
    /// the body's topology.
    Offset(OffsetReason),
    /// A shell refused its openings; what the offset of its walls refuses
    /// is the offset's reason.
    Shell(ShellReason),
    /// A query refused the shape or the point it was handed.
    Query(QueryReason),
}

impl Reason {
    /// The leaf reason's name — `"BlendTooLarge"`, `"NonFinite"` — without
    /// its group, its entities or its numbers: stable across regroupings,
    /// so a refusal histogram keys on it.
    ///
    /// ```
    /// use arris_ops::{BlendReason, Reason};
    ///
    /// assert_eq!(Reason::Blend(BlendReason::TooLarge).name(), "BlendTooLarge");
    /// ```
    pub fn name(&self) -> &'static str {
        match self {
            Reason::Input(reason) => reason.name(),
            Reason::Sweep(reason) => reason.name(),
            Reason::Boolean(reason) => reason.name(),
            Reason::Blend(reason) => reason.name(),
            Reason::Offset(reason) => reason.name(),
            Reason::Shell(reason) => reason.name(),
            Reason::Query(reason) => reason.name(),
        }
    }
}

impl core::fmt::Display for Reason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Reason::Input(reason) => reason.fmt(f),
            Reason::Sweep(reason) => reason.fmt(f),
            Reason::Boolean(reason) => reason.fmt(f),
            Reason::Blend(reason) => reason.fmt(f),
            Reason::Offset(reason) => reason.fmt(f),
            Reason::Shell(reason) => reason.fmt(f),
            Reason::Query(reason) => reason.fmt(f),
        }
    }
}

/// An operand or a parameter no operation can use: the reasons more than one operation raises.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InputReason {
    /// A parameter is NaN or infinite.
    NonFinite {
        /// Which parameter.
        what: &'static str,
    },
    /// A length or a measure that must be positive is not: a radius, a
    /// height, an extent of a box whose `min` is not below its `max`, the
    /// volume a body's faces enclose.
    NotPositive {
        /// Which length.
        what: &'static str,
        /// What was given.
        value: f64,
    },
    /// The result has no thickness: a common of flush bodies, a sweep of
    /// zero length.
    ZeroThickness,
    /// The result's shells would touch along an edge or at a vertex — an
    /// edge used by four faces, a vertex two lumps share, a shell touching
    /// itself at a vertex where its faces close into more than one fan, a
    /// full revolve's profile touching its axis at a vertex with no
    /// segment along it — which a manifold `Solid`'s shells never do
    /// (ADR-0006); the error's entities are the shared edges or vertices —
    /// a section vertex named by the edges and faces whose hits and
    /// crossings made it — none for a sweep. A body
    /// that touches itself so is a `General` one, which no operation builds
    /// yet.
    NonManifold,
    /// The query needs an enclosed volume and the body is not a solid:
    /// a sheet, a wire, a general body.
    NotSolid,
}

impl InputReason {
    /// The leaf reason's name: the name it had before the reasons were
    /// grouped, without its fields.
    pub fn name(&self) -> &'static str {
        match self {
            InputReason::NonFinite { .. } => "NonFinite",
            InputReason::NotPositive { .. } => "NotPositive",
            InputReason::ZeroThickness => "ZeroThickness",
            InputReason::NonManifold => "NonManifold",
            InputReason::NotSolid => "NotSolid",
        }
    }
}

impl core::fmt::Display for InputReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            InputReason::NonFinite { what } => write!(f, "{what} is not finite"),
            InputReason::NotPositive { what, value } => {
                write!(f, "{what} must be positive, not {value}")
            }
            InputReason::ZeroThickness => f.write_str("the result has no thickness"),
            InputReason::NonManifold => f.write_str(
                "the result's shells would touch along an edge or at a vertex, which a solid does not hold",
            ),
            InputReason::NotSolid => f.write_str("the body is not a solid"),
        }
    }
}

/// A revolve or an extrude refused its profile, axis, angle or direction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SweepReason {
    /// A revolve profile crosses its axis: it has points on both sides of
    /// the axis line in its plane, beyond the tolerance.
    ProfileCrossesAxis,
    /// A revolve axis does not lie in the profile's plane within the
    /// tolerances: its direction is off the plane by more than the angular
    /// tolerance, or its origin is off it by more than the linear one.
    AxisNotInProfilePlane,
    /// A revolve angle is above a full turn, by more than the angular
    /// tolerance.
    AngleAboveTurn,
    /// A revolve profile has an arc whose centre is nearer the axis than
    /// its radius, so its circle crosses the axis and the face it sweeps
    /// would be a self-intersecting torus, which the data model does not
    /// hold (`docs/DATA-MODEL.md` §Surfaces: `R > r`).
    SpindleTorus,
    /// A revolve profile has an elliptic segment or a full-ellipse loop
    /// (ADR-0014): the surface it would sweep — a spheroid, an elliptic
    /// torus — has no variant in the data model, and the meridian
    /// intersections it would need have no arm. The reason names the
    /// first such segment in the consumer's own order, `0` the outer
    /// loop and `0` an ellipse loop's one segment. Radii that agree
    /// within the linear tolerance are a circle edge and revolve.
    EllipticRevolve {
        /// The loop: `0` the outer, the holes from `1`.
        loop_index: usize,
        /// The segment of that loop, as the consumer wrote it.
        segment: usize,
    },
    /// An extrude direction is off the profile plane's normal by more than
    /// the angular tolerance; an oblique extrusion is not built yet
    /// (ADR-0047).
    DirectionNotNormal,
}

impl SweepReason {
    /// The leaf reason's name: the name it had before the reasons were
    /// grouped, without its fields.
    pub fn name(&self) -> &'static str {
        match self {
            SweepReason::ProfileCrossesAxis => "ProfileCrossesAxis",
            SweepReason::AxisNotInProfilePlane => "AxisNotInProfilePlane",
            SweepReason::AngleAboveTurn => "AngleAboveTurn",
            SweepReason::SpindleTorus => "SpindleTorus",
            SweepReason::EllipticRevolve { .. } => "EllipticRevolve",
            SweepReason::DirectionNotNormal => "DirectionNotNormal",
        }
    }
}

impl core::fmt::Display for SweepReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SweepReason::ProfileCrossesAxis => f.write_str("the profile crosses the revolve axis"),
            SweepReason::AxisNotInProfilePlane => {
                f.write_str("the revolve axis does not lie in the profile's plane")
            }
            SweepReason::AngleAboveTurn => f.write_str("the revolve angle is above a full turn"),
            SweepReason::SpindleTorus => {
                f.write_str("an arc's circle crosses the revolve axis: a spindle torus")
            }
            SweepReason::EllipticRevolve {
                loop_index,
                segment,
            } => write!(
                f,
                "loop {loop_index}, segment {segment} is elliptic, which a revolve does not sweep"
            ),
            SweepReason::DirectionNotNormal => {
                f.write_str("the extrude direction is not the profile plane's normal")
            }
        }
    }
}

/// A fuse, cut or common whose result a manifold solid cannot hold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BooleanReason {
    /// A boolean selected no material: a `common` of disjoint operands,
    /// a target swallowed by its tool.
    Empty,
    /// Two faces touch along a curve that would be interior to both
    /// result faces — a hole wall tangent to a side face along a ruling,
    /// a section curve tangent to a loop edge at a vertex — which a
    /// manifold `Solid` cannot represent (ADR-0004, plan `⚠ OPEN` 1).
    TangentContact,
    /// A section curve passes a face's singular point — a cone's apex, a
    /// sphere's pole — without running through it, nearer than the face's
    /// (u, v) can carry: its pcurve turns through up to half a turn of
    /// `u` over a stretch as long as the miss, which the face's polygons
    /// do not resolve at their finest, and by an apex the curve itself
    /// doubles back within a tolerance (ADR-0021). Through the point is
    /// built, and so is clear of it; the error's entities are the face,
    /// the other face of the pair and the singular vertex.
    BesideSingularity,
    /// A boolean over several tools was given none, or a fuse of several
    /// bodies fewer than two: there is nothing to combine. The error's
    /// entities are the target, or the one body.
    NoTools,
    /// One body is an operand of a boolean twice — a tool listed twice, or
    /// the target among its tools — so its faces would be cut by
    /// themselves. The error's entity is the body.
    RepeatedOperand,
}

impl BooleanReason {
    /// The leaf reason's name: the name it had before the reasons were
    /// grouped, without its fields.
    pub fn name(&self) -> &'static str {
        match self {
            BooleanReason::Empty => "Empty",
            BooleanReason::TangentContact => "TangentContact",
            BooleanReason::BesideSingularity => "BesideSingularity",
            BooleanReason::NoTools => "NoTools",
            BooleanReason::RepeatedOperand => "RepeatedOperand",
        }
    }
}

impl core::fmt::Display for BooleanReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BooleanReason::Empty => f.write_str("the result has no material"),
            BooleanReason::TangentContact => {
                f.write_str("the faces touch along a curve interior to both result faces")
            }
            BooleanReason::BesideSingularity => f.write_str(
                "a section passes a face's apex or pole without running through it, nearer than the face's (u, v) resolves",
            ),
            BooleanReason::NoTools => {
                f.write_str("the boolean has no tool, or fewer than two bodies, to combine")
            }
            BooleanReason::RepeatedOperand => {
                f.write_str("a body is an operand of the boolean twice")
            }
        }
    }
}

/// A fillet or a chamfer refused an edge, a chain or a corner.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BlendReason {
    /// A blend was asked for no edges at all; the error's entity is the
    /// body.
    NoEdges,
    /// An edge is listed twice in one blend call; the error's entity is
    /// the edge.
    RepeatedEdge,
    /// An edge id resolves in the model but is not an edge of the body a
    /// blend was asked to blend; the error's entities are the edge and
    /// the body.
    EdgeNotInBody,
    /// A blend does not fit its faces: a contact curve or an end arc
    /// leaves the face it lies on through an edge that is not one of the
    /// corner's own, or a corner edge is shorter than the trim would cut
    /// from it (ADR-0007). The error's entities are the blended edge and
    /// the face or edge the blend runs out of.
    TooLarge,
    /// The blended edge's two faces meet at a tangent dihedral — the arc
    /// and the line of a slot's wall, a blend face and its neighbour — so
    /// there is no corner to roll a ball into; or the edge ends at a
    /// vertex where a corner edge's faces are tangent — a blend's contact
    /// line, where a second blend reaches a first one's end — so the blend
    /// would run on along a chain, at a vertex the chain walk does not take. The
    /// error's entities are the edge and its two faces, or at an end the
    /// edge, the tangent corner edge and the vertex.
    TangentChain,
    /// A corner the closed forms do not cover (ADR-0007): a vertex of
    /// other than three edges; a miter whose two blends' far contacts miss
    /// each other on its third edge; or three blended edges at a vertex
    /// whose faces are not all planes, whose blends are not all convex or
    /// all concave, or — three fillets — none of whose faces is square to
    /// the other two. The error's entities are the blended edges and the
    /// vertex.
    VertexBlend,
}

impl BlendReason {
    /// The leaf reason's name: the name it had before the reasons were
    /// grouped, without its fields.
    pub fn name(&self) -> &'static str {
        match self {
            BlendReason::NoEdges => "NoEdges",
            BlendReason::RepeatedEdge => "RepeatedEdge",
            BlendReason::EdgeNotInBody => "EdgeNotInBody",
            BlendReason::TooLarge => "BlendTooLarge",
            BlendReason::TangentChain => "TangentChain",
            BlendReason::VertexBlend => "VertexBlend",
        }
    }
}

impl core::fmt::Display for BlendReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BlendReason::NoEdges => f.write_str("no edges were given to blend"),
            BlendReason::RepeatedEdge => f.write_str("an edge is listed twice"),
            BlendReason::EdgeNotInBody => f.write_str("the edge is not an edge of the body"),
            BlendReason::TooLarge => f.write_str(
                "the blend leaves its face through an edge that is not the corner's own",
            ),
            BlendReason::TangentChain => f.write_str(
                "the edge's faces meet at a tangent dihedral, which has no corner to blend",
            ),
            BlendReason::VertexBlend => f.write_str(
                "the corner at the edge's end is one the blend's closed forms do not cover",
            ),
        }
    }
}

/// An offset of faces refused its faces, or a move that would change the
/// body's topology (ADR-0048 §3): the result keeps every vertex, edge and
/// face of the body, or the offset is not made.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OffsetReason {
    /// An offset was asked for no faces at all; the error's entity is the
    /// body.
    NoFaces,
    /// A face is listed twice in one offset call; the error's entity is
    /// the face.
    RepeatedFace,
    /// A face id resolves in the model but is not a face of the body the
    /// offset was asked to move; the error's entities are the face and
    /// the body.
    FaceNotInBody,
    /// The move would make an edge or a face vanish or reverse: an edge
    /// whose ends meet or cross, a face whose loop turns inside out. The
    /// error's entity is that edge or face.
    Vanishes,
    /// The faces around a vertex no longer meet in one point once moved —
    /// a vertex of four or more faces, a pyramid's apex with one side
    /// pushed — so the vertex would split into an edge. The error's
    /// entity is the vertex.
    VertexSplits,
    /// A face the offset moves, or must move with it, lies on a surface
    /// with no exact offset of its own kind: an elliptic cylinder (the
    /// parallel curve of an ellipse is no ellipse) or a free-form surface.
    /// The error's entity is the face.
    NoExactOffset,
    /// The offset drives a moved face's surface through zero — a
    /// cylinder, a sphere or a torus tube whose radius is gone, a cone
    /// whose move reaches its axis. The error's entity is the face.
    SurfaceCollapses,
    /// A face dragged along by a tangent neighbour no longer meets a face
    /// beside it that stays: its offset has pulled clear of the other's
    /// surface where their edge was. The error's entity is that edge.
    Gap,
    /// The offset is built and its faces run into each other away from
    /// their shared edges — a boss pushed through the body's side, a
    /// pocket floor pulled through the bottom — which the checker's global
    /// level reports and the local construction cannot see. The error's
    /// entities are the faces the report names (the edge, for one that
    /// crosses itself).
    SelfIntersects,
}

impl OffsetReason {
    /// The variant's name, as [`Reason::name`] gives it.
    pub fn name(&self) -> &'static str {
        match self {
            OffsetReason::NoFaces => "NoFaces",
            OffsetReason::RepeatedFace => "RepeatedFace",
            OffsetReason::FaceNotInBody => "FaceNotInBody",
            OffsetReason::Vanishes => "Vanishes",
            OffsetReason::VertexSplits => "VertexSplits",
            OffsetReason::NoExactOffset => "NoExactOffset",
            OffsetReason::SurfaceCollapses => "SurfaceCollapses",
            OffsetReason::Gap => "Gap",
            OffsetReason::SelfIntersects => "SelfIntersects",
        }
    }
}

impl core::fmt::Display for OffsetReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            OffsetReason::NoFaces => f.write_str("no faces were given to offset"),
            OffsetReason::RepeatedFace => f.write_str("a face is listed twice"),
            OffsetReason::FaceNotInBody => f.write_str("the face is not a face of the body"),
            OffsetReason::Vanishes => {
                f.write_str("the offset would make the edge or face vanish or turn inside out")
            }
            OffsetReason::VertexSplits => {
                f.write_str("the faces around the vertex no longer meet in one point once moved")
            }
            OffsetReason::NoExactOffset => {
                f.write_str("the face's surface has no exact offset of its own kind")
            }
            OffsetReason::SurfaceCollapses => {
                f.write_str("the offset drives the face's surface through zero")
            }
            OffsetReason::Gap => f.write_str(
                "a face dragged by a tangent neighbour no longer meets the face beside it that stays",
            ),
            OffsetReason::SelfIntersects => {
                f.write_str("the offset body's faces run into each other")
            }
        }
    }
}

/// A shell refused its openings (ADR-0049 §5). Everything the offset of
/// its walls refuses — a vanishing edge, a split vertex, a collapsing
/// surface, a skin running into the outer faces — is [`Reason::Offset`],
/// naming the entity, since it is the offset's.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ShellReason {
    /// Every face of the body is an opening, so there is no wall to
    /// thicken; the error's entity is the body.
    NoWalls,
    /// An opening is listed twice; the error's entity is the face.
    RepeatedOpening,
    /// An opening resolves in the model but is not a face of the body the
    /// shell was asked to hollow; the error's entities are the face and
    /// the body.
    OpeningNotInBody,
    /// An opening is tangent to a wall, which the wall's offset would
    /// drag along with it (ADR-0048 §6): an opening stays where it is, so
    /// the two cannot both hold. The error's entity is the opening.
    OpeningDragged,
}

impl ShellReason {
    /// The variant's name, as [`Reason::name`] gives it.
    pub fn name(&self) -> &'static str {
        match self {
            ShellReason::NoWalls => "NoWalls",
            ShellReason::RepeatedOpening => "RepeatedOpening",
            ShellReason::OpeningNotInBody => "OpeningNotInBody",
            ShellReason::OpeningDragged => "OpeningDragged",
        }
    }
}

impl core::fmt::Display for ShellReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ShellReason::NoWalls => f.write_str("every face is an opening, so no wall is left"),
            ShellReason::RepeatedOpening => f.write_str("an opening is listed twice"),
            ShellReason::OpeningNotInBody => f.write_str("the opening is not a face of the body"),
            ShellReason::OpeningDragged => {
                f.write_str("the opening is tangent to a wall whose offset would drag it")
            }
        }
    }
}

/// A query refused the shape or the point it was handed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QueryReason {
    /// A query that projects edges and vertices was handed a face, a
    /// shell or a body; the error's entity is that shape.
    NotProjectable,
    /// A query that needs an edge's 3D curve was handed a degenerate edge,
    /// which has none; the error's entity is the edge.
    DegenerateEdge,
    /// An edge's curve projects onto the plane as a point or a segment —
    /// a line perpendicular to it, a conic whose plane is — which no
    /// `Curve2` over a range represents; the error's entity is the edge.
    ProjectionCollapses,
    /// `face_frame` was asked for a face whose surface is not a plane: a
    /// plane's own frame stands for the whole face, a curved surface's
    /// varies with `(u, v)` and is `frame_at`'s to answer. The error's
    /// entity is the face.
    NotPlanar,
    /// `frame_at` was asked for a `(u, v)` outside the face's own domain
    /// (`arris_check::domain::FaceDomain`); the error's entity is the
    /// face.
    OutOfDomain,
    /// `frame_at` was asked for a `(u, v)` where the surface's
    /// parametrisation is singular — a sphere's pole, a cone's apex —
    /// so `Surface::normal` has none to give; the error's entity is the
    /// face.
    Singular,
}

impl QueryReason {
    /// The leaf reason's name: the name it had before the reasons were
    /// grouped, without its fields.
    pub fn name(&self) -> &'static str {
        match self {
            QueryReason::NotProjectable => "NotProjectable",
            QueryReason::DegenerateEdge => "DegenerateEdge",
            QueryReason::ProjectionCollapses => "ProjectionCollapses",
            QueryReason::NotPlanar => "NotPlanar",
            QueryReason::OutOfDomain => "OutOfDomain",
            QueryReason::Singular => "Singular",
        }
    }
}

impl core::fmt::Display for QueryReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            QueryReason::NotProjectable => {
                f.write_str("only an edge or a vertex projects onto a plane")
            }
            QueryReason::DegenerateEdge => {
                f.write_str("the edge is degenerate: it has no 3D curve")
            }
            QueryReason::ProjectionCollapses => {
                f.write_str("the curve projects onto the plane as a point or a segment")
            }
            QueryReason::NotPlanar => f.write_str("the face's surface is not a plane"),
            QueryReason::OutOfDomain => f.write_str("the point is outside the face's own domain"),
            QueryReason::Singular => {
                f.write_str("the surface's parametrisation is singular there: it has no normal")
            }
        }
    }
}

/// A kernel bug an operation caught in its own output or its own
/// sequence, rather than a fault of the input. `Checker` is returned only
/// in release builds with the `paranoid` feature on, a debug build
/// panicking with the same report; every other fault is returned in any
/// build.
#[derive(Debug, Clone, PartialEq)]
pub enum Fault {
    /// The checker rejected the operation's output.
    Checker(Box<Report>),
    /// The builder refused a step of the operation's fixed sequence.
    Builder(BuildError),
    /// A frame the operation placed could not be built from inputs it had
    /// already validated.
    Frame(FrameError),
    /// A point the operation had to classify against a body could not be:
    /// every ray direction grazed it, a surface has no closed form
    /// against a ray, or an id did not resolve (`arris_check::classify`).
    Classify(ClassifyError),
    /// A geometry query on inputs the operation had already validated
    /// failed for a reason other than a missing closed form: a
    /// projection with no unique answer, a degenerate operand, a pcurve
    /// fit that would not converge.
    Geometry(GeomError),
    /// A section edge of a face pair crosses a seam of `face` inside
    /// itself. The seam's own hit on `other` is the pave that should
    /// have split it there (ADR-0004), so the two decisions disagreed.
    Seam {
        /// The face whose seam is crossed.
        face: FaceId,
        /// The other face of the pair.
        other: FaceId,
    },
    /// The (u, v) arrangement of a face's loops and section edges is not
    /// the planar subdivision the pave model promised (ADR-0004): see
    /// [`SplitFault`].
    Split(SplitFault),
    /// A piece of `edge`, of one face of a coincident pair, lies along
    /// the boundary of `face`, the other, on the same curve as one of
    /// its edges, but matches no piece of that edge between its paves:
    /// the two edges were paved differently by the vertices they share
    /// (ADR-0004).
    CommonBlock {
        /// The edge whose piece has no match.
        edge: EdgeId,
        /// The face whose boundary it lies along.
        face: FaceId,
    },
    /// The shells of a result could not be ordered into lumps
    /// (`arris_check::lumps`, ADR-0006): the pieces an operation kept,
    /// which share no edge and no vertex between shells, did not nest, or
    /// their nesting could not be decided.
    Lumps(LumpError),
    /// An invariant the operation's own fixed sequence should have kept
    /// broke: an internal lookup by index or key, never a model id, found
    /// nothing. Never a property of the input.
    Invariant {
        /// What was missing.
        what: &'static str,
    },
    /// A sweep's fixed sequence did not make an entity for `segment` that
    /// its own later step needed: a kernel bug, never a property of the
    /// validated sketch.
    Unmade {
        /// The profile segment.
        segment: usize,
    },
    /// A surface has no normal at the point on `face` an operation needed
    /// one at: every partial derivative it tried was degenerate there,
    /// which the checker's own tolerances should have ruled out.
    NoNormal {
        /// The face.
        face: FaceId,
    },
    /// A profile edge's curve is not one of the kinds `Profile::edges`
    /// makes: a kernel bug in the fixed sequence, never a property of the
    /// validated sketch.
    ProfileCurve(GeomError),
}

/// How the arrangement a boolean splits a face by failed to be a planar
/// subdivision. Each is a disagreement between the pave model and the
/// face's own (u, v): a kernel bug, never a property of the input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SplitFault {
    /// A pave lies within the tolerance of an end of `edge` without
    /// being at its vertex, so the piece between them has no length.
    EmptySubEdge {
        /// The edge.
        edge: EdgeId,
    },
    /// A node of `face`'s arrangement has a single half-edge: a section
    /// edge ends inside the face without meeting anything.
    Dangling {
        /// The face.
        face: FaceId,
    },
    /// The walk from a half-edge of `face` re-entered a half-edge
    /// already walked before closing its cycle, or a cycle turned by
    /// something other than one full turn: two edges cross without a
    /// vertex there.
    Turn {
        /// The face.
        face: FaceId,
    },
    /// A clockwise cycle of `face` — a hole — lies inside no region.
    Hole {
        /// The face.
        face: FaceId,
    },
    /// A region of `face` holds no point at the clearance its polygons
    /// need: a sliver thinner than its own chord deviation.
    NoInterior {
        /// The face.
        face: FaceId,
    },
}

impl core::fmt::Display for SplitFault {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SplitFault::EmptySubEdge { edge } => {
                write!(f, "a pave on {edge} is at its end without being its vertex")
            }
            SplitFault::Dangling { face } => {
                write!(
                    f,
                    "a section edge of {face} ends at a node nothing else reaches"
                )
            }
            SplitFault::Turn { face } => {
                write!(f, "a cycle of {face}'s arrangement does not turn once")
            }
            SplitFault::Hole { face } => write!(f, "a hole of {face} lies inside no region"),
            SplitFault::NoInterior { face } => {
                write!(
                    f,
                    "a piece of {face} holds no interior point at its clearance"
                )
            }
        }
    }
}

impl core::fmt::Display for Fault {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Fault::Checker(report) => write!(f, "the output fails the checker:\n{report}"),
            Fault::Builder(e) => write!(f, "the builder refused: {e}"),
            Fault::Frame(e) => write!(f, "a frame could not be placed: {e}"),
            Fault::Classify(e) => write!(f, "a point could not be classified: {e}"),
            Fault::Geometry(e) => write!(f, "a geometry query failed: {e}"),
            Fault::Seam { face, other } => write!(
                f,
                "a section edge of {face} and {other} crosses a seam of {face} without a pave there"
            ),
            Fault::Split(e) => write!(f, "the face arrangement is not a subdivision: {e}"),
            Fault::CommonBlock { edge, face } => write!(
                f,
                "a piece of {edge} lies along the boundary of {face} but matches no piece of it"
            ),
            Fault::Lumps(e) => write!(f, "the result's shells are not lumps: {e}"),
            Fault::Invariant { what } => write!(f, "{what} was not found"),
            Fault::Unmade { segment } => {
                write!(f, "segment {segment} was not made")
            }
            Fault::NoNormal { face } => write!(f, "{face} has no normal there"),
            Fault::ProfileCurve(e) => write!(f, "a profile edge's curve is invalid: {e}"),
        }
    }
}

/// Why an operation failed. Every variant names what it is about, so the
/// message a consumer shows says *which* face pair, *which* parameter,
/// not "boolean failed"; the model is as it was before the call.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum OpError {
    /// An input body fails the checker (checked in debug builds before the
    /// operation starts, and in release with the `paranoid` feature).
    #[error("{body} fails the checker:\n{report}")]
    InvalidInput {
        /// The body.
        body: Body,
        /// What it fails.
        report: Box<Report>,
    },
    /// The exhaustive dispatch reached a surface or curve pair the kernel
    /// has no formula for yet.
    #[error("no closed form for {} ({}) against {} ({})", .a.1, .a.0, .b.1, .b.0)]
    Unsupported {
        /// The first kind and entity.
        a: (GeomKind, Shape),
        /// The second.
        b: (GeomKind, Shape),
    },
    /// The requested result has no valid representation: a parameter that
    /// makes no geometry, a zero-thickness intersection, a profile
    /// crossing its revolve axis. Never a silently empty body.
    #[error("degenerate result: {reason}{}", entities_suffix(.entities))]
    Degenerate {
        /// The entities involved; none for a primitive, which has no
        /// input.
        entities: Vec<Shape>,
        /// Why.
        reason: Reason,
    },
    /// The profile of a sweep is not a valid sketch: `Profile::edges`
    /// refused it, naming the loop and segment. An invalid profile has no
    /// entities to name, so it is neither `InvalidInput` nor `Degenerate`.
    #[error("the profile is not valid: {0}")]
    Profile(ProfileError),
    /// The result would need an entity tolerance above
    /// `Precision::max_tolerance`.
    #[error("{entity} would need tolerance {wanted}, above the model's maximum")]
    Tolerance {
        /// The entity.
        entity: Shape,
        /// The tolerance it wanted.
        wanted: f64,
    },
    /// An id does not resolve in this model: the wrong model, or
    /// compacted away. Names the id that failed to resolve itself, not an
    /// entity that holds it.
    #[error("{0} does not resolve in this model")]
    NotFound(AnyId),
    /// A kernel bug, caught: see [`Fault`].
    #[error("kernel bug: {0}")]
    Internal(Fault),
    /// [`build`](crate::build) was handed a live slot its keys name no
    /// key for: the record would have an output with no origin.
    #[error("{slot} has no key")]
    Unkeyed {
        /// The slot.
        slot: BuildSlot,
    },
    /// [`build`](crate::build) refused the consumer's topology as a
    /// solid: see [`Rejection`].
    #[error("the consumer's body is refused: {0}")]
    Rejected(Rejection),
    /// The caller stopped the operation, by its poll or its budget of
    /// steps (ADR-0030). Not a refusal of the input: the model is as it
    /// was before the call, ids included, and the same call with a
    /// larger budget or a poll that stays `false` can succeed.
    #[error("{0}")]
    Interrupted(Interrupted),
}

impl From<Interrupted> for OpError {
    fn from(stop: Interrupted) -> Self {
        OpError::Interrupted(stop)
    }
}

/// A geometry error the operation has no name for as its own: a stop is
/// the caller's [`OpError::Interrupted`], anything else a kernel fault.
pub(crate) fn fault_of(e: GeomError) -> OpError {
    match e {
        GeomError::Interrupted(stop) => OpError::Interrupted(stop),
        other => OpError::Internal(Fault::Geometry(other)),
    }
}

fn entities_suffix(entities: &[Shape]) -> String {
    if entities.is_empty() {
        String::new()
    } else {
        let names: Vec<String> = entities.iter().map(|s| s.to_string()).collect();
        format!(" ({})", names.join(", "))
    }
}

impl From<ProfileError> for OpError {
    fn from(e: ProfileError) -> Self {
        OpError::Profile(e)
    }
}

impl From<BuildError> for OpError {
    fn from(e: BuildError) -> Self {
        OpError::Internal(Fault::Builder(e))
    }
}

impl From<FrameError> for OpError {
    fn from(e: FrameError) -> Self {
        OpError::Internal(Fault::Frame(e))
    }
}

impl From<NotFound> for OpError {
    fn from(e: NotFound) -> Self {
        OpError::NotFound(e.id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every leaf reason prints the message it printed before the reasons
    /// were grouped, and names itself as it was named: the Python binding's
    /// `reason` text and every refusal histogram depend on both.
    #[test]
    fn every_leaf_reason_keeps_its_message_and_its_name() {
        let leaves: Vec<(Reason, &str, &str)> = vec![
            (
                Reason::Input(InputReason::NonFinite { what: "radius" }),
                "NonFinite",
                "radius is not finite",
            ),
            (
                Reason::Input(InputReason::NotPositive {
                    what: "radius",
                    value: 0.0,
                }),
                "NotPositive",
                "radius must be positive, not 0",
            ),
            (
                Reason::Sweep(SweepReason::EllipticRevolve {
                    loop_index: 1,
                    segment: 2,
                }),
                "EllipticRevolve",
                "loop 1, segment 2 is elliptic, which a revolve does not sweep",
            ),
            (
                Reason::Input(InputReason::ZeroThickness),
                "ZeroThickness",
                "the result has no thickness",
            ),
            (
                Reason::Input(InputReason::NonManifold),
                "NonManifold",
                "the result's shells would touch along an edge or at a vertex, which a solid does not hold",
            ),
            (
                Reason::Input(InputReason::NotSolid),
                "NotSolid",
                "the body is not a solid",
            ),
            (
                Reason::Sweep(SweepReason::ProfileCrossesAxis),
                "ProfileCrossesAxis",
                "the profile crosses the revolve axis",
            ),
            (
                Reason::Sweep(SweepReason::AxisNotInProfilePlane),
                "AxisNotInProfilePlane",
                "the revolve axis does not lie in the profile's plane",
            ),
            (
                Reason::Sweep(SweepReason::AngleAboveTurn),
                "AngleAboveTurn",
                "the revolve angle is above a full turn",
            ),
            (
                Reason::Sweep(SweepReason::SpindleTorus),
                "SpindleTorus",
                "an arc's circle crosses the revolve axis: a spindle torus",
            ),
            (
                Reason::Sweep(SweepReason::DirectionNotNormal),
                "DirectionNotNormal",
                "the extrude direction is not the profile plane's normal",
            ),
            (
                Reason::Boolean(BooleanReason::Empty),
                "Empty",
                "the result has no material",
            ),
            (
                Reason::Boolean(BooleanReason::TangentContact),
                "TangentContact",
                "the faces touch along a curve interior to both result faces",
            ),
            (
                Reason::Boolean(BooleanReason::BesideSingularity),
                "BesideSingularity",
                "a section passes a face's apex or pole without running through it, nearer than the face's (u, v) resolves",
            ),
            (
                Reason::Boolean(BooleanReason::NoTools),
                "NoTools",
                "the boolean has no tool, or fewer than two bodies, to combine",
            ),
            (
                Reason::Boolean(BooleanReason::RepeatedOperand),
                "RepeatedOperand",
                "a body is an operand of the boolean twice",
            ),
            (
                Reason::Blend(BlendReason::NoEdges),
                "NoEdges",
                "no edges were given to blend",
            ),
            (
                Reason::Blend(BlendReason::RepeatedEdge),
                "RepeatedEdge",
                "an edge is listed twice",
            ),
            (
                Reason::Blend(BlendReason::EdgeNotInBody),
                "EdgeNotInBody",
                "the edge is not an edge of the body",
            ),
            (
                Reason::Blend(BlendReason::TooLarge),
                "BlendTooLarge",
                "the blend leaves its face through an edge that is not the corner's own",
            ),
            (
                Reason::Blend(BlendReason::TangentChain),
                "TangentChain",
                "the edge's faces meet at a tangent dihedral, which has no corner to blend",
            ),
            (
                Reason::Blend(BlendReason::VertexBlend),
                "VertexBlend",
                "the corner at the edge's end is one the blend's closed forms do not cover",
            ),
            (
                Reason::Query(QueryReason::NotProjectable),
                "NotProjectable",
                "only an edge or a vertex projects onto a plane",
            ),
            (
                Reason::Query(QueryReason::DegenerateEdge),
                "DegenerateEdge",
                "the edge is degenerate: it has no 3D curve",
            ),
            (
                Reason::Query(QueryReason::ProjectionCollapses),
                "ProjectionCollapses",
                "the curve projects onto the plane as a point or a segment",
            ),
            (
                Reason::Query(QueryReason::NotPlanar),
                "NotPlanar",
                "the face's surface is not a plane",
            ),
            (
                Reason::Query(QueryReason::OutOfDomain),
                "OutOfDomain",
                "the point is outside the face's own domain",
            ),
            (
                Reason::Query(QueryReason::Singular),
                "Singular",
                "the surface's parametrisation is singular there: it has no normal",
            ),
        ];
        assert_eq!(leaves.len(), 28);
        for (reason, name, message) in leaves {
            assert_eq!(reason.name(), name);
            assert_eq!(reason.to_string(), message, "{name}");
        }
    }
}
