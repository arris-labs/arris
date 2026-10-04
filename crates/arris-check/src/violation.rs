//! One variant per invariant of `docs/DATA-MODEL.md` §Invariants.

use core::fmt;

use arris_topo::{BodyId, EdgeId, EntityId, FaceId, GeometryId, ShellId, VertexId};

/// When an invariant runs (`docs/ARCHITECTURE.md` §The checker).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Level {
    /// Combinatorial and local geometric checks, linear in the body: what
    /// runs after every operation in debug builds.
    Fast,
    /// `Fast` plus the global checks (face–face intersection, shell
    /// nesting, enclosed volume). Not linear; run on demand and in the
    /// fixture corpus.
    Full,
}

/// Something an entity refers to: another entity, or a geometry value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Reference {
    /// A topological entity.
    Entity(EntityId),
    /// A curve, surface or pcurve.
    Geometry(GeometryId),
}

impl fmt::Display for Reference {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Reference::Entity(id) => id.fmt(f),
            Reference::Geometry(id) => id.fmt(f),
        }
    }
}

impl From<EntityId> for Reference {
    fn from(id: EntityId) -> Self {
        Reference::Entity(id)
    }
}

impl From<GeometryId> for Reference {
    fn from(id: GeometryId) -> Self {
        Reference::Geometry(id)
    }
}

/// Which number of an entity was not finite (M3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Quantity {
    /// A point or control-point coordinate.
    Coordinate,
    /// A curve or surface parameter, a range end, a knot or a weight.
    Parameter,
    /// An entity's tolerance.
    Tolerance,
}

/// Why an edge's vertices do not match the shape of its curve over its
/// range (E2). The geometric match of each end is V2's.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum EndMismatch {
    /// The curve does not return to its start over the range but the edge
    /// names one vertex for both ends.
    OpenWithOneVertex {
        /// The vertex.
        vertex: VertexId,
        /// Distance between the curve at the two ends of the range.
        gap: f64,
    },
    /// The curve returns to its start over the range but the edge names two
    /// different vertices.
    ClosedWithTwoVertices {
        /// The start vertex.
        start: VertexId,
        /// The end vertex.
        end: VertexId,
    },
}

/// Why a degenerate edge is not a valid one (E6). A degenerate edge has
/// no curve by construction (`EdgeGeometry::Degenerate`), so that is not a
/// fault it can have.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum DegenerateFault {
    /// `start != end`.
    TwoVertices,
    /// The surface is not singular along its pcurve: the pcurve's image
    /// spans more than the vertex's tolerance.
    NotSingular {
        /// The face whose surface was sampled.
        face: FaceId,
        /// The largest distance between two sampled image points.
        extent: f64,
    },
}

/// Why two coedges of one loop do not form a seam (E7).
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum SeamFault {
    /// Both uses run the same way.
    SameOrientation,
    /// The two pcurves do not differ by the surface's period.
    PeriodMismatch {
        /// The difference found in the periodic parameter.
        offset: f64,
        /// The surface's period in that parameter.
        period: f64,
    },
}

/// Why a loop does not close (L1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum LoopBreak {
    /// The loop has no coedges.
    Empty,
    /// Coedge `coedge`'s effective end vertex is not the next coedge's
    /// effective start vertex.
    Between {
        /// Index of the coedge whose end does not meet its successor.
        coedge: usize,
    },
}

/// Why the loops of a face do not nest (L4).
#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum NestingFault {
    /// A loop's signed area in (u, v) is zero.
    ZeroArea {
        /// Index of the loop in the face.
        loop_index: usize,
    },
    /// A connected component of the domain has no loop of positive winding.
    NoOuter,
    /// A component has several loops of positive winding.
    MultipleOuter {
        /// Their indices in the face.
        loops: Vec<usize>,
    },
    /// A loop of negative winding lies outside every outer loop.
    HoleOutside {
        /// Index of the hole.
        loop_index: usize,
    },
}

/// Why a face is malformed (F1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum FaceFault {
    /// No loops at all.
    NoLoops,
    /// A pcurve leaves the surface's bounded (non-periodic) domain.
    PcurveOutsideDomain {
        /// Index of the loop.
        loop_index: usize,
        /// Index of the coedge in the loop.
        coedge: usize,
    },
}

/// Which bound a tolerance broke.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum ToleranceBound {
    /// Below `Precision::min_tolerance`.
    BelowMinimum,
    /// Above `Precision::max_tolerance`.
    AboveMaximum,
    /// Out of order with a neighbouring entity's tolerance
    /// (`vertex ≥ edge ≥ face`).
    Neighbour {
        /// The neighbour.
        entity: EntityId,
        /// Its tolerance.
        tolerance: f64,
    },
}

/// Why an edge's coedge uses are wrong for the body's kind (S2).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EdgeUseFault {
    /// The number of coedges is wrong for the body kind (two for a solid,
    /// one or two for a sheet).
    Count {
        /// Coedges found.
        coedges: usize,
    },
    /// The uses do not pair up with opposite effective orientation: the
    /// faces disagree about which side the material is on.
    Orientation,
}

/// Why a solid's shells do not nest into lumps (B1, ADR-0006).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ShellNestingFault {
    /// No shells at all.
    NoShells,
    /// No shell encloses positive volume: every one is a void of nothing.
    NoOuter,
    /// A void shell — one enclosing negative volume — is inside no shell.
    VoidOutside {
        /// The shell.
        shell: ShellId,
    },
    /// A void shell's innermost container is another void, not the outer
    /// shell of a lump.
    VoidInVoid {
        /// The void.
        shell: ShellId,
        /// Its innermost container, a void too.
        container: ShellId,
    },
    /// An outer shell's innermost container is another outer shell: one
    /// region of material inside another with no cavity between them.
    OuterInOuter {
        /// The inner shell.
        shell: ShellId,
        /// Its innermost container, an outer shell too.
        container: ShellId,
    },
    /// Two shells of the body meet: a face of one intersects a face of the
    /// other, which shells that share no edge never may.
    Overlap {
        /// The two shells, ascending.
        shells: [ShellId; 2],
    },
    /// A shell encloses no volume, or one that is not finite, so it has
    /// no role: neither outer nor void.
    InsideOut {
        /// The shell.
        shell: ShellId,
    },
}

/// Why a wire body is malformed (B3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum WireFault {
    /// A wire body lists a shell.
    HasShell {
        /// The shell.
        shell: ShellId,
    },
    /// A vertex is used by more than two free edges.
    VertexOverused {
        /// The vertex.
        vertex: VertexId,
        /// Free edges at it.
        edges: usize,
    },
}

/// One broken invariant on one entity. The variants are the rows of
/// `docs/DATA-MODEL.md` §Invariants, in the same order and with the row's
/// number at the head of each doc comment; a doc-drift test asserts the two
/// lists are the same set, so a row cannot be added or renamed on one side
/// only. Every variant names the entity that violates it and, where the row
/// says so, the parameter or the second entity.
#[derive(Debug, Clone, PartialEq, PartialOrd)]
#[non_exhaustive]
pub enum Violation {
    /// **M1** — an id referenced by an entity of the body does not resolve
    /// in this model with the stored generation.
    Unresolved {
        /// The referencing entity.
        from: EntityId,
        /// What it referenced.
        to: Reference,
    },
    /// **M2** — a reference a parent in the body makes is missing from the
    /// adjacency index of the entity it names: the reference did not
    /// resolve when the parent was appended, so the index never saw it.
    NotIndexed {
        /// The entity named.
        entity: EntityId,
        /// The parent that names it.
        referenced_by: EntityId,
    },
    /// **M3** — a coordinate, parameter or tolerance is not finite.
    NonFinite {
        /// The entity holding the number (for geometry, the entity that
        /// references it).
        entity: EntityId,
        /// Which kind of number.
        quantity: Quantity,
    },
    /// **V1** — a vertex's tolerance is outside
    /// `[Precision::min_tolerance, Precision::max_tolerance]`.
    VertexTolerance {
        /// The vertex.
        vertex: VertexId,
        /// Its tolerance.
        tolerance: f64,
        /// Which bound it broke.
        bound: ToleranceBound,
    },
    /// **V2** — an incident edge's curve, at the end of its range that
    /// names this vertex, is farther from the vertex's point than the
    /// vertex's tolerance.
    VertexOffEdge {
        /// The vertex.
        vertex: VertexId,
        /// The edge.
        edge: EdgeId,
        /// Distance from the curve end to the vertex's point.
        distance: f64,
    },
    /// **V3** — for a face the vertex lies on, the surface at the pcurve's
    /// end is farther from the vertex's point than the vertex's tolerance.
    VertexOffFace {
        /// The vertex.
        vertex: VertexId,
        /// The face.
        face: FaceId,
        /// Distance from the surface point to the vertex's point.
        distance: f64,
    },
    /// **E1** — a non-degenerate edge has no curve, an empty range, a range
    /// outside the curve's domain, or one that crosses the period more than
    /// once.
    EdgeRange {
        /// The edge.
        edge: EdgeId,
    },
    /// **E2** — the edge's vertices do not match the shape of its curve
    /// over its range: a closed curve with two vertices, or an open one
    /// with a single vertex.
    EdgeEnds {
        /// The edge.
        edge: EdgeId,
        /// What is wrong.
        fault: EndMismatch,
    },
    /// **E3** — an edge of the body is used by no coedge and is not a free
    /// edge of a wire or general body.
    EdgeUnused {
        /// The edge.
        edge: EdgeId,
    },
    /// **E4** — the surface evaluated along a coedge's pcurve is farther
    /// from the 3D curve at the same parameter than the edge's tolerance and
    /// the rounding of their coordinates.
    PcurveOffCurve {
        /// The edge.
        edge: EdgeId,
        /// The face whose pcurve was sampled.
        face: FaceId,
        /// The parameter at which the gap was largest.
        parameter: f64,
        /// The gap.
        distance: f64,
    },
    /// **E5** — the edge's tolerance is below a face it bounds or above a
    /// vertex it ends at.
    EdgeTolerance {
        /// The edge.
        edge: EdgeId,
        /// Its tolerance.
        tolerance: f64,
        /// Which bound it broke.
        bound: ToleranceBound,
    },
    /// **E6** — a degenerate edge has two vertices, or a pcurve along which
    /// the surface is not singular.
    DegenerateEdge {
        /// The edge.
        edge: EdgeId,
        /// What is wrong.
        fault: DegenerateFault,
    },
    /// **E7** — an edge used twice by one loop is not a seam: same
    /// orientation, or pcurves not one period apart.
    Seam {
        /// The edge.
        edge: EdgeId,
        /// The face whose loop uses it twice.
        face: FaceId,
        /// What is wrong.
        fault: SeamFault,
    },
    /// **E8** — the edge's curve self-intersects within its range.
    EdgeSelfIntersects {
        /// The edge.
        edge: EdgeId,
        /// The first parameter of the crossing.
        t0: f64,
        /// The second parameter of the crossing.
        t1: f64,
    },
    /// **L1** — a loop is empty or does not close: a coedge's effective end
    /// vertex is not its successor's effective start vertex.
    LoopOpen {
        /// The face.
        face: FaceId,
        /// Index of the loop in the face.
        loop_index: usize,
        /// Where it breaks.
        fault: LoopBreak,
    },
    /// **L2** — the pcurves jump in (u, v) at a coedge junction by more than
    /// `Precision::parametric_tolerance`, and the junction is not a seam.
    PcurveGap {
        /// The face.
        face: FaceId,
        /// Index of the loop.
        loop_index: usize,
        /// Index of the coedge whose end does not meet its successor's start.
        coedge: usize,
        /// The (u, v) distance of the jump.
        gap: f64,
    },
    /// **L3** — an edge is used twice in one loop, or by two loops of the
    /// same face, without being a seam.
    EdgeReusedInFace {
        /// The face.
        face: FaceId,
        /// The edge.
        edge: EdgeId,
    },
    /// **L4** — a loop has zero signed area, or the loops of the face do not
    /// form one outer loop per domain component with holes inside it.
    LoopNesting {
        /// The face.
        face: FaceId,
        /// What is wrong.
        fault: NestingFault,
    },
    /// **L5** — two loops of a face intersect in (u, v), or one loop
    /// intersects itself (`loop_a == loop_b`).
    LoopsIntersect {
        /// The face.
        face: FaceId,
        /// Index of the first loop.
        loop_a: usize,
        /// Index of the second loop.
        loop_b: usize,
    },
    /// **F1** — the face has no loop, or a pcurve leaves the surface's
    /// bounded domain.
    FaceMalformed {
        /// The face.
        face: FaceId,
        /// What is wrong.
        fault: FaceFault,
    },
    /// **F2** — the face's tolerance is below `Precision::min_tolerance` or
    /// above an incident edge's.
    FaceTolerance {
        /// The face.
        face: FaceId,
        /// Its tolerance.
        tolerance: f64,
        /// Which bound it broke.
        bound: ToleranceBound,
    },
    /// **S1** — a shell uses the same face twice.
    FaceUsedTwice {
        /// The shell.
        shell: ShellId,
        /// The face.
        face: FaceId,
    },
    /// **S2** — an edge of the shell has the wrong number of coedge uses for
    /// the body's kind, or uses that do not pair with opposite effective
    /// orientation.
    EdgeUses {
        /// The shell.
        shell: ShellId,
        /// The edge.
        edge: EdgeId,
        /// What is wrong.
        fault: EdgeUseFault,
    },
    /// **S3** — the shell's faces do not form one edge-connected component.
    ShellDisconnected {
        /// The shell.
        shell: ShellId,
        /// Number of components found.
        components: usize,
    },
    /// **S4** — a shell of a solid has an edge with a single coedge.
    ShellOpen {
        /// The shell.
        shell: ShellId,
        /// The edge with one coedge.
        edge: EdgeId,
    },
    /// **S5** — two faces of a shell intersect away from their shared edges
    /// and vertices.
    FacesIntersect {
        /// The shell.
        shell: ShellId,
        /// The first face.
        face_a: FaceId,
        /// The second face.
        face_b: FaceId,
    },
    /// **B1** — a solid body's shells do not nest into lumps: outer shells
    /// enclosing positive volume, each with the voids whose innermost
    /// container it is, and no two shells meeting.
    ShellNesting {
        /// The body.
        body: BodyId,
        /// What is wrong.
        fault: ShellNestingFault,
    },
    /// **B2** — a solid body's enclosed volume, by Gauss over its faces, is
    /// not positive.
    NonPositiveVolume {
        /// The body.
        body: BodyId,
        /// The volume found.
        volume: f64,
    },
    /// **B3** — a wire body has a shell, or its free edges do not form
    /// chains.
    WireMalformed {
        /// The body.
        body: BodyId,
        /// What is wrong.
        fault: WireFault,
    },
}

impl Violation {
    /// The row number in `docs/DATA-MODEL.md` §Invariants: `"M1"`,
    /// `"E4"`, … Stable; what a report sorts and prints by.
    pub const fn code(&self) -> &'static str {
        match self {
            Violation::Unresolved { .. } => "M1",
            Violation::NotIndexed { .. } => "M2",
            Violation::NonFinite { .. } => "M3",
            Violation::VertexTolerance { .. } => "V1",
            Violation::VertexOffEdge { .. } => "V2",
            Violation::VertexOffFace { .. } => "V3",
            Violation::EdgeRange { .. } => "E1",
            Violation::EdgeEnds { .. } => "E2",
            Violation::EdgeUnused { .. } => "E3",
            Violation::PcurveOffCurve { .. } => "E4",
            Violation::EdgeTolerance { .. } => "E5",
            Violation::DegenerateEdge { .. } => "E6",
            Violation::Seam { .. } => "E7",
            Violation::EdgeSelfIntersects { .. } => "E8",
            Violation::LoopOpen { .. } => "L1",
            Violation::PcurveGap { .. } => "L2",
            Violation::EdgeReusedInFace { .. } => "L3",
            Violation::LoopNesting { .. } => "L4",
            Violation::LoopsIntersect { .. } => "L5",
            Violation::FaceMalformed { .. } => "F1",
            Violation::FaceTolerance { .. } => "F2",
            Violation::FaceUsedTwice { .. } => "S1",
            Violation::EdgeUses { .. } => "S2",
            Violation::ShellDisconnected { .. } => "S3",
            Violation::ShellOpen { .. } => "S4",
            Violation::FacesIntersect { .. } => "S5",
            Violation::ShellNesting { .. } => "B1",
            Violation::NonPositiveVolume { .. } => "B2",
            Violation::WireMalformed { .. } => "B3",
        }
    }

    /// The level at which this invariant runs.
    pub const fn level(&self) -> Level {
        match self {
            Violation::EdgeSelfIntersects { .. }
            | Violation::LoopsIntersect { .. }
            | Violation::FacesIntersect { .. }
            | Violation::ShellNesting { .. }
            | Violation::NonPositiveVolume { .. } => Level::Full,
            Violation::Unresolved { .. }
            | Violation::NotIndexed { .. }
            | Violation::NonFinite { .. }
            | Violation::VertexTolerance { .. }
            | Violation::VertexOffEdge { .. }
            | Violation::VertexOffFace { .. }
            | Violation::EdgeRange { .. }
            | Violation::EdgeEnds { .. }
            | Violation::EdgeUnused { .. }
            | Violation::PcurveOffCurve { .. }
            | Violation::EdgeTolerance { .. }
            | Violation::DegenerateEdge { .. }
            | Violation::Seam { .. }
            | Violation::LoopOpen { .. }
            | Violation::PcurveGap { .. }
            | Violation::EdgeReusedInFace { .. }
            | Violation::LoopNesting { .. }
            | Violation::FaceMalformed { .. }
            | Violation::FaceTolerance { .. }
            | Violation::FaceUsedTwice { .. }
            | Violation::EdgeUses { .. }
            | Violation::ShellDisconnected { .. }
            | Violation::ShellOpen { .. }
            | Violation::WireMalformed { .. } => Level::Fast,
        }
    }

    /// The entity that violates the invariant: what the report is sorted
    /// by and what an error names first.
    pub fn entity(&self) -> EntityId {
        match *self {
            Violation::Unresolved { from, .. } => from,
            Violation::NotIndexed { entity, .. } => entity,
            Violation::NonFinite { entity, .. } => entity,
            Violation::VertexTolerance { vertex, .. }
            | Violation::VertexOffEdge { vertex, .. }
            | Violation::VertexOffFace { vertex, .. } => vertex.into(),
            Violation::EdgeRange { edge }
            | Violation::EdgeEnds { edge, .. }
            | Violation::EdgeUnused { edge }
            | Violation::PcurveOffCurve { edge, .. }
            | Violation::EdgeTolerance { edge, .. }
            | Violation::DegenerateEdge { edge, .. }
            | Violation::Seam { edge, .. }
            | Violation::EdgeSelfIntersects { edge, .. } => edge.into(),
            Violation::LoopOpen { face, .. }
            | Violation::PcurveGap { face, .. }
            | Violation::EdgeReusedInFace { face, .. }
            | Violation::LoopNesting { face, .. }
            | Violation::LoopsIntersect { face, .. }
            | Violation::FaceMalformed { face, .. }
            | Violation::FaceTolerance { face, .. } => face.into(),
            Violation::FaceUsedTwice { shell, .. }
            | Violation::EdgeUses { shell, .. }
            | Violation::ShellDisconnected { shell, .. }
            | Violation::ShellOpen { shell, .. }
            | Violation::FacesIntersect { shell, .. } => shell.into(),
            Violation::ShellNesting { body, .. }
            | Violation::NonPositiveVolume { body, .. }
            | Violation::WireMalformed { body, .. } => body.into(),
        }
    }
}

impl fmt::Display for ShellNestingFault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ShellNestingFault::NoShells => f.write_str("solid with no shell"),
            ShellNestingFault::NoOuter => f.write_str("no outer shell"),
            ShellNestingFault::VoidOutside { shell } => {
                write!(f, "void {shell} is inside no shell")
            }
            ShellNestingFault::VoidInVoid { shell, container } => {
                write!(f, "void {shell} is inside void {container}")
            }
            ShellNestingFault::OuterInOuter { shell, container } => write!(
                f,
                "outer shell {shell} is inside outer shell {container} with no void between"
            ),
            ShellNestingFault::Overlap { shells } => {
                write!(f, "shells {} and {} meet", shells[0], shells[1])
            }
            ShellNestingFault::InsideOut { shell } => {
                write!(
                    f,
                    "{shell} encloses no volume, so it is neither outer nor void"
                )
            }
        }
    }
}

impl fmt::Display for ToleranceBound {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ToleranceBound::BelowMinimum => f.write_str("below Precision::min_tolerance"),
            ToleranceBound::AboveMaximum => f.write_str("above Precision::max_tolerance"),
            ToleranceBound::Neighbour { entity, tolerance } => {
                write!(f, "out of order with {entity} at {tolerance:e}")
            }
        }
    }
}

impl fmt::Display for Violation {
    /// One line: the code, the entity, and what is wrong with which
    /// numbers. The line is deterministic for equal violations.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} {}: ", self.code(), self.entity())?;
        match self {
            Violation::Unresolved { to, .. } => {
                write!(f, "references {to}, which does not resolve")
            }
            Violation::NotIndexed { referenced_by, .. } => {
                write!(
                    f,
                    "referenced by {referenced_by} but absent from the adjacency index"
                )
            }
            Violation::NonFinite { quantity, .. } => {
                let q = match quantity {
                    Quantity::Coordinate => "coordinate",
                    Quantity::Parameter => "parameter",
                    Quantity::Tolerance => "tolerance",
                };
                write!(f, "non-finite {q}")
            }
            Violation::VertexTolerance {
                tolerance, bound, ..
            }
            | Violation::EdgeTolerance {
                tolerance, bound, ..
            }
            | Violation::FaceTolerance {
                tolerance, bound, ..
            } => {
                write!(f, "tolerance {tolerance:e} is {bound}")
            }
            Violation::VertexOffEdge { edge, distance, .. } => {
                write!(f, "{edge}'s curve ends {distance:e} away")
            }
            Violation::VertexOffFace { face, distance, .. } => {
                write!(f, "{face}'s surface at the pcurve end is {distance:e} away")
            }
            Violation::EdgeRange { .. } => f.write_str("no curve or an invalid range"),
            Violation::EdgeEnds { fault, .. } => match fault {
                EndMismatch::OpenWithOneVertex { vertex, gap } => {
                    write!(
                        f,
                        "open curve (ends {gap:e} apart) but one vertex, {vertex}"
                    )
                }
                EndMismatch::ClosedWithTwoVertices { start, end } => {
                    write!(f, "closed curve but two vertices, {start} and {end}")
                }
            },
            Violation::EdgeUnused { .. } => f.write_str("used by no coedge and not a free edge"),
            Violation::PcurveOffCurve {
                face,
                parameter,
                distance,
                ..
            } => {
                write!(
                    f,
                    "pcurve on {face} is {distance:e} off the curve at t = {parameter}"
                )
            }
            Violation::DegenerateEdge { fault, .. } => match fault {
                DegenerateFault::TwoVertices => f.write_str("degenerate edge with two vertices"),
                DegenerateFault::NotSingular { face, extent } => {
                    write!(
                        f,
                        "{face}'s surface is not singular along the pcurve (extent {extent:e})"
                    )
                }
            },
            Violation::Seam { face, fault, .. } => match fault {
                SeamFault::SameOrientation => {
                    write!(f, "used twice by a loop of {face} in the same orientation")
                }
                SeamFault::PeriodMismatch { offset, period } => write!(
                    f,
                    "seam on {face}: pcurves differ by {offset} where the period is {period}"
                ),
            },
            Violation::EdgeSelfIntersects { t0, t1, .. } => {
                write!(f, "self-intersects at t = {t0} and t = {t1}")
            }
            Violation::LoopOpen {
                loop_index, fault, ..
            } => match fault {
                LoopBreak::Empty => write!(f, "loop {loop_index} is empty"),
                LoopBreak::Between { coedge } => {
                    write!(f, "loop {loop_index} breaks after coedge {coedge}")
                }
            },
            Violation::PcurveGap {
                loop_index,
                coedge,
                gap,
                ..
            } => {
                write!(
                    f,
                    "loop {loop_index}: pcurves jump {gap:e} in (u, v) after coedge {coedge}"
                )
            }
            Violation::EdgeReusedInFace { edge, .. } => {
                write!(f, "uses {edge} more than once without a seam")
            }
            Violation::LoopNesting { fault, .. } => match fault {
                NestingFault::ZeroArea { loop_index } => {
                    write!(f, "loop {loop_index} has zero signed area")
                }
                NestingFault::NoOuter => f.write_str("no outer loop"),
                NestingFault::MultipleOuter { loops } => {
                    write!(f, "several outer loops: {loops:?}")
                }
                NestingFault::HoleOutside { loop_index } => {
                    write!(f, "hole loop {loop_index} lies outside every outer loop")
                }
            },
            Violation::LoopsIntersect { loop_a, loop_b, .. } => {
                if loop_a == loop_b {
                    write!(f, "loop {loop_a} intersects itself")
                } else {
                    write!(f, "loops {loop_a} and {loop_b} intersect")
                }
            }
            Violation::FaceMalformed { fault, .. } => match fault {
                FaceFault::NoLoops => f.write_str("no loops"),
                FaceFault::PcurveOutsideDomain { loop_index, coedge } => write!(
                    f,
                    "loop {loop_index} coedge {coedge}: pcurve leaves the surface's domain"
                ),
            },
            Violation::FaceUsedTwice { face, .. } => write!(f, "uses {face} twice"),
            Violation::EdgeUses { edge, fault, .. } => match fault {
                EdgeUseFault::Count { coedges } => {
                    write!(f, "{edge} is used by {coedges} coedge(s)")
                }
                EdgeUseFault::Orientation => {
                    write!(f, "{edge}'s uses do not pair with opposite orientation")
                }
            },
            Violation::ShellDisconnected { components, .. } => {
                write!(f, "{components} edge-connected components")
            }
            Violation::ShellOpen { edge, .. } => write!(f, "open at {edge}, which has one coedge"),
            Violation::FacesIntersect { face_a, face_b, .. } => {
                write!(
                    f,
                    "{face_a} and {face_b} intersect away from their shared edges"
                )
            }
            Violation::ShellNesting { fault, .. } => write!(f, "{fault}"),
            Violation::NonPositiveVolume { volume, .. } => {
                write!(f, "encloses volume {volume}")
            }
            Violation::WireMalformed { fault, .. } => match fault {
                WireFault::HasShell { shell } => write!(f, "wire body lists shell {shell}"),
                WireFault::VertexOverused { vertex, edges } => {
                    write!(f, "{vertex} is used by {edges} free edges")
                }
            },
        }
    }
}
