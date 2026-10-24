//! `BuildError` nests its refusals by kind (`SlotError`, `EulerError`,
//! `ClosureError`); the nesting is transparent, so every message is the
//! one the flat enum had, which the binding and the refusal histograms
//! print.

use arris_topo::builder::{
    BuildError, ClosureError, Counts, EdgeRef, EulerError, FaceRef, Position, SlotError, VertexRef,
};
use arris_topo::entity::BodyKind;
use arris_topo::{EntityId, EntityKind, SurfaceId, VertexId};

#[test]
fn every_group_prints_the_message_the_flat_variant_had() {
    let (v, w) = (VertexRef::new(1), VertexRef::new(4));
    let (e, d) = (EdgeRef::new(2), EdgeRef::new(5));
    let (f, g) = (FaceRef::new(3), FaceRef::new(6));
    let at = Position::new(f, 0, 2);
    let to = Position::new(f, 1, 0);
    let counts = Counts {
        vertices: 1,
        edges: 2,
        faces: 3,
        loops: 4,
        shells: 1,
        genus: 0,
    };
    let (s, t) = (SurfaceId::new(0, 0), SurfaceId::new(1, 0));
    let id = EntityId::Vertex(VertexId::new(7, 0));

    let slot = |x: SlotError| BuildError::Slot(x).to_string();
    let euler = |x: EulerError| BuildError::Euler(x).to_string();
    let closure = |x: ClosureError| BuildError::Closure(x).to_string();

    assert_eq!(
        slot(SlotError::NoVertex(v)),
        format!("{v} is not a vertex of the builder")
    );
    assert_eq!(
        slot(SlotError::NoEdge(e)),
        format!("{e} is not an edge of the builder")
    );
    assert_eq!(
        slot(SlotError::NoFace(f)),
        format!("{f} is not a face of the builder")
    );
    assert_eq!(
        slot(SlotError::NoLoop {
            face: f,
            loop_index: 2
        }),
        format!("{f} has no loop 2")
    );
    assert_eq!(
        slot(SlotError::BadPosition {
            position: at,
            len: 2
        }),
        format!("{at} is past the end of a loop of 2 coedge(s)")
    );
    assert_eq!(
        slot(SlotError::NoSpec {
            kind: EntityKind::Edge,
            index: 9
        }),
        "the assembly has no edge spec at index 9"
    );

    assert_eq!(
        euler(EulerError::NotEmpty),
        "the builder already holds a body; mvfs starts one from nothing"
    );
    assert_eq!(
        euler(EulerError::NotOneLoop { from: at, to }),
        format!("{at} and {to} are not in the same loop")
    );
    assert_eq!(
        euler(EulerError::NotTwoLoops { from: at, to }),
        format!("{at} and {to} are not two loops of one face")
    );
    assert_eq!(
        euler(EulerError::Ambiguous {
            vertex: v,
            face: f,
            loop_index: 1,
            positions: vec![0, 2]
        }),
        format!("{v} is at junctions [0, 2] of loop 1 of {f}")
    );
    assert_eq!(
        euler(EulerError::NotInLoop {
            vertex: v,
            face: f,
            loop_index: 1
        }),
        format!("{v} is not on loop 1 of {f}")
    );
    assert_eq!(
        euler(EulerError::NotASeed),
        "kvfs needs one vertex, one face with one empty loop and nothing else"
    );
    assert_eq!(
        euler(EulerError::NotAStrut { edge: e }),
        format!("{e} is not a strut: its two uses are not consecutive in one loop")
    );
    assert_eq!(
        euler(EulerError::VertexInUse {
            vertex: v,
            other: d
        }),
        format!("{v} is used by {d}")
    );
    assert_eq!(
        euler(EulerError::VertexSeeds {
            vertex: v,
            face: f,
            loop_index: 1
        }),
        format!("{v} seeds loop 1 of {f}")
    );
    assert_eq!(
        euler(EulerError::NotSeparating { edge: e }),
        format!("{e} does not separate two faces")
    );
    assert_eq!(
        euler(EulerError::NotInOneLoop { edge: e }),
        format!("{e} is not used twice by one loop")
    );
    assert_eq!(
        euler(EulerError::Rings { face: f, loops: 2 }),
        format!("{f} has 2 loops; it must have one")
    );
    assert_eq!(
        euler(EulerError::NoRing { face: f, ring: 3 }),
        format!("{f} has no loop 3 to make a face of, or it is the only one")
    );
    assert_eq!(
        euler(EulerError::OneFace { face: f }),
        format!("{f} cannot be joined to itself")
    );
    assert_eq!(
        euler(EulerError::SurfaceMismatch {
            kill: f,
            kill_surface: s,
            into: g,
            into_surface: t
        }),
        format!("{f} is on {s} and {g} on {t}; kfmrh needs one surface")
    );
    assert_eq!(
        euler(EulerError::SameOrientation { kill: f, into: g }),
        format!("{f} and {g} are used with the same orientation; a handle needs opposite ones")
    );
    assert_eq!(
        euler(EulerError::NoHandle),
        "no handle to remove: the genus is zero"
    );
    assert_eq!(
        euler(EulerError::DegenerateStrut),
        "a strut has two vertices and needs a curve; a degenerate edge has neither"
    );
    assert_eq!(
        euler(EulerError::DegenerateEnds { start: v, end: w }),
        format!("a degenerate edge joins one vertex to itself, not {v} to {w}")
    );

    assert_eq!(closure(ClosureError::Empty), "the builder holds no face");
    assert_eq!(
        closure(ClosureError::Kind(BodyKind::Wire)),
        format!(
            "the builder makes a solid or a sheet, not a {} body",
            BodyKind::Wire
        )
    );
    assert_eq!(
        closure(ClosureError::EmptyLoop {
            face: f,
            loop_index: 1
        }),
        format!("loop 1 of {f} has no coedges")
    );
    assert_eq!(
        closure(ClosureError::MissingPcurve { position: at }),
        format!("the coedge at {at} has no pcurve")
    );
    assert_eq!(
        closure(ClosureError::EdgeUses { edge: e, uses: 3 }),
        format!("{e} is used by 3 coedge(s); a solid's edge needs two, a degenerate edge one")
    );
    assert_eq!(
        closure(ClosureError::SameDirection { edge: e }),
        format!("{e} is used twice in the same direction")
    );
    assert_eq!(
        closure(ClosureError::Duplicate(id)),
        format!("{id} is kept twice")
    );
    assert_eq!(
        closure(ClosureError::LoopOpen {
            face: f,
            loop_index: 1,
            coedge_index: 2,
            ended: v,
            starts: w
        }),
        format!("loop 1 of {f} is open at junction 2: {v} then {w}")
    );
    assert_eq!(
        closure(ClosureError::Disconnected { face: g, from: f }),
        format!("{g} is not edge-connected to {f}, the first face of its shell")
    );
    assert_eq!(
        closure(ClosureError::NotClosed { counts }),
        format!("{counts} is not a closed surface of whole genus")
    );
    assert_eq!(
        closure(ClosureError::EmptyShell { shell: 2 }),
        "shell 2 of the assembly has no faces"
    );
    assert_eq!(
        closure(ClosureError::SharedEdge {
            edge: e,
            shells: [0, 1]
        }),
        format!("{e} is used by shells 0 and 1")
    );
    assert_eq!(
        closure(ClosureError::SharedVertex {
            vertex: v,
            shells: [0, 1]
        }),
        format!("{v} is on edges of shells 0 and 1")
    );
    assert_eq!(
        BuildError::Invariant { face: f }.to_string(),
        format!("{f} has neither the use nor the empty loop a kill left a position after")
    );
}
