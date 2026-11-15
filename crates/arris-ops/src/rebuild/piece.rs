//! The vocabulary a producer hands [`assembly`](super::assembly) its
//! pieces in: a vertex, an edge and a use of an edge, each named before it
//! has an id. `rebuild` owns it, so the boolean that produces pieces
//! depends on `rebuild` and never the reverse (ADR-0004).

use arris_math::{Interval, Point3};
use arris_topo::{Curve2Id, CurveId, EdgeId, Orientation, VertexId};

/// A vertex of the result as the split names it before it has an id.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum VRef {
    /// A vertex of an operand, kept or re-tolerated.
    Existing(VertexId),
    /// The `k`-th section vertex the producer hands `assembly`.
    Section(usize),
}

/// An edge of the result as the split names it before it has an id.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum ERef {
    /// The `index`-th piece of an operand edge between consecutive paves;
    /// index 0 is the whole edge when it has none.
    Sub {
        /// The edge.
        edge: EdgeId,
        /// Which piece, ascending along the edge's parameter.
        index: usize,
    },
    /// The `k`-th section edge the producer hands `assembly`.
    Section(usize),
}

/// A piece of an operand edge between consecutive paves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SubEdge {
    /// The parameter range on the edge's curve.
    pub range: Interval,
    /// The vertex at `range.lo()`.
    pub start: VRef,
    /// The vertex at `range.hi()`.
    pub end: VRef,
}

/// A section vertex the producer hands `assembly`, indexed by
/// [`VRef::Section`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct NewVertex {
    /// Where it is.
    pub point: Point3,
    /// Its tolerance.
    pub tolerance: f64,
}

/// A section edge the producer hands `assembly`, indexed by
/// [`ERef::Section`]: its curve is already the model's id and its ends
/// are already vertices of the result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct NewSection {
    /// The curve, in the model.
    pub curve: CurveId,
    /// The parameter range on it.
    pub range: Interval,
    /// The vertex at `range.lo()`.
    pub start: VRef,
    /// The vertex at `range.hi()`.
    pub end: VRef,
    /// Its tolerance.
    pub tolerance: f64,
}

/// One use of an edge piece by a piece's loop, in the *stored* sense:
/// `Forward` walks along the edge's parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PieceUse {
    /// The edge piece.
    pub edge: ERef,
    /// Along or against its parameter.
    pub orientation: Orientation,
    /// The pcurve of the use on this face.
    pub pcurve: Curve2Id,
}
