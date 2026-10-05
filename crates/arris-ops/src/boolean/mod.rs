//! The boolean decomposition (ADR-0004): the pave model as a value.
//!
//! `fuse`, `common` and `cut` are three selections over one
//! decomposition of their two operands, and [`interferences`] is that
//! decomposition made visible — every face pair whose boxes overlap with
//! what its surfaces have in common, every point where an edge of one
//! operand pierces a face of the other, those points merged into section
//! vertices, the paves they put on every edge and every section curve,
//! and the section edges with a pcurve on each face. A boolean that goes
//! wrong is debugged here, at the pave, before any result exists (the
//! `inspect` skill).

mod faces;
mod pave;
pub(crate) mod pieces;
mod result;

use core::fmt;
use std::collections::BTreeMap;

use arris_geom::{Curve, Curve2, MeetKind, SurfaceIntersection};
use arris_math::{Control, Interval, Meter, Point2, Point3};
use arris_topo::{Body, Curve2Id, EdgeId, FaceId, Model, Provenance, Shape, VertexId};

use crate::error::OpError;

/// The curves of `intersection` that meet as `kind`, each with its
/// index among the `Meets` curves: what a section curve's or a contact's
/// index names.
pub(super) fn meet_curves(
    intersection: &SurfaceIntersection,
    kind: MeetKind,
) -> impl Iterator<Item = (usize, &Curve)> {
    intersection
        .curves()
        .iter()
        .enumerate()
        .filter(move |(_, c)| c.kind == kind)
        .map(|(i, c)| (i, &c.curve))
}

/// One face pair whose bounding boxes overlap, with what their surfaces
/// have in common: the pair a boolean has to decide.
#[derive(Debug, Clone, PartialEq)]
pub struct FacePair {
    /// The face of the pair's first operand, `operands[0]`.
    pub a: FaceId,
    /// The face of the second, `operands[1]`.
    pub b: FaceId,
    /// The operands the two faces belong to, indices into
    /// [`Interferences::operands`], the lower first: a pair is always of
    /// two different operands.
    pub operands: [usize; 2],
    /// Their surfaces' intersection. The crossing curves of a `Meets`
    /// pair are the section curves; a `Coincident` pair is decided by the
    /// arrangement of the two faces on one surface — its
    /// [`Interferences::crossings`], [`Interferences::images`] and
    /// [`Interferences::blocks`] — and the touching curves of a `Meets`
    /// by its [`Interferences::contacts`] — the blocks of the tangent
    /// curve interior to both faces — and the curvature rule at each;
    /// neither contributes a section edge. A `Meets` holding both kinds
    /// is read as both, its crossing curves sections and its touching
    /// curves contacts. Its points are read only as a traced section's
    /// singular points, where its branches end (ADR-0018), each a section
    /// crossing.
    pub intersection: SurfaceIntersection,
}

/// What a hit landed on within the face it pierced.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Landing {
    /// Strictly inside the face's region in (u, v).
    Interior,
    /// On the face's boundary: within the tolerance of this edge or
    /// vertex of the face, the most specific one — an edge of one
    /// operand crossing an edge of the other, or passing through its
    /// vertex.
    Boundary(Shape),
}

/// One point where an edge of one operand meets a face of the other:
/// the edge's curve against the face's surface, kept when the parameter
/// is in the edge's range and the (u, v) is on the face — or, beside a
/// touch that landed on no vertex, the edge's curve against a section
/// curve of that face and a face of the edge's own (ADR-0016): the same
/// kind of hit, found where the surface alone places it only to a square
/// root of rounding.
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeFaceHit {
    /// The edge.
    pub edge: EdgeId,
    /// The face.
    pub face: FaceId,
    /// The edge's parameter, in its range.
    pub t: f64,
    /// The face's (u, v), in the translate of the domain its loops are
    /// written in.
    pub uv: Point2,
    /// The edge's point at `t`.
    pub point: Point3,
    /// `true` when the edge touches the surface here without crossing
    /// it. A touch pierces nothing: it makes no section vertex of its
    /// own; it paves a touching curve of the pair, whose blocks
    /// between touches are the pair's [`Interferences::contacts`]. A
    /// touch that lands on a section vertex the hits and crossings made —
    /// a ruling or a rim circle through the crossing of two ellipses,
    /// where the walls are tangent to each other — passes through that
    /// vertex, so it joins it and paves the edge there. A touch that
    /// lands on none is a verdict on depth alone, the edge within the
    /// tolerance of the surface, and may stand for two crossings far
    /// apart along it: those are found through the section curves and
    /// listed as hits of their own beside it (ADR-0016).
    pub tangent: bool,
    /// Where on the face.
    pub landing: Landing,
    /// The end vertex of the edge the hit lies within the tolerance of,
    /// when it does: a vertex of this operand on a face of the other.
    pub at_vertex: Option<VertexId>,
    /// The section vertex the hit was merged into; `None` for a touch
    /// that landed on none.
    pub vertex: Option<usize>,
}

/// Where a section vertex came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VertexSource {
    /// Hits or edge–edge crossings merged: at least one of either. A
    /// section crossing may have joined it too.
    Hits,
    /// Two section curves of one crossing pair crossing each other, or
    /// a traced section's branches ending at its singular point, where
    /// no edge of either operand pierces: at least one section crossing,
    /// and no hit but touches landing on it.
    SectionCrossing,
    /// A closed section curve of the pair no hit paves, interior to both
    /// faces: its own point at the start of its domain — parameter zero
    /// for a conic.
    CurveStart {
        /// The pair, an index into [`Interferences::pairs`].
        pair: usize,
        /// Which of the pair's curves.
        curve: usize,
    },
    /// A singular vertex of an operand face — a cone's apex, a sphere's
    /// pole — that a section curve runs through where no edge pierces
    /// (ADR-0021): the seam that ends there only touches the other
    /// surface, or the face has none. No crossing hit, the operand's
    /// vertex in `existing`. Where a seam does pierce, the vertex is that
    /// hit's and its source [`VertexSource::Hits`]; either way the face's
    /// degenerate edge is paved for every section edge that ends there.
    Singular,
}

/// A point of the section, made once and shared: a pave on the edge
/// that pierced (every hit merged into it) and on every section curve it
/// lies on, so the pieces of the two operands meet exactly (ADR-0004).
#[derive(Debug, Clone, PartialEq)]
pub struct SectionVertex {
    /// The point: the first operand vertex it coincides with, else its
    /// first candidate on an operand edge — a hit's, a touch's or a
    /// crossing's, the edge being cut there exactly — else its first
    /// candidate's, in the order of making.
    pub point: Point3,
    /// The largest tolerance of the entities whose hits it merges, plus
    /// the spread of the merged points about `point`
    /// (`docs/DATA-MODEL.md` §Tolerances).
    pub tolerance: f64,
    /// The hits merged into it, ascending indices into
    /// [`Interferences::hits`].
    pub hits: Vec<usize>,
    /// The edge–edge crossings merged into it, ascending indices into
    /// [`Interferences::crossings`].
    pub crossings: Vec<usize>,
    /// The section crossings merged into it, ascending indices into
    /// [`Interferences::section_crossings`].
    pub section_crossings: Vec<usize>,
    /// The operand vertices it coincides with — a hit at an edge's end,
    /// or one landing on a vertex of the face — ascending. Usually none.
    pub existing: Vec<VertexId>,
    /// Where it came from.
    pub source: VertexSource,
}

/// A section vertex at a parameter of a curve: on an operand's edge, or
/// on a section curve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pave {
    /// The curve's parameter; a periodic one in `[lo, lo + period)` of
    /// its domain — `[0, 2π)` for a conic.
    pub t: f64,
    /// The section vertex, an index into [`Interferences::vertices`].
    pub vertex: usize,
}

/// One curve two faces cross along, with the paves that cut it into
/// blocks and the blocks that survived as section edges.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionCurve {
    /// The pair, an index into [`Interferences::pairs`].
    pub pair: usize,
    /// Which of the pair's crossing curves: an index into its `Meets`
    /// curves.
    pub index: usize,
    /// The curve, as `intersect_surfaces` gave it.
    pub curve: Curve,
    /// The paves, ascending by `t`.
    pub paves: Vec<Pave>,
    /// The section edges on it, indices into [`Interferences::sections`].
    pub edges: Vec<usize>,
}

/// One point where two section curves of one pair cross
/// each other on both faces: the two curves intersected, kept when the
/// point is inside each face or on its boundary. Two surfaces meet in
/// two curves that cross where they are tangent to each other — the two
/// ellipses of equal cylinders with crossing axes, at `±R` along the
/// axes' common perpendicular — and no edge of either operand marks the
/// point, so the crossing is a section vertex of its own: it paves both
/// curves, and the pieces of both faces meet at it (ADR-0004). A traced
/// pair's fitted branches meet only at its singular points, where they
/// end exactly (ADR-0018): its crossings are those points on both faces,
/// the first branch ending at one against each other end there — against
/// its own other end when one branch leaves the point and comes back —
/// and no two fitted curves are intersected.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionCrossing {
    /// The pair, an index into [`Interferences::pairs`].
    pub pair: usize,
    /// Which two of the pair's crossing curves, indices into its `Meets`
    /// curves, the lower first; one curve twice for a traced branch whose
    /// two ends are the singular point.
    pub curves: [usize; 2],
    /// The parameter on each of the two curves, a periodic one in
    /// `[lo, lo + period)` of its domain; at a singular point, the end of
    /// each branch there.
    pub t: [f64; 2],
    /// The point, on the first curve; a singular point as the tracer
    /// gave it.
    pub point: Point3,
    /// `true` when the curves touch here without crossing. A touch makes
    /// no vertex and no pave.
    pub tangent: bool,
    /// The section vertex it was merged into; `None` for a touch.
    pub vertex: Option<usize>,
}

/// A block of a section curve interior to both faces: what becomes an
/// edge of the result, with its pcurve on each face already fitted.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionEdge {
    /// The curve, an index into [`Interferences::curves`].
    pub curve: usize,
    /// The parameter range on it, between two consecutive paves; on a
    /// periodic curve with one pave, one whole turn from it.
    pub range: Interval,
    /// The section vertex at `range.lo()`.
    pub start: usize,
    /// The section vertex at `range.hi()`.
    pub end: usize,
    /// The larger of the two faces' tolerances, raised to the largest
    /// deviation of either pcurve's image from the curve.
    pub tolerance: f64,
    /// The pcurves on the pair's faces, `a` then `b`, same-parameter
    /// with `range`, each in the translate of the domain that face's
    /// loops are written in, and each ending within half of L2's band of
    /// its vertices' own (u, v) on that face — moved there where the
    /// vertex merged points further apart (`docs/ARCHITECTURE.md`
    /// §Operations).
    pub pcurves: [Curve2; 2],
}

/// A block of a pair's touching curve interior to both faces: where the
/// two faces touch along a curve. The curve is paved by every hit of
/// either face's edges on the other face that lies on it — the touches,
/// which are where the curve leaves one face inside the other — and a
/// block between consecutive paves whose midpoint is inside both faces
/// is a contact; on a closed curve the last block wraps round to the
/// first pave, and a closed curve with none is one block of a whole
/// period. A contact contributes no section edge and splits
/// nothing; the boolean decides at its midpoint, by the curvature rule,
/// whether the piece of each face through it would survive, and refuses
/// with [`crate::BooleanReason::TangentContact`] when both would — the slit no
/// manifold `Solid` can carry (ADR-0004).
#[derive(Debug, Clone, PartialEq)]
pub struct Contact {
    /// The pair, an index into [`Interferences::pairs`].
    pub pair: usize,
    /// Which of the pair's touching curves: an index into its `Meets`
    /// curves.
    pub curve: usize,
    /// The parameter range on it, between two consecutive paves.
    pub range: Interval,
    /// The curve's point at the range's midpoint.
    pub point: Point3,
    /// That point's (u, v) on the pair's faces, `a` then `b`, each in
    /// the translate of the domain that face's loops are written in.
    pub uv: [Point2; 2],
}

/// One point where an edge of one face of a `Coincident` pair crosses an
/// edge of the other face on their shared surface: the two curves
/// intersected, kept when both parameters are in range. Such a point is
/// usually also an edge-on-face hit of a neighbouring face and merges
/// into the same section vertex; the pass exists so that a pair whose
/// neighbours are coincident too still gets its vertex.
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeEdgeHit {
    /// The pair, an index into [`Interferences::pairs`].
    pub pair: usize,
    /// The edge of `a`'s face.
    pub a: EdgeId,
    /// Its parameter, in its range.
    pub ta: f64,
    /// The edge of `b`'s face.
    pub b: EdgeId,
    /// Its parameter, in its range.
    pub tb: f64,
    /// The point, on `a`'s edge.
    pub point: Point3,
    /// `true` when the curves touch here without crossing. A touch makes
    /// no vertex and no pave.
    pub tangent: bool,
    /// The section vertex it was merged into; `None` for a touch.
    pub vertex: Option<usize>,
}

/// A piece of an edge of one face of a `Coincident` pair lying strictly
/// inside the other face: what splits that face where the two overlap,
/// with its pcurve there (ADR-0004). The same for an edge lying in a face
/// of the other operand that no face of its own is coincident with — a
/// seam on the ruling two parallel walls cross along, which is that
/// block of the section curve — under the pair of the first face that
/// uses it. The piece is the edge's own — the result uses one edge for
/// both faces — and the pcurve is what the other face's loop uses it
/// with.
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeImage {
    /// The pair, an index into [`Interferences::pairs`].
    pub pair: usize,
    /// The operand the edge belongs to, an index into
    /// [`Interferences::operands`]: one of the pair's two.
    pub operand: usize,
    /// The operand whose face of the pair the edge is placed on: the
    /// pair's other one.
    pub on: usize,
    /// The edge.
    pub edge: EdgeId,
    /// Which piece of the edge between consecutive paves, ascending along
    /// its parameter; `0` is the whole edge when it has none.
    pub index: usize,
    /// The piece's parameter range on the edge's curve.
    pub range: Interval,
    /// The pcurve on the other face, same-parameter with the edge's
    /// curve, in the translate of the domain that face's loops are
    /// written in.
    pub pcurve: Curve2,
    /// The edge's tolerance raised to the pcurve's residual.
    pub tolerance: f64,
}

/// A piece of an edge of `b`'s face that is a piece of an edge of `a`'s
/// face: the same curve within the tolerances, between the same section
/// vertices, on the boundary of both faces of a `Coincident` pair. The
/// result holds it once, as `a`'s piece, and every use of `b`'s piece by
/// a face of `b` is rewritten to `a`'s with a pcurve fitted to `a`'s
/// curve (Open CASCADE's common block, on this representation).
#[derive(Debug, Clone, PartialEq)]
pub struct CommonBlock {
    /// The pair, an index into [`Interferences::pairs`].
    pub pair: usize,
    /// `a`'s edge and the index of its piece.
    pub a: (EdgeId, usize),
    /// `b`'s edge and the index of its piece.
    pub b: (EdgeId, usize),
    /// `true` when `b`'s piece runs against `a`'s parameter.
    pub reversed: bool,
    /// The pcurve of `a`'s piece for every use of `b`'s piece by a face of
    /// `b`, keyed by that use's own pcurve id (a seam's two uses have
    /// two), same-parameter with `a`'s curve and placed in the translate
    /// the use is written in.
    pub pcurves: Vec<(Curve2Id, Curve2)>,
    /// The larger of the two edges' tolerances raised to the pcurves'
    /// residuals.
    pub tolerance: f64,
}

/// The pave model of two bodies (ADR-0004): what [`interferences`]
/// returns. Every list is in a deterministic order — pairs in the
/// operands' face iteration order, hits by `(edge id, t)`, vertices in
/// the order they were first hit — so two runs give the same value and
/// the same `Display`.
#[derive(Debug, Clone, PartialEq)]
pub struct Interferences {
    /// The operands, in the caller's order: [`FacePair::operands`] and
    /// [`EdgeImage::operand`] index into this.
    pub operands: Vec<Body>,
    /// Every face pair of two operands whose boxes overlap and whose
    /// faces' boxes overlap, the lower operand's faces outer, operand
    /// pairs in ascending order.
    pub pairs: Vec<FacePair>,
    /// Every edge-on-face hit: an operand's edges against the faces of
    /// every other, ascending by `(edge id, t)`.
    pub hits: Vec<EdgeFaceHit>,
    /// Every crossing of two section curves of one crossing pair on
    /// both faces, ascending by `(pair, curves, t on the first)`.
    pub section_crossings: Vec<SectionCrossing>,
    /// The section vertices.
    pub vertices: Vec<SectionVertex>,
    /// The paves on every operand edge that has one, ascending by `t`,
    /// one per section vertex — but on a degenerate edge, which has no
    /// curve and is paved in its pcurve's parameter, one for every `u` a
    /// section edge arrives at its vertex with, so a circle through a
    /// pole paves it twice with the one vertex (ADR-0021).
    pub paves: BTreeMap<EdgeId, Vec<Pave>>,
    /// The section curves of every crossing pair, in pair order.
    pub curves: Vec<SectionCurve>,
    /// The section edges, in curve order and then along each curve —
    /// the split order a pair's section edges reach the record in
    /// (ADR-0009, `docs/DATA-MODEL.md` §Provenance).
    pub sections: Vec<SectionEdge>,
    /// The contacts of every pair with a touching curve, in pair order
    /// and then along each curve.
    pub contacts: Vec<Contact>,
    /// Edges of one operand lying in the surface of a face of the other
    /// within their tolerances; no hit is recorded for them. Where the
    /// face is one of a `Coincident` pair the edge's pieces appear below
    /// as images or common blocks.
    pub coincident: Vec<(EdgeId, FaceId)>,
    /// Every edge–edge crossing of a `Coincident` pair, ascending by
    /// `(a's edge, ta, b's edge, tb)`.
    pub crossings: Vec<EdgeEdgeHit>,
    /// The pieces of each `Coincident` pair's edges that lie inside the
    /// other face, in pair order, `a`'s edges first; then the pieces of
    /// every edge lying in a face of the other operand that no face of its
    /// own is coincident with, in the order of `coincident`.
    pub images: Vec<EdgeImage>,
    /// The pieces of `b`'s edges that are pieces of `a`'s, in pair order.
    pub blocks: Vec<CommonBlock>,
}

/// The pave model of `a` and `b`: the decomposition every boolean of the
/// two is a selection over, computed and returned without building
/// anything (a query: `&Model`, no body, no provenance).
///
/// Guarantees (ADR-0004): every face pair whose boxes overlap has been
/// intersected; every edge of each operand has been intersected with
/// every face of the other whose box it reaches, and a hit is kept
/// exactly when its parameter is in the edge's range and its (u, v) is
/// on the face; hits within tolerance of one another are one section
/// vertex, made once, whose tolerance follows the growth rule; two
/// section curves of one crossing pair that cross each other at a
/// point on both faces make a section vertex there too, merged with any
/// hit at the point by the same rule; a section
/// vertex paves every edge that hit it and every section curve it lies
/// on; a section curve's blocks between consecutive paves are kept when
/// their midpoint is inside both faces, and each kept block carries a
/// pcurve on each face that is same-parameter with the curve within the
/// edge's tolerance, translated into the copy of the domain the face's
/// loops are written in. A touching curve is paved by the touches —
/// the hits of either face's edges on the other face lying on it — and
/// every block between consecutive paves whose midpoint is inside both
/// faces, the last wrapping round on a closed curve, is a [`Contact`],
/// with no section edge and no pave on any operand edge. For a `Coincident` pair, the two faces' edges
/// have been intersected with one another and every crossing is a
/// section vertex; every edge of either face is paved by every section
/// vertex on it; and each piece of each edge between consecutive paves
/// is an image on the other face when it lies inside it, with a pcurve
/// there, or a common block when it lies along the other face's
/// boundary, with the piece of the other face's edge it coincides with —
/// a piece along the boundary that matches no piece there is
/// [`crate::Fault::CommonBlock`], a kernel bug. The result is
/// deterministic: the same value on every platform.
///
/// Errors: [`OpError::InvalidInput`] when an operand fails the checker
/// (debug builds, and release with `paranoid`); [`OpError::NotFound`]
/// when one does not resolve; [`OpError::Unsupported`] naming the face
/// pair, or the edge and the face, the intersector has no arm for — a
/// NURBS operand, and nothing else: no guard stands before it;
/// [`OpError::Tolerance`] when a section vertex would
/// need a tolerance above the model's maximum; [`OpError::Internal`]
/// for a geometry query that failed on validated input or a section
/// edge crossing a seam without a pave; [`OpError::Interrupted`] when
/// `control` stops the build, with nothing to undo — the model is only
/// read.
///
/// ```
/// use arris_ops::boolean::interferences;
/// use arris_ops::{primitive_box, primitive_cylinder};
/// use arris_topo::Model;
/// use arris_math::{Axis, Control, Point3};
///
/// let mut m = Model::default();
/// let (plate, _) = primitive_box(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0), &arris_ops::Control::NONE)?;
/// let axis = Axis::z_at(Point3::new(20.0, 15.0, -1.0));
/// let (hole, _) = primitive_cylinder(&mut m, axis, 4.0, 12.0, &arris_ops::Control::NONE)?;
/// let i = interferences(&m, plate, hole, &Control::NONE)?;
/// // The hole's seam pierces the top and the bottom: two section
/// // circles, each paved once, at the seam's hits.
/// assert_eq!(i.hits.len(), 2);
/// assert_eq!(i.sections.len(), 2);
/// assert!(i.curves.iter().all(|c| c.paves.len() == 1));
/// # Ok::<(), arris_ops::OpError>(())
/// ```
pub fn interferences(
    m: &Model,
    a: Body,
    b: Body,
    control: &Control<'_>,
) -> Result<Interferences, OpError> {
    interferences_many(m, &[a, b], control)
}

/// The pave model of any number of operands: [`interferences`]'s
/// decomposition over every pair of them, one build — every face pair of
/// two different operands whose boxes overlap intersected once, every
/// section vertex made once and shared, whichever operands' entities it
/// lies on (ADR-0050 §1). [`FacePair::operands`] and
/// [`EdgeImage::operand`] index into `operands`, in the caller's order,
/// and the pairs are in ascending operand pairs, `(0, 1)`, `(0, 2)`, …,
/// `(1, 2)`, … With two operands the value is [`interferences`]'s.
///
/// Errors: as [`interferences`], for every operand.
///
/// ```
/// use arris_ops::boolean::interferences_many;
/// use arris_ops::primitive_box;
/// use arris_topo::Model;
/// use arris_math::{Control, Point3};
///
/// let mut m = Model::default();
/// let none = arris_ops::Control::NONE;
/// let (a, _) = primitive_box(&mut m, Point3::origin(), Point3::new(4.0, 4.0, 4.0), &none)?;
/// let (b, _) = primitive_box(&mut m, Point3::new(1.0, 1.0, 1.0), Point3::new(5.0, 5.0, 5.0), &none)?;
/// let (c, _) = primitive_box(&mut m, Point3::new(20.0, 0.0, 0.0), Point3::new(24.0, 4.0, 4.0), &none)?;
/// let i = interferences_many(&m, &[a, b, c], &Control::NONE)?;
/// // `c` is far from both: every pair is of `a` and `b`.
/// assert!(i.pairs.iter().all(|p| p.operands == [0, 1]));
/// # Ok::<(), arris_ops::OpError>(())
/// ```
pub fn interferences_many(
    m: &Model,
    operands: &[Body],
    control: &Control<'_>,
) -> Result<Interferences, OpError> {
    for &body in operands {
        crate::verify_input(m, body)?;
    }
    pave::build(m, operands, &mut Meter::new(control))
}

/// `target` minus `tool`: the boolean difference of two solids whose
/// faces lie on planes and cylinders (`docs/ARCHITECTURE.md`
/// §Operations, ADR-0004).
///
/// Guarantees. The result is a `Solid` that passes the checker; the
/// decomposition is [`interferences`]'s, and the selection is the
/// table's: a piece of the target is kept when it is outside the tool, a
/// piece of the tool when it is inside the target, reversed. Every face
/// is split in its own (u, v) through its pcurves and the section
/// edges', each piece classified by `arris_check::classify_point` at a
/// point strictly inside it. Every entity of the target the operation
/// did not touch keeps its id — a face whose loops changed at all, even
/// only by a split edge, is a new face `Modified` from the old, and so
/// is a re-tolerated vertex and everything at it; a split edge is
/// `Modified` into its surviving pieces; whatever has no piece left is
/// `Deleted`. Every entity of the tool is `Deleted`, and a piece of it
/// that survives is `Generated` from the tool entity it is a piece of —
/// the hole's wall from the tool's wall (`docs/DATA-MODEL.md`
/// §Provenance). A section vertex is `Generated` from the edge and the
/// face of every hit it merges, a section edge from both faces of its
/// pair; the result's body is `Modified` from the target's. The result
/// may be several shells — a tool that splits its target, a cavity the
/// tool leaves inside it — each an outer shell or a void, stored lump by
/// lump (ADR-0006): a result shell is `Modified` from the target's shells
/// it holds pieces of, and a cavity made of the tool's pieces alone is
/// `Generated` from the tool's shell.
/// Tolerances follow the growth rule: a section vertex's is the largest
/// of what it merges plus their spread, a section edge's the larger of
/// its faces' raised to the pcurves' residual, a piece keeps its
/// parent's. The result is deterministic: the same ids on every run and
/// every platform.
///
/// A face of the target coincident with a face of the tool is the
/// flush case, a named one: the two faces' edges split each other on
/// their shared surface, and a piece of the target's face lying on the
/// tool's is kept exactly when the two effective normals oppose — the
/// tool touching from outside — and dropped when they agree, while the
/// tool's piece is always dropped; a piece of the tool's edge that is
/// a piece of the target's is one edge of the result, the target's
/// (`docs/ARCHITECTURE.md` §Operations, the selection table). A face
/// of the target tangent to a face of the tool along a curve — a plane
/// and a cylinder, or two parallel cylinders, touching along a ruling, a
/// sphere and a cylinder along a circle — is the other named case: the
/// curve is no section edge and splits nothing, a piece whose interior
/// point lies on it is classified by the curvature rule (it lies inside
/// the other operand exactly when its surface's normal curvature across
/// the curve, signed against the other face's outward
/// normal, is below the other surface's), and a tool touching from
/// outside leaves the target as it was, every id kept.
///
/// Errors, the model untouched on each: [`OpError::InvalidInput`] and
/// [`OpError::NotFound`] as every operation; [`OpError::Unsupported`]
/// naming the pair for a surface pair or an edge–face pair with no
/// closed form, and for a piece lying on an edge or a vertex of the
/// other operand, or on a face of it that its own face is neither
/// coincident nor tangent with; [`OpError::Degenerate`] with
/// [`crate::BooleanReason::Empty`] when nothing survives (the target inside the
/// tool), [`crate::InputReason::ZeroThickness`] when nothing survives and
/// what was dropped lay on the other operand (two solids touching along
/// a face), [`crate::InputReason::NonManifold`] naming the shared edges or
/// vertices when two shells of the result would touch along an edge or at
/// a vertex, [`crate::BooleanReason::TangentContact`] when two faces touch along a
/// curve interior to both and both pieces through it would survive —
/// a hole wall tangent to a side face, a ball in a bore of its radius —
/// or a section edge is tangent to
/// a loop edge at a vertex;
/// [`OpError::Tolerance`] when a section vertex or edge would exceed the
/// model's maximum; [`OpError::Internal`] for a kernel bug the operation
/// caught — an arrangement that is not a subdivision, a point that
/// could not be classified, the builder refusing the assembly;
/// [`OpError::Interrupted`] when `control`'s poll answers `true` or its
/// budget of steps is spent: the model is as it was, ids included. The
/// same operands and budget stop at the same step on every platform and
/// with `parallel` on or off (ADR-0030).
///
/// ```
/// use arris_ops::{cut, primitive_box, primitive_cylinder};
/// use arris_ops::measure::mass_properties;
/// use arris_topo::Model;
/// use arris_math::{Axis, Control, Point3};
/// use core::f64::consts::PI;
///
/// let mut m = Model::default();
/// let (plate, _) = primitive_box(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0), &arris_ops::Control::NONE)?;
/// let axis = Axis::z_at(Point3::new(20.0, 15.0, -1.0));
/// let (hole, _) = primitive_cylinder(&mut m, axis, 4.0, 12.0, &arris_ops::Control::NONE)?;
/// let (plate_with_hole, provenance) = cut(&mut m, plate, hole, &Control::NONE)?;
/// assert_eq!(m.faces(plate_with_hole)?.len(), 7);
/// let volume = mass_properties(&m, plate_with_hole, &arris_ops::Control::NONE)?.volume;
/// assert!((volume - (12000.0 - PI * 16.0 * 10.0)).abs() < 1e-9 * 12000.0);
/// // The four side faces are kept: not a word about them in the record.
/// assert_eq!(provenance.outputs().len(), 10);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn cut(
    m: &mut Model,
    target: Body,
    tool: Body,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    crate::verify_input(m, target)?;
    crate::verify_input(m, tool)?;
    let mut meter = Meter::new(control);
    let i = pave::build(m, &[target, tool], &mut meter)?;
    result::boolean(m, &i, result::Op::Cut, &mut meter)
}

/// `a` ∪ `b`: the boolean union of two solids whose faces lie on planes
/// and cylinders (`docs/ARCHITECTURE.md` §Operations, ADR-0004).
///
/// Guarantees. The result is a `Solid` that passes the checker; the
/// decomposition is [`interferences`]'s, and the selection is the
/// table's: a piece of either operand is kept when it is outside the
/// other, in the operand's own orientation. Every face is split in its
/// own (u, v) through its pcurves and the section edges', each piece
/// classified by `arris_check::classify_point` at a point strictly
/// inside it. Entities are reused from both operands: whatever the
/// operation did not touch keeps its id — a face whose loops changed at
/// all, even only by a split edge, is a new face `Modified` from the
/// old, and so is a re-tolerated vertex and everything at it; a split
/// edge is `Modified` into its surviving pieces; whatever has no piece
/// left is `Deleted`. A section vertex is `Generated` from the edge and
/// the face of every hit it merges, a section edge from both faces of
/// its pair; the result's body is `Modified` from both operands', and
/// each result shell from the operand shells its pieces came from — two
/// operands that do not meet are two lumps of one solid (ADR-0006,
/// `docs/DATA-MODEL.md` §Provenance). Tolerances follow
/// the growth rule, as [`cut`]. Two faces on one surface are the flush
/// case: a piece of `a`'s face lying on `b`'s is kept, once and in `a`'s
/// orientation, exactly when the two effective normals agree, and
/// `Modified` from `a`'s face and `Generated` from `b`'s; when they
/// oppose — the operands touching along the face — both pieces vanish;
/// an edge piece the two operands share is one edge of the result,
/// `a`'s. The result is deterministic: the same ids on every run and
/// every platform.
///
/// Errors, the model untouched on each: as [`cut`]'s.
///
/// ```
/// use arris_ops::{fuse, primitive_box, primitive_cylinder};
/// use arris_ops::measure::mass_properties;
/// use arris_topo::Model;
/// use arris_math::{Axis, Control, Point3};
/// use core::f64::consts::PI;
///
/// let mut m = Model::default();
/// let (plate, _) = primitive_box(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0), &arris_ops::Control::NONE)?;
/// let axis = Axis::z_at(Point3::new(20.0, 15.0, 5.0));
/// let (boss, _) = primitive_cylinder(&mut m, axis, 4.0, 15.0, &arris_ops::Control::NONE)?;
/// let (plate_with_boss, provenance) = fuse(&mut m, plate, boss, &Control::NONE)?;
/// // The boss's wall crosses the top face; its bottom cap is swallowed.
/// assert_eq!(m.faces(plate_with_boss)?.len(), 8);
/// let volume = mass_properties(&m, plate_with_boss, &arris_ops::Control::NONE)?.volume;
/// assert!((volume - (12000.0 + PI * 16.0 * 10.0)).abs() < 1e-9 * 12000.0);
/// // The plate's four sides and bottom and the boss's top cap are kept:
/// // not a word about them in the record.
/// assert_eq!(provenance.outputs().len(), 7);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn fuse(
    m: &mut Model,
    a: Body,
    b: Body,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    crate::verify_input(m, a)?;
    crate::verify_input(m, b)?;
    let mut meter = Meter::new(control);
    let i = pave::build(m, &[a, b], &mut meter)?;
    result::boolean(m, &i, result::Op::Fuse, &mut meter)
}

/// `a` ∩ `b`: the boolean intersection of two solids whose faces lie on
/// planes and cylinders (`docs/ARCHITECTURE.md` §Operations,
/// ADR-0004).
///
/// Guarantees. As [`fuse`], with the other selection: a piece of either
/// operand is kept when it is inside the other, in the operand's own
/// orientation, and a piece of `a`'s face lying on a coincident face of
/// `b` is kept once, from `a`, when the normals agree. Entities are
/// reused from both operands and the provenance is written the same
/// way; the result's body is `Modified` from both operands', and each
/// result shell from the operand shells its pieces came from.
///
/// Errors, the model untouched on each: as [`cut`]'s, with
/// [`crate::BooleanReason::Empty`] where two operands share no material — the
/// common of disjoint solids — and [`crate::InputReason::ZeroThickness`]
/// where they share only a face.
///
/// ```
/// use arris_ops::{common, primitive_box};
/// use arris_ops::measure::mass_properties;
/// use arris_topo::Model;
/// use arris_math::{Control, Point3};
///
/// let mut m = Model::default();
/// let (a, _) = primitive_box(&mut m, Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0), &arris_ops::Control::NONE)?;
/// let (b, _) = primitive_box(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 2.0), &arris_ops::Control::NONE)?;
/// let (unit_cube, _) = common(&mut m, a, b, &Control::NONE)?;
/// assert_eq!(m.faces(unit_cube)?.len(), 6);
/// assert!((mass_properties(&m, unit_cube, &arris_ops::Control::NONE)?.volume - 1.0).abs() < 1e-12);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn common(
    m: &mut Model,
    a: Body,
    b: Body,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    crate::verify_input(m, a)?;
    crate::verify_input(m, b)?;
    let mut meter = Meter::new(control);
    let i = pave::build(m, &[a, b], &mut meter)?;
    result::boolean(m, &i, result::Op::Common, &mut meter)
}

/// A number as the dump writes it: the shortest decimal that round-trips.
fn num(x: f64) -> String {
    format!("{x}")
}

fn point3(p: Point3) -> String {
    format!("({}, {}, {})", num(p.x), num(p.y), num(p.z))
}

fn point2(p: Point2) -> String {
    format!("({}, {})", num(p.x), num(p.y))
}

/// A curve in one line: its kind and the numbers that place it.
fn curve(c: &Curve) -> String {
    match c {
        Curve::Line { origin, direction } => format!(
            "line origin {} direction ({}, {}, {})",
            point3(*origin),
            num(direction.x),
            num(direction.y),
            num(direction.z)
        ),
        Curve::Circle { frame, radius } => format!(
            "circle centre {} radius {}",
            point3(frame.origin()),
            num(*radius)
        ),
        Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => format!(
            "ellipse centre {} major {} minor {}",
            point3(frame.origin()),
            num(*major_radius),
            num(*minor_radius)
        ),
        Curve::Nurbs(n) => format!(
            "nurbs degree {} points {}",
            n.degree(),
            n.control_points().len()
        ),
    }
}

/// A pcurve in one line.
fn curve2(c: &Curve2) -> String {
    match c {
        Curve2::Line { origin, direction } => format!(
            "line origin {} direction ({}, {})",
            point2(*origin),
            num(direction.x),
            num(direction.y)
        ),
        Curve2::Circle { frame, radius } => format!(
            "circle centre {} radius {}",
            point2(frame.origin()),
            num(*radius)
        ),
        Curve2::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => format!(
            "ellipse centre {} major {} minor {}",
            point2(frame.origin()),
            num(*major_radius),
            num(*minor_radius)
        ),
        Curve2::Nurbs(n) => format!(
            "nurbs degree {} points {} u [{}, {}]",
            n.degree(),
            n.control_points().len(),
            num(n
                .control_points()
                .iter()
                .map(|p| p.x)
                .fold(f64::INFINITY, f64::min)),
            num(n
                .control_points()
                .iter()
                .map(|p| p.x)
                .fold(f64::NEG_INFINITY, f64::max))
        ),
    }
}

impl fmt::Display for Interferences {
    /// The whole model, one entity per line: the pairs with their
    /// intersection kind and curves, the hits, the vertices, the paves
    /// per edge, the section curves with their paves, the section edges
    /// with their pcurves, the contacts of the tangent pairs, the
    /// coincident edges, the crossings, images and common blocks of the
    /// coincident pairs. Deterministic, so two runs are compared by their
    /// text.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let names: Vec<String> = self.operands.iter().map(|b| b.to_string()).collect();
        writeln!(f, "interferences {}", names.join(" vs "))?;
        writeln!(f, "pairs {}", self.pairs.len())?;
        for (i, p) in self.pairs.iter().enumerate() {
            match &p.intersection {
                SurfaceIntersection::Empty => writeln!(f, "  p{i} {} x {}: empty", p.a, p.b)?,
                SurfaceIntersection::Coincident => {
                    writeln!(f, "  p{i} {} x {}: coincident", p.a, p.b)?;
                }
                SurfaceIntersection::Meets { curves, points } => {
                    writeln!(f, "  p{i} {} x {}: meets", p.a, p.b)?;
                    let kind = |k: MeetKind| match k {
                        MeetKind::Crossing => "crossing",
                        MeetKind::Touch => "touch",
                    };
                    for c in curves {
                        writeln!(f, "    {} {}", kind(c.kind), curve(&c.curve))?;
                    }
                    for q in points {
                        writeln!(f, "    {} point {}", kind(q.kind), point3(q.point))?;
                    }
                }
            }
        }
        writeln!(f, "hits {}", self.hits.len())?;
        for (i, h) in self.hits.iter().enumerate() {
            let landing = match h.landing {
                Landing::Interior => "interior".to_string(),
                Landing::Boundary(s) => format!("boundary {s}"),
            };
            let vertex = match h.vertex {
                Some(v) => format!("v{v}"),
                None => "-".to_string(),
            };
            writeln!(
                f,
                "  h{i} {} t {} on {} uv {} at {} {}{}{} -> {vertex}",
                h.edge,
                num(h.t),
                h.face,
                point2(h.uv),
                point3(h.point),
                landing,
                if h.tangent { " tangent" } else { "" },
                match h.at_vertex {
                    Some(v) => format!(" at {v}"),
                    None => String::new(),
                }
            )?;
        }
        writeln!(f, "section crossings {}", self.section_crossings.len())?;
        for (i, x) in self.section_crossings.iter().enumerate() {
            let vertex = match x.vertex {
                Some(v) => format!("v{v}"),
                None => "-".to_string(),
            };
            writeln!(
                f,
                "  k{i} p{} curve {} t {} x curve {} t {} at {}{} -> {vertex}",
                x.pair,
                x.curves[0],
                num(x.t[0]),
                x.curves[1],
                num(x.t[1]),
                point3(x.point),
                if x.tangent { " tangent" } else { "" }
            )?;
        }
        writeln!(f, "vertices {}", self.vertices.len())?;
        for (i, v) in self.vertices.iter().enumerate() {
            let hits: Vec<String> = v.hits.iter().map(|h| format!("h{h}")).collect();
            let existing: Vec<String> = v.existing.iter().map(|e| e.to_string()).collect();
            let source = match v.source {
                VertexSource::Hits => String::new(),
                VertexSource::SectionCrossing => {
                    let list: Vec<String> = v
                        .section_crossings
                        .iter()
                        .map(|k| format!("k{k}"))
                        .collect();
                    format!(" section crossing [{}]", list.join(" "))
                }
                VertexSource::CurveStart { pair, curve } => {
                    format!(" start of curve {curve} of p{pair}")
                }
                VertexSource::Singular => " singular".to_string(),
            };
            writeln!(
                f,
                "  v{i} {} tol {} hits [{}] existing [{}]{source}",
                point3(v.point),
                num(v.tolerance),
                hits.join(" "),
                existing.join(" ")
            )?;
        }
        writeln!(f, "paves")?;
        for (edge, paves) in &self.paves {
            let list: Vec<String> = paves
                .iter()
                .map(|p| format!("t {} v{}", num(p.t), p.vertex))
                .collect();
            writeln!(f, "  {edge}: {}", list.join(", "))?;
        }
        writeln!(f, "section curves {}", self.curves.len())?;
        for (i, c) in self.curves.iter().enumerate() {
            let paves: Vec<String> = c
                .paves
                .iter()
                .map(|p| format!("t {} v{}", num(p.t), p.vertex))
                .collect();
            let edges: Vec<String> = c.edges.iter().map(|e| format!("s{e}")).collect();
            writeln!(
                f,
                "  c{i} p{} {}: paves [{}] edges [{}]",
                c.pair,
                curve(&c.curve),
                paves.join(", "),
                edges.join(" ")
            )?;
        }
        writeln!(f, "section edges {}", self.sections.len())?;
        for (i, s) in self.sections.iter().enumerate() {
            writeln!(
                f,
                "  s{i} c{} [{}, {}] v{} -> v{} tol {}",
                s.curve,
                num(s.range.lo()),
                num(s.range.hi()),
                s.start,
                s.end,
                num(s.tolerance)
            )?;
            writeln!(f, "    on a: {}", curve2(&s.pcurves[0]))?;
            writeln!(f, "    on b: {}", curve2(&s.pcurves[1]))?;
        }
        writeln!(f, "contacts {}", self.contacts.len())?;
        for (i, c) in self.contacts.iter().enumerate() {
            writeln!(
                f,
                "  t{i} p{} curve {} [{}, {}] at {} uv {} / {}",
                c.pair,
                c.curve,
                num(c.range.lo()),
                num(c.range.hi()),
                point3(c.point),
                point2(c.uv[0]),
                point2(c.uv[1])
            )?;
        }
        writeln!(f, "coincident {}", self.coincident.len())?;
        for (e, face) in &self.coincident {
            writeln!(f, "  {e} in {face}")?;
        }
        writeln!(f, "crossings {}", self.crossings.len())?;
        for (i, x) in self.crossings.iter().enumerate() {
            let vertex = match x.vertex {
                Some(v) => format!("v{v}"),
                None => "-".to_string(),
            };
            writeln!(
                f,
                "  x{i} p{} {} t {} x {} t {} at {}{} -> {vertex}",
                x.pair,
                x.a,
                num(x.ta),
                x.b,
                num(x.tb),
                point3(x.point),
                if x.tangent { " tangent" } else { "" }
            )?;
        }
        writeln!(f, "images {}", self.images.len())?;
        for (i, im) in self.images.iter().enumerate() {
            let pair = &self.pairs[im.pair];
            let on = if im.on == pair.operands[1] {
                pair.b
            } else {
                pair.a
            };
            writeln!(
                f,
                "  i{i} p{} {}#{} [{}, {}] on {} tol {}: {}",
                im.pair,
                im.edge,
                im.index,
                num(im.range.lo()),
                num(im.range.hi()),
                on,
                num(im.tolerance),
                curve2(&im.pcurve)
            )?;
        }
        writeln!(f, "common blocks {}", self.blocks.len())?;
        for (i, b) in self.blocks.iter().enumerate() {
            writeln!(
                f,
                "  b{i} p{} {}#{} = {}#{}{} tol {}",
                b.pair,
                b.b.0,
                b.b.1,
                b.a.0,
                b.a.1,
                if b.reversed { " reversed" } else { "" },
                num(b.tolerance)
            )?;
            for (id, pc) in &b.pcurves {
                writeln!(f, "    for {id}: {}", curve2(pc))?;
            }
        }
        Ok(())
    }
}
