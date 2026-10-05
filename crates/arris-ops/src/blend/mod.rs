//! Blends: `fillet` rolls a ball of constant radius along named edges of
//! a solid, `chamfer` cuts them flat at a distance (ADR-0007). Each blend
//! is built directly from its edge's two faces in closed form — two
//! planes blend to a cylinder on the line where their offset planes meet,
//! and chamfer to the plane through the lines at the distance from the
//! edge on each; a plane and a cylinder along a ruling blend to a cylinder
//! on the line where the plane's offset meets the cylinder's, and a plane
//! and a cylinder or a cone along a coaxial circle to a torus found in the
//! half-plane through the axis or chamfer to a cone — with its
//! contact curves read off the construction, each end of an open edge
//! trimmed by the face across the corner or met by the blends sharing its
//! vertex — two in a miter, three in a sphere or a triangle, or the next
//! blend of a chain at a tangent vertex on the ball's great circle — and the
//! result assembled through `rebuild::rewrite` with every untouched
//! entity kept by id (`docs/ARCHITECTURE.md` §Operations,
//! `docs/DATA-MODEL.md` §Provenance).

use std::collections::BTreeSet;

use arris_math::{Control, Meter};
use arris_topo::{Body, Edge, EdgeId, Model, Provenance, Shape};

use self::build::build;
use self::ends::EndKind;
use self::stripe::{Contact, Stripe};
use crate::error::{Fault, OpError, Reason};
use crate::rebuild::forward;

mod build;
mod chain;
mod corner;
mod ends;
mod junction;
mod miter;
mod mixed;
mod ring;
mod stripe;
mod traced;
mod view;

fn degenerate(entities: Vec<Shape>, reason: Reason) -> OpError {
    OpError::Degenerate { entities, reason }
}

fn invariant(what: &'static str) -> OpError {
    OpError::Internal(Fault::Invariant { what })
}

/// What a blend is: a rolling ball's fillet or an equal-distance chamfer.
#[derive(Debug, Clone, Copy)]
enum Kind {
    Fillet { radius: f64 },
    Chamfer { distance: f64 },
}

impl Kind {
    /// The fillet's radius or the chamfer's distance: how far from the
    /// edge the blend reaches.
    fn size(self) -> f64 {
        match self {
            Kind::Fillet { radius } => radius,
            Kind::Chamfer { distance } => distance,
        }
    }
}

/// One edge's blend, decided and checked, before anything is written.
struct Blend {
    stripe: Stripe,
    /// The contacts, at `u = 0` then at `u = β`.
    contacts: [Contact; 2],
    /// At the edge's `range.lo()` end, then its `range.hi()` end.
    ends: [EndKind; 2],
}

/// Blends `edges` of `body` with a rolling ball of `radius`: each edge's
/// two faces are replaced by the same faces cut back to the ball's
/// contact curves, the edge by a blend face tangent to both along them,
/// and each end of the blend is trimmed by the face across the corner,
/// whose vertex goes and whose two other edges are shortened to the
/// trim's arc (ADR-0007). Two planes blend to a cylinder of `radius` on
/// the line where their offset planes meet — its frame's `X` at one
/// contact ruling and `Z` along the edge, so the contacts sit at `u = 0`
/// and `u = π − φ` for the dihedral's normals `φ` apart and `v` is the
/// edge's own parameter — with the contact lines at distance
/// `r tan(φ/2)` from the edge on each face. A plane and a cylinder of
/// radius `R` along a ruling blend to a cylinder of `radius` on the line
/// where the plane's offset by `radius` meets the cylinder coaxial with
/// the face at `R − radius` or `R + radius` — the ball inside the face's
/// cylinder or outside it — on the edge's side of the axis, the contact
/// on the plane a line and on the cylinder the ruling toward the ball's
/// centre, the contacts at `u = 0` and at the angle between them. The arc
/// across an end is a
/// circle when that face is perpendicular to the edge and an ellipse
/// otherwise, exact on the plane and a `Line` or a fitted `Nurbs` on the
/// cylinder; on a cylinder or a cone across — a rib running into a boss —
/// the quartic of the two, traced and fitted between where each contact
/// pierces that face (ADR-0037), the one fitted curve on an exact surface.
/// At a face across whose two corner edges differ in convexity — a blend
/// running into a step, on a stripe or a ring's open arc — the corner edge of the blend's own convexity is
/// lengthened past the vertex along its curve to the trim, and the face
/// across takes the region the arc bounds (ADR-0038). Where the vertex has
/// more edges, every one sharp and of the blend's sense, and the faces
/// across from one corner edge to the other are a fan — the hexagonal
/// boss's chamfer facet at its foot — the end is one section per face, as
/// a lone face across would cut it, and each edge between two of them is
/// cut where the blend's surface crosses it (ADR-0043).
/// Two blends meeting at a vertex whose third edge stays sharp
/// meet in a miter: the ellipse of the two cylinders in the plane
/// through the ball's centre bisecting their axes, from where the two
/// contacts on the shared face cross to where the other two meet the
/// third edge, a fitted `Nurbs` pcurve on each cylinder; the third edge
/// is shortened to that point and no face takes an arc. Where their
/// dihedrals differ (ADR-0044) the far contacts meet the third edge at two
/// points: the ellipse stops at `m` on the narrower blend's far contact,
/// and the wider blend — the one reaching farther along the third edge —
/// is trimmed on from `m` by the narrower's far face, in a circle or an
/// ellipse that face takes; the third edge is shortened to its end. Three blends at a
/// vertex of three planes, all convex or all concave, meet in the sphere of
/// `radius` about the ball's one centre, tangent to each cylinder along the
/// great circle through the centre square to its axis, between the points
/// where each face's two contacts cross: its frame's `Z` toward the point
/// of a face square to the other two, so its sides are its equator and two
/// meridians, exact lines in (u, v), meeting at its pole, a degenerate
/// edge; no corner edge is cut. A plane, a
/// cylinder or a cone against another of them, at most one a plane, along
/// a circle coaxial with both — a closed edge, a hole's rim, a boss's
/// base, a frustum's rim, a turned shoulder — blend with no ends to a
/// torus coaxial with the curved faces, its centre circle where the
/// first face's offset meets the second's in the half-plane through the
/// axis (ADR-0036) and its minor radius `radius`, each contact the foot of
/// that circle on its face, a parallel of it, the torus's `u` seam a tube
/// circle from one contact's vertex to the other's in the half-plane of
/// each curved face's seam, which is shortened to its contact. An open arc
/// of such a
/// circle blends to the same torus over the arc's own range, each end
/// trimmed by the face across: a plane through the axis on
/// the tube circle at the vertex's angle (ADR-0035); a plane parallel to
/// the axis or a cylinder about a parallel axis, off it, on the section of
/// the torus with that face, traced and fitted between where each contact
/// meets it, the contacts then trimmed at their own angles (ADR-0037) —
/// the one fitted curve on an exact surface. The selection
/// follows chains: at a tangent vertex — three edges, the third's two
/// faces tangent there, the next edge sharing one face with this one,
/// of its sense and running on — the blend runs on into the next edge,
/// and on until a vertex that is not one, each edge with its own blend
/// face; two blends meet there on the ball's great circle square to the
/// edges' direction, an exact line in (u, v) on each, from where their
/// contacts on the shared face meet to where the other two meet on the
/// third edge, which is shortened to it. It also runs on through a vertex
/// of four edges where both of the edge's faces turn — the next edge
/// sharing no face with it, each other edge tangent there between a face
/// of each — and the great circle there runs between the two points where
/// the contacts meet on those two tangent edges, both shortened. And it
/// runs on where nothing turns: the next edge sharing both the edge's
/// faces, the vertex carrying no other edge or only a seam of one of them,
/// or a seam of each — the vertices of a rim split in arcs — the great circle between the
/// point where the contacts meet on one face and where they meet on the
/// other, the seam shortened to its point and nothing cut at a vertex of
/// two edges. An end at a cusp — a vertex of three edges where the next
/// edge leaves the way this one does, the walls tangent along the third,
/// both edges convex or both concave — is cut by the next wall, from where
/// the contact on the shared face crosses the next edge to where the other
/// contact meets the third edge, both shortened there, on the section
/// traced and fitted as an end on a curved face across is (ADR-0042). Two
/// faces are tangent for the blend where their normals agree to the
/// angular precision, or where the radius times the sine between them is
/// within the faces' tolerance: a ball touching one then touches the
/// other, as across a tangency a file wrote to a few 1e-10. Convex or
/// concave is read from the dihedral. Every surface is exact, every
/// untouched entity keeps its id, and the result's ids are the same for
/// any order of the same edges (the blends are built in the body's
/// iteration order of them).
///
/// Provenance, every record against the blended edge: the blend face, its
/// two contact edges, its two end arcs and the four trim vertices
/// `Generated` from it; each of the edge's faces, each face across an
/// end and each corner edge the trim shortens or lengthens `Modified` into its new
/// self; the edge and the two corner vertices `Deleted`; the shell and
/// the body `Modified`. At a fan, each section and each crossing vertex
/// is `Generated` from the edge too, and each edge crossed `Modified`. A miter's edge and two vertices are `Generated`
/// from both edges they join; a trim arc and its end on the third edge from
/// the wider blend's edge alone. At a corner of three, each side of the
/// sphere is its blend's end arc and each corner point is `Generated` from
/// the two edges whose contacts cross there; the sphere face and its pole
/// from all three. A closed edge's torus face, its two contact
/// circles, its seam and the seam's two vertices are `Generated` from it,
/// the curved face's seam `Modified` and the edge's vertex `Deleted`; an open
/// arc's torus face, its two contacts, its two end sections and their four
/// vertices are `Generated` from it, its faces across and corner edges
/// `Modified`, the arc and its two vertices `Deleted`. An edge the chain
/// reached is recorded as a named one is; the great circle where two
/// blends of a chain meet and its two vertices are `Generated` from both
/// edges, as a miter's are, the third edge — or, at a vertex of four, both
/// tangent edges, at the seam's vertex of a split rim its seam (both seams
/// where both faces turn), and at a vertex of two edges none — `Modified` and the vertex `Deleted`.
/// `arris_topo::provenance::audit` holds on every result
/// (`docs/DATA-MODEL.md` §Provenance).
///
/// Errors, the model untouched: [`OpError::Degenerate`] with
/// [`Reason::NonFinite`] or [`Reason::NotPositive`] on the radius,
/// [`Reason::NoEdges`] for an empty list, [`Reason::RepeatedEdge`] for
/// an edge listed twice, [`Reason::EdgeNotInBody`] for one that is not
/// the body's, [`Reason::TangentChain`] where the edge's faces meet at a
/// tangent dihedral or an end's corner edge has tangent faces at a vertex
/// the chain does not run on through and that is no cusp it is cut at —
/// the next edge turning back, itself a tangent dihedral, of the other
/// sense, or blended too at a cusp —
/// [`Reason::VertexBlend`] at a corner the closed
/// forms do not cover — a vertex of other than three edges that the chain
/// does not run on through and that is no fan — the faces across not one
/// walk through its star, an edge at it smooth, a seam or of the other
/// sense, or a fan at a miter, a corner or a ring's end — a miter
/// whose two blends are not both convex or both concave, or whose
/// dihedrals differ at a third edge of the other sense, a miter with a blend along a ruling, whose contact on
/// the cylinder misses the other's on the third edge, or a corner of three
/// blended edges whose faces are not all planes, whose blends are mixed or
/// none of whose faces is square to the other two — and
/// [`Reason::BlendTooLarge`] where a contact line or an end arc leaves
/// its face through an edge that is not the corner's own or a corner
/// edge is shorter than the trim, a fan's crossing lies past its edge, a trim past the corner's vertex that is
/// no such lengthening or whose stretch leaves its face, or a closed edge's torus would not be a
/// ring torus, a contact reaches the axis or a cone's apex, or its seam is
/// shorter than the trim, or the third edge or a tangent edge at
/// a chain's junction is shorter than the cut; a closed edge whose vertex
/// carries more than the curved faces' seams, or an open arc that meets
/// another blended edge at a vertex that is no tangent vertex, is
/// [`Reason::VertexBlend`];
/// [`OpError::Unsupported`] naming the two faces for a pair outside the
/// table (every pair but two planes, a plane and a cylinder along a
/// ruling, and, along a circle coaxial with both, a plane, a cylinder or a
/// cone against another of them, today), an edge the chain reached included, and
/// naming the face
/// across an end that is none of a plane, a cylinder and a cone, or one of
/// the last two that a contact misses or grazes or the tracer does not
/// decide, or, at an open arc's end, that is
/// none of a plane through the axis, a plane parallel to it and a cylinder
/// about a parallel axis — or one of those two the tracer does not decide,
/// a chamfer's cone on a plane off the axis among them;
/// [`OpError::NotFound`] for an edge id that does not resolve.
///
/// ```
/// use arris_ops::{fillet, primitive_box};
/// use arris_topo::Model;
/// use arris_math::Point3;
///
/// let mut m = Model::default();
/// let (cube, _) = primitive_box(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 2.0), &arris_ops::Control::NONE).unwrap();
/// // The vertical edge at x = 2, y = 2.
/// let edge = m.edges(cube).unwrap().into_iter().find(|e| {
///     let entity = m.edge(e.id).unwrap();
///     let (curve, range) = entity.curve().unwrap();
///     let mid = m.curve(curve).unwrap().point(range.midpoint());
///     (mid - Point3::new(2.0, 2.0, 1.0)).norm() < 1e-9
/// }).unwrap();
/// let (blended, provenance) = fillet(&mut m, cube, &[edge], 0.2, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.faces(blended).unwrap().len(), 7, "six faces and the blend");
/// let generated = provenance.generated_from(edge.shape());
/// assert_eq!(generated.len(), 9, "the blend face, four edges and four vertices");
/// ```
pub fn fillet(
    m: &mut Model,
    body: Body,
    edges: &[Edge],
    radius: f64,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    blend(m, body, edges, Kind::Fillet { radius }, control)
}

/// Cuts `edges` of `body` flat at `distance`: each edge's two faces are
/// replaced by the same faces cut back to the lines at `distance` from
/// the edge along each, the edge by the plane through those two lines,
/// and each end of the chamfer is trimmed by the face across the corner
/// as a [`fillet`]'s is — its vertex goes and its two other edges are
/// shortened to the segment across it (ADR-0007). The plane's frame has
/// its origin on one contact line, `X` across to the other and `Y` along
/// the edge, so the contacts sit at `u = 0` and at the chamfer's width
/// and `v` is the edge's own parameter; every curve is a line, exact on
/// every plane it lies on. Two chamfers meeting at a vertex whose third
/// edge stays sharp meet in the line from where their two contacts on
/// the shared face cross to where the other two meet the third edge,
/// which is shortened to that point; those two meet it at one point
/// exactly when the two edges make equal angles with it (a box corner,
/// any right prism). Where the angles differ (ADR-0044) the line stops at
/// `m` on the narrower chamfer's far contact, and the wider chamfer, the one
/// reaching farther along the third edge, runs on in a chord in the
/// narrower's far face, which takes it; the third edge is shortened to the
/// chord's end. Three chamfers at a vertex of three planes, all convex
/// or all concave, meet in the triangle of the points where each face's
/// two contacts cross, each side a segment in one chamfer's plane, at any
/// such corner; the triangle and every corner point are recorded as a
/// fillet's sphere and points are. A plane and a cylinder or a cone along
/// a coaxial circle chamfer with no ends to the cone coaxial with it
/// through the two circles at `distance` from the edge along each face —
/// 45° against a cylinder — its `Z` toward the wider, its `u` seam the
/// ruling between the contacts' vertices, the curved face's seam
/// shortened as a fillet's is; an open arc chamfers to the same cone over
/// its range, each end on the cone's ruling in a plane through the axis,
/// or on a cylinder about a parallel axis off it on the cone's traced and
/// fitted section with it, as a [`fillet`]'s torus ends there.
/// Convex or concave is read from the dihedral: a
/// concave chamfer adds a prism. The result's ids are the same for any
/// order of the same edges.
///
/// Provenance, every record against the chamfered edge, as a
/// [`fillet`]'s: the chamfer face, its two contact edges, its two end
/// segments and the four trim vertices `Generated`; the edge's faces,
/// the faces across its ends and the corner edges the trim shortens
/// `Modified`; the edge and its corner vertices `Deleted`. The line where
/// two chamfers meet and its two vertices are `Generated` from both
/// edges; a trim chord and its end on the third edge from the wider
/// chamfer's edge alone. `arris_topo::provenance::audit` holds on every result.
///
/// Errors, the model untouched: a [`fillet`]'s, with
/// [`Reason::NonFinite`] or [`Reason::NotPositive`] naming the distance,
/// [`Reason::VertexBlend`] for two chamfers at a corner of mixed
/// convexity, and [`OpError::Unsupported`]
/// for a plane and a cylinder along a ruling, which fillets but has no
/// chamfer in the table.
///
/// ```
/// use arris_ops::measure::mass_properties;
/// use arris_ops::{chamfer, primitive_box};
/// use arris_topo::Model;
/// use arris_math::Point3;
///
/// let mut m = Model::default();
/// let (cube, _) = primitive_box(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 2.0), &arris_ops::Control::NONE).unwrap();
/// // The vertical edge at x = 2, y = 2.
/// let edge = m.edges(cube).unwrap().into_iter().find(|e| {
///     let entity = m.edge(e.id).unwrap();
///     let (curve, range) = entity.curve().unwrap();
///     let mid = m.curve(curve).unwrap().point(range.midpoint());
///     (mid - Point3::new(2.0, 2.0, 1.0)).norm() < 1e-9
/// }).unwrap();
/// let (chamfered, provenance) = chamfer(&mut m, cube, &[edge], 0.2, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.faces(chamfered).unwrap().len(), 7, "six faces and the chamfer");
/// // A right prism cut away, its triangle's legs 0.2, two long.
/// let volume = mass_properties(&m, chamfered, &arris_ops::Control::NONE).unwrap().volume;
/// assert!((volume - 7.96).abs() < 1e-9);
/// assert_eq!(provenance.generated_from(edge.shape()).len(), 9);
/// ```
pub fn chamfer(
    m: &mut Model,
    body: Body,
    edges: &[Edge],
    distance: f64,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    blend(m, body, edges, Kind::Chamfer { distance }, control)
}

/// What [`fillet`] and [`chamfer`] share: the input, the size and the
/// edge list checked before anything is built, then the build over the
/// edges in the body's order inside a transaction.
fn blend(
    m: &mut Model,
    body: Body,
    edges: &[Edge],
    kind: Kind,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    crate::verify_input(m, body)?;
    let b = body.shape();
    let (what, size) = match kind {
        Kind::Fillet { radius } => ("radius", radius),
        Kind::Chamfer { distance } => ("distance", distance),
    };
    if !size.is_finite() {
        return Err(degenerate(vec![b], Reason::NonFinite { what }));
    }
    if size <= 0.0 {
        return Err(degenerate(
            vec![b],
            Reason::NotPositive { what, value: size },
        ));
    }
    if edges.is_empty() {
        return Err(degenerate(vec![b], Reason::NoEdges));
    }
    let closure = m.closure(body)?;
    let mut selected: BTreeSet<EdgeId> = BTreeSet::new();
    for edge in edges {
        m.edge(edge.id)?;
        if !selected.insert(edge.id) {
            return Err(degenerate(vec![forward(edge.id)], Reason::RepeatedEdge));
        }
        if closure.edges.binary_search(&edge.id).is_err() {
            return Err(degenerate(vec![forward(edge.id), b], Reason::EdgeNotInBody));
        }
    }
    let ordered: Vec<EdgeId> = m
        .edges(body)?
        .into_iter()
        .map(|e| e.id)
        .filter(|id| selected.contains(id))
        .collect();
    let mut meter = Meter::new(control);
    m.transaction(|m| build(m, body, &ordered, kind, &mut meter))
}
