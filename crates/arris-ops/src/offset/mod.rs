//! Offset faces (ADR-0048): the chosen faces of a solid moved along their
//! outward normals by a signed distance, the rest extended or trimmed to
//! meet them, topology kept. By phase over `body_view`: the chain (the
//! chosen faces closed over tangent edges, each tangent edge carried along
//! the faces' shared normal — `chain`), the moves (each moved face's offset
//! surface), the vertices (each the point nearest its old one on the
//! surfaces around it — `vertices`, `meet`), the edges (each on the section
//! of its two new surfaces nearest the old edge, a seam on its plane's, a
//! tangent edge its carried curve), the faces (their loops re-used over the
//! new edges), then `rebuild::rewrite` and the checker at `Level::Full` in
//! every profile.

use std::collections::{BTreeMap, BTreeSet};

use arris_check::{Level, Violation};
use arris_geom::Surface;
use arris_math::{Control, Meter, Tolerance};
use arris_topo::{Body, Face, FaceId, Model, Provenance};

use crate::body_view::BodyView;
use crate::error::{Fault, InputReason, OffsetReason, OpError, Reason};
use crate::rebuild::{self, forward};
use chain::Chain;

mod chain;
mod edges;
mod faces;
mod meet;
mod vertices;

/// The moved faces' new surfaces, by face, and the distance they moved.
pub(crate) struct Moves {
    pub(crate) surfaces: BTreeMap<FaceId, Surface>,
    /// The moved faces no one chose, dragged by a tangent neighbour.
    pub(crate) dragged: BTreeSet<FaceId>,
    pub(crate) distance: f64,
}

impl Moves {
    /// The surface `face` lies on once the offset is made: its offset when
    /// it moves, its own otherwise.
    pub(crate) fn surface_of(&self, m: &Model, face: FaceId) -> Result<Surface, OpError> {
        match self.surfaces.get(&face) {
            Some(s) => Ok(s.clone()),
            None => Ok(m.surface(m.face(face)?.surface())?.clone()),
        }
    }

    /// Whether one of `a` and `b` is dragged and the other stays: an edge
    /// between them that the offset cannot hold is a [`OffsetReason::Gap`].
    pub(crate) fn gap_between(&self, a: FaceId, b: FaceId) -> bool {
        let stays = |f: &FaceId| !self.surfaces.contains_key(f);
        (self.dragged.contains(&a) && stays(&b)) || (self.dragged.contains(&b) && stays(&a))
    }
}

/// Moves `faces` of the solid `body` along their outward normals by
/// `distance` — positive adds material, a pushed top face makes the body
/// taller and a pushed hole wall narrows the hole — and returns the
/// result with its provenance (ADR-0048). Each moved face lies on the
/// offset of its own surface; every face beside it is extended or
/// trimmed to meet it, on its own surface: an edge between two moved
/// faces is where their offsets meet, an edge between a moved and a
/// fixed face where the offset meets the fixed face, an edge between two
/// fixed faces at a moved vertex its own curve re-ranged, and every
/// vertex of a moved face the meeting of its faces. Moving every face
/// offsets the whole body, and at a convex edge of an outward offset the
/// join is sharp: the offsets meet. Topology is kept — every vertex,
/// edge and face of the body is in the result, its ids new where it
/// moved and kept where it did not. A plane's offset is a plane, a
/// cylinder's a coaxial cylinder, a cone's a coaxial cone, a sphere's a
/// concentric sphere and a torus's a torus of the same major radius; the
/// edge between two offsets is the intersector's section on the branch
/// nearest the old edge, and a seam is carried as the section with the
/// plane it lies in. A face tangent to a moved one moves with it, and so
/// on along the chain — a fillet and the face beyond it are dragged — and
/// the edge between two tangent faces is carried along their shared
/// normal: a line shifted, a circle re-radiused and moved along its axis.
/// The result passes the checker at `Level::Full` in every build profile.
///
/// Provenance: each moved face, each face whose loop changed, each
/// recomputed edge and each moved vertex `Modified` into its new self;
/// nothing generated or deleted; the shell and the body `Modified`.
/// `arris_topo::provenance::audit` holds on every result.
///
/// Errors, the model untouched: [`OpError::Degenerate`] with
/// [`InputReason::NonFinite`] on a non-finite distance and
/// [`InputReason::NotPositive`] on `|distance|` for zero,
/// [`OffsetReason::NoFaces`] for an empty list,
/// [`OffsetReason::RepeatedFace`] for a face listed twice,
/// [`OffsetReason::FaceNotInBody`] for one that is not the body's,
/// [`OffsetReason::Vanishes`] naming an edge whose ends meet or cross, or
/// that no branch of its new surfaces' section passes through, or a face
/// whose loop turns inside out, once moved,
/// [`OffsetReason::VertexSplits`] naming a vertex whose faces no longer
/// meet in one point — a pyramid's apex with one side pushed,
/// [`OffsetReason::NoExactOffset`] naming a moved face on an elliptic
/// cylinder or a free-form surface, or a tangent edge that is no line or
/// circle, or along which the faces' normal turns,
/// [`OffsetReason::SurfaceCollapses`] naming a moved face whose radius the
/// move drives through zero or whose cone it carries to the axis — a
/// dragged fillet moved in past its radius — and [`OffsetReason::Gap`]
/// naming the edge where a dragged face no longer meets a face beside it
/// that stays, [`OffsetReason::SelfIntersects`] naming the faces the
/// checker's global level finds running into each other — a pocket floor
/// pulled through the bottom of the block; [`OpError::Unsupported`] naming a face on an elliptic
/// cylinder or a free-form surface beside a moved one, two faces whose new
/// surfaces only touch, and a pole or apex the move cannot place;
/// [`OpError::NotFound`] for a face id that does not resolve;
/// [`OpError::Internal`] with the report where the result fails the
/// checker.
///
/// ```
/// use arris_ops::measure::mass_properties;
/// use arris_ops::{Control, offset_faces, primitive_box};
/// use arris_math::Point3;
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let (cube, _) = primitive_box(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 2.0), &Control::NONE).unwrap();
/// // The top face, at z = 2.
/// let top = m.faces(cube).unwrap().into_iter().find(|f| {
///     arris_check::classify::classify_point(&m, cube, Point3::new(1.0, 1.0, 2.0)).unwrap()
///         == arris_check::classify::Classification::On(f.shape())
/// }).unwrap();
/// let (taller, provenance) = offset_faces(&mut m, cube, &[top], 0.5, &Control::NONE).unwrap();
/// let volume = mass_properties(&m, taller, &Control::NONE).unwrap().volume;
/// assert!((volume - 10.0).abs() < 1e-9, "2 × 2 × 2.5");
/// assert_eq!(m.faces(taller).unwrap().len(), 6);
/// assert!(provenance.generated_from(top.shape()).is_empty());
/// ```
pub fn offset_faces(
    m: &mut Model,
    body: Body,
    faces: &[Face],
    distance: f64,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    crate::verify_input(m, body)?;
    let b = body.shape();
    let refuse = |entities, reason| OpError::Degenerate { entities, reason };
    if !distance.is_finite() {
        return Err(refuse(
            vec![b],
            Reason::Input(InputReason::NonFinite { what: "distance" }),
        ));
    }
    if distance == 0.0 {
        return Err(refuse(
            vec![b],
            Reason::Input(InputReason::NotPositive {
                what: "|distance|",
                value: distance.abs(),
            }),
        ));
    }
    if faces.is_empty() {
        return Err(refuse(vec![b], Reason::Offset(OffsetReason::NoFaces)));
    }
    let closure = m.closure(body)?;
    let mut selected: BTreeSet<FaceId> = BTreeSet::new();
    for face in faces {
        m.face(face.id)?;
        if !selected.insert(face.id) {
            return Err(refuse(
                vec![forward(face.id)],
                Reason::Offset(OffsetReason::RepeatedFace),
            ));
        }
        if closure.faces.binary_search(&face.id).is_err() {
            return Err(refuse(
                vec![forward(face.id), b],
                Reason::Offset(OffsetReason::FaceNotInBody),
            ));
        }
    }
    let mut meter = Meter::new(control);
    m.transaction(|m| build(m, body, &selected, distance, &mut meter))
}

/// The offset's pieces before the body is rebuilt: the rewrite that moves
/// the faces — new vertices with their parents, new edges with theirs,
/// each touched face's new loops and each moved face's new surface — and
/// the faces that move, the chosen ones closed over tangent edges.
/// `offset_faces` rewrites the body with it in place; `shell` keeps the
/// body and assembles the moved faces beside it as a second skin.
pub(crate) struct Offset {
    pub(crate) rewrite: rebuild::Rewrite,
    pub(crate) moved: BTreeSet<FaceId>,
}

/// The offset of `chosen` by `distance`, built and checked.
fn build(
    m: &mut Model,
    body: Body,
    chosen: &BTreeSet<FaceId>,
    distance: f64,
    meter: &mut Meter<'_>,
) -> Result<(Body, Provenance), OpError> {
    let offset = pieces(m, body, chosen, distance, meter)?;
    let out = rebuild::rewrite_unverified(m, body, offset.rewrite)?;
    checked_full(m, out.body)?;
    Ok((out.body, out.provenance))
}

/// The phases of the offset of `chosen` by `distance` up to the rewrite:
/// the chain, the moves, the vertices, the edges and the faces' new loops.
pub(crate) fn pieces(
    m: &mut Model,
    body: Body,
    chosen: &BTreeSet<FaceId>,
    distance: f64,
    meter: &mut Meter<'_>,
) -> Result<Offset, OpError> {
    let tol = m.precision().tolerance();
    let view = BodyView::of(m, body)?;
    let chain = chain::chain(m, &view, chosen, distance, tol)?;
    let moves = moves(m, &view, &chain, distance, tol)?;
    let carried = chain::carried(m, &view, &chain, distance, tol)?;
    let points = vertices::moved_vertices(m, &view, &moves, &carried, tol, meter)?;
    let edges = edges::moved_edges(m, &view, &moves, &carried, &points, tol, meter)?;
    let rewrite = faces::rewrite_of(m, &view.faces, &moves, &points, &edges, tol, meter)?;
    Ok(Offset {
        rewrite,
        moved: chain.moved,
    })
}

/// The checker at `Level::Full` on `body`, in every build profile
/// (ADR-0048 §7): a report of global violations alone is
/// [`OffsetReason::SelfIntersects`], any other the construction's own
/// fault, [`OpError::Internal`] with the report.
pub(crate) fn checked_full(m: &Model, body: Body) -> Result<(), OpError> {
    let report = arris_check::check(m, body, Level::Full);
    if report.is_ok() {
        return Ok(());
    }
    Err(self_intersection(&report)
        .unwrap_or_else(|| OpError::Internal(Fault::Checker(Box::new(report)))))
}

/// [`OffsetReason::SelfIntersects`] naming what the report names, when
/// every violation is one the global level finds in a locally well-built
/// result — faces or loops crossing, an edge crossing itself, a shell
/// nested wrongly or enclosing no volume; any other violation is the
/// construction's own fault, and `None`.
fn self_intersection(report: &arris_check::Report) -> Option<OpError> {
    let mut entities = Vec::new();
    for v in report {
        match v {
            Violation::FacesIntersect { face_a, face_b, .. } => {
                entities.extend([forward(*face_a), forward(*face_b)])
            }
            Violation::LoopsIntersect { face, .. } | Violation::LoopNesting { face, .. } => {
                entities.push(forward(*face));
            }
            Violation::EdgeSelfIntersects { edge, .. } => entities.push(forward(*edge)),
            Violation::ShellNesting { body, .. } | Violation::NonPositiveVolume { body, .. } => {
                entities.push(forward(*body));
            }
            _ => return None,
        }
    }
    entities.dedup();
    Some(OpError::Degenerate {
        entities,
        reason: Reason::Offset(OffsetReason::SelfIntersects),
    })
}

/// Each moved face's offset surface: its own surface's offset by the
/// distance along the outward normal, the surface's normal composed with
/// the body's use of the face. An elliptic cylinder or a free-form surface
/// has none of its own kind ([`OffsetReason::NoExactOffset`]); a surface
/// whose radius the move drives through zero, or a cone one of whose
/// vertices it carries to the axis, collapses
/// ([`OffsetReason::SurfaceCollapses`]).
fn moves(
    m: &Model,
    view: &BodyView,
    chain: &Chain,
    distance: f64,
    tol: Tolerance,
) -> Result<Moves, OpError> {
    let refuse = |f: FaceId, reason| OpError::Degenerate {
        entities: vec![forward(f)],
        reason: Reason::Offset(reason),
    };
    let mut surfaces = BTreeMap::new();
    for &f in &chain.moved {
        let surface = m.surface(m.face(f)?.surface())?;
        let by = distance * view.orientation[&f].sign();
        let offset = match surface {
            Surface::EllipticCylinder { .. } | Surface::Nurbs(_) => {
                return Err(refuse(f, OffsetReason::NoExactOffset));
            }
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. } => surface.offset(by),
        };
        let Some(offset) = offset else {
            return Err(refuse(f, OffsetReason::SurfaceCollapses));
        };
        if let Surface::Cone {
            frame, half_angle, ..
        } = surface
        {
            // A vertex of the face at radius `ρ` from the axis moves to
            // `ρ + by · cos α`: at or below zero it has crossed the axis.
            for l in m.face(f)?.loops() {
                for c in l.coedges() {
                    let e = m.edge(c.edge())?;
                    for v in [e.start(), e.end()] {
                        let q = frame.to_local(m.vertex(v)?.point());
                        if q.x.hypot(q.y) + by * half_angle.cos() <= tol.linear {
                            return Err(refuse(f, OffsetReason::SurfaceCollapses));
                        }
                    }
                }
            }
        }
        surfaces.insert(f, offset);
    }
    Ok(Moves {
        surfaces,
        dragged: chain.dragged.clone(),
        distance,
    })
}
