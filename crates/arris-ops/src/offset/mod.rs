//! Offset faces (ADR-0048): the chosen faces of a solid moved along their
//! outward normals by a signed distance, the rest extended or trimmed to
//! meet them, topology kept. By phase over `body_view`: the moves (each
//! moved face's offset surface), the vertices (each the meeting of its
//! planes), the edges (each on its planes' line between its new ends),
//! the faces (their loops re-used over the new edges), then
//! `rebuild::rewrite` and the checker at `Level::Full` in every profile.

use std::collections::{BTreeMap, BTreeSet};

use arris_check::Level;
use arris_geom::Surface;
use arris_math::{Control, Meter};
use arris_topo::{Body, Face, FaceId, Model, Provenance};

use crate::body_view::BodyView;
use crate::error::{Fault, InputReason, OffsetReason, OpError, Reason};
use crate::rebuild::{self, forward};

mod edges;
mod faces;
mod vertices;

/// The moved faces' new surfaces, by face.
pub(crate) struct Moves {
    pub(crate) surfaces: BTreeMap<FaceId, Surface>,
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
/// moved and kept where it did not. The faces this release moves, and
/// the faces around every vertex they move, are planes. The result
/// passes the checker at `Level::Full` in every build profile.
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
/// a face whose loop turns inside out, once moved, and
/// [`OffsetReason::VertexSplits`] naming a vertex whose faces no longer
/// meet in one point — a pyramid's apex with one side pushed;
/// [`OpError::Unsupported`] naming a face that is not a plane and the
/// moved face whose move reaches it; [`OpError::NotFound`] for a face id
/// that does not resolve; [`OpError::Internal`] with the report where the
/// result fails the checker.
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

/// The offset of `moved` by `distance`, built and checked.
fn build(
    m: &mut Model,
    body: Body,
    moved: &BTreeSet<FaceId>,
    distance: f64,
    meter: &mut Meter<'_>,
) -> Result<(Body, Provenance), OpError> {
    let tol = m.precision().tolerance();
    let view = BodyView::of(m, body)?;
    let moves = moves(m, &view, moved, distance)?;
    let points = vertices::moved_vertices(m, &view, &moves, tol, meter)?;
    let edges = edges::moved_edges(m, &view, &moves, &points, tol, meter)?;
    let rw = faces::rewrite_of(m, &view.faces, &moves, &points, &edges, tol, meter)?;
    let out = rebuild::rewrite(m, body, rw)?;
    let report = arris_check::check(m, out.body, Level::Full);
    if !report.is_ok() {
        return Err(OpError::Internal(Fault::Checker(Box::new(report))));
    }
    Ok((out.body, out.provenance))
}

/// Each moved face's offset surface: its own surface's offset by the
/// distance along the outward normal, the surface's normal composed with
/// the body's use of the face. A face that is not a plane is outside the
/// planar core.
fn moves(
    m: &Model,
    view: &BodyView,
    moved: &BTreeSet<FaceId>,
    distance: f64,
) -> Result<Moves, OpError> {
    let mut surfaces = BTreeMap::new();
    for &f in moved {
        let surface = m.surface(m.face(f)?.surface())?;
        let sign = view.orientation[&f].sign();
        let offset = match surface {
            Surface::Plane { .. } => surface.offset(distance * sign),
            _ => None,
        };
        let Some(offset) = offset else {
            let kind = arris_geom::GeomKind::Surface(surface.kind());
            return Err(OpError::Unsupported {
                a: (kind, forward(f)),
                b: (kind, forward(f)),
            });
        };
        surfaces.insert(f, offset);
    }
    Ok(Moves { surfaces })
}
