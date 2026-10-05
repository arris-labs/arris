//! Offset faces (ADR-0048): the chosen faces of a solid moved along their
//! outward normals by a signed distance, the rest extended or trimmed to
//! meet them, topology kept. By phase over `body_view`: the moves (each
//! moved face's offset surface), the vertices (each the point nearest its
//! old one on the surfaces around it — `vertices`, `meet`), the edges (each
//! on the section of its two new surfaces nearest the old edge, a seam on
//! its plane's), the faces (their loops re-used over the new edges), then
//! `rebuild::rewrite` and the checker at `Level::Full` in every profile.

use std::collections::{BTreeMap, BTreeSet};

use arris_check::Level;
use arris_geom::Surface;
use arris_math::{Control, Meter, Tolerance};
use arris_topo::{Body, Face, FaceId, Model, Provenance};

use crate::body_view::BodyView;
use crate::error::{Fault, InputReason, OffsetReason, OpError, Reason};
use crate::rebuild::{self, forward};

mod edges;
mod faces;
mod meet;
mod vertices;

/// The moved faces' new surfaces, by face, and the distance they moved.
pub(crate) struct Moves {
    pub(crate) surfaces: BTreeMap<FaceId, Surface>,
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
/// plane it lies in. A face tangent to a moved one is not dragged along
/// yet. The result passes the checker at `Level::Full` in every build
/// profile.
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
/// cylinder or a free-form surface, and [`OffsetReason::SurfaceCollapses`]
/// naming a moved face whose radius the move drives through zero or whose
/// cone it carries to the axis; [`OpError::Unsupported`] naming a face on
/// an elliptic cylinder or a free-form surface beside a moved one, two
/// faces tangent along an edge one of which moves, and a pole or apex the
/// move cannot place; [`OpError::NotFound`] for a face id that does not
/// resolve; [`OpError::Internal`] with the report where the result fails
/// the checker.
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
    let moves = moves(m, &view, moved, distance, tol)?;
    refuse_tangent(m, &view, &moves, tol)?;
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
/// the body's use of the face. An elliptic cylinder or a free-form surface
/// has none of its own kind ([`OffsetReason::NoExactOffset`]); a surface
/// whose radius the move drives through zero, or a cone one of whose
/// vertices it carries to the axis, collapses
/// ([`OffsetReason::SurfaceCollapses`]).
fn moves(
    m: &Model,
    view: &BodyView,
    moved: &BTreeSet<FaceId>,
    distance: f64,
    tol: Tolerance,
) -> Result<Moves, OpError> {
    let refuse = |f: FaceId, reason| OpError::Degenerate {
        entities: vec![forward(f)],
        reason: Reason::Offset(reason),
    };
    let mut surfaces = BTreeMap::new();
    for &f in moved {
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
    Ok(Moves { surfaces, distance })
}

/// An edge between two faces tangent along it, one of which moves, has no
/// section of the new surfaces to hold it — they are tangent or apart —
/// and so is outside this release, [`OpError::Unsupported`] naming the
/// two faces: the move set closed over tangent edges is the dragged
/// chain's (ADR-0048). Read at the edge's midpoint, as the blend reads a
/// tangent edge.
fn refuse_tangent(
    m: &Model,
    view: &BodyView,
    moves: &Moves,
    tol: Tolerance,
) -> Result<(), OpError> {
    for (&e, uses) in &view.uses {
        let [a, b] = uses.as_slice() else { continue };
        if a.face == b.face
            || !(moves.surfaces.contains_key(&a.face) || moves.surfaces.contains_key(&b.face))
        {
            continue;
        }
        let Some((_, range)) = m.edge(e)?.curve() else {
            continue;
        };
        let t = range.midpoint();
        let n1 = view.outward(m, a.face, m.curve2(a.pcurve)?.point(t))?;
        let n2 = view.outward(m, b.face, m.curve2(b.pcurve)?.point(t))?;
        if n1.cross(&n2).norm() <= tol.angular {
            let kind = |f: FaceId| -> Result<arris_geom::GeomKind, OpError> {
                Ok(arris_geom::GeomKind::Surface(
                    m.surface(m.face(f)?.surface())?.kind(),
                ))
            };
            return Err(OpError::Unsupported {
                a: (kind(a.face)?, forward(a.face)),
                b: (kind(b.face)?, forward(b.face)),
            });
        }
    }
    Ok(())
}
