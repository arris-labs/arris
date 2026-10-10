//! `query`: what a consumer's facade asks of a model that no operation
//! answers (`docs/ARCHITECTURE.md` §Operations). A query beside
//! [`crate::measure`], of the same shape: it takes `&Model`, makes no
//! body, records no provenance and opens no transaction.

use std::collections::BTreeMap;

mod distance;
mod interference;

pub use distance::{Distance, Foot, FootParam, Operand, distance};
pub use interference::{Clash, Contact, Interference, interference, interference_many};

use arris_check::domain::FaceDomain;
use arris_check::tree::FaceTree;
use arris_geom::region2::Side;
use arris_geom::{
    Curve, Curve2, CurveIntersection, CurveSurfaceIntersection, GeomError, Surface,
    intersect_curve_surface, intersect_curves, project_to_plane as project,
};
use arris_math::{
    Frame, Interval, Meter, Point2, Point3, Tolerance, UnitVec3, Vec2, Vec3, wrap_angle,
};
use arris_topo::{Body, EdgeId, EntityId, Face, FaceId, Model, Orientation, Shape, VertexId};

use crate::error::{Fault, InputReason, OpError, QueryReason, Reason};

/// One entity of a [`project_to_plane`] call, projected: what the
/// consumer draws on its sketch plane. Coordinates are the plane frame's
/// own `(x, y)`.
#[derive(Debug, Clone, PartialEq)]
pub enum Projection {
    /// A vertex's stored point, projected.
    Vertex {
        /// The vertex.
        vertex: VertexId,
        /// Its projection.
        point: Point2,
    },
    /// An edge's piece of curve, projected: `curve` over `range` is the
    /// orthogonal projection of the edge's curve over the edge's range,
    /// point for point — `curve.point(range.lerp(s))` is the projection of
    /// the 3D curve at `edge.range().lerp(s)` for every `s` in `[0, 1]`,
    /// so the piece starts at the projected start vertex, ends at the
    /// projected end vertex, and covers the edge and no more.
    Edge {
        /// The edge.
        edge: EdgeId,
        /// The projected curve, in its own parametrisation
        /// (`geom::project_to_plane`).
        curve: Curve2,
        /// The edge's range carried into `curve`'s parameter.
        range: Interval,
    },
}

/// The orthogonal projection of each of `shapes` onto `plane`, in the
/// order given, one [`Projection`] per shape. A vertex projects to a
/// point. An edge projects to the variant `geom::project_to_plane` makes
/// of its curve — a line a line, a conic a circle or an ellipse, a NURBS
/// a NURBS — with the edge's range carried through the projected curve's
/// *own* parameter: a line's arc length scales by the cosine of its
/// angle to the plane, and an oblique conic's angle is shifted by the
/// phase between its axes and the image ellipse's, so the range is
/// shifted with it rather than copied. A periodic range starts in
/// `[0, 2π)` and keeps its length, so it may cross the period as an
/// edge's own range may.
///
/// The projection is of the edge's own curve in its own direction; the
/// handle's orientation does not reverse it. Only edges and vertices are
/// projected: a face, a shell or a body is refused, not expanded, since
/// which of its edges a view shows is the caller's to decide.
///
/// Errors, each naming the shape: [`OpError::Degenerate`] with
/// [`QueryReason::NotProjectable`] for a shape that is not an edge or a
/// vertex, [`QueryReason::DegenerateEdge`] for an edge with no 3D curve, and
/// [`QueryReason::ProjectionCollapses`] for a curve whose projection is a
/// point or a segment — a line perpendicular to the plane, a conic whose
/// plane is; [`OpError::NotFound`] for an id, or a curve an edge holds,
/// that does not resolve.
///
/// ```
/// use arris_ops::primitive_cylinder;
/// use arris_ops::query::{Projection, project_to_plane};
/// use arris_ops::{OpError, QueryReason, Reason};
/// use arris_geom::{Curve, Curve2};
/// use arris_math::{Axis, Frame, Point3, Vec3};
/// use arris_topo::Model;
/// use core::f64::consts::FRAC_1_SQRT_2;
///
/// let mut m = Model::default();
/// let (body, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 1.0, 2.0, &arris_ops::Control::NONE).unwrap();
/// let is_circle = |id| {
///     let (curve, _) = m.edge(id).unwrap().curve().unwrap();
///     matches!(m.curve(curve).unwrap(), Curve::Circle { .. })
/// };
/// let rim = m.edges(body).unwrap().into_iter().find(|e| is_circle(e.id)).unwrap();
///
/// // Seen at 45°, the unit rim is an ellipse 1 × √½ over the whole turn.
/// let tilted = Frame::from_z(Point3::origin(), Vec3::new(0.0, 1.0, 1.0)).unwrap();
/// let p = project_to_plane(&m, &[rim.shape()], &tilted).unwrap();
/// let Projection::Edge { curve: Curve2::Ellipse { minor_radius, .. }, range, .. } = &p[0] else {
///     panic!("{p:?}")
/// };
/// assert!((minor_radius - FRAC_1_SQRT_2).abs() < 1e-15);
/// assert!((range.length() - core::f64::consts::TAU).abs() < 1e-15);
///
/// // Seen from the side, it is a segment: refused, naming the edge.
/// let side = Frame::from_z(Point3::origin(), Vec3::y()).unwrap();
/// let err = project_to_plane(&m, &[rim.shape()], &side).unwrap_err();
/// assert!(matches!(err, OpError::Degenerate { reason: Reason::Query(QueryReason::ProjectionCollapses), .. }));
/// ```
pub fn project_to_plane(
    m: &Model,
    shapes: &[Shape],
    plane: &Frame,
) -> Result<Vec<Projection>, OpError> {
    shapes
        .iter()
        .map(|&shape| match shape.id {
            EntityId::Vertex(vertex) => {
                let q = plane.to_local(m.vertex(vertex)?.point());
                Ok(Projection::Vertex {
                    vertex,
                    point: Point2::new(q.x, q.y),
                })
            }
            EntityId::Edge(edge) => project_edge(m, shape, edge, plane),
            EntityId::Face(_) | EntityId::Shell(_) | EntityId::Body(_) => Err(degenerate(
                shape,
                Reason::Query(QueryReason::NotProjectable),
            )),
        })
        .collect()
}

fn degenerate(shape: Shape, reason: Reason) -> OpError {
    OpError::Degenerate {
        entities: vec![shape],
        reason,
    }
}

fn project_edge(
    m: &Model,
    shape: Shape,
    edge: EdgeId,
    plane: &Frame,
) -> Result<Projection, OpError> {
    let Some((curve_id, range)) = m.edge(edge)?.curve() else {
        return Err(degenerate(
            shape,
            Reason::Query(QueryReason::DegenerateEdge),
        ));
    };
    let curve = m.curve(curve_id)?;
    let projected = project(curve, plane).map_err(|e| match e {
        GeomError::Degenerate { .. } => {
            degenerate(shape, Reason::Query(QueryReason::ProjectionCollapses))
        }
        other => OpError::Internal(Fault::Geometry(other)),
    })?;
    let range = carried_range(curve, &projected, range, plane)?;
    Ok(Projection::Edge {
        edge,
        curve: projected,
        range,
    })
}

/// `range` of `curve` in the parameter of `projected`, its projection.
/// Every arm of `geom::project_to_plane` is affine in the parameter: a
/// line's is `s = |D'| t` with `D'` the direction's in-plane part, a
/// NURBS keeps `t`, and a conic's is `s = t + φ`. The phase `φ` is read
/// off the image at `t = 0` in the image's own frame: there the
/// projected point's local `x` is `a cos φ` and the projected tangent's
/// local `x` is `−a sin φ`, with the one positive major radius `a` —
/// never the minor radius, so a nearly edge-on conic costs no precision.
/// A conic parallel to the plane comes out with `φ` zero to rounding.
///
/// Errors: [`Fault::Invariant`] when the projection is not the variant
/// `geom::project_to_plane` makes of the curve's kind, or the carried
/// range is not an interval.
fn carried_range(
    curve: &Curve,
    projected: &Curve2,
    range: Interval,
    plane: &Frame,
) -> Result<Interval, OpError> {
    let invariant = |what| OpError::Internal(Fault::Invariant { what });
    let (lo, hi) = match (curve, projected) {
        (&Curve::Line { direction, .. }, Curve2::Line { .. }) => {
            let d = plane.vec_to_local(direction.into_inner());
            let k = Vec2::new(d.x, d.y).norm();
            (k * range.lo(), k * range.hi())
        }
        (
            Curve::Circle { .. } | Curve::Ellipse { .. },
            Curve2::Circle { frame, .. } | Curve2::Ellipse { frame, .. },
        ) => {
            let at_zero = curve.eval(0.0);
            let p = plane.to_local(at_zero.point);
            let w = plane.vec_to_local(at_zero.d1);
            let q = frame.to_local(Point2::new(p.x, p.y));
            let w = frame.vec_to_local(Vec2::new(w.x, w.y));
            let lo = wrap_angle(range.lo() + (-w.x).atan2(q.x));
            (lo, lo + range.length())
        }
        (Curve::Nurbs(_), Curve2::Nurbs(_)) => return Ok(range),
        (
            Curve::Line { .. } | Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_),
            _,
        ) => {
            return Err(invariant("the projection of the curve's own kind"));
        }
    };
    Interval::new(lo, hi).map_err(|_| invariant("a carried range that is an interval"))
}

/// `face`'s surface frame, `Z` the outward normal: the surface's own
/// frame, `Z` (and, to keep it right-handed, `X`) negated when `face`'s
/// use is [`Orientation::Reversed`]. Only a plane has one frame for its
/// whole domain — a curved surface's normal, and so its frame, varies
/// with `(u, v)`, which [`frame_at`] answers.
///
/// Errors: [`OpError::NotFound`] for an id that does not resolve;
/// [`OpError::Degenerate`] with [`QueryReason::NotPlanar`], naming the face,
/// when its surface is not a plane.
///
/// ```
/// use arris_ops::primitive_box;
/// use arris_ops::query::face_frame;
/// use arris_topo::Model;
/// use arris_geom::Surface;
/// use arris_math::Point3;
///
/// let mut m = Model::default();
/// let (body, _) =
///     primitive_box(&mut m, Point3::new(-1.0, -1.0, -1.0), Point3::new(1.0, 1.0, 1.0), &arris_ops::Control::NONE).unwrap();
/// let top = m
///     .faces(body)
///     .unwrap()
///     .into_iter()
///     .find(|f| {
///         let Surface::Plane { frame } = m.surface(m.face(f.id).unwrap().surface()).unwrap()
///         else {
///             return false;
///         };
///         frame.z().z > 0.5
///     })
///     .unwrap();
/// let frame = face_frame(&m, top).unwrap();
/// assert!((frame.z().z - 1.0).abs() < 1e-12);
/// ```
pub fn face_frame(m: &Model, face: Face) -> Result<Frame, OpError> {
    let entity = m.face(face.id)?;
    let surface = m.surface(entity.surface())?;
    let Surface::Plane { frame } = surface else {
        return Err(degenerate(
            face.shape(),
            Reason::Query(QueryReason::NotPlanar),
        ));
    };
    Ok(if face.orientation == Orientation::Reversed {
        Frame::from_orthonormal(
            frame.origin(),
            -frame.x().into_inner(),
            frame.y().into_inner(),
            -frame.z().into_inner(),
        )?
    } else {
        *frame
    })
}

/// The outward-oriented frame of `face`'s surface at `(u, v)`: `Z` is
/// [`Surface::normal`] there, composed with `face`'s use as
/// [`face_frame`]'s is; `X` is the projection of `∂P/∂u` onto the
/// tangent plane, `Y = Z × X`. Over a planar face `frame_at`'s `Z`
/// agrees with [`face_frame`]'s at every `(u, v)`, since a plane's
/// normal does not depend on the parameter.
///
/// Errors: [`OpError::NotFound`] for an id that does not resolve;
/// [`OpError::Degenerate`], naming the face, with [`QueryReason::OutOfDomain`]
/// when `(u, v)` lies outside the face's own domain
/// ([`arris_check::domain::FaceDomain`]) and [`QueryReason::Singular`] when
/// the surface's parametrisation is singular there — a sphere's pole, a
/// cone's apex — so it has no normal to compose.
///
/// ```
/// use arris_ops::primitive_cylinder;
/// use arris_ops::query::frame_at;
/// use arris_topo::Model;
/// use arris_geom::Surface;
/// use arris_math::{Axis, Point2, Point3};
/// use core::f64::consts::FRAC_PI_2;
///
/// let mut m = Model::default();
/// let (body, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 1.0, 2.0, &arris_ops::Control::NONE).unwrap();
/// let wall = m
///     .faces(body)
///     .unwrap()
///     .into_iter()
///     .find(|f| {
///         matches!(
///             m.surface(m.face(f.id).unwrap().surface()).unwrap(),
///             Surface::Cylinder { .. }
///         )
///     })
///     .unwrap();
/// let frame = frame_at(&m, wall, Point2::new(FRAC_PI_2, 1.0)).unwrap();
/// assert!((frame.z().y - 1.0).abs() < 1e-12);
/// ```
pub fn frame_at(m: &Model, face: Face, uv: Point2) -> Result<Frame, OpError> {
    let entity = m.face(face.id)?;
    let domain = FaceDomain::of(m, face.id, entity.tolerance())?;
    if domain.side(uv).0 == Side::Outside {
        return Err(degenerate(
            face.shape(),
            Reason::Query(QueryReason::OutOfDomain),
        ));
    }
    let surface = m.surface(entity.surface())?;
    let Some(normal) = surface.normal(uv.x, uv.y) else {
        return Err(degenerate(
            face.shape(),
            Reason::Query(QueryReason::Singular),
        ));
    };
    let normal = if face.orientation == Orientation::Reversed {
        -normal.into_inner()
    } else {
        normal.into_inner()
    };
    let e = surface.eval(uv.x, uv.y);
    Ok(Frame::new(e.point, normal, e.du)?)
}

/// A half-line to fire at a body: the points `origin + t · direction`,
/// `t >= 0`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ray {
    /// Where it starts.
    pub origin: Point3,
    /// Which way it runs. Any length but zero; a hit's `t` is measured
    /// along its normalisation, so it is a distance.
    pub direction: Vec3,
}

/// How many of a ray's hits [`ray_fire`] returns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RayHits {
    /// The hit with the smallest `t`, if there is one.
    First,
    /// Every hit, by ascending `t`.
    All,
}

/// One place a ray meets a body.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    /// The most specific entity the hit is within the tolerance of: a
    /// vertex, else an edge, else a face (ADR-0053 §1), as
    /// [`arris_check::classify::Classification::On`] names it. A face
    /// carries its use's orientation in the body.
    pub shape: Shape,
    /// The distance along the ray's unit direction; never negative.
    pub t: f64,
    /// The `(u, v)` of the hit on its face: on the face with the lowest
    /// id, for a hit that names an edge or a vertex.
    pub uv: Point2,
    /// `origin + t · direction`, normalised.
    pub point: Point3,
    /// The outward normal of the hit face, composed with its use as
    /// [`frame_at`]'s `Z` is: of the face with the lowest id, for a hit
    /// that names an edge or a vertex.
    pub normal: UnitVec3,
    /// `true` when the ray touches the surface without crossing it
    /// (ADR-0053 §3): a graze counts as a hit, and never as a crossing.
    pub tangent: bool,
}

/// A hit before the faces that share it are merged.
struct Candidate {
    face: FaceId,
    hit: RayHit,
}

/// Where `ray` meets `body`: every hit, or the first, each with the
/// entity it lands on, the ray parameter, the face's `(u, v)`, the point
/// and the face's outward normal (ADR-0053).
///
/// The ray is a half-line: a hit behind its origin is never reported, and
/// an origin on the boundary (within the entity's own tolerance) is a hit
/// at `t = 0`. A ray from inside a solid reports what it leaves through.
/// A hit names the most specific entity: a ray through a box's edge is one
/// hit naming the edge, not two on its faces, and its `uv` and `normal`
/// are the lowest-id face's of the two. A ray that only touches a surface
/// is a hit with `tangent` set. Hits come by ascending `t`, ties by the
/// entity's id. `First` stops at the nearest face box that cannot beat the
/// hit found.
///
/// The faces are reached through a [`FaceTree`] over their boxes, and a
/// line meets a NURBS face through `arris_geom::intersect_curve_surface`,
/// so every face kind is answered.
///
/// Errors: [`OpError::InvalidInput`] when the body fails the checker
/// (debug builds, and release with the `paranoid` feature);
/// [`OpError::NotFound`] when the body or an entity it refers to does not
/// resolve; [`OpError::Degenerate`] with [`QueryReason::ZeroDirection`],
/// naming the body, for a direction of no length, and with
/// [`InputReason::NonFinite`] for a non-finite origin or direction;
/// [`QueryReason::RayInFace`], naming the face, for a ray lying in a face
/// — a plane face, or a ruling of a cylinder, a cone or a ruled NURBS
/// surface — which has no single hit to give (ADR-0053 §4);
/// [`QueryReason::Singular`], naming the face, for a hit at a pole or an
/// apex, where the surface has no normal. [`OpError::Internal`] when the
/// intersector fails on validated input.
///
/// ```
/// use arris_ops::primitive_box;
/// use arris_ops::query::{Ray, RayHits, ray_fire};
/// use arris_math::{Point3, Vec3};
/// use arris_topo::{EntityId, Model};
///
/// let mut m = Model::default();
/// let (body, _) = primitive_box(&mut m, Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 3.0, 2.0), &arris_ops::Control::NONE).unwrap();
///
/// // Up through the middle of the box: in at the bottom, out at the top.
/// let up = Ray { origin: Point3::new(2.0, 1.5, -1.0), direction: Vec3::new(0.0, 0.0, 2.0) };
/// let hits = ray_fire(&m, body, &up, RayHits::All).unwrap();
/// assert_eq!(hits.len(), 2);
/// assert!((hits[0].t - 1.0).abs() < 1e-12 && (hits[1].t - 3.0).abs() < 1e-12);
/// assert!(hits.iter().all(|h| matches!(h.shape.id, EntityId::Face(_))));
/// assert!((hits[0].normal.z + 1.0).abs() < 1e-12 && (hits[1].normal.z - 1.0).abs() < 1e-12);
///
/// // Diagonally through an edge, it is one hit naming the edge.
/// let edge = Ray { origin: Point3::new(5.0, 4.0, 1.0), direction: Vec3::new(-1.0, -1.0, 0.0) };
/// let hits = ray_fire(&m, body, &edge, RayHits::First).unwrap();
/// assert!(matches!(hits[0].shape.id, EntityId::Edge(_)));
/// ```
pub fn ray_fire(m: &Model, body: Body, ray: &Ray, which: RayHits) -> Result<Vec<RayHit>, OpError> {
    crate::verify_input(m, body)?;
    let whole = Shape::new(body.id, body.orientation);
    if !ray.origin.coords.iter().all(|c| c.is_finite()) {
        return Err(non_finite(whole, "the ray's origin"));
    }
    if !ray.direction.iter().all(|c| c.is_finite()) {
        return Err(non_finite(whole, "the ray's direction"));
    }
    if ray.direction.norm() == 0.0 {
        return Err(degenerate(whole, Reason::Query(QueryReason::ZeroDirection)));
    }
    let direction = UnitVec3::new_normalize(ray.direction);
    let line = Curve::Line {
        origin: ray.origin,
        direction,
    };

    let uses = face_uses(m, body)?;
    let mut reach = 0.0f64;
    for &id in uses.keys() {
        reach = reach.max(m.face(id)?.tolerance());
    }
    let tree = FaceTree::of_body(m, body)?;
    let entries = tree.entries(ray.origin, direction.into_inner(), (-reach, f64::INFINITY));

    let mut found: Vec<Candidate> = Vec::new();
    for (entry, id) in entries {
        // Entries ascend, and a face is hit no nearer than its box is
        // entered, so once the nearest hit beats an entry it beats every
        // later one.
        if which == RayHits::First
            && let Some(nearest) = found.iter().map(|c| c.hit.t).reduce(f64::min)
            && entry > nearest + reach
        {
            break;
        }
        let Some(&orientation) = uses.get(&id) else {
            continue;
        };
        face_hits(m, id, orientation, &line, ray.origin, direction, &mut found)?;
    }

    let merged = merge(found, reach);
    Ok(match which {
        RayHits::All => merged,
        RayHits::First => merged.into_iter().take(1).collect(),
    })
}

fn non_finite(shape: Shape, what: &'static str) -> OpError {
    degenerate(shape, Reason::Input(InputReason::NonFinite { what }))
}

/// Each face of `body` with its use's orientation in the body, composed
/// through the shell and body handles; the first use of a face used twice.
fn face_uses(m: &Model, body: Body) -> Result<BTreeMap<FaceId, Orientation>, OpError> {
    let mut uses = BTreeMap::new();
    for shell_use in m.body(body.id)?.shells() {
        let orientation = body.orientation.compose(shell_use.orientation);
        for face_use in m.shell(shell_use.id)?.faces() {
            uses.entry(face_use.id)
                .or_insert_with(|| orientation.compose(face_use.orientation));
        }
    }
    Ok(uses)
}

/// The hits of `line`, from `origin`, on `face`, appended to `found`:
/// each a point of the surface at `t >= 0` that is in the face's domain,
/// named by the most specific entity the point is within the tolerance of.
fn face_hits(
    m: &Model,
    id: FaceId,
    orientation: Orientation,
    line: &Curve,
    origin: Point3,
    direction: UnitVec3,
    found: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    let face = m.face(id)?;
    let surface = m.surface(face.surface())?;
    let shape = Shape::new(id, orientation);
    let precision = m.precision();
    let tolerance = Tolerance::new(face.tolerance(), precision.angular_tolerance);
    let domain = FaceDomain::of(m, id, precision.parametric_tolerance)?;
    let hits = match intersect_curve_surface(line, surface, tolerance, &mut Meter::default())
        .map_err(|e| OpError::Internal(Fault::Geometry(e)))?
    {
        CurveSurfaceIntersection::Points(hits) => hits,
        CurveSurfaceIntersection::Coincident => {
            // The ray lies in the face's surface: it is in the face when it
            // runs through the face's domain, and otherwise it misses it.
            if lies_in_face(m, id, &domain, line, origin, direction, tolerance)? {
                return Err(degenerate(shape, Reason::Query(QueryReason::RayInFace)));
            }
            return Ok(());
        }
    };
    for hit in hits {
        if hit.t < -face.tolerance() {
            continue;
        }
        let (side, _) = domain.side(hit.uv);
        if side == Side::Outside {
            continue;
        }
        let entity = match domain.boundary_entity(m, hit.point)? {
            Some(entity) => entity,
            None if side == Side::Inside || domain.winds_around(hit.uv) => shape,
            // Within the band of the boundary but on none of its edges
            // and not inside it: beside the face.
            None => continue,
        };
        let Some(normal) = surface.normal(hit.uv.x, hit.uv.y) else {
            return Err(degenerate(shape, Reason::Query(QueryReason::Singular)));
        };
        let normal = if orientation == Orientation::Reversed {
            -normal.into_inner()
        } else {
            normal.into_inner()
        };
        found.push(Candidate {
            face: id,
            hit: RayHit {
                shape: entity,
                t: hit.t.max(0.0),
                uv: hit.uv,
                point: hit.point,
                normal: UnitVec3::new_normalize(normal),
                tangent: hit.tangent,
            },
        });
    }
    Ok(())
}

/// Whether the ray, which lies in `face`'s surface, runs through the
/// face: along one of its edges, or through a stretch between two of its
/// crossings of the boundary (or its origin and the first) that is in the
/// face's domain. Beyond the last crossing a ray has left a bounded face.
fn lies_in_face(
    m: &Model,
    id: FaceId,
    domain: &FaceDomain<'_>,
    line: &Curve,
    origin: Point3,
    direction: UnitVec3,
    tolerance: Tolerance,
) -> Result<bool, OpError> {
    let face = m.face(id)?;
    let surface = m.surface(face.surface())?;
    let mut edges: Vec<EdgeId> = face
        .loops()
        .iter()
        .flat_map(|l| l.coedges())
        .map(|c| c.edge())
        .collect();
    edges.sort_unstable();
    edges.dedup();
    let mut crossings = vec![0.0];
    for e in edges {
        let Some((curve, _)) = m.edge(e)?.curve() else {
            continue;
        };
        match intersect_curves(line, m.curve(curve)?, tolerance, &mut Meter::default())
            .map_err(|e| OpError::Internal(Fault::Geometry(e)))?
        {
            CurveIntersection::Coincident => return Ok(true),
            CurveIntersection::Points(hits) => {
                crossings.extend(hits.iter().map(|h| h.ta).filter(|&t| t > 0.0));
            }
        }
    }
    crossings.sort_unstable_by(f64::total_cmp);
    let at = |t: f64| origin + direction.into_inner() * t;
    let mut stretches: Vec<f64> = crossings.windows(2).map(|w| 0.5 * (w[0] + w[1])).collect();
    if crossings.len() == 1 {
        stretches.push(0.0);
    }
    for t in stretches {
        let Ok(projection) = surface.project(at(t)) else {
            continue;
        };
        if domain.side(projection.uv).0 != Side::Outside {
            return Ok(true);
        }
    }
    Ok(false)
}

/// The candidates by ascending `t`, ties by the entity, with those that
/// name one edge or vertex at the same place (within `reach`, the largest
/// face tolerance) merged into one: the lowest-id face's.
fn merge(mut found: Vec<Candidate>, reach: f64) -> Vec<RayHit> {
    found.sort_by(|a, b| {
        a.hit
            .t
            .total_cmp(&b.hit.t)
            .then(a.hit.shape.id.cmp(&b.hit.shape.id))
            .then(a.face.cmp(&b.face))
    });
    let mut merged: Vec<Candidate> = Vec::new();
    for c in found {
        let shared = !matches!(c.hit.shape.id, EntityId::Face(_));
        let same = merged.iter_mut().find(|k| {
            shared && k.hit.shape.id == c.hit.shape.id && (k.hit.t - c.hit.t).abs() <= reach
        });
        match same {
            Some(k) if c.face < k.face => *k = c,
            Some(_) => {}
            None => merged.push(c),
        }
    }
    merged.sort_by(|a, b| {
        a.hit
            .t
            .total_cmp(&b.hit.t)
            .then(a.hit.shape.id.cmp(&b.hit.shape.id))
    });
    merged.into_iter().map(|c| c.hit).collect()
}
