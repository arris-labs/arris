//! `query`: what a consumer's facade asks of a model that no operation
//! answers (`docs/ARCHITECTURE.md` §Operations). A query beside
//! [`crate::measure`], of the same shape: it takes `&Model`, makes no
//! body, records no provenance and opens no transaction.

use arris_check::domain::FaceDomain;
use arris_geom::region2::Side;
use arris_geom::{Curve, Curve2, GeomError, Surface, project_to_plane as project};
use arris_math::{Frame, Interval, Point2, Vec2, wrap_angle};
use arris_topo::{EdgeId, EntityId, Face, Model, Orientation, Shape, VertexId};

use crate::error::{Fault, OpError, QueryReason, Reason};

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
