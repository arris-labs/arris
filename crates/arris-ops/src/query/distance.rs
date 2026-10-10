//! The minimum distance between two operands (ADR-0054): point sets, found
//! by enumerating the critical points of the distance on the carriers and
//! recursing into the boundaries, never by sampling.
//!
//! A point or a vertex against an entity is the base case: the feet of the
//! point on the carrier that lie inside the entity's trim, and the same
//! question asked of the entity's boundary (face → edges, edge → end
//! vertices). A continuum of feet adds a candidate only where its one
//! representative lies inside the trim; the rest of its level set reaches
//! a boundary, whose own terms attain it.

use arris_check::classify::{Classification, Classifier};
use arris_check::domain::{FaceDomain, shifts};
use arris_check::tree::FaceTree;
use arris_geom::region2::Side;
use arris_geom::{
    Curve, GeomError, extrema_curve_curve, extrema_curve_surface, extrema_surface_surface,
    intersect_surfaces,
};
use arris_math::{Aabb, Control, Interval, Meter, Point2, Point3, Tolerance};
use arris_topo::entity::BodyKind;
use arris_topo::{Body, EdgeId, EntityId, FaceId, Model, Orientation, Shape, VertexId};

use crate::error::{Fault, InputReason, OpError, QueryReason, Reason, fault_of};

/// What a distance is asked between.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Operand {
    /// A bare point.
    Point(Point3),
    /// A vertex, an edge, a face or a body of the model. A shell, and a
    /// body that is not a solid, are refused
    /// ([`QueryReason::NotMeasurable`]).
    Shape(Shape),
}

/// Where on an entity a [`Foot`] lies.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FootParam {
    /// A point, a vertex, or a body's interior: nothing to parametrise.
    None,
    /// The parameter on an edge's curve, within the edge's range.
    Edge(f64),
    /// The `(u, v)` on a face's surface.
    Face(Point2),
}

/// The closest point on one operand.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Foot {
    /// The point.
    pub point: Point3,
    /// The most specific entity the point lies on: a vertex, else an edge,
    /// else a face (ADR-0053 §1). `None` for a bare point operand, and for
    /// a point inside a solid.
    pub shape: Option<Shape>,
    /// The entity's parameter at the point.
    pub param: FootParam,
}

/// The minimum distance between two operands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Distance {
    /// The distance between the operands as point sets; `0.0` when they
    /// meet or one contains the other.
    pub distance: f64,
    /// The closest point on the first operand.
    pub a: Foot,
    /// The closest point on the second.
    pub b: Foot,
    /// `true` when this pair is the only one at the minimum (within the
    /// model's linear tolerance); `false` for a continuum of them or
    /// several apart. The pair returned is then the lowest by entity id and
    /// parameter, so the answer is deterministic.
    pub unique: bool,
    /// `true` when a point or a vertex of one operand lies in the interior
    /// of the other's solid, or an operand lies wholly in it.
    pub contained: bool,
}

/// One end of a candidate pair: a point on an operand.
#[derive(Debug, Clone, Copy)]
struct End {
    point: Point3,
    shape: Option<Shape>,
    param: FootParam,
}

impl End {
    fn foot(self) -> Foot {
        Foot {
            point: self.point,
            shape: self.shape,
            param: self.param,
        }
    }

    /// Vertex before edge before face: the most specific entity a point
    /// within the tolerance of several is on.
    fn rank(&self) -> u8 {
        match self.shape.map(|s| s.id) {
            Some(EntityId::Vertex(_)) | None => 0,
            Some(EntityId::Edge(_)) => 1,
            Some(EntityId::Face(_) | EntityId::Shell(_) | EntityId::Body(_)) => 2,
        }
    }

    fn params(&self) -> [f64; 2] {
        match self.param {
            FootParam::None => [0.0, 0.0],
            FootParam::Edge(t) => [t, 0.0],
            FootParam::Face(uv) => [uv.x, uv.y],
        }
    }
}

/// One place the distance is attained or stationary, before the choice.
#[derive(Debug, Clone, Copy)]
struct Candidate {
    distance: f64,
    a: End,
    b: End,
    /// Found as the representative of a continuum of equal pairs.
    continuum: bool,
}

type Key = (u8, u8, Option<EntityId>, Option<EntityId>, [f64; 4]);

impl Candidate {
    fn swapped(self) -> Candidate {
        Candidate {
            a: self.b,
            b: self.a,
            ..self
        }
    }

    /// The order the pick is made in: the most specific entities, then
    /// their ids, then their parameters.
    fn key(&self) -> Key {
        let (pa, pb) = (self.a.params(), self.b.params());
        (
            self.a.rank(),
            self.b.rank(),
            self.a.shape.map(|s| s.id),
            self.b.shape.map(|s| s.id),
            [pa[0], pa[1], pb[0], pb[1]],
        )
    }
}

/// What an entity against which a distance is measured can be.
#[derive(Debug, Clone, Copy)]
enum Entity {
    Edge(EdgeId),
    Face(FaceId, Orientation),
}

/// An operand, resolved.
enum Resolved {
    /// A bare point, or a vertex (a point with its shape).
    Point(Point3, Option<Shape>),
    Entity(Entity),
    Body(Body),
}

fn not_measurable(shapes: impl IntoIterator<Item = Shape>) -> OpError {
    OpError::Degenerate {
        entities: shapes.into_iter().collect(),
        reason: Reason::Query(QueryReason::NotMeasurable),
    }
}

fn resolve(m: &Model, operand: Operand) -> Result<Resolved, OpError> {
    match operand {
        Operand::Point(p) => {
            if !p.coords.iter().all(|c| c.is_finite()) {
                return Err(OpError::Degenerate {
                    entities: vec![],
                    reason: Reason::Input(InputReason::NonFinite { what: "the point" }),
                });
            }
            Ok(Resolved::Point(p, None))
        }
        Operand::Shape(shape) => match shape.id {
            EntityId::Vertex(v) => Ok(Resolved::Point(m.vertex(v)?.point(), Some(shape))),
            EntityId::Edge(e) => {
                let edge = m.edge(e)?;
                if edge.curve().is_none() {
                    return Err(OpError::Degenerate {
                        entities: vec![shape],
                        reason: Reason::Query(QueryReason::DegenerateEdge),
                    });
                }
                Ok(Resolved::Entity(Entity::Edge(e)))
            }
            EntityId::Face(f) => {
                m.face(f)?;
                Ok(Resolved::Entity(Entity::Face(f, shape.orientation)))
            }
            EntityId::Shell(_) => Err(not_measurable([shape])),
            EntityId::Body(b) => {
                if m.body(b)?.kind() != BodyKind::Solid {
                    return Err(not_measurable([shape]));
                }
                Ok(Resolved::Body(Body::new(b, shape.orientation)))
            }
        },
    }
}

/// The minimum distance between two operands: the length of the shortest
/// segment between a point of one and a point of the other, the closest
/// point on each with the most specific entity it lies on and that
/// entity's parameter, whether the minimum is unique, and whether one
/// operand lies in a solid (ADR-0054).
///
/// An operand is a point set: a body is its solid, so a point inside it,
/// or a body inside another, is at distance `0.0` with `contained` set. The
/// answer is exact to the entities' tolerances, from the critical points
/// of the distance on each carrier and the same question asked of the
/// boundary, not from a mesh. Of several closest pairs one is returned,
/// the lowest by entity id then parameter among those within the model's
/// linear tolerance of the minimum, and `unique` says whether it was alone.
///
/// A point or a vertex against any entity is answered, with a face's
/// feet through its [`FaceDomain`] and a body through the nearest-first
/// walk of its [`FaceTree`]. Two entities (edges and faces) are the
/// recursion on boundaries: the carriers' stationary pairs inside both
/// trims, each operand's boundary against the other, and for two faces the
/// places their surfaces touch inside both. A solid against an edge, a face
/// or another solid is that recursion over the pairs of its faces and the
/// other's parts, nearest boxes first, skipping every pair whose boxes are
/// farther apart than the best answer so far. A solid with a vertex of the
/// other operand inside it is `contained`, at distance `0.0`, as is one
/// holding the other wholly where no vertex says so. The `control` is
/// polled once per entity and once per face pair.
///
/// Errors: [`OpError::InvalidInput`] when a body fails the checker (debug
/// builds, and release with `paranoid`); [`OpError::NotFound`] for an id
/// that does not resolve; [`OpError::Degenerate`] with
/// [`QueryReason::NotMeasurable`] for a shell or a body that is not a
/// solid, with [`QueryReason::DegenerateEdge`] for a degenerate edge
/// operand, and with `InputReason::NonFinite` for a non-finite point;
/// [`OpError::Unsupported`] for a pair of entities the kernel has no
/// formula for (a cone against a cone off their common axis, a cone or an
/// elliptic cylinder against an elliptic cylinder, a curve or a surface
/// against a NURBS surface's interior), naming both entities;
/// [`OpError::Interrupted`] when the control stops it.
///
/// ```
/// use arris_math::{Point3, Vec3};
/// use arris_ops::primitive_box;
/// use arris_ops::query::{Operand, distance};
/// use arris_topo::{EntityId, Model, Shape};
///
/// let mut m = Model::default();
/// let (body, _) = primitive_box(&mut m, Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 3.0, 2.0), &arris_ops::Control::NONE).unwrap();
/// let whole = Operand::Shape(Shape::new(body.id, body.orientation));
///
/// // A point above the top face lands in its middle.
/// let d = distance(&m, Operand::Point(Point3::new(1.0, 1.0, 5.0)), whole, &arris_ops::Control::NONE).unwrap();
/// assert!((d.distance - 3.0).abs() < 1e-12);
/// assert!(matches!(d.b.shape.unwrap().id, EntityId::Face(_)));
/// assert!(d.unique && !d.contained);
///
/// // Past a corner, it lands on the vertex.
/// let d = distance(&m, Operand::Point(Point3::new(5.0, 4.0, 3.0)), whole, &arris_ops::Control::NONE).unwrap();
/// assert!((d.distance - 3f64.sqrt()).abs() < 1e-12);
/// assert!(matches!(d.b.shape.unwrap().id, EntityId::Vertex(_)));
///
/// // Inside the solid it is zero.
/// let d = distance(&m, Operand::Point(Point3::new(1.0, 1.0, 1.0)), whole, &arris_ops::Control::NONE).unwrap();
/// assert!(d.contained && d.distance == 0.0);
///
/// // Two solids: the gap between facing faces.
/// let (other, _) = primitive_box(&mut m, Point3::new(1.0, 1.0, 5.0), Point3::new(3.0, 2.0, 8.0), &arris_ops::Control::NONE).unwrap();
/// let apart = Operand::Shape(Shape::new(other.id, other.orientation));
/// let d = distance(&m, whole, apart, &arris_ops::Control::NONE).unwrap();
/// assert!((d.distance - 3.0).abs() < 1e-12);
/// ```
pub fn distance(
    m: &Model,
    a: Operand,
    b: Operand,
    control: &Control<'_>,
) -> Result<Distance, OpError> {
    let (ra, rb) = (resolve(m, a)?, resolve(m, b)?);
    let body_of = |r: &Resolved| match r {
        Resolved::Body(body) => Some(*body),
        _ => None,
    };
    let (ra_body, rb_body) = (body_of(&ra), body_of(&rb));
    for r in [&ra, &rb] {
        if let Resolved::Body(body) = r {
            crate::verify_input(m, *body)?;
        }
    }
    // A vertex of one operand inside the other's solid: contained, and at
    // zero, whatever the boundaries do (ADR-0054 §2).
    for (solid, probe, solid_is_a) in [(&ra, &rb, true), (&rb, &ra, false)] {
        let (Resolved::Body(body), Resolved::Entity(_) | Resolved::Body(_)) = (solid, probe) else {
            continue;
        };
        let vertices = vertices_of(m, probe)?;
        let classifier = Classifier::of_body(m, *body).map_err(fault_classify)?;
        for v in vertices {
            let at = vertex_end(m, v)?;
            if classifier.classify(at.point).map_err(fault_classify)? == Classification::Inside {
                let within = End {
                    point: at.point,
                    shape: None,
                    param: FootParam::None,
                };
                let (a, b) = if solid_is_a {
                    (within, at)
                } else {
                    (at, within)
                };
                return Ok(Distance {
                    distance: 0.0,
                    a: a.foot(),
                    b: b.foot(),
                    unique: true,
                    contained: true,
                });
            }
        }
    }
    let mut meter = Meter::new(control);
    let tol = m.precision().tolerance();
    let mut found: Vec<Candidate> = Vec::new();
    match (ra, rb) {
        (Resolved::Point(p, sp), Resolved::Point(q, sq)) => {
            found.push(Candidate {
                distance: (p - q).norm(),
                a: End {
                    point: p,
                    shape: sp,
                    param: FootParam::None,
                },
                b: End {
                    point: q,
                    shape: sq,
                    param: FootParam::None,
                },
                continuum: false,
            });
        }
        (Resolved::Point(p, shape), other) => {
            let src = End {
                point: p,
                shape,
                param: FootParam::None,
            };
            if let Some(d) = point_against(m, src, other, &mut meter, &mut found)? {
                return Ok(d);
            }
        }
        (other, Resolved::Point(p, shape)) => {
            let src = End {
                point: p,
                shape,
                param: FootParam::None,
            };
            let mut swapped = Vec::new();
            if let Some(mut d) = point_against(m, src, other, &mut meter, &mut swapped)? {
                std::mem::swap(&mut d.a, &mut d.b);
                return Ok(d);
            }
            found.extend(swapped.into_iter().map(Candidate::swapped));
        }
        (Resolved::Entity(x), Resolved::Entity(y)) => {
            entity_vs(m, &mut meter, x, y, &mut found)?;
        }
        (Resolved::Body(x), other) => {
            body_vs(m, x, &other, &mut meter, tol, &mut found)?;
        }
        (other, Resolved::Body(y)) => {
            let mut swapped = Vec::new();
            body_vs(m, y, &other, &mut meter, tol, &mut swapped)?;
            swapped_into(swapped, &mut found);
        }
    }
    let shapes = [a, b].map(|o| match o {
        Operand::Shape(s) => Some(s),
        Operand::Point(_) => None,
    });
    let mut answer =
        choose(&found, tol).ok_or_else(|| not_measurable(shapes.into_iter().flatten()))?;
    if answer.distance > tol.linear {
        // Apart, so neither operand crosses the other's boundary: an
        // operand lies wholly in a solid or wholly outside it, and its
        // closest point says which.
        for (solid, other) in [(ra_body, false), (rb_body, true)] {
            let Some(body) = solid else { continue };
            // The operand `solid` is `a` unless `other`; the foot tested is
            // the other operand's.
            let foot = if other { answer.a } else { answer.b };
            if inside(m, body, foot.point)? {
                // The solid's side of the pair is the same point, inside it.
                let at = End {
                    point: foot.point,
                    shape: None,
                    param: FootParam::None,
                }
                .foot();
                answer.distance = 0.0;
                answer.contained = true;
                if other {
                    answer.b = at;
                } else {
                    answer.a = at;
                }
                break;
            }
        }
    }
    Ok(answer)
}

/// `src` against one resolved operand that is not a point: appends the
/// candidates, or returns the finished answer for a point inside a solid.
fn point_against(
    m: &Model,
    src: End,
    other: Resolved,
    meter: &mut Meter,
    found: &mut Vec<Candidate>,
) -> Result<Option<Distance>, OpError> {
    match other {
        Resolved::Point(..) => unreachable!("two points are measured directly"),
        Resolved::Entity(e) => point_vs(m, meter, src, e, found)?,
        Resolved::Body(body) => {
            let classified = Classifier::of_body(m, body)
                .and_then(|c| c.classify(src.point))
                .map_err(|e| OpError::Internal(Fault::Classify(e)))?;
            if classified == Classification::Inside {
                let at = End {
                    point: src.point,
                    shape: None,
                    param: FootParam::None,
                };
                return Ok(Some(Distance {
                    distance: 0.0,
                    a: src.foot(),
                    b: at.foot(),
                    unique: true,
                    contained: true,
                }));
            }
            body_candidates(m, body, src, meter, found)?;
        }
    }
    Ok(None)
}

/// The pick among `found`: the lowest by `(rank, entity, parameters)` of
/// those within the linear tolerance of the least distance, and whether
/// it was alone.
fn choose(found: &[Candidate], tol: Tolerance) -> Option<Distance> {
    let least = found.iter().map(|c| c.distance).min_by(f64::total_cmp)?;
    let pool: Vec<&Candidate> = found
        .iter()
        .filter(|c| c.distance <= least + tol.linear)
        .collect();
    let pick = pool.iter().copied().min_by(|x, y| {
        let (kx, ky) = (x.key(), y.key());
        kx.0.cmp(&ky.0)
            .then(kx.1.cmp(&ky.1))
            .then(kx.2.cmp(&ky.2))
            .then(kx.3.cmp(&ky.3))
            .then_with(|| {
                kx.4.iter()
                    .zip(&ky.4)
                    .map(|(p, q)| p.total_cmp(q))
                    .find(|o| o.is_ne())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    })?;
    let unique = pool.iter().all(|c| {
        !c.continuum
            && (c.a.point - pick.a.point).norm() <= tol.linear
            && (c.b.point - pick.b.point).norm() <= tol.linear
    });
    Some(Distance {
        distance: pick.distance,
        a: pick.a.foot(),
        b: pick.b.foot(),
        unique,
        contained: false,
    })
}

/// A geometry error of a pair of entities: no formula for their carriers
/// is the operation's own `Unsupported`, naming both.
fn geometry(e: GeomError, a: Shape, b: Shape) -> OpError {
    match e {
        GeomError::Unsupported { a: ka, b: kb } => OpError::Unsupported {
            a: (ka, a),
            b: (kb, b),
        },
        other => fault_of(other),
    }
}

/// `t` or its translate by a period that is in `range`.
fn in_range(curve: &Curve, range: Interval, t: f64) -> Option<f64> {
    shifts(curve.period())
        .map(|d| t + d)
        .find(|t| range.contains(*t))
}

fn vertex_end(m: &Model, v: VertexId) -> Result<End, OpError> {
    Ok(End {
        point: m.vertex(v)?.point(),
        shape: Some(Shape::new(v, Orientation::Forward)),
        param: FootParam::None,
    })
}

fn edge_shape(e: EdgeId) -> Shape {
    Shape::new(e, Orientation::Forward)
}

/// The edges of a face, once each, in id order.
fn face_edges(m: &Model, f: FaceId) -> Result<Vec<EdgeId>, OpError> {
    let mut edges: Vec<EdgeId> = m
        .face(f)?
        .loops()
        .iter()
        .flat_map(|l| l.coedges())
        .map(|c| c.edge())
        .collect();
    edges.sort_unstable();
    edges.dedup();
    Ok(edges)
}

/// The tolerance a pair of entities is measured at: the larger of theirs.
fn tolerance_of(m: &Model, linear: f64) -> Tolerance {
    Tolerance::new(linear, m.precision().angular_tolerance)
}

/// `src` against an edge or a face.
fn point_vs(
    m: &Model,
    meter: &mut Meter,
    src: End,
    other: Entity,
    out: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    match other {
        Entity::Edge(e) => edge_candidates(m, meter, src, e, out),
        Entity::Face(f, o) => face_candidates(m, meter, src, f, o, out),
    }
}

/// The ends of `e`, and every foot of `src` on its curve inside its range.
fn edge_candidates(
    m: &Model,
    meter: &mut Meter,
    src: End,
    e: EdgeId,
    out: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    meter.tick()?;
    let edge = m.edge(e)?;
    for v in [edge.start(), edge.end()] {
        let at = vertex_end(m, v)?;
        out.push(Candidate {
            distance: (src.point - at.point).norm(),
            a: src,
            b: at,
            continuum: false,
        });
    }
    let Some((curve_id, range)) = edge.curve() else {
        return Ok(());
    };
    let curve = m.curve(curve_id)?;
    let tol = tolerance_of(m, edge.tolerance());
    let feet = curve.feet(src.point, tol, meter).map_err(fault_of)?;
    for (list, continuum) in [(&feet.isolated, false), (&feet.continua, true)] {
        for foot in list {
            let Some(t) = in_range(curve, range, foot.t) else {
                continue;
            };
            out.push(Candidate {
                distance: foot.distance,
                a: src,
                b: End {
                    point: foot.point,
                    shape: Some(edge_shape(e)),
                    param: FootParam::Edge(t),
                },
                continuum,
            });
        }
    }
    Ok(())
}

/// The feet of `src` on `f`'s surface inside its trim, and its edges.
fn face_candidates(
    m: &Model,
    meter: &mut Meter,
    src: End,
    f: FaceId,
    orientation: Orientation,
    out: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    meter.tick()?;
    let face = m.face(f)?;
    let surface = m.surface(face.surface())?;
    let precision = m.precision();
    let tol = tolerance_of(m, face.tolerance());
    let domain = FaceDomain::of(m, f, precision.parametric_tolerance)?;
    let feet = surface.feet(src.point, tol, meter).map_err(fault_of)?;
    for (list, continuum) in [(&feet.isolated, false), (&feet.continua, true)] {
        for foot in list {
            if domain.side(foot.uv).0 == Side::Outside {
                continue;
            }
            out.push(Candidate {
                distance: foot.distance,
                a: src,
                b: End {
                    point: foot.point,
                    shape: Some(Shape::new(f, orientation)),
                    param: FootParam::Face(foot.uv),
                },
                continuum,
            });
        }
    }
    for e in face_edges(m, f)? {
        edge_candidates(m, meter, src, e, out)?;
    }
    Ok(())
}

/// Two entities, an edge or a face each: the recursion on boundaries.
/// Between two entities the minimum is the least of the carriers'
/// stationary pairs that lie inside both trims and of each operand's
/// boundary against the other (ADR-0054).
fn entity_vs(
    m: &Model,
    meter: &mut Meter,
    a: Entity,
    b: Entity,
    out: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    match (a, b) {
        (Entity::Edge(x), Entity::Edge(y)) => edge_edge(m, meter, x, y, out),
        (Entity::Edge(e), Entity::Face(f, o)) => edge_face(m, meter, e, f, o, out),
        (Entity::Face(f, o), Entity::Edge(e)) => {
            let mut swapped = Vec::new();
            edge_face(m, meter, e, f, o, &mut swapped)?;
            out.extend(swapped.into_iter().map(Candidate::swapped));
            Ok(())
        }
        (Entity::Face(f, of), Entity::Face(g, og)) => face_face(m, meter, (f, of), (g, og), out),
    }
}

fn swapped_into(from: Vec<Candidate>, out: &mut Vec<Candidate>) {
    out.extend(from.into_iter().map(Candidate::swapped));
}

fn edge_edge(
    m: &Model,
    meter: &mut Meter,
    ea: EdgeId,
    eb: EdgeId,
    out: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    let (xa, xb) = (m.edge(ea)?, m.edge(eb)?);
    for v in [xa.start(), xa.end()] {
        edge_candidates(m, meter, vertex_end(m, v)?, eb, out)?;
    }
    let mut back = Vec::new();
    for v in [xb.start(), xb.end()] {
        edge_candidates(m, meter, vertex_end(m, v)?, ea, &mut back)?;
    }
    swapped_into(back, out);
    let (Some((ca, ra)), Some((cb, rb))) = (xa.curve(), xb.curve()) else {
        return Ok(());
    };
    let (curve_a, curve_b) = (m.curve(ca)?, m.curve(cb)?);
    let tol = tolerance_of(m, xa.tolerance().max(xb.tolerance()));
    let extrema = extrema_curve_curve(curve_a, curve_b, tol, meter)
        .map_err(|e| geometry(e, edge_shape(ea), edge_shape(eb)))?;
    for (list, continuum) in [(&extrema.isolated, false), (&extrema.continua, true)] {
        for pair in list {
            let (Some(ta), Some(tb)) = (
                in_range(curve_a, ra, pair.ta),
                in_range(curve_b, rb, pair.tb),
            ) else {
                continue;
            };
            out.push(Candidate {
                distance: pair.distance,
                a: End {
                    point: pair.a,
                    shape: Some(edge_shape(ea)),
                    param: FootParam::Edge(ta),
                },
                b: End {
                    point: pair.b,
                    shape: Some(edge_shape(eb)),
                    param: FootParam::Edge(tb),
                },
                continuum,
            });
        }
    }
    Ok(())
}

fn edge_face(
    m: &Model,
    meter: &mut Meter,
    e: EdgeId,
    f: FaceId,
    orientation: Orientation,
    out: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    let edge = m.edge(e)?;
    for v in [edge.start(), edge.end()] {
        face_candidates(m, meter, vertex_end(m, v)?, f, orientation, out)?;
    }
    for fe in face_edges(m, f)? {
        edge_edge(m, meter, e, fe, out)?;
    }
    let Some((curve_id, range)) = edge.curve() else {
        return Ok(());
    };
    let face = m.face(f)?;
    let curve = m.curve(curve_id)?;
    let surface = m.surface(face.surface())?;
    let domain = FaceDomain::of(m, f, m.precision().parametric_tolerance)?;
    let tol = tolerance_of(m, edge.tolerance().max(face.tolerance()));
    let extrema = extrema_curve_surface(curve, surface, tol, meter)
        .map_err(|err| geometry(err, edge_shape(e), Shape::new(f, orientation)))?;
    for (list, continuum) in [(&extrema.isolated, false), (&extrema.continua, true)] {
        for pair in list {
            let Some(t) = in_range(curve, range, pair.t) else {
                continue;
            };
            if domain.side(pair.uv).0 == Side::Outside {
                continue;
            }
            out.push(Candidate {
                distance: pair.distance,
                a: End {
                    point: pair.curve_point,
                    shape: Some(edge_shape(e)),
                    param: FootParam::Edge(t),
                },
                b: End {
                    point: pair.surface_point,
                    shape: Some(Shape::new(f, orientation)),
                    param: FootParam::Face(pair.uv),
                },
                continuum,
            });
        }
    }
    Ok(())
}

fn face_face(
    m: &Model,
    meter: &mut Meter,
    (fa, oa): (FaceId, Orientation),
    (fb, ob): (FaceId, Orientation),
    out: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    for e in face_edges(m, fa)? {
        edge_face(m, meter, e, fb, ob, out)?;
    }
    let mut back = Vec::new();
    for e in face_edges(m, fb)? {
        edge_face(m, meter, e, fa, oa, &mut back)?;
    }
    swapped_into(back, out);

    let precision = m.precision();
    let (face_a, face_b) = (m.face(fa)?, m.face(fb)?);
    let (surface_a, surface_b) = (m.surface(face_a.surface())?, m.surface(face_b.surface())?);
    let da = FaceDomain::of(m, fa, precision.parametric_tolerance)?;
    let db = FaceDomain::of(m, fb, precision.parametric_tolerance)?;
    let (sa, sb) = (Shape::new(fa, oa), Shape::new(fb, ob));
    let tol = tolerance_of(m, face_a.tolerance().max(face_b.tolerance()));
    let extrema = extrema_surface_surface(surface_a, surface_b, tol, meter)
        .map_err(|e| geometry(e, sa, sb))?;
    for (list, continuum) in [(&extrema.isolated, false), (&extrema.continua, true)] {
        for pair in list {
            if da.side(pair.uv_a).0 == Side::Outside || db.side(pair.uv_b).0 == Side::Outside {
                continue;
            }
            out.push(Candidate {
                distance: pair.distance,
                a: End {
                    point: pair.point_a,
                    shape: Some(sa),
                    param: FootParam::Face(pair.uv_a),
                },
                b: End {
                    point: pair.point_b,
                    shape: Some(sb),
                    param: FootParam::Face(pair.uv_b),
                },
                continuum,
            });
        }
    }

    // Contact. A section that crosses a boundary of either face is a
    // meeting of an edge with the other face, found above. What the
    // boundaries cannot see is a section lying inside both faces: a closed
    // loop (a pin through a plate) or an isolated touching point. One
    // point of each decides it, since a loop that does not cross a
    // boundary is wholly inside or wholly outside.
    let (Some(box_a), Some(box_b)) = (da.bounds(), db.bounds()) else {
        return Ok(());
    };
    if box_a.distance(&box_b) > tol.linear {
        return Ok(());
    }
    let within = Aabb {
        min: [0, 1, 2].map(|i| box_a.min[i].max(box_b.min[i]) - tol.linear),
        max: [0, 1, 2].map(|i| box_a.max[i].min(box_b.max[i]) + tol.linear),
    };
    let meets = intersect_surfaces(surface_a, surface_b, &within, tol, meter)
        .map_err(|e| geometry(e, sa, sb))?;
    let mut reps: Vec<Point3> = meets.points().iter().map(|p| p.point).collect();
    reps.extend(
        meets
            .curves()
            .iter()
            .filter(|c| c.curve.period().is_some())
            .map(|c| c.curve.point(0.0)),
    );
    for point in reps {
        let (Ok(pa), Ok(pb)) = (surface_a.project(point), surface_b.project(point)) else {
            continue;
        };
        if da.side(pa.uv).0 == Side::Outside || db.side(pb.uv).0 == Side::Outside {
            continue;
        }
        out.push(Candidate {
            distance: 0.0,
            a: End {
                point,
                shape: Some(sa),
                param: FootParam::Face(pa.uv),
            },
            b: End {
                point,
                shape: Some(sb),
                param: FootParam::Face(pb.uv),
            },
            continuum: false,
        });
    }
    Ok(())
}

fn fault_classify(e: arris_check::classify::ClassifyError) -> OpError {
    OpError::Internal(Fault::Classify(e))
}

/// Whether `p` is strictly inside the solid `body`.
fn inside(m: &Model, body: Body, p: Point3) -> Result<bool, OpError> {
    let classified = Classifier::of_body(m, body)
        .and_then(|c| c.classify(p))
        .map_err(fault_classify)?;
    Ok(classified == Classification::Inside)
}

/// The vertices of an edge, a face or a solid, in id order, once each.
fn vertices_of(m: &Model, r: &Resolved) -> Result<Vec<VertexId>, OpError> {
    let mut edges: Vec<EdgeId> = Vec::new();
    match r {
        Resolved::Point(..) => {}
        Resolved::Entity(Entity::Edge(e)) => edges.push(*e),
        Resolved::Entity(Entity::Face(f, _)) => edges = face_edges(m, *f)?,
        Resolved::Body(body) => edges = m.closure(*body)?.edges,
    }
    let mut vertices: Vec<VertexId> = Vec::new();
    for e in edges {
        let edge = m.edge(e)?;
        vertices.extend([edge.start(), edge.end()]);
    }
    vertices.sort_unstable();
    vertices.dedup();
    Ok(vertices)
}

/// A solid against `src`: the faces nearest first, stopping at the first
/// whose box is farther than the best so far.
fn body_candidates(
    m: &Model,
    body: Body,
    src: End,
    meter: &mut Meter,
    out: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    let uses = super::face_uses(m, body)?;
    let tree = FaceTree::of_body(m, body)?;
    let region = Aabb::of_point(src.point);
    let mut best = f64::INFINITY;
    for (box_distance, f) in tree.nearest(&region, f64::INFINITY) {
        if box_distance > best {
            break;
        }
        let Some(&orientation) = uses.get(&f) else {
            continue;
        };
        let before = out.len();
        face_candidates(m, meter, src, f, orientation, out)?;
        for c in &out[before..] {
            best = best.min(c.distance);
        }
    }
    Ok(())
}

/// The parts a solid is measured against: its faces, or the one entity,
/// each with its box (`None` when no box bounds it).
fn parts_of(m: &Model, other: &Resolved) -> Result<Vec<(Entity, Option<Aabb>)>, OpError> {
    let ptol = m.precision().parametric_tolerance;
    let face_box = |f: FaceId| FaceDomain::of(m, f, ptol).ok().and_then(|d| d.bounds());
    match other {
        Resolved::Entity(e @ Entity::Face(f, _)) => Ok(vec![(*e, face_box(*f))]),
        Resolved::Entity(e @ Entity::Edge(id)) => {
            let bounds = match m.edge(*id)?.curve() {
                Some((curve, range)) => m.curve(curve)?.bounds(range),
                None => None,
            };
            Ok(vec![(*e, bounds)])
        }
        Resolved::Body(body) => Ok(super::face_uses(m, *body)?
            .into_iter()
            .map(|(f, o)| (Entity::Face(f, o), face_box(f)))
            .collect()),
        Resolved::Point(..) => Ok(Vec::new()),
    }
}

/// A solid against an edge, a face or another solid (ADR-0054): the pairs
/// of one face of the solid and one part of the other, nearest first by box
/// distance, each by the boundary recursion, which includes their contact.
/// A pair whose boxes are farther apart than the best answer so far plus
/// the linear tolerance cannot matter and is skipped. A pair no formula
/// covers is held back and refuses only if it was not skipped in the end.
fn body_vs(
    m: &Model,
    body: Body,
    other: &Resolved,
    meter: &mut Meter,
    tol: Tolerance,
    out: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    let uses = super::face_uses(m, body)?;
    let tree = FaceTree::of_body(m, body)?;
    let mut best = f64::INFINITY;
    let mut held: Vec<(f64, OpError)> = Vec::new();
    for (part, bounds) in parts_of(m, other)? {
        // A part no box bounds is within reach of every face.
        let reach = match bounds {
            Some(b) => tree.nearest(&b, best + tol.linear),
            None => tree
                .nearest(&Aabb::of_point(Point3::origin()), f64::INFINITY)
                .into_iter()
                .map(|(_, f)| (0.0, f))
                .collect(),
        };
        for (box_distance, f) in reach {
            if box_distance > best + tol.linear {
                break;
            }
            let Some(&orientation) = uses.get(&f) else {
                continue;
            };
            meter.tick()?;
            let mut pair = Vec::new();
            match entity_vs(m, meter, Entity::Face(f, orientation), part, &mut pair) {
                Ok(()) => {}
                Err(e @ OpError::Unsupported { .. }) => {
                    held.push((box_distance, e));
                    continue;
                }
                Err(e) => return Err(e),
            }
            for c in &pair {
                best = best.min(c.distance);
            }
            out.append(&mut pair);
        }
    }
    held.sort_by(|x, y| x.0.total_cmp(&y.0));
    match held.into_iter().find(|(d, _)| *d <= best + tol.linear) {
        Some((_, e)) => Err(e),
        None => Ok(()),
    }
}
