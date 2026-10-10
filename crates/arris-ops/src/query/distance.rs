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
use arris_math::{Aabb, Control, Meter, Point2, Point3, Tolerance};
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
    /// `true` when one operand lies in the interior of a solid operand.
    pub contained: bool,
}

/// One place the distance is attained or stationary, before the choice.
#[derive(Debug, Clone, Copy)]
struct Candidate {
    distance: f64,
    /// The point on the entity.
    point: Point3,
    shape: Shape,
    param: FootParam,
    /// Found as the representative of a continuum of equal feet.
    continuum: bool,
}

impl Candidate {
    /// Vertex before edge before face: the most specific entity a point
    /// within the tolerance of several is on.
    fn rank(&self) -> u8 {
        match self.shape.id {
            EntityId::Vertex(_) => 0,
            EntityId::Edge(_) => 1,
            EntityId::Face(_) | EntityId::Shell(_) | EntityId::Body(_) => 2,
        }
    }

    fn key(&self) -> (u8, EntityId, [f64; 2]) {
        let param = match self.param {
            FootParam::None => [0.0, 0.0],
            FootParam::Edge(t) => [t, 0.0],
            FootParam::Face(uv) => [uv.x, uv.y],
        };
        (self.rank(), self.shape.id, param)
    }
}

/// The point-set an operand is, resolved.
enum Resolved {
    Point(Point3, Option<Shape>),
    Edge(EdgeId, Shape),
    Face(FaceId, Shape),
    Body(Body),
}

fn not_measurable(shape: Shape) -> OpError {
    OpError::Degenerate {
        entities: vec![shape],
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
                m.edge(e)?;
                Ok(Resolved::Edge(e, shape))
            }
            EntityId::Face(f) => {
                m.face(f)?;
                Ok(Resolved::Face(f, shape))
            }
            EntityId::Shell(_) => Err(not_measurable(shape)),
            EntityId::Body(b) => {
                if m.body(b)?.kind() != BodyKind::Solid {
                    return Err(not_measurable(shape));
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
/// walk of its [`FaceTree`]. The `control` is polled once per entity pair.
///
/// Errors: [`OpError::InvalidInput`] when a body fails the checker (debug
/// builds, and release with `paranoid`); [`OpError::NotFound`] for an id
/// that does not resolve; [`OpError::Degenerate`] with
/// [`QueryReason::NotMeasurable`] for a shell or a body that is not a
/// solid, with [`QueryReason::DegenerateEdge`] for a degenerate edge
/// operand, and with `InputReason::NonFinite` for a non-finite point;
/// [`OpError::Unsupported`] for a pair of entities the kernel has no
/// formula for; [`OpError::Interrupted`] when the control stops it.
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
/// ```
pub fn distance(
    m: &Model,
    a: Operand,
    b: Operand,
    control: &Control<'_>,
) -> Result<Distance, OpError> {
    let (ra, rb) = (resolve(m, a)?, resolve(m, b)?);
    for r in [&ra, &rb] {
        if let Resolved::Body(body) = r {
            crate::verify_input(m, *body)?;
        }
    }
    let mut meter = Meter::new(control);
    match (ra, rb) {
        (Resolved::Point(p, shape), other) => {
            let at = shape_of(&other);
            let mut d = point_against(m, p, at, other, &mut meter)?;
            d.a = Foot {
                point: p,
                shape,
                param: FootParam::None,
            };
            Ok(d)
        }
        (other, Resolved::Point(p, shape)) => {
            let at = shape_of(&other);
            let mut d = point_against(m, p, at, other, &mut meter)?;
            std::mem::swap(&mut d.a, &mut d.b);
            d.b = Foot {
                point: p,
                shape,
                param: FootParam::None,
            };
            Ok(d)
        }
        (ra, rb) => Err(unsupported_pair(&ra, &rb)),
    }
}

fn shape_of(r: &Resolved) -> Option<Shape> {
    match r {
        Resolved::Point(_, shape) => *shape,
        Resolved::Edge(_, s) | Resolved::Face(_, s) => Some(*s),
        Resolved::Body(body) => Some(Shape::new(body.id, body.orientation)),
    }
}

/// Entity pairs beyond a point wait for the recursion over two entities.
fn unsupported_pair(a: &Resolved, b: &Resolved) -> OpError {
    OpError::Degenerate {
        entities: [a, b].into_iter().filter_map(shape_of).collect(),
        reason: Reason::Query(QueryReason::NotMeasurable),
    }
}

/// `p` against one resolved operand: the closest point on it.
fn point_against(
    m: &Model,
    p: Point3,
    shape: Option<Shape>,
    other: Resolved,
    meter: &mut Meter,
) -> Result<Distance, OpError> {
    let tol = m.precision().tolerance();
    let mut found: Vec<Candidate> = Vec::new();
    let mut contained = false;
    match other {
        Resolved::Point(q, shape) => {
            // A vertex (or a point) against a vertex: the segment.
            return Ok(Distance {
                distance: (p - q).norm(),
                a: Foot {
                    point: p,
                    shape: None,
                    param: FootParam::None,
                },
                b: Foot {
                    point: q,
                    shape,
                    param: FootParam::None,
                },
                unique: true,
                contained: false,
            });
        }
        Resolved::Edge(e, shape) => {
            if m.edge(e)?.curve().is_none() {
                return Err(OpError::Degenerate {
                    entities: vec![shape],
                    reason: Reason::Query(QueryReason::DegenerateEdge),
                });
            }
            edge_candidates(m, e, p, meter, &mut found)?;
        }
        Resolved::Face(f, shape) => {
            face_candidates(m, f, shape.orientation, p, meter, &mut found)?;
        }
        Resolved::Body(body) => {
            body_candidates(m, body, p, meter, &mut found, &mut contained)?;
        }
    }
    if contained {
        return Ok(Distance {
            distance: 0.0,
            a: Foot {
                point: p,
                shape: None,
                param: FootParam::None,
            },
            b: Foot {
                point: p,
                shape: None,
                param: FootParam::None,
            },
            unique: true,
            contained: true,
        });
    }
    // An operand with no vertex and no foot in its trim has no point to
    // measure to.
    choose(&found, tol).ok_or_else(|| OpError::Degenerate {
        entities: shape.into_iter().collect(),
        reason: Reason::Query(QueryReason::NotMeasurable),
    })
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
            .then(kx.2[0].total_cmp(&ky.2[0]))
            .then(kx.2[1].total_cmp(&ky.2[1]))
    })?;
    let unique = pool
        .iter()
        .all(|c| !c.continuum && (c.point - pick.point).norm() <= tol.linear);
    Some(Distance {
        distance: pick.distance,
        a: Foot {
            point: Point3::origin(),
            shape: None,
            param: FootParam::None,
        },
        b: Foot {
            point: pick.point,
            shape: Some(pick.shape),
            param: pick.param,
        },
        unique,
        contained: false,
    })
}

fn vertex_candidate(m: &Model, v: VertexId, p: Point3) -> Result<Candidate, OpError> {
    let at = m.vertex(v)?.point();
    Ok(Candidate {
        distance: (p - at).norm(),
        point: at,
        shape: Shape::new(v, Orientation::Forward),
        param: FootParam::None,
        continuum: false,
    })
}

/// The ends of `e`, and every foot of `p` on its curve inside its range.
fn edge_candidates(
    m: &Model,
    e: EdgeId,
    p: Point3,
    meter: &mut Meter,
    out: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    meter.tick()?;
    let edge = m.edge(e)?;
    out.push(vertex_candidate(m, edge.start(), p)?);
    out.push(vertex_candidate(m, edge.end(), p)?);
    let Some((curve_id, range)) = edge.curve() else {
        return Ok(());
    };
    let curve = m.curve(curve_id)?;
    let tol = Tolerance::new(edge.tolerance(), m.precision().angular_tolerance);
    let feet = curve.feet(p, tol, meter).map_err(fault_of)?;
    for (list, continuum) in [(&feet.isolated, false), (&feet.continua, true)] {
        for foot in list {
            let Some(t) = shifts(curve.period())
                .map(|d| foot.t + d)
                .find(|t| range.contains(*t))
            else {
                continue;
            };
            out.push(Candidate {
                distance: foot.distance,
                point: foot.point,
                shape: Shape::new(e, Orientation::Forward),
                param: FootParam::Edge(t),
                continuum,
            });
        }
    }
    Ok(())
}

/// The feet of `p` on `f`'s surface inside its trim, and its edges.
fn face_candidates(
    m: &Model,
    f: FaceId,
    orientation: Orientation,
    p: Point3,
    meter: &mut Meter,
    out: &mut Vec<Candidate>,
) -> Result<(), OpError> {
    meter.tick()?;
    let face = m.face(f)?;
    let surface = m.surface(face.surface())?;
    let precision = m.precision();
    let tol = Tolerance::new(face.tolerance(), precision.angular_tolerance);
    let domain = FaceDomain::of(m, f, precision.parametric_tolerance)?;
    let feet = surface.feet(p, tol, meter).map_err(fault_of)?;
    for (list, continuum) in [(&feet.isolated, false), (&feet.continua, true)] {
        for foot in list {
            if domain.side(foot.uv).0 == Side::Outside {
                continue;
            }
            out.push(Candidate {
                distance: foot.distance,
                point: foot.point,
                shape: Shape::new(f, orientation),
                param: FootParam::Face(foot.uv),
                continuum,
            });
        }
    }
    let mut edges: Vec<EdgeId> = face
        .loops()
        .iter()
        .flat_map(|l| l.coedges())
        .map(|c| c.edge())
        .collect();
    edges.sort_unstable();
    edges.dedup();
    for e in edges {
        edge_candidates(m, e, p, meter, out)?;
    }
    Ok(())
}

/// A solid against `p`: containment first, then the faces nearest first,
/// stopping at the first whose box is farther than the best so far.
fn body_candidates(
    m: &Model,
    body: Body,
    p: Point3,
    meter: &mut Meter,
    out: &mut Vec<Candidate>,
    contained: &mut bool,
) -> Result<(), OpError> {
    let classified = Classifier::of_body(m, body)
        .and_then(|c| c.classify(p))
        .map_err(|e| OpError::Internal(Fault::Classify(e)))?;
    if classified == Classification::Inside {
        *contained = true;
        return Ok(());
    }
    let uses = super::face_uses(m, body)?;
    let tree = FaceTree::of_body(m, body)?;
    let region = Aabb::of_point(p);
    let mut best = f64::INFINITY;
    for (box_distance, f) in tree.nearest(&region, f64::INFINITY) {
        if box_distance > best {
            break;
        }
        let Some(&orientation) = uses.get(&f) else {
            continue;
        };
        let before = out.len();
        face_candidates(m, f, orientation, p, meter, out)?;
        for c in &out[before..] {
            best = best.min(c.distance);
        }
    }
    Ok(())
}
