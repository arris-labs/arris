//! Point classification against a body (`docs/ARCHITECTURE.md` §The
//! checker, ADR-0004): inside, outside, or on a named entity.
//!
//! This is B1's ray cast, made public and complete. The row that proves a
//! shell nesting and the predicate that decides which piece of a split
//! face a boolean keeps are one piece of code, so the two can never
//! disagree about a point — which is the consistency a boolean needs from
//! its classifier, not merely correctness case by case.

use std::collections::BTreeMap;
use std::fmt;

use arris_geom::region2::Side;
use arris_geom::{Curve, CurveSurfaceIntersection, GeomError, intersect_curve_surface};
use arris_math::{Point3, Precision, Tolerance, UnitVec3, Vec3};
use arris_topo::{Body, FaceId, Model, NotFound, Orientation, Shape};

use crate::domain::{FaceDomain, boundary_entity};

/// The directions a containment ray is tried in, in order: the axes
/// first, then directions no two faces of an axis-aligned body share a
/// plane with. The first that meets no face boundary within tolerance
/// decides, so the answer is the same on every platform.
pub(crate) const RAY_DIRECTIONS: [[f64; 3]; 8] = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 1.0, 1.0],
    [1.0, 2.0, 3.0],
    [-3.0, 1.0, 2.0],
    [2.0, -3.0, 1.0],
    [1.0, -2.0, -5.0],
];

/// Where a point lies with respect to a body.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Classification {
    /// Strictly inside the material.
    Inside,
    /// Strictly outside it.
    Outside,
    /// On the boundary, within the tolerance of the entity named: the
    /// vertex, else the edge, else the face — the most specific one the
    /// point is within, so a point at a corner is `On` the vertex and
    /// never one of the edges that meet there.
    On(Shape),
}

/// Why a point could not be classified. Never a guess: an operation that
/// gets one of these reports it with the entities it was about.
#[derive(Debug, Clone, PartialEq)]
pub enum ClassifyError {
    /// An id the body reaches does not resolve.
    Unresolved(NotFound),
    /// A ray against one of the body's surfaces has no closed form: a
    /// NURBS. A line meets every analytic surface by closed form.
    Geometry(GeomError),
    /// Every one of the eight ray directions was abandoned: each grazed a
    /// face's boundary, touched a surface tangentially or lay in one.
    Undecided {
        /// The body.
        body: Body,
        /// The point.
        point: Point3,
    },
}

impl fmt::Display for ClassifyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ClassifyError::Unresolved(e) => write!(f, "{e}"),
            ClassifyError::Geometry(e) => write!(f, "{e}"),
            ClassifyError::Undecided { body, point } => write!(
                f,
                "no ray from ({}, {}, {}) decides {body}: every direction grazed it",
                point.x, point.y, point.z
            ),
        }
    }
}

impl std::error::Error for ClassifyError {}

impl From<NotFound> for ClassifyError {
    fn from(e: NotFound) -> Self {
        ClassifyError::Unresolved(e)
    }
}

impl From<GeomError> for ClassifyError {
    fn from(e: GeomError) -> Self {
        ClassifyError::Geometry(e)
    }
}

/// Where `point` lies with respect to `body`: `On` the most specific
/// entity it is within the tolerance of, and otherwise `Inside` or
/// `Outside` by the parity of the crossings of a ray from it.
///
/// The boundary test comes first and is by the entities' *own*
/// tolerances (`docs/DATA-MODEL.md` §Tolerances): a vertex within its
/// tolerance, then an edge within its, then a face within its. Only a
/// point that is on nothing is cast for, and a direction whose ray grazes
/// a face's boundary, touches a surface tangentially or lies in one — by
/// the face's own tolerance — is abandoned for the next; all eight
/// abandoned is
/// [`ClassifyError::Undecided`], never a guess.
///
/// Errors: an id that does not resolve, a surface a ray has no closed
/// form against (a NURBS), or
/// `Undecided`.
///
/// ```
/// use arris_check::classify::{Classification, classify_point};
/// use arris_debug::sample;
/// use arris_topo::Model;
/// use arris_math::Point3;
///
/// let mut m = Model::default();
/// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
/// assert_eq!(classify_point(&m, body, Point3::new(0.0, 0.0, 6.0))?, Classification::Inside);
/// assert_eq!(classify_point(&m, body, Point3::new(9.0, 0.0, 6.0))?, Classification::Outside);
/// let on = classify_point(&m, body, Point3::new(4.0, 0.0, 6.0))?;
/// assert!(matches!(on, Classification::On(_)), "on the wall");
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn classify_point(
    model: &Model,
    body: Body,
    point: Point3,
) -> Result<Classification, ClassifyError> {
    Classifier::of_body(model, body)?.classify(point)
}

/// A body's faces read once for classifying many points: what
/// [`classify_point`] builds for one point and B1 builds over one shell.
/// Each face is a [`FaceDomain`] at the model's parametric tolerance.
///
/// Guarantees: [`Classifier::classify`] answers every point exactly as
/// [`classify_point`] does for the same body, since that function is this
/// type built and asked once.
pub struct Classifier<'m> {
    model: &'m Model,
    precision: Precision,
    body: Body,
    faces: Vec<FaceId>,
    domains: BTreeMap<FaceId, FaceDomain<'m>>,
}

impl<'m> Classifier<'m> {
    /// A classifier over every face of `body`'s closure. Errors: the body
    /// does not resolve.
    ///
    /// ```
    /// use arris_check::classify::{Classification, Classifier};
    /// use arris_debug::sample;
    /// use arris_topo::Model;
    /// use arris_math::Point3;
    ///
    /// let mut m = Model::default();
    /// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
    /// let classifier = Classifier::of_body(&m, body)?;
    /// for z in [1.0, 6.0, 11.0] {
    ///     assert_eq!(classifier.classify(Point3::new(0.0, 0.0, z))?, Classification::Inside);
    /// }
    /// assert_eq!(classifier.classify(Point3::new(0.0, 0.0, 13.0))?, Classification::Outside);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn of_body(model: &'m Model, body: Body) -> Result<Self, ClassifyError> {
        let faces = model.closure(body)?.faces;
        Ok(Self::over(model, body, faces))
    }

    /// Where `point` lies with respect to the body: [`classify_point`]'s
    /// answer, by its rules, without reading the faces again.
    ///
    /// Errors: as [`classify_point`].
    pub fn classify(&self, point: Point3) -> Result<Classification, ClassifyError> {
        if let Some(shape) = self.on(point)? {
            return Ok(Classification::On(shape));
        }
        match self.contains(point)? {
            Some(true) => Ok(Classification::Inside),
            Some(false) => Ok(Classification::Outside),
            None => Err(ClassifyError::Undecided {
                body: self.body,
                point,
            }),
        }
    }

    /// A classifier over `faces` of `body`, their domains read at the
    /// model's parametric tolerance, as the checker's `Full` rows read
    /// them. A face whose domain does not resolve has none and is
    /// `Outside` everywhere; the ray cast reports it by finding no
    /// crossing, never by panicking.
    pub(crate) fn over(model: &'m Model, body: Body, faces: Vec<FaceId>) -> Self {
        let precision = model.precision();
        let tolerance = precision.parametric_tolerance;
        let domains = faces
            .iter()
            .filter_map(|&id| Some((id, FaceDomain::of(model, id, tolerance).ok()?)))
            .collect();
        Classifier {
            model,
            precision,
            body,
            faces,
            domains,
        }
    }

    /// Where `uv` lies with respect to `face`'s loops, in whichever
    /// translate they are written ([`FaceDomain::side`]).
    fn side(&self, face: FaceId, uv: arris_math::Point2) -> Side {
        self.domains
            .get(&face)
            .map_or(Side::Outside, |d| d.side(uv).0)
    }

    /// The entity `point` is on, most specific first, or `None`: a vertex
    /// or edge of any of the faces ([`boundary_entity`]), else a face.
    pub(crate) fn on(&self, point: Point3) -> Result<Option<Shape>, ClassifyError> {
        let model = self.model;
        let edges: Vec<_> = self
            .faces
            .iter()
            .filter_map(|&id| model.face(id).ok())
            .flat_map(|face| face.loops().iter().flat_map(|l| l.coedges()))
            .map(|c| c.edge())
            .collect();
        if let Some(shape) = boundary_entity(model, edges, point)? {
            return Ok(Some(shape));
        }
        for &id in &self.faces {
            let face = model.face(id)?;
            let surface = model.surface(face.surface())?;
            let Ok(projection) = surface.project(point) else {
                continue;
            };
            if projection.distance <= face.tolerance()
                && self.side(id, projection.uv) != Side::Outside
            {
                return Ok(Some(Shape::new(id, Orientation::Forward)));
            }
        }
        Ok(None)
    }

    /// Whether `point` is inside the closed surface the faces make, by
    /// the parity of the crossings of a ray from it; `None` when every
    /// one of the eight directions was abandoned, which the caller that
    /// knows the body turns into [`ClassifyError::Undecided`]. Errors:
    /// an id that does not resolve, or a ray against a surface with no
    /// closed form.
    pub(crate) fn contains(&self, point: Point3) -> Result<Option<bool>, ClassifyError> {
        let model = self.model;
        let angular = self.precision.angular_tolerance;
        'direction: for d in RAY_DIRECTIONS {
            let ray = Curve::Line {
                origin: point,
                direction: UnitVec3::new_normalize(Vec3::new(d[0], d[1], d[2])),
            };
            let mut crossings = 0usize;
            for &id in &self.faces {
                let face = model.face(id)?;
                let surface = model.surface(face.surface())?;
                // The ray meets, grazes or lies in the surface by the
                // face's own tolerance (`docs/DATA-MODEL.md` §Tolerances).
                let tolerance = Tolerance::new(face.tolerance(), angular);
                let hits = match intersect_curve_surface(
                    &ray,
                    surface,
                    tolerance,
                    &mut arris_math::Meter::default(),
                )? {
                    CurveSurfaceIntersection::Points(hits) => hits,
                    CurveSurfaceIntersection::Coincident => continue 'direction,
                };
                for hit in hits {
                    if hit.t.abs() <= face.tolerance() {
                        // The ray starts on this surface. On the face
                        // itself the point has no parity to take; off
                        // it, the surface is merely passed through at
                        // the origin, which is no crossing.
                        match self.side(id, hit.uv) {
                            Side::Outside => continue,
                            Side::Inside | Side::Boundary => continue 'direction,
                        }
                    }
                    if hit.t < 0.0 {
                        continue;
                    }
                    match self.side(id, hit.uv) {
                        Side::Inside if hit.tangent => continue 'direction,
                        Side::Inside => crossings += 1,
                        Side::Boundary => continue 'direction,
                        Side::Outside => {}
                    }
                }
            }
            return Ok(Some(crossings % 2 == 1));
        }
        Ok(None)
    }
}
