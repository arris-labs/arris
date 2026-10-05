//! Where a (u, v) point is on a face: its loops as polygons, its (u, v)
//! and 3D boxes, the side of a point in any translate of a periodic
//! surface's domain, and the vertex or edge a point on its boundary is on
//! (`docs/ARCHITECTURE.md` §The checker).
//!
//! This is the one answer the checker's `Full` rows, the classifier, the
//! boolean and tessellation all ask for, so B1 and a boolean cannot
//! disagree about which face a point is inside (ADR-0004). The caller
//! chooses the tolerance the loops are read at: the checker passes the
//! model's parametric tolerance, the boolean a face's own.

use std::collections::BTreeSet;

use arris_geom::Surface;
use arris_geom::region2::{Piece, Polygon2, Side, SideIndex, discretise};
use arris_math::{Aabb, Interval, Point2, Point3, Vec2, is_negligible};
use arris_topo::entity::{Face, Loop};
use arris_topo::{EdgeId, FaceId, Model, NotFound, Orientation, Shape};

/// A loop's pieces in (u, v), in walking order (`Model::loop_pieces`), or
/// `None` when the loop has no coedge or a range is not bounded and
/// increasing: a loop no polygon, winding or integral is taken over,
/// which L1 and E1 report. Errors: a reference does not resolve.
pub(crate) fn bounded_pieces<'m>(
    model: &'m Model,
    l: &Loop,
) -> Result<Option<Vec<Piece<'m>>>, NotFound> {
    let pieces = model.loop_pieces(l)?;
    let bounded = pieces.iter().all(|p| {
        let r = p.range;
        r.lo().is_finite() && r.hi().is_finite() && r.lo() < r.hi()
    });
    Ok((bounded && !pieces.is_empty()).then_some(pieces))
}

/// The offsets a periodic parameter is tried at: nothing, and a period
/// either way; once for a direction without a period. A loop written in
/// the translate next to the one a projection returns is reached by one
/// of them.
pub fn shifts(period: Option<f64>) -> impl Iterator<Item = f64> {
    let p = period.unwrap_or(0.0);
    [0.0, p, -p].into_iter().take(if p == 0.0 { 1 } else { 3 })
}

/// How far a (u, v) step may be in each direction and still be within
/// `tolerance` in 3D at `uv`: the tolerance divided by the surface's
/// speed in that direction, and the tolerance itself where a speed is
/// zero or negligible beside the other (a pole). `docs/DATA-MODEL.md`
/// §Pcurves states the conversion.
///
/// ```
/// use arris_check::domain::bands;
/// use arris_geom::Surface;
/// use arris_math::{Frame, Point2};
///
/// let cylinder = Surface::Cylinder { frame: Frame::world(), radius: 4.0 };
/// let [du, dv] = bands(&cylinder, Point2::new(1.0, 2.0), 1e-6);
/// assert!((du - 0.25e-6).abs() < 1e-18 && dv == 1e-6);
/// ```
pub fn bands(surface: &Surface, uv: Point2, tolerance: f64) -> [f64; 2] {
    let e = surface.eval(uv.x, uv.y);
    let speeds = [e.du.norm(), e.dv.norm()];
    [0, 1].map(|i| {
        if is_negligible(speeds[i], speeds[1 - i]) || speeds[i] == 0.0 {
            tolerance
        } else {
            tolerance / speeds[i]
        }
    })
}

/// [`bands`] in the tighter of the two directions: a (u, v) distance no
/// coarser than `tolerance` in 3D whichever way it is taken.
pub fn band(surface: &Surface, uv: Point2, tolerance: f64) -> f64 {
    let [du, dv] = bands(surface, uv, tolerance);
    du.min(dv)
}

/// The chord a face's loops are discretised at for `tolerance`: the
/// [`band`] at the first point of its first pcurve that resolves, or at
/// the origin of (u, v) for a face with none.
pub fn chord(model: &Model, face: &Face, surface: &Surface, tolerance: f64) -> f64 {
    let at = face
        .loops()
        .iter()
        .flat_map(|l| l.coedges())
        .find_map(|c| model.curve2(c.pcurve()).ok())
        .map_or(Point2::origin(), |p| p.point(0.0));
    band(surface, at, tolerance)
}

/// The vertex, else the edge, among `edges` and their end vertices that
/// `point` is within the entity's own tolerance of: every vertex is tried
/// before any edge, each in id order, so a point at a corner is on the
/// vertex and never on one of the edges meeting there. A closed edge's
/// range may be in any translate of its curve's period. A degenerate
/// edge is answered for by its vertex. `None` when the point is on
/// neither.
///
/// Errors: an edge, vertex or curve that does not resolve.
pub fn boundary_entity(
    model: &Model,
    edges: impl IntoIterator<Item = EdgeId>,
    point: Point3,
) -> Result<Option<Shape>, NotFound> {
    let edges: BTreeSet<EdgeId> = edges.into_iter().collect();
    let mut vertices = BTreeSet::new();
    for &e in &edges {
        let edge = model.edge(e)?;
        vertices.extend([edge.start(), edge.end()]);
    }
    for v in vertices {
        let vertex = model.vertex(v)?;
        if (point - vertex.point()).norm() <= vertex.tolerance() {
            return Ok(Some(Shape::new(v, Orientation::Forward)));
        }
    }
    for e in edges {
        let edge = model.edge(e)?;
        let Some((curve_id, range)) = edge.curve() else {
            continue;
        };
        let curve = model.curve(curve_id)?;
        // A point on the axis of a circle has no nearest parameter; it is
        // a radius away from the curve, so it is not on it.
        let Ok(projection) = curve.project(point) else {
            continue;
        };
        let within = shifts(curve.period()).any(|d| range.contains(projection.t + d));
        if within && projection.distance <= edge.tolerance() {
            return Ok(Some(Shape::new(e, Orientation::Forward)));
        }
    }
    Ok(None)
}

/// A face read once for the question "where is this (u, v) point": its
/// loops as polygons within a chord of the pcurves, its boxes, and the
/// tolerance its boundary band is taken at.
#[derive(Debug, Clone)]
pub struct FaceDomain<'m> {
    face: FaceId,
    surface: &'m Surface,
    tolerance: f64,
    polygons: Vec<Polygon2>,
    index: SideIndex,
    uv_box: Option<[Interval; 2]>,
    bounds: Option<Aabb>,
    edges: Vec<EdgeId>,
}

impl<'m> FaceDomain<'m> {
    /// The domain of `face` at `tolerance`, a 3D length.
    ///
    /// Guarantees: each loop is a polygon within [`chord`] of its pcurves
    /// at `tolerance`, in the translate the loop is written in; a loop
    /// with no coedge or with a range that is not bounded and positive
    /// has no polygon (L1's and E1's to report). The (u, v) box holds the
    /// true boundary: it is the polygons' box grown by their chord
    /// deviation. The 3D box holds the face: its edges' curve boxes, each
    /// grown by the edge's tolerance, and the surface over the (u, v)
    /// box, all grown by the face's own tolerance.
    ///
    /// Errors: the face, its surface, or a pcurve, edge or curve it uses
    /// does not resolve.
    ///
    /// ```
    /// use arris_check::domain::FaceDomain;
    /// use arris_debug::sample;
    /// use arris_topo::Model;
    /// use arris_geom::region2::Side;
    /// use arris_math::{Point2, Vec2};
    /// use core::f64::consts::TAU;
    ///
    /// let mut m = Model::default();
    /// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
    /// let wall = m.faces(body)?[0].id;
    /// let domain = FaceDomain::of(&m, wall, m.precision().parametric_tolerance)?;
    /// assert_eq!(domain.side(Point2::new(1.0, 6.0)), (Side::Inside, Vec2::zeros()));
    /// // A period along is the same point, answered in the loops' translate.
    /// assert_eq!(domain.side(Point2::new(1.0 + TAU, 6.0)), (Side::Inside, Vec2::new(-TAU, 0.0)));
    /// assert_eq!(domain.side(Point2::new(1.0, 13.0)).0, Side::Outside);
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn of(model: &'m Model, face: FaceId, tolerance: f64) -> Result<Self, NotFound> {
        let entity = model.face(face)?;
        let surface = model.surface(entity.surface())?;
        let chord = chord(model, entity, surface, tolerance);
        let mut polygons = Vec::with_capacity(entity.loops().len());
        for l in entity.loops() {
            if let Some(pieces) = bounded_pieces(model, l)? {
                polygons.push(discretise(&pieces, chord));
            }
        }
        let mut lo = Point2::new(f64::INFINITY, f64::INFINITY);
        let mut hi = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for polygon in &polygons {
            let d = polygon.chord_deviation();
            for p in polygon.points() {
                lo = Point2::new(lo.x.min(p.x - d), lo.y.min(p.y - d));
                hi = Point2::new(hi.x.max(p.x + d), hi.y.max(p.y + d));
            }
        }
        let uv_box = match (Interval::new(lo.x, hi.x), Interval::new(lo.y, hi.y)) {
            (Ok(u), Ok(v)) => Some([u, v]),
            _ => None,
        };
        let mut edges = Vec::new();
        let mut seen = BTreeSet::new();
        let mut curves: Option<Aabb> = None;
        let mut bounded = true;
        for c in entity.loops().iter().flat_map(|l| l.coedges()) {
            if seen.insert(c.edge()) {
                edges.push(c.edge());
            }
            let edge = model.edge(c.edge())?;
            let Some((curve, range)) = edge.curve() else {
                continue;
            };
            match model.curve(curve)?.bounds(range) {
                Some(b) => {
                    let b = b.inflated(edge.tolerance());
                    curves = Some(curves.map_or(b, |acc| acc.union(b)));
                }
                None => bounded = false,
            }
        }
        let bounds = match uv_box.and_then(|uv| surface.bounds(uv)) {
            Some(b) if bounded => Some(
                curves
                    .map_or(b, |acc| acc.union(b))
                    .inflated(entity.tolerance()),
            ),
            _ => None,
        };
        let index = SideIndex::new(&polygons);
        Ok(FaceDomain {
            face,
            surface,
            tolerance,
            polygons,
            index,
            uv_box,
            bounds,
            edges,
        })
    }

    /// The face.
    pub fn face(&self) -> FaceId {
        self.face
    }

    /// The face's surface.
    pub fn surface(&self) -> &'m Surface {
        self.surface
    }

    /// The tolerance the domain was read at.
    pub fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// The loops as polygons in (u, v), in stored order, each in the
    /// translate its pcurves are written in.
    pub fn polygons(&self) -> &[Polygon2] {
        &self.polygons
    }

    /// The (u, v) box holding the face's true boundary, or `None` for a
    /// face with no polygon.
    pub fn uv_box(&self) -> Option<[Interval; 2]> {
        self.uv_box
    }

    /// The 3D box holding the face, or `None` when a curve range or the
    /// (u, v) box is not bounded — a face no box rejects.
    pub fn bounds(&self) -> Option<Aabb> {
        self.bounds
    }

    /// The edges of the loops, each once, in loop order.
    pub fn edges(&self) -> &[EdgeId] {
        &self.edges
    }

    /// Where `uv` lies with respect to the loops, with the whole periods
    /// added to `uv` that put it in the translate they are written in:
    /// `Inside` wins over `Boundary` over `Outside` across the translates,
    /// and the shift returned is the first that gave the answer. The
    /// boundary band is the domain's tolerance converted at `uv`
    /// ([`band`]).
    pub fn side(&self, uv: Point2) -> (Side, Vec2) {
        let near = band(self.surface, uv, self.tolerance);
        let periods = self.surface.period();
        let mut best = (Side::Outside, Vec2::zeros());
        for du in shifts(periods[0]) {
            for dv in shifts(periods[1]) {
                let shift = Vec2::new(du, dv);
                match self.index.side(uv + shift, near) {
                    Side::Inside => return (Side::Inside, shift),
                    Side::Boundary => {
                        if best.0 == Side::Outside {
                            best = (Side::Boundary, shift);
                        }
                    }
                    Side::Outside => {}
                }
            }
        }
        best
    }

    /// `true` when the loops wind around `uv` in any translate: the
    /// answer without a boundary band, for a point [`FaceDomain::side`]
    /// called `Boundary` that no edge or vertex of the face claims.
    pub fn winds_around(&self, uv: Point2) -> bool {
        let periods = self.surface.period();
        for du in shifts(periods[0]) {
            for dv in shifts(periods[1]) {
                let p = uv + Vec2::new(du, dv);
                let winding: i32 = self.polygons.iter().map(|g| g.winding_number(p)).sum();
                if winding != 0 {
                    return true;
                }
            }
        }
        false
    }

    /// The vertex, else the edge, of the face that `point` is on
    /// ([`boundary_entity`] over the face's edges).
    ///
    /// Errors: an edge, vertex or curve that does not resolve.
    pub fn boundary_entity(&self, model: &Model, point: Point3) -> Result<Option<Shape>, NotFound> {
        boundary_entity(model, self.edges.iter().copied(), point)
    }
}
