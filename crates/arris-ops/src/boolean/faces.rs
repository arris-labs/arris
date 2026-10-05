//! The operands as the pave model reads them: every face with its
//! [`FaceDomain`] at its own tolerance — the (u, v) answer to "is this
//! point on me" — and its boxes; every edge with its curve, range, ends
//! and box.

use arris_check::domain::{FaceDomain, shifts};
use arris_geom::{Curve, Curve2, Surface};
use arris_math::{Aabb, Interval, Point2, Point3};
use arris_topo::{Body, EdgeId, FaceId, Model, NotFound, Orientation, Shape, VertexId};

/// A singular point of a face's surface on its boundary — a cone's apex,
/// a sphere's pole — as the face holds it: the degenerate edge there, a
/// whole line of (u, v) over one vertex (ADR-0021).
pub(crate) struct Singular<'m> {
    /// The degenerate edge.
    pub edge: EdgeId,
    /// The vertex it starts and ends on.
    pub vertex: VertexId,
    pub point: Point3,
    /// The vertex's tolerance.
    pub tolerance: f64,
    /// The edge's pcurve on the face, along the singular parameter line.
    pub pcurve: &'m Curve2,
    pub range: Interval,
}

/// A face of an operand, read once.
pub(crate) struct FaceInfo<'m> {
    pub id: FaceId,
    pub surface: &'m Surface,
    pub tolerance: f64,
    /// The loops at the face's tolerance.
    pub domain: FaceDomain<'m>,
    /// The (u, v) box of the loops, grown by their chord deviation.
    pub uv_lo: Point2,
    pub uv_hi: Point2,
    /// The 3D box, grown by the tolerance.
    pub bounds: Aabb,
    /// The degenerate edges of the loops, in loop order: the edges
    /// [`EdgeInfo`] leaves out.
    pub singular: Vec<Singular<'m>>,
    /// Every other coedge of the loops, in loop order: its edge and its
    /// pcurve on the face — a seam's twice, once per use.
    pub uses: Vec<(EdgeId, &'m Curve2)>,
}

impl<'m> FaceInfo<'m> {
    /// Every face of `body` in iteration order.
    pub(crate) fn of_body(m: &'m Model, body: Body) -> Result<Vec<Self>, NotFound> {
        m.faces(body)?
            .into_iter()
            .map(|f| Self::of(m, f.id))
            .collect()
    }

    fn of(m: &'m Model, id: FaceId) -> Result<Self, NotFound> {
        let face = m.face(id)?;
        let surface = m.surface(face.surface())?;
        let tolerance = face.tolerance();
        let domain = FaceDomain::of(m, id, tolerance)?;
        // A face with no loop has no (u, v) box, and its 3D box is the
        // point at the origin of (u, v), where its first pcurve would
        // have been read from.
        let (uv_lo, uv_hi) = domain
            .uv_box()
            .map_or((Point2::origin(), Point2::origin()), |[u, v]| {
                (Point2::new(u.lo(), v.lo()), Point2::new(u.hi(), v.hi()))
            });
        let bounds = domain
            .bounds()
            .unwrap_or_else(|| Aabb::of_point(surface.point(uv_lo.x, uv_lo.y)).inflated(tolerance));
        let mut singular = Vec::new();
        let mut uses = Vec::new();
        for c in face.loops().iter().flat_map(|l| l.coedges()) {
            let edge = m.edge(c.edge())?;
            if !edge.is_degenerate() {
                uses.push((c.edge(), m.curve2(c.pcurve())?));
                continue;
            }
            let vertex = m.vertex(edge.start())?;
            singular.push(Singular {
                edge: c.edge(),
                vertex: edge.start(),
                point: vertex.point(),
                tolerance: vertex.tolerance(),
                pcurve: m.curve2(c.pcurve())?,
                range: edge.range(),
            });
        }
        Ok(FaceInfo {
            id,
            surface,
            tolerance,
            domain,
            uv_lo,
            uv_hi,
            bounds,
            singular,
            uses,
        })
    }

    /// The face as a `Forward` shape, for an error.
    pub(crate) fn shape(&self) -> Shape {
        Shape::new(self.id, Orientation::Forward)
    }

    /// The edges of the loops, each once, in loop order.
    pub(crate) fn edges(&self) -> &[EdgeId] {
        self.domain.edges()
    }
}

/// An edge of an operand with a curve, read once. Degenerate edges have
/// no curve to pierce anything with and are left out.
pub(crate) struct EdgeInfo<'m> {
    pub id: EdgeId,
    pub curve: &'m Curve,
    pub range: Interval,
    pub tolerance: f64,
    pub ends: [(VertexId, Point3, f64); 2],
    pub bounds: Aabb,
}

impl<'m> EdgeInfo<'m> {
    /// Every non-degenerate edge of `body` in iteration order.
    pub(crate) fn of_body(m: &'m Model, body: Body) -> Result<Vec<Self>, NotFound> {
        let mut out = Vec::new();
        for e in m.edges(body)? {
            let edge = m.edge(e.id)?;
            let Some((curve_id, range)) = edge.curve() else {
                continue;
            };
            let curve = m.curve(curve_id)?;
            let (start, end) = (m.vertex(edge.start())?, m.vertex(edge.end())?);
            let bounds = curve
                .bounds(range)
                .unwrap_or_else(|| Aabb::of_point(start.point()).union(Aabb::of_point(end.point())))
                .inflated(edge.tolerance());
            out.push(EdgeInfo {
                id: e.id,
                curve,
                range,
                tolerance: edge.tolerance(),
                ends: [
                    (edge.start(), start.point(), start.tolerance()),
                    (edge.end(), end.point(), end.tolerance()),
                ],
                bounds,
            });
        }
        Ok(out)
    }

    /// The edge as a `Forward` shape, for an error.
    pub(crate) fn shape(&self) -> Shape {
        Shape::new(self.id, Orientation::Forward)
    }

    /// `t` placed in the edge's range — a periodic parameter moved by a
    /// whole period when that puts it there — or `None` when it lies
    /// outside by more than the edge's tolerance converted to the
    /// parameter. A value within that band of an end is clamped to it.
    pub(crate) fn in_range(&self, t: f64) -> Option<f64> {
        let speed = self.curve.eval(t).d1.norm();
        let slack = if speed > 0.0 {
            self.tolerance / speed
        } else {
            0.0
        };
        for d in shifts(self.curve.period()) {
            let s = t + d;
            if self.range.lo() - slack <= s && s <= self.range.hi() + slack {
                return Some(self.range.clamp(s));
            }
        }
        None
    }

    /// The end vertex `point` is within the tolerance of, the start
    /// first.
    pub(crate) fn vertex_at(&self, point: Point3) -> Option<VertexId> {
        self.ends
            .iter()
            .find(|(_, p, tol)| (point - p).norm() <= *tol)
            .map(|(v, _, _)| *v)
    }
}
