//! The `Full` rows — E8, L5, S5, B1 and B2 of `docs/DATA-MODEL.md`
//! §Invariants — and the Gauss volume they share.
//!
//! These rows are not linear in the body: L5 sweeps every loop of a face
//! against every other, S5 every face of a shell against every other, and
//! B1 every face of one shell against every face of another and a ray from
//! each shell against every other (`crate::lumps`). Each face's loops are
//! discretised once, at a chord tolerance that is the model's parametric
//! tolerance scaled to the surface's speed, and the polygons are kept for
//! the row that needs them next.
//!
//! A pair the geometry kernel does not decide is never guessed at: it is
//! recorded through [`crate::Unchecked`], which is neither a violation nor
//! a pass.

use std::collections::{BTreeMap, BTreeSet};

use arris_topo::arris_geom::region2::Side;
use arris_topo::arris_geom::{
    Curve, Surface, SurfaceIntersection, SurfaceKind, intersect_surfaces,
};
use arris_topo::arris_math::{Aabb, Interval, Point2, Point3, Tolerance};
use arris_topo::entity::BodyKind;
use arris_topo::{EdgeId, FaceId, Orientation, ShellId, VertexId};

use crate::check::{Checker, coedges, samples};
use crate::classify::Classifier;
use crate::domain::{FaceDomain, boundary_entity};
use crate::flux::face_flux;
use crate::unchecked::Unchecked;
use crate::violation::{ShellNestingFault, Violation};

/// Every face's box, or `None` for one no box bounds: what S5 and B1
/// reject face pairs by before any intersector is asked.
pub(crate) type FaceBoxes = BTreeMap<FaceId, Option<Aabb>>;

impl<'m> Checker<'m> {
    /// E8, L5, S5, B1 and B2.
    pub(crate) fn full_rows(&mut self) {
        self.discretise_faces();
        self.e8_self_intersections();
        self.l5_loops_intersect();
        self.s5_face_pairs();
        self.b1_b2_shells();
    }

    /// Every face's [`FaceDomain`] at the model's parametric tolerance:
    /// what L5, S5 and B1 all ask. A face whose domain does not resolve
    /// has none, answers `Outside` everywhere and is never rejected by a
    /// box; M1 reports why.
    pub(crate) fn discretise_faces(&mut self) {
        let model = self.model;
        let tolerance = self.precision.parametric_tolerance;
        self.domains = self
            .closure
            .faces
            .iter()
            .filter_map(|&face| Some((face, FaceDomain::of(model, face, tolerance).ok()?)))
            .collect();
    }

    /// E8: an analytic curve over a range E1 accepted cannot cross
    /// itself, so only a NURBS is tested — as a polyline sampled
    /// `Precision::check_samples` times per knot span, two non-adjacent
    /// segments closer than the edge's tolerance being the crossing when
    /// more than that tolerance of polyline runs between them. Nearer
    /// along the curve they are one stretch of it, however many samples
    /// a short span — a periodic section edge running a few units in the
    /// last place past its knots' end — put between them.
    fn e8_self_intersections(&mut self) {
        let model = self.model;
        let mut found = Vec::new();
        for &edge_id in &self.closure.edges {
            let Ok(edge) = model.edge(edge_id) else {
                continue;
            };
            let Some((curve_id, range)) = edge.curve() else {
                continue;
            };
            let Ok(curve) = model.curve(curve_id) else {
                continue;
            };
            let Curve::Nurbs(nurbs) = curve else {
                continue;
            };
            if !(range.lo().is_finite() && range.hi().is_finite() && range.lo() < range.hi()) {
                continue;
            }
            let breaks = nurbs.breaks_within(range);
            let mut parameters: Vec<f64> = Vec::new();
            let mut lo = range.lo();
            for hi in breaks.iter().copied().chain([range.hi()]) {
                let Ok(span) = Interval::new(lo, hi) else {
                    continue;
                };
                for t in samples(span, self.precision.check_samples.max(2)) {
                    if parameters.last() != Some(&t) {
                        parameters.push(t);
                    }
                }
                lo = hi;
            }
            let points: Vec<Point3> = parameters.iter().map(|&t| curve.point(t)).collect();
            // `along[k]`: the polyline's length up to point `k`.
            let mut along = Vec::with_capacity(points.len());
            let mut length = 0.0;
            for (k, p) in points.iter().enumerate() {
                if k > 0 {
                    length += (p - points[k - 1]).norm();
                }
                along.push(length);
            }
            let closed = edge.start() == edge.end();
            let n = points.len().saturating_sub(1);
            for i in 0..n {
                for j in i + 2..n {
                    if closed && i == 0 && j == n - 1 {
                        continue;
                    }
                    let between = along[j] - along[i + 1];
                    let around = if closed {
                        between.min(length - (along[j + 1] - along[i]))
                    } else {
                        between
                    };
                    if around <= edge.tolerance() {
                        continue;
                    }
                    let (sa, sb) = ((points[i], points[i + 1]), (points[j], points[j + 1]));
                    let (distance, u, v) = segment_distance(sa, sb);
                    if distance <= edge.tolerance() {
                        found.push(Violation::EdgeSelfIntersects {
                            edge: edge_id,
                            t0: parameters[i] + u * (parameters[i + 1] - parameters[i]),
                            t1: parameters[j] + v * (parameters[j + 1] - parameters[j]),
                        });
                    }
                }
            }
        }
        for v in found {
            self.push(v);
        }
    }

    /// L5: no loop of a face crosses itself or another of its loops.
    fn l5_loops_intersect(&mut self) {
        let mut found = Vec::new();
        for (&face_id, domain) in &self.domains {
            let polygons = domain.polygons();
            for (i, a) in polygons.iter().enumerate() {
                if !a.self_intersections().is_empty() {
                    found.push(Violation::LoopsIntersect {
                        face: face_id,
                        loop_a: i,
                        loop_b: i,
                    });
                }
                for (j, b) in polygons.iter().enumerate().skip(i + 1) {
                    if !a.intersections(b).is_empty() {
                        found.push(Violation::LoopsIntersect {
                            face: face_id,
                            loop_a: i,
                            loop_b: j,
                        });
                    }
                }
            }
        }
        for v in found {
            self.push(v);
        }
    }

    /// Every face's box ([`FaceDomain::bounds`]); `None` for a face with
    /// no domain or no bounded box, which S5 never rejects by its box.
    pub(crate) fn face_boxes(&self) -> FaceBoxes {
        self.closure
            .faces
            .iter()
            .map(|&face| (face, self.domains.get(&face).and_then(FaceDomain::bounds)))
            .collect()
    }

    /// Whether faces `a` and `b` meet away from the edges and vertices
    /// they share — S5's test, and B1's between two shells, which share
    /// none. A pair whose boxes are apart shares no point and is decided
    /// without an intersector; a pair that does not resolve is M1's and
    /// meets nowhere here. Errors: the intersector does not decide the
    /// pair's surfaces, whose kinds it names.
    pub(crate) fn faces_meet(
        &self,
        a: FaceId,
        b: FaceId,
        boxes: &FaceBoxes,
    ) -> Result<bool, (SurfaceKind, SurfaceKind)> {
        let model = self.model;
        let (Ok(fa), Ok(fb)) = (model.face(a), model.face(b)) else {
            return Ok(false);
        };
        let (Ok(sa), Ok(sb)) = (model.surface(fa.surface()), model.surface(fb.surface())) else {
            return Ok(false);
        };
        let (ba, bb) = (
            boxes.get(&a).copied().flatten(),
            boxes.get(&b).copied().flatten(),
        );
        if let (Some(ba), Some(bb)) = (ba, bb) {
            if !ba.intersects(&bb) {
                return Ok(false);
            }
        }
        // The pair is decided at the larger of the two faces' own
        // tolerances (`docs/DATA-MODEL.md` §Tolerances), as a boolean
        // between them would decide it.
        let tolerance = fa.tolerance().max(fb.tolerance());
        let query = Tolerance::new(tolerance, self.precision.angular_tolerance);
        // A point interior to both faces is in both boxes, so their
        // overlap — or the one box there is — bounds every traced section
        // S5 can find; the closed forms ignore it. With neither, no face
        // has a bounded domain to place a point in, and the pair meets
        // nowhere here — L1's and E1's to report.
        let overlap = match (ba, bb) {
            (Some(ba), Some(bb)) => Some(Aabb {
                min: [0, 1, 2].map(|i| ba.min[i].max(bb.min[i])),
                max: [0, 1, 2].map(|i| ba.max[i].min(bb.max[i])),
            }),
            (one, other) => one.or(other),
        };
        let Some(within) = overlap.map(|b| b.inflated(tolerance)) else {
            return Ok(false);
        };
        Ok(
            match intersect_surfaces(
                sa,
                sb,
                &within,
                query,
                &mut arris_topo::arris_math::Meter::default(),
            ) {
                Err(_) => return Err((sa.kind(), sb.kind())),
                Ok(SurfaceIntersection::Empty) => false,
                Ok(SurfaceIntersection::Coincident) => {
                    self.regions_overlap(a, sa, b, sb, tolerance)
                }
                // Crossing or touching, every curve and every point is held to
                // the one rule.
                Ok(SurfaceIntersection::Meets { curves, points }) => {
                    curves
                        .iter()
                        .any(|c| self.curve_is_interior_to_both(a, sa, b, sb, &c.curve, tolerance))
                        || points.iter().any(|p| {
                            self.point_is_interior_to_both(a, sa, b, sb, p.point, tolerance)
                        })
                }
            },
        )
    }

    /// `true` when an isolated meeting point of the two surfaces — a
    /// touch, or a crossing through an apex — is interior to both faces,
    /// within `tolerance` of both surfaces, and not within tolerance of a
    /// vertex both faces reach or of an edge they share. The vertex
    /// clause is what excuses two cones closing on one apex, or a blend
    /// sphere touching a plane at the corner of its contact lines: a
    /// vertex shared through no edge.
    fn point_is_interior_to_both(
        &self,
        a: FaceId,
        sa: &Surface,
        b: FaceId,
        sb: &Surface,
        point: Point3,
        tolerance: f64,
    ) -> bool {
        let model = self.model;
        let shared = self.shared_edges(a, b);
        if boundary_entity(model, shared.iter().copied(), point).is_ok_and(|on| on.is_some()) {
            return false;
        }
        let at_shared_vertex = self.shared_vertices(a, b).into_iter().any(|v| {
            model
                .vertex(v)
                .is_ok_and(|vx| (vx.point() - point).norm() <= vx.tolerance().max(tolerance))
        });
        if at_shared_vertex {
            return false;
        }
        let (Ok(pa), Ok(pb)) = (sa.project(point), sb.project(point)) else {
            return false;
        };
        if pa.distance > tolerance || pb.distance > tolerance {
            return false;
        }
        let inside = |face: FaceId, uv: Point2| {
            self.domains
                .get(&face)
                .is_some_and(|d| d.side(uv).0 == Side::Inside)
        };
        inside(a, pa.uv) && inside(b, pb.uv)
    }

    /// The vertices two faces both reach through their edges, in id order.
    fn shared_vertices(&self, a: FaceId, b: FaceId) -> Vec<VertexId> {
        let model = self.model;
        let of = |face: FaceId| -> BTreeSet<VertexId> {
            model.face(face).map_or_else(
                |_| BTreeSet::new(),
                |f| {
                    coedges(f)
                        .filter_map(|(_, _, c)| model.edge(c.edge()).ok())
                        .flat_map(|e| [e.start(), e.end()])
                        .collect()
                },
            )
        };
        of(a).intersection(&of(b)).copied().collect()
    }

    /// S5: two faces of a shell meet only along the edges and vertices
    /// they share ([`Checker::faces_meet`]); a pair the intersector does
    /// not decide is unchecked.
    fn s5_face_pairs(&mut self) {
        let model = self.model;
        let mut found = Vec::new();
        let mut undecided = Vec::new();
        let boxes = self.face_boxes();
        for &shell_id in &self.closure.shells {
            let Ok(shell) = model.shell(shell_id) else {
                continue;
            };
            let faces: Vec<FaceId> = shell
                .faces()
                .iter()
                .map(|f| f.id)
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect();
            for (i, &a) in faces.iter().enumerate() {
                for &b in faces.iter().skip(i + 1) {
                    match self.faces_meet(a, b, &boxes) {
                        Ok(false) => {}
                        Ok(true) => found.push(Violation::FacesIntersect {
                            shell: shell_id,
                            face_a: a,
                            face_b: b,
                        }),
                        Err(kinds) => undecided.push(Unchecked::FacePair {
                            shell: shell_id,
                            face_a: a,
                            face_b: b,
                            kinds,
                        }),
                    }
                }
            }
        }
        for v in found {
            self.push(v);
        }
        self.unchecked.extend(undecided);
    }

    /// `true` when two faces on the same surface share interior (u, v):
    /// a deterministic grid over each face's parameter box, the points
    /// strictly inside that face carried through 3D and classified
    /// against the other when within `tolerance` of its surface. An
    /// overlap smaller than the grid's spacing is not seen; the loops
    /// crossing is L5's and S5's own curve test.
    fn regions_overlap(
        &self,
        a: FaceId,
        sa: &Surface,
        b: FaceId,
        sb: &Surface,
        tolerance: f64,
    ) -> bool {
        let n = self.precision.check_samples.max(2);
        for (from, from_surface, to, to_surface) in [(a, sa, b, sb), (b, sb, a, sa)] {
            let Some(domain) = self.domains.get(&from) else {
                continue;
            };
            let points: Vec<Point2> = domain
                .polygons()
                .iter()
                .flat_map(|p| p.points())
                .copied()
                .collect();
            let (Some(lo), Some(hi)) = (corner(&points, f64::min), corner(&points, f64::max))
            else {
                continue;
            };
            for i in 0..n {
                for j in 0..n {
                    let uv = Point2::new(
                        lerp(lo.x, hi.x, i as f64 / (n - 1) as f64),
                        lerp(lo.y, hi.y, j as f64 / (n - 1) as f64),
                    );
                    if domain.side(uv).0 != Side::Inside {
                        continue;
                    }
                    let point = from_surface.point(uv.x, uv.y);
                    let Ok(projection) = to_surface.project(point) else {
                        continue;
                    };
                    if projection.distance > tolerance {
                        continue;
                    }
                    if self
                        .domains
                        .get(&to)
                        .is_some_and(|d| d.side(projection.uv).0 == Side::Inside)
                    {
                        return true;
                    }
                }
            }
        }
        false
    }

    /// `true` when the surfaces' intersection curve has a point interior
    /// to both faces, within `tolerance` of both surfaces, that is not on
    /// an edge or vertex they share. The curve is sampled over the
    /// parameters both faces' boundaries reach — its whole domain when
    /// that is bounded ([`Checker::curve_range`]).
    fn curve_is_interior_to_both(
        &self,
        a: FaceId,
        sa: &Surface,
        b: FaceId,
        sb: &Surface,
        curve: &Curve,
        tolerance: f64,
    ) -> bool {
        let Some(range) = self.curve_range(a, sa, b, sb, curve) else {
            return false;
        };
        let shared = self.shared_edges(a, b);
        let inside = |face: FaceId, uv: Point2| {
            self.domains
                .get(&face)
                .is_some_and(|d| d.side(uv).0 == Side::Inside)
        };
        for t in samples(range, self.precision.check_samples.max(2)) {
            let point = curve.point(t);
            // On an edge or vertex the two faces share, where the
            // surfaces are allowed to meet.
            if boundary_entity(self.model, shared.iter().copied(), point)
                .is_ok_and(|on| on.is_some())
            {
                continue;
            }
            let (Ok(pa), Ok(pb)) = (sa.project(point), sb.project(point)) else {
                continue;
            };
            if pa.distance > tolerance || pb.distance > tolerance {
                continue;
            }
            if inside(a, pa.uv) && inside(b, pb.uv) {
                return true;
            }
        }
        false
    }

    /// The parameters of `curve` both faces' boundaries reach: its whole
    /// domain when that is bounded — a periodic curve, or a NURBS, which
    /// the intersector already bounded by the overlap of the two faces'
    /// boxes — else, for a line, the overlap of the hulls of each face's
    /// boundary projected onto it. `None` when they do not overlap. A
    /// traced branch clipped by that overlap is a NURBS of a hundred
    /// control points, and projecting every point of both faces' polygons
    /// onto it took seconds a pair.
    fn curve_range(
        &self,
        a: FaceId,
        sa: &Surface,
        b: FaceId,
        sb: &Surface,
        curve: &Curve,
    ) -> Option<Interval> {
        let domain = curve.domain();
        if curve.period().is_some() || (domain.lo().is_finite() && domain.hi().is_finite()) {
            return Some(domain);
        }
        let mut hulls = Vec::with_capacity(2);
        for (face, surface) in [(a, sa), (b, sb)] {
            let polygons = self.domains.get(&face)?.polygons();
            let mut hull: Option<Interval> = None;
            for uv in polygons.iter().flat_map(|p| p.points()) {
                let Ok(projection) = curve.project(surface.point(uv.x, uv.y)) else {
                    continue;
                };
                let t = projection.t;
                hull = Some(match hull {
                    Some(h) => h.hull(&Interval::new(t, t).ok()?),
                    None => Interval::new(t, t).ok()?,
                });
            }
            hulls.push(hull?);
        }
        hulls[0].intersection(&hulls[1])
    }

    /// The edges two faces share, in id order.
    fn shared_edges(&self, a: FaceId, b: FaceId) -> Vec<EdgeId> {
        let model = self.model;
        let of = |face: FaceId| -> BTreeSet<EdgeId> {
            model.face(face).map_or_else(
                |_| BTreeSet::new(),
                |f| coedges(f).map(|(_, _, c)| c.edge()).collect(),
            )
        };
        of(a).intersection(&of(b)).copied().collect()
    }

    /// The signed volume a shell encloses: the flux of `P / 3`, whose
    /// divergence is one, through each face use composed with `outer`
    /// (the body handle's orientation and the shell use's). `None` when a
    /// reference does not resolve or a loop cannot be integrated — M1's,
    /// L1's and E1's to report.
    pub(crate) fn shell_volume(&self, shell_id: ShellId, outer: Orientation) -> Option<f64> {
        let shell = self.model.shell(shell_id).ok()?;
        let mut total = 0.0;
        for face_use in shell.faces() {
            let volume = face_flux(self.model, face_use.id, |p, n| p.coords.dot(&n) / 3.0).ok()?;
            total += outer.compose(face_use.orientation).sign() * volume;
        }
        Some(total)
    }

    /// B1 (the shells nest into lumps, `crate::lumps`) and B2 (positive
    /// enclosed volume), for a `Solid` body.
    fn b1_b2_shells(&mut self) {
        let model = self.model;
        let Ok(body) = model.body(self.body.id) else {
            return;
        };
        if body.kind() != BodyKind::Solid {
            return;
        }
        if body.shells().is_empty() {
            self.push(Violation::ShellNesting {
                body: self.body.id,
                fault: ShellNestingFault::NoShells,
            });
            return;
        }
        let Ok(shells) = self.shell_volumes() else {
            return;
        };
        // B2 first: the body's volume is the sum over its shells.
        let total: f64 = shells.iter().map(|&(_, v)| v).sum();
        if !(total.is_finite() && total > 0.0) {
            self.push(Violation::NonPositiveVolume {
                body: self.body.id,
                volume: total,
            });
        }
        let nesting = self.nesting(&shells);
        for fault in nesting.faults {
            self.push(Violation::ShellNesting {
                body: self.body.id,
                fault,
            });
        }
        self.unchecked.extend(nesting.unchecked);
    }

    /// A point of `shell`: the start vertex of the first edge of its first
    /// face that resolves. `None` when nothing does.
    pub(crate) fn shell_point(&self, shell: ShellId) -> Option<Point3> {
        let model = self.model;
        model
            .shell(shell)
            .ok()?
            .faces()
            .iter()
            .filter_map(|f| model.face(f.id).ok())
            .flat_map(coedges)
            .filter_map(|(_, _, c)| model.edge(c.edge()).ok())
            .filter_map(|e| model.vertex(e.start()).ok())
            .map(|v| v.point())
            .next()
    }

    /// The classifier over the closed shell `shell`'s faces alone, in
    /// stored order: B1's ray cast is [`crate::classify`]'s, the same code
    /// the public classifier runs, so B1 and a boolean can never disagree
    /// about a point (ADR-0004). Its `contains` is `None` when every
    /// direction was abandoned and an error when a surface has no closed
    /// form against a ray, which the row records as unchecked. `None`
    /// when the shell does not resolve.
    pub(crate) fn shell_classifier(&self, shell_id: ShellId) -> Option<Classifier<'m>> {
        let faces = self
            .model
            .shell(shell_id)
            .ok()?
            .faces()
            .iter()
            .map(|f| f.id)
            .collect();
        Some(Classifier::over(self.model, self.body, faces))
    }
}

fn lerp(a: f64, b: f64, s: f64) -> f64 {
    a + (b - a) * s
}

/// The corner of the bounding box of `points` under `pick` (`f64::min`
/// for the low corner, `f64::max` for the high one).
fn corner(points: &[Point2], pick: fn(f64, f64) -> f64) -> Option<Point2> {
    points
        .iter()
        .copied()
        .reduce(|a, b| Point2::new(pick(a.x, b.x), pick(a.y, b.y)))
}

/// The distance between two segments in 3D and the parameters in `[0, 1]`
/// of the nearest point on each: the standard clamped solution of the
/// two-parameter least-squares problem, with the degenerate cases (a
/// point, two parallel segments) falling out of the clamps.
fn segment_distance(a: (Point3, Point3), b: (Point3, Point3)) -> (f64, f64, f64) {
    let (d1, d2, r) = (a.1 - a.0, b.1 - b.0, a.0 - b.0);
    let (aa, e, f) = (d1.dot(&d1), d2.dot(&d2), d2.dot(&r));
    let (c, bb) = (d1.dot(&r), d1.dot(&d2));
    let denominator = aa * e - bb * bb;
    let s = if denominator > 0.0 {
        ((bb * f - c * e) / denominator).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let t = if e > 0.0 {
        ((bb * s + f) / e).clamp(0.0, 1.0)
    } else {
        0.0
    };
    // One clamp can invalidate the other; re-solve `s` for the clamped
    // `t`, which is what makes the parallel and end-on cases right.
    let s = if aa > 0.0 {
        ((t * bb - c) / aa).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let closest = (a.0 + d1 * s) - (b.0 + d2 * t);
    (closest.norm(), s, t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segment_distance_is_the_clamped_nearest_approach() {
        let p = |x, y, z| Point3::new(x, y, z);
        // Crossing at the origin, one unit apart in z.
        let (d, s, t) = segment_distance(
            (p(-1.0, 0.0, 0.0), p(1.0, 0.0, 0.0)),
            (p(0.0, -1.0, 1.0), p(0.0, 1.0, 1.0)),
        );
        assert!((d - 1.0).abs() < 1e-15 && (s - 0.5).abs() < 1e-15 && (t - 0.5).abs() < 1e-15);
        // Parallel, offset along their own direction: the ends decide.
        let (d, _, _) = segment_distance(
            (p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0)),
            (p(3.0, 4.0, 0.0), p(4.0, 4.0, 0.0)),
        );
        assert!((d - (4.0f64 + 4.0 * 4.0).sqrt()).abs() < 1e-12);
        // Touching at an endpoint.
        let (d, _, _) = segment_distance(
            (p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0)),
            (p(1.0, 0.0, 0.0), p(1.0, 1.0, 0.0)),
        );
        assert_eq!(d, 0.0);
    }
}
