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

use arris_geom::region2::Side;
use arris_geom::{
    Curve, CurveIntersection, MeetKind, Surface, SurfaceIntersection, SurfaceKind,
    intersect_curves, intersect_surfaces, trace_section,
};
use arris_math::{Aabb, Interrupted, Interval, Meter, Point2, Point3, Tolerance, Vec3, wrap_into};
use arris_topo::entity::BodyKind;
use arris_topo::{Body, EdgeId, EntityId, FaceId, Model, NotFound, Orientation, ShellId, VertexId};

use crate::check::{Checker, coedges};
use crate::classify::Classifier;
use crate::domain::{FaceDomain, boundary_entity};
use crate::flux::face_flux;
use crate::unchecked::Unchecked;
use crate::violation::{ShellNestingFault, Violation};

/// How many boxes [`Checker::patches_apart`] may bound before it gives
/// a pair up as not proved apart.
const PATCH_BUDGET: usize = 20_000;

/// How many times [`Checker::patches_apart`] quarters a pair before it
/// gives it up: a (u, v) box a 256th of the face's across.
const PATCH_DEPTH: u32 = 8;

/// Every face's box, or `None` for one no box bounds: what S5 and B1
/// reject face pairs by before any intersector is asked.
pub(crate) type FaceBoxes = BTreeMap<FaceId, Option<Aabb>>;

/// What a meeting of two faces' surfaces is held to by
/// [`Checker::faces_meet`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Meeting {
    /// Every meeting interior to both faces is a violation, a touch as
    /// much as a crossing: two faces of one shell (S5).
    Any,
    /// A touch (`MeetKind::Touch`: tangent along a curve or at a point,
    /// the surfaces not crossing) is admitted; a crossing and an overlap
    /// of coincident regions are violations: two faces of two shells,
    /// whose lumps may touch (B1, ADR-0088).
    Crossing,
    /// Only touching curves and points, for the boolean's inherited
    /// contacts. Isolated points may lie on the face boundary.
    Touching,
}

impl Meeting {
    fn holds(self, kind: MeetKind) -> bool {
        match self {
            Self::Any => true,
            Self::Crossing => kind == MeetKind::Crossing,
            Self::Touching => kind == MeetKind::Touch,
        }
    }
}

/// The first supplied pair of faces of `body` that touch along a curve
/// interior to both or at an isolated point on both trims, away from
/// shared topology. Used by booleans when formerly separate shells
/// contribute to one lump. An undecided pair does not establish a touch.
/// Returns an error if `body` does not resolve.
#[doc(hidden)]
pub fn first_touching_face_pair(
    model: &Model,
    body: Body,
    pairs: &[[FaceId; 2]],
) -> Result<Option<[FaceId; 2]>, NotFound> {
    match first_touching_face_pair_metered(model, body, pairs, &mut Meter::default()) {
        Ok(pair) => Ok(pair),
        Err(TouchingFacePairError::NotFound(e)) => Err(e),
        Err(TouchingFacePairError::Interrupted(_)) => unreachable!("the default meter cannot stop"),
    }
}

/// Why a search for touching faces could not finish.
#[doc(hidden)]
#[derive(Debug)]
pub enum TouchingFacePairError {
    /// The body does not resolve.
    NotFound(NotFound),
    /// The caller stopped the search.
    Interrupted(Interrupted),
}

/// The same search as [`first_touching_face_pair`], charging each face pair
/// and its intersections to the operation's `meter`.
#[doc(hidden)]
pub fn first_touching_face_pair_metered(
    model: &Model,
    body: Body,
    pairs: &[[FaceId; 2]],
    meter: &mut Meter<'_>,
) -> Result<Option<[FaceId; 2]>, TouchingFacePairError> {
    let mut checker = Checker::new(model, body).map_err(TouchingFacePairError::NotFound)?;
    let tolerance = checker.precision.parametric_tolerance;
    for face in pairs.iter().flatten().copied().collect::<BTreeSet<_>>() {
        if let Ok(domain) = FaceDomain::of(model, face, tolerance) {
            checker.domains.insert(face, domain);
        }
    }
    let boxes = checker.face_boxes();
    for &[a, b] in pairs {
        meter.tick().map_err(TouchingFacePairError::Interrupted)?;
        let meets = checker.faces_meet(a, b, &boxes, Meeting::Touching, meter);
        // An intersection that was stopped can be reported as undecided.
        if let Some(stop) = meter.stopped() {
            return Err(TouchingFacePairError::Interrupted(stop));
        }
        if meets == Ok(true) {
            return Ok(Some([a, b]));
        }
    }
    Ok(None)
}

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
    /// the polyline between them leaves the tolerance ball round the
    /// nearer point — both ways round on a closed edge. A stretch that
    /// stays inside it is one stretch of the curve, however many samples
    /// a short span put on it: a periodic section edge running a few
    /// units in the last place past its knots' end, or a read edge
    /// bending through its first tolerance's worth of length (FTC-07's,
    /// `real/edge-bending-within-its-tolerance`), whose chord across the
    /// bend is a hair shorter than the arc.
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
                for t in span.samples(self.precision.check_samples.max(2)) {
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
                    if distance > edge.tolerance() {
                        continue;
                    }
                    let at = sa.0 + (sa.1 - sa.0) * u;
                    let leaves = |k: usize| (points[k] - at).norm() > edge.tolerance();
                    let ahead = (i + 1..=j).any(leaves);
                    let behind = !closed || (j + 1..=n).chain(0..=i).any(leaves);
                    if ahead && behind {
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

    /// L5: no loop of a face crosses itself or another of its loops. A
    /// loop crossing itself inside the tolerance ball of one of the
    /// face's vertices does not: its pieces meet at that vertex only to
    /// its tolerance, and a pcurve ending a little past the next one's
    /// start — FTC-07's sphere faces, `1e-6` past at a vertex of
    /// `2.7e-5`, `real/loop-crossing-inside-a-vertex` — zigzags there.
    /// Two loops meeting inside a vertex's ball meet there alike: a hole
    /// through the vertex where it touches its outer loop, Open
    /// CASCADE's crescent, which Arris writes as one loop through it
    /// twice.
    fn l5_loops_intersect(&mut self) {
        let model = self.model;
        let mut found = Vec::new();
        for (&face_id, domain) in &self.domains {
            let polygons = domain.polygons();
            let vertices: BTreeSet<VertexId> = model.face(face_id).map_or_else(
                |_| BTreeSet::new(),
                |f| {
                    coedges(f)
                        .filter_map(|(_, _, c)| model.edge(c.edge()).ok())
                        .flat_map(|e| [e.start(), e.end()])
                        .collect()
                },
            );
            let at_a_vertex = |pa: &arris_geom::region2::Polygon2,
                               pb: &arris_geom::region2::Polygon2,
                               (i, j): (usize, usize)| {
                let lift = |polygon: &arris_geom::region2::Polygon2, k: usize| {
                    let points = polygon.points();
                    let p = points[k % points.len()];
                    Point3::new(p.x, p.y, 0.0)
                };
                let (sa, sb) = (
                    (lift(pa, i), lift(pa, i + 1)),
                    (lift(pb, j), lift(pb, j + 1)),
                );
                let (_, u, _) = segment_distance(sa, sb);
                let uv = sa.0 + (sa.1 - sa.0) * u;
                let point = domain.surface().point(uv.x, uv.y);
                vertices.iter().any(|&v| {
                    model
                        .vertex(v)
                        .is_ok_and(|vx| (vx.point() - point).norm() <= vx.tolerance())
                })
            };
            for (i, a) in polygons.iter().enumerate() {
                if !a
                    .self_intersections()
                    .into_iter()
                    .all(|pair| at_a_vertex(a, a, pair))
                {
                    found.push(Violation::LoopsIntersect {
                        face: face_id,
                        loop_a: i,
                        loop_b: i,
                    });
                }
                for (j, b) in polygons.iter().enumerate().skip(i + 1) {
                    if !a
                        .intersections(b)
                        .into_iter()
                        .all(|pair| at_a_vertex(a, b, pair))
                    {
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

    /// Whether faces `a` and `b` meet, as `meeting` holds them to, away
    /// from the edges and vertices they share — S5's test
    /// ([`Meeting::Any`]), and B1's between two shells, which share none
    /// ([`Meeting::Crossing`]: lumps may touch, ADR-0088). A pair whose boxes are apart shares no point and is decided
    /// without an intersector; a pair that does not resolve is M1's and
    /// meets nowhere here. Errors: the intersector does not decide the
    /// pair's surfaces, whose kinds it names.
    pub(crate) fn faces_meet(
        &self,
        a: FaceId,
        b: FaceId,
        boxes: &FaceBoxes,
        meeting: Meeting,
        meter: &mut Meter<'_>,
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
        let nurbs = |s: &Surface| matches!(s, Surface::Nurbs(_));
        if nurbs(sa) || nurbs(sb) {
            return match self.nurbs_pair_meets(a, sa, b, sb, &within, query, meeting, meter) {
                // A pair the intersector refuses is still apart when
                // every piece of one face's patch is clear of every piece
                // of the other's (the nesting of two lumps on one wall).
                Err(_)
                    if meter.stopped().is_none() && self.patches_apart(a, sa, b, sb, tolerance) =>
                {
                    Ok(false)
                }
                found => found,
            };
        }
        Ok(match intersect_surfaces(sa, sb, &within, query, meter) {
            Err(_) => return Err((sa.kind(), sb.kind())),
            Ok(SurfaceIntersection::Empty) => false,
            Ok(SurfaceIntersection::Coincident) => {
                meeting != Meeting::Touching && self.regions_overlap(a, sa, b, sb, tolerance)
            }
            // Crossing or touching, every curve and every point is held to
            // the one rule, but for the touches `meeting` admits.
            Ok(SurfaceIntersection::Meets { curves, points }) => {
                curves.iter().any(|c| {
                    meter.stopped().is_none()
                        && meeting.holds(c.kind)
                        && self.curve_is_interior_to_both(
                            a, sa, b, sb, &c.curve, query, meeting, c.kind, meter,
                        )
                }) || points.iter().any(|p| {
                    meter.stopped().is_none()
                        && meeting.holds(p.kind)
                        && self.isolated_point_meets(
                            [(a, sa), (b, sb)],
                            p.point,
                            tolerance,
                            meeting,
                        )
                })
            }
        })
    }

    /// Whether the faces' patches share no point within `tolerance`,
    /// proved from boxes alone: the (u, v) boxes of the two faces are
    /// quartered together wherever their clipped boxes still overlap,
    /// until none does. A pair still overlapping when [`PATCH_BUDGET`]
    /// boxes are spent (surfaces that touch, or lie flush) is not proved
    /// apart.
    fn patches_apart(
        &self,
        a: FaceId,
        sa: &Surface,
        b: FaceId,
        sb: &Surface,
        tolerance: f64,
    ) -> bool {
        let (Some(da), Some(db)) = (self.domains.get(&a), self.domains.get(&b)) else {
            return false;
        };
        let (Some(ua), Some(ub)) = (da.uv_box(), db.uv_box()) else {
            return false;
        };
        let mut budget = PATCH_BUDGET;
        let patch = |surface, domain, uv| Patch {
            surface,
            domain,
            uv,
        };
        patches_clear(
            &patch(sa, da, ua),
            &patch(sb, db, ub),
            tolerance,
            PATCH_DEPTH,
            &mut budget,
        )
    }

    /// [`Checker::faces_meet`] for a pair with a NURBS surface in it,
    /// which the intersector traces and then fits ([`trace_section`]): S5
    /// only samples the section, so it reads the traced branches'
    /// own points and never pays for the fit (a fit held to a quarter of
    /// the tolerance was the larger share of a NURBS part's `Full`
    /// check). Every branch, exact isoline and singular point is held to
    /// the rule the fitted curves were.
    #[allow(clippy::too_many_arguments)]
    fn nurbs_pair_meets(
        &self,
        a: FaceId,
        sa: &Surface,
        b: FaceId,
        sb: &Surface,
        within: &Aabb,
        query: Tolerance,
        meeting: Meeting,
        meter: &mut Meter<'_>,
    ) -> Result<bool, (SurfaceKind, SurfaceKind)> {
        let tolerance = query.linear;
        let kinds = (sa.kind(), sb.kind());
        let trace = trace_section(sa, sb, within, query, meter).map_err(|_| kinds)?;
        let interior = |range: Interval, at: &dyn Fn(f64) -> Point3| {
            let samples = range.samples(self.precision.check_samples.max(2));
            self.section_is_interior_to_both(a, sa, b, sb, samples, at, tolerance, meeting)
        };
        // A tangent isoline and an isolated singular point are the touches
        // the intersector would name `MeetKind::Touch`; a branch crosses.
        let kind = |touch| {
            if touch {
                MeetKind::Touch
            } else {
                MeetKind::Crossing
            }
        };
        Ok(trace.circles().iter().any(|c| {
            let kind = kind(c.tangent);
            meter.stopped().is_none()
                && meeting.holds(kind)
                && self
                    .curve_is_interior_to_both(a, sa, b, sb, &c.circle, query, meeting, kind, meter)
        }) || (meter.stopped().is_none()
            && meeting.holds(MeetKind::Crossing)
            && trace
                .branches()
                .iter()
                .any(|branch| interior(branch.domain(), &|t| branch.point(t))))
            || trace.points().iter().any(|p| {
                meter.stopped().is_none()
                    && meeting.holds(kind(p.isolated))
                    && self.isolated_point_meets([(a, sa), (b, sb)], p.point, tolerance, meeting)
            }))
    }

    /// B1 and S5 test face interiors. A boolean's inherited isolated
    /// contact also survives on a seam or pole that belongs to just one
    /// of the participating faces.
    fn isolated_point_meets(
        &self,
        [(a, sa), (b, sb)]: [(FaceId, &Surface); 2],
        point: Point3,
        tolerance: f64,
        meeting: Meeting,
    ) -> bool {
        if meeting == Meeting::Touching {
            let on = |face| {
                self.domains
                    .get(&face)?
                    .entity_at_contact(self.model, point)
                    .ok()
                    .flatten()
            };
            matches!([on(a), on(b)], [Some(a), Some(b)] if a != b)
        } else {
            self.point_is_interior_to_both(a, sa, b, sb, point, tolerance)
        }
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
                    match self.faces_meet(a, b, &boxes, Meeting::Any, &mut Meter::default()) {
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
    /// an edge or vertex they share. Touching curves also have a witness
    /// tested between consecutive trim hits, where a slit can survive
    /// between the samples over the full range.
    #[allow(clippy::too_many_arguments)]
    fn curve_is_interior_to_both(
        &self,
        a: FaceId,
        sa: &Surface,
        b: FaceId,
        sb: &Surface,
        curve: &Curve,
        tolerance: Tolerance,
        meeting: Meeting,
        kind: MeetKind,
        meter: &mut Meter<'_>,
    ) -> bool {
        let Some(range) = self.curve_range(a, sa, b, sb, curve) else {
            return false;
        };
        let samples = range.samples(self.precision.check_samples.max(2));
        let midpoints = std::iter::once_with(|| {
            if kind == MeetKind::Touch {
                self.trimmed_section_midpoints([a, b], curve, range, tolerance, meter)
            } else {
                Vec::new()
            }
        })
        .flatten();
        self.section_is_interior_to_both(
            a,
            sa,
            b,
            sb,
            samples.into_iter().chain(midpoints),
            &|t| curve.point(t),
            tolerance.linear,
            meeting,
        )
    }

    /// The midpoints between a contact curve's intersections with the
    /// trimmed boundary edges. Vertices also mark the ends of edges that
    /// lie along the contact itself, such as a seam.
    fn trimmed_section_midpoints(
        &self,
        faces: [FaceId; 2],
        curve: &Curve,
        range: Interval,
        tolerance: Tolerance,
        meter: &mut Meter<'_>,
    ) -> Vec<f64> {
        let in_range = |t: f64, curve: &Curve, range: Interval| {
            let t = curve.period().map_or(t, |p| wrap_into(t, range.lo(), p));
            range.contains(t).then_some(t)
        };
        let edges: BTreeSet<_> = faces
            .iter()
            .filter_map(|face| self.domains.get(face))
            .flat_map(FaceDomain::edges)
            .copied()
            .collect();
        let mut cuts = vec![range.lo(), range.hi()];
        let bounds = curve.bounds(range);
        for id in edges {
            let Ok(edge) = self.model.edge(id) else {
                continue;
            };
            for id in [edge.start(), edge.end()] {
                if let Ok(vertex) = self.model.vertex(id)
                    && let Ok(hit) = curve.project(vertex.point())
                    && hit.distance <= tolerance.linear.max(vertex.tolerance())
                    && let Some(t) = in_range(hit.t, curve, range)
                {
                    cuts.push(t);
                }
            }
            let Some((id, edge_range)) = edge.curve() else {
                continue;
            };
            let Ok(edge_curve) = self.model.curve(id) else {
                continue;
            };
            let tol = Tolerance::new(tolerance.linear.max(edge.tolerance()), tolerance.angular);
            if let (Some(bounds), Some(edge_bounds)) = (bounds, edge_curve.bounds(edge_range))
                && !bounds.inflated(tol.linear).intersects(&edge_bounds)
            {
                continue;
            }
            let section = intersect_curves(curve, edge_curve, tol, meter);
            if meter.stopped().is_some() {
                return Vec::new();
            }
            if let Ok(CurveIntersection::Points(hits)) = section {
                cuts.extend(hits.into_iter().filter_map(|hit| {
                    in_range(hit.tb, edge_curve, edge_range)?;
                    in_range(hit.ta, curve, range)
                }));
            }
        }
        cuts.sort_by(f64::total_cmp);
        cuts.dedup();
        cuts.windows(2).map(|w| w[0].midpoint(w[1])).collect()
    }

    /// Test the supplied section parameters against both trimmed faces.
    /// The section can be a curve or a branch known only by its points.
    #[allow(clippy::too_many_arguments)]
    fn section_is_interior_to_both(
        &self,
        a: FaceId,
        sa: &Surface,
        b: FaceId,
        sb: &Surface,
        samples: impl IntoIterator<Item = f64>,
        at: &dyn Fn(f64) -> Point3,
        tolerance: f64,
        meeting: Meeting,
    ) -> bool {
        let shared = self.shared_edges(a, b);
        let inside = |face: FaceId, uv: Point2, point: Point3| {
            let Some(domain) = self.domains.get(&face) else {
                return false;
            };
            match domain.side(uv).0 {
                Side::Inside => true,
                // A seam used twice by this face is interior in 3D. The
                // boolean's contact test counts it even though the chart
                // puts it on the boundary.
                Side::Boundary if meeting == Meeting::Touching => {
                    let Ok(Some(on)) = domain.boundary_entity(self.model, point) else {
                        return false;
                    };
                    let EntityId::Edge(edge) = on.id else {
                        return false;
                    };
                    self.model.face(face).is_ok_and(|f| {
                        coedges(f)
                            .filter(|(_, _, c)| c.edge() == edge)
                            .nth(1)
                            .is_some()
                    })
                }
                Side::Boundary | Side::Outside => false,
            }
        };
        for t in samples {
            let point = at(t);
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
            if inside(a, pa.uv, point) && inside(b, pb.uv, point) {
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
    ///
    /// The flux is taken about the centre of the box round the shell's
    /// own vertices, not the origin: the faces of a shell close only to
    /// their tolerance, each gap leaks flux in proportion to its distance
    /// from the point the integral is taken about, and a sliver shell
    /// posed 86 from the origin enclosing 3e-13 lost its sign to the
    /// leak about the origin, where about itself the leak is a thousand
    /// times smaller — as `arris_ops::measure::mass_properties` does for
    /// the same reason.
    pub(crate) fn shell_volume(&self, shell_id: ShellId, outer: Orientation) -> Option<f64> {
        let shell = self.model.shell(shell_id).ok()?;
        let reference = self.shell_reference(shell)?;
        let mut total = 0.0;
        for face_use in shell.faces() {
            let volume = face_flux(self.model, face_use.id, |p, n| {
                (p.coords - reference).dot(&n) / 3.0
            })
            .ok()?;
            total += outer.compose(face_use.orientation).sign() * volume;
        }
        Some(total)
    }

    /// The centre of the box round the vertices of `shell`'s edges, the
    /// origin for a shell with none; `None` when a reference does not
    /// resolve.
    fn shell_reference(&self, shell: &arris_topo::entity::Shell) -> Option<Vec3> {
        let mut points: Vec<[f64; 3]> = Vec::new();
        for face_use in shell.faces() {
            let face = self.model.face(face_use.id).ok()?;
            for (_, _, c) in coedges(face) {
                let edge = self.model.edge(c.edge()).ok()?;
                for v in [edge.start(), edge.end()] {
                    points.push(self.model.vertex(v).ok()?.point().coords.into());
                }
            }
        }
        Some(Aabb::of_points(&points).map_or_else(Vec3::zeros, |b| Vec3::from(b.center())))
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

    /// Points of `shell` to place it by, best first: the start vertices of
    /// its edges in face order, each once, then interior points of its
    /// faces ([`Checker::face_interior_points`]), so that a shell whose
    /// every vertex is on a contact with another lump (a cylinder's two
    /// seam vertices, on the line another cylinder touches it along) is
    /// still placed. Empty when nothing resolves.
    pub(crate) fn shell_points(&self, shell: ShellId) -> impl Iterator<Item = Point3> + '_ {
        let model = self.model;
        let faces: Vec<FaceId> = model.shell(shell).ok().map_or_else(Vec::new, |s| {
            s.faces()
                .iter()
                .map(|f| f.id)
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect()
        });
        let mut seen = BTreeSet::new();
        let vertices: Vec<Point3> = faces
            .iter()
            .filter_map(|&f| model.face(f).ok())
            .flat_map(coedges)
            .filter_map(|(_, _, c)| model.edge(c.edge()).ok())
            .map(|e| e.start())
            .filter(|&v| seen.insert(v))
            .filter_map(|v| model.vertex(v).ok())
            .map(|v| v.point())
            .collect();
        vertices.into_iter().chain(
            faces
                .into_iter()
                .flat_map(move |f| self.face_interior_points(f)),
        )
    }

    /// A deterministic grid of points strictly inside `face`: the
    /// `check_samples`-per-side grid over its (u, v) box, its cells'
    /// centres that [`FaceDomain::side`] holds inside the loops, carried
    /// to 3D. Empty for a face with no domain.
    fn face_interior_points(&self, face: FaceId) -> Vec<Point3> {
        let Some(domain) = self.domains.get(&face) else {
            return Vec::new();
        };
        let points: Vec<Point2> = domain
            .polygons()
            .iter()
            .flat_map(|p| p.points())
            .copied()
            .collect();
        let (Some(lo), Some(hi)) = (corner(&points, f64::min), corner(&points, f64::max)) else {
            return Vec::new();
        };
        let n = self.precision.check_samples.max(2);
        let at = |k: usize| (k as f64 + 0.5) / n as f64;
        let mut out = Vec::new();
        for i in 0..n {
            for j in 0..n {
                let uv = Point2::new(lerp(lo.x, hi.x, at(i)), lerp(lo.y, hi.y, at(j)));
                if domain.side(uv).0 == Side::Inside {
                    out.push(domain.surface().point(uv.x, uv.y));
                }
            }
        }
        out
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

/// A face's surface over a rectangle of its (u, v) domain, with the face
/// whose loops cut it.
#[derive(Clone, Copy)]
struct Patch<'a> {
    surface: &'a Surface,
    domain: &'a FaceDomain<'a>,
    uv: [Interval; 2],
}

impl Patch<'_> {
    /// The box of the surface over the rectangle, `None` for one no box
    /// bounds.
    fn bounds(&self) -> Option<Aabb> {
        self.surface.bounds(self.uv)
    }

    /// The four quarters of the rectangle that the face reaches, `None`
    /// when the rectangle cannot be halved.
    fn quarters(&self) -> Option<Vec<Self>> {
        let halves = |r: Interval| {
            let mid = 0.5 * (r.lo() + r.hi());
            Some([
                Interval::new(r.lo(), mid).ok()?,
                Interval::new(mid, r.hi()).ok()?,
            ])
        };
        let (us, vs) = (halves(self.uv[0])?, halves(self.uv[1])?);
        Some(
            us.into_iter()
                .flat_map(|u| vs.into_iter().map(move |v| [u, v]))
                .filter(|&uv| !self.domain.misses(uv))
                .map(|uv| Patch { uv, ..*self })
                .collect(),
        )
    }
}

/// The two patches have no point within `tolerance` of each other: their
/// boxes are clear, or each quarter of one the face reaches is clear of
/// each such quarter of the other. `budget` counts the boxes still to be
/// bounded; spent, the answer is `false`.
fn patches_clear(a: &Patch, b: &Patch, tolerance: f64, depth: u32, budget: &mut usize) -> bool {
    let (Some(ba), Some(bb)) = (a.bounds(), b.bounds()) else {
        return false;
    };
    if !ba.inflated(tolerance).intersects(&bb) {
        return true;
    }
    if depth == 0 {
        return false;
    }
    let (Some(qa), Some(qb)) = (a.quarters(), b.quarters()) else {
        return false;
    };
    for x in &qa {
        for y in &qb {
            if *budget < 2 {
                return false;
            }
            *budget -= 2;
            if !patches_clear(x, y, tolerance, depth - 1, budget) {
                return false;
            }
        }
    }
    true
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
