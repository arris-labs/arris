//! The pave model's first phases: the edge-on-face hits and the crossings of edges and of section curves, merged into section vertices, and the paves they put on the operands' edges.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::PoisonError;

use arris_geom::region2::{MAX_SEGMENTS_PER_PIECE, Side};
use arris_geom::{
    Curve, CurveIntersection, CurveSurfaceIntersection, MeetKind, PCURVE_SINGULAR_BAND,
    SurfaceIntersection, conic_crossings, intersect_curve_surface, intersect_curves,
    intersect_surfaces,
};
use arris_math::{Point2, Point3, wrap_into};
use arris_topo::{EdgeId, FaceId, Shape, Vertex as VertexHandle, VertexId};

use super::{
    Build, Candidate, Member, SINGULAR_CLEARANCE, VertexBuild, at_shared_end, components, geometry,
    shape_of, straight_through, tolerance_of,
};
use crate::boolean::faces::{EdgeInfo, FaceInfo};
use crate::boolean::{
    EdgeEdgeHit, EdgeFaceHit, FacePair, Landing, Pave, SectionCrossing, TriplePoint, meet_curves,
};
use crate::error::{BooleanReason, Fault, OpError, Reason};
use crate::pass::pass;

/// A candidate face pair: face `ia` of operand `oa` and face `ib` of
/// operand `ob`, `oa < ob`.
#[derive(Clone, Copy)]
pub(super) struct Candidate2 {
    oa: usize,
    ia: usize,
    ob: usize,
    ib: usize,
}

impl<'m, 'c> Build<'m, 'c> {
    /// Every face pair of two operands whose boxes overlap, intersected:
    /// operand pairs in ascending order, the lower operand's faces outer.
    pub(super) fn face_pairs(&mut self) -> Result<(), OpError> {
        let mut candidates: Vec<Candidate2> = Vec::new();
        for oa in 0..self.operands.len() {
            for ob in oa + 1..self.operands.len() {
                for (ia, fa) in self.faces[oa].iter().enumerate() {
                    for (ib, fb) in self.faces[ob].iter().enumerate() {
                        if fa.bounds.intersects(&fb.bounds) {
                            candidates.push(Candidate2 { oa, ia, ob, ib });
                        }
                    }
                }
            }
        }
        for (c, intersection) in candidates
            .iter()
            .copied()
            .zip(self.intersect_pairs(&candidates)?)
        {
            self.pairs.push(FacePair {
                a: self.faces[c.oa][c.ia].id,
                b: self.faces[c.ob][c.ib].id,
                operands: [c.oa, c.ob],
                intersection,
            });
            self.pair_faces.push((c.ia, c.ib));
        }
        Ok(())
    }

    /// [`intersect_surfaces`] over each candidate pair, in the
    /// candidates' order in the result however it was computed: over
    /// `rayon` behind `parallel`, a plain iterator otherwise. A pair's
    /// intersection reads two surfaces and their tolerances and touches
    /// nothing else, which is what makes it the pave model's parallel
    /// step (ADR-0004, `docs/ARCHITECTURE.md` §Threading).
    pub(super) fn intersect_pairs(
        &mut self,
        candidates: &[Candidate2],
    ) -> Result<Vec<SurfaceIntersection>, OpError> {
        let mut meter = *self.meter.get_mut().unwrap_or_else(PoisonError::into_inner);
        let found = pass(
            candidates,
            &mut meter,
            |&Candidate2 { oa, ia, ob, ib }, mt| {
                let (fa, fb) = (&self.faces[oa][ia], &self.faces[ob][ib]);
                let tol = tolerance_of(&self.precision, fa.tolerance, fb.tolerance);
                mt.tick()?;
                intersect_surfaces(fa.surface, fb.surface, &self.within[&[oa, ob]], tol, mt)
                    .map_err(|e| geometry(e, fa.shape(), fb.shape()))
            },
        );
        *self.meter.get_mut().unwrap_or_else(PoisonError::into_inner) = meter;
        found
    }

    /// Every edge of each operand against every face of every other
    /// whose box it reaches; the hits sorted by `(edge id, t)`, the
    /// coincident edge–face pairs recorded.
    pub(super) fn hits(&mut self) -> Result<(), OpError> {
        let mut found: Vec<(EdgeFaceHit, f64)> = Vec::new();
        let mut coincident = Vec::new();
        for side in 0..self.operands.len() {
            for other in (0..self.operands.len()).filter(|&o| o != side) {
                for e in &self.edges[side] {
                    for f in &self.faces[other] {
                        if !e.bounds.intersects(&f.bounds) {
                            continue;
                        }
                        self.hit_edge_face(e, f, &mut found, &mut coincident)?;
                    }
                }
            }
        }
        found.sort_by(|x, y| {
            x.0.edge
                .cmp(&y.0.edge)
                .then_with(|| x.0.t.total_cmp(&y.0.t))
                .then_with(|| x.0.face.cmp(&y.0.face))
        });
        for (hit, tolerance) in found {
            self.hits.push(hit);
            self.hit_tolerance.push(tolerance);
        }
        self.coincident = coincident;
        Ok(())
    }

    /// The hits of one edge on one face.
    pub(super) fn hit_edge_face(
        &self,
        e: &EdgeInfo<'m>,
        f: &FaceInfo<'m>,
        found: &mut Vec<(EdgeFaceHit, f64)>,
        coincident: &mut Vec<(EdgeId, FaceId)>,
    ) -> Result<(), OpError> {
        self.tick()?;
        let tol = tolerance_of(&self.precision, e.tolerance, f.tolerance);
        let hits = match self
            .metered(|mt| intersect_curve_surface(e.curve, f.surface, tol, mt))
            .map_err(|err| geometry(err, e.shape(), f.shape()))?
        {
            CurveSurfaceIntersection::Coincident => {
                coincident.push((e.id, f.id));
                return Ok(());
            }
            CurveSurfaceIntersection::Points(hits) => hits,
        };
        for h in hits {
            if let Some(hit) = self.land(e, f, h.t, h.uv, h.point, h.tangent)? {
                found.push((hit, tol.linear));
            }
        }
        Ok(())
    }

    /// A point of `e`'s curve on `f`'s surface as a hit: `None` when the
    /// parameter is outside the edge's range or the (u, v) outside the
    /// face.
    pub(super) fn land(
        &self,
        e: &EdgeInfo<'m>,
        f: &FaceInfo<'m>,
        t: f64,
        uv: Point2,
        point: Point3,
        tangent: bool,
    ) -> Result<Option<EdgeFaceHit>, OpError> {
        let Some(t) = e.in_range(t) else {
            return Ok(None);
        };
        let (side, shift) = f.domain.side(uv);
        let landing = match side {
            Side::Outside => return Ok(None),
            Side::Inside => Landing::Interior,
            Side::Boundary => match f.domain.boundary_entity(self.m, point)? {
                Some(shape) => Landing::Boundary(shape),
                // Within the (u, v) band of a loop's polygon but within
                // no edge's or vertex's own tolerance: not on the
                // boundary, so the winding number alone decides.
                None if f.domain.winds_around(uv) => Landing::Interior,
                None => return Ok(None),
            },
        };
        Ok(Some(EdgeFaceHit {
            edge: e.id,
            face: f.id,
            t,
            uv: uv + shift,
            point,
            tangent,
            landing,
            at_vertex: e.vertex_at(point),
            vertex: None,
        }))
    }

    /// The crossings a touch that landed on no vertex stands for: where
    /// the touching edge crosses a section curve of the touched face and
    /// a face of its own, as hits (ADR-0016). The intersector's touch is
    /// a verdict on depth, and a chord `h` deep is `2√(2Rh)` long — 7e-4
    /// at `h` = 6e-8 and `R` = 1 — so one touch can stand for two
    /// crossings far apart, which the edge against the surface places
    /// only to a square root of rounding. The edge against the curve the
    /// two surfaces meet in is two curves of one surface crossing at an
    /// angle, and exact. It matters where the two surfaces are tangent to
    /// each other, the crossing of two ellipses: there the section curves
    /// leave a touch at any distance, and a seam beside the crossing is
    /// cut by both ellipses with no hit to pave it. A crossing already
    /// among the hits of the edge on the face is not made twice.
    pub(super) fn resolve_touch(&self, touch: usize) -> Result<Vec<EdgeFaceHit>, OpError> {
        let (edge, face) = (self.hits[touch].edge, self.hits[touch].face);
        let Some(side) = self.operand_of_edge(edge) else {
            return Ok(Vec::new());
        };
        let Some((other, f)) = (0..self.operands.len())
            .filter(|&o| o != side)
            .find_map(|o| Some((o, self.faces[o].iter().find(|f| f.id == face)?)))
        else {
            return Ok(Vec::new());
        };
        let Some(e) = self.edge_info(side, edge) else {
            return Ok(Vec::new());
        };
        let tol = tolerance_of(&self.precision, e.tolerance, f.tolerance);
        let mut found: Vec<EdgeFaceHit> = Vec::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            if !(pair.operands == [side, other] || pair.operands == [other, side]) {
                continue;
            }
            let (ia, ib) = self.pair_faces[pi];
            let (own, theirs) = if pair.operands[0] == side {
                (ia, ib)
            } else {
                (ib, ia)
            };
            let g = &self.faces[side][own];
            if self.faces[other][theirs].id != face || !g.edges().contains(&edge) {
                continue;
            }
            for (_, curve) in meet_curves(&pair.intersection, MeetKind::Crossing) {
                // An edge known by its surfaces to run along the section
                // curve crosses nothing there, as a coincident one below.
                if self.known_by_surfaces(side, e, g, f, curve, tol) == Some(true) {
                    continue;
                }
                self.tick()?;
                let hits = match self
                    .metered(|mt| intersect_curves(e.curve, curve, tol, mt))
                    .map_err(|err| geometry(err, e.shape(), f.shape()))?
                {
                    // The edge runs along the section curve: a block of
                    // it is that edge, and nothing crosses.
                    CurveIntersection::Coincident => continue,
                    CurveIntersection::Points(hits) => hits,
                };
                // A touch of two curves is the intersector's verdict on
                // depth again, one level down: two conics of one surface
                // crossing at a shallow angle twice are one touch as long
                // as they stay within the tolerance between, however far
                // apart the crossings. Where both are conics their common
                // points are exact along the line their planes share.
                let hits = if hits.iter().any(|h| h.tangent) {
                    conic_crossings(e.curve, curve, tol)
                        .map_err(|err| geometry(err, e.shape(), f.shape()))?
                        .unwrap_or(hits)
                } else {
                    hits
                };
                for h in hits {
                    if h.tangent {
                        continue;
                    }
                    let Ok(projection) = f.surface.project(h.point) else {
                        continue;
                    };
                    let Some(hit) = self.land(e, f, h.ta, projection.uv, h.point, false)? else {
                        continue;
                    };
                    let seen = self
                        .hits
                        .iter()
                        .filter(|x| x.edge == edge && x.face == face && !x.tangent)
                        .chain(found.iter())
                        .any(|x| (x.point - hit.point).norm() <= tol.linear);
                    if !seen {
                        found.push(hit);
                    }
                }
            }
        }
        Ok(found)
    }

    /// The edges of the two faces of every `Coincident` pair against one
    /// another: a crossing in both ranges is recorded, sorted by
    /// `(a's edge, ta, b's edge, tb)` and made once per edge pair and
    /// point; a pair on the same curve is remembered for the common
    /// blocks.
    pub(super) fn crossings(&mut self) -> Result<(), OpError> {
        let mut found: Vec<(EdgeEdgeHit, f64)> = Vec::new();
        let mut same_curve = BTreeSet::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            if pair.intersection != SurfaceIntersection::Coincident {
                continue;
            }
            let [oa, ob] = pair.operands;
            let (fa, fb) = self.pair_infos(pi);
            for &ea in fa.edges() {
                let Some(ea) = self.edge_info(oa, ea) else {
                    continue;
                };
                for &eb in fb.edges() {
                    let Some(eb) = self.edge_info(ob, eb) else {
                        continue;
                    };
                    if !ea.bounds.intersects(&eb.bounds) {
                        continue;
                    }
                    let tol = tolerance_of(&self.precision, ea.tolerance, eb.tolerance);
                    // Two edges whose other faces lie on one surface too
                    // run along one section of the two, and are the same
                    // curve where one lies on the other at all.
                    if self.same_section(ea, fa, eb, fb, tol) {
                        same_curve.insert((ea.id, eb.id));
                        continue;
                    }
                    self.tick()?;
                    let hits = match self
                        .metered(|mt| intersect_curves(ea.curve, eb.curve, tol, mt))
                        .map_err(|e| geometry(e, ea.shape(), eb.shape()))?
                    {
                        CurveIntersection::Coincident => {
                            same_curve.insert((ea.id, eb.id));
                            continue;
                        }
                        CurveIntersection::Points(hits) => hits,
                    };
                    for h in hits {
                        let (Some(ta), Some(tb)) = (ea.in_range(h.ta), eb.in_range(h.tb)) else {
                            continue;
                        };
                        let (ta, tb, point) = at_shared_end(ea, ta, eb, tb, h.point, tol)
                            .unwrap_or((ta, tb, h.point));
                        let seen = found.iter().any(|(x, t)| {
                            x.a == ea.id
                                && x.b == eb.id
                                && (x.point - point).norm() <= t.max(tol.linear)
                        });
                        if seen {
                            continue;
                        }
                        found.push((
                            EdgeEdgeHit {
                                pair: pi,
                                a: ea.id,
                                ta,
                                b: eb.id,
                                tb,
                                point,
                                tangent: h.tangent,
                                vertex: None,
                            },
                            tol.linear,
                        ));
                    }
                }
            }
        }
        found.sort_by(|x, y| {
            x.0.a
                .cmp(&y.0.a)
                .then_with(|| x.0.ta.total_cmp(&y.0.ta))
                .then_with(|| x.0.b.cmp(&y.0.b))
                .then_with(|| x.0.tb.total_cmp(&y.0.tb))
        });
        for (hit, tolerance) in found {
            self.crossings.push(hit);
            self.crossing_tolerance.push(tolerance);
        }
        self.same_curve = same_curve;
        Ok(())
    }

    /// The curves of every crossing pair against one another: a
    /// crossing on both faces is recorded, sorted by `(pair, curves, t on
    /// the first)`. Two curves of one pair meet where the surfaces are
    /// tangent to each other — the two ellipses of equal cylinders with
    /// crossing axes — and no edge of either operand is there to make a
    /// hit, so the crossing makes the section vertex both curves need.
    pub(super) fn section_crossings(&mut self) -> Result<(), OpError> {
        let mut found: Vec<(SectionCrossing, f64)> = Vec::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            let (fa, fb) = self.pair_infos(pi);
            let tol = tolerance_of(&self.precision, fa.tolerance, fb.tolerance);
            let curves: Vec<(usize, &Curve)> =
                meet_curves(&pair.intersection, MeetKind::Crossing).collect();
            if curves.iter().any(|(_, c)| matches!(c, Curve::Nurbs(_))) {
                // A traced section's branches meet only at its singular
                // points, where they end exactly (ADR-0018): the crossings
                // are those, not two fitted curves intersected.
                for crossing in self.singular_crossings(pi, &curves, tol.linear)? {
                    found.push((crossing, tol.linear));
                }
                continue;
            }
            for (k, &(ci, ca)) in curves.iter().enumerate() {
                for &(cj, cb) in &curves[k + 1..] {
                    self.tick()?;
                    let hits = match self
                        .metered(|mt| intersect_curves(ca, cb, tol, mt))
                        .map_err(|e| geometry(e, fa.shape(), fb.shape()))?
                    {
                        // Two distinct curves of one intersection are
                        // never the same curve.
                        CurveIntersection::Coincident => {
                            return Err(OpError::Internal(Fault::Invariant {
                                what: "two section curves of one pair coinciding",
                            }));
                        }
                        CurveIntersection::Points(hits) => hits,
                    };
                    for h in hits {
                        if self.on_face(fa, h.point)?.is_none()
                            || self.on_face(fb, h.point)?.is_none()
                        {
                            continue;
                        }
                        found.push((
                            SectionCrossing {
                                pair: pi,
                                curves: [ci, cj],
                                t: [
                                    ca.period()
                                        .map_or(h.ta, |p| wrap_into(h.ta, ca.domain().lo(), p)),
                                    cb.period()
                                        .map_or(h.tb, |p| wrap_into(h.tb, cb.domain().lo(), p)),
                                ],
                                point: h.point,
                                tangent: h.tangent,
                                vertex: None,
                            },
                            tol.linear,
                        ));
                    }
                }
            }
        }
        found.sort_by(|x, y| {
            x.0.pair
                .cmp(&y.0.pair)
                .then_with(|| x.0.curves.cmp(&y.0.curves))
                .then_with(|| x.0.t[0].total_cmp(&y.0.t[0]))
        });
        for (crossing, tolerance) in found {
            self.section_crossings.push(crossing);
            self.section_crossing_tolerance.push(tolerance);
        }
        Ok(())
    }

    /// The triple points (ADR-0050 §5): each crossing curve of a pair of
    /// faces of operands `i < j` against every face of an operand `k > j`
    /// whose box reaches both, a crossing on all three faces recorded,
    /// sorted by `(pair, curve, face, t)`. Each point of three faces is
    /// found once, from its two lower operands' curve, and the other two
    /// curves through it are paved by its vertex as by any other. A curve
    /// lying in the third face's surface is three faces along one curve,
    /// not a point, and a touch splits nothing: neither makes one.
    pub(super) fn triple_points(&mut self) -> Result<(), OpError> {
        let mut found: Vec<(TriplePoint, f64)> = Vec::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            let [_, ob] = pair.operands;
            if ob + 1 >= self.operands.len() {
                continue;
            }
            let (fa, fb) = self.pair_infos(pi);
            for (ci, curve) in meet_curves(&pair.intersection, MeetKind::Crossing) {
                for fc in self.faces[ob + 1..].iter().flatten() {
                    if !(fc.bounds.intersects(&fa.bounds) && fc.bounds.intersects(&fb.bounds)) {
                        continue;
                    }
                    self.tick()?;
                    let tol = tolerance_of(
                        &self.precision,
                        fa.tolerance.max(fb.tolerance),
                        fc.tolerance,
                    );
                    let hits = match self
                        .metered(|mt| intersect_curve_surface(curve, fc.surface, tol, mt))
                        .map_err(|e| geometry(e, fa.shape(), fc.shape()))?
                    {
                        CurveSurfaceIntersection::Coincident => continue,
                        CurveSurfaceIntersection::Points(hits) => hits,
                    };
                    for h in hits {
                        if h.tangent {
                            continue;
                        }
                        let mut on_all = true;
                        for f in [fa, fb, fc] {
                            if self.on_face(f, h.point)?.is_none() {
                                on_all = false;
                                break;
                            }
                        }
                        if !on_all {
                            continue;
                        }
                        found.push((
                            TriplePoint {
                                pair: pi,
                                curve: ci,
                                face: fc.id,
                                t: curve
                                    .period()
                                    .map_or(h.t, |p| wrap_into(h.t, curve.domain().lo(), p)),
                                point: h.point,
                                vertex: None,
                            },
                            tol.linear,
                        ));
                    }
                }
            }
        }
        found.sort_by(|x, y| {
            x.0.pair
                .cmp(&y.0.pair)
                .then_with(|| x.0.curve.cmp(&y.0.curve))
                .then_with(|| x.0.face.cmp(&y.0.face))
                .then_with(|| x.0.t.total_cmp(&y.0.t))
        });
        for (point, tolerance) in found {
            self.triple_points.push(point);
            self.triple_point_tolerance.push(tolerance);
        }
        Ok(())
    }

    /// The crossings a traced pair's singular points make: each `Crossing`
    /// point of the pair's `Meets` on both faces, with the branches that
    /// end there — within `tolerance` of it, which they end at exactly —
    /// the first against each other, or against itself when one branch
    /// both starts and ends there. A `Touch` point, where the surfaces
    /// meet at that point alone, splits nothing and makes no vertex, as a
    /// touch off every vertex makes none.
    pub(super) fn singular_crossings(
        &self,
        pi: usize,
        curves: &[(usize, &Curve)],
        tolerance: f64,
    ) -> Result<Vec<SectionCrossing>, OpError> {
        let (fa, fb) = self.pair_infos(pi);
        let mut out = Vec::new();
        for p in self.pairs[pi].intersection.points() {
            if p.kind != MeetKind::Crossing {
                continue;
            }
            if self.on_face(fa, p.point)?.is_none() || self.on_face(fb, p.point)?.is_none() {
                continue;
            }
            let mut ends: Vec<(usize, f64)> = Vec::new();
            for &(ci, c) in curves {
                if c.period().is_some() {
                    continue;
                }
                let domain = c.domain();
                for t in [domain.lo(), domain.hi()] {
                    if (c.point(t) - p.point).norm() <= tolerance {
                        ends.push((ci, t));
                    }
                }
            }
            let Some((&first, rest)) = ends.split_first() else {
                return Err(OpError::Internal(Fault::Invariant {
                    what: "a singular point of a traced section that no branch ends at",
                }));
            };
            for &other in rest {
                let [(ci, ti), (cj, tj)] = if other.0 < first.0 {
                    [other, first]
                } else {
                    [first, other]
                };
                out.push(SectionCrossing {
                    pair: pi,
                    curves: [ci, cj],
                    t: [ti, tj],
                    point: p.point,
                    tangent: false,
                    vertex: None,
                });
            }
        }
        Ok(out)
    }

    /// Hits, then crossings, then section crossings, then the singular
    /// vertices a crossing curve runs through, as candidate points; the
    /// section vertices are their components under "the same within a
    /// tolerance" ([`components`]), not the first vertex each point
    /// happened to reach. A touch whose component holds one of those
    /// joins it; any other touch joins nothing itself, while the crossings
    /// it stands for ([`Self::resolve_touch`]) are hits, candidates like
    /// the rest, and the components are taken again with them. A vertex
    /// is numbered by its first member in that order and carries that
    /// member's source.
    pub(super) fn merge(&mut self) -> Result<(), OpError> {
        let mut members: Vec<(Member, Candidate)> = Vec::new();
        for (i, h) in self.hits.iter().enumerate() {
            if h.tangent {
                continue;
            }
            let mut existing: Vec<VertexId> = Vec::new();
            if let Some(v) = h.at_vertex {
                existing.push(v);
            }
            if let Landing::Boundary(shape) = h.landing {
                if let Ok(v) = VertexHandle::try_from(shape) {
                    if !existing.contains(&v.id) {
                        existing.push(v.id);
                    }
                }
            }
            members.push((
                Member::Hit(i),
                Candidate {
                    point: h.point,
                    tolerance: self.hit_tolerance[i],
                    existing,
                },
            ));
        }
        for (i, x) in self.crossings.iter().enumerate() {
            if x.tangent {
                continue;
            }
            let mut existing: Vec<VertexId> = Vec::new();
            let operands = self.pairs[x.pair].operands;
            for (side, id) in [(operands[0], x.a), (operands[1], x.b)] {
                if let Some(v) = self.edge_info(side, id).and_then(|e| e.vertex_at(x.point)) {
                    if !existing.contains(&v) {
                        existing.push(v);
                    }
                }
            }
            members.push((
                Member::Crossing(i),
                Candidate {
                    point: x.point,
                    tolerance: self.crossing_tolerance[i],
                    existing,
                },
            ));
        }
        for (i, x) in self.section_crossings.iter().enumerate() {
            if x.tangent {
                continue;
            }
            members.push((
                Member::SectionCrossing(i),
                Candidate {
                    point: x.point,
                    tolerance: self.section_crossing_tolerance[i],
                    existing: Vec::new(),
                },
            ));
        }
        for (i, x) in self.triple_points.iter().enumerate() {
            members.push((
                Member::TriplePoint(i),
                Candidate {
                    point: x.point,
                    tolerance: self.triple_point_tolerance[i],
                    existing: Vec::new(),
                },
            ));
        }
        for (point, tolerance, vertex) in self.singular_vertices()? {
            members.push((
                Member::Singular,
                Candidate {
                    point,
                    tolerance,
                    existing: vec![vertex],
                },
            ));
        }
        // A touch makes no vertex of its own, but one landing on a vertex
        // made above passes through it: a ruling or a rim circle through
        // the crossing of two ellipses, where the walls are tangent to
        // each other. It joins that vertex, which then paves its edge. A
        // touch at its edge's own end vertex is a node of the loops there
        // already — a hole's rim touching a plane at its seam, the
        // plane's section across the plate running through it — and is a
        // vertex like a hit at a vertex, which paves that section.
        let made = members.len();
        let touches: Vec<(Member, Candidate)> = self
            .hits
            .iter()
            .enumerate()
            .filter(|(_, h)| h.tangent)
            .map(|(i, h)| {
                (
                    Member::Hit(i),
                    Candidate {
                        point: h.point,
                        tolerance: self.hit_tolerance[i],
                        existing: h.at_vertex.into_iter().collect(),
                    },
                )
            })
            .collect();
        let mut off_every_vertex: Vec<usize> = Vec::new();
        {
            let all: Vec<&Candidate> = members.iter().chain(&touches).map(|(_, c)| c).collect();
            let label = self.labels(&all)?;
            let reached: BTreeSet<usize> = label[..made].iter().copied().collect();
            for (k, (member, candidate)) in touches.into_iter().enumerate() {
                if reached.contains(&label[made + k]) || !candidate.existing.is_empty() {
                    members.push((member, candidate));
                } else if let Member::Hit(i) = member {
                    off_every_vertex.push(i);
                }
            }
        }
        // A touch off every vertex may still stand for crossings: those
        // are hits like any other.
        let mut resolved = false;
        for i in off_every_vertex {
            let hit_tol = self.hit_tolerance[i];
            for hit in self.resolve_touch(i)? {
                members.push((
                    Member::Hit(self.hits.len()),
                    Candidate {
                        point: hit.point,
                        tolerance: hit_tol,
                        existing: hit.at_vertex.into_iter().collect(),
                    },
                ));
                self.hits.push(hit);
                self.hit_tolerance.push(hit_tol);
                resolved = true;
            }
        }
        let all: Vec<&Candidate> = members.iter().map(|(_, c)| c).collect();
        let label = self.labels(&all)?;
        let mut vertex_of: BTreeMap<usize, usize> = BTreeMap::new();
        for ((member, candidate), l) in members.into_iter().zip(label) {
            let mut base = candidate.tolerance;
            for &v in &candidate.existing {
                base = base.max(self.m.vertex(v)?.tolerance());
            }
            let k = match vertex_of.get(&l) {
                Some(&k) => {
                    let v = &mut self.vertices[k];
                    v.points.push(candidate.point);
                    if v.on_edge.is_none() && member.on_edge() {
                        v.on_edge = Some(candidate.point);
                    }
                    v.base = v.base.max(base);
                    for x in candidate.existing {
                        if !v.existing.contains(&x) {
                            v.existing.push(x);
                        }
                    }
                    v.existing.sort();
                    k
                }
                None => {
                    self.vertices.push(VertexBuild {
                        points: vec![candidate.point],
                        on_edge: member.on_edge().then_some(candidate.point),
                        base,
                        floor: 0.0,
                        hits: Vec::new(),
                        crossings: Vec::new(),
                        section_crossings: Vec::new(),
                        triple_points: Vec::new(),
                        existing: candidate.existing,
                        source: member.source(),
                    });
                    vertex_of.insert(l, self.vertices.len() - 1);
                    self.vertices.len() - 1
                }
            };
            let v = &mut self.vertices[k];
            match member {
                Member::Hit(i) => {
                    v.hits.push(i);
                    self.hits[i].vertex = Some(k);
                }
                Member::Crossing(i) => {
                    v.crossings.push(i);
                    self.crossings[i].vertex = Some(k);
                }
                Member::SectionCrossing(i) => {
                    v.section_crossings.push(i);
                    self.section_crossings[i].vertex = Some(k);
                }
                Member::TriplePoint(i) => {
                    v.triple_points.push(i);
                    self.triple_points[i].vertex = Some(k);
                }
                Member::Singular => {}
            }
        }
        for v in &mut self.vertices {
            v.hits.sort_unstable();
        }
        if resolved {
            self.sort_hits();
        }
        self.one_per_operand()?;
        let m = self.m;
        for v in &self.vertices {
            let wanted = v.tolerance(m);
            if wanted > self.precision.max_tolerance {
                let entity = v
                    .hits
                    .first()
                    .map(|&h| Shape::new(self.hits[h].edge, self.orientation()))
                    .or_else(|| {
                        v.crossings
                            .first()
                            .map(|&x| Shape::new(self.crossings[x].a, self.orientation()))
                    })
                    .or_else(|| {
                        v.section_crossings.first().map(|&x| {
                            Shape::new(
                                self.pairs[self.section_crossings[x].pair].a,
                                self.orientation(),
                            )
                        })
                    })
                    .or_else(|| {
                        v.triple_points
                            .first()
                            .map(|&x| Shape::new(self.triple_points[x].face, self.orientation()))
                    })
                    .unwrap_or_else(|| shape_of(self.operands[0]));
                return Err(OpError::Tolerance { entity, wanted });
            }
        }
        Ok(())
    }

    /// [`components`] over `candidates` and, beside them, every operand
    /// vertex one of them names, as a ball of its own point and
    /// tolerance: a point within reach of that vertex is the same point
    /// as the hit at it. The labels of the candidates alone.
    pub(super) fn labels(&self, candidates: &[&Candidate]) -> Result<Vec<usize>, OpError> {
        let named: BTreeSet<VertexId> = candidates
            .iter()
            .flat_map(|c| c.existing.iter().copied())
            .collect();
        let mut nodes: Vec<Candidate> = candidates
            .iter()
            .map(|c| Candidate {
                point: c.point,
                tolerance: c.tolerance,
                existing: c.existing.clone(),
            })
            .collect();
        for v in named {
            let vertex = self.m.vertex(v)?;
            nodes.push(Candidate {
                point: vertex.point(),
                tolerance: vertex.tolerance(),
                existing: vec![v],
            });
        }
        let mut label = components(&nodes);
        label.truncate(candidates.len());
        Ok(label)
    }

    /// No section vertex holds two vertices of one operand: the edge or
    /// the stretch of face between them would collapse to a point, which
    /// no tolerance of this boolean may do. The two named, as
    /// `OpError::Tolerance` with the tolerance the vertex would need.
    pub(super) fn one_per_operand(&self) -> Result<(), OpError> {
        let m = self.m;
        let own = self
            .operands
            .iter()
            .map(|&body| {
                m.vertices(body)
                    .map(|vs| vs.into_iter().map(|v| v.id).collect::<BTreeSet<_>>())
            })
            .collect::<Result<Vec<_>, _>>()?;
        for v in &self.vertices {
            for side in &own {
                let mut of_side = v.existing.iter().filter(|x| side.contains(x));
                if let (Some(_), Some(&second)) = (of_side.next(), of_side.next()) {
                    return Err(OpError::Tolerance {
                        entity: Shape::new(second, self.orientation()),
                        wanted: v.tolerance(m),
                    });
                }
            }
        }
        Ok(())
    }

    /// Every singular vertex of an operand face — a cone's apex, a
    /// sphere's pole, held by a degenerate edge that pierces nothing and
    /// makes no hit — that a crossing curve of one of the face's pairs
    /// runs through, on the other face of the pair, as a section vertex
    /// over that operand vertex: the one a seam ending there made by
    /// piercing the other face, where it did, and a new one where the
    /// seam only touches it or the face has no seam there. It then paves
    /// the curve like any other,
    /// so no block has the singular point inside it and every pcurve on
    /// the face ends there or stays clear (ADR-0021). *Through* is
    /// [`pcurve_on`]'s own band, decided in length, so the pave and the
    /// pcurve of the block it ends never disagree; a curve outside the
    /// band and inside [`SINGULAR_CLEARANCE`] is
    /// [`BooleanReason::BesideSingularity`].
    pub(super) fn singular_vertices(&self) -> Result<Vec<(Point3, f64, VertexId)>, OpError> {
        let mut wanted: Vec<(Point3, f64, VertexId)> = Vec::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            let (fa, fb) = self.pair_infos(pi);
            let tol = tolerance_of(&self.precision, fa.tolerance, fb.tolerance);
            let band = PCURVE_SINGULAR_BAND * tol.linear;
            for (f, other) in [(fa, fb), (fb, fa)] {
                let clearance =
                    SINGULAR_CLEARANCE * f.bounds.diagonal() / MAX_SEGMENTS_PER_PIECE as f64;
                for s in &f.singular {
                    let (mut through, mut beside) = (false, false);
                    for (_, curve) in meet_curves(&pair.intersection, MeetKind::Crossing) {
                        let Ok(on) = curve.project(s.point) else {
                            continue;
                        };
                        if on.distance <= band && straight_through(f.surface, curve, on.t) {
                            through = true;
                        } else if on.distance <= band.max(clearance) {
                            beside = true;
                        }
                    }
                    // A hit of either face's edge on the other is a point
                    // of the true section. Beside the singular point and
                    // not on its vertex, it says the section misses the
                    // point whatever the curves say: a plane a hair off an
                    // apex meets the cone in two lines through it, decided
                    // in length, and the seam all the same pierces the
                    // plane several tolerances down the ruling.
                    for (k, h) in self.hits.iter().enumerate() {
                        let on_pair = (h.face == other.id && f.edges().contains(&h.edge))
                            || (h.face == f.id && other.edges().contains(&h.edge));
                        let miss = (h.point - s.point).norm();
                        if on_pair
                            && !h.tangent
                            && miss > s.tolerance.max(self.hit_tolerance[k])
                            && miss <= clearance
                        {
                            beside = true;
                        }
                    }
                    if !(through || beside) || self.on_face(other, s.point)?.is_none() {
                        continue;
                    }
                    if beside {
                        return Err(OpError::Degenerate {
                            entities: vec![
                                f.shape(),
                                other.shape(),
                                Shape::new(s.vertex, self.orientation()),
                            ],
                            reason: Reason::Boolean(BooleanReason::BesideSingularity),
                        });
                    }
                    wanted.push((s.point, s.tolerance, s.vertex));
                }
            }
        }
        Ok(wanted)
    }

    /// The paves on the degenerate edges: a section edge ending on a
    /// face's singular vertex arrives there at one `u` of the whole line
    /// of (u, v) the vertex stands for, and the face's arrangement needs a
    /// node at it, so the degenerate edge is paved by that vertex at the
    /// parameter its own pcurve has that (u, v) at — once per arrival, a
    /// curve through a pole leaving it half a turn from where it came in.
    /// An arrival within the angular tolerance of an end of the edge, or
    /// of a pave already there, is that node.
    pub(super) fn pave_singular_edges(&mut self) -> Result<(), OpError> {
        let mut wanted: Vec<(EdgeId, f64, usize, f64)> = Vec::new();
        for section in &self.sections {
            let pair = self.curves[section.curve].pair;
            let (fa, fb) = self.pair_infos(pair);
            for (side, f) in [(0, fa), (1, fb)] {
                let ends = [
                    (section.start, section.range.lo()),
                    (section.end, section.range.hi()),
                ];
                for (vertex, t) in ends {
                    for s in &f.singular {
                        if !self.vertices[vertex].existing.contains(&s.vertex) {
                            continue;
                        }
                        let uv = section.pcurves[side].point(t);
                        let on = s
                            .pcurve
                            .project(uv)
                            .map_err(|e| geometry(e, f.shape(), f.shape()))?;
                        let speed = s.pcurve.eval(on.t).d1.norm();
                        let slack = if speed > 0.0 {
                            self.precision.angular_tolerance / speed
                        } else {
                            0.0
                        };
                        if on.t > s.range.lo() + slack && on.t < s.range.hi() - slack {
                            wanted.push((s.edge, on.t, vertex, slack));
                        }
                    }
                }
            }
        }
        for (edge, t, vertex, slack) in wanted {
            let list = self.paves.entry(edge).or_default();
            if list.iter().all(|p| (p.t - t).abs() > slack) {
                list.push(Pave { t, vertex });
                list.sort_by(|x, y| x.t.total_cmp(&y.t));
            }
        }
        Ok(())
    }

    /// The hits back in `(edge id, t, face)` order after the resolved
    /// ones were appended, the vertices' hit lists following them.
    pub(super) fn sort_hits(&mut self) {
        let mut order: Vec<usize> = (0..self.hits.len()).collect();
        order.sort_by(|&x, &y| {
            let (x, y) = (&self.hits[x], &self.hits[y]);
            x.edge
                .cmp(&y.edge)
                .then_with(|| x.t.total_cmp(&y.t))
                .then_with(|| x.face.cmp(&y.face))
        });
        let mut now = vec![0; order.len()];
        for (new, &old) in order.iter().enumerate() {
            now[old] = new;
        }
        self.hits = order.iter().map(|&old| self.hits[old].clone()).collect();
        self.hit_tolerance = order.iter().map(|&old| self.hit_tolerance[old]).collect();
        for v in &mut self.vertices {
            for h in &mut v.hits {
                *h = now[*h];
            }
            v.hits.sort_unstable();
        }
    }

    /// The paves on the operand edges: each hit's vertex at its `t`,
    /// unless the hit is at the edge's own end or merged into the vertex
    /// that holds it; each crossing's vertex
    /// on both edges likewise; one pave per vertex per edge, ascending
    /// by `t`.
    pub(super) fn pave_edges(&mut self) {
        let mut wanted: Vec<(EdgeId, f64, usize)> = Vec::new();
        for h in &self.hits {
            let (Some(vertex), None) = (h.vertex, h.at_vertex) else {
                continue;
            };
            // Merged into the section vertex that holds the edge's own end,
            // as a crossing's is below: a hit a hair past that end's
            // tolerance would pave the end's vertex beside it.
            let at_end = self
                .operand_of_edge(h.edge)
                .and_then(|side| self.edge_info(side, h.edge))
                .is_some_and(|e| {
                    e.ends
                        .iter()
                        .any(|end| self.vertices[vertex].existing.contains(&end.0))
                });
            if !at_end {
                wanted.push((h.edge, h.t, vertex));
            }
        }
        for x in &self.crossings {
            let Some(vertex) = x.vertex else {
                continue;
            };
            let operands = self.pairs[x.pair].operands;
            for (side, id, t) in [(operands[0], x.a, x.ta), (operands[1], x.b, x.tb)] {
                // At the edge's own end: within that vertex's tolerance,
                // or merged into the section vertex that holds it — a
                // crossing a hair past the end's tolerance would otherwise
                // pave the end's own vertex beside it, a sliver block.
                let at_end = self.edge_info(side, id).is_some_and(|e| {
                    e.vertex_at(x.point).is_some()
                        || e.ends
                            .iter()
                            .any(|end| self.vertices[vertex].existing.contains(&end.0))
                });
                if !at_end {
                    wanted.push((id, t, vertex));
                }
            }
        }
        for (edge, t, vertex) in wanted {
            let list = self.paves.entry(edge).or_default();
            if list.iter().all(|p| p.vertex != vertex) {
                list.push(Pave { t, vertex });
            }
        }
        for list in self.paves.values_mut() {
            list.sort_by(|x, y| x.t.total_cmp(&y.t));
        }
    }
}
