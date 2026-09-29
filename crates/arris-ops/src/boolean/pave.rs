//! Building the pave model (ADR-0004): face pairs, edge-on-face hits,
//! the crossings of a pair's section curves with one another, the hits
//! and crossings merged into section vertices, a touch off every vertex
//! resolved into its crossings through the section curves (ADR-0016),
//! paves, section curves cut into blocks and the blocks kept as section
//! edges with their pcurves; and, for the coincident pairs, the edge–edge
//! crossings, the paves every section vertex puts on the pairs' edges,
//! and each edge piece placed on the other face as an image or matched
//! to a piece of its boundary as a common block.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::{Mutex, PoisonError};

use arris_check::arris_topo::arris_geom::region2::{MAX_SEGMENTS_PER_PIECE, Side};
use arris_check::arris_topo::arris_geom::{
    Curve, Curve2, CurveIntersection, CurveSurfaceIntersection, GeomError, MeetKind,
    PCURVE_SINGULAR_BAND, Surface, SurfaceIntersection, conic_crossings, curves_coincide,
    intersect_curve_surface, intersect_curves, intersect_surfaces, pcurve_ending_on, pcurve_on,
};
use arris_check::arris_topo::arris_math::{
    Aabb, Interval, Meter, Point2, Point3, Precision, RELATIVE_ROUNDING, Tolerance, Vec2,
    period_end,
};
use arris_check::arris_topo::{
    Body, EdgeId, FaceId, Model, Shape, Vertex as VertexHandle, VertexId,
};

use arris_check::domain::bands;

use super::faces::{EdgeInfo, FaceInfo};
use super::{
    CommonBlock, Contact, EdgeEdgeHit, EdgeFaceHit, EdgeImage, FacePair, Interferences, Landing,
    Pave, SectionCrossing, SectionCurve, SectionEdge, SectionVertex, VertexSource, meet_curves,
};
use crate::error::{Fault, OpError, Reason};
use crate::pass::pass;

/// A section vertex while hits are still being merged into it.
struct VertexBuild {
    /// The merged points, the first hit's first.
    points: Vec<Point3>,
    /// The first merged point that lies on an operand edge — a hit's,
    /// a touch's or a crossing's — when one does.
    on_edge: Option<Point3>,
    /// The largest tolerance of the entities merged.
    base: f64,
    /// What the section edges ending here need: a vertex is never below
    /// its edges.
    floor: f64,
    hits: Vec<usize>,
    crossings: Vec<usize>,
    section_crossings: Vec<usize>,
    existing: Vec<VertexId>,
    source: VertexSource,
}

impl VertexBuild {
    /// The representative point: the first operand vertex's, else the
    /// first merged point on an operand edge, else the first merged
    /// point. An edge the vertex paves is cut there exactly, so a
    /// section edge ending on it is what moves to meet it, never the
    /// operand's edge.
    fn point(&self, m: &Model) -> Point3 {
        self.existing
            .first()
            .and_then(|&v| m.vertex(v).ok())
            .map_or(self.on_edge.unwrap_or(self.points[0]), |v| v.point())
    }

    /// The base tolerance plus the spread of the merged points about the
    /// representative one, never below the floor.
    fn tolerance(&self, m: &Model) -> f64 {
        let centre = self.point(m);
        let spread = self
            .points
            .iter()
            .map(|p| (p - centre).norm())
            .fold(0.0, f64::max);
        (self.base + spread).max(self.floor)
    }

    fn finish(&self, m: &Model) -> SectionVertex {
        SectionVertex {
            point: self.point(m),
            tolerance: self.tolerance(m),
            hits: self.hits.clone(),
            crossings: self.crossings.clone(),
            section_crossings: self.section_crossings.clone(),
            existing: self.existing.clone(),
            source: self.source,
        }
    }
}

/// What a candidate point of a section vertex is.
#[derive(Debug, Clone, Copy)]
enum Member {
    Hit(usize),
    Crossing(usize),
    SectionCrossing(usize),
    Singular,
}

impl Member {
    /// The source of a vertex this member is the first of.
    fn source(self) -> VertexSource {
        match self {
            Member::Hit(_) | Member::Crossing(_) => VertexSource::Hits,
            Member::SectionCrossing(_) => VertexSource::SectionCrossing,
            Member::Singular => VertexSource::Singular,
        }
    }

    /// Whether its point lies on an operand edge.
    fn on_edge(self) -> bool {
        matches!(self, Member::Hit(_) | Member::Crossing(_))
    }
}

/// A point that may be a section vertex, or an operand vertex one names:
/// its ball — the tolerance of the entities that made it — and the
/// operand vertices it coincides with, in the order it names them.
#[derive(Clone)]
struct Candidate {
    point: Point3,
    tolerance: f64,
    existing: Vec<VertexId>,
}

/// The connected components of `nodes` under "the same within a
/// tolerance": two whose balls meet, `|p − q| ≤ tp + tq` — a point of
/// both is within tolerance of each, so nothing the model holds tells
/// them apart — or that name one operand vertex. Per node, the least
/// index in its component: the partition does not depend on the nodes'
/// order, only the labels do.
fn components(nodes: &[Candidate]) -> Vec<usize> {
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut parent: Vec<usize> = (0..nodes.len()).collect();
    for (i, p) in nodes.iter().enumerate() {
        for (j, q) in nodes.iter().enumerate().skip(i + 1) {
            let same = (p.point - q.point).norm() <= p.tolerance + q.tolerance
                || p.existing.iter().any(|x| q.existing.contains(x));
            if !same {
                continue;
            }
            let (ri, rj) = (root(&mut parent, i), root(&mut parent, j));
            parent[ri.max(rj)] = ri.min(rj);
        }
    }
    (0..nodes.len()).map(|i| root(&mut parent, i)).collect()
}

/// An end of an edge piece: the edge's own vertex, or the section vertex
/// of a pave.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum End {
    Operand(VertexId),
    Section(usize),
}

/// A piece of an operand edge between consecutive paves, as the result
/// will cut it: the same enumeration `result::Build::sub_edges` makes.
struct Block {
    index: usize,
    range: Interval,
    start: End,
    end: End,
}

/// A piece of an operand edge that a block of a section curve of pair
/// `pair` lies along: the edge's block, placed on the pair's face of the
/// other operand as an image instead of a section edge on both.
struct Along {
    pair: usize,
    side: usize,
    edge: EdgeId,
    block: Block,
}

/// The whole build, over the two operands read once.
struct Build<'m, 'c> {
    m: &'m Model,
    /// The caller's meter for the whole build (ADR-0030): behind a lock
    /// because the methods that geometry calls hang off take `&self` and
    /// the parallel pass shares the build. The pass gives each item a
    /// split of it and never locks it from a thread.
    meter: Mutex<Meter<'c>>,
    precision: Precision,
    a: Body,
    b: Body,
    faces: [Vec<FaceInfo<'m>>; 2],
    edges: [Vec<EdgeInfo<'m>>; 2],
    /// The one region every face pair's traced section is clipped to
    /// ([`region`]).
    within: Aabb,
    pairs: Vec<FacePair>,
    /// Per pair, the indices of its faces in `faces`.
    pair_faces: Vec<(usize, usize)>,
    hits: Vec<EdgeFaceHit>,
    /// Per hit, the tolerance of the entities that made it.
    hit_tolerance: Vec<f64>,
    crossings: Vec<EdgeEdgeHit>,
    crossing_tolerance: Vec<f64>,
    section_crossings: Vec<SectionCrossing>,
    /// Per section crossing, the tolerance of the pair that made it.
    section_crossing_tolerance: Vec<f64>,
    /// Edge pairs of the coincident face pairs whose curves are the same
    /// curve, `a`'s edge first.
    same_curve: BTreeSet<(EdgeId, EdgeId)>,
    vertices: Vec<VertexBuild>,
    paves: BTreeMap<EdgeId, Vec<Pave>>,
    curves: Vec<SectionCurve>,
    sections: Vec<SectionEdge>,
    contacts: Vec<Contact>,
    coincident: Vec<(EdgeId, FaceId)>,
    /// The pieces of operand edges a block of a section curve lies along
    /// ([`Self::along_block`]), each with the pair whose section it is.
    along: Vec<Along>,
    images: Vec<EdgeImage>,
    blocks: Vec<CommonBlock>,
}

pub(super) fn build<'c>(
    m: &Model,
    a: Body,
    b: Body,
    meter: &mut Meter<'c>,
) -> Result<Interferences, OpError> {
    let faces = [FaceInfo::of_body(m, a)?, FaceInfo::of_body(m, b)?];
    let edges = [EdgeInfo::of_body(m, a)?, EdgeInfo::of_body(m, b)?];
    let within = region(&faces);
    let mut build = Build {
        m,
        meter: Mutex::new(*meter),
        precision: m.precision(),
        a,
        b,
        faces,
        edges,
        within,
        pairs: Vec::new(),
        pair_faces: Vec::new(),
        hits: Vec::new(),
        hit_tolerance: Vec::new(),
        crossings: Vec::new(),
        crossing_tolerance: Vec::new(),
        section_crossings: Vec::new(),
        section_crossing_tolerance: Vec::new(),
        same_curve: BTreeSet::new(),
        vertices: Vec::new(),
        paves: BTreeMap::new(),
        curves: Vec::new(),
        sections: Vec::new(),
        contacts: Vec::new(),
        coincident: Vec::new(),
        along: Vec::new(),
        images: Vec::new(),
        blocks: Vec::new(),
    };
    build.face_pairs()?;
    build.hits()?;
    build.crossings()?;
    build.section_crossings()?;
    build.merge()?;
    build.pave_edges();
    build.sections()?;
    build.pave_coincident_edges();
    build.pave_singular_edges()?;
    build.contacts()?;
    build.coincident()?;
    *meter = build.meter.lock().map_or_else(|e| *e.into_inner(), |g| *g);
    Ok(build.finish())
}

/// The region every face pair of the boolean is intersected in: the
/// overlap of the two operands' boxes, grown on every side by its own
/// diagonal. A section interior to both faces of a pair is inside both
/// operands' boxes, so the overlap bounds everything the boolean keeps;
/// the growth keeps the clip of a traced section well away from any
/// face, since it only has to bound what runs to infinity. One region
/// for the whole boolean, so every face pair on the same two surfaces
/// gets the same curve bit for bit (ADR-0018). Operands whose boxes are
/// apart have no candidate pair, and their region is never read.
fn region(faces: &[Vec<FaceInfo<'_>>; 2]) -> Aabb {
    let [a, b] = faces.each_ref().map(|side| {
        side.iter()
            .map(|f| f.bounds)
            .reduce(Aabb::union)
            .unwrap_or(Aabb::of_point(Point3::origin()))
    });
    let overlap = Aabb {
        min: [0, 1, 2].map(|i| a.min[i].max(b.min[i])),
        max: [0, 1, 2].map(|i| a.max[i].min(b.max[i])),
    };
    if (0..3).any(|i| overlap.min[i] > overlap.max[i]) {
        return a.union(b);
    }
    overlap.inflated(overlap.diagonal())
}

fn shape_of(body: Body) -> Shape {
    Shape::new(body.id, body.orientation)
}

/// The tolerance a geometric query between two entities runs at: the
/// larger of their tolerances for lengths, the model's angle.
fn tolerance_of(precision: &Precision, a: f64, b: f64) -> Tolerance {
    Tolerance::new(a.max(b), precision.angular_tolerance)
}

/// How near a face's singular point a section curve may pass without
/// running through it, in polygon segments: the face's box diagonal over
/// [`MAX_SEGMENTS_PER_PIECE`] is the length of one segment of the finest
/// polygon a pcurve across the face is given, and a pcurve passing a pole
/// at a distance `d` turns `u` by up to `π` over a stretch `d` long —
/// nearer than a few segments the polygon cannot follow it, and the
/// split finds no region beside it. Measured on a ball of radius 2 under
/// an oblique plane: built from a miss of `1e-4`, not at `1e-5`, where a
/// segment is `1e-4`; four segments is that with a margin for a section
/// that is not a circle. A ratio of the polygon's resolution, not a
/// tolerance (ADR-0021).
const SINGULAR_CLEARANCE: f64 = 4.0;

/// `true` when `curve`, within the band of a singular point of `surface`
/// at `t`, runs straight through it, which is what carrying its pcurve
/// onto the point assumes ([`pcurve_on`]). A sphere's pole is a smooth
/// point of the surface and every curve through it does. A cone's apex is
/// not: a curve on the cone through the apex leaves it along a ruling,
/// its tangent at the half-angle to the axis, while one that only comes
/// within the band — the hyperbola of a plane a hair off the apex — turns
/// back there between two rulings `r₁` and `r₂` with its tangent along
/// `r₁ − r₂`, which is perpendicular to the axis whatever the plane. Half
/// of the ruling's own `cos α` along the axis tells the two apart: a
/// ratio between the two cases, not a tolerance.
fn straight_through(surface: &Surface, curve: &Curve, t: f64) -> bool {
    let Surface::Cone {
        frame, half_angle, ..
    } = surface
    else {
        return true;
    };
    curve
        .eval(t)
        .d1
        .try_normalize(0.0)
        .is_some_and(|d| d.dot(&frame.z()).abs() > 0.5 * half_angle.cos())
}

/// The points [`at_shared_end`] samples along the stretch from a
/// crossing to an edge's end.
const END_STRETCH_SAMPLES: usize = 8;

/// A crossing of `a` and `b` moved to an end vertex of either when it is
/// the same meeting: the end lies within the tolerance of the other
/// curve, and so does the edge from the crossing to that end. Two edges
/// on one surface crossing at a shallow angle stay within the tolerance
/// of each other over a stretch longer than it, and the intersector's
/// point may lie anywhere along it; where one edge already ends on the
/// other there — the vertex an earlier operation made at that crossing
/// — the crossing is that vertex, not a second one a hair past its
/// tolerance beside it. `None` when the crossing is within an end's own
/// tolerance, which [`EdgeInfo::vertex_at`] reads, or joins no end.
fn at_shared_end(
    a: &EdgeInfo<'_>,
    ta: f64,
    b: &EdgeInfo<'_>,
    tb: f64,
    point: Point3,
    tol: Tolerance,
) -> Option<(f64, f64, Point3)> {
    for (side, (x, tx, y)) in [(a, ta, b), (b, tb, a)].into_iter().enumerate() {
        for (k, &(_, end, end_tolerance)) in x.ends.iter().enumerate() {
            if (point - end).norm() <= end_tolerance {
                return None;
            }
            let within = tol.linear.max(end_tolerance);
            let Ok(on_y) = y.curve.project(end) else {
                continue;
            };
            let Some(ty) = y.in_range(on_y.t) else {
                continue;
            };
            if on_y.distance > within {
                continue;
            }
            let te = if k == 0 { x.range.lo() } else { x.range.hi() };
            let stays = (1..END_STRETCH_SAMPLES).all(|i| {
                let s = i as f64 / END_STRETCH_SAMPLES as f64;
                y.curve
                    .project(x.curve.point(tx + s * (te - tx)))
                    .is_ok_and(|q| q.distance <= within)
            });
            if stays {
                return Some(if side == 0 {
                    (te, ty, end)
                } else {
                    (ty, te, end)
                });
            }
        }
    }
    None
}

/// A geometry error on validated input as the operation's: a missing
/// closed form names the two entities, anything else is a kernel fault.
fn geometry(e: GeomError, a: Shape, b: Shape) -> OpError {
    match e {
        GeomError::Unsupported { a: ka, b: kb } => OpError::Unsupported {
            a: (ka, a),
            b: (kb, b),
        },
        other => crate::error::fault_of(other),
    }
}

/// `t` on a periodic curve wrapped into `[lo, lo + period)` of its
/// domain — `[0, 2π)` for a conic, the knots' for a periodic NURBS
/// section loop — and unchanged on any other curve.
fn wrap_on(c: &Curve, t: f64) -> f64 {
    let Some(period) = c.period() else {
        return t;
    };
    let lo = c.domain().lo();
    let w = if t >= lo && t - lo < period {
        t
    } else {
        lo + (t - lo).rem_euclid(period)
    };
    if w - lo >= period { lo } else { w }
}

/// `n` parameters over `range`, both ends included.
fn samples(range: Interval, n: usize) -> Vec<f64> {
    let n = n.max(2);
    (0..n)
        .map(|i| range.lerp(i as f64 / (n - 1) as f64))
        .collect()
}

impl<'m, 'c> Build<'m, 'c> {
    /// `f` with the meter, for one geometry call; the lock is held for
    /// the call and no longer.
    fn metered<T>(&self, f: impl FnOnce(&mut Meter<'c>) -> T) -> T {
        f(&mut self.meter.lock().unwrap_or_else(PoisonError::into_inner))
    }

    /// One step of the build's own loops (ADR-0030 §5).
    fn tick(&self) -> Result<(), OpError> {
        self.metered(Meter::tick).map_err(OpError::Interrupted)
    }

    /// The edge of operand `side` with this id, when it has a curve.
    fn edge_info(&self, side: usize, id: EdgeId) -> Option<&EdgeInfo<'m>> {
        self.edges[side].iter().find(|e| e.id == id)
    }

    /// Every face pair whose boxes overlap, intersected.
    fn face_pairs(&mut self) -> Result<(), OpError> {
        let mut candidates: Vec<(usize, usize)> = Vec::new();
        for (ia, fa) in self.faces[0].iter().enumerate() {
            for (ib, fb) in self.faces[1].iter().enumerate() {
                if fa.bounds.intersects(&fb.bounds) {
                    candidates.push((ia, ib));
                }
            }
        }
        for ((ia, ib), intersection) in candidates
            .iter()
            .copied()
            .zip(self.intersect_pairs(&candidates)?)
        {
            self.pairs.push(FacePair {
                a: self.faces[0][ia].id,
                b: self.faces[1][ib].id,
                intersection,
            });
            self.pair_faces.push((ia, ib));
        }
        Ok(())
    }

    /// [`intersect_surfaces`] over each candidate pair, in the
    /// candidates' order in the result however it was computed: over
    /// `rayon` behind `parallel`, a plain iterator otherwise. A pair's
    /// intersection reads two surfaces and their tolerances and touches
    /// nothing else, which is what makes it the pave model's parallel
    /// step (ADR-0004, `docs/ARCHITECTURE.md` §Threading).
    fn intersect_pairs(
        &mut self,
        candidates: &[(usize, usize)],
    ) -> Result<Vec<SurfaceIntersection>, OpError> {
        let mut meter = *self.meter.get_mut().unwrap_or_else(PoisonError::into_inner);
        let found = pass(candidates, &mut meter, |&(ia, ib), mt| {
            let (fa, fb) = (&self.faces[0][ia], &self.faces[1][ib]);
            let tol = tolerance_of(&self.precision, fa.tolerance, fb.tolerance);
            mt.tick()?;
            intersect_surfaces(fa.surface, fb.surface, &self.within, tol, mt)
                .map_err(|e| geometry(e, fa.shape(), fb.shape()))
        });
        *self.meter.get_mut().unwrap_or_else(PoisonError::into_inner) = meter;
        found
    }

    /// Every edge of each operand against every face of the other whose
    /// box it reaches; the hits sorted by `(edge id, t)`, the coincident
    /// edge–face pairs recorded.
    fn hits(&mut self) -> Result<(), OpError> {
        let mut found: Vec<(EdgeFaceHit, f64)> = Vec::new();
        let mut coincident = Vec::new();
        for side in 0..2 {
            let other = 1 - side;
            for e in &self.edges[side] {
                for f in &self.faces[other] {
                    if !e.bounds.intersects(&f.bounds) {
                        continue;
                    }
                    self.hit_edge_face(e, f, &mut found, &mut coincident)?;
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
    fn hit_edge_face(
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
    fn land(
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
    fn resolve_touch(&self, touch: usize) -> Result<Vec<EdgeFaceHit>, OpError> {
        let (edge, face) = (self.hits[touch].edge, self.hits[touch].face);
        let side = usize::from(self.edge_info(0, edge).is_none());
        let (Some(e), Some(f)) = (
            self.edge_info(side, edge),
            self.faces[1 - side].iter().find(|f| f.id == face),
        ) else {
            return Ok(Vec::new());
        };
        let tol = tolerance_of(&self.precision, e.tolerance, f.tolerance);
        let mut found: Vec<EdgeFaceHit> = Vec::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            let (ia, ib) = self.pair_faces[pi];
            let (own, other) = if side == 0 { (ia, ib) } else { (ib, ia) };
            let g = &self.faces[side][own];
            if self.faces[1 - side][other].id != face || !g.edges().contains(&edge) {
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
    fn crossings(&mut self) -> Result<(), OpError> {
        let mut found: Vec<(EdgeEdgeHit, f64)> = Vec::new();
        let mut same_curve = BTreeSet::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            if pair.intersection != SurfaceIntersection::Coincident {
                continue;
            }
            let (ia, ib) = self.pair_faces[pi];
            let (fa, fb) = (&self.faces[0][ia], &self.faces[1][ib]);
            for &ea in fa.edges() {
                let Some(ea) = self.edge_info(0, ea) else {
                    continue;
                };
                for &eb in fb.edges() {
                    let Some(eb) = self.edge_info(1, eb) else {
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

    /// `point` on face `f`: its (u, v) in the translate the face's loops
    /// use when the point lies inside the face or on its boundary — on an
    /// edge or a vertex of it, or within a loop's band and wound around —
    /// and `None` when it lies outside, as [`Self::hit_edge_face`]
    /// decides a hit's landing.
    fn on_face(&self, f: &FaceInfo<'m>, point: Point3) -> Result<Option<Point2>, OpError> {
        let Ok(projection) = f.surface.project(point) else {
            return Ok(None);
        };
        let (side, shift) = f.domain.side(projection.uv);
        let on = match side {
            Side::Outside => false,
            Side::Inside => true,
            Side::Boundary => {
                f.domain.boundary_entity(self.m, point)?.is_some()
                    || f.domain.winds_around(projection.uv)
            }
        };
        Ok(on.then_some(projection.uv + shift))
    }

    /// The curves of every crossing pair against one another: a
    /// crossing on both faces is recorded, sorted by `(pair, curves, t on
    /// the first)`. Two curves of one pair meet where the surfaces are
    /// tangent to each other — the two ellipses of equal cylinders with
    /// crossing axes — and no edge of either operand is there to make a
    /// hit, so the crossing makes the section vertex both curves need.
    fn section_crossings(&mut self) -> Result<(), OpError> {
        let mut found: Vec<(SectionCrossing, f64)> = Vec::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            let (ia, ib) = self.pair_faces[pi];
            let (fa, fb) = (&self.faces[0][ia], &self.faces[1][ib]);
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
                                t: [wrap_on(ca, h.ta), wrap_on(cb, h.tb)],
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

    /// The crossings a traced pair's singular points make: each `Crossing`
    /// point of the pair's `Meets` on both faces, with the branches that
    /// end there — within `tolerance` of it, which they end at exactly —
    /// the first against each other, or against itself when one branch
    /// both starts and ends there. A `Touch` point, where the surfaces
    /// meet at that point alone, splits nothing and makes no vertex, as a
    /// touch off every vertex makes none.
    fn singular_crossings(
        &self,
        pi: usize,
        curves: &[(usize, &Curve)],
        tolerance: f64,
    ) -> Result<Vec<SectionCrossing>, OpError> {
        let (ia, ib) = self.pair_faces[pi];
        let (fa, fb) = (&self.faces[0][ia], &self.faces[1][ib]);
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
    fn merge(&mut self) -> Result<(), OpError> {
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
            for (side, id) in [(0, x.a), (1, x.b)] {
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
        // each other. It joins that vertex, which then paves its edge.
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
                if reached.contains(&label[made + k]) {
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
                    .map(|&h| Shape::new(self.hits[h].edge, self.a.orientation))
                    .or_else(|| {
                        v.crossings
                            .first()
                            .map(|&x| Shape::new(self.crossings[x].a, self.a.orientation))
                    })
                    .or_else(|| {
                        v.section_crossings.first().map(|&x| {
                            Shape::new(
                                self.pairs[self.section_crossings[x].pair].a,
                                self.a.orientation,
                            )
                        })
                    })
                    .unwrap_or_else(|| shape_of(self.a));
                return Err(OpError::Tolerance { entity, wanted });
            }
        }
        Ok(())
    }

    /// [`components`] over `candidates` and, beside them, every operand
    /// vertex one of them names, as a ball of its own point and
    /// tolerance: a point within reach of that vertex is the same point
    /// as the hit at it. The labels of the candidates alone.
    fn labels(&self, candidates: &[&Candidate]) -> Result<Vec<usize>, OpError> {
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
    fn one_per_operand(&self) -> Result<(), OpError> {
        let m = self.m;
        let own = [self.a, self.b].map(|body| {
            m.vertices(body)
                .map(|vs| vs.into_iter().map(|v| v.id).collect::<BTreeSet<_>>())
        });
        let own = [own[0].clone()?, own[1].clone()?];
        for v in &self.vertices {
            for side in &own {
                let mut of_side = v.existing.iter().filter(|x| side.contains(x));
                if let (Some(_), Some(&second)) = (of_side.next(), of_side.next()) {
                    return Err(OpError::Tolerance {
                        entity: Shape::new(second, self.a.orientation),
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
    /// [`Reason::BesideSingularity`].
    fn singular_vertices(&self) -> Result<Vec<(Point3, f64, VertexId)>, OpError> {
        let mut wanted: Vec<(Point3, f64, VertexId)> = Vec::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            let (ia, ib) = self.pair_faces[pi];
            let (fa, fb) = (&self.faces[0][ia], &self.faces[1][ib]);
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
                                Shape::new(s.vertex, self.a.orientation),
                            ],
                            reason: Reason::BesideSingularity,
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
    fn pave_singular_edges(&mut self) -> Result<(), OpError> {
        let mut wanted: Vec<(EdgeId, f64, usize, f64)> = Vec::new();
        for section in &self.sections {
            let pair = self.curves[section.curve].pair;
            let (ia, ib) = self.pair_faces[pair];
            for (side, f) in [(0, &self.faces[0][ia]), (1, &self.faces[1][ib])] {
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
    fn sort_hits(&mut self) {
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
    fn pave_edges(&mut self) {
        let mut wanted: Vec<(EdgeId, f64, usize)> = Vec::new();
        for h in &self.hits {
            let (Some(vertex), None) = (h.vertex, h.at_vertex) else {
                continue;
            };
            // Merged into the section vertex that holds the edge's own end,
            // as a crossing's is below: a hit a hair past that end's
            // tolerance would pave the end's vertex beside it.
            let side = usize::from(self.edge_info(0, h.edge).is_none());
            let at_end = self.edge_info(side, h.edge).is_some_and(|e| {
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
            for (side, id, t) in [(0, x.a, x.ta), (1, x.b, x.tb)] {
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

    /// Every edge of a face of a `Coincident` pair paved by every section
    /// vertex that lies on it within the vertex's tolerance and is not
    /// one of its ends: the vertices the other operand's edges made on
    /// neighbouring faces, which the pieces along the shared surface
    /// have to meet at (ADR-0004). A vertex nothing ends at — two branches
    /// of a traced section crossing on the faces' boundary and leaving
    /// both, no hit, no operand vertex and no section edge there — has
    /// nothing for the pieces to meet, and paves nothing.
    fn pave_coincident_edges(&mut self) {
        let m = self.m;
        let mut used = vec![false; self.vertices.len()];
        for (k, v) in self.vertices.iter().enumerate() {
            used[k] = !(v.hits.is_empty() && v.crossings.is_empty() && v.existing.is_empty());
        }
        for s in &self.sections {
            for k in [s.start, s.end] {
                if let Some(u) = used.get_mut(k) {
                    *u = true;
                }
            }
        }
        let mut edges: BTreeSet<(usize, EdgeId)> = BTreeSet::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            if pair.intersection != SurfaceIntersection::Coincident {
                continue;
            }
            let (ia, ib) = self.pair_faces[pi];
            edges.extend(self.faces[0][ia].edges().iter().map(|&e| (0, e)));
            edges.extend(self.faces[1][ib].edges().iter().map(|&e| (1, e)));
        }
        // An edge lying in a face of the other operand is paved the same
        // way, whether or not a face of its own is coincident with it: its
        // pieces inside that face are images there.
        for &(e, _) in &self.coincident {
            edges.insert((usize::from(self.edge_info(0, e).is_none()), e));
        }
        let mut wanted: Vec<(EdgeId, f64, usize)> = Vec::new();
        for (side, id) in edges {
            let Some(e) = self.edge_info(side, id) else {
                continue;
            };
            for (k, v) in self.vertices.iter().enumerate() {
                if !used[k] {
                    continue;
                }
                let point = v.point(m);
                if e.vertex_at(point).is_some() || e.ends.iter().any(|x| v.existing.contains(&x.0))
                {
                    continue;
                }
                let Ok(projection) = e.curve.project(point) else {
                    continue;
                };
                if projection.distance > v.tolerance(m) {
                    continue;
                }
                let Some(t) = e.in_range(projection.t) else {
                    continue;
                };
                if t == e.range.lo() || t == e.range.hi() {
                    continue;
                }
                wanted.push((id, t, k));
            }
        }
        for (edge, t, vertex) in wanted {
            let list = self.paves.entry(edge).or_default();
            if list.iter().all(|p| p.vertex != vertex) {
                list.push(Pave { t, vertex });
                list.sort_by(|x, y| x.t.total_cmp(&y.t));
            }
        }
    }

    /// `point` projected onto both faces' surfaces and inside both, with
    /// the (u, v) of each in the translate the face's loops use.
    fn inside_both(fa: &FaceInfo<'m>, fb: &FaceInfo<'m>, point: Point3) -> Option<[Point2; 2]> {
        let mut out = [Point2::origin(); 2];
        for (i, f) in [fa, fb].into_iter().enumerate() {
            let projection = f.surface.project(point).ok()?;
            let (side, shift) = f.domain.side(projection.uv);
            if side != Side::Inside {
                return None;
            }
            out[i] = projection.uv + shift;
        }
        Some(out)
    }

    /// The section curves of every pair, paved and cut into blocks, the
    /// blocks interior to both faces kept as section edges. A pair's
    /// crossing curves are its sections whatever else its `Meets` holds:
    /// a touching curve beside them — a pipe's bend tangent to the
    /// straight run along the tube circle and crossing it in a quartic —
    /// is [`Self::contacts`]'.
    fn sections(&mut self) -> Result<(), OpError> {
        for pi in 0..self.pairs.len() {
            let curves: Vec<(usize, Curve)> =
                meet_curves(&self.pairs[pi].intersection, MeetKind::Crossing)
                    .map(|(ci, c)| (ci, c.clone()))
                    .collect();
            for (ci, curve) in &curves {
                self.section_curve(pi, *ci, curve)?;
            }
        }
        Ok(())
    }

    /// The touching curves of every pair, paved by the touches and cut
    /// into blocks, the blocks interior to both faces kept as contacts.
    fn contacts(&mut self) -> Result<(), OpError> {
        for pi in 0..self.pairs.len() {
            let curves: Vec<(usize, Curve)> =
                meet_curves(&self.pairs[pi].intersection, MeetKind::Touch)
                    .map(|(ci, c)| (ci, c.clone()))
                    .collect();
            for (ci, curve) in &curves {
                self.contact_curve(pi, *ci, curve)?;
            }
        }
        Ok(())
    }

    /// One tangent curve: its paves are the hits of either face's edges
    /// on the other face that lie on it — where the curve leaves one
    /// face inside the other, a touch, since every curve in a face
    /// tangent to the other surface is tangent to it there — and each
    /// block between consecutive paves whose midpoint is inside both
    /// faces is a contact. A block that is interior to both faces has a
    /// pave at each end: the ends of the overlap of the two faces' spans
    /// along it are each an end of one span inside the other. On a
    /// closed curve — a ball in a bore of its radius, along the circle
    /// they share — the last block wraps round to the first pave, as a
    /// section loop's does, and a curve no edge reaches is one block, a
    /// contact when it is interior to both faces.
    fn contact_curve(&mut self, pi: usize, ci: usize, curve: &Curve) -> Result<(), OpError> {
        let (ia, ib) = self.pair_faces[pi];
        let (fa, fb) = (&self.faces[0][ia], &self.faces[1][ib]);
        let mut paves: Vec<f64> = Vec::new();
        for (k, h) in self.hits.iter().enumerate() {
            let on_pair = (h.face == fb.id && fa.edges().contains(&h.edge))
                || (h.face == fa.id && fb.edges().contains(&h.edge));
            if !on_pair {
                continue;
            }
            let Ok(projection) = curve.project(h.point) else {
                continue;
            };
            if projection.distance <= self.hit_tolerance[k] {
                paves.push(wrap_on(curve, projection.t));
            }
        }
        paves.sort_by(f64::total_cmp);
        let mut blocks: Vec<(f64, f64)> = paves.windows(2).map(|w| (w[0], w[1])).collect();
        if let Some(period) = curve.period() {
            match (paves.first(), paves.last()) {
                (Some(&first), Some(&last)) => {
                    blocks.push((last, (first + period).min(period_end(last, period))));
                }
                _ => {
                    let lo = curve.domain().lo();
                    blocks.push((lo, period_end(lo, period)));
                }
            }
        }
        for (lo, hi) in blocks {
            let Ok(range) = Interval::new(lo, hi) else {
                continue;
            };
            if range.length() <= 0.0 {
                continue;
            }
            let point = curve.point(range.midpoint());
            let Some(uv) = Self::inside_both(fa, fb, point) else {
                continue;
            };
            self.contacts.push(Contact {
                pair: pi,
                curve: ci,
                range,
                point,
                uv,
            });
        }
        Ok(())
    }

    /// One section curve: its paves from the vertices on it, a seed
    /// vertex for a closed curve with none, then its blocks.
    fn section_curve(&mut self, pi: usize, ci: usize, curve: &Curve) -> Result<(), OpError> {
        let m = self.m;
        let (ia, ib) = self.pair_faces[pi];
        let (fa, fb) = (&self.faces[0][ia], &self.faces[1][ib]);
        let periodic = curve.period().is_some();
        // An open curve that ends on a section vertex is paved at that end
        // — at both, for a traced branch that leaves a singular point and
        // comes back to it, which a projection finds once (ADR-0018).
        let mut ends: Vec<Pave> = Vec::new();
        if !periodic {
            let domain = curve.domain();
            for t in [domain.lo(), domain.hi()] {
                if !t.is_finite() {
                    continue;
                }
                let at = curve.point(t);
                for (k, v) in self.vertices.iter().enumerate() {
                    if (v.point(m) - at).norm() <= v.tolerance(m) {
                        ends.push(Pave { t, vertex: k });
                    }
                }
            }
        }
        let mut paves: Vec<Pave> = Vec::new();
        for (k, v) in self.vertices.iter().enumerate() {
            let point = v.point(m);
            let Ok(projection) = curve.project(point) else {
                continue;
            };
            let at_an_end = ends.iter().any(|e| {
                e.vertex == k && (curve.point(e.t) - projection.point).norm() <= v.tolerance(m)
            });
            if at_an_end {
                continue;
            }
            if projection.distance <= v.tolerance(m) {
                // Where a section crossing of this curve and another is
                // one of the vertex's members, the curve is paved at that
                // crossing's own parameter, as an edge is at its hit's:
                // the curves then end on the same point whatever the
                // vertex's representative one is.
                let t = v
                    .section_crossings
                    .iter()
                    .map(|&x| &self.section_crossings[x])
                    .filter(|x| x.pair == pi && x.curves[0] != x.curves[1])
                    .find_map(|x| x.curves.iter().position(|&c| c == ci).map(|i| x.t[i]))
                    .unwrap_or(projection.t);
                paves.push(Pave {
                    t: wrap_on(curve, t),
                    vertex: k,
                });
            }
        }
        paves.extend(ends);
        paves.sort_by(|x, y| x.t.total_cmp(&y.t).then(x.vertex.cmp(&y.vertex)));
        let curve_index = self.curves.len();
        if paves.is_empty() && periodic {
            // Nothing cuts it: it is interior to both faces or clear of
            // one, and only in the first case does it become an edge, from
            // its own point at the start of its domain.
            let domain = curve.domain();
            if Self::inside_both(fa, fb, curve.point(domain.midpoint())).is_some() {
                self.vertices.push(VertexBuild {
                    points: vec![curve.point(domain.lo())],
                    on_edge: None,
                    base: fa.tolerance.max(fb.tolerance),
                    floor: 0.0,
                    hits: Vec::new(),
                    crossings: Vec::new(),
                    section_crossings: Vec::new(),
                    existing: Vec::new(),
                    source: VertexSource::CurveStart {
                        pair: pi,
                        curve: ci,
                    },
                });
                paves.push(Pave {
                    t: domain.lo(),
                    vertex: self.vertices.len() - 1,
                });
            }
        }
        // The blocks between consecutive paves; on a periodic curve the
        // last wraps round to the first.
        let mut blocks: Vec<(f64, f64, usize, usize)> = Vec::new();
        for w in paves.windows(2) {
            blocks.push((w[0].t, w[1].t, w[0].vertex, w[1].vertex));
        }
        if periodic {
            if let (Some(first), Some(last)) = (paves.first(), paves.last()) {
                let period = curve.period().ok_or(OpError::Internal(Fault::Invariant {
                    what: "a periodic curve's period",
                }))?;
                // A block that wraps ends one period after the first
                // pave, and never more than one period after the last —
                // with a single pave the two are the same turn, and the
                // sum can round one unit in the last place past it
                // (the checker's E1).
                let hi = (first.t + period).min(period_end(last.t, period));
                blocks.push((last.t, hi, last.vertex, first.vertex));
            }
        }
        // The operand edges of the two faces that lie on this curve: a
        // block that is a piece of one of them is that edge, not a section
        // edge, whatever the polygons' band says of its midpoint — its
        // split of the other face, where one is needed, is the edge's
        // image through the coincident neighbour (ADR-0004).
        let mut along: Vec<&EdgeInfo<'m>> = Vec::new();
        for (side, f, other) in [(0, fa, fb), (1, fb, fa)] {
            for &eid in f.edges() {
                let Some(e) = self.edge_info(side, eid) else {
                    continue;
                };
                let tol =
                    tolerance_of(&self.precision, e.tolerance, fa.tolerance.max(fb.tolerance));
                // The surfaces first: an edge whose other face lies on
                // the other face's surface is on this pair's section, and
                // along this branch when it lies on it at all.
                if let Some(on) = self.known_by_surfaces(side, e, f, other, curve, tol) {
                    if on {
                        along.push(e);
                    }
                    continue;
                }
                // Only the verdict: an edge in the plane of a planar
                // section conic — a rim circle beside the ellipse its cap
                // plane cuts from the other wall — has no closed form for
                // where the two meet, and needs none here.
                if self
                    .metered(|mt| curves_coincide(curve, e.curve, tol, mt))
                    .map_err(|err| geometry(err, fa.shape(), e.shape()))?
                {
                    along.push(e);
                }
            }
        }
        let is_an_edge: Vec<bool> = blocks
            .iter()
            .map(|&(lo, hi, _, _)| {
                let mid = curve.point(0.5 * (lo + hi));
                along.iter().any(|e| {
                    e.curve
                        .project(mid)
                        .is_ok_and(|on| Self::strictly_inside(e, e.range, on.t))
                })
            })
            .collect();
        drop(along);
        let mut edges = Vec::new();
        for (k, (lo, hi, start, end)) in blocks.into_iter().enumerate() {
            if is_an_edge[k] {
                continue;
            }
            let Ok(range) = Interval::new(lo, hi) else {
                continue;
            };
            if range.length() <= 0.0 {
                continue;
            }
            // Along an edge of one face, the block is on that face's
            // boundary and its polygons cannot say it is inside: the
            // verdict comes first, and the other face decides the image.
            let (fa, fb) = (&self.faces[0][ia], &self.faces[1][ib]);
            let along = [(0, fa), (1, fb)].into_iter().find_map(|(side, f)| {
                self.along_block(side, f, fa, fb, curve, range, start, end)
                    .map(|(edge, block)| (side, edge, block))
            });
            if let Some((side, edge, block)) = along {
                // Along an edge of the other face as well, wherever that
                // edge is paved, it is both faces' boundary: each edge's
                // own piece there, neither a section edge nor an image of
                // one on the other face.
                let other = if side == 0 { (1, fb) } else { (0, fa) };
                if !self.along_boundary(other.0, other.1, fa, fb, curve, range) {
                    self.along.push(Along {
                        pair: pi,
                        side,
                        edge,
                        block,
                    });
                }
                continue;
            }
            let Some(uv) = Self::inside_both(fa, fb, curve.point(range.midpoint())) else {
                continue;
            };
            let section = self.section_edge(fa, fb, curve, range, uv, curve_index, start, end)?;
            // Every vertex ≥ its edges: the ends carry at least the
            // section edge's tolerance.
            for k in [start, end] {
                let v = &mut self.vertices[k];
                v.floor = v.floor.max(section.tolerance);
            }
            edges.push(self.sections.len());
            self.sections.push(section);
        }
        self.curves.push(SectionCurve {
            pair: pi,
            index: ci,
            curve: curve.clone(),
            paves,
            edges,
        });
        Ok(())
    }

    /// Whether a face of operand `side` other than `f` that uses `e` lies
    /// on a surface `Coincident` with `other`'s, a face of the other
    /// operand, by the face pair's own verdict.
    fn beside_on(&self, side: usize, e: EdgeId, f: FaceId, other: FaceId) -> bool {
        self.faces[side]
            .iter()
            .filter(|g| g.id != f && g.edges().contains(&e))
            .any(|g| {
                let (a, b) = if side == 0 {
                    (g.id, other)
                } else {
                    (other, g.id)
                };
                self.pairs.iter().any(|p| {
                    p.a == a && p.b == b && p.intersection == SurfaceIntersection::Coincident
                })
            })
    }

    /// Whether `e`, an edge of face `f` of operand `side`, runs along
    /// `curve`, a section of `f`'s surface and `other`'s, known by the
    /// surfaces rather than by comparing the curves: where a face beside
    /// `f` in `e`'s own operand lies on a surface `Coincident` with
    /// `other`'s, the edge lies on both of the pair's surfaces to its own
    /// tolerance (E4) and so on their section, and along `curve` exactly
    /// when its midpoint lies on it within `tol`. Two fits of one traced
    /// section over different regions are two splines no closed form
    /// compares, and each is held to the exact branch (ADR-0022), so the
    /// midpoint decides. `None` when no such face is beside `f`, and the
    /// curves have to be asked.
    fn known_by_surfaces(
        &self,
        side: usize,
        e: &EdgeInfo<'m>,
        f: &FaceInfo<'m>,
        other: &FaceInfo<'m>,
        curve: &Curve,
        tol: Tolerance,
    ) -> Option<bool> {
        if !self.beside_on(side, e.id, f.id, other.id) {
            return None;
        }
        let mid = e.curve.point(e.range.midpoint());
        Some(curve.project(mid).is_ok_and(|on| on.distance <= tol.linear))
    }

    /// Whether `ea` of `fa` and `eb` of `fb`, two faces on one surface,
    /// are one section by their surfaces: a face beside `fa` using `ea`
    /// and one beside `fb` using `eb` lie on one surface too, so both
    /// edges lie on the section of the two surfaces, and they are the
    /// same curve when `ea`'s midpoint lies on `eb` within `tol`.
    fn same_section(
        &self,
        ea: &EdgeInfo<'m>,
        fa: &FaceInfo<'m>,
        eb: &EdgeInfo<'m>,
        fb: &FaceInfo<'m>,
        tol: Tolerance,
    ) -> bool {
        let beside = self.faces[1]
            .iter()
            .filter(|g| g.id != fb.id && g.edges().contains(&eb.id))
            .any(|g| self.beside_on(0, ea.id, fa.id, g.id));
        beside
            && eb
                .curve
                .project(ea.curve.point(ea.range.midpoint()))
                .is_ok_and(|on| on.distance <= tol.linear)
    }

    /// The piece of an operand edge of `f`, the pair's face of operand
    /// `side`, that the block `range` of `curve` of the pair `fa`, `fb`,
    /// from section vertex `start` to `end`, lies along: a piece between
    /// the same two vertices, in either order,
    /// that every point the model checks the block at lies within the
    /// tolerance of, inside the piece's own range. Two curves of one
    /// surface crossing at a shallow angle twice stay within the
    /// tolerance of each other over the whole stretch between — a small
    /// circle through a sphere's pole 2e-4 of a radian off the seam, back
    /// across it 1.8e-4 on — and the block between the two crossings is
    /// then no curve of its own but the edge's piece: built as a section
    /// edge it would bound a sliver of zero area on the edge's face. The
    /// verdict is the block's alone, beside the whole-curve one
    /// (`curves_coincide`) the curve's blocks are first held to; `None`
    /// when no piece is along it.
    #[allow(clippy::too_many_arguments)]
    fn along_block(
        &self,
        side: usize,
        f: &FaceInfo<'m>,
        fa: &FaceInfo<'m>,
        fb: &FaceInfo<'m>,
        curve: &Curve,
        range: Interval,
        start: usize,
        end: usize,
    ) -> Option<(EdgeId, Block)> {
        let (start, end) = (End::Section(start), End::Section(end));
        let points: Vec<Point3> = samples(range, self.precision.check_samples)
            .into_iter()
            .map(|t| curve.point(t))
            .collect();
        for &eid in f.edges() {
            let Some(e) = self.edge_info(side, eid) else {
                continue;
            };
            let within = e.tolerance.max(fa.tolerance.max(fb.tolerance));
            for block in self.blocks_of(e) {
                let ends = (self.canonical(block.start), self.canonical(block.end));
                if ends != (start, end) && ends != (end, start) {
                    continue;
                }
                let lies_along = points.iter().all(|&p| {
                    e.curve.project(p).is_ok_and(|on| {
                        on.distance <= within
                            && e.in_range(on.t)
                                .is_some_and(|t| Self::within_block(e, block.range, t))
                    })
                });
                if lies_along {
                    return Some((eid, block));
                }
            }
        }
        None
    }

    /// Whether every point the model checks the block `range` of `curve`
    /// at lies within the tolerance of an edge of `f`, the face of operand
    /// `side` of the pair `fa`, `fb`, inside that edge's range: the block
    /// runs along `f`'s boundary, whatever vertices pave the edges there.
    /// Two edges crossing at a grazing angle are hit at points scattered
    /// along them by the rounding over the angle, further apart than a
    /// tolerance, so the pieces of the two edges need not end on the same
    /// vertices.
    fn along_boundary(
        &self,
        side: usize,
        f: &FaceInfo<'m>,
        fa: &FaceInfo<'m>,
        fb: &FaceInfo<'m>,
        curve: &Curve,
        range: Interval,
    ) -> bool {
        let edges: Vec<&EdgeInfo<'m>> = f
            .edges()
            .iter()
            .filter_map(|&eid| self.edge_info(side, eid))
            .collect();
        samples(range, self.precision.check_samples)
            .into_iter()
            .all(|t| {
                let p = curve.point(t);
                edges.iter().any(|e| {
                    let within = e.tolerance.max(fa.tolerance.max(fb.tolerance));
                    e.curve
                        .project(p)
                        .is_ok_and(|on| on.distance <= within && e.in_range(on.t).is_some())
                })
            })
    }

    /// `t`, placed in `e`'s range, inside `range` or beyond either end by
    /// no more than the edge's tolerance converted to the parameter there.
    fn within_block(e: &EdgeInfo<'m>, range: Interval, t: f64) -> bool {
        let speed = e.curve.eval(t).d1.norm();
        let slack = if speed > 0.0 {
            e.tolerance / speed
        } else {
            0.0
        };
        t >= range.lo() - slack && t <= range.hi() + slack
    }

    /// A kept block as a section edge: the pcurve on each face, placed
    /// in the face's translate of the domain and ending on its vertices'
    /// own (u, v) there ([`Self::ended`]), and the tolerance raised to
    /// the pcurves' residual.
    #[allow(clippy::too_many_arguments)]
    fn section_edge(
        &self,
        fa: &FaceInfo<'m>,
        fb: &FaceInfo<'m>,
        curve: &Curve,
        range: Interval,
        uv_mid: [Point2; 2],
        curve_index: usize,
        start: usize,
        end: usize,
    ) -> Result<SectionEdge, OpError> {
        let base = fa.tolerance.max(fb.tolerance);
        let m = self.m;
        let ends = [start, end].map(|k| {
            let v = &self.vertices[k];
            (v.point(m), v.tolerance(m))
        });
        let (pa, ra) = self.pcurve_of(fa, fb, curve, range, uv_mid[0], base, &ends)?;
        let (pb, rb) = self.pcurve_of(fb, fa, curve, range, uv_mid[1], base, &ends)?;
        let (pa, ra) = self.ended(0, fa, fb, curve, range, base, [start, end], (pa, ra))?;
        let (pb, rb) = self.ended(1, fb, fa, curve, range, base, [start, end], (pb, rb))?;
        let tolerance = base.max(ra).max(rb);
        if tolerance > self.precision.max_tolerance {
            return Err(OpError::Tolerance {
                entity: fa.shape(),
                wanted: tolerance,
            });
        }
        Ok(SectionEdge {
            curve: curve_index,
            range,
            start,
            end,
            tolerance,
            pcurves: [pa, pb],
        })
    }

    /// The pcurve of a curve block on one face, placed, with the largest
    /// deviation of its image from the curve at the model's check
    /// parameters. `base` is the tolerance the fit is asked for; `ends`
    /// the balls of the vertices the block ends on, as [`Self::place`]
    /// reads them.
    #[allow(clippy::too_many_arguments)]
    fn pcurve_of(
        &self,
        f: &FaceInfo<'m>,
        other: &FaceInfo<'m>,
        curve: &Curve,
        range: Interval,
        uv_mid: Point2,
        base: f64,
        ends: &[(Point3, f64)],
    ) -> Result<(Curve2, f64), OpError> {
        let tol = Tolerance::new(base, self.precision.angular_tolerance);
        let pc = self
            .metered(|mt| pcurve_on(curve, range, f.surface, tol, mt))
            .map_err(|e| geometry(e, other.shape(), f.shape()))?;
        let pc = self.place(f, other, pc, range, uv_mid, base, ends)?;
        let residual = self.residual(f, curve, range, &pc);
        Ok((pc, residual))
    }

    /// The largest deviation of `pc`'s image on `f` from `curve` at the
    /// model's check parameters over `range`.
    fn residual(&self, f: &FaceInfo<'m>, curve: &Curve, range: Interval, pc: &Curve2) -> f64 {
        samples(range, self.precision.check_samples)
            .into_iter()
            .map(|t| {
                let q = pc.point(t);
                (f.surface.point(q.x, q.y) - curve.point(t)).norm()
            })
            .fold(0.0, f64::max)
    }

    /// A section edge's placed pcurve on face `f` of operand `side`, with
    /// its residual, ended on the (u, v) of the section vertices it ends
    /// on ([`Self::vertex_uv`]) wherever it lies further from it than
    /// half the band L2 holds a junction to — so that any two ends there
    /// meet within the band — and its residual measured again with the
    /// move. A section vertex merged from points further apart than a
    /// face's tolerance ([`components`]) has the curve's ends and the
    /// edge it paves apart by that much, and exact curves cannot meet on
    /// it: the section edge's pcurve moves, and its tolerance with it
    /// (`docs/DATA-MODEL.md` §Tolerances), never the operand's.
    #[allow(clippy::too_many_arguments)]
    fn ended(
        &self,
        side: usize,
        f: &FaceInfo<'m>,
        other: &FaceInfo<'m>,
        curve: &Curve,
        range: Interval,
        base: f64,
        vertices: [usize; 2],
        (pc, residual): (Curve2, f64),
    ) -> Result<(Curve2, f64), OpError> {
        let [start, end] = [(range.lo(), vertices[0]), (range.hi(), vertices[1])].map(|(t, k)| {
            let at = pc.point(t);
            let target = self
                .vertex_uv(side, f, k, at)
                .or_else(|| self.singular_arrival(f, k, at))?;
            let bound = bands(f.surface, target, self.precision.parametric_tolerance);
            let d = target - at;
            (d.x.abs() > 0.5 * bound[0] || d.y.abs() > 0.5 * bound[1]).then_some(target)
        });
        if start.is_none() && end.is_none() {
            return Ok((pc, residual));
        }
        let pc = self
            .metered(|mt| pcurve_ending_on(&pc, range, [start, end], f.surface, base, mt))
            .map_err(|e| geometry(e, other.shape(), f.shape()))?;
        // The move is the residual's largest term, at an end, and the
        // tolerance it sets is exactly that: rounding at the positions'
        // own scale above it keeps the edge within its tube when the body
        // is moved, which rounds every point it compares.
        let scale = [range.lo(), range.hi()]
            .map(|t| curve.point(t).coords.norm())
            .into_iter()
            .fold(0.0, f64::max);
        let residual = self.residual(f, curve, range, &pc) + RELATIVE_ROUNDING * scale;
        Ok((pc, residual))
    }

    /// Where a pcurve arriving at `at` on section vertex `k`, a singular
    /// vertex of `f`, ends: `at` with each periodic parameter held to the
    /// face's (u, v) box. The vertex is a whole line of (u, v), any `u` on
    /// it one point, and the `u` a curve arrives with is its tangent there;
    /// one that arrives past the box's edge crossed the seam inside the
    /// vertex's ball — a small circle through a pole a hair off the seam's
    /// meridian, crossing it again nearer the pole than the vertex's
    /// tolerance — and ends on the seam's own corner of the box, where the
    /// degenerate edge meets it. `None` for any other vertex, and for an
    /// arrival inside the box.
    fn singular_arrival(&self, f: &FaceInfo<'m>, k: usize, at: Point2) -> Option<Point2> {
        let v = &self.vertices[k];
        if !f.singular.iter().any(|s| v.existing.contains(&s.vertex)) {
            return None;
        }
        let mut held = at;
        for (d, period) in f.surface.period().iter().enumerate() {
            if period.is_some() {
                held[d] = at[d].clamp(f.uv_lo[d], f.uv_hi[d]);
            }
        }
        (held != at).then_some(held)
    }

    /// Where section vertex `k` is on face `f` of operand `side`, in the
    /// translate nearest `near`: an operand edge of the face paved there
    /// at its pave, or ending on an operand vertex it holds at that end —
    /// the point every piece of that edge will meet the vertex at — and
    /// otherwise the (u, v) of the vertex's point. Where the face holds
    /// several, the nearest; ties to the first in loop order. `None` at a
    /// singular vertex of the face, a whole line of (u, v) whose pcurves
    /// meet along the degenerate edge (ADR-0021), and where the point
    /// does not project.
    fn vertex_uv(&self, side: usize, f: &FaceInfo<'m>, k: usize, near: Point2) -> Option<Point2> {
        let v = &self.vertices[k];
        if f.singular.iter().any(|s| v.existing.contains(&s.vertex)) {
            return None;
        }
        let mut candidates: Vec<Point2> = Vec::new();
        for &(eid, pcurve) in &f.uses {
            if let Some(paves) = self.paves.get(&eid) {
                candidates.extend(
                    paves
                        .iter()
                        .filter(|p| p.vertex == k)
                        .map(|p| pcurve.point(p.t)),
                );
            }
            if let Some(e) = self.edge_info(side, eid) {
                for (end, t) in [(e.ends[0].0, e.range.lo()), (e.ends[1].0, e.range.hi())] {
                    if v.existing.contains(&end) {
                        candidates.push(pcurve.point(t));
                    }
                }
            }
        }
        if candidates.is_empty() {
            candidates.push(f.surface.project(v.point(self.m)).ok()?.uv);
        }
        let periods = f.surface.period();
        candidates
            .into_iter()
            .map(|mut c| {
                for (d, period) in periods.iter().enumerate() {
                    if let Some(p) = period {
                        c[d] += ((near[d] - c[d]) / p).round() * p;
                    }
                }
                c
            })
            .min_by(|x, y| (x - near).norm().total_cmp(&(y - near).norm()))
    }

    /// The pcurve translated by whole periods so its point at the block's
    /// midpoint is the face's own (u, v) there, and held to the face's
    /// (u, v) box in every periodic direction within `tolerance` converted
    /// at each point checked, in that direction: a block that still
    /// leaves it crosses a seam inside itself, which the seam's own hit
    /// should have paved (ADR-0004). Converted where the pcurve is and not
    /// once at the midpoint, because on a sphere or a cone a tolerance is
    /// no one step in `u`: a loop round a pole ends on the seam at a
    /// latitude where the fit's rounding in `u` is many times what the
    /// same length allows at the loop's far side. Inside the ball of a
    /// vertex it ends on (`ends`, point and tolerance) the block is that
    /// vertex, and held to its tolerance: a section vertex merged from
    /// points a tolerance apart ([`components`]) has the seam crossing inside
    /// its ball, and the stretch of the block up to it is no crossing the
    /// block makes.
    #[allow(clippy::too_many_arguments)]
    fn place(
        &self,
        f: &FaceInfo<'m>,
        other: &FaceInfo<'m>,
        pc: Curve2,
        range: Interval,
        uv_mid: Point2,
        tolerance: f64,
        ends: &[(Point3, f64)],
    ) -> Result<Curve2, OpError> {
        let periods = f.surface.period();
        let at = pc.point(range.midpoint());
        let mut by = Vec2::zeros();
        for (d, period) in periods.iter().enumerate() {
            if let Some(p) = period {
                by[d] = ((uv_mid[d] - at[d]) / p).round() * p;
            }
        }
        let pc = pc.translated(by);
        for (d, period) in periods.iter().enumerate() {
            if period.is_none() {
                continue;
            }
            for t in samples(range, self.precision.check_samples) {
                let at = pc.point(t);
                let outside = |reach: f64| {
                    let near = bands(f.surface, at, reach)[d];
                    at[d] < f.uv_lo[d] - near || at[d] > f.uv_hi[d] + near
                };
                if !outside(tolerance) {
                    continue;
                }
                let point = f.surface.point(at.x, at.y);
                // Within the ball of a singular point of the face, a step
                // of `u` is no length at all ([`Self::singular_arrival`]).
                let singular = |p: Point3, reach: f64| {
                    f.singular.iter().any(|s| (s.point - p).norm() <= reach)
                };
                let within_an_end = ends.iter().any(|&(p, reach)| {
                    (point - p).norm() <= reach && (singular(p, reach) || !outside(reach))
                });
                if !within_an_end {
                    return Err(OpError::Internal(Fault::Seam {
                        face: f.id,
                        other: other.id,
                    }));
                }
            }
        }
        Ok(pc)
    }

    // -- the coincident pairs ------------------------------------------

    /// The pieces of an edge between consecutive paves, as the result
    /// cuts it.
    fn blocks_of(&self, e: &EdgeInfo<'m>) -> Vec<Block> {
        let mut stops: Vec<(f64, End)> = vec![(e.range.lo(), End::Operand(e.ends[0].0))];
        if let Some(paves) = self.paves.get(&e.id) {
            stops.extend(paves.iter().map(|p| (p.t, End::Section(p.vertex))));
        }
        stops.push((e.range.hi(), End::Operand(e.ends[1].0)));
        stops
            .windows(2)
            .enumerate()
            .filter_map(|(index, w)| {
                let range = Interval::new(w[0].0, w[1].0).ok()?;
                (range.length() > 0.0).then_some(Block {
                    index,
                    range,
                    start: w[0].1,
                    end: w[1].1,
                })
            })
            .collect()
    }

    /// An end as the section vertex it is merged into, when it is.
    fn canonical(&self, end: End) -> End {
        match end {
            End::Section(_) => end,
            End::Operand(v) => self
                .vertices
                .iter()
                .position(|x| x.existing.contains(&v))
                .map_or(end, End::Section),
        }
    }

    /// `t`, placed in `e`'s range, strictly inside `range` by more than
    /// the edge's tolerance converted to the parameter there.
    fn strictly_inside(e: &EdgeInfo<'m>, range: Interval, t: f64) -> bool {
        let Some(t) = e.in_range(t) else {
            return false;
        };
        let speed = e.curve.eval(t).d1.norm();
        let slack = if speed > 0.0 {
            e.tolerance / speed
        } else {
            0.0
        };
        t > range.lo() + slack && t < range.hi() - slack
    }

    /// The piece of an edge of `other` that `block` of `e` is a piece
    /// of: an edge on the same curve (the curve–curve `Coincident`
    /// verdict, never the polygon band) whose piece overlaps it — either
    /// midpoint strictly inside the other's range — and then the same
    /// piece: both midpoints inside, the same ends. `None` when no piece
    /// of `other` overlaps; the fault when one overlaps without being the
    /// same piece, since every vertex the two edges share should have
    /// paved both.
    fn matching_block(
        &self,
        side: usize,
        e: &EdgeInfo<'m>,
        block: &Block,
        other: &FaceInfo<'m>,
    ) -> Result<Option<(EdgeId, Block, bool)>, OpError> {
        let fault = || {
            OpError::Internal(Fault::CommonBlock {
                edge: e.id,
                face: other.id,
            })
        };
        let mid = e.curve.point(block.range.midpoint());
        for &gid in other.edges() {
            let same = if side == 0 {
                self.same_curve.contains(&(e.id, gid))
            } else {
                self.same_curve.contains(&(gid, e.id))
            };
            if !same {
                continue;
            }
            let Some(g) = self.edge_info(1 - side, gid) else {
                continue;
            };
            let Ok(on_g) = g.curve.project(mid) else {
                continue;
            };
            for gb in self.blocks_of(g) {
                let g_mid = g.curve.point(gb.range.midpoint());
                let Ok(on_e) = e.curve.project(g_mid) else {
                    continue;
                };
                let e_in_g = Self::strictly_inside(g, gb.range, on_g.t);
                let g_in_e = Self::strictly_inside(e, block.range, on_e.t);
                if !(e_in_g || g_in_e) {
                    continue;
                }
                if !(e_in_g && g_in_e) {
                    return Err(fault());
                }
                let reversed = e.curve.eval(block.range.midpoint()).d1.dot(
                    &g.curve
                        .eval(g.in_range(on_g.t).unwrap_or(gb.range.midpoint()))
                        .d1,
                ) < 0.0;
                let (es, ee) = (self.canonical(block.start), self.canonical(block.end));
                let (gs, ge) = (self.canonical(gb.start), self.canonical(gb.end));
                let ends_match = if reversed {
                    es == ge && ee == gs
                } else {
                    es == gs && ee == ge
                };
                if !ends_match {
                    return Err(fault());
                }
                return Ok(Some((gid, gb, reversed)));
            }
        }
        Ok(None)
    }

    /// `block` of `e`, an edge of face `f` of operand `side`, as an image
    /// on `other` under pair `pi`: kept when the block lies inside that
    /// face by its polygons — within the band of a loop without being on
    /// any edge of it, the winding number decides — with its pcurve there
    /// and the edge's tolerance raised to the pcurve's residual; `None`
    /// when it lies outside.
    fn image(
        &self,
        pi: usize,
        side: usize,
        e: &EdgeInfo<'m>,
        f: &FaceInfo<'m>,
        other: &FaceInfo<'m>,
        block: &Block,
    ) -> Result<Option<EdgeImage>, OpError> {
        let mid = e.curve.point(block.range.midpoint());
        let projection = other
            .surface
            .project(mid)
            .map_err(|err| geometry(err, e.shape(), other.shape()))?;
        let (s, shift) = other.domain.side(projection.uv);
        let uv_mid = projection.uv + shift;
        let inside = match s {
            Side::Outside => false,
            Side::Inside => true,
            Side::Boundary => other.domain.winds_around(projection.uv),
        };
        if !inside {
            return Ok(None);
        }
        let fit = e.tolerance + f.tolerance.max(other.tolerance);
        let (pcurve, residual) =
            self.pcurve_of(other, f, e.curve, block.range, uv_mid, fit, &[])?;
        let tolerance = e.tolerance.max(residual);
        if tolerance > self.precision.max_tolerance {
            return Err(OpError::Tolerance {
                entity: e.shape(),
                wanted: tolerance,
            });
        }
        Ok(Some(EdgeImage {
            pair: pi,
            side,
            edge: e.id,
            index: block.index,
            range: block.range,
            pcurve,
            tolerance,
        }))
    }

    /// Every piece of every edge of both faces of each `Coincident` pair
    /// decided against the other face: inside it, an image with its
    /// pcurve there; along its boundary, a common block with the piece
    /// of the other face's edge it coincides with, recorded once — from
    /// `b`'s side, under the first pair it is met in — with a pcurve for
    /// each use of `b`'s piece; outside it, nothing.
    fn coincident(&mut self) -> Result<(), OpError> {
        let mut images = Vec::new();
        let mut blocks = Vec::new();
        let mut floors: Vec<(End, f64)> = Vec::new();
        for pi in 0..self.pairs.len() {
            if self.pairs[pi].intersection != SurfaceIntersection::Coincident {
                continue;
            }
            let (ia, ib) = self.pair_faces[pi];
            for side in 0..2 {
                let (f, other) = if side == 0 {
                    (&self.faces[0][ia], &self.faces[1][ib])
                } else {
                    (&self.faces[1][ib], &self.faces[0][ia])
                };
                for &eid in f.edges() {
                    let Some(e) = self.edge_info(side, eid) else {
                        continue;
                    };
                    for block in self.blocks_of(e) {
                        // Along the other face's boundary, by the curves: a
                        // common block, recorded once, from `b`'s side and
                        // under the first pair it is met in.
                        if let Some((gid, gb, reversed)) =
                            self.matching_block(side, e, &block, other)?
                        {
                            let seen = blocks
                                .iter()
                                .any(|x: &CommonBlock| x.b == (e.id, block.index));
                            if side == 1 && !seen {
                                let (b, raised) =
                                    self.common_block(pi, e, &block, gid, &gb, reversed)?;
                                floors.extend(
                                    [block.start, block.end, gb.start, gb.end]
                                        .into_iter()
                                        .map(|end| (end, raised)),
                                );
                                blocks.push(b);
                            }
                            continue;
                        }
                        if let Some(image) = self.image(pi, side, e, f, other, &block)? {
                            floors.push((block.start, image.tolerance));
                            floors.push((block.end, image.tolerance));
                            images.push(image);
                        }
                    }
                }
            }
        }
        // An edge lying in a face of the other operand that no face of its
        // own is coincident with — a seam on a ruling two parallel walls
        // cross along, whose block of the section curve is the edge and
        // not a section edge — has no coincident neighbour to place it on
        // that face, so its pieces inside it are placed here, under the
        // pair of the first face that uses it.
        for &(eid, gid) in &self.coincident {
            let side = usize::from(self.edge_info(0, eid).is_none());
            let (Some(e), Some(other)) = (
                self.edge_info(side, eid),
                self.faces[1 - side].iter().find(|g| g.id == gid),
            ) else {
                continue;
            };
            let pair_with = |f: FaceId| {
                self.pairs.iter().position(|p| {
                    if side == 0 {
                        p.a == f && p.b == gid
                    } else {
                        p.a == gid && p.b == f
                    }
                })
            };
            let owners: Vec<(&FaceInfo<'m>, Option<usize>)> = self.faces[side]
                .iter()
                .filter(|f| f.edges().contains(&eid))
                .map(|f| (f, pair_with(f.id)))
                .collect();
            let placed = owners.iter().any(|&(_, pi)| {
                pi.is_some_and(|pi| self.pairs[pi].intersection == SurfaceIntersection::Coincident)
            });
            let Some((f, pi)) = owners.iter().find_map(|&(f, pi)| Some((f, pi?))) else {
                continue;
            };
            if placed {
                continue;
            }
            for block in self.blocks_of(e) {
                // A piece along the other face's boundary is that face's
                // edge, not a curve inside it — a pipe's cap circle on the
                // bend it runs into, the two walls tangent along it — and
                // the coincident caps beside it hold the common block.
                if let Some((gid, gb, _)) = self.matching_block(side, e, &block, other)? {
                    let (ours, theirs) = ((e.id, block.index), (gid, gb.index));
                    let held = blocks.iter().any(|x: &CommonBlock| {
                        (x.a, x.b) == (theirs, ours) || (x.a, x.b) == (ours, theirs)
                    });
                    if !held {
                        return Err(OpError::Internal(Fault::Invariant {
                            what: "an edge along a face's boundary with no common block",
                        }));
                    }
                    continue;
                }
                if let Some(image) = self.image(pi, side, e, f, other, &block)? {
                    floors.push((block.start, image.tolerance));
                    floors.push((block.end, image.tolerance));
                    images.push(image);
                }
            }
        }
        // A piece of an edge that a block of a section curve lies along
        // ([`Self::along_block`]) is that block on the pair's other face:
        // its image there, where the section edge would have been.
        for along in &self.along {
            let (ia, ib) = self.pair_faces[along.pair];
            let (f, other) = if along.side == 0 {
                (&self.faces[0][ia], &self.faces[1][ib])
            } else {
                (&self.faces[1][ib], &self.faces[0][ia])
            };
            let Some(e) = self.edge_info(along.side, along.edge) else {
                continue;
            };
            // The piece is placed once on a face, whichever pair placed it
            // first: an edge lying in the face is imaged there already.
            let on = |x: &EdgeImage| {
                let pair = &self.pairs[x.pair];
                if x.side == 0 { pair.b } else { pair.a }
            };
            let held = images.iter().any(|x: &EdgeImage| {
                (x.edge, x.index, on(x)) == (along.edge, along.block.index, other.id)
            });
            if held {
                continue;
            }
            // Outside the other face, the block bounds nothing there.
            let Some(image) = self.image(along.pair, along.side, e, f, other, &along.block)? else {
                continue;
            };
            floors.push((along.block.start, image.tolerance));
            floors.push((along.block.end, image.tolerance));
            images.push(image);
        }
        for (end, tolerance) in floors {
            if let End::Section(k) = self.canonical(end) {
                let v = &mut self.vertices[k];
                v.floor = v.floor.max(tolerance);
            }
        }
        self.images = images;
        self.blocks = blocks;
        Ok(())
    }

    /// A common block: `b`'s piece `block` of `e` is `a`'s piece `gb` of
    /// `gid`, with the pcurve of `a`'s piece for every use of `b`'s by a
    /// face of `b`, and the tolerance the two edges' raised to the
    /// residuals.
    fn common_block(
        &self,
        pi: usize,
        e: &EdgeInfo<'m>,
        block: &Block,
        gid: EdgeId,
        gb: &Block,
        reversed: bool,
    ) -> Result<(CommonBlock, f64), OpError> {
        let m = self.m;
        let g = self
            .edge_info(0, gid)
            .ok_or(OpError::Internal(Fault::Invariant {
                what: "a's edge info for a common block",
            }))?;
        let g_mid = g.curve.point(gb.range.midpoint());
        // `b`'s parameter of `a`'s midpoint: where each use's own pcurve
        // gives the (u, v) the fitted one has to be placed at.
        let on_e = e
            .curve
            .project(g_mid)
            .map_err(|err| geometry(err, g.shape(), e.shape()))?;
        let t_e = e.in_range(on_e.t).unwrap_or(block.range.midpoint());
        let mut tolerance = g.tolerance.max(e.tolerance);
        let mut pcurves = Vec::new();
        for fb in &self.faces[1] {
            if !fb.edges().contains(&e.id) {
                continue;
            }
            let face = m.face(fb.id)?;
            for c in face.loops().iter().flat_map(|l| l.coedges()) {
                if c.edge() != e.id {
                    continue;
                }
                let own = m.curve2(c.pcurve())?;
                let uv_mid = own.point(t_e);
                let fit = g.tolerance.max(e.tolerance) + fb.tolerance;
                let (pc, residual) = self.pcurve_of(
                    fb,
                    g_face_of(self, gid),
                    g.curve,
                    gb.range,
                    uv_mid,
                    fit,
                    &[],
                )?;
                tolerance = tolerance.max(residual);
                pcurves.push((c.pcurve(), pc));
            }
        }
        if tolerance > self.precision.max_tolerance {
            return Err(OpError::Tolerance {
                entity: e.shape(),
                wanted: tolerance,
            });
        }
        Ok((
            CommonBlock {
                pair: pi,
                a: (gid, gb.index),
                b: (e.id, block.index),
                reversed,
                pcurves,
                tolerance,
            },
            tolerance,
        ))
    }

    fn finish(self) -> Interferences {
        let m = self.m;
        let vertices = self.vertices.iter().map(|v| v.finish(m)).collect();
        Interferences {
            a: self.a,
            b: self.b,
            pairs: self.pairs,
            hits: self.hits,
            section_crossings: self.section_crossings,
            vertices,
            paves: self.paves,
            curves: self.curves,
            sections: self.sections,
            contacts: self.contacts,
            coincident: self.coincident,
            crossings: self.crossings,
            images: self.images,
            blocks: self.blocks,
        }
    }
}

/// A face of `a` that uses edge `gid`, for naming in an error; the first
/// face of `a` when none does.
fn g_face_of<'b, 'm>(build: &'b Build<'m, '_>, gid: EdgeId) -> &'b FaceInfo<'m> {
    build.faces[0]
        .iter()
        .find(|f| f.edges().contains(&gid))
        .unwrap_or(&build.faces[0][0])
}

#[cfg(test)]
mod tests {
    use super::{BTreeMap, Candidate, VertexId, components};
    use arris_check::arris_topo::arris_math::Point3;

    fn at(x: f64, y: f64, tolerance: f64, existing: &[u32]) -> Candidate {
        Candidate {
            point: Point3::new(x, y, 0.0),
            tolerance,
            existing: existing.iter().map(|&i| VertexId::new(i, 0)).collect(),
        }
    }

    /// The partition as sets of the nodes' own names, whatever their
    /// order.
    fn partition(nodes: &[(usize, Candidate)]) -> Vec<Vec<usize>> {
        let cands: Vec<Candidate> = nodes.iter().map(|(_, c)| c.clone()).collect();
        let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for (k, l) in components(&cands).into_iter().enumerate() {
            groups.entry(l).or_default().push(nodes[k].0);
        }
        let mut out: Vec<Vec<usize>> = groups
            .into_values()
            .map(|mut g| {
                g.sort_unstable();
                g
            })
            .collect();
        out.sort();
        out
    }

    /// The seam fixture's three points — the crossing vertex and the
    /// seam's two crossings, 1.7e-7 and 2.4e-7 apart at 1e-7 — are one
    /// component in every order; a first-come merge made three of them
    /// in one order and two in another. A fourth point three tolerances
    /// off stays apart, and a fifth naming an operand vertex another
    /// names joins it at any distance.
    #[test]
    fn components_do_not_depend_on_creation_order() {
        let e = 1.22e-7;
        let nodes = [
            (0, at(0.0, 0.0, 1e-7, &[])),
            (1, at(e, e, 1e-7, &[])),
            (2, at(-e, e, 1e-7, &[])),
            (3, at(3e-7 + e, e, 1e-7, &[])),
            (4, at(1.0, 0.0, 1e-7, &[7])),
            (5, at(1.0, 5e-6, 1e-7, &[7])),
        ];
        let want = vec![vec![0, 1, 2], vec![3], vec![4, 5]];
        let n = nodes.len();
        // Every rotation and its reverse: each node first, each last.
        for r in 0..n {
            let mut order: Vec<(usize, Candidate)> =
                (0..n).map(|i| nodes[(i + r) % n].clone()).collect();
            assert_eq!(partition(&order), want, "rotation {r}");
            order.reverse();
            assert_eq!(partition(&order), want, "rotation {r}, reversed");
        }
    }

    /// The label is the least index of the component, so the first
    /// member in the nodes' order numbers it.
    #[test]
    fn a_component_is_labelled_by_its_first_member() {
        let nodes = [
            at(5.0, 0.0, 1e-7, &[]),
            at(0.0, 0.0, 1e-7, &[]),
            at(1.5e-7, 0.0, 1e-7, &[]),
            at(5.0 + 1.9e-7, 0.0, 1e-7, &[]),
        ];
        assert_eq!(components(&nodes), vec![0, 1, 1, 0]);
    }
}
