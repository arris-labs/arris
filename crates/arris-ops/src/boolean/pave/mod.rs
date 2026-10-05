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

use arris_geom::region2::Side;
use arris_geom::{Curve, GeomError, Surface};
use arris_math::{Aabb, Interval, Meter, Point2, Point3, Precision, Tolerance};
use arris_topo::{Body, EdgeId, FaceId, Model, Shape, VertexId};

use super::faces::{EdgeInfo, FaceInfo};
use super::{
    CommonBlock, Contact, EdgeEdgeHit, EdgeFaceHit, EdgeImage, FacePair, Interferences, Pave,
    SectionCrossing, SectionCurve, SectionEdge, SectionVertex, VertexSource,
};
use crate::error::OpError;

mod coincident;
mod hits;
mod sections;

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
    use arris_math::Point3;

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
