//! The boolean's result (ADR-0004): every face of both operands split
//! into pieces, each piece classified at a point inside it against the
//! other operand and kept or dropped by the selection table — a piece
//! lying on a coincident face of the other operand by the two normals —
//! the survivors grouped into shells by the edges they share and ordered
//! into lumps (ADR-0006), assembled through `Builder::assemble` with every
//! untouched entity kept by id, and the provenance written from the pieces
//! as they are made.

use std::collections::{BTreeMap, BTreeSet};

use arris_check::arris_topo::arris_geom::{
    GeomError, GeomKind, MeetKind, Surface, SurfaceIntersection,
};
use arris_check::arris_topo::arris_math::{
    Interval, Meter, Point2, Point3, Precision, Tolerance, Vec3,
};
use arris_check::arris_topo::builder::Builder;
use arris_check::arris_topo::entity::BodyKind;
use arris_check::arris_topo::{
    Body, Curve2Id, CurveId, EdgeId, EntityId, Face as FaceHandle, FaceId, Model, Provenance,
    Shape, ShellId, VertexId,
};
use arris_check::{Classification, Classifier, ClassifyError, lumps};

use super::pieces::{Alias, ERef, EdgeOnFace, PieceUse, SplitFace, SubEdge, VRef, split_face};
use super::{Interferences, VertexSource, meet_curves};
use crate::error::{Fault, OpError, Reason, SplitFault};
use crate::pass::pass;
use crate::rebuild::{self, Kept, Plan, Policy, forward};

/// Which selection over the decomposition: one table, three
/// operations (`docs/ARCHITECTURE.md` §Operations).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Op {
    /// Keep what is outside the other operand.
    Fuse,
    /// Keep what is inside the other operand.
    Common,
    /// Keep the target outside the tool and the tool inside the target,
    /// reversed.
    Cut,
}

impl Op {
    /// Whether a piece of operand `side` that is inside (`true`) or
    /// outside the other survives, and whether reversed
    /// (`docs/ARCHITECTURE.md` §Operations, the selection table).
    fn select(self, side: usize, inside: bool) -> Option<bool> {
        match (self, side, inside) {
            (Op::Fuse, _, false) | (Op::Common, _, true) | (Op::Cut, 0, false) => Some(false),
            (Op::Cut, 1, true) => Some(true),
            _ => None,
        }
    }

    /// Whether a piece of operand `side` lying on a coincident face of
    /// the other survives, given whether the two effective normals
    /// agree: once, from the first operand, when they agree in `fuse`
    /// and `common` and when they oppose in `cut`; never from the
    /// second (the selection table's coincident row).
    fn select_on(self, side: usize, agree: bool) -> bool {
        side == 0
            && match self {
                Op::Fuse | Op::Common => agree,
                Op::Cut => !agree,
            }
    }

    fn policy(self, side: usize) -> Policy {
        match (self, side) {
            (Op::Cut, 1) => Policy::Regenerate,
            _ => Policy::Reuse,
        }
    }
}

/// A piece classified `On` something of the other operand its own face
/// is neither coincident nor tangent with — an edge, a vertex, a face of
/// a crossing pair — that the transversal rule cannot decide either,
/// or a tangent pair the curvature rule cannot decide, its two
/// curvatures equal: the pair the kernel has no recipe for, named.
fn unsupported(m: &Model, face: FaceId, on: Shape) -> OpError {
    let kind = |s: Shape| -> GeomKind {
        match s.id {
            EntityId::Face(f) => m
                .face(f)
                .ok()
                .and_then(|f| m.surface(f.surface()).ok())
                .map_or(GeomKind::Point, |s| GeomKind::Surface(s.kind())),
            EntityId::Edge(e) => m
                .edge(e)
                .ok()
                .and_then(|e| e.curve())
                .and_then(|(c, _)| m.curve(c).ok())
                .map_or(GeomKind::Point, |c| GeomKind::Curve(c.kind())),
            EntityId::Vertex(_) | EntityId::Shell(_) | EntityId::Body(_) => GeomKind::Point,
        }
    };
    let a = forward(face);
    OpError::Unsupported {
        a: (kind(a), a),
        b: (kind(on), on),
    }
}

/// A piece of `face` that `body`'s classifier could not classify. A ray
/// has no closed form against a NURBS face (`arris_check::classify_point`),
/// and meets the first one of `body`'s faces in the classifier's order
/// on every direction it does not abandon, so that face is the one the
/// error is about: the pair is named as the unsupported one it is, the
/// NURBS cycle's (ADR-0026 §5), never an internal fault. Anything else
/// the classifier reports is one.
fn classify_fault(m: &Model, face: FaceId, body: Body, e: ClassifyError) -> OpError {
    if let ClassifyError::Geometry(GeomError::Unsupported { .. }) = e {
        let nurbs = m.closure(body).ok().and_then(|c| {
            c.faces.into_iter().find(|&g| {
                m.face(g)
                    .and_then(|g| m.surface(g.surface()))
                    .is_ok_and(|s| matches!(s, Surface::Nurbs(_)))
            })
        });
        if let Some(g) = nurbs {
            return unsupported(m, face, forward(g));
        }
    }
    OpError::Internal(Fault::Classify(e))
}

/// The whole build, over the pave model.
struct Build<'m> {
    m: &'m Model,
    precision: Precision,
    i: &'m Interferences,
    op: Op,
    bodies: [Body; 2],
    /// Each operand's vertices, ascending.
    vertices: [Vec<VertexId>; 2],
    /// Each operand's edges, in iteration order.
    edges: [Vec<EdgeId>; 2],
    /// Each operand's faces with their uses, in iteration order.
    faces: [Vec<FaceHandle>; 2],
    curve_ids: Vec<CurveId>,
    section_pcurves: Vec<[Curve2Id; 2]>,
    /// Per image, its pcurve on the face it lies in.
    image_pcurves: Vec<Curve2Id>,
    /// Per common block, the pcurve for each use of `b`'s piece, keyed
    /// by the use's own pcurve.
    block_pcurves: Vec<Vec<(Curve2Id, Curve2Id)>>,
    /// The vertex each section vertex is realised as.
    vref_of: Vec<VRef>,
    /// Operand vertices whose tolerance a section vertex raised.
    retolerated: BTreeMap<VertexId, f64>,
    /// Operand vertices merged into a section vertex that another
    /// operand vertex represents.
    merged_into: BTreeMap<VertexId, usize>,
    sub_edges: BTreeMap<EdgeId, Vec<SubEdge>>,
    touched: BTreeSet<EdgeId>,
    /// Every use of a piece of `b`'s edge that is a common block,
    /// rewritten to `a`'s piece.
    alias_uses: BTreeMap<(ERef, Curve2Id), Alias>,
    /// The piece of `a` each common block of `b` stands for.
    alias_of: BTreeMap<ERef, ERef>,
    /// Edge pieces whose tolerance an image or a common block raised.
    edge_tolerance: BTreeMap<ERef, f64>,
    kept: Vec<Kept>,
    /// Per operand face, whether it was untouched and which of `kept` are
    /// its pieces.
    face_pieces: BTreeMap<FaceId, (bool, Vec<usize>)>,
    /// `true` once a piece lying on the other operand was dropped: what
    /// distinguishes a result with no thickness from one with no
    /// material.
    dropped_on: bool,
}

impl<'m> Build<'m> {
    fn side_of_vertex(&self, v: VertexId) -> usize {
        usize::from(self.vertices[0].binary_search(&v).is_err())
    }

    /// The vertex an operand vertex is realised as: itself, or the one
    /// standing for the section vertex it was merged into.
    fn vref_of_operand(&self, v: VertexId) -> VRef {
        match self.merged_into.get(&v) {
            Some(&k) => self.vref_of[k],
            None => VRef::Existing(v),
        }
    }

    /// Every section vertex as a vertex of the result: its own, or the
    /// operand vertex it coincides with — one of a `Reuse` operand where
    /// there is a choice — re-tolerated when the merge grew past the
    /// stored tolerance.
    fn realise_vertices(&mut self) -> Result<(), OpError> {
        for v in &self.i.vertices {
            if v.existing.is_empty() {
                self.vref_of.push(VRef::Section(self.vref_of.len()));
                continue;
            }
            let rep = v
                .existing
                .iter()
                .copied()
                .find(|&x| self.op.policy(self.side_of_vertex(x)) == Policy::Reuse)
                .or_else(|| v.existing.first().copied())
                .ok_or(OpError::Internal(Fault::Invariant {
                    what: "a section vertex's existing operand vertices",
                }))?;
            let stored = self.m.vertex(rep)?.tolerance();
            if v.tolerance > stored {
                self.retolerated.insert(rep, v.tolerance);
            }
            let k = self.vref_of.len();
            for &x in &v.existing {
                if x != rep {
                    self.merged_into.insert(x, k);
                }
            }
            self.vref_of.push(VRef::Existing(rep));
        }
        Ok(())
    }

    /// Every operand edge cut at its paves, its ends the vertices they
    /// are realised as; an edge with a pave, a re-tolerated end or an end
    /// merged into another operand's vertex is touched.
    ///
    /// The pieces are made **in split order** (ADR-0009,
    /// `docs/DATA-MODEL.md` §Provenance): ascending along the edge's own
    /// curve from its range's start — the paves come ascending by `t`
    /// ([`Interferences::paves`]) and a closed edge's range is one
    /// interval across the seam — and `index` is that order, which
    /// [`rebuild::write_provenance`] lists the images in.
    fn sub_edges(&mut self) -> Result<(), OpError> {
        for side in 0..2 {
            for &e in &self.edges[side] {
                let edge = *self.m.edge(e)?;
                let ends = [edge.start(), edge.end()].map(|v| self.vref_of_operand(v));
                let mut params = vec![edge.range().lo()];
                let mut vrefs = vec![ends[0]];
                let mut touched = ends
                    != [VRef::Existing(edge.start()), VRef::Existing(edge.end())]
                    || self.retolerated.contains_key(&edge.start())
                    || self.retolerated.contains_key(&edge.end());
                if let Some(paves) = self.i.paves.get(&e) {
                    for p in paves {
                        params.push(p.t);
                        vrefs.push(self.vref_of[p.vertex]);
                        touched = true;
                    }
                }
                params.push(edge.range().hi());
                vrefs.push(ends[1]);
                let mut subs = Vec::with_capacity(params.len() - 1);
                for k in 0..params.len() - 1 {
                    let (lo, hi) = (params[k], params[k + 1]);
                    let empty =
                        || OpError::Internal(Fault::Split(SplitFault::EmptySubEdge { edge: e }));
                    if hi.partial_cmp(&lo) != Some(core::cmp::Ordering::Greater) {
                        return Err(empty());
                    }
                    subs.push(SubEdge {
                        range: Interval::new(lo, hi).map_err(|_| empty())?,
                        start: vrefs[k],
                        end: vrefs[k + 1],
                    });
                }
                self.sub_edges.insert(e, subs);
                if touched {
                    self.touched.insert(e);
                }
            }
        }
        Ok(())
    }

    /// The common blocks as aliases: every use of `b`'s piece rewritten
    /// to `a`'s, `b`'s edge touched, `a`'s piece raised to the block's
    /// tolerance.
    fn aliases(&mut self) {
        for (k, b) in self.i.blocks.iter().enumerate() {
            let source = ERef::Sub {
                edge: b.b.0,
                index: b.b.1,
            };
            let target = ERef::Sub {
                edge: b.a.0,
                index: b.a.1,
            };
            let Some(sub) = self.sub_edges.get(&b.a.0).and_then(|subs| subs.get(b.a.1)) else {
                continue;
            };
            let (range, start, end) = (sub.range, sub.start, sub.end);
            self.alias_of.insert(source, target);
            self.touched.insert(b.b.0);
            for &(own, pcurve) in &self.block_pcurves[k] {
                self.alias_uses.insert(
                    (source, own),
                    Alias {
                        edge: target,
                        range,
                        reversed: b.reversed,
                        start,
                        end,
                        pcurve,
                    },
                );
            }
            let t = self.edge_tolerance.entry(target).or_insert(0.0);
            *t = t.max(b.tolerance);
        }
    }

    /// The images' and common blocks' tolerances applied: an edge piece
    /// raised above its edge's stored tolerance touches the edge, and an
    /// operand vertex at its end below it is re-tolerated, which touches
    /// every edge at that vertex (every vertex ≥ its edges).
    fn raise_tolerances(&mut self) -> Result<(), OpError> {
        for im in &self.i.images {
            let r = ERef::Sub {
                edge: im.edge,
                index: im.index,
            };
            let t = self.edge_tolerance.entry(r).or_insert(0.0);
            *t = t.max(im.tolerance);
        }
        let raised: Vec<(ERef, f64)> = self.edge_tolerance.iter().map(|(r, t)| (*r, *t)).collect();
        for (r, tolerance) in raised {
            let ERef::Sub { edge, index } = r else {
                continue;
            };
            let stored = self.m.edge(edge)?.tolerance();
            if tolerance <= stored {
                continue;
            }
            self.touched.insert(edge);
            let Some(sub) = self.sub_edges.get(&edge).and_then(|s| s.get(index)) else {
                continue;
            };
            for end in [sub.start, sub.end] {
                let VRef::Existing(v) = end else {
                    continue;
                };
                let stored = self.m.vertex(v)?.tolerance();
                let current = self.retolerated.get(&v).copied().unwrap_or(stored);
                if tolerance > current {
                    self.retolerated.insert(v, tolerance);
                }
            }
        }
        let mut touched = Vec::new();
        for (e, subs) in &self.sub_edges {
            let at_retolerated = subs.iter().any(|s| {
                [s.start, s.end]
                    .iter()
                    .any(|v| matches!(v, VRef::Existing(id) if self.retolerated.contains_key(id)))
            });
            if at_retolerated {
                touched.push(*e);
            }
        }
        self.touched.extend(touched);
        Ok(())
    }

    /// The section edges and the images as each face sees them.
    fn edges_by_face(&self) -> BTreeMap<FaceId, Vec<EdgeOnFace>> {
        let mut on: BTreeMap<FaceId, Vec<EdgeOnFace>> = BTreeMap::new();
        for (k, s) in self.i.sections.iter().enumerate() {
            let pair = &self.i.pairs[self.i.curves[s.curve].pair];
            for (side, face, other) in [(0, pair.a, pair.b), (1, pair.b, pair.a)] {
                on.entry(face).or_default().push(EdgeOnFace {
                    edge: ERef::Section(k),
                    range: s.range,
                    start: self.vref_of[s.start],
                    end: self.vref_of[s.end],
                    pcurve: self.section_pcurves[k][side],
                    other,
                });
            }
        }
        for (k, im) in self.i.images.iter().enumerate() {
            let pair = &self.i.pairs[im.pair];
            let (face, other) = if im.side == 0 {
                (pair.b, pair.a)
            } else {
                (pair.a, pair.b)
            };
            let Some(sub) = self.sub_edges.get(&im.edge).and_then(|s| s.get(im.index)) else {
                continue;
            };
            on.entry(face).or_default().push(EdgeOnFace {
                edge: ERef::Sub {
                    edge: im.edge,
                    index: im.index,
                },
                range: sub.range,
                start: sub.start,
                end: sub.end,
                pcurve: self.image_pcurves[k],
                other,
            });
        }
        on
    }

    /// The face of the other operand that `on` names, when face `f` of
    /// operand `side` makes a pair with it whose intersection `is` accepts.
    fn partner(
        &self,
        side: usize,
        f: FaceId,
        on: Shape,
        is: fn(&SurfaceIntersection) -> bool,
    ) -> Option<FaceHandle> {
        let EntityId::Face(g) = on.id else {
            return None;
        };
        let paired = self.i.pairs.iter().any(|p| {
            is(&p.intersection)
                && if side == 0 {
                    p.a == f && p.b == g
                } else {
                    p.a == g && p.b == f
                }
        });
        if !paired {
            return None;
        }
        self.faces[1 - side].iter().copied().find(|h| h.id == g)
    }

    /// The face of the other operand that `on` names, when face `f` of
    /// operand `side` is coincident with it.
    fn coincident_partner(&self, side: usize, f: FaceId, on: Shape) -> Option<FaceHandle> {
        self.partner(side, f, on, |i| *i == SurfaceIntersection::Coincident)
    }

    /// The face of the other operand that `on` names, when face `f` of
    /// operand `side` is tangent to it.
    fn tangent_partner(&self, side: usize, f: FaceId, on: Shape) -> Option<FaceHandle> {
        self.partner(side, f, on, |i| {
            meet_curves(i, MeetKind::Touch).next().is_some()
        })
    }

    fn surface_of(&self, h: FaceHandle) -> Result<&'m Surface, OpError> {
        let face = self.m.face(h.id)?;
        Ok(self.m.surface(face.surface())?)
    }

    /// The effective outward normal of face `h` at `point` — at its
    /// (u, v) `uv` when given, else at the surface's projection of it.
    fn outward_normal(
        &self,
        h: FaceHandle,
        uv: Option<Point2>,
        point: Point3,
    ) -> Result<Vec3, OpError> {
        let surface = self.surface_of(h)?;
        let uv = match uv {
            Some(uv) => uv,
            None => {
                surface
                    .project(point)
                    .map_err(|e| OpError::Internal(Fault::Geometry(e)))?
                    .uv
            }
        };
        let n = surface
            .normal(uv.x, uv.y)
            .ok_or(OpError::Internal(Fault::NoNormal { face: h.id }))?;
        let n = n.into_inner();
        Ok(if h.orientation.is_reversed() { -n } else { n })
    }

    /// `true` when the effective normals of `f` at its (u, v) `uv` and
    /// of `g` at the same point agree.
    fn normals_agree(
        &self,
        f: FaceHandle,
        uv: Point2,
        point: Point3,
        g: FaceHandle,
    ) -> Result<bool, OpError> {
        Ok(self
            .outward_normal(f, Some(uv), point)?
            .dot(&self.outward_normal(g, None, point)?)
            > 0.0)
    }

    /// Whether face `f`, tangent to face `g` of the other operand along a
    /// contact curve through `point` whose tangent there is `along`, lies
    /// inside the other operand beside the curve: the curvature rule
    /// (`docs/ARCHITECTURE.md` §Operations). With `n` the effective
    /// outward normal of `g` at the point, each surface leaves the shared
    /// tangent plane across the curve as `κ s² / 2` along `n`, `κ` its
    /// normal curvature across the curve signed against `n`; `g`'s body
    /// lies on the side of its surface away from `n`, so `f`'s piece is
    /// inside it exactly when `κ_f < κ_g`. The two surfaces agree along
    /// the curve to second order, so every direction across it decides
    /// the same: each surface is read along its own normal crossed with
    /// `along`, which lies in its tangent plane exactly. For a plane and a
    /// cylinder this is the plane outside the cylinder's surface and the
    /// cylinder on its axis's side of the plane. `None` where the two
    /// curvatures are equal — a touch of higher order the second forms
    /// cannot decide, compared exactly — or either is undefined.
    fn tangent_side(
        &self,
        f: FaceHandle,
        point: Point3,
        g: FaceHandle,
        along: Vec3,
    ) -> Result<Option<bool>, OpError> {
        let n = self.outward_normal(g, None, point)?;
        let tol = Tolerance::new(
            self.precision.default_tolerance,
            self.precision.angular_tolerance,
        );
        let signed = |h: FaceHandle| -> Result<Option<f64>, OpError> {
            let surface = self.surface_of(h)?;
            let uv = surface
                .project(point)
                .map_err(|e| OpError::Internal(Fault::Geometry(e)))?
                .uv;
            let Some(own) = surface.normal(uv.x, uv.y) else {
                return Ok(None);
            };
            let own = own.into_inner();
            let across = own.cross(&along);
            Ok(surface
                .normal_curvature(uv.x, uv.y, across, tol)
                .map(|k| if own.dot(&n) < 0.0 { -k } else { k }))
        };
        let (Some(kf), Some(kg)) = (signed(f)?, signed(g)?) else {
            return Ok(None);
        };
        Ok((kf != kg).then_some(kf < kg))
    }

    /// Whether a piece of face `f` of operand `side`, bounded by `loops`,
    /// lies inside the other operand, read where one of its section
    /// edges crosses the other operand's boundary: the *transversal
    /// rule* (`docs/ARCHITECTURE.md` §Operations). At the midpoint of a
    /// section edge of a crossing pair of `f` and `g`, the direction
    /// into the piece is the surface's normal crossed with the edge's
    /// tangent as the loop walks it — the loops run counter-clockwise
    /// about the surface's normal with the piece on their left — and the
    /// piece is inside exactly when that direction is against `g`'s
    /// effective outward normal. The piece crosses no face of the other
    /// operand inside itself, so every section edge on its boundary
    /// decides the same, and the one read is the one where the two
    /// surfaces are furthest from tangent. It asks nothing of the
    /// distance from the piece to the other operand, so a piece lying
    /// within the tolerance of it throughout — a sliver between a seam
    /// and two section curves beside the point where they cross — is
    /// decided like any other. `None` when the piece has no section edge
    /// of a crossing pair, or the surfaces are tangent within the
    /// angular tolerance along every one.
    fn transversal_side(
        &self,
        side: usize,
        f: FaceHandle,
        loops: &[Vec<PieceUse>],
    ) -> Result<Option<bool>, OpError> {
        let surface = self.surface_of(f)?;
        let mut best: Option<(f64, bool)> = None;
        for u in loops.iter().flatten() {
            let ERef::Section(k) = u.edge else {
                continue;
            };
            let Some(s) = self.i.sections.get(k) else {
                continue;
            };
            let Some(curve) = self.i.curves.get(s.curve) else {
                continue;
            };
            let Some(pair) = self.i.pairs.get(curve.pair) else {
                continue;
            };
            let crossing = pair
                .intersection
                .curves()
                .get(curve.index)
                .is_some_and(|c| c.kind == MeetKind::Crossing);
            if !crossing {
                continue;
            }
            let g = if side == 0 { pair.b } else { pair.a };
            let Some(g) = self.faces[1 - side].iter().copied().find(|h| h.id == g) else {
                continue;
            };
            let e = curve.curve.eval(s.range.midpoint());
            let along = if u.orientation.is_reversed() {
                -e.d1
            } else {
                e.d1
            };
            let uv = surface
                .project(e.point)
                .map_err(|e| OpError::Internal(Fault::Geometry(e)))?
                .uv;
            let own = surface
                .normal(uv.x, uv.y)
                .ok_or(OpError::Internal(Fault::NoNormal { face: f.id }))?
                .into_inner();
            let into = own.cross(&along);
            let length = into.norm();
            if !(length.is_finite() && length > 0.0) {
                continue;
            }
            let n = self.outward_normal(g, None, e.point)?;
            let cos = into.dot(&n) / length;
            if cos.abs() > self.precision.angular_tolerance
                && best.is_none_or(|(c, _)| cos.abs() > c)
            {
                best = Some((cos.abs(), cos < 0.0));
            }
        }
        Ok(best.map(|(_, inside)| inside))
    }

    /// The tangent at `point` of the curve face `f` of operand `side`
    /// touches face `g` along: the nearest of their pair's touching curves.
    fn contact_tangent(&self, side: usize, f: FaceId, g: FaceId, point: Point3) -> Option<Vec3> {
        let pair = self.i.pairs.iter().find(|p| {
            if side == 0 {
                p.a == f && p.b == g
            } else {
                p.a == g && p.b == f
            }
        })?;
        meet_curves(&pair.intersection, MeetKind::Touch)
            .filter_map(|(_, c)| {
                let on = c.project(point).ok()?;
                Some((on.distance, c.eval(on.t).d1))
            })
            .min_by(|x, y| x.0.total_cmp(&y.0))
            .map(|(_, d)| d)
    }

    /// Every contact decided at its midpoint: the piece of each face
    /// through it lies inside or outside the other operand by the
    /// curvature rule, and the selection table says whether it survives.
    /// A contact both pieces survive is two result faces touching along
    /// a curve interior to both, the slit refused by name (ADR-0004).
    fn contacts(&self) -> Result<(), OpError> {
        for c in &self.i.contacts {
            let pair = self
                .i
                .pairs
                .get(c.pair)
                .ok_or(OpError::Internal(Fault::Invariant {
                    what: "a contact's face pair",
                }))?;
            let find = |side: usize, id: FaceId| {
                self.faces[side]
                    .iter()
                    .copied()
                    .find(|h| h.id == id)
                    .ok_or(OpError::Internal(Fault::Invariant {
                        what: "a contact's face among its operand",
                    }))
            };
            let (fa, fb) = (find(0, pair.a)?, find(1, pair.b)?);
            let along = pair
                .intersection
                .curves()
                .get(c.curve)
                .filter(|m| m.kind == MeetKind::Touch)
                .ok_or(OpError::Internal(Fault::Invariant {
                    what: "a contact's curve touches",
                }))?
                .curve
                .eval(c.range.midpoint())
                .d1;
            let (Some(inside_a), Some(inside_b)) = (
                self.tangent_side(fa, c.point, fb, along)?,
                self.tangent_side(fb, c.point, fa, along)?,
            ) else {
                return Err(unsupported(
                    self.m,
                    fa.id,
                    Shape::new(fb.id, fb.orientation),
                ));
            };
            if self.op.select(0, inside_a).is_some() && self.op.select(1, inside_b).is_some() {
                return Err(OpError::Degenerate {
                    entities: vec![
                        Shape::new(fa.id, fa.orientation),
                        Shape::new(fb.id, fb.orientation),
                    ],
                    reason: Reason::TangentContact,
                });
            }
        }
        Ok(())
    }

    /// Every face of both operands split, each piece classified against
    /// the other operand and kept by the selection table.
    fn select(&mut self, meter: &mut Meter<'_>) -> Result<(), OpError> {
        let on = self.edges_by_face();
        // Every face split first — the step that runs in parallel —
        // then the pieces classified and selected in one order.
        let work: Vec<FaceHandle> = (0..2).flat_map(|side| self.faces[side].clone()).collect();
        let splits = self.split_faces(&work, &on, meter)?;
        // One classifier per operand, its faces read once for every piece
        // of the other operand's faces.
        let m = self.m;
        let fault = |e| OpError::Internal(Fault::Classify(e));
        let bodies = self.bodies;
        let classifiers = [
            Classifier::of_body(m, self.bodies[0]).map_err(fault)?,
            Classifier::of_body(m, self.bodies[1]).map_err(fault)?,
        ];
        let first = self.faces[0].len();
        for (k, (f, split)) in work.into_iter().zip(splits).enumerate() {
            let side = usize::from(k >= first);
            let other = &classifiers[1 - side];
            let policy = self.op.policy(side);
            let mut pieces = Vec::new();
            for piece in split.pieces {
                meter.tick()?;
                let class = other
                    .classify(piece.interior)
                    .map_err(|e| classify_fault(m, f.id, bodies[1 - side], e))?;
                let (flip, stands_for) = match class {
                    Classification::Inside | Classification::Outside => {
                        let inside = class == Classification::Inside;
                        let Some(flip) = self.op.select(side, inside) else {
                            continue;
                        };
                        (flip, None)
                    }
                    Classification::On(shape) => {
                        if let Some(g) = self.coincident_partner(side, f.id, shape) {
                            let agree = self.normals_agree(f, piece.uv, piece.interior, g)?;
                            if !self.op.select_on(side, agree) {
                                self.dropped_on = true;
                                continue;
                            }
                            (false, Some(g.id))
                        } else if let Some(g) = self.tangent_partner(side, f.id, shape) {
                            // The interior point lies on the curve the
                            // two faces touch along; the piece lies to one
                            // side of the other operand everywhere else.
                            let inside =
                                match self.contact_tangent(side, f.id, g.id, piece.interior) {
                                    Some(along) => {
                                        self.tangent_side(f, piece.interior, g, along)?
                                    }
                                    None => None,
                                };
                            let Some(inside) = inside else {
                                return Err(unsupported(self.m, f.id, shape));
                            };
                            let Some(flip) = self.op.select(side, inside) else {
                                continue;
                            };
                            (flip, None)
                        } else {
                            // Within the tolerance of a face its own is
                            // transversal to, or of an edge or a vertex:
                            // read at a section edge the piece has instead.
                            let Some(inside) = self.transversal_side(side, f, &piece.loops)? else {
                                return Err(unsupported(self.m, f.id, shape));
                            };
                            let Some(flip) = self.op.select(side, inside) else {
                                continue;
                            };
                            (flip, None)
                        }
                    }
                };
                pieces.push(self.kept.len());
                self.kept.push(Kept {
                    side,
                    face: f,
                    whole: split.untouched && policy == Policy::Reuse,
                    orientation: if flip {
                        f.orientation.flipped()
                    } else {
                        f.orientation
                    },
                    loops: piece.loops,
                    uv: piece.uv,
                    stands_for,
                });
            }
            self.split_order(&mut pieces);
            self.face_pieces.insert(f.id, (split.untouched, pieces));
        }
        Ok(())
    }

    /// `pieces` — indices into `kept`, the surviving pieces of one face —
    /// put in **split order** (ADR-0009, `docs/DATA-MODEL.md`
    /// §Provenance), the order the record lists them in: ascending by
    /// boundary key, and for equal keys by the interior point in the
    /// face's own (u, v), `u` first, two coordinates within the model's
    /// parametric tolerance counting as equal. The order the walk found
    /// the pieces in decides nothing but a full tie of both.
    fn split_order(&self, pieces: &mut [usize]) {
        let keys: BTreeMap<usize, Vec<Shape>> = pieces
            .iter()
            .map(|&k| (k, self.boundary_key(&self.kept[k])))
            .collect();
        // The face's own tolerance in (u, v): rounding in an interior
        // point never orders two pieces a symmetric split makes alike.
        let tolerance = self.precision.parametric_tolerance;
        let uv = |a: Point2, b: Point2| {
            [(a.x, b.x), (a.y, b.y)]
                .into_iter()
                .find(|(x, y)| (x - y).abs() > tolerance)
                .map_or(core::cmp::Ordering::Equal, |(x, y)| x.total_cmp(&y))
        };
        pieces.sort_by(|&a, &b| {
            keys[&a]
                .cmp(&keys[&b])
                .then_with(|| uv(self.kept[a].uv, self.kept[b].uv))
        });
    }

    /// A piece's **boundary key**: the origins of its boundary edges as
    /// the record names them — an operand edge for a piece of one, and
    /// both faces of the pair for a section edge — sorted, without
    /// duplicates. Every one is an input of the operation, whose ids a
    /// rebuild of the same upstream recipe repeats, so the key reads no
    /// output id and no geometry, and two pieces bounded by different
    /// entities compare the same way in every variant of the recipe.
    fn boundary_key(&self, piece: &Kept) -> Vec<Shape> {
        let mut key: Vec<Shape> = Vec::new();
        for u in piece.loops.iter().flatten() {
            match u.edge {
                ERef::Sub { edge, .. } => key.push(forward(edge)),
                ERef::Section(k) => {
                    let pair = &self.i.pairs[self.i.curves[self.i.sections[k].curve].pair];
                    key.push(forward(pair.a));
                    key.push(forward(pair.b));
                }
            }
        }
        key.sort();
        key.dedup();
        key
    }

    /// [`split_face`] over every face of both operands, in `a`'s faces'
    /// iteration order then `b`'s in the result however it was computed:
    /// over `rayon` behind `parallel`, a plain iterator otherwise.
    /// Splitting a face reads the model, the paves and the section edges
    /// on that face and writes nothing, which is what makes it the
    /// boolean's second parallel step (ADR-0004,
    /// `docs/ARCHITECTURE.md` §Threading).
    fn split_faces(
        &self,
        work: &[FaceHandle],
        on: &BTreeMap<FaceId, Vec<EdgeOnFace>>,
        meter: &mut Meter<'_>,
    ) -> Result<Vec<SplitFace>, OpError> {
        pass(work, meter, |f, mt| {
            mt.tick()?;
            split_face(
                self.m,
                &self.precision,
                f.id,
                &self.sub_edges,
                &self.touched,
                on.get(&f.id).map_or(&[][..], Vec::as_slice),
                &self.alias_uses,
            )
        })
    }

    /// The surviving pieces grouped into shells by the edges they share:
    /// each shell its pieces' indices into `kept`, ascending, and the
    /// shells in the order of their first piece. No piece at all is the
    /// typed refusal of a result with no material.
    fn shells(&self) -> Result<Vec<Vec<usize>>, OpError> {
        let entities = || {
            vec![
                Shape::new(self.bodies[0].id, self.bodies[0].orientation),
                Shape::new(self.bodies[1].id, self.bodies[1].orientation),
            ]
        };
        if self.kept.is_empty() {
            return Err(OpError::Degenerate {
                entities: entities(),
                reason: if self.dropped_on {
                    Reason::ZeroThickness
                } else {
                    Reason::Empty
                },
            });
        }
        let mut parent: Vec<usize> = (0..self.kept.len()).collect();
        fn root(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        let mut owner: BTreeMap<ERef, usize> = BTreeMap::new();
        for (k, piece) in self.kept.iter().enumerate() {
            for u in piece.loops.iter().flatten() {
                match owner.get(&u.edge) {
                    Some(&j) => {
                        let (a, b) = (root(&mut parent, k), root(&mut parent, j));
                        parent[a] = b;
                    }
                    None => {
                        owner.insert(u.edge, k);
                    }
                }
            }
        }
        let mut shell_of_root: BTreeMap<usize, usize> = BTreeMap::new();
        let mut shells: Vec<Vec<usize>> = Vec::new();
        for k in 0..self.kept.len() {
            let r = root(&mut parent, k);
            let shell = *shell_of_root.entry(r).or_insert(shells.len());
            if shell == shells.len() {
                shells.push(Vec::new());
            }
            if let Some(pieces) = shells.get_mut(shell) {
                pieces.push(k);
            }
        }

        // A manifold solid uses every edge twice and its shells share no
        // vertex (ADR-0006): an edge of more uses is two lumps touching
        // along it — the grouping above has already joined them — and a
        // vertex two shells reach is two touching at a point. Either is
        // named before anything is assembled.
        let mut uses: BTreeMap<ERef, usize> = BTreeMap::new();
        for u in self.kept.iter().flat_map(|p| p.loops.iter().flatten()) {
            *uses.entry(u.edge).or_default() += 1;
        }
        let mut shared: BTreeSet<Shape> = uses
            .iter()
            .filter(|&(_, &n)| n > 2)
            .flat_map(|(&e, _)| self.edge_entities(e))
            .collect();
        if shared.is_empty() {
            let mut shell_at: BTreeMap<VRef, usize> = BTreeMap::new();
            for (shell, pieces) in shells.iter().enumerate() {
                let edges = pieces
                    .iter()
                    .filter_map(|&k| self.kept.get(k))
                    .flat_map(|p| p.loops.iter().flatten())
                    .map(|u| u.edge);
                for v in edges.flat_map(|e| self.edge_ends(e)).flatten() {
                    match shell_at.get(&v) {
                        Some(&s) if s != shell => shared.extend(self.vertex_entities(v)),
                        Some(_) => {}
                        None => {
                            shell_at.insert(v, shell);
                        }
                    }
                }
            }
        }
        if !shared.is_empty() {
            return Err(OpError::Degenerate {
                entities: shared.into_iter().collect(),
                reason: Reason::NonManifold,
            });
        }
        // One shell touching itself at a vertex is the same statement: the
        // faces round the vertex close into more than one fan — a wall
        // left two pieces that meet only at a singular point of the
        // section, pinched between a drill's two exits — which no
        // manifold `Solid` holds (ADR-0022, the pinch).
        if let Some(v) = self.pinched() {
            return Err(OpError::Degenerate {
                entities: self
                    .vertex_entities(v)
                    .into_iter()
                    .collect::<BTreeSet<_>>()
                    .into_iter()
                    .collect(),
                reason: Reason::NonManifold,
            });
        }
        Ok(shells)
    }

    /// The first vertex, in `VRef` order, whose kept face uses close into
    /// more than one fan: the corners there — a use arriving at the
    /// vertex and the next leaving it, in one loop of one piece — joined
    /// wherever two share an edge piece. A manifold vertex's corners are
    /// one fan, a disc round it; `None` when every vertex's are.
    fn pinched(&self) -> Option<VRef> {
        let ends = |u: &PieceUse| {
            let [a, b] = self.edge_ends(u.edge)?;
            Some(if u.orientation.is_reversed() {
                [b, a]
            } else {
                [a, b]
            })
        };
        // Per vertex, its corners as the two edge pieces of each.
        let mut corners: BTreeMap<VRef, Vec<[ERef; 2]>> = BTreeMap::new();
        for piece in &self.kept {
            for l in &piece.loops {
                for (k, u) in l.iter().enumerate() {
                    let Some(next) = l.get((k + 1) % l.len()) else {
                        continue;
                    };
                    let Some([_, v]) = ends(u) else {
                        continue;
                    };
                    corners.entry(v).or_default().push([u.edge, next.edge]);
                }
            }
        }
        corners.into_iter().find_map(|(v, cs)| {
            let mut parent: Vec<usize> = (0..cs.len()).collect();
            fn root(parent: &mut [usize], mut i: usize) -> usize {
                while parent[i] != i {
                    parent[i] = parent[parent[i]];
                    i = parent[i];
                }
                i
            }
            let mut owner: BTreeMap<ERef, usize> = BTreeMap::new();
            for (k, c) in cs.iter().enumerate() {
                for e in c {
                    match owner.get(e) {
                        Some(&j) => {
                            let (a, b) = (root(&mut parent, k), root(&mut parent, j));
                            parent[a] = b;
                        }
                        None => {
                            owner.insert(*e, k);
                        }
                    }
                }
            }
            let fans = (0..cs.len()).filter(|&k| root(&mut parent, k) == k).count();
            (fans > 1).then_some(v)
        })
    }

    /// The vertices at the two ends of an edge piece, `None` for a piece
    /// the decomposition does not hold.
    fn edge_ends(&self, e: ERef) -> Option<[VRef; 2]> {
        match e {
            ERef::Sub { edge, index } => {
                let s = self.sub_edges.get(&edge)?.get(index)?;
                Some([s.start, s.end])
            }
            ERef::Section(k) => {
                let s = self.i.sections.get(k)?;
                Some([*self.vref_of.get(s.start)?, *self.vref_of.get(s.end)?])
            }
        }
    }

    /// What an error names for an edge piece: the operand edge it is a
    /// piece of, or the two faces a section edge was cut along.
    fn edge_entities(&self, e: ERef) -> Vec<Shape> {
        match e {
            ERef::Sub { edge, .. } => vec![forward(edge)],
            ERef::Section(k) => self
                .i
                .sections
                .get(k)
                .and_then(|s| self.i.curves.get(s.curve))
                .and_then(|c| self.i.pairs.get(c.pair))
                .map_or_else(Vec::new, |p| vec![forward(p.a), forward(p.b)]),
        }
    }

    /// What an error names for a vertex: the operand vertex it is, or the
    /// edges and faces whose hits and crossings made a section vertex —
    /// both faces of the pair for a section crossing.
    fn vertex_entities(&self, v: VRef) -> Vec<Shape> {
        match v {
            VRef::Existing(id) => vec![forward(id)],
            VRef::Section(k) => {
                let Some(sv) = self.i.vertices.get(k) else {
                    return Vec::new();
                };
                let hits = sv
                    .hits
                    .iter()
                    .filter_map(|&h| self.i.hits.get(h))
                    .flat_map(|h| [forward(h.edge), forward(h.face)]);
                let crossings = sv
                    .crossings
                    .iter()
                    .filter_map(|&x| self.i.crossings.get(x))
                    .flat_map(|x| [forward(x.a), forward(x.b)]);
                let section_crossings = sv
                    .section_crossings
                    .iter()
                    .filter_map(|&x| self.i.section_crossings.get(x))
                    .filter_map(|x| self.i.pairs.get(x.pair))
                    .flat_map(|p| [forward(p.a), forward(p.b)]);
                hits.chain(crossings).chain(section_crossings).collect()
            }
        }
    }
}

/// The result of a boolean over the pave model `i` under `op`.
pub(super) fn boolean(
    m: &mut Model,
    i: &Interferences,
    op: Op,
    meter: &mut Meter<'_>,
) -> Result<(Body, Provenance), OpError> {
    let bodies = [i.a, i.b];
    let closures = [m.closure(i.a)?, m.closure(i.b)?];
    let vertices = [closures[0].vertices.clone(), closures[1].vertices.clone()];
    let mut edges: [Vec<EdgeId>; 2] = [Vec::new(), Vec::new()];
    let mut faces: [Vec<FaceHandle>; 2] = [Vec::new(), Vec::new()];
    for side in 0..2 {
        edges[side] = m.edges(bodies[side])?.into_iter().map(|e| e.id).collect();
        faces[side] = m.faces(bodies[side])?;
    }
    // The shell of each operand face: what a result shell's provenance is
    // written against.
    let mut shell_of: [BTreeMap<FaceId, ShellId>; 2] = [BTreeMap::new(), BTreeMap::new()];
    for side in 0..2 {
        for shell in m.shells(bodies[side])? {
            let entity = m.shell(shell.id)?;
            for face in entity.faces() {
                shell_of[side].entry(face.id).or_insert(shell.id);
            }
        }
    }
    let precision = m.precision();

    m.transaction(|m| {
        // The section geometry, once.
        let curve_ids: Vec<CurveId> = i
            .curves
            .iter()
            .map(|c| m.add_curve(c.curve.clone()))
            .collect();
        let section_pcurves: Vec<[Curve2Id; 2]> = i
            .sections
            .iter()
            .map(|s| {
                [
                    m.add_curve2(s.pcurves[0].clone()),
                    m.add_curve2(s.pcurves[1].clone()),
                ]
            })
            .collect();
        let image_pcurves: Vec<Curve2Id> = i
            .images
            .iter()
            .map(|im| m.add_curve2(im.pcurve.clone()))
            .collect();
        let block_pcurves: Vec<Vec<(Curve2Id, Curve2Id)>> = i
            .blocks
            .iter()
            .map(|b| {
                b.pcurves
                    .iter()
                    .map(|(own, pc)| (*own, m.add_curve2(pc.clone())))
                    .collect()
            })
            .collect();

        let mut b = Build {
            m: &*m,
            precision,
            i,
            op,
            bodies,
            vertices: vertices.clone(),
            edges: edges.clone(),
            faces: faces.clone(),
            curve_ids,
            section_pcurves,
            image_pcurves,
            block_pcurves,
            vref_of: Vec::new(),
            retolerated: BTreeMap::new(),
            merged_into: BTreeMap::new(),
            sub_edges: BTreeMap::new(),
            touched: BTreeSet::new(),
            alias_uses: BTreeMap::new(),
            alias_of: BTreeMap::new(),
            edge_tolerance: BTreeMap::new(),
            kept: Vec::new(),
            face_pieces: BTreeMap::new(),
            dropped_on: false,
        };
        b.realise_vertices()?;
        b.sub_edges()?;
        b.aliases();
        b.raise_tolerances()?;
        b.contacts()?;
        b.select(meter)?;
        let mut shells = b.shells()?;
        if shells.len() > 1 {
            shells = b.lump_order(shells)?;
        }
        let plan = b.assembly(&shells)?;
        let Build {
            face_pieces,
            vref_of,
            merged_into,
            sub_edges,
            alias_of,
            kept,
            ..
        } = b;

        let (builder, slots) = Builder::assemble(m, precision.default_tolerance, plan.assembly)?;
        let built = builder.finish(m, BodyKind::Solid)?;

        // The output ids behind every reference.
        let mut out_vertex: BTreeMap<VRef, VertexId> = BTreeMap::new();
        for (v, &slot) in plan.new_vertices.iter().zip(&slots.vertices) {
            out_vertex.insert(*v, built.vertices[&slot]);
        }
        let mut out_edge: BTreeMap<ERef, EdgeId> = BTreeMap::new();
        for (e, &slot) in plan.new_edges.iter().zip(&slots.edges) {
            out_edge.insert(*e, built.edges[&slot]);
        }
        // Face slots follow the assembly's shells; `plan.faces` is the kept
        // piece behind each.
        let out_faces: BTreeMap<usize, FaceId> = plan
            .faces
            .iter()
            .copied()
            .zip(slots.faces.iter().flatten().map(|&slot| built.faces[&slot]))
            .collect();
        let vertex_id = |v: VRef| -> Option<VertexId> {
            match v {
                VRef::Existing(id) if plan.kept_vertices.contains(&id) => Some(id),
                other => out_vertex.get(&other).copied(),
            }
        };
        let edge_id = |e: ERef| -> Option<EdgeId> {
            let e = alias_of.get(&e).copied().unwrap_or(e);
            match e {
                ERef::Sub { edge, index: 0 } if plan.kept_edges.contains(&edge) => Some(edge),
                other => out_edge.get(&other).copied(),
            }
        };

        let operand = |side: usize| rebuild::OperandWrite {
            body: bodies[side],
            policy: op.policy(side),
            vertices: &vertices[side],
            edges: &edges[side],
            faces: &faces[side],
            shells: &closures[side].shells,
            shell_of: &shell_of[side],
        };
        let mut p = rebuild::write_provenance(
            [operand(0), operand(1)],
            built.body,
            &sub_edges,
            &merged_into,
            &vref_of,
            vertex_id,
            edge_id,
            &face_pieces,
            &out_faces,
            &kept,
            &shells,
            &built.shells,
        );
        for (k, v) in i.vertices.iter().enumerate() {
            let VRef::Section(_) = vref_of[k] else {
                continue;
            };
            let Some(id) = out_vertex.get(&vref_of[k]) else {
                continue;
            };
            match v.source {
                VertexSource::Hits | VertexSource::SectionCrossing => {
                    for &h in &v.hits {
                        p.add_generated(forward(i.hits[h].edge), forward(*id));
                        p.add_generated(forward(i.hits[h].face), forward(*id));
                    }
                    for &x in &v.crossings {
                        p.add_generated(forward(i.crossings[x].a), forward(*id));
                        p.add_generated(forward(i.crossings[x].b), forward(*id));
                    }
                    for &x in &v.section_crossings {
                        let pair = &i.pairs[i.section_crossings[x].pair];
                        p.add_generated(forward(pair.a), forward(*id));
                        p.add_generated(forward(pair.b), forward(*id));
                    }
                }
                VertexSource::CurveStart { pair, .. } => {
                    p.add_generated(forward(i.pairs[pair].a), forward(*id));
                    p.add_generated(forward(i.pairs[pair].b), forward(*id));
                }
                // Always an operand's vertex, never a new one.
                VertexSource::Singular => {}
            }
        }
        // `i.sections` is curve order and then along each curve, so the
        // section edges one pair generates reach the record in split
        // order: along their own curve (ADR-0009).
        for (k, s) in i.sections.iter().enumerate() {
            let Some(&id) = out_edge.get(&ERef::Section(k)) else {
                continue;
            };
            let pair = &i.pairs[i.curves[s.curve].pair];
            p.add_generated(forward(pair.a), forward(id));
            p.add_generated(forward(pair.b), forward(id));
        }
        crate::verify(m, built.body)?;
        Ok((built.body, p))
    })
}

impl Build<'_> {
    /// `shells` reordered lump by lump — each outer shell, then the voids
    /// whose innermost container it is (ADR-0006) — by assembling them once
    /// into a copy of the model and reading `arris_check::lumps` of that
    /// body: the order B1 proves, from the code that proves it. The copy
    /// is dropped; the model is only read. Errors: the builder's refusal,
    /// and [`Fault::Lumps`] when the shells do not nest or the nesting is
    /// undecided.
    fn lump_order(&self, shells: Vec<Vec<usize>>) -> Result<Vec<Vec<usize>>, OpError> {
        let plan = self.assembly(&shells)?;
        let mut scratch = self.m.clone();
        let built = Builder::assemble(&scratch, self.precision.default_tolerance, plan.assembly)?
            .0
            .finish(&mut scratch, BodyKind::Solid)?;
        let found = lumps(&scratch, built.body).map_err(|e| OpError::Internal(Fault::Lumps(e)))?;
        let index: BTreeMap<ShellId, usize> = built
            .shells
            .iter()
            .enumerate()
            .map(|(i, &s)| (s, i))
            .collect();
        let mut shells: Vec<Option<Vec<usize>>> = shells.into_iter().map(Some).collect();
        // `lumps` names every shell once, as an outer shell or a void of
        // one, so every shell is taken.
        Ok(found
            .iter()
            .flat_map(|lump| core::iter::once(&lump.outer).chain(&lump.voids))
            .filter_map(|s| index.get(&s.id))
            .filter_map(|&i| shells.get_mut(i).and_then(Option::take))
            .collect())
    }

    /// The assembly of the kept pieces: a vertex spec for every new
    /// vertex they reach, an edge spec for every new edge piece, a face
    /// spec per piece — `Keep` for a whole untouched face of a `Reuse`
    /// operand — shell by shell in `shells`' order, in a deterministic
    /// order. `rebuild::assembly` does the work; this is the boolean's
    /// `Build` handed to it as explicit pieces and a per-operand policy.
    fn assembly(&self, shells: &[Vec<usize>]) -> Result<Plan, OpError> {
        rebuild::assembly(
            self.m,
            [self.op.policy(0), self.op.policy(1)],
            &self.vertices,
            &self.edges,
            &self.sub_edges,
            &self.touched,
            &self.retolerated,
            &self.edge_tolerance,
            self.i,
            &self.curve_ids,
            &self.vref_of,
            &self.kept,
            shells,
        )
    }
}
