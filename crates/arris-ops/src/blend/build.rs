//! Assembly: the insertions, face edits and cuts a set of stripes makes, rewritten through `rebuild::rewrite`.

use std::collections::{BTreeMap, BTreeSet};

use arris_geom::{Curve, Curve2, pcurve_on};
use arris_math::{Interval, Meter, Tolerance};
use arris_topo::builder::{EdgeKey, EdgeSpec, VertexKey, VertexSpec};
use arris_topo::entity::EdgeGeometry;
use arris_topo::{Body, Curve2Id, EdgeId, FaceId, Model, Orientation, Provenance, VertexId};

use super::chain::{chain, cusp_at, tangent_vertex};
use super::corner::{Corner, corner};
use super::ends::{EndKind, Trim, face_end};
use super::junction::{Run, junction};
use super::miter::{Miter, miter};
use super::ring::{Ring, RingEnd, RingEnds, placed_uv, ring, vertex_tolerance_of};
use super::stripe::{Stripe, contacts, stripe};
use super::{Blend, Kind, degenerate, invariant};
use crate::body_view::BodyView;
use crate::error::{BlendReason, OpError, Reason, fault_of};
use crate::rebuild;
use crate::rebuild::{AddedFace, Rewrite, StoredUse, forward};

/// The arc inserted into the face across a corner, at the junction the
/// consumed vertex stood at.
#[derive(Debug, Clone, Copy)]
pub(super) struct Insertion {
    pub(super) arc: EdgeKey,
    pub(super) pcurve: Curve2Id,
    /// `true` when the arc's `range.lo()` is at the contact at `u = 0`.
    pub(super) lo_first: bool,
    /// The corner edge the contact at `u = 0` cuts.
    pub(super) edge_at_lo: EdgeId,
    /// Where the face across is met twice (ADR-0043 §4): the two corner
    /// edges, whose stretch of the loop through the vertex becomes a loop
    /// of its own, closed by the arc.
    pub(super) splits: Option<[EdgeId; 2]>,
}

/// What a face's loops are rewritten with: a blended edge replaced by
/// the contact on this face, and an arc inserted at a consumed vertex.
#[derive(Default)]
pub(super) struct FaceEdit {
    pub(super) replace: BTreeMap<EdgeId, (EdgeKey, Curve2Id)>,
    pub(super) insert: BTreeMap<VertexId, Insertion>,
}

/// Where a corner edge is cut at each of its ends: the new vertex and the
/// parameter.
#[derive(Default, Clone, Copy)]
pub(super) struct Cuts {
    pub(super) lo: Option<(usize, f64)>,
    pub(super) hi: Option<(usize, f64)>,
}

/// The rewrite's indices of one blend's entities: its four trim vertices
/// `[end][contact]`, its two contact edges, its two end arcs and its
/// added face. At a miter or a corner the vertices and the arc are the
/// miter's or the corner's, shared with the other blends.
#[derive(Default, Clone, Copy)]
pub(super) struct Made {
    pub(super) vertices: [[usize; 2]; 2],
    pub(super) contacts: [usize; 2],
    pub(super) arcs: [usize; 2],
    pub(super) added: usize,
}

/// The rewrite's indices of one stripe's entities: its four trim vertices
/// `[end][contact]` and a fan's crossings, its two contact edges, each
/// end's arcs in the end's order and its added face. At a miter or a corner
/// the vertices and the one arc are the miter's or the corner's, shared
/// with the other blends.
#[derive(Default, Clone)]
pub(super) struct StripeMade {
    pub(super) vertices: [[usize; 2]; 2],
    pub(super) crossings: Vec<usize>,
    pub(super) contacts: [usize; 2],
    pub(super) arcs: [Vec<usize>; 2],
    pub(super) added: usize,
}

/// The rewrite's indices of one miter's entities: its two vertices, `q`
/// then `p3`, its edge, and a trim arc's end vertex and edge.
#[derive(Clone, Copy)]
pub(super) struct MiterMade {
    pub(super) vertices: [usize; 2],
    pub(super) edge: usize,
    pub(super) trim_arc: Option<(usize, usize)>,
}

impl MiterMade {
    /// The vertex the stripe at `side`'s far contact ends at: `p3`, or the
    /// trim arc's end on the wider blend.
    pub(super) fn far(&self, mt: &Miter, side: usize) -> usize {
        match (&mt.trim_arc, self.trim_arc) {
            (Some(arc), Some((end, _))) if arc.wide == side => end,
            _ => self.vertices[1],
        }
    }

    /// The stripe at `side`'s edges at the miter in its end's order, as
    /// `Miter::arcs` lists their curves.
    pub(super) fn arcs(&self, mt: &Miter, side: usize) -> Vec<usize> {
        match (&mt.trim_arc, self.trim_arc) {
            (Some(arc), Some((_, edge))) if arc.wide == side => {
                if mt.shared[side] == 0 {
                    vec![self.edge, edge]
                } else {
                    vec![edge, self.edge]
                }
            }
            _ => vec![self.edge],
        }
    }
}

/// The rewrite's indices of one corner's entities: its three vertices,
/// its three arcs by side, a sphere's pole and its added face.
#[derive(Clone, Copy)]
pub(super) struct CornerMade {
    pub(super) vertices: [usize; 3],
    pub(super) arcs: [usize; 3],
    pub(super) pole: Option<usize>,
    pub(super) added: usize,
}

/// Records `cut` at one end of a corner edge, once.
pub(super) fn cut_once(
    cuts: &mut BTreeMap<EdgeId, Cuts>,
    trim: &Trim,
    vertex: usize,
) -> Result<(), OpError> {
    let cut = cuts.entry(trim.edge).or_default();
    let slot = if trim.cuts_lo {
        &mut cut.lo
    } else {
        &mut cut.hi
    };
    if slot.replace((vertex, trim.t)).is_some() {
        return Err(invariant("one cut per end of a corner edge"));
    }
    Ok(())
}

/// What every phase of a blend reads and writes through: the model, the
/// body's view, the kind of blend, the tolerance and sample count of the
/// model's precision, and the meter whose polls `cancel_counts.txt` holds.
pub(super) struct BlendCtx<'a, 'b> {
    pub(super) m: &'a mut Model,
    pub(super) view: BodyView,
    pub(super) kind: Kind,
    pub(super) tol: Tolerance,
    pub(super) samples: usize,
    pub(super) meter: &'a mut Meter<'b>,
}

impl<'a, 'b> BlendCtx<'a, 'b> {
    /// Reads `body` and the model's precision.
    fn new(
        m: &'a mut Model,
        body: Body,
        kind: Kind,
        meter: &'a mut Meter<'b>,
    ) -> Result<Self, OpError> {
        let precision = m.precision();
        let view = BodyView::of(m, body)?;
        Ok(BlendCtx {
            m,
            view,
            kind,
            tol: precision.tolerance(),
            samples: precision.check_samples,
            meter,
        })
    }

    /// The read-only environment and the meter, split so a phase holds both.
    pub(super) fn split(&mut self) -> (Env<'_>, &mut Meter<'b>) {
        (
            Env {
                m: &*self.m,
                view: &self.view,
                kind: self.kind,
                tol: self.tol,
                samples: self.samples,
            },
            &mut *self.meter,
        )
    }

    /// The fields a phase uses, split so each borrows on its own.
    fn parts(
        &mut self,
    ) -> (
        &mut Model,
        &BodyView,
        Kind,
        Tolerance,
        usize,
        &mut Meter<'b>,
    ) {
        (
            &mut *self.m,
            &self.view,
            self.kind,
            self.tol,
            self.samples,
            &mut *self.meter,
        )
    }
}

/// The corner edges replaced by a shortened or lengthened one.
type Shortened = BTreeMap<EdgeId, EdgeKey>;

/// The pcurves derived again over a lengthened edge's range.
type Rederived = BTreeMap<(EdgeId, Curve2Id), Curve2Id>;

/// The blended edges at each vertex.
type AtVertex = BTreeMap<VertexId, Vec<EdgeId>>;

/// What a phase reads without changing it: the model, the body's view, the
/// kind of blend and the model's tolerance and sample count. `Copy`, so a
/// phase takes it beside the meter it polls.
#[derive(Clone, Copy)]
pub(super) struct Env<'a> {
    pub(super) m: &'a Model,
    pub(super) view: &'a BodyView,
    pub(super) kind: Kind,
    pub(super) tol: Tolerance,
    pub(super) samples: usize,
}

/// The chained edges sorted by what builds them.
struct Classified {
    /// Stripes: the open edges that are not circular arcs.
    open: Vec<EdgeId>,
    /// Circular arcs, built once the junctions are known.
    arcs: Vec<EdgeId>,
    /// The closed edges' rings, built already.
    rings: Vec<Ring>,
}

/// The vertex joins of the blended edges, by vertex.
struct Joins {
    miters: Vec<Miter>,
    miter_at: BTreeMap<VertexId, usize>,
    corners: Vec<Corner>,
    corner_at: BTreeMap<VertexId, usize>,
}

/// The rewrite being filled in: its entities, the cuts to corner edges and
/// the edits to faces.
struct Draft {
    rw: Rewrite,
    cuts: BTreeMap<EdgeId, Cuts>,
    edits: BTreeMap<FaceId, FaceEdit>,
}

/// Every blend entity's indices in the rewrite.
struct Drafted {
    miters: Vec<MiterMade>,
    corners: Vec<CornerMade>,
    stripes: Vec<StripeMade>,
    rings: Vec<Made>,
}

/// Builds the blends of `edges`, in that order, into a rewrite of `body`
/// and returns the result with its provenance.
pub(super) fn build(
    m: &mut Model,
    body: Body,
    edges: &[EdgeId],
    kind: Kind,
    meter: &mut Meter<'_>,
) -> Result<(Body, Provenance), OpError> {
    let mut ctx = BlendCtx::new(m, body, kind, meter)?;
    let Classified {
        open,
        arcs,
        mut rings,
    } = classify(&mut ctx, body, edges)?;
    let (at_vertex, junctions) = vertex_joins(&mut ctx, &open, &arcs, &mut rings)?;
    let stripes = make_stripes(&mut ctx, &open)?;
    let joins = join_blends(&mut ctx, &open, &stripes, &rings, &at_vertex, &junctions)?;
    let blends = make_blends(&mut ctx, stripes, &joins)?;
    let mut draft = Draft {
        rw: Rewrite::default(),
        cuts: BTreeMap::new(),
        edits: BTreeMap::new(),
    };
    let miter_made = add_miters(&mut ctx, &mut draft, &joins.miters)?;
    let corner_made = add_corners(&mut ctx, &mut draft, &joins.corners);
    let mut made = Drafted {
        miters: miter_made,
        corners: corner_made,
        stripes: Vec::new(),
        rings: Vec::new(),
    };
    made.stripes = add_stripes(&mut ctx, &mut draft, &blends, &joins, &made)?;
    made.rings = add_rings(&mut ctx, &mut draft, &rings, &joins, &made)?;
    let (shortened, rederived) = shorten_edges(&mut ctx, &mut draft)?;
    rewrite_faces(&mut ctx, &mut draft, &shortened, &rederived)?;
    add_blend_faces(&mut ctx, &mut draft, &blends, &joins, &mut made)?;
    add_corner_faces(&mut ctx, &mut draft, &joins, &mut made);
    add_ring_faces(&mut ctx, &mut draft, &rings, &joins, &mut made);
    let out = rebuild::rewrite(ctx.m, body, draft.rw)?;
    Ok(record(out, &blends, &rings, &joins, &made))
}

/// The chained edges, each into the group that builds it: the closed edges' rings are built here.
fn classify(
    ctx: &mut BlendCtx<'_, '_>,
    body: Body,
    edges: &[EdgeId],
) -> Result<Classified, OpError> {
    let (env, meter) = ctx.split();
    let Env {
        m, view, kind, tol, ..
    } = env;
    // The named edges and every edge their chains run on into, in the
    // body's order.
    let reached = chain(m, view, edges, kind.size(), tol, meter)?;
    let chained: Vec<EdgeId> = m
        .edges(body)?
        .into_iter()
        .map(|e| e.id)
        .filter(|id| reached.contains(id))
        .collect();
    let no_junctions = BTreeSet::new();
    // A closed edge and a circular arc are rings, the one with no ends and
    // the other trimmed at its two or met at a junction; the rest are
    // stripes.
    let mut open: Vec<EdgeId> = Vec::with_capacity(chained.len());
    let mut arcs: Vec<EdgeId> = Vec::new();
    let mut rings: Vec<Ring> = Vec::new();
    for &e in &chained {
        let entity = *m.edge(e)?;
        match entity.curve() {
            Some(_) if entity.start() == entity.end() => {
                rings.push(ring(&env, meter, e, &no_junctions)?);
            }
            Some((curve, _)) if matches!(m.curve(curve)?, Curve::Circle { .. }) => {
                arcs.push(e);
            }
            Some(_) | None => open.push(e),
        }
    }
    Ok(Classified { open, arcs, rings })
}

/// The blended edges at each vertex, and the vertices that are junctions; the arcs' rings are built here.
fn vertex_joins(
    ctx: &mut BlendCtx<'_, '_>,
    open: &[EdgeId],
    arcs: &[EdgeId],
    rings: &mut Vec<Ring>,
) -> Result<(AtVertex, BTreeSet<VertexId>), OpError> {
    let (env, meter) = ctx.split();
    let Env {
        m, view, kind, tol, ..
    } = env;
    let edges = open;
    // The blended edges at each vertex: two at a tangent vertex meet in a
    // junction, two elsewhere in a miter, three in a corner, and more at a
    // vertex the closed forms do not cover; an arc meets no other blend but
    // at a junction.
    let mut at_vertex: BTreeMap<VertexId, Vec<EdgeId>> = BTreeMap::new();
    for &e in edges.iter().chain(arcs) {
        let entity = *m.edge(e)?;
        for v in [entity.start(), entity.end()] {
            at_vertex.entry(v).or_default().push(e);
        }
    }
    let mut junctions: BTreeSet<VertexId> = BTreeSet::new();
    for (&v, es) in &at_vertex {
        if let [ea, eb] = es[..]
            && tangent_vertex(m, view, ea, v, kind.size(), tol)? == Some(eb)
        {
            junctions.insert(v);
            continue;
        }
        // Both edges of a cusp blended meet in a corner patch, which no
        // closed form holds (ADR-0042 §6).
        if let [ea, eb] = es[..]
            && let Some((next, spine)) = cusp_at(m, view, ea, v, kind.size(), tol)?
            && next == eb
        {
            return Err(degenerate(
                vec![forward(ea), forward(spine), forward(v)],
                Reason::Blend(BlendReason::TangentChain),
            ));
        }
        if es.len() > 3 || (es.len() > 1 && es.iter().any(|e| arcs.contains(e))) {
            let entities = es.iter().map(|&e| forward(e)).chain([forward(v)]).collect();
            return Err(degenerate(
                entities,
                Reason::Blend(BlendReason::VertexBlend),
            ));
        }
    }
    for &e in arcs {
        rings.push(ring(&env, meter, e, &junctions)?);
    }
    at_vertex.retain(|v, es| junctions.contains(v) || !es.iter().any(|e| arcs.contains(e)));
    Ok((at_vertex, junctions))
}

/// One stripe per open edge.
fn make_stripes(ctx: &mut BlendCtx<'_, '_>, open: &[EdgeId]) -> Result<Vec<Stripe>, OpError> {
    let (m, view, kind, tol, _, meter) = ctx.parts();
    let edges = open;
    let mut stripes: Vec<Stripe> = Vec::with_capacity(edges.len());
    for &e in edges {
        meter.tick()?;
        stripes.push(stripe(m, view, e, kind, tol)?);
    }
    Ok(stripes)
}

/// The miters, the junctions and the corners, in vertex order.
fn join_blends(
    ctx: &mut BlendCtx<'_, '_>,
    open: &[EdgeId],
    stripes: &[Stripe],
    rings: &[Ring],
    at_vertex: &AtVertex,
    junctions: &BTreeSet<VertexId>,
) -> Result<Joins, OpError> {
    let (env, meter) = ctx.split();
    let Env { m, view, tol, .. } = env;
    let edges = open;
    let index_of: BTreeMap<EdgeId, usize> =
        edges.iter().enumerate().map(|(i, &e)| (e, i)).collect();
    let ring_of: BTreeMap<EdgeId, usize> =
        rings.iter().enumerate().map(|(i, r)| (r.edge, i)).collect();
    let run = |e: EdgeId| match (index_of.get(&e), ring_of.get(&e)) {
        (Some(&i), _) => Ok(Run::Line(&stripes[i])),
        (None, Some(&i)) => Ok(Run::Arc(&rings[i])),
        (None, None) => Err(invariant("a blended edge's stripe or ring")),
    };
    // The miters, the junctions and the corners, in vertex order.
    let mut miters: Vec<Miter> = Vec::new();
    let mut miter_at: BTreeMap<VertexId, usize> = BTreeMap::new();
    let mut corners: Vec<Corner> = Vec::new();
    let mut corner_at: BTreeMap<VertexId, usize> = BTreeMap::new();
    for (&v, es) in at_vertex {
        match es[..] {
            [ea, eb] if junctions.contains(&v) => {
                miter_at.insert(v, miters.len());
                miters.push(junction(m, view, run(ea)?, run(eb)?, v, tol, meter)?);
            }
            [ea, eb] => {
                miter_at.insert(v, miters.len());
                miters.push(miter(
                    &env,
                    meter,
                    &stripes[index_of[&ea]],
                    &stripes[index_of[&eb]],
                    v,
                )?);
            }
            [ea, eb, ec] => {
                corner_at.insert(v, corners.len());
                corners.push(corner(
                    &env,
                    meter,
                    [
                        &stripes[index_of[&ea]],
                        &stripes[index_of[&eb]],
                        &stripes[index_of[&ec]],
                    ],
                    v,
                )?);
            }
            _ => {}
        }
    }
    Ok(Joins {
        miters,
        miter_at,
        corners,
        corner_at,
    })
}

/// Each stripe's ends, then its contacts between them.
fn make_blends(
    ctx: &mut BlendCtx<'_, '_>,
    stripes: Vec<Stripe>,
    joins: &Joins,
) -> Result<Vec<Blend>, OpError> {
    let (env, meter) = ctx.split();
    let Env {
        m, tol, samples, ..
    } = env;
    let Joins {
        miters,
        miter_at,
        corners,
        corner_at,
    } = joins;
    // Each stripe's ends, then its contacts between them.
    let mut blends: Vec<Blend> = Vec::with_capacity(stripes.len());
    for s in stripes {
        let mut ends: Vec<EndKind> = Vec::with_capacity(2);
        for (at_lo, vertex) in [(true, s.start), (false, s.end)] {
            ends.push(match (miter_at.get(&vertex), corner_at.get(&vertex)) {
                (Some(&at), _) => EndKind::Miter {
                    at,
                    side: usize::from(miters[at].edges[0] != s.edge),
                },
                (None, Some(&at)) => EndKind::Corner {
                    at,
                    side: corners[at]
                        .edges
                        .iter()
                        .position(|&e| e == s.edge)
                        .ok_or(invariant("a corner's own blend"))?,
                },
                (None, None) => EndKind::Face(Box::new(face_end(&env, meter, &s, at_lo)?)),
            });
        }
        let ends: [EndKind; 2] = ends
            .try_into()
            .map_err(|_| invariant("two ends of the blend"))?;
        let t = [0, 1].map(|end| [0, 1].map(|k| ends[end].t(miters, corners, k)));
        let contacts = contacts(m, &s, t, tol, samples, meter)?;
        blends.push(Blend {
            stripe: s,
            contacts,
            ends,
        });
    }

    Ok(blends)
}

/// The miters first: their two vertices, their edge, the third edge cut.
fn add_miters(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    miters: &[Miter],
) -> Result<Vec<MiterMade>, OpError> {
    let m = &mut *ctx.m;
    let Draft { rw, cuts, edits } = draft;
    let mut miter_made: Vec<MiterMade> = Vec::with_capacity(miters.len());
    for mt in miters {
        let q = rw.vertices.len();
        rw.vertices.push(VertexSpec::New {
            point: mt.q,
            tolerance: mt.q_tolerance,
        });
        let p3 = rw.vertices.len();
        rw.vertices.push(VertexSpec::New {
            point: mt.p3,
            tolerance: mt.p3_tolerance,
        });
        let (first, second) = if mt.q_first { (q, p3) } else { (p3, q) };
        let edge = rw.edges.len();
        rw.edges.push((
            EdgeSpec::New {
                geometry: EdgeGeometry::Curve {
                    curve: m.add_curve(mt.curve.clone()),
                    range: mt.range,
                },
                start: VertexKey::New(first),
                end: VertexKey::New(second),
                tolerance: mt.tolerance,
            },
            None,
        ));
        // A miter of unequal dihedrals: the trim arc from `m` on to the
        // third edge, which the narrower blend's far face takes at the
        // corner vertex (ADR-0044 §4).
        let trim_arc = match &mt.trim_arc {
            None => None,
            Some(arc) => {
                let end = rw.vertices.len();
                rw.vertices.push(VertexSpec::New {
                    point: arc.end,
                    tolerance: arc.end_tolerance,
                });
                let (first, second) = if arc.m_first { (p3, end) } else { (end, p3) };
                let edge = rw.edges.len();
                rw.edges.push((
                    EdgeSpec::New {
                        geometry: EdgeGeometry::Curve {
                            curve: m.add_curve(arc.curve.clone()),
                            range: arc.range,
                        },
                        start: VertexKey::New(first),
                        end: VertexKey::New(second),
                        tolerance: arc.tolerance,
                    },
                    None,
                ));
                let pcurve = m.add_curve2(arc.on_face.clone());
                edits.entry(arc.face).or_default().insert.insert(
                    arc.vertex,
                    Insertion {
                        arc: EdgeKey::New(edge),
                        pcurve,
                        lo_first: arc.m_first,
                        edge_at_lo: arc.narrow_edge,
                        splits: None,
                    },
                );
                Some((end, edge))
            }
        };
        if let Some(trim) = &mt.trim {
            cut_once(cuts, trim, trim_arc.map_or(p3, |(end, _)| end))?;
        }
        if let Some(trim) = &mt.q_trim {
            cut_once(cuts, trim, q)?;
        }
        miter_made.push(MiterMade {
            vertices: [q, p3],
            edge,
            trim_arc,
        });
    }
    Ok(miter_made)
}

/// The corners next: their three vertices, their three arcs and a sphere's pole; no corner edge is cut.
fn add_corners(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    corners: &[Corner],
) -> Vec<CornerMade> {
    let m = &mut *ctx.m;
    let rw = &mut draft.rw;
    let mut corner_made: Vec<CornerMade> = Vec::with_capacity(corners.len());
    for c in corners {
        let mut vertices = [0usize; 3];
        for (slot, &point) in vertices.iter_mut().zip(&c.points) {
            *slot = rw.vertices.len();
            rw.vertices.push(VertexSpec::New {
                point,
                tolerance: c.tolerance,
            });
        }
        let mut arcs = [0usize; 3];
        for (slot, arc) in arcs.iter_mut().zip(&c.arcs) {
            *slot = rw.edges.len();
            rw.edges.push((
                EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(arc.curve.clone()),
                        range: arc.range,
                    },
                    start: VertexKey::New(vertices[arc.ends[0]]),
                    end: VertexKey::New(vertices[arc.ends[1]]),
                    tolerance: c.tolerance,
                },
                None,
            ));
        }
        let pole = c.pole.as_ref().map(|pole| {
            let at = VertexKey::New(vertices[pole.point]);
            rw.edges.push((
                EdgeSpec::New {
                    geometry: EdgeGeometry::Degenerate { range: pole.range },
                    start: at,
                    end: at,
                    tolerance: c.tolerance,
                },
                None,
            ));
            rw.edges.len() - 1
        });
        corner_made.push(CornerMade {
            vertices,
            arcs,
            pole,
            added: 0,
        });
    }
    corner_made
}

/// The stripes: four trim vertices, two contact edges and each end's arcs.
fn add_stripes(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    blends: &[Blend],
    joins: &Joins,
    drafted: &Drafted,
) -> Result<Vec<StripeMade>, OpError> {
    let m = &mut *ctx.m;
    let Draft { rw, cuts, edits } = draft;
    let Joins {
        miters,
        miter_at: _,
        corners,
        corner_at: _,
    } = joins;
    let (miter_made, corner_made) = (&drafted.miters, &drafted.corners);
    let mut made: Vec<StripeMade> = Vec::with_capacity(blends.len());
    for blend in blends {
        let mut vertices = [[0usize; 2]; 2];
        let mut crossings: Vec<usize> = Vec::new();
        for (end_index, end) in blend.ends.iter().enumerate() {
            match end {
                EndKind::Face(face_end) => {
                    for (k, slot) in vertices[end_index].iter_mut().enumerate() {
                        *slot = rw.vertices.len();
                        rw.vertices.push(VertexSpec::New {
                            point: face_end.points[k],
                            tolerance: vertex_tolerance_of(&blend.ends, k),
                        });
                    }
                }
                EndKind::Miter { at, side } => {
                    let shared = miters[*at].shared[*side];
                    vertices[end_index][shared] = miter_made[*at].vertices[0];
                    vertices[end_index][1 - shared] = miter_made[*at].far(&miters[*at], *side);
                }
                EndKind::Corner { at, side } => {
                    for (k, slot) in vertices[end_index].iter_mut().enumerate() {
                        *slot = corner_made[*at].vertices[corners[*at].contact_point[*side][k]];
                    }
                }
            }
        }
        let mut contact_edges = [0usize; 2];
        for (k, contact) in blend.contacts.iter().enumerate() {
            contact_edges[k] = rw.edges.len();
            rw.edges.push((
                EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(contact.line.clone()),
                        range: contact.range,
                    },
                    start: VertexKey::New(vertices[0][k]),
                    end: VertexKey::New(vertices[1][k]),
                    tolerance: contact.tolerance,
                },
                None,
            ));
        }
        let mut arcs: [Vec<usize>; 2] = Default::default();
        for (end_index, end) in blend.ends.iter().enumerate() {
            let face_end = match end {
                EndKind::Face(face_end) => face_end,
                EndKind::Miter { at, side } => {
                    arcs[end_index] = miter_made[*at].arcs(&miters[*at], *side);
                    continue;
                }
                EndKind::Corner { at, side } => {
                    arcs[end_index].push(corner_made[*at].arcs[*side]);
                    continue;
                }
            };
            // The points of the end in its order: the first trim point, a
            // fan's crossings, the other trim point; each crossing cuts its
            // extra edge (ADR-0043 §5).
            let mut along = vec![vertices[end_index][0]];
            for crossing in &face_end.crossings {
                let v = rw.vertices.len();
                rw.vertices.push(VertexSpec::New {
                    point: crossing.point,
                    tolerance: crossing.tolerance,
                });
                cut_once(cuts, &crossing.trim, v)?;
                crossings.push(v);
                along.push(v);
            }
            along.push(vertices[end_index][1]);
            for (k, trim) in face_end.trims.iter().enumerate() {
                cut_once(cuts, trim, vertices[end_index][k])?;
            }
            for (i, piece) in face_end.pieces.iter().enumerate() {
                let (from, to) = (along[i], along[i + 1]);
                let (first, second) = if piece.arc.lo_first {
                    (from, to)
                } else {
                    (to, from)
                };
                let arc = rw.edges.len();
                arcs[end_index].push(arc);
                rw.edges.push((
                    EdgeSpec::New {
                        geometry: EdgeGeometry::Curve {
                            curve: m.add_curve(piece.arc.curve.clone()),
                            range: piece.arc.range,
                        },
                        start: VertexKey::New(first),
                        end: VertexKey::New(second),
                        tolerance: piece.arc.tolerance,
                    },
                    None,
                ));
                // The edge cut at the arc's start: the first corner edge,
                // or the extra edge before this piece.
                let edge_at_lo = match i.checked_sub(1) {
                    None => face_end.trims[0].edge,
                    Some(j) => face_end.crossings[j].trim.edge,
                };
                let pcurve = m.add_curve2(piece.arc.on_face.clone());
                edits.entry(piece.face).or_default().insert.insert(
                    face_end.vertex,
                    Insertion {
                        arc: EdgeKey::New(arc),
                        pcurve,
                        lo_first: piece.arc.lo_first,
                        edge_at_lo,
                        splits: (!face_end.stays.is_empty())
                            .then_some(face_end.trims.map(|trim| trim.edge)),
                    },
                );
            }
        }
        for (k, contact) in blend.contacts.iter().enumerate() {
            let pcurve = m.add_curve2(contact.on_face.clone());
            edits
                .entry(contact.face)
                .or_default()
                .replace
                .insert(blend.stripe.edge, (EdgeKey::New(contact_edges[k]), pcurve));
        }
        made.push(StripeMade {
            vertices,
            crossings,
            contacts: contact_edges,
            arcs,
            added: 0,
        });
    }
    Ok(made)
}

/// The rings: a closed edge's two vertices, two contact circles, the blend's seam and the cylinder's seam cut; an open arc's four vertices, two contact arcs and two ends, each inserted into its face across with the corner edges cut.
fn add_rings(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    rings: &[Ring],
    joins: &Joins,
    drafted: &Drafted,
) -> Result<Vec<Made>, OpError> {
    let m = &mut *ctx.m;
    let Draft { rw, cuts, edits } = draft;
    let Joins {
        miters, miter_at, ..
    } = joins;
    let miter_made = &drafted.miters;
    let mut ring_made: Vec<Made> = Vec::with_capacity(rings.len());
    for r in rings {
        let mut vertices = [[0usize; 2]; 2];
        match &r.ends {
            RingEnds::Seam(seam) => {
                for (slot, contact) in vertices[0].iter_mut().zip(&r.contacts) {
                    *slot = rw.vertices.len();
                    rw.vertices.push(VertexSpec::New {
                        point: contact.points[0],
                        tolerance: seam.vertex_tolerance,
                    });
                }
                vertices[1] = vertices[0];
            }
            RingEnds::Open(ends) => {
                for (j, end) in ends.iter().enumerate() {
                    match end {
                        RingEnd::Face(end) => {
                            for (c, contact) in r.contacts.iter().enumerate() {
                                vertices[j][c] = rw.vertices.len();
                                rw.vertices.push(VertexSpec::New {
                                    point: contact.points[j],
                                    tolerance: end.vertex_tolerance[c],
                                });
                            }
                        }
                        RingEnd::Junction(vertex) => {
                            let at = *miter_at
                                .get(vertex)
                                .ok_or(invariant("a junction at the ring's end"))?;
                            let side = usize::from(miters[at].edges[0] != r.edge);
                            let shared = miters[at].shared[side];
                            vertices[j][shared] = miter_made[at].vertices[0];
                            vertices[j][1 - shared] = miter_made[at].vertices[1];
                        }
                    }
                }
            }
        }
        let mut contact_edges = [0usize; 2];
        for (c, contact) in r.contacts.iter().enumerate() {
            contact_edges[c] = rw.edges.len();
            rw.edges.push((
                EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(contact.curve.clone()),
                        range: contact.range,
                    },
                    start: VertexKey::New(vertices[0][c]),
                    end: VertexKey::New(vertices[1][c]),
                    tolerance: r.tolerance,
                },
                None,
            ));
            let pcurve = m.add_curve2(contact.on_face.clone());
            edits
                .entry(contact.face)
                .or_default()
                .replace
                .insert(r.edge, (EdgeKey::New(contact_edges[c]), pcurve));
        }
        let mut arcs = [0usize; 2];
        match &r.ends {
            RingEnds::Seam(seam) => {
                arcs = [rw.edges.len(); 2];
                rw.edges.push((
                    EdgeSpec::New {
                        geometry: EdgeGeometry::Curve {
                            curve: m.add_curve(seam.curve.clone()),
                            range: seam.range,
                        },
                        start: VertexKey::New(vertices[0][0]),
                        end: VertexKey::New(vertices[0][1]),
                        tolerance: r.tolerance,
                    },
                    None,
                ));
                for (trim, by) in &seam.cuts {
                    cut_once(cuts, trim, vertices[0][*by])?;
                }
            }
            RingEnds::Open(ends) => {
                for (j, end) in ends.iter().enumerate() {
                    let end = match end {
                        RingEnd::Face(end) => end,
                        RingEnd::Junction(vertex) => {
                            arcs[j] = miter_made[miter_at[vertex]].edge;
                            continue;
                        }
                    };
                    let (first, second) = if end.lo_first { (0, 1) } else { (1, 0) };
                    arcs[j] = rw.edges.len();
                    rw.edges.push((
                        EdgeSpec::New {
                            geometry: EdgeGeometry::Curve {
                                curve: m.add_curve(end.curve.clone()),
                                range: end.range,
                            },
                            start: VertexKey::New(vertices[j][first]),
                            end: VertexKey::New(vertices[j][second]),
                            tolerance: end.tolerance,
                        },
                        None,
                    ));
                    for (c, trim) in end.trims.iter().enumerate() {
                        cut_once(cuts, trim, vertices[j][c])?;
                    }
                    let pcurve = m.add_curve2(end.on_face.clone());
                    edits.entry(end.face).or_default().insert.insert(
                        end.vertex,
                        Insertion {
                            arc: EdgeKey::New(arcs[j]),
                            pcurve,
                            lo_first: end.lo_first,
                            edge_at_lo: end.trims[0].edge,
                            splits: None,
                        },
                    );
                }
            }
        }
        ring_made.push(Made {
            vertices,
            contacts: contact_edges,
            arcs,
            added: 0,
        });
    }
    Ok(ring_made)
}

/// The corner edges shortened or lengthened, in id order; a lengthened edge's pcurves derived again over its new range (ADR-0038 §4), by the edge and the pcurve each replaces.
fn shorten_edges(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
) -> Result<(Shortened, Rederived), OpError> {
    let (m, view, _, tol, _, meter) = ctx.parts();
    let Draft { rw, cuts, .. } = draft;
    let mut shortened: BTreeMap<EdgeId, EdgeKey> = BTreeMap::new();
    let mut rederived: BTreeMap<(EdgeId, Curve2Id), Curve2Id> = BTreeMap::new();
    for (&edge, cut) in &*cuts {
        let entity = *m.edge(edge)?;
        let Some((curve, old)) = entity.curve() else {
            return Err(invariant("a corner edge's curve"));
        };
        let lo = cut.lo.map_or(old.lo(), |(_, t)| t);
        let hi = cut.hi.map_or(old.hi(), |(_, t)| t);
        let range = Interval::new(lo, hi)
            .map_err(|_| degenerate(vec![forward(edge)], Reason::Blend(BlendReason::TooLarge)))?;
        // Placed where the old pcurve still runs: at the end lengthened.
        let anchor = if lo < old.lo() {
            Some(old.lo())
        } else if hi > old.hi() {
            Some(old.hi())
        } else {
            None
        };
        if let Some(anchor) = anchor {
            let edge_tol = Tolerance::new(entity.tolerance(), tol.angular);
            for u in view.uses.get(&edge).into_iter().flatten() {
                let surface = m.surface(m.face(u.face)?.surface())?;
                let fresh = pcurve_on(m.curve(curve)?, range, surface, edge_tol, meter)
                    .map_err(fault_of)?;
                let fresh = placed_uv(fresh, anchor, m.curve2(u.pcurve)?.point(anchor));
                let id = m.add_curve2(fresh);
                rederived.insert((edge, u.pcurve), id);
            }
        }
        shortened.insert(edge, EdgeKey::New(rw.edges.len()));
        rw.edges.push((
            EdgeSpec::New {
                geometry: EdgeGeometry::Curve { curve, range },
                start: cut
                    .lo
                    .map_or(VertexKey::Kept(entity.start()), |(v, _)| VertexKey::New(v)),
                end: cut
                    .hi
                    .map_or(VertexKey::Kept(entity.end()), |(v, _)| VertexKey::New(v)),
                tolerance: entity.tolerance(),
            },
            Some(edge),
        ));
    }
    Ok((shortened, rederived))
}

/// Every touched face's loops rewritten, in the body's face order.
fn rewrite_faces(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    shortened: &Shortened,
    rederived: &Rederived,
) -> Result<(), OpError> {
    let (m, view, ..) = ctx.parts();
    let Draft { rw, edits, .. } = draft;
    for &face in &view.faces {
        let entity = m.face(face)?;
        let edit = edits.get(&face);
        let touched = edit.is_some()
            || entity
                .loops()
                .iter()
                .flat_map(|l| l.coedges())
                .any(|c| shortened.contains_key(&c.edge()));
        if !touched {
            continue;
        }
        let mut loops: Vec<Vec<StoredUse>> = Vec::with_capacity(entity.loops().len());
        for l in entity.loops() {
            let mut uses: Vec<StoredUse> = Vec::with_capacity(l.coedges().len() + 1);
            // Where each coedge's use sits in `uses`.
            let mut at: Vec<usize> = Vec::with_capacity(l.coedges().len());
            for c in l.coedges() {
                let id = c.edge();
                let (edge, pcurve) = match edit.and_then(|e| e.replace.get(&id)) {
                    Some(&(key, pcurve)) => (key, pcurve),
                    None => (
                        shortened.get(&id).copied().unwrap_or(EdgeKey::Kept(id)),
                        (rederived.get(&(id, c.pcurve())).copied()).unwrap_or(c.pcurve()),
                    ),
                };
                at.push(uses.len());
                uses.push(StoredUse {
                    edge,
                    orientation: c.orientation(),
                    pcurve,
                });
                let ce = m.edge(id)?;
                let junction = if c.orientation() == Orientation::Forward {
                    ce.end()
                } else {
                    ce.start()
                };
                if let Some(ins) = edit
                    .and_then(|e| e.insert.get(&junction))
                    .filter(|ins| ins.splits.is_none())
                {
                    // The arc runs from the end of the corner edge just
                    // walked to the start of the next.
                    let arrived_at_lo = id == ins.edge_at_lo;
                    let orientation = if ins.lo_first == arrived_at_lo {
                        Orientation::Forward
                    } else {
                        Orientation::Reversed
                    };
                    uses.push(StoredUse {
                        edge: ins.arc,
                        orientation,
                        pcurve: ins.pcurve,
                    });
                }
            }
            // The face across met twice: the stretch of the loop from the
            // corner edge leaving the surviving vertex to the one arriving
            // at it, closed by the arc, is a loop of its own; the rest
            // keeps the vertex with the edges that stay (ADR-0043 §4).
            let mut inner: Option<Vec<StoredUse>> = None;
            for (&vertex, ins) in edit.iter().flat_map(|e| &e.insert) {
                let Some(corners) = ins.splits else { continue };
                let ends = |i: usize| -> Result<(VertexId, VertexId), OpError> {
                    let c = l.coedges()[i];
                    let ce = m.edge(c.edge())?;
                    Ok(if c.orientation() == Orientation::Forward {
                        (ce.start(), ce.end())
                    } else {
                        (ce.end(), ce.start())
                    })
                };
                let (mut leaves, mut arrives) = (None, None);
                for i in 0..l.coedges().len() {
                    let (from, to) = ends(i)?;
                    if !corners.contains(&l.coedges()[i].edge()) {
                        continue;
                    }
                    if from == vertex {
                        leaves = Some(i);
                    }
                    if to == vertex {
                        arrives = Some(i);
                    }
                }
                let (Some(leaves), Some(arrives)) = (leaves, arrives) else {
                    continue;
                };
                let (first, last) = (at[leaves], at[arrives]);
                let (mut stretch, rest): (Vec<StoredUse>, Vec<StoredUse>) = if first <= last {
                    let stretch = uses[first..=last].to_vec();
                    let rest = [&uses[..first], &uses[last + 1..]].concat();
                    (stretch, rest)
                } else {
                    let stretch = [&uses[first..], &uses[..=last]].concat();
                    (stretch, uses[last + 1..first].to_vec())
                };
                let arrived_at_lo = l.coedges()[arrives].edge() == ins.edge_at_lo;
                stretch.push(StoredUse {
                    edge: ins.arc,
                    orientation: if ins.lo_first == arrived_at_lo {
                        Orientation::Forward
                    } else {
                        Orientation::Reversed
                    },
                    pcurve: ins.pcurve,
                });
                uses = rest;
                inner = Some(stretch);
            }
            loops.push(uses);
            loops.extend(inner);
        }
        rw.faces.insert(face, loops);
    }
    Ok(())
}

/// The blend faces, one per edge, after the shell's own: each end's arc walked from one contact to the other, the miter's with its pcurve on this blend's cylinder.
fn add_blend_faces(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    blends: &[Blend],
    joins: &Joins,
    made: &mut Drafted,
) -> Result<(), OpError> {
    let (m, view, ..) = ctx.parts();
    let rw = &mut draft.rw;
    let Joins {
        miters, corners, ..
    } = joins;
    let made = &mut made.stripes;
    for (blend, made) in blends.iter().zip(made.iter_mut()) {
        let contact_edges = made.contacts;
        let [lo, hi] = &blend.contacts;
        let surface = m.add_surface(blend.stripe.surface.clone());
        let mut end_uses: [Vec<StoredUse>; 2] = Default::default();
        for (end_index, end) in blend.ends.iter().enumerate() {
            let arcs: Vec<(bool, &Curve2)> = match end {
                EndKind::Face(face_end) => face_end
                    .pieces
                    .iter()
                    .map(|p| (p.arc.lo_first, &p.arc.on_blend))
                    .collect(),
                EndKind::Miter { at, side } => miters[*at].arcs(*side),
                EndKind::Corner { at, side } => {
                    let arc = &corners[*at].arcs[*side];
                    vec![(arc.lo_first, &arc.on_blend)]
                }
            };
            if arcs.len() != made.arcs[end_index].len() {
                return Err(invariant("an edge per arc of the blend's end"));
            }
            // The start's arcs are walked from `u = 0` to `u = β`, the
            // end's back, a fan's pieces in reverse.
            let uses = &mut end_uses[end_index];
            for (&(lo_first, pcurve), &edge) in arcs.iter().zip(&made.arcs[end_index]) {
                let along = lo_first == (end_index == 0);
                uses.push(StoredUse {
                    edge: EdgeKey::New(edge),
                    orientation: if along {
                        Orientation::Forward
                    } else {
                        Orientation::Reversed
                    },
                    pcurve: m.add_curve2(pcurve.clone()),
                });
            }
            if end_index == 1 {
                uses.reverse();
            }
        }
        let [start_arcs, end_arcs] = end_uses;
        let mut loop_uses = start_arcs;
        loop_uses.push(StoredUse {
            edge: EdgeKey::New(contact_edges[1]),
            orientation: Orientation::Forward,
            pcurve: m.add_curve2(hi.on_blend.clone()),
        });
        loop_uses.extend(end_arcs);
        loop_uses.push(StoredUse {
            edge: EdgeKey::New(contact_edges[0]),
            orientation: Orientation::Reversed,
            pcurve: m.add_curve2(lo.on_blend.clone()),
        });
        made.added = rw.added.len();
        rw.added.push(AddedFace {
            shell: view.shell_of[&lo.face],
            surface,
            orientation: if blend.stripe.convex {
                Orientation::Forward
            } else {
                Orientation::Reversed
            },
            loops: vec![loop_uses],
            tolerance: blend.stripe.tolerance,
        });
    }
    Ok(())
}

/// The corner faces, one per corner, their sides in walking order with a sphere's pole crossed where its meridians meet.
fn add_corner_faces(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    joins: &Joins,
    made: &mut Drafted,
) {
    let (m, view, ..) = ctx.parts();
    let rw = &mut draft.rw;
    let corners = &joins.corners;
    let corner_made = &mut made.corners;
    for (c, made) in corners.iter().zip(corner_made.iter_mut()) {
        let surface = m.add_surface(c.surface.clone());
        let mut loop_uses: Vec<StoredUse> = Vec::with_capacity(4);
        for (w, &side) in c.walk.iter().enumerate() {
            let arc = &c.arcs[side];
            loop_uses.push(StoredUse {
                edge: EdgeKey::New(made.arcs[side]),
                orientation: if arc.along {
                    Orientation::Forward
                } else {
                    Orientation::Reversed
                },
                pcurve: m.add_curve2(arc.on_corner.clone()),
            });
            if let (Some(pole), Some(edge)) = (&c.pole, made.pole)
                && pole.after == w
            {
                loop_uses.push(StoredUse {
                    edge: EdgeKey::New(edge),
                    orientation: Orientation::Reversed,
                    pcurve: m.add_curve2(pole.pcurve.clone()),
                });
            }
        }
        made.added = rw.added.len();
        rw.added.push(AddedFace {
            shell: view.shell_of[&c.faces[0]],
            surface,
            orientation: c.orientation,
            loops: vec![loop_uses],
            tolerance: c.tolerance,
        });
    }
}

/// The ring faces, one per circular edge, in (u, v) counter-clockwise: the lower contact along `u`, up at its far `u` — the seam at `u = 2π` or the end there — the upper contact back, down at its near `u`.
fn add_ring_faces(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    rings: &[Ring],
    joins: &Joins,
    made: &mut Drafted,
) {
    let (m, view, ..) = ctx.parts();
    let rw = &mut draft.rw;
    let Joins {
        miters, miter_at, ..
    } = joins;
    let ring_made = &mut made.rings;
    for (r, made) in rings.iter().zip(ring_made.iter_mut()) {
        let [lower, upper] = &r.contacts;
        let surface = m.add_surface(r.surface.clone());
        let along = |yes: bool| {
            if yes {
                Orientation::Forward
            } else {
                Orientation::Reversed
            }
        };
        // Each end's edge, its pcurve, and whether it runs up from the
        // lower contact: a seam's and a face end's do, a junction's when
        // its `q` is on the lower contact.
        let (far, near) = match &r.ends {
            RingEnds::Seam(seam) => (
                (made.arcs[0], seam.on_blend[1].clone(), true),
                (made.arcs[0], seam.on_blend[0].clone(), true),
            ),
            RingEnds::Open(ends) => {
                // The edge's end is at the far `u` when the contacts run
                // with `u`, its start otherwise.
                let (hi, lo) = if lower.along_u { (1, 0) } else { (0, 1) };
                let end = |j: usize| match &ends[j] {
                    RingEnd::Face(end) => (made.arcs[j], end.on_blend.clone(), end.lo_first),
                    RingEnd::Junction(vertex) => {
                        let junction = &miters[miter_at[vertex]];
                        let side = usize::from(junction.edges[0] != r.edge);
                        (
                            made.arcs[j],
                            junction.on_blend[side].clone(),
                            junction.shared[side] == 0,
                        )
                    }
                };
                (end(hi), end(lo))
            }
        };
        let loop_uses = vec![
            StoredUse {
                edge: EdgeKey::New(made.contacts[0]),
                orientation: along(lower.along_u),
                pcurve: m.add_curve2(lower.on_blend.clone()),
            },
            StoredUse {
                edge: EdgeKey::New(far.0),
                orientation: along(far.2),
                pcurve: m.add_curve2(far.1),
            },
            StoredUse {
                edge: EdgeKey::New(made.contacts[1]),
                orientation: along(!upper.along_u),
                pcurve: m.add_curve2(upper.on_blend.clone()),
            },
            StoredUse {
                edge: EdgeKey::New(near.0),
                orientation: along(!near.2),
                pcurve: m.add_curve2(near.1),
            },
        ];
        made.added = rw.added.len();
        rw.added.push(AddedFace {
            shell: view.shell_of[&lower.face],
            surface,
            orientation: r.orientation,
            loops: vec![loop_uses],
            tolerance: r.tolerance,
        });
    }
}

/// The provenance of the rewrite: every record against the blended edge.
fn record(
    out: rebuild::Rewritten,
    blends: &[Blend],
    rings: &[Ring],
    joins: &Joins,
    drafted: &Drafted,
) -> (Body, Provenance) {
    let Joins { miters, .. } = joins;
    let corners = &joins.corners;
    let (made, miter_made, corner_made, ring_made) = (
        &drafted.stripes,
        &drafted.miters,
        &drafted.corners,
        &drafted.rings,
    );
    let mut p = out.provenance;
    // Every record against the blended edge; a miter's edge and vertices
    // are generated from both edges it joins, so each records them.
    for (blend, made) in blends.iter().zip(made) {
        let origin = forward(blend.stripe.edge);
        p.add_generated(origin, forward(out.added[made.added]));
        for &k in made.contacts.iter().chain(made.arcs.iter().flatten()) {
            p.add_generated(origin, forward(out.edges[k]));
        }
        for &v in made.vertices.iter().flatten().chain(&made.crossings) {
            p.add_generated(origin, forward(out.vertices[v]));
        }
    }
    // A trim arc's `m`, the narrower blend's vertex, from the wider
    // blend's edge too (ADR-0044 §5).
    for (mt, made) in miters.iter().zip(miter_made) {
        if let Some(arc) = &mt.trim_arc {
            p.add_generated(
                forward(mt.edges[arc.wide]),
                forward(out.vertices[made.vertices[1]]),
            );
        }
    }
    // A corner's face and a sphere's pole from each of its three edges;
    // its vertices and sides are its blends' own, recorded above.
    for (c, made) in corners.iter().zip(corner_made) {
        for &edge in &c.edges {
            let origin = forward(edge);
            p.add_generated(origin, forward(out.added[made.added]));
            if let Some(pole) = made.pole {
                p.add_generated(origin, forward(out.edges[pole]));
            }
        }
    }
    // A ring's face, its two contacts, its seam or its two ends, and its
    // two or four vertices.
    for (r, made) in rings.iter().zip(ring_made) {
        let origin = forward(r.edge);
        let ends = match r.ends {
            RingEnds::Seam(_) => 1,
            RingEnds::Open(_) => 2,
        };
        p.add_generated(origin, forward(out.added[made.added]));
        for &k in made.contacts.iter().chain(&made.arcs[..ends]) {
            p.add_generated(origin, forward(out.edges[k]));
        }
        for &v in made.vertices[..ends].iter().flatten() {
            p.add_generated(origin, forward(out.vertices[v]));
        }
    }
    (out.body, p)
}
