//! Assembly: the insertions, face edits and cuts a set of stripes makes, rewritten through `rebuild::rewrite`.

use std::collections::{BTreeMap, BTreeSet};

use arris_check::domain::bands;
use arris_geom::{Curve, Curve2, pcurve_ending_on, pcurve_on};
use arris_math::{Interval, Meter, Point2, RELATIVE_ROUNDING, Tolerance};
use arris_topo::builder::{EdgeKey, EdgeSpec, VertexKey, VertexSpec};
use arris_topo::entity::EdgeGeometry;
use arris_topo::{
    Body, Curve2Id, CurveId, EdgeId, FaceId, Model, Orientation, Provenance, VertexId,
};

use super::chain::{chain, cusp_at, kinked_vertex, tangent_vertex};
use super::corner::{Corner, corner};
use super::ends::{EndKind, Pole, Trim, face_end};
use super::junction::{Run, junction, pin_kinks};
use super::miter::{Miter, miter};
use super::mutual::{Cusp, CuspJoin, cusp_join};
use super::ring::{
    CuspEnd, Ring, RingEnd, RingEnds, contact_on, in_row, placed_uv, ring, vertex_tolerance_of,
};
use super::ring_rolled::add_ring_rolled_faces;
use super::rolled::{Rolled, RolledMade, add_crowded_edge, add_rolled_faces, add_run_over, rolled};
use super::stripe::Section;
use super::stripe::{Stripe, contacts, stripe};
use super::walked_blend::{
    walked_contacts, walked_cut, walked_end, walked_pole, walked_ring, walked_stripe,
};
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
    /// Where the face is the first piece of a flush fan (ADR-0082 §2): the
    /// corner edge's kept part, which runs on from the arc to the vertex.
    pub(super) tail: Option<Tail>,
}

/// The part of a flush fan's second corner edge from its trim point to the
/// vertex, which stays as an edge between the first and the last piece.
#[derive(Debug, Clone, Copy)]
pub(super) struct Tail {
    pub(super) edge: EdgeKey,
    pub(super) pcurve: Curve2Id,
    /// `true` when the edge runs from the trim point to the vertex.
    pub(super) to_vertex: bool,
}

/// What a face's loops are rewritten with: a blended edge replaced by
/// the contact on this face, and an arc inserted at a consumed vertex.
#[derive(Default)]
pub(super) struct FaceEdit {
    pub(super) replace: BTreeMap<EdgeId, (EdgeKey, Curve2Id)>,
    pub(super) insert: BTreeMap<VertexId, Insertion>,
    /// The edges a run-over cuts into pieces on this face (ADR-0071 §2),
    /// each piece in the edge's own direction and a gap where the face
    /// no longer runs along it: a contact's stretch off `F`, `E`'s stretch
    /// the ball rolls on. On `F` the loop with `E`'s gap is joined into the
    /// contact's at its gap.
    pub(super) split: BTreeMap<EdgeId, Vec<SplitPiece>>,
}

/// A piece of an edge a run-over cuts, on one face.
#[derive(Clone, Copy)]
pub(super) enum SplitPiece {
    /// The face runs along this edge.
    Use(EdgeKey, Curve2Id),
    /// The face no longer runs along it here: the gap of the run-over of
    /// this blended edge, which joins the face's loops.
    Gap(EdgeId),
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
    /// A closed edge's run-over entities, its contacts' pieces among them:
    /// `contacts` is then the pieces before the first crossing and `arcs`
    /// is unused, the torus having no seam edge (ADR-0071).
    pub(super) rolled: Option<RolledMade>,
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
    /// A run-over's entities, its contacts' pieces among them: `contacts`
    /// is then the pieces before the first crossing.
    pub(super) rolled: Option<RolledMade>,
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
    /// The kinks the chains run through (ADR-0075), each a junction.
    kinks: BTreeSet<VertexId>,
}

/// How the blended edges meet at their vertices, before the joins are
/// built: the edges at each vertex, the vertices that are junctions and
/// the cusps both of whose edges are blended.
struct Meets {
    at_vertex: AtVertex,
    junctions: BTreeSet<VertexId>,
    cusps: Vec<CuspJoin>,
}

/// The vertex joins of the blended edges, by vertex.
struct Joins {
    miters: Vec<Miter>,
    miter_at: BTreeMap<VertexId, usize>,
    /// The miters that are cusps' mutual trims (ADR-0084 §4′), whose
    /// edge and vertices each edge records after its own.
    cusps: BTreeSet<usize>,
    corners: Vec<Corner>,
    corner_at: BTreeMap<VertexId, usize>,
}

/// The rewrite being filled in: its entities, the cuts to corner edges and
/// the edits to faces.
struct Draft {
    rw: Rewrite,
    cuts: BTreeMap<EdgeId, Cuts>,
    edits: BTreeMap<FaceId, FaceEdit>,
    /// The edges two or more run-overs cross, cut once among them
    /// (`rolled::add_crowded_edge`).
    crowded: BTreeSet<EdgeId>,
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
        kinks,
    } = classify(&mut ctx, body, edges)?;
    let meets = vertex_joins(&mut ctx, &open, &arcs, &mut rings, &kinks)?;
    let stripes = make_stripes(&mut ctx, &open)?;
    let mut joins = join_blends(&mut ctx, &open, &stripes, &rings, &meets, &kinks)?;
    let mut blends = make_blends(&mut ctx, stripes, &joins)?;
    pin_kinks(ctx.m, &mut joins.miters, &mut blends)?;
    let mut draft = Draft {
        rw: Rewrite::default(),
        cuts: BTreeMap::new(),
        edits: BTreeMap::new(),
        crowded: crowded_edges(&blends, &rings),
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
    add_crowded_edges(&mut ctx, &mut draft, &blends, &rings, &mut made)?;
    let (shortened, rederived) = shorten_edges(&mut ctx, &mut draft)?;
    rewrite_faces(&mut ctx, &mut draft, &shortened, &rederived)?;
    add_blend_faces(&mut ctx, &mut draft, &blends, &joins, &mut made)?;
    add_corner_faces(&mut ctx, &mut draft, &joins, &mut made);
    add_ring_faces(&mut ctx, &mut draft, &rings, &joins, &mut made)?;
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
    let mut reached = chain(m, view, edges, kind.size(), tol, meter)?;
    let kinks = through_kinks(&env, meter, &mut reached)?;
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
        // A fillet whose faces or curve the closed forms do not hold is
        // walked (ADR-0062).
        let round = match entity.curve() {
            Some((curve, _)) => {
                entity.start() == entity.end() || matches!(m.curve(curve)?, Curve::Circle { .. })
            }
            None => false,
        };
        let walk = round && !in_row(&env, e)?;
        match entity.curve() {
            Some(_) if entity.start() == entity.end() => {
                rings.push(if walk {
                    walked_ring(&env, meter, e)?
                } else {
                    ring(&env, meter, e, &no_junctions, &BTreeMap::new())?
                });
            }
            Some((curve, _)) if !walk && matches!(m.curve(curve)?, Curve::Circle { .. }) => {
                arcs.push(e);
            }
            Some(_) | None => open.push(e),
        }
    }
    Ok(Classified {
        open,
        arcs,
        rings,
        kinks,
    })
}

/// `reached` run on through every kink a chain end's vertex writes as
/// tangent within its tolerance where the end across it meets nothing
/// (ADR-0075), each continued to where it next ends; the kinks, each a
/// junction of the build's. A kink's end is made as the build would make
/// it, and the chain runs through only where that end is the
/// [`near_tangent_corner`](super::ends::near_tangent_corner) refusal naming
/// the same corner edge: a contact that misses the face across. Errors: the meter's, and a fault of the model's.
fn through_kinks(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    reached: &mut BTreeSet<EdgeId>,
) -> Result<BTreeSet<VertexId>, OpError> {
    let Env {
        m, view, kind, tol, ..
    } = *env;
    let mut kinks: BTreeSet<VertexId> = BTreeSet::new();
    let mut todo: Vec<EdgeId> = reached.iter().copied().collect();
    while let Some(edge) = todo.pop() {
        meter.tick()?;
        let entity = *m.edge(edge)?;
        if entity.start() == entity.end() {
            continue;
        }
        for vertex in [entity.start(), entity.end()] {
            if kinks.contains(&vertex) {
                continue;
            }
            let Some((next, w)) = kinked_vertex(m, view, edge, vertex, kind.size(), tol)? else {
                continue;
            };
            if reached.contains(&next) || !end_meets_nothing(env, meter, edge, vertex, w)? {
                continue;
            }
            kinks.insert(vertex);
            for e in chain(m, view, &[next], kind.size(), tol, meter)? {
                if reached.insert(e) {
                    todo.push(e);
                }
            }
        }
    }
    Ok(kinks)
}

/// Whether the end of `edge`'s blend at `vertex` meets nothing across
/// it, beside the corner edge `w`: its end, made as the build makes it,
/// is `TangentChain` naming the edge, `w` and the vertex. Any other end,
/// built or refused, is the build's to make. Errors: the meter's, and a
/// fault of the model's.
fn end_meets_nothing(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    edge: EdgeId,
    vertex: VertexId,
    w: EdgeId,
) -> Result<bool, OpError> {
    let Env {
        m, view, kind, tol, ..
    } = *env;
    let made = match stripe(m, view, edge, kind, tol) {
        Err(OpError::Unsupported { .. }) => walked_stripe(env, meter, edge),
        other => other,
    };
    let s = match made {
        Ok(s) => s,
        Err(e @ (OpError::Interrupted(_) | OpError::Internal(_))) => return Err(e),
        Err(_) => return Ok(false),
    };
    let at_lo = s.start == vertex;
    if matches!(s.section, Section::Walked { poles, .. } if poles[usize::from(!at_lo)]) {
        return Ok(false);
    }
    let end = match s.section {
        Section::Walked { .. } => walked_end(env, meter, &s, at_lo),
        Section::Round { .. } | Section::Flat => face_end(env, meter, &s, at_lo),
    };
    match end {
        Err(OpError::Degenerate {
            entities,
            reason: Reason::Blend(BlendReason::TangentChain),
        }) => Ok(entities == [forward(edge), forward(w), forward(vertex)]),
        Err(e @ (OpError::Interrupted(_) | OpError::Internal(_))) => Err(e),
        Ok(_) | Err(_) => Ok(false),
    }
}

/// The blended edges at each vertex, the vertices that are junctions and
/// the cusps both of whose edges are blended; the arcs' rings are built
/// here.
fn vertex_joins(
    ctx: &mut BlendCtx<'_, '_>,
    open: &[EdgeId],
    arcs: &[EdgeId],
    rings: &mut Vec<Ring>,
    kinks: &BTreeSet<VertexId>,
) -> Result<Meets, OpError> {
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
    let mut cusps: Vec<CuspJoin> = Vec::new();
    for (&v, es) in &at_vertex {
        // A kink the chain runs through is a junction too (ADR-0075 §2).
        if let [ea, eb] = es[..]
            && (kinks.contains(&v) || tangent_vertex(m, view, ea, v, kind.size(), tol)? == Some(eb))
        {
            junctions.insert(v);
            continue;
        }
        // Both edges of a cusp blended: two arcs of one sense, filleted or
        // chamfered under a plane, are the stripes' mutual trim (ADR-0084 §4′); any
        // other such cusp keeps ADR-0042 §6's refusal.
        if let [ea, eb] = es[..]
            && let Some((next, spine)) = cusp_at(m, view, ea, v, kind.size(), tol)?
            && next == eb
        {
            match cusp_of(&env, [ea, eb], spine, v, (open, arcs))? {
                Some(join) => {
                    cusps.push(join);
                    continue;
                }
                None => {
                    return Err(degenerate(
                        vec![forward(ea), forward(spine), forward(v)],
                        Reason::Blend(BlendReason::TangentChain),
                    ));
                }
            }
        }
        if es.len() > 3 || (es.len() > 1 && es.iter().any(|e| arcs.contains(e))) {
            let entities = es.iter().map(|&e| forward(e)).chain([forward(v)]).collect();
            return Err(degenerate(
                entities,
                Reason::Blend(BlendReason::VertexBlend),
            ));
        }
    }
    let mut ends: BTreeMap<VertexId, CuspEnd> = BTreeMap::new();
    for join in &cusps {
        ends.insert(join.vertex, join.end(&env)?);
    }
    for &e in arcs {
        rings.push(ring(&env, meter, e, &junctions, &ends)?);
    }
    at_vertex.retain(|v, es| junctions.contains(v) || !es.iter().any(|e| arcs.contains(e)));
    Ok(Meets {
        at_vertex,
        junctions,
        cusps,
    })
}

/// The cusp at `vertex` of the blended `edges` and the `spine`, read for
/// the mutual trim (ADR-0084 §1, §4′): the edges blended, of one sense,
/// sharing one face `T`, a plane, and each with a wall square to it, and
/// each an arc among `arcs` (a circle, its wall a cylinder about an axis
/// normal to `T`) or a line among `open` (a closed-form stripe, its
/// wall a plane normal to `T`), not both lines. `None` for any other such
/// cusp, which keeps its refusal.
fn cusp_of(
    env: &Env<'_>,
    edges: [EdgeId; 2],
    spine: EdgeId,
    vertex: VertexId,
    (open, arcs): (&[EdgeId], &[EdgeId]),
) -> Result<Option<CuspJoin>, OpError> {
    let (m, view) = (env.m, env.view);
    let size = env.kind.size();
    let is_arc = edges.map(|e| arcs.contains(&e));
    if is_arc == [false, false] || !edges.iter().all(|e| arcs.contains(e) || open.contains(e)) {
        return Ok(None);
    }
    let [Some(convex), Some(other)] = [view.convex(m, edges[0])?, view.convex(m, edges[1])?] else {
        return Ok(None);
    };
    if convex != other {
        return Ok(None);
    }
    let faces_of = |e: EdgeId| -> BTreeSet<FaceId> {
        (view.uses.get(&e).into_iter().flatten())
            .map(|u| u.face)
            .collect()
    };
    let shared: Vec<FaceId> = faces_of(edges[0])
        .intersection(&faces_of(edges[1]))
        .copied()
        .collect();
    let [top] = shared[..] else {
        return Ok(None);
    };
    let &arris_geom::Surface::Plane { frame } = m.surface(m.face(top)?.surface())? else {
        return Ok(None);
    };
    let normal = frame.z().into_inner();
    let mut tops: Vec<Curve> = Vec::with_capacity(2);
    for (e, is_arc) in edges.into_iter().zip(is_arc) {
        for wall in faces_of(e).into_iter().filter(|&f| f != top) {
            let square = match m.surface(m.face(wall)?.surface())? {
                arris_geom::Surface::Cylinder { frame: axis, .. } if is_arc => {
                    axis.z().cross(&normal).norm() <= env.tol.angular
                }
                arris_geom::Surface::Plane { frame: wall } if !is_arc => {
                    wall.z().dot(&normal).abs() <= env.tol.angular
                }
                _ => false,
            };
            if !square {
                return Ok(None);
            }
        }
        // The edge's contact with `T`, before any end trims it.
        if is_arc {
            tops.push(contact_on(env, e, top)?);
        } else {
            let Ok(s) = stripe(m, view, e, env.kind, env.tol) else {
                return Ok(None);
            };
            let (Section::Round { .. } | Section::Flat, Some(k)) =
                (s.section, s.faces.iter().position(|&f| f == top))
            else {
                return Ok(None);
            };
            tops.push(s.lines[k].clone());
        }
    }
    let Ok(tops) = <[Curve; 2]>::try_from(tops) else {
        return Err(invariant("a cusp join's two contacts"));
    };
    let point = m.vertex(vertex)?.point();
    let uv = frame.to_local(point);
    let outward = view.outward(m, top, Point2::new(uv.x, uv.y))?;
    let inward = if convex { -outward } else { outward };
    let Some(inward) = arris_math::UnitVec3::try_new(inward, env.tol.linear) else {
        return Err(invariant("a plane's unit normal"));
    };
    let (curve, range) = m
        .edge(edges[0])?
        .curve()
        .ok_or(invariant("a cusp edge's curve"))?;
    Ok(Some(CuspJoin {
        vertex,
        edges,
        spine,
        top,
        tops,
        cusp: Cusp {
            top: frame,
            vertex: point,
            side: m.curve(curve)?.point(range.midpoint()),
            inward,
            size,
        },
    }))
}

/// One stripe per open edge.
fn make_stripes(ctx: &mut BlendCtx<'_, '_>, open: &[EdgeId]) -> Result<Vec<Stripe>, OpError> {
    let (env, meter) = ctx.split();
    let Env {
        m, view, kind, tol, ..
    } = env;
    let edges = open;
    let mut stripes: Vec<Stripe> = Vec::with_capacity(edges.len());
    for &e in edges {
        meter.tick()?;
        stripes.push(match stripe(m, view, e, kind, tol) {
            // The closed forms hold neither faces nor curve: a fillet is
            // walked (ADR-0062).
            Err(OpError::Unsupported { .. }) => walked_stripe(&env, meter, e)?,
            other => other?,
        });
    }
    Ok(stripes)
}

/// The miters, the junctions and the corners, in vertex order.
fn join_blends(
    ctx: &mut BlendCtx<'_, '_>,
    open: &[EdgeId],
    stripes: &[Stripe],
    rings: &[Ring],
    meets: &Meets,
    kinks: &BTreeSet<VertexId>,
) -> Result<Joins, OpError> {
    let Meets {
        at_vertex,
        junctions,
        cusps,
    } = meets;
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
        // A walked stripe meets another blend only at a tangent vertex of
        // its chain, on the ball's arc there; a miter or a corner patch
        // has no closed form against it.
        if es.len() > 1
            && !junctions.contains(&v)
            && es.iter().any(|e| {
                index_of
                    .get(e)
                    .is_some_and(|&i| matches!(stripes[i].section, Section::Walked { .. }))
            })
        {
            let entities = es.iter().map(|&e| forward(e)).chain([forward(v)]).collect();
            return Err(degenerate(
                entities,
                Reason::Blend(BlendReason::VertexBlend),
            ));
        }
        match es[..] {
            [ea, eb] if junctions.contains(&v) => {
                miter_at.insert(v, miters.len());
                // Across a kink the runs meet within the vertex's tolerance
                // (ADR-0075 §2).
                let kink = if kinks.contains(&v) {
                    Some(m.vertex(v)?.tolerance())
                } else {
                    None
                };
                miters.push(junction(
                    m,
                    view,
                    [run(ea)?, run(eb)?],
                    v,
                    kink,
                    tol,
                    meter,
                )?);
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
    // The cusps' mutual trims, recorded as miters (ADR-0084 §4′).
    let mut cusp_miters: BTreeSet<usize> = BTreeSet::new();
    for join in cusps {
        let [ra, rb] = join.edges.map(run);
        miter_at.insert(join.vertex, miters.len());
        cusp_miters.insert(miters.len());
        miters.push(cusp_join(&env, meter, [ra?, rb?], join)?);
    }
    Ok(Joins {
        miters,
        miter_at,
        cusps: cusp_miters,
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
    let Joins {
        miters,
        miter_at,
        corners,
        corner_at,
        ..
    } = joins;
    // Each stripe's ends, then its contacts between them.
    let mut blends: Vec<Blend> = Vec::with_capacity(stripes.len());
    for mut s in stripes {
        let mut ends: Vec<EndKind> = Vec::with_capacity(2);
        for (at_lo, vertex) in [(true, s.start), (false, s.end)] {
            // A walked stripe narrowed to its vertex meets no other blend
            // there: the vertex is kept, with nothing to share.
            if matches!(s.section, Section::Walked { poles, .. } if poles[usize::from(!at_lo)]) {
                if miter_at.contains_key(&vertex) || corner_at.contains_key(&vertex) {
                    return Err(degenerate(
                        vec![forward(s.edge), forward(vertex)],
                        Reason::Blend(BlendReason::VertexBlend),
                    ));
                }
                ends.push(EndKind::Pole(walked_pole(&s, at_lo)?));
                continue;
            }
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
                (None, None) => EndKind::Face(Box::new(match s.section {
                    Section::Walked { .. } => walked_end(&env, meter, &s, at_lo)?,
                    Section::Round { .. } | Section::Flat => face_end(&env, meter, &s, at_lo)?,
                })),
            });
        }
        let ends: [EndKind; 2] = ends
            .try_into()
            .map_err(|_| invariant("two ends of the blend"))?;
        let t = [0, 1].map(|end| [0, 1].map(|k| ends[end].t(miters, corners, k)));
        if matches!(s.section, Section::Walked { .. }) {
            let cut = [0, 1].map(|end| match ends[end] {
                EndKind::Miter { at, .. } if miters[at].leaning => Some(t[end][0]),
                _ => None,
            });
            walked_cut(&env, meter, &mut s, cut)?;
        }
        let (contacts, rolled) = match s.section {
            Section::Walked { .. } => (walked_contacts(&env, meter, &s, t)?, None),
            Section::Round { .. } | Section::Flat => {
                let (contacts, run) = contacts(&env, &s, t, meter)?;
                let rolled = match run {
                    Some((k, run)) => Some(Box::new(rolled(&env, meter, &s, &contacts, k, run)?)),
                    None => None,
                };
                (contacts, rolled)
            }
        };
        blends.push(Blend {
            stripe: s,
            contacts,
            ends,
            rolled,
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
    let Draft {
        rw, cuts, edits, ..
    } = draft;
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
                        tail: None,
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

/// The stripes: four trim vertices, two contact edges and each end's arcs; at a pole, its vertex kept for both contacts and a degenerate edge for the arc.
fn add_stripes(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    blends: &[Blend],
    joins: &Joins,
    drafted: &Drafted,
) -> Result<Vec<StripeMade>, OpError> {
    let (m, view, _, tol, _, meter) = ctx.parts();
    let Draft {
        rw,
        cuts,
        edits,
        crowded,
    } = draft;
    let Joins {
        miters,
        miter_at: _,
        cusps: _,
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
                EndKind::Pole(pole) => {
                    let at = rw.vertices.len();
                    rw.vertices.push(match pole.remnant {
                        None => VertexSpec::Keep(pole.vertex),
                        Some(r) => VertexSpec::New {
                            point: r.point,
                            tolerance: r.tolerance,
                        },
                    });
                    vertices[end_index] = [at; 2];
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
        // A run-over cuts the contacts at its crossings, and its own edits
        // of the faces take the contacts' place (ADR-0071 §2).
        let rolled = match &blend.rolled {
            Some(rolled) => Some(add_run_over(
                m,
                view,
                rw,
                edits,
                rolled,
                vertices,
                crowded.contains(&rolled.run.edge),
            )?),
            None => None,
        };
        let mut contact_edges = [0usize; 2];
        for (k, contact) in blend.contacts.iter().enumerate() {
            if let Some(rolled) = &rolled {
                contact_edges[k] = rolled.contacts[k][0];
                continue;
            }
            // An edge is no wider than its vertices: a contact pinned to a
            // kink's junction (ADR-0075) widens the vertices it ends at.
            for end in vertices {
                if let VertexSpec::New { tolerance, .. } = &mut rw.vertices[end[k]] {
                    *tolerance = tolerance.max(contact.tolerance);
                }
            }
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
                EndKind::Pole(_) => {
                    // The collapsed row, across from one contact to the
                    // other at the kept vertex.
                    let at = VertexKey::New(vertices[end_index][0]);
                    arcs[end_index].push(rw.edges.len());
                    rw.edges.push((
                        EdgeSpec::New {
                            geometry: EdgeGeometry::Degenerate {
                                range: Interval::UNIT,
                            },
                            start: at,
                            end: at,
                            tolerance: blend.stripe.tolerance,
                        },
                        None,
                    ));
                    continue;
                }
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
            // A flush fan keeps the second corner edge's part from the
            // vertex to its trim, an edge between the first and the last
            // piece (ADR-0082 §2).
            let mut tail = None;
            if let Some(last) = face_end.flush {
                let first = face_end.pieces[0].face;
                let (for_first, for_last) = flush_tail(
                    m,
                    view,
                    rw,
                    face_end,
                    vertices[end_index][1],
                    [first, last],
                    tol,
                    meter,
                )?;
                edits
                    .entry(last)
                    .or_default()
                    .insert
                    .insert(face_end.vertex, for_last);
                tail = Some(for_first);
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
                        tail,
                    },
                );
            }
        }
        // Where the stripe stops short of a vertex, the edge's remnant runs
        // on to it, and on each face the edge is the remnants and the
        // contact between them, in the edge's direction (ADR-0074).
        let remnants = remnant_edges(m, rw, blend, vertices)?;
        if rolled.is_some() && remnants.iter().any(Option::is_some) {
            return Err(degenerate(
                vec![forward(blend.stripe.edge)],
                Reason::Blend(BlendReason::TangentChain),
            ));
        }
        for (k, contact) in blend.contacts.iter().enumerate() {
            if rolled.is_some() {
                continue;
            }
            let pcurve = m.add_curve2(contact.on_face.clone());
            let edit = edits.entry(contact.face).or_default();
            let contact_use = (EdgeKey::New(contact_edges[k]), pcurve);
            if remnants.iter().all(Option::is_none) {
                edit.replace.insert(blend.stripe.edge, contact_use);
                continue;
            }
            let own = (view.uses.get(&blend.stripe.edge).into_iter().flatten())
                .find(|u| u.face == contact.face)
                .ok_or(invariant("the blended edge's use by its face"))?
                .pcurve;
            let [lo, hi] = remnants.map(|r| r.map(|r| SplitPiece::Use(EdgeKey::New(r), own)));
            let pieces = lo
                .into_iter()
                .chain([SplitPiece::Use(contact_use.0, contact_use.1)]);
            edit.split
                .insert(blend.stripe.edge, pieces.chain(hi).collect());
        }
        made.push(StripeMade {
            vertices,
            crossings,
            contacts: contact_edges,
            arcs,
            added: 0,
            rolled,
        });
    }
    Ok(made)
}

/// A flush fan's kept edge (ADR-0082 §2): the second corner edge's curve from
/// the vertex to its trim point `p2` (the rewrite's vertex), added to `rw`
/// with the corner edge as its parent. Returns the first piece's tail, whose
/// pcurve is derived on its plane, and the last piece's insertion, which is
/// the edge itself at the junction, on the corner edge's own pcurve.
#[allow(clippy::too_many_arguments)]
fn flush_tail(
    m: &mut Model,
    view: &BodyView,
    rw: &mut Rewrite,
    end: &super::ends::End,
    p2: usize,
    [first, last]: [FaceId; 2],
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<(Tail, Insertion), OpError> {
    let trim = end.trims[1];
    let entity = *m.edge(trim.edge)?;
    let (curve, range) = entity
        .curve()
        .ok_or(invariant("a flush fan's corner edge curve"))?;
    let kept = if trim.cuts_lo {
        Interval::new(range.lo(), trim.t)
    } else {
        Interval::new(trim.t, range.hi())
    }
    .map_err(|_| invariant("a flush fan's corner edge cut inside its range"))?;
    let (start, finish) = if trim.cuts_lo {
        (VertexKey::Kept(end.vertex), VertexKey::New(p2))
    } else {
        (VertexKey::New(p2), VertexKey::Kept(end.vertex))
    };
    let key = EdgeKey::New(rw.edges.len());
    rw.edges.push((
        EdgeSpec::New {
            geometry: EdgeGeometry::Curve { curve, range: kept },
            start,
            end: finish,
            tolerance: entity.tolerance(),
        },
        Some(trim.edge),
    ));
    let surface = m.surface(m.face(first)?.surface())?.clone();
    let on_first = pcurve_on(
        m.curve(curve)?,
        kept,
        &surface,
        Tolerance::new(entity.tolerance(), tol.angular),
        meter,
    )
    .map_err(fault_of)?;
    let on_first = m.add_curve2(on_first);
    let on_last = view
        .uses
        .get(&trim.edge)
        .and_then(|u| u.iter().find(|u| u.face == last))
        .ok_or(invariant("a flush fan's corner edge on its last piece"))?
        .pcurve;
    let to_vertex = !trim.cuts_lo;
    Ok((
        Tail {
            edge: key,
            pcurve: on_first,
            to_vertex,
        },
        Insertion {
            arc: key,
            pcurve: on_last,
            lo_first: to_vertex,
            edge_at_lo: trim.edge,
            splits: None,
            tail: None,
        },
    ))
}

/// The blended edge's remnant at each end where its stripe stops short of
/// the vertex (ADR-0074), added to `rw`: the edge's curve from the vertex
/// to the cut, kept with the edge's tolerance and recorded against it, the
/// cut being the stripe's end vertex `vertices[end][0]`. Each is `None`
/// where the end is not such a pole.
fn remnant_edges(
    m: &Model,
    rw: &mut Rewrite,
    blend: &Blend,
    vertices: [[usize; 2]; 2],
) -> Result<[Option<usize>; 2], OpError> {
    let edge = blend.stripe.edge;
    let entity = *m.edge(edge)?;
    let mut out = [None; 2];
    for (end_index, end) in blend.ends.iter().enumerate() {
        let EndKind::Pole(Pole {
            remnant: Some(r), ..
        }) = end
        else {
            continue;
        };
        let (curve, old) = entity
            .curve()
            .ok_or(invariant("the blended edge's curve"))?;
        let (lo, hi, start, end) = if end_index == 0 {
            (
                old.lo(),
                r.t,
                VertexKey::Kept(entity.start()),
                VertexKey::New(vertices[0][0]),
            )
        } else {
            (
                r.t,
                old.hi(),
                VertexKey::New(vertices[1][0]),
                VertexKey::Kept(entity.end()),
            )
        };
        let range = Interval::new(lo, hi).map_err(|_| invariant("a remnant inside its edge"))?;
        out[end_index] = Some(rw.edges.len());
        rw.edges.push((
            EdgeSpec::New {
                geometry: EdgeGeometry::Curve { curve, range },
                start,
                end,
                tolerance: entity.tolerance().max(r.tolerance),
            },
            Some(edge),
        ));
    }
    Ok(out)
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
    let view = &ctx.view;
    let Draft {
        rw,
        cuts,
        edits,
        crowded,
    } = draft;
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
        // A run-over cuts the contacts into pieces and leaves the torus
        // without a seam edge (ADR-0071 §2).
        let rolled_made = match &r.rolled {
            Some(rolled) => Some(add_run_over(
                m,
                view,
                rw,
                edits,
                rolled,
                vertices,
                crowded.contains(&rolled.run.edge),
            )?),
            None => None,
        };
        let mut contact_edges = [0usize; 2];
        if let Some(rolled) = &rolled_made {
            contact_edges = [rolled.contacts[0][0], rolled.contacts[1][0]];
        } else {
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
        }
        let mut arcs = [0usize; 2];
        match &r.ends {
            RingEnds::Seam(seam) => {
                if rolled_made.is_none() {
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
                }
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
                            tail: None,
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
            rolled: rolled_made,
        });
    }
    Ok(ring_made)
}

/// The edges that two or more run-overs cross.
fn crowded_edges(blends: &[Blend], rings: &[Ring]) -> BTreeSet<EdgeId> {
    let mut seen: BTreeSet<EdgeId> = BTreeSet::new();
    let mut crowded: BTreeSet<EdgeId> = BTreeSet::new();
    let runs = (blends.iter().filter_map(|b| b.rolled.as_deref()))
        .chain(rings.iter().filter_map(|r| r.rolled.as_deref()));
    for rolled in runs {
        if !seen.insert(rolled.run.edge) {
            crowded.insert(rolled.run.edge);
        }
    }
    crowded
}

/// Each crowded edge cut at the crossings of all the run-overs over it
/// (ADR-0071 §2), once, in the edge's own order.
fn add_crowded_edges(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    blends: &[Blend],
    rings: &[Ring],
    made: &mut Drafted,
) -> Result<(), OpError> {
    if draft.crowded.is_empty() {
        return Ok(());
    }
    let m = &mut *ctx.m;
    let view = &ctx.view;
    let Draft {
        rw, edits, crowded, ..
    } = draft;
    let mut by_edge: BTreeMap<EdgeId, Vec<(&Rolled, &mut RolledMade)>> = BTreeMap::new();
    let stripes = (blends.iter().zip(made.stripes.iter_mut()))
        .filter_map(|(b, mk)| Some((b.rolled.as_deref()?, mk.rolled.as_mut()?)));
    let rings = (rings.iter().zip(made.rings.iter_mut()))
        .filter_map(|(r, mk)| Some((r.rolled.as_deref()?, mk.rolled.as_mut()?)));
    for (rolled, rolled_made) in stripes.chain(rings) {
        if crowded.contains(&rolled.run.edge) {
            by_edge
                .entry(rolled.run.edge)
                .or_default()
                .push((rolled, rolled_made));
        }
    }
    for runs in by_edge.values_mut() {
        add_crowded_edge(m, view, rw, edits, runs)?;
    }
    Ok(())
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
    let (m, view, _, tol, samples, meter) = ctx.parts();
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
        // A run-over's gaps, by loop and the place in it (ADR-0071 §2).
        let mut gaps: Vec<(EdgeId, usize, usize)> = Vec::new();
        for l in entity.loops() {
            let mut uses: Vec<StoredUse> = Vec::with_capacity(l.coedges().len() + 1);
            // Where each coedge's use sits in `uses`.
            let mut at: Vec<usize> = Vec::with_capacity(l.coedges().len());
            let mut gapped: Vec<(usize, EdgeId)> = Vec::new();
            for c in l.coedges() {
                let id = c.edge();
                if let Some(pieces) = edit.and_then(|e| e.split.get(&id)) {
                    // The pieces walked the way the coedge walks the edge.
                    at.push(uses.len());
                    let forward = c.orientation() == Orientation::Forward;
                    let ordered: Vec<_> = if forward {
                        pieces.iter().collect()
                    } else {
                        pieces.iter().rev().collect()
                    };
                    for piece in ordered {
                        match *piece {
                            SplitPiece::Use(edge, pcurve) => uses.push(StoredUse {
                                edge,
                                orientation: c.orientation(),
                                pcurve,
                            }),
                            SplitPiece::Gap(by) => gapped.push((uses.len(), by)),
                        }
                    }
                    continue;
                }
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
                    let arc = StoredUse {
                        edge: ins.arc,
                        orientation,
                        pcurve: ins.pcurve,
                    };
                    // A flush fan's first piece goes on to the vertex by
                    // the kept edge, before the arc where it arrives by an
                    // extra (ADR-0082 §2).
                    match ins.tail {
                        None => uses.push(arc),
                        Some(tail) => {
                            let kept = StoredUse {
                                edge: tail.edge,
                                orientation: if arrived_at_lo == tail.to_vertex {
                                    Orientation::Forward
                                } else {
                                    Orientation::Reversed
                                },
                                pcurve: tail.pcurve,
                            };
                            if arrived_at_lo {
                                uses.extend([arc, kept]);
                            } else {
                                uses.extend([kept, arc]);
                            }
                        }
                    }
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
            if !gapped.is_empty() {
                if inner.is_some() || uses.is_empty() {
                    return Err(invariant("a run-over's gap in a loop of its own"));
                }
                // Gaps with no use between them, round the loop too, are one.
                for (g, by) in gapped {
                    let gap = (by, loops.len(), g % uses.len());
                    if !gaps.contains(&gap) {
                        gaps.push(gap);
                    }
                }
            }
            loops.push(uses);
            loops.extend(inner);
        }
        join_at_gaps(&mut loops, &gaps)?;
        end_on_kept_pcurves(m, rw, face, &mut loops, tol, samples, meter)?;
        rw.faces.insert(face, loops);
    }
    Ok(())
}

/// A loop being joined: a use, or the gap of a run-over.
enum Item {
    Use(StoredUse),
    Gap(EdgeId),
}

/// `F`'s loops with each run-over's gap joined into one (ADR-0071 §2):
/// the contact's loop runs to one crossing and stops, the loop that held
/// `E` runs on from it along `E`'s outer pieces to the other crossing, so
/// each loop from its gap round to it, one after the other, is the
/// joined loop. It takes the place of the lower loop. A loop with the gaps
/// of several run-overs, the plate's outer loop with its edges crossed by
/// several, is joined to one contact's loop after another. `gaps` are
/// `(run-over, loop, place in it)`; none leaves the loops as they are.
fn join_at_gaps(
    loops: &mut Vec<Vec<StoredUse>>,
    gaps: &[(EdgeId, usize, usize)],
) -> Result<(), OpError> {
    if gaps.is_empty() {
        return Ok(());
    }
    let mut items: Vec<Vec<Item>> = (loops.drain(..))
        .map(|l| l.into_iter().map(Item::Use).collect())
        .collect();
    for &(by, l, at) in gaps.iter().rev() {
        items
            .get_mut(l)
            .ok_or(invariant("a run-over's gap in a loop of its face"))?
            .insert(at, Item::Gap(by));
    }
    let mut ids: Vec<EdgeId> = Vec::new();
    for g in gaps {
        if !ids.contains(&g.0) {
            ids.push(g.0);
        }
    }
    for id in ids {
        let find = |items: &[Vec<Item>]| -> Vec<(usize, usize)> {
            let mut at = Vec::new();
            for (l, list) in items.iter().enumerate() {
                for (i, item) in list.iter().enumerate() {
                    if matches!(item, Item::Gap(g) if *g == id) {
                        at.push((l, i));
                    }
                }
            }
            at
        };
        let [(a, at_a), (b, at_b)] = find(&items)[..] else {
            return Err(invariant("a run-over's two gaps on its face"));
        };
        if a == b {
            return Err(invariant("a run-over's gaps on two loops of its face"));
        }
        let (a, at_a, b, at_b) = if a < b {
            (a, at_a, b, at_b)
        } else {
            (b, at_b, a, at_a)
        };
        let mut joined: Vec<Item> = Vec::new();
        let (second, first): (Vec<Item>, Vec<Item>) =
            (std::mem::take(&mut items[b]), std::mem::take(&mut items[a]));
        for (list, at) in [(first, at_a), (second, at_b)] {
            let (before, after) = list.split_at(at);
            let after = after.iter().skip(1);
            joined.extend(after.chain(before.iter()).map(|i| match i {
                Item::Use(u) => Item::Use(*u),
                Item::Gap(g) => Item::Gap(*g),
            }));
        }
        items[a] = joined;
        items.remove(b);
    }
    for list in items {
        let mut uses = Vec::with_capacity(list.len());
        for item in list {
            match item {
                Item::Use(u) => uses.push(u),
                Item::Gap(_) => return Err(invariant("a run-over's gap left in a loop")),
            }
        }
        loops.push(uses);
    }
    Ok(())
}

/// The edge a stored use is over, its curve, range and tolerance, and
/// whether it is an operand edge the blend shortened (`Some`, its index in
/// the rewrite) or one it made or kept whole (`None`). `None` overall for
/// an edge with no curve.
type UseEdge = (CurveId, Interval, f64, Option<usize>);

fn use_edge(m: &Model, rw: &Rewrite, key: EdgeKey) -> Result<Option<UseEdge>, OpError> {
    Ok(match key {
        EdgeKey::Kept(id) => {
            let edge = m.edge(id)?;
            edge.curve().map(|(c, r)| (c, r, edge.tolerance(), None))
        }
        EdgeKey::New(k) => match rw.edges.get(k) {
            Some((
                EdgeSpec::New {
                    geometry: EdgeGeometry::Curve { curve, range },
                    tolerance,
                    ..
                },
                parent,
            )) => Some((*curve, *range, *tolerance, parent.map(|_| k))),
            Some(_) => None,
            None => return Err(invariant("a stored use's edge in the rewrite")),
        },
    })
}

/// The loops of `face` with every shortened operand edge's stored pcurve
/// ended where the blend's new edge meets it at the cut: a read edge's
/// pcurve may stand off its curve by as much as the edge's tolerance,
/// while the blend's contacts and arcs are exact, so at the cut the two
/// would jump (L2). The shortened edge is a new edge already and keeps its
/// tolerance, which its cut vertex holds; the move is checked against it
/// at the model's samples (ADR-0062, step 10b). A junction within half the
/// L2 band is left as it is, and so is one whose two points are further
/// apart in 3D than the edge's tolerance, or a move the tolerance does not
/// hold, for the checker to report.
fn end_on_kept_pcurves(
    m: &mut Model,
    rw: &Rewrite,
    face: FaceId,
    loops: &mut [Vec<StoredUse>],
    tol: Tolerance,
    samples: usize,
    meter: &mut Meter<'_>,
) -> Result<(), OpError> {
    let parametric = m.precision().parametric_tolerance;
    let surface = m.surface(m.face(face)?.surface())?.clone();
    let end_param = |range: Interval, o: Orientation, at_end: bool| {
        if (o == Orientation::Forward) == at_end {
            range.hi()
        } else {
            range.lo()
        }
    };
    for uses in loops.iter_mut() {
        let n = uses.len();
        // The moved ends of each shortened use, by its place in the loop:
        // `[range.lo(), range.hi()]`.
        let mut moves: BTreeMap<usize, [Option<Point2>; 2]> = BTreeMap::new();
        for i in 0..n {
            let j = (i + 1) % n;
            let (Some(a), Some(b)) = (
                use_edge(m, rw, uses[i].edge)?,
                use_edge(m, rw, uses[j].edge)?,
            ) else {
                continue;
            };
            let made = |key: EdgeKey| matches!(key, EdgeKey::New(k) if rw.edges[k].1.is_none());
            // The shortened use and the new one, each with the parameter
            // it meets the junction at: the end of `i`, the start of `j`.
            let ((cut, cut_t, cut_range), (new, new_t)) = match (a.3, b.3) {
                (Some(_), None) if made(uses[j].edge) => (
                    (i, end_param(a.1, uses[i].orientation, true), a.1),
                    (j, end_param(b.1, uses[j].orientation, false)),
                ),
                (None, Some(_)) if made(uses[i].edge) => (
                    (j, end_param(b.1, uses[j].orientation, false), b.1),
                    (i, end_param(a.1, uses[i].orientation, true)),
                ),
                _ => continue,
            };
            let target = m.curve2(uses[new].pcurve)?.point(new_t);
            let at = m.curve2(uses[cut].pcurve)?.point(cut_t);
            let bound = bands(&surface, target, parametric);
            let d = target - at;
            if d.x.abs() <= 0.5 * bound[0] && d.y.abs() <= 0.5 * bound[1] {
                continue;
            }
            // Further apart in 3D than the shortened edge's tolerance —
            // a whole period across a seam, which L2 takes — no move
            // within it explains.
            let apart = (surface.point(target.x, target.y) - surface.point(at.x, at.y)).norm();
            let cut_tolerance = if cut == i { a.2 } else { b.2 };
            if apart > cut_tolerance {
                continue;
            }
            let slot = usize::from(cut_t != cut_range.lo());
            moves.entry(cut).or_default()[slot] = Some(target);
        }
        for (cut, ends) in moves {
            let Some((curve, range, tolerance, _)) = use_edge(m, rw, uses[cut].edge)? else {
                return Err(invariant("a shortened use's edge has a curve"));
            };
            let pc = m.curve2(uses[cut].pcurve)?.clone();
            let moved = pcurve_ending_on(&pc, range, ends, &surface, tol.linear, meter)
                .map_err(fault_of)?;
            let curve = m.curve(curve)?;
            let n = samples.max(2);
            let residual = (0..n)
                .map(|i| range.lerp(i as f64 / (n - 1) as f64))
                .map(|t| {
                    let q = moved.point(t);
                    let (on_surface, on_curve) = (surface.point(q.x, q.y), curve.point(t));
                    (on_surface - on_curve).norm()
                        + RELATIVE_ROUNDING * on_surface.coords.norm().max(on_curve.coords.norm())
                })
                .fold(0.0, f64::max);
            if residual <= tolerance {
                uses[cut].pcurve = m.add_curve2(moved);
            }
        }
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
        let mut end_uses: [Vec<StoredUse>; 2] = Default::default();
        for (end_index, end) in blend.ends.iter().enumerate() {
            let arcs: Vec<(bool, &Curve2)> = match end {
                EndKind::Face(face_end) => face_end
                    .pieces
                    .iter()
                    .map(|p| (p.arc.lo_first, &p.arc.on_blend))
                    .collect(),
                EndKind::Pole(pole) => vec![(true, &pole.on_blend)],
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
        let shell = view.shell_of[&lo.face];
        if let (Some(rolled), Some(rolled_made)) = (&blend.rolled, &mut made.rolled) {
            add_rolled_faces(
                m,
                rw,
                shell,
                blend,
                rolled,
                rolled_made,
                &mut made.added,
                start_arcs,
                end_arcs,
            )?;
            continue;
        }
        let surface = m.add_surface(blend.stripe.surface.clone());
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
            shell,
            surface,
            orientation: if blend.stripe.convex
                != matches!(blend.stripe.section, Section::Walked { flipped: true, .. })
            {
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
) -> Result<(), OpError> {
    let (m, view, ..) = ctx.parts();
    let rw = &mut draft.rw;
    let Joins {
        miters, miter_at, ..
    } = joins;
    let ring_made = &mut made.rings;
    for (r, made) in rings.iter().zip(ring_made.iter_mut()) {
        let [lower, upper] = &r.contacts;
        if let Some(rolled) = &r.rolled {
            add_ring_rolled_faces(m, rw, view.shell_of[&lower.face], r, rolled, made)?;
            continue;
        }
        let surface = m.add_surface(r.surface.clone());
        let along = |yes: bool| {
            if yes {
                Orientation::Forward
            } else {
                Orientation::Reversed
            }
        };
        if r.across {
            // A walked ring: `u` across, `v` along the edge, the seam at
            // `v`'s two ends — the loop of an open stripe whose two ends
            // are one arc.
            let RingEnds::Seam(seam) = &r.ends else {
                return Err(invariant("a walked ring's seam"));
            };
            let loop_uses = vec![
                StoredUse {
                    edge: EdgeKey::New(made.arcs[0]),
                    orientation: Orientation::Forward,
                    pcurve: m.add_curve2(seam.on_blend[0].clone()),
                },
                StoredUse {
                    edge: EdgeKey::New(made.contacts[1]),
                    orientation: Orientation::Forward,
                    pcurve: m.add_curve2(upper.on_blend.clone()),
                },
                StoredUse {
                    edge: EdgeKey::New(made.arcs[0]),
                    orientation: Orientation::Reversed,
                    pcurve: m.add_curve2(seam.on_blend[1].clone()),
                },
                StoredUse {
                    edge: EdgeKey::New(made.contacts[0]),
                    orientation: Orientation::Reversed,
                    pcurve: m.add_curve2(lower.on_blend.clone()),
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
            continue;
        }
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
                        // Up from the lower contact when `range.lo()`
                        // is on it: at `q` when `q` is.
                        (
                            made.arcs[j],
                            junction.on_blend[side].clone(),
                            junction.q_first == (junction.shared[side] == 0),
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
    Ok(())
}

/// The provenance of the rewrite: every record against the blended edge.
fn record(
    out: rebuild::Rewritten,
    blends: &[Blend],
    rings: &[Ring],
    joins: &Joins,
    drafted: &Drafted,
) -> (Body, Provenance) {
    let Joins {
        miters,
        miter_at,
        cusps,
        ..
    } = joins;
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
        // An end at a cusp's mutual trim is the join's, recorded after.
        let at_cusp =
            |end: &EndKind| matches!(end, EndKind::Miter { at, .. } if cusps.contains(at));
        let arcs = (made.arcs.iter().zip(&blend.ends))
            .filter(|(_, end)| !at_cusp(end))
            .flat_map(|(arcs, _)| arcs);
        for &k in made.contacts.iter().chain(arcs) {
            p.add_generated(origin, forward(out.edges[k]));
        }
        // A pole's vertex is the operand's, kept, unless the stripe stops
        // short of it.
        let made_here = (made.vertices.iter().zip(&blend.ends))
            .filter(|(_, end)| !matches!(end, EndKind::Pole(p) if p.remnant.is_none()))
            .filter(|(_, end)| !at_cusp(end))
            .flat_map(|(vertices, _)| vertices);
        for &v in made_here.chain(&made.crossings) {
            p.add_generated(origin, forward(out.vertices[v]));
        }
        // A run-over's second stripe face and rolled face, its contacts'
        // other pieces, the rolled face's foot, the section arcs and the
        // crossings; `E`'s pieces are its own, `Modified` (ADR-0071 §4).
        if let Some(rolled) = &made.rolled {
            for &k in &rolled.added {
                p.add_generated(origin, forward(out.added[k]));
            }
            let edges = (rolled.contacts.iter().map(|c| &c[1]))
                .chain([&rolled.foot])
                .chain(&rolled.arcs);
            for &k in edges {
                p.add_generated(origin, forward(out.edges[k]));
            }
            for &v in rolled.vertices.iter().flatten() {
                p.add_generated(origin, forward(out.vertices[v]));
            }
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
    // two or four vertices; an end at a cusp's mutual trim is the join's,
    // recorded after.
    for (r, made) in rings.iter().zip(ring_made) {
        let origin = forward(r.edge);
        let (ends, own) = match &r.ends {
            RingEnds::Seam(_) => (1, [true, false]),
            RingEnds::Open(ends) => (
                2,
                ends.each_ref().map(|end| match end {
                    RingEnd::Junction(v) => !miter_at.get(v).is_some_and(|at| cusps.contains(at)),
                    RingEnd::Face(_) => true,
                }),
            ),
        };
        p.add_generated(origin, forward(out.added[made.added]));
        let arcs = if made.rolled.is_some() {
            &[][..]
        } else {
            &made.arcs[..ends]
        };
        let arcs = arcs.iter().zip(own).filter(|(_, own)| *own);
        for &k in made.contacts.iter().chain(arcs.map(|(k, _)| k)) {
            p.add_generated(origin, forward(out.edges[k]));
        }
        let vertices = made.vertices[..ends].iter().zip(own);
        for (vertices, _) in vertices.filter(|(_, own)| *own) {
            for &v in vertices {
                p.add_generated(origin, forward(out.vertices[v]));
            }
        }
        // A run-over's rolled face, its contacts' other pieces, the rolled
        // face's foot, the section arcs and the crossings (ADR-0071 §4).
        if let Some(rolled) = &made.rolled {
            p.add_generated(origin, forward(out.added[rolled.added[1]]));
            let edges = (rolled.contacts.iter().map(|c| &c[1]))
                .chain([&rolled.foot])
                .chain(&rolled.arcs);
            for &k in edges {
                p.add_generated(origin, forward(out.edges[k]));
            }
            for &v in rolled.vertices.iter().flatten() {
                p.add_generated(origin, forward(out.vertices[v]));
            }
        }
    }
    // A cusp's section edge, `T*` and `S`, from each of its two edges after
    // that edge's own records (ADR-0084 §7′).
    for &at in cusps {
        let (mt, made) = (&miters[at], &miter_made[at]);
        for &edge in &mt.edges {
            let origin = forward(edge);
            p.add_generated(origin, forward(out.edges[made.edge]));
            for &v in &made.vertices {
                p.add_generated(origin, forward(out.vertices[v]));
            }
        }
    }
    (out.body, p)
}
