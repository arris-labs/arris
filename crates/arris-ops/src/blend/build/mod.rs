//! Assembly: the insertions, face edits and cuts a set of stripes makes, rewritten through `rebuild::rewrite`.

mod classify;
mod entities;
mod faces;
mod make;

use std::collections::{BTreeMap, BTreeSet};

use arris_math::{Meter, Tolerance};
use arris_topo::builder::EdgeKey;
use arris_topo::{Body, Curve2Id, EdgeId, FaceId, Model, Provenance, VertexId};

use self::classify::{classify, vertex_joins};
use self::entities::{add_crowded_edges, add_rings, add_stripes, crowded_edges, shorten_edges};
use self::faces::{add_blend_faces, add_corner_faces, add_ring_faces, rewrite_faces};
use self::make::{add_corners, add_miters, join_blends, make_blends, make_stripes};
use super::corner::Corner;
use super::crowd::crowded_faces;
use super::ends::{EndKind, Trim};
use super::junction::pin_kinks;
use super::miter::Miter;
use super::mutual::CuspJoin;
use super::ring::{Ring, RingEnd, RingEnds};
use super::rolled::RolledMade;
use super::{Blend, Kind, invariant};
use crate::body_view::BodyView;
use crate::error::OpError;
use crate::rebuild;
use crate::rebuild::{Rewrite, forward};

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
    crowded_faces(&ctx.split().0, &rings)?;
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
