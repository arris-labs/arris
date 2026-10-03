//! The fillet column, counted by part (ADR-0037, `docs/ROADMAP.md` §C6):
//! what each part's first refusal at the battery's radius is, and
//! what every other edge of it would meet once that one is cleared.
//!
//! The battery's fillet stage blends a sample of [`FILLET_EDGES`] edges
//! together and records the first refusal (`crate::battery`); the refusal
//! histogram then counts a part once, under that refusal. That says what
//! stops the stage and nothing about what stands behind it. [`blend_census`]
//! runs the same sample as one operation, and then every blendable edge of
//! the solid alone at the same radius, and classifies each outcome by
//! [`class_of`]: a part's row reads *first refusal → the others, with the
//! edges each holds*. A refusal behind the first one is a refusal a plan
//! that clears only the first leaves in place. No oracle is asked: the
//! census is about what Arris refuses, and the battery's survey holds the
//! agreement.
//!
//! A `BlendTooLarge` is the one refusal the census takes apart further
//! (ADR-0038): [`run_over_cause`] names which site
//! refused from the entities the error carries, and
//! [`ask_the_oracle`] puts each such edge to Open CASCADE alone at the same
//! radius, because a radius it refuses too is no run-over to build.
//!
//! [`FILLET_EDGES`]: crate::battery::FILLET_EDGES

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::Path;

use serde::{Deserialize, Serialize};

use arris_io::arris_check::arris_topo::arris_geom::GeomKind;
use arris_io::arris_check::arris_topo::arris_math::{Point3, Vec3};
use arris_io::arris_check::arris_topo::{Body, EdgeId, EntityId, FaceId, Model, VertexId};
use arris_io::step::ReadOptions;
use arris_ops::{OpError, Reason};

use crate::battery::Class;
use crate::battery::{fillet_edges, fillet_radius};
use crate::differential::panicked;
use crate::fixtures::{PrecisionSpec, corpus_root};
use crate::histogram::{COMMITTED_TIER, Stage, blocks_reason};
use crate::part;
use crate::unmetered::{fillet, step_read};
use std::panic::{AssertUnwindSafe, catch_unwind};

/// What a built edge is counted under.
pub const BUILT: &str = "built";

/// What a blend that panics is counted under: a kernel bug, never a
/// refusal, so it is named apart (in a debug build the checker's verdict
/// on an output is a panic, `.agents/rules/kernel.md`).
pub const PANICKED: &str = "PANIC";

/// How a blend of the census ends when it is not built.
enum Refused {
    Op(OpError),
    Panic(String),
}

fn blend(
    m: &mut Model,
    body: Body,
    edges: &[arris_io::arris_check::arris_topo::Edge],
    radius: f64,
) -> Result<(), Refused> {
    match catch_unwind(AssertUnwindSafe(|| fillet(m, body, edges, radius))) {
        Ok(Ok(_)) => Ok(()),
        Ok(Err(e)) => Err(Refused::Op(e)),
        Err(payload) => Err(Refused::Panic(panicked(&*payload).to_string())),
    }
}

/// One solid's census.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SolidCensus {
    /// The solid, `#id[instance]`.
    pub solid: String,
    /// The fillet radius of the battery's stage for this solid.
    pub radius: f64,
    /// How the battery's sample, blended together, ends: [`BUILT`], or the
    /// refusal's [`class_of`]. `None` when the solid has no edge to blend.
    pub sample: Option<String>,
    /// The cycle that first refusal blocks, where it is one.
    pub blocks: Option<String>,
    /// The battery's sampled edges, each blended alone: what clearing the
    /// first refusal leaves in the stage, in sample order.
    pub sampled: Vec<String>,
    /// Every blendable edge blended alone at the radius: the class and how
    /// many edges fall in it, sorted by class.
    pub alone: BTreeMap<String, usize>,
    /// Every blendable edge alone that is refused `BlendTooLarge`, with the
    /// site that refused it ([`run_over_cause`]) and, once
    /// [`ask_the_oracle`] ran, Open CASCADE's verdict on it.
    #[serde(default)]
    pub too_large: Vec<RunOverEdge>,
    /// Every blendable edge alone that is refused `VertexBlend`, with the
    /// site that refused it ([`vertex_blend_cause`]) and, once
    /// [`ask_the_oracle_vertex`] ran, Open CASCADE's verdict on it.
    #[serde(default)]
    pub vertex_blend: Vec<RunOverEdge>,
    /// The battery's sample blended together, where that is refused
    /// `VertexBlend`.
    #[serde(default)]
    pub vertex_set: Option<VertexSet>,
    /// Every blendable edge alone that is refused `TangentChain`, with the
    /// site that refused it ([`tangent_chain_cause`]); no oracle is asked.
    #[serde(default)]
    pub tangent_chain: Vec<RunOverEdge>,
}

/// The battery's sample, blended together and refused `VertexBlend`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VertexSet {
    /// [`vertex_set_cause`]: the miter or corner the edges make.
    pub cause: String,
    /// A point on each blended edge, the oracle's name for it.
    pub points: Vec<[f64; 3]>,
    /// Whether every one of the sampled edges builds alone, so that only
    /// the set is refused.
    pub each_alone_builds: bool,
    /// Open CASCADE's verdict on the set together; `None` where not asked.
    pub occt: Option<String>,
}

/// One edge refused `BlendTooLarge` when blended alone.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RunOverEdge {
    /// The edge, as `{:?}` prints its id.
    pub edge: String,
    /// The midpoint of its curve, the oracle's name for it.
    pub at: [f64; 3],
    /// [`run_over_cause`]'s sub-cause.
    pub cause: String,
    /// One of the battery's sampled edges: the four that decide whether the
    /// part leaves the fillet column.
    #[serde(default)]
    pub sampled: bool,
    /// Open CASCADE's verdict on this edge alone at the census's radius
    /// (`builds`, `invalid`, `refuses`, `no-edge`); `None` where it was not
    /// asked.
    pub occt: Option<String>,
}

/// A refusal's class: what a plan can name and a census can add up. An
/// unsupported refusal is a pair, an edge or an end, with its two kinds; a degenerate result is the
/// reason's name; any other error is its variant.
pub fn class_of(e: &OpError) -> String {
    match e {
        OpError::Unsupported { a, b } => {
            // What the refusal names: two faces are a pair; a curve kind
            // against a surface is an edge the construction cannot read
            // (a conic written as a B-spline, an ellipse); a surface kind
            // carried by an edge, against the face across, is an end the
            // table cannot trim.
            let what = match (a.0, a.1.id) {
                (GeomKind::Curve(_), _) => "edge",
                (_, EntityId::Edge(_)) => "end",
                _ => "pair",
            };
            let (mut x, mut y) = (a.0.to_string(), b.0.to_string());
            if what == "pair" && y < x {
                core::mem::swap(&mut x, &mut y);
            }
            format!("Unsupported {what}: {x} × {y}")
        }
        OpError::Degenerate { reason, .. } => {
            let name = format!("{reason:?}");
            let end = name
                .find(|c: char| !c.is_alphanumeric())
                .unwrap_or(name.len());
            format!("Degenerate: {}", &name[..end])
        }
        OpError::Internal(_) => "Internal".to_string(),
        OpError::InvalidInput { .. } => "InvalidInput".to_string(),
        OpError::Profile(_) => "Profile".to_string(),
        OpError::Tolerance { .. } => "Tolerance".to_string(),
        OpError::NotFound(_) => "NotFound".to_string(),
        OpError::Unkeyed { .. } => "Unkeyed".to_string(),
        OpError::Rejected(_) => "Rejected".to_string(),
        OpError::Interrupted(_) => "Interrupted".to_string(),
    }
}

/// Counts `body`'s fillet column: the battery's sample blended together,
/// then each blendable edge alone, all at the battery's radius, in a
/// scratch copy of `m` (the model passed is not touched).
///
/// # Errors
/// A walk of `body` fails: never a refusal of the operation, which is the
/// census's data.
pub fn blend_census(m: &Model, body: Body, solid: &str) -> Result<SolidCensus, String> {
    let (blendable, sample) = fillet_edges(m, body)?;
    let mut out = SolidCensus {
        solid: solid.to_string(),
        radius: fillet_radius(&sample),
        sample: None,
        blocks: None,
        sampled: Vec::new(),
        alone: BTreeMap::new(),
        too_large: Vec::new(),
        vertex_blend: Vec::new(),
        vertex_set: None,
        tangent_chain: Vec::new(),
    };
    if sample.is_empty() {
        return Ok(out);
    }
    let handles = m.edges(body).map_err(|e| e.to_string())?;
    let handle = |id| handles.iter().find(|h| h.id == id).copied();
    let mut scratch = m.clone();
    let chosen: Vec<_> = sample.iter().filter_map(|s| handle(s.0)).collect();
    out.sample = Some(match blend(&mut scratch, body, &chosen, out.radius) {
        Ok(_) => BUILT.to_string(),
        Err(Refused::Op(e)) => {
            out.blocks = blocks_reason(Stage::Fillet, &e).map(|c| c.to_string());
            if let Some(cause) = vertex_set_cause(m, &e) {
                let points: Vec<[f64; 3]> = chosen
                    .iter()
                    .filter_map(|h| edge_midpoint(m, h.id))
                    .map(|p| [p.x, p.y, p.z])
                    .collect();
                out.vertex_set = Some(VertexSet {
                    cause,
                    points,
                    each_alone_builds: false,
                    occt: None,
                });
            }
            class_of(&e)
        }
        Err(Refused::Panic(_)) => PANICKED.to_string(),
    });
    let mut classes: BTreeMap<EdgeId, String> = BTreeMap::new();
    let mut ids = blendable.clone();
    ids.extend(sample.iter().map(|s| s.0));
    ids.sort();
    ids.dedup();
    for id in ids {
        let Some(edge) = handle(id) else { continue };
        let class = match blend(&mut scratch, body, &[edge], out.radius) {
            Ok(_) => BUILT.to_string(),
            Err(Refused::Op(e)) => {
                let entry = |cause: String| {
                    edge_midpoint(m, id).map(|at| RunOverEdge {
                        edge: format!("{id:?}"),
                        at: [at.x, at.y, at.z],
                        cause,
                        sampled: sample.iter().any(|x| x.0 == id),
                        occt: None,
                    })
                };
                if let Some(x) = run_over_cause(m, id, &e).and_then(entry) {
                    out.too_large.push(x);
                } else if let Some(x) = vertex_blend_cause(m, id, &e).and_then(entry) {
                    out.vertex_blend.push(x);
                } else if let Some(x) = tangent_chain_cause(m, id, &e).and_then(entry) {
                    out.tangent_chain.push(x);
                }
                class_of(&e)
            }
            Err(Refused::Panic(why)) => {
                eprintln!("{solid}: edge {id:?} alone panics: {why}");
                PANICKED.to_string()
            }
        };
        classes.insert(id, class);
    }
    out.sampled = sample
        .iter()
        .filter_map(|s| classes.get(&s.0).cloned())
        .collect();
    if let Some(set) = &mut out.vertex_set {
        set.each_alone_builds = !out.sampled.is_empty() && out.sampled.iter().all(|c| c == BUILT);
    }
    for id in blendable {
        if let Some(class) = classes.get(&id) {
            *out.alone.entry(class.clone()).or_default() += 1;
        }
    }
    Ok(out)
}

/// The midpoint of `edge`'s curve; `None` for an edge with none.
fn edge_midpoint(m: &Model, edge: EdgeId) -> Option<Point3> {
    let (c, range) = m.edge(edge).ok()?.curve()?;
    Some(m.curve(c).ok()?.point(range.lerp(0.5)))
}

/// The surface kind of face `f`.
fn face_kind(m: &Model, f: arris_io::arris_check::arris_topo::FaceId) -> Option<String> {
    let surface = m.surface(m.face(f).ok()?.surface()).ok()?;
    Some(surface.kind().to_string())
}

/// The two surface kinds an edge separates, sorted: the pair a blend of it
/// asks the construction for.
fn pair_of(m: &Model, edge: EdgeId) -> Option<String> {
    let mut kinds: Vec<String> = m
        .edge_uses(edge)
        .ok()?
        .iter()
        .filter_map(|u| face_kind(m, u.face))
        .collect();
    kinds.sort();
    Some(kinds.join(" × "))
}

/// Which site refused a `BlendTooLarge` of `edge` blended alone, from the
/// entities the error names (`None` for any other refusal): *the contact
/// leaves a face* (the edge and the face it runs out of), *the ball finds
/// no place on either face* (the edge and both faces), *a corner edge or a
/// seam is shorter than the trim* (the edge and that edge), *a corner of
/// three edges whose trims run past one another* (three edges), *the
/// blend's closing or its fit* (the edge alone). Each is followed by the
/// pair of surface kinds the blended edge separates, which is what a
/// construction is keyed by.
pub fn run_over_cause(m: &Model, edge: EdgeId, e: &OpError) -> Option<String> {
    let OpError::Degenerate {
        entities,
        reason: Reason::BlendTooLarge,
    } = e
    else {
        return None;
    };
    let pair = pair_of(m, edge).unwrap_or_default();
    let ids: Vec<EntityId> = entities.iter().map(|s| s.id).collect();
    let what = match ids.as_slice() {
        [EntityId::Edge(_), EntityId::Face(f)] => {
            format!(
                "the contact leaves a {} face",
                face_kind(m, *f).unwrap_or_default()
            )
        }
        [EntityId::Edge(_), EntityId::Face(_), EntityId::Face(_)] => {
            "the ball finds no place on either face".to_string()
        }
        [EntityId::Edge(_), EntityId::Edge(other)] => {
            let seam = m
                .edge_uses(*other)
                .is_ok_and(|u| u.len() == 2 && u[0].face == u[1].face);
            if seam {
                "a seam shorter than the trim".to_string()
            } else {
                "a corner edge shorter than the trim".to_string()
            }
        }
        [EntityId::Edge(_), EntityId::Edge(_), EntityId::Edge(_)] => {
            "a corner whose trims run past one another".to_string()
        }
        [EntityId::Edge(_)] => "the closing or the fit".to_string(),
        _ => "another site".to_string(),
    };
    Some(format!("{what}; edge {pair}"))
}

/// How parallel two directions must be for the census to call them one
/// line or one tangent plane: the sine of the angle between them. The
/// census names a site, it decides no geometry, so the cut is wide enough
/// for a part's file digits and far from any real dihedral.
const PARALLEL: f64 = 1e-6;

/// The edges at `vertex` next to `edge` in the loops of its faces, the
/// corner edges ADR-0007 names: for each use of `edge`, the neighbouring
/// coedge in that loop that touches `vertex`.
fn corner_edges(m: &Model, edge: EdgeId, vertex: VertexId) -> Vec<EdgeId> {
    let mut out = Vec::new();
    let Ok(uses) = m.edge_uses(edge) else {
        return out;
    };
    for u in uses {
        let Ok(face) = m.face(u.face) else { continue };
        let Some(l) = face.loops().get(u.loop_index) else {
            continue;
        };
        let n = l.coedges().len();
        for j in [(u.coedge_index + n - 1) % n, (u.coedge_index + 1) % n] {
            let c = l.coedges()[j].edge();
            let touches = m
                .edge(c)
                .is_ok_and(|x| x.start() == vertex || x.end() == vertex);
            if c != edge && touches && !out.contains(&c) {
                out.push(c);
            }
        }
    }
    out
}

/// The direction of `edge`'s curve at its end on `vertex`, sign as the
/// curve runs.
fn tangent_at_vertex(m: &Model, edge: EdgeId, vertex: VertexId) -> Option<Vec3> {
    let e = m.edge(edge).ok()?;
    let (c, range) = e.curve()?;
    let t = if e.start() == vertex {
        range.lo()
    } else {
        range.hi()
    };
    let d = m.curve(c).ok()?.eval(t).d1;
    (d.norm() > 0.0).then_some(d)
}

fn parallel(a: Vec3, b: Vec3) -> bool {
    a.cross(&b).norm() <= PARALLEL * a.norm() * b.norm()
}

/// Whether the two faces of `edge` are tangent at its midpoint.
fn smooth_edge(m: &Model, edge: EdgeId) -> bool {
    let Ok(uses) = m.edge_uses(edge) else {
        return false;
    };
    let (Some(at), [a, b]) = (edge_midpoint(m, edge), uses) else {
        return false;
    };
    let normal = |f| {
        let surface = m.surface(m.face(f).ok()?.surface()).ok()?;
        let uv = surface.project(at).ok()?.uv;
        Some(surface.normal(uv.x, uv.y)?.into_inner())
    };
    match (normal(a.face), normal(b.face)) {
        (Some(x), Some(y)) => parallel(x, y),
        _ => false,
    }
}

/// What an extra edge at a vertex of more than three edges is; `blended`
/// is the edge whose blend was refused there.
fn extra_kind(
    m: &Model,
    edge: EdgeId,
    vertex: VertexId,
    blended: EdgeId,
    corners: &[EdgeId],
) -> &'static str {
    let seam = m
        .edge_uses(edge)
        .is_ok_and(|u| u.len() == 2 && u[0].face == u[1].face);
    if seam {
        return "a seam";
    }
    let Some(t) = tangent_at_vertex(m, edge, vertex) else {
        return "a degenerate edge";
    };
    let continues = tangent_at_vertex(m, blended, vertex).is_some_and(|b| parallel(b, t));
    let collinear = corners
        .iter()
        .filter_map(|&c| tangent_at_vertex(m, c, vertex))
        .any(|c| parallel(c, t));
    if continues {
        "the blended edge's tangent continuation"
    } else if collinear {
        "an edge continuing a corner edge"
    } else if smooth_edge(m, edge) {
        "a smooth edge"
    } else {
        "a sharp edge between two faces across"
    }
}

/// What the second edge at a vertex of two edges is, `blended` being the
/// edge whose blend was refused there: its continuation on one curve between
/// the same two faces (a split rim's second vertex, the seam being at the
/// other), the same two faces turning, or a different pair of faces.
fn second_kind(m: &Model, other: EdgeId, vertex: VertexId, blended: EdgeId) -> &'static str {
    let faces = |e| {
        let mut f: Vec<_> = m
            .edge_uses(e)
            .map(|u| u.iter().map(|x| x.face).collect())
            .unwrap_or_default();
        f.sort();
        f.dedup();
        f
    };
    if faces(blended) != faces(other) {
        return "an edge between a different pair of faces";
    }
    let runs_on = match (
        tangent_at_vertex(m, blended, vertex),
        tangent_at_vertex(m, other, vertex),
    ) {
        (Some(a), Some(b)) => parallel(a, b),
        _ => false,
    };
    if runs_on {
        "a continuation between the same two faces"
    } else {
        "an edge between the same two faces, turning"
    }
}

/// The faces of `edge`'s uses, sorted and without repeats.
fn faces_of(m: &Model, edge: EdgeId) -> Vec<FaceId> {
    let mut f: Vec<FaceId> = m
        .edge_uses(edge)
        .map(|u| u.iter().map(|x| x.face).collect())
        .unwrap_or_default();
    f.sort();
    f.dedup();
    f
}

/// The one edge beside `edge` in `face`'s loop that also touches `vertex`.
fn next_at_vertex(m: &Model, face: FaceId, edge: EdgeId, vertex: VertexId) -> Option<EdgeId> {
    let f = m.face(face).ok()?;
    let mut found = None;
    for l in f.loops() {
        let n = l.coedges().len();
        for i in (0..n).filter(|&i| l.coedges()[i].edge() == edge) {
            for j in [(i + n - 1) % n, (i + 1) % n] {
                let c = l.coedges()[j].edge();
                let touches = m
                    .edge(c)
                    .is_ok_and(|x| x.start() == vertex || x.end() == vertex);
                if c != edge && touches && found.is_some_and(|x| x != c) {
                    return None;
                }
                if c != edge && touches {
                    found = Some(c);
                }
            }
        }
    }
    found
}

/// What the faces across a blended edge's end are at `vertex`: the one face
/// met by both corner edges (*the face across met twice*), or the faces in
/// the vertex's star from one corner edge to the other, each a piece of the
/// end (*a fan of k faces across*). `None` where the star is not one simple
/// walk between the two corner edges (ADR-0043).
fn faces_across(
    m: &Model,
    blended: EdgeId,
    vertex: VertexId,
    corners: &[EdgeId],
) -> Option<String> {
    let [c1, c2] = corners[..] else { return None };
    let blend_faces = faces_of(m, blended);
    let across = |c| match &faces_of(m, c)
        .into_iter()
        .filter(|f| !blend_faces.contains(f))
        .collect::<Vec<_>>()[..]
    {
        [f] => Some(*f),
        _ => None,
    };
    let (a1, a2) = (across(c1)?, across(c2)?);
    if a1 == a2 {
        return Some("the face across met twice".to_string());
    }
    let (mut e, mut f, mut k) = (c1, a1, 1);
    for _ in 0..m.vertex_edges(vertex).ok()?.len() {
        let next = next_at_vertex(m, f, e, vertex)?;
        if next == c2 {
            return (f == a2).then(|| format!("a fan of {k} faces across"));
        }
        f = *faces_of(m, next).iter().find(|&&x| x != f)?;
        (e, k) = (next, k + 1);
    }
    None
}

/// Which site refused a `VertexBlend` of `edge` blended alone, from the
/// entities the error names and the vertex's edges (`None` for any other
/// refusal): *a vertex of more than three edges*, with what the extra edges
/// are (sharp edges between two faces across, named by what they make of the
/// end: a fan of k faces across, or the face across met twice; a smooth edge; a seam; a
/// degenerate edge; the blended edge's tangent continuation; an edge
/// continuing a corner edge); *a corner edge with
/// no curve*; *corner edges that share no face across*; *a closed edge's
/// vertex*; *a blended edge with no curve*; or *an end that more than one
/// blended edge meets*, by its vertex's edges — the chain's junction at a
/// vertex of four edges where both faces turn is one (ADR-0039). Each is followed by the pair of surface kinds the
/// blended edge separates.
pub fn vertex_blend_cause(m: &Model, edge: EdgeId, e: &OpError) -> Option<String> {
    let OpError::Degenerate {
        entities,
        reason: Reason::VertexBlend,
    } = e
    else {
        return None;
    };
    let pair = pair_of(m, edge).unwrap_or_default();
    let ids: Vec<EntityId> = entities.iter().map(|s| s.id).collect();
    let vertex = ids.iter().find_map(|i| match i {
        EntityId::Vertex(v) => Some(*v),
        EntityId::Edge(_) | EntityId::Face(_) | EntityId::Shell(_) | EntityId::Body(_) => None,
    });
    let blended = ids
        .iter()
        .filter(|i| matches!(i, EntityId::Edge(_)))
        .count();
    let what = match (vertex, blended) {
        (None, _) => "a blended edge with no curve".to_string(),
        (Some(v), n) if n > 1 => {
            let at = m.vertex_edges(v).map_or(0, <[EdgeId]>::len);
            format!("{n} edges of the run meet at a vertex of {at} edges")
        }
        (Some(v), _) => {
            let at = m
                .vertex_edges(v)
                .map(<[EdgeId]>::to_vec)
                .unwrap_or_default();
            let corners = corner_edges(m, edge, v);
            let closed = m.edge(edge).is_ok_and(|x| x.start() == x.end());
            if closed {
                format!("a closed edge's vertex of {} edges", at.len())
            } else if at.len() > 3 {
                let mut kinds: Vec<&str> = at
                    .iter()
                    .filter(|&&x| x != edge && !corners.contains(&x))
                    .map(|&x| extra_kind(m, x, v, edge, &corners))
                    .collect();
                kinds.sort_unstable();
                kinds.dedup();
                let sharp = kinds == ["a sharp edge between two faces across"];
                match sharp.then(|| faces_across(m, edge, v, &corners)).flatten() {
                    Some(across) => format!("a vertex of {} edges, {across}", at.len()),
                    None => format!(
                        "a vertex of {} edges, the extra: {}",
                        at.len(),
                        kinds.join(", ")
                    ),
                }
            } else if corners
                .iter()
                .any(|&c| m.edge(c).is_ok_and(|x| x.curve().is_none()))
            {
                "a corner edge with no curve".to_string()
            } else if let [other] = at
                .iter()
                .copied()
                .filter(|&x| x != edge)
                .collect::<Vec<_>>()[..]
            {
                format!(
                    "a vertex of 2 edges, the second: {}",
                    second_kind(m, edge, v, other)
                )
            } else {
                format!("corner edges that share no face across, {} edges", at.len())
            }
        }
    };
    Some(format!("{what}; edge {pair}"))
}

/// How far from the vertex, as a fraction of the tangent edge's chord, the
/// census probes a face's height above the common tangent plane.
const PROBE_STEP: f64 = 1e-2;

/// A height above the tangent plane under this fraction of the probe step
/// is a flat face's: the census names a site, so the cut is a wide one.
const FLAT: f64 = 1e-9;

/// Which way the faces of the tangent edge `w` curve at `vertex`, each the
/// side of their common tangent plane that the surface lies on a step away
/// along the plane and across `w`: `Some(true)` if opposite sides (an
/// inflection), `Some(false)` if one, `None` if either face is flat or
/// cannot be read.
fn turn_across(m: &Model, w: EdgeId, vertex: VertexId) -> Option<Option<bool>> {
    let uses = m.edge_uses(w).ok()?;
    let [a, b] = uses else { return None };
    let at = m.vertex(vertex).ok()?.point();
    let t = tangent_at_vertex(m, w, vertex)?;
    let entity = m.edge(w).ok()?;
    let chord = (m.vertex(entity.start()).ok()?.point() - m.vertex(entity.end()).ok()?.point())
        .norm()
        .max(t.norm());
    let step = PROBE_STEP * chord;
    let surface = |f| m.surface(m.face(f).ok()?.surface()).ok();
    let (sa, sb) = (surface(a.face)?, surface(b.face)?);
    let uv = sa.project(at).ok()?.uv;
    let n0 = sa.normal(uv.x, uv.y)?.into_inner();
    let across = n0.cross(&t).normalize();
    let height = |s: &arris_io::arris_check::arris_topo::arris_geom::Surface| {
        let q = at + across * step;
        let on = s.project(q).ok()?;
        Some((s.point(on.uv.x, on.uv.y) - at).dot(&n0))
    };
    let (ha, hb) = (height(sa)?, height(sb)?);
    if ha.abs() <= FLAT * step || hb.abs() <= FLAT * step {
        return Some(None);
    }
    Some(Some((ha > 0.0) != (hb > 0.0)))
}

/// The unit direction in which `edge` leaves `vertex`.
fn leaving_vertex(m: &Model, edge: EdgeId, vertex: VertexId) -> Option<Vec3> {
    let d = tangent_at_vertex(m, edge, vertex)?;
    let away = if m.edge(edge).ok()?.start() == vertex {
        d
    } else {
        -d
    };
    Some(away.normalize())
}

/// Whether, at `vertex` of three edges, `edge` and the third edge beside the
/// tangent `w` leave it the same way: the outline then doubles back on
/// itself there (a cusp) instead of running on through.
fn doubles_back(m: &Model, edge: EdgeId, w: EdgeId, vertex: VertexId) -> Option<bool> {
    let at = m.vertex_edges(vertex).ok()?;
    let [next] = at
        .iter()
        .copied()
        .filter(|&x| x != edge && x != w)
        .collect::<Vec<_>>()[..]
    else {
        return None;
    };
    let (a, b) = (
        leaving_vertex(m, edge, vertex)?,
        leaving_vertex(m, next, vertex)?,
    );
    Some(a.dot(&b) > 0.0)
}

/// Whether the two faces of the tangent edge `w` fold back at `vertex`:
/// their outward normals there opposite, both walls of a cusp on one side
/// of the face the cusp's edges share (a knife-edge sliver, material or
/// void), rather than equal, the walls on either side of it (an overhang
/// tip, ADR-0042). Each normal is its face's in its shell, whose own sense
/// in the body is the same for both and cancels. `None` where a face's
/// normal or sense cannot be read.
fn walls_fold(m: &Model, w: EdgeId, vertex: VertexId) -> Option<bool> {
    let uses = m.edge_uses(w).ok()?;
    let [a, b] = uses else { return None };
    let at = m.vertex(vertex).ok()?.point();
    let outward = |face| -> Option<Vec3> {
        let surface = m.surface(m.face(face).ok()?.surface()).ok()?;
        let uv = surface.project(at).ok()?.uv;
        let n = surface.normal(uv.x, uv.y)?.into_inner();
        let shell = *m.face_shells(face).ok()?.first()?;
        let sense = m
            .shell(shell)
            .ok()?
            .faces()
            .iter()
            .find(|f| f.id == face)?
            .orientation
            .sign();
        Some(n * sense)
    };
    Some(outward(a.face)?.dot(&outward(b.face)?) < 0.0)
}

/// Which site refused a `TangentChain` of `edge` blended alone, from the
/// entities the error names (`None` for any other refusal): *the edge
/// itself*, a tangent dihedral (the error names the edge and its two
/// faces), or *an end* at a vertex whose corner edge is tangent (the edge,
/// the corner edge and the vertex: the edge is the run's, not always the
/// one asked), by what is there — *a cusp* (the
/// blended edge and the third edge leave the vertex the same way, the
/// outline doubling back instead of running on), with its walls *on one
/// side* of the face the two edges share, which a blend of one sense is cut
/// at and is refused only where the stripe is not a ring's, or *on either
/// side*, an overhang tip (ADR-0042), else by what turns:
/// *an inflection*
/// (the two faces of the tangent edge curve opposite ways: a floor whose
/// wall is an S-bend), *a turn one way* (they curve the same way), *a flat
/// face against a curved one*, or *a turn the census cannot read*. Each is
/// followed by the pair of surface kinds the blended edge separates.
pub fn tangent_chain_cause(m: &Model, edge: EdgeId, e: &OpError) -> Option<String> {
    let OpError::Degenerate {
        entities,
        reason: Reason::TangentChain,
    } = e
    else {
        return None;
    };
    let pair = pair_of(m, edge).unwrap_or_default();
    let ids: Vec<EntityId> = entities.iter().map(|s| s.id).collect();
    let what = match ids.as_slice() {
        [EntityId::Edge(_), EntityId::Face(_), EntityId::Face(_)] => {
            "the edge itself, a tangent dihedral".to_string()
        }
        [EntityId::Edge(run), EntityId::Edge(w), EntityId::Vertex(v)] => {
            if doubles_back(m, *run, *w, *v) == Some(true) {
                match walls_fold(m, *w, *v) {
                    Some(true) => "an end at a cusp, walls on one side".to_string(),
                    Some(false) => "an end at a cusp, walls on either side".to_string(),
                    None => "an end at a cusp".to_string(),
                }
            } else {
                match turn_across(m, *w, *v) {
                    Some(Some(true)) => "an end at an inflection".to_string(),
                    Some(Some(false)) => "an end at a turn one way".to_string(),
                    Some(None) => "an end at a flat face against a curved one".to_string(),
                    None => "an end at a turn the census cannot read".to_string(),
                }
            }
        }
        _ => "another site".to_string(),
    };
    Some(format!("{what}; edge {pair}"))
}

/// The miter or corner a `VertexBlend` of a set of edges blended together
/// names (`None` for any other refusal): how many blended edges meet at
/// how many edges, and the kinds of face pair they separate.
pub fn vertex_set_cause(m: &Model, e: &OpError) -> Option<String> {
    let OpError::Degenerate {
        entities,
        reason: Reason::VertexBlend,
    } = e
    else {
        return None;
    };
    let mut edges = Vec::new();
    let mut vertex = None;
    for s in entities {
        match s.id {
            EntityId::Edge(x) => edges.push(x),
            EntityId::Vertex(v) => vertex = Some(v),
            EntityId::Face(_) | EntityId::Shell(_) | EntityId::Body(_) => {}
        }
    }
    let mut pairs: Vec<String> = edges.iter().filter_map(|&x| pair_of(m, x)).collect();
    pairs.sort();
    pairs.dedup();
    let at = vertex
        .and_then(|v| m.vertex_edges(v).ok().map(<[EdgeId]>::len))
        .map_or_else(String::new, |n| format!(" at a vertex of {n} edges"));
    Some(format!(
        "{} blended edges{at}; {}",
        edges.len(),
        pairs.join(" / ")
    ))
}

/// Puts a sample of every cause's `VertexBlend` edges of `census`, a solid
/// of `file`, to Open CASCADE alone at the census's radius, as
/// [`ask_the_oracle`] does for `BlendTooLarge`, and the battery's sample
/// together where that is refused `VertexBlend`.
///
/// # Errors
/// As [`ask_the_oracle`].
pub fn ask_the_oracle_vertex(file: &Path, census: &mut SolidCensus) -> Result<(), String> {
    let id = solid_id(&census.solid)?;
    let asked = stride_per_cause(&census.vertex_blend);
    if !asked.is_empty() {
        let points: Vec<[f64; 3]> = asked.iter().map(|&i| census.vertex_blend[i].at).collect();
        let verdicts = crate::oracle::fillet_edges(file, id, census.radius, ORACLE_PROBE, &points)
            .map_err(|e| e.to_string())?;
        for (&i, v) in asked.iter().zip(verdicts) {
            census.vertex_blend[i].occt = Some(v.verdict);
        }
    }
    let radius = census.radius;
    if let Some(set) = &mut census.vertex_set {
        let verdicts = crate::oracle::fillet_edge_sets(
            file,
            id,
            radius,
            ORACLE_PROBE,
            std::slice::from_ref(&set.points),
        )
        .map_err(|e| e.to_string())?;
        set.occt = verdicts.into_iter().next().map(|v| v.verdict);
    }
    Ok(())
}

fn solid_id(solid: &str) -> Result<u64, String> {
    solid
        .trim_start_matches('#')
        .split('[')
        .next()
        .and_then(|s| s.parse().ok())
        .ok_or_else(|| format!("{solid}: not a solid key"))
}

/// The indices a stride over each cause, in id order, asks the oracle
/// about ([`ASKED_PER_CAUSE`] of each), and every sampled edge besides.
fn stride_per_cause(edges: &[RunOverEdge]) -> Vec<usize> {
    let mut by_cause: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
    for (i, e) in edges.iter().enumerate() {
        by_cause.entry(&e.cause).or_default().push(i);
    }
    let mut asked: Vec<usize> = Vec::new();
    for list in by_cause.values() {
        let stride = list.len().div_ceil(ASKED_PER_CAUSE).max(1);
        asked.extend(list.iter().step_by(stride));
    }
    asked.extend((0..edges.len()).filter(|&i| edges[i].sampled));
    asked.sort_unstable();
    asked.dedup();
    asked
}

/// [`census_file`] of `file`, with each solid's `VertexBlend` edges and
/// sampled set put to Open CASCADE ([`ask_the_oracle_vertex`]).
///
/// # Errors
/// As [`census_file`] and [`ask_the_oracle_vertex`].
pub fn vertex_blend_file(
    file: &Path,
    wanted: &dyn Fn(&str) -> bool,
) -> Result<Vec<SolidCensus>, String> {
    let mut solids = census_file(file, wanted)?;
    for s in &mut solids {
        ask_the_oracle_vertex(file, s)?;
    }
    Ok(solids)
}

/// [`committed`] with each solid's `VertexBlend` edges and sampled set put
/// to Open CASCADE.
///
/// # Errors
/// As [`committed`] and [`ask_the_oracle_vertex`].
pub fn vertex_blend_committed() -> Result<Vec<(String, Vec<SolidCensus>)>, String> {
    let mut out = Vec::new();
    for name in COMMITTED_TIER {
        let fixture = part::load(&corpus_root().join(name)).map_err(|e| e.to_string())?;
        let refused: Vec<String> = (fixture.part.solids.iter())
            .filter(|s| matches!(s.battery.get("fillet"), Some(Class::ArrisRefuses(_))))
            .map(|s| crate::battery::key(s.id, s.instance))
            .collect();
        let file = fixture.dir.join(&fixture.part.file);
        let part = name.rsplit('/').next().unwrap_or(name).to_string();
        out.push((
            part,
            vertex_blend_file(&file, &|k| refused.iter().any(|r| r == k))?,
        ));
    }
    Ok(out)
}

/// The `VertexBlend` edges of the solids, counted by `part → cause`, and
/// each part's sampled set where that is refused: how many edges, how many
/// were put to Open CASCADE and what it said.
pub fn vertex_blend_markdown(parts: &[(String, Vec<SolidCensus>)]) -> String {
    let mut out = String::from(
        "| part | solid | cause | edges | asked | builds | invalid | refuses | no edge |\n|---|---|---|---:|---:|---:|---:|---:|---:|\n",
    );
    let mut totals = [0usize; 6];
    for (part, solids) in parts {
        for s in solids {
            let mut by_cause: BTreeMap<&str, Vec<&RunOverEdge>> = BTreeMap::new();
            for e in &s.vertex_blend {
                by_cause.entry(&e.cause).or_default().push(e);
            }
            for (cause, edges) in by_cause {
                let said = |v: &str| {
                    edges
                        .iter()
                        .filter(|e| e.occt.as_deref() == Some(v))
                        .count()
                };
                let asked = edges.iter().filter(|e| e.occt.is_some()).count();
                let row = [
                    edges.len(),
                    asked,
                    said("builds"),
                    said("invalid"),
                    said("refuses"),
                    said("no-edge"),
                ];
                for (t, r) in totals.iter_mut().zip(row) {
                    *t += r;
                }
                let _ = writeln!(
                    out,
                    "| {part} | {} | {cause} | {} | {} | {} | {} | {} | {} |",
                    s.solid, row[0], row[1], row[2], row[3], row[4], row[5]
                );
            }
        }
    }
    out.push_str("\nThe battery's sampled edges refused `VertexBlend`, each alone:\n\n");
    for (part, solids) in parts {
        for s in solids {
            for e in s.vertex_blend.iter().filter(|e| e.sampled) {
                let _ = writeln!(
                    out,
                    "- {part} {}: {} → Open CASCADE {}",
                    s.solid,
                    e.cause,
                    e.occt.as_deref().unwrap_or("not asked")
                );
            }
        }
    }
    out.push_str("\nThe battery's sample blended together, refused `VertexBlend`:\n\n");
    for (part, solids) in parts {
        for s in solids {
            if let Some(set) = &s.vertex_set {
                let _ = writeln!(
                    out,
                    "- {part} {}: {} ({}) → Open CASCADE {}",
                    s.solid,
                    set.cause,
                    if set.each_alone_builds {
                        "each edge alone builds"
                    } else {
                        "an edge alone is refused too"
                    },
                    set.occt.as_deref().unwrap_or("not asked")
                );
            }
        }
    }
    let _ = writeln!(
        out,
        "\nTotal: {} `VertexBlend` edges, {} put to Open CASCADE: {} build, {} build invalid, {} refused, {} found no edge.",
        totals[0], totals[1], totals[2], totals[3], totals[4], totals[5]
    );
    out
}

/// How many edges of one (solid, cause) [`ask_the_oracle`] puts to Open
/// CASCADE: a stride over them in id order. A cause with fewer is asked
/// whole.
pub const ASKED_PER_CAUSE: usize = 12;

/// How close to an edge a census point must lie for Open CASCADE to name
/// that edge by it: the midpoints agree to the file's own digits, and the
/// edges of a part are far further apart.
pub const ORACLE_PROBE: f64 = 1e-4;

/// Puts a sample of every cause's `BlendTooLarge` edges of `census`, a
/// solid of `file`, to Open CASCADE alone at the census's radius, and
/// records each verdict in `occt`. At most [`ASKED_PER_CAUSE`] per cause,
/// and every edge of the battery's sample besides.
///
/// # Errors
/// The solid's key is not `#id[instance]`, or the oracle could not run.
pub fn ask_the_oracle(file: &Path, census: &mut SolidCensus) -> Result<(), String> {
    let id = solid_id(&census.solid)?;
    let asked = stride_per_cause(&census.too_large);
    if asked.is_empty() {
        return Ok(());
    }
    let points: Vec<[f64; 3]> = asked.iter().map(|&i| census.too_large[i].at).collect();
    let verdicts = crate::oracle::fillet_edges(file, id, census.radius, ORACLE_PROBE, &points)
        .map_err(|e| e.to_string())?;
    for (&i, v) in asked.iter().zip(verdicts) {
        census.too_large[i].occt = Some(v.verdict);
    }
    Ok(())
}

/// Puts a sample of every site's `TangentChain` edges of `census`, a solid
/// of `file`, to Open CASCADE alone at the census's radius, as
/// [`ask_the_oracle`] does for `BlendTooLarge`.
///
/// # Errors
/// As [`ask_the_oracle`].
pub fn ask_the_oracle_tangent(file: &Path, census: &mut SolidCensus) -> Result<(), String> {
    let id = solid_id(&census.solid)?;
    let asked = stride_per_cause(&census.tangent_chain);
    if asked.is_empty() {
        return Ok(());
    }
    let points: Vec<[f64; 3]> = asked.iter().map(|&i| census.tangent_chain[i].at).collect();
    let verdicts = crate::oracle::fillet_edges(file, id, census.radius, ORACLE_PROBE, &points)
        .map_err(|e| e.to_string())?;
    for (&i, v) in asked.iter().zip(verdicts) {
        census.tangent_chain[i].occt = Some(v.verdict);
    }
    Ok(())
}

/// [`census_file`] of `file`, with each solid's `TangentChain` edges put to
/// Open CASCADE ([`ask_the_oracle_tangent`]), printed by site and verdict.
///
/// # Errors
/// As [`census_file`] and [`ask_the_oracle_tangent`].
pub fn tangent_chain_file(file: &Path) -> Result<String, String> {
    let mut out = String::new();
    for mut s in census_file(file, &|_| true)? {
        ask_the_oracle_tangent(file, &mut s)?;
        let mut by: BTreeMap<(&str, &str), usize> = BTreeMap::new();
        for e in &s.tangent_chain {
            *by.entry((&e.cause, e.occt.as_deref().unwrap_or("not asked")))
                .or_default() += 1;
        }
        for e in s.tangent_chain.iter().filter(|e| e.sampled) {
            let _ = writeln!(
                out,
                "- sampled {:?} at {:?}: {} → Open CASCADE {}",
                e.edge,
                e.at,
                e.cause,
                e.occt.as_deref().unwrap_or("not asked")
            );
        }
        for ((cause, verdict), n) in by {
            let _ = writeln!(
                out,
                "- {} {}: {n} × {cause} → Open CASCADE {verdict}",
                file.display(),
                s.solid
            );
        }
    }
    Ok(out)
}

/// The `BlendTooLarge` edges of the solids, counted by `part → cause`:
/// how many edges, how many were put to Open CASCADE and what it said.
pub fn run_over_markdown(parts: &[(String, Vec<SolidCensus>)]) -> String {
    let mut out = String::from(
        "| part | solid | cause | edges | asked | builds | invalid | refuses | no edge |\n|---|---|---|---:|---:|---:|---:|---:|---:|\n",
    );
    let mut totals = [0usize; 6];
    for (part, solids) in parts {
        for s in solids {
            let mut by_cause: BTreeMap<&str, Vec<&RunOverEdge>> = BTreeMap::new();
            for e in &s.too_large {
                by_cause.entry(&e.cause).or_default().push(e);
            }
            for (cause, edges) in by_cause {
                let said = |v: &str| {
                    edges
                        .iter()
                        .filter(|e| e.occt.as_deref() == Some(v))
                        .count()
                };
                let asked = edges.iter().filter(|e| e.occt.is_some()).count();
                let row = [
                    edges.len(),
                    asked,
                    said("builds"),
                    said("invalid"),
                    said("refuses"),
                    said("no-edge"),
                ];
                for (t, r) in totals.iter_mut().zip(row) {
                    *t += r;
                }
                let _ = writeln!(
                    out,
                    "| {part} | {} | {cause} | {} | {} | {} | {} | {} | {} |",
                    s.solid, row[0], row[1], row[2], row[3], row[4], row[5]
                );
            }
        }
    }
    out.push_str("\nThe battery's sampled edges refused `BlendTooLarge`, each alone:\n\n");
    for (part, solids) in parts {
        for s in solids {
            for e in s.too_large.iter().filter(|e| e.sampled) {
                let _ = writeln!(
                    out,
                    "- {part} {}: {} → Open CASCADE {}",
                    s.solid,
                    e.cause,
                    e.occt.as_deref().unwrap_or("not asked")
                );
            }
        }
    }
    let _ = writeln!(
        out,
        "\nTotal: {} `BlendTooLarge` edges, {} put to Open CASCADE: {} build, {} build invalid, {} refused, {} found no edge.",
        totals[0], totals[1], totals[2], totals[3], totals[4], totals[5]
    );
    out
}

/// Reads the STEP file `file` and counts the first placement of each
/// solid Arris reads whose key (`#id[instance]`) `wanted` accepts. A solid the reader refuses has no row: the
/// histogram's `read` column holds it.
///
/// # Errors
/// The file cannot be read or has no model to read into.
pub fn census_file(file: &Path, wanted: &dyn Fn(&str) -> bool) -> Result<Vec<SolidCensus>, String> {
    let bytes = std::fs::read(file).map_err(|e| format!("{}: {e}", file.display()))?;
    let mut m = Model::new(PrecisionSpec::default().precision()).map_err(|e| e.to_string())?;
    let read = step_read(
        &mut m,
        &String::from_utf8_lossy(&bytes),
        &ReadOptions::default(),
    )
    .map_err(|e| format!("{}: {e}", file.display()))?;
    let mut seen = Vec::new();
    let mut out = Vec::new();
    for s in &read.solids {
        let Ok(back) = &s.result else { continue };
        if seen.contains(&s.entity.id) {
            continue;
        }
        seen.push(s.entity.id);
        let key = crate::battery::key(s.entity.id, s.entity.instance);
        if !wanted(&key) {
            continue;
        }
        out.push(blend_census(&m, back.body, &key)?);
    }
    Ok(out)
}

/// The committed tier's fillet column counted: every solid whose fillet
/// stage the fixture records as refused by Arris
/// ([`Class::ArrisRefuses`]), by part.
///
/// # Errors
/// A fixture or its file cannot be read.
pub fn committed() -> Result<Vec<(String, Vec<SolidCensus>)>, String> {
    let mut out = Vec::new();
    for name in COMMITTED_TIER {
        let fixture = part::load(&corpus_root().join(name)).map_err(|e| e.to_string())?;
        let refused: Vec<String> = (fixture.part.solids.iter())
            .filter(|s| matches!(s.battery.get("fillet"), Some(Class::ArrisRefuses(_))))
            .map(|s| crate::battery::key(s.id, s.instance))
            .collect();
        let file = fixture.dir.join(&fixture.part.file);
        let part = name.rsplit('/').next().unwrap_or(name).to_string();
        out.push((
            part,
            census_file(&file, &|k| refused.iter().any(|r| r == k))?,
        ));
    }
    Ok(out)
}

/// [`census_file`] of `file`, and each solid's `BlendTooLarge` edges put to
/// Open CASCADE ([`ask_the_oracle`]).
///
/// # Errors
/// As [`census_file`] and [`ask_the_oracle`].
pub fn run_over_file(
    file: &Path,
    wanted: &dyn Fn(&str) -> bool,
) -> Result<Vec<SolidCensus>, String> {
    let mut solids = census_file(file, wanted)?;
    for s in &mut solids {
        ask_the_oracle(file, s)?;
    }
    Ok(solids)
}

/// [`committed`] with each solid's `BlendTooLarge` edges put to Open CASCADE.
///
/// # Errors
/// As [`committed`] and [`ask_the_oracle`].
pub fn run_over_committed() -> Result<Vec<(String, Vec<SolidCensus>)>, String> {
    let mut out = Vec::new();
    for name in COMMITTED_TIER {
        let fixture = part::load(&corpus_root().join(name)).map_err(|e| e.to_string())?;
        let refused: Vec<String> = (fixture.part.solids.iter())
            .filter(|s| matches!(s.battery.get("fillet"), Some(Class::ArrisRefuses(_))))
            .map(|s| crate::battery::key(s.id, s.instance))
            .collect();
        let file = fixture.dir.join(&fixture.part.file);
        let part = name.rsplit('/').next().unwrap_or(name).to_string();
        out.push((
            part,
            run_over_file(&file, &|k| refused.iter().any(|r| r == k))?,
        ));
    }
    Ok(out)
}

/// The table `part → first refusal → what stands behind it`, markdown: one
/// row per solid whose fillet stage is refused, the refusals of the other
/// edges beside the count of edges each holds, and a last line counting
/// the parts by first refusal.
pub fn markdown(parts: &[(String, Vec<SolidCensus>)]) -> String {
    let mut out = String::from(
        "| part | solid | radius | first refusal (the sample together) | cycle | the sampled edges, each alone | each blendable edge alone |\n|---|---|---:|---|---|---|---|\n",
    );
    let mut by_first: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for (part, solids) in parts {
        for s in solids {
            let Some(first) = &s.sample else { continue };
            if first == BUILT {
                continue;
            }
            let mut alone: Vec<_> = s.alone.iter().collect();
            alone.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
            let alone = alone
                .iter()
                .map(|(c, n)| format!("{c} {n}"))
                .collect::<Vec<_>>()
                .join("; ");
            let cycle = s.blocks.clone().unwrap_or_default();
            let _ = writeln!(
                out,
                "| {part} | {} | {:.4} | {first} | {cycle} | {} | {alone} |",
                s.solid,
                s.radius,
                s.sampled.join("; ")
            );
            let list = by_first.entry(first.clone()).or_default();
            if !list.contains(part) {
                list.push(part.clone());
            }
        }
    }
    out.push_str("\nParts by first refusal:\n\n");
    let mut rows: Vec<_> = by_first.iter().collect();
    rows.sort_by(|a, b| b.1.len().cmp(&a.1.len()).then(a.0.cmp(b.0)));
    for (first, list) in rows {
        let _ = writeln!(out, "- {first}: {} ({})", list.len(), list.join(", "));
    }
    out.push_str(&tangent_chain_section(parts));
    out
}

/// The `TangentChain` edges of the solids, each alone, counted by
/// `part → site` ([`tangent_chain_cause`]); empty where there are none.
fn tangent_chain_section(parts: &[(String, Vec<SolidCensus>)]) -> String {
    let mut out = String::new();
    for (part, solids) in parts {
        for s in solids {
            let mut by_cause: BTreeMap<&str, usize> = BTreeMap::new();
            for e in &s.tangent_chain {
                *by_cause.entry(&e.cause).or_default() += 1;
            }
            for (cause, n) in by_cause {
                let _ = writeln!(out, "- {part} {}: {n} × {cause}", s.solid);
            }
        }
    }
    if out.is_empty() {
        out
    } else {
        format!("\n`TangentChain` edges, each alone, by site:\n\n{out}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::unmetered::{fuse, primitive_box, primitive_cylinder};
    use arris_io::arris_check::arris_topo::arris_math::Point3 as P3;
    use arris_io::arris_check::arris_topo::arris_math::{Axis, Point3, Vec3};

    #[test]
    fn a_box_blends_on_every_edge() {
        let mut m = Model::default();
        let (b, _) = primitive_box(&mut m, Point3::origin(), Point3::new(2.0, 3.0, 4.0)).unwrap();
        let c = blend_census(&m, b, "box").unwrap();
        assert_eq!(c.sample.as_deref(), Some(BUILT));
        assert_eq!(c.blocks, None);
        assert_eq!(c.alone, BTreeMap::from([(BUILT.to_string(), 12)]));
    }

    #[test]
    fn crossing_cylinders_are_refused_by_their_pair() {
        let mut m = Model::default();
        let (a, _) =
            primitive_cylinder(&mut m, Axis::z_at(Point3::new(0.0, 0.0, -2.0)), 1.0, 4.0).unwrap();
        let along_x = Axis::new(Point3::new(-2.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let (b, _) = primitive_cylinder(&mut m, along_x, 0.6, 4.0).unwrap();
        let (cross, _) = fuse(&mut m, a, b).unwrap();
        let c = blend_census(&m, cross, "cross").unwrap();
        let pair = "Unsupported pair: cylinder surface × cylinder surface";
        assert!(
            c.alone.get(pair).copied().unwrap_or(0) >= 1,
            "the quartic edge is the pair's: {:?}",
            c.alone
        );
        assert!(!c.alone.contains_key(PANICKED), "{:?}", c.alone);
        let table = markdown(&[("cross".to_string(), vec![c.clone()])]);
        if let Some(first) = &c.sample {
            if first != BUILT {
                assert!(table.contains(first), "{table}");
            }
        }
    }

    /// A box fillet at a radius past its faces is refused at its corner
    /// edge, named with the pair its edge separates; a radius that builds
    /// is no refusal and has no cause, and nor is another reason.
    #[test]
    fn a_radius_past_the_faces_meets_the_corner_edge() {
        let mut m = Model::default();
        let (b, _) = primitive_box(&mut m, P3::origin(), P3::new(2.0, 3.0, 4.0)).unwrap();
        let handles = m.edges(b).unwrap();
        let edge = handles[0];
        let mut scratch = m.clone();
        let refused = match blend(&mut scratch, b, &[edge], 5.0) {
            Err(Refused::Op(e)) => e,
            other => panic!(
                "a radius of 5 on a 2 × 3 × 4 box is refused: {}",
                other.is_ok()
            ),
        };
        let cause = run_over_cause(&m, edge.id, &refused).expect("a BlendTooLarge");
        assert_eq!(
            cause,
            "a corner edge shorter than the trim; edge plane × plane"
        );
        let mut scratch = m.clone();
        assert!(blend(&mut scratch, b, &[edge], 0.1).is_ok());
        let other = OpError::Degenerate {
            entities: Vec::new(),
            reason: Reason::NoEdges,
        };
        assert_eq!(run_over_cause(&m, edge.id, &other), None);
    }

    /// Each shape of entity list is its own site.
    #[test]
    fn the_entities_a_refusal_names_decide_its_site() {
        use arris_io::arris_check::arris_topo::{Orientation, Shape};
        let mut m = Model::default();
        let (b, _) = primitive_box(&mut m, P3::origin(), P3::new(2.0, 3.0, 4.0)).unwrap();
        let handles = m.edges(b).unwrap();
        let edge = handles[0].id;
        let uses = m.edge_uses(edge).unwrap().to_vec();
        let e = |id: EdgeId| Shape::new(id, Orientation::Forward);
        let f = |u: usize| Shape::new(uses[u].face, Orientation::Forward);
        let cause = |entities: Vec<Shape>| {
            let err = OpError::Degenerate {
                entities,
                reason: Reason::BlendTooLarge,
            };
            run_over_cause(&m, edge, &err).unwrap()
        };
        let pair = "edge plane × plane";
        assert_eq!(
            cause(vec![e(edge), f(0)]),
            format!("the contact leaves a plane face; {pair}")
        );
        assert_eq!(
            cause(vec![e(edge), f(0), f(1)]),
            format!("the ball finds no place on either face; {pair}")
        );
        assert_eq!(
            cause(vec![e(edge), e(handles[1].id)]),
            format!("a corner edge shorter than the trim; {pair}")
        );
        assert_eq!(
            cause(vec![e(edge), e(handles[1].id), e(handles[2].id)]),
            format!("a corner whose trims run past one another; {pair}")
        );
        assert_eq!(
            cause(vec![e(edge)]),
            format!("the closing or the fit; {pair}")
        );
    }

    /// The census's table keeps a cause apart from its pair and totals
    /// what the oracle said.
    #[test]
    fn the_run_over_table_counts_by_cause_and_verdict() {
        let edge = |cause: &str, occt: Option<&str>| RunOverEdge {
            edge: "e".into(),
            at: [0.0; 3],
            cause: cause.into(),
            sampled: false,
            occt: occt.map(str::to_string),
        };
        let solid = SolidCensus {
            solid: "#1[0]".into(),
            radius: 1.0,
            sample: None,
            blocks: None,
            sampled: Vec::new(),
            alone: BTreeMap::new(),
            vertex_blend: Vec::new(),
            vertex_set: None,
            tangent_chain: Vec::new(),
            too_large: vec![
                edge("a; edge plane × plane", Some("builds")),
                edge("a; edge plane × plane", Some("refuses")),
                edge("a; edge plane × plane", None),
                edge("b; edge cone × plane", Some("refuses")),
            ],
        };
        let table = run_over_markdown(&[("p".into(), vec![solid])]);
        assert!(
            table.contains("| p | #1[0] | a; edge plane × plane | 3 | 2 | 1 | 0 | 1 | 0 |"),
            "{table}"
        );
        assert!(
            table.contains("| p | #1[0] | b; edge cone × plane | 1 | 1 | 0 | 0 | 1 | 0 |"),
            "{table}"
        );
        assert!(table.contains("Total: 4 `BlendTooLarge` edges, 3 put to Open CASCADE: 1 build, 0 build invalid, 2 refused"), "{table}");
    }

    /// The fixture `blend/five-edge-vertex`: a rise ending at a vertex of
    /// five edges, the two extra being the sharp halves of the top edge.
    #[test]
    fn a_vertex_of_five_edges_is_named_by_what_the_extra_edges_are() {
        use crate::unmetered::transform;
        use arris_io::arris_check::arris_topo::arris_math::Isometry;
        use arris_io::arris_check::arris_topo::arris_math::nalgebra::UnitQuaternion;
        let mut m = Model::default();
        let (lower, _) = primitive_box(&mut m, P3::origin(), P3::new(2.0, 2.0, 2.0)).unwrap();
        let (unit, _) = primitive_box(&mut m, P3::origin(), P3::new(1.0, 1.0, 1.0)).unwrap();
        let turn = UnitQuaternion::from_axis_angle(&Vec3::z_axis(), -135f64.to_radians());
        let motion = Isometry::new(turn, Vec3::new(1.0, 2.0, 2.0));
        let (upper, _) = transform(&mut m, unit, &motion).unwrap();
        let (fused, _) = fuse(&mut m, lower, upper).unwrap();
        let rise = m
            .edges(fused)
            .unwrap()
            .into_iter()
            .find(|h| {
                edge_midpoint(&m, h.id).is_some_and(|p| (p - P3::new(1.0, 2.0, 2.5)).norm() < 1e-9)
            })
            .expect("the rise at (1, 2, 2.5)");
        let mut scratch = m.clone();
        let Err(Refused::Op(e)) = blend(&mut scratch, fused, &[rise], 0.1) else {
            panic!("a fillet into a five-edge vertex is refused");
        };
        assert_eq!(
            vertex_blend_cause(&m, rise.id, &e).unwrap(),
            "a vertex of 5 edges, the face across met twice; edge plane × plane"
        );
        assert_eq!(run_over_cause(&m, rise.id, &e), None);
        let set = vertex_set_cause(&m, &e).unwrap();
        assert!(
            set.starts_with("1 blended edges at a vertex of 5 edges"),
            "{set}"
        );
    }

    /// The fixture `blend/chamfered-stadium-foot-fillet`: the chamfer's
    /// foot runs on into the half cone's at a vertex of four edges where
    /// both faces turn (ADR-0039), and the junction there builds. A
    /// junction refused at such a vertex, naming the run's two edges and
    /// the vertex, is named by them.
    #[test]
    fn a_junction_at_a_vertex_of_four_edges_is_named_by_the_run() {
        use crate::unmetered::{chamfer, extrude};
        use arris_io::arris_check::arris_topo::arris_geom::{Profile, ProfileLoop, ProfileSegment};
        use arris_io::arris_check::arris_topo::arris_math::{Frame, Point2};
        let mut m = Model::default();
        let p = |u, v| Point2::new(u, v);
        let profile = Profile {
            plane: Frame::world(),
            outer: ProfileLoop::Path {
                start: p(0.0, -1.0),
                segments: vec![
                    ProfileSegment::LineTo(p(2.0, -1.0)),
                    ProfileSegment::ArcTo {
                        to: p(2.0, 1.0),
                        via: p(3.0, 0.0),
                    },
                    ProfileSegment::LineTo(p(0.0, 1.0)),
                    ProfileSegment::ArcTo {
                        to: p(0.0, -1.0),
                        via: p(-1.0, 0.0),
                    },
                ],
            },
            holes: Vec::new(),
        };
        let (stadium, _) = extrude(&mut m, &profile, Vec3::z(), 1.0).unwrap();
        let at = |m: &Model, body, q: P3| {
            m.edges(body)
                .unwrap()
                .into_iter()
                .find(|h| edge_midpoint(m, h.id).is_some_and(|p| (p - q).norm() < 1e-9))
                .unwrap()
        };
        let top = at(&m, stadium, P3::new(1.0, 1.0, 1.0));
        let (chamfered, _) = chamfer(&mut m, stadium, &[top], 0.25).unwrap();
        let foot = at(&m, chamfered, P3::new(1.0, 1.0, 0.75));
        let mut scratch = m.clone();
        assert!(blend(&mut scratch, chamfered, &[foot], 0.1).is_ok());
        // The foot line's end at the corner (0, 1), and the foot arc there.
        let corner = m
            .vertices(chamfered)
            .unwrap()
            .into_iter()
            .find(|v| (m.vertex(v.id).unwrap().point() - P3::new(0.0, 1.0, 0.75)).norm() < 1e-9)
            .unwrap();
        let arc = at(&m, chamfered, P3::new(-1.0, 0.0, 0.75));
        use arris_io::arris_check::arris_topo::{Orientation, Shape};
        let e = OpError::Degenerate {
            entities: vec![
                Shape::new(foot.id, Orientation::Forward),
                Shape::new(arc.id, Orientation::Forward),
                Shape::new(corner.id, Orientation::Forward),
            ],
            reason: Reason::VertexBlend,
        };
        assert_eq!(
            vertex_blend_cause(&m, foot.id, &e).unwrap(),
            "2 edges of the run meet at a vertex of 4 edges; edge plane × plane"
        );
    }

    /// A refusal that is not `VertexBlend` has no site here, and a set's
    /// miter is named by how many edges meet and what they separate.
    #[test]
    fn only_a_vertex_blend_has_a_vertex_site() {
        use arris_io::arris_check::arris_topo::{Orientation, Shape};
        let mut m = Model::default();
        let (b, _) = primitive_box(&mut m, P3::origin(), P3::new(2.0, 3.0, 4.0)).unwrap();
        let handles = m.edges(b).unwrap();
        let other = OpError::Degenerate {
            entities: Vec::new(),
            reason: Reason::BlendTooLarge,
        };
        assert_eq!(vertex_blend_cause(&m, handles[0].id, &other), None);
        assert_eq!(vertex_set_cause(&m, &other), None);
        let v = m.edge(handles[0].id).unwrap().start();
        let shape = |id: EdgeId| Shape::new(id, Orientation::Forward);
        let miter = OpError::Degenerate {
            entities: vec![
                shape(handles[0].id),
                shape(handles[1].id),
                Shape::new(v, Orientation::Forward),
            ],
            reason: Reason::VertexBlend,
        };
        assert_eq!(
            vertex_set_cause(&m, &miter).unwrap(),
            "2 blended edges at a vertex of 3 edges; plane × plane"
        );
        let curveless = OpError::Degenerate {
            entities: vec![shape(handles[0].id)],
            reason: Reason::VertexBlend,
        };
        assert_eq!(
            vertex_blend_cause(&m, handles[0].id, &curveless).unwrap(),
            "a blended edge with no curve; edge plane × plane"
        );
    }

    /// The table keeps the sites apart and totals Open CASCADE's verdicts,
    /// and says what the sample together was.
    #[test]
    fn the_vertex_blend_table_counts_by_site_and_verdict() {
        let edge = |cause: &str, occt: Option<&str>| RunOverEdge {
            edge: "e".into(),
            at: [0.0; 3],
            cause: cause.into(),
            sampled: false,
            occt: occt.map(str::to_string),
        };
        let solid = SolidCensus {
            solid: "#1[0]".into(),
            radius: 1.0,
            sample: None,
            blocks: None,
            sampled: Vec::new(),
            alone: BTreeMap::new(),
            too_large: Vec::new(),
            tangent_chain: Vec::new(),
            vertex_blend: vec![
                edge("a vertex of 5 edges", Some("builds")),
                edge("a vertex of 5 edges", Some("refuses")),
                edge("a vertex of 5 edges", None),
            ],
            vertex_set: Some(VertexSet {
                cause: "2 blended edges".into(),
                points: Vec::new(),
                each_alone_builds: true,
                occt: Some("builds".into()),
            }),
        };
        let table = vertex_blend_markdown(&[("p".into(), vec![solid])]);
        assert!(
            table.contains("| p | #1[0] | a vertex of 5 edges | 3 | 2 | 1 | 0 | 1 | 0 |"),
            "{table}"
        );
        assert!(
            table.contains(
                "- p #1[0]: 2 blended edges (each edge alone builds) → Open CASCADE builds"
            ),
            "{table}"
        );
        assert!(
            table.contains("Total: 3 `VertexBlend` edges, 2 put to Open CASCADE"),
            "{table}"
        );
    }

    /// The committed `nist-ftc-06`'s holes: a rim circle in two half arcs
    /// between one plane and one cylinder, the cylinder's seam at the first
    /// vertex and nothing but the two arcs at the second. The chain runs on
    /// through both (ADR-0041), so no edge of the part is refused at a
    /// vertex of two edges that continue one another, and the arc the
    /// census once named there blends.
    #[test]
    fn no_split_rims_second_vertex_is_refused() {
        let file = corpus_root().join("real/nist-ftc-06/nist_ftc_06_asme1_rd.stp");
        let bytes = std::fs::read(&file).unwrap();
        let mut m = Model::new(PrecisionSpec::default().precision()).unwrap();
        let read = step_read(
            &mut m,
            &String::from_utf8_lossy(&bytes),
            &ReadOptions::default(),
        )
        .unwrap();
        let body = read
            .solids
            .iter()
            .find_map(|s| s.result.as_ref().ok())
            .unwrap()
            .body;
        let handles = m.edges(body).unwrap();
        let mut arcs = 0;
        for h in &handles {
            let mut scratch = m.clone();
            match blend(&mut scratch, body, &[*h], 0.05) {
                Err(Refused::Op(e)) => {
                    let cause = vertex_blend_cause(&m, h.id, &e).unwrap_or_default();
                    assert!(
                        !cause.contains("a continuation between the same two faces"),
                        "{cause}"
                    );
                }
                Ok(_) => {
                    let entity = m.edge(h.id).unwrap();
                    let at = |v| m.vertex_edges(v).unwrap().len();
                    if entity.start() != entity.end()
                        && (at(entity.start()) == 2 || at(entity.end()) == 2)
                    {
                        arcs += 1;
                    }
                }
                Err(_) => {}
            }
        }
        assert!(
            arcs > 0,
            "a split rim's arc blends through its second vertex"
        );
    }

    /// The committed `nist-ftc-06`'s sampled edge, at (−86.40, 52.19,
    /// −234.95), is refused `TangentChain` at an end, and the census names
    /// what is there.
    #[test]
    fn ftc_06_sampled_edge_is_an_end_at_a_cusp() {
        let file = corpus_root().join("real/nist-ftc-06/nist_ftc_06_asme1_rd.stp");
        let bytes = std::fs::read(&file).unwrap();
        let mut m = Model::new(PrecisionSpec::default().precision()).unwrap();
        let read = step_read(
            &mut m,
            &String::from_utf8_lossy(&bytes),
            &ReadOptions::default(),
        )
        .unwrap();
        let body = read
            .solids
            .iter()
            .find_map(|s| s.result.as_ref().ok())
            .unwrap()
            .body;
        let sampled = m
            .edges(body)
            .unwrap()
            .into_iter()
            .find(|h| {
                edge_midpoint(&m, h.id)
                    .is_some_and(|p| (p - P3::new(-86.40, 52.19, -234.95)).norm() < 0.01)
            })
            .expect("the sampled edge");
        let mut scratch = m.clone();
        let Err(Refused::Op(e)) = blend(&mut scratch, body, &[sampled], 0.254) else {
            panic!("the sampled edge is refused");
        };
        assert_eq!(
            tangent_chain_cause(&m, sampled.id, &e).unwrap(),
            "an end at a cusp, walls on either side; edge cylinder × plane"
        );
    }

    /// The edge of `recipe`'s body nearest `at` blended alone at `radius`,
    /// refused `TangentChain`, and the census's cause.
    fn cusp_cause(recipe: &str, at: P3, radius: f64) -> String {
        let recipe: crate::fixtures::Recipe = serde_json::from_str(recipe).unwrap();
        let chain = crate::corpus::build("generated/cusp", &recipe).unwrap();
        let body = chain.result().unwrap();
        // The part alone, as a read part is: no operand shares its edges.
        let mut m = chain.model;
        m.retain(&[body]).unwrap();
        let edge = m
            .edges(body)
            .unwrap()
            .into_iter()
            .find(|h| edge_midpoint(&m, h.id).is_some_and(|p| (p - at).norm() < 1e-6))
            .expect("the blended edge");
        let mut scratch = m.clone();
        let Err(Refused::Op(e)) = blend(&mut scratch, body, &[edge], radius) else {
            panic!("the edge is refused");
        };
        tangent_chain_cause(&m, edge.id, &e).unwrap()
    }

    /// An overhang tip, the lip's underside arc against the pillar's, is a
    /// cusp with its walls on either side (ADR-0042 §6).
    #[test]
    fn an_overhang_tip_is_a_cusp_with_walls_on_either_side() {
        let dir = corpus_root().join("blend/cusp-overhang-tip-fillet");
        let mut fixture: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(dir.join("fixture.json")).unwrap())
                .unwrap();
        // The part before its fillet.
        fixture["steps"].as_array_mut().unwrap().pop();
        fixture["result"] = "part".into();
        assert_eq!(
            cusp_cause(&fixture.to_string(), P3::new(-3.0, -1.0, 1.0), 0.1),
            "an end at a cusp, walls on either side; edge cylinder × plane"
        );
    }

    /// A spandrel prism's spine, where the arc leaves the bottom edge
    /// tangent with material in the sliver between the walls, folds: its
    /// walls are on one side of the top (ADR-0042).
    #[test]
    fn a_spandrel_spine_folds() {
        let recipe = r#"{"steps": [
            {"name": "s", "op": "profile", "plane": {"origin": [0, 0, 0], "x": [1, 0, 0], "y": [0, 1, 0]},
             "outer": {"start": [0, 0], "segments": [{"line_to": [10, 0]}, {"line_to": [10, 10]},
               {"arc_to": [0, 0], "via": [7.0710678118654755, 2.9289321881345245]}]}},
            {"name": "p", "op": "extrude", "profile": "s", "direction": [0, 0, 1], "length": 2}],
            "result": "p"}"#;
        let recipe: crate::fixtures::Recipe = serde_json::from_str(recipe).unwrap();
        let chain = crate::corpus::build("generated/spandrel", &recipe).unwrap();
        let body = chain.result().unwrap();
        let m = chain.model;
        let spine = m
            .edges(body)
            .unwrap()
            .into_iter()
            .find(|h| {
                edge_midpoint(&m, h.id).is_some_and(|p| (p - P3::new(0.0, 0.0, 1.0)).norm() < 1e-6)
            })
            .expect("the spine");
        let tip = m.edge(spine.id).unwrap().start();
        assert_eq!(walls_fold(&m, spine.id, tip), Some(true));
    }

    /// The part of the fixture `dir` before its last step (the blend), the
    /// edge nearest `at` blended alone, and the census's cause.
    fn vertex_cause_of(dir: &str, at: P3, radius: f64) -> String {
        let dir = corpus_root().join(dir);
        let mut fixture: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(dir.join("fixture.json")).unwrap())
                .unwrap();
        let last = fixture["steps"].as_array_mut().unwrap().pop().unwrap();
        fixture["result"] = fixture["steps"].as_array().unwrap().last().unwrap()["name"].clone();
        assert!(last["op"] == "fillet" || last["op"] == "chamfer");
        let recipe: crate::fixtures::Recipe = serde_json::from_str(&fixture.to_string()).unwrap();
        let chain = crate::corpus::build("generated/fan", &recipe).unwrap();
        let body = chain.result().unwrap();
        let mut m = chain.model;
        m.retain(&[body]).unwrap();
        let edge = m
            .edges(body)
            .unwrap()
            .into_iter()
            .find(|h| edge_midpoint(&m, h.id).is_some_and(|p| (p - at).norm() < 1e-6))
            .expect("the blended edge");
        let mut scratch = m.clone();
        let Err(Refused::Op(e)) = blend(&mut scratch, body, &[edge], radius) else {
            panic!("the edge is refused");
        };
        vertex_blend_cause(&m, edge.id, &e).unwrap()
    }

    /// The hexagonal prism's chamfered top, the foot of a facet blended: at
    /// each end the faces across are two, separated by a sharp edge, and
    /// `blend/five-edge-vertex`'s are one face met twice (ADR-0043).
    #[test]
    fn a_fan_is_named_apart_from_the_face_across_met_twice() {
        for dir in [
            "regression/hex-chamfer-foot-fan-fillet",
            "regression/hex-chamfer-foot-fan-chamfer",
        ] {
            assert_eq!(
                vertex_cause_of(dir, P3::new(0.75, 3f64.sqrt() / 4.0, 1.8), 0.1),
                "a vertex of 4 edges, a fan of 2 faces across; edge plane × plane",
                "{dir}"
            );
        }
        assert_eq!(
            vertex_cause_of("blend/five-edge-vertex", P3::new(1.0, 2.0, 2.5), 0.1),
            "a vertex of 5 edges, the face across met twice; edge plane × plane"
        );
    }
}
