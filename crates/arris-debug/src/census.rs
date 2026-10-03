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
use arris_io::arris_check::arris_topo::{Body, EdgeId, EntityId, Model, VertexId};
use arris_io::step::ReadOptions;
use arris_ops::{OpError, Reason};

use crate::battery::Class;
use crate::battery::{blendable_edges, fillet_radius, fillet_sampled};
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
    let sample = fillet_sampled(m, body)?;
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
    let mut ids = blendable_edges(m, body)?;
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
    for id in blendable_edges(m, body)? {
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

/// What an extra edge at a vertex of more than three edges is.
fn extra_kind(m: &Model, edge: EdgeId, vertex: VertexId, corners: &[EdgeId]) -> &'static str {
    let seam = m
        .edge_uses(edge)
        .is_ok_and(|u| u.len() == 2 && u[0].face == u[1].face);
    if seam {
        return "a seam";
    }
    let Some(t) = tangent_at_vertex(m, edge, vertex) else {
        return "a degenerate edge";
    };
    let collinear = corners
        .iter()
        .filter_map(|&c| tangent_at_vertex(m, c, vertex))
        .any(|c| parallel(c, t));
    if collinear {
        "an edge continuing a corner edge"
    } else if smooth_edge(m, edge) {
        "a smooth edge"
    } else {
        "a sharp edge between two faces across"
    }
}

/// Which site refused a `VertexBlend` of `edge` blended alone, from the
/// entities the error names and the vertex's edges (`None` for any other
/// refusal): *a vertex of more than three edges*, with what the extra edges
/// are (a sharp edge between two faces across; a smooth edge; a seam; a
/// degenerate edge; an edge continuing a corner edge); *a corner edge with
/// no curve*; *corner edges that share no face across*; *a closed edge's
/// vertex*; *a blended edge with no curve*; or *an end that more than one
/// blended edge meets*. Each is followed by the pair of surface kinds the
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
        (Some(_), n) if n > 1 => format!("{n} edges of the run meet at an end"),
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
                    .map(|&x| extra_kind(m, x, v, &corners))
                    .collect();
                kinds.sort_unstable();
                kinds.dedup();
                format!(
                    "a vertex of {} edges, the extra: {}",
                    at.len(),
                    kinds.join(", ")
                )
            } else if corners
                .iter()
                .any(|&c| m.edge(c).is_ok_and(|x| x.curve().is_none()))
            {
                "a corner edge with no curve".to_string()
            } else {
                format!("corner edges that share no face across, {} edges", at.len())
            }
        }
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
    out
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
            "a vertex of 5 edges, the extra: a sharp edge between two faces across; edge plane × plane"
        );
        assert_eq!(run_over_cause(&m, rise.id, &e), None);
        let set = vertex_set_cause(&m, &e).unwrap();
        assert!(
            set.starts_with("1 blended edges at a vertex of 5 edges"),
            "{set}"
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
}
