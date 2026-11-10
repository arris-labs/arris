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

use arris_geom::GeomKind;
use arris_io::step::ReadOptions;
use arris_math::{Point3, Vec3};
use arris_ops::{BlendReason, OpError, Reason};
use arris_topo::{Body, EdgeId, EntityId, FaceId, Model, VertexId};

use crate::battery::Class;
use crate::battery::{fillet_edges, fillet_radius};
use crate::differential::panicked;
use crate::fixtures::{PrecisionSpec, corpus_root};
use crate::histogram::{COMMITTED_TIER, Stage, blocks_reason_in, cylinder_pair};
use crate::part;
use crate::report::{Align, Table};
use crate::unmetered::{fillet, step_read};
use std::panic::{AssertUnwindSafe, catch_unwind};

mod cusp;
mod parallel;
mod run_over;
mod tangent;
mod tier;
mod vertex;

pub use cusp::*;
pub use parallel::*;
pub use run_over::*;
pub use tangent::*;
pub use vertex::*;

use tangent::tangent_chain_section;
use tier::over_committed_tier;

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
    edges: &[arris_topo::Edge],
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
        OpError::Degenerate { reason, .. } => format!("Degenerate: {}", reason.name()),
        OpError::Internal(_) => "Internal".to_string(),
        OpError::InvalidInput { .. } => "InvalidInput".to_string(),
        OpError::Profile(_) => "Profile".to_string(),
        OpError::Path(_) => "Path".to_string(),
        OpError::Loft(_) => "Loft".to_string(),
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
            out.blocks = blocks_reason_in(m, Stage::Fillet, &e).map(|c| c.to_string());
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
            // A cylinder pair is named by its axes.
            match cylinder_pair(m, &e) {
                Some(pair) => format!("{} ({pair})", class_of(&e)),
                None => class_of(&e),
            }
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
pub(crate) fn edge_midpoint(m: &Model, edge: EdgeId) -> Option<Point3> {
    let (c, range) = m.edge(edge).ok()?.curve()?;
    Some(m.curve(c).ok()?.point(range.lerp(0.5)))
}

/// The surface kind of face `f`.
fn face_kind(m: &Model, f: arris_topo::FaceId) -> Option<String> {
    let surface = m.surface(m.face(f).ok()?.surface()).ok()?;
    Some(surface.kind().to_string())
}

/// The two surface kinds an edge separates, sorted: the pair a blend of it
/// asks the construction for.
pub(crate) fn pair_of(m: &Model, edge: EdgeId) -> Option<String> {
    let mut kinds: Vec<String> = m
        .edge_uses(edge)
        .ok()?
        .iter()
        .filter_map(|u| face_kind(m, u.face))
        .collect();
    kinds.sort();
    Some(kinds.join(" × "))
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
    over_committed_tier(true, census_file)
}

/// The table `part → first refusal → what stands behind it`, markdown: one
/// row per solid whose fillet stage is refused, the refusals of the other
/// edges beside the count of edges each holds, and a last line counting
/// the parts by first refusal.
pub fn markdown(parts: &[(String, Vec<SolidCensus>)]) -> String {
    let mut table = Table::new([
        ("part", Align::Left),
        ("solid", Align::Left),
        ("radius", Align::Right),
        ("first refusal (the sample together)", Align::Left),
        ("cycle", Align::Left),
        ("the sampled edges, each alone", Align::Left),
        ("each blendable edge alone", Align::Left),
    ]);
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
            table.row([
                part.clone(),
                s.solid.to_string(),
                format!("{:.4}", s.radius),
                first.clone(),
                cycle,
                s.sampled.join("; "),
                alone,
            ]);
            let list = by_first.entry(first.clone()).or_default();
            if !list.contains(part) {
                list.push(part.clone());
            }
        }
    }
    let mut out = table.to_string();
    out.push_str("\nParts by first refusal:\n\n");
    let mut rows: Vec<_> = by_first.iter().collect();
    rows.sort_by(|a, b| b.1.len().cmp(&a.1.len()).then(a.0.cmp(b.0)));
    for (first, list) in rows {
        let _ = writeln!(out, "- {first}: {} ({})", list.len(), list.join(", "));
    }
    out.push_str(&tangent_chain_section(parts));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::unmetered::{fuse, primitive_box, primitive_cylinder};
    use arris_math::{Axis, Point3, Vec3};

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
    fn crossing_cylinders_blend_as_walked_stripes() {
        let mut m = Model::default();
        let (a, _) =
            primitive_cylinder(&mut m, Axis::z_at(Point3::new(0.0, 0.0, -2.0)), 1.0, 4.0).unwrap();
        let along_x = Axis::new(Point3::new(-2.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        let (b, _) = primitive_cylinder(&mut m, along_x, 0.6, 4.0).unwrap();
        let (cross, _) = fuse(&mut m, a, b).unwrap();
        let c = blend_census(&m, cross, "cross").unwrap();
        // The quartic loop is two walked stripes (ADR-0062) that meet at
        // their two vertices on the ball's arcs (plan step 5b): every
        // sampled edge builds.
        assert_eq!(c.sample.as_deref(), Some(BUILT));
        assert_eq!(c.alone, BTreeMap::from([(BUILT.to_string(), 3)]));
    }

    /// The committed CTC-01's sample builds: its crease of two cylinders
    /// whose axes cross is a walked stripe that narrows to a pole where the
    /// faces turn tangent (ADR-0062 §5), where it was refused
    /// `TangentChain`. The census names a pair of cylinders by its axes for
    /// an `Unsupported`, and the survey puts a pair of crossing axes with
    /// the NURBS cycle where a pair of parallel axes stays the blend
    /// network's.
    #[test]
    fn ctc_01s_crossing_cylinders_are_the_nurbs_cycles() {
        let fixture = part::load(&corpus_root().join("real/nist-ctc-01")).unwrap();
        let file = fixture.dir.join(&fixture.part.file);
        let text = std::fs::read_to_string(&file).unwrap();
        let mut m = Model::new(PrecisionSpec::default().precision()).unwrap();
        let read = step_read(&mut m, &text, &ReadOptions::default()).unwrap();
        let body = read.solids[0].result.as_ref().unwrap().body;
        let (_, sample) = fillet_edges(&m, body).unwrap();
        let handles = m.edges(body).unwrap();
        let chosen: Vec<_> = (sample.iter())
            .filter_map(|s| handles.iter().find(|h| h.id == s.0).copied())
            .collect();
        let mut scratch = m.clone();
        // The crease ends on a pole at the vertex where the cylinders turn
        // tangent, and on the corner its chain runs on into at the other
        // (ADR-0062).
        match blend(&mut scratch, body, &chosen, fillet_radius(&sample)) {
            Ok(_) => {}
            Err(Refused::Op(e)) => panic!("the sample is refused: {e}"),
            Err(Refused::Panic(what)) => panic!("the sample panics: {what}"),
        }

        // The pair's census name stays for the `Unsupported` that other
        // pairs raise: two parallel cylinders, built by hand.
        let mut m = Model::default();
        let (a, _) =
            primitive_cylinder(&mut m, Axis::z_at(Point3::new(0.0, 0.0, -2.0)), 1.0, 4.0).unwrap();
        let (b, _) =
            primitive_cylinder(&mut m, Axis::z_at(Point3::new(5.0, 0.0, -2.0)), 1.0, 4.0).unwrap();
        let faces = |body| {
            m.faces(body)
                .unwrap()
                .into_iter()
                .find(|f| {
                    matches!(
                        m.surface(m.face(f.id).unwrap().surface()),
                        Ok(arris_geom::Surface::Cylinder { .. })
                    )
                })
                .unwrap()
        };
        let (fa, fb) = (faces(a), faces(b));
        let shape =
            |f: arris_topo::Face| arris_topo::Shape::new(f.id, arris_topo::Orientation::Forward);
        let kind = GeomKind::Surface(arris_geom::SurfaceKind::Cylinder);
        let error = OpError::Unsupported {
            a: (kind, shape(fa)),
            b: (kind, shape(fb)),
        };
        assert_eq!(
            crate::histogram::cylinder_pair(&m, &error),
            Some(crate::histogram::CylinderPair::Parallel)
        );
        assert_eq!(
            blocks_reason_in(&m, Stage::Fillet, &error),
            Some(crate::histogram::Cycle::BlendNetwork)
        );
    }
}
