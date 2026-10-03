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
use arris_io::arris_check::arris_topo::arris_math::Point3;
use arris_io::arris_check::arris_topo::{Body, EdgeId, EntityId, Model};
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
                if let Some(cause) = run_over_cause(m, id, &e) {
                    if let Some(at) = edge_midpoint(m, id) {
                        out.too_large.push(RunOverEdge {
                            edge: format!("{id:?}"),
                            at: [at.x, at.y, at.z],
                            cause,
                            sampled: sample.iter().any(|x| x.0 == id),
                            occt: None,
                        });
                    }
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
    let id: u64 = census
        .solid
        .trim_start_matches('#')
        .split('[')
        .next()
        .and_then(|s| s.parse().ok())
        .ok_or_else(|| format!("{}: not a solid key", census.solid))?;
    let mut by_cause: BTreeMap<String, Vec<usize>> = BTreeMap::new();
    for (i, e) in census.too_large.iter().enumerate() {
        by_cause.entry(e.cause.clone()).or_default().push(i);
    }
    let mut asked: Vec<usize> = Vec::new();
    for list in by_cause.values() {
        let stride = list.len().div_ceil(ASKED_PER_CAUSE).max(1);
        asked.extend(list.iter().step_by(stride));
    }
    asked.extend((0..census.too_large.len()).filter(|&i| census.too_large[i].sampled));
    asked.sort_unstable();
    asked.dedup();
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
}
