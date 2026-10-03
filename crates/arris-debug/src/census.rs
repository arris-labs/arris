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
//! [`FILLET_EDGES`]: crate::battery::FILLET_EDGES

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::Path;

use serde::{Deserialize, Serialize};

use arris_io::arris_check::arris_topo::arris_geom::GeomKind;
use arris_io::arris_check::arris_topo::{Body, EdgeId, EntityId, Model};
use arris_io::step::ReadOptions;
use arris_ops::OpError;

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
            Err(Refused::Op(e)) => class_of(&e),
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
}
