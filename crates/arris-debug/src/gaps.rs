//! The gap census (ADR-0058): every gap a STEP file's solids hold past the
//! tolerance a read grows silently, beside the tolerance Open CASCADE
//! carries at the same place.
//!
//! The reader refuses the first gap past its cap ([`Refusal::Gap`]).
//! Here a file is read into a model whose `max_tolerance` is raised to
//! [`CENSUS_CEILING`], so that the cap is the part's alone
//! (`arris_math::READ_GAP_FRACTION` of its size), and every vertex and edge
//! whose tolerance the read grew past the model's default ceiling is a
//! row: its file entity, the gap it measured, the pair of surfaces it
//! separates, and Open CASCADE's tolerance at the nearest vertex or edge
//! midpoint of its healed reading (`tools/oracle/occt_tolerances.py`). A
//! solid still refused names the gap the refusal carries. Nothing here is
//! a kernel operation: it measures what a healing ceiling must admit.

use std::fmt::Write as _;
use std::path::Path;

use arris_io::step::{Heal, ReadOptions, Refusal};
use arris_math::{Point3, Precision};
use arris_topo::provenance::Role;
use arris_topo::{Model, Orientation, Origin, Shape};

use crate::battery::key;
use crate::census::{edge_midpoint, pair_of};
use crate::fixtures::{PrecisionSpec, corpus_root};
use crate::oracle::{self, CarriedTolerance};

/// The `max_tolerance` the census reads under: far past any gap of the
/// corpus, so that only the part-relative cap bounds a read. A survey
/// setting, not a kernel one.
pub const CENSUS_CEILING: f64 = 1.0;

/// How near, as a fraction of the part's size, an Open CASCADE vertex or
/// edge midpoint must be to an Arris one to be the same entity.
const MATCH_FRACTION: f64 = 1e-3;

/// What kind of entity a gap is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    /// A `VERTEX_POINT`.
    Vertex,
    /// An `EDGE_CURVE`.
    Edge,
}

/// One entity whose tolerance a read grew past the default ceiling.
#[derive(Debug, Clone, PartialEq)]
pub struct Gap {
    /// Vertex or edge.
    pub kind: Kind,
    /// The file entity it was read from.
    pub file: u64,
    /// The tolerance the read gave it: the gap it measured.
    pub gap: f64,
    /// For an edge, the surface kinds it separates.
    pub surfaces: String,
    /// Open CASCADE's tolerance at the matching entity, if one carries
    /// one past the oracle's floor.
    pub occt: Option<f64>,
}

/// A solid of a file, censused.
#[derive(Debug, Clone, PartialEq)]
pub struct SolidGaps {
    /// `#id[instance]`.
    pub solid: String,
    /// The diagonal of Open CASCADE's bounding box, 0 when it has no such
    /// solid.
    pub size: f64,
    /// Open CASCADE's largest vertex and edge tolerances there.
    pub occt_max: (f64, f64),
    /// The gaps read, largest first; or the refusal that stopped the read.
    pub result: Result<Vec<Gap>, String>,
}

fn nearest(at: Point3, carried: &[CarriedTolerance], within: f64) -> Option<f64> {
    carried
        .iter()
        .map(|c| {
            (
                (at - Point3::new(c.point[0], c.point[1], c.point[2])).norm(),
                c,
            )
        })
        .filter(|(d, _)| *d <= within)
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .map(|(_, c)| c.tolerance)
}

/// Censuses the gaps of every solid of `file`, the first placement of
/// each.
///
/// # Errors
/// The file cannot be read, or the oracle cannot read it.
pub fn census_file(file: &Path) -> Result<Vec<SolidGaps>, String> {
    let bytes = std::fs::read(file).map_err(|e| format!("{}: {e}", file.display()))?;
    let precision = Precision {
        max_tolerance: CENSUS_CEILING,
        ..PrecisionSpec::default().precision()
    };
    let mut m = Model::new(precision).map_err(|e| e.to_string())?;
    let read = crate::unmetered::step_read(
        &mut m,
        &String::from_utf8_lossy(&bytes),
        // Unhealed: the census measures what healing would have to grow.
        &ReadOptions {
            heal: Heal::Off,
            ..ReadOptions::default()
        },
    )
    .map_err(|e| format!("{}: {e}", file.display()))?;
    // A file Open CASCADE's tolerances cannot be read from is censused
    // without them: the column reads `–`.
    let occt = oracle::occt_tolerances(file).unwrap_or_default();
    let default_ceiling = Precision::DEFAULT.max_tolerance;
    let mut seen = Vec::new();
    let mut out = Vec::new();
    for s in &read.solids {
        if seen.contains(&s.entity.id) {
            continue;
        }
        seen.push(s.entity.id);
        let theirs = occt.iter().find(|o| o.id == s.entity.id);
        let size = theirs.map_or(0.0, |o| o.size);
        let within = MATCH_FRACTION * size;
        let result = match &s.result {
            Err(Refusal::Gap { entity, gap, cap }) => Err(format!(
                "refused: #{entity} lies {gap:.4} off, past the cap of {cap:.4}"
            )),
            Err(other) => Err(format!("refused: {other}")),
            Ok(body) => {
                let origin = |shape: Shape| {
                    body.provenance
                        .origins(shape)
                        .into_iter()
                        .find_map(|(_, o)| match o {
                            Origin::Role(Role::File(f)) => Some(f.id),
                            _ => None,
                        })
                        .unwrap_or(0)
                };
                let mut gaps = Vec::new();
                for v in m.vertices(body.body).map_err(|e| e.to_string())? {
                    let vertex = m.vertex(v.id).map_err(|e| e.to_string())?;
                    if vertex.tolerance() > default_ceiling {
                        gaps.push(Gap {
                            kind: Kind::Vertex,
                            file: origin(Shape::new(v.id, Orientation::Forward)),
                            gap: vertex.tolerance(),
                            surfaces: String::new(),
                            occt: theirs.and_then(|o| nearest(vertex.point(), &o.vertices, within)),
                        });
                    }
                }
                for e in m.edges(body.body).map_err(|e| e.to_string())? {
                    let edge = m.edge(e.id).map_err(|e| e.to_string())?;
                    if edge.tolerance() > default_ceiling {
                        gaps.push(Gap {
                            kind: Kind::Edge,
                            file: origin(Shape::new(e.id, Orientation::Forward)),
                            gap: edge.tolerance(),
                            surfaces: pair_of(&m, e.id).unwrap_or_default(),
                            occt: edge_midpoint(&m, e.id)
                                .and_then(|p| theirs.and_then(|o| nearest(p, &o.edges, within))),
                        });
                    }
                }
                gaps.sort_by(|a, b| b.gap.total_cmp(&a.gap));
                Ok(gaps)
            }
        };
        out.push(SolidGaps {
            solid: key(s.entity.id, s.entity.instance),
            size,
            occt_max: theirs.map_or((0.0, 0.0), |o| (o.max_vertex, o.max_edge)),
            result,
        });
    }
    Ok(out)
}

/// The committed parts whose file the census reads under `--gaps-committed`:
/// those the reader refuses a gap of today.
pub const COMMITTED: [&str; 2] = ["real/nist-ctc-02", "real/slow-gap-refusal"];

/// The committed census: `(part, solids)`.
///
/// # Errors
/// As [`census_file`], or a fixture cannot be read.
pub fn committed() -> Result<Vec<(String, Vec<SolidGaps>)>, String> {
    COMMITTED
        .iter()
        .map(|name| {
            let fixture =
                crate::part::load(&corpus_root().join(name)).map_err(|e| e.to_string())?;
            let part = name.rsplit('/').next().unwrap_or(name).to_string();
            Ok((part, census_file(&fixture.dir.join(&fixture.part.file))?))
        })
        .collect()
}

/// The census as markdown: per part and solid, Open CASCADE's largest
/// tolerances and size, then one row per gap past the default ceiling.
/// Gaps are also given as a fraction of the part's size, the unit
/// `HEAL_GAP_FRACTION` is in.
pub fn markdown(parts: &[(String, Vec<SolidGaps>)]) -> String {
    let mut out = String::new();
    for (part, solids) in parts {
        for s in solids {
            if matches!(&s.result, Ok(gaps) if gaps.is_empty()) {
                continue;
            }
            let _ = writeln!(
                out,
                "### {part} {} — size {:.1}, Open CASCADE carries up to {:.4} (vertex) and {:.4} (edge)\n",
                s.solid, s.size, s.occt_max.0, s.occt_max.1
            );
            match &s.result {
                Err(why) => {
                    let _ = writeln!(out, "{why}\n");
                }
                Ok(gaps) if gaps.is_empty() => {}
                Ok(gaps) => {
                    out.push_str("| gap | of size | vertices | edges | surfaces | file entities | Open CASCADE |\n|---:|---:|---:|---:|---|---|---:|\n");
                    // One row per gap measured to the same four places:
                    // a seam of vertices and edges is one gap.
                    let mut rows: Vec<(f64, Vec<&Gap>)> = Vec::new();
                    for g in gaps {
                        match rows.last_mut() {
                            Some((gap, set)) if (g.gap - *gap).abs() < 5e-5 => set.push(g),
                            _ => rows.push((g.gap, vec![g])),
                        }
                    }
                    for (gap, set) in rows {
                        let vertices = set.iter().filter(|g| g.kind == Kind::Vertex).count();
                        let edges = set.iter().filter(|g| g.kind == Kind::Edge).count();
                        let mut surfaces: Vec<&str> = (set.iter())
                            .map(|g| g.surfaces.as_str())
                            .filter(|s| !s.is_empty())
                            .collect();
                        surfaces.sort_unstable();
                        surfaces.dedup();
                        let mut files: Vec<u64> =
                            set.iter().map(|g| g.file).filter(|&f| f != 0).collect();
                        files.sort_unstable();
                        files.dedup();
                        let shown: Vec<String> =
                            files.iter().take(3).map(|f| format!("#{f}")).collect();
                        let more = if files.len() > 3 { ", …" } else { "" };
                        let theirs: Vec<f64> = set.iter().filter_map(|g| g.occt).collect();
                        let theirs = match (
                            theirs.iter().copied().reduce(f64::min),
                            theirs.iter().copied().reduce(f64::max),
                        ) {
                            (Some(lo), Some(hi)) if (hi - lo) > 5e-5 => {
                                format!("{lo:.4} – {hi:.4}")
                            }
                            (Some(lo), _) => format!("{lo:.4}"),
                            _ => "–".to_string(),
                        };
                        let fraction = if s.size > 0.0 { gap / s.size } else { 0.0 };
                        let _ = writeln!(
                            out,
                            "| {gap:.4} | {fraction:.2e} | {vertices} | {edges} | {} | {}{more} | {theirs} |",
                            surfaces.join(", "),
                            shown.join(", "),
                        );
                    }
                    out.push('\n');
                }
            }
        }
    }
    out
}
