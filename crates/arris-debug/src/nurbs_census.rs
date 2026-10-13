//! The NURBS census (`plans/nurbs-analytic-section` step 1): every battery
//! stage Arris refuses as the NURBS cycle's, taken apart by the pair the
//! refusal names and by the NURBS faces the stage's tool reaches.
//!
//! For each such stage of a part fixture — a committed one, or a scratch
//! one `tools/real-parts.sh` leaves under `target/real-parts/work/` — the
//! case is built again in Arris from the solid as read. The refusal's two
//! entities are described: a NURBS face by its degrees, its knot spans, its
//! closure and its collapsed rows, a NURBS edge by its curve's degree and
//! spans and the faces it bounds. A boolean stage also builds its tool
//! alone and lists every NURBS face of the part whose box meets the tool's,
//! with the bidegree the tool's implicit form gives it, and the curve kinds
//! of the tool's edges, which are the curve–surface pairs the pave asks of
//! those faces. Whether the stage is reachable without NURBS × NURBS or a
//! blend is then read off: a fillet never is, a boolean is when every
//! surface the tool brings is analytic. Nothing here is a kernel
//! operation.

use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::Path;

use arris_check::domain::FaceDomain;
use arris_geom::{Curve, GeomKind, Surface, SurfaceKind};
use arris_math::Aabb;
use arris_ops::OpError;
use arris_topo::{EntityId, Model};

use crate::battery::{self, Case};
use crate::corpus::{self, Made};
use crate::fixtures::{Expected, Fixture};
use crate::histogram::{Cycle, Stage, blocks_reason_in};
use crate::part::{self, Outcome as ReadOutcome, PartFixture};
use crate::unmetered::step_read;

/// A NURBS face, described.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsFace {
    /// The face, as the model names it.
    pub face: String,
    /// `[p, q]`.
    pub degree: [usize; 2],
    /// Distinct knot spans in each direction's domain.
    pub spans: [usize; 2],
    /// Whether any weight differs from the first.
    pub rational: bool,
    /// Each direction's closure: `p` periodic, `c` closed, `-` open.
    pub closure: [char; 2],
    /// Collapsed rows: `(fixed direction, value)`.
    pub poles: Vec<(usize, f64)>,
}

/// One refused stage.
#[derive(Debug, Clone, PartialEq)]
pub struct StageCensus {
    /// `#id[instance]`.
    pub solid: String,
    /// The battery stage.
    pub stage: String,
    /// The refusal, as the differential names it.
    pub refusal: String,
    /// The two entities the refusal names, each with its kind.
    pub pair: String,
    /// The NURBS entity the refusal names, described: a face as
    /// [`NurbsFace`], an edge by its curve and the faces it bounds.
    pub named: String,
    /// For a boolean stage: the tool's surface kinds and edge curve kinds.
    pub tool: Option<(Vec<SurfaceKind>, Vec<String>)>,
    /// For a boolean stage: every NURBS face of the part whose box meets
    /// the tool's, with the bidegree the tool's highest implicit degree
    /// gives it.
    pub reached: Vec<(NurbsFace, [usize; 2])>,
    /// Reachable without NURBS × NURBS or a blend, and why not where not.
    pub reachable: Result<(), String>,
}

/// The distinct spans of a clamped or periodic knot vector over its
/// domain `[knots[degree], knots[len − degree − 1]]`.
fn spans(knots: &[f64], degree: usize) -> usize {
    let hi = knots.len().saturating_sub(degree + 1);
    (degree..hi)
        .filter(|&k| knots.get(k + 1).is_some_and(|b| *b > knots[k]))
        .count()
}

/// `face` of `m` described, if its surface is a NURBS surface.
fn describe(m: &Model, face: arris_topo::FaceId) -> Option<NurbsFace> {
    let surface = m.surface(m.face(face).ok()?.surface()).ok()?;
    let Surface::Nurbs(n) = surface else {
        return None;
    };
    let [p, q] = n.degree();
    let [ku, kv] = n.knots();
    let w = n.weights();
    let closure = [0, 1].map(|d| match (n.period()[d], n.closure()[d]) {
        (Some(_), _) => 'p',
        (None, Some(_)) => 'c',
        (None, None) => '-',
    });
    Some(NurbsFace {
        face: face.to_string(),
        degree: [p, q],
        spans: [spans(ku, p), spans(kv, q)],
        rational: w.iter().any(|x| (x - w[0]).abs() > 0.0),
        closure,
        poles: (surface.singularities().iter())
            .map(|s| (s.fixed, s.value))
            .collect(),
    })
}

fn face_text(f: &NurbsFace) -> String {
    let poles = if f.poles.is_empty() {
        "no pole".to_string()
    } else {
        let at: Vec<String> = (f.poles.iter())
            .map(|(k, v)| format!("{}={v:.3}", if *k == 0 { 'u' } else { 'v' }))
            .collect();
        format!("poles {}", at.join(", "))
    };
    format!(
        "{} deg ({},{}) spans {}×{} {} closure {}{} {poles}",
        f.face,
        f.degree[0],
        f.degree[1],
        f.spans[0],
        f.spans[1],
        if f.rational { "rational" } else { "polynomial" },
        f.closure[0],
        f.closure[1],
    )
}

/// The degree of a surface's implicit form; `None` for a NURBS surface,
/// which has none.
fn implicit_degree(kind: SurfaceKind) -> Option<usize> {
    match kind {
        SurfaceKind::Plane => Some(1),
        SurfaceKind::Cylinder
        | SurfaceKind::EllipticCylinder
        | SurfaceKind::Cone
        | SurfaceKind::Sphere => Some(2),
        SurfaceKind::Torus => Some(4),
        SurfaceKind::Nurbs => None,
    }
}

/// The NURBS entity `id` of `m`, described.
fn named(m: &Model, id: EntityId) -> String {
    match id {
        EntityId::Face(f) => describe(m, f).map_or_else(|| f.to_string(), |d| face_text(&d)),
        EntityId::Edge(e) => {
            let Some((c, _)) = m.edge(e).ok().and_then(|edge| edge.curve()) else {
                return e.to_string();
            };
            let curve = match m.curve(c) {
                Ok(Curve::Nurbs(n)) => {
                    format!("deg {} spans {}", n.degree(), spans(n.knots(), n.degree()))
                }
                Ok(other) => other.kind().to_string(),
                Err(_) => String::new(),
            };
            let faces: Vec<String> = (m.edge_uses(e).ok().into_iter().flatten())
                .map(|u| {
                    describe(m, u.face).map_or_else(
                        || {
                            m.face(u.face)
                                .ok()
                                .and_then(|f| m.surface(f.surface()).ok())
                                .map_or_else(String::new, |s| format!("{} {}", u.face, s.kind()))
                        },
                        |d| face_text(&d),
                    )
                })
                .collect();
            format!("{e} NURBS curve {curve}, on {}", faces.join(" | "))
        }
        other => other.to_string(),
    }
}

/// The box of every face of `body`, `None` where a face has none.
fn body_box(m: &Model, body: arris_topo::Body, tol: f64) -> Option<Aabb> {
    let faces = m.faces(body).ok()?;
    let mut out: Option<Aabb> = None;
    for f in faces {
        let b = FaceDomain::of(m, f.id, tol).ok()?.bounds()?;
        out = Some(out.map_or(b, |o| o.union(b)));
    }
    out
}

fn fixture_for(fixture: &PartFixture, name: &str, case: &Case) -> Fixture {
    Fixture {
        dir: fixture.dir.clone(),
        name: name.to_string(),
        recipe: battery::recipe(fixture, case),
        recipe_sha256: String::new(),
        expected: Expected {
            occt: String::new(),
            recipe_sha256: String::new(),
            results: BTreeMap::new(),
        },
    }
}

/// Censuses every stage of `fixture` that Arris refuses as the NURBS
/// cycle's, the first placement of each solid.
///
/// # Errors
/// The file does not read, or a `read` solid is refused.
pub fn census(fixture: &PartFixture) -> Result<Vec<StageCensus>, String> {
    let mut m = Model::new(fixture.part.precision.precision()).map_err(|e| e.to_string())?;
    let path = fixture.dir.join(&fixture.part.file);
    let bytes = std::fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let read = step_read(
        &mut m,
        &String::from_utf8_lossy(&bytes),
        &arris_io::step::ReadOptions::default(),
    )
    .map_err(|e| format!("{}: {e}", path.display()))?;
    let tol = m.precision().default_tolerance;
    let mut out = Vec::new();
    for (solid, spec) in read.solids.iter().zip(&fixture.part.solids) {
        if spec.outcome != ReadOutcome::Read {
            continue;
        }
        let key = battery::key(spec.id, spec.instance);
        let Some(cases) = fixture.part.battery.get(&key) else {
            continue;
        };
        let back = solid
            .result
            .as_ref()
            .map_err(|r| format!("{key} is refused: {r}"))?;
        for (stage_name, case) in cases {
            let Some(stage) = Stage::of_name(stage_name) else {
                continue;
            };
            let Some(first) = case.steps.first() else {
                continue;
            };
            let made = || {
                BTreeMap::from([(
                    first.name().to_string(),
                    Made {
                        body: back.body,
                        provenance: back.provenance.clone(),
                        inputs: Vec::new(),
                        outputs: vec![back.body],
                    },
                )])
            };
            let whole = fixture_for(fixture, stage_name, case);
            let error = match corpus::chain_from(&whole, "default", m.clone(), made()) {
                Err(corpus::CorpusError::Op { source, .. }) => source,
                _ => continue,
            };
            if blocks_reason_in(&m, stage, &error) != Some(Cycle::Nurbs) {
                continue;
            }
            let (pair, named_text) = match &error {
                OpError::Unsupported { a, b } => {
                    let nurbs = |k: GeomKind| {
                        matches!(
                            k,
                            GeomKind::Surface(SurfaceKind::Nurbs)
                                | GeomKind::Curve(arris_geom::CurveKind::Nurbs)
                        )
                    };
                    let described: Vec<String> = [a, b]
                        .into_iter()
                        .filter(|(k, _)| nurbs(*k))
                        .map(|(_, s)| named(&m, s.id))
                        .collect();
                    (
                        format!("{} ({}) × {} ({})", a.1, a.0, b.1, b.0),
                        described.join("; "),
                    )
                }
                other => (other.to_string(), String::new()),
            };
            let mut census = StageCensus {
                solid: key.clone(),
                stage: stage_name.clone(),
                refusal: crate::differential::refusal(&error),
                pair,
                named: named_text,
                tool: None,
                reached: Vec::new(),
                reachable: Err("a blend".into()),
            };
            if stage != Stage::Fillet {
                // The tool alone: every step but the cut, its result the
                // cut's tool.
                let mut tool_case = case.clone();
                let cut = tool_case.steps.pop();
                let tool_name = match cut {
                    Some(crate::fixtures::Step::Cut {
                        tool: Some(tool), ..
                    }) => tool,
                    _ => continue,
                };
                tool_case.result = tool_name.clone();
                let built = corpus::chain_from(
                    &fixture_for(fixture, stage_name, &tool_case),
                    "default",
                    m.clone(),
                    made(),
                )
                .map_err(|e| format!("{key} {stage_name} tool: {e}"))?;
                let tool = built.steps.get(&tool_name).ok_or("the tool step")?.body;
                let tm = &built.model;
                let mut kinds: Vec<SurfaceKind> = (tm.faces(tool).map_err(|e| e.to_string())?)
                    .iter()
                    .filter_map(|f| Some(tm.surface(tm.face(f.id).ok()?.surface()).ok()?.kind()))
                    .collect();
                kinds.sort_by_key(|k| k.to_string());
                kinds.dedup();
                let mut curves: Vec<String> = (tm.edges(tool).map_err(|e| e.to_string())?)
                    .iter()
                    .filter_map(|e| {
                        let (c, _) = tm.edge(e.id).ok()?.curve()?;
                        Some(tm.curve(c).ok()?.kind().to_string())
                    })
                    .collect();
                curves.sort();
                curves.dedup();
                let degree = kinds.iter().filter_map(|k| implicit_degree(*k)).max();
                let tool_box = body_box(tm, tool, tol);
                for f in m.faces(back.body).map_err(|e| e.to_string())? {
                    let Some(d) = describe(&m, f.id) else {
                        continue;
                    };
                    let meets = match (
                        tool_box,
                        FaceDomain::of(&m, f.id, tol).ok().and_then(|d| d.bounds()),
                    ) {
                        (Some(t), Some(b)) => t.inflated(tol).intersects(&b),
                        _ => true,
                    };
                    if meets {
                        let k = degree.unwrap_or(0);
                        let bi = [k * d.degree[0], k * d.degree[1]];
                        census.reached.push((d, bi));
                    }
                }
                census.reachable = if kinds.contains(&SurfaceKind::Nurbs) {
                    Err("the tool has a NURBS face".into())
                } else {
                    Ok(())
                };
                census.tool = Some((kinds, curves));
            }
            out.push(census);
        }
    }
    Ok(out)
}

/// The census of the part fixture in `dir`.
///
/// # Errors
/// As [`census`], or the fixture does not load.
pub fn census_dir(dir: &Path) -> Result<Vec<StageCensus>, String> {
    census(&part::load(dir).map_err(|e| e.to_string())?)
}

/// The census as markdown: per part, one block per refused stage.
pub fn markdown(parts: &[(String, Vec<StageCensus>)]) -> String {
    let mut out = String::new();
    for (part, stages) in parts {
        for s in stages {
            let _ = writeln!(out, "### {part} {} {} — {}\n", s.solid, s.stage, s.refusal);
            let _ = writeln!(out, "- pair: {}", s.pair);
            if !s.named.is_empty() {
                let _ = writeln!(out, "- named: {}", s.named);
            }
            if let Some((kinds, curves)) = &s.tool {
                let kinds: Vec<String> = kinds.iter().map(ToString::to_string).collect();
                let _ = writeln!(
                    out,
                    "- tool: faces {}; edges {}",
                    kinds.join(", "),
                    curves.join(", ")
                );
                let _ = writeln!(
                    out,
                    "- NURBS faces the tool's box meets: {}",
                    s.reached.len()
                );
                for (f, bi) in &s.reached {
                    let _ = writeln!(out, "  - {} → bidegree ({},{})", face_text(f), bi[0], bi[1]);
                }
            }
            let _ = writeln!(
                out,
                "- reachable without NURBS × NURBS or a blend: {}\n",
                match &s.reachable {
                    Ok(()) => "yes".to_string(),
                    Err(why) => format!("no, {why}"),
                }
            );
        }
    }
    out
}
