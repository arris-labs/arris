//! The NURBS census (`nurbs-analytic-section` step 1): every battery
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
//!
//! The fillet census (`nurbs-face-blend` step 1, [`fillet_census_dir`])
//! takes a fillet stage the NURBS cycle blocks apart edge by edge: each
//! sampled edge filleted alone in both kernels, its faces and curve (what
//! conic a B-spline hides, against the faces' own section), a NURBS face's
//! least principal radius on the ball's side, each end's faces across and
//! whether the ball reaches past the B-spline's domain there, and the
//! tangent chain a fillet of it propagates along.

use arris_math::real;
use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::Path;

use arris_check::domain::FaceDomain;
use arris_geom::{Curve, GeomKind, Surface, SurfaceKind};
use arris_math::Aabb;
use arris_math::{Interval, Point2, Point3, Vec2, Vec3};
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

/// The turn, in radians, below which an edge at a censused edge's end is
/// reported as continuing it: wide enough to show a chain a looser
/// tangency test would form, tighter than any corner a part is drawn with.
const CHAIN_TURN: f64 = 0.01;

/// One end of a censused fillet edge.
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeEnd {
    /// The vertex.
    pub vertex: String,
    /// How many edges meet at it.
    pub degree: usize,
    /// The faces at the vertex other than the edge's two, with their
    /// surface kinds: the faces across the end.
    pub across: Vec<String>,
    /// Another edge of the battery's sample meets the vertex.
    pub sampled_meets: bool,
    /// Per NURBS face of the edge: how far, in the face's own `(u, v)`,
    /// the end, the ball's contact there, and that contact carried past
    /// the end by the radius lie outside the surface's domain, each as a
    /// fraction of the domain's width (0 inside).
    pub past_domain: Vec<(String, [f64; 3])>,
    /// The edges at the vertex that continue the edge tangentially (a
    /// chain), each with its curve's kind.
    pub continued_by: Vec<String>,
}

/// One edge of a fillet stage the NURBS cycle blocks, filleted alone.
#[derive(Debug, Clone, PartialEq)]
pub struct FilletEdge {
    /// `#id[instance]`.
    pub solid: String,
    /// The edge.
    pub edge: String,
    /// Its curve's midpoint: Open CASCADE's name for it.
    pub at: [f64; 3],
    /// The battery's radius.
    pub radius: f64,
    /// Arris, the edge filleted alone: built, or the refusal.
    pub alone: String,
    /// Open CASCADE, the edge filleted alone.
    pub occt: String,
    /// The two faces, each `id kind`, a NURBS face described.
    pub faces: [String; 2],
    /// The edge's curve, a NURBS one with what conic it hides.
    pub curve: String,
    /// The edge is convex: the ball sits against the faces' outward
    /// normals.
    pub convex: Option<bool>,
    /// Per NURBS face: its least principal radius on the ball's side over
    /// the band the ball sweeps, `INFINITY` where it never curves that
    /// way.
    pub least_radius: Vec<(String, f64)>,
    /// The edge's ends; empty for a closed edge.
    pub ends: Vec<EdgeEnd>,
    /// The tangent chain through the edge, as a fillet propagates it: each
    /// sharp edge that continues the last through a vertex within
    /// `CHAIN_TURN` and shares a face with it, in walk order from the
    /// edge out of its start and then out of its end, each described.
    pub chain: Vec<String>,
}

/// The principal curvatures at `(u, v)` against the surface's normal
/// (positive where it bends toward the normal), `None` where it has none.
fn principal_curvatures(s: &Surface, u: f64, v: f64) -> Option<[f64; 2]> {
    let n = s.normal(u, v)?.into_inner();
    let e = s.eval(u, v);
    let (ee, ff, gg) = (e.du.dot(&e.du), e.du.dot(&e.dv), e.dv.dot(&e.dv));
    let (l, mm, nn) = (e.duu.dot(&n), e.duv.dot(&n), e.dvv.dot(&n));
    let det = ee * gg - ff * ff;
    if det <= 0.0 {
        return None;
    }
    let k = (l * nn - mm * mm) / det;
    let h = (ee * nn - 2.0 * ff * mm + gg * l) / (2.0 * det);
    let root = (h * h - k).max(0.0).sqrt();
    Some([h - root, h + root])
}

/// The parameter step that moves `(u, v)` by `w` in space, to first order:
/// `w` resolved onto `∂P/∂u` and `∂P/∂v`.
fn uv_step(s: &Surface, uv: Point2, w: Vec3) -> Option<Vec2> {
    let e = s.eval(uv.x, uv.y);
    let (ee, ff, gg) = (e.du.dot(&e.du), e.du.dot(&e.dv), e.dv.dot(&e.dv));
    let det = ee * gg - ff * ff;
    if det <= 0.0 {
        return None;
    }
    let (wu, wv) = (w.dot(&e.du), w.dot(&e.dv));
    Some(Vec2::new(
        (gg * wu - ff * wv) / det,
        (ee * wv - ff * wu) / det,
    ))
}

/// How far `uv` lies outside `domain`, as a fraction of its width, the
/// larger direction's; a periodic direction never.
fn outside(s: &Surface, uv: Point2) -> f64 {
    let d = s.domain();
    let p = s.period();
    let mut worst: f64 = 0.0;
    for (k, x) in [uv.x, uv.y].into_iter().enumerate() {
        if p[k].is_some() || !d[k].lo().is_finite() || !d[k].hi().is_finite() {
            continue;
        }
        let w = d[k].hi() - d[k].lo();
        let past = (d[k].lo() - x).max(x - d[k].hi()).max(0.0);
        worst = worst.max(past / w);
    }
    worst
}

/// What conic a NURBS curve hides over `range`: a line or a circle the
/// samples lie on within `tol`, or the deviations from the best of each.
fn conic_of(c: &Curve, range: Interval, tol: f64) -> String {
    const SAMPLES: usize = 64;
    let pts: Vec<Point3> = (0..=SAMPLES)
        .map(|i| c.point(range.lerp(i as f64 / SAMPLES as f64)))
        .collect();
    let (a, b, m) = (pts[0], pts[SAMPLES], pts[SAMPLES / 2]);
    let chord = b - a;
    let line_dev = pts
        .iter()
        .map(|p| {
            let w = p - a;
            if chord.norm() > 0.0 {
                (w - chord * (w.dot(&chord) / chord.norm_squared())).norm()
            } else {
                w.norm()
            }
        })
        .fold(0.0, f64::max);
    if line_dev <= tol {
        return format!("a line within {line_dev:.1e}");
    }
    // The circle through the ends and the middle — or, for a closed
    // curve, through three points a third apart.
    let (p, q, r) = if chord.norm() <= tol {
        (a, pts[SAMPLES / 3], pts[2 * SAMPLES / 3])
    } else {
        (a, m, b)
    };
    let (u, w) = (q - p, r - p);
    let n = u.cross(&w);
    if n.norm_squared() == 0.0 {
        return format!("no conic: off a line by {line_dev:.1e}");
    }
    let centre = p
        + (w.cross(&n) * u.norm_squared() + n.cross(&u) * w.norm_squared())
            / (2.0 * n.norm_squared());
    let normal = n.normalize();
    let radius = (p - centre).norm();
    let (mut plane_dev, mut circle_dev): (f64, f64) = (0.0, 0.0);
    for x in &pts {
        let off = (x - centre).dot(&normal);
        plane_dev = plane_dev.max(off.abs());
        let radial = ((x - centre) - normal * off).norm();
        circle_dev = circle_dev.max((real::powi(radial - radius, 2) + off * off).sqrt());
    }
    if circle_dev <= tol {
        format!("a circle r {radius:.6} within {circle_dev:.1e}")
    } else {
        format!(
            "no circle (off the best by {circle_dev:.1e}, off its plane by {plane_dev:.1e}, off a line by {line_dev:.1e})"
        )
    }
}

/// For an edge between two analytic faces: the kinds of the curves the
/// two surfaces meet in near it, and how far the edge's curve over `range`
/// strays from the nearest of them.
fn faces_section(m: &Model, edge: arris_topo::EdgeId, curve: &Curve, range: Interval) -> String {
    const SAMPLES: usize = 64;
    const DENSE: usize = 20_000;
    let Ok(uses) = m.edge_uses(edge) else {
        return String::new();
    };
    let surfaces: Vec<&Surface> = (uses.iter())
        .filter_map(|u| m.surface(m.face(u.face).ok()?.surface()).ok())
        .collect();
    let [a, b] = surfaces[..] else {
        return String::new();
    };
    if a.kind() == SurfaceKind::Nurbs || b.kind() == SurfaceKind::Nurbs {
        return String::new();
    }
    let Some(within) = curve.bounds(range) else {
        return String::new();
    };
    let within = within.inflated(1.0 + 0.1 * within.diagonal());
    let hit = match arris_geom::intersect_surfaces(
        a,
        b,
        &within,
        m.precision().tolerance(),
        &mut arris_math::Meter::default(),
    ) {
        Ok(hit) => hit,
        Err(e) => return format!("; the faces' section refused: {e}"),
    };
    let kinds: Vec<String> = hit
        .curves()
        .iter()
        .map(|c| c.curve.kind().to_string())
        .collect();
    let domains: Vec<Interval> = (hit.curves().iter())
        .map(|c| {
            let d = c.curve.domain();
            if d.lo().is_finite() && d.hi().is_finite() {
                d
            } else {
                // A line: over the box's reach either side of its origin.
                let r = within.diagonal();
                Interval::new(-r, r).unwrap_or(d)
            }
        })
        .collect();
    let mut worst: f64 = 0.0;
    for i in 0..=SAMPLES {
        let p = curve.point(range.lerp(i as f64 / SAMPLES as f64));
        let mut near = f64::INFINITY;
        for (c, d) in hit.curves().iter().zip(&domains) {
            let at = |k: usize| d.lerp(k as f64 / DENSE as f64);
            let dist = |t: f64| (c.curve.point(t) - p).norm();
            let best = (0..=DENSE)
                .min_by(|&a, &b| dist(at(a)).total_cmp(&dist(at(b))))
                .unwrap_or(0);
            // Golden-section search between the nearest sample's neighbours.
            let (mut lo, mut hi) = (at(best.saturating_sub(1)), at((best + 1).min(DENSE)));
            let g = 0.5 * (5f64.sqrt() - 1.0);
            for _ in 0..80 {
                let (a, b) = (hi - g * (hi - lo), lo + g * (hi - lo));
                if dist(a) < dist(b) {
                    hi = b;
                } else {
                    lo = a;
                }
            }
            near = near.min(dist(0.5 * (lo + hi)));
        }
        worst = worst.max(near);
    }
    format!(
        "; the faces meet in [{}], the edge off the nearest by {worst:.1e}",
        kinds.join(", ")
    )
}

/// The sine between two faces' normals above which an edge is sharp: a
/// smooth edge ends a chain, a blend has nothing to fill there.
const SHARP_SINE: f64 = 1e-3;

/// The direction `x` leaves `vertex` along, unit; `None` without one.
fn leaving_at(m: &Model, x: arris_topo::EdgeId, vertex: arris_topo::VertexId) -> Option<Vec3> {
    let e = m.edge(x).ok()?;
    let (c, r) = e.curve()?;
    let c = m.curve(c).ok()?;
    let (t, s) = if e.start() == vertex {
        (r.lo(), 1.0)
    } else {
        (r.hi(), -1.0)
    };
    let d = c.eval(t).d1 * s;
    (d.norm() > 0.0).then(|| d.normalize())
}

/// [`FilletEdge::chain`] of `edge`.
fn chain_through(
    m: &Model,
    orientation: &BTreeMap<arris_topo::FaceId, arris_topo::Orientation>,
    edge: arris_topo::EdgeId,
) -> Result<Vec<String>, String> {
    let err = |e: arris_topo::NotFound| e.to_string();
    let faces_of = |x: arris_topo::EdgeId| -> Vec<arris_topo::FaceId> {
        m.edge_uses(x)
            .map(|u| u.iter().map(|c| c.face).collect())
            .unwrap_or_default()
    };
    let describe_edge = |x: arris_topo::EdgeId| -> String {
        let Ok(y) = m.edge(x) else {
            return x.to_string();
        };
        let Some((c, r)) = y.curve() else {
            return format!("{x} (no curve)");
        };
        let Ok(c) = m.curve(c) else {
            return x.to_string();
        };
        let faces: Vec<String> = faces_of(x).into_iter().map(|f| face_line(m, f)).collect();
        let conic = match c {
            Curve::Nurbs(_) => {
                let t = m.precision().default_tolerance.max(y.tolerance());
                format!(": {}{}", conic_of(c, r, t), faces_section(m, x, c, r))
            }
            _ => String::new(),
        };
        format!("{x} {} on {}{conic}", c.kind(), faces.join(" | "))
    };
    let mut seen = vec![edge];
    let mut out = vec![describe_edge(edge)];
    let e = m.edge(edge).map_err(err)?;
    for start in [e.start(), e.end()] {
        let (mut at, mut vertex) = (edge, start);
        while let Some(own) = leaving_at(m, at, vertex) {
            let mut next = None;
            for &x in m.vertex_edges(vertex).map_err(err)? {
                if x == at || seen.contains(&x) {
                    continue;
                }
                let shares = faces_of(x).iter().any(|f| faces_of(at).contains(f));
                let sharp = dihedral_sine(m, orientation, x).is_some_and(|s| s > SHARP_SINE);
                let turn = leaving_at(m, x, vertex).map_or(f64::INFINITY, |b| {
                    real::acos((-own.dot(&b)).clamp(-1.0, 1.0))
                });
                if shares && sharp && turn <= CHAIN_TURN {
                    next = Some(x);
                }
            }
            let Some(x) = next else { break };
            seen.push(x);
            out.push(format!(
                "{} {}",
                if start == e.start() { "←" } else { "→" },
                describe_edge(x)
            ));
            let y = m.edge(x).map_err(err)?;
            if y.start() == y.end() {
                break;
            }
            vertex = if y.start() == vertex {
                y.end()
            } else {
                y.start()
            };
            at = x;
        }
    }
    Ok(out)
}

/// The sine between the outward normals of `edge`'s two faces at its
/// curve's midpoint: zero where they meet tangentially.
fn dihedral_sine(
    m: &Model,
    orientation: &BTreeMap<arris_topo::FaceId, arris_topo::Orientation>,
    edge: arris_topo::EdgeId,
) -> Option<f64> {
    let (_, range) = m.edge(edge).ok()?.curve()?;
    let mut normals = Vec::new();
    for u in m.edge_uses(edge).ok()? {
        let face = m.face(u.face).ok()?;
        let coedge = face
            .loops()
            .get(u.loop_index)?
            .coedges()
            .get(u.coedge_index)?;
        let uv = m.curve2(coedge.pcurve()).ok()?.point(range.midpoint());
        let n = m.surface(face.surface()).ok()?.normal(uv.x, uv.y)?;
        normals.push(n.into_inner() * orientation.get(&u.face)?.sign());
    }
    let [a, b] = normals[..] else { return None };
    Some(a.cross(&b).norm())
}

/// `face` of `m`, `id kind`, a NURBS face described.
fn face_line(m: &Model, face: arris_topo::FaceId) -> String {
    describe(m, face).map_or_else(
        || {
            m.face(face)
                .ok()
                .and_then(|f| m.surface(f.surface()).ok())
                .map_or_else(|| face.to_string(), |s| format!("{face} {}", s.kind()))
        },
        |d| face_text(&d),
    )
}

/// Describes `edge` of `body` for a fillet of `radius`, filleted alone in
/// a scratch copy of `m`.
fn fillet_edge(
    m: &Model,
    body: arris_topo::Body,
    solid: &str,
    edge: arris_topo::EdgeId,
    radius: f64,
    sample: &[arris_topo::EdgeId],
) -> Result<FilletEdge, String> {
    use arris_topo::Orientation;
    let err = |e: arris_topo::NotFound| e.to_string();
    let mut orientation: BTreeMap<arris_topo::FaceId, Orientation> = BTreeMap::new();
    for shell in m.shells(body).map_err(|e| e.to_string())? {
        for face_use in m.shell(shell.id).map_err(|e| e.to_string())?.faces() {
            let face = face_use.oriented_by(shell.orientation);
            orientation.insert(face.id, face.orientation);
        }
    }
    let e = m.edge(edge).map_err(err)?;
    let (cid, range) = e.curve().ok_or("an edge with no curve")?;
    let curve = m.curve(cid).map_err(err)?.clone();
    let uses = m.edge_uses(edge).map_err(err)?.to_vec();
    let [ua, ub] = uses[..] else {
        return Err(format!("{edge} has {} uses", uses.len()));
    };
    let tol = m.precision().default_tolerance.max(e.tolerance());
    let curve_text = match &curve {
        Curve::Nurbs(n) => format!(
            "NURBS deg {} spans {}: {}{}",
            n.degree(),
            spans(n.knots(), n.degree()),
            conic_of(&curve, range, tol),
            faces_section(m, edge, &curve, range)
        ),
        other => other.kind().to_string(),
    };
    let curve_text = format!("{curve_text} (edge tolerance {:.1e})", e.tolerance());
    // Per use: surface, outward sign, the coedge's sense along the edge's
    // curve, and its pcurve.
    let mut per = Vec::new();
    for u in [ua, ub] {
        let face = m.face(u.face).map_err(err)?;
        let coedge = face
            .loops()
            .get(u.loop_index)
            .and_then(|l| l.coedges().get(u.coedge_index))
            .ok_or("an edge use names no coedge")?;
        let sign = orientation.get(&u.face).ok_or("a face of no shell")?;
        let surface = m.surface(face.surface()).map_err(err)?.clone();
        let pcurve = m.curve2(coedge.pcurve()).map_err(err)?.clone();
        per.push((
            u.face,
            surface,
            sign.sign(),
            sign.compose(coedge.orientation()).sign(),
            pcurve,
        ));
    }
    let mid = range.midpoint();
    let at = curve.point(mid);
    // The outward normal, the direction into the face, at `t`.
    let frame = |k: usize, t: f64| -> Option<(Point2, Vec3, Vec3)> {
        let (_, s, out, along, pc) = &per[k];
        let uv = pc.point(t);
        let n = s.normal(uv.x, uv.y)?.into_inner() * *out;
        let tangent = curve.eval(t).d1 * *along;
        Some((uv, n, n.cross(&tangent).normalize()))
    };
    let convex = match (frame(0, mid), frame(1, mid)) {
        (Some((_, _, w1)), Some((_, n2, _))) => Some(w1.dot(&(-n2)) > 0.0),
        _ => None,
    };
    // The ball's side against each outward normal.
    let side = if convex == Some(false) { 1.0 } else { -1.0 };
    // The contact's distance from the edge along a face: r / tan(α / 2),
    // α the angle between the directions into the two faces.
    let reach = |t: f64| -> f64 {
        match (frame(0, t), frame(1, t)) {
            (Some((_, _, w1)), Some((_, _, w2))) => {
                let alpha = real::acos(w1.dot(&w2).clamp(-1.0, 1.0));
                // A dihedral near flat is no corner the census meets.
                radius / real::tan(alpha / 2.0).max(f64::EPSILON.sqrt())
            }
            _ => radius,
        }
    };
    let mut least_radius = Vec::new();
    for (k, (face, s, out, _, _)) in per.iter().enumerate() {
        if s.kind() != SurfaceKind::Nurbs {
            continue;
        }
        let mut least = f64::INFINITY;
        for i in 0..=32 {
            let t = range.lerp(i as f64 / 32.0);
            let Some((uv, _, w)) = frame(k, t) else {
                continue;
            };
            let d = reach(t);
            for f in [0.0, 0.5, 1.0, 1.5] {
                let Some(step) = uv_step(s, uv, w * (d * f)) else {
                    continue;
                };
                let p = uv + step;
                let Some(ks) = principal_curvatures(s, p.x, p.y) else {
                    continue;
                };
                for kappa in ks {
                    let toward = side * out * kappa;
                    if toward > 0.0 {
                        least = least.min(1.0 / toward);
                    }
                }
            }
        }
        least_radius.push((face.to_string(), least));
    }
    let mut ends = Vec::new();
    if e.start() != e.end() {
        let faces = [ua.face, ub.face];
        for (vertex, t, sense) in [(e.start(), range.lo(), -1.0), (e.end(), range.hi(), 1.0)] {
            let star = m.vertex_edges(vertex).map_err(err)?.to_vec();
            let mut across: Vec<arris_topo::FaceId> = Vec::new();
            for &x in &star {
                for u in m.edge_uses(x).map_err(err)? {
                    if !faces.contains(&u.face) && !across.contains(&u.face) {
                        across.push(u.face);
                    }
                }
            }
            across.sort();
            let mut past_domain = Vec::new();
            for (k, (face, s, _, _, _)) in per.iter().enumerate() {
                if s.kind() != SurfaceKind::Nurbs {
                    continue;
                }
                let Some((uv, _, w)) = frame(k, t) else {
                    continue;
                };
                let away = curve.eval(t).d1.normalize() * sense;
                let contact = uv_step(s, uv, w * reach(t)).map(|x| uv + x);
                let beyond = contact.and_then(|c| uv_step(s, c, away * radius).map(|x| c + x));
                let at = |p: Option<Point2>| p.map_or(f64::NAN, |p| outside(s, p));
                past_domain.push((face.to_string(), [at(Some(uv)), at(contact), at(beyond)]));
            }
            // The edge's direction leaving the vertex, and each other
            // edge's: a chain where they are opposite.
            let leaving = |x: &arris_topo::entity::Edge| -> Option<Vec3> {
                let (c, r) = x.curve()?;
                let c = m.curve(c).ok()?;
                let (t, s) = if x.start() == vertex {
                    (r.lo(), 1.0)
                } else {
                    (r.hi(), -1.0)
                };
                let d = c.eval(t).d1 * s;
                (d.norm() > 0.0).then(|| d.normalize())
            };
            let own = leaving(e);
            let mut continued_by = Vec::new();
            for &x in &star {
                if x == edge {
                    continue;
                }
                let Ok(other) = m.edge(x) else { continue };
                // The turn from the edge into `x`, as a chain would read it.
                let turn = match (own, leaving(other)) {
                    (Some(a), Some(b)) => real::acos((-a.dot(&b)).clamp(-1.0, 1.0)),
                    _ => continue,
                };
                if turn <= CHAIN_TURN {
                    let kind = other
                        .curve()
                        .and_then(|(c, _)| m.curve(c).ok())
                        .map_or("no curve".to_string(), |c| c.kind().to_string());
                    let faces: Vec<String> = (m.edge_uses(x).map_err(err)?.iter())
                        .map(|u| face_line(m, u.face))
                        .collect();
                    let sine = dihedral_sine(m, &orientation, x)
                        .map_or_else(|| "unknown".to_string(), |s| format!("{s:.1e}"));
                    continued_by.push(format!(
                        "{x} ({kind}) on {}, turning {:.2e} rad, the sine between its faces' normals {sine}",
                        faces.join(" | "),
                        turn
                    ));
                }
            }
            ends.push(EdgeEnd {
                continued_by,
                vertex: vertex.to_string(),
                degree: star.len(),
                across: across.iter().map(|&f| face_line(m, f)).collect(),
                sampled_meets: sample.iter().any(|&x| {
                    x != edge
                        && m.edge(x)
                            .is_ok_and(|y| y.start() == vertex || y.end() == vertex)
                }),
                past_domain,
            });
        }
    }
    let handle = m
        .edges(body)
        .map_err(|e| e.to_string())?
        .into_iter()
        .find(|h| h.id == edge)
        .ok_or("the edge is not the body's")?;
    let chain = chain_through(m, &orientation, edge)?;
    let (e_start, e_end) = (e.start(), e.end());
    let mut scratch = m.clone();
    let alone = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        crate::unmetered::fillet(&mut scratch, body, &[handle], radius)
    })) {
        Ok(Ok(_)) => crate::census::BUILT.to_string(),
        Ok(Err(e)) => {
            // A refusal naming another edge's NURBS curve: that edge,
            // described.
            let other = match &e {
                OpError::Unsupported { a, .. } => match a.1.id {
                    EntityId::Edge(x) if x != edge => Some(x),
                    _ => None,
                },
                _ => None,
            };
            let named = other.map_or_else(String::new, |x| {
                let text = m.edge(x).ok().and_then(|y| {
                    let (c, r) = y.curve()?;
                    let c = m.curve(c).ok()?;
                    let t = m.precision().default_tolerance.max(y.tolerance());
                    let faces: Vec<String> = (m.edge_uses(x).ok()?.iter())
                        .map(|u| face_line(m, u.face))
                        .collect();
                    let at_start = y.start() == e_start || y.start() == e_end;
                    let at_end = y.end() == e_start || y.end() == e_end;
                    Some(format!(
                        "; {x} (tolerance {t:.1e}, {} this edge's end) on {}: {}{}",
                        if at_start || at_end {
                            "meets"
                        } else {
                            "does not meet"
                        },
                        faces.join(" | "),
                        conic_of(c, r, t),
                        faces_section(m, x, c, r)
                    ))
                });
                text.unwrap_or_default()
            });
            format!("{} — {e}{named}", crate::census::class_of(&e))
        }
        Err(_) => crate::census::PANICKED.to_string(),
    };
    Ok(FilletEdge {
        solid: solid.to_string(),
        edge: edge.to_string(),
        at: [at.x, at.y, at.z],
        radius,
        alone,
        occt: String::new(),
        faces: [face_line(m, ua.face), face_line(m, ub.face)],
        curve: curve_text,
        convex,
        least_radius,
        ends,
        chain,
    })
}

/// The fillet stage of every solid of the part fixture in `dir` that the
/// NURBS cycle blocks (the battery's sample blended together is refused
/// as the NURBS cycle's), each sampled edge described and filleted alone
/// in Arris and in Open CASCADE at the battery's radius.
///
/// # Errors
/// The fixture or its file does not read, or the oracle could not run.
pub fn fillet_census_dir(dir: &Path) -> Result<Vec<FilletEdge>, String> {
    let fixture = part::load(dir).map_err(|e| e.to_string())?;
    let mut m = Model::new(fixture.part.precision.precision()).map_err(|e| e.to_string())?;
    let path = fixture.dir.join(&fixture.part.file);
    let bytes = std::fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let read = step_read(
        &mut m,
        &String::from_utf8_lossy(&bytes),
        &arris_io::step::ReadOptions::default(),
    )
    .map_err(|e| format!("{}: {e}", path.display()))?;
    let mut out = Vec::new();
    let mut seen = Vec::new();
    for solid in &read.solids {
        let Ok(back) = &solid.result else { continue };
        if seen.contains(&solid.entity.id) {
            continue;
        }
        seen.push(solid.entity.id);
        let key = battery::key(solid.entity.id, solid.entity.instance);
        let (_, sample) = battery::fillet_edges(&m, back.body)?;
        if sample.is_empty() {
            continue;
        }
        let radius = battery::fillet_radius(&sample);
        let handles = m.edges(back.body).map_err(|e| e.to_string())?;
        let chosen: Vec<_> = (sample.iter())
            .filter_map(|s| handles.iter().find(|h| h.id == s.0).copied())
            .collect();
        let mut scratch = m.clone();
        let Err(error) = crate::unmetered::fillet(&mut scratch, back.body, &chosen, radius) else {
            continue;
        };
        if blocks_reason_in(&m, Stage::Fillet, &error) != Some(Cycle::Nurbs) {
            continue;
        }
        let ids: Vec<_> = sample.iter().map(|s| s.0).collect();
        let mut edges = Vec::new();
        for &id in &ids {
            edges.push(fillet_edge(&m, back.body, &key, id, radius, &ids)?);
        }
        let points: Vec<[f64; 3]> = edges.iter().map(|e| e.at).collect();
        let verdicts = crate::oracle::fillet_edges(
            &path,
            solid.entity.id,
            radius,
            crate::census::ORACLE_PROBE,
            &points,
        )
        .map_err(|e| e.to_string())?;
        for (e, v) in edges.iter_mut().zip(verdicts) {
            e.occt = if v.why.is_empty() {
                v.verdict
            } else {
                format!("{} ({})", v.verdict, v.why)
            };
        }
        out.extend(edges);
    }
    Ok(out)
}

/// [`fillet_census_dir`]'s edges as markdown, one block per edge.
pub fn fillet_markdown(parts: &[(String, Vec<FilletEdge>)]) -> String {
    let mut out = String::new();
    for (part, edges) in parts {
        for e in edges {
            let _ = writeln!(
                out,
                "### {part} {} {} at ({}, {}, {}), r {}\n",
                e.solid, e.edge, e.at[0], e.at[1], e.at[2], e.radius
            );
            let _ = writeln!(out, "- Arris alone: {}", e.alone);
            let _ = writeln!(out, "- Open CASCADE alone: {}", e.occt);
            let _ = writeln!(out, "- faces: {} | {}", e.faces[0], e.faces[1]);
            let _ = writeln!(out, "- curve: {}", e.curve);
            let _ = writeln!(
                out,
                "- convex: {}",
                e.convex.map_or("unknown".to_string(), |c| c.to_string())
            );
            for (f, r) in &e.least_radius {
                let _ = writeln!(
                    out,
                    "- {f}: least principal radius on the ball's side {r:.4} (r / it {:.3})",
                    e.radius / r
                );
            }
            if e.chain.len() > 1 {
                let _ = writeln!(out, "- tangent chain of {}:", e.chain.len());
                for c in &e.chain {
                    let _ = writeln!(out, "  - {c}");
                }
            }
            if e.ends.is_empty() {
                let _ = writeln!(out, "- ends: none, a closed edge");
            }
            for end in &e.ends {
                let past: Vec<String> = (end.past_domain.iter())
                    .map(|(f, x)| {
                        format!(
                            "{f} past its domain: the end {:.3}, the contact {:.3}, carried past by r {:.3}",
                            x[0], x[1], x[2]
                        )
                    })
                    .collect();
                let _ = writeln!(
                    out,
                    "- end {}: degree {}, across [{}], continued by [{}], another sampled edge {}{}",
                    end.vertex,
                    end.degree,
                    end.across.join("; "),
                    end.continued_by.join("; "),
                    if end.sampled_meets {
                        "meets it"
                    } else {
                        "does not"
                    },
                    if past.is_empty() {
                        String::new()
                    } else {
                        format!("; {}", past.join(", "))
                    }
                );
            }
            out.push('\n');
        }
    }
    out
}
