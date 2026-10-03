//! The battery (ADR-0025 §6, ADR-0026 §5): a fixed set of operations run
//! on every solid a part fixture reads, each held to Open CASCADE's answer
//! where it builds one and sorted into the differential's classes
//! (ADR-0024 §2).
//!
//! The stages, in [`STAGES`]' order:
//!
//! - `write_read`: the body written to STEP and read back by Arris, held
//!   as the reader's round-trip property holds it — every solid read,
//!   the checker green, the same counts, and the mass properties within
//!   the body's own tolerance. Arris's alone, so it has no operands.
//! - `box_cut`: cut by a box with a corner at the oracle's centroid and
//!   its edges along the oracle's principal axes, reaching past the part.
//! - `drill_x`, `drill_y`, `drill_z`: cut by a cylinder through the
//!   centroid along each principal axis, smallest moment first.
//! - `fillet`: a sample of the solid's edges blended.
//!
//! [`derive()`] writes the operation stages' operands: recipes over the
//! `step` operand, stored in the fixture's `battery` so both kernels
//! build the same thing. Once written they are data — a kernel change
//! that would derive other operands does not move the fixture. [`judge`]
//! builds one in Arris against the oracle's answer, and [`write_read`]
//! runs the round trip; `part::run` holds each to the class the fixture
//! records.

use crate::unmetered::mass_properties;
use crate::unmetered::step_read;
use std::collections::BTreeMap;
#[cfg(not(target_arch = "wasm32"))]
use std::panic::{AssertUnwindSafe, catch_unwind};

use arris_io::arris_check::arris_topo::arris_math::nalgebra::SymmetricEigen;
use arris_io::arris_check::arris_topo::arris_math::{Matrix3, Point3, Vec3};
use arris_io::arris_check::arris_topo::{Body, EdgeId, FaceId, Model, Orientation};
use arris_io::step::{self, ReadOptions};
use serde::{Deserialize, Serialize};

use crate::fixtures::{Analytic, Loop, Measured, Num, Plane, Recipe, Segment, Step};
use crate::part::{Outcome as ReadOutcome, PartFixture, PartSolid};
#[cfg(not(target_arch = "wasm32"))]
use {
    crate::corpus::{self, CorpusError, Made, Stage, check_leaving_nurbs, compare_mass},
    crate::differential::{Outcome, held_to, net_of_removable, panicked, refusal},
    crate::fixtures::{Counts, Expected, Fixture, Tolerances},
    arris_io::step::ReadBody,
    arris_ops::OpError,
};

/// The battery's stages, in the order they run and print (ADR-0026 §5).
pub const STAGES: [&str; 6] = [
    "write_read",
    "box_cut",
    "drill_x",
    "drill_y",
    "drill_z",
    "fillet",
];

/// How far past the part the box and the drills reach: this many times
/// the distance from the centroid to the farthest corner of the part's
/// box, so that neither ends inside the part or on its boundary.
pub const REACH: f64 = 1.5;

/// A drill's radius as a fraction of the part's smallest radius of
/// gyration: small enough to leave material around it in a plate, large
/// enough to be no sliver.
pub const DRILL_FRACTION: f64 = 0.1;

/// How many edges the fillet stage blends at most: the solid's edges in
/// id order, taken at the stride that gives no more than this many.
pub const FILLET_EDGES: usize = 4;

/// The fillet's radius as a fraction of the shortest sampled edge's
/// length.
pub const FILLET_FRACTION: f64 = 0.1;

/// How many chords an edge's length is measured over: the radius needs
/// it to a few per cent.
const LENGTH_CHORDS: usize = 64;

/// The class a battery stage is recorded at: the differential's classes
/// that count (ADR-0024 §2). A disagreement, a checker violation, a panic
/// or an internal fault is a kernel bug and never recorded.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Class {
    /// Both kernels build it, and every stage holds.
    Agree,
    /// Neither builds it.
    BothRefuse,
    /// Arris builds it and Open CASCADE does not.
    OracleRefuses,
    /// Open CASCADE builds it and Arris refuses it: the name
    /// [`refusal`] gives the `OpError`, or the reader's refusal kind for
    /// `write_read`.
    ArrisRefuses(String),
    /// Not run: the stage meets a kernel bug, shrunk to the fixture under
    /// `regression/` named here (ADR-0026 §4). The runner fails once that
    /// fixture has left `regression/`, so the fix that moves it records
    /// the class the stage then has.
    WaitsOn(String),
}

#[cfg(not(target_arch = "wasm32"))]
impl Class {
    /// The class of an outcome that counts, or `None` for one that is a
    /// kernel bug. Never [`Class::WaitsOn`], which a person records.
    pub fn of(outcome: &Outcome) -> Option<Class> {
        match outcome {
            Outcome::Agree => Some(Class::Agree),
            Outcome::BothRefuse => Some(Class::BothRefuse),
            Outcome::OracleRefuses(_) => Some(Class::OracleRefuses),
            Outcome::ArrisRefuses(name) => Some(Class::ArrisRefuses(name.clone())),
            Outcome::Disagree { .. }
            | Outcome::CheckerViolation(_)
            | Outcome::Panic(_)
            | Outcome::Internal(_)
            | Outcome::Excluded { .. } => None,
        }
    }
}

/// What Arris refused a stage with, typed: what the histogram's table
/// maps to a cycle (`crate::histogram`).
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, PartialEq)]
pub enum Refused {
    /// The reader's refusal: of the solid, or of the file written and
    /// read back.
    Read(Box<step::Refusal>),
    /// An operation's error.
    Op(Box<OpError>),
}

#[cfg(not(target_arch = "wasm32"))]
impl Refused {
    /// The cycle it blocks at `stage`, by ADR-0026 §5's table; `None` for
    /// an internal fault, which is never a count.
    pub fn blocks(&self, stage: crate::histogram::Stage) -> Option<crate::histogram::Cycle> {
        match self {
            Refused::Read(r) => Some(crate::histogram::blocks_refusal(r)),
            Refused::Op(e) => crate::histogram::blocks_reason(stage, e),
        }
    }

    /// [`Refused::blocks`] for a refusal raised on the model `m`: a
    /// cylinder pair whose axes are not parallel is the NURBS cycle's
    /// (`crate::histogram::blocks_reason_in`).
    pub fn blocks_in(
        &self,
        m: &arris_io::arris_check::arris_topo::Model,
        stage: crate::histogram::Stage,
    ) -> Option<crate::histogram::Cycle> {
        match self {
            Refused::Read(r) => Some(crate::histogram::blocks_refusal(r)),
            Refused::Op(e) => crate::histogram::blocks_reason_in(m, stage, e),
        }
    }
}

/// A stage judged: its outcome, and the refusal behind it where Arris
/// refused.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Debug, Clone, PartialEq)]
pub struct Judged {
    /// The differential's class.
    pub outcome: Outcome,
    /// What Arris refused with, typed; `None` where it built.
    pub refused: Option<Refused>,
}

#[cfg(not(target_arch = "wasm32"))]
impl From<Outcome> for Judged {
    fn from(outcome: Outcome) -> Judged {
        Judged {
            outcome,
            refused: None,
        }
    }
}

/// One operation stage over a solid: a recipe whose first step is the
/// `step` operand naming the solid.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Case {
    /// The steps.
    pub steps: Vec<Step>,
    /// The step whose body is the result.
    pub result: String,
}

/// Open CASCADE's answer to one [`Case`] in `expected.json`: what
/// `measure` records of its result, with its own tolerance, or why it
/// could not build it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum OracleCase {
    /// It raised, or refused the recipe.
    Refused {
        /// Why, in its words.
        refused: String,
    },
    /// It built a result.
    Built(Box<Measured>),
}

/// The key a solid's battery is stored under: `#<id>[<instance>]`.
pub fn key(id: u64, instance: u32) -> String {
    format!("#{id}[{instance}]")
}

/// A number written as its literal.
fn lit(x: f64) -> Num {
    Num::Literal(x)
}

fn lits(p: [f64; 3]) -> [Num; 3] {
    p.map(lit)
}

/// The principal axes of an inertia tensor, as columns, ascending by
/// moment: each eigenvector signed so that its largest component is
/// positive, the third the cross product of the first two, so the frame
/// is right-handed and the same for the same tensor.
///
/// ```
/// use arris_debug::battery::principal_axes;
///
/// let ([e1, e2, e3], moments) = principal_axes(&[[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 3.0]]);
/// assert_eq!(moments, [1.0, 2.0, 3.0]);
/// assert_eq!((e1.y, e2.x, e3.z), (1.0, 1.0, -1.0));
/// ```
pub fn principal_axes(inertia: &[[f64; 3]; 3]) -> ([Vec3; 3], [f64; 3]) {
    let tensor = Matrix3::from_fn(|i, j| inertia[i][j]);
    let eigen = SymmetricEigen::new(tensor);
    let mut pairs: Vec<(f64, Vec3)> = (0..3)
        .map(|k| {
            (
                eigen.eigenvalues[k],
                eigen.eigenvectors.column(k).into_owned(),
            )
        })
        .collect();
    pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
    let signed = |v: Vec3| {
        let k = v.iamax();
        let v = v.normalize();
        if v[k] < 0.0 { -v } else { v }
    };
    let e1 = signed(pairs[0].1);
    // The second orthogonalised against the first, so rounding in a
    // near-repeated moment leaves no skew in the frame.
    let e2 = signed(pairs[1].1 - e1 * e1.dot(&pairs[1].1));
    let e3 = e1.cross(&e2).normalize();
    ([e1, e2, e3], [pairs[0].0, pairs[1].0, pairs[2].0])
}

/// The farthest corner of `body`'s box — over its vertices and its edges'
/// curve bounds — from `centre`.
fn reach(m: &Model, body: Body, centre: Point3) -> Result<f64, String> {
    let closure = m.closure(body).map_err(|e| e.to_string())?;
    let (mut lo, mut hi) = (Vec3::repeat(f64::INFINITY), Vec3::repeat(f64::NEG_INFINITY));
    for &v in &closure.vertices {
        let p = m.vertex(v).map_err(|e| e.to_string())?.point().coords;
        lo = lo.inf(&p);
        hi = hi.sup(&p);
    }
    for &e in &closure.edges {
        let Some((c, range)) = m.edge(e).map_err(|e| e.to_string())?.curve() else {
            continue;
        };
        if let Some(b) = m.curve(c).map_err(|e| e.to_string())?.bounds(range) {
            lo = lo.inf(&Vec3::from(b.min));
            hi = hi.sup(&Vec3::from(b.max));
        }
    }
    let mut far: f64 = 0.0;
    for x in [lo.x, hi.x] {
        for y in [lo.y, hi.y] {
            for z in [lo.z, hi.z] {
                far = far.max((Point3::new(x, y, z) - centre).norm());
            }
        }
    }
    Ok(far)
}

/// Whether the two faces of `e` meet tangentially at its curve's midpoint
/// as the kernel reads it (ADR-0040): the outward normals are parallel
/// within the model's angular tolerance, or, given the `size` of a blend,
/// `size` times the sine between them is within the faces' tolerance — the
/// model's default or a face's own, the larger — so there is no corner for
/// a rolling ball to fill. Open CASCADE refuses
/// such an edge (`There are no suitable edges for chamfer or fillet`), as
/// it did every one of the fetched tier's 2479, and Arris refuses it as
/// `Reason::TangentChain` — a refusal the battery's sample
/// has no use for. A face with no normal there is not tangent.
fn is_tangent_dihedral(
    m: &Model,
    orientation: &BTreeMap<FaceId, Orientation>,
    e: EdgeId,
    size: Option<f64>,
) -> Result<bool, String> {
    let edge = m.edge(e).map_err(|e| e.to_string())?;
    let Some((_, range)) = edge.curve() else {
        return Ok(false);
    };
    let mut normals = Vec::new();
    let mut tolerance = m.precision().default_tolerance;
    for u in m.edge_uses(e).map_err(|e| e.to_string())? {
        let face = m.face(u.face).map_err(|e| e.to_string())?;
        tolerance = tolerance.max(face.tolerance());
        let coedge = face
            .loops()
            .get(u.loop_index)
            .and_then(|l| l.coedges().get(u.coedge_index))
            .ok_or("an edge use names no coedge")?;
        let uv = m
            .curve2(coedge.pcurve())
            .map_err(|e| e.to_string())?
            .point(range.midpoint());
        let surface = m.surface(face.surface()).map_err(|e| e.to_string())?;
        let Some(n) = surface.normal(uv.x, uv.y) else {
            return Ok(false);
        };
        let sign = orientation.get(&u.face).ok_or("a face of no shell")?.sign();
        normals.push(n.into_inner() * sign);
    }
    let [a, b] = normals[..] else {
        return Ok(false);
    };
    let sine = a.cross(&b).norm();
    Ok(sine <= m.precision().angular_tolerance || size.is_some_and(|s| s * sine <= tolerance))
}

/// The edges a fillet may be asked of: every edge of `body` between two
/// distinct faces that has a curve and is not a tangent dihedral for a
/// blend of `size` (no rolling ball blends one: [`is_tangent_dihedral`]),
/// in id order.
pub(crate) fn blendable_edges(
    m: &Model,
    body: Body,
    size: Option<f64>,
) -> Result<Vec<EdgeId>, String> {
    let closure = m.closure(body).map_err(|e| e.to_string())?;
    let mut orientation: BTreeMap<FaceId, Orientation> = BTreeMap::new();
    for shell in m.shells(body).map_err(|e| e.to_string())? {
        for face_use in m.shell(shell.id).map_err(|e| e.to_string())?.faces() {
            let face = face_use.oriented_by(shell.orientation);
            orientation.insert(face.id, face.orientation);
        }
    }
    let mut edges: Vec<EdgeId> = Vec::new();
    for &e in &closure.edges {
        let edge = m.edge(e).map_err(|e| e.to_string())?;
        if edge.curve().is_none() {
            continue;
        }
        let uses = m.edge_uses(e).map_err(|e| e.to_string())?;
        let mut faces: Vec<_> = uses.iter().map(|u| u.face).collect();
        faces.sort();
        faces.dedup();
        if uses.len() == 2 && faces.len() == 2 && !is_tangent_dihedral(m, &orientation, e, size)? {
            edges.push(e);
        }
    }
    Ok(edges)
}

/// The edges the fillet stage blends and the sample of them: [`blendable_edges`]
/// of `body` at the battery's radius, which is [`FILLET_FRACTION`] of the
/// shortest edge of a sample of those edges, so the two are settled
/// together — the edges tangent at the radius a sample gives are dropped and
/// the sample taken again, until none is. The sample is at the stride that
/// leaves at most [`FILLET_EDGES`], each edge with its curve's midpoint and
/// its length. Deterministic: the edges only shrink, in id order.
pub(crate) fn fillet_edges(m: &Model, body: Body) -> Result<FilletEdges, String> {
    let mut edges = blendable_edges(m, body, None)?;
    loop {
        let sample = sampled(m, &edges)?;
        let radius = fillet_radius(&sample);
        let kept = blendable_edges(m, body, Some(radius))?
            .into_iter()
            .filter(|e| edges.contains(e))
            .collect::<Vec<_>>();
        if kept.len() == edges.len() {
            return Ok((edges, sample));
        }
        edges = kept;
    }
}

/// The edges the fillet stage may be asked of and the sample taken from
/// them: [`fillet_edges`].
pub(crate) type FilletEdges = (Vec<EdgeId>, Vec<(EdgeId, Point3, f64)>);

/// The sample of [`fillet_edges`]: at most [`FILLET_EDGES`] of `body`'s
/// edges, each with its curve's midpoint and its length.
pub(crate) fn fillet_sampled(m: &Model, body: Body) -> Result<Vec<(EdgeId, Point3, f64)>, String> {
    Ok(fillet_edges(m, body)?.1)
}

/// `edges` taken at the stride that leaves at most [`FILLET_EDGES`].
fn sampled(m: &Model, edges: &[EdgeId]) -> Result<Vec<(EdgeId, Point3, f64)>, String> {
    let stride = edges.len().div_ceil(FILLET_EDGES).max(1);
    let mut out = Vec::new();
    for &e in edges.iter().step_by(stride) {
        let Some((c, range)) = m.edge(e).map_err(|e| e.to_string())?.curve() else {
            continue;
        };
        let curve = m.curve(c).map_err(|e| e.to_string())?;
        let points: Vec<Point3> = (0..=LENGTH_CHORDS)
            .map(|i| curve.point(range.lerp(i as f64 / LENGTH_CHORDS as f64)))
            .collect();
        let length = points.windows(2).map(|w| (w[1] - w[0]).norm()).sum();
        out.push((e, curve.point(range.lerp(0.5)), length));
    }
    Ok(out)
}

/// The fillet stage's radius for a sample: [`FILLET_FRACTION`] of its
/// shortest edge.
pub(crate) fn fillet_radius(sample: &[(EdgeId, Point3, f64)]) -> f64 {
    FILLET_FRACTION * sample.iter().map(|s| s.2).fold(f64::INFINITY, f64::min)
}

/// The operation stages' operands for one solid Arris read, `body` in
/// `m`, whose oracle reading is `oracle`: the `step` operand naming it —
/// with `near` at the oracle's centroid where the file places it more
/// than once — and the box, the drills and the fillet placed from the
/// oracle's centroid and principal axes and from Arris's edges.
/// Deterministic: the same file and the same `expected.json` give the
/// same operands, bit for bit. Errors: an oracle solid with no centroid,
/// volume or inertia, or a body that does not resolve, in words.
pub fn operands(
    fixture: &PartFixture,
    spec: &PartSolid,
    placed_more_than_once: bool,
    m: &Model,
    body: Body,
    oracle: &Measured,
) -> Result<BTreeMap<String, Case>, String> {
    let (Some(c), Some(volume), Some(inertia)) = (oracle.centroid, oracle.volume, oracle.inertia)
    else {
        return Err("the oracle records no centroid, volume or inertia".into());
    };
    let centre = Point3::new(c[0], c[1], c[2]);
    let ([e1, e2, e3], moments) = principal_axes(&inertia);
    let length = REACH * reach(m, body, centre)?;
    let gyration = (moments[0].max(0.0) / volume.abs()).sqrt();
    let part = Step::Read {
        name: "part".into(),
        file: fixture.part.file.clone(),
        sha256: fixture.part.sha256.clone(),
        id: spec.id,
        near: placed_more_than_once.then(|| lits(c)),
    };
    let cut = |tool: &str| Step::Cut {
        name: "result".into(),
        target: "part".into(),
        tool: tool.into(),
    };
    let arr = |v: Vec3| [v.x, v.y, v.z];
    let mut out = BTreeMap::new();

    let side = 2.0 * length;
    out.insert(
        "box_cut".to_string(),
        Case {
            steps: vec![
                part.clone(),
                Step::Profile {
                    name: "square".into(),
                    plane: Plane {
                        origin: lits(c),
                        x: lits(arr(e1)),
                        y: lits(arr(e2)),
                    },
                    outer: Loop::Path {
                        start: [lit(0.0), lit(0.0)],
                        segments: [[side, 0.0], [side, side], [0.0, side], [0.0, 0.0]]
                            .into_iter()
                            .map(|p| Segment::Line {
                                line_to: p.map(lit),
                            })
                            .collect(),
                    },
                    holes: Vec::new(),
                },
                Step::Extrude {
                    name: "box".into(),
                    profile: "square".into(),
                    direction: lits(arr(e3)),
                    length: lit(side),
                },
                cut("box"),
            ],
            result: "result".into(),
        },
    );
    for (stage, axis) in [("drill_x", e1), ("drill_y", e2), ("drill_z", e3)] {
        out.insert(
            stage.to_string(),
            Case {
                steps: vec![
                    part.clone(),
                    Step::Cylinder {
                        name: "drill".into(),
                        base: lits(arr(centre.coords - axis * length)),
                        axis: lits(arr(axis)),
                        radius: lit(DRILL_FRACTION * gyration),
                        height: lit(2.0 * length),
                    },
                    cut("drill"),
                ],
                result: "result".into(),
            },
        );
    }
    let sample = fillet_sampled(m, body)?;
    let radius = fillet_radius(&sample);
    if !sample.is_empty() {
        out.insert(
            "fillet".to_string(),
            Case {
                steps: vec![
                    part,
                    Step::Fillet {
                        name: "result".into(),
                        of: "part".into(),
                        edges: sample
                            .iter()
                            .map(|(_, p, _)| lits([p.x, p.y, p.z]))
                            .collect(),
                        radius: lit(radius),
                    },
                ],
                result: "result".into(),
            },
        );
    }
    Ok(out)
}

/// A case as the recipe the corpus machinery builds, in the part's
/// directory, under its precision and tolerances.
pub fn recipe(fixture: &PartFixture, case: &Case) -> Recipe {
    Recipe {
        description: String::new(),
        params: BTreeMap::new(),
        variants: BTreeMap::new(),
        steps: case.steps.clone(),
        result: case.result.clone(),
        probes: Vec::new(),
        precision: fixture.part.precision,
        tolerances: fixture.part.tolerances,
        analytic: Analytic::default(),
    }
}

#[cfg(not(target_arch = "wasm32"))]
/// Builds `case` in Arris and sorts it against `oracle`, as the
/// differential sorts a drawn recipe (ADR-0024 §2): the result held to
/// the checker at `Full` (leaving what it cannot decide on a NURBS face),
/// to the oracle's counts where `hold_counts` — false where Open CASCADE's
/// healing changed the part's topology, so its counts are not the file's
/// — to its mass properties within both shapes' own tolerances, to a
/// closed mesh, and to the provenance audit of every step. `read` is the
/// solid the case's `step` operand names, already read into its model by
/// a fresh read of the file, which the case then starts from; `None`
/// reads it. Never panics on the kernel's behalf: a kernel panic is the
/// outcome.
pub fn judge(
    fixture: &PartFixture,
    name: &str,
    case: &Case,
    oracle: &OracleCase,
    hold_counts: bool,
    read: Option<(&Model, &ReadBody)>,
) -> Judged {
    let expected = match oracle {
        OracleCase::Built(m) => Some(&**m),
        OracleCase::Refused { .. } => None,
    };
    let built_fixture = Fixture {
        dir: fixture.dir.clone(),
        name: name.to_string(),
        recipe: recipe(fixture, case),
        recipe_sha256: String::new(),
        expected: Expected {
            occt: String::new(),
            recipe_sha256: String::new(),
            results: expected
                .map(|m| BTreeMap::from([("default".to_string(), m.clone())]))
                .unwrap_or_default(),
        },
    };
    // The operand as the part runner read it, where it did: the same
    // model a fresh read of the file gives, without reading it again.
    let start = read.and_then(|(m, back)| {
        let first = case.steps.first()?;
        matches!(first, Step::Read { .. }).then(|| {
            let made = Made {
                body: back.body,
                provenance: back.provenance.clone(),
                inputs: Vec::new(),
            };
            (
                m.clone(),
                BTreeMap::from([(first.name().to_string(), made)]),
            )
        })
    });
    let built = match catch_unwind(AssertUnwindSafe(|| match start {
        Some((m, made)) => corpus::chain_from(&built_fixture, "default", m, made),
        None => corpus::chain_of(&built_fixture, "default"),
    })) {
        Ok(built) => built,
        Err(payload) => return panicked(&*payload).into(),
    };
    let oracle_solid = expected.is_some_and(|m| !m.degenerate);
    let chain = match built {
        Ok(chain) => chain,
        Err(CorpusError::Op { source, .. }) => {
            let outcome = match &source {
                OpError::InvalidInput { .. } | OpError::Internal(arris_ops::Fault::Checker(_)) => {
                    Outcome::CheckerViolation(source.to_string())
                }
                OpError::Internal(_) => Outcome::Internal(refusal(&source)),
                _ if oracle_solid => Outcome::ArrisRefuses(refusal(&source)),
                _ => Outcome::BothRefuse,
            };
            return Judged {
                outcome,
                refused: Some(Refused::Op(Box::new(source))),
            };
        }
        Err(e) => {
            return Outcome::Disagree {
                stage: e.stage(),
                what: format!("the case does not build as written: {e}"),
            }
            .into();
        }
    };
    let expected = match oracle {
        OracleCase::Refused { refused } => return Outcome::OracleRefuses(refused.clone()).into(),
        OracleCase::Built(m) => m,
    };
    if expected.degenerate {
        return Outcome::Disagree {
            stage: Stage::Build,
            what: "Open CASCADE records no solid and Arris builds a body".into(),
        }
        .into();
    }
    let mut held = built_fixture.clone();
    held.recipe.tolerances = held_to(&held, &chain, expected);
    let expected = &net_of_removable(expected, &chain);
    let stages = || -> Result<(), CorpusError> {
        let body = chain.result().ok_or_else(|| CorpusError::Reference {
            fixture: name.to_string(),
            step: "result".into(),
            name: chain.result.clone(),
        })?;
        let report = check_leaving_nurbs(name, &chain.model, body)?;
        if hold_counts {
            corpus::counts_stage(&held, &chain, &report, expected)?;
        }
        let target = corpus::measure_stage(&held, &chain, expected)?;
        corpus::mesh_stage(&held, &chain, &target)?;
        corpus::provenance_stage(&held, &chain)
    };
    match catch_unwind(AssertUnwindSafe(stages)) {
        Ok(Ok(())) => Outcome::Agree,
        Ok(Err(e)) if e.stage() == Stage::Check => Outcome::CheckerViolation(e.to_string()),
        Ok(Err(e)) => Outcome::Disagree {
            stage: e.stage(),
            what: e.to_string(),
        },
        Err(payload) => panicked(&*payload),
    }
    .into()
}

#[cfg(not(target_arch = "wasm32"))]
/// The `write_read` stage: `body` in `m` written to STEP and read back
/// into a model of the same precision — every solid read, the checker at
/// `Full` green but for what it cannot decide on a NURBS face, the same
/// counts, and the same volume, area, centroid and inertia within
/// `tolerances` widened to the body's own (ADR-0023). A refusal of the
/// file read back is `ArrisRefuses` by its kind; Open CASCADE takes no
/// part. Never panics on the kernel's behalf.
pub fn write_read(name: &str, m: &Model, body: Body, tolerances: &Tolerances) -> Judged {
    let run = || -> Result<Judged, (Stage, String)> {
        let fail = |stage: Stage| move |e: String| (stage, e);
        let text = step::write(m, &[body]).map_err(|e| (Stage::Step, e.to_string()))?;
        let mut back = Model::new(m.precision()).map_err(|e| (Stage::Step, e.to_string()))?;
        let read = step_read(&mut back, &text, &ReadOptions::default())
            .map_err(|e| (Stage::ReadBack, e.to_string()))?;
        let [solid] = &read.solids[..] else {
            return Err((
                Stage::ReadBack,
                format!("{} solids read back from one written", read.solids.len()),
            ));
        };
        let read = match &solid.result {
            Ok(read) => read,
            Err(refusal) => {
                return Ok(Judged {
                    outcome: Outcome::ArrisRefuses(refusal.kind().to_string()),
                    refused: Some(Refused::Read(Box::new(refusal.clone()))),
                });
            }
        };
        let was = check_leaving_nurbs(name, m, body).map_err(|e| (Stage::Check, e.to_string()))?;
        let is = check_leaving_nurbs(name, &back, read.body)
            .map_err(|e| (Stage::Check, e.to_string()))?;
        let counts = |r: &arris_io::arris_check::Report| {
            r.euler().map(|l| Counts {
                vertices: l.vertices,
                edges: l.edges,
                faces: l.faces,
                loops: l.loops,
                shells: l.shells,
                solids: 1,
            })
        };
        let (a, b) = (counts(&was), counts(&is));
        if a != b {
            return Err((Stage::Counts, format!("{b:?} read back of {a:?}")));
        }
        let own =
            corpus::within_own_tolerance(tolerances, m, body).map_err(fail(Stage::Measure))?;
        let mass = mass_properties(m, body).map_err(|e| (Stage::Measure, e.to_string()))?;
        let written = Measured {
            degenerate: false,
            counts: a.unwrap_or(Counts {
                vertices: 0,
                edges: 0,
                faces: 0,
                loops: 0,
                shells: 0,
                solids: 0,
            }),
            volume: Some(mass.volume),
            area: Some(mass.area),
            centroid: Some([mass.centroid.x, mass.centroid.y, mass.centroid.z]),
            inertia: Some(std::array::from_fn(|i| {
                std::array::from_fn(|j| mass.inertia[(i, j)])
            })),
            euler_characteristic: None,
            genus: None,
            probes: Vec::new(),
            nurbs_counts: None,
            nurbs_fails: None,
            own: None,
        };
        compare_mass(&back, read.body, &written, "the body written", &own)
            .map_err(fail(Stage::Measure))?;
        Ok(Outcome::Agree.into())
    };
    match catch_unwind(AssertUnwindSafe(run)) {
        Ok(Ok(judged)) => judged,
        Ok(Err((Stage::Check, what))) => Outcome::CheckerViolation(what).into(),
        Ok(Err((stage, what))) => Outcome::Disagree { stage, what }.into(),
        Err(payload) => panicked(&*payload).into(),
    }
}

#[cfg(not(target_arch = "wasm32"))]
/// Every outcome of `fixture`'s battery, by solid key and stage, as
/// [`derive()`] would have it and `part::run` holds it: each `read` solid
/// the fixture has a battery for, read by Arris, `write_read` run on it
/// and each case of its battery judged against `expected.json`. A solid
/// the reader refuses has no battery, and its refusal is its `read`
/// stage, `ArrisRefuses` by its kind. Errors: a file that does not read,
/// or a battery case with no oracle answer (a stale `expected.json`).
pub fn outcomes(fixture: &PartFixture) -> Result<BTreeMap<(String, String), Judged>, String> {
    let mut m = Model::new(fixture.part.precision.precision()).map_err(|e| e.to_string())?;
    let read = read_file(fixture, &mut m)?;
    let mut out = BTreeMap::new();
    for (solid, spec) in read.solids.iter().zip(&fixture.part.solids) {
        let key = key(spec.id, spec.instance);
        if let Err(r) = &solid.result {
            out.insert(
                (key.clone(), "read".to_string()),
                Judged {
                    outcome: Outcome::ArrisRefuses(r.kind().to_string()),
                    refused: Some(Refused::Read(Box::new(r.clone()))),
                },
            );
        }
        let Some(cases) = fixture.part.battery.get(&key) else {
            continue;
        };
        let (ReadOutcome::Read, Ok(back)) = (&spec.outcome, &solid.result) else {
            continue;
        };
        out.insert(
            (key.clone(), "write_read".to_string()),
            write_read(
                &format!("{} {key} write_read", fixture.name),
                &m,
                back.body,
                &fixture.part.tolerances,
            ),
        );
        let hold = holds_counts(fixture, spec.id);
        for (stage, case) in cases {
            let oracle = oracle_case(fixture, &key, stage)?;
            let name = format!("{} {key} {stage}", fixture.name);
            out.insert(
                (key.clone(), stage.clone()),
                judge(fixture, &name, case, oracle, hold, Some((&m, back))),
            );
        }
    }
    Ok(out)
}

/// The oracle's answer to `stage` of the solid `key`. Errors: none
/// recorded — `expected.json` is stale.
pub fn oracle_case<'a>(
    fixture: &'a PartFixture,
    key: &str,
    stage: &str,
) -> Result<&'a OracleCase, String> {
    fixture
        .expected
        .battery
        .get(key)
        .and_then(|cases| cases.get(stage))
        .ok_or_else(|| {
            format!("{key} {stage} has no answer in expected.json: rerun tools/oracle/expected.py")
        })
}

/// Whether the battery's results on the solid `#id` are held to the
/// oracle's counts: where healing changed no topology of every oracle
/// solid of that entity.
pub fn holds_counts(fixture: &PartFixture, id: u64) -> bool {
    (fixture.expected.solids.iter())
        .filter(|s| s.id == id)
        .all(|s| s.held_counts().is_some())
}

fn read_file(fixture: &PartFixture, m: &mut Model) -> Result<step::Read, String> {
    let path = fixture.dir.join(&fixture.part.file);
    let bytes = std::fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    step_read(m, &String::from_utf8_lossy(&bytes), &ReadOptions::default())
        .map_err(|e| format!("{}: {e}", path.display()))
}

/// Derives the battery of every `read` solid of `fixture` ([`operands`]),
/// each solid matched to the oracle's solid of its entity nearest by
/// centroid, as `part::run` matches it. Errors: a file that does not
/// read, a `read` solid the reader refuses or the oracle does not have,
/// in words.
///
/// ```no_run
/// use arris_debug::{battery, fixtures, part};
///
/// let fixture = part::load(&fixtures::corpus_root().join("real/nist-ftc-11")).unwrap();
/// assert_eq!(battery::derive(&fixture).unwrap(), fixture.part.battery);
/// ```
pub fn derive(fixture: &PartFixture) -> Result<BTreeMap<String, BTreeMap<String, Case>>, String> {
    let mut m = Model::new(fixture.part.precision.precision()).map_err(|e| e.to_string())?;
    let read = read_file(fixture, &mut m)?;
    derive_from(fixture, &m, &read)
}

/// [`derive()`] over the file already read, `read` into `m`: what a caller
/// that holds the reading passes, so a large file is read once.
pub fn derive_from(
    fixture: &PartFixture,
    m: &Model,
    read: &step::Read,
) -> Result<BTreeMap<String, BTreeMap<String, Case>>, String> {
    let oracle = &fixture.expected.solids;
    let mut taken = vec![false; oracle.len()];
    let mut out = BTreeMap::new();
    for (solid, spec) in read.solids.iter().zip(&fixture.part.solids) {
        if spec.outcome != ReadOutcome::Read {
            continue;
        }
        let key = key(spec.id, spec.instance);
        let back = solid
            .result
            .as_ref()
            .map_err(|r| format!("{key} is refused: {r}"))?;
        let mass = mass_properties(m, back.body).map_err(|e| format!("{key}: {e}"))?;
        let nearest = (oracle.iter().enumerate())
            .filter(|(k, s)| s.id == spec.id && !taken[*k])
            .filter_map(|(k, s)| {
                let c = s.measured.centroid?;
                Some(((Point3::new(c[0], c[1], c[2]) - mass.centroid).norm(), k))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0));
        let Some((_, k)) = nearest else {
            return Err(format!("{key} has no solid in the oracle's reading"));
        };
        taken[k] = true;
        let placements = (fixture.part.solids.iter())
            .filter(|s| s.id == spec.id)
            .count();
        let cases = operands(
            fixture,
            spec,
            placements > 1,
            m,
            back.body,
            &oracle[k].measured,
        )
        .map_err(|e| format!("{key}: {e}"))?;
        out.insert(key, cases);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The operands are data once written: derived again from the same
    /// file and the same oracle reading, they are the committed ones, bit
    /// for bit — FTC-09's carry axis components near 1e-5, where the two
    /// sides' number formats once parted.
    #[test]
    fn a_battery_derived_again_writes_the_same_operands() {
        for name in ["real/nist-ftc-11", "real/nist-ftc-09"] {
            let fixture =
                crate::part::load(&crate::fixtures::corpus_root().join(name)).expect("the fixture");
            let derived = derive(&fixture).expect("a battery");
            assert_eq!(
                serde_json::to_string(&derived).expect("json"),
                serde_json::to_string(&fixture.part.battery).expect("json"),
                "{name}"
            );
        }
    }

    /// The frame is right-handed, orthonormal, ascending by moment, and
    /// each axis signed by its largest component.
    #[test]
    fn principal_axes_are_a_signed_ascending_frame() {
        // A box 4 × 2 × 1 turned a quarter about z: its long axis is y.
        let (a, b, c) = (4.0f64, 2.0f64, 1.0f64);
        let (ix, iy, iz) = (b * b + c * c, a * a + c * c, a * a + b * b);
        let inertia = [[iy, 0.0, 0.0], [0.0, ix, 0.0], [0.0, 0.0, iz]];
        let ([e1, e2, e3], moments) = principal_axes(&inertia);
        assert_eq!(e1, Vec3::y());
        assert_eq!(e2, Vec3::x());
        assert_eq!(e3, -Vec3::z());
        assert!(moments[0] <= moments[1] && moments[1] <= moments[2]);
        assert!((e1.cross(&e2) - e3).norm() < 1e-15);
    }
}
