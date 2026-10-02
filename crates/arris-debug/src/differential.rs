//! The differential (ADR-0024 §2): recipes drawn by [`prop::recipe`]
//! built by Arris and by the Open CASCADE oracle, every outcome sorted
//! into an [`Outcome`] class.
//!
//! [`run`] writes each drawn recipe as a scratch fixture under
//! `target/inspect/differential/`, has one [`oracle::expected_batch`] call
//! answer all of them, then builds each in Arris and holds the result to
//! the corpus's [`corpus::stages`] — the checker, counts, measure, mesh,
//! probes and provenance — against the oracle's answer, with no dump and
//! no STEP round trip: those read committed files, and the corpus holds
//! them. A panic is caught here, on the test side, with `catch_unwind`;
//! the kernel never catches its own.
//!
//! `Agree`, `BothRefuse`, `ArrisRefuses` and `OracleRefuses` are counted;
//! `Disagree`, `CheckerViolation`, `Panic` and `Internal` fail the run
//! ([`Outcome::fails`]), unless a named [`Exclusion`] covers the failure,
//! which is then counted under its name. The mass properties are held to
//! what both shapes' own tolerances support ([`held_to`]), and the counts
//! net of vertices that only split an edge ([`removable_vertices`]):
//! each kernel vouches for its boundary to within its tolerance, and two
//! valid B-reps of one solid may split an edge differently. Each failing case is shrunk through the
//! strategy's `ValueTree`, the oracle run per candidate through its
//! cache, and printed as a `fixture.json` ready to commit under
//! `tests/fixtures/regression/` (`tests/fixtures/README.md`
//! §Property-test failures).

use std::collections::{BTreeMap, BTreeSet};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use arris_ops::arris_check::arris_topo::{Body, EdgeId, FaceId, Model};
use arris_ops::{Fault, OpError};
use proptest::strategy::{Strategy, ValueTree};

use crate::corpus::{self, CorpusError, Stage};
use crate::fixtures::{self, Fixture, Measured, Recipe, Tolerances};
use crate::oracle::{self, OracleError};
use crate::prop::{self, recipe::recipe};

/// The environment variable that sets how many recipes [`run`] draws.
pub const CASES_VAR: &str = "ARRIS_DIFF_CASES";
/// Recipes per run when [`CASES_VAR`] is unset: what the pre-commit hook
/// runs, a draw small enough to stay a few seconds warm.
pub const DEFAULT_CASES: usize = 32;
/// The environment variable that caps the candidates one failing case's
/// shrink tries.
pub const SHRINK_VAR: &str = "ARRIS_DIFF_SHRINK";
/// Candidates per shrink when [`SHRINK_VAR`] is unset. Each costs an
/// oracle process on a cold cache, so the cap is what bounds a failing
/// run's time; a shrink it stops is still a smaller failing recipe.
pub const DEFAULT_SHRINK: usize = 64;

/// The panic message's start that says the debug build's guard caught an
/// operation's output failing the checker (`arris_ops`'s `verify`): a
/// [`Outcome::CheckerViolation`], not a plain [`Outcome::Panic`].
const CHECKER_GUARD: &str = "kernel bug: an operation's output fails the checker";

/// The number of recipes: [`CASES_VAR`], or [`DEFAULT_CASES`].
pub fn cases() -> usize {
    count(CASES_VAR, DEFAULT_CASES)
}

/// The shrink budget: [`SHRINK_VAR`], or [`DEFAULT_SHRINK`].
pub fn shrink_budget() -> usize {
    count(SHRINK_VAR, DEFAULT_SHRINK)
}

fn count(var: &str, default: usize) -> usize {
    std::env::var(var)
        .ok()
        .and_then(|v| v.trim().parse().ok())
        .unwrap_or(default)
}

/// What one recipe did in the two kernels.
#[derive(Debug, Clone, PartialEq)]
pub enum Outcome {
    /// Both build, and every stage passes.
    Agree,
    /// Open CASCADE builds no solid or refuses the recipe, and Arris
    /// returns a typed refusal.
    BothRefuse,
    /// Open CASCADE builds a solid and Arris returns a typed refusal,
    /// named by [`refusal`]: the kernel's rule working, counted per name.
    ArrisRefuses(String),
    /// Arris builds and Open CASCADE refuses the recipe, with why.
    OracleRefuses(String),
    /// Both answer and a stage differs — or Arris builds a body where
    /// Open CASCADE records no solid, a [`Stage::Build`] disagreement.
    Disagree {
        /// The stage.
        stage: Stage,
        /// What differed.
        what: String,
    },
    /// Arris returns `Ok` with a shape the checker rejects — at `Full`
    /// after the recipe, or at `Fast` in the debug build's guard after an
    /// operation — or an operation's input fails it.
    CheckerViolation(String),
    /// Arris panics.
    Panic(String),
    /// Arris returns `OpError::Internal`: a kernel bug it caught and
    /// typed, named by [`refusal`] — `Internal(Split)`. Whatever the
    /// oracle did, a fault is not a refusal (ADR-0024 step
    /// 5b).
    Internal(String),
    /// A failing outcome a named [`Exclusion`] covers: counted under its
    /// name while the regression fixtures it cites wait for their fix.
    Excluded {
        /// The exclusion's name.
        name: &'static str,
        /// The outcome it covered.
        outcome: Box<Outcome>,
    },
}

impl Outcome {
    /// `true` for the classes that fail the run: a disagreement, a
    /// checker violation, a panic.
    pub fn fails(&self) -> bool {
        matches!(
            self,
            Outcome::Disagree { .. }
                | Outcome::CheckerViolation(_)
                | Outcome::Panic(_)
                | Outcome::Internal(_)
        )
    }

    /// The class's name, as the histogram prints it.
    pub fn class(&self) -> &'static str {
        match self {
            Outcome::Agree => "Agree",
            Outcome::BothRefuse => "BothRefuse",
            Outcome::ArrisRefuses(_) => "ArrisRefuses",
            Outcome::OracleRefuses(_) => "OracleRefuses",
            Outcome::Disagree { .. } => "Disagree",
            Outcome::CheckerViolation(_) => "CheckerViolation",
            Outcome::Panic(_) => "Panic",
            Outcome::Internal(_) => "Internal",
            Outcome::Excluded { .. } => "Excluded",
        }
    }

    /// `true` when both kernels built the recipe and it was compared: an
    /// agreement, or a disagreement past the build.
    pub fn compared(&self) -> bool {
        match self {
            Outcome::Agree => true,
            Outcome::Disagree { stage, .. } => *stage != Stage::Build,
            _ => false,
        }
    }

    /// Whether a shrink candidate's outcome still shows the failure
    /// `self` is: the same class; for a disagreement the same stage, for
    /// a checker violation the same first row (`L4`), for an internal
    /// fault the same fault. A shrink that drifts to another fault would
    /// hand back a fixture of a different bug than the one drawn.
    fn same_failure(&self, other: &Outcome) -> bool {
        match (self, other) {
            (Outcome::Disagree { stage: a, .. }, Outcome::Disagree { stage: b, .. }) => a == b,
            (Outcome::Internal(a), Outcome::Internal(b)) => a == b,
            (Outcome::CheckerViolation(a), Outcome::CheckerViolation(b)) => {
                checker_row(a) == checker_row(b)
            }
            _ => self.class() == other.class(),
        }
    }
}

impl core::fmt::Display for Outcome {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Outcome::Agree | Outcome::BothRefuse => f.write_str(self.class()),
            Outcome::ArrisRefuses(why) | Outcome::OracleRefuses(why) | Outcome::Internal(why) => {
                write!(f, "{}: {why}", self.class())
            }
            Outcome::Disagree { stage, what } => write!(f, "Disagree({stage}): {what}"),
            Outcome::CheckerViolation(what) | Outcome::Panic(what) => {
                write!(f, "{}: {what}", self.class())
            }
            Outcome::Excluded { name, outcome } => write!(f, "Excluded({name}): {outcome}"),
        }
    }
}

/// A kernel failure the draw keeps reaching, excluded by name while the
/// regression fixtures it cites wait for their fix (ADR-0024 §2 and its
/// amendment). It covers the failing outcomes its `covers` names by
/// their symptom — a checker row, a fault, a count gone one way — never
/// by recipe, since which draws reach a kernel bug is not something a
/// generator can state; a failure with another symptom still fails the
/// run.
///
/// The corpus lint holds every cited fixture to still wait under
/// `tests/fixtures/regression/` ([`exclusion_problems`]): the commit
/// that fixes one moves it into its area, the lint fails, and that
/// commit lifts the exclusion.
#[derive(Debug, Clone, Copy)]
pub struct Exclusion {
    /// The name the histogram counts it under.
    pub name: &'static str,
    /// The slugs under `tests/fixtures/regression/` it waits on.
    pub fixtures: &'static [&'static str],
    /// The symptom, in words.
    pub symptom: &'static str,
    /// Whether an outcome shows the symptom; asked of failing outcomes
    /// only.
    pub covers: fn(&Outcome) -> bool,
}

impl Exclusion {
    /// Whether this exclusion covers `outcome`: a failing one with its
    /// symptom.
    pub fn covers(&self, outcome: &Outcome) -> bool {
        outcome.fails() && (self.covers)(outcome)
    }
}

/// A checker violation whose report says `what`.
fn checker_says(outcome: &Outcome, what: &str) -> bool {
    matches!(outcome, Outcome::CheckerViolation(text) if text.contains(what))
}

/// An internal fault named `fault`, as [`refusal`] names it.
fn internal_is(outcome: &Outcome, fault: &str) -> bool {
    matches!(outcome, Outcome::Internal(name) if name == fault)
}

/// Arris's count of `field` and the oracle's, from a counts
/// disagreement's text (`Counts { … faces: 9, … } but the oracle says
/// Counts { … faces: 7, … }`).
fn counts_of(outcome: &Outcome, field: &str) -> Option<(usize, usize)> {
    let Outcome::Disagree {
        stage: Stage::Counts,
        what,
    } = outcome
    else {
        return None;
    };
    let key = format!("{field}: ");
    let mut values = what.match_indices(&key).filter_map(|(at, _)| {
        what[at + key.len()..]
            .split(|c: char| !c.is_ascii_digit())
            .next()?
            .parse()
            .ok()
    });
    Some((values.next()?, values.next()?))
}

/// Every named exclusion, each citing the regression fixtures that pin
/// its failure (ADR-0024).
pub const EXCLUSIONS: &[Exclusion] = &[
    Exclusion {
        name: "loop-crosses-itself",
        fixtures: &["revolve-fuse-extrude-cut-loop-crosses-itself"],
        symptom: "the checker's L5: a loop intersects itself",
        covers: |o| checker_says(o, "intersects itself"),
    },
    Exclusion {
        name: "split-fault",
        fixtures: &["three-cylinders-fuse-split-fault"],
        symptom: "OpError::Internal(Split)",
        covers: |o| internal_is(o, "Internal(Split)"),
    },
    Exclusion {
        name: "geometry-fault",
        fixtures: &["box-revolve-cylinder-chamfer-fuse-geometry-fault"],
        symptom: "OpError::Internal(Geometry)",
        covers: |o| internal_is(o, "Internal(Geometry)"),
    },
    Exclusion {
        name: "seam-fault",
        fixtures: &["mirrored-revolve-common-seam-fault"],
        symptom: "OpError::Internal(Seam)",
        covers: |o| internal_is(o, "Internal(Seam)"),
    },
    Exclusion {
        name: "missing-shell",
        fixtures: &["revolve-cylinder-extrude-fuse-misses-a-shell"],
        symptom: "fewer shells than Open CASCADE: a cavity missing",
        covers: |o| counts_of(o, "shells").is_some_and(|(arris, oracle)| arris < oracle),
    },
    Exclusion {
        name: "extra-faces",
        fixtures: &["revolve-cut-by-extrusion-extra-faces"],
        symptom: "more faces than Open CASCADE",
        covers: |o| counts_of(o, "faces").is_some_and(|(arris, oracle)| arris > oracle),
    },
    Exclusion {
        name: "mesh-not-closed",
        fixtures: &["revolve-box-fuse-mesh-not-closed"],
        symptom: "the tessellation is not closed",
        covers: |o| matches!(o, Outcome::Disagree { stage: Stage::Mesh, what } if what.contains("the mesh is not closed")),
    },
];

/// `outcome`, or [`Outcome::Excluded`] under the first of `exclusions`
/// that covers it.
pub fn excluded(outcome: Outcome, exclusions: &[Exclusion]) -> Outcome {
    match exclusions.iter().find(|e| e.covers(&outcome)) {
        Some(e) => Outcome::Excluded {
            name: e.name,
            outcome: Box::new(outcome),
        },
        None => outcome,
    }
}

/// What is wrong with `exclusions` against the fixture corpus at `root`:
/// one line per exclusion that cites no fixture, and per cited fixture
/// with no `fixture.json` under `root/regression/<slug>/` — gone, or
/// moved into its area by the commit that fixed it, which is the commit
/// that lifts the exclusion.
///
/// ```
/// use arris_debug::differential::{EXCLUSIONS, exclusion_problems};
/// use arris_debug::fixtures::corpus_root;
///
/// assert!(exclusion_problems(&corpus_root(), EXCLUSIONS).is_empty());
/// ```
pub fn exclusion_problems(root: &Path, exclusions: &[Exclusion]) -> Vec<String> {
    let mut problems = Vec::new();
    for e in exclusions {
        if e.fixtures.is_empty() {
            problems.push(format!("exclusion {} cites no regression fixture", e.name));
        }
        for slug in e.fixtures {
            if !root
                .join("regression")
                .join(slug)
                .join("fixture.json")
                .is_file()
            {
                problems.push(format!(
                    "exclusion {} cites regression/{slug}, which is not there: lift the exclusion with the fix that moved it",
                    e.name
                ));
            }
        }
    }
    problems
}

/// The first checker row a report names — `L4` of `L4 f21: hole loop 0
/// lies outside every outer loop` — or `None`.
fn checker_row(text: &str) -> Option<&str> {
    text.lines().find_map(|line| {
        let row = line.split_whitespace().next()?;
        let mut chars = row.chars();
        (chars.next()?.is_ascii_uppercase() && row.len() > 1 && chars.all(|c| c.is_ascii_digit()))
            .then_some(row)
    })
}

/// The name a typed refusal is counted under: the `Reason` of a
/// `Degenerate` — `Degenerate(TangentContact)` — the kinds of an
/// `Unsupported`, the fault of an `Internal`, the variant otherwise.
/// Without the entities or numbers, so one cause is one row.
pub fn refusal(e: &OpError) -> String {
    /// The variant's name from its `Debug`: up to the first field.
    fn head(debug: String) -> String {
        let end = debug.find([' ', '(', '{']).unwrap_or(debug.len());
        debug[..end].to_string()
    }
    match e {
        OpError::Degenerate { reason, .. } => {
            format!("Degenerate({})", head(format!("{reason:?}")))
        }
        OpError::Unsupported { a, b } => format!("Unsupported({} × {})", a.0, b.0),
        OpError::Internal(fault) => format!("Internal({})", head(format!("{fault:?}"))),
        OpError::InvalidInput { .. } => "InvalidInput".into(),
        OpError::Profile(_) => "Profile".into(),
        OpError::Tolerance { .. } => "Tolerance".into(),
        OpError::NotFound(_) => "NotFound".into(),
        OpError::Unkeyed { .. } => "Unkeyed".into(),
        OpError::Rejected(r) => format!("Rejected({})", head(format!("{r:?}"))),
        OpError::Interrupted(_) => "Interrupted".into(),
    }
}

/// The named exclusion that covers a panic caught on the test side, if
/// any: how a property test over the same operations holds the same
/// [`EXCLUSIONS`] as the differential, so one fix lifts both
/// (ADR-0024).
///
/// ```
/// use arris_debug::differential::exclusion_of_panic;
///
/// let guard = "kernel bug: an operation's output fails the checker\n  L5 f18: loop 0 intersects itself";
/// let caught = std::panic::catch_unwind(|| panic!("{guard}")).unwrap_err();
/// assert_eq!(exclusion_of_panic(&*caught).unwrap().name, "loop-crosses-itself");
/// let other = std::panic::catch_unwind(|| panic!("index out of bounds")).unwrap_err();
/// assert!(exclusion_of_panic(&*other).is_none());
/// ```
pub fn exclusion_of_panic(payload: &(dyn std::any::Any + Send)) -> Option<&'static Exclusion> {
    let outcome = panicked(payload);
    EXCLUSIONS.iter().find(|e| e.covers(&outcome))
}

/// The named exclusion that covers a typed kernel fault a property test
/// met, if any: [`exclusion_of_panic`]'s twin for the fault that comes
/// back as `OpError::Internal` rather than as the checker guard's panic,
/// sorted by [`refusal`]'s name as the differential sorts it.
///
/// ```
/// use arris_debug::differential::exclusion_of_error;
/// use arris_geom::GeomError;
/// use arris_math::Tolerance;
/// use arris_ops::{Fault, OpError};
///
/// let fault = OpError::Internal(Fault::Geometry(GeomError::InvalidTolerance(Tolerance::new(0.0, 0.0))));
/// assert_eq!(exclusion_of_error(&fault).unwrap().name, "geometry-fault");
/// ```
pub fn exclusion_of_error(e: &OpError) -> Option<&'static Exclusion> {
    let OpError::Internal(_) = e else {
        return None;
    };
    let outcome = Outcome::Internal(refusal(e));
    EXCLUSIONS.iter().find(|x| x.covers(&outcome))
}

/// The outcome of a caught panic: the debug build's checker guard is a
/// [`Outcome::CheckerViolation`], any other a [`Outcome::Panic`].
pub(crate) fn panicked(payload: &(dyn std::any::Any + Send)) -> Outcome {
    let message = payload
        .downcast_ref::<&str>()
        .map(|s| s.to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "a panic with no message".into());
    if message.starts_with(CHECKER_GUARD) {
        Outcome::CheckerViolation(message)
    } else {
        Outcome::Panic(message)
    }
}

/// Builds `recipe` in Arris under the name `name` and sorts the result
/// against `oracle`: the fixture the oracle answered with (`recipe` and
/// its `expected.json`), or why it refused, held to [`held_to`]'s
/// tolerances and to counts net of [`removable_vertices`]. A failure one
/// of the [`EXCLUSIONS`] covers comes back [`Outcome::Excluded`].
/// Deterministic, and never panics on the kernel's behalf — a kernel
/// panic is the outcome.
///
/// ```
/// use arris_debug::differential::{Outcome, judge};
/// use arris_debug::fixtures::Recipe;
///
/// let recipe: Recipe = serde_json::from_str(
///     r#"{"steps": [{"op": "box", "name": "b", "min": [0, 0, 0], "max": [1, 2, 3]}], "result": "b"}"#,
/// )
/// .unwrap();
/// let outcome = judge("generated/box", &recipe, Err("not asked"));
/// assert!(matches!(outcome, Outcome::OracleRefuses(_)));
/// ```
pub fn judge(name: &str, recipe: &Recipe, oracle: Result<&Fixture, &str>) -> Outcome {
    excluded(sort(name, recipe, oracle), EXCLUSIONS)
}

/// [`judge`] before the [`EXCLUSIONS`].
fn sort(name: &str, recipe: &Recipe, oracle: Result<&Fixture, &str>) -> Outcome {
    let built = match catch_unwind(AssertUnwindSafe(|| corpus::build(name, recipe))) {
        Ok(built) => built,
        Err(payload) => return panicked(&*payload),
    };
    let oracle_solid =
        matches!(oracle, Ok(f) if f.expected.results.get("default").is_some_and(|r| !r.degenerate));
    let chain = match built {
        Ok(chain) => chain,
        Err(CorpusError::Op { source, .. }) => {
            return match &source {
                OpError::InvalidInput { .. } | OpError::Internal(Fault::Checker(_)) => {
                    Outcome::CheckerViolation(source.to_string())
                }
                OpError::Internal(_) => Outcome::Internal(refusal(&source)),
                _ if oracle_solid => Outcome::ArrisRefuses(refusal(&source)),
                _ => Outcome::BothRefuse,
            };
        }
        // The strategy writes only well-formed recipes, so anything else
        // is the generator's fault — and fails the run so it is fixed.
        Err(e) => {
            return Outcome::Disagree {
                stage: e.stage(),
                what: format!("the recipe does not build as written: {e}"),
            };
        }
    };
    let fixture = match oracle {
        Err(why) => return Outcome::OracleRefuses(why.to_string()),
        Ok(fixture) => fixture,
    };
    let Some(expected) = fixture.expected.results.get("default") else {
        return Outcome::OracleRefuses("expected.json has no default result".into());
    };
    if expected.degenerate {
        return Outcome::Disagree {
            stage: Stage::Build,
            what: "Open CASCADE records no solid and Arris builds a body".into(),
        };
    }
    let mut fixture = fixture.clone();
    fixture.recipe.tolerances = held_to(&fixture, &chain, expected);
    let expected = &net_of_removable(expected, &chain);
    match catch_unwind(AssertUnwindSafe(|| {
        corpus::stages(&fixture, &chain, expected)
    })) {
        Ok(Ok(_)) => Outcome::Agree,
        Ok(Err(e)) if e.stage() == Stage::Check => Outcome::CheckerViolation(e.to_string()),
        Ok(Err(e)) => Outcome::Disagree {
            stage: e.stage(),
            what: e.to_string(),
        },
        Err(payload) => panicked(&*payload),
    }
}

/// The tolerances a drawn recipe's result is held to against `expected`:
/// the fixture's own, widened by [`Tolerances::within`] to the sum of the
/// largest tolerance of Arris's result — over its vertices, edges and
/// faces — and the oracle's shape's (`expected.own`). Each kernel
/// vouches for its boundary only to within its own tolerance, and a
/// section fitted in either one moves the mass properties by up to that
/// (ADR-0024). Counts, probes and the mesh are
/// held as the fixture holds them. The fixture's own tolerances where
/// the result does not resolve or the oracle recorded no `own`.
pub fn held_to(fixture: &Fixture, chain: &corpus::Chain, expected: &Measured) -> Tolerances {
    let own = fixture.recipe.tolerances;
    let Some(oracle) = expected.own else {
        return own;
    };
    let m = &chain.model;
    let Some(closure) = chain.result().and_then(|b| m.closure(b).ok()) else {
        return own;
    };
    let arris = closure
        .vertices
        .iter()
        .filter_map(|&v| m.vertex(v).ok().map(|v| v.tolerance()))
        .chain(
            closure
                .edges
                .iter()
                .filter_map(|&e| m.edge(e).ok().map(|e| e.tolerance())),
        )
        .chain(
            closure
                .faces
                .iter()
                .filter_map(|&f| m.face(f).ok().map(|f| f.tolerance())),
        )
        .fold(0.0f64, f64::max);
    own.within(expected, arris + oracle.tolerance)
}

/// The vertices of `body` that only split an edge between two faces in
/// two: exactly two distinct edges of the body meet there, neither closed
/// nor degenerate, and each is used by the same two faces of the body.
/// Removing one merges its two edges and changes no face, loop or shell.
/// The count `oracle.measure.removable_vertices` takes of Open CASCADE's
/// shape, taken of Arris's; zero where the body does not resolve.
pub fn removable_vertices(m: &Model, body: Body) -> usize {
    let Ok(closure) = m.closure(body) else {
        return 0;
    };
    let edges: BTreeSet<EdgeId> = closure.edges.iter().copied().collect();
    let faces: BTreeSet<FaceId> = closure.faces.iter().copied().collect();
    let faces_of = |e: EdgeId| -> BTreeSet<FaceId> {
        m.edge_uses(e)
            .map(|uses| {
                uses.iter()
                    .map(|u| u.face)
                    .filter(|f| faces.contains(f))
                    .collect()
            })
            .unwrap_or_default()
    };
    closure
        .vertices
        .iter()
        .filter(|&&v| {
            let Ok(at) = m.vertex_edges(v) else {
                return false;
            };
            let at: Vec<EdgeId> = at
                .iter()
                .copied()
                .filter(|e| edges.contains(e))
                .filter(|&e| m.edge(e).is_ok_and(|e| e.curve().is_some()))
                .collect();
            let [a, b] = at[..] else {
                return false;
            };
            let open = |e: EdgeId| m.edge(e).is_ok_and(|e| !e.is_closed());
            let (fa, fb) = (faces_of(a), faces_of(b));
            open(a) && open(b) && fa.len() == 2 && fa == fb
        })
        .count()
}

/// `expected` with its vertex and edge counts moved by the difference
/// between Arris's [`removable_vertices`] and the oracle's
/// (`expected.own`), so the counts stage compares the two net of them —
/// a vertex that merges two edges into one is not a different solid.
/// Unchanged where the oracle recorded no `own`.
pub(crate) fn net_of_removable(expected: &Measured, chain: &corpus::Chain) -> Measured {
    let mut out = expected.clone();
    let (Some(own), Some(body)) = (expected.own, chain.result()) else {
        return out;
    };
    let arris = removable_vertices(&chain.model, body);
    for count in [&mut out.counts.vertices, &mut out.counts.edges] {
        *count = (*count + arris).saturating_sub(own.removable_vertices);
    }
    out
}

/// One drawn recipe and what it did.
#[derive(Debug, Clone)]
pub struct Case {
    /// Its place in the draw, from 0.
    pub index: usize,
    /// The recipe.
    pub recipe: Recipe,
    /// Its outcome.
    pub outcome: Outcome,
    /// Arris's wall clock on it: the build and the stages.
    pub arris: Duration,
}

/// A failing case shrunk.
#[derive(Debug, Clone)]
pub struct Failure {
    /// The case as drawn.
    pub index: usize,
    /// The drawn outcome.
    pub outcome: Outcome,
    /// The smallest recipe found that fails the same way.
    pub recipe: Recipe,
    /// Its outcome.
    pub shrunk: Outcome,
    /// The candidates the shrink tried.
    pub tried: usize,
}

impl Failure {
    /// The shrunk recipe as a `fixture.json`, its description saying
    /// where it came from and how it fails — ready to commit under
    /// `tests/fixtures/regression/<slug>/` beside the oracle's
    /// `expected.json`.
    pub fn fixture_json(&self, seed: &[u8; 32]) -> String {
        let mut recipe = self.recipe.clone();
        recipe.description = format!(
            "drawn by the differential, case {} of seed {}, shrunk in {} candidates: {}",
            self.index,
            prop::seed_hex(seed),
            self.tried,
            first_line(&self.shrunk.to_string()),
        );
        serde_json::to_string_pretty(&recipe).unwrap_or_else(|e| e.to_string())
    }
}

fn first_line(text: &str) -> &str {
    text.lines().next().unwrap_or("")
}

/// A differential run: every case in draw order, the failing ones shrunk.
#[derive(Debug, Clone)]
pub struct Run {
    /// The seed drawn from.
    pub seed: [u8; 32],
    /// Every case, in draw order.
    pub cases: Vec<Case>,
    /// Every failing case, shrunk, in draw order.
    pub failures: Vec<Failure>,
    /// The wall clock of the whole run, shrinking apart.
    pub elapsed: Duration,
}

impl Run {
    /// Cases per class, and per refusal name under `ArrisRefuses`; the
    /// keys are the class names, and `ArrisRefuses/<name>` for a refusal.
    pub fn histogram(&self) -> BTreeMap<String, usize> {
        let mut out = BTreeMap::new();
        for case in &self.cases {
            *out.entry(case.outcome.class().to_string()).or_default() += 1;
            match &case.outcome {
                Outcome::Excluded { name, .. } => {
                    *out.entry(format!("Excluded/{name}")).or_default() += 1;
                }
                Outcome::ArrisRefuses(name) | Outcome::Internal(name) => {
                    *out.entry(format!("{}/{name}", case.outcome.class()))
                        .or_default() += 1;
                }
                Outcome::Disagree { stage, .. } => {
                    *out.entry(format!("Disagree/{stage}")).or_default() += 1;
                }
                _ => {}
            }
        }
        out
    }

    /// The summary a test prints: the seed, the count and time, the share
    /// of the draw that reached a comparison, and the histogram.
    pub fn report(&self) -> String {
        let n = self.cases.len();
        let compared = self.cases.iter().filter(|c| c.outcome.compared()).count();
        let arris: Duration = self.cases.iter().map(|c| c.arris).sum();
        let per = |d: Duration| d.as_secs_f64() / n.max(1) as f64;
        let mut out = format!(
            "differential: {n} recipes, {}={}\n\
             {compared} of {n} compared ({:.0}%), the rest refused by one kernel or both\n\
             {:.2} s wall clock, {:.3} s per recipe; Arris {:.3} s per recipe on one thread\n",
            prop::SEED_VAR,
            prop::seed_hex(&self.seed),
            100.0 * compared as f64 / n.max(1) as f64,
            self.elapsed.as_secs_f64(),
            per(self.elapsed),
            per(arris),
        );
        for (class, count) in self.histogram() {
            let indent = if class.contains('/') { "    " } else { "  " };
            let name = class.rsplit('/').next().unwrap_or(&class);
            out.push_str(&format!("{indent}{name:<40} {count:>5}\n"));
        }
        out
    }

    /// Every failure: how it failed as drawn, how it fails shrunk, and its
    /// `fixture.json`.
    pub fn failures_text(&self) -> String {
        let mut out = String::new();
        for f in &self.failures {
            out.push_str(&format!(
                "case {}: {}\nshrunk in {} candidates: {}\n{}\n\n",
                f.index,
                f.outcome,
                f.tried,
                f.shrunk,
                f.fixture_json(&self.seed)
            ));
        }
        out
    }
}

/// The scratch directory case `name` of `family` is written to: runs
/// over different families may be at once, so they never share one.
fn case_dir(family: &str, name: &str) -> PathBuf {
    oracle::scratch_dir()
        .join("differential")
        .join(family)
        .join(name)
}

/// The oracle's answer for a scratch fixture it answered: the fixture
/// loaded, or why it could not be.
fn load(dir: &Path, answer: Result<(), String>) -> Result<Fixture, String> {
    answer?;
    fixtures::load(dir).map_err(|e| e.to_string())
}

/// Draws `n` recipes from `seed`, has the oracle answer them in one batch
/// and judges each ([`judge`]) on as many threads as the machine has,
/// then shrinks every failing case ([`shrink`], [`shrink_budget`]
/// candidates at most). Deterministic in everything but the times.
/// Errors: the oracle's, when it could not run or a scratch fixture could
/// not be written — never a refusal, which is an outcome.
///
/// ```no_run
/// use arris_debug::{differential, prop};
///
/// let run = differential::run(&prop::DEFAULT_SEED, 8).unwrap();
/// println!("{}", run.report());
/// assert!(run.failures.is_empty(), "{}", run.failures_text());
/// ```
pub fn run(seed: &[u8; 32], n: usize) -> Result<Run, OracleError> {
    run_over("", recipe(), seed, n)
}

/// [`run`] over the recipes `strategy` draws instead of [`recipe`]'s: a
/// family of them, held to Open CASCADE alone, its scratch cases under
/// `family` so a run of another family at once does not share them.
/// Errors: [`run`]'s.
pub fn run_over<S>(family: &str, strategy: S, seed: &[u8; 32], n: usize) -> Result<Run, OracleError>
where
    S: Strategy<Value = Recipe>,
{
    let start = Instant::now();
    let mut runner = prop::runner_with_seed(seed);
    let mut trees = Vec::with_capacity(n);
    for _ in 0..n {
        trees.push(
            strategy
                .new_tree(&mut runner)
                .map_err(|e| OracleError::Environment {
                    message: format!("the recipe strategy drew nothing: {e}"),
                })?,
        );
    }
    let recipes: Vec<Recipe> = trees.iter().map(|t| t.current()).collect();
    let names: Vec<String> = (0..n).map(|i| format!("{i:04}")).collect();
    let dirs: Vec<PathBuf> = names.iter().map(|name| case_dir(family, name)).collect();
    for (dir, recipe) in dirs.iter().zip(&recipes) {
        oracle::write_recipe(dir, recipe)?;
    }
    let answers = oracle::expected_batch(&dirs)?;
    let oracles: Vec<Result<Fixture, String>> = dirs
        .iter()
        .zip(answers)
        .map(|(dir, answer)| load(dir, answer))
        .collect();

    // Each case builds in its own model, so the cases run at once; the
    // results go back in draw order.
    let judged: Mutex<Vec<Option<(Outcome, Duration)>>> = Mutex::new(vec![None; n]);
    let next = AtomicUsize::new(0);
    let threads = std::thread::available_parallelism().map_or(1, |p| p.get());
    std::thread::scope(|s| {
        for _ in 0..threads.min(n) {
            s.spawn(|| {
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    if i >= n {
                        break;
                    }
                    let begun = Instant::now();
                    let outcome = judge(
                        &format!("differential/{}", names[i]),
                        &recipes[i],
                        oracles[i].as_ref().map_err(String::as_str),
                    );
                    let took = begun.elapsed();
                    judged.lock().unwrap_or_else(|e| e.into_inner())[i] = Some((outcome, took));
                }
            });
        }
    });
    let judged = judged.into_inner().unwrap_or_else(|e| e.into_inner());
    let cases: Vec<Case> = recipes
        .into_iter()
        .zip(judged)
        .enumerate()
        .map(|(index, (recipe, judged))| {
            let (outcome, arris) =
                judged.unwrap_or_else(|| (Outcome::Panic("never judged".into()), Duration::ZERO));
            Case {
                index,
                recipe,
                outcome,
                arris,
            }
        })
        .collect();
    let elapsed = start.elapsed();

    let mut failures = Vec::new();
    let budget = shrink_budget();
    for (case, tree) in cases.iter().zip(trees.iter_mut()) {
        if !case.outcome.fails() {
            continue;
        }
        let dir = case_dir(family, &format!("{}-shrink", names[case.index]));
        let name = format!("differential/{}-shrink", names[case.index]);
        let (recipe, shrunk, tried) = shrink(tree, &case.outcome, budget, |candidate| {
            oracle::write_recipe(&dir, candidate)?;
            let answer = oracle::expected_batch(std::slice::from_ref(&dir))?
                .pop()
                .unwrap_or_else(|| Err("no answer".into()));
            let fixture = load(&dir, answer);
            Ok(judge(
                &name,
                candidate,
                fixture.as_ref().map_err(String::as_str),
            ))
        })?;
        let recipe = with_held(&name, &dir, recipe)?;
        failures.push(Failure {
            index: case.index,
            outcome: case.outcome.clone(),
            recipe,
            shrunk,
            tried,
        });
    }
    Ok(Run {
        seed: *seed,
        cases,
        failures,
        elapsed,
    })
}

/// `recipe` with the tolerances [`held_to`] held it to, so the
/// `fixture.json` a failure prints is judged in the corpus as it was
/// here: the oracle's answer through its cache (the shrink asked for it
/// already), and `recipe` unchanged where either kernel builds nothing.
/// Errors: the oracle's.
fn with_held(name: &str, dir: &Path, mut recipe: Recipe) -> Result<Recipe, OracleError> {
    oracle::write_recipe(dir, &recipe)?;
    let answer = oracle::expected_batch(&[dir.to_path_buf()])?
        .pop()
        .unwrap_or_else(|| Err("no answer".into()));
    let Ok(fixture) = load(dir, answer) else {
        return Ok(recipe);
    };
    let built = catch_unwind(AssertUnwindSafe(|| corpus::build(name, &recipe)));
    if let (Ok(Ok(chain)), Some(expected)) = (built, fixture.expected.results.get("default")) {
        recipe.tolerances = held_to(&fixture, &chain, expected);
    }
    Ok(recipe)
}

/// Shrinks the failing case `tree` holds, whose outcome is `first`:
/// simplifies while `judge` still finds the same failure
/// ([`Outcome::fails`], the same class and for a disagreement the same
/// stage), complicates back when it does not, for at most `budget`
/// candidates. Returns the smallest failing recipe found, its outcome and
/// the candidates tried. Errors: `judge`'s.
pub fn shrink<T, F>(
    tree: &mut T,
    first: &Outcome,
    budget: usize,
    mut judge: F,
) -> Result<(Recipe, Outcome, usize), OracleError>
where
    T: ValueTree<Value = Recipe>,
    F: FnMut(&Recipe) -> Result<Outcome, OracleError>,
{
    let mut best = (tree.current(), first.clone());
    let mut tried = 0;
    if !tree.simplify() {
        return Ok((best.0, best.1, tried));
    }
    while tried < budget {
        let candidate = tree.current();
        let outcome = judge(&candidate)?;
        tried += 1;
        if outcome.fails() && first.same_failure(&outcome) {
            best = (candidate, outcome);
            if !tree.simplify() {
                break;
            }
        } else if !tree.complicate() {
            break;
        }
    }
    Ok((best.0, best.1, tried))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fixtures::Step;

    fn a_box(max: [f64; 3]) -> Recipe {
        serde_json::from_str(&format!(
            r#"{{"steps": [{{"op": "box", "name": "b", "min": [0, 0, 0], "max": {max:?}}}], "result": "b"}}"#
        ))
        .unwrap()
    }

    /// A box's own fixture: the corpus's `primitive/box` answer, which is
    /// the oracle's for a 40 × 30 × 10 box, under this recipe.
    fn answered(recipe: &Recipe) -> Fixture {
        let mut fixture = fixtures::load(&fixtures::corpus_root().join("primitive/box")).unwrap();
        fixture.recipe = recipe.clone();
        fixture
            .expected
            .results
            .get_mut("default")
            .unwrap()
            .probes
            .clear();
        fixture
    }

    #[test]
    fn each_answer_sorts_into_its_class() {
        let good = a_box([40.0, 30.0, 10.0]);
        let fixture = answered(&good);
        assert_eq!(judge("t/agree", &good, Ok(&fixture)), Outcome::Agree);

        // A different box against the same answer: counts agree, the
        // volume does not.
        let other = a_box([40.0, 30.0, 11.0]);
        let Outcome::Disagree { stage, .. } = judge("t/other", &other, Ok(&answered(&other)))
        else {
            panic!("a larger box is not the oracle's");
        };
        assert_eq!(stage, Stage::Measure);

        // Arris builds, the oracle refuses.
        assert!(matches!(
            judge("t/refused", &good, Err("no")),
            Outcome::OracleRefuses(_)
        ));

        // Arris refuses a box with no height: counted by its reason when
        // the oracle builds, both refuse when it does not.
        let flat = a_box([40.0, 30.0, 0.0]);
        assert_eq!(
            judge("t/flat", &flat, Ok(&answered(&flat))),
            Outcome::ArrisRefuses("Degenerate(NotPositive)".into())
        );
        assert_eq!(judge("t/flat", &flat, Err("no")), Outcome::BothRefuse);
        let mut none = answered(&flat);
        none.expected.results.get_mut("default").unwrap().degenerate = true;
        assert_eq!(judge("t/flat", &flat, Ok(&none)), Outcome::BothRefuse);

        // Arris builds a body where the oracle records none.
        let mut empty = answered(&good);
        empty
            .expected
            .results
            .get_mut("default")
            .unwrap()
            .degenerate = true;
        assert!(matches!(
            judge("t/empty", &good, Ok(&empty)),
            Outcome::Disagree {
                stage: Stage::Build,
                ..
            }
        ));
    }

    #[test]
    fn a_panic_is_a_checker_violation_only_from_the_guard() {
        let guard = panicked(&format!("{CHECKER_GUARD}\nL4 …"));
        assert!(matches!(guard, Outcome::CheckerViolation(_)));
        assert_eq!(
            panicked(&"index out of bounds"),
            Outcome::Panic("index out of bounds".into())
        );
        assert!(guard.fails() && !Outcome::BothRefuse.fails());
    }

    /// Every vertex of a box meets three edges, and a cylinder's seam
    /// vertex meets its seam and a closed cap circle: none only splits
    /// an edge.
    #[test]
    fn a_primitive_has_no_removable_vertex() {
        use arris_ops::arris_check::arris_topo::arris_math::{Axis, Point3};

        let mut m = Model::default();
        let (cuboid, _) =
            crate::unmetered::primitive_box(&mut m, Point3::origin(), Point3::new(4.0, 3.0, 2.0))
                .unwrap();
        let (cylinder, _) =
            crate::unmetered::primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 2.0, 5.0)
                .unwrap();
        assert_eq!(removable_vertices(&m, cuboid), 0);
        assert_eq!(removable_vertices(&m, cylinder), 0);
    }

    #[test]
    fn an_exclusion_covers_its_symptom_and_nothing_else() {
        let l4 = Outcome::CheckerViolation("L5 f21: loop 0 intersects itself".into());
        let e1 = Outcome::CheckerViolation("E1 e7: the range is not increasing".into());
        let Outcome::Excluded { name, outcome } = excluded(l4.clone(), EXCLUSIONS) else {
            panic!("the L4 symptom is excluded");
        };
        assert_eq!(name, "loop-crosses-itself");
        assert_eq!(*outcome, l4);
        assert!(!excluded(l4, EXCLUSIONS).fails());
        assert_eq!(excluded(e1.clone(), EXCLUSIONS), e1);
        assert_eq!(excluded(Outcome::Agree, EXCLUSIONS), Outcome::Agree);
        let fewer = Outcome::Disagree {
            stage: Stage::Counts,
            what: "counts Counts { vertices: 6, edges: 9, faces: 5, loops: 5, shells: 1, solids: 1 } but the oracle says Counts { vertices: 6, edges: 9, faces: 5, loops: 6, shells: 2, solids: 1 }".into(),
        };
        assert_eq!(counts_of(&fewer, "shells"), Some((1, 2)));
        assert!(matches!(
            excluded(fewer, EXCLUSIONS),
            Outcome::Excluded {
                name: "missing-shell",
                ..
            }
        ));
        let more = Outcome::Disagree {
            stage: Stage::Counts,
            what: "counts Counts { vertices: 7, edges: 10, faces: 5, loops: 5, shells: 1, solids: 1 } but the oracle says Counts { vertices: 6, edges: 9, faces: 5, loops: 5, shells: 1, solids: 1 }".into(),
        };
        assert!(
            excluded(more, EXCLUSIONS).fails(),
            "a vertex more is no exclusion's"
        );
        assert!(internal_is(
            &Outcome::Internal("Internal(Split)".into()),
            "Internal(Split)"
        ));
        assert!(matches!(
            excluded(Outcome::Internal("Internal(Seam)".into()), EXCLUSIONS),
            Outcome::Excluded {
                name: "seam-fault",
                ..
            }
        ));
        assert!(
            excluded(Outcome::Internal("Internal(Lumps)".into()), EXCLUSIONS).fails(),
            "a lumps fault is no seam fault"
        );
        assert_eq!(
            checker_row("the result fails the checker:\nL5 f47: loop 0 intersects itself"),
            Some("L5")
        );
    }

    #[test]
    fn an_exclusion_whose_fixture_is_gone_is_a_problem() {
        let gone = Exclusion {
            name: "gone",
            fixtures: &["no-such-fixture"],
            symptom: "",
            covers: |_| true,
        };
        let problems = exclusion_problems(&fixtures::corpus_root(), &[gone]);
        assert_eq!(problems.len(), 1, "{problems:?}");
        assert!(problems[0].contains("regression/no-such-fixture"));
        let none = Exclusion {
            fixtures: &[],
            ..gone
        };
        assert!(exclusion_problems(&fixtures::corpus_root(), &[none])[0].contains("cites no"));
    }

    #[test]
    fn the_histogram_counts_classes_and_refusals() {
        let case = |outcome| Case {
            index: 0,
            recipe: a_box([1.0, 1.0, 1.0]),
            outcome,
            arris: Duration::ZERO,
        };
        let run = Run {
            seed: prop::DEFAULT_SEED,
            cases: vec![
                case(Outcome::Agree),
                case(Outcome::ArrisRefuses("Degenerate(TangentContact)".into())),
                case(Outcome::ArrisRefuses("Degenerate(TangentContact)".into())),
                case(Outcome::Disagree {
                    stage: Stage::Probes,
                    what: String::new(),
                }),
            ],
            failures: Vec::new(),
            elapsed: Duration::ZERO,
        };
        let h = run.histogram();
        assert_eq!(h["Agree"], 1);
        assert_eq!(h["ArrisRefuses"], 2);
        assert_eq!(h["ArrisRefuses/Degenerate(TangentContact)"], 2);
        assert_eq!(h["Disagree/probes"], 1);
        assert!(run.report().contains("2 of 4 compared (50%)"));
    }

    /// The shrink keeps the failure while it simplifies: a stand-in judge
    /// that fails every recipe with a `cut` shrinks a drawn one to a
    /// recipe that still has one, and no longer than it was.
    #[test]
    fn a_shrink_keeps_the_failure() {
        let fails = |r: &Recipe| r.steps.iter().any(|s| matches!(s, Step::Cut { .. }));
        let mut runner = prop::runner_with_seed(&prop::DEFAULT_SEED);
        let failure = Outcome::Panic("a cut".into());
        let mut shrunk_one = false;
        for _ in 0..64 {
            let mut tree = recipe().new_tree(&mut runner).unwrap();
            let drawn = tree.current();
            if !fails(&drawn) {
                continue;
            }
            let (small, outcome, tried) = shrink(&mut tree, &failure, 256, |r| {
                Ok(if fails(r) {
                    failure.clone()
                } else {
                    Outcome::Agree
                })
            })
            .unwrap();
            assert!(fails(&small));
            assert_eq!(outcome, failure);
            assert!(small.steps.len() <= drawn.steps.len());
            assert!(tried > 0);
            shrunk_one = true;
            break;
        }
        assert!(shrunk_one, "no draw of 64 has a cut");
    }
}
