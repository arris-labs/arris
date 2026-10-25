//! The corpus runner: the fixture test of `docs/ROADMAP.md` §Fixtures,
//! one call per fixture and variant. [`run`] builds the recipe in Arris
//! and runs its [`Stage`]s in order, one function each: the checker at
//! `Full` ([`check_stage`]); counts and genus against the oracle's
//! `expected.json`, or the recipe's own where it states a convention
//! Arris does not follow, `analytic.counts_differ` ([`counts_stage`]);
//! volume, area, centroid and inertia over the B-Rep within the
//! fixture's tolerances of the oracle's, or of the recipe's closed forms
//! where it states the oracle's are wrong, `analytic.measure_differs`,
//! ADR-0015 ([`measure_stage`]); the mesh at `mesh_chord`, closed, its
//! signed volume within `mesh_volume_rel` of that volume, with the corner
//! block asked for and every face-local vertex held to ADR-0012's two
//! invariants so the whole corpus covers them ([`mesh_stage`]); the
//! probes against the oracle's classes ([`probe_stage`]); the provenance
//! accounting of every step ([`provenance_stage`]); STEP written and read
//! back by the oracle (`compare.py`); and the text dump diffed against
//! the committed `dump.txt` — written only under `ARRIS_BLESS=1`. The six
//! stages that read no file are [`stages`], what the differential holds
//! a generated recipe to. Every stage that fails is a typed
//! [`CorpusError`] saying which fixture, which stage and what differed.
//! A `profile`
//! step builds a `geom::Profile` kept beside the bodies for the sweep
//! steps that name it; it makes no body and needs no accounting. A result the
//! oracle recorded no solid for (`expected.degenerate`) must fail with
//! `OpError::Degenerate`, and one the recipe marks `analytic.expect_error`
//! must fail with that typed refusal; either ends the run there, the
//! oracle's numbers recorded but not compared.

use crate::unmetered::step_read;
use crate::unmetered::{mass_properties, surface_properties};
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use crate::dump::dump_text;
use crate::fixtures::geom::{self as geom_spec, build_profile};
use crate::fixtures::{
    self, Class, Counts, ExpectError, Expected, ExprError, Fixture, FixtureError, Measured, Num,
    PathSegment as RecipeSegment, Recipe, Rotate, ShellSide, Step, SweepLaw, SweepPath, Tolerances,
};
use crate::oracle::{self, OracleError};
use crate::polyhedron::{PolyhedronError, polyhedron};
use arris_check::classify::{Classification, classify_point};
use arris_check::{Level, LumpError, Report, Unchecked, check, lumps};
use arris_geom::{CurveKind, GeomKind, Surface, SurfaceKind};
use arris_geom::{
    FrameLaw, LoftError, NurbsCurve, Path as GeomPath, PathError, PathSegment as GeomSegment,
    Profile,
};
use arris_io::step::{self as io_step, StepError};
use arris_math::nalgebra::UnitQuaternion;
use arris_math::{
    Axis, Control, Frame, FrameError, Isometry, Point3, Reflection, ReflectionError, UnitVec3, Vec3,
};
use arris_mesh::{MeshRequest, TriMesh};
use arris_ops::{
    BlendReason, BooleanReason, InputReason, OffsetReason, OpError, Reason, ShellReason,
    SplitReason, SweepReason, common,
};
use arris_topo::FaceId;
use arris_topo::builder::{Assembly, Builder, FaceSpec};
use arris_topo::entity::BodyKind;
use arris_topo::{Body, Edge, EntityId, Face, Model, Orientation, Provenance, Shape, TopoError};
use sha2::{Digest, Sha256};

mod inputs;
mod read_back;
mod stages;
mod step;

use {inputs::*, read_back::*, stages::*, step::*};

pub use inputs::{Inputs, inputs, inputs_for_step};
pub(crate) use read_back::{check_leaving_nurbs, check_leaving_nurbs_pairs};
pub use read_back::{read_back_nurbs_stage, read_back_stage};
pub use stages::{
    Target, check_stage, counts_stage, measure_stage, mesh_stage, probe_stage, provenance_stage,
    stages, within_own_tolerance,
};
pub(crate) use stages::{compare_mass, mesh_check};

/// The environment variable that makes [`run`] write `dump.txt` instead
/// of diffing against it.
pub const BLESS_VAR: &str = "ARRIS_BLESS";

/// Why a fixture did not pass, by stage.
#[derive(Debug, thiserror::Error)]
pub enum CorpusError {
    /// The fixture directory could not be loaded.
    #[error(transparent)]
    Fixture(#[from] FixtureError),
    /// The variant is not in the recipe.
    #[error("{fixture}: no variant {variant:?}")]
    Variant {
        /// The fixture.
        fixture: String,
        /// The variant asked for.
        variant: String,
    },
    /// A step refers to a step that does not exist or comes later.
    #[error("{fixture}: step {step:?} refers to {name:?}, which is not a step before it")]
    Reference {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The name it refers to.
        name: String,
    },
    /// A `fuse` or `cut` step names its operands both singly and as a
    /// list, or neither.
    #[error("{fixture}: step {step:?}: {what}")]
    Operands {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// What is wrong.
        what: String,
    },
    /// A number in the recipe could not be evaluated.
    #[error("{fixture}: step {step:?}: {source}")]
    Expression {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The cause.
        source: ExprError,
    },
    /// A `profile` step's plane or numbers do not build.
    #[error("{fixture}: step {step:?}: profile: {source}")]
    Profile {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The cause, boxed to keep the error small.
        source: Box<geom_spec::BuildError>,
    },
    /// A `sweep` step's NURBS path segment does not build.
    #[error("{fixture}: step {step:?}: path segment {segment}: {source}")]
    Path {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The segment.
        segment: usize,
        /// The cause.
        source: arris_geom::GeomError,
    },
    /// A point naming an edge to blend is not on exactly one edge of the
    /// body: it classifies to a vertex, a face, the inside or the
    /// outside, or lies within `probe` of two edges.
    #[error("{fixture}: step {step:?}: the edge point {point:?} {what}")]
    EdgePoint {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The point.
        point: [f64; 3],
        /// What it names instead of one edge.
        what: String,
    },
    /// An `offset` step's face point does not name one face of its
    /// body: it classifies to an edge, a vertex, the inside or the
    /// outside.
    #[error("{fixture}: step {step:?}: the face point {point:?} {what}")]
    FacePoint {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The point.
        point: [f64; 3],
        /// What it names instead of one face.
        what: String,
    },
    /// The recipe's `precision` is not a consistent
    /// `arris_math::Precision`, so no model could be created for it.
    #[error("{fixture}: precision: {source}")]
    Precision {
        /// The fixture.
        fixture: String,
        /// The cause.
        source: TopoError,
    },
    /// A recipe axis, or a `split` step's plane frame, is not one.
    #[error("{fixture}: step {step:?}: axis: {source}")]
    Axis {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The cause.
        source: FrameError,
    },
    /// A `mirror` step's plane has no normal.
    #[error("{fixture}: step {step:?}: plane: {source}")]
    Plane {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The cause.
        source: ReflectionError,
    },
    /// A `polyhedron` step's points and faces describe no builder.
    #[error("{fixture}: step {step:?}: {source}")]
    Polyhedron {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The cause.
        source: PolyhedronError,
    },
    /// A step names an operation Arris does not have yet: the oracle builds
    /// it and the recipe parses, the interpreter refuses.
    #[error("{fixture}: step {step:?}: {what}")]
    Unsupported {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// What is not supported.
        what: String,
    },
    /// The result step was expected to fail with a typed error —
    /// `expected.degenerate`, or the recipe's `analytic.expect_error` —
    /// and built a body, or failed with another error.
    #[error("{fixture}: step {step:?}: expected {expected}, found {found}")]
    Expectation {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The error expected.
        expected: String,
        /// What happened instead.
        found: String,
    },
    /// An operation failed.
    #[error("{fixture}: step {step:?}: {source}")]
    Op {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The cause.
        source: OpError,
    },
    /// The result fails the checker at `Full`, or a `Full` row could not
    /// be decided.
    #[error("{fixture}: the result fails the checker:\n{report}")]
    Check {
        /// The fixture.
        fixture: String,
        /// The report.
        report: Box<Report>,
    },
    /// The result's lumps could not be read (`arris_check::lumps`), which
    /// for a result the checker passed at `Full` is a kernel bug.
    #[error("{fixture}: lumps: {source}")]
    Lumps {
        /// The fixture.
        fixture: String,
        /// Why.
        source: LumpError,
    },
    /// Arris classifies a probe point differently from the oracle.
    #[error(
        "{fixture}: probe {label:?} at {point:?}: Arris says {found}, the oracle says {expected:?}"
    )]
    Probe {
        /// The fixture.
        fixture: String,
        /// The probe's label.
        label: String,
        /// The point.
        point: [f64; 3],
        /// The oracle's class.
        expected: Class,
        /// What Arris said — a classification, or why it could not.
        found: String,
    },
    /// The counts differ from the oracle's (or from the recipe's own,
    /// under `analytic.counts_differ`).
    #[error("{fixture}: counts {found:?} but the oracle says {expected:?}")]
    Counts {
        /// The fixture.
        fixture: String,
        /// The oracle's counts.
        expected: Counts,
        /// Arris's.
        found: Counts,
    },
    /// The genus differs from the oracle's.
    #[error("{fixture}: genus {found} but the oracle says {expected}")]
    Genus {
        /// The fixture.
        fixture: String,
        /// The oracle's genus.
        expected: i64,
        /// Arris's.
        found: i64,
    },
    /// The result could not be written as STEP.
    #[error("{fixture}: STEP: {source}")]
    Step {
        /// The fixture.
        fixture: String,
        /// The cause.
        source: StepError,
    },
    /// Arris's own STEP did not parse as Part 21, or parsed to another
    /// number of instances than the writer defined.
    #[error("{fixture}: its STEP does not parse back: {what}")]
    Part21 {
        /// The fixture.
        fixture: String,
        /// The parse error, or the counts that differ.
        what: String,
    },
    /// The oracle did not match Arris's STEP, or could not run.
    #[error(transparent)]
    Oracle(#[from] OracleError),
    /// Open CASCADE's own STEP of the result did not read back
    /// ([`read_back_stage`]): a parse error, or a solid refused.
    #[error("{fixture}: Open CASCADE's STEP ({file}) reads back {what}")]
    ReadBack {
        /// The fixture.
        fixture: String,
        /// Which file: plain, or converted to NURBS.
        file: &'static str,
        /// What went wrong.
        what: String,
    },
    /// A mass property could not be computed, or is not the oracle's
    /// within the fixture's tolerance for it.
    #[error("{fixture}: measure: {what}")]
    Measure {
        /// The fixture.
        fixture: String,
        /// Which quantity differed, and by how much.
        what: String,
    },
    /// The mesh could not be built, is not closed, or its volume is not
    /// the oracle's within `mesh_volume_rel`.
    #[error("{fixture}: mesh at chord {chord}: {what}")]
    Mesh {
        /// The fixture.
        fixture: String,
        /// The chord tolerance the result was meshed at.
        chord: f64,
        /// What went wrong.
        what: String,
    },
    /// A step's provenance does not account for every entity.
    #[error("{fixture}: step {step:?}: provenance: {what}")]
    Provenance {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// What is unaccounted for.
        what: String,
    },
    /// The dump differs from the committed one, or none is committed.
    #[error("{fixture}: {path} {what}")]
    Dump {
        /// The fixture.
        fixture: String,
        /// The dump file.
        path: PathBuf,
        /// The difference, or that the file is missing.
        what: String,
    },
    /// A `step` operand's file is missing, is not the file the recipe
    /// hashed, does not parse, or names no single solid.
    #[error("{fixture}: step {step:?}: {what}")]
    StepFile {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// What is wrong.
        what: String,
    },
    /// The STEP reader refused the solid a `step` operand names: the
    /// refusal is the step's typed error (ADR-0025 §2).
    #[error("{fixture}: step {step:?}: the reader refuses it: {refusal}")]
    Refused {
        /// The fixture.
        fixture: String,
        /// The step's name.
        step: String,
        /// The reader's refusal.
        refusal: Box<io_step::Refusal>,
    },
    /// A part fixture's solid is not what the fixture records: another
    /// set of solid instances, another outcome, or a solid one reading of
    /// the file has and the other does not (`crate::part`).
    #[error("{fixture}: {what}")]
    Part {
        /// The fixture.
        fixture: String,
        /// What differs.
        what: String,
    },
    /// A file could not be read or written.
    #[error("{path}: {source}")]
    Io {
        /// The file.
        path: PathBuf,
        /// The cause.
        source: std::io::Error,
    },
}

/// The dump file of a variant: `dump.txt` for `default`,
/// `dump.<variant>.txt` otherwise.
pub fn dump_path(dir: &Path, variant: &str) -> PathBuf {
    if variant == "default" {
        dir.join("dump.txt")
    } else {
        dir.join(format!("dump.{variant}.txt"))
    }
}

/// `true` when [`BLESS_VAR`] is set to anything but `0` or empty.
pub fn blessing() -> bool {
    std::env::var(BLESS_VAR).is_ok_and(|v| !(v.is_empty() || v == "0"))
}

/// What one step produced: the body, its record and the bodies it took.
#[derive(Debug)]
pub struct Made {
    /// The body the step built.
    pub body: Body,
    /// The record the operation returned.
    pub provenance: Provenance,
    /// The bodies the step consumed, in the operation's argument order.
    pub inputs: Vec<Body>,
    /// Every body the step made, which `provenance` accounts for: `body`
    /// alone, or both sides of a split, positive first.
    pub outputs: Vec<Body>,
}

/// A recipe built whole: the model every step was built in, each body
/// step by name, the profiles the `profile` steps described, and the name
/// of the step the recipe calls its result — what a test of a chain of
/// operations reads a fixture through when it wants the steps' records,
/// which [`run`] only accounts for.
#[derive(Debug)]
pub struct Chain {
    /// The model the steps were built in.
    pub model: Model,
    /// Every body step of the recipe, by name.
    pub steps: BTreeMap<String, Made>,
    /// Every `profile` step, by name.
    pub profiles: BTreeMap<String, Profile>,
    /// The name of the result step in [`Chain::steps`].
    pub result: String,
    /// The recipe's parameters under the variant it was built at.
    pub params: BTreeMap<String, f64>,
}

impl Chain {
    /// The result step's body.
    pub fn result(&self) -> Option<Body> {
        Some(self.steps.get(&self.result)?.body)
    }
}

/// The model a fixture's recipe builds in: empty, at the recipe's
/// `precision` (`arris_math::Precision::DEFAULT` where it names none).
fn model_for(fixture: &Fixture) -> Result<Model, CorpusError> {
    Model::new(fixture.recipe.precision.precision()).map_err(|source| CorpusError::Precision {
        fixture: fixture.name.clone(),
        source,
    })
}

/// Builds every step of `dir`'s recipe under `variant` in one model and
/// returns them with their records. Errors: as [`run`]'s build stage.
/// A recipe whose result Arris refuses by design (`expected.degenerate`
/// or `analytic.expect_error`) fails here with that error; [`run`] is
/// what asserts a refusal.
///
/// ```no_run
/// use arris_debug::{corpus, fixtures};
///
/// let dir = fixtures::corpus_root().join("boolean/through-hole");
/// let chain = corpus::chain(&dir, "default").unwrap();
/// assert!(chain.result().is_some());
/// assert!(!chain.steps["result"].provenance.is_empty());
/// ```
pub fn chain(dir: &Path, variant: &str) -> Result<Chain, CorpusError> {
    chain_of(&fixtures::load(dir)?, variant)
}

/// [`chain`] for a recipe with no directory and no oracle answer yet — a
/// generated one (`prop::recipe`) — built under its `default` variant;
/// `name` is what its errors call it. Errors: as [`chain`]'s, and every
/// one but [`CorpusError::Op`] says the recipe itself is malformed.
///
/// ```
/// use arris_debug::corpus;
/// use arris_debug::fixtures::Recipe;
///
/// let recipe: Recipe = serde_json::from_str(
///     r#"{"steps": [{"op": "box", "name": "b", "min": [0, 0, 0], "max": [1, 2, 3]}], "result": "b"}"#,
/// )
/// .unwrap();
/// assert!(corpus::build("generated/box", &recipe).unwrap().result().is_some());
/// ```
pub fn build(name: &str, recipe: &Recipe) -> Result<Chain, CorpusError> {
    let fixture = Fixture {
        dir: PathBuf::new(),
        name: name.to_string(),
        recipe: recipe.clone(),
        recipe_sha256: String::new(),
        expected: Expected {
            occt: String::new(),
            recipe_sha256: String::new(),
            results: BTreeMap::new(),
        },
    };
    chain_of(&fixture, "default")
}

pub(crate) fn chain_of(fixture: &Fixture, variant: &str) -> Result<Chain, CorpusError> {
    chain_from(fixture, variant, model_for(fixture)?, BTreeMap::new())
}

/// [`chain_of`] starting from `model` with the steps of `steps` already
/// built in it: those are not built again. How the battery starts each
/// case from the solid the part runner already read, where a fresh read
/// of the same file would give the same model (`crate::battery`).
pub(crate) fn chain_from(
    fixture: &Fixture,
    variant: &str,
    mut model: Model,
    mut steps: BTreeMap<String, Made>,
) -> Result<Chain, CorpusError> {
    let Some(params) = fixture.recipe.params_of(variant) else {
        return Err(CorpusError::Variant {
            fixture: fixture.name.clone(),
            variant: variant.to_string(),
        });
    };
    let mut profiles = BTreeMap::new();
    build_all(
        &mut model,
        fixture,
        &params,
        None,
        &mut steps,
        &mut profiles,
    )?;
    Ok(Chain {
        model,
        steps,
        profiles,
        result: fixture.recipe.result.clone(),
        params,
    })
}

/// Builds every step of `fixture`'s recipe into `m` and `made`, a
/// `profile` step into `profiles`. Returns `false` when `refusal` is set
/// and the result step failed with the refusal expected — the recipe is
/// built no further and nothing after it is comparable; `true` when every
/// step built.
fn build_all(
    m: &mut Model,
    fixture: &Fixture,
    params: &BTreeMap<String, f64>,
    refusal: Option<Refusal>,
    made: &mut BTreeMap<String, Made>,
    profiles: &mut BTreeMap<String, Profile>,
) -> Result<bool, CorpusError> {
    for step in &fixture.recipe.steps {
        if made.contains_key(step.name()) {
            continue;
        }
        let built = build_step(m, fixture, step, params, made, profiles, &Control::NONE);
        if let Some(refusal) = refusal {
            if step.name() == fixture.recipe.result {
                refusal.assert(&fixture.name, step.name(), built.map(|_| ()))?;
                return Ok(false);
            }
        }
        if let Some(body) = built? {
            made.insert(step.name().to_string(), body);
        }
    }
    Ok(true)
}

/// The name of the STEP file [`run`] writes for the oracle:
/// `<area>-<slug>-<variant>` for a fixture in the corpus, and the same
/// with a short digest of the directory for a copy of that recipe
/// anywhere else.
///
/// Two runs of one recipe from different directories — the corpus's own
/// copy and a scratch copy in a test of this runner — otherwise name the
/// same file, and they run at the same time: under libtest as two threads
/// of one binary, and under `cargo nextest` as two processes. The
/// canonical directory keeps the plain name `docs/ARCHITECTURE.md`
/// §Formats and tools and the `inspect` skill quote, so a failure is still
/// looked at under the name the docs give it.
fn step_tag(name: &str, variant: &str, dir: &Path) -> String {
    let tag = format!("{}-{variant}", name.replace('/', "-"));
    let canonical = crate::fixtures::corpus_root().join(name);
    let same = dir == canonical
        || matches!((dir.canonicalize(), canonical.canonicalize()), (Ok(a), Ok(b)) if a == b);
    if same {
        return tag;
    }
    let digest = Sha256::digest(dir.to_string_lossy().as_bytes());
    format!("{tag}-{:02x}{:02x}{:02x}", digest[0], digest[1], digest[2])
}

/// A stage of [`run`], in the order it runs them: what a
/// [`CorpusError`] says failed ([`CorpusError::stage`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Stage {
    /// Loading the fixture and building its recipe, or the typed refusal
    /// the result step was expected to fail with.
    Build,
    /// The checker at `Full` and the result's lumps ([`check_stage`]).
    Check,
    /// Counts and genus against the oracle's ([`counts_stage`]).
    Counts,
    /// Mass properties against the oracle's or the closed forms'
    /// ([`measure_stage`]).
    Measure,
    /// The mesh, its corner block and its volume ([`mesh_stage`]).
    Mesh,
    /// The probes against the oracle's classifications
    /// ([`probe_stage`]).
    Probes,
    /// The provenance accounting of every step ([`provenance_stage`]).
    Provenance,
    /// STEP written and read back by the oracle.
    Step,
    /// Open CASCADE's STEP read back by Arris ([`read_back_stage`]).
    ReadBack,
    /// The text dump against the committed one.
    Dump,
}

impl core::fmt::Display for Stage {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Stage::Build => "build",
            Stage::Check => "check",
            Stage::Counts => "counts",
            Stage::Measure => "measure",
            Stage::Mesh => "mesh",
            Stage::Probes => "probes",
            Stage::Provenance => "provenance",
            Stage::Step => "step",
            Stage::ReadBack => "read back",
            Stage::Dump => "dump",
        })
    }
}

impl CorpusError {
    /// The stage that failed.
    pub fn stage(&self) -> Stage {
        match self {
            CorpusError::Fixture(_)
            | CorpusError::Variant { .. }
            | CorpusError::Reference { .. }
            | CorpusError::Operands { .. }
            | CorpusError::Expression { .. }
            | CorpusError::Profile { .. }
            | CorpusError::EdgePoint { .. }
            | CorpusError::FacePoint { .. }
            | CorpusError::Precision { .. }
            | CorpusError::Axis { .. }
            | CorpusError::Path { .. }
            | CorpusError::Plane { .. }
            | CorpusError::Polyhedron { .. }
            | CorpusError::Expectation { .. }
            | CorpusError::Unsupported { .. }
            | CorpusError::Op { .. }
            | CorpusError::StepFile { .. }
            | CorpusError::Refused { .. }
            | CorpusError::Part { .. } => Stage::Build,
            CorpusError::Check { .. } | CorpusError::Lumps { .. } => Stage::Check,
            CorpusError::Counts { .. } | CorpusError::Genus { .. } => Stage::Counts,
            CorpusError::Measure { .. } => Stage::Measure,
            CorpusError::Mesh { .. } => Stage::Mesh,
            CorpusError::Probe { .. } => Stage::Probes,
            CorpusError::Provenance { .. } => Stage::Provenance,
            CorpusError::Step { .. } | CorpusError::Part21 { .. } | CorpusError::Oracle(_) => {
                Stage::Step
            }
            CorpusError::ReadBack { .. } => Stage::ReadBack,
            CorpusError::Dump { .. } | CorpusError::Io { .. } => Stage::Dump,
        }
    }
}

/// Runs every stage on `dir`'s recipe under `variant` — the STEP stage
/// parses Arris's own file back through `arris_io::step::part21` before
/// the oracle reads it. Errors: the first
/// stage that fails, with what differed. Writes `target/inspect/<area>-
/// <slug>-<variant>.step` for the oracle, and the dump file under
/// [`BLESS_VAR`].
///
/// ```no_run
/// use arris_debug::{corpus, fixtures};
///
/// let dir = fixtures::corpus_root().join("primitive/box");
/// corpus::run(&dir, "default").unwrap();
/// ```
pub fn run(dir: &Path, variant: &str) -> Result<(), CorpusError> {
    let fixture = fixtures::load(dir)?;
    let name = fixture.name.clone();
    let Some(params) = fixture.recipe.params_of(variant) else {
        return Err(CorpusError::Variant {
            fixture: name,
            variant: variant.to_string(),
        });
    };
    let Some(expected) = fixture.expected.results.get(variant) else {
        return Err(CorpusError::Variant {
            fixture: name,
            variant: variant.to_string(),
        });
    };
    // A result the recipe says Arris refuses by design, or one the oracle
    // records no solid for, must fail with its typed error at the result
    // step; nothing after it is compared. The recipe's named error comes
    // first: an oracle that makes nothing does not loosen it to any
    // `Degenerate`.
    let refusal = match fixture.recipe.analytic.expect_error {
        Some(error) => Some(Refusal::Error(error)),
        None => expected.degenerate.then_some(Refusal::Degenerate),
    };
    let mut model = model_for(&fixture)?;
    let mut steps: BTreeMap<String, Made> = BTreeMap::new();
    let mut profiles: BTreeMap<String, Profile> = BTreeMap::new();
    if !build_all(
        &mut model,
        &fixture,
        &params,
        refusal,
        &mut steps,
        &mut profiles,
    )? {
        return Ok(());
    }
    let chain = Chain {
        model,
        steps,
        profiles,
        result: fixture.recipe.result.clone(),
        params,
    };
    let body = stages(&fixture, &chain, expected)?;
    let m = &chain.model;
    // STEP through the oracle.
    let text = io_step::write(m, &[body]).map_err(|source| CorpusError::Step {
        fixture: name.clone(),
        source,
    })?;
    // Every file the corpus writes parses, each instance the writer
    // defined — one per line starting `#` — kept once.
    let parsed = io_step::part21::parse(&text).map_err(|e| CorpusError::Part21 {
        fixture: name.clone(),
        what: e.to_string(),
    })?;
    let written = text.lines().filter(|l| l.starts_with('#')).count();
    if parsed.instances.len() != written {
        return Err(CorpusError::Part21 {
            fixture: name.clone(),
            what: format!(
                "{} instances parsed of {written} written",
                parsed.instances.len()
            ),
        });
    }
    let tag = step_tag(&name, variant, dir);
    oracle::compare_dir(dir, &text, Some(variant), &tag)?;

    // Open CASCADE's own STEP of the recipe, read back by Arris — unless
    // the recipe says that file is lossy (ADR-0023), or that reading it
    // measures the reader on a walked blend, which a regression fixture
    // holds instead.
    let analytic = &fixture.recipe.analytic;
    if expected.sheet {
        sheet_round_trip(&fixture, &chain, body, &text, expected)?;
    } else if analytic.step_differs.is_none() && analytic.occt_step_unread.is_none() {
        let occt = oracle::occt_step(dir, Some(variant), false, &format!("occt-{tag}"))?;
        read_back_stage(&fixture, variant, &occt, expected)?;
        // And converted to B-splines first: free-form faces with seams
        // and poles, held to the converted shape's own counts — but not
        // for a result on a file's solid, which the reader has already
        // read, and whose conversion is a whole real part in B-splines:
        // minutes of reading that measure the reader on a file of that
        // size, not the operation (ADR-0026, amendment of step 7).
        if expected.nurbs_fails.is_none() && !reads_a_file(&fixture) {
            let occt = oracle::occt_step(dir, Some(variant), true, &format!("occt-nurbs-{tag}"))?;
            read_back_nurbs_stage(&fixture, variant, &occt, expected)?;
        }
    }

    // The dump.
    let dump = dump_text(m, body).map_err(|e| CorpusError::Dump {
        fixture: name.clone(),
        path: dump_path(dir, variant),
        what: e.to_string(),
    })?;
    check_dump(&name, &dump_path(dir, variant), &dump, blessing())
}

/// `dump` against the file committed at `path` — or, when `bless`,
/// written there instead. `name` is what the error calls the fixture.
/// Errors: [`CorpusError::Dump`] for a dump that differs or is not
/// committed, [`CorpusError::Io`] for a file that cannot be read or
/// written.
pub(crate) fn check_dump(
    name: &str,
    path: &Path,
    dump: &str,
    bless: bool,
) -> Result<(), CorpusError> {
    if bless {
        return std::fs::write(path, dump).map_err(|source| CorpusError::Io {
            path: path.to_path_buf(),
            source,
        });
    }
    let committed = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            return Err(CorpusError::Dump {
                fixture: name.to_string(),
                path: path.to_path_buf(),
                what: format!("is not committed yet; run with {BLESS_VAR}=1 to write it"),
            });
        }
        Err(source) => {
            return Err(CorpusError::Io {
                path: path.to_path_buf(),
                source,
            });
        }
    };
    if committed != dump {
        return Err(CorpusError::Dump {
            fixture: name.to_string(),
            path: path.to_path_buf(),
            what: format!(
                "differs from the dump of this build:\n{}",
                diff(&committed, dump)
            ),
        });
    }
    Ok(())
}

/// The lines that differ, with their numbers: `-` for the committed
/// side, `+` for this build's.
fn diff(committed: &str, actual: &str) -> String {
    let a: Vec<&str> = committed.lines().collect();
    let b: Vec<&str> = actual.lines().collect();
    let mut out = String::new();
    let n = a.len().max(b.len());
    let mut shown = 0;
    for i in 0..n {
        let (x, y) = (a.get(i), b.get(i));
        if x != y {
            if let Some(x) = x {
                out.push_str(&format!("-{:>5} {x}\n", i + 1));
            }
            if let Some(y) = y {
                out.push_str(&format!("+{:>5} {y}\n", i + 1));
            }
            shown += 1;
            if shown >= 20 {
                out.push_str("… (more)\n");
                break;
            }
        }
    }
    if a.len() != b.len() {
        out.push_str(&format!(
            "({} lines committed, {} in this build)\n",
            a.len(),
            b.len()
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::unmetered::primitive_box;

    /// The recipe's `precision` is the model's, and an inconsistent one
    /// is the fixture's own error rather than a panic.
    #[test]
    fn the_recipe_names_the_model_s_precision() {
        let metres = crate::fixtures::corpus_root().join("boolean/probe-through-hole-m");
        let metric = chain(&metres, "default").expect("the consumer's units");
        assert_eq!(metric.model.precision().default_tolerance, 1e-6);

        let millimetres = crate::fixtures::corpus_root().join("boolean/through-hole");
        let default = chain(&millimetres, "default").expect("the default units");
        assert_eq!(default.model.precision(), arris_math::Precision::DEFAULT);

        // A floor above the default tolerance is no precision at all, and
        // the runner says which fixture rather than unwrapping.
        let scratch =
            std::env::temp_dir().join(format!("arris-corpus-precision-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&scratch);
        std::fs::create_dir_all(&scratch).expect("a scratch directory");
        for file in ["fixture.json", "expected.json"] {
            std::fs::copy(millimetres.join(file), scratch.join(file)).expect("a copy");
        }
        let text = std::fs::read_to_string(scratch.join("fixture.json")).expect("the recipe");
        std::fs::write(
            scratch.join("fixture.json"),
            text.replace(
                "\"steps\"",
                "\"precision\": {\"min_tolerance\": 1.0}, \"steps\"",
            ),
        )
        .expect("the edited recipe");
        assert!(matches!(
            chain(&scratch, "default"),
            Err(CorpusError::Precision { .. })
        ));
        let _ = std::fs::remove_dir_all(&scratch);
    }

    /// A `step` operand reads only the file it hashes, names a solid the
    /// file has, and carries the reader's refusal as its own.
    #[test]
    fn a_step_operand_is_the_solid_of_the_file_it_hashes() {
        let dir = crate::fixtures::corpus_root().join("boolean/step-operand-cut");
        let file = "operand.step";
        let bytes = std::fs::read(dir.join(file)).expect("the committed operand");
        let digest: String = Sha256::digest(&bytes)
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        let probe = Tolerances::default().probe;
        let mut m = Model::default();
        let read = read_solid(&mut m, &dir, file, &digest, 22, None, probe)
            .unwrap_or_else(|_| panic!("#22 of {file}"));
        assert!(check(&m, read.body, Level::Full).is_ok());

        let fault = |r: Result<io_step::ReadBody, ReadFault>| match r {
            Ok(_) => panic!("read a solid it should not have"),
            Err(ReadFault::File(what)) => what,
            Err(ReadFault::Refused(r)) => format!("refused: {r}"),
        };
        let mut m = Model::default();
        let other = "0".repeat(64);
        let what = fault(read_solid(&mut m, &dir, file, &other, 22, None, probe));
        assert!(what.contains("not the recipe's"), "{what}");
        let what = fault(read_solid(&mut m, &dir, file, &digest, 23, None, probe));
        assert!(what.contains("has no solid #23"), "{what}");
        let what = fault(read_solid(
            &mut m,
            &dir,
            "absent.step",
            &digest,
            22,
            None,
            probe,
        ));
        assert!(what.contains("absent.step"), "{what}");

        // The same file with its context's length unit taken away: the
        // reader refuses the solid, and the operand is that refusal.
        let scratch =
            std::env::temp_dir().join(format!("arris-corpus-step-operand-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&scratch);
        std::fs::create_dir_all(&scratch).expect("a scratch directory");
        let text = String::from_utf8(bytes).expect("Arris writes ASCII");
        let unitless = text.replace(
            "GLOBAL_UNIT_ASSIGNED_CONTEXT((#10,",
            "GLOBAL_UNIT_ASSIGNED_CONTEXT((",
        );
        assert_ne!(unitless, text, "the operand declares a length unit");
        std::fs::write(scratch.join(file), &unitless).expect("the edited operand");
        let digest: String = Sha256::digest(unitless.as_bytes())
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect();
        let mut m = Model::default();
        let what = fault(read_solid(&mut m, &scratch, file, &digest, 22, None, probe));
        assert!(what.starts_with("refused: "), "{what}");
        let _ = std::fs::remove_dir_all(&scratch);
    }

    #[test]
    fn one_recipe_in_two_directories_names_two_step_files() {
        // The corpus's own copy keeps the name the docs quote.
        let canonical = crate::fixtures::corpus_root().join("primitive/cylinder");
        assert_eq!(
            step_tag("primitive/cylinder", "default", &canonical),
            "primitive-cylinder-default"
        );
        // A scratch copy of the same recipe — what this module's own tests
        // build — does not, so the two never write one file while they run
        // at the same time.
        let scratch = Path::new("/tmp/arris-corpus-dump-diff-1");
        let other = Path::new("/tmp/arris-corpus-dump-diff-2");
        let a = step_tag("primitive/cylinder", "default", scratch);
        let b = step_tag("primitive/cylinder", "default", other);
        assert!(a.starts_with("primitive-cylinder-default-"), "{a}");
        assert_ne!(a, "primitive-cylinder-default");
        assert_ne!(a, b, "two scratch copies name two files");
        assert_eq!(a, step_tag("primitive/cylinder", "default", scratch));
    }

    #[test]
    fn diff_names_the_lines_that_differ() {
        let d = diff("a\nb\nc\n", "a\nx\nc\nd\n");
        assert_eq!(
            d,
            "-    2 b\n+    2 x\n+    4 d\n(3 lines committed, 4 in this build)\n"
        );
        assert_eq!(diff("same\n", "same\n"), "");
    }

    #[test]
    fn the_measure_stage_holds_the_box_to_the_oracle_and_names_what_differs() {
        let dir = fixtures::corpus_root().join("primitive/box");
        let fixture = fixtures::load(&dir).unwrap();
        let expected = fixture.expected.results["default"].clone();
        let tolerances = fixture.recipe.tolerances;
        let mut m = Model::default();
        let (body, _) = primitive_box(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0))
            .expect("the box of the recipe");
        compare_mass(&m, body, &expected, "the oracle's", &tolerances)
            .expect("the oracle's numbers");
        // Every quantity is actually compared: move each one just past
        // its tolerance and the stage says which.
        let mut wrong = expected.clone();
        wrong.volume = Some(expected.volume.unwrap() * (1.0 + 1e-6));
        assert!(
            compare_mass(&m, body, &wrong, "the oracle's", &tolerances)
                .unwrap_err()
                .starts_with("volume ")
        );
        let mut wrong = expected.clone();
        wrong.area = Some(expected.area.unwrap() * (1.0 + 1e-6));
        assert!(
            compare_mass(&m, body, &wrong, "the oracle's", &tolerances)
                .unwrap_err()
                .starts_with("area ")
        );
        let mut wrong = expected.clone();
        let mut centroid = expected.centroid.unwrap();
        centroid[1] += 1e-3;
        wrong.centroid = Some(centroid);
        assert!(
            compare_mass(&m, body, &wrong, "the oracle's", &tolerances)
                .unwrap_err()
                .starts_with("centroid ")
        );
        let mut wrong = expected.clone();
        let mut inertia = expected.inertia.unwrap();
        inertia[0][1] += inertia[2][2] * 1e-6;
        wrong.inertia = Some(inertia);
        assert!(
            compare_mass(&m, body, &wrong, "the oracle's", &tolerances)
                .unwrap_err()
                .starts_with("inertia[0][1] ")
        );
    }

    /// Under `measure_differs` the stage is held to the recipe's closed
    /// forms, every one of which it needs (ADR-0015); without it, to the
    /// oracle's measurements.
    #[test]
    fn measure_differs_holds_the_stage_to_the_closed_forms() {
        let dir = fixtures::corpus_root().join("boolean/cross-cylinders-fuse");
        let mut fixture = fixtures::load(&dir).unwrap();
        let params = fixture.recipe.params_of("default").unwrap();
        let expected = fixture.expected.results["default"].clone();
        let (target, by) = measure_target(&fixture, &params, &expected).unwrap();
        assert_eq!((target, by), (expected.clone(), "the oracle's"));

        fixture.recipe.analytic.measure_differs = Some("a test".into());
        let Err(CorpusError::Measure { what, .. }) = measure_target(&fixture, &params, &expected)
        else {
            panic!("a closed form missing is an error");
        };
        assert_eq!(what, "analytic.measure_differs needs analytic.inertia");

        let row = |r: [&str; 3]| r.map(|x| Num::Expr(x.into()));
        fixture.recipe.analytic.inertia = Some([
            row(["2", "0", "0"]),
            row(["0", "3", "0"]),
            row(["0", "0", "R + 3"]),
        ]);
        let (target, by) = measure_target(&fixture, &params, &expected).unwrap();
        assert_eq!(by, "the closed form's");
        let volume = 2.0 * std::f64::consts::PI * 6.0 - 16.0 / 3.0;
        assert!((target.volume.unwrap() - volume).abs() < 1e-12);
        assert_eq!(target.centroid, Some([0.0; 3]));
        assert_eq!(target.inertia.unwrap()[2][2], 4.0);
        // What the oracle says of everything else is kept.
        assert_eq!(target.counts, expected.counts);
        assert_eq!(target.probes, expected.probes);
    }

    #[test]
    fn dump_paths_and_blessing() {
        let dir = Path::new("x");
        assert_eq!(dump_path(dir, "default"), Path::new("x/dump.txt"));
        assert_eq!(dump_path(dir, "tighter"), Path::new("x/dump.tighter.txt"));
    }
}
