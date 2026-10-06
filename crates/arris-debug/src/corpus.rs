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

use crate::unmetered::mass_properties;
use crate::unmetered::step_read;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use crate::dump::dump_text;
use crate::fixtures::geom::{self as geom_spec, build_profile};
use crate::fixtures::{
    self, Class, Counts, ExpectError, Expected, ExprError, Fixture, FixtureError, Measured, Num,
    Recipe, Rotate, ShellSide, Step, Tolerances,
};
use crate::oracle::{self, OracleError};
use crate::polyhedron::{PolyhedronError, polyhedron};
use arris_check::classify::{Classification, classify_point};
use arris_check::{Level, LumpError, Report, Unchecked, check, lumps};
use arris_geom::Profile;
use arris_geom::{CurveKind, GeomKind, Surface, SurfaceKind};
use arris_io::step::{self, StepError};
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
        refusal: Box<step::Refusal>,
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

/// How a result step is expected to fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Refusal {
    /// `OpError::Degenerate`, any reason: the oracle recorded no solid.
    Degenerate,
    /// The recipe's `analytic.expect_error`.
    Error(ExpectError),
}

impl Refusal {
    fn expected(self) -> String {
        match self {
            Refusal::Degenerate => "OpError::Degenerate".into(),
            Refusal::Error(ExpectError::TangentContact) => {
                "OpError::Degenerate with Reason::TangentContact".into()
            }
            Refusal::Error(ExpectError::NonManifold) => {
                "OpError::Degenerate with Reason::NonManifold".into()
            }
            Refusal::Error(ExpectError::BlendTooLarge) => {
                "OpError::Degenerate with Reason::BlendTooLarge".into()
            }
            Refusal::Error(ExpectError::TangentChain) => {
                "OpError::Degenerate with Reason::TangentChain".into()
            }
            Refusal::Error(ExpectError::VertexBlend) => {
                "OpError::Degenerate with Reason::VertexBlend".into()
            }
            Refusal::Error(ExpectError::EllipticRevolve) => {
                "OpError::Degenerate with Reason::EllipticRevolve".into()
            }
            Refusal::Error(ExpectError::Vanishes) => {
                "OpError::Degenerate with Reason::Vanishes".into()
            }
            Refusal::Error(ExpectError::VertexSplits) => {
                "OpError::Degenerate with Reason::VertexSplits".into()
            }
            Refusal::Error(ExpectError::NoExactOffset) => {
                "OpError::Degenerate with Reason::NoExactOffset".into()
            }
            Refusal::Error(ExpectError::SurfaceCollapses) => {
                "OpError::Degenerate with Reason::SurfaceCollapses".into()
            }
            Refusal::Error(ExpectError::Gap) => "OpError::Degenerate with Reason::Gap".into(),
            Refusal::Error(ExpectError::SelfIntersects) => {
                "OpError::Degenerate with Reason::SelfIntersects".into()
            }
            Refusal::Error(ExpectError::OpeningDragged) => {
                "OpError::Degenerate with Reason::OpeningDragged".into()
            }
            Refusal::Error(ExpectError::NoCrossing) => {
                "OpError::Degenerate with Reason::NoCrossing".into()
            }
            Refusal::Error(ExpectError::Nurbs) => {
                "OpError::Unsupported with a NURBS surface or curve in the pair".into()
            }
            Refusal::Error(ExpectError::Unsupported) => "OpError::Unsupported".into(),
        }
    }

    /// `Ok` when `built` is the failure expected.
    fn assert(
        self,
        fixture: &str,
        step: &str,
        built: Result<(), CorpusError>,
    ) -> Result<(), CorpusError> {
        let found = match built {
            Ok(()) => "a body".to_string(),
            Err(CorpusError::Op {
                source: OpError::Degenerate { reason, .. },
                ..
            }) => {
                let matches = match self {
                    Refusal::Degenerate => true,
                    Refusal::Error(ExpectError::TangentContact) => {
                        reason == Reason::Boolean(BooleanReason::TangentContact)
                    }
                    Refusal::Error(ExpectError::NonManifold) => {
                        reason == Reason::Input(InputReason::NonManifold)
                    }
                    Refusal::Error(ExpectError::BlendTooLarge) => {
                        reason == Reason::Blend(BlendReason::TooLarge)
                    }
                    Refusal::Error(ExpectError::TangentChain) => {
                        reason == Reason::Blend(BlendReason::TangentChain)
                    }
                    Refusal::Error(ExpectError::VertexBlend) => {
                        reason == Reason::Blend(BlendReason::VertexBlend)
                    }
                    Refusal::Error(ExpectError::EllipticRevolve) => {
                        matches!(reason, Reason::Sweep(SweepReason::EllipticRevolve { .. }))
                    }
                    Refusal::Error(ExpectError::Vanishes) => {
                        reason == Reason::Offset(OffsetReason::Vanishes)
                    }
                    Refusal::Error(ExpectError::VertexSplits) => {
                        reason == Reason::Offset(OffsetReason::VertexSplits)
                    }
                    Refusal::Error(ExpectError::NoExactOffset) => {
                        reason == Reason::Offset(OffsetReason::NoExactOffset)
                    }
                    Refusal::Error(ExpectError::SurfaceCollapses) => {
                        reason == Reason::Offset(OffsetReason::SurfaceCollapses)
                    }
                    Refusal::Error(ExpectError::Gap) => reason == Reason::Offset(OffsetReason::Gap),
                    Refusal::Error(ExpectError::SelfIntersects) => {
                        reason == Reason::Offset(OffsetReason::SelfIntersects)
                    }
                    Refusal::Error(ExpectError::OpeningDragged) => {
                        reason == Reason::Shell(ShellReason::OpeningDragged)
                    }
                    Refusal::Error(ExpectError::NoCrossing) => {
                        reason == Reason::Split(SplitReason::NoCrossing)
                    }
                    Refusal::Error(ExpectError::Nurbs | ExpectError::Unsupported) => false,
                };
                if matches {
                    return Ok(());
                }
                format!("OpError::Degenerate with {reason}")
            }
            Err(CorpusError::Op {
                source: OpError::Unsupported { a, b },
                ..
            }) => {
                let nurbs = |k: GeomKind| {
                    matches!(
                        k,
                        GeomKind::Surface(SurfaceKind::Nurbs) | GeomKind::Curve(CurveKind::Nurbs)
                    )
                };
                if self == Refusal::Error(ExpectError::Nurbs) && (nurbs(a.0) || nurbs(b.0)) {
                    return Ok(());
                }
                if self == Refusal::Error(ExpectError::Unsupported) {
                    return Ok(());
                }
                format!(
                    "OpError::Unsupported: no closed form for {} ({}) against {} ({})",
                    a.1, a.0, b.1, b.0
                )
            }
            Err(e) => e.to_string(),
        };
        Err(CorpusError::Expectation {
            fixture: fixture.to_string(),
            step: step.to_string(),
            expected: self.expected(),
            found,
        })
    }
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
            | CorpusError::Plane { .. }
            | CorpusError::Polyhedron { .. }
            | CorpusError::Expectation { .. }
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
    let text = step::write(m, &[body]).map_err(|source| CorpusError::Step {
        fixture: name.clone(),
        source,
    })?;
    // Every file the corpus writes parses, each instance the writer
    // defined — one per line starting `#` — kept once.
    let parsed = step::part21::parse(&text).map_err(|e| CorpusError::Part21 {
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
    if analytic.step_differs.is_none() && analytic.occt_step_unread.is_none() {
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

/// Open CASCADE's own STEP of the fixture's result under `variant`,
/// `text`, read by `arris_io::step::read` into a model of the fixture's
/// precision (ADR-0025): every solid read — none refused — and together,
/// a lump each as the oracle counts them, held to the checker at `Full`
/// with nothing unchecked, to the oracle's own counts and genus (the file
/// is Open CASCADE's topology, so `analytic.counts_differ` does not
/// apply), and to the oracle's own measurements — the file carries Open
/// CASCADE's shape, so `analytic.measure_differs`, which blames its
/// boolean, does not apply either — at the fixture's tolerances, widened
/// to the read body's own where those are wider (ADR-0023). A fixture
/// whose `analytic.occt_step_refused` names a refusal passes on that
/// refusal and fails on a read. Errors:
/// [`CorpusError::ReadBack`] for a file that does not read, and the
/// stage errors of [`check_stage`], [`counts_stage`] and
/// [`measure_stage`].
pub fn read_back_stage(
    fixture: &Fixture,
    variant: &str,
    text: &str,
    expected: &Measured,
) -> Result<(), CorpusError> {
    read_back(fixture, variant, text, expected, "plain")
}

/// [`read_back_stage`] of Open CASCADE's STEP of the result converted to
/// B-splines by `BRepBuilderAPI_NurbsConvert` (`occt_step.py --nurbs`):
/// every face a NURBS surface, every edge a NURBS curve — the proof on
/// files of the projection, the pcurves and the rebuilt poles onto NURBS
/// (ADR-0025). Held as the plain file is, but to the
/// converted shape's own counts (`nurbs_counts` in `expected.json`, since
/// conversion can add seams), and with the rows the checker cannot
/// decide on a NURBS face left unchecked: S5's and B1's face pairs with
/// a NURBS face in them, and B1's nesting of a shell no ray is cast from,
/// which a NURBS face never answers. `analytic.occt_step_refused` passes
/// this file too where it refuses the same way, and
/// `analytic.occt_nurbs_refused` names a refusal of this file alone, which
/// passes on it and fails on a read. Errors: as [`read_back_stage`], and
/// [`CorpusError::ReadBack`] where `expected.json` records no
/// `nurbs_counts`.
pub fn read_back_nurbs_stage(
    fixture: &Fixture,
    variant: &str,
    text: &str,
    expected: &Measured,
) -> Result<(), CorpusError> {
    let counts = expected.nurbs_counts.ok_or_else(|| CorpusError::ReadBack {
        fixture: fixture.name.clone(),
        file: "NURBS",
        what: "against no nurbs_counts in expected.json: rerun expected.py".into(),
    })?;
    let converted = Measured {
        counts,
        ..expected.clone()
    };
    read_back(fixture, variant, text, &converted, "NURBS")
}

/// [`read_back_stage`] of the file `which` names.
fn read_back(
    fixture: &Fixture,
    variant: &str,
    text: &str,
    expected: &Measured,
    which: &'static str,
) -> Result<(), CorpusError> {
    let fail = |what: String| CorpusError::ReadBack {
        fixture: fixture.name.clone(),
        file: which,
        what,
    };
    let Some(params) = fixture.recipe.params_of(variant) else {
        return Err(CorpusError::Variant {
            fixture: fixture.name.clone(),
            variant: variant.to_string(),
        });
    };
    let mut model = model_for(fixture)?;
    let read = step_read(&mut model, text, &step::ReadOptions::default())
        .map_err(|e| fail(e.to_string()))?;
    let nurbs = which == "NURBS";
    if let (true, Some(refused)) = (nurbs, &fixture.recipe.analytic.occt_nurbs_refused) {
        // The converted file alone is refused, for the reader's own gap:
        // the refusal it names is the pass, and a read lifts it.
        let kinds: Vec<String> = (read.solids.iter())
            .filter_map(|s| s.result.as_ref().err())
            .map(|r| r.kind().to_string())
            .collect();
        return if kinds.contains(&refused.kind) {
            Ok(())
        } else {
            Err(fail(format!(
                "with {kinds:?}, not the refusal analytic.occt_nurbs_refused names ({}): lift it if it reads",
                refused.kind
            )))
        };
    }
    if let Some(refused) = &fixture.recipe.analytic.occt_step_refused {
        // The file describes no solid by the standard: the refusal it
        // names is the pass, and a read is the failure that lifts it. The
        // converted file is passed on the same refusal where it has it,
        // and read as any other where it does not: a conversion of a
        // walked face can refuse for the reader's own reason (ADR-0025).
        let kinds: Vec<String> = (read.solids.iter())
            .filter_map(|s| s.result.as_ref().err())
            .map(|r| r.kind().to_string())
            .collect();
        if kinds.contains(&refused.kind) {
            return Ok(());
        }
        if !nurbs {
            return Err(fail(format!(
                "with {kinds:?}, not the refusal analytic.occt_step_refused names ({}): lift it if it reads",
                refused.kind
            )));
        }
    }
    let mut bodies = Vec::with_capacity(read.solids.len());
    for solid in &read.solids {
        let back = solid
            .result
            .as_ref()
            .map_err(|r| fail(format!("refused: {r}")))?;
        bodies.push(back.body);
    }
    let body = match bodies[..] {
        [] => return Err(fail("no solid".into())),
        [one] => one,
        _ => {
            // The lumps as one body, each read body's faces kept in a
            // shell of it.
            let mut shells = Vec::new();
            for &b in &bodies {
                for shell in model.shells(b).map_err(|e| fail(e.to_string()))? {
                    let faces = model.shell(shell.id).map_err(|e| fail(e.to_string()))?;
                    shells.push(faces.faces().iter().copied().map(FaceSpec::Keep).collect());
                }
            }
            let assembly = Assembly {
                shells,
                ..Assembly::default()
            };
            let tolerance = model.precision().default_tolerance;
            let (builder, _) =
                Builder::assemble(&model, tolerance, assembly).map_err(|e| fail(e.to_string()))?;
            builder
                .finish(&mut model, BodyKind::Solid)
                .map_err(|e| fail(e.to_string()))?
                .body
        }
    };
    let chain = Chain {
        model,
        steps: BTreeMap::from([(
            "read".to_string(),
            Made {
                body,
                provenance: Provenance::new(),
                inputs: Vec::new(),
                outputs: vec![body],
            },
        )]),
        profiles: BTreeMap::new(),
        result: "read".into(),
        params,
    };
    let mut held = fixture.clone();
    held.name = format!("{} [{which} OCCT STEP]", fixture.name);
    held.recipe.analytic.counts_differ = None;
    // Every `measure_differs` blames Open CASCADE's boolean, not its
    // measure: its file carries its own shape, which its own measurements
    // are of.
    held.recipe.analytic.measure_differs = None;
    held.recipe.tolerances =
        within_own_tolerance(&fixture.recipe.tolerances, &chain.model, body).map_err(fail)?;
    let report = if nurbs || fixture.recipe.analytic.occt_walked.is_some() {
        check_leaving_nurbs(&held.name, &chain.model, body)?
    } else {
        check_stage(&held, &chain)?.1
    };
    // Counts as the oracle's, a solid per solid read — each is one of
    // the file's, as Open CASCADE counts them, and no lump query is asked
    // of shells no ray may be cast against (a NURBS face's).
    let line = report.euler().ok_or_else(|| CorpusError::Check {
        fixture: held.name.clone(),
        report: Box::new(report.clone()),
    })?;
    let found = Counts {
        vertices: line.vertices,
        edges: line.edges,
        faces: line.faces,
        loops: line.loops,
        shells: line.shells,
        solids: bodies.len(),
    };
    if found != expected.counts {
        return Err(CorpusError::Counts {
            fixture: held.name.clone(),
            expected: expected.counts,
            found,
        });
    }
    if let Some(genus) = expected.genus.filter(|&g| g != line.genus) {
        return Err(CorpusError::Genus {
            fixture: held.name.clone(),
            expected: genus,
            found: line.genus,
        });
    }
    measure_stage(&held, &chain, expected)?;
    Ok(())
}

/// The checker at `Full` on a body read from a file, with nothing
/// violated and nothing left undecided but what the checker cannot decide
/// on a NURBS face: S5's and B1's face pairs with one in them, and B1's
/// nesting of a shell no ray is cast from, which a NURBS face never
/// answers (ADR-0025). `name` is what the error calls the body. Errors:
/// [`CorpusError::Check`].
pub(crate) fn check_leaving_nurbs(
    name: &str,
    m: &Model,
    body: Body,
) -> Result<Report, CorpusError> {
    let report = check(m, body, Level::Full);
    let has_nurbs = |f: FaceId| {
        (m.face(f))
            .and_then(|f| m.surface(f.surface()))
            .is_ok_and(|s| matches!(s, Surface::Nurbs(_)))
    };
    let any_nurbs = m
        .closure(body)
        .is_ok_and(|closure| closure.faces.into_iter().any(has_nurbs));
    let undecidable = |u: &Unchecked| match u {
        Unchecked::FacePair { kinds, .. } | Unchecked::ShellFacePair { kinds, .. } => {
            kinds.0 == SurfaceKind::Nurbs || kinds.1 == SurfaceKind::Nurbs
        }
        Unchecked::ShellNesting { .. } => any_nurbs,
        // A row added later is not known to be a NURBS limit.
        _ => false,
    };
    if !report.is_ok() || !report.unchecked().iter().all(undecidable) {
        return Err(CorpusError::Check {
            fixture: name.to_string(),
            report: Box::new(report),
        });
    }
    Ok(report)
}

/// Every stage of [`run`] that reads no file and starts no process, in
/// its order — [`check_stage`], [`counts_stage`], [`measure_stage`],
/// [`mesh_stage`], [`probe_stage`] and [`provenance_stage`] — on a
/// recipe already built, against the oracle's `expected` result of it.
/// What the differential holds a generated recipe to (ADR-0024 §2).
/// Returns the result body. Errors: the first stage that fails.
///
/// ```no_run
/// use arris_debug::{corpus, fixtures};
///
/// let dir = fixtures::corpus_root().join("boolean/through-hole");
/// let fixture = fixtures::load(&dir).unwrap();
/// let chain = corpus::chain(&dir, "default").unwrap();
/// corpus::stages(&fixture, &chain, &fixture.expected.results["default"]).unwrap();
/// ```
pub fn stages(fixture: &Fixture, chain: &Chain, expected: &Measured) -> Result<Body, CorpusError> {
    let (body, report) = check_stage(fixture, chain)?;
    counts_stage(fixture, chain, &report, expected)?;
    let target = measure_stage(fixture, chain, expected)?;
    mesh_stage(fixture, chain, &target)?;
    probe_stage(fixture, chain, expected)?;
    provenance_stage(fixture, chain)?;
    Ok(body)
}

/// The result body of `chain`. Errors: [`CorpusError::Reference`] when
/// the recipe's result names no body step.
fn result_of(fixture: &Fixture, chain: &Chain) -> Result<Body, CorpusError> {
    chain.result().ok_or_else(|| CorpusError::Reference {
        fixture: fixture.name.clone(),
        step: "result".into(),
        name: chain.result.clone(),
    })
}

/// The checker at `Full` on the result, with nothing left undecided.
/// Returns the result body and the report. Errors:
/// [`CorpusError::Check`].
pub fn check_stage(fixture: &Fixture, chain: &Chain) -> Result<(Body, Report), CorpusError> {
    let body = result_of(fixture, chain)?;
    let report = check(&chain.model, body, Level::Full);
    if !report.is_ok() || !report.unchecked().is_empty() {
        return Err(CorpusError::Check {
            fixture: fixture.name.clone(),
            report: Box::new(report),
        });
    }
    Ok((body, report))
}

/// Counts and genus of the result, from [`check_stage`]'s `report`,
/// against the oracle's — a solid per lump, as the oracle counts them —
/// or the recipe's own under `analytic.counts_differ`. Errors:
/// [`CorpusError::Counts`], [`CorpusError::Genus`], and
/// [`CorpusError::Lumps`] for a result whose lumps cannot be read.
pub fn counts_stage(
    fixture: &Fixture,
    chain: &Chain,
    report: &Report,
    expected: &Measured,
) -> Result<(), CorpusError> {
    let name = fixture.name.clone();
    let body = result_of(fixture, chain)?;
    let line = report.euler().ok_or_else(|| CorpusError::Check {
        fixture: name.clone(),
        report: Box::new(report.clone()),
    })?;
    let solids = lumps(&chain.model, body)
        .map_err(|source| CorpusError::Lumps {
            fixture: name.clone(),
            source,
        })?
        .len();
    let found = Counts {
        vertices: line.vertices,
        edges: line.edges,
        faces: line.faces,
        loops: line.loops,
        shells: line.shells,
        solids,
    };
    // The oracle's counts, unless the recipe states a convention Arris
    // does not follow and gives its own.
    let analytic = &fixture.recipe.analytic;
    let counts = match (&analytic.counts_differ, analytic.counts) {
        (Some(_), Some(own)) => own,
        _ => expected.counts,
    };
    if found != counts {
        return Err(CorpusError::Counts {
            fixture: name,
            expected: counts,
            found,
        });
    }
    // The oracle's genus, unless the recipe's own counts change it.
    let genus = match (&analytic.counts_differ, analytic.genus) {
        (Some(_), Some(own)) => Some(own),
        _ => expected.genus,
    };
    if let Some(genus) = genus {
        if line.genus != genus {
            return Err(CorpusError::Genus {
                fixture: name,
                expected: genus,
                found: line.genus,
            });
        }
    }
    Ok(())
}

/// What [`measure_stage`] held the result to: the oracle's measurements,
/// or the recipe's closed forms under `analytic.measure_differs`.
#[derive(Debug, Clone, PartialEq)]
pub struct Target {
    /// The numbers.
    pub measured: Measured,
    /// Whose they are, as the errors say it: "the oracle's" or "the
    /// closed form's".
    pub by: &'static str,
}

/// `true` when the recipe starts from a file's solid, a `step` operand.
fn reads_a_file(fixture: &Fixture) -> bool {
    (fixture.recipe.steps.iter()).any(|s| matches!(s, Step::Read { .. }))
}

/// `measure` over the B-Rep against what the oracle measured of the same
/// recipe — or, where the recipe says the oracle is wrong, its closed
/// forms (ADR-0015): volume, area, centroid and the inertia tensor, each
/// within the fixture's tolerance for it — widened to the result's own
/// ([`within_own_tolerance`]) where the recipe reads a file's solid, a
/// `step` operand. Returns the [`Target`] held to.
/// Errors: [`CorpusError::Measure`] naming the quantity.
pub fn measure_stage(
    fixture: &Fixture,
    chain: &Chain,
    expected: &Measured,
) -> Result<Target, CorpusError> {
    let body = result_of(fixture, chain)?;
    let (measured, by) = measure_target(fixture, &chain.params, expected)?;
    let fail = |what| CorpusError::Measure {
        fixture: fixture.name.clone(),
        what,
    };
    // A recipe on a file's solid holds its result as the part's own
    // reading and battery hold it: within the tolerances the file gave
    // the body, which a kernel-built primitive's defaults do not size
    // (ADR-0023, ADR-0026 §6).
    let tolerances = if reads_a_file(fixture) {
        within_own_tolerance(&fixture.recipe.tolerances, &chain.model, body).map_err(fail)?
    } else {
        fixture.recipe.tolerances
    };
    compare_mass(&chain.model, body, &measured, by, &tolerances).map_err(fail)?;
    Ok(Target { measured, by })
}

/// How many chords [`within_own_tolerance`] measures an edge's length
/// over: a first-order bound needs the length to a few per cent, which a
/// polyline of this many chords gives on any curve the corpus has.
const OWN_TOLERANCE_CHORDS: usize = 64;

/// `tolerances` widened to what the body itself declares (ADR-0023): a
/// reader may move any point of the boundary by up to its largest vertex
/// tolerance `t` and hand back the same shape, so a round trip of it is
/// held to the first-order change such a move makes and no closer — over
/// a boundary of area `A` and volume `V`, edges of total length `L`, and
/// the corners of its box within `R` of the centroid, `A·t` of volume,
/// `L·t` of area, `A·t·R / V` of centroid and `A·t·R²` of each inertia
/// component, as the oracle's `within_own_tolerance` widens its own. A
/// tolerance of the fixture's that is wider stays. Errors: what the
/// model or `mass_properties` refuses, said in words.
///
/// ```
/// use arris_debug::corpus::within_own_tolerance;
/// use arris_debug::fixtures::Tolerances;
/// use arris_debug::sample;
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
/// let own = within_own_tolerance(&Tolerances::default(), &m, body)?;
/// assert!(own.volume_rel >= Tolerances::default().volume_rel);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn within_own_tolerance(
    tolerances: &Tolerances,
    m: &Model,
    body: Body,
) -> Result<Tolerances, String> {
    let closure = m.closure(body).map_err(|e| e.to_string())?;
    let mut t: f64 = 0.0;
    let mut corners: Vec<Point3> = Vec::new();
    for &v in &closure.vertices {
        let v = m.vertex(v).map_err(|e| e.to_string())?;
        t = t.max(v.tolerance());
        corners.push(v.point());
    }
    let mut length = 0.0;
    for &e in &closure.edges {
        let e = m.edge(e).map_err(|e| e.to_string())?;
        let Some((curve, range)) = e.curve() else {
            continue;
        };
        let curve = m.curve(curve).map_err(|e| e.to_string())?;
        let points: Vec<Point3> = (0..=OWN_TOLERANCE_CHORDS)
            .map(|i| curve.point(range.lerp(i as f64 / OWN_TOLERANCE_CHORDS as f64)))
            .collect();
        length += points.windows(2).map(|w| (w[1] - w[0]).norm()).sum::<f64>();
        if let Some(b) = curve.bounds(range) {
            corners.push(Point3::new(b.min[0], b.min[1], b.min[2]));
            corners.push(Point3::new(b.max[0], b.max[1], b.max[2]));
        }
    }
    let mass = mass_properties(m, body).map_err(|e| e.to_string())?;
    let (lo, hi) = corners.iter().fold(
        (Vec3::repeat(f64::INFINITY), Vec3::repeat(f64::NEG_INFINITY)),
        |(lo, hi), p| (lo.inf(&p.coords), hi.sup(&p.coords)),
    );
    let mut reach: f64 = 0.0;
    for x in [lo.x, hi.x] {
        for y in [lo.y, hi.y] {
            for z in [lo.z, hi.z] {
                reach = reach.max((Point3::new(x, y, z) - mass.centroid).norm());
            }
        }
    }
    let (volume, area) = (mass.volume.abs(), mass.area);
    let scale = mass.inertia.iter().fold(0.0, |a: f64, x| a.max(x.abs()));
    let scale = if scale > 0.0 { scale } else { 1.0 };
    Ok(Tolerances {
        volume_rel: tolerances.volume_rel.max(area * t / volume),
        area_rel: tolerances.area_rel.max(length * t / area),
        centroid_abs: tolerances.centroid_abs.max(area * t * reach / volume),
        inertia_rel: tolerances.inertia_rel.max(area * t * reach * reach / scale),
        ..*tolerances
    })
}

/// The mesh at the fixture's `mesh_chord`: closed, positive, with the
/// corner block's ADR-0012 invariants on every face-local vertex (so the
/// whole corpus covers them rather than one focused test), and its
/// signed volume within `mesh_volume_rel` of `target`'s. Errors:
/// [`CorpusError::Mesh`].
pub fn mesh_stage(fixture: &Fixture, chain: &Chain, target: &Target) -> Result<(), CorpusError> {
    let body = result_of(fixture, chain)?;
    mesh_check(
        &fixture.name,
        &chain.model,
        body,
        &fixture.recipe.tolerances,
        target,
    )?;
    mesh_keeps_stage(fixture, chain, body)
}

/// Every edge the result shares by id with a step of the recipe's
/// `mesh_keeps` has the same polyline, bit for bit, in the mesh of each at
/// `mesh_chord`. Errors: [`CorpusError::Mesh`], naming the edge and the
/// two counts; [`CorpusError::Reference`] for a step that is no body.
fn mesh_keeps_stage(fixture: &Fixture, chain: &Chain, result: Body) -> Result<(), CorpusError> {
    if fixture.recipe.mesh_keeps.is_empty() {
        return Ok(());
    }
    let chord = fixture.recipe.tolerances.mesh_chord;
    let mesh_failure = |what: String| CorpusError::Mesh {
        fixture: fixture.name.clone(),
        chord,
        what,
    };
    let request = MeshRequest::new(chord);
    let mesh_of = |body: Body| {
        arris_mesh::tessellate_with(&chain.model, body, &request, &Control::NONE)
            .map_err(|e| mesh_failure(e.to_string()))
    };
    let after = mesh_of(result)?;
    let polyline = |mesh: &arris_mesh::TriMesh, e: arris_topo::EdgeId| -> Vec<[f64; 3]> {
        mesh.edge_polyline(e)
            .unwrap_or(&[])
            .iter()
            .filter_map(|&i| mesh.positions().get(i as usize).copied())
            .collect()
    };
    for name in &fixture.recipe.mesh_keeps {
        let made = chain
            .steps
            .get(name)
            .ok_or_else(|| CorpusError::Reference {
                fixture: fixture.name.clone(),
                step: "mesh_keeps".into(),
                name: name.clone(),
            })?;
        let before = mesh_of(made.body)?;
        let kept = chain
            .model
            .closure(made.body)
            .map_err(|e| mesh_failure(e.to_string()))?
            .edges;
        for e in kept {
            let (then, now) = (polyline(&before, e), polyline(&after, e));
            if now.is_empty() {
                continue;
            }
            if then != now {
                return Err(mesh_failure(format!(
                    "edge {e} is kept from {name:?} and meshed as {} points there, {} in the result",
                    then.len(),
                    now.len()
                )));
            }
        }
    }
    Ok(())
}

/// Every face of `body` meshed alone by [`arris_mesh::tessellate_faces`]
/// and the parts welded, in reverse order, equals `whole`: the same edge
/// polylines and the same triangles per face, as positions, bit for bit,
/// the same face-local normals and (u, v)s, and a closed mesh.
fn weld_stage(
    m: &Model,
    body: Body,
    request: &MeshRequest,
    whole: &arris_mesh::TriMesh,
) -> Result<(), String> {
    let faces = m.faces(body).map_err(|e| e.to_string())?;
    let mut parts = Vec::with_capacity(faces.len());
    for f in &faces {
        parts.push(
            arris_mesh::tessellate_faces(m, body, &[f.id], request, &Control::NONE)
                .map_err(|e| format!("{} meshed alone: {e}", f.id))?,
        );
    }
    parts.reverse();
    let welded = arris_mesh::TriMesh::weld(&parts).map_err(|e| format!("weld: {e}"))?;
    if !welded.is_closed() {
        return Err("the faces meshed one by one and welded are not closed".into());
    }
    let triples = |mesh: &arris_mesh::TriMesh, face| -> Option<Vec<[[f64; 3]; 3]>> {
        let range = mesh.faces().iter().find(|r| r.face == face)?;
        Some(
            mesh.triangles()[range.triangles.clone()]
                .iter()
                .map(|t| t.map(|i| mesh.positions()[i as usize]))
                .collect(),
        )
    };
    let polyline = |mesh: &arris_mesh::TriMesh, e| -> Option<Vec<[f64; 3]>> {
        Some(
            mesh.edge_polyline(e)?
                .iter()
                .map(|&i| mesh.positions()[i as usize])
                .collect(),
        )
    };
    for f in &faces {
        if triples(&welded, f.id) != triples(whole, f.id) {
            return Err(format!("{}'s triangles differ face by face", f.id));
        }
        let (a, b) = (
            welded.corners().and_then(|c| c.face(f.id)),
            whole.corners().and_then(|c| c.face(f.id)),
        );
        if let (Some(a), Some(b)) = (a, b) {
            let (wc, oc) = (welded.corners().ok_or("")?, whole.corners().ok_or("")?);
            if wc.normals()[a.vertices.clone()] != oc.normals()[b.vertices.clone()]
                || wc.uvs()[a.vertices.clone()] != oc.uvs()[b.vertices.clone()]
            {
                return Err(format!(
                    "{}'s corner normals or (u, v)s differ face by face",
                    f.id
                ));
            }
        } else if a.is_some() != b.is_some() {
            return Err(format!("{}'s corner block differs face by face", f.id));
        }
    }
    if welded.edges().len() != whole.edges().len() {
        return Err(format!(
            "{} edges face by face, {} in one call",
            welded.edges().len(),
            whole.edges().len()
        ));
    }
    for range in whole.edges() {
        if polyline(&welded, range.edge) != polyline(whole, range.edge) {
            return Err(format!("{}'s polyline differs face by face", range.edge));
        }
    }
    Ok(())
}

/// [`mesh_stage`] of `body` in `m`, at `tolerances`; `name` is what the
/// error calls it.
pub(crate) fn mesh_check(
    name: &str,
    m: &Model,
    body: Body,
    tolerances: &Tolerances,
    target: &Target,
) -> Result<(), CorpusError> {
    let mesh_failure = |what: String| CorpusError::Mesh {
        fixture: name.to_string(),
        chord: tolerances.mesh_chord,
        what,
    };
    let request = MeshRequest::new(tolerances.mesh_chord).with_corners();
    let mesh = arris_mesh::tessellate_with(m, body, &request, &Control::NONE)
        .map_err(|e| mesh_failure(e.to_string()))?;
    corners_stage(m, body, &mesh).map_err(mesh_failure)?;
    let Some(mesh_volume) = mesh.signed_volume() else {
        return Err(mesh_failure("the mesh is not closed".into()));
    };
    if !mesh_volume.is_finite() || mesh_volume <= 0.0 {
        return Err(mesh_failure(format!(
            "the mesh's signed volume is {mesh_volume}, not positive"
        )));
    }
    weld_stage(m, body, &request, &mesh).map_err(mesh_failure)?;
    if let Some(volume) = target.measured.volume {
        let relative = (mesh_volume - volume).abs() / volume.abs();
        if relative.is_nan() || relative > tolerances.mesh_volume_rel {
            return Err(mesh_failure(format!(
                "mesh volume {mesh_volume} vs {} {volume}: {relative:e} relative, above mesh_volume_rel {:e}",
                target.by, tolerances.mesh_volume_rel
            )));
        }
    }
    Ok(())
}

/// Arris's classification of each of the oracle's probe points against
/// the oracle's, exactly. Both sides have their own tolerance for "on" —
/// the fixture's `probe` for the oracle, the entities' own for Arris —
/// and a probe is placed so that the two agree; a disagreement is a
/// finding, never something a band is widened to cover. Errors:
/// [`CorpusError::Probe`].
pub fn probe_stage(
    fixture: &Fixture,
    chain: &Chain,
    expected: &Measured,
) -> Result<(), CorpusError> {
    let body = result_of(fixture, chain)?;
    for probe in &expected.probes {
        let point = Point3::new(probe.point[0], probe.point[1], probe.point[2]);
        let found = match classify_point(&chain.model, body, point) {
            Ok(Classification::Inside) => Ok(Class::In),
            Ok(Classification::Outside) => Ok(Class::Out),
            Ok(Classification::On(_)) => Ok(Class::On),
            Err(e) => Err(e.to_string()),
        };
        let matches = match &found {
            Ok(class) => *class == probe.class,
            Err(_) => false,
        };
        if !matches {
            return Err(CorpusError::Probe {
                fixture: fixture.name.clone(),
                label: probe.label.clone(),
                point: probe.point,
                expected: probe.class,
                found: match found {
                    Ok(class) => format!("{class:?}"),
                    Err(e) => e,
                },
            });
        }
    }
    Ok(())
}

/// The provenance accounting of every body step, in recipe order (a
/// profile step makes none). Errors: [`CorpusError::Provenance`] naming
/// the step.
pub fn provenance_stage(fixture: &Fixture, chain: &Chain) -> Result<(), CorpusError> {
    for step in &fixture.recipe.steps {
        let Some(out) = chain.steps.get(step.name()) else {
            continue;
        };
        arris_topo::provenance::audit_many(
            &chain.model,
            &out.inputs,
            &out.outputs,
            &out.provenance,
        )
        .map_err(|e| CorpusError::Provenance {
            fixture: fixture.name.clone(),
            step: step.name().to_string(),
            what: e.to_string(),
        })?;
    }
    Ok(())
}

/// A fixture's recipe built up to, but not including, its result step:
/// what a test of the boolean decomposition reads a `boolean/*` fixture
/// through before the boolean exists to run.
#[derive(Debug)]
pub struct Inputs {
    /// The model the steps were built in.
    pub model: Model,
    /// Every body step before the result, by name.
    pub bodies: BTreeMap<String, Body>,
    /// Every `profile` step before the result, by name.
    pub profiles: BTreeMap<String, Profile>,
    /// The result step, unbuilt.
    pub result: Step,
    fixture: Fixture,
    params: BTreeMap<String, f64>,
    made: BTreeMap<String, Made>,
}

impl Inputs {
    /// The result step run in `model` under `control`: what a test that
    /// interrupts an operation calls on a clone of [`Inputs::model`].
    /// Errors: [`CorpusError::Op`] with [`OpError::Interrupted`] when
    /// `control` stops it, and as [`run`]'s build stage otherwise.
    ///
    /// ```no_run
    /// use arris_debug::{corpus, fixtures};
    /// use arris_ops::Control;
    ///
    /// let dir = fixtures::corpus_root().join("boolean/through-hole");
    /// let inputs = corpus::inputs(&dir, "default").unwrap();
    /// let mut model = inputs.model.clone();
    /// let made = inputs.run_result(&mut model, &Control::NONE).unwrap();
    /// assert!(!made.provenance.is_empty());
    /// ```
    pub fn run_result(
        &self,
        model: &mut Model,
        control: &Control<'_>,
    ) -> Result<Made, CorpusError> {
        let mut profiles = self.profiles.clone();
        build_step(
            model,
            &self.fixture,
            &self.result,
            &self.params,
            &self.made,
            &mut profiles,
            control,
        )?
        .ok_or_else(|| CorpusError::Reference {
            fixture: self.fixture.name.clone(),
            step: "result".into(),
            name: self.fixture.recipe.result.clone(),
        })
    }

    /// The target and the tools of a result step that is a `cut`, in the
    /// recipe's order — what the one call and the chain of cuts the
    /// benchmark times against each other both take — or `None` when the
    /// result is not a cut or a name does not resolve.
    ///
    /// ```no_run
    /// use arris_debug::{corpus, fixtures};
    ///
    /// let dir = fixtures::corpus_root().join("boolean/plate-2x2-holes-cut-many");
    /// let inputs = corpus::inputs(&dir, "default").unwrap();
    /// let (_plate, holes) = inputs.cut_operands().unwrap();
    /// assert_eq!(holes.len(), 4);
    /// ```
    pub fn cut_operands(&self) -> Option<(Body, Vec<Body>)> {
        let Step::Cut {
            target,
            tool,
            tools,
            ..
        } = &self.result
        else {
            return None;
        };
        let names: Vec<&String> = tool.iter().chain(tools.iter()).collect();
        let tools = names
            .into_iter()
            .map(|n| self.bodies.get(n).copied())
            .collect::<Option<Vec<_>>>()?;
        Some((*self.bodies.get(target)?, tools))
    }

    /// The two bodies the result step combines — `a` and `b` of a `fuse`
    /// or `common`, the target and the tool of a `cut` — or `None` when
    /// the result is not a boolean.
    pub fn operands(&self) -> Option<(Body, Body)> {
        let (x, y) = match &self.result {
            Step::Fuse {
                a: Some(a),
                b: Some(b),
                ..
            }
            | Step::Common { a, b, .. } => (a, b),
            Step::Cut {
                target,
                tool: Some(tool),
                ..
            } => (target, tool),
            Step::Fuse { .. } | Step::Cut { .. } => return None,
            Step::Box { .. }
            | Step::Cylinder { .. }
            | Step::Profile { .. }
            | Step::Extrude { .. }
            | Step::Revolve { .. }
            | Step::Transform { .. }
            | Step::Mirror { .. }
            | Step::Split { .. }
            | Step::Fillet { .. }
            | Step::Chamfer { .. }
            | Step::Offset { .. }
            | Step::Shell { .. }
            | Step::Polyhedron { .. }
            | Step::Read { .. } => return None,
        };
        Some((*self.bodies.get(x)?, *self.bodies.get(y)?))
    }
}

/// Builds every step of `dir`'s recipe under `variant` before the one
/// named as the result, and returns them with the result step itself.
/// Errors: as [`run`]'s build stage, and [`CorpusError::Reference`] when
/// no step is named as the result.
///
/// ```no_run
/// use arris_debug::{corpus, fixtures};
///
/// let dir = fixtures::corpus_root().join("boolean/through-hole");
/// let inputs = corpus::inputs(&dir, "default").unwrap();
/// let (plate, hole) = inputs.operands().unwrap();
/// assert_ne!(plate, hole);
/// ```
pub fn inputs(dir: &Path, variant: &str) -> Result<Inputs, CorpusError> {
    inputs_of(fixtures::load(dir)?, variant)
}

/// [`inputs`] for the step `step` of a recipe with no directory — a
/// generated one (`prop::recipe`) — under its `default` variant: every
/// step before it is built (unmetered) and `step` is left as the result
/// step [`Inputs::run_result`] runs. Errors: as [`inputs`]'s, and
/// [`CorpusError::Reference`] when `step` names no step of the recipe.
///
/// ```
/// use arris_debug::corpus;
/// use arris_debug::fixtures::Recipe;
/// use arris_ops::Control;
///
/// let recipe: Recipe = serde_json::from_str(
///     r#"{"steps": [{"op": "box", "name": "b", "min": [0, 0, 0], "max": [1, 2, 3]}], "result": "b"}"#,
/// )
/// .unwrap();
/// let inputs = corpus::inputs_for_step("generated/box", &recipe, "b").unwrap();
/// let mut model = inputs.model.clone();
/// assert!(inputs.run_result(&mut model, &Control::budget(0)).is_err());
/// ```
pub fn inputs_for_step(name: &str, recipe: &Recipe, step: &str) -> Result<Inputs, CorpusError> {
    let mut recipe = recipe.clone();
    recipe.result = step.to_string();
    inputs_of(
        Fixture {
            dir: PathBuf::new(),
            name: name.to_string(),
            recipe,
            recipe_sha256: String::new(),
            expected: Expected {
                occt: String::new(),
                recipe_sha256: String::new(),
                results: BTreeMap::new(),
            },
        },
        "default",
    )
}

fn inputs_of(fixture: Fixture, variant: &str) -> Result<Inputs, CorpusError> {
    let name = fixture.name.clone();
    let Some(params) = fixture.recipe.params_of(variant) else {
        return Err(CorpusError::Variant {
            fixture: name,
            variant: variant.to_string(),
        });
    };
    let mut model = model_for(&fixture)?;
    let mut made: BTreeMap<String, Made> = BTreeMap::new();
    let mut profiles: BTreeMap<String, Profile> = BTreeMap::new();
    for step in &fixture.recipe.steps {
        if step.name() == fixture.recipe.result {
            let bodies = made.iter().map(|(k, v)| (k.clone(), v.body)).collect();
            return Ok(Inputs {
                model,
                bodies,
                profiles,
                result: step.clone(),
                fixture: fixture.clone(),
                params,
                made,
            });
        }
        if let Some(out) = build_step(
            &mut model,
            &fixture,
            step,
            &params,
            &made,
            &mut profiles,
            &Control::NONE,
        )? {
            made.insert(step.name().to_string(), out);
        }
    }
    Err(CorpusError::Reference {
        fixture: name,
        step: "result".into(),
        name: fixture.recipe.result.clone(),
    })
}

fn number(
    fixture: &Fixture,
    step: &Step,
    n: &Num,
    params: &BTreeMap<String, f64>,
) -> Result<f64, CorpusError> {
    n.eval(params).map_err(|source| CorpusError::Expression {
        fixture: fixture.name.clone(),
        step: step.name().to_string(),
        source,
    })
}

fn point(
    fixture: &Fixture,
    step: &Step,
    p: &[Num; 3],
    params: &BTreeMap<String, f64>,
) -> Result<Point3, CorpusError> {
    Ok(Point3::new(
        number(fixture, step, &p[0], params)?,
        number(fixture, step, &p[1], params)?,
        number(fixture, step, &p[2], params)?,
    ))
}

fn vector(
    fixture: &Fixture,
    step: &Step,
    v: &[Num; 3],
    params: &BTreeMap<String, f64>,
) -> Result<Vec3, CorpusError> {
    Ok(Vec3::new(
        number(fixture, step, &v[0], params)?,
        number(fixture, step, &v[1], params)?,
        number(fixture, step, &v[2], params)?,
    ))
}

/// The rigid motion of a `Step::Transform`: the rotation about `origin`
/// (the world origin when absent) applied first, then the translation —
/// `tools/oracle/oracle/recipe.py`'s `gp_Trsf` composition.
fn motion(
    fixture: &Fixture,
    step: &Step,
    rotate: &Option<Rotate>,
    translate: &Option<[Num; 3]>,
    params: &BTreeMap<String, f64>,
) -> Result<Isometry, CorpusError> {
    let mut m = Isometry::identity();
    if let Some(rot) = rotate {
        let axis = vector(fixture, step, &rot.axis, params)?;
        let origin = match &rot.origin {
            Some(o) => point(fixture, step, o, params)?,
            None => Point3::origin(),
        };
        let angle = number(fixture, step, &rot.angle_deg, params)?.to_radians();
        let rotation = UnitQuaternion::from_axis_angle(&UnitVec3::new_normalize(axis), angle);
        let about_origin = Isometry::from_rotation(rotation);
        m = Isometry::new(
            rotation,
            origin.coords - about_origin.apply_vec(origin.coords),
        );
    }
    if let Some(t) = translate {
        m = m.then(&Isometry::from_translation(vector(
            fixture, step, t, params,
        )?));
    }
    Ok(m)
}

/// The profile a sweep step names, built by an earlier `profile` step.
fn profile_reference<'a>(
    fixture: &Fixture,
    step: &Step,
    name: &str,
    profiles: &'a BTreeMap<String, Profile>,
) -> Result<&'a Profile, CorpusError> {
    profiles.get(name).ok_or_else(|| CorpusError::Reference {
        fixture: fixture.name.clone(),
        step: step.name().to_string(),
        name: name.to_string(),
    })
}

/// Builds one step: a body step into the [`Made`] it returns, a `profile`
/// step into `profiles` and `None`, since a sketch is a value and makes no
/// body.
fn build_step(
    m: &mut Model,
    fixture: &Fixture,
    step: &Step,
    params: &BTreeMap<String, f64>,
    made: &BTreeMap<String, Made>,
    profiles: &mut BTreeMap<String, Profile>,
    control: &Control<'_>,
) -> Result<Option<Made>, CorpusError> {
    let name = fixture.name.clone();
    let op = |source: OpError| CorpusError::Op {
        fixture: name.clone(),
        step: step.name().to_string(),
        source,
    };
    let body = |(body, provenance): (Body, Provenance), inputs: Vec<Body>| {
        Ok(Some(Made {
            body,
            provenance,
            inputs,
            outputs: vec![body],
        }))
    };
    match step {
        Step::Box { min, max, .. } => {
            let (min, max) = (
                point(fixture, step, min, params)?,
                point(fixture, step, max, params)?,
            );
            body(
                arris_ops::primitive_box(m, min, max, control).map_err(op)?,
                Vec::new(),
            )
        }
        Step::Cylinder {
            base,
            axis,
            radius,
            height,
            ..
        } => {
            let base = point(fixture, step, base, params)?;
            let direction = point(fixture, step, axis, params)?;
            let axis = Axis::new(base, Vec3::from(direction.coords)).map_err(|source| {
                CorpusError::Axis {
                    fixture: name.clone(),
                    step: step.name().to_string(),
                    source,
                }
            })?;
            let radius = number(fixture, step, radius, params)?;
            let height = number(fixture, step, height, params)?;
            body(
                arris_ops::primitive_cylinder(m, axis, radius, height, control).map_err(op)?,
                Vec::new(),
            )
        }
        Step::Profile {
            name,
            plane,
            outer,
            holes,
        } => {
            let profile = build_profile(name, plane, outer, holes, params).map_err(|source| {
                CorpusError::Profile {
                    fixture: fixture.name.clone(),
                    step: name.clone(),
                    source: Box::new(source),
                }
            })?;
            profiles.insert(name.clone(), profile);
            Ok(None)
        }
        Step::Extrude {
            profile,
            direction,
            length,
            ..
        } => {
            let profile = profile_reference(fixture, step, profile, profiles)?;
            let direction = vector(fixture, step, direction, params)?;
            let length = number(fixture, step, length, params)?;
            body(
                arris_ops::extrude(m, profile, direction, length, control).map_err(op)?,
                Vec::new(),
            )
        }
        Step::Revolve {
            profile,
            axis,
            angle_deg,
            ..
        } => {
            let profile = profile_reference(fixture, step, profile, profiles)?;
            let origin = point(fixture, step, &axis.origin, params)?;
            let direction = vector(fixture, step, &axis.direction, params)?;
            let axis = Axis::new(origin, direction).map_err(|source| CorpusError::Axis {
                fixture: name.clone(),
                step: step.name().to_string(),
                source,
            })?;
            let angle = number(fixture, step, angle_deg, params)?.to_radians();
            body(
                arris_ops::revolve(m, profile, axis, angle, control).map_err(op)?,
                Vec::new(),
            )
        }
        Step::Transform {
            of,
            translate,
            rotate,
            ..
        } => {
            let of_body = reference(fixture, step, of, made)?;
            let motion = motion(fixture, step, rotate, translate, params)?;
            body(
                arris_ops::transform(m, of_body, &motion, control).map_err(op)?,
                vec![of_body],
            )
        }
        Step::Mirror { of, plane, .. } => {
            let of_body = reference(fixture, step, of, made)?;
            let reflection = Reflection::new(
                point(fixture, step, &plane.origin, params)?,
                vector(fixture, step, &plane.normal, params)?,
            )
            .map_err(|source| CorpusError::Plane {
                fixture: fixture.name.clone(),
                step: step.name().to_string(),
                source,
            })?;
            body(
                arris_ops::mirror(m, of_body, &reflection, control).map_err(op)?,
                vec![of_body],
            )
        }
        Step::Split {
            of, plane, side, ..
        } => {
            let of_body = reference(fixture, step, of, made)?;
            let origin = point(fixture, step, &plane.origin, params)?;
            let normal = vector(fixture, step, &plane.normal, params)?;
            let frame = match &plane.x {
                Some(x) => Frame::new(origin, normal, vector(fixture, step, x, params)?),
                None => Frame::from_z(origin, normal),
            }
            .map_err(|source| CorpusError::Axis {
                fixture: fixture.name.clone(),
                step: step.name().to_string(),
                source,
            })?;
            let positive = match side {
                Some(side) => number(fixture, step, side, params)? > 0.0,
                None => true,
            };
            let made = arris_ops::split(m, of_body, &frame, control).map_err(op)?;
            Ok(Some(Made {
                body: if positive {
                    made.positive
                } else {
                    made.negative
                },
                provenance: made.provenance,
                inputs: vec![of_body],
                outputs: vec![made.positive, made.negative],
            }))
        }
        Step::Fuse { a, b, bodies, .. } => {
            let names = operand_names(fixture, step, [a, b], bodies, "fuse")?;
            let operands = names
                .into_iter()
                .map(|n| reference(fixture, step, n, made))
                .collect::<Result<Vec<Body>, CorpusError>>()?;
            body(
                arris_ops::fuse_many(m, &operands, control).map_err(op)?,
                operands,
            )
        }
        Step::Common { a, b, .. } => {
            let a = reference(fixture, step, a, made)?;
            let b = reference(fixture, step, b, made)?;
            body(common(m, a, b, control).map_err(op)?, vec![a, b])
        }
        Step::Cut {
            target,
            tool,
            tools,
            ..
        } => {
            let names = operand_names(fixture, step, [tool, &None], tools, "cut")?;
            let target = reference(fixture, step, target, made)?;
            let tools = names
                .into_iter()
                .map(|n| reference(fixture, step, n, made))
                .collect::<Result<Vec<Body>, CorpusError>>()?;
            let operands: Vec<Body> = core::iter::once(target)
                .chain(tools.iter().copied())
                .collect();
            body(
                arris_ops::cut_many(m, target, &tools, control).map_err(op)?,
                operands,
            )
        }
        Step::Fillet {
            of,
            edges,
            radius: size,
            ..
        }
        | Step::Chamfer {
            of,
            edges,
            distance: size,
            ..
        } => {
            let of_body = reference(fixture, step, of, made)?;
            let probe = fixture.recipe.tolerances.probe;
            let mut selected = Vec::with_capacity(edges.len());
            for p in edges {
                let point = point(fixture, step, p, params)?;
                let edge =
                    edge_at(m, of_body, point, probe).map_err(|what| CorpusError::EdgePoint {
                        fixture: name.clone(),
                        step: step.name().to_string(),
                        point: [point.x, point.y, point.z],
                        what,
                    })?;
                selected.push(edge);
            }
            let size = number(fixture, step, size, params)?;
            let blended = if matches!(step, Step::Chamfer { .. }) {
                arris_ops::chamfer(m, of_body, &selected, size, control)
            } else {
                arris_ops::fillet(m, of_body, &selected, size, control)
            };
            body(blended.map_err(op)?, vec![of_body])
        }
        Step::Offset {
            of,
            faces,
            distance,
            ..
        } => {
            let of_body = reference(fixture, step, of, made)?;
            let mut selected = Vec::with_capacity(faces.len());
            for p in faces {
                let point = point(fixture, step, p, params)?;
                let face = face_at(m, of_body, point).map_err(|what| CorpusError::FacePoint {
                    fixture: name.clone(),
                    step: step.name().to_string(),
                    point: [point.x, point.y, point.z],
                    what,
                })?;
                selected.push(face);
            }
            let distance = number(fixture, step, distance, params)?;
            body(
                arris_ops::offset_faces(m, of_body, &selected, distance, control).map_err(op)?,
                vec![of_body],
            )
        }
        Step::Shell {
            of,
            openings,
            thickness,
            side,
            ..
        } => {
            let of_body = reference(fixture, step, of, made)?;
            let mut selected = Vec::with_capacity(openings.len());
            for p in openings {
                let point = point(fixture, step, p, params)?;
                let face = face_at(m, of_body, point).map_err(|what| CorpusError::FacePoint {
                    fixture: name.clone(),
                    step: step.name().to_string(),
                    point: [point.x, point.y, point.z],
                    what,
                })?;
                selected.push(face);
            }
            let thickness = number(fixture, step, thickness, params)?;
            let side = match side {
                ShellSide::Inward => arris_ops::ShellSide::Inward,
                ShellSide::Outward => arris_ops::ShellSide::Outward,
            };
            body(
                arris_ops::shell(m, of_body, &selected, thickness, side, control).map_err(op)?,
                vec![of_body],
            )
        }
        Step::Polyhedron {
            points,
            faces,
            namespace,
            ..
        } => {
            let points = points
                .iter()
                .map(|p| point(fixture, step, p, params))
                .collect::<Result<Vec<_>, _>>()?;
            // The staging and the build in one transaction: a refusal or
            // an interrupt leaves no geometry of the staged solid behind.
            let made = m.transaction(|m| {
                let (builder, keys) =
                    polyhedron(m, &points, faces, *namespace).map_err(|source| {
                        CorpusError::Polyhedron {
                            fixture: name.clone(),
                            step: step.name().to_string(),
                            source,
                        }
                    })?;
                arris_ops::build(m, builder, &keys, control).map_err(op)
            })?;
            body(made, Vec::new())
        }
        Step::Read {
            file,
            sha256,
            id,
            near,
            ..
        } => {
            let near = match near {
                Some(p) => Some(point(fixture, step, p, params)?),
                None => None,
            };
            let probe = fixture.recipe.tolerances.probe;
            let read =
                read_solid(m, &fixture.dir, file, sha256, *id, near, probe).map_err(|fault| {
                    match fault {
                        ReadFault::Refused(refusal) => CorpusError::Refused {
                            fixture: name.clone(),
                            step: step.name().to_string(),
                            refusal: Box::new(refusal),
                        },
                        ReadFault::File(what) => CorpusError::StepFile {
                            fixture: name.clone(),
                            step: step.name().to_string(),
                            what,
                        },
                    }
                })?;
            body((read.body, read.provenance), Vec::new())
        }
    }
}

/// Why a `step` operand gave no solid.
enum ReadFault {
    /// The reader refused the solid named.
    Refused(step::Refusal),
    /// The file is missing, is not the one the recipe hashed, does not
    /// parse, or names no single solid.
    File(String),
}

/// The solid a `step` operand names ([`Step::Read`]): the file `file`
/// beside the fixture in `dir`, held to `sha256`, read into `m` by
/// `arris_io::step::read` in millimetres; of its instances of `#id`, the
/// only one, or the one whose centroid is nearest `near`. Every instance
/// of `#id` must read: a refusal of any is the operand's. A tie for
/// nearest — two centroids whose distances from `near` differ by no more
/// than `probe` — names no solid.
fn read_solid(
    m: &mut Model,
    dir: &Path,
    file: &str,
    sha256: &str,
    id: u64,
    near: Option<Point3>,
    probe: f64,
) -> Result<step::ReadBody, ReadFault> {
    let path = dir.join(file);
    let bytes =
        std::fs::read(&path).map_err(|e| ReadFault::File(format!("{}: {e}", path.display())))?;
    let digest: String = Sha256::digest(&bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    if digest != sha256 {
        return Err(ReadFault::File(format!(
            "{} hashes to {digest}, not the recipe's {sha256}",
            path.display()
        )));
    }
    // Part 21 is ISO 8859-1 outside its escapes; a byte that is not UTF-8
    // can only sit in a string, whose text the reader does not use.
    let text = String::from_utf8_lossy(&bytes);
    let read = step_read(m, &text, &step::ReadOptions::default())
        .map_err(|e| ReadFault::File(format!("{}: {e}", path.display())))?;
    let mut candidates = Vec::new();
    for solid in read.solids.into_iter().filter(|s| s.entity.id == id) {
        candidates.push(solid.result.map_err(ReadFault::Refused)?);
    }
    match (candidates.len(), near) {
        (0, _) => Err(ReadFault::File(format!("{file} has no solid #{id}"))),
        (1, _) => Ok(candidates.remove(0)),
        (n, None) => Err(ReadFault::File(format!(
            "{file} places #{id} {n} times: name one by `near`"
        ))),
        (_, Some(near)) => {
            let mut by_distance = Vec::with_capacity(candidates.len());
            for (k, c) in candidates.iter().enumerate() {
                let props = mass_properties(m, c.body)
                    .map_err(|e| ReadFault::File(format!("#{id}, placement {k}: {e}")))?;
                by_distance.push(((props.centroid - near).norm(), k));
            }
            by_distance.sort_by(|a, b| a.0.total_cmp(&b.0));
            if by_distance[1].0 - by_distance[0].0 <= probe {
                return Err(ReadFault::File(format!(
                    "two placements of #{id} are as near {near:?}"
                )));
            }
            Ok(candidates.swap_remove(by_distance[0].1))
        }
    }
}

/// The edge of `body` a recipe names by `point`: the one
/// `classify_point` answers `On(Edge)` for, when no second edge of the
/// body passes within `probe` of the point — the rule the oracle's
/// nearest-edge search keeps too (`tests/fixtures/README.md`). Errors:
/// what the point names instead.
/// The face of `body` that `point` classifies `On`: a point inside or
/// outside the body, or on an edge or a vertex, names no one face.
fn face_at(m: &Model, body: Body, point: Point3) -> Result<Face, String> {
    match classify_point(m, body, point) {
        Ok(Classification::On(Shape {
            id: EntityId::Face(id),
            ..
        })) => Ok(Face::forward(id)),
        Ok(Classification::On(shape)) => Err(format!("is on {shape}, not inside a face")),
        Ok(Classification::Inside) => Err("is inside the body, on no face".into()),
        Ok(Classification::Outside) => Err("is outside the body, on no face".into()),
        Err(e) => Err(format!("could not be classified: {e}")),
    }
}

fn edge_at(m: &Model, body: Body, point: Point3, probe: f64) -> Result<Edge, String> {
    let on = match classify_point(m, body, point) {
        Ok(Classification::On(shape)) => shape,
        Ok(Classification::Inside) => return Err("is inside the body, on no edge".into()),
        Ok(Classification::Outside) => return Err("is outside the body, on no edge".into()),
        Err(e) => return Err(format!("could not be classified: {e}")),
    };
    let EntityId::Edge(id) = on.id else {
        return Err(format!("is on {on}, not on an edge"));
    };
    let mut near: Vec<EntityId> = Vec::new();
    for edge in m.edges(body).map_err(|e| e.to_string())? {
        let entity = m.edge(edge.id).map_err(|e| e.to_string())?;
        let Some((curve, range)) = entity.curve() else {
            continue;
        };
        let curve = m.curve(curve).map_err(|e| e.to_string())?;
        let Ok(projection) = curve.project(point) else {
            continue;
        };
        let t = match curve.period() {
            Some(p) => {
                let k = ((range.lo() - projection.t) / p).ceil();
                projection.t + k * p
            }
            None => projection.t,
        };
        if projection.distance <= probe && range.lo() - probe <= t && t <= range.hi() + probe {
            near.push(EntityId::Edge(edge.id));
        }
    }
    if near.len() != 1 {
        return Err(format!(
            "is within the probe tolerance of {} edges ({})",
            near.len(),
            near.iter()
                .map(|e| e.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    Ok(Edge::forward(id))
}

/// The operand names of a `fuse` or `cut` step, from its two single-name
/// fields or its list, never both.
fn operand_names<'a>(
    fixture: &Fixture,
    step: &Step,
    single: [&'a Option<String>; 2],
    list: &'a [String],
    what: &str,
) -> Result<Vec<&'a str>, CorpusError> {
    let named: Vec<&str> = single.iter().filter_map(|n| n.as_deref()).collect();
    let wrong = |what: String| CorpusError::Operands {
        fixture: fixture.name.clone(),
        step: step.name().to_string(),
        what,
    };
    match (named.is_empty(), list.is_empty()) {
        (false, false) => Err(wrong(format!(
            "a {what} names its operands singly or as a list, not both"
        ))),
        (true, true) => Err(wrong(format!("a {what} names no operands"))),
        (false, true) => Ok(named),
        (true, false) => Ok(list.iter().map(String::as_str).collect()),
    }
}

fn reference(
    fixture: &Fixture,
    step: &Step,
    name: &str,
    made: &BTreeMap<String, Made>,
) -> Result<Body, CorpusError> {
    let missing = || CorpusError::Reference {
        fixture: fixture.name.clone(),
        step: step.name().to_string(),
        name: name.to_string(),
    };
    if let Some(m) = made.get(name) {
        return Ok(m.body);
    }
    // `<split>.positive` and `<split>.negative`: a side of a split step.
    let (base, side) = name.rsplit_once('.').ok_or_else(missing)?;
    let sides = made.get(base).map(|m| m.outputs.as_slice());
    match (side, sides) {
        ("positive", Some([positive, _])) => Ok(*positive),
        ("negative", Some([_, negative])) => Ok(*negative),
        _ => Err(missing()),
    }
}

/// ADR-0012's invariants on every face-local vertex of the mesh: the
/// face's surface at the corner's own (u, v) is the shared position it
/// stands on, within the largest tolerance of the face and the edges and
/// vertices bounding it — a corner on an edge stands on the edge's curve
/// and takes its (u, v) from the pcurve, which agree to the edge's
/// tolerance and not the face's (docs/DATA-MODEL.md §Tolerances,
/// ADR-0026's amendment of step 5) — and its normal is the
/// surface's own in the face use's sense — outward — wherever the
/// parametrisation is not singular, and a unit vector in the tangent
/// plane where it is.
fn corners_stage(m: &Model, body: Body, mesh: &TriMesh) -> Result<(), String> {
    let corners = mesh
        .corners()
        .ok_or_else(|| "the corner block was asked for and is missing".to_string())?;
    let faces = m.faces(body).map_err(|e| e.to_string())?;
    if corners.faces().len() != faces.len() {
        return Err(format!(
            "{} corner faces for {} faces",
            corners.faces().len(),
            faces.len()
        ));
    }
    for (used, cf) in faces.iter().zip(corners.faces()) {
        if cf.face != used.id {
            return Err(format!(
                "corner face {} where the body has {}",
                cf.face, used.id
            ));
        }
        let face = m.face(used.id).map_err(|e| e.to_string())?;
        let surface = m.surface(face.surface()).map_err(|e| e.to_string())?;
        let reversed = used.orientation == Orientation::Reversed;
        let mut within = face.tolerance();
        for coedge in face.loops().iter().flat_map(|l| l.coedges()) {
            let edge = m.edge(coedge.edge()).map_err(|e| e.to_string())?;
            within = within.max(edge.tolerance());
            for v in [edge.start(), edge.end()] {
                within = within.max(m.vertex(v).map_err(|e| e.to_string())?.tolerance());
            }
        }
        for i in cf.vertices.clone() {
            let ([u, v], normal, shared) = (
                corners.uvs()[i],
                corners.normals()[i],
                corners.positions()[i] as usize,
            );
            let at = Point3::from(mesh.positions()[shared]);
            let on_surface = surface.point(u, v);
            let off = (on_surface - at).norm();
            if off.is_nan() || off > within {
                return Err(format!(
                    "{}: ({u}, {v}) evaluates to {on_surface}, {off:e} from the {at} it stands on, \
                     above the face's and its boundary's tolerance {within:e}",
                    cf.face
                ));
            }
            let n = Vec3::from(normal);
            if let Some(own) = surface.normal(u, v) {
                let outward = if reversed {
                    -own.into_inner()
                } else {
                    own.into_inner()
                };
                if (n - outward).norm() > CORNER_NORMAL_SLACK {
                    return Err(format!(
                        "{}: the normal at ({u}, {v}) is {n}, not the outward {outward}",
                        cf.face
                    ));
                }
            } else {
                let e = surface.eval(u, v);
                for d in [e.du, e.dv] {
                    if n.dot(&d).abs() > CORNER_NORMAL_SLACK * d.norm().max(1.0) {
                        return Err(format!(
                            "{}: the normal at the singular ({u}, {v}) leaves the tangent plane",
                            cf.face
                        ));
                    }
                }
            }
        }
    }
    Ok(())
}

/// How far a corner normal may lie from the direction it should be: the
/// two are the same computation, so this is rounding on a unit vector,
/// not a geometric tolerance.
const CORNER_NORMAL_SLACK: f64 = 1e-12;

/// What the measure stage holds a result to, and whose it is: the
/// oracle's measurements, or under `analytic.measure_differs` the
/// recipe's closed forms of the volume, area, centroid and inertia, each
/// required (ADR-0015).
fn measure_target(
    fixture: &Fixture,
    params: &BTreeMap<String, f64>,
    expected: &Measured,
) -> Result<(Measured, &'static str), CorpusError> {
    let analytic = &fixture.recipe.analytic;
    if analytic.measure_differs.is_none() {
        return Ok((expected.clone(), "the oracle's"));
    }
    let missing = |field: &str| CorpusError::Measure {
        fixture: fixture.name.clone(),
        what: format!("analytic.measure_differs needs analytic.{field}"),
    };
    let eval = |n: &Num| {
        n.eval(params).map_err(|source| CorpusError::Expression {
            fixture: fixture.name.clone(),
            step: "analytic".into(),
            source,
        })
    };
    let mut target = expected.clone();
    target.volume = Some(eval(
        analytic.volume.as_ref().ok_or_else(|| missing("volume"))?,
    )?);
    target.area = Some(eval(
        analytic.area.as_ref().ok_or_else(|| missing("area"))?,
    )?);
    let centroid = analytic
        .centroid
        .as_ref()
        .ok_or_else(|| missing("centroid"))?;
    target.centroid = Some([
        eval(&centroid[0])?,
        eval(&centroid[1])?,
        eval(&centroid[2])?,
    ]);
    let inertia = analytic
        .inertia
        .as_ref()
        .ok_or_else(|| missing("inertia"))?;
    let mut rows = [[0.0; 3]; 3];
    for (row, forms) in rows.iter_mut().zip(inertia) {
        for (x, form) in row.iter_mut().zip(forms) {
            *x = eval(form)?;
        }
    }
    target.inertia = Some(rows);
    Ok((target, "the closed form's"))
}

/// Compares Arris's mass properties against the oracle's, quantity by
/// quantity: volume and area relative, the centroid's distance
/// absolute, each component of the inertia tensor relative to the
/// tensor's largest one, so a product of inertia that cancels to zero is
/// not compared against itself. A quantity the oracle did not record is
/// skipped. Errors: the first quantity that differs, named with both
/// values.
pub(crate) fn compare_mass(
    m: &Model,
    body: Body,
    expected: &Measured,
    by: &str,
    tolerances: &Tolerances,
) -> Result<(), String> {
    let found = mass_properties(m, body).map_err(|e| e.to_string())?;
    let relative = |name: &str, a: f64, e: f64, tolerance: f64| -> Result<(), String> {
        let difference = (a - e).abs() / e.abs().max(a.abs()).max(f64::MIN_POSITIVE);
        if difference.is_nan() || difference > tolerance {
            return Err(format!(
                "{name} {a} vs {by} {e}: {difference:e} relative, above {tolerance:e}"
            ));
        }
        Ok(())
    };
    if let Some(volume) = expected.volume {
        relative("volume", found.volume, volume, tolerances.volume_rel)?;
    }
    if let Some(area) = expected.area {
        relative("area", found.area, area, tolerances.area_rel)?;
    }
    if let Some(centroid) = expected.centroid {
        let oracle = Point3::new(centroid[0], centroid[1], centroid[2]);
        let distance = (found.centroid - oracle).norm();
        if distance.is_nan() || distance > tolerances.centroid_abs {
            return Err(format!(
                "centroid {} vs {by} {oracle}: {distance:e} apart, above centroid_abs {:e}",
                found.centroid, tolerances.centroid_abs
            ));
        }
    }
    if let Some(inertia) = expected.inertia {
        let scale = inertia
            .iter()
            .flatten()
            .fold(0.0f64, |m, x| m.max(x.abs()))
            .max(f64::MIN_POSITIVE);
        for (i, row) in inertia.iter().enumerate() {
            for (j, &e) in row.iter().enumerate() {
                let a = found.inertia[(i, j)];
                let difference = (a - e).abs() / scale;
                if difference.is_nan() || difference > tolerances.inertia_rel {
                    return Err(format!(
                        "inertia[{i}][{j}] {a} vs {by} {e}: {difference:e} of the tensor, above inertia_rel {:e}",
                        tolerances.inertia_rel
                    ));
                }
            }
        }
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

        let fault = |r: Result<step::ReadBody, ReadFault>| match r {
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
