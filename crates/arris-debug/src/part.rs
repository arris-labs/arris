//! The `part` fixture kind (ADR-0026): a STEP file Arris did not write,
//! beside `fixture.json`, `expected.json` and a dump per solid read. The
//! format is `tests/fixtures/README.md` §Part fixtures; this module is its
//! Rust reading, its runner ([`run`]) and its lint ([`lint`]).
//!
//! `fixture.json` names the file, where it came from, its licence and
//! its SHA-256, and records every solid instance Arris's reader returns,
//! in the reader's order, with its expected outcome: `read`, or the
//! refusal it is and why that refusal is right. `expected.json` is the
//! oracle's reading of the same file, healed as Open CASCADE heals by
//! default: one entry per solid it reads, with the `#id` behind it, and
//! `occt_heals` where it had to heal it (ADR-0026 §3).
//!
//! A `read` solid is held to the checker at `Full` (what it cannot decide
//! on a NURBS face left unchecked), to one of the oracle's solids of its
//! `#id` — the nearest by centroid, each taken once — in counts and genus
//! where healing changed no topology, in volume, area, centroid and
//! inertia within the fixture's tolerances widened to the body's own, to
//! a closed mesh, and to its committed dump. A `refused` one is held to
//! exactly its refusal kind. A solid of the oracle's that no read matches
//! must be one Arris refuses; an Arris read that no oracle solid matches
//! is a solid the file does not have.

use crate::unmetered::mass_properties;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use arris_io::arris_check::arris_topo::arris_math::Point3;
use arris_io::arris_check::arris_topo::{Body, Model};
use arris_io::step::{self, ReadOptions};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use crate::battery::{Case, Class, OracleCase};
use crate::corpus::{
    self, CorpusError, Target, blessing, check_leaving_nurbs, compare_mass, mesh_check,
    within_own_tolerance,
};
use crate::dump::dump_text;
use crate::fixtures::{
    Counts, FixtureError, Measured, PrecisionSpec, REGRESSION_AREA, ReadRefused, Tolerances,
    name_of, recipe_hash,
};

/// What a part fixture expects of one solid instance of its file.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Outcome {
    /// The reader returns a body, held to the oracle's reading.
    Read,
    /// The reader refuses it, with this kind (as `RefusalKind` prints it),
    /// for the reason given: why the refusal is right, not which it is
    /// (ADR-0026 §4).
    Refused(ReadRefused),
}

/// One solid instance of a part's file, as the reader numbers it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PartSolid {
    /// The `#id` of its `MANIFOLD_SOLID_BREP` or `BREP_WITH_VOIDS`, or
    /// of the entity that stands where one would.
    pub id: u64,
    /// Which placement of it (`FileEntity::instance`).
    pub instance: u32,
    /// What the reader is expected to make of it.
    pub outcome: Outcome,
    /// The class each battery stage of a `read` solid is recorded at, by
    /// stage (`crate::battery`): what the runner holds it to. Empty for a
    /// solid the fixture has no battery for.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub battery: BTreeMap<String, Class>,
    /// The cycle each refusal of this solid blocks, as
    /// `crate::histogram::Cycle` prints it, by stage: `read` for a solid
    /// the reader refuses, and each battery stage recorded as
    /// [`Class::ArrisRefuses`]. What the histogram counts from; the runner
    /// holds it to ADR-0026 §5's table on every run.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub blocks: BTreeMap<String, String>,
}

/// A part fixture's `fixture.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Part {
    /// What the part is.
    #[serde(default)]
    pub description: String,
    /// The STEP file, beside `fixture.json`.
    pub file: String,
    /// Where the file came from: its URL and archive.
    pub source: String,
    /// Its licence, quoted from the source.
    pub licence: String,
    /// The file's SHA-256, lower-case hex.
    pub sha256: String,
    /// Every solid instance the reader returns, in its order.
    pub solids: Vec<PartSolid>,
    /// The precision the model is read into.
    #[serde(default, skip_serializing_if = "is_default")]
    pub precision: PrecisionSpec,
    /// Comparison tolerances, before each read body's own widens them.
    #[serde(default, skip_serializing_if = "is_default")]
    pub tolerances: Tolerances,
    /// The fixtures under `regression/` this part waits on: each a reader
    /// bug the part meets, shrunk (ADR-0026 §4). While any is listed the
    /// runner does not read the part, and the lint fails once one has
    /// left `regression/`, so the fix that moves it lifts the exclusion.
    /// `solids` records the outcomes the part is to have once they are
    /// fixed.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub waits_on: Vec<String>,
    /// How long the read may take, in seconds of the test profile's
    /// optimised build: set on the fixture a slow read is shrunk to
    /// (ADR-0026 §6), and on no other, since a shared machine's timing is
    /// no assertion.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub read_seconds: Option<f64>,
    /// The battery's operands, by solid (`crate::battery::key`) and stage:
    /// written once by `crate::battery::derive`, then data both kernels
    /// build. Empty for a part shrunk from another, whose battery is its
    /// source's.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub battery: BTreeMap<String, BTreeMap<String, Case>>,
}

fn is_default<T: Default + PartialEq>(value: &T) -> bool {
    *value == T::default()
}

/// One solid of the oracle's reading of a part's file.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OracleSolid {
    /// The `#id` of the solid entity it came from.
    pub id: u64,
    /// Open CASCADE had to heal it to make it a solid: unhealed, a solid
    /// of this `#id` fails `BRepCheck_Analyzer`, has other counts, or is
    /// not read at all.
    pub occt_heals: bool,
    /// The counts of the unhealed reading, where it has one: where they
    /// are [`Measured::counts`], healing changed no topology.
    pub unhealed_counts: Option<Counts>,
    /// What `measure` records of the healed solid.
    #[serde(flatten)]
    pub measured: Measured,
}

impl OracleSolid {
    /// The counts Arris's reading is held to, where healing changed no
    /// topology.
    pub fn held_counts(&self) -> Option<Counts> {
        (!self.occt_heals || self.unhealed_counts == Some(self.measured.counts))
            .then_some(self.measured.counts)
    }
}

/// A part fixture's `expected.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PartExpected {
    /// The `cadquery-ocp` version that wrote it.
    pub occt: String,
    /// The hash of the fixture's [`crate::fixtures::PART_KEYS`].
    pub recipe_sha256: String,
    /// Every solid the oracle reads, in its transfer's order.
    pub solids: Vec<OracleSolid>,
    /// Its answer to each battery case, by solid and stage.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub battery: BTreeMap<String, BTreeMap<String, OracleCase>>,
}

/// A loaded part fixture.
#[derive(Debug, Clone, PartialEq)]
pub struct PartFixture {
    /// The directory.
    pub dir: PathBuf,
    /// `<area>/<slug>`.
    pub name: String,
    /// `fixture.json`.
    pub part: Part,
    /// The hash of `fixture.json` as loaded.
    pub recipe_sha256: String,
    /// `expected.json`.
    pub expected: PartExpected,
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T, FixtureError> {
    let text = std::fs::read_to_string(path).map_err(|source| FixtureError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    serde_json::from_str(&text).map_err(|source| FixtureError::Json {
        path: path.to_path_buf(),
        source,
    })
}

/// Loads a part fixture: both files, and the hash of `fixture.json`.
pub fn load(dir: &Path) -> Result<PartFixture, FixtureError> {
    let path = dir.join("fixture.json");
    let raw: serde_json::Value = read_json(&path)?;
    let part: Part = serde_json::from_value(raw.clone()).map_err(|source| FixtureError::Json {
        path: path.clone(),
        source,
    })?;
    let recipe_sha256 =
        recipe_hash(&raw).map_err(|kind| FixtureError::UnknownKind { path, kind })?;
    Ok(PartFixture {
        dir: dir.to_path_buf(),
        name: name_of(dir),
        part,
        recipe_sha256,
        expected: read_json(&dir.join("expected.json"))?,
    })
}

/// Writes `part` as the `fixture.json` in `dir`: `"kind": "part"` first,
/// then its fields in declaration order, two-space indented, as
/// `crate::battery::derive`'s caller rewrites a fixture. Errors: the file
/// cannot be written.
pub fn save(dir: &Path, part: &Part) -> std::io::Result<()> {
    #[derive(Serialize)]
    struct File<'a> {
        kind: &'static str,
        #[serde(flatten)]
        part: &'a Part,
    }
    let text = serde_json::to_string_pretty(&File { kind: "part", part })
        .map_err(std::io::Error::other)?;
    std::fs::write(dir.join("fixture.json"), text + "\n")
}

/// The dump of one solid read: `dump.<id>.<instance>.txt`.
pub fn dump_path(dir: &Path, id: u64, instance: u32) -> PathBuf {
    dir.join(format!("dump.{id}.{instance}.txt"))
}

pub(crate) fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}

/// Runs a part fixture: the file read, every solid held to its outcome,
/// and every solid of the oracle's accounted for (module docs). Errors:
/// [`CorpusError::StepFile`] for a file that is missing, is not the one
/// hashed or does not parse; [`CorpusError::Part`] for a solid whose
/// outcome is not the one recorded, or that one side reads and the other
/// does not have; and the stage errors of a read solid.
///
/// ```no_run
/// use arris_debug::{fixtures, part};
///
/// part::run(&fixtures::corpus_root().join("real/nist-ftc-09")).unwrap();
/// ```
pub fn run(dir: &Path) -> Result<(), CorpusError> {
    let fixture = load(dir)?;
    let name = &fixture.name;
    let fail = |what: String| CorpusError::Part {
        fixture: name.clone(),
        what,
    };
    if !fixture.part.waits_on.is_empty() {
        // Excluded until the bugs it meets are fixed: each must still be
        // waiting under `regression/`.
        return match waiting_problems(&fixture).first() {
            Some(problem) => Err(fail(problem.clone())),
            None => Ok(()),
        };
    }
    let file = |what: String| CorpusError::StepFile {
        fixture: name.clone(),
        step: fixture.part.file.clone(),
        what,
    };
    let path = dir.join(&fixture.part.file);
    let bytes = std::fs::read(&path).map_err(|e| file(e.to_string()))?;
    let digest = sha256_hex(&bytes);
    if digest != fixture.part.sha256 {
        return Err(file(format!(
            "hashes to {digest}, not the fixture's {}",
            fixture.part.sha256
        )));
    }
    let mut model = Model::new(fixture.part.precision.precision()).map_err(|source| {
        CorpusError::Precision {
            fixture: name.clone(),
            source,
        }
    })?;
    let text = String::from_utf8_lossy(&bytes);
    let started = std::time::Instant::now();
    let read =
        step::read(&mut model, &text, &ReadOptions::default()).map_err(|e| file(e.to_string()))?;
    let seconds = started.elapsed().as_secs_f64();
    if let Some(budget) = fixture.part.read_seconds.filter(|&b| seconds > b) {
        return Err(fail(format!(
            "the read took {seconds:.1} s, past its budget of {budget} s"
        )));
    }

    let found: Vec<(u64, u32)> = (read.solids.iter())
        .map(|s| (s.entity.id, s.entity.instance))
        .collect();
    let listed: Vec<(u64, u32)> = (fixture.part.solids.iter())
        .map(|s| (s.id, s.instance))
        .collect();
    if found != listed {
        return Err(fail(format!(
            "the file reads to the solid instances {found:?}, but the fixture lists {listed:?}"
        )));
    }
    let oracle = &fixture.expected.solids;
    let mut taken = vec![false; oracle.len()];
    let mut refused: BTreeMap<u64, usize> = BTreeMap::new();
    for (solid, spec) in read.solids.iter().zip(&fixture.part.solids) {
        let at = format!("#{}[{}]", spec.id, spec.instance);
        match (&spec.outcome, &solid.result) {
            (Outcome::Refused(expected), Err(refusal)) => {
                let kind = refusal.kind().to_string();
                if kind != expected.kind {
                    return Err(fail(format!(
                        "{at} is refused as {kind} ({refusal}), not as {}",
                        expected.kind
                    )));
                }
                let cycle = crate::histogram::blocks_refusal(refusal).to_string();
                if spec.blocks.get("read") != Some(&cycle) {
                    return Err(fail(format!(
                        "{at} is refused as blocking {cycle} (ADR-0026 §5), but records {:?}",
                        spec.blocks.get("read")
                    )));
                }
                *refused.entry(spec.id).or_default() += 1;
            }
            (Outcome::Refused(expected), Ok(_)) => {
                return Err(fail(format!(
                    "{at} reads, where the fixture records a refusal as {}: lift it",
                    expected.kind
                )));
            }
            (Outcome::Read, Err(refusal)) => {
                return Err(CorpusError::Refused {
                    fixture: name.clone(),
                    step: at,
                    refusal: Box::new(refusal.clone()),
                });
            }
            (Outcome::Read, Ok(back)) => {
                if spec.blocks.contains_key("read") {
                    return Err(fail(format!(
                        "{at} reads, but records the cycle its read refusal blocks"
                    )));
                }
                // A part waiting under `regression/` commits no dump.
                let dump = !name.starts_with(&format!("{REGRESSION_AREA}/"));
                read_stages(&fixture, &model, back.body, spec, &mut taken, dump)?;
                #[cfg(not(target_arch = "wasm32"))]
                battery_stages(&fixture, &model, back, spec)?;
            }
        }
    }
    // Every solid the oracle reads is one Arris reads, or one of the same
    // entity it refuses.
    let mut left: BTreeMap<u64, usize> = BTreeMap::new();
    for (solid, _) in oracle.iter().zip(&taken).filter(|(_, t)| !**t) {
        *left.entry(solid.id).or_default() += 1;
    }
    for (id, n) in left {
        let refusals = refused.get(&id).copied().unwrap_or(0);
        if n > refusals {
            return Err(fail(format!(
                "Open CASCADE reads {n} solid(s) of #{id} that Arris neither reads nor refuses"
            )));
        }
    }
    Ok(())
}

/// The stages of one solid read (module docs), matching it to the
/// oracle's solid of its entity nearest by centroid, which `taken` marks;
/// the dump held to the committed one where `dump`, which a part the
/// corpus does not hold (the fetched tier) has none of.
pub(crate) fn read_stages(
    fixture: &PartFixture,
    m: &Model,
    body: Body,
    spec: &PartSolid,
    taken: &mut [bool],
    dump: bool,
) -> Result<(), CorpusError> {
    let name = format!("{} #{}[{}]", fixture.name, spec.id, spec.instance);
    let fail = |what: String| CorpusError::Part {
        fixture: fixture.name.clone(),
        what: format!("#{}[{}] {what}", spec.id, spec.instance),
    };
    let report = check_leaving_nurbs(&name, m, body)?;
    let mass =
        mass_properties(m, body).map_err(|e| fail(format!("has no mass properties: {e}")))?;
    let oracle = &fixture.expected.solids;
    let mut nearest: Option<(f64, usize)> = None;
    for (k, solid) in oracle.iter().enumerate() {
        if solid.id != spec.id || taken[k] {
            continue;
        }
        let Some(c) = solid.measured.centroid else {
            continue;
        };
        let d = (Point3::new(c[0], c[1], c[2]) - mass.centroid).norm();
        if nearest.is_none_or(|(best, _)| d < best) {
            nearest = Some((d, k));
        }
    }
    let Some((_, k)) = nearest else {
        return Err(fail(format!(
            "reads to a solid (centroid {:?}) that Open CASCADE's reading of the file does not have",
            mass.centroid
        )));
    };
    taken[k] = true;
    let expected = &oracle[k];

    // Counts and genus, where healing changed no topology.
    if let Some(counts) = expected.held_counts() {
        let line = report.euler().ok_or_else(|| CorpusError::Check {
            fixture: name.clone(),
            report: Box::new(report.clone()),
        })?;
        let found = Counts {
            vertices: line.vertices,
            edges: line.edges,
            faces: line.faces,
            loops: line.loops,
            shells: line.shells,
            solids: 1,
        };
        if found != counts {
            return Err(CorpusError::Counts {
                fixture: name,
                expected: counts,
                found,
            });
        }
        if let Some(genus) = expected.measured.genus.filter(|&g| g != line.genus) {
            return Err(CorpusError::Genus {
                fixture: name,
                expected: genus,
                found: line.genus,
            });
        }
    }

    // Mass properties, to the fixture's tolerances or the body's own.
    let tolerances = within_own_tolerance(&fixture.part.tolerances, m, body).map_err(&fail)?;
    compare_mass(m, body, &expected.measured, "the oracle's", &tolerances).map_err(|what| {
        CorpusError::Measure {
            fixture: name.clone(),
            what,
        }
    })?;
    let target = Target {
        measured: expected.measured.clone(),
        by: "the oracle's",
    };
    mesh_check(&name, m, body, &tolerances, &target)?;
    if !dump {
        return Ok(());
    }

    // The dump.
    let path = dump_path(&fixture.dir, spec.id, spec.instance);
    let dump = dump_text(m, body).map_err(|e| CorpusError::Dump {
        fixture: name.clone(),
        path: path.clone(),
        what: e.to_string(),
    })?;
    corpus::check_dump(&name, &path, &dump, blessing())
}

/// The battery of one solid read (`crate::battery`): every stage the
/// fixture has operands for, and `write_read`, run and held to the class
/// the solid records for it — or, where it records that the stage waits
/// on a fixture under [`REGRESSION_AREA`], skipped while that fixture is
/// there. A solid with no battery and no classes passes. Errors: [`CorpusError::Part`] for a stage with no class or a
/// class with no stage, an outcome that is a kernel bug — a
/// disagreement, a checker violation, a panic, an internal fault — and
/// an outcome of another class than the one recorded, and a wait on a
/// fixture that has left [`REGRESSION_AREA`].
#[cfg(not(target_arch = "wasm32"))]
fn battery_stages(
    fixture: &PartFixture,
    m: &Model,
    back: &arris_io::step::ReadBody,
    spec: &PartSolid,
) -> Result<(), CorpusError> {
    use crate::battery::{self, STAGES};
    let key = battery::key(spec.id, spec.instance);
    let fail = |what: String| CorpusError::Part {
        fixture: fixture.name.clone(),
        what: format!("{key} {what}"),
    };
    let cases = fixture.part.battery.get(&key);
    if cases.is_none() && spec.battery.is_empty() {
        return match spec.blocks.keys().next() {
            Some(stage) => Err(fail(format!(
                "records the cycle {stage} blocks, and has no battery"
            ))),
            None => Ok(()),
        };
    }
    let stages: Vec<&str> = STAGES
        .into_iter()
        .filter(|&s| s == "write_read" || cases.is_some_and(|c| c.contains_key(s)))
        .collect();
    let recorded: Vec<&str> = spec.battery.keys().map(String::as_str).collect();
    let mut sorted = stages.clone();
    sorted.sort_unstable();
    if recorded != sorted {
        return Err(fail(format!(
            "records classes for the stages {recorded:?}, but its battery has {stages:?}"
        )));
    }
    let hold = battery::holds_counts(fixture, spec.id);
    let waits = |slug: &str| {
        slug.starts_with(&format!("{REGRESSION_AREA}/"))
            && crate::fixtures::corpus_root()
                .join(slug)
                .join("fixture.json")
                .is_file()
    };
    for stage in stages {
        let expected = &spec.battery[stage];
        if let Class::WaitsOn(slug) = expected {
            if !waits(slug) {
                return Err(fail(format!(
                    "{stage} waits on {slug}, which is not a fixture under {REGRESSION_AREA}/ any more: record the class the fix gives"
                )));
            }
            continue;
        }
        let name = format!("{} {key} {stage}", fixture.name);
        let judged = match cases.and_then(|c| c.get(stage)) {
            None => battery::write_read(&name, m, back.body, &fixture.part.tolerances),
            Some(case) => {
                let oracle = battery::oracle_case(fixture, &key, stage).map_err(&fail)?;
                battery::judge(fixture, &name, case, oracle, hold, Some((m, back)))
            }
        };
        let outcome = &judged.outcome;
        // The cycle an Arris refusal blocks, held to the table.
        let cycle = match (outcome, &judged.refused) {
            (crate::differential::Outcome::ArrisRefuses(_), Some(refused)) => {
                crate::histogram::Stage::of_name(stage)
                    .and_then(|s| refused.blocks(s))
                    .map(|c| c.to_string())
            }
            _ => None,
        };
        if spec.blocks.get(stage) != cycle.as_ref() {
            return Err(fail(format!(
                "{stage} blocks {cycle:?} by ADR-0026 §5's table, but records {:?}",
                spec.blocks.get(stage)
            )));
        }
        match Class::of(outcome) {
            None => {
                return Err(fail(format!(
                    "{stage}: {outcome} — a kernel bug, shrunk to regression/ (ADR-0026 §4)"
                )));
            }
            Some(class) if class != *expected => {
                return Err(fail(format!(
                    "{stage} is {outcome}, recorded as {expected:?}"
                )));
            }
            Some(_) => {}
        }
    }
    Ok(())
}

/// What is wrong with the exclusions `fixture` waits on: each must name a
/// part fixture still under [`REGRESSION_AREA`].
fn waiting_problems(fixture: &PartFixture) -> Vec<String> {
    let root = crate::fixtures::corpus_root();
    let mut problems = Vec::new();
    for slug in &fixture.part.waits_on {
        let dir = root.join(slug);
        let waits = slug.starts_with(&format!("{REGRESSION_AREA}/"))
            && crate::fixtures::kind_of(&dir).is_ok_and(|k| k == crate::fixtures::Kind::Part);
        if !waits {
            problems.push(format!(
                "waits on {slug}, which is not a part fixture under {REGRESSION_AREA}/ any more: lift the exclusion and record the outcomes the fix gives"
            ));
        }
    }
    problems
}

/// The lint of a part fixture: both files present and parseable, the
/// file present and its SHA-256 the fixture's, `expected.json` not stale,
/// every oracle solid on the Euler line with its genus, every refusal
/// with a reason, every exclusion and every battery stage's wait a
/// fixture still under [`REGRESSION_AREA`], and every `read` solid's dump committed — unless
/// the part waits, having not passed yet — with none under
/// [`REGRESSION_AREA`], where a fixture waits for its fix. Returns every
/// problem found, empty when clean.
pub fn lint(dir: &Path) -> Vec<String> {
    let fixture = match load(dir) {
        Ok(f) => f,
        Err(e) => return vec![format!("{}: {e}", dir.display())],
    };
    let name = &fixture.name;
    let mut problems = Vec::new();
    let mut problem = |text: String| problems.push(format!("{name}: {text}"));
    if fixture.expected.recipe_sha256 != fixture.recipe_sha256 {
        problem(format!(
            "expected.json is stale: recipe hash {} but the fixture hashes to {} — rerun tools/oracle/expected.py",
            fixture.expected.recipe_sha256, fixture.recipe_sha256
        ));
    }
    match std::fs::read(dir.join(&fixture.part.file)) {
        Ok(bytes) if sha256_hex(&bytes) != fixture.part.sha256 => problem(format!(
            "{} hashes to {}, not the fixture's {}",
            fixture.part.file,
            sha256_hex(&bytes),
            fixture.part.sha256
        )),
        Ok(_) => {}
        Err(e) => problem(format!("{}: {e}", fixture.part.file)),
    }
    for field in [&fixture.part.source, &fixture.part.licence] {
        if field.trim().is_empty() {
            problem("a part names its source and quotes its licence".into());
        }
    }
    for solid in &fixture.expected.solids {
        let m = &solid.measured;
        let (Some(chi), Some(genus)) = (m.euler_characteristic, m.genus) else {
            continue;
        };
        let line = 2 * (m.counts.shells as i64 - genus);
        if chi != line {
            problem(format!(
                "the oracle's #{} is off the Euler line: χ {chi}, 2(S − G) {line}",
                solid.id
            ));
        }
    }
    for p in waiting_problems(&fixture) {
        problem(p);
    }
    let root = crate::fixtures::corpus_root();
    for solid in &fixture.part.solids {
        for (stage, class) in &solid.battery {
            let Class::WaitsOn(slug) = class else {
                continue;
            };
            let open = slug.starts_with(&format!("{REGRESSION_AREA}/"))
                && root.join(slug).join("fixture.json").is_file();
            if !open {
                problem(format!(
                    "#{}[{}] {stage} waits on {slug}, which is not a fixture under {REGRESSION_AREA}/ any more: record the class the fix gives",
                    solid.id, solid.instance
                ));
            }
        }
    }
    let area = name.split('/').next().unwrap_or_default();
    let waiting = !fixture.part.waits_on.is_empty();
    for solid in &fixture.part.solids {
        let dump = dump_path(dir, solid.id, solid.instance);
        match &solid.outcome {
            Outcome::Refused(r) if r.why.trim().is_empty() => problem(format!(
                "#{}[{}] is refused as {} with no reason why the refusal is right",
                solid.id, solid.instance, r.kind
            )),
            Outcome::Refused(_) => {}
            Outcome::Read if area == REGRESSION_AREA && dump.is_file() => problem(format!(
                "{} is committed under {REGRESSION_AREA}/: the part passes, so it moves into its area",
                dump.display()
            )),
            Outcome::Read if area != REGRESSION_AREA && !waiting && !dump.is_file() => {
                problem(format!(
                    "#{}[{}] reads but {} is not committed (ARRIS_BLESS=1)",
                    solid.id,
                    solid.instance,
                    dump.file_name().unwrap_or_default().to_string_lossy()
                ))
            }
            Outcome::Read => {}
        }
    }
    problems
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A scratch copy of the part fixture `name` whose `fixture.json` has
    /// `from` replaced by `to`.
    fn edited(name: &str, tag: &str, from: &str, to: &str) -> PathBuf {
        let source = crate::fixtures::corpus_root().join(name);
        let scratch = std::env::temp_dir().join(format!("arris-part-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&scratch);
        std::fs::create_dir_all(&scratch).expect("a scratch directory");
        for entry in std::fs::read_dir(&source).expect("the fixture") {
            let path = entry.expect("an entry").path();
            std::fs::copy(&path, scratch.join(path.file_name().expect("a file"))).expect("a copy");
        }
        let recipe = scratch.join("fixture.json");
        let text = std::fs::read_to_string(&recipe).expect("the fixture");
        let changed = text.replace(from, to);
        assert_ne!(changed, text, "{from} is in {name}'s fixture.json");
        std::fs::write(&recipe, changed).expect("the edited fixture");
        scratch
    }

    /// The outcome recorded is the one held: a refusal of another kind
    /// fails, and so does a refusal recorded for a solid that reads.
    #[test]
    fn a_part_is_held_to_the_outcome_it_records() {
        let scratch = edited(
            "real/nist-ftc-09-offset",
            "kind",
            r#""kind": "offset""#,
            r#""kind": "gap past the cap""#,
        );
        let e = run(&scratch).expect_err("an offset is not a gap");
        assert!(
            matches!(&e, CorpusError::Part { what, .. } if what.contains("is refused as offset")),
            "{e}"
        );
        let _ = std::fs::remove_dir_all(&scratch);

        let scratch = edited(
            "real/nist-ftc-09",
            "lift",
            r#""outcome": "read""#,
            r#""outcome": {"refused": {"kind": "offset", "why": "none"}}"#,
        );
        let e = run(&scratch).expect_err("the part reads");
        assert!(
            matches!(&e, CorpusError::Part { what, .. } if what.contains("lift it")),
            "{e}"
        );
        let _ = std::fs::remove_dir_all(&scratch);
    }

    /// A battery stage is held to the class it records, and a stage that
    /// waits must wait on a fixture still under `regression/`.
    #[test]
    fn a_battery_stage_is_held_to_its_class() {
        let scratch = edited(
            "real/nist-ftc-11",
            "battery",
            r#""fillet": "both-refuse""#,
            r#""fillet": "agree""#,
        );
        let e = run(&scratch).expect_err("both kernels refuse the fillet");
        assert!(
            matches!(&e, CorpusError::Part { what, .. } if what.contains("fillet is BothRefuse, recorded as Agree")),
            "{e}"
        );
        let _ = std::fs::remove_dir_all(&scratch);

        let scratch = edited(
            "real/nist-ftc-11",
            "wait",
            r#""fillet": "both-refuse""#,
            r#""fillet": {"waits-on": "regression/no-such-fixture"}"#,
        );
        let e = run(&scratch).expect_err("the wait names no fixture");
        assert!(
            matches!(&e, CorpusError::Part { what, .. } if what.contains("record the class the fix gives")),
            "{e}"
        );
        assert!(
            lint(&scratch)
                .iter()
                .any(|p| p.contains("regression/no-such-fixture"))
        );
        let _ = std::fs::remove_dir_all(&scratch);
    }

    /// A refusal is held to the cycle it records, by ADR-0026 §5's table.
    #[test]
    fn a_refusal_is_held_to_the_cycle_it_blocks() {
        let scratch = edited(
            "real/nist-ctc-02",
            "blocks",
            r#""read": "healing""#,
            r#""read": "NURBS""#,
        );
        let e = run(&scratch).expect_err("a gap blocks healing");
        assert!(
            matches!(&e, CorpusError::Part { what, .. } if what.contains("blocking healing")),
            "{e}"
        );
        let _ = std::fs::remove_dir_all(&scratch);
    }
}
