//! Running the Open CASCADE oracle on a file Arris wrote
//! (`tools/oracle/README.md`): the seam between a test and `compare.py`
//! for a STEP file, `mesh.py` for an STL one, and between a test's own
//! recipe and `expected.py` for a body the corpus has no fixture for —
//! one at a time, or many in one process ([`expected_batch`]). A
//! missing environment is a loud error naming the command that creates
//! it, never a skip. Every answer the oracle settles is kept in
//! [`cache`] by what produced it, so an unchanged call starts no Python.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::sync::atomic::{AtomicUsize, Ordering};

pub mod cache;

use crate::fixtures::Recipe;

/// Why the oracle did not answer, or answered no.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum OracleError {
    /// The STEP text could not be written to the scratch directory.
    #[error("could not write {path}: {message}")]
    Write {
        /// The file.
        path: PathBuf,
        /// The cause.
        message: String,
    },
    /// `uv` could not be run, or `compare.py` or `expected.py` refused:
    /// no environment, a stale `expected.json`, an unknown variant, a
    /// recipe the oracle cannot build.
    #[error(
        "the oracle could not run: {message}\nrun `uv sync --project tools/oracle` (tools/oracle/README.md)"
    )]
    Environment {
        /// What `uv` or `compare.py` said.
        message: String,
    },
    /// `compare.py` ran and found a difference; the table says which.
    #[error("{fixture} does not match Arris's STEP ({file}):\n{table}")]
    Mismatch {
        /// The fixture compared against.
        fixture: String,
        /// The STEP file, kept for inspection.
        file: PathBuf,
        /// The comparison table.
        table: String,
    },
}

static SPAWNS: AtomicUsize = AtomicUsize::new(0);

/// How many `uv` processes this process has started for the oracle: the
/// number a test asserts on to show a call was answered from [`cache`].
pub fn spawns() -> usize {
    SPAWNS.load(Ordering::Relaxed)
}

/// `uv run --project tools/oracle tools/oracle/<script>` from the
/// workspace root, with its arguments still to add.
fn uv(script: &str) -> Command {
    let mut command = Command::new("uv");
    command
        .current_dir(workspace_root())
        .args(["run", "--project", "tools/oracle"])
        .arg(format!("tools/oracle/{script}"));
    command
}

/// Runs `command`, counted by [`spawns`].
fn spawn(command: &mut Command) -> Result<Output, OracleError> {
    SPAWNS.fetch_add(1, Ordering::Relaxed);
    command.output().map_err(|e| OracleError::Environment {
        message: format!("could not run `uv`: {e}"),
    })
}

/// Stores a settled answer. A cache that cannot be written is no reason
/// to fail a call the oracle answered, so the error is dropped.
fn keep(slot: Option<(PathBuf, cache::Key)>, bytes: &[u8]) {
    if let Some((dir, key)) = slot {
        let _ = cache::store(&dir, &key, bytes);
    }
}

fn environment(output: &Output) -> OracleError {
    OracleError::Environment {
        message: format!(
            "{}{}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        ),
    }
}

/// The workspace root: two levels above this crate's manifest.
pub fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap_or_else(|_| Path::new(env!("CARGO_MANIFEST_DIR")).join("../.."))
}

/// The scratch directory STEP files go to for the oracle to read:
/// `target/inspect/` under the workspace root (gitignored, kept after a
/// failure so the file can be looked at).
pub fn scratch_dir() -> PathBuf {
    workspace_root().join("target/inspect")
}

/// Writes `recipe` as `target/inspect/<name>/fixture.json` and has
/// `expected.py` write its `expected.json` beside it: a scratch fixture
/// for a test that holds a body to the oracle's reading of its STEP
/// through [`compare_dir`] without a fixture in the corpus — a body whose
/// fixture the corpus cannot yet run, held to closed forms and to the
/// oracle's volume here instead. Returns the directory. Errors:
/// [`OracleError::Write`]; [`OracleError::Environment`] when `uv` or
/// `expected.py` could not run or the oracle refused the recipe.
pub fn scratch_fixture(name: &str, recipe: &Recipe) -> Result<PathBuf, OracleError> {
    let dir = scratch_dir().join(name);
    write_recipe(&dir, recipe)?;
    match expected_batch(std::slice::from_ref(&dir))?.pop() {
        Some(Ok(())) => Ok(dir),
        Some(Err(message)) => Err(OracleError::Environment { message }),
        None => Err(OracleError::Environment {
            message: "expected.py gave no answer".into(),
        }),
    }
}

/// Writes `recipe` as `dir/fixture.json`, creating `dir`: what
/// [`expected_batch`] reads. Errors: [`OracleError::Write`].
pub fn write_recipe(dir: &Path, recipe: &Recipe) -> Result<(), OracleError> {
    let file = dir.join("fixture.json");
    let text = serde_json::to_string_pretty(recipe).map_err(|e| OracleError::Write {
        path: file.clone(),
        message: e.to_string(),
    })?;
    std::fs::create_dir_all(dir)
        .and_then(|()| std::fs::write(&file, &text))
        .map_err(|e| OracleError::Write {
            path: file.clone(),
            message: e.to_string(),
        })
}

/// Has `expected.py --own` write `expected.json` into every one of
/// `dirs` — each a scratch fixture holding a `fixture.json`
/// ([`write_recipe`]) — in one process for all of them, the cache
/// answering every one it holds first. `--own` records each result's
/// [`Own`](crate::fixtures::Own), which the differential bounds its
/// comparison by. Returns one answer per directory in `dirs`' order: `Ok` with
/// its `expected.json` written, or `Err` with why the oracle refused that
/// recipe, its `expected.json` removed. A refusal is not cached (ADR-0024
/// §1); an answer is, keyed by the `fixture.json` bytes. If Open CASCADE
/// kills the process on one recipe, that recipe is refused and the rest
/// run again.
///
/// Errors: [`OracleError::Write`] when an `expected.json` could not be
/// written or removed; [`OracleError::Environment`] when `uv` could not
/// run or the oracle answered for none of the recipes it was given —
/// no environment, never a refusal of all of them.
///
/// ```no_run
/// use arris_debug::fixtures::Recipe;
/// use arris_debug::oracle;
///
/// let recipe: Recipe = serde_json::from_str(
///     r#"{"steps": [{"op": "box", "name": "b", "min": [0, 0, 0], "max": [1, 2, 3]}], "result": "b"}"#,
/// )
/// .unwrap();
/// let dir = oracle::scratch_dir().join("batch-box");
/// oracle::write_recipe(&dir, &recipe).unwrap();
/// assert_eq!(oracle::expected_batch(&[dir]).unwrap(), vec![Ok(())]);
/// ```
pub fn expected_batch(dirs: &[PathBuf]) -> Result<Vec<Result<(), String>>, OracleError> {
    let mut answers: Vec<Option<Result<(), String>>> = vec![None; dirs.len()];
    let mut slots = Vec::with_capacity(dirs.len());
    let mut pending = Vec::new();
    for (i, dir) in dirs.iter().enumerate() {
        let expected = dir.join("expected.json");
        let spec = std::fs::read(dir.join("fixture.json")).ok();
        let slot = cache::slot("expected.py --own", &[spec.as_deref()], None);
        if let Some(bytes) = slot.as_ref().and_then(|(d, k)| cache::load(d, k)) {
            std::fs::write(&expected, bytes).map_err(|e| OracleError::Write {
                path: expected.clone(),
                message: e.to_string(),
            })?;
            answers[i] = Some(Ok(()));
        } else {
            // A stale answer would read as this run's.
            match std::fs::remove_file(&expected) {
                Ok(()) => {}
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                Err(e) => {
                    return Err(OracleError::Write {
                        path: expected,
                        message: e.to_string(),
                    });
                }
            }
            pending.push(i);
        }
        slots.push(slot);
    }
    while !pending.is_empty() {
        let output = spawn(
            uv("expected.py")
                .arg("--own")
                .args(pending.iter().map(|&i| &dirs[i])),
        )?;
        let stderr = String::from_utf8_lossy(&output.stderr);
        let mut progress = false;
        for &i in &pending {
            let dir = &dirs[i];
            if let Ok(bytes) = std::fs::read(dir.join("expected.json")) {
                keep(slots[i].take(), &bytes);
                answers[i] = Some(Ok(()));
                progress = true;
            } else if let Some(why) = refusal(&stderr, dir) {
                answers[i] = Some(Err(why));
                progress = true;
            }
        }
        let unanswered: Vec<usize> = pending
            .iter()
            .copied()
            .filter(|&i| answers[i].is_none())
            .collect();
        if unanswered.is_empty() {
            break;
        }
        // A process that ran and said nothing of any recipe had no
        // environment to run in; one Open CASCADE killed (a signal, no
        // exit code) died on the first recipe it had not answered.
        match (progress, output.status.code()) {
            (_, None) => {
                answers[unanswered[0]] = Some(Err(format!(
                    "the oracle died building it ({})",
                    output.status
                )));
            }
            (false, Some(_)) => return Err(environment(&output)),
            (true, Some(_)) => {}
        }
        pending = unanswered
            .into_iter()
            .filter(|&i| answers[i].is_none())
            .collect();
    }
    Ok(answers
        .into_iter()
        .map(|a| a.unwrap_or_else(|| Err("no answer".into())))
        .collect())
}

/// The reason `expected.py` printed for refusing `dir`'s recipe: the rest
/// of its `<dir>: ERROR ` line.
fn refusal(stderr: &str, dir: &Path) -> Option<String> {
    let prefix = format!("{}: ERROR ", dir.display());
    stderr
        .lines()
        .find_map(|l| l.strip_prefix(&prefix))
        .map(str::to_owned)
}

/// Writes `step_text` as `target/inspect/<tag>.step` and runs
/// `compare.py` on it against `tests/fixtures/<fixture>` (`variant`, or
/// `default`). Returns the comparison table on a match. Errors:
/// [`OracleError::Mismatch`] with the table; [`OracleError::Environment`]
/// when `uv` or the oracle could not run — never a silent skip.
pub fn compare(
    fixture: &str,
    step_text: &str,
    variant: Option<&str>,
    tag: &str,
) -> Result<String, OracleError> {
    compare_dir(
        &workspace_root().join("tests/fixtures").join(fixture),
        step_text,
        variant,
        tag,
    )
}

/// [`compare`] against a fixture directory anywhere — a scratch copy in
/// a test, a fixture outside the corpus.
///
/// A `MATCH` is kept in [`cache`] under the STEP text, the directory's
/// `fixture.json` and `expected.json` and the variant; a later call with
/// all of them unchanged returns the same table without running the
/// oracle. A mismatch is never kept.
pub fn compare_dir(
    dir: &Path,
    step_text: &str,
    variant: Option<&str>,
    tag: &str,
) -> Result<String, OracleError> {
    let fixture = dir.to_string_lossy().into_owned();
    let scratch = scratch_dir();
    let file = scratch.join(format!("{tag}.step"));
    std::fs::create_dir_all(&scratch)
        .and_then(|()| std::fs::write(&file, step_text))
        .map_err(|e| OracleError::Write {
            path: file.clone(),
            message: e.to_string(),
        })?;
    let read = |name: &str| std::fs::read(dir.join(name)).ok();
    let (spec, expected) = (read("fixture.json"), read("expected.json"));
    let slot = cache::slot(
        "compare.py",
        &[
            Some(step_text.as_bytes()),
            spec.as_deref(),
            expected.as_deref(),
        ],
        Some(variant.unwrap_or("default")),
    );
    if let Some(bytes) = slot.as_ref().and_then(|(d, k)| cache::load(d, k)) {
        return Ok(String::from_utf8_lossy(&bytes).into_owned());
    }
    let mut command = uv("compare.py");
    command.arg(dir).arg(&file);
    if let Some(v) = variant {
        command.args(["--variant", v]);
    }
    let output = spawn(&mut command)?;
    let stdout = String::from_utf8_lossy(&output.stdout).into_owned();
    let stderr = String::from_utf8_lossy(&output.stderr).into_owned();
    match output.status.code() {
        Some(0) if stdout.contains("MATCH") => {
            keep(slot, stdout.as_bytes());
            Ok(stdout)
        }
        Some(1) => Err(OracleError::Mismatch {
            fixture,
            file,
            table: stdout,
        }),
        _ => Err(OracleError::Environment {
            message: format!("{stdout}{stderr}"),
        }),
    }
}

/// Open CASCADE's own STEP of the result of the fixture in `dir` under
/// `variant` (`default` when `None`) — built from the recipe by the
/// oracle and written by `STEPControl_Writer`, or, with `nurbs`, passed
/// through `BRepBuilderAPI_NurbsConvert` first — as text: the file the
/// STEP reader is held to (`tools/oracle/occt_step.py`, ADR-0025). Kept in
/// [`cache`] under the directory's `fixture.json`, the variant and the
/// flag, so an unchanged recipe starts no Python; the file is also left
/// at `target/inspect/<tag>.step` for inspection.
///
/// Errors: [`OracleError::Write`]; [`OracleError::Environment`] when `uv`
/// could not run, the environment is missing, or the oracle could not
/// build the recipe.
///
/// ```no_run
/// use arris_debug::{fixtures, oracle};
///
/// let dir = fixtures::corpus_root().join("primitive/box");
/// let text = oracle::occt_step(&dir, None, false, "occt-box").unwrap();
/// assert!(text.contains("MANIFOLD_SOLID_BREP"));
/// ```
pub fn occt_step(
    dir: &Path,
    variant: Option<&str>,
    nurbs: bool,
    tag: &str,
) -> Result<String, OracleError> {
    let scratch = scratch_dir();
    let file = scratch.join(format!("{tag}.step"));
    let spec = std::fs::read(dir.join("fixture.json")).ok();
    let variant = variant.unwrap_or("default");
    let script = if nurbs {
        "occt_step.py --nurbs"
    } else {
        "occt_step.py"
    };
    let slot = cache::slot(script, &[spec.as_deref()], Some(variant));
    let written = |text: &str| {
        std::fs::create_dir_all(&scratch)
            .and_then(|()| std::fs::write(&file, text))
            .map_err(|e| OracleError::Write {
                path: file.clone(),
                message: e.to_string(),
            })
    };
    if let Some(bytes) = slot.as_ref().and_then(|(d, k)| cache::load(d, k)) {
        let text = String::from_utf8_lossy(&bytes).into_owned();
        written(&text)?;
        return Ok(text);
    }
    std::fs::create_dir_all(&scratch).map_err(|e| OracleError::Write {
        path: scratch.clone(),
        message: e.to_string(),
    })?;
    let mut command = uv("occt_step.py");
    command.arg(dir).arg(&file).args(["--variant", variant]);
    if nurbs {
        command.arg("--nurbs");
    }
    let output = spawn(&mut command)?;
    if !output.status.success() {
        return Err(environment(&output));
    }
    let text = std::fs::read_to_string(&file).map_err(|e| OracleError::Environment {
        message: format!("occt_step.py wrote no file {}: {e}", file.display()),
    })?;
    keep(slot, text.as_bytes());
    Ok(text)
}

/// One placed instance of [`occt_assembly`]'s assembly, as Open CASCADE
/// measures the placed shape.
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct AssemblyInstance {
    /// Its volume.
    pub volume: f64,
    /// Its centroid.
    pub centroid: [f64; 3],
}

/// One occurrence of [`AssemblyOracle::tree`]: what Open CASCADE's XCAF
/// document holds for it, which Arris's product tree of the file is held to
/// (ADR-0033).
#[derive(Debug, Clone, PartialEq, serde::Deserialize)]
pub struct OracleOccurrence {
    /// The product's name.
    pub name: String,
    /// The placement in the parent as a 3×4 row-major matrix, in
    /// millimetres; `None` at the root.
    pub placement: Option<[f64; 12]>,
    /// The colour of the part, as it was set.
    pub colour: Option<[f64; 3]>,
    /// The indices, into [`AssemblyOracle::instances`], of the solids the
    /// occurrence holds itself; empty where the oracle read a file rather
    /// than wrote it ([`occt_read_assembly`]).
    #[serde(default)]
    pub solids: Vec<usize>,
    /// The occurrences placed in it.
    pub children: Vec<OracleOccurrence>,
}

/// The one face [`occt_assembly`]'s script colours apart from its part.
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct OracleFaceColour {
    /// The instance, an index into [`AssemblyOracle::instances`].
    pub instance: usize,
    /// The placed face's centroid.
    pub centroid: [f64; 3],
    /// The face's area.
    pub area: f64,
    /// Its colour, as it was set.
    pub colour: [f64; 3],
}

/// Everything [`occt_assembly_oracle`] measures.
#[derive(Debug, Clone, PartialEq, serde::Deserialize)]
pub struct AssemblyOracle {
    /// Each placed instance's volume and centroid, in placement order.
    pub instances: Vec<AssemblyInstance>,
    /// The expected product tree: `assembly`, holding `part-a` and
    /// `sub-assembly`, which holds `part-b` twice.
    pub tree: OracleOccurrence,
    /// The coloured face.
    pub faces: Vec<OracleFaceColour>,
}

/// An Open CASCADE XCAF assembly of the results of the fixtures in `a`
/// and `b` — `a` placed once, `b` twice, each by a turn and a shift —
/// written by `STEPCAFControl_Writer` with its product structure
/// (`tools/oracle/occt_assembly.py`): the STEP text, and each placed
/// instance's volume and centroid in placement order, which Arris's
/// reader of the file is held to (ADR-0025 §5). Kept in [`cache`] under
/// both `fixture.json`s; the file is also left at
/// `target/inspect/<tag>.step`. [`occt_assembly_oracle`] returns the
/// product tree as well.
///
/// Errors: [`OracleError::Write`]; [`OracleError::Environment`] when `uv`
/// could not run, the environment is missing, or a recipe did not build.
///
/// ```no_run
/// use arris_debug::{fixtures, oracle};
///
/// let root = fixtures::corpus_root();
/// let (text, placed) =
///     oracle::occt_assembly(&root.join("primitive/box"), &root.join("primitive/cylinder"), "asm")
///         .unwrap();
/// assert_eq!(placed.len(), 3);
/// assert!(text.contains("NEXT_ASSEMBLY_USAGE_OCCURRENCE"));
/// ```
pub fn occt_assembly(
    a: &Path,
    b: &Path,
    tag: &str,
) -> Result<(String, Vec<AssemblyInstance>), OracleError> {
    let (text, oracle) = occt_assembly_oracle(a, b, tag)?;
    Ok((text, oracle.instances))
}

/// [`occt_assembly`] with the whole of what the script measures: the
/// instances, the product tree of names, placements and colours, and the
/// coloured face (ADR-0033).
///
/// Errors: as [`occt_assembly`].
///
/// ```no_run
/// use arris_debug::{fixtures, oracle};
///
/// let root = fixtures::corpus_root();
/// let (_, o) = oracle::occt_assembly_oracle(
///     &root.join("primitive/box"),
///     &root.join("primitive/cylinder"),
///     "asm",
/// )
/// .unwrap();
/// assert_eq!(o.tree.name, "assembly");
/// assert_eq!(o.tree.children.len(), 2);
/// ```
pub fn occt_assembly_oracle(
    a: &Path,
    b: &Path,
    tag: &str,
) -> Result<(String, AssemblyOracle), OracleError> {
    #[derive(serde::Serialize, serde::Deserialize)]
    struct Kept {
        step: String,
        measured: String,
    }
    let parse = |measured: &str| {
        serde_json::from_str::<AssemblyOracle>(measured).map_err(|e| OracleError::Environment {
            message: format!("occt_assembly.py's output did not parse as JSON: {e}"),
        })
    };
    let scratch = scratch_dir();
    let file = scratch.join(format!("{tag}.step"));
    let specs = [a, b].map(|d| std::fs::read(d.join("fixture.json")).ok());
    let slot = cache::slot(
        "occt_assembly.py",
        &[specs[0].as_deref(), specs[1].as_deref()],
        None,
    );
    std::fs::create_dir_all(&scratch).map_err(|e| OracleError::Write {
        path: scratch.clone(),
        message: e.to_string(),
    })?;
    if let Some(bytes) = slot.as_ref().and_then(|(d, k)| cache::load(d, k)) {
        if let Ok(kept) = serde_json::from_slice::<Kept>(&bytes) {
            if let Ok(oracle) = parse(&kept.measured) {
                std::fs::write(&file, &kept.step).map_err(|e| OracleError::Write {
                    path: file.clone(),
                    message: e.to_string(),
                })?;
                return Ok((kept.step, oracle));
            }
        }
    }
    let output = spawn(uv("occt_assembly.py").arg(a).arg(b).arg(&file))?;
    if !output.status.success() {
        return Err(environment(&output));
    }
    let measured = String::from_utf8_lossy(&output.stdout).into_owned();
    let oracle = parse(&measured)?;
    let step = std::fs::read_to_string(&file).map_err(|e| OracleError::Environment {
        message: format!("occt_assembly.py wrote no file {}: {e}", file.display()),
    })?;
    let kept = Kept {
        step: step.clone(),
        measured,
    };
    if let Ok(bytes) = serde_json::to_vec(&kept) {
        keep(slot, &bytes);
    }
    Ok((step, oracle))
}

/// What Open CASCADE's XCAF reader makes of a STEP file
/// ([`occt_read_assembly`]).
#[derive(Debug, Clone, PartialEq, serde::Deserialize)]
pub struct ReadAssembly {
    /// The free shapes of the document, as occurrences: names, placements in
    /// their parents, part colours, children; `solids` is empty.
    pub roots: Vec<OracleOccurrence>,
    /// The volume and centroid of every leaf part at the placement its path
    /// composes, depth first.
    pub instances: Vec<AssemblyInstance>,
}

/// Open CASCADE's `STEPCAFControl_Reader` reading of `step_text`, names and
/// colours on (`tools/oracle/occt_read_assembly.py`): the assembly structure
/// an independent reader finds in what Arris's writer wrote (ADR-0033).
/// Kept in [`cache`] under the text.
///
/// Errors: [`OracleError::Write`]; [`OracleError::Environment`] when `uv`
/// could not run, the environment is missing, or the file does not read.
///
/// ```no_run
/// use arris_debug::oracle;
///
/// let read = oracle::occt_read_assembly("ISO-10303-21;\n…", "asm-read").unwrap();
/// assert!(!read.roots.is_empty());
/// ```
pub fn occt_read_assembly(step_text: &str, tag: &str) -> Result<ReadAssembly, OracleError> {
    let scratch = scratch_dir();
    let file = scratch.join(format!("{tag}.step"));
    std::fs::create_dir_all(&scratch).map_err(|e| OracleError::Write {
        path: scratch.clone(),
        message: e.to_string(),
    })?;
    std::fs::write(&file, step_text).map_err(|e| OracleError::Write {
        path: file.clone(),
        message: e.to_string(),
    })?;
    let slot = cache::slot("occt_read_assembly.py", &[Some(step_text.as_bytes())], None);
    let parse = |bytes: &[u8]| {
        serde_json::from_slice::<ReadAssembly>(bytes).map_err(|e| OracleError::Environment {
            message: format!("occt_read_assembly.py's output did not parse as JSON: {e}"),
        })
    };
    if let Some(bytes) = slot.as_ref().and_then(|(d, k)| cache::load(d, k)) {
        if let Ok(read) = parse(&bytes) {
            return Ok(read);
        }
    }
    let output = spawn(uv("occt_read_assembly.py").arg(&file))?;
    if !output.status.success() {
        return Err(environment(&output));
    }
    let read = parse(&output.stdout)?;
    keep(slot, &output.stdout);
    Ok(read)
}

/// Open CASCADE's verdict on one edge filleted alone (`occt_fillet_edges.py`).
#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize)]
pub struct EdgeVerdict {
    /// `builds` (done, and the result passes `BRepCheck_Analyzer`),
    /// `invalid` (done, the result fails it), `refuses`, or `no-edge`
    /// where the point is not within the probe of exactly one edge.
    pub verdict: String,
    /// Open CASCADE's reason, where it refuses or no edge was found.
    #[serde(default)]
    pub why: String,
}

/// Each edge of solid `#solid_id` of the STEP file `file`, named by a
/// point on it in the model's frame, filleted alone at `radius` in Open
/// CASCADE, in order: what a census holds a refusal of Arris's against.
/// A point is an edge within `probe` of it and no other. Cached under the
/// file's bytes and the request (ADR-0024). Errors:
/// [`OracleError::Write`]; [`OracleError::Environment`] when `uv` could not
/// run or the file or solid cannot be read.
pub fn fillet_edges(
    file: &Path,
    solid_id: u64,
    radius: f64,
    probe: f64,
    points: &[[f64; 3]],
) -> Result<Vec<EdgeVerdict>, OracleError> {
    let bytes = std::fs::read(file).map_err(|e| OracleError::Write {
        path: file.to_path_buf(),
        message: e.to_string(),
    })?;
    let request = format!("{solid_id} {radius:e} {probe:e} {points:?}");
    let slot = cache::slot(
        "occt_fillet_edges.py",
        &[Some(&bytes), Some(request.as_bytes())],
        None,
    );
    let parse = |bytes: &[u8]| {
        serde_json::from_slice::<Vec<EdgeVerdict>>(bytes).map_err(|e| OracleError::Environment {
            message: format!("occt_fillet_edges.py's output did not parse as JSON: {e}"),
        })
    };
    if let Some(bytes) = slot.as_ref().and_then(|(d, k)| cache::load(d, k)) {
        if let Ok(v) = parse(&bytes) {
            return Ok(v);
        }
    }
    let scratch = scratch_dir();
    let list = scratch.join(format!("fillet-edges-{}.json", std::process::id()));
    std::fs::create_dir_all(&scratch)
        .and_then(|()| std::fs::write(&list, serde_json::to_vec(points).unwrap_or_default()))
        .map_err(|e| OracleError::Write {
            path: list.clone(),
            message: e.to_string(),
        })?;
    let output = spawn(
        uv("occt_fillet_edges.py")
            .arg(file)
            .arg(solid_id.to_string())
            .arg(format!("{radius:e}"))
            .arg(format!("{probe:e}"))
            .arg(&list),
    );
    let _ = std::fs::remove_file(&list);
    let output = output?;
    if !output.status.success() {
        return Err(environment(&output));
    }
    let verdicts = parse(&output.stdout)?;
    keep(slot, &output.stdout);
    Ok(verdicts)
}

/// Open CASCADE's `RWStl` reading of an STL file: how many facets it saw,
/// their total area and their signed volume by the divergence theorem —
/// the same formula `arris_mesh::TriMesh::signed_volume` and `area` use —
/// so a test holds its own mesh's numbers to an independent reader of the
/// bytes `arris_io::stl` wrote rather than to itself. `tools/oracle/mesh.py`
/// is the script; unlike [`compare`], nothing here knows a fixture's
/// expected numbers, so there is no [`OracleError::Mismatch`] — a test
/// compares the fields itself.
#[derive(Debug, Clone, Copy, PartialEq, serde::Deserialize)]
pub struct StlReading {
    /// How many facets `RWStl` read.
    pub triangles: usize,
    /// The sum of the facets' areas.
    pub area: f64,
    /// `Σ a · (b × c) / 6` over the facets in file order.
    pub volume: f64,
}

/// Writes `stl` (ASCII text or binary bytes) to
/// `target/inspect/<tag>.stl` and has `tools/oracle/mesh.py` read it back
/// through Open CASCADE's `RWStl`. Errors: [`OracleError::Write`];
/// [`OracleError::Environment`] when `uv` could not run, the environment
/// is missing, or the file did not parse as STL.
pub fn compare_stl(stl: &[u8], tag: &str) -> Result<StlReading, OracleError> {
    let scratch = scratch_dir();
    let file = scratch.join(format!("{tag}.stl"));
    std::fs::create_dir_all(&scratch)
        .and_then(|()| std::fs::write(&file, stl))
        .map_err(|e| OracleError::Write {
            path: file.clone(),
            message: e.to_string(),
        })?;
    let slot = cache::slot("mesh.py", &[Some(stl)], None);
    if let Some(bytes) = slot.as_ref().and_then(|(d, k)| cache::load(d, k)) {
        if let Ok(reading) = serde_json::from_slice(&bytes) {
            return Ok(reading);
        }
    }
    let output = spawn(uv("mesh.py").arg(&file))?;
    if !output.status.success() {
        return Err(environment(&output));
    }
    let reading = serde_json::from_slice(&output.stdout).map_err(|e| OracleError::Environment {
        message: format!("mesh.py's output did not parse as JSON: {e}"),
    })?;
    keep(slot, &output.stdout);
    Ok(reading)
}
