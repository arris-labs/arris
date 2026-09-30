# ADR-0024 — The measuring harness: a cached oracle, a differential over recipes, three property tiers, an in-house timer and fuzzing outside the workspace

- Status: accepted (2026-09-24)
- Plan: `measuring-harness` step 1
- Follows: ADR-0020 §2 (a measuring harness beside the cycles, its first
  follow-up), ADR-0015 and ADR-0023 (what the oracle is trusted with)

## Context

ADR-0020 put a measuring harness beside the cycles and named its parts:
the oracle cached so an unchanged recipe is not re-run, random recipes
through both kernels, a property tier above CI's count run nightly,
benchmarks over the corpus, and fuzz targets seeded from it. It left the
shape of each to the plan, and one question to the human: the property
case count the pre-commit hook runs. The charter delegates that question,
so it is decided here with the rest.

The facts each part starts from:

- **The oracle runs every time.** `arris_debug::oracle` starts
  `uv run … compare.py` for every corpus fixture and variant, `mesh.py`
  for every STL test and `expected.py` for every scratch fixture, whether
  or not anything it reads has changed. Each start pays for Python and
  Open CASCADE's import before any work.
- **The recipe grammar is shared.** `Step` in `arris-debug` and the `op`
  dispatch in `tools/oracle/oracle/recipe.py` carry the same eleven
  operations, and `recipe_hash` names a recipe on both sides.
- **The hook's count let a fault through.** The pre-commit hook runs every
  property at 256 cases on the fixed seed, CI at 1000. The seam
  parametrisation fault's counterexample lived past case 32 of a shard:
  the hook passed it, CI caught it, and a 0.1.0 publish was already half
  out.
- **Nothing times anything.** `tools/test-timings.sh` times test binaries,
  not operations.

## Decision

### 1. The oracle cache

- **The key** is sha256 over, in order: the script's name; the bytes of
  every file the script reads (the STEP or STL text, `fixture.json`,
  `expected.json`); the variant; and a digest of the oracle itself — every
  `tools/oracle/**/*.py`, `pyproject.toml` and `uv.lock`, by path and
  bytes, sorted by path. A change to any input, or to any line of the
  oracle, misses. The oracle digest is computed once per process.
- **Only a settled answer is cached**: a `compare.py` table that says
  `MATCH`, a scratch fixture's `expected.json`, an STL reading. A
  mismatch or an environment error always runs again, so a cache can
  never hide a failure and a fixed environment is seen on the next run.
- **Where it lives:** `target/oracle-cache/`, one file per key. It is
  gitignored with `target/` and `cargo clean` removes it.
- **`ARRIS_ORACLE_CACHE=off`** bypasses it for reading and writing.
  `ci.yml` and the nightly set it, so CI always runs the oracle and never
  trusts a cache: the cache is a local speed-up only, and a green CI run
  says the oracle agreed on that commit.

### 2. The differential

Random recipes, drawn by a seeded `proptest` strategy over `Step`, are
built by both kernels and each outcome is sorted into a class:

| Class | Meaning | Fails the run |
|---|---|---|
| `Agree` | both build, every corpus stage passes | no |
| `BothRefuse` | Open CASCADE builds no solid, Arris refuses | no |
| `ArrisRefuses(Reason)` | Open CASCADE builds, Arris returns a typed refusal | no — counted per `Reason` |
| `OracleRefuses` | Arris builds, Open CASCADE does not | no |
| `Disagree(stage)` | both build, a corpus stage differs | **yes** |
| `CheckerViolation` | Arris returns `Ok` with a shape the checker rejects | **yes** |
| `Panic` | Arris panics | **yes** |

A typed refusal is the kernel's rule working (`.agents/rules/kernel.md`
§Correctness), so it is counted, not failed, and its count per `Reason` is
the same table the reader cycle's refusal histogram reads (ADR-0020 §2).
The last three break a rule of the kernel and fail the run, shrunk to a
recipe ready to commit under `tests/fixtures/regression/`. A kernel cause
the draw keeps hitting becomes a *named exclusion* in the generator that
cites that fixture's slug; the corpus lint fails an exclusion whose
fixture is gone or has left `regression/`, so the commit that fixes it
lifts it. `Panic` is caught with `catch_unwind` on the test side; the
kernel never catches its own panics.

### 3. The property tiers

| Tier | Cases | Seed |
|---|---|---|
| pre-commit hook | 256 | the fixed seed |
| CI (`ci.yml`) | 1000 | the fixed seed |
| nightly (`nightly.yml`) | a multiple of CI's, set from measured wall clock to keep each job under the runner's six hours | `sha256` of the date, printed as `ARRIS_PROPTEST_SEED=…` |

The nightly also runs the differential at a large recipe count on the
same seed, the benchmarks and the fuzz targets, with
`ARRIS_ORACLE_CACHE=off` throughout.

**The hook keeps 256.** One commit per plan step is the unit of work
(`.agents/rules/git.md`), and a hook that costs minutes more per commit is
paid on every step of every plan. A higher fixed count also only reaches
further into the *same* draw: the seam fault sat past case 32 of a shard
on the one seed every tier used. Depth comes from new seeds instead — the
nightly draws a different sample each night, so its coverage adds up
across nights where a larger fixed count would keep re-testing one
sample. CI remains the first catch of what the hook misses, and a
release waits on CI's run of the tagged commit (`release.yml`).

### 4. Benchmarks

An in-house timer in `arris_debug::bench`: a warm-up, a fixed iteration
count, the median and the median absolute deviation, a JSON report, and a
comparison that prints each case's ratio against a saved report. No new
dependency: the harness exists to catch a robustness fix that costs 10×,
not a 5% drift, and a statistics crate's precision buys nothing at that
threshold. Time is **never a gate** — not in the hook, not in `ci.yml`.
Shared runners vary too much; the nightly flags ratios and fails nothing
on them.

### 5. Fuzzing

The fuzz targets live in `fuzz/`, a crate **outside the workspace**
(`exclude = ["fuzz"]`), unpublished, on the nightly toolchain with
`libfuzzer-sys` and `arbitrary`. None of those becomes a workspace
dependency, so the stable build, the hook and CI never see them. The
`#![forbid(unsafe_code)]` rule binds the kernel's crates; `fuzz/` is not
one of them, and `libfuzzer-sys`'s entry macro is its business. The
targets decode geometry from bytes and call the kernel's public API only.

### 6. What a nightly failure does

It shows as a red workflow run and nothing else. It opens no issue and
posts nothing: anything that leaves the repository is the human's call.

## Consequences

- A warm corpus run starts no Python, and `oracle::spawns()` makes that a
  number a test asserts on. A cold run — a fresh clone, CI, an oracle
  change — costs what it costs today.
- A stale cache entry cannot exist by construction: every input is in the
  key. A wrong one could only come from an input the key misses, which is
  why the oracle's own sources are in it.
- The hook stays as cheap as it is. A fault deep in one seed's draw still
  reaches `main` and is caught by CI; one outside that draw is caught by
  some night's seed, and reproduces from the printed seed.
- The refusal histogram exists before the reader does, over generated
  recipes, and the reader cycle's corpus adds to the same table.
- The kernel gains no dependency, no public type and no signature: the
  harness is in `arris-debug`, the tests, the benches, `fuzz/` and
  workflows.

## Alternatives considered

- **Raise the hook to 1000.** It would have caught the seam fault, at
  several minutes per commit on every step of every plan, and it still
  tests one sample of one seed. Rejected for the nightly's rotating seed.
- **Key the cache by `recipe_hash` alone**, as ADR-0020 first put it. A
  corpus comparison reads Arris's STEP text, which changes with every
  kernel change while the recipe does not, and the oracle's own scripts
  change its answers too. The key has to cover every input.
- **Trust the cache in CI**, persisted by `actions/cache`. It saves CI's
  oracle time and makes a green run mean "agreed on some earlier commit
  with the same inputs", which is true by construction but no longer
  checked. The oracle is CI's ground truth, so CI runs it.
- **`criterion` or `divan`.** Better statistics and reports, a dependency
  tree the workspace does not otherwise carry, and precision the 10×
  question does not need.
- **Fuzz targets inside the workspace behind a feature.** The nightly
  toolchain and `libfuzzer-sys` would then reach `Cargo.lock` and the
  stable CI build, and the crate would need an exception to the unsafe
  rule inside the workspace.
- **Open an issue on a red nightly.** Useful, and outward-facing; the
  human can add it.

## Amendment (2026-09-24, plan `measuring-harness` step 5b)

The first run at CI's count (1000 recipes, the fixed seed) changed four
things in §2.

- **`Internal` fails the run.** An `OpError::Internal` is a fault the
  kernel caught and typed, not a refusal of the input. It is now a class
  of its own, named by its fault (`Internal(Split)`), and it fails the run
  whatever the oracle did. Counting it beside the typed refusals would
  have let a kernel bug pass as the kernel's rule working.
- **Mass properties are held to what both shapes support.** Almost every
  measure disagreement of the first run was one kernel's fitted section.
  Each kernel vouches for its boundary only to within its own tolerance.
  Open CASCADE's boolean output declared 1e-7 to 4e-3, and Arris's fits
  are made to 1e-7. So the differential holds each drawn result to the
  first-order bound of a boundary moved within `t_arris + t_occ`:
  - `A·t` of volume;
  - `L·t` of area;
  - `A·t·R / V` of centroid;
  - `A·t·R²` of each inertia component;
  - `A·(δ + t)` of the mesh's volume, at mesh chord `δ`.

  `oracle.measure.within_own_tolerance` already holds the oracle's STEP
  round trip to the same bound. The fixture defaults are the floor, and
  the corpus's own fixtures keep them, because their geometry meets 1e-9.
  The evidence is in the plan: two closed forms, Arris rebuilt at 1e-9
  and 1e-10, and the one real defect the comparison showed, which was
  `mass_properties` integrating about the origin and is fixed.
- **Counts are compared net of removable vertices.** A removable vertex
  is one that only splits an edge between the same two faces into two.
  Open CASCADE keeps one where a section crossed an operand's seam, and
  Arris merges it. The two B-reps are of one solid, so a difference in
  such vertices alone is not a disagreement. `expected.py --own` records
  the oracle's count, and the differential counts Arris's the same way.
- **A named exclusion matches a symptom, not a recipe.** No generator
  can say which draws reach a kernel bug. An exclusion is a predicate
  over the failing outcome instead, stating one symptom and nothing
  else: a checker row ("lies outside every outer loop"), an internal
  fault (`Internal(Split)`), or a count off in one direction (fewer
  shells than the oracle). It cites the regression fixtures that pin the
  symptom, and the corpus lint fails once one of them has left
  `regression/`. The CI job runs the differential at 1000 recipes
  (`ARRIS_DIFF_CASES`), and the hook at its default of 32.

The other decisions of §2 are unchanged.

## Amendment (2026-09-25, plan `measuring-harness` retirement)

§6 holds, and says what a red night is not: it opens nothing and posts
nothing. What it is, the first three nights settled. Each drew a new
seed and found two to five things the fixed seeds never reached: bounds
that did not scale with magnitude, a boolean fault, a NaN line of two
planes a hair from parallel, a fit that gives up at its span cap, and
the first nightly fuzz crash. A night green end to end would mean the
harness had stopped finding things, so it is not the proof that the
harness works.

- **The workflow is proven by a run whose every job completes within
  its time limit**, with the property tier, the differential and the
  benchmark green on the run's date seed. The fuzz jobs run their time;
  a crash they find is a finding like any other.
- **A red night is a finding, not a gate.** It is triaged the way any
  finding is (`.agents/rules/kernel.md` §Testing): a fix with a test
  that fails without it, or a shrunk fixture with a named exclusion that
  cites it, plus a backlog line. The seed the job printed reproduces it.
- **CI's fixed seed stays the gate.** A kernel fix made for a night's
  finding is held to CI's own count before it counts as done. In the
  first nights a knot rule green at the hook's 32 recipes turned two
  agreeing recipes of CI's 1000 into faults, and one of them came back
  from a later night's seed as a new finding. The fixed seed told the
  two apart; a new seed could not have.

## Amendment (2026-09-30, plan `lean-gate` step 4)

The hook row of §3's table (256) and the paragraph "The hook keeps 256" are
superseded by ADR-0032: the hook runs a path-scoped `fast` profile at 32
cases, and 256 is run once per plan, at `/retire-plan`. The reasoning about
fixed seeds and nightly depth stands.
