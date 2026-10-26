# Plan: file-concerns

- Started: 2026-10-26
- Start commit: d7a0fd10 (the acceptance's `<start>`)
- Milestone: the refactor side slot beside C11 (cycle C11, docs/ROADMAP.md)
- Idea: docs/ideas/refactor-after-c10.md (absorbed: its last part,
  `file-concerns`; decision 2026-10-23, option B, the side slot, Q5 delete
  `nurbs_census`; no ADR). The `arris-debug` split it left for its own idea
  is a backlog line.

## Goal
Each of the five files the idea names holds one concern per file:
`py/src/model.rs` is a `model/` module whose `#[pymethods]` blocks are filed
by ops, queries, topology and io, none past 800 lines; `check/check.rs`,
`mesh/cdt.rs` and `geom/pcurve.rs` are modules split by concern, no file past
600 lines; the checker's `Fast` tests are one test binary filed by entity
row, not by the plan step that wrote them. The corpus reports write their
tables through one `Table` writer, and `nurbs_census` is gone. The Python
API is unchanged (stubtest, pytest and the docstrings pass as they are);
every blessed `dump.txt`, `fixture.json`, `cancel_counts.txt`,
`tolerance_band.txt`, id and refusal string is byte-identical to the start,
and so is every report the committed modes of `real_parts` print.

## Non-goals
- No behaviour change; a defect found while moving code becomes a regression
  fixture and a backlog line, never a fix in the same commit.
- No Rust public type or signature changes (every moved item keeps its path
  through re-exports; `cdt`, `pcurve`'s functions, `check` and `Report` as
  now); no `CHANGELOG.md`, since the wheel's API does not move.
- The idea's *Leave these alone*: the tracers' algorithms, the blend phase
  tree, `ops/error.rs`, `part21::lex`, `arris-math`'s structure, the
  kernel-seams seams. No algorithm inside `cdt` or `pcurve` is rewritten.
- The `arris-debug` split (testkit, corpus crate, renderer): its own idea
  and ADR after a measured relink time; a backlog line now.
- `census.rs` (2095 lines) and `histogram.rs` (1359) beyond their table
  writing: their causes and cycle maps are one concern each, read and kept.

## Design deltas
- **`arris-py`: `model.rs` → `model.rs` + `model/`** (the crate's
  `bernstein_n.rs` + `bernstein_n/` style). `model.rs` keeps `Shared`,
  `Model`, its `run`/`run_with`/`operate`/`refuse`, the shared argument
  checks, `new`, `contains`, the tolerances, `retain`, `__repr__` and the
  handle tests. One `#[pymethods] impl Model` per file: `model/ops.rs`
  (primitives, transform, mirror, the booleans, split, section, the sweeps,
  the features), `model/query.rs` (mass and surface properties, point,
  curve, surface, bounds, frames, ray, distance, interference, check,
  tessellate), `model/topology.rs` (shells to `edges_at`, `body_kind`),
  `model/io.rs` (STEP, body JSON and native, `write_body_with`/
  `read_body_with`). This turns on pyo3's `multiple-pymethods` feature,
  which registers the blocks through the `inventory` crate (see Open
  questions). Docstrings stay on their methods, so the doctests and
  `_arris.pyi` are untouched.
- **`arris-check`: `check.rs` + `check/`**: `check.rs` keeps `check`,
  `Checker`, `push`, `coedges`; `check/references.rs` (M1, M2: `references`,
  `indices`), `check/finite.rs` (M3: the `*_finite`/`*_non_finite` helpers,
  `finiteness`), `check/vertex.rs` (V rows), `check/edge.rs` (E rows,
  `e1_range` … `seam_fault`). Rows run in the same order, so the report's
  order holds.
- **`arris-check` tests: `fast_part1.rs` + `fast_part2.rs` →
  `tests/fast/`**, one binary (`main.rs`) with `support.rs` (the rebuild,
  `lines`/`assert_lines`, the plate builders `full.rs` also names),
  `model.rs` (M rows, the clean samples, the determinism test),
  `vertex.rs`, `edge.rs`, `loops.rs`, `face.rs`, `shell.rs` (S rows, B3,
  the Euler line, the `Full` rerun). Test names lose `_partN`, nothing
  else; one binary fewer to link.
- **`arris-mesh`: `cdt.rs` + `cdt/`**: `cdt.rs` keeps the public types,
  `triangulate`, `triangulate_metered`, the input ordering and the tests of
  the public seam; `cdt/mesh.rs` (`Mesh`, `Tri`, `Location`, `Fail`: locate,
  insert, legalize, flip, fan), `cdt/constraint.rs` (`insert_constraint`,
  `fill`), `cdt/winding.rs` (`windings`, the classification).
- **`arris-geom`: `pcurve.rs` + `pcurve/`**: `pcurve.rs` keeps `pcurve_on`
  (the dispatch), `check_on`, `pcurve_ending_on` and the shared helpers;
  `pcurve/plane.rs` (in-plane and projected curves, `project_to_plane`,
  `projected_conic`, `principal_axes`), `pcurve/revolution.rs` (cylinder,
  elliptic cylinder, cone, sphere, torus, parallels and meridians),
  `pcurve/fitted.rs` (`SingularEnd`, `singular_ends`, `fitted_on`). Each
  file's tests move with it.
- **`arris-debug`: `report::Table`** (crate-private): columns with an
  alignment, rows of cells, `Display` writing the header, the `|---:|`
  rule and the rows exactly as the writers do now. `census::markdown`,
  `run_over_markdown`, `vertex_blend_markdown`, `gaps::markdown`,
  `Histogram::markdown` and `script_markdown` write their tables through
  it; prose and lists around the tables stay theirs.
- **`nurbs_census` deleted** with `real_parts --nurbs` and
  `--nurbs-fillet` and their lines in the example's doc (Q5): NURBS blocks
  0 of 38 parts.
- No new term; no ADR (structure under existing ADRs; the dependency is
  recorded in the `arris-py` row of `docs/ARCHITECTURE.md`).

## Steps
Complexity grades: **[1]** routine — the design says exactly what to write
and the tests are mechanical; **[2]** careful — a geometric or numeric case
to get right within a given design; **[3]** unproven — an algorithm whose
robustness or bound has to be established here.

- [x] Step 1 **[2]** — `multiple-pymethods` on; `model/ops.rs` split out
  first, alone. `tools/py-test.sh` green (stubtest, docstrings, pytest);
  the wheel builds locally with `maturin`. Riskiest: if the feature does
  not link under `abi3`, stop and take the fallback (Open questions).
- [ ] Step 2 **[1]** — `model/query.rs`, `model/topology.rs`, `model/io.rs`;
  no `model/` file past 800 lines; `tools/py-test.sh` green.
- [ ] Step 3 **[1]** — Record the committed reports: `real_parts`
  `--gaps-committed`, `--run-over-committed`, `--vertex-blend-committed`,
  the census over the committed tier and the script-corpus histogram,
  written to `target/file-concerns/before/` (the commands in the body).
  Delete `nurbs_census` and its two modes.
- [ ] Step 4 **[1]** — `report::Table` and the six writers on it; the
  step-3 reports regenerated byte-identical (`diff -r`).
- [ ] Step 5 **[1]** — `check/` per row family; the check crate's tests
  and every corpus area's checker output unchanged.
- [ ] Step 6 **[1]** — `tests/fast/` by entity row; the test count equals
  `fast_part1` + `fast_part2` (`cargo nextest list`, in the body).
- [ ] Step 7 **[2]** — `cdt/`; the mesh corpus and `mesh/` dumps unchanged
  (the `Fail` paths keep their variant and naming order).
- [ ] Step 8 **[2]** — `pcurve/`; the geom tests, the boolean and blend
  corpora's dumps unchanged; `refactor-review`'s `file_sizes` shows none of
  the five files past its bound.

## Acceptance
- `git diff <start> -- tests/fixtures` is empty.
- `tools/py-test.sh` green; `_arris.pyi` unchanged since `<start>`.
- The step-3 reports byte-identical after step 4.
- `cargo nextest run -p arris-check -p arris-mesh -p arris-geom -p
  arris-ops -p arris-debug` green; the full gate (`tools/full-gate.sh`)
  green at `/retire-plan`. The wheels job on Linux, macOS and Windows is
  read in CI by the human before the next push that matters.
- The file bounds of the Goal, measured by
  `.agents/skills/refactor-review/scripts/`.

## Docs to update on completion
- `docs/ARCHITECTURE.md` — the `arris-py` row (`multiple-pymethods`,
  `inventory`); the `arris-debug` row loses nothing it names, but drop any
  `nurbs_census` mention found; `cdt`'s mention keeps its path.
- `crates/arris-check/tests/full.rs:54` — the comment naming `fast_part2.rs`.
- `docs/ROADMAP.md` §Beside the cycles — the refactor sequence done.
- `AGENTS.md` and `docs/ROADMAP.md` — the `plans/file-concerns` citation
  that replaced the deleted idea's path.
- `AGENTS.md` current state — the refactor sequence closed; C11's first
  plan next.
- `docs/BACKLOG.md` — the `arris-debug` split line (added with this plan);
  any defect found while moving.
- `GLOSSARY.md` — no new term expected.

## Open questions
- Settled (agent): **`multiple-pymethods`, not free functions.** Of the
  block's ~1870 lines, ~940 are docstrings and the bodies are already
  5–10 lines over `operate`; free functions behind thin methods would
  leave a ~1500-line block and add a hop per method. The feature's cost is
  the `inventory` dependency (link-time registration, supported on the
  three wheel platforms; the crate is empty on wasm). Fallback if step 1
  fails to link or a wheel platform refuses it in CI: the methods stay in
  one block in `model.rs`, their helpers move to `model/`, and the Goal's
  800-line bound for `model/` is dropped with the reason in the body.
- Settled (agent): `nurbs_census` goes whole, `--nurbs-fillet` with it —
  the fillet stages it took apart are the ones the NURBS cycle blocks,
  which are none.
