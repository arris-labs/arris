# Plan: classify-points

- Started: 2026-11-16
- Milestone: C12's first plan (docs/ROADMAP.md §C12, the training harness's refusals)
- Idea: docs/ideas/classify-points-batch.md (absorbed): option A, a batch
  classifier in the binding. Settled with C12's re-scope (ADR-0020, amendment
  of 2026-11-16). It also covers `interference` of a cone against a cone for
  the harness (the backlog line).

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal

A Python caller classifies a batch of points against a body exactly:
`Model.classify_points(body, points)` returns inside, outside, on or
undecided for each point, in the order given. Each answer is the one
`arris_check::classify::classify_point` gives for that point, and the
answers are the same on every machine. Cancel and budget work as they do
for `interference`. The training harness then scores the pairs
`interference` refuses (8.6% of its pairs) by exact classification instead
of a mesh's winding number. The Rust crate offers the same batch as
`arris::ops::query::classify_points`, and the corpus's probe stage runs
through it. 10,648 points (the harness's per-pair sample) against one real
fixture's body take at most 0.15 s in release.

## Non-goals

- The entity a point is `On`. The batch returns a code; the Rust
  `Classification::On(Shape)` keeps the entity for Rust callers. A
  single-point Python query that returns the entity is a backlog line if a
  caller asks for one.
- Threads. The batch runs in order on one thread, so the answers do not
  depend on the thread count. The boolean's sequential classifier is a
  separate backlog line.
- Classifying against a sheet body or a wire. These are refused, as
  `classify_point` refuses them today.
- `interference` of a cone against a cone, and the rest of C12 (its other
  ideas, each its own plan).

## Design deltas

- **`arris-ops` (public, additive):**
  `query::classify_points(m: &Model, body: Body, points: &[Point3], control: &Control) -> Result<Vec<Option<Classification>>, OpError>`,
  re-exported from `arris::ops::query`.
  - `None` means no ray decided the point (`ClassifyError::Undecided`). It
    is a per-point answer, so one grazing point never fails the batch.
  - A non-finite point fails the call with
    `Reason::Input(InputReason::NonFinite { what: "the point" })`, as
    `distance` does.
  - An unresolved id or a ray's geometry error fails the call as
    `OpError::Internal(Fault::Classify(_))`, as `distance` does. Its
    `fault_classify` moves to `query/mod.rs` and is shared.
  - One meter tick per point, so `budget` counts points and `Interrupted`
    reports how many points were done.
- **`arris-check` (only if step 2 needs it):** `Classifier::classify` gets
  an early exit for a point outside the body's box grown by `reach`: it is
  `Outside` without the `on` pass. This is the same answer, since nothing
  can be `On` there, and the property in step 1 holds it to that.
- **`arris-py` (public, additive):**
  `Model.classify_points(body, points, *, cancel=None, budget=None) -> bytes`.
  - `points` is a `Sequence[_Point3]`, or `bytes` / `bytearray` /
    `memoryview` of little-endian `f64` triples. That is `Mesh.positions`'
    layout, and numpy's `a.astype("<f8").tobytes()`. abi3-py310 has no
    buffer protocol for us, so this is how numpy data comes in without the
    package needing numpy.
  - The result is one `int8` per point: `0` outside, `1` inside, `2` on,
    `3` undecided. `numpy.frombuffer(out, dtype="i1")` reads it.
  - `ValueError` for a byte length that is not a multiple of 24.
  - `OpInvalidInputError` for a non-finite point, `Interrupted` at the
    budget or cancel. Both classes already exist.
  - The stub goes in `_arris.pyi`, with a docstring example.
- **`arris-debug`:** `corpus::stages::probe_stage` classifies a fixture's
  probes with one `classify_points` call, so every corpus fixture with
  probes tests the batch against Open CASCADE.
- No ADR: it is a query over the existing classifier. The code's meaning is
  recorded in the rustdoc and the docstring.

## Steps

Complexity grades; a step's grade is its tier, the model that executes it
(`.agents/tiers.md`), and `**[1, tier 3]**` overrides the tier: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[1]** — `query::classify_points` in `arris-ops`, as in the
  deltas, with `fault_classify` shared. Rustdoc with an example. A property
  test (`prop_shards!`) over random poses of the sample bodies: random
  points (inside, outside, and on faces, edges and vertices) give the same
  answer one by one as `classify_point`, and in the same order. Also
  tested: `budget = k` stops after `k` points with `Interrupted`; a NaN
  point is `NonFinite`.
- [ ] Step 2 **[2]** — The speed bar. A test in `arris-ops/tests`, ignored
  unless `--release`: 10,648 points on a 22³ grid over the box of a
  real part of about 100 faces or more (one `real_*` corpus part, named in
  the test) take ≤ 0.15 s, printed. Over the bar, add the
  box early exit to `Classifier::classify` (deltas), and then whatever
  `samply` shows next, until under the bar. The answers stay equal to step
  1's property. Record the time before and after in the commit body.
- [ ] Step 3 **[1]** — `probe_stage` goes through `classify_points` (one
  call per fixture). `None` becomes a probe failure that says "undecided".
  The corpus's probe stage passes unchanged over every area.
- [ ] Step 4 **[1]** — `Model.classify_points` in `arris-py`, with its stub
  and docstring example. Tests in `tests/test_queries.py`:
  - a box and a cylinder, giving the codes `0/1/2`;
  - sequence and bytes inputs giving equal bytes;
  - `Mesh.positions` of the body itself, all `2`;
  - a numpy round trip, skipped without numpy;
  - `ValueError` for 23 bytes, `OpInvalidInputError` for NaN,
    `Interrupted` at `budget=1` with three points.

  `tools/py-test.sh tests/test_queries.py tests/test_stubs.py` passes. The
  same commit writes the `CHANGELOG.md` `Unreleased` bullet (Rust and
  Python): the codes, `3` undecided, and the bytes layout, for a reader of
  the README only.

## Acceptance

- Step 1's property at the full gate's 256 cases (and CI's 1000) holds the
  batch equal to `classify_point` point by point.
- The corpus probe stage, now through the batch, matches Open CASCADE's
  probe classes on every fixture that carries probes
  (`tools/full-gate.sh`).
- Step 2's timing test prints ≤ 0.15 s for 10,648 points in release.
- `tools/py-test.sh` is green: pytest, docstrings, `mypy.stubtest`,
  `mypy --strict`.
- No panic. Every crate stays `forbid(unsafe_code)`.

## Docs to update on completion

- `docs/ARCHITECTURE.md`, the `arris-ops` row and §queries: `classify_points`
  beside `distance` and `interference`, one batch over one `Classifier`,
  sequential.
- `README.md` Queries row: point classification.
- `docs/COMPARISON.md`'s queries row: point classification
  (`BRepClass3d_SolidClassifier` on Open CASCADE's side).
- `docs/ROADMAP.md` §C12: the first in-list item marked done.
- `AGENTS.md` current state: Next becomes C12's second plan
  (`flat-edge-read-as-tangent-contact`).

## Open questions

- ⚠ OPEN: the speed bar (0.15 s for 10,648 points, half the harness's mesh
  route per pair). The agent decides at step 2. If a real fixture's body
  cannot reach it without a new acceleration structure, the bar is relaxed
  to what was measured and a backlog line records why.
