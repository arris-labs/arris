# Plan: python-binding

- Started: 2026-10-02
- Milestone: beside the cycles — the first-party binding (docs/ROADMAP.md §Beside the cycles; ADR-0020 §2 and its amendment)
- Idea: docs/ideas/python-binding.md (absorbed)
- Idea (verbatim from the human): "for the first-party binding (the Python / agent-scripting consumer)"
- Decision (the human, 2026-10-02): all six preferred answers of the idea's "Decision for the human", below.

## Goal

An agent or a plugin can `pip install arris` and, in a script, build a
body from primitives and profiles, fillet and cut it, ask what it
measures and what the checker says, tessellate it, and write and read
STEP, STL, OBJ and body bytes — without compiling a crate. The binding is
`crates/arris-py`: a thin 1:1 layer over the `arris` facade, one layer
above it, `#![forbid(unsafe_code)]`, never published to crates.io and
published to PyPI as `arris` in lockstep from the same `v*` tag. Every
operation returns the body and its provenance; every error is a typed
exception naming the entities it concerns; a handle carries its model, so
an id from one model cannot reach another. Python tests run in CI against
a `maturin develop` build and reproduce the oracle's numbers for a set of
corpus fixtures.

## Non-goals

- An ergonomic modelling layer (workplanes, selectors, fluent chains). It
  belongs in another repository on top of this one (idea, option C).
- A recipe interpreter or JSON surface as the agent's way in (option B).
- Rendering and PNG output in the wheel: an agent gets the mesh and
  renders it with its own tools; `arris-debug` stays dev tooling (SEED §4).
- `rust-numpy`, the buffer protocol, or any hand-written `unsafe`.
- The first PyPI upload: the human reserves the name and gives the
  go-ahead (step 12 stops there).
- wasm for the binding: pyo3 does not build for `wasm32-unknown-unknown`,
  so the crate is empty there (step 2).
- A Python wrapper around cancellation beyond a token, a step budget and
  Ctrl-C (step 6); `region2` and multi-tool booleans have no API to bind
  yet (BACKLOG).
- Wheels for every platform at release: CI builds the Linux wheel; the
  platform matrix is a release-time decision (open question 2).

## Design deltas

**Decisions taken (the idea, all six answered "preferred", 2026-10-02):**

1. A thin 1:1 binding over the facade in `crates/arris-py`, above `arris`.
2. `#![forbid(unsafe_code)]` kept: pyo3 0.26 with `abi3-py310` compiles
   under it for `#[pyclass]`, `#[pymethods]`, `#[pymodule]`,
   `create_exception!` and `py.detach`; the first step re-runs the check
   (the idea's "what would change my mind").
3. PyPI as `arris`, in lockstep from the same `v*` tag; the crate itself
   `publish = false` on crates.io; the first upload waits for the human.
4. Python tests (pytest over a `maturin develop` build in a `uv` venv) in
   CI only, never in the pre-commit hook; the hook's Rust checks still
   compile the crate, so a Rust API break still fails locally.
5. The mesh crosses as `bytes` with an optional `numpy` shim; no PNG.
6. One ADR, at step 1: ADR-0034, recording the layer position, the forbid
   kept with the evidence, the wasm gate, lockstep PyPI, and handles
   carrying their model. It amends ADR-0020 §2's "exception a binding
   needs".

**Crates and layers.**

- New crate `crates/arris-py`, layer 8 in `tools/check-layers.sh` and in
  `docs/ARCHITECTURE.md` §Crates and the layer rule: depends on `arris`
  only; nothing depends on it.
- `publish = false` (as `arris-debug`), so `cargo publish --workspace`,
  `cargo package --workspace` and cargo-semver-checks skip it.
- pyo3 is a `cfg(not(target_arch = "wasm32"))` dependency and the crate is
  empty on wasm, so CI's `cargo build --workspace --target
  wasm32-unknown-unknown` is unchanged.
- `crate-type = ["cdylib", "rlib"]`: the `rlib` keeps `cargo test` and the
  hook's clippy and nextest runs working.

**The Python surface** (the shape each step fills in, in `arris`'s own
vocabulary):

- `Model`: a class owning an `arris::topo::Model` behind a lock, so a
  long operation can release the GIL (`py.detach`). Not the facade's
  `Model` re-exported: a handle carries its model (decision 6), and a
  foreign handle raises `ForeignHandleError` before it reaches the kernel,
  closing `docs/ARCHITECTURE.md` §The model's "ids have no model
  identity" for Python callers with no kernel change.
- Handles: frozen, hashable classes `Body`, `Shell`, `Face`, `Edge`,
  `Vertex`, each a model reference plus the slot and generation.
- Every operation returns `(Body, Provenance)`; `Provenance` reads as
  `generated`, `modified` and `deleted` lists of `(origin, outputs)`.
- Exceptions: a common `ArrisError`, one subclass per `OpError`,
  `GeomError`, `TopoError`, `StepError`, `BodyError`, `MeshError` and
  `FitError` variant that a binding can raise, the entities and reason a
  variant names as attributes. A kernel change that adds a variant breaks
  the binding's exhaustive match at compile time — the kernel rule's
  intent (`.agents/rules/kernel.md` §API) — so the same commit updates it.
- `Control` as a `Cancel` token (a flag another thread sets), an optional
  step budget, and Ctrl-C (`KeyboardInterrupt`), every operation taking
  them as keyword arguments; the kernel's `Interrupted` is a subclass of
  `ArrisError`.
- Queries: mass properties, face frames, the checker's `Report` (and
  `Violation`s), adjacency walks (faces of a body, edges of a face, …).
- Tessellation and io: `tessellate` returns a `Mesh` of `bytes`
  (little-endian `f64` positions and `u32` indices, counts alongside) with
  a pure-Python `Mesh.to_numpy()`; `step.write`, `step.read` (every solid
  its own result or refusal), `stl`, `obj`, `native` and `body` bytes and
  JSON.
- Hand-written `.pyi` stubs and docstrings with runnable examples.

**Release and CI.**

- `pyproject.toml` (maturin, `abi3-py310`), the package version read from
  `[workspace.package].version` (`0.5.0-dev` is PEP 440 `0.5.0.dev0`; a
  release is `0.5.0`).
- `ci.yml` gains a `python` job (maturin develop, pytest, stub check) and
  a Linux wheel build; `release.yml` gains a `pypi` job behind its own
  `pypi` environment reviewer, using PyPI trusted publishing, that
  refuses a pre-release as the crates job does. The first upload is the
  human's act (step 12).
- `.agents/rules/git.md` §Tags (the PyPI half of the release),
  `/release` and `/close-cycle` skills (the wheel is part of the
  packaging proof), `.agents/rules/kernel.md` §API (a break also updates
  the binding in the same commit) each gain the sentence for it.

**A public change.** None to an existing type or signature of a published
crate. A `CHANGELOG.md` bullet at release: the Python package exists.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — Re-run the idea's pyo3 check and write ADR-0034. In
  a scratch crate: pyo3 0.26, `abi3-py310`, `#![forbid(unsafe_code)]`,
  one `#[pyclass]` with `&mut self`, `create_exception!` with a subclass
  chain, `py.detach`, and `cargo build --target wasm32-unknown-unknown`
  with the dependency gated; record the versions and the result in the
  ADR. The ADR holds the six decisions, the layer position, the wasm gate,
  lockstep PyPI, handles carrying their model, and the amendment of
  ADR-0020 §2. If the forbid does not hold, the ADR records the exception
  confined to this crate and the plan stops for the human. Result:
  `docs/adr/0034-the-python-binding.md`, and `docs/adr/README.md`'s index
  line.
- [x] Step 2 **[1]** — The crate skeleton. `crates/arris-py`
  (`publish = false`, `cdylib` + `rlib`, pyo3 gated off wasm), a module
  that exposes `arris.__version__` and `ArrisError`; `pyproject.toml`; the
  layer table gains layer 8 in `tools/check-layers.sh` (self-test still
  detects its forbidden edge) and `docs/ARCHITECTURE.md` §Crates; the
  workspace's `wasm`, `no-default-features` and `layers` CI jobs pass on
  the new crate. Test: `cargo test -p arris-py`, `tools/check-layers.sh`,
  the wasm build, and a pytest that imports the built module.
- [x] Step 3 **[2]** — `Model` and handles carrying their model. The
  lock, the frozen hashable handle classes, `ForeignHandleError` raised
  before the kernel, handle equality and hashing by `(model, slot,
  generation)`, a stale handle surfacing the kernel's `NotFound` as
  `StaleHandleError`. Test (Rust and pytest): two models, the same slot in
  each, a handle from one refused by the other — the case that today
  resolves to a different real entity. *Found at step 3:* no operation
  exists in Python yet to mint a handle, so the foreign, stale and
  retained-handle cases run in Rust (`model::tests`, over the kernel's own
  `primitive_box`) and the pytest holds what Python can reach; step 6's
  pytest adds the same three cases through `Model.primitive_box`.
- [x] Step 4 **[1]** — Errors: the exception hierarchy. One subclass per
  variant of every error enum the binding can raise, entities and reasons
  as attributes, built by an exhaustive `match` per enum (no wildcard
  arm). Test: a Rust test that each variant maps to its own class, and a
  pytest that catches a boolean's `Unsupported` and reads its operand
  ids. *Found at step 4:* the maps are pure (`kernel_error::Mapped`), so
  the Rust test covers every variant and its attributes without an
  interpreter; the pytest checks the hierarchy, and the raised-with-
  attributes case waits for step 6, where a Python operation can fail
  (add it to step 6's pytest). `OpError::InvalidInput`, `MeshError::
  InvalidInput` and the `Rejected` variants carry the checker's report as
  text until step 8 has a `Report` class; step 8 makes it one. Step 6
  gives `Interrupted` the builtin `InterruptedError` as a second base.
- [x] Step 5 **[1]** — `Provenance` as Python values. `generated`,
  `modified`, `deleted` as lists of `(origin, outputs)`, the roles as
  readable objects (`Role::Consumer`'s namespace and key included), and
  `Provenance.then`. Test: the bolt-pattern fixture's eight walls under
  one origin, read from Python. *Found at step 5:* no operation exists in
  Python yet to produce a record, so the bolt pattern's eight walls run in
  Rust (`provenance::tests`, over the kernel's own primitives and cuts,
  read through the same `recorded` view the getters use) and the pytest
  holds `Role`; step 6's pytest reads the same record through
  `Model.primitive_*` and `cut` (add it there). `Role(kind, part, *fields)`
  is also the way to name an origin for `generated_from`; a record of one
  model `then`-ed with another's raises `ForeignHandleError`.
- [x] Step 6 **[2]** — Operations. `primitive_box`, `primitive_cylinder`,
  `transform`, `mirror`, `cut`, `fuse`, `common`, `fillet`, `chamfer`,
  `extrude`, `revolve`, each returning `(Body, Provenance)`, with the
  `Cancel` token, a step budget and Ctrl-C as keyword arguments, and the
  GIL released around the kernel call. Test: pytest of each against the
  closed form; a handle from one model refused by another and a freed
  one stale (step 3's pytest half) and a boolean's `Unsupported` raised with
  its operands as attributes (step 4's), through the operations; an operation cancelled from another thread returns
  `InterruptedError`'s subclass and leaves the model as it was (ids
  included); a budget stops at the same step run twice. *Found at step 6:*
  `extrude` and `revolve` take a `Profile`, which step 7 builds, so they
  are bound there with the Pappus test; and no query exists yet to read a
  volume, so the booleans' closed forms (volume, area) are step 8's pytest
  and this step's check the structure through provenance (the bolt
  pattern's eight walls under `Role("cylinder", "Wall")`, step 5's pytest
  half) and the error cases: a disjoint `common` raises `OpDegenerateError`
  with its operands as `entities`; a boolean's `OpUnsupportedError` with
  `a`/`b` is Rust-tested (step 4) and has no reachable case from primitives
  alone, so step 7's `revolve` is where to look for one. Operation
  arguments the kernel's types refuse (a zero axis, a non-finite corner)
  are `OpDegenerateError` naming the argument, raised by the binding.
- [ ] Step 7 **[2]** — Profiles. `Profile` of lines, arcs and elliptic
  arcs with holes, built from Python values and checked on `extrude` and
  `revolve`. Test: Pappus's theorems on a profile built in Python, to the
  fixtures' tolerance.
- [ ] Step 8 **[1]** — Queries and walks. Mass properties, face frames,
  `check(body, level)` returning the `Report` and its `Violation`s,
  adjacency walks. Test: the oracle's volume, area, centroid and inertia
  for three corpus fixtures read from their `expected.json`, to the
  fixture's tolerance.
- [ ] Step 9 **[2]** — Tessellation and mesh bytes. `tessellate(body,
  chord, angle)` returns a `Mesh` of bytes plus counts, `Mesh.to_numpy()`
  in a pure-Python shim that imports `numpy` only when called. Test: the
  mesh of a cylinder is closed, its signed volume within the corpus's
  `mesh_volume_rel` of the oracle's, and `to_numpy()` agrees with the
  bytes when numpy is installed and raises a clear `ImportError` when it
  is not.
- [ ] Step 10 **[1]** — io. `step.write`, `step.read` (every solid its
  own result or `Refusal`, the product tree beside it), `stl`, `obj`,
  `native` and `body` bytes and JSON. Test: write → read round trip of a
  filleted body keeps volume to the entities' tolerance; body bytes
  written in one `Model` are imported into a second and keep their record
  through `Imported.translated` (the A10 interop the idea names).
- [ ] Step 11 **[1]** — Stubs, docstrings, and the stub check. A `.pyi`
  per module, a docstring with a runnable example on every public item,
  and a CI check that the stubs match the built module (`mypy.stubtest`)
  and that every example runs (pytest `--doctest-modules`). Test: a stub
  drifting from the module, or an example that fails, fails the job.
- [ ] Step 12 **[2]** — CI and release. `ci.yml`'s `python` job (uv,
  maturin develop, pytest, stubtest, the Linux wheel as an artifact) and
  `release.yml`'s `pypi` job behind a `pypi` environment reviewer with
  trusted publishing, refusing a pre-release version; the version read
  from the workspace. Test: the CI job passes on `main`; the `pypi` job's
  build half runs without publishing (`maturin build`, `twine check`). **The
  step stops at the `pypi` environment and the trusted-publisher
  registration, which the human does** (the idea's decision 3); the first
  upload is not part of this plan.
- [ ] Step 13 **[1]** — The rules and the docs of the workflow. The
  sentence in `.agents/rules/git.md` §Tags, `/release` and
  `/close-cycle` skills (the wheel packages and the stubs check join the
  proof; `pypi` joins the human's tag steps), `.agents/rules/kernel.md`
  §API (a break updates the binding in the same commit),
  `tools/gate.sh` (a path under `crates/arris-py` runs its own tests and
  the crate's rdeps; nothing else) with its test `tools/gate-test.sh`.
  Test: `tools/gate-test.sh` and `tools/check-layers.sh --self-test`.

## Acceptance

- The Python suite (`pytest` over `maturin develop`, run in CI's `python`
  job) reproduces the oracle's volume, area and centroid to the fixtures'
  tolerance for the corpus fixtures it scripts — through hole, blind
  hole, the 8-hole bolt pattern, a filleted box, an extruded and a
  revolved profile — each built by a Python script, not a recipe, and
  holds the bolt pattern's provenance (8 of 8 walls under one origin).
- A body written by `body.write` in one `Model` and read in a second keeps
  its counts, measures and record; a handle from one model is refused by
  the other with `ForeignHandleError`.
- An operation cancelled from another thread, and one stopped by a
  budget, leave the model as it was.
- Every error variant has a class (the exhaustive `match` compiles) and
  `mypy.stubtest` finds no drift between the stubs and the module.
- `cargo test --workspace`, `cargo clippy --workspace -D warnings`,
  `tools/check-layers.sh`, `cargo build --workspace --target
  wasm32-unknown-unknown`, `cargo package --workspace` (which skips the
  crate) and the semver gate all pass; the `python` CI job is green on
  `main`; the Linux wheel builds and `twine check` accepts it.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Crates and the layer rule — the new layer-8
  crate, its dependency on `arris` alone, the wasm gate; §Formats and
  tools — the wheel, the Python job, the stubs; §The model — handles in
  Python carry their model.
- `docs/ROADMAP.md` §Beside the cycles — the first-party binding goes
  from "can start now" to what it is: scope, the PyPI lockstep, what it
  waits on.
- `docs/adr/0020-…` — the amendment is ADR-0034's; ADR-0020 itself is
  not edited. `docs/adr/README.md` — ADR-0034's index line.
- `docs/BACKLOG.md` — the binding's own backlog: platform wheels beyond
  Linux, a pyo3 upgrade rule, wheels for the product tree, whatever a
  step finds and defers.
- `README.md` — a short "From Python" paragraph and an install line; no
  version stated (`.agents/rules/git.md` §The version).
- `CHANGELOG.md` `## Unreleased` — the Python package exists (a bullet
  for a consumer who has read only the README), written by `/retire-plan`.
- `.agents/rules/git.md`, `.agents/rules/kernel.md` and the `/release`
  and `/close-cycle` skills — written by step 13.
- `AGENTS.md` current state — the binding landed beside C6.

## Open questions

- **Settled at step 6: Ctrl-C inside a released-GIL call.** The poll
  re-attaches every 256 polls and calls `check_signals`; a `KeyboardInterrupt`
  surfaces and the model is as it was. Measured on eight bolt-hole cuts
  (26 steps each, 20 s of CPU in total): signals every poll, every 256 and
  never differ by less than the run-to-run noise (0.71–0.80 s for forty
  builds), so the interval is not tuned further. The thin-elliptic
  section the question named needs `Profile` and waits for step 7; the
  interval is a constant (`control::SIGNAL_EVERY`) if it shows a cost.
- - **⚠ OPEN: platforms of the wheel (human, by step 12).** The plan builds
  and tests the Linux x86_64 wheel in CI. manylinux/macOS/Windows wheels
  for release are a matrix the release job can add; which ones PyPI
  carries at 0.5.0 is the human's call, with the first upload.
- **⚠ OPEN: the PyPI name and trusted publisher (human, by step 12).**
  `arris`, `arris-kernel` and `pyarris` were free on 2026-10-02 (404). The
  human reserves `arris` and registers the trusted publisher (owner
  `arris-labs`, repository `arris`, workflow `release.yml`, environment
  `pypi`); the crates.io first-release precedent (a token, then trusted
  publishing) applies here as a pending publisher.
- **⚠ OPEN: how the version reads (agent decides at step 12).** maturin
  reads `[workspace.package].version`; `0.5.0-dev` becomes `0.5.0.dev0`,
  and the release job must still refuse it. The step checks the mapping
  with `maturin build` rather than assuming it.
- **Settled at step 3: a handle's lifetime across `Model.retain`.**
  `retain` never renumbers (ADR-0010), so a handle to a retained entity
  stays valid and one to a dropped entity is stale; `StaleHandleError` is
  the kernel's `NotFound` and the binding adds no generation check of its
  own (`model::tests::a_freed_entity_is_stale_and_a_kept_one_stays_valid`).
