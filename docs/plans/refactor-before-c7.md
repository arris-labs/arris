# Plan: refactor-before-c7

- Started: 2026-10-05
- Milestone: C7, before its first feature plan (docs/ROADMAP.md §C7)
- Idea: docs/ideas/refactor-before-c7.md (absorbed; option B, every decision answered as recommended)
- Idea (verbatim from the human): "We just finished C6 and before starting C7 I want you to review whole codebase and /idea refactoring - we added features without any refactoring for quite long time already and now its perfect time to do it."

## Goal

Prepare the code C7 will build on, with no behaviour change:
- **Blend.** `arris-ops`'s blend is a module tree split by phase, where
  no function runs past 300 lines and none needs
  `too_many_arguments`. Its body view (effective face orientations, edge
  uses and vertex edges scoped to one body, outward normals, edge
  convexity) is a crate-private ops module that shell and offset can
  share.
- **Boolean.** The pave model is split into files by phase.
- **Periodic parameters.** They are wrapped by one toolkit in
  `arris-math`, which takes the period from the curve or surface and
  never assumes `TAU`.
- **Dependencies.** Every crate names the crates it uses directly; no
  `arris_a::arris_b::…` chain or chain re-export is left.
- **`Reason`.** It is grouped by operation, its refusal names stable.
- **Corpus tests.** They come from one compact table, and a lint fails
  when a fixture directory has no test.

Behaviour is held fixed throughout:
- every blessed `dump.txt`, `fixture.json`, `cancel_counts.txt` and
  `tolerance_band.txt` is byte-identical to `7bcfd33`;
- entity ids, meter polls and refusal strings are unchanged.

## Non-goals

- **The boolean's two-operand shape** (`[_; 2]`, `side: usize`): the
  multi-tool plan decides the N-ary form (idea decision 3). Pave moves
  between files here, and not one type changes.
- **Code C7 does not open:**
  - the STEP reader (`topology::build`);
  - the geom tracers (`trace`, `trace_torus`, `torus_walk`);
  - `sweep.rs`'s twin `revolve`/`extrude`;
  - `topo/builder.rs`, `check/check.rs`, `mesh/cdt.rs`,
    `geom/pcurve.rs`;
  - `BuildError`, `GeomError::Degenerate { reason: String }`;
  - the `arris-debug` crate split.

  Each keeps its backlog line.
- **Any fix.** A bug found while moving code becomes a regression fixture
  and a backlog line, never a change in the same commit.
- **The public body view.** It stays crate-private in `arris-ops`;
  promoting it to `arris-topo` is a later decision.

## Design deltas

- **`arris-ops` `Reason`** (public, breaking): grouped by the operation
  that raises it. Proposed shape:
  - **`Reason::Input(InputReason)`**: `NonFinite`, `NotPositive`,
    `ZeroThickness`, `NonManifold`, `NotSolid`, the variants more than one
    operation raises.
  - **`Reason::Sweep(SweepReason)`**: `ProfileCrossesAxis`,
    `AxisNotInProfilePlane`, `AngleAboveTurn`, `SpindleTorus`,
    `EllipticRevolve`, `DirectionNotNormal`.
  - **`Reason::Boolean(BooleanReason)`**: `Empty`, `TangentContact`,
    `BesideSingularity`.
  - **`Reason::Blend(BlendReason)`**: `NoEdges`, `RepeatedEdge`,
    `EdgeNotInBody`, `BlendTooLarge`, `TangentChain`, `VertexBlend`.
  - **`Reason::Query(QueryReason)`**: `NotProjectable`, `DegenerateEdge`,
    `ProjectionCollapses`, `NotPlanar`, `OutOfDomain`, `Singular`.

  C7's shell, offset and split reasons later add their own groups. The
  rules for the regrouping:
  - `Display` text is unchanged, so the Python binding's `reason: str`
    and every message stay as they are.
  - A new `Reason::name(&self) -> &'static str` returns the leaf variant's
    name. It is what `differential::refusal`, `census` and `histogram`
    key on, so `Degenerate(TangentChain)` in `tests/fixtures/real/*.json`
    stays byte-identical. Today they parse it out of `Debug`, which the
    nesting would change.
  - Named in the step's commit body and under `CHANGELOG.md`
    `### Breaking`.
- **Chain re-exports removed** (public, breaking):
  - `arris_topo::{arris_geom, arris_math}`, `arris_check::arris_topo`,
    `arris_io::{arris_check, arris_mesh}`, `arris_ops::arris_check`;
  - each crate declares the layer-legal dependencies it uses (ADR-0013's
    order is unchanged; only reach changes);
  - the `arris` facade's `arris::{math, geom, topo, …}` is untouched and
    remains the consumer's path;
  - named in the commit body and under `### Breaking`.
- **`arris-math`**: a public periodic-parameter toolkit beside
  `wrap_angle` and `period_end` (wrap into `[lo, lo + p)`, a difference
  into `(−p/2, p/2]`, the whole-period shift nearest a target, into a
  range when possible). Additive; rustdoc and an example on each.
- **`arris-ops` internals** (no public change):
  - `blend.rs` becomes `blend/` (mod, view-independent phases: chain,
    stripe, ends, miter, junction, corner, ring, build, beside the existing
    `traced` and `mixed`);
  - a crate-private `body_view` module (`BodyView`) used by the blend;
  - `boolean/pave.rs` becomes `boolean/pave/` by phase (hits, sections,
    coincident, the `Build` core).
- **`crates/arris/tests/corpus.rs`**: a `corpus_tests!` table that keeps
  every test name, `#[ignore]` reason and doc comment. Names are kept
  because `tools/gate.sh` and the nextest profiles select by name prefix.
  A lint checks the table against `fixtures::corpus()`.
- No ADR: every item is structure under ADR-0004, 0007 and 0013. The
  `Reason` shape is a design delta, recorded in ARCHITECTURE §Errors.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

Every step ends with `git diff 7bcfd33 -- tests/fixtures
crates/arris-ops/tests/cancel_counts.txt
crates/arris-ops/tests/tolerance_band.txt` empty. A move step also keeps
`cargo test -p arris --test corpus` green with the same set of passing
and ignored tests.

- [x] Step 1 **[1]** — **The corpus table and its coverage lint first,
  so every later step is held by every fixture.**
  - `corpus_tests!` in `crates/arris/tests/corpus.rs` takes
    `$(#[$meta])* name => "area/dir"`, a `[variant]` form for
    `run_variant` and a `part` form for `run_part`.
  - Every one of the 359 tests is rewritten into it with its name, doc
    comment and `#[ignore = …]` unchanged.
  - The macro also emits the table's directory list. A new
    `every_fixture_directory_has_a_corpus_test` in `corpus_lint.rs` holds
    it against `fixtures::corpus()` (geometry fixtures excepted, as the
    runner does), both ways.
  - Any fixture the lint finds unrun gets its test here. If that test
    fails, it is a finding: an `#[ignore]` with the reason and a backlog
    line, not a fix.
  - Test: `cargo nextest list -p arris --test corpus` gives the same
    names before and after (diff the two listings in the commit body's
    check).
- [x] Step 2 **[1]** — **Direct dependencies, chains gone.**
  - Each of `arris-topo`, `-check`, `-mesh`, `-ops`, `-io`, `-debug`,
    `-py` and the crates' tests declares and imports the crates it names.
    This replaces about 600 `arris_x::arris_y::` paths.
  - Remove the chain `pub use`s listed under the design deltas, and
    `arris-mesh`'s `Aabb`/`Interval` re-export if nothing outside needs
    it.
  - Update the doctests that spell a chain, and the crate table in
    ARCHITECTURE §Crates.
  - `cargo tree` CI layer checks unchanged.
  - CHANGELOG `### Breaking`: "a lower crate no longer re-exports the
    crates beneath it: depend on each directly, or use `arris::{math,
    geom, topo, …}`".
  - Test: the full workspace builds, and
    `rg 'arris_[a-z]+::arris_' crates` is empty.
- [x] Step 3 **[2]** — **One periodic-parameter toolkit.** Add it to
  `arris-math` with doctests at the boundaries:
  - a value exactly at `lo + p`;
  - `−1e-300`, mirroring `wrap_angle`'s own;
  - a period that is not `TAU`.

  Replace with it:
  - `boolean/pave.rs::wrap_on`, `intersect_curves::wrap_angle_if_periodic`,
    `pcurve::wrap_half`, `torus_walk::wrap_pi`;
  - the STEP reader's `whole_periods`;
  - blend's `into_range` and `placed`, which now takes the surface's
    period instead of `TAU`.

  Each replacement must be bit-identical on the values its caller
  passes. Where an old helper's fast path, such as `wrap_on`'s "already
  in range, unchanged", differs from plain `rem_euclid`, the toolkit
  keeps it, with a unit test pinning the difference. The other
  `rem_euclid` sites move only where they are one of these shapes.

  Test: the toolkit's unit and property tests (seeded), and the step-wide
  byte-identity check.
- [x] Step 4 **[1]** — **`blend.rs` into `blend/`, a pure move.** Split
  by the phases in its outline:
  - `mod.rs` (`fillet`, `chamfer`, `blend`, `Kind`, `Blend`);
  - `view.rs`;
  - `chain.rs` (chain, tangent and cusp vertices);
  - `stripe.rs`;
  - `ends.rs` (face, fan, corner and cut ends);
  - `miter.rs`, `junction.rs`, `corner.rs`, `ring.rs`;
  - `build.rs` (insertions, face edits, `build`);
  - `traced.rs` and `mixed.rs` as they are.

  Visibility is widened to `pub(super)` only. No body or signature
  changes, so `git diff -M --stat` shows moves, and `git log --follow`
  works on the largest file. Test: the gate's blend area and
  `blend_prop` at the fast case count.
- [x] Step 5 **[2]** — **`BodyView` lifted to `arris-ops/src/body_view.rs`
  (crate-private).**
  - Moves: `View::of`, `outward`, the edge uses and vertex edges,
    `convex_edge` (as `BodyView::convex`), `tangent_at` and
    `faces_tolerance`.
  - The blend calls it, with every iteration order and every
    floating-point expression unchanged.
  - New unit tests on it alone:
    - a box's edge is convex, an L-prism's inner edge concave;
    - a void shell's face is reversed;
    - edge uses come in the body's order;
    - a seam's two uses are in one face.
- [x] Step 6 **[2]** — **`blend::build` in phases over a context.**
  - A `BlendCtx { m, view, kind, tol, samples, meter }` replaces the
    threaded arguments.
  - `build` (956 lines) splits into: classify the chained edges
    (rings, arcs, open); find junctions, miters and corners per vertex;
    make the stripes and ends; cut the faces; assemble through
    `rebuild::rewrite`.
  - `Meter` polls stay at the same points in the same order. This is
    what `cancel_counts.txt` holds.
  - Test: `cancel` and `cancel_prop`, the blend corpus, `blend_prop` at
    the fast case count, `provenance_prop`.
- [x] Step 7 **[2]** — **`ring`, `miter`, `corner` and `face_end` in
  phases over the same context.** No function in `blend/` is over 300
  lines, and none of `blend/` keeps
  `#[allow(clippy::too_many_arguments)]`. Same tests as step 6.
  - Landed as `Env { m, view, kind, tol, samples }` (`Copy`, read-only) beside
    the meter, split off `BlendCtx::split`: a phase that only reads takes
    `(env, meter, …)`, so ring, miter, corner, face_end and the helpers
    under them no longer thread six arguments.
- [x] Step 8 **[1]** — **`boolean/pave.rs` into `boolean/pave/`, a pure
  move.**
  - `mod.rs` (`Build` and its core);
  - `hits.rs` (edge-on-face hits, vertex builds);
  - `sections.rs` (section curves, merge);
  - `coincident.rs`;
  - shared helpers in `mod.rs`.

  No type changes, including `[_; 2]` and `side`. Test: the gate's
  boolean area, `boolean_prop` at the fast case count, `tolerance_band`.
- [ ] Step 9 **[2]** — **`Reason` grouped by operation**, as the design
  deltas give it.
  - Add `Reason::name`.
  - `Display` is unchanged, held by a test that formats every leaf, so
    every message stays exactly as it was.
  - `differential::refusal`, `census` and `histogram` key on `name()`.
  - Every `Reason::X` in ops, debug, the binding and the tests becomes
    its grouped path.
  - `arris-py`: its exhaustive match and the `_arris.pyi` docstrings
    compile and read the same (`reason: str` unchanged), and
    `kernel_error`'s `every_variant` covers the groups.
  - ARCHITECTURE §Errors' `Degenerate` row lists the groups.
  - CHANGELOG `### Breaking`: "`Reason` is grouped by operation:
    `Reason::BlendTooLarge` is `Reason::Blend(BlendReason::TooLarge)`, …,
    with the full old → new table".

  Test: the semver gate passes with the Breaking bullet, pytest and
  `mypy.stubtest` pass, the real-part fixtures are byte-identical, and
  the histogram's output is unchanged.

## Acceptance

- **Full gate.** `ARRIS_GATE=full` green at 256 cases, `real_*`
  included, and CI's 1000-case run green on the pushed tip.
- **Behaviour unchanged.** `git diff 7bcfd33 --stat -- tests/fixtures
  crates/arris-ops/tests/cancel_counts.txt
  crates/arris-ops/tests/tolerance_band.txt` is empty: every dump, every
  entity id, every meter poll count and every refusal name is unchanged.
- **Same corpus.** `cargo nextest list -p arris --test corpus` names the
  same tests, ignored the same, plus any step 1 added, and the new
  coverage lint passes.
- **Structure measured.**
  - `rg 'arris_[a-z]+::arris_' crates` is empty.
  - No `too_many_arguments` in `crates/arris-ops/src/blend`.
  - No function in `blend/` over 300 lines, and no file in
    `crates/arris-ops/src` over 2000.
  - None of the seven local wrap helpers remains.
- **Python.** `pytest`, the docstring examples and `mypy.stubtest` pass.

## Docs to update on completion

- `docs/ARCHITECTURE.md`:
  - §Crates and the layer rule: the crate table loses its "re-exports …"
    clauses; each crate names its direct dependencies.
  - §Operations: the blend's layout and `body_view` as the seam shell and
    offset read a body through, beside `rebuild::rewrite`.
  - §Errors: the `Reason` groups (written by step 9), and `Reason::name`
    as the stable refusal name the histograms key on.
- `docs/DATA-MODEL.md` §Geometry (the periodic parameters paragraph): the
  `arris-math` toolkit is how a parameter is wrapped, with the period
  taken from the entity.
- `tests/fixtures/README.md`: `corpus.rs` is a table under
  `corpus_tests!`, and a fixture directory without a row fails the lint.
  Refusals are named in the grouped form.
- `CHANGELOG.md` `Unreleased`: the two Breaking bullets (written by steps
  2 and 9) and an additive bullet for the periodic toolkit.
- `docs/BACKLOG.md`: drop what this plan closed (done at planning), and
  add any finding steps 1–9 produced.
- `docs/ROADMAP.md` §C7: one line under the status that the preparatory
  refactor landed.
- `AGENTS.md` current state: one clause on the C7 line.

## Open questions

- `⚠ OPEN:` the exact `Reason` grouping, such as whether `NotSolid`
  belongs under `Input` or `Query`, and the leaf names inside a group
  (`BlendReason::TooLarge` vs `BlendTooLarge`). Who decides: the agent,
  by step 9, keeping `name()` returning today's names whatever the leaf
  is called.
- `⚠ OPEN:` whether `arris-mesh` keeps re-exporting `Aabb`/`Interval` (its
  public types use them in signatures, so a consumer of `arris-mesh`
  alone would need `arris-math`). Who decides: the agent, by step 2;
  default keep, since it is a convenience re-export of a type, not a
  crate chain.
