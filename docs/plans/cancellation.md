# Plan: cancellation

- Started: 2026-09-29
- Milestone: C5 — the consumer's API, its third line (docs/ROADMAP.md §C5, "Cancellation (A3)")
- Idea: the ask is `docs/ideas/plugin-cad-consumer-asks.md` A3 (the idea
  stays until C5's last plan absorbs it); no idea of its own — the
  roadmap fixed the shape, the ADR in step 1 fixes the rest
- Idea (verbatim from the human): "next point from plugin-cad-consumer-asks"

## Goal

A consumer can stop an operation that is running, on native and on
wasm, and get the model back as it was. Every operation on a model, and
every long query beside one, takes a `&Control`: a poll the consumer
supplies (a closure it answers from an atomic flag, a
`SharedArrayBuffer`, whatever its platform has) and an optional
**budget** of steps. The kernel counts a step at each loop boundary of
the work that can grow past its input's size — tracing, fitting,
subdivision, the boolean's passes, the blend's corners, triangulation,
the reader's solids — and at each one asks the budget and then the poll.
A stop returns `Interrupted` in the operation's own error type, and the
transaction every operation already runs in rolls the model back, ids
included. A budget is deterministic: the same input and budget stop at
the same step on every platform and with `parallel` on or off, so a
consumer can cap an evaluation reproducibly and the corpus can interrupt
at every point. `Control::NONE` (no poll, no budget) is what a caller
that does not care passes. The six-minute thin-elliptic section of the
backlog stops within a bounded number of steps of a poll turning true.

## Non-goals

- No clock, no thread, no timer, no callback into the consumer beyond
  the poll (ARCHITECTURE §Threading and wasm). A wall-clock timeout is
  the consumer's: its poll can read its own clock.
- No progress reporting (a fraction done, a stage name). The step count
  is returned on `Interrupted`, nothing more; progress is a backlog line
  if a consumer asks.
- No resumption: an interrupted operation is gone; the consumer calls it
  again.
- Step counts are stable within one release, not across releases: an
  algorithm change moves them. A budget is a cap a consumer tunes, not
  a contract.
- The checker (`arris_check::check`) is not instrumented: it runs after
  the operation's work, is linear in the body at `Fast`, and an
  interrupt that lands in it waits for it. Revisited if `Full` on a real
  part is measured to be slow.
- Writers (`step::write`, `body::write`, the mesh formats) and
  constant-work calls (`face_frame`, `frame_at`, the projections) take
  no `Control`: their work is linear in what they are handed.
- Mirror (A11) and STEP product structure (A4) are C5's next plans.

## Design deltas

- **ADR-0030, cancellation** (step 1). Decisions, recorded with the
  alternatives weighed:
  1. **Passed, not stored.** A trailing `&Control` argument, not a field
     of `Model` (interior mutability in a representation crate, and a
     cloned model would share its interrupt), not a thread-local (none
     exist in a kernel crate), not a `…_with` twin of every operation
     (doubles the surface a consumer's facade wraps). Pre-1.0 the break
     is allowed; it is one `Breaking` bullet per crate.
  2. **A poll, not an `AtomicBool`.** `&(dyn Fn() -> bool + Sync)`: on
     native it reads the consumer's atomic; on wasm without shared
     memory an atomic set on another worker is never seen, and a poll can
     read whatever the consumer has (a `SharedArrayBuffer` through
     `js-sys`, a message counter). `Sync` because the `parallel` passes
     poll from several threads.
  3. **A budget in steps, because it is cheap.** A `u64` counter beside
     the poll; exhausting it is `Interrupted { by: Stop::Budget }`, a
     poll is `Stop::Poll`. It is what makes interruption testable at
     every point and deterministic, which the roadmap's acceptance asks.
  4. **Determinism under `parallel`.** A parallel pass hands each item a
     meter capped at the budget left when the pass starts, and after the
     pass sums the items' steps in sequential order and stops at the
     first item whose running sum crosses the cap. The sequential build
     runs the same rule, so both stop on the same item with the same
     count; the parallel build only wastes the work of items past it. A
     poll is by nature not deterministic, and only its *result*
     (the model as it was) is promised.
  5. **What a step is.** One iteration of a loop whose trip count is not
     bounded by the input's entity count: a tracer's march, a fit's
     refinement, a subdivision's split, a Newton polish, a boolean pair,
     a face split, a blend corner, a CDT insertion, a reader's entity.
     The ADR lists the sites so a new algorithm knows it owes one.
  6. **Where the types live.** `Control`, `Meter`, `Stop` and the
     `Interrupted` value in `arris-math`, so `arris-geom`'s tracer ticks
     without naming an upper crate (ADR-0013).
- **New public API, `arris-math`:**
  - `pub struct Control<'a> { poll: Option<&'a (dyn Fn() -> bool + Sync)>, budget: Option<u64> }`
    with `Control::NONE`, `Control::poll(f)`, `Control::budget(n)`,
    `with_budget`, `with_poll`.
  - `pub struct Meter<'a>` (a `Control` and the steps taken) with
    `Meter::new(&Control)`, `tick(&mut self) -> Result<(), Interrupted>`,
    `steps()`, and the parallel split/join of decision 4.
  - `pub struct Interrupted { pub by: Stop, pub steps: u64 }`,
    `pub enum Stop { Poll, Budget }`.
- **Changed signatures (breaking, each named in its commit and under
  `CHANGELOG.md` `### Breaking`):**
  - `arris-geom`: `intersect_surfaces`, the curve–surface and curve–curve
    intersections, the NURBS fit and projection entry points take
    `&mut Meter` last; `GeomError::Interrupted(Interrupted)`.
  - `arris-ops`: `cut`, `fuse`, `common`, `interferences`, `fillet`,
    `chamfer`, `extrude`, `revolve`, `transform`, `build`,
    `primitive_box`, `primitive_cylinder` (and every other primitive),
    `mass_properties`, and the rebuild entry points take `&Control`
    last — every operation on a model, one rule for the consumer;
    `OpError::Interrupted(Interrupted)`.
  - `arris-mesh`: `tessellate`, `tessellate_with` take `&Control` last;
    `MeshError::Interrupted(Interrupted)`.
  - `arris-io`: `step::read` and `body::read`/`from_json` take `&Control`
    last; `ReadError::Interrupted(Interrupted)` and
    `BodyError::Interrupted(Interrupted)`. `step::read` runs inside one
    `Model::transaction`, so an interrupt drops the solids already read
    rather than returning a partial `Read` (a refusal is per solid; an
    interrupt is the caller's and is the whole call).
  - `arris` re-exports `Control`, `Stop`, `Interrupted`.
- **`docs/ARCHITECTURE.md`** §Errors and the operation contract (the
  argument and the rollback), §Threading and wasm (the poll, the budget
  under `parallel`).
- **Crate boundary:** unchanged; `arris-math` gains the types.
- **Callers:** `arris-debug` (corpus runner, recipes, battery, bench),
  the fuzz targets and every test pass `Control::NONE`; each step
  updates the callers of what it changes so every commit is green.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[1]** — ADR-0030, cancellation: the six decisions above,
  the alternatives (a model field, a `…_with` twin, an `AtomicBool`,
  a clock), the step-site list; the ADR index updated. Docs only.
- [x] Step 2 **[3]** — the riskiest unknown first: `Control`, `Meter`,
  `Stop`, `Interrupted` in `arris-math` with unit tests of the budget,
  the poll and the split/join rule; `arris-geom`'s intersections, tracer
  (`trace`, `torus_walk`), fit, projection and Bernstein subdivision
  tick; `GeomError::Interrupted`; `arris-ops` passes a
  `Meter::new(&Control::NONE)` internally so its signatures wait for
  step 3. The proof is the backlog's slow case, as a test in
  `crates/arris-geom/tests/`: the torus of major radius 1000 against the elliptic cylinder of
  semi-axes 990 and 0.5, run with a poll that turns true after its
  first call, returns `Interrupted` after at most a stated number of
  steps, and the test measures (in the test, never the kernel) that the
  longest stretch between two ticks on it and on every fuzz
  `slow-unit` input the backlog names is under a stated bound. A
  stretch found unbounded — one fit solve, one resultant — gets a tick
  inside it or is named in the ADR as the latency floor.
- [x] Step 3 **[2]** — the booleans: `cut`, `fuse`, `common`,
  `interferences` take `&Control`; `OpError::Interrupted`; the pair pass
  and the split pass follow decision 4. Tests: on
  `boolean/`'s fixtures, the unbudgeted step count `N` is equal with
  `parallel` on and off; a budget of `k < N` returns `Interrupted {
  steps }` at the same `k` both ways with the model's native bytes equal
  before and after; a budget of `N` gives the unbudgeted dump.
- [x] Step 4 **[2]** — every other operation: sweeps, blends,
  primitives, `transform`, `build`, the rebuild entry points and
  `mass_properties` take `&Control` and tick at their sites; each
  refusing one returns `Interrupted` with the model unchanged, tested on
  one fixture per area (`sweep/`, `blend/`, `build/`, `provenance/`,
  `transform/`) by the same three assertions as step 3.
- [x] Step 5 **[2]** — `arris-mesh`: `tessellate`, `tessellate_with`
  take `&Control`; the edge pass and the CDT insertions tick, the
  parallel face pass follows decision 4; `MeshError::Interrupted`. The
  same count/interrupt/identity tests on a curved fixture both ways.
- [x] Step 6 **[2]** — `arris-io`: `step::read` inside one transaction,
  ticking per entity converted and per solid; `body::read` and
  `from_json`; `ReadError::Interrupted`, `BodyError::Interrupted`. On a
  real-part fixture from the corpus: an interrupt at `k` leaves the
  model's native bytes as before, and the budget of `N` reads the same
  refusal table and dumps as unbudgeted; the `body_read` and STEP fuzz
  targets pass `Control::NONE` and still build.
- [x] Step 7 **[2]** — the roadmap's property: recipes drawn by
  `prop::recipe`, each step's unbudgeted count `N` measured, then every
  drawn `k < N` interrupts with the model's native bytes unchanged and
  `k ≥ N` gives the unbudgeted dump; a poll that answers true on its
  `k`-th call stops where budget `k` stops (sequential build); both
  `parallel` settings; sharded with `prop_shards!` at the configured
  case count. `arris` re-exports the three types, with the operation
  contract's doctest showing a poll over an `AtomicBool` and a budget.

## Acceptance

- `cargo nextest run --workspace` green with `parallel` on and off, the
  corpus included: every fixture still matches its oracle and its dump
  with `Control::NONE` (no fixture expectation changes).
- Step 7's property green at its configured case count: every operation
  interrupted at a random step returns `Interrupted` and leaves the model
  as it was, and a budget at or above the count changes nothing —
  ROADMAP §C5's cancellation clause.
- Step 2's slow-section test green: the six-minute case stops within its
  stated step and latency bound after the poll turns true.
- `cargo build --target wasm32-unknown-unknown` for every crate (CI's
  existing check), with the poll path compiled in.
- `tools/semver-gate.sh` green: every changed signature is under
  `CHANGELOG.md` `### Breaking`.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Errors and the operation contract — the
  trailing `&Control`, `Interrupted` in each error type, the rollback
  covering it, `step::read` as one transaction.
- `docs/ARCHITECTURE.md` §Threading and wasm — the poll as the only
  channel in, the budget, decision 4's rule for the `parallel` passes.
- `docs/ARCHITECTURE.md` crate table — `arris-math` holds `Control`.
- `docs/DATA-MODEL.md` — nothing: a `Control` is never stored.
- `docs/ROADMAP.md` §C5 — cancellation's line to done, with ADR-0030;
  the status line.
- `docs/BACKLOG.md` — the thin-elliptic section line: note it is now
  interruptible, the time finding itself stays for the breadth-and-speed
  cycle; add "progress reporting" if step 2 shows a consumer would want
  it.
- `CHANGELOG.md` `Unreleased` — the feature bullet (stop a running
  operation, cap one by a budget); the `Breaking` bullets are written
  by steps 2–6 as they land.
- `AGENTS.md` current state — C5's "Next" drops cancellation.

## Open questions

- Decided in step 2 (ADR-0030 amendment): the latency bound is 3.9 ms
  release on the slow section and every kept slow-unit input, 500 ms in
  the test profile; no solve is left unticked. Projection and
  `Profile`'s pcurves take no meter; `arris-ops`, `arris-io` and the
  fuzz targets pass `Meter::default()` at each call until their own
  steps thread the `Control`.
- Decided in step 1 (ADR-0030 §7): `Interrupted` carries the step count
  and the cause only, not the entities at the stopping step.
- Decided in step 3: every thread of a `parallel` pass polls (each item
  carries the poll in its split meter), since a poll promises only the
  rollback; where a poll lands under `parallel` is the schedule's, and
  only a budget is exact. `Meter::charge_stop` clamps a budget stop to
  the budget, because the parallel build splits every item off the
  budget left when the pass starts. Counts live in
  `crates/arris-ops/tests/cancel_counts.txt`, one record both builds are
  held to (`ARRIS_BLESS=1` rewrites it). Tests import the operations
  from `arris_debug::unmetered` (each op with `Control::NONE`), which
  steps 4–6 extend. Step 4 added `corpus::Inputs::run_result` (the
  result step under a `Control`), put a polyhedron fixture's staging and
  `build` in one transaction so an interrupt leaves no staged geometry,
  and re-exports `Control`, `Interrupted` and `Stop` from `arris_ops`.
  `rebuild::rewrite` takes no meter: its callers tick per stripe, corner
  and face; `transform` ticks per face, a primitive once.
- Decided in step 6: the reader's steps are per solid placement, per file
  edge, per face (twice: the singular-point pass and the walk), per
  pcurve use and every fit inside (`Meter` is threaded to `fitted`,
  `meet`, `seam_to`, `band`). An interrupt travels up inside a solid as a
  placeholder refusal and is replaced at `Geometry::solid` by the meter's
  first stop (`Meter::stopped`, new in `arris-math`), which also covers
  the sites that try a fit and fall back on failure. `file_solid` and the
  Part 21 parse are unticked (linear in the text).
  `arris_debug::unmetered::{step_read, body_read, body_from_json}` serve the tests.
- Decided in step 7: the property (`crates/arris-ops/tests/cancel_prop.rs`)
  runs over every body step of a drawn recipe, each rebuilt as the result
  step of its own `corpus::inputs_for_step` (the steps before it
  unmetered), eight shards; a step the kernel refuses ends its recipe's
  measurement. The recipe's steps are the operations of the corpus grammar
  (primitives, sweeps, blends, transform, booleans), so the readers and
  the tessellation are held to the property by their own tests
  (`arris-io`'s and `arris-mesh`'s `cancel.rs`), not by a drawn recipe.
