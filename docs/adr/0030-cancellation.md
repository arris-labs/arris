# ADR-0030 — Cancellation: a `&Control` passed last, a poll and a budget of steps, rollback by the transaction every operation already runs in

- Status: accepted (2026-09-29)
- Plan: `cancellation` steps 1–7
- Follows: ADR-0013 (the layer order that puts the types in
  `arris-math`), ADR-0009 (the transaction an operation runs in),
  `docs/ARCHITECTURE.md` §Threading and wasm (no clock, no thread, no
  timer in a kernel crate)

## Context

The first consumer evaluates a feature tree on a user's click and must be
able to abandon an evaluation the user has already replaced. Some
operations run for minutes: the backlog's thin-elliptic section (a torus
of major radius 1000 against an elliptic cylinder of semi-axes 990 and
0.5) takes about six minutes in the tracer and the fit. Today the only
stop is dropping the thread, which on wasm is not available and on
native leaves the model half-built.

The kernel already has the rollback: every operation on a model runs in
a `Model::transaction` and an `Err` restores the model, ids included.
What it lacks is a way for the outside to make an operation return one,
and a way to test that the return is clean at every point.

## Decision

1. **Passed, not stored.** Every operation on a model, and every long
   query beside one, takes a trailing `&Control`. `Control::NONE` is what
   a caller that does not care passes.
2. **A poll, not an `AtomicBool`.** A `Control` holds an optional
   `&(dyn Fn() -> bool + Sync)`. On native it reads the consumer's
   atomic. On wasm without shared memory an atomic set from another
   worker is never seen; a poll reads whatever the consumer has (a
   `SharedArrayBuffer` through `js-sys`, a message counter, its own
   clock). `Sync` because the `parallel` passes poll from several
   threads.
3. **A budget in steps.** A `Control` also holds an optional `u64`.
   Exhausting it is `Interrupted { by: Stop::Budget, steps }`, a poll
   answering true is `Stop::Poll`. The budget is what makes interruption
   testable at every point and reproducible on every platform, which the
   roadmap's acceptance asks; it costs one counter.
4. **Determinism under `parallel`.** A parallel pass hands each item a
   `Meter` capped at the budget left when the pass starts. After the
   pass the items' steps are summed in sequential order and the pass
   stops at the first item whose running sum crosses the cap. The
   sequential build applies the same rule, so both stop on the same item
   with the same count; the parallel build only wastes the work of items
   past it. A poll is by nature not deterministic; only its result, the
   model as it was, is promised.
5. **What a step is.** One iteration of a loop whose trip count is not
   bounded by the input's entity count. The sites, so that a new
   algorithm knows it owes one:
   - `arris-geom`: a tracer's march step (`trace`, `torus_walk`), a
     fit's refinement pass, a Bernstein subdivision split, a Newton
     polish iteration, a projection's seed and iteration;
   - `arris-ops`: a boolean face pair, a face split, a blend corner, a
     sweep's section, a rebuild's face;
   - `arris-mesh`: an edge's sampling, a CDT insertion;
   - `arris-io`: an entity converted, a solid read.

   Work bounded by the input's entity count, and constant-work calls,
   tick nowhere of their own.
6. **Where the types live.** `Control`, `Meter`, `Stop` and `Interrupted`
   are in `arris-math`, so `arris-geom`'s tracer ticks without naming an
   upper crate. Each crate's error gains an `Interrupted(Interrupted)`
   variant; `arris` re-exports the three consumer-facing types.
7. **`Interrupted` carries the step count and the cause, nothing else.**
   It does not name the entity being worked on at the stop. Kernel.md's
   "errors name entities" is about geometry refusing; an interrupt is
   the caller's and not the geometry's, and the entity at the stopping
   step is noise to a consumer. Steps are stable within one release, not
   across releases: an algorithm change moves them, so a budget is a cap
   a consumer tunes, not a contract.
8. **Not instrumented, by choice.** The checker (linear in the body at
   `Fast`; an interrupt that lands in it waits for it), the writers, and
   the constant-work calls take no `Control`. `step::read` runs in one
   transaction, so an interrupt drops the solids already read instead of
   returning a partial result.

## Consequences

- One `### Breaking` bullet per crate for the trailing argument; a
  consumer's facade adds one parameter per call it wraps.
- Every corpus fixture runs with `Control::NONE` and must not change: a
  meter that is never asked costs a counter increment and a branch.
- A test can interrupt an operation at every step `k` below its count
  `N` and assert the model's bytes are unchanged, and at `k >= N` that
  the result is the unbudgeted one.
- The latency between two ticks on the slow cases is measured in step 2
  (in a test, never in the kernel) and recorded there; a stretch found
  unbounded gets a tick inside it or is named as the latency floor in an
  amendment to this ADR.
- Whether the `parallel` build's threads poll mid-pass or only at its
  join is decided in step 3; the budget rule in decision 4 holds either
  way.

## Alternatives considered

- **A field of `Model`.** Interior mutability in a representation crate,
  and a cloned model (clone-evaluate-import) would share its interrupt.
- **A thread-local.** None exist in a kernel crate, and wasm has no
  threads to key it by.
- **A `..._with` twin of every operation.** Doubles the surface a
  consumer's facade wraps, for the one argument a facade adds anyway.
- **An `AtomicBool` the kernel owns.** Not seen across wasm workers
  without shared memory; a poll subsumes it (`|| flag.load(..)`).
- **A wall-clock timeout in the kernel.** The kernel touches no clock
  (§Threading and wasm) and a timeout is not reproducible; the
  consumer's poll can read its own clock.
- **Progress reporting** (a fraction, a stage name). Nobody asked; it
  is a backlog line if a consumer does.
