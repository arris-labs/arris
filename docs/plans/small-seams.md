# Plan: small-seams

- Started: 2026-10-23
- Milestone: beside C11 (side slot, docs/ROADMAP.md §Beside the cycles)
- Idea: docs/ideas/refactor-after-c10.md (its `small-seams` part absorbed;
  decision round 1 settled: option B, Q4 drop `#[non_exhaustive]`)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
Every helper the workspace wrote two to seven times exists once, in the
lowest crate that needs it (ADR-0013): a union-find, a golden-section
minimiser, `next_down`, interval samples, isoline pcurves, a skin's
control column, the io number formatter, `read_json`, the Bernstein
evaluator; the tests take `close`, `tol`, `EXACT` and their primitive
builders from `arris_debug::testing`; every bare plan citation is caught by
the lint; every `unwrap_or(0)` in library code has been read and either
justified or turned into a fixture; `Violation` and `Unchecked` are
exhaustive. Behaviour does not move: every blessed dump, fixture,
`cancel_counts.txt` and `tolerance_band.txt` byte-identical to the plan's
start, ids and refusal strings unchanged.

## Non-goals
- **The damped descents of `nurbs/line.rs` and `spline_nurbs.rs` stay
  apart.** Read at planning: one is Levenberg–Marquardt on two unknowns
  with trace-scaled damping and a travel stop, the other Marquardt's
  diagonal scaling on three unknowns with a metered residual stop. Equal
  budget constants are a coincidence of tuning, not one solver.
- Per-algorithm budget constants (`NEWTON_POLISH_STEPS`, `PATCH_SLACK`,
  `FLUSH_SAMPLES`, `STEP_HALVINGS`, `GOLDEN_STEPS`) stay where they are:
  each documents its own algorithm's bound.
- A defect found while moving code: a regression fixture and a backlog
  line, never a fix here.
- The other milestone plans of the idea, in order: `builder-split` (next,
  before C11's first plan), then `sweep-skeleton`, `one-bernstein`,
  `long-functions` in the side slot. The `arris-debug` split is its own
  idea after this plan.

## Design deltas
- **`arris-math`**: `DisjointSets` (`new(n)`, `find`, `union`, path
  halving, the lower root wins so ids stay as each copy made them);
  `roots::golden_section(f, a, b, steps) -> Bracket` (the final bracket
  and the two probes with their values), each caller keeping its own
  ending;
  `real::next_down` (MSRV 1.85 stand-in for `f64::next_down`);
  `Interval::samples(n)`. All additive.
- **`arris-geom`**: `Curve2::along_v(u)` / `along_u(v)`,
  `NurbsSurface::control_column(i)`; `extrema`'s `eval` calls
  `bernstein2::casteljau`. Additive.
- **`arris-io`**: one crate-private `format::real` for STL and OBJ (STEP's
  `real` carries an owner and an error and stays).
- **`arris-debug`**: one private `read_json`; `testing` gains `tol()`,
  `EXACT`, `cylinder`, `cube` with the signature most copies already use;
  a copy whose shape differs keeps a local name that says how.
- **`arris-check`**: `Violation` and `Unchecked` lose `#[non_exhaustive]`
  (Q4). Not a semver break; a consumer-visible change, so one
  `CHANGELOG.md` bullet. The binding already maps both exhaustively.
- **`arris/tests/docs_refs.rs`**: also matches `` `<slug>` step N `` and
  `<slug> steps N` where `<slug>` is a retired plan's, and fails on it as
  on `plans/<slug>`.
- A union-find's root choice or a golden section's probe order reaches
  geometry: a copy that does not match the shared form bit for bit stays,
  with a one-line comment naming the difference.

## Steps
Complexity grades: **[1]** routine — the design says exactly what to write
and the tests are mechanical; **[2]** careful — a geometric or numeric case
to get right within a given design; **[3]** unproven — an algorithm whose
robustness or bound has to be established here.

- [x] Step 1 **[2]** — `roots::golden_section` with unit tests; the five
  copies (`pcurve.rs`, `spline_nurbs.rs`, `torus_walk.rs` ×2,
  `blend/traced.rs`) compared and the matching ones moved. Blessed dumps
  and fixtures byte-identical. *Settled:* two forms exist; the cached-probe
  one (pcurve, traced, io's `worst_gap` by negating the gap) moved, the
  re-evaluating one (`torus_walk` ×2, `spline_nurbs`, debug's census) is
  not bit-identical and stays with a comment.
- [x] Step 2 **[2]** — `DisjointSets` with a property test against a naive
  partition; the seven copies moved (ops ×3, geom ×2, check, mesh; the
  `BTreeMap` one in `topo/builder.rs` over `VertexRef` indices if its root
  order matches). Byte-identical. *Settled:* result.rs ×2 and check
  joined with the higher root winning, but only count roots or number
  groups by first member, so the partition and every output are the same;
  `topo/builder.rs` stays (sparse slots), with a comment.
- [x] Step 3 **[2]** — read the ~32 `unwrap_or(0)`/`unwrap_or(0.0)` in
  library code: a justified one gets a typed default or a comment; a
  silent fallback becomes a regression fixture under
  `tests/fixtures/regression/` and a backlog line. No fix. *Settled:* 35
  read; the map counts, empty-`max` degrees and coefficient padding are
  plain, seven non-obvious ones got a comment; two (`trace_torus` order,
  `check` seam period) are not shown unreachable and no input reproduces
  them, so a backlog line each, no fixture.
- [x] Step 4 **[1]** — math/geom helpers: `real::next_down`,
  `Interval::samples`, `Curve2::along_v`/`along_u`,
  `NurbsSurface::control_column`, `casteljau` for `extrema`'s `eval`, each
  with a doc example; callers moved. *Settled:* the pcurves are named by
  the parameter that runs (as walked blend's copies were) and take no
  range, since a line is unbounded; `iso_u`/`iso_v` would read opposite to
  `arris-debug`'s private ones. `control_column` returns an error past the
  net. `pave`'s `samples` stays (a count of one gives both ends there).
- [x] Step 5 **[1]** — io's `format::real` and debug's single `read_json`. *Settled:* STEP's `real` keeps its owner and error and calls `format::real` after its finite check, so no third spelling remains.
- [x] Step 6 **[1]** — `Violation`/`Unchecked` exhaustive; the
  workspace's `_ =>` over them replaced by named arms; `CHANGELOG.md`
  bullet. *Settled:* two `_ =>` arms held them (`offset`'s self-intersection
  filter, now every other `Violation` named; `debug`'s corpus NURBS filter,
  whose arm was unreachable and went).
- [x] Step 7 **[1]** — test helpers: `close`, `tol`, `EXACT` from
  `arris_debug::testing` in every test file that defines its own equal
  copy. *Settled:* `close` was already shared; the local ones take a `rel`
  or `floor` argument or a `found`/`wanted` pair, so they are different
  functions and stay. `tol()` (24 geom files) and `EXACT = 1e-12 ·
  DEFAULT_SCALE` (identical copies) moved; the `1e-12`, `1e-9` and
  `SCALE` ones are different bounds and stay.
- [x] Step 8 **[1]** — `cylinder`/`cube` test builders into
  `arris_debug::testing`, the copies that match moved. *Settled:*
  `cube(m, side)` (fillet's signature; shell, offset and chamfer pass
  their edge) and `cylinder(m)` (the 4 × 12 sample, mesh's `cancel` and
  check's `fast_part1`). Different shapes keep their names: `interference`
  (corners), mesh's unit-box `cube`, `Aabb`/`Surface` builders in geom,
  the hand-built `cylinder` of debug's `builder.rs`.
- [x] Step 9 **[1]** — `docs_refs.rs` widened (with its own planted-file
  test); the ~30 bare citations rewritten to the ADR or design-doc section
  they mean, or deleted where the code says it. *Settled:* a retired plan's
  slug is any hyphenated name before `step N`/`steps N` (filler of a
  wrapped comment allowed) that is not a live plan; no list of retired
  slugs to keep. 31 files rewritten: the NURBS cycle's three to ADR-0061,
  -0062, -0068, the rest (`blend-corners`, `blend-miters`, `ray-fire`,
  `facade-swap`, `m4-booleans`, `multi-tool-boolean`,
  `seam-parametrisation-faults`) deleted, the code or `docs/BACKLOG.md`
  saying it.

## Acceptance
`tools/full-gate.sh` green; `git diff <plan start>.. -- 'tests/fixtures/**'`
empty except new regression fixtures from step 3; the corpus areas'
blessed dumps unchanged; `tools/py-test.sh` green; the refactor-review
scripts re-run show no union-find, `below`, `samples`, `column`, `real`,
`read_json`, `golden` or `fn close` copies left except those a comment
names.

## Docs to update on completion
- `docs/ARCHITECTURE.md` §arris-math — `DisjointSets` and the minimiser
  as shared numerics.
- `CHANGELOG.md` `Unreleased` — the exhaustive `Violation`/`Unchecked`
  (written in step 6).
- `docs/ideas/refactor-after-c10.md` — already cut to the unplanned part.
- `GLOSSARY.md` — no new term.
- `AGENTS.md` current state — the refactor sequence under way.

## Open questions
None for the human. Decided at planning by the agent: the descents stay
apart (Non-goals, read); copies that are not bit-identical stay with a
comment rather than be re-blessed, since this plan moves no behaviour.
