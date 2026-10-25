# Idea: refactor-after-c10

- Status: Accepted (B); `small-seams` done and retired 2026-10-23; `builder-split` done and retired 2026-10-24; `sweep-skeleton` done and retired 2026-10-25; `one-bernstein` done and retired 2026-10-25
- Raised: 2026-10-22
- Prompt (verbatim from the human): "We were developing features without any refactoring for too long. Your task is to review whole codebase and write refactoring plan, but put it to gitignored notes folder for now" — then, 2026-10-23: "go ahead" (as an idea through `/refactor-review`)

## Problem

Since `refactor-before-c7` retired (2026-10-05, `0812861f`): 297 commits,
+82.4k / −3.4k lines under `crates/`, 8 `refactor(…)` commits, all of
them local. C8–C10 each wrote their feature beside the last one, and none
merged with it. The workspace is 141k lines of `src` (geom 42.5k, ops 31.8k,
debug 28.5k, io 11.9k, py 7.6k, topo 6.9k, check 5.2k, mesh 3.7k, math
3.0k) and 74k of tests. 55 library functions run past 150 lines.

The kernel rules still hold: library code has no `HashMap`/`HashSet` and
no `unwrap`. The `_ =>` arms near geometry enums are property lookups,
not dispatches. The plan-reference lint works. The kernel-seams and
`refactor-before-c7` items are fixed: one `FaceDomain`, one Euler line,
slot-per-spec `assemble`, `ops::rebuild`, direct dependencies, the
periodic toolkit, grouped `Reason`, the corpus table, the interior-grid
cap. The debt is structural. A change to a shared step in a sweep, a
NURBS intersection or a section tracer has to be made in two to four
places, and C11's blends sit on the builder that both earlier plans
deferred.

Measured at `8e472b73` with `.agents/skills/refactor-review/scripts/`.
Each finding below is **read** (the copies compared in the source) or
**measured** (a count or a hash).

## Findings

### For a `small-seams` plan
Done (its descent finding read and dropped: the two descents are different
algorithms).

### For a `sweep-skeleton` plan
Done (its `Cavity` drift read: `Profile::edges` keeps the consumer's loop
order, so the two namings agree).

### For a `one-bernstein` plan
Done and retired.

### For a `long-functions` plan (measured sizes, read structure)

| Function | Lines | Along |
|---|---|---|
| `debug/differential.rs` `refusal` | 711 | a table per recipe op |
| `io/step/reader/topology.rs` `build` | 628 | points, edges, faces, loops, healing, assemble — phases over a context, as `blend::build` |
| `debug/corpus.rs` `build_step` (file 3385) | 399 | one interpreter per op family; `corpus/` module tree |
| `debug/prop/recipe.rs` `write` | 383 | per op family |
| `mesh/tessellate.rs` `tessellate_under` | 330 | edge counts, edge samples, face work |
| `ops/boolean/result.rs` `boolean_each_witnessed` | 245 | select, shells, assembly, provenance |
| `ops/boolean/mod.rs` `fmt` | 237 | the `Interferences` dump into its own file |

- **The recipe op table is interpreted four times:** `build_step`,
  `prop::recipe::write`, `differential::refusal`, and the oracle's
  `recipe.py`. Nothing holds that a new op reaches all of them.
  - → In Rust, one `Op` enum matched exhaustively by all three.
  - → In Python, a lint on op names.
- **`py/src/model.rs`** (2326 lines) is one `#[pymethods]` block of ~1870
  lines. → Keep the block as thin methods over free functions in
  `model/{ops, query, io, topology}.rs`. pyo3's `multiple-pymethods` is
  off.
- **The files the kernel-seams review listed as holding several concerns:**
  - `check/check.rs` (857 lines): rows by entity;
  - `mesh/cdt.rs` (1169): locate/insert/legalize, constraint recovery,
    classification;
  - `geom/pcurve.rs` (1602, twice its size then): plane, revolution,
    fitted.
- **The check tests are still filed by the plan step that wrote them**
  (`fast_part1`/`fast_part2`) instead of by topic.
- **Four corpus reports each have their own markdown writer:** `census`,
  `nurbs_census`, `histogram` and `gaps`.
  - `nurbs_census` is reached only from `examples/real_parts.rs`, and
    NURBS now blocks 0 of 38 parts.

### Waits for its own idea
The `arris-debug` split. A light testkit above topo, a corpus crate and
the renderer would mean the crate (now 3.5× its size at the kernel-seams
review) stops linking into every lower crate's test build. That needs an
ADR and a layer row. It comes after `small-seams` has fixed what the
testkit exports, and after a measured relink time.

### Leave these alone
- the tracers' algorithms;
- the blend phase tree: no function past 300 lines and no
  `too_many_arguments` in `blend/`;
- `ops/error.rs`: long because each `Reason` group documents its
  refusals;
- the STEP lexer (`part21::lex`): long and regular;
- `arris-math`'s structure;
- the kernel-seams seams.

## Constraints it runs into

- **No behaviour change**, as in `refactor-before-c7`:
  - every blessed `dump.txt`, `fixture.json`, `cancel_counts.txt` and
    `tolerance_band.txt` byte-identical to each plan's start;
  - ids and refusal strings unchanged;
  - a defect found while moving code becomes a regression fixture and a
    backlog line, never a fix in the same commit.
- **Layers** (ADR-0013): every new helper goes to the lowest crate that
  needs it (`DisjointSets` and the minimisers to math, the isoline
  pcurves to geom). None creates an upward edge.
- **Breaking changes** (`.agents/rules/kernel.md` §API, ADR-0027): nested
  `BuildError` and `DegenerateReason` each take a `### Breaking` bullet
  (both reach Python as text, so no class or `_arris.pyi` line changed). `Curve2::iso_u`/
  `iso_v` and `NurbsSurface::control_column` are additive.
- **At most two active plans**, and closure before breadth (ADR-0020).
  C11 is open and its first plan is not written, so refactor plans share
  the slots with it.

## Options

### A — Everything before C11
Five plans, about 35 steps, then C11's first plan. This is the cleanest
base. It delays the blend residue, which blocks 12 of 38 parts and 3 of
the script corpus's 4 refusals, by about five plans.

### B — Interleaved: what C11 stands on first, the rest in the side slot
1. `small-seams` (about 8 steps).
2. `builder-split` (about 8): C11's blends assemble through it.
3. C11's first plan.
4. `sweep-skeleton` (about 6), `one-bernstein` (about 5) and
   `long-functions` (about 7) one at a time in the side slot.

C11 waits two plans.

### C — Only when a file is next touched
This is the rule `refactor-before-c7` and the kernel-seams review left
in the backlog. It has not worked: `builder.rs`, the sweeps and the STEP
reader were touched by C8–C10 and grew instead of splitting, because a
feature commit avoids an unrelated move.

### Do nothing
Every shared fix lands in two to four copies, and the copies already
drift (the sweep's `Cavity`). The next sweep, intersection arm or tracer
adds one more.

## Recommendation

**B.**
- **A** buys little beyond B: C11 touches the builder and the blend, not
  the sweeps or the Bernstein code.
- **C** is what produced this review.
- **`small-seams` goes first** because it is mechanical and fixes the
  testkit surface the `arris-debug` idea needs.
- **`builder-split` goes second** because C11 would otherwise grow
  `builder.rs` again.
- **What would change it:** C11's first plan turning out to need a sweep
  (a blend that sweeps a profile). Then `sweep-skeleton` moves up to
  second.

## Decision for the human

Round 1:

❓ **Q1 — Order**: A (all before C11), B (interleaved), C (when touched)?
➡️ B — C11 waits two plans, not five.

❓ **Q2 — Breaking errors in `builder-split`**: nest `BuildError` and type `DegenerateReason` in that plan (yes), or keep the free text?
➡️ Yes. Both have been deferred twice, and the binding changes once.

❓ **Q3 — 2-D isolation in `one-bernstein`**: keep it bit-identical with the joint exclusion off for N = 2 (yes), or turn the exclusion on and re-bless?
➡️ Yes, identical. Turning it on is a measured change of its own and a backlog line.

❓ **Q4 — `#[non_exhaustive]` on `Violation`/`Unchecked`**: drop it for the exhaustive-enum rule in `small-seams` (yes), or keep it and record why?
➡️ Yes, drop it. Both are matched inside the workspace, and the rule exists so a new row fails to compile there. This has been open since the kernel-seams review.

❓ **Q5 — `nurbs_census`**: delete it in `long-functions` (yes), or fold it into `census` as a column?
➡️ Yes, delete it. Its one question (what the NURBS cycle blocks) is answered by 0 of 38.

No ADR for the five plans: each is structure under existing ADRs. *Later*:
the `arris-debug` split, as its own idea and ADR.

## Decision

2026-10-23, round 1 (the human delegated it; the agent took each
recommendation): **B**, interleaved — `small-seams`, then `builder-split`,
then C11's first plan, the rest in the side slot. Q2 yes: `builder-split`
nests `BuildError` and types `DegenerateReason`. Q3 yes: 2-D isolation
bit-identical, the joint exclusion off for N = 2. Q4 yes: drop
`#[non_exhaustive]` (in `small-seams`). Q5 yes: delete `nurbs_census` in
`long-functions`. No ADR.

