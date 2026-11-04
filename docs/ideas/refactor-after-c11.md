# Idea: refactor-after-c11

- Status: Accepted (B); `rewrite-seams` absorbed by plans/rewrite-seams; the rest (`blend-phases`, then `census-and-tests`) waits for it
- Raised: 2026-11-04
- Prompt (verbatim from the human): "/refactor-review" (handed over by C11's close)

## Problem

Since `file-concerns` retired (2026-10-26, `4798d23f`), the last plan of
`refactor-after-c10`, there have been 186 commits and +14.5k / −0.9k lines
under `crates/`, with no `refactor(…)` commit. Library `src` is 152.3k
lines (143.7k at the base) and tests are 79.2k (74.3k). C11 put its growth
in one place. `ops/blend/` grew from 11.4k to 17.0k lines in 33 commits,
with four new modules (`mutual`, `rolled`, `run_over`, `ring_rolled`).
Every function that grew past 150 lines since the base is in `blend/`,
except the corpus lint.

The kernel rules still hold in library code. There are no `unwrap`s and
no `HashMap`/`HashSet`. The `_ =>` arms near geometry enums are property
lookups and guards, not dispatches (read: `intersect.rs`, `surface.rs`,
`torus_walk/band.rs`, `blend/{build, mutual, stripe}.rs`). The 21
`unwrap_or(0)`s, measured but not read last time, are now read, and each
is deliberate: a `max` over an empty set, a missing period, Bernstein
coefficients past the degree, and one documented on-axis foot in
`extrema.rs`. This review has no correctness lead.

The debt is in the shape of two areas:
- **The blend's assembly.** It writes every vertex, edge and edge use
  by hand. Draft and thicken, C12's theme, will write through the same
  `Rewrite`.
- **C11's surveys.** They each copied the census's reading of the
  committed tier.

Measured at `1a7bc1e6` with `.agents/skills/refactor-review/scripts/`,
plus one fix to its parser (below). Each finding is **read** (the copies
compared in the source) or **measured** (a count or a hash).

## Findings

### The earlier reviews' open items
- **The `arris-debug` split**: *open*. The crate is 29.2k lines. It still
  waits for its own idea, after a measured relink time. The backlog line
  stays.
- **`FitError::Degenerate(String)`**: *open*, one variant
  (`geom/nurbs/fit.rs`). It is typed when `FitError` is next touched.
  The backlog line stays, since it is a break that no plan below needs.
- **`Build::lump_order` clones the model**: *open*
  (`ops/boolean/result.rs`). The backlog line already holds its decision,
  so it stays.
- **`loft_prop`'s `Section` and `prop::recipe`'s `LoftSection`**: *open*
  (read). Each defines a rectangle-or-circle section. It is absorbed by
  `census-and-tests`, and that clause leaves the loft backlog line.
- The repeated solver constants (`NEWTON_POLISH_STEPS`, `PATCH_SLACK`,
  `STEP_HALVINGS`, `FLUSH_SAMPLES`, and the line/spline descent's six):
  *no longer apply*. `small-seams` read the descents and found two
  different algorithms. Each constant is its own budget.

### For a `rewrite-seams` plan
Absorbed by `plans/rewrite-seams`.

### For a `blend-phases` plan (read structure, measured sizes)
- **`blend/build.rs` is 2557 lines (1701 at the base)** and holds five
  phases:
  - classify and joins (`classify` … `cusp_of`);
  - make (`make_stripes` … `add_corners`);
  - entities (`add_stripes` … `shorten_edges`);
  - faces (`rewrite_faces` … `add_ring_faces`);
  - `record`.

  → `blend/build/` by phase, as `boolean/pave/` was split.
- **The functions C11 grew**, with their lengths at the base:

  | Function | Lines | Was | Along |
  |---|---|---|---|
  | `junction::junction` | 376 | 228 | pair the contacts, exits across a kink, the lean, the meeting, the slide onto a seam, the trims: phases over a context, as `blend::build` |
  | `stripe::stripe` | 279 | 199 | one function per row of the surface-pair table (two planes, a plane and a cylinder, two cylinders), the dihedral read once |
  | `build::add_stripes` | 258 | 172 | end vertices, the run-over, contact edges, end arcs, remnants |
  | `ring::place_contacts` | 205 | 158 | — |
  | `build::rewrite_faces` | 186 | 127 | — |

  `rolled_of` and `add_run_over` (153 each) are new and stay as they are.
- **`add_ring_faces` writes a ring's four-use loop twice**: once for the
  walked ring, once for the closed form. With `StoredUse::new` it is one
  loop over `(edge, forward, pcurve)`.

### For a `census-and-tests` plan (read)
- **`debug/census.rs` is 2705 lines (+623)** and holds five censuses:
  the blend census, vertex blends, tangent chains, run-overs and parallel
  cylinders, plus cusp pairs. Each has its own `_file`, `_committed` and
  `_markdown`.
  - `committed`, `vertex_blend_committed` and `run_over_committed` are
    the same loop over `COMMITTED_TIER` (load, keep the solids whose
    `fillet` stage Arris refuses, call the per-file function). They
    differ only in that function.
  - `cusp_pairs_committed` is the same loop without the filter.
  - → One `over_committed_tier` taking the per-file function, and
    `census/` by family.
- **`fixtures::lint` is 363 lines (284 at the base)**: one function over
  every field of a fixture. → One check per row (recipe, oracle, second
  oracle, analytic, variants), each returning its problems.
- **`ops/tests/blend_prop.rs` is 5339 lines (+2064)**, in 14 sections by
  ADR. → `tests/blend_prop/` as one binary with one module per section,
  as the check tests became.
  - The binary keeps its name, so the nightly's
    `binary_id(arris-ops::blend_prop)` filters stand.
  - Test paths gain a module prefix. The plan greps `docs/BACKLOG.md`'s
    reproduce lines and `.github/` for anchored test names before the
    move.
- **The loft section types** (above): one `LoftSection` in
  `arris_debug::prop`, which `loft_prop` uses.

### Leave these alone
- `cross` and `dot` over `[f64; 3]` in `debug/render.rs`, `io/stl.rs` and
  `mesh/trimesh.rs`. They are three 7-line functions in three crates at
  the mesh boundary, and one shared copy would be public API.
- `rolled.rs`, `run_over.rs` and `mutual.rs`: new, each one ADR's
  construction, no function past 153 lines.
- The tracers (`trace_torus.rs` 2730, `trace.rs` 2068, `march/walk.rs`
  1719): each is one algorithm, unchanged in shape since `one-bernstein`.
- The STEP reader's `geometry.rs` and `topology.rs`: unchanged since
  `long-functions` split their phases.
- `ops/error.rs`, the STEP lexer, `arris-math`'s structure: as the last
  review said.
- The 39 `allow(too_many_arguments)`: none is in a function this review
  splits, except the two in `rolled.rs` that `StoredUse::new` removes.

## Constraints it runs into

- **No behaviour change**, as in `refactor-after-c10`:
  - every blessed `dump.txt`, `fixture.json`, `cancel_counts.txt` and
    `tolerance_band.txt` byte-identical to each plan's start;
  - ids and refusal strings unchanged;
  - a defect found while moving code becomes a regression fixture and a
    backlog line, never a fix in the same commit.
- **Layers** (ADR-0013): `Rewrite`'s methods and `StoredUse::new` stay in
  `arris-ops`, the closure helper in `arris-geom`, the census loop in
  `arris-debug`.
- **No public type or signature changes.** Every item above is
  crate-private or test code, so the binding, `_arris.pyi` and
  `CHANGELOG.md` are untouched.
- **The nightly's slot-chain panic**
  (`slot-chain-and-elbow-outputs-fail-the-checker`) is in the blend's
  kink and vanishing-end code, the code `blend-phases` moves. Its bisect
  range (`5e4e7023..bbcea937`) and fix should land on the code as it is.
- **At most two active plans** (ADR-0020's closure before breadth). C12
  takes the cycle's slot. The side slot holds the nightly's two panics,
  then these plans.

## Options

### A — All three before C12's first plan
About 18 steps. C12 waits three plans for structure it needs only in
part (`rewrite-seams`).

### B — Interleaved in the side slot
1. The nightly's two panics.
2. `rewrite-seams` (about 5 steps), before C12's first code step, so
   draft and thicken write through `Rewrite::edge` and `StoredUse::new`
   from their first commit.
3. `blend-phases` (about 7 steps).
4. `census-and-tests` (about 5 steps).

C12's main slot does not wait.

### C — Only when a file is next touched
Both earlier reviews recorded that this does not happen. C11 touched
`build.rs`, `junction.rs` and `census.rs` in 33 commits, and each grew.

### Do nothing
Draft and thicken add a third and fourth hand-written `Rewrite`. The next
blend construction adds a sixth phase to `build.rs` and another
`_committed` copy to the census. The review scripts keep missing
functions with array parameters.

## Recommendation

**B.**
- **A** holds C12 for `blend-phases` and `census-and-tests`, which C12
  does not touch. C12 is draft, thicken and `region2`: offset, shell and
  the rewrite, not the blend.
- **C** is what produced this review.
- **`rewrite-seams` comes first** because it is the one plan C12 stands
  on, and it is small.
- **`blend-phases` comes after the panics** so their fix lands on the
  code as the bisect reads it.
- **What would change it:** a C12 plan that needs a blend (a fillet
  after a draft, say) moves `blend-phases` ahead of that plan.

## Decision for the human

Round 1:

❓ **Q1 — Order**: A (all before C12), B (interleaved in the side slot), or C (when touched)?
➡️ B. The panics first, then `rewrite-seams` before C12's first code step, then `blend-phases`, then `census-and-tests`.

❓ **Q2 — The script fix**: fix `_rust.py`'s parser as `rewrite-seams` step 1 (yes), or in this review's commit?
➡️ Yes, step 1. The review commit stays prose, and the fix is verified like any tool change.

❓ **Q3 — `blend_prop` as a module tree**: split the 5339-line test file into `tests/blend_prop/` as one binary (yes), or leave it whole?
➡️ Yes. The binary name, and so the nightly's filters, stay the same.

No ADR: each plan is structure under existing ADRs. *Later*: the
`arris-debug` split, as its own idea and ADR.

## Decision

2026-11-04, round 1: the human answered "yes" to every recommendation.
- **Q1: B**, interleaved in the side slot, in this order: the nightly's
  two panics, `rewrite-seams` before C12's first code step,
  `blend-phases`, `census-and-tests`.
- **Q2: yes.** `_rust.py`'s parser fix is `rewrite-seams` step 1.
- **Q3: yes.** `blend_prop` becomes `tests/blend_prop/`, still one binary.

No ADR.
