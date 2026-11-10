# Plan: census-and-tests

- Started: 2026-11-10
- Milestone: C12's side slot (docs/ROADMAP.md §C12)
- Idea: docs/ideas/refactor-after-c11.md (absorbed: its last part; deleted)

## Goal
The debug crate's census, the fixtures lint and the blend properties have
the shape their content has, with nothing they produce changed.
`census.rs` (2705 lines) becomes `census/` by family, and its four
`_committed` functions share one loop over the committed tier.
`fixtures::lint` is one check per row instead of one 363-line function.
`ops/tests/blend_prop.rs` (5345 lines) becomes `tests/blend_prop/`: still
one binary, one module per section. The nightly's filters, known list and
backlog reproduce lines name the moved tests. `loft_prop` and
`prop::recipe` share one `LoftSection`. Every blessed dump and fixture
expectation, every census table and every lint message stays
byte-identical to the plan's start.

## Non-goals
- No behaviour change (the idea's *Constraints*). A defect found while
  moving code becomes a regression fixture and a backlog line, not a fix.
- No public API of the published crates. `arris-debug` is
  `publish = false`, and `arris_debug::census::*` keeps its paths through
  re-exports, so `real_parts`, `horn_census`, `parallel_cylinder_census`,
  `survey.rs` and `gaps.rs` don't change. No binding, stub or
  `CHANGELOG.md` line.
- The `arris-debug` split, `FitError::Degenerate(String)` and
  `Build::lump_order`'s clone stay on the backlog, as the idea says.
- The other `*_prop.rs` files and the cusp pair census's convex/concave
  backlog line.

## Design deltas
- **`tests/blend_prop/`**: `main.rs` keeps the file's `//!` header and its
  `mod` lines. `support.rs` holds what several sections share (imports,
  `REL`-style constants, scene builders). There is one module per section
  comment, named for its family: `plane_pairs` (ADR-0007/0037, lines 1–547),
  `turned` (0036), `step_end` (0038), `four_edge_vertex` (0039),
  `rim_arcs` (0041), `cusp` (0042/0079), `miter` (0044),
  `run_over_edge` (0071), `real_part_kinks` (0074/0075), `horn` (0080),
  `parallel_cylinders` (0036 §5), `seam_vertex` (0083) and
  `cusp_corner` (0084). Fan (0043) and flush pieces (0082) go wherever
  their tests sit now. The step's commit body lists the map.
  - Test paths become `<module>::<name>[::shard_N]`. The binary id
    `arris-ops::blend_prop` stays the same. Each nightly
    `test(/^name…/)` becomes `test(/^<module>::name…/)`, and each
    `tools/nightly-known.txt` source becomes
    `arris-ops::blend_prop::<module>::<name>`. Seeds don't read the test
    path (`prop::shard_seed`), so every shard draws the same cases.
- **`census/`**:
  - `mod.rs` holds `BUILT`, `PANICKED`, `class_of`, `blend_census`,
    `census_file`, `committed`, `markdown`, the shared readers
    (`edge_midpoint`, `pair_of`, `face_kind`) and `pub use` of each family.
  - `tier.rs` holds `over_committed_tier`.
  - One file per family: `cusp.rs`, `run_over.rs`, `vertex.rs` (vertex
    blends), `tangent.rs` and `parallel.rs`. Each family's unit tests go
    with it.
  - The loop is crate-private:
    `fn over_committed_tier<T>(only_refused: bool, per_file: impl Fn(&Path,
    &dyn Fn(&str) -> bool) -> Result<T, String>) -> Result<Vec<(String, T)>,
    String>`. It loads each fixture, keeps the solids whose `fillet` stage
    Arris refuses unless `only_refused` is false, and calls `per_file`.
    `committed`, `vertex_blend_committed` and `run_over_committed` pass
    `true`. `cusp_pairs_committed` passes `false` and ignores the filter.
- **`fixtures::lint`**: the dispatch, load and `problem` prefix stay in
  `lint`. The rest becomes private functions, each returning its
  `Vec<String>`, called in today's order, so the messages and their order
  don't change:
  - `header` (the recipe hash, `pose_of`) and `recipe_names` (step names,
    `result`), once per fixture;
  - per variant: `flags` (dropped edges, `degenerate`, `expect_error`,
    `counts_differ`, `checker_undecided`, `step_differs`, the refusals),
    `dump_presence`, `euler`, `closed_forms` (volume, area, centroid,
    inertia), `second_oracle`, `counts` and `probes`.
  - `ANALYTIC_REL`'s `rel` becomes one private `fn`.
- **`LoftSection`** moves to `arris_debug::prop::profile` as `pub`, with
  `area`, `scaled` and `at(z) -> Profile`. `recipe::loft_shape` and
  `loft_prop` use it, and `loft_prop`'s `Section` and recipe's private enum
  are deleted. Both strategies draw what they draw today.
- No ADR (the idea's decision: structure under existing ADRs). No new
  term.

## Steps
Scale: **[1]** routine, **[2]** careful, **[3]** unproven
(`.agents/tiers.md`).

- [x] Step 1 **[2]** — `blend_prop.rs` → `tests/blend_prop/` per the
  deltas. In the same commit, update `.github/workflows/nightly.yml`'s six
  `blend_prop` filters, `tools/nightly-known.txt`'s four sources,
  `docs/BACKLOG.md`'s anchored names and `blend_prop.rs:391`, and the path
  in `prop/turned.rs` and `geom/tests/intersect_nurbs.rs`. Test: the
  binary lists the same test count. Each nightly filter's
  `cargo nextest list -E` lists the same names with their prefix dropped,
  before and after (a scratch script; the counts go in the body).
  `tools/nightly-triage-test.sh` passes.
- [x] Step 2 **[1]** — one `LoftSection` in `prop::profile`, used by
  `loft_prop` and `recipe::loft_shape`. Test: `loft_prop` and the recipe's
  properties pass at 256 cases. The multi-tool differential's fixed-seed
  case list is unchanged (`regression/revolve-cut-many-box-*` still case
  230).
- [x] Step 3 **[1]** — `over_committed_tier`, and the four `_committed`
  functions on it. First, save the parent's output of `real_parts
  --census-committed`, `--run-over-committed`, `--vertex-blend-committed`
  and `--cusp-pairs-committed` (release, the oracle's cache) under
  `target/census-baseline/`. Test: all four outputs byte-identical, and
  `Some(Class::ArrisRefuses(_))` appears once in the census.
- [ ] Step 4 **[1]** — `census.rs` → `census/` by family per the deltas,
  tests with their family. Test: step 3's four outputs byte-identical
  again, and the crate's census tests (`horn_census`,
  `parallel_cylinder_census`, the unit tests) list and pass the same.
- [ ] Step 5 **[1]** — `fixtures::lint` as one check per row. Test:
  `corpus_lint` passes unchanged, its mutated-fixture tests included. A
  scratch harness also runs the lint before and after over every corpus
  directory, with each of `corpus_lint`'s mutations applied to a copy of
  three fixtures (a plain, a posed and a second-oracle one), and diffs the
  output: empty.

## Acceptance
- `tools/full-gate.sh` green on the tip.
- `git diff --stat <plan start>.. -- tests/fixtures crates/*/tests/*.txt`
  is empty. Blessed dumps, `fixture.json`, `cancel_counts.txt` and
  `tolerance_band.txt` are untouched.
- The four `--*-committed` census tables are byte-identical to
  `target/census-baseline/`.
- Each nightly filter lists the same tests, by name without the prefix.
- `rg -n 'blend_prop\.rs' --glob '!docs/adr/**' --glob '!CHANGELOG.md'
  --glob '!docs/ideas/**' --glob '!docs/plans/**'` has no hit. Run at planning, it hit
  `docs/BACKLOG.md` (lines 275, 276, 425), `docs/ROADMAP.md:433`,
  `prop/turned.rs:14` and `geom/tests/intersect_nurbs.rs:402`, all in
  step 1. ROADMAP's line goes at retirement.
- `rg -n 'enum (Loft)?Section' crates/arris-ops/tests/loft_prop.rs
  crates/arris-debug/src/prop/recipe.rs` has no hit.
- The refactor-review scripts show no function in `census/` or
  `fixtures/mod.rs`'s lint over 150 lines, and no `blend_prop/` module
  over 1000.

## Docs to update on completion
- `docs/ROADMAP.md:433`: `tests/blend_prop/` for `blend_prop.rs`. C12's
  side-slot line: refactor-after-c11 done.
- `AGENTS.md` current state: drop the side-slot clause naming this plan.
- No `GLOSSARY.md`, ADR, `CHANGELOG.md` or backlog change (no line holds
  the loft section clause).

## Open questions
- Settled (step 1): filters name the module (`^<module>::name`). It is
  exact, and a name reused in two sections cannot widen a nightly job.
  The exact slow-test exclusion in `.config/nextest.toml` and its
  `tools/profile-test.sh` record also take the new `real_part_kinks::`
  prefix, preserving the `fast` slice.
- Settled (step 3): all four release census tables match the parent
  byte for byte with the oracle cache enabled. No nondeterministic
  fields needed excluding; baselines are in `target/census-baseline/`.
