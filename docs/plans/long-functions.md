# Plan: long-functions

- Started: 2026-10-25
- Start commit: b257bfe0 (the acceptance's `<start>`); `fn_lengths.py` there, char literals blanked: 3064 functions, 125 past 100, 54 past 150 lines
- Milestone: the refactor side slot beside C11 (cycle C11, docs/ROADMAP.md)
- Idea: docs/ideas/refactor-after-c10.md, the long functions and the recipe
  table of its `long-functions` part absorbed (decision 2026-10-23: option B,
  the side slot; no ADR). The file concerns and the reports stay in the idea
  for `file-concerns`.

## Goal
The six long functions the idea names that are really long are phases over a
context or tables per op family, none past 150 lines: the STEP reader's
`build`, the corpus's `build_step` (in a `corpus/` module tree, no file past
1000 lines), `prop::recipe::write`, `tessellate_under`,
`boolean_each_witnessed` and the `Interferences` dump (its own file). The
oracle's `recipe.py` cannot miss a recipe op the Rust `Step` names: a test
fails. The review scripts measure what they claim. Every blessed `dump.txt`,
`fixture.json`, `cancel_counts.txt`, `tolerance_band.txt`, id and refusal
string is byte-identical to the start.

## Non-goals
- No behaviour change; a defect found while moving code becomes a regression
  fixture and a backlog line, never a fix in the same commit.
- No public type or signature changes (all crate-private or `arris-debug`
  internals no consumer reaches); no `CHANGELOG.md`.
- The idea's *Leave these alone* list (tracers, blend phase tree, `ops/error.rs`,
  `part21::lex`, `arris-math`) and the other functions past 150 lines
  (`survey`, `fixtures::lint`, `placements`, the blend and pave ones): each
  is a backlog line only if a feature next grows it.
- **`file-concerns`** (next milestone plan): `py/src/model.rs` as thin
  `#[pymethods]` over `model/{ops, query, io, topology}.rs`; `check/check.rs`,
  `mesh/cdt.rs`, `geom/pcurve.rs` split by concern; the check tests by topic;
  one markdown table writer for `census`, `histogram`, `gaps`; `nurbs_census`
  deleted (Q5). The `arris-debug` split stays its own idea and ADR.

## Design deltas
- **The idea's measurement was partly an artefact.** `_rust.py`'s `blank`
  leaves char literals, so `'{'`/`'('` in `differential::refusal`'s `head`
  (24 lines, not 711) and in `part21::lex` (not 469) unbalanced the brace
  count. Read at `b257bfe0`: there is already one recipe enum,
  `fixtures::Step` (`serde(tag = "op")`); `build_step` matches it with no
  `_ =>`, and `prop::recipe::write` *produces* steps from random shapes, it
  does not interpret them. So "one `Op` enum matched by all three" is done;
  what is left is the Python side, which nothing ties to `Step`.
- **`Step::TAGS` and `Step::tag(&self)`**: the serde tag of each variant,
  matched exhaustively (a new variant fails to compile there). A test asserts
  `tag()` equals the `op` serde writes for one step of each variant, and that
  every tag appears as an `op == "<tag>"` branch of
  `tools/oracle/oracle/recipe.py` (read as text; no Python run).
- **STEP `build` → `reader/topology/build/`**: a `Build<'a>` context (the reader, the
  model, the file solid, `points`, `curves`, `surfaces`, `edges`, the id
  maps, `splits`, `rebuilt`, the meter) and one method per phase as the body
  reads now — `edges` (points and curves), `singular_splits`, `turned`,
  `walk_shell` (itself split by face and loop), `propagate_splits`,
  `assemble` (gaps, uses, slots), `grown`, `provenance`. `build` calls them
  in order. The pattern is `blend::build`'s.
- **`corpus/`**: `mod.rs` (`CorpusError`, `Stage`, `Chain`, `run`, `build`),
  `stages.rs` (the `*_stage` checks), `read_back.rs`, `inputs.rs` (`Inputs`,
  `number`/`point`/`vector`/`motion`/`sweep_path`), and `step/` with
  `build_step` as one dispatch over per-family functions: `solids.rs` (box,
  cylinder, polyhedron, read), `sketch.rs` (profile, planar face, extrude,
  revolve, sweep, loft), `motion.rs` (transform, mirror, split),
  `boolean.rs` (fuse, common, cut), `features.rs` (fillet, chamfer, offset,
  shell). Paths `docs/ARCHITECTURE.md` and the `inspect` skill quote keep
  resolving (`arris_debug::corpus::…` re-exports unchanged).
- **`prop::recipe::write`**: one function per operand `Shape` (box,
  cylinder, extrude, rim, turned, revolve) returning its steps and local
  probes, and one for the ops between operands; `write` loops and places.
- **`tessellate_under`**: its three phases (edge counts, edge samples, face
  work) as functions over a `Tessellation` context; the debug check stays
  first.
- **`boolean_each_witnessed`**: select, shells, assembly, provenance as
  functions; `impl Display for Interferences` moves to `boolean/dump.rs`.

## Steps
Complexity grades: **[1]** routine — the design says exactly what to write
and the tests are mechanical; **[2]** careful — a geometric or numeric case
to get right within a given design; **[3]** unproven — an algorithm whose
robustness or bound has to be established here.

- [x] Step 1 **[1]** — Record the start commit here. `_rust.py` blanks char
  literals (not lifetimes); `fn_lengths.py` re-run, its numbers in the
  commit body; the idea's table corrected.
- [x] Step 2 **[1]** — `Step::TAGS`/`tag()` and the serde and `recipe.py`
  test.
- [x] Step 3 **[2]** — STEP `build` as `Build` and its phases; the
  `real_*`, `step/` and healing corpora's dumps unchanged.
- [x] Step 4 **[2]** — `walk_shell` split by face and loop; no function in
  `reader/topology/build/` past 150 lines.
- [x] Step 5 **[1]** — `corpus/` with `mod.rs`, `stages.rs`, `read_back.rs`,
  `inputs.rs`; moves only.
- [ ] Step 6 **[1]** — `corpus/step/` per family; `build_step` a dispatch;
  no `corpus/` file past 1000 lines.
- [ ] Step 7 **[1]** — `prop::recipe::write` per shape and op family; the
  property tests' recipes (seeded) serialise byte-identically.
- [ ] Step 8 **[2]** — `tessellate_under` by phase; the mesh corpus and
  `mesh/` dumps unchanged.
- [ ] Step 9 **[1]** — `boolean_each_witnessed` by phase, `boolean/dump.rs`;
  `fn_lengths.py` shows none of the six past 150 and no new one.

## Acceptance
- `git diff <start> -- tests/fixtures` is empty.
- The step-2 test passes, and fails when a tag is removed from `recipe.py`
  (checked by hand once, said in step 2's body).
- `cargo nextest run -p arris-io -p arris-mesh -p arris-ops -p arris-debug`
  green with `real_*`; the full gate (`tools/full-gate.sh`) green at
  `/retire-plan`.
- `fn_lengths.py`: the six functions under 150 lines; the count past 150
  lower than step 1's by at least six.

## Docs to update on completion
- `docs/ARCHITECTURE.md` — the `arris-debug` and `arris-io` module lists, if
  they name `corpus.rs` or `reader/topology.rs`'s `build`.
- `.agents/skills/inspect` and `docs/ARCHITECTURE.md` §Formats and tools —
  any path to `corpus.rs` becomes `corpus/`.
- `docs/ideas/refactor-after-c10.md` — cut to `file-concerns`; `Status`
  names `long-functions` absorbed and retired.
- `AGENTS.md` current state — the refactor sequence line, `file-concerns` next.
- `docs/BACKLOG.md` — any defect found while moving; the other functions
  past 150 if the plan's reading finds one a feature is about to grow.
- `GLOSSARY.md` — no new term expected.

## Open questions
- Settled (agent): the plan is split; the file concerns and the reports go
  to `file-concerns`, because together they run to ~16 steps.
- Settled (agent): the Python lint is a Rust test reading `recipe.py` as
  text, not a Python script — the hook runs it, and `Step` is the source.
- Settled (agent, step 3): `Build` owns the `&mut Model` for the whole read; the
  reader's entities are only used in `new`, so no borrow competes.
- Settled (agent, step 3): the module is `reader/topology/build/`, a child of
  `topology`, not `reader/topology/build/`: the helpers and types it uses are private
  to `topology` and a child sees them with no visibility change.
