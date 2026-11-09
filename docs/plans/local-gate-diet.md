# Plan: local-gate-diet

- Started: 2026-11-09
- Milestone: side slot of C12 (docs/ROADMAP.md); tooling, no kernel change
- Idea: docs/ideas/local-gate-diet.md (absorbed; `git show 7fa41626:docs/ideas/local-gate-diet.md` holds its measurements), decided A, B, C (Q3 a), D1, D2 (Q6 a)

## Goal
A local verification runs only what a commit can change: YAML runs no test, a
blend change runs the fixtures that blend, and `fast` holds what ADR-0066's
rule says. A code commit never waits for an earlier commit's verification: one
run at a time, the newest commit queued over everything since the last
verified one. `/close-cycle` and `/release` reuse a full gate taken on the same
code. A pushed tip is verified, as before.

## Non-goals
- Moving `/retire-plan`'s full gate to CI (left out by the human).
- Placing `.githooks/*`, `tests/fixtures/{geom,sheet}/` or any path the idea
  did not name; a module map over `sweep/` (`extrude` is in nearly every recipe).
- CI, the nightly and the full gate's contents: every test still runs there.

## Design deltas
Tools only: no public type, signature or crate boundary changes. **ADR-0085**
(new) amends ADR-0032's unplaced default and ADR-0078 §7, citing ADR-0066,
ADR-0069, ADR-0081.

**Corrections to the idea, each checked in the code:**
1. Blending fixtures are read from the whole `fixture.json`: 13 part fixtures
   hold their blends under `battery` (238 files; `steps` alone finds 225).
2. A row's name comes from `corpus.rs` by its `"area/dir"` string: 6
   directories have several rows (`provenance_split_bar_cut_left`).
3. `regression/` needs more than its test by name: all 79 rows are `#[ignore]`,
   so that term runs nothing; other tests are the readers (delta A).
4. `tools/oracle/*` reaches beyond `package(arris)`: `arris-debug`, 8 `arris-io`
   binaries and 3 `arris-ops` binaries call the oracle (delta A).
5. B's code paths gain `tools/oracle/`, whose sources are in the cache key.

**C, `tools/verify.sh`.**
- The base is computed after the lock: the nearest `pass` on the first-parent
  walk from `<sha>^`, at most 200 commits, else the empty tree (the hook's walk,
  moved here). With a verified parent this is `<sha>^`, as today.
- `arris-verify/newest` names the queued commit, under a short
  `queue.lock`. A new run whose predecessor there is still `queued` **and its
  ancestor** writes `covered <new-sha>` to it and takes its place; another
  line's commit is left alone. A waiter that gets the lock and reads `covered`
  exits, logging `covered by <sha>`. `queued` → `running` is under `queue.lock`.
- `--status` follows `covered` and still prints one word, so its readers are
  unchanged. `--in-flight <sha>` exits 0 while `queued` or `running` with a live
  runner, without waiting.

**C, `.githooks/pre-commit`.** HEAD in flight: `deferred`, no wait.
`ARRIS_VERIFY=inline` still waits; `fail` or `none` runs inline as today.

**D2, `tools/gate.sh`.** `src/blend/*` adds area `blend` and the rows of every
fixture outside `blend/`, `real/`, `regression/` whose file has a fillet or
chamfer op (34 today; `real_*` is out of `fast`, regression rows are ignored),
as `test(/^(name|…)$/)` beside the prefix regex. **The guard runs at
classification:** any `blend::`, `crate::fillet` or `crate::chamfer` under
`crates/arris-ops/src` outside `blend/`, or a `lib.rs` re-export other than
`pub use blend::{chamfer, fillet};`, sets `corpus=all`.

**A, `tools/gate.sh`** (before the `tools/*/*` arm; `*.md` stays prose):
- `.github/*`: no nextest; `tool_tests` gains `tools/workflows-test.sh` (new:
  `actionlint` when installed, else a line saying so).
- `.config/nextest.toml`: no nextest; `tool_tests` gains `tools/profile-test.sh`.
- `tests/fixtures/regression/<slug>/*`: `binary_id(arris::corpus_lint)`,
  `test(=every_fixture_directory_has_a_corpus_test)`, and each file naming
  `regression/<slug>` (`git grep -F` under `crates/`): a `tests/<f>.rs` gives
  its binary, anything else its package.
- `tools/oracle/*`: `package(arris)` with the corpus, `package(arris-debug)`,
  and the binaries whose non-comment lines call `oracle::`, `compare_stl`,
  `corpus::run`, `part::run` or `query::run`, read by `git grep`. `tool_tests`
  gains `tools/oracle-test.sh` (new: `selftest.py` over `primitive/`; the
  11-minute whole selftest stays CI's).

**B, `tools/gate-reuse.sh`** (new, with its `-test.sh`):
- `--record`: writes `arris-verify/gated/<HEAD>`, refused when a code path is
  dirty; `tools/full-gate.sh` calls it when no stage failed.
- No argument: the nearest recorded ancestor of HEAD whose diff to HEAD reaches
  no code path, exit 1 when none. A swept record costs one more gate, never a
  wrong reuse.
- `--reaches-code <a> <b>`: the list's one home, `/retire-plan`'s today plus
  `tools/oracle/`, `tools/check-*` and the non-`.sh` files of `tools/`
  (`real-parts.waits` is read by tests).

**D1.** `.config/nextest.toml` and `tools/profile-test.sh` take what
`tools/test-timings.sh --slow-set` prints (ADR-0066).

## Steps
A step's grade is its tier (`.agents/tiers.md`): **[1]** routine, the design
says what to write; **[2]** careful, a case to get right within a given design;
**[3]** unproven. `**[2, tier 3]**` overrides the tier.

- [x] Step 1 **[1]** — ADR-0085 and its README row, from the deltas and the
      idea's measurements.
- [x] Step 2 **[2, tier 3]** — `verify.sh`: the base under the lock, `covered`,
      `--in-flight`. `verify-test.sh`: three runs leave the middle `covered`,
      reading the last's verdict; a covered red reads `fail`; another line's
      queued commit is not covered; the stub sees the wider base after a red.
- [x] Step 3 **[2]** — `pre-commit` does not wait. `hook-flow-test.sh`: a commit
      over a running HEAD returns first; a prose commit between them; inline
      still waits; `pre-push` passes a covered commit whose run passed.
      `git.md`; `GLOSSARY.md` *Covered*, *Verified*, *Gate*.
- [x] Step 4 **[2]** — D2's reader, row lookup and guard. `gate-test.sh`, in a
      scratch tree: a blending fixture outside `blend/` is reached, a
      non-blending one is not, a variant fixture reaches every row, a caller
      outside `blend/` gives `corpus=all`, `sweep/mod.rs` keeps it.
- [x] Step 5 **[1]** — `.github/*` and `.config/nextest.toml` placed;
      `workflows-test.sh`; `gate-test.sh`'s `unplaced` list loses both and
      asserts `none()` with the right `tool_tests`.
- [x] Step 6 **[2]** — `regression/` and `tools/oracle/*` placed;
      `oracle-test.sh`; `gate-test.sh` asserts `arris-ops::boolean` for
      `regression/ball-beside-pole-slice-cut`, `arris-io::step` and no
      `twin_prop` for an oracle file; `regression` leaves `whole_dirs`.
- [x] Step 7 **[2]** — `gate-reuse.sh` and its test (a docs commit reuses, a
      code commit does not, a dirty code path refuses `--record`, a red gate
      records nothing); the call in `full-gate.sh`.
- [x] Step 8 **[1]** — `/close-cycle` and `/release` step 1 run `gate-reuse.sh`
      first, name the sha they reuse, and run the gate at once when it prints
      none; `/retire-plan` step 1 cites `--reaches-code` for its list.
- [ ] Step 9 **[1]** — D1: `--slow-set` in the background, then the two lists.
      The body gives the whole `fast` run's seconds before and after, a record
      and not a gate (ADR-0024 §4).

## Acceptance
- `tools/{gate,verify,hook-flow,profile,gate-reuse,oracle}-test.sh` pass.
- `tools/gate.sh plan .github/workflows/nightly.yml` prints `filter=none()`.
- `tools/gate.sh plan crates/arris-ops/src/blend/stripe.rs`, its filter given to
  `cargo nextest list --profile fast -E`: the corpus tests listed are the rows a
  separate JSON walk of `steps`, `variants`, `battery` and `solids` finds, plus
  the `blend_` area and the base rows. No other corpus row.
- `/retire-plan`'s full gate writes its record, and `tools/gate-reuse.sh` on the
  retirement commit prints that sha. The gate is green and HEAD reads `pass`.

## Docs to update on completion
- `docs/ARCHITECTURE.md` *The gate's tiers* — placed paths, blend's reach,
  covered runs, the reused verdict.
- `tools/gate.sh`'s table, `.githooks/pre-commit`'s header, `.agents/rules/git.md`.
- `docs/adr/0032-lean-gate.md`, `0078-…md` — `Amended by: ADR-0085`.
- `docs/BACKLOG.md` — a `.githooks/*` line: whole profile, read only by the
  shell tests the static half already runs.
- `GLOSSARY.md` — *Covered* (step 3). `AGENTS.md`, `CHANGELOG.md`: no change.

## Open questions
- The agent's picks, for the human to overturn before `/work`: corrections 3–5.
  Q1 said "nothing left untested", so where the idea's rule left a reader out,
  the rule grew. The oracle's rule saves about the properties' share of a whole
  run (2,400 of 9,236 s), not the 10 min the idea counted.
- `--status` on a covered commit prints its run's verdict; the state file and
  log carry `covered <sha>`. Every reader compares to `pass`, and Q4 says a
  covered commit takes that verdict.
