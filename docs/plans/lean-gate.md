# Plan: lean-gate

- Started: 2026-09-30
- Milestone: beside the cycles — the measuring harness's process side
  (docs/ROADMAP.md §Beside the cycles); no cycle line of its own
- Idea (verbatim from the human): "I want to implement this … review what
  tests, precommits, etc. do we run at each skill and step — we have a lot
  of heavy tests and I want us to keep lean and run only necessary ones
  during plan implementation and move as much as possible to CI, nightly
  runs, end of cycle implementation or release"
- No idea file: the human decided in conversation after a measured review
  (recorded under *Context*); the ADR in step 4 carries the reasons.

## Context (measured 2026-09-30, 32-core dev machine)

- Full suite, `cargo nextest run --workspace`: **315 s wall, 9,235
  CPU-s, 1511 tests**. Shares of the CPU: `boolean_prop` 41%, the corpus
  22%, `arris-io::body` round-trip properties 10.5%, `cancel_prop` 5%,
  `tolerance_band` 3.5%, `inertia` 3%.
- The 23 `real_*` corpus tests (committed NIST excerpts under
  `tests/fixtures/real/`, *not* `tools/real-parts.sh`'s downloaded survey)
  are 643 CPU-s; `real_nist_ctc_04` alone takes 170 s, `real_nist_ftc_07`
  157 s. Other slow singles: `arris-io::body guard::coverage` 93 s, the
  part-battery test ~42 s, `step_read boolean_fixtures_read_back` ~42 s.
- At `ARRIS_PROPTEST_CASES=32` without `real_*`: **118 s wall, 2,656
  CPU-s**. The corpus is then most of it (1,415 CPU-s at 32 cases:
  `boolean_*` 977, `provenance_*` 163, `transform_*` 107, `sweep_*` 97,
  `blend_*` 47).
- Other gates, warm: `cargo doc` ~31 s, doctests ~4 s, fmt, clippy and the
  layer check under 1 s each.
- **The same gate runs twice per step.** `/work` step 3 tells the agent to
  run fmt, clippy, `cargo test --workspace` and `cargo doc`; the
  pre-commit hook then runs all of it again on the commit.
- **ADR-0024 §3 decided "the hook keeps 256"**, after the seam
  parametrisation fault passed the hook and a 0.1.0 publish was half out.
  Its reasons — depth comes from nightly seeds, CI is the first catch,
  a release waits on CI's run of the tag — still hold, but the decision
  weighed a hook that "costs minutes more", not one that costs five
  minutes twice. This plan reverses it openly, in an ADR, not silently.

## Goal

A plan step costs what its own change needs. `/work` runs the step's own
tests and nothing else; the pre-commit hook is one fast, path-scoped gate
(layers, fmt, clippy, doctests, the `fast` nextest profile at 32 cases
over the crates the staged paths reach, `cargo doc` only when a doc
comment changed), about 40–120 s instead of about 350 s; the full profile
— 256 cases, every corpus area, `real_*`, `cancel_counts`, `parallel` on
and off, `cargo doc`, the oracle selftest, the wasm build — runs once, at
`/retire-plan`, and again in CI, nightly, `/close-cycle` and `/release`
as today. A path the classifier cannot place falls back to the whole
`fast` profile, never to less, and `ARRIS_GATE=full` forces the old gate.
`main` is still green at every pushed tip: CI, which the human reads
before any push that matters and which the release waits on, is the net
under the faster hook, and ADR-0032 says so.

## Non-goals

- No change to CI's jobs or case counts (`ci.yml` keeps 1000 cases and
  runs `real_*`), to nightly, or to the case count or seeds of any
  property's own definition.
- No change to what any test asserts; a test is excluded from a *profile*,
  never weakened or deleted.
- No time gate: ADR-0024 §4 (time is never a gate) stands; the `fast`
  profile is chosen by measured cost once, not enforced by a timer.
- No new test runner and no `sccache`/`mold`/linker tuning; build time is
  out of scope (the measurement shows the suite, not the compile, is the
  cost).
- No change to the prose-only or version-bump-only hook paths.

## Design deltas

- **`.config/nextest.toml` (new):** profile `fast` whose `default-filter`
  excludes `real_*` and a named *slow* set (step 1 fixes the set from the
  measurements: `guard::coverage`, the part-battery test,
  `boolean_fixtures_read_back`, `cancel_counts`'s corpus-wide test, and
  any other single test over ~40 s at 32 cases); profile `full` is the
  default profile's behaviour, named so a skill can say it. The default
  profile is unchanged, so CI's plain `cargo nextest run` is too.
  Per-profile environment is not a nextest feature, so the case counts
  are exported by the hook script, not the profile.
- **`tools/gate.sh` (new):** `tools/gate.sh plan <path>…` prints the
  gate the paths need as `key=value` lines — `filter` (a nextest filterset:
  `rdeps(<crate>)` of the changed crates, corpus areas added or not),
  `cases`, `diff_cases`, `doc` (yes when the staged diff adds a `///` or
  `//!` line, or a path is a `.md` under `crates/`), `full`
  (`ARRIS_GATE=full`); `tools/gate.sh` with no arguments reads the staged
  paths. Classification: a path under `crates/<c>/` reaches `<c>` and its
  reverse dependencies; `crates/arris-{math,geom,topo,check,io}` or
  `arris-ops/src/boolean` adds corpus `boolean_*` and `provenance_*`;
  `arris-ops/src/{sweep,blend,transform,mirror}` adds that area's
  corpus tests; `crates/arris-debug/` and `crates/arris/` add the whole
  corpus; `Cargo.toml`, `Cargo.lock`, `.config/`, `tools/`, `fuzz/`,
  `tests/fixtures/` (by area directory) and any path it cannot place fall
  back to the whole `fast` profile. It is the only place the rule lives.
- **`.githooks/pre-commit`:** the non-prose, non-bump branch calls
  `tools/gate.sh` and runs what it says; its header states the tiers.
  The prose-only and version-bump branches are untouched.
- **Skills:** `work` step 3 becomes "the step's own tests" (and, for a
  geometric change, the corpus area it touches); `retire-plan` gains the
  acceptance run and loses the rebuild in its sweep step (`cargo test
  --no-run` stamps the same set); `close-cycle` and `release` keep the
  full gate and CI/nightly checks, worded against the new hook.
- **ADR-0032, lean gate** (step 4): supersedes ADR-0024 §3's hook row
  ("pre-commit hook | 256") and its "The hook keeps 256" paragraph only;
  the rest of ADR-0024 stands. States the tiers, why the seam-fault
  incident does not recur by this route (the ADR-0024 reasoning: the
  fault sat past case 32 *of one fixed seed*, which 256 and 32 both
  share — depth is nightly seeds and CI's 1000; `/retire-plan` now runs
  256 once per plan), and the risk it accepts.
- **Rules and docs:** `.agents/rules/git.md` (the green-on-every-commit
  paragraph and the hook's description), `docs/ARCHITECTURE.md` (the
  layer-rule line that says the hook runs the script stays true; the
  tiers go where §Formats and tools describes the suite), `docs/ROADMAP.md`
  §Beside the cycles (the line stating the hook's 256 cases), the ADR
  index. `AGENTS.md` only if it restates the gate (it says the hook runs
  the suite under nextest: stays true, worded "a fast profile").
- **Public API, crate boundaries, `CHANGELOG.md`:** none. Process only.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — the profiles, measured: `.config/nextest.toml`
  with `fast` and `full`; `tools/test-timings.sh` takes `--profile` so
  the before and after compare like with like. Run it for `default` at
  256 and `fast` at 32 and record both tables in this plan under
  *Measured*, fixing the slow set from the numbers (anything over ~40 s
  at 32 cases that is not a shard of a property). Test: `cargo nextest
  list --profile fast` lists none of the excluded names and every other
  test the default lists (a script asserting the difference is exactly
  the named set, so a renamed test cannot silently leave or enter it).
- [ ] Step 2 **[2]** — `tools/gate.sh` and its test. `tools/gate-test.sh`
  feeds path sets (a geom-only change, an ops `boolean` change, an ops
  `mirror` change, a `crates/arris-debug` change, a fixture under
  `tests/fixtures/transform/`, a `.config/` change, a path in no known
  crate, prose plus code, an empty set, a doc-comment-only diff against a
  scratch repo) and asserts each printed gate: the filter names exactly
  the expected crates and areas, anything unplaceable yields the whole
  `fast` filter, and no input ever yields a gate smaller than its own
  crate's tests. The script is the classifier and nothing else; it runs
  no test itself.
- [ ] Step 3 **[2]** — the hook calls it. `.githooks/pre-commit`'s full
  branch: layers, fmt, clippy, doctests, `cargo nextest run --profile
  fast -E <filter>` with `ARRIS_PROPTEST_CASES` and `ARRIS_DIFF_CASES`
  from the gate, `cargo doc` only when the gate says so; `ARRIS_GATE=full`
  runs the old gate unchanged. The header comment states the tiers.
  Test: replay three real staged sets with `git stash`-free scratch
  commits — `466c619` (ops/topo), `2c52b27` (fixtures/debug/ops), and a
  prose-only one — and record each wall time under *Measured*; the
  `466c619` set must come in well under the old 350 s and the prose-only
  path must be unchanged.
- [ ] Step 4 **[1]** — ADR-0032 (lean gate) as designed above, the ADR
  index, `.agents/rules/git.md`, ROADMAP §Beside the cycles, ARCHITECTURE
  §Formats and tools, `AGENTS.md` wording. Docs only. The `docs_refs` and
  `doc_drift` tests run under the prose-only hook path.
- [ ] Step 5 **[1]** — the skills: `work` (step 3, the Don't list: no
  workspace suite, no `cargo doc` per step), `retire-plan` (the acceptance
  run, the lighter sweep stamp), `close-cycle` and `release` (their gate
  wording). Docs only; each skill edited once, the four descriptions'
  front matter unchanged.
- [ ] Step 6 **[2]** — the acceptance run below, once, recorded. It is
  the plan's own proof that the two-tier gate loses nothing the old one
  caught: the full profile green, and a *planted* fault in each of a
  property and a corpus area the `fast` hook skips is caught at
  `ARRIS_GATE=full` and named in the notes as what the tier defers to
  retire and CI, not hidden.

## Acceptance

- `tools/gate-test.sh` green; `tools/gate.sh` has no input that yields a
  smaller gate than the changed crate's own tests.
- `cargo nextest list --profile fast` differs from `default` by exactly
  the named slow set and `real_*`.
- The hook on the replayed `466c619` set (ops and topo, no doc lines) runs
  in under 120 s wall on the 32-core machine, the prose-only path is
  unchanged, and `ARRIS_GATE=full` reproduces the old gate's command list.
- `cargo nextest run --workspace` (the default profile, 256 cases) green,
  with `parallel` on and off; `cargo doc` and the oracle selftest green.
- `ci.yml` and `nightly.yml` unchanged, and the next CI run on `main` green
  in every job.

## Docs to update on completion

- `docs/adr/0032-lean-gate.md` and its index row (written in step 4).
- `.agents/rules/git.md` — the hook's gate and the green-on-every-commit
  wording.
- `docs/ARCHITECTURE.md` §Formats and tools — the tiers.
- `docs/ROADMAP.md` §Beside the cycles — the hook's case count.
- `AGENTS.md` — the setup comment on the hook; "Current state" only if a
  milestone line changes (it does not).
- `CHANGELOG.md` — nothing: no consumer can see a process change.

## Open questions

- `⚠ OPEN:` human, before step 4: this reverses ADR-0024 §3's "the hook
  keeps 256", taken after the seam fault shipped half a release. The
  plan's answer is that the hook's count never reached that fault (it sat
  past case 32 of the one fixed seed, which 256 shares) and that `/retire-plan`
  and CI now carry 256 and 1000; say so if you want the hook to keep 256
  on the crates it reaches instead (the gate's `cases` line is one number).
- `⚠ OPEN:` agent, by step 1: whether nextest's installed version
  supports `default-filter` per profile (0.9.80 and later do); if not the
  profile is a filter the script passes with `-E`, and the `.config` file
  shrinks to the test-group definitions.
- `⚠ OPEN:` agent, by step 2: whether `rdeps(<crate>)` in nextest's
  filterset addresses a *package* by its cargo name (`arris-ops`) — the
  ADR-0032 text fixes the spelling the test shows.
- Not a question, a correction to the review that led here: `real_*`
  stay in CI's `test` job (they are ordinary workspace tests), so nothing
  moves to nightly; only the hook stops running them.

## Measured

### Step 1 — the profiles (32 cores, 2026-09-30)

`tools/test-timings.sh --nextest-only` (default profile, 256 cases), the
slowest tests; its wall line (315 s, 1511 tests, 9,235 CPU-s) is the
*Context* measurement, the script's summary rows having been lost to a
`pipefail`/`head` fault fixed in the same step. `real_nist_ctc_04` 170 s,
`real_nist_ftc_07` 163 s, `body every_drawn_body_round_trips…::s6` 146 s
(a shard), `boolean_prop crossing_cylinders…::shard_0` 106 s (shards,
105–68 s, fill the rest of the top 25), `guard::coverage` 90 s.

`tools/test-timings.sh --profile fast` at `ARRIS_PROPTEST_CASES=32`
`ARRIS_DIFF_CASES=32`, run with the three-name first cut of the slow set
(`real_*`, `guard::coverage`, `boolean_fixtures_read_back`): **110 s wall,
1489 tests, 2,538 CPU-s.** Slowest single tests, all under contention:
`boolean_nist_ctc_04_face_arrangement_turn` 53 s, `boolean_pipe_elbow_…`
48 s, `boolean_ring_corner_common` 47 s, `boolean_revolve_extrude_…` 46 s,
`boolean_seam_beside_crossing_fuse` 41 s, `part::tests::a_battery_stage_
is_held_to_its_class` 41 s, `tolerance_band the_band_at_the_fixtures_
numbers` 37 s, `arris-ops::cancel a_budget_stops_every_operation…` 28 s.

**The slow set** (fixed from that table; `.config/nextest.toml`, held by
`tools/profile-test.sh`): `real_*` (20 tests), `guard::coverage`,
`boolean_fixtures_read_back`, `part::tests::a_battery_stage_is_held_to_its_
class`, and `arris-ops::cancel a_budget_stops_every_operation…`. The corpus
singles and `tolerance_band` stay in `fast` — they are the tests of the
crates that own them, and `tools/gate.sh` (step 2) leaves the corpus out
of a commit that cannot reach it; the set is not a list of everything
slow, it is what no commit's own change should pay for. 24 tests out.

Finding: nextest 0.9.144 supports `default-filter` per profile (the
first ⚠ OPEN); a test in a unit-test module is named by its whole path
(`part::tests::…`), an integration test by its function name.
