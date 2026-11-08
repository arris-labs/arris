# Idea: local-gate-diet

- Status: Accepted (A, B, C, D1, D2); to be planned as plans/local-gate-diet
- Raised: 2026-11-09
- Prompt (verbatim from the human): "what about tests that slows down development and waste compute on this PC? Can we cut them down somehow too (pre/post-commit hooks, full gate runs on retirement and closing cycle, etc.)?" — then, on the five proposals: "yes, /idea 1-4" (the fifth, the retirement's full gate moved to CI, left out).

## Problem

Measured from `.git/arris-verify/*.log`, 2026-10-31 to 11-09 (152
commits, 93 of them code commits with a verification):

| What | Runs | Each | Total |
|---|---|---|---|
| verification, whole `fast` profile (2910 tests, 16 cases, 16 threads) | 64 | ~10 min | **10.2 h** |
| verification, a narrower slice | 29 | 13 s – 7 min | 2.2 h |
| full gate (`/retire-plan`, `/close-cycle`, `/release`) | one per plan and per boundary | ~47 min (nextest 2,780 s, ADR-0081) | |

Two thirds of the verifications test the whole workspace, and some of
them cannot catch anything:

- **Paths no local test reads go whole.** `tools/gate.sh` sends any path
  it cannot place to `whole=yes` (ADR-0032: "a path it cannot place, the
  whole `fast` profile, never less"). `plans/nightly-panics` step 8's five
  commits changed `nightly.yml`, `.config/nextest.toml` and the plan. Each
  ran 2910 tests for 608–624 s: 51 min for YAML that no local test reads.
  `.github/workflows` was in 7 whole runs over the 9 days, `tools/oracle` in
  11, `.config/nextest.toml` in 3. `tests/fixtures/regression/*` falls to
  the same default.
- **A crate's source reaches its reverse dependencies, and for `arris-ops`
  that is almost everything.** `arris-io`, `arris-mesh`, `arris-debug` and
  `arris` (and `arris-topo`'s tests) depend on it, and most module rules
  in `gate.sh` add `corpus=all`. `crates/arris-ops/src` was in 55 of the 64
  whole runs, `arris-geom/src` in 26. These are the legitimate whole runs,
  and the expensive ones.
- **The full gate runs again on a tree it already passed.** `/close-cycle`
  follows the last `/retire-plan` with only docs changed in between, and
  `/release` follows `/close-cycle`. Each runs the full gate "now"
  (`close-cycle` step 1, `release` step 1): up to two more 47-minute runs on
  the same code.
- **The hook waits.** ADR-0078 §7: a code commit that arrives while
  `HEAD`'s verification runs waits for its verdict. 19 of the 93 code
  commits came within 10 min of the previous one and waited for the rest of
  its run.

## Constraints it runs into

- **ADR-0032**: "a path it cannot place, the whole `fast` profile, never
  less", and `tools/gate-test.sh` asserts every rule. Option A *places*
  more paths rather than weakening the default; the default stays.
- **ADR-0078 §4 and §7**: a pushed tip is verified; one verification
  runs at a time, for memory (§6). Option C keeps both and changes only
  §7's wait.
- **ADR-0081**: the full gate runs on the tip's own tree. Option B reuses
  its verdict, keyed by that tree.
- **ADR-0069 / `tools/ci-reuse.sh`**: the precedent for B. A diff from a
  green commit made only of paths no test reads lets that commit's verdict
  stand.
- **`/retire-plan` already skips** the full gate for a plan whose diff
  reaches none of `crates/`, `tests/`, `fuzz/`, `Cargo.*`, `clippy.toml`,
  `rust-toolchain.toml`, `.config/`, `.githooks/`, `tools/*gate*`. B's
  "code paths" are that list, so the two rules cannot disagree.
- **Kernel rules** are untouched. Every test still runs at the full gate,
  in CI (1000 cases) and at night; this is only about which commit runs
  what locally.

## Options

### A — Place the paths no local test reads
New rules in `tools/gate.sh`, each with a `gate-test.sh` case:
- `.github/*`: no tests; `actionlint` when installed.
- `.config/nextest.toml`: `tools/profile-test.sh` (it holds the `fast`
  filter to the slow set) and `cargo nextest list` under each profile. A
  changed test group or thread count is left to the full gate.
- `tests/fixtures/regression/*`: the corpus lint and that fixture's
  `regression_*` test by name.
- `tools/oracle/*`: `package(arris)`'s corpus and the oracle's selftest,
  not the whole profile. The oracle's sources are in the cache key
  (ADR-0024 §1), so the corpus re-asks the oracle.

Cost: 2 steps. Saves 10 min on each such commit (about 20 in these 9 days)
at no loss: none of these paths is read by a test the rule leaves out.

### B — Reuse a full gate's verdict for an identical code tree
`tools/full-gate.sh` records `pass` against the sha it gated. Before
running, `/close-cycle` and `/release` ask for the nearest gated ancestor
whose diff to `HEAD` is outside B's code paths, and skip with that sha in
the reply. Cost: 2 steps (the record and lookup with a self-test, and the
two skills). Saves up to 94 min per cycle boundary. Risk: a stale build
poisoning a reused verdict (the backlog's d398a43b line) is no worse than
today, since the verdict was taken on the verification's own tree.

### C — Don't wait: one verification running, the newest queued
When a code commit arrives while `HEAD`'s verification runs, its hook
does not wait. It queues itself over `since <last verified>`, replacing
any commit already queued, which then reads `covered by <sha>`. One
verification still runs at a time, so ADR-0078 §6's memory bound holds.
`pre-push` still refuses an unverified tip. Cost: 2–3 steps
(`verify.sh` and `post-commit`, `hook-flow-test.sh`, the ADR amending
§7). Saves the agent's waits (19 of 93 commits) and every superseded
run. Trade-offs: a red that covers several commits names no single one,
so the fix's attribution needs a bisect, rare at today's red rate. A red
`HEAD` is met at the next commit's hook as now.

### D — What a whole run spends, and two cuts
Timed 2026-11-09 at `56a1d585`: the `fast` profile at 16 cases on 16
threads, 2910 tests in **677 s**, 9,236 s of test time (a 577 s floor at
16 threads, so the wall is throughput, not one long test):

| Share | Test time |
|---|---|
| `arris::corpus`, 507 tests | **5,983 s (65%)**: blend area 2,841 s, boolean 1,496, loft 433, sweep 301, offset 269, the rest under 250 each |
| every property binary together | about 2,400 s; `twin_prop`'s `meridian_rows` 8 shards 826 s |
| 37 single tests (not shards) at 30 s or more | 2,752 s (30%); the slowest `blend_nist_ctc_02_ap242_ellipse_chain_fillet` 368 s, `horn_census` 213 s |

So a module map over test binaries (the first reading of D) buys little.
The corpus is the cost, and it runs whole for every `src/blend/*` or
`src/sweep/*` change (`corpus=all`). Two cuts:

- **D1 — Refresh the slow set by its own rule.** ADR-0066: a single test at
  30 s or more at the hook's counts leaves `fast` for the full gate, CI
  and the nightly. The set was last refreshed on 10-20 (`5878341d`). C11
  has since added the 37 tests above, which the rule already puts out. No
  new decision: `tools/test-timings.sh --slow-set` and its edit, 1 step.
  The times above were taken under 16-thread contention, as the rule's
  own run is, so its run decides the names.
- **D2 — A blend change reaches the fixtures whose recipe blends.** No
  module of `arris-ops` outside `src/blend/` calls into it (only `lib.rs`
  re-exports), so a blend change can alter only a recipe that runs
  `fillet` or `chamfer`. That is 215 of the 576 fixtures (181 in blend,
  34 across 10 other areas), read from each `fixture.json`'s `steps`, so
  a new fixture is placed with no list to edit. `src/blend/*` then stops
  setting `corpus=all`, and the boolean, loft, sweep and other areas'
  non-blending fixtures (about 3,000 s) leave a blend commit. The same
  rule does not help `sweep/`: `extrude` is in nearly every recipe.
  Cost: 2 steps (the recipe reader in `gate.sh`'s corpus filter, a
  `gate-test.sh` case, and the rule that a module called from outside its
  directory keeps `corpus=all`).

Together they roughly halve a blend commit's verification; D1 alone cuts
every whole run by about a quarter. Risk: D1's tests run only at the
full gate and CI, which is the slow set's existing trade. D2 misses a
regression only if blend code gains a caller outside `src/blend/`, and
the rule is to check for that.

### Do nothing
About 1.1 h a day of whole-workspace verification on half the machine, a
third of it on commits no test could fail, plus up to 94 min of repeated
full gates at each cycle boundary.

## Recommendation

**A, B, C, D1 and D2, one plan of about 9 steps.** A and B lose no
coverage at all: every run they drop tests code its last run already
tested. C loses only the attribution of a red to one commit inside a
coalesced range. D1 applies ADR-0066's existing rule. D2 is the only one
that can miss a regression, and only if blend code gains a caller outside
`src/blend/`, which its rule checks for.

## Decision for the human

Round 1:

❓ **Q1 — Place the paths no local test reads** (A): `.github/*`,
`.config/nextest.toml`, `tests/fixtures/regression/*`, `tools/oracle/*`,
each with its rule and a `gate-test.sh` case; the whole-profile default
for any other path stays.
➡️ Yes — 10 min per such commit, nothing left untested.

❓ **Q2 — Reuse a full gate's verdict** (B): `/close-cycle` and `/release`
skip the full gate when `HEAD`'s diff from a gated commit is outside the
code paths `/retire-plan` already lists.
➡️ Yes — up to 94 min per boundary on a tree already gated.

❓ **Q3 — Coalesce verifications** (C): (a) the hook does not wait;
the newest commit queues over `since <last verified>`, replacing a queued
one; (b) keep ADR-0078 §7's wait.
➡️ a — one run at a time still holds, and only a rare red loses its single-commit name.

❓ **Q4 — The word**: if Q3 is (a), add to `GLOSSARY.md` beside
*Verified*: **Covered** ("a commit whose own verification was replaced by
a later commit's over a range that includes it; it takes that run's
verdict, and a red names the range, not the commit").
➡️ Yes — `--status` and `pre-push` need a name for the state.

ADR: one new ADR amending ADR-0032 (the placed paths) and ADR-0078 §7
(the wait), written by the plan; B's reuse rides on ADR-0069's reasoning
and goes in the same ADR.

Round 2 (the timings, D above):

❓ **Q5 — D1, refresh the slow set**: rerun ADR-0066's
`--slow-set` rule and move what it prints out of `fast`, as one step of
this plan.
➡️ Yes — the rule already says so; it was last applied before C11's tests existed.

❓ **Q6 — D2, a blend change reaches the blending fixtures**: (a)
`src/blend/*` reaches the fixtures whose `steps` run `fillet` or
`chamfer` (215 of 576), read from the recipes, with `corpus=all` kept
for any module called from outside its directory; (b) keep `corpus=all`
for `src/blend/*`.
➡️ a — about 3,000 s off every blend commit, and the reach is read from the recipes, not hand-listed.

*Later*: none.

## Decision

2026-11-09, the human: "yes" to round 1.

- Q1: A — the four paths placed, each with its `gate-test.sh` case; the
  whole-profile default for any other path stays.
- Q2: B — `/close-cycle` and `/release` reuse a full gate's verdict when
  `HEAD`'s diff from the gated commit is outside `/retire-plan`'s code
  paths.
- Q3: a — the hook does not wait; the newest commit queues over
  `since <last verified>`, replacing a queued one.
- Q4: *Covered* added to `GLOSSARY.md`.
- ADR: one new ADR amending ADR-0032 and ADR-0078 §7, written by the plan.
2026-11-09, the human: "yes" to round 2.

- Q5: D1 — the slow set refreshed by ADR-0066's rule, one step of the plan.
- Q6: a — `src/blend/*` reaches the fixtures whose `steps` run `fillet`
  or `chamfer`, read from the recipes; `corpus=all` kept for any module
  called from outside its directory.
