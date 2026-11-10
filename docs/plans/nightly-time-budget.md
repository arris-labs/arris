# Plan: nightly-time-budget

- Started: 2026-11-09
- Milestone: the side slot beside C12 (docs/ROADMAP.md)
- Idea: docs/ideas/nightly-time-budget.md (absorbed; decided 2026-11-09: A then C, B dropped)
- Runs on `plan/nightly-time-budget`: steps 4 and 8 are CI dispatches and
  need the branch pushed. Dispatch by day; the 02:17 UTC cron shares the 20 jobs.

## Goal
The nightly fits GitHub's limits by construction and its colour means
something. A property shard runs chunks of cases until its budget is spent,
each chunk replayable alone; 12 property jobs fit the 20 concurrent slots
and none is cancelled unreported. A failure with a known signature, naming an
open finding, is listed in the job summary and leaves the job green; only a
new one is red. The hook, CI and their fixed seeds do not change.

## Non-goals
Fixing any finding. Rotation (option B). A faster test build. The failure
capsule; signatures for fuzz crashes, which stay red. Anything posted outside
the run (ADR-0024 §6 holds that half). arris-gym as the differential.

## Design deltas
**ADR-0086** amends ADR-0024 §3 (the nightly's count is a budget, the tier's
count its ceiling) and §6 (red means a new signature).

**Harness** (`arris_debug::prop`, unpublished). `ARRIS_PROPTEST_BUDGET=<s>`,
read by `check` (shard 0 of 1, about 230 properties) and `check_shard_at`.
`CHUNK_CASES = 8`;
`chunk_seed = sha256(shard_seed ‖ chunk)`; the last chunk is cut at the
ceiling. Chunk 0 always runs; the loop stops before a chunk whose predicted
time (the slowest so far) would pass the budget. The clock is a parameter of
the `try_` form. A shard prints `prop-budget: ran N of M cases in T s (stop:
budget|ceiling)`; a failure names its chunk and prints `ARRIS_PROPTEST_SEED=…
ARRIS_PROPTEST_CHUNK=<i>`. Unset, today's path and streams are unchanged.

**`.config/nextest.toml`**: `[profile.nightly]`: `test-threads = "num-cpus"`,
a 2 h per-test cap (30m × 4), `global-timeout = "315m"` (JUnit is written even
at the limit), JUnit with `store-success-output`. The four elbow and standoff
overrides go. Step 7 adds `[[profile.nightly.overrides]]` `binary(differential)`
at `slow-timeout = { period = "30m", terminate-after = 8 }` (4 h), the cap the
differential had under the default profile: its two long tests hit the 2 h cap
in run 42724063416 (TIMEOUT 7200 s), which the triage read as new `timeout -`;
run 42659620551 finished in 229 min.

**The budget is 3600 s.** Run 42522745189 (2983 tests, 966,000
thread-seconds): 70 tests capped, sum 630,000, about 219 min a job over 12
jobs of 4 threads; with the uncapped tests 1.7× slower, 311 min, under the 330
left after setup. Per job: *budget = wall × threads ÷ capped tests*.

**Jobs**: 21 property jobs become 12, beside differential, 5 fuzz, real-parts
and bench: 20 once `seed` ends. Filters merge with `or`, packed
first-fit-decreasing on `sum(min(t, budget))`, the two `-rest` catch-alls
kept. The pack comes from step 4's run: three jobs were cancelled on 11-08,
so their unreported tests are unknown.

**Signature, sharpened** (`GLOSSARY.md`). Of 18 property failures on 11-08,
one carries a checker row, none a variant path, and every panic site is the
harness's `prop.rs:271`: the key as defined classifies one in eighteen. The
source is `binary::property`, shard suffix dropped (a property names its
operation). Class and key, first match wins:
1. `timeout`: nextest's status. 2. `abort`: `property aborted`.
3. `panic`, site `file:line`: first `panicked at` outside the harness's
   `prop.rs` and `differential.rs`.
4. `checker`, row: `checker report: <row>`.
5. `differential`, per `case N: <class>` line (`Disagree(counts)`).
6. `variant`: a typed token (`Internal(Split)`, `A::B`).
7. `assert`, the **stem**: the message up to its first digit, trailing
   punctuation cut. It is the test's format string, so no draw changes it; a
   reworded test reads as new, which errs red.

**`tools/nightly-triage.sh <junit> <known>`** (Python stdlib in a heredoc, a
top-level `.sh` so the gate's table needs no row): each failure new or known,
a passing known test quiet. It writes the job summary (`prop-budget` totals,
known by slug and count, new in full) and exits 1 on a new signature or a
missing JUnit, so a failed build stays red.

**`tools/nightly-known.txt`**, shaped as `tools/real-parts.waits`: `<slug>
<binary::property> <class> <key…>`. `known_signatures_name_open_findings` in
`crates/arris/tests/docs_refs.rs` (the prose gate runs it) holds each slug to
a §Findings heading, so a finding that goes takes its entry with it.

**`nightly.yml`**: the budget env, `--profile nightly`, nextest `|| true` with
triage deciding the colour, JUnit uploaded `if: always()`. The differential
drops `--no-capture`, which empties JUnit. `ci.yml` is not touched.

## Steps
Grades are tiers (`.agents/tiers.md`): **[1]** routine; **[2]** careful within
a given design; **[3]** unproven. **[CI]**: the only test is a CI run.

- [x] Step 1 **[1]** — ADR-0086, its amendment line in ADR-0024, *Signature*
  sharpened. Docs tests.
- [x] Step 2 **[2]** — The chunk loop in both entry points, its variables, the
  `prop-budget` line, rustdoc with an example. Fake-clock tests: stops before
  the overrun, runs chunk 0 past any budget, stops at the ceiling; a seeded
  failure names a chunk whose replay fails alone; unset, a stream equals today's.
- [x] Step 3 **[1]** — The profile, overrides removed, `nightly.yml` on it
  with `ARRIS_PROPTEST_BUDGET: "3600"` and JUnit; 21 jobs. Profile and
  workflow tests; JUnit holds a `prop-budget` line.
- [x] Step 4 **[1]** **[CI]** — Dispatch step 3; pack the 12 jobs from its
  per-test times, each with its figure in a comment. `cargo nextest list` over
  the 12 filters holds each test in exactly one (commit body). A job past
  300 min lowers the budget by the rule, and the commit says so.
- [x] Step 5 **[2]** — `tools/nightly-triage.sh` and its `-test.sh`, on a small
  JUnit of 11-08's message shapes, a differential failure, a timeout, a harness
  panic, a kernel panic and a pass: signature, new / known / quiet, exit code.
- [x] Step 6 **[1]** — `tools/nightly-known.txt` seeded from run 42522745189
  and step 4's; a failure with no §Findings block gets one first. The lint,
  with a scratch-tree case that catches a dangling slug.
- [x] Step 7 **[1]** — The triage step in the property and differential jobs,
  nextest `|| true`, `--no-capture` dropped. `tools/workflows-test.sh`.
- [ ] Step 8 **[1]** **[CI]** — Dispatch the whole workflow against the
  acceptance; a failure read as new gets its block and entry, or stands as a
  real new one. Run 42724063416 was the first dispatch: every acceptance line
  held, the listed `sweep_prop` timeouts admitted (Open questions); the
  differential's cap was repaired by 5a284f7b. One more dispatch confirms it.

## Acceptance
- `ARRIS_PROPTEST_CASES=16 cargo nextest run -p arris-debug -E
  'test(/budget|chunk/)'` and `tools/nightly-triage-test.sh` pass.
- Step 8's dispatch: 21 jobs, none `cancelled`; no `TIMEOUT` line for a test
  not listed in `tools/nightly-known.txt` in any property job's log
  (`gh api repos/arris-labs/arris/actions/jobs/<id>/logs`); every property job
  under 350 min with its `prop-budget` totals in the summary.
- Every failure is listed by slug or printed as new; the run is red only for a
  new signature or a fuzz crash. One listed failure replays alone by its
  printed `ARRIS_PROPTEST_CHUNK` line.
- `grep -n 'terminate-after = 11' .config/nextest.toml` is empty (today's hits
  are step 3's overrides); `git diff main -- .github/workflows/ci.yml` is empty.

## Docs to update on completion
- `docs/ARCHITECTURE.md` §`nightly.yml`; `docs/ROADMAP.md`'s "5000 cases …
  seventeen jobs"; `README.md`'s "five times CI's case count"; the header
  comment of `nightly.yml`.
- `tests/fixtures/README.md` §Property-test failures, `inspect`'s SKILL (replay
  by chunk), `close-cycle`'s SKILL step 2 (a known signature is its finding).
- `docs/BACKLOG.md`: delete the 5.5 h cap line and, once step 8 shows each
  gone, `nightly-property-jobs-reach-the-350-minute-limit`, `nightly-runner-disk-is-full`,
  `nightly-fuzz-jobs-exceed-their-90-minutes`. Keep `nightly-tests-time-out-at-four-hours`:
  the budget cannot stop those two `sweep_prop` tests (Settled, 2026-11-10), so
  the finding now means those two shards.
- `AGENTS.md` current state: the budget leaves the side slot. `GLOSSARY.md`:
  *Signature* in step 1. `CHANGELOG.md`: nothing a consumer can see.

## Open questions
- Settled (2026-11-10, human): the second Acceptance bullet reads "no `TIMEOUT`
  line for a test not listed in `tools/nightly-known.txt`". In run 42724063416
  two `arris-ops::sweep_prop` shard_3 tests (`a_swept_body_round_trips_through_step`,
  `the_checker_is_green_or_the_sweep_is_a_named_refusal`) ran past the 2 h cap
  inside their first chunk of 8 cases, which the budget cannot stop; triage
  lists them under `nightly-tests-time-out-at-four-hours`, so the job is green.
- Settled (step 1, ADR-0086 §3): the `assert` key is a stem, the text up to
  the first digit: 17 of 18 real failures carry no other key, and the property
  alone would hide every other assertion in a listed test.
- Settled (step 6): no `tools/real-parts.waits` line for CTC-02 AP242 e2's
  drill: an ops fixture takes no part wait, and a part fixture needs an oracle
  record that cannot be written honestly (`docs/BACKLOG.md:109`). The part
  stays red in `real-parts` until that line's condition is met.
- ⚠ OPEN: a known `Disagree(counts)` hides a new count disagreement in the
  differential. Agent's pick: list it under `differential-multi-tool-disagreements`
  only if step 4's run repeats it.
