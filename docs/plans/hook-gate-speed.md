# Plan: hook-gate-speed

- Started: 2026-10-17
- Milestone: none (tooling beside C10, docs/ROADMAP.md)
- Idea: verbatim from the human, after `/retro loft` measured the hook: "yes, /plan 1-3"
  (1: the slow-set rule re-applied, with a command that re-measures it;
  2: the hook at 16 property cases; 3: narrower `arris-ops` arms in `gate.sh`)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
The pre-commit hook's whole `fast` profile costs about 40% less than its
2026-10-17 measurement (534 s wall, 7,631 CPU-s, 2,652 tests, 16 threads, 32
cases), and a loft or sweep-along-a-path source change no longer pays for the
whole corpus. The slow set is again what ADR-0032's rule says it is: every
single test of about 30 s or more that is not a property shard, found by a
command rather than by hand, so re-measuring it is one run. ADR-0066 records
the numbers and amends ADR-0032's 32 cases and its slow set.

## Non-goals
- The 116 s relink of the test binaries after a lower-crate change. That is a
  build cost, not a gate choice; an idea if it stays the larger half.
- `opt-level = 3` for tests: measured at 1.35× (139 s → 99 s on four corpus
  tests, 148 s → 109 s on a `twin_prop` shard), with slower builds. A backlog
  line, not a step.
- More test threads: `.config/nextest.toml` measured 16 → 32 at 8%, and the
  half-cores rule stands.
- Narrowing `tools/oracle/*`: the oracle cache key hashes every `.py`, so any
  edit re-runs Open CASCADE in every test that reaches it.
- `/retire-plan`, CI and nightly: their 256 / 1000 / 5000 cases and the
  default profile are unchanged; everything the hook drops still runs there.

## Design deltas
- **`tools/test-timings.sh --slow-set [SECS]`** (default 30): runs the
  `fast` profile at the hook's case count and prints, one per line, the
  test names of every passing test at or above `SECS` that is not a
  property shard (`::shard_N`) — the list `.config/nextest.toml` and
  `tools/profile-test.sh` hold. Measured on 10-17: 25 tests, 1,534 CPU-s,
  20% of the profile (the 6 above 90 s: two NIST CTC-04 corpus parts, the
  spring sweep, the pipe elbow's band ends, CTC-05 AP242, `step_read`'s
  `blend_fixtures_read_back`).
- **The slow set** in `.config/nextest.toml` `[profile.fast]` and the
  `slow=` list of `tools/profile-test.sh` grow from 5 names to what the
  command prints; `profile-test.sh` keeps holding the two lists equal. They
  stay out of the hook only: the default profile runs them.
- **Hook cases**: `tools/gate.sh` prints `cases=16` (was 32); `diff_cases`
  follows it unless step 4's measurement shows the differential needs 32
  (⚠ OPEN below). Measured share of the property shards on 10-17: about
  2,750 CPU-s, 36%.
- **`tools/gate.sh`'s `arris-ops` arms**, from the module graph (`loft.rs`
  uses only `loft_exact`; `along.rs` uses `exact`, which uses `revolve`):
  - `src/sweep/loft.rs`, `src/sweep/loft_exact.rs` → corpus area `loft`;
  - `src/sweep/along.rs` → corpus area `sweep`;
  - the rest of `src/sweep/` (extrude, revolve, `exact`, `refuse`, `mod`) and
    `src/blend/` → the whole corpus, as now. The blend op appears in seven
    areas' fixtures and extrude in nearly every recipe, so this is correct
    and is now written down, not a fall-through;
  - the dead `src/sweep.rs` and `src/blend.rs` arms (both now directories)
    go.
  `rdeps(arris-ops)` stays in every `arris-ops` gate: a loft change still
  runs `arris::differential` and the recipe properties that draw a loft.
- **`tools/gate-test.sh`** gains: each arm's path pattern matches at least
  one tracked file, so an arm cannot outlive its file again; each
  `tests/fixtures/<dir>/` is either listed as an area or deliberately whole
  (`geom`, `regression`, `sheet`).
- **ADR-0066**, amending ADR-0032 (§Decision "Profiles", "The hook's 32
  cases"; §Measured): the 10-17 measurement, the slow set as a command's
  output, 16 cases, the before and after.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — `test-timings.sh --slow-set`: the mode and its
  usage line. Check: one run prints the 25 names measured on 10-17 (within
  the noise at the 30 s edge, which the commit lists). Ran: 30 names, the
  25 plus the 5 already in the set, which the mode times too.
- [ ] Step 2 **[1]** — The slow set moved: `.config/nextest.toml` and
  `profile-test.sh` take the step 1 list. Check: `tools/profile-test.sh`
  passes; `--profile fast` timed before and after in the commit body.
- [ ] Step 3 **[2]** — `gate.sh`'s `arris-ops` arms as in the deltas,
  the dead arms gone, `gate-test.sh`'s cases for `loft.rs`, `along.rs`,
  `extrude.rs` and `blend/` plus the two drift checks. Check:
  `tools/gate-test.sh` passes, and fails with a dead arm planted.
- [ ] Step 4 **[2]** — Hook at 16 cases: `gate.sh`, `gate-test.sh`'s
  expectations, `git.md`, the hook's comment. Measure the whole `fast`
  profile at 16 and the differential at 16 against 32 to settle
  `diff_cases`. Check: `gate-test.sh`, the timing in the commit body.
- [ ] Step 5 **[1]** — ADR-0066 with the before / after numbers from
  steps 2–4, listed in `docs/adr/README.md`; ARCHITECTURE §The gate's tiers
  says the slow set is the command's output (it says "four" today, the
  set holds five).

## Acceptance
`ARRIS_GATE=` (the hook's default) whole `fast` profile, timed by
`tools/test-timings.sh --profile fast` at the new case count: at most
~330 s wall on 16 threads (from 534 s), every test passing.
`tools/gate.sh plan crates/arris-ops/src/sweep/loft.rs` selects the `loft`
corpus area and no other area. `tools/gate-test.sh` and
`tools/profile-test.sh` pass. The full gate at `/retire-plan` passes with
the slow set included (the default profile is untouched).

## Docs to update on completion
- `docs/ARCHITECTURE.md` §The gate's tiers — the slow set by command, 16
  cases, the `arris-ops` arms.
- `.agents/rules/git.md` — "at 32 cases" → 16 (step 4).
- `.githooks/pre-commit` header comment — 16 cases, the slow set's source.
- `.config/nextest.toml` comment over `[profile.fast]` — "from the 32-case
  measurement in ADR-0032" → ADR-0066's.
- `docs/adr/README.md` — ADR-0066; ADR-0032's entry notes the amendment.
- `docs/BACKLOG.md` — `opt-level = 3` for tests (1.35× measured) and the
  116 s relink, one line each.

## Open questions
- ⚠ OPEN: does the differential keep 32 cases at the hook (`diff_cases`)?
  Agent decides at step 4 from its measured cost at 16 and 32.
- Decided at step 1: the threshold's edge is a margin, not a median — a
  test enters at 30 s and a name already in the set stays from 25 s (5/6
  of the threshold), so one run settles it. ADR-0066 records it.
