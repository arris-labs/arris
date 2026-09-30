# ADR-0032 — The lean gate: a path-scoped fast hook, the full suite at retirement and in CI

- Status: accepted (2026-09-30)
- Plan: `lean-gate` steps 1–6
- Amends: ADR-0024 §3, the hook row of "The property tiers" (256 cases) and
  the paragraph "The hook keeps 256". The rest of ADR-0024 stands, time
  never being a gate (§4) included.
- Follows: `.agents/rules/git.md` (trunk-based, always green)

## Context

The suite costs 315 s of wall clock and 9,235 CPU-seconds on a 32-core
machine (1511 tests). Three property files (`boolean_prop`, the body
round-trips, `cancel_prop`) are 57% of it and the corpus 22%; the 23
`real_*` excerpts alone are 643 CPU-seconds, two of them 160 s each.

The gate ran **twice per plan step**: `/work` told the agent to run fmt,
clippy, `cargo test --workspace` and `cargo doc`, and the pre-commit hook
then ran the same again on the commit. ADR-0024 §3 decided "the hook keeps
256", weighing a hook that "costs minutes more"; the measured cost is five
minutes, twice, on every step of every plan, for a change that usually
touches one crate.

## Decision

**Three tiers, each a superset of the one before.**

| Tier | Runs | Cases |
|---|---|---|
| `/work` step | the step's own tests, and for a geometric change the corpus area it touches | as the test sets |
| pre-commit hook | layers, fmt, clippy, doctests, the `fast` nextest profile over what the staged paths reach, `cargo doc` only when a doc comment changed | 32 |
| `/retire-plan`, CI, nightly, `/close-cycle`, `/release` | the full profile: every test, `real_*` included, `cargo doc`, the oracle selftest, the wasm build | 256 at retirement, 1000 in CI, more at night |

- **Profiles** (`.config/nextest.toml`). `default` is unchanged, so CI's
  plain `cargo nextest run` is too. `full` is the default's behaviour, named
  so a skill can say it. `fast` is the default minus `real_*` and four
  single tests that cost 30 s or more and that no commit's own change needs
  (`guard::coverage`, `boolean_fixtures_read_back`,
  `part::tests::a_battery_stage_is_held_to_its_class`, the ops cancel test
  over the corpus). A test is excluded from a profile, never weakened;
  `tools/profile-test.sh` holds the difference to exactly that set, so a
  rename cannot leave it silently.
- **`tools/gate.sh`** prints the gate a path set needs and runs nothing:
  a crate's source reaches the crate and its reverse dependencies
  (`rdeps(<package>)`, the cargo name, `arris-ops`) and the corpus areas it
  can change; a crate's tests, only the crate; a fixture, its area; prose,
  the docs tests; **a path it cannot place, the whole `fast` profile**,
  never less. `tools/gate-test.sh` asserts it, including that no crate path
  yields a gate that leaves its own crate's tests out. `ARRIS_GATE=full`
  runs the old gate, every test at 256 cases and `cargo doc`.
- **The hook's 32 cases reverse ADR-0024 §3's 256.** The reasons it gave
  still hold and now point the other way: the seam fault sat past case 32
  of *one fixed seed*, which 256 and 32 both share, so the hook's count
  never reached it — depth is new seeds (nightly) and CI's 1000. What 256
  bought at the hook was a wider sample of the same draw; it is now bought
  once per plan, at `/retire-plan`, instead of once per step.
- `ci.yml` and `nightly.yml` are unchanged: 1000 cases, `real_*` among the
  ordinary workspace tests, and a release waits on CI's run of its tag.

## Consequences

- A plan step costs what its change needs; a change that stays in one
  crate's area selects a fraction of the suite, and the worst case (a path
  the classifier cannot place, or `arris-debug`, which reaches the whole
  corpus) is the whole `fast` profile, about 110 s against 312 s.
- **The accepted risk.** A commit can now land on `main` with a failure the
  hook did not look for: a `real_*` excerpt, a corpus area outside the
  crate's own (a `geom` change reaches `boolean_*` and `provenance_*` but
  not `sweep_*`), a property case past 32, a doc build break when no `///`
  line changed, a `parallel` feature combination. `main` stays green at
  every *pushed* tip because the human reads CI before a push that
  matters, `/retire-plan` runs the full profile once per plan, and a
  release waits on CI. A failure found late is bisected by the commit
  suffix that names its plan step; it becomes a fixture like any other
  (`.agents/rules/kernel.md`).
- The corpus areas a crate reaches (`boolean` and `provenance` for the
  lower crates, the named area for `sweep`, `blend`, `transform`) are a
  judgement kept in one script; widening one is a line in `gate.sh` and a
  case in `gate-test.sh`.
- Not a timer: the `fast` set was chosen once from measured cost
  (below) and nothing enforces a duration.

## Measured

32-core machine, 2026-09-30. Default profile, 256 cases: 315 s wall,
9,235 CPU-s, 1511 tests; the slowest singles `real_nist_ctc_04` 170 s,
`real_nist_ftc_07` 163 s, `guard::coverage` 90 s. `fast` at 32 cases:
1489 tests, 110 s wall, 2,538 CPU-s, its slowest single tests
(corpus `boolean_*`, the part battery, `tolerance_band`) 37–53 s under
contention. The hook on the whole `fast` profile: 106 s nextest plus about
10 s of fmt, clippy and doctests, against 312 s for the old gate. A
planted fault that needs a property's depth passed at 32 cases and failed
49 tests at 256; one in a `real_*` excerpt passed the hook's gate and
failed the default profile. No fault was found that the per-area scoping
alone loses, which is not a proof that none exists.

## Alternatives considered

- **Keep 256 at the hook on the crates it reaches.** One number in
  `gate.sh`; rejected because the extra depth is the same fixed draw (see
  above) at 8× the property cost.
- **Drop the hook and rely on CI.** `main` would then be green only after a
  push; the hook's fmt, clippy, layer and path-scoped tests are what make a
  local commit trustworthy and cost seconds to a couple of minutes.
- **A time budget per test, excluding whatever is slow.** ADR-0024 §4:
  time is never a gate, and a timer makes the profile drift with the
  machine.
- **Sharding the `real_*` excerpts so they fit in `fast`.** The two
  160-second tests are single solids; sharding them is a different piece of
  work and they are in CI either way.
