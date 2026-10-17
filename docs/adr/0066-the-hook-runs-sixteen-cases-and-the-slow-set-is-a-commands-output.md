# ADR-0066 — The hook runs 16 cases and the slow set is a command's output

- Status: accepted (2026-10-17)
- Plan: `hook-gate-speed` steps 1–4
- Amends: ADR-0032 (§Decision "Profiles", "The hook's 32 cases", and
  §Measured)
- Follows: `.agents/rules/git.md` (the hook's gate, `main` always green)

## Context

`/retro loft` measured the hook on the whole `fast` profile at 534 s wall
(7,631 CPU-s, 2,652 tests, 16 threads, 32 cases): the cost of any change
the classifier cannot narrow. Three things made it larger than it had to be.
The slow set, ADR-0032's "every single test of about 30 s or more that is
not a property shard", had been found by hand and had drifted to 5 names.
The property shards were about 2,750 CPU-s, 36% of the profile, at a count
whose extra depth ADR-0032 already called a wider sample of the same fixed
draw. And a change to `loft.rs` or `along.rs` paid for the whole corpus
because `gate.sh` had arms for `src/sweep.rs` and `src/blend.rs`, both
directories by now.

## Decision

- **The slow set is `tools/test-timings.sh --slow-set [SECS]`'s output**
  (default 30): it times `fast` plus the current set at the hook's case
  counts and prints every passing, non-shard test at or above `SECS`.
  `.config/nextest.toml` and `tools/profile-test.sh` hold what it prints.
  The edge is a margin, not a median: a test enters at 30 s and a name
  already in the set stays from 25 s (5/6), so one run decides. The set is
  30 names (it was 5); they leave the hook only, the default profile runs
  them.
- **The hook runs 16 property cases and 16 differential recipes**
  (`tools/gate.sh`: `cases`, and `diff_cases` follows it). The reasoning of
  ADR-0032 stands: depth is new seeds (nightly) and CI's 1000, 256 at
  `/retire-plan`. The differential costs 82 CPU-s at 16 recipes against
  184 at 32 and its wall, 42 s, is set by the additivity test in either.
  A test that asserts coverage over its draws must not rest on one 16-case
  stream: `revolve`'s axis test was the only red of the profile at 16 and
  now draws four seeds.
- **`gate.sh`'s `arris-ops` arms follow the module graph**: `sweep/loft.rs`
  and `sweep/loft_exact.rs` select the `loft` area, `sweep/along.rs` the
  `sweep` area; the rest of `sweep/` and all of `blend/` select the whole
  corpus, on purpose (the blend op is in seven areas' fixtures, extrude in
  nearly every recipe). `rdeps(arris-ops)` stays in each. `gate-test.sh`
  fails when an arm matches no tracked file and when a `tests/fixtures/`
  directory is neither an area nor deliberately whole (`geom`,
  `regression`, `sheet`).

## Consequences

- The `fast` profile: 534 s → 428 s wall with the new slow set (32 cases),
  → 321 s at 16 cases; CPU 7,631 → 6,008 → 4,542 s; tests 2,652 → 2,627.
- A property fault that needs more than 16 cases of the one fixed seed now
  reaches `main` and is found at `/retire-plan` (256) or CI (1000), the
  same accepted risk ADR-0032 took at 32.
- Re-measuring the set is one command; a new slow test is found by it, not
  by someone noticing.

## Alternatives considered

- **`opt-level = 3` for tests**: measured 1.35× (139 s → 99 s on four
  corpus tests) with slower builds; a backlog line.
- **More test threads**: 16 → 32 measured 8%; the half-cores rule stands.
- **Keep 32 for the differential**: its extra 100 CPU-s buys the same
  draw's next 16 recipes; CI draws 1000.
