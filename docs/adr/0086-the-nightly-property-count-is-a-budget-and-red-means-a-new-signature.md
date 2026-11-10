# ADR-0086 — The nightly's property count is a budget, and red means a new signature

- Status: accepted (2026-11-09)
- Plan: the nightly time budget's step 1 (retired)
- Amends: ADR-0024 §3 (the nightly's count is a budget, the tier's count its
  ceiling) and §6 (a red night is a new signature, not any failure)
- Follows: ADR-0024's amendment of 2026-10-21 (a property carries a cost),
  ADR-0013 (the harness stays in `arris-debug`)

## Context

ADR-0024 §3 gave the nightly a fixed count, 5000 cases, set from a measured
wall clock. Run 42522745189 (11-08) showed that the count no longer fits
GitHub's job limit: 2983 tests took 966,000 thread-seconds, and 70 of them hit
the four-hour per-test cap, 630,000 seconds in all. The property jobs had been
split by hand, one split at a time, and three of them were cancelled at the
job limit, so their unreported tests are unknown.

§6 made every red night a finding to triage. The open findings in
`docs/BACKLOG.md` §Findings keep the nightly red until each is fixed, and a
red night says nothing new when the failure is one already on the list. Of the
18 property failures of 11-08, the signature as first defined, a panic's site
or a variant path, classified one in eighteen: every panic site was the
harness's own `prop.rs:271`, so the nightly had no way to tell a known
failure from a new one.

## Decision

### 1. The count is a budget

- A property shard runs **chunks** of `CHUNK_CASES = 8` cases. Chunk `i` is
  seeded by `sha256(shard_seed ‖ i)`, so a chunk replays alone.
- `ARRIS_PROPTEST_BUDGET=<s>` is read by `check` (as shard 0 of 1) and by
  `check_shard_at`. The loop stops before a chunk whose predicted time, the
  slowest chunk so far, would pass the budget. The last chunk is cut at the
  tier's case count, the ceiling. Chunk 0 always runs, so no budget makes a
  property run nothing.
- The clock is a parameter of the `try_` form, so the stopping rule is
  tested without waiting.
- The nightly sets the budget to 3600 seconds. Unset, the path and the random
  streams are today's, byte for byte, so the hook, CI and their fixed seeds do
  not change.
- A shard prints `prop-budget: ran N of M cases in T s (stop: budget|ceiling)`.

### 2. Replay

A failure names its chunk and prints `ARRIS_PROPTEST_SEED=… ARRIS_PROPTEST_CHUNK=<i>`.
With that pair the one chunk reruns on its own shard, so a listed failure
reproduces without the rest of its job.

### 3. The signature

A failure's signature is its (source, operation, class, key), as
`GLOSSARY.md` defines it. The source is the property's `binary::property`,
shard suffix dropped. The class and key come from the first match, in order:

1. `timeout`: nextest's status.
2. `abort`: `property aborted`.
3. `panic`, keyed by `file:line`: the first `panicked at` outside the
   harness's `prop.rs` and `differential.rs`.
4. `checker`, keyed by the row: `checker report: <row>`.
5. `differential`, per `case N: <class>` line, keyed by its class such as
   `Disagree(counts)`.
6. `variant`, keyed by a typed token (`Internal(Split)`, `A::B`).
7. `assert`, keyed by the **stem**: the message up to its first digit, with
   trailing punctuation cut.

The stem is the one key that is message text, which the glossary bars. It is
allowed because it is the test's format string: no draw changes the text
before its first number, and a test reworded at that point reads as a new
signature, which errs red. Keying `assert` on the property alone would hide
every other assertion of a test that is listed as known.

### 4. Known findings

`tools/nightly-known.txt` lists, one line per open finding, the slug, the
`binary::property`, the class and the key. A failure that matches an entry is
printed by slug and count in the job summary and leaves the job green. A
failure that matches none is printed in full and turns the run red.
`known_signatures_name_open_findings` (in `crates/arris/tests/docs_refs.rs`)
holds each slug to a §Findings heading in `docs/BACKLOG.md`, so the entry goes
when the finding does.

A known signature is accounted for, not accepted: `.agents/rules/kernel.md`
§Testing still requires the finding to get its fixture and its fix.

### 5. What makes a night red

A new signature, a missing JUnit file (so a failed build stays red), or a fuzz
crash. Nothing else. A known signature never does.

### 6. Scheduling

The property tests run in 12 jobs, packed first-fit-decreasing on
`sum(min(t, budget))` from one measured run, beside the differential, the
fuzz, real-parts and bench jobs. The pack and the budget figures live in
the comments of `.github/workflows/nightly.yml`'s matrix, each job with its
estimated minutes.

## Consequences

- The nightly's case count is no longer one number. Each night spends its
  budget, and its depth is the budget's, not a count's. Coverage still adds up
  across nights, as ADR-0024 §3 said, because each night's date seed draws its
  own chunks.
- A green night can carry a known finding. The job summary names it by slug
  and count, so the green says what it covers.
- A finding that is fixed leaves `tools/nightly-known.txt` in the same commit
  as its §Findings entry, and the docs test fails until it does.
- The signature's stem can change with a reworded assertion. The result is a
  red night for a known finding that was reworded, which a human then reads
  and re-lists. That is the cost of keying on text, and it errs the right way.

## Alternatives considered

- **Rotation** (a property runs on some nights and not others). A property not
  run tonight is not tested tonight, and the budget makes rotation
  unnecessary. Dropped as option B of the idea this ADR decided.
- **A per-test cap alone.** It stops the job from being cancelled, but a
  timeout never reaches an assertion, so every capped test stays red each
  night with no way to tell a known timeout from a new one.
- **Keying `assert` on the full message.** The message changes with every
  draw, so no two nights would agree.
- **Fewer cases.** It gives up the depth the nightly findings came from (the
  2026-09-25 amendment to ADR-0024).
- **Opening an issue for each known signature.** Outward-facing, so it stays
  ADR-0024 §6's call, and the human's.

## Amendments

- 2026-11-10, §5: a night is also red in two places the triage does not
  cover. The `real-parts` job stays red on `nist_ctc_02_asme1_ap242-e2`: a
  CTC-02 drill line cannot exist in `tools/real-parts.waits` (an ops fixture
  takes no part wait, and a part fixture needs an oracle record that cannot
  be written honestly, `docs/BACKLOG.md`), and the triage does not read that
  job. An unknown corpus failure of the differential job (its ignored corpus
  tests) is red outside the triage too.
- 2026-11-10, §3: a differential's `case N: <class>` lines are classified
  first, one signature per line; `panic` and `checker` keys apply only when
  no case lines are present. The differential catches panics without
  silencing the panic hook, so its output carries a panic site that would
  otherwise win as class 3 and hide the case's own class.
