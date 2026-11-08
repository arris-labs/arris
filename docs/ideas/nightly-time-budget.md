# Idea: nightly-time-budget

- Status: Accepted (A then C, B rejected); to be planned as plans/nightly-time-budget
- Raised: 2026-11-09
- Prompt (verbatim from the human): "The problem is that those tests do not fit into github 6h limit. SO my questions to you are: what is github policy for open projects on free plan for CI and if we soon exhaust all limits? Do we really need 5000 cases? Can we shard it more? Are those nightlies even efficient in finding problems? maybe @../arris-gym more efficient way and we better drop nightlies at all? What optimal solution in your view?" — then, on the three proposals (time budget, rotation, signatures of known findings): "yes, /idea 1-3".

## Problem

The nightly runs every property at a fixed `ARRIS_PROPTEST_CASES=5000`
(`nightly.yml:152`) and has been fitted to GitHub's limits by hand, one
job split at a time (`plans/nightly-panics` step 8 alone took five
commits). It still does not fit, and it says little when it does run:

- **Limits.** The repository is public: standard runners cost no minutes
  and there is no quota to exhaust. The walls are **6 h per job** and **20
  concurrent jobs** on the Free plan. The nightly has 30 jobs (21
  property, 5 fuzz, differential, real-parts, bench, seed), so about ten
  wait for a free runner and the night runs past 6 h end to end.
- **Cost per case, not count, is what breaks.** Measured on the 4-vCPU
  runner in the dev profile (`opt-level = 1`): the elbow
  (`/ 16`, 2 shards, 157 cases a shard) about **76 s a case**; the
  walked seam (8 shards, 625 cases) about 22 s; standoff roots (4 shards,
  1250 cases) about 11 s. The same shard varies 1.7× between runners (the
  slot chain 2,300 s and 4,000 s). A fixed count against a per-case
  cost that grows with every blend feature and a runner that varies by
  1.7× ends at the limit sooner or later, whatever the split.
- **A timeout reports nothing.** nextest's cap or the job's 350 minutes
  ends the shard with no case named (`nightly-tests-time-out-at-four-hours`).
- **Red carries no signal.** The last 19 nightlies (10-23 to 11-08) were
  all red. The same families recur ("12 of the 14 nightlies", "every
  night 10-22 to 11-03" in §Findings), and most §Findings entries are
  `State: raw`. ADR-0024 §6 makes a red run the nightly's *only* output, so
  a run that is always red tells nobody about the new defect inside it.

What 5000 buys: ADR-0024 §3 already says the nightly's depth comes from
seeds adding up across nights, not from one night's count. At a fixed
seed the extra cases add coverage. With a new seed every night they only
find a rare defect a few nights sooner, and finding it is not where the
pipeline is slow.

## Constraints it runs into

- **ADR-0024 §3**: the nightly's count is "a multiple of CI's, set from
  measured wall clock to keep each job under the runner's six hours". A
  budget is what that row asks for, set at run time instead of in YAML;
  the row is amended, not reversed.
- **ADR-0024 §6**: a red run is the only output; nothing posts. Kept: the
  output stays the run's colour and its job summary. What colour means
  changes, so §6 is amended.
- **Determinism** (`.agents/rules/kernel.md`): every case stays
  reproducible from a printed seed. Under a budget, *how many* cases ran
  depends on the runner; *which* case failed does not.
- **"Never kept as accepted behaviour"** (kernel.md §Testing): a known
  signature is not acceptance. Each one names an open §Findings entry and
  goes when the entry goes, as `differential::EXCLUSIONS` names its
  fixtures today.
- **`docs/ideas/failure-capsule-and-triage.md`** (Parked): its option B
  defines a *signature* as (source, operation, class, key), where the key is a
  variant path, a checker row or a panic's `file:line`, never free text.
  Option C here uses that definition without the capsule. A later capsule
  plan then extends C instead of replacing it.
- **ADR-0013**: harness code stays in `arris-debug` (`prop.rs`), which
  is `publish = false`; no published API changes.
- **arris-gym** is a skeleton ("Nothing runs yet", its README). Its
  CadQuery corpus will test realistic programs, a distribution the random
  poses do not cover. It cannot replace this tier now. When its
  fidelity run works it is a candidate for the `differential` job's slot.

## Options

### A — A time budget per property shard
`ARRIS_PROPTEST_BUDGET=<seconds>`, read by `prop::check_shard_at` only
when set; the hook and CI keep their counts. The shard runs **chunks** of
cases, each chunk seeded from (base seed, shard, chunk index). It stops
before a chunk whose predicted time would pass the budget, with the
tier's count as a ceiling. It prints `ran N cases in T s` to the job
summary. A failure names the chunk, so its replay is exact. The job
layout follows from the rule *per-test budget = job wall budget ×
threads ÷ capped tests in the job*. The 21 property jobs regroup into the
about 12 that fit beside the 8 other jobs under the 20-job limit.
nextest's per-test cap overrides (`.config/nextest.toml`) go. Cost: about
4 steps (the harness and its own test; the regrouped matrix; the caps
removed; the ADR). Trade-offs: the night's count varies with the runner,
so coverage is counted, not promised. A property that slows 10× runs a
tenth of the cases rather than timing out. The summary's count shows
that, and the bench catches a 10× slowdown separately. A single case past
the whole budget still runs to the end once; the elbow's 76 s is far
from that.

### A′ — A case count per matrix job
A `cases:` key per matrix entry, from measured cost; no harness change.
1 step. It is today's method with a finer knob: the 1.7× runner variance
still reaches the limit, every new heavy property needs measuring again,
and the 30-job queue stays.

### B — Rotate the heavy properties' shards across nights
The `seed` job picks `shard_(day mod k)` for the elbow, standoff and seam.
1–2 steps. Weekly coverage does not stay the same: each property runs 1/k
of its cases a night, and a rare defect is found k times later. Under A,
B is not needed. A gives every property some cases every night, which
beats some properties getting all of their cases on one night in k.
Rotation only helps a property whose one case does not fit the budget,
and none is near that.

### C — Signatures of known findings
A tracked list, one entry per open §Findings slug: the test (a nextest
name pattern) and the key (an error variant, a checker row, or a panic
site). nextest writes JUnit, and a post-step classifies each failure as
**new** (it fails the job), **known** (listed by slug and count in the
summary; the job stays green), or **quiet** (a known entry whose test
passed tonight, a candidate to re-check). A lint holds every entry to an
existing §Findings heading. Cost: about 3 steps (JUnit and the
classifier; the list seeded from today's findings, with the lint; the
ADR-0024 §6 amendment). Trade-offs: a new defect that fails a listed
test with the same key is hidden. The key's precision is the guard, and
a key never matches on free text.

### C′ — Fold known families into each property's own exclusions
As `differential::exclusion_of_error` does. Per-test code. It hides the
family from the hook and CI as well, and nothing lists what is excluded.
Worse than C.

### Do nothing
The job split keeps getting tuned by hand: three rounds of step 8 so far,
each costing a night to measure. The run stays red every night, and a
new panic is found only if someone reads every log.

## Recommendation

**A then C; B rejected; A′ only as a stopgap if A slips.** A makes "fits
in 6 h and 20 jobs" true by construction rather than by measurement, and
ends the nameless timeouts. C makes red mean "something new", which is
what ADR-0024 §6 relied on. Both need one new ADR amending ADR-0024 §3 and
§6, written by the plan. They are one plan of about 7 steps, A's steps first
so the nights C is seeded from run to the end.

What would change my mind: if a property's single case nears the whole
budget, B comes back for that property alone. A faster build buys more
cases under A but changes nothing in its shape: the backlog's
`opt-level = 3` line measured only 1.35× on a `twin_prop` shard, so it
stays a line of its own.

`plans/nightly-panics` step 9: its goal (no panics, slot chain and elbow
fixed) does not need run 42522745189 to fit the limit. Another round of
step 8 is the hand-tuning A replaces.

## Decision for the human

Round 1:

❓ **Q1 — How the nightly sizes a property**: (A) a time budget per shard
in the harness, the jobs regrouped to fit 20; (A′) a case count per job in
the matrix; (N) keep 5000 and split further.
➡️ A — the only one that fits by construction against a 1.7× runner variance.

❓ **Q2 — What a known failure does to the run**: (a) reported by slug in
the summary, the job green, only a new failure red; (b) reported, the job
still red.
➡️ a — under (b) the run stays red every night and the classification changes nothing.

❓ **Q3 — Rotation**: drop B, keeping it as the fallback for a property
whose single case nears the budget.
➡️ Yes — under A every property gets cases every night, which beats rotating them.

❓ **Q4 — `plans/nightly-panics` step 9**: (a) read run 42522745189 for
panics only, hand any timeout to this idea's plan, and retire; (b) keep
re-splitting under step 8 until the run fits.
➡️ a — another round of step 8 is the hand-tuning A replaces.

❓ **Q5 — The words**: add **Budget** ("the wall clock a property shard
spends at the nightly tier; it runs chunks of cases until the next would
pass it, the tier's count a ceiling") and **Signature** ("a failure's
(source, operation, class, key), the key a variant, a checker row or a
panic site, never free text; a *known* signature names an open finding")
to `GLOSSARY.md`.
➡️ Yes — the plan and the ADR need both, and *Signature* matches the parked capsule idea's definition.

ADR: one new ADR amending ADR-0024 §3 (the nightly's count is a budget)
and §6 (red means a new signature), written by the plan.

*Later*: none; the budget figures and the job layout are the plan's,
from the runs it measures.

## Decision

2026-11-09, the human: "yes" to round 1.

- Q1: A — a time budget per shard in the harness, the jobs regrouped to
  fit 20 concurrent.
- Q2: a — a known signature is listed by slug in the summary and leaves
  the job green; only a new one is red.
- Q3: B dropped; the fallback for a property whose single case nears the
  budget.
- Q4: a — `plans/nightly-panics` step 9 reads run 42522745189 for panics
  only, hands any timeout to this idea's plan, and the plan retires.
- Q5: *Budget* and *Signature* added to `GLOSSARY.md`.
- ADR: one new ADR amending ADR-0024 §3 and §6, written by the plan.
