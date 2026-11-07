# Plan: nightly-panics

- Started: 2026-11-07
- Milestone: beside the cycles (the side slot; C12 stays open, docs/ROADMAP.md)
- Idea: none; verbatim from the human: "fixing them starting from panics". `docs/ideas/refactor-after-c11.md` Q1 orders this before its own plans.
- Branch: runs on `plan/nightly-panics`: step 9 is **[CI]** and needs a push.
- The second oracle's machine: only if step 2's fixture has no Open CASCADE answer (ADR-0073); step 2 says so before any law is chosen.

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal

The nightly's "two panics" are gone, and so is the rest of what the
triage of 2026-11-07 found behind them: no property or fuzz target ends in
`kernel bug` or a panic outside the harness's own report. The slot chain
and the elbow build clean at `Full` or are refused by name; the curve
intersector does not call a curve `Coincident` with a line hundreds of
units from it. The nightly's `blend` job runs to its end, so the next
panic is read the night it happens instead of being cut off by the job
limit. The nightly may stay red on the findings the *Non-goals* name.

## What the triage measured (2026-11-07, at `96696781`)

- **Slot chain**: live. Replayed on the 11-03 seed, `shard_1` panics in
  363 s: `V3 v99: f42's surface at the pcurve end is 2.5e-3 away` (v100
  too). Shrunk input: radius `0.004`, rotation `[0.6601067078114292,
  0.3063697241533592, -0.6583548691682864, -0.1922643822109537]`,
  translation `[-95.06916248407532, 0, 0]`. The nights 11-04 to 11-07 show
  no failure only because the job is cancelled at 350 minutes with the
  test still running, so the backlog's bisect is not needed.
- **Elbow**: 11-01's panic was the same message; on 11-05 and 11-07
  `shard_1` ends in `kernel bug: a geometry query failed: fit: the fit
  still deviates by 2.7e-8 with 3796 spans` (`3018` on 11-07): a typed
  error, but `Internal`. Seeds `cd3af2c6…1bc4`, `5f5f0065…29df`.
- **Fuzz crash**: reproduces. Not a denormal parameter as the backlog
  says: a degree-2 NURBS whose whole domain is about 1e-71 wide, its spans
  from 1e-274 to 1e-71, against a line, answered `Ok(Coincident)` with the
  curve's start about 3.16e2 off (reading the message's truncated exponent;
  step 6 confirms). The panic is the harness's assertion.
- **Two skew lines**: the backlog's "panic" is the test's `hits_of`, and
  the shrunk draw is `turn = 0.0`, where the two lines are one line, so
  `Coincident` is right there. The unshrunk draw is unread.

## Non-goals

Each is a later milestone plan, or stays a Findings block:

- `nightly-section-faces`: the arrangement's `Internal`, `split` L5/S5/E8, `cut_many` +2 faces, the 1e-8 sums.
- `nightly-geom-properties`: `extrema_surfaces`, the tracer's turning points (now on `every_quadric_pair…` too), the twins' counts, `line_nurbs`.
- `nightly-budget`: the 4 h timeouts of `sweep_prop`, `twin_prop`, `a_multi_tool_result…`, `every_other_pair_is_unsupported`, past what step 8 buys.
- `cusp_corner_*`'s `no closed form` for torus against torus (three nights, new): recorded by step 1, not fixed here.
- The differential's disagreements, and CTC-02's `real-parts` line.
- Blend or checker redesign; `docs/ideas/refactor-after-c11.md`'s `blend-phases`.

## Design deltas

- **Tolerances of V3** do not move: a fix that loosens V3 to pass the slot
  chain is out; the pcurve end is brought onto its face or the blend refused.
- **Elbow's fit**: the walked surface's `FitError` past `MAX_FIT_SPANS`
  leaves a blend as a named `OpError` refusal if step 5 cannot make the fit
  converge. An `Internal` for a fit that ran out of spans is the defect
  either way. A new variant is a design delta: `Breaking` bullet and
  `crates/arris-py` in the same commit.
- **NURBS coincidence** (step 6): two candidates, the agent picks and
  records why. (a) the on-surface reader samples every non-empty span, not
  the domain evenly; (b) `NurbsCurve::new` refuses a span below the
  parameter resolution. (a) changes no public type and is the default; (b)
  is a public change and an ADR, and is taken only if spans that narrow break
  other algorithms too.
- **`nightly.yml`**: nextest gets `--test-threads num-cpus`, as `ci.yml`
  has; `blend` is split so the slot chain, `oblique_chains` and the elbow
  start at once in a job of their own; `fuzz` gets 150 minutes; the
  `properties` job frees the runner's disk before the build. The matrix's
  CPU-second comments are refreshed from the night's timings.
- No crate boundary changes. No new term.

## Steps

Complexity grades; a step's grade is its tier, the model that executes it
(`.agents/tiers.md`), and `**[1, tier 3]**` overrides the tier: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here. **[CI]** after the grade: the step's
only test is a CI run, which needs a push, so the plan runs on
`plan/nightly-panics`.

- [x] Step 1 **[1]** — Record the triage: `docs/BACKLOG.md`'s Findings take the nights 11-05 to 11-07 with their seeds, this plan's corrections to the slot-chain, fuzz and skew-line blocks, and new blocks for `cusp_corner_*`, `every_quadric_pair…` and the 4 h timeouts. The crash input (run 40469593843's artifact, expiring 2027-01-25) is committed under `fuzz/`, past its `.gitignore`.
- [x] Step 2 **[2]** — Fixture `regression/slot-chain-posed-pcurve-end`: FTC-11's slot chain at the shrunk radius and pose, `#[ignore]`d, failing V3 in seconds. Its oracle is Open CASCADE's answer for the fillet; none means ask for the second oracle here.
- [ ] Step 3 **[3]** — Find why v99 and v100's pcurve ends sit 2.5e-3 off f42 and fix it. The fixture moves to `blend/` with its blessed dump; `slot_chain…` holds on the 11-03 seed at 5000 cases, both shards.
- [ ] Step 4 **[2]** — Replay the elbow on seeds `fc72dc4e…` (11-01), `cd3af2c6…`, `5f5f0065…`, and shrink each signature into `regression/` (`#[ignore]`d). One fixture if one pose carries both the V3 and the fit failure.
- [ ] Step 5 **[3]** — Fix the elbow's fixtures: the fit converges within `MAX_FIT_SPANS`, or the blend is refused by name per the delta. If the V3 is step 3's cause, this step only proves it. `an_elbows_vanishing_end…` holds on the three seeds.
- [ ] Step 6 **[2]** — The crash as a `crates/arris-geom` test: a curve with spans that narrow is never `Coincident` with a line it leaves. Fix per the delta; `cargo +nightly fuzz run -s none intersect_curves <input>` exits clean.
- [ ] Step 7 **[2]** — Read the skew-line property's unshrunk draw on seed `bb2adb02…`. Near-parallel within the tolerance: the generator keeps `turn` off 0 and π, by a named constant. Otherwise a kernel defect: a `regression/` fixture and a fix.
- [ ] Step 8 **[1]** — `nightly.yml` per the delta. Checked by `actionlint` if present and by every filter still naming its tests: `cargo nextest list -E` on each new filter, the two `-rest` jobs still taking the remainder.
- [ ] Step 9 **[1, CI]** — The human pushes `plan/nightly-panics`; `gh workflow run nightly.yml --ref plan/nightly-panics`; the run is read per *Acceptance*. A job that still ends at its limit goes back to step 8 with its timings.

## Acceptance

- The fixtures of steps 2 and 4 pass in their areas, their `regression_*` tests no longer `#[ignore]`d; the step 6 test passes.
- At 5000 cases: `slot_chain_through_its_kink_is_pose_independent` on seed `5078719e…`, `an_elbows_vanishing_end_is_pose_independent` on `fc72dc4e…`, `cd3af2c6…`, `5f5f0065…`, `two_skew_lines…` on `bb2adb02…`, each green or, for the elbow, passing through its named refusal.
- The crash input passes under `cargo +nightly fuzz run -s none intersect_curves`.
- The dispatched nightly on the branch: `properties (blend)` concludes without cancellation, and no job's log carries a `panicked at` outside `crates/arris-debug/src/prop.rs` or the text `kernel bug`, beyond the arrangement finding the *Non-goals* leave. A red job naming a non-goal's test does not fail this.
- `tools/full-gate.sh` is green on the tip.

## Docs to update on completion

- `docs/BACKLOG.md` §Findings — the slot-chain, fuzz, skew-line and timeout blocks moved or deleted; the elbow's, if a refusal, says so.
- `AGENTS.md` current state — the "two panics" and the side slot's next item.
- `docs/ideas/refactor-after-c11.md` — its panic prerequisite reworded as met (`/retire-plan`'s rule for an absorbed-by mention).
- `CHANGELOG.md` — under `Unreleased`: the new refusal if step 5 takes one (`Breaking`), the corrected `Coincident` verdict.
- `tests/fixtures/README.md` — only if a fixture needs a pose the format lacks.
- `docs/adr/` — only if step 6 takes (b).

## Open questions

- ⚠ OPEN: the human decides by step 9 — one extra full nightly dispatched on the branch (about 6 h of the matrix's runners), and the push it needs. The agent's pick: do it once; without it `blend`'s fix waits for a merge to be read.
- ⚠ OPEN: the agent decides at step 3 — the fix versus refusing at this radius, if the pcurve cannot reach the face: refusal only with the reason in the fixture's commit, since FTC-11 needs this chain only at its file's radius.
- ⚠ OPEN: the agent decides at step 6 — (a) or (b), default (a).
