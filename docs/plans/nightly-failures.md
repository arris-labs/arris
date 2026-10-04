# Plan: nightly-failures

- Started: 2026-10-04
- Milestone: none — upkeep beside C6 (docs/ROADMAP.md); the nightly has been red every night since 2026-09-28
- Idea (verbatim from the human): "work through the remaining nightly (property, differential, fuzz) failures, one fixture/test per failure, commit per fix"

## Goal
The Nightly workflow is green again, or each failure it still reports is a
committed `regression/` fixture (or deterministic test) with its oracle
value, `#[ignore]`d with the reason until fixed. Each night draws a fresh
property seed, so "green" means the failing seeds of 2026-09-28 … 10-04
reproduce clean locally and the next nightly shows no failure that is not
a new one.

Already landed (before this plan): the differential exclusion
`oracle-subdivides-faces` and fixture `regression/turned-dome-three-rim-fillet`
(d021d0c, 8d43785); `NurbsSurface::project` pole-row noise (a0bcd35); the
checker's E4 rounding allowance (0f43155); the projection search's patch-box
descent, sufficient-decrease line search and thin-feature leaf split
(61bca5f, 27ad1b1; seeds `29a5102c…`, `0123456789ab…`, `aa11bb22…` clean at
20 000 cases).

## Non-goals
- New features or new surface pairs; a failure that needs one is parked in
  `docs/BACKLOG.md` with its fixture.
- Widening a tolerance to make a property pass. Where one must move, the
  commit body says so and the constant lives in `arris-math` (kernel.md).
- Pushing; the human reads CI and pushes.

## Design deltas
None planned. A fix that changes a public type or signature names it in the
commit body and `CHANGELOG.md` (git.md §The changelog). A fix with a
non-obvious decision (e.g. an oracle convention excluded from the
differential) gets an ADR step added here.

## Steps
Complexity grades: **[1]** routine — the design says exactly what to write
and the tests are mechanical; **[2]** careful — a geometric or numeric case
to get right within a given design; **[3]** unproven — an algorithm whose
robustness or bound has to be established here.

Every step: reproduce locally with the printed seed first; shrink; commit a
deterministic test or fixture with the desired assertion; fix; `cargo fmt
--all`; commit with `run_in_background` (the hook takes minutes); never wait
with `pgrep -f` loops (they match themselves).

- [x] Step 1 **[1]** — Triage. Done 2026-10-04 from the nightly logs (`gh api repos/{owner}/{repo}/actions/jobs/<id>/logs`; `gh run view --log` returns nothing here). Seeds differ per night: 09-28 `9c84d436…`, 09-29 `a0a764f4…`, 09-30 `9f2abc6e…`, 10-01 `e016178c…`, 10-02 `29a5102c…`, 10-03 `56c7709c…` (5000 cases each; differential 1000). Re-run locally against `main` (each at its own night's seed unless noted); the full-length re-runs were stopped as not worth hours — the shrunk inputs in the logs are the repro, the long runs belong to step 9:
  - **Cleared**: `nurbs_project::a_pole_is_a_row…` (09-28/29/01/02) and `on_a_free_form_surface_nothing_is_nearer…` (10-02), `trace_torus::random_torus_and_cone_pairs_trace` (10-02), `pcurve::every_section_of_a_cone…` shards 0, 1, 3 and at 10-03's seed all four — cleared by a0a764f/61bca5f/27ad1b1; `step_round_trip::a_written_body_reads_back…` (09-28 shard 5, 09-30 shard 3, 10-01 shard 2: clean at their seeds) and 10-03 `body::every_drawn_body_round_trips…::s13` (all 16 shards clean).
  - **Surviving, geom**: `pcurve::a_circle_projected_to_an_oblique_plane…` (10-03 seed, fails in 0.02 s) and `intersect_spline_surface::a_straight_nurbs_meets…` (10-03) → step 2; `intersect_surfaces::constructed_touches_are_tangent…` fails at the **09-29** seed (clean at 10-03's; panic `{-0.3189…, 0, 47.063…}`, tilt 0.1, r2 0.1) → step 2(c); `pcurve::every_section_of_a_cone…::shard_2` fails at the **09-28** seed (shrunk input is a cone with a degree-5 Nurbs section) → step 2(e), new; `intersect_surfaces::coaxial_pairs_meet…` "Too many global rejects" at 10-03 → step 3 (09-30's failure there, a swapped circle `radius 628633`, is clean at its seed).
  - **Surviving, io**: `body::every_drawn_body_round_trips…::s3` fails at the **10-01** seed (10-01's own failing shard s15 is clean; the nightly's `L5 f4: loops 0 and 1 intersect` / `L4 f0: hole loop 0 lies outside every outer loop` text is the family) → step 4. The 09-30, 09-28 and 10-03 io failures reproduce clean.
  - **Not in the original plan**, surviving: `boolean_prop::quartic_cylinders_obey_every_identity` shard 13 at 10-03 (E4 e65 "pcurve on f52 is 1.00001529e-7 off the curve") — fails locally, 168 s; shard 6 and `quadric_operands_obey_every_identity::shard_2` at 10-02 fail locally (E4 ≈1.0004e-7 pcurve off the curve, same family); 10-01 shard 13 (volume additivity) and 09-30 quadric shard 4 ("void s10 is inside no shell") not re-run → step 10.
  - **Not re-run here**: `cancel_prop` (10-01) → step 5; the four differential disagreements → step 6; the two fuzz crashes → step 7. 09-28's differential/09-30's fuzz-lump `Internal(Lumps)` / `Internal(Builder)` cases are named in step 6.
- [x] Step 2 **[2]** — (done: (a), (b), (c) — property conditioning; (e) — a kernel fix in the pcurve unwrapping; see their commits) arris-geom one-offs from the 10-03 workspace-rest job, each at the seed recorded in step 1, one fixture and commit each: (a) `intersect_spline_surface::a_straight_nurbs_meets_each_surface_where_the_line_does` (rational: no hit at a tangent origin, a hit at t≈-9e-11 exists but is not reported as tangent/at origin); (b) `pcurve::a_circle_projected_to_an_oblique_plane_is_the_expected_ellipse` (implicit 1.0000000020430246; decide kernel error vs the property's conditioning at radius 0.1 and distance ≈100 — if the latter, fix the property's scale, saying so); (c) (seed 09-29) `intersect_surfaces::constructed_touches_are_tangent_and_touches_on_the_axis_are_points`; (d) `trace_torus::random_torus_and_cone_pairs_trace` — cleared in step 1, drop; (e) `pcurve::every_section_of_a_cone_has_a_pcurve_on_it::shard_2` (seed 09-28, new). Test: each property at its recorded seed, then `-p arris-geom` whole.
- [x] Step 3 **[1]** — `intersect_surfaces::coaxial_pairs_meet_where_their_meridians_meet`, "Too many global rejects": a generator problem. Loosen the strategy (construct valid pairs instead of filtering) so rejects stay under proptest's cap, with no change to what the property asserts. Test: the property at three seeds, 5000 cases.
- [x] Step 4 **[2]** (done: the cause was `Curve::bounds` of a periodic NURBS edge past its domain, an ops/geom defect, not the writer or reader; the fixture `regression/body-bytes-revolved-hole-loops-intersect` stays `#[ignore]`d on the reader's refusal of Open CASCADE's STEP of it, which is step 8's, so step 8 now also moves this fixture; the other io shards of step 4 were cleared in step 1) — arris-io STEP round trips (10-01 seed `e016178c…` s3; earlier shards cleared): `step_round_trip::a_written_body_reads_back_as_itself` (the "degenerate result: a section passes a face's apex or pole…" shards are cleared) and `body::every_drawn_body_round_trips_through_its_bytes` shard 3 at the 10-01 seed. First decide with the `inspect` skill whether the defect is in the writer's geometry, the reader, or the operation that produced the body (the "degenerate result" text suggests an operation upstream); fixture under `regression/` with the oracle values, `#[ignore]`d until fixed. Test: the fixture, then both properties at the seed.
- [x] Step 5 **[3]** (done: cleared, no new fixture — all seven failing shards (0, 1, 2, 4, 5, 6, 7) panicked in the debug checker guard on a boolean's output, "L4 f<n>: hole loop <i> lies outside every outer loop", not in cancellation; the shrunk shard-2 case, two posed cylinders cut, passes the checker at `Full` and matches the oracle on `main`. The cause was the checker's coarse L4 polygon, fixed by bfe62da (10-01 18:30, after that night's run) with its fixtures `boolean/pin-at-disc-rim-common-fuse` and `boolean/tilted-cylinder-slot-cut`; all 8 shards clean at seed `e016178c…`, 5000 cases) — arris-ops `cancel_prop::every_step_stops_at_a_random_step_and_leaves_the_model_as_it_was` (10-01, shards 0, 1, 2, 4, 5, 6): "an operation's output fails the checker / L4 f40: hole loop 1 lies outside every outer loop" after a cancelled step. Get the log of run 36839407043 job `workspace-rest`, shrink to the smallest recipe + cancel point, fixture, then find which operation leaves a loop un-nested. Test: the property at the recorded seed; the checker green in debug.
- [x] Step 6 **[2]** — (done, 2026-10-04: ADR-0046 and the harness path for the oracle's impossible answers (cases 335, 696, 856), 862/476/991/846 agree now, 328 is `regression/prism-mirror-revolve-fuse-mesh-crossing` with exclusion `mesh-polygon-crosses`; 3549c46a case 1 agrees at 1000 cases (drawn and turned: 792 and 1000 compared, none failing); 9c84d436 case 60 is the S5 family, `regression/revolve-cut-extrusion-s5-undecided` with the exclusion `s5-undecided`, the only failure at that seed once run alone. Two 1000-case differentials run at once share the oracle's scratch and answer garbage (659 oracle refusals, 28 false disagreements): run one at a time) Differential count disagreements. Original text: per case `inspect` both results; an oracle convention or an Arris defect (a `regression/` fixture, `#[ignore]`d). Test: `arris::differential` at 1000 cases, the failing seeds clean or each remaining case fixtured.
- [x] Step 7 **[2]** — (done: 41271dc, 904138d, a regression test each; `fuzz/corpus` is untracked so there is no corpus entry) The two fuzz crashes: `intersect_curve_surface` (09-28) and `intersect_surfaces` (09-30).
- [x] Step 8 **[2]** — (done in part: 624f8cf reads an edge ending at a closed B-spline's seam; three fixtures left `regression/` for `boolean/`; `regression/turned-dome-three-rim-fillet` now meets a pcurve-fit refusal, in BACKLOG, so it and the `oracle-subdivides-faces` exclusion stay) Optional: the STEP reader refuses Open CASCADE's own STEP of fixtures with an edge ending at a seam.
- [ ] Step 9 **[1]** — Final check: full local `ARRIS_GATE=full` gate is `/retire-plan`'s job; here, run each touched crate's properties at 1000 cases at two fresh seeds and note anything new in Open questions.
- [x] Step 10 **[2]** — (done: 3bd1650 fixes the checker's shell volume about the origin and puts the additivity misses under `fitted_rel`; the E4 gaps of 1.0003e-7 and 1.0027e-7 were not 3.6e-11 and 2.7e-9 over the tolerance but ~9e-15 over the edge's own raised tolerance, a hair past the checker's rounding allowance: `pave.rs`'s `residual` now carries that allowance at every sample, not only in `ended`; the frustum test took the whole shard's final shrink, which fails in 1.3 s before the fix; `quadric_operands_obey_every_identity::shard_2` at `29a5102c` and `quartic_cylinders_obey_every_identity::shard_8` at `3549c46a` pass at 5000 cases, 653 s and 1014 s) arris-ops boolean properties.

## Acceptance
- Every failing test named in the nightlies of 2026-09-28 … 10-04 passes at its recorded seed, or is an `#[ignore = "…"]`d regression fixture with an oracle value and a backlog line.
- `cargo nextest run --workspace` at the `fast` profile green; `arris::differential` at `ARRIS_DIFF_CASES=1000 ARRIS_ORACLE_CACHE=off` green on the recorded seeds.
- The next nightly after the human pushes shows no failure from this list (the human reads it; not an agent check).

## Docs to update on completion
- `CHANGELOG.md` `Unreleased` — one bullet per consumer-visible fix (projection correctness on thin or collapsed free-form surfaces; any STEP read/write or operation fix from steps 4, 5, 8).
- `docs/ROADMAP.md` §Fixtures / the fixture counts if fixtures move areas.
- `docs/BACKLOG.md` — any parked failure.
- An ADR if step 6 adds a differential exclusion convention.
- `AGENTS.md` current state — only if a fixture area count line changes (probably not).

## Open questions
- ⚠ OPEN: step 2(b) — is the pcurve ellipse failure a kernel error or the property's conditioning (small circle, far from the origin)? Agent decides at step 2 with evidence; the commit body says which.
- ⚠ OPEN: step 6 — a new differential exclusion is an oracle-convention decision; agent proposes with an ADR, human confirms before it is relied on.
- ⚠ OPEN: step 8 is optional, and now also what lets `regression/body-bytes-revolved-hole-loops-intersect` leave `regression/`; human decides whether it belongs here or to a reader plan, by step 7.
