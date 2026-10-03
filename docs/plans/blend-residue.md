# Plan: blend-residue

- Started: 2026-10-03
- Milestone: C6 (the blend network, docs/ROADMAP.md), third plan
- Idea: docs/ideas/blend-residue.md (absorbed)
- Idea (verbatim from the human): "the blend residue"

## Goal
The fetched tier's `fillet` column (8 parts) and the committed tier's (6 of
11) are measured **by part**, each part's first refusal named and what
stands behind it, and the end families ADR-0036 §6 postponed (an open arc's torus ending on a plane
parallel to its axis and off it, or on a cylinder across; a ruling stripe ending on a cylinder or
cone square across) build, the blend's surface exact and only the end curve
traced and fitted at the arc's tolerance, as ADR-0019 fits a section. The
column is printed again beside C4's 17 of 38 and this plan's first line.

## Non-goals
- Pairs with no closed form — crossing cylinders, a torus against a
  cylinder off its axis (quartic edges): a fitted *surface* is ADR-0007's
  rejected alternative; they stay `Unsupported` and wait for the NURBS
  cycle (idea decision 3).
- The run-over (`BlendTooLarge`) and tangent continuation (`TangentChain`
  past a corner): their own idea after this plan (decision 4).
- The closed-form pairs left (parallel cylinders, a plane through a cone's
  apex, the tilted three-plane corner) and other `VertexBlend`s: the
  re-census decides whether a next plan takes them; none is built here.
- NURBS faces as operands, healing, variable radius, other chamfer modes.
- The B-spline-written conic edge (idea option B): the step-1 census found it
  a first refusal only in two NURBS-cycle parts, so it frees no part; it
  stays a backlog line (decided 2026-10-03, option (a) of step 2's gate).

## Design deltas
- **ADR (step 5), new:** a blend's end curve may be traced and fitted while
  its surface stays exact. Amends ADR-0007 §Consequences ("nothing is
  fitted but a pcurve") and closes ADR-0036 §6's deferral; cites ADR-0019's
  `SECTION_FIT_FRACTION` rule. Decided at step 5 (ADR-0037): both — the
  tracers' dispatch and the fit leave `section::traced` as two public
  `arris-geom` entries, `trace_section` and `fit_branch`, which the
  intersector and `arris-ops`' `blend/traced.rs` share (additive).
- `arris-ops` `blend.rs`: edge curve recognition (step 3) and two end
  trimmers (steps 6, 7). No public type or signature changes expected;
  `Unsupported` loses cases, no `Reason` variant is added. If step 5 finds
  otherwise it is a design delta named in that commit.
- `arris-check`: no change (ADR-0037 §5) — E4 holds the fitted pcurves,
  S5 decides the blend against the face across by the same tracers;
  steps 6 and 7 show it with their fixtures at `Full`.
- `docs/ARCHITECTURE.md` §blend, `docs/DATA-MODEL.md` §Tolerances if a
  fitted end curve carries its own tolerance; ROADMAP §C6 status.
- Python binding: no new error variants planned; if one appears the stub and
  class are in the same commit (`kernel.md` §API).

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The re-census by part. Extend the survey's fillet
  stage (`arris_debug::survey`, `tools/real-parts.sh`) to print, for every
  part in the column of both tiers, the **first refusal at the battery's
  radius** (kind, faces, how they sit) and the refusals the part's other
  edges would meet once that one is cleared (each edge blended alone, as
  step 1 of `c6-blend-pairs` did). Output: a table `part → first refusal →
  next refusal` in `target/real-parts/` and its summary in the commit body;
  a unit test on a small recipe holds the classification. Decides step 2's
  go/no-go and whether steps 6–7 are what frees parts.
- [x] Step 2 **[1]** — Record the census in docs/ROADMAP.md §C6 and
  BACKLOG: the 14 parts, each with the residue that holds it; the order of
  steps 3–7 confirmed or changed against it. **Gate (idea's "change my
  mind" line):** if fewer than three of the 14 parts leave the column by
  steps 3 + 7 together, stop and ask the human before step 4. A change of
  order is recorded here, not silent.
- ~~Step 3~~ dropped 2026-10-03: the conic edge frees no part (step 2's gate). Backlog line kept.
- ~~Step 4~~ dropped with step 3.
- [x] Step 5 **[3]** — The decision and the tracer: write the ADR (a
  blend's end curve fitted, the surface exact; the fit's tolerance
  fraction; which checker arms decide it; what is refused when the fit
  misses) and reuse `section::traced` to trace the intersection of an
  exact blend surface with the face across. Riskiest unknown: the trace's
  robustness at a tangent start and the fit's miss rate. Test: the torus ×
  plane-parallel-to-the-axis end and the ruling-on-cylinder end traced on
  fixtures of step 1's list, the fit's distance from a dense sample within
  `SECTION_FIT_FRACTION · tol.linear`; a `regression/` fixture for every
  trace that fails, `#[ignore]`d with its desired assertion.
- [x] Step 6 **[3]** — The open arc's torus ending on a plane parallel to
  its axis and off it (the spiric section) and, by the same trace, on a
  cylinder across (the census's third family, leading `stc_06` and
  `stc_08`): the end trimmed by the fitted
  curve, the face across and the blend taking it as an edge with a
  fitted pcurve, the neighbours' edges shortened, provenance as ADR-0036
  §7 (end vertices `Generated`, shortened edges `Modified`). Fixtures with
  Open CASCADE's oracle (volume, area, centroid, counts, probes) at a
  radius where the ball fits; checker green at `Full`; one chamfer.
- [x] Step 7 **[3]** — The ruling stripe ending on a cylinder or cone
  square across (the quartic): the same machinery on the stripe's end;
  `Unsupported` stays for an oblique case the trace does not take, naming
  the face across. Fixtures with oracle as step 6; the regression twins
  (`ogive-bar-ruling-fillet` is the parallel-cylinder row and stays out)
  move into `blend/` when they pass.
- [x] Step 8 **[2]** — A property over random poses for every pair now
  taken to a blend by steps 3, 6 and 7 (`blend_prop`): checker green,
  volume additivity of the blended and the removed material, STEP
  round-trip, determinism of ids. Seeded; shards via `prop_shards!`.
- [ ] Step 9 **[1]** — Measure the fetched and committed tiers again
  (`tools/real-parts.sh`), print the column beside C4's 17 of 38 and
  step 1's line, move the `fixtures:` expectations whose refusals changed
  with the commit body saying why, and update `docs/ROADMAP.md` §C6's
  status paragraph with the new count and what each remaining part meets.

## Acceptance
- The column's per-part table of step 1 is reproduced and its summary line
  in §C6 is the number `tools/real-parts.sh` prints after step 9.
- Every fixture of steps 3, 6 and 7 passes under `blend/` with Open
  CASCADE's oracle (or its closed form under `analytic.measure_differs`
  with ADR-0015's evidence), checker green at `Full`, and the step 8
  property green at 256 cases (retirement) and 1000 (CI).
- Every part leaving the column does so either agreeing with Open CASCADE's
  fillet at the battery's radius or being refused as another cycle's
  (NURBS, run-over); the full profile (`ARRIS_GATE=full`) is green.

## Docs to update on completion
- `docs/ROADMAP.md` §C6 — status paragraph with the new `fillet` column
  counts and the lines this plan struck (the end families); the corners and the run-over named as the next plan's.
- `docs/BACKLOG.md` — the conic-edge line stays (it frees no part today:
  the census); the quartic pairs and the run-over as lines or ideas; the closed-form pairs left (parallel
  cylinders, apex plane) kept.
- `docs/adr/` — the new ADR (step 5), ADR-0007 and ADR-0036 get a
  one-line "Amended by" pointer.
- `docs/ARCHITECTURE.md` §blend and `docs/DATA-MODEL.md` §Tolerances — the
  fitted end curve and its tolerance, if they changed.
- `CHANGELOG.md` `Unreleased` — what a consumer can now blend and which
  refusals remain (no ADR numbers or fixture names).
- `AGENTS.md` current state — C6's third plan landed, the column's new
  count.

## Open questions
- Resolved 2026-10-03 (step 5, ADR-0037 §3): the end curve's fit is
  `SECTION_FIT_DEGREE` (5) at `SECTION_FIT_FRACTION` as ADR-0019; over 80
  posed ends none missed, the worst sample at 1.2e-8 against the 2.5e-8
  bound.
- Resolved 2026-10-03 (step 5, ADR-0037 §4): both pcurves are fitted from
  the 3D curve (`pcurve_on`), as C3's sections are.
- Found at step 5: the tracer's tests pose the three end families
  directly (`blend/traced.rs`) rather than on shrunk parts — no part
  fixture exists for an end the kernel does not build yet; steps 6 and 7
  add the part-shaped fixtures with Open CASCADE's oracle. No trace
  failed, so no `regression/` fixture was added.
- Found at step 6: a chamfer's cone ending on a plane parallel to its
  axis meets it in a hyperbola, which neither tracer takes (a plane has no
  quadric form in `trace_quadrics`); it stays `Unsupported` naming the cone
  and the plane, a backlog line (its exact form is `plane_cone`'s). The
  torus on both families and the cone on a cylinder across build;
  `blend/oblique-end-unsupported` was that family's refusal and is now
  `blend/d-notch-off-axis-rim-fillet`, built against the same oracle.
- Found at step 7: no `regression/` twin was an end family — the seven
  blend regressions fail as before on their own lines (the parallel-cylinder
  row, an edge off its plane, the checker fault at CTC-03, the turned
  stadium's torus pair, a fuse's fit, the walked blend's file), so none
  moved. A chamfer's plane on a cylinder or a cone across takes the
  intersector's exact conic between the trim points rather than a trace;
  `blend/rib-into-cone-chamfer` holds the ellipse.
- Found at step 8: `blend_prop`'s end families (a rib into a round or conical
  boss; a twin boss's foot) hold at 1000 cases, the volume against a
  closed-form section integral over the face across. One pose fails: a
  twin boss's chamfer cone against the second wall's cylinder is refused
  `Unsupported` after a turn about y (ratio 1.2/1.2, 0.3 apart, d 0.1 at 20°
  and 45°; at rest, 10° and 90° it builds). Shrunk to
  `regression/twin-boss-foot-turned-chamfer-cone-cylinder`, `#[ignore]`d;
  the property rejects that refusal alone, by name. The fix is a backlog
  line; step 9's count does not depend on it.
- Resolved 2026-10-03 (human): step 2's gate fired (steps 3, 6 and 7 as written free 2 of 15 parts; step 3 none); option (a) taken: steps 3–4 dropped, the torus-on-cylinder end added to step 6, which frees 3 (`ftc_08`, `ctc_01` ap242, `stc_08`). `fillet-by-part.md` has the rows.
