# Plan: blend-residue

- Started: 2026-10-03
- Milestone: C6 (the blend network, docs/ROADMAP.md), third plan
- Idea: docs/ideas/blend-residue.md (absorbed)
- Idea (verbatim from the human): "the blend residue"

## Goal
The fetched tier's `fillet` column (8 parts) and the committed tier's (6 of
11) are measured **by part**, each part's first refusal named and what
stands behind it; an edge a B-spline writes where two analytic faces meet
in a circle or a line blends as that circle or line; and the two end
families ADR-0036 §6 postponed (an open arc's torus ending on a plane
parallel to its axis and off it; a ruling stripe ending on a cylinder or
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

## Design deltas
- **ADR (step 5), new:** a blend's end curve may be traced and fitted while
  its surface stays exact. Amends ADR-0007 §Consequences ("nothing is
  fitted but a pcurve") and closes ADR-0036 §6's deferral; cites ADR-0019's
  `SECTION_FIT_FRACTION` rule. Whether the end curve's fit is a new
  `arris-geom` entry or `section::traced` reused is decided by step 5's
  read of the code.
- `arris-ops` `blend.rs`: edge curve recognition (step 3) and two end
  trimmers (steps 6, 7). No public type or signature changes expected;
  `Unsupported` loses cases, no `Reason` variant is added. If step 5 finds
  otherwise it is a design delta named in that commit.
- `arris-check`: the S5/E4 rows decide a fitted end curve on a blend at the
  arc's tolerance (step 5 states which arms, step 6 adds them).
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

- [ ] Step 1 **[2]** — The re-census by part. Extend the survey's fillet
  stage (`arris_debug::survey`, `tools/real-parts.sh`) to print, for every
  part in the column of both tiers, the **first refusal at the battery's
  radius** (kind, faces, how they sit) and the refusals the part's other
  edges would meet once that one is cleared (each edge blended alone, as
  step 1 of `c6-blend-pairs` did). Output: a table `part → first refusal →
  next refusal` in `target/real-parts/` and its summary in the commit body;
  a unit test on a small recipe holds the classification. Decides step 2's
  go/no-go and whether steps 6–7 are what frees parts.
- [ ] Step 2 **[1]** — Record the census in docs/ROADMAP.md §C6 and
  BACKLOG: the 14 parts, each with the residue that holds it; the order of
  steps 3–7 confirmed or changed against it. **Gate (idea's "change my
  mind" line):** if fewer than three of the 14 parts leave the column by
  steps 3 + 7 together, stop and ask the human before step 4. A change of
  order is recorded here, not silent.
- [ ] Step 3 **[2]** — The conic edge: an edge whose curve is a B-spline
  within the edge's tolerance of the circle or line where its two analytic
  faces meet is blended as that circle or line (the faces' own
  intersection, ADR-0018's closed forms, not a refit of the B-spline).
  Refused as before when it is not within tolerance. Fixtures: a plane ×
  cylinder and a plane × plane filleted and chamfered with the edge
  written as a B-spline (recipe option on the fixture builder; oracle:
  Open CASCADE's blend of the same recipe, Pappus forms by closed form).
  Property: random pose, the B-spline-written solid and the analytic one
  blend to the same volume and counts.
- [ ] Step 4 **[2]** — Move the 287-edge census through step 3 on the
  fetched tier; record in the commit body how many edges and parts leave
  `Unsupported(… B-spline curve …)`; shrink a leftover into
  `regression/` if one does not build. (Skip if step 3's own commit
  already carries the numbers.)
- [ ] Step 5 **[3]** — The decision and the tracer: write the ADR (a
  blend's end curve fitted, the surface exact; the fit's tolerance
  fraction; which checker arms decide it; what is refused when the fit
  misses) and reuse `section::traced` to trace the intersection of an
  exact blend surface with the face across. Riskiest unknown: the trace's
  robustness at a tangent start and the fit's miss rate. Test: the torus ×
  plane-parallel-to-the-axis end and the ruling-on-cylinder end traced on
  fixtures of step 1's list, the fit's distance from a dense sample within
  `SECTION_FIT_FRACTION · tol.linear`; a `regression/` fixture for every
  trace that fails, `#[ignore]`d with its desired assertion.
- [ ] Step 6 **[3]** — The open arc's torus ending on a plane parallel to
  its axis and off it (the spiric section): the end trimmed by the fitted
  curve, the face across and the blend taking it as an edge with a
  fitted pcurve, the neighbours' edges shortened, provenance as ADR-0036
  §7 (end vertices `Generated`, shortened edges `Modified`). Fixtures with
  Open CASCADE's oracle (volume, area, centroid, counts, probes) at a
  radius where the ball fits; checker green at `Full`; one chamfer.
- [ ] Step 7 **[3]** — The ruling stripe ending on a cylinder or cone
  square across (the quartic): the same machinery on the stripe's end;
  `Unsupported` stays for an oblique case the trace does not take, naming
  the face across. Fixtures with oracle as step 6; the regression twins
  (`ogive-bar-ruling-fillet` is the parallel-cylinder row and stays out)
  move into `blend/` when they pass.
- [ ] Step 8 **[2]** — A property over random poses for every pair now
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
  counts and the lines this plan struck (the end families, the B-spline
  conic edge); the corners and the run-over named as the next plan's.
- `docs/BACKLOG.md` — the conic-edge line removed; the quartic pairs
  and the run-over as lines or ideas; the closed-form pairs left (parallel
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
- ⚠ OPEN: does step 1's per-part table say the B-spline-written conic is
  the first refusal of any part, or only a refusal behind others?
  Agent decides at step 2 (the gate above); the human is asked only if
  fewer than three parts leave the column.
- ⚠ OPEN: the end curve's fit degree and fraction — `SECTION_FIT_DEGREE`
  (5) and `SECTION_FIT_FRACTION` as ADR-0019, or tighter for an edge a
  neighbour must share. Agent decides at step 5 from the trace's miss
  rate on the fixtures.
- ⚠ OPEN: whether the fitted end curve's pcurve on the blend face is a
  fit from the 3D curve, as C3's sections, or traced in `(u, v)`. Agent,
  step 5.
