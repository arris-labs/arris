# Plan: blend-run-over

- Started: 2026-10-03
- Milestone: C6 (the blend network, docs/ROADMAP.md), fourth plan
- Idea: none. `blend-residue` (non-goals, decision 4) named the run-over as
  "its own idea after this plan"; the census it left (`fillet-by-part.md`)
  stands in for the brainstorm, and step 2's gate is the idea's "change my
  mind" line. If step 1 shows the run-over is mostly radii Open CASCADE also
  refuses, the plan stops there and the human is asked.
- Idea (verbatim from the human): "/plan blend-run-over"

## Goal
`BlendTooLarge` is counted by sub-cause, each edge against Open CASCADE's
verdict on it alone, so a refusal Open CASCADE shares is told from one it
builds. The sub-causes Open CASCADE builds and a closed form or ADR-0037's
trace can take are built: a blend running into a step — a corner of mixed
convexity, the trim past the vertex — lengthens its corner edge of the
blend's convexity and ends on the face across as every corner end does,
the surface exact (ADR-0038; step 3 found this, not a clip on a face run
onto, to be the run-over). The fetched tier's `fillet` column (8 of 9) and the
committed tier's (6 of 11) are measured again beside C4's 17 of 38 and
`blend-residue`'s line, and what is left of the run-over is named by part.

## Non-goals
- Radii Open CASCADE also refuses: they stay `BlendTooLarge`, correctly
  (`both refuse` in the battery).
- The horn torus (`regression/tangent-chain-horn-torus`): a surface kind the
  ring-torus rule excludes, not a run-over. Stays a backlog line.
- Corners (`VertexBlend`), tangent continuation (`TangentChain`), the
  crossing-cylinder and torus-off-axis pairs (NURBS cycle), variable radius,
  blends over blends, other chamfer modes.
- A fitted blend *surface* (ADR-0007's rejected alternative; ADR-0037 keeps
  the surface exact and fits only the end curve).
- NURBS faces as operands, healing.

## Design deltas
- **ADR-0038 (step 3), new:** a blend running into a step — a corner whose
  two edges differ in convexity — lengthens the corner edge of the blend's
  convexity past the vertex to its trim, cuts the other, and the face
  across takes the end arc from outside; amends ADR-0007 (the end, the
  `BlendTooLarge` bound, the mixed corner's `VertexBlend`).
- `arris-ops` `blend/mixed.rs` (`corner_trims`, step 3) replaces
  `cut_corner` and `end_side` at a face end (step 4) and at the ring's open
  arc ends (step 5); `build` re-derives a lengthened edge's pcurves over its
  new range (ADR-0038 §4). No public type or signature change;
  `BlendTooLarge` loses cases and keeps its variant, `Reason` gains none.
- `arris-check`: expected no change (S5 decides the blend against the face
  across by the same tracers, ADR-0037 §5); step 4 shows it at `Full`.
- `arris-debug` `census.rs`: the sub-cause column and Open CASCADE's verdict
  per edge (step 1).
- `docs/ARCHITECTURE.md` §blend, `docs/DATA-MODEL.md` §Tolerances if the
  clip's end curve carries its own tolerance; ROADMAP §C6.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The census of `BlendTooLarge` by sub-cause. Extend
  `arris_debug::census` so every `BlendTooLarge` edge (each blended alone,
  at the battery's radius) is classified by the site that refused it
  (contact leaves its face through which kind of edge; a corner edge shorter
  than the trim; a seam; a ring's apex or closing; a fit that failed) and by
  **Open CASCADE's verdict on that edge alone at that radius** (builds /
  refuses, through `tools/oracle/`). Output `target/real-parts/run-over.md`:
  `part → sub-cause → count → OCC builds / refuses`; a unit test on a small
  recipe per sub-cause holds the classification. Decides step 2's go/no-go.
- [x] Step 2 **[1]** — Record the census in docs/ROADMAP.md §C6 and
  BACKLOG: the sub-causes, each with its count and Open CASCADE's verdict,
  and the order of steps 3–6 confirmed or changed. **Gate:** if the edges
  Open CASCADE builds are fewer than three parts' worth of the fetched
  tier's column, or fall in a sub-cause with no closed form and no trace,
  stop and ask the human before step 3. A change of order is recorded here,
  not silent.
- [x] Step 3 **[3]** — The decision and its trims (reshaped by what the
  step found, Open questions): ADR-0038, the mixed corner; `blend/mixed.rs`
  `corner_trims` — each trim located on its corner edge, a trim past the
  vertex lengthening the edge of the blend's convexity at a mixed corner
  only, its curve analytic and the stretch inside its face, the arc's side
  outside the face across — tested on posed L-steps (fillet and chamfer,
  upright and leaning both ways) against the closed forms, the far box
  corner unchanged, and a hole across the stretch refused naming the corner
  edge; run over every corner end of four fetched parts (170 mixed-corner
  ends lengthen, none refused). `regression/fillet-into-a-step`, the shrunk
  CTC-03 corner with Open CASCADE's oracle, `#[ignore]`d.
- [x] Step 4 **[3]** — Build it for a plane across: `face_end` takes
  `corner_trims`; `build` lengthens the edge, its pcurves derived again
  over the new range (ADR-0038 §4), the face across taking the arc from
  outside; provenance as ADR-0007 roots an end. `regression/fillet-into-a-step`
  moves to `blend/` with its blessed dump; a chamfer twin, a leaning step
  and a concave blend at a mixed corner, each with Open CASCADE's oracle
  and `counts_differ` (ADR-0038 §5); checker green at `Full`.
- [ ] Step 5 **[3]** — The mixed corner on a cylinder or a cone across (the
  end traced, ADR-0037) and at the ring rows' open arc ends (`ring`'s
  `cut_corner` and `end_side`), for the 16 such ends of the four parts
  probed and what the census adds. `Unsupported` stays, naming the face
  across, where the trace does not take it. Fixtures with oracle as step 4.
- [ ] Step 6 **[2]** — A property over random poses for every case steps 4
  and 5 build (`blend_prop`): checker green, volume additivity of the blended
  and the removed material, STEP round-trip, determinism of ids; poses beyond
  the lengthening's reach (a stretch out of its face) rejected by the
  refusal's name, not silently. Seeded; shards via `prop_shards!`.
- [ ] Step 7 **[1]** — Measure the fetched and committed tiers again
  (`tools/real-parts.sh`), print the column beside C4's 17 of 38 and
  `blend-residue`'s line, move the `fixtures:` expectations whose refusals
  changed (the commit body saying why; `cancel_counts.txt` blessed for the
  new fixtures), and update `docs/ROADMAP.md` §C6's status paragraph with the
  new count and what each remaining part meets. Run the docs-refs tests: the
  roadmap's histogram is checked against the printout.

## Acceptance
- The sub-cause table of step 1 is reproduced, and the column's summary line
  in §C6 is the number `tools/real-parts.sh` prints after step 7.
- Every fixture of steps 4 and 5 passes under `blend/` with Open CASCADE's
  oracle (or its closed form under `analytic.measure_differs` with ADR-0015's
  evidence), checker green at `Full`; the step 6 property is green at 256
  cases (retirement) and 1000 (CI).
- Every part leaving the column does so agreeing with Open CASCADE's fillet
  at the battery's radius, or is refused as another cycle's (corners, NURBS,
  a radius Open CASCADE refuses); the full profile is green.

## Docs to update on completion
- `docs/ROADMAP.md` §C6 — status paragraph with the new `fillet` counts and
  the line this plan struck (the run-over); corners and tangent
  continuation named as the next plan's.
- `docs/BACKLOG.md` — what the census leaves (radii Open CASCADE refuses are
  not backlog; the sub-causes with no closed form are), the horn-torus line
  kept.
- `docs/adr/` — ADR-0038, its ADR-0007 pointer and README row (done at
  step 3).
- `docs/ARCHITECTURE.md` §blend (the `BlendTooLarge` paragraph: which cases
  build now — the mixed corner — and which refuse); `docs/DATA-MODEL.md`
  §Tolerances only if a lengthened edge's tolerance changed.
- `CHANGELOG.md` `Unreleased` — that a blend running into a step now
  builds and which refusals remain (no ADR numbers or fixture names).
- `AGENTS.md` current state — C6's fourth plan landed, the column's count.

## Open questions
- Resolved at step 3 (agent): the face across is neither clipped nor split;
  at a mixed corner it grows by the end's region (ADR-0038 §2).
- Resolved at step 1 (agent): the census asks `occt_fillet_edges.py` once
  per solid, reading its STEP once, for a stride of twelve edges per cause
  and every edge of the battery's sample; both tiers take about five
  minutes, cached after. Every cause is present in the table.
- Found at step 1: the cause is read from the entities the refusal names
  (`run_over_cause`), so no `Reason` and no `arris-ops` change was needed.
  The census over both tiers is in ROADMAP §C6. Open CASCADE builds 205 of
  the 285 fetched-tier edges it was asked, every one on a plane against a
  plane or a cylinder; *the ball finds no place on either face* (89 edges)
  and *the contact leaves a torus face* (24) it refuses too, so they are not
  this plan's. The six sampled edges of the fetched tier build in Open
  CASCADE: clearing them takes `ctc_03` e2, `ftc_08` e2 and `stc_06` out of
  the column (three, the gate's bound), and `ctc_04`, `stc_09` wait on their
  `VertexBlend` and `TangentChain`. Step 2's gate did not fire.
- Found at step 3 (agent): probed by site on four fetched parts (CTC-01,
  CTC-03, FTC-08, STC-06; 277 of the tier's 515 edges), every *corner edge
  shorter than the trim* (178) is a trim past the vertex at a mixed corner,
  every *contact leaves a plane / cylinder face* (98) is the ring's contact
  at the axis — a horn or spindle torus, ADR-0036 §3, a surface-kind
  question this plan does not take — and no blend runs onto a third face.
  The plan's clip did not exist; steps 3–5 are reshaped to the mixed corner
  (ADR-0038), and the open question above on a face across split in two is
  moot. STC-06's sampled edges are the ring's, so the parts this plan can
  clear are CTC-03 and FTC-08: two, under step 2's gate of three. Decided
  (agent, the human delegates such calls): go on. The mixed corner is the
  largest single refusal of the blend network (178 of the 277 edges probed),
  Open CASCADE builds it, and the gate's bound counted parts where the
  edges are the measure; the census's mislabel and the third-face case are
  backlog lines.
- Found at step 4 (agent): Open CASCADE builds the leaning step's end with
  exact conics but off them: its volume is 2.4e-9 (leaning back) and 1.6e-9
  (overhanging) relative from the closed form and its area 5.9e-9 and
  4.2e-9. Its fixed-order, adaptive and Gauss–Kronrod integrations agree,
  so the error is in its geometry, and it turns sign with the lean. Arris
  matches both closed forms to 1e-11. The difference is under ADR-0015's
  1e-6, so `blend/fillet-into-a-leaning-step` states volume and area
  tolerances of 1e-8 with the closed forms beside them, as
  `blend/turned-shoulder-fillet` does; no kernel tolerance moved. A mixed
  corner on a cylinder or a cone across keeps its refusal from before
  ADR-0038 (`BlendTooLarge` naming the lengthened corner edge) until step 5.
- Order of steps 3–6 confirmed at step 2. The dominant cause (a corner edge shorter
  than the trim, plane × plane, 62 of `ctc_03` e2's 90 edges) is the case
  step 3 proves first, not the plane wall nearer the edge than `r` the plan
  named: the ADR's clip must hold both, and the sampled edges of the three
  parts are the fixtures' source.
