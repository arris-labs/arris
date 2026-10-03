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
trace can take are built: a blend whose contact would leave its face meets
the face it runs onto and ends on the trace of its surface with that face,
the surface exact. The fetched tier's `fillet` column (8 of 9) and the
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
- **ADR (step 3), new:** how a blend whose contact would leave its face
  through a non-corner edge is completed — the face across clips the blend
  at the trace of the blend surface with it, and the face's own boundary is
  re-cut there. Closes ADR-0007 §"The `BlendTooLarge` bound" ("a regression
  entry for C6") and amends it; cites ADR-0037 (tracers, fit) and ADR-0035
  (the chain walk). Shaped by step 1's sub-cause table; the numbering is the
  next free one at the time.
- `arris-ops` `blend.rs` / `blend/traced.rs`: the contact-leaves-face sites
  (`too_large` in the contacts, ring and seam paths) stop at the clip rather
  than refuse where the face across takes it. No public type or signature
  change expected; `BlendTooLarge` loses cases and keeps its variant. If a
  new `Reason` is wanted (for example a sub-cause naming) it is a design
  delta named in that commit, with the Python class, attributes and stub in
  the same commit (`kernel.md` §API).
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
- [ ] Step 3 **[3]** — The decision and its tracer: write the ADR (the clip
  on the face across; what the face's boundary becomes; the end curve fitted
  at `SECTION_FIT_FRACTION` as ADR-0037; what is refused when the clip
  misses) and prove the riskiest case first, a plane wall nearer the edge
  than `r` (the ADR-0007 "third face" case), by tracing the exact blend
  surface against it through `section::trace_section`. Test: the trace and
  fit on posed fixtures, the fit's distance from a dense sample within the
  bound; a `regression/` fixture, `#[ignore]`d with its desired assertion,
  for every trace that fails.
- [ ] Step 4 **[3]** — Build it for the plane wall: the face across's loop
  re-cut at the clip, the blend taking the trace as an edge with a fitted
  pcurve, the neighbours shortened, provenance as ADR-0036 §7 (new vertices
  `Generated`, shortened edges `Modified`). Fixtures with Open CASCADE's
  oracle (volume, area, centroid, counts, probes) at a radius where it
  builds; checker green at `Full`; one chamfer.
- [ ] Step 5 **[3]** — The same clip on a cylinder or a cone across, and on
  the ring rows (a cone, sphere or torus blend's contact running off its
  face), for the sub-causes the census says Open CASCADE builds. `Unsupported`
  stays, naming the face across, where the clip is oblique and the trace does
  not take it. Fixtures with oracle as step 4; `regression/` twins that now
  pass move into `blend/` with their blessed dump.
- [ ] Step 6 **[2]** — A property over random poses for every case steps 4
  and 5 build (`blend_prop`): checker green, volume additivity of the blended
  and the removed material, STEP round-trip, determinism of ids; poses beyond
  the clip's reach rejected by the refusal's name, not silently. Seeded;
  shards via `prop_shards!`.
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
- `docs/adr/` — the new ADR (step 3); ADR-0007's `BlendTooLarge`-bound
  paragraph gets an "Amended by" pointer; `docs/adr/README.md` row.
- `docs/ARCHITECTURE.md` §blend (the `BlendTooLarge` paragraph: which cases
  build now, which refuse) and `docs/DATA-MODEL.md` §Tolerances if the
  clip's end curve changed them.
- `CHANGELOG.md` `Unreleased` — which blends that ran out of their face now
  build and which refusals remain (no ADR numbers or fixture names).
- `AGENTS.md` current state — C6's fourth plan landed, the column's count.

## Open questions
- ⚠ OPEN: does the face across's boundary become the clip with a new edge,
  or does the blend overrun split the face across into two (a rib narrower
  than `r`: the blend meets both walls)? Agent decides at step 3 from step
  1's table; if both shapes occur, step 4 takes the one-wall case and the
  two-wall case is recorded for step 5 or the backlog.
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
- Order of steps 3–6 confirmed. The dominant cause (a corner edge shorter
  than the trim, plane × plane, 62 of `ctc_03` e2's 90 edges) is the case
  step 3 proves first, not the plane wall nearer the edge than `r` the plan
  named: the ADR's clip must hold both, and the sampled edges of the three
  parts are the fixtures' source.
