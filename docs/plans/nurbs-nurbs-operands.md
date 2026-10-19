# Plan: nurbs-nurbs-operands

- Started: 2026-10-19
- Milestone: C10, the NURBS cycle (docs/ROADMAP.md §C10), its third item,
  M2 of two; the cycle's last plan before `/close-cycle`
- Idea: docs/ideas/nurbs-nurbs.md (M2 absorbed, the file deleted; settled
  there: option C, flush faces stay refused (Q4), M2 amends ADR-0062's
  two-B-spline row (Q5), profiling the NURBS × plane walk first (Q8), the
  walked stripe between two NURBS faces in scope (Q9)). M1's section is
  ADR-0068.

## Goal

Two NURBS faces are operands of each other. A sweep cut by a sweep, a
loft fused with a loft, a fitted blend face against a fitted sweep face:
each boolean gives a body clean at `Full` whose measures match Open
CASCADE's, or a typed refusal naming the pair. The checker decides S5
and B1 on two NURBS faces, with no false positive along an edge they
share, and the twin property holds with **both** operands twinned. An
edge between two NURBS faces fillets and chamfers as a walked stripe, so
`sweep/sweep-thread-chamfer-unsupported` builds. The NURBS × plane walk
under it is profiled and its hot spot fixed, so C10's accept can be
re-run on the fetched tier in reasonable time.

## Non-goals

- **Two NURBS faces flush over an area**: refused `Unsupported` (Q4,
  BACKLOG §79). The boolean's coincident arm stays same-kind.
- **`CrowdedSingularity`** on a cone twin against a plane square to its
  axis, and some sphere × cylinder twins (BACKLOG §76): a typed refusal
  the properties tolerate, unless step 2 measures it past the bar in the
  open questions.
- **A touch of two NURBS curves** missed by `extrema_curve_curve`'s
  settle (M1's open question, ignored test), and the fuzz target's slow
  units (BACKLOG §78).
- **Draft and thicken**: the side slot, after `modelling-layer`.
- **Speed beyond step 1's hot spot**: the reader's B-spline read-backs
  (BACKLOG §96–97) and the walked blend's span count (§84).

## Design deltas

- **Step 1's finding**: CTC-02 AP242's `box_cut` corpus run (676 s)
  was not the walk. Sampled, 75 % was the weld stage's 640
  `tessellate_faces` calls, each running the debug `Fast` check, whose
  L4 discretised every loop at the parametric tolerance (6.5 M segments,
  0.68 s a check). L4 now decides on the coarsest polygons that answer
  beyond their deviation (`docs/DATA-MODEL.md` §Invariants, L4): 0.04 s
  a check, the run 256 s. No ADR: a checker-internal cost, with the
  answers unchanged.
- **`arris-check`** (steps 3–4): S5 and B1 read the marched section as
  they read a traced one. A section that runs along an edge two faces
  share is that edge, not a crossing: the false positive of
  `regression/loft-rect-circle-rect-ruled-s5-false-positive` is fixed
  where it is (the marcher returning a curve off the edge, or
  `curve_is_interior_to_both` not excusing it), found at step 3. B1's
  shell nesting with NURBS faces on both shells classifies by the ray
  fire over NURBS faces (C8's query cycle). `Unchecked::FacePair` and
  `ShellFacePair` remain only for an intersector refusal; no type
  changes.
- **`arris-ops::boolean`** (step 2, 5): the pave on two NURBS faces: the
  marched section's ends against each face's edges (the curve × surface
  arm, ADR-0068 step 4), its crossings of either face's seam paved, its
  pcurves the walk's own. What breaks is found by step 2's property, each
  finding a `regression/` fixture moved by its fix.
- **ADR-0062 amendment** (step 6): the two-B-spline row. A walked
  stripe between two NURBS faces takes its spine as the marched section
  of the two faces' offset fits, the contacts by projection as today; the
  row becomes built and `Unsupported` stays only for the stripe's end
  across a B-spline face (the next row) and for a flush pair.
- **Public API**: no signature changes. `blend::fillet` and `chamfer`
  lose an `Unsupported` case; `CHANGELOG.md` says what a caller can now
  cut, fuse and blend.

## Steps

Complexity grades: **[1]** routine, the design says exactly what to write
and the tests are mechanical; **[2]** careful, a geometric or numeric case
to get right within a given design; **[3]** unproven, an algorithm whose
robustness or bound has to be established here.

- [x] Step 1 **[2]** — **Profile CTC-02's box cut and drill.** Found
  the debug check's L4, not the walk (deltas); L4 decides coarse first,
  held by two near-rim tests in `fast_part2.rs`. The run is 256 s, short
  of the 60 s bar: BACKLOG §82 narrowed to what is left, S5's fit moved
  to step 3.
- [ ] Step 2 **[3]** — **The twin property with both operands twinned.**
  `twin_prop`'s booleans draw every analytic pair with both bodies'
  faces replaced by their `to_nurbs` twins, held to the analytic
  result's measures and counts. Every failure is shrunk into a
  `regression/` fixture; the pave fixes it brings land here when small,
  as their own step 5 otherwise. The refusal rate is measured.
- [ ] Step 3 **[2]** — **S5 on two NURBS faces.** The loft false
  positive fixed; the fixture moves to `loft/` with its dump. S5 samples
  a traced branch's own points instead of fitting it first (~74 s of
  the CTC-02 box cut's run).
  `loft_prop` and `twin_prop` drop their `FacePair` exemptions, and a
  `full.rs` test holds an all-twin body clean with nothing unchecked.
- [ ] Step 4 **[2]** — **B1 with NURBS faces on both shells.** Nesting by
  the ray fire over NURBS faces; `regression/nurbs-cylinder-slab-cut-two-lumps`
  moves to `boolean/` with its dump, and `twin_prop`'s `Lumps` arm is
  lifted.
- [ ] Step 5 **[3]** — **The pave on fitted pairs, against Open CASCADE.**
  Fixtures with oracle values: a spline sweep cut by a spline sweep
  crossing it, a smooth loft fused with a sweep, a loft through a loft
  across both seams, a walked blend face cut by a sweep (fuse, common and
  cut each). The pave findings step 2 left are fixed here.
- [ ] Step 6 **[3]** — **The walked stripe between two NURBS faces;
  ADR-0062 amendment.** The spine as the marched section of the offset
  fits. `sweep-thread-chamfer-unsupported` builds as Open CASCADE builds
  it (renamed `sweep-thread-chamfer`), plus a fillet on a ruled loft's
  edge between two NURBS faces. `twin_prop`'s blends twin both faces.
- [ ] Step 7 **[1]** — **Python and the corpus.** The binding's tests
  cut a sweep by a loft and chamfer the thread; `CHANGELOG.md`
  `Unreleased` bullets; the fetched tier's histogram re-run and printed
  for C10's accept, the NURBS rows 0 and the blend residue's row beside.

## Acceptance

- `cargo nextest run -p arris-ops -E 'test(twin_prop) | test(loft_prop) | test(sweep_prop)'`
  at 256 cases: both operands twinned, no `FacePair` or `Lumps`
  exemption; `CrowdedSingularity` the only refusal tolerated, below the
  bar of the open question.
- The step 5 and 6 fixtures and the two moved `regression/` fixtures pass
  the corpus runner against Open CASCADE (measures within the fixture's
  tolerance, counts, probes, dump).
- CTC-02 AP242's `box_cut` corpus run at most step 1's 256 s, less
  step 3's S5 saving.
- The full gate is green; the fetched tier's histogram is printed with
  NURBS 0 of 38 at every column.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §the operands paragraph and the `Unsupported`
  row: two NURBS faces in the boolean and the blend; §`arris-check`: S5
  and B1 decide them.
- `docs/DATA-MODEL.md` §Provenance only if the stripe's provenance
  differs from ADR-0062's (it should not).
- `docs/ROADMAP.md` §C10 third item: done, with the fixtures, the
  properties and the histogram; `AGENTS.md` current state; next is
  `/close-cycle`.
- `docs/BACKLOG.md`: §75 and §87 dropped, §82 narrowed again by step 3,
  §76 updated with step 2's rate.
- `CHANGELOG.md` `Unreleased` (step 7). `GLOSSARY.md`: no new term
  expected; *Walked stripe* sharpened if ADR-0062's amendment changes it.

## Open questions

- Settled (human, 2026-10-19): the speed bar is the CTC-02 AP242
  `box_cut` under 60 s in the test profile (from 680 s); short of it, the
  hot spot found and fixed, the rest a narrowed backlog line. Step 1
  reached 256 s; the rest is five stages of 16 to 74 s (BACKLOG §82).
- ⚠ OPEN (agent, by step 2): the refusal bar. If `CrowdedSingularity`
  refuses more than one twin×twin boolean in ten, its cause (BACKLOG
  §76) joins this plan as a step before step 5; otherwise it stays a
  backlog line.
- ⚠ OPEN (agent, by step 3): whether the loft false positive is the
  marcher's (a section off the shared edge, an `arris-geom` fix with a
  `geom/` fixture) or the checker's; the fixing commit says which.
