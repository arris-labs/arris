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
`sweep/sweep-thread-chamfer-unsupported` builds. CTC-02's box cut is
profiled and its hot spot fixed, for C10's accept on the fetched tier.

## Non-goals

- **Two NURBS faces flush over an area**: refused `Unsupported` (Q4,
  BACKLOG §79). The boolean's coincident arm stays same-kind.
- **`CrowdedSingularity`** on a cone twin against a plane square to its
  axis, and some sphere × cylinder twins (BACKLOG §76): a typed refusal
  the properties tolerate, 3 in 192 at step 2.
- **A touch of two NURBS curves** missed by `extrema_curve_curve` and
  the fuzz target's slow units (BACKLOG §77–78).
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
- **Step 2's finding**: two equal cylinders on crossing axes (the
  Steinmetz pair) are refused `DegenerateSection` both twinned, always,
  and with one twinned mostly: neither the marcher nor ADR-0061's tracer
  accounts for the points where the walls touch. The `geometry-fault`
  exclusion had hidden the one-twin cases; `twin_prop` names the arm.
  Of 192 both-twin booleans: box × cylinder 64 agree, quadric pairs 56
  (3 `CrowdedSingularity`, 5 `Lumps`), crossing pairs 0. Step 5 is new.
- **Step 5's finding**: the marcher's reach at a singular point was
  12.4 at rest, not 5e-4: `NO_NORMAL` was a share of the nets' diagonal
  squared, and a twin 600 long has none of its normals above it. It is
  now a share of the surface's own speeds. Beyond that, ADR-0068's
  marcher finds every singular point before it accounts for continua,
  drops a seam's seed inside a reach (an arm crossing the seam), and
  isolates the departures over the patches near the ball, halved to a
  few reaches. ADR-0061's critical points halve by look-ahead: on such
  a twin the scored halving left the touch uncertified. Timing is the
  same (`meridian_rows` shards ±1 s). No ADR: within both designs, no
  type changes.
- **Step 7's finding**: no spine code changed. `trace_section` already
  marches two NURBS offsets (ADR-0068), so lifting `roll`'s refusal of two
  NURBS faces walks the fillet and the chamfer; ADR-0062's amendment holds
  the evidence. Open CASCADE's walked blend on the twisted loft misses the
  ball by `1.0e-3` at `r` 0.5 (its area `1.6e-5` off, past the walked
  cap), so the fixture is at `r` 0.25 (`1.3e-4`). Its thread chamfer is
  `1.4e-3` short of the removed volume's closed form, which Arris meets
  to `2.4e-8`. The thread's `Full` check is 1207 s, `sweep-thread`'s
  own already 463 s since the marcher: S5 on long NURBS neighbours
  (BACKLOG), both fixtures in the slow set. `twin_prop`'s meridian
  shards at 16 cases take 455 s wall against 272 s with one face twinned.
- **`arris-check`** (steps 3–4): S5 reads a NURBS pair's section from
  `trace_section`, the traced branches' own points, never the fitted
  curves. Step 3's finding: the loft "false positive" was a real fold (the
  circle's start unmatched to the rectangle's twists the rulings, mesh
  slices cross themselves), so S5 was right; the pairs a shared edge's
  neighbours leave to the intersector's refusals stay `Unchecked::FacePair`
  and the properties' exemptions stay. B1's
  shell nesting with NURBS faces on both shells classifies by the ray
  fire over NURBS faces (C8's query cycle). `Unchecked::FacePair` and
  `ShellFacePair` remain only for an intersector refusal; no type
  changes.
- **`arris-ops::boolean`** (step 6): the pave on two NURBS faces: the
  marched section's ends against each face's edges (the curve × surface
  arm, ADR-0068 step 4), its crossings of either face's seam paved, its
  pcurves the walk's own. Step 2's property found no pave fault, and
  step 6's fixtures none: no code changed. One fixture trap, now in
  `tests/fixtures/README.md`: a sweep's profile tilted to a non-planar
  path moved a small common's centroid 2.6e-4, Open CASCADE's corrected
  Frenet not being Arris' rotation-minimising frame. Squared, the common
  on the unfilleted box agrees to 1e-9 with Open CASCADE's pipe at
  `SetTolerance` 1e-8; on the filleted one the walked blend's oracle
  miss is 1.7e-6 of its volume, under ADR-0062's 1e-5.
- **ADR-0062 amendment** (step 7): the two-B-spline row. A walked
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
- [x] Step 2 **[3]** — **The twin property with both operands twinned.**
  Half the cases twin both. No pave finding; the Steinmetz refusal
  (deltas) is an ignored `nurbs_nurbs_prop` test, since Open CASCADE's
  own NURBS booleans of the pair fail (no oracle for a fixture).
- [x] Step 3 **[2]** — **S5 on two NURBS faces.** The loft "false
  positive" was a true fold (deltas); its fixture is replaced by a
  matched-start `loft/loft-rect-circle-rect-ruled` and a `loft.rs` test
  holding the folded one to `FacesIntersect`. S5 samples a traced
  branch's own points instead of fitting it (`trace_section`): CTC-02's
  box cut 256 s to 176 s. The `FacePair` exemptions stay (deltas).
- [x] Step 4 **[2]** — **B1 with NURBS faces on both shells.** The
  nesting was never the ray fire: the meeting test refused pairs whose
  face boxes were the control hull of every span touched, so two lumps on
  one B-spline wall overlapped. `Surface::bounds` and `Curve::bounds` clip
  a NURBS to the range (the clipped control points), and a pair the tracer
  refuses is apart when 256 clipped boxes a face (`bounds_pieces`) are
  clear of each other. `regression/nurbs-cylinder-slab-cut-two-lumps` is
  `boolean/` with its dump. `twin_prop`'s `Lumps` arm is narrowed, not
  lifted (open question).
- [x] Step 5 **[3]** — **The section at the tangent points of a
  Steinmetz pair.** Found first, before continua and crowded boxes are
  refused (deltas). `the_twins_of_two_equal_cylinders_on_crossing_axes_meet_in_two_ellipses`
  passes with either twinned and with both at rest; the tilted pair both
  twinned is its own ignored test (open question). `twin_prop`'s section
  arm keeps `CrowdedSingularity` alone (28 shards at 64 cases).
- [x] Step 6 **[3]** — **The pave on fitted pairs, against Open CASCADE.**
  Six fixtures, passing with no code changed (deltas): a spline sweep cut
  by a spline sweep crossing it, a smooth loft fused with a sweep, the
  common of two lofts across both seams, and the bicubic top's walked
  fillet cut by, fused with and common with a spline sweep.
- [x] Step 7 **[3]** — **The walked stripe between two NURBS faces;
  ADR-0062 amendment.** The spine as the marched section of the offset
  fits. `sweep-thread-chamfer-unsupported` builds as Open CASCADE builds
  it (renamed `sweep-thread-chamfer`), plus a fillet on a ruled loft's
  edge between two NURBS faces. `twin_prop`'s blends twin both faces.
  Lifting `roll`'s refusal was all (deltas); both fixtures pass at the
  walked allowance, the oracle's miss measured.
- [ ] Step 8 **[1]** — **Python and the corpus.** The binding's tests
  cut a sweep by a loft and chamfer the thread; `CHANGELOG.md`
  `Unreleased` bullets; the fetched tier's histogram re-run and printed
  for C10's accept, the NURBS rows 0 and the blend residue's row beside.

## Acceptance

- `cargo nextest run -p arris-ops -E 'test(twin_prop) | test(loft_prop) | test(sweep_prop)'`
  at 256 cases: both operands twinned, no `Lumps` exemption (the
  `FacePair` one stays, step 3); `CrowdedSingularity` the only refusal tolerated (3 in 192
  at step 2).
- The step 6 and 7 fixtures and the two moved `regression/` fixtures pass
  the corpus runner against Open CASCADE (measures within the fixture's
  tolerance, counts, probes, dump).
- CTC-02 AP242's `box_cut` corpus run at most step 3's 176 s.
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
- `CHANGELOG.md` `Unreleased` (step 8). `GLOSSARY.md`: no new term
  expected; *Walked stripe* sharpened if ADR-0062's amendment changes it.

## Open questions

- Settled (human, 2026-10-19): the speed bar is the CTC-02 AP242
  `box_cut` under 60 s in the test profile (from 680 s); short of it, the
  hot spot found and fixed, the rest a narrowed backlog line. Step 1
  reached 256 s; the rest is five stages of 16 to 74 s (BACKLOG §82).
- Settled (agent, step 2): `CrowdedSingularity` (§76) refused 3 of 192,
  under the one-in-ten bar, and stays a backlog line; the Steinmetz
  refusals (42 of 192) are their own cause, step 5.
- Settled (agent, step 3 inquiry): the loft "false positive" is neither
  the marcher's nor the checker's. The fixture's ruled loft genuinely
  folds: mesh slices of the body at z 0.6–0.7 and 1.24–1.33 cross
  themselves (the circle's start does not match the rectangle's, so the
  rulings twist), and Open CASCADE's measures say nothing of validity.
  S5's five findings are true positives; the fixture's "desired clean" is
  wrong and `loft_prop`'s `FacesIntersect` exemption hides real folds of
  random lofts.
- Settled (human, step 3, agent's recommendation): S5 cannot decide every
  NURBS pair while the intersector refuses the neighbours of a shared edge
  (`Unsupported` for a flush split, `DegenerateSection` `TubeCircle`,
  `UnresolvedTurning`, `CrowdedSingularity`; `twin_prop`, 32 cases). The
  `FacePair` exemptions of `twin_prop` and `loft_prop` stay, and step 3
  took the sampling saving and the true-positive fixtures only. What would
  lift them is the intersector deciding shared-edge neighbours: BACKLOG.
- Open (human, step 4): `twin_prop`'s `Lumps` arm (acceptance: "no
  `Lumps` exemption") stays, narrowed. With the boxes clipped and the
  refused pairs quartered where they overlap, the box × cylinder cases
  and most quadric ones decide; 2 of 8 quadric shards at 16 cases still
  leave a pair whose (u, v) regions coincide on one twin surface to
  1e-5 — two lumps that touch along a curve or overlap, which B1 may
  rightly refuse. Whether those are the twin's fault (a boolean that
  returns touching shells) or B1's is undiagnosed; it wants a shrunk
  `regression/` fixture, BACKLOG. Lifting the arm, or accepting it as the
  tolerated refusal, is the human's.
- Settled (agent, step 5): the tilted Steinmetz pairs both twinned stay
  refused (`TubeCircle` or `CrowdedSingularity`), held by the ignored
  `the_tilted_twins_of_two_equal_cylinders_on_crossing_axes_meet_in_two_ellipses`,
  with a backlog line. The loop condition vanishes on the product of the
  two rulings through each tangent point, and on twins 600 long in a pose
  that mixes the axes `bernstein_n` halves the short parameters to the
  bottom and never the long ones. Each of four halving rules tried (a
  value-weighted score, polynomials clear of their floor only, the
  variable nearest exclusion, halving past a flat polynomial) broke a
  `march_` test or tripled the survey's time. That is a change to the
  isolation's design, not this step's. `twin_prop` drew none at 64
  cases; if 256 does, its `TubeCircle` arm comes back for that pair alone.
