# Plan: overhang-tip-cap

- Started: 2026-10-31
- Milestone: M5 (cycle C11, docs/ROADMAP.md §C11)
- Idea: docs/ideas/blend-residue-real-parts.md (absorbed: M5, its last
  part; decision Q1 A, round 1, 2026-10-26)

## Goal
A fillet or chamfer of a convex edge that runs to an overhang tip builds
where Open CASCADE builds it. An overhang tip is a cusp whose two edges are
of opposite senses, so its walls lie on either side of the face they share.
The stripe is cut by the next wall exactly as ADR-0042 cuts a cusp whose
walls are on one side: the surfaces are exact and the cut is the one fitted
edge. A concave edge at an overhang tip stays `TangentChain`; Open CASCADE
refuses it at every radius. The lip-over-pillar fixtures pass at Open
CASCADE's measures, a cusp property holds the case at random poses, and
FTC-06's overhang tips are re-counted.
## Non-goals
- A fitted cap surface. The evidence (Design deltas) shows that Open
  CASCADE builds none.
- A cusp whose next edge is blended too (the corner patch,
  `cusp-crescent-both-arcs-fillet`), and the horn torus on an arc: later
  C11 plans.
- FTC-06's cusps that Open CASCADE builds invalid (11 of 21). Where Arris
  builds them checker-green, no oracle holds their measures; a second
  oracle's record of them is a backlog line.
- The lean far past the end's reach (`a_lean_far_past_the_reach…`,
  backlog): a leaning step, not a cusp.

## Design deltas
**The evidence (2026-10-31, `tools/oracle` on
`blend/cusp-overhang-tip-fillet`).** Open CASCADE's "B-spline cap" is one
three-sided rational face, degree 2×4, with corners `P`, `R` and `Q`:
- `P` is where the contact on the shared face `T` crosses `n` (ADR-0042 §2).
- `R` is the contact on the blended edge's wall `W_e`, at `P`'s angle.
- `Q` is where that contact meets the third edge `w`.

The cap is tangent to the stripe along `P–R` and to `W_e` along `R–Q`. It
meets the next wall `W_n` along `Q–P`, with a crease of 81° at `P` falling
to 0° at `Q`.

The cap's surface is the stripe's own surface: within 4.1e-7 of the torus
for a fillet, and within 4.2e-7 of the cone for a chamfer, at every
`r ∈ {0.05, 0.1, 0.3, 0.6, 0.9}`. That is Open CASCADE's fit, not a second
surface. Its `Q–P` edge lies on `W_n` within 1.5e-9. So Open CASCADE's
result is ADR-0042's cut, with the stripe split at the cross-section through
`P`, and counts 19 / 30 / 13 against the operand's 16 / 25 / 11. ADR-0042's
*Alternatives* said this cut "would not be Open CASCADE's answer… the volume
differs"; the measurement says otherwise.

Filleting or chamfering the pillar's concave arc (`e` concave): Open CASCADE
refuses at every radius.

**ADR-0079** (step 1) amends ADR-0042 §6 and corrects its *Alternatives*.
- §1's site no longer requires one sense when `e` is convex and `n`
  concave. §2–§5 apply unchanged: `P`, `Q`, the traced cut, the node at `Q`
  for a fillet, and the topology and provenance.
- `e` concave at an overhang tip stays `TangentChain`, naming `e`, `w` and
  `v` (both kernels refuse).
- **Settled: one face, not Open CASCADE's two.** Arris does not split a face
  along a cross-section across which nothing changes. The fixtures say
  `analytic.counts_differ` (one vertex, two edges and one face fewer: 18 /
  28 / 12), the convention case of ADR-0015. Matching the split would
  leave a seam between two patches of one torus that no other blend has.
  **Settled (2026-10-31):** correct the initial 29-edge typo to 28 (human
  approved): remove the cross-section and join the wall contact's two
  pieces, losing two edges, one vertex and one face. Euler stays
  18 − 28 + 12 = 2; 29 edges would give 1.
- Code: `corner_of`'s `one_sense` gate (`crates/arris-ops/src/blend/ends.rs`)
  and its doc comment. `cusp_trims` reads the two cusp edges' senses:
  the cut is inside the next wall for one sense, outside its old face
  domain at an overhang tip, extending the face previously buried by the
  lip. The pillar fixture turns its seam onto the exposed half so its
  far end is an ordinary corner, isolating the concave cusp refusal.
  `Reason` gains no variant; no public signature or binding changes.
- Oracle: an explicit `expect_error` recipe records no solid when its
  final fillet or chamfer reports not done; operand errors and script
  checkpoints retain their refusals. This lets the pillar refusal have
  a replayable oracle record.

**Tolerances.** Each variant is kept where Open CASCADE's fit error, times
the cap's area, falls within the default `volume_rel` and `area_rel`
(1e-9): `r ≤ 0.3`. Wider radii are held by the property against the
section's integral, not against Open CASCADE.

**The property's change.** For an overhang scene, the corner section is
integrated over its band, taking at each angle the part of the section that
lies outside `W_n`'s disc. This is `Cusped::change`'s method, with the
pillar's disc in place of the crescent's gap.

## Steps
Grades are tiers (`.agents/tiers.md`): **[1]** routine; **[2]** careful, a
geometric case within a given design; **[3]** unproven.

- [x] Step 1 **[2]** — Overhang tip built (fillet). Lift the gate for a convex `e` and
  write ADR-0079 (with its README line). `blend/cusp-overhang-tip-fillet`
  passes with `counts_differ` and a blessed dump. The pillar's arc becomes
  `blend/cusp-overhang-tip-pillar-fillet`, `expect_error: tangent-chain`.
  The census test `an_overhang_tip_is_a_cusp_with_walls_on_either_side`
  moves to that fixture. Look at the result (`inspect`) before the dump.
- [ ] Step 2 **[2]** — Variants and the chamfer. Add fillet variants at
  `r` 0.05 and 0.3, and a mirrored pose (Open CASCADE's invalid caps on
  FTC-06 differ by hand). Add `blend/cusp-overhang-tip-chamfer`, whose cut is
  transverse at `Q`, with variants at 0.05 and 0.3. Each is held at Open
  CASCADE's measures and `counts_differ`.
- [ ] Step 3 **[2]** — FTC-06 re-counted. `real_parts --tangent-chain` on
  FTC-06, each overhang-tip end tested against `occt_fillet_edges.py`.
  Wherever Open CASCADE builds, Arris builds, checker-green. A miss is fixed
  in this step if it is the design's; otherwise it becomes an ignored
  `regression/` fixture and a backlog line. The tallies go in the commit
  body.
- [ ] Step 4 **[2]** — The property. Add `Cusped::Overhang { … }` to
  `blend_prop.rs`, with a lip over a pillar, either hand, and fillet and
  chamfer at random poses. Its `change` is the clipped section's integral
  (Design deltas). It runs the existing body: checker, provenance, volume,
  move/blend commutation, STEP round trip, determinism. Name the slowest
  shard's time at 16 cases in the body.
## Acceptance
- `blend/cusp-overhang-tip-fillet` and `-chamfer`, every variant, pass the
  corpus runner at Open CASCADE's volume, area, centroid and inertia
  (default tolerances), with `counts_differ` 18 / 28 / 12.
- `blend/cusp-overhang-tip-pillar-fillet` is refused as `TangentChain`.
- `cusps_{fillet,chamfer}_as_their_sections` pass over the overhang scene at
  256 cases in the full gate.
- On FTC-06, no overhang-tip end that Open CASCADE builds is refused by
  Arris.
- The real parts' `fillet` row stays 0 of 38, and the script corpus's
  `fillet` refusals stay at 1.
- `tools/full-gate.sh` is green, with no panic.

## Docs to update on completion
- `docs/ARCHITECTURE.md` §blend ends: the cusp paragraph covers an
  overhang tip with a convex `e`; the `TangentChain` list keeps only the
  concave one.
- `docs/ROADMAP.md` §C11 *In*: remove the overhang tip from the
  `BlendTooLarge` line (it was `TangentChain`) and mark it done with its
  date and ADR-0079.
- `docs/BACKLOG.md`: remove the overhang-tip line. In the C6 blend-pairs
  line, FTC-06's `TangentChain` is no longer the overhang tip. Add the
  second-oracle line for FTC-06's invalid cusps.
- `GLOSSARY.md`: **Overhang tip**, a cusp whose two edges are of opposite
  senses, its walls on either side of the face they share.
- `CHANGELOG.md` `Unreleased`: the overhang tip builds for a convex edge;
  the concave one is still refused.
- `AGENTS.md` current state: M5 done; next is the cycle's next plan.
## Open questions
None left for the human. Settled by the agent: the face count (one face,
`counts_differ`, above) and the variants' radii (Tolerances). A variant
that misses Open CASCADE beyond its fit error is a design question for
step 2. Its tolerance is never widened.
