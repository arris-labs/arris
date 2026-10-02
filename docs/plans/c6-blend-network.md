# Plan: c6-blend-network

- Started: 2026-10-02
- Milestone: C6, the blend network (docs/ROADMAP.md §C6) — this is its first plan: lines 1 and 2 of the section's "in" list, in the part of line 2 the census below ranks first
- Idea (verbatim from the human): "c6-blend-network" (the cycle's name, as given to `/plan`)

## Goal

A fillet or chamfer runs along an outline, not only along one straight edge
or one closed circle: a stripe follows a chain of edges that meet at
tangent vertices (a slot's rim of two lines and two arcs, a hole's rim split
into two half circles, a D-shaped notch), and an open circular arc between
a plane and a cylinder blends to a torus (a fillet) or a cone (a chamfer)
with its ends trimmed in closed form. Every result is checker-green at
`Full` with provenance rooted at the edges, and matches Open CASCADE's
fillet of the same body on the committed fixtures and the real parts the
census names. What C6's other lines carry (the pairs with a cone, a sphere
or a torus, the corners refused as `VertexBlend`, the other chamfer modes,
variable radius, blends over blends) stays with later C6 plans; the cycle
stays open when this one retires.

**The census this plan is chosen on** (2026-10-02; every edge of the 38
fetched parts' solids with two distinct faces and a curve, blended alone at
a tenth of its length, 14066 edges; a throwaway probe, not committed):

| What the edge is | Edges | Refusal today |
|---|---:|---|
| open circular arc, plane × cylinder | 2748 | `Unsupported(circle curve × plane)` |
| line, plane × cylinder, at a tangent junction | 2297 | `TangentChain` |
| line, plane × plane, at a tangent junction | 1242 | `TangentChain` |
| line, plane × plane, blend runs out of its face | 1198 | `BlendTooLarge` (the probe's radius, not a finding) |
| line, cylinder × cylinder, at a tangent junction | 852 | `TangentChain` |
| arc, cylinder × torus / × sphere / plane × torus (a part's own fillets) | 1301 | `TangentChain` |
| arc, cone × cylinder / cone × plane | 700 | `Unsupported` (the pair, later C6) |
| closed circle, plane × cylinder | 497 | 293 build, 204 `BlendTooLarge` |

Two readings. The open arc is the largest `Unsupported` there is, and in
the NIST files it is a hole's rim split at its seam into two half circles.
And `TangentChain` is two things the refusal does not separate: an edge
that is itself a tangent dihedral (a blend's own contact edge, which no
rolling ball can blend), and an edge whose end sits at a tangent junction,
which a chain blends. Step 1 separated them on the fetched tier (a second probe, not committed,
each edge blended alone at a tenth of its length, entities of the refusal
read): of 2854 `TangentChain` edges **2479 are tangent dihedrals themselves**
(1766 lines, 614 circles, 99 NURBS) and **375 sit at a junction** (all
lines), 13% of the two. That probe saw 6087 edges where the census says
14066; the difference is unexplained and the census table's counts are
therefore orders of magnitude, re-measured at step 9. The ranking holds: the
open arc is `Unsupported` on every edge it touches, and the junction rows are
the chain's.

## Non-goals

- Pairs with a cone, a sphere, a torus or a NURBS face (the circle arcs on
  cones and a part's own blends, 700 + 1301 edges above): C6's later plans,
  one pair family each. A chain through them is a refusal naming the pair.
- Corners: a vertex of other than three edges, the miters of unequal
  dihedrals and the tilted-great-circle sphere corner (`VertexBlend`).
- The remaining chamfer modes, variable radius, and a blend over a blend.
- A closed form for a torus meeting an oblique plane (a quartic): an arc's
  end on such a plane is `BlendTooLarge` or `Unsupported`, never fitted past
  the arc's own tolerance (ADR-0007: nothing is fitted but a pcurve).
- The first consumer's side-by-side run: ranking input only (ADR-0020).

## Design deltas

- **ADR-0035 (step 2): a stripe follows a chain.** A spine is an ordered
  list of edges joined at *tangent vertices* (the two faces across the
  vertex's other edge are tangent), open or closed. Each edge keeps its own
  closed-form stripe (ADR-0007); at a tangent vertex the two stripes end on
  one shared cross-section arc of the ball (the circle of the ball in the
  plane through its centre normal to the spine), whose two ends are the two
  contact points, so neither stripe is trimmed by a face across. The ADR
  decides the arc's pcurves, its tolerance, which entity it is `Generated`
  from in provenance, and amends ADR-0007's "an edge that meets a blend
  face is `TangentChain`" to what is left of it.
- `docs/ARCHITECTURE.md` §Operations (blends): the chain, the open-arc
  stripe and the junction arc, present tense; the paragraph that says an
  open circle is `Unsupported` and a tangent end is `TangentChain` shrinks to
  what still is.
- `docs/DATA-MODEL.md` §Provenance: the junction arc's origin, if the ADR
  gives it one (a vertex of the spine) — checked against the audit.
- **Public types.** `Reason::TangentChain`'s meaning narrows (the edge's own
  tangent dihedral, and a chain whose junction the closed forms do not
  cover). If a chain that cannot be built needs its own reason, `Reason`
  gains a variant: an exhaustive-enum change, named in that step's commit
  body and under `CHANGELOG.md`'s `### Breaking`. No other public type or
  signature changes: `fillet` and `chamfer` take the same edges.
- `arris-debug`: the battery's `fillet` stage no longer samples an edge whose
  faces are tangent along it (step 1; `battery::is_tangent_dihedral`, the
  test `blend.rs` refuses with, at the curve's midpoint).
  `prop::recipe` gains an outline-fillet recipe (step 8).
- Crate boundaries and layers: none. All of it is `arris-ops`'s `blend.rs`
  (2976 lines today); a chain module split out of it is a refactor step 2
  names if the ADR wants one.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The anatomy, as fixtures, before any code. Shrink
  the census's two top rows to `tests/fixtures/regression/` entries with
  Open CASCADE's oracle values, `#[ignore]`d with the desired assertion: a
  plate with a D-shaped notch (an open arc between the top plane and the
  notch's cylinder, its ends at the flat's corners), a disc with its rim
  split in two half circles (the real parts' shape), a stadium's top
  outline (line, arc, line, arc: a closed chain), and the already
  committed `blend/tangent-chain-cap-edge` re-read as a chain through an
  arc. Separate, on the real parts, the `TangentChain` edges that are
  themselves tangent from those at a junction, and ask the oracle what Open
  CASCADE does with the first (open question 1). Test: the fixtures run
  ignored and fail for the reason the census names; `cargo nextest run -p
  arris --run-ignored only -E 'test(/regression_/)'` shows each with its
  refusal.
- [x] Step 2 **[3]** — ADR-0035, the chain model, written against step 1's
  fixtures: the spine, tangent vertices, the shared cross-section arc at a
  junction, open and closed chains, the open-arc stripe's ends (a plane
  through the axis gives a meridian circle of the torus, a plane square to
  it a parallel circle; anything else is refused by name), the provenance
  of each new entity, and what `TangentChain` still means. Add its line to
  `docs/adr/README.md` and amend ADR-0007's refusal paragraph by reference.
  Test: the ADR's worked example (the stadium outline's entity counts,
  Euler line and provenance records) is the expected value of step 4's
  fixture, computed here by hand and by the oracle's counts.
- [x] Step 3 **[3]** — The open-arc stripe. `ring` generalises from a closed
  circle to an open arc between a plane and a cylinder: a torus section
  (fillet) or cone section (chamfer) over the arc's range, its two contacts
  arcs of the same circles, the cylinder's seam kept or shortened as the
  closed case does, each end trimmed by the face across when that face is a
  plane through the axis or square to it (and the corner is the usual three
  faces). Fixture: the D-notch moves from `regression/` to `blend/` with its
  oracle, checker green at `Full`, volume to the fixture's tolerance and to
  the closed form (the removed corner's section `(1 − π/4) r²` swept by
  Pappus about the axis at `R ± r (10 − 3π) / (12 − 3π)`, half a turn, no
  end effect past the trimming plane — the fixture's `analytic.volume`,
  which the corpus lint already holds to the oracle).
- [ ] Step 4 **[3]** — The tangent junction. Two stripes meeting at a
  tangent vertex end on one shared cross-section arc: no trim by a face
  across, the contact points meet, the blend faces meet along the arc with
  a tangent dihedral and the checker's rows agree. The first chain is an
  open one: a straight wall's cylinder stripe into an arc's torus stripe,
  and the committed `tangent-chain-cap-edge` (a cap edge continuing through
  the first fillet's arc) moves to `blend/` building. Test: those two
  fixtures against the oracle, and the junction arc's pcurves held to the
  checker's E4 at the arc's tolerance.
- [ ] Step 5 **[2]** — Closed chains. The stadium's whole outline, a hole's
  rim as two half circles, a rounded rectangle: a chain that returns to
  its first vertex has no ends and no trim, as a closed circle has none.
  Orientation of the junction arcs around the loop, and the seam of a
  torus face that spans several edges (one face or one per edge: the ADR's
  answer, tested). Test: the stadium and split-rim fixtures against the
  oracle; the fixtures' `Full` check with nothing unchecked.
- [ ] Step 6 **[2]** — The chain's refusals, each named. A chain through a
  pair with no stripe (cone, sphere, torus, NURBS) is `Unsupported` naming
  the pair and the edge in the chain; a junction whose far contact leaves
  its face is `BlendTooLarge`; an end on a plane neither through the axis
  nor square to it is `Unsupported`; a tangent dihedral edge is
  `TangentChain` with the edge. Each is a committed fixture with
  `expect_error` and the oracle's own result recorded beside it (the
  `analytic.occt_*` fields) where Open CASCADE builds one.
- [ ] Step 7 **[2]** — Chain chamfers. (Step 3 already builds the open
  arc's cone section: the D-notch chamfered at 0.25 checked by hand at
  `Full` with nothing unchecked, volume to the closed form within 2e-15;
  this step commits it as a fixture.) The same chains with a flat cut: the
  open arc's cone section, the junction a straight chord between the two
  contact points, the closed forms exact on every plane they lie on.
  Fixtures: the D-notch, the stadium and the split rim chamfered, against
  the oracle.
- [ ] Step 8 **[2]** — The property. `prop::recipe` gains an outline fillet:
  a random convex outline (a stadium, a rounded rectangle, a D) extruded in
  a random pose, filleted along the top outline in one call at a random
  radius below the arc's, then chamfered. Checks: the checker at `Full`,
  volume additivity with the tool-free closed form: the removed corner's
  section `(1 − π/4) r²` swept along the outline by Pappus (`L` times it on
  a line; on an arc the centroid's radius `R ∓ r (10 − 3π) / (12 − 3π)`
  times the angle, as step 1's fixtures hold), and `d² / 2` swept the same
  way for a chamfer, its centroid `d / 3` from each face, the call
  deterministic and its record complete, the differential against Open
  CASCADE counting it among the agreeing recipes.
- [ ] Step 9 **[1]** — Measure and close the plan's loop. Rerun
  `tools/real-parts.sh`; the battery's `fillet` column, the committed tier's
  and `docs/ROADMAP.md` §C6 and §C4's table get the new numbers beside
  C4's 17 of 38; each part that left the column agrees with Open CASCADE or
  is refused as another C6 line, named. The residue (the pairs with a cone,
  a sphere or a torus, the corners) becomes the next plan's sizing, in the
  roadmap and `docs/BACKLOG.md`.

Each step is one commit-sized unit with its own test, fixture or oracle
comparison. Steps 3 and 4 may split if the end trim and the junction turn
out to need more than a commit each; say so in that commit.

## Acceptance

- `cargo nextest run -p arris -E 'test(/^blend_/)'`: every blend fixture
  green, the new ones against Open CASCADE's volume, area, centroid and
  counts to their stated tolerances, the checker at `Full` with nothing
  unchecked, and no fixture left under `regression/` for a line of this plan
  (the ones for later C6 lines keep their `#[ignore]` and their reason).
- The outline-fillet property at 256 cases (the retirement run) and at 1000
  in CI, seeded; the differential counts those recipes among the agreeing.
- `tools/real-parts.sh`: of the 17 parts C4 blocked at `fillet`, the nine
  held by `Unsupported(circle curve × plane)` and the five by
  `TangentChain` each either agree with Open CASCADE's fillet within the
  part fixture's tolerance or are refused as a named later C6 line; the
  committed tier's column printed beside it.
- `cargo test --workspace --doc`, `cargo clippy --workspace -D warnings`,
  `tools/check-layers.sh`, the wasm build and the semver gate pass.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Operations (blends) — the chain, the open-arc
  stripe and the junction arc; the refusals that remain; §Errors' table rows
  for `Unsupported` and `Degenerate` where the blend text narrows.
- `docs/DATA-MODEL.md` §Provenance — the junction arc's record, if any.
- `docs/ROADMAP.md` §C6 — the status line becomes "first plan landed" with
  the new histogram numbers; lines 1 and the arc half of line 2 marked done
  within the section; the remaining lines stay.
- `docs/adr/0035-….md` and `docs/adr/README.md` — written at step 2.
- `docs/BACKLOG.md` — drop the lines this plan covers (the open arc and
  the tangent chain in the `Blends on face pairs outside ADR-0007's table`
  line, the two miter lines if a chain absorbs them); add the residue.
- `CHANGELOG.md` `## Unreleased` — a fillet or chamfer along an outline of
  lines and arcs, including a hole's rim split in two arcs, and which
  refusals remain; a `### Breaking` bullet only if `Reason` gained a variant.
- `AGENTS.md` current state — C6's first plan landed.

## Open questions

- **Answered early (2026-10-02, agent; step 1 confirms on real parts): Open
  CASCADE refuses an edge whose dihedral is tangent.** Run through the
  oracle's `cadquery-ocp`: a box edge filleted at r=1, then each plane ×
  cylinder edge of the result filleted and chamfered at 0.3. The two
  tangent contact lines raise `Standard_Failure: There are no suitable
  edges for chamfer or fillet` (`ChFi3d_Builder.cxx`, the throw;
  `ChFi3d::IsTangentFaces` is the test); the two end arcs build and
  check valid. **Confirmed on the fetched tier in step 1: all 2479
  tangent-dihedral edges (lines, circles, NURBS) are refused with that
  message; the battery's stage skips them, and the `fillet` column moves
  without any kernel change** — on this tier's 27 parts the column's
  blocked parts went from 11 to 12, `TangentChain` from 4 refusals to 1,
  `Unsupported(circle curve × plane)` from 6 to 8, `BlendTooLarge` from 0 to
  2, and 4 stages where both refused became 3 (step 9's baseline is the
  new numbers, not C4's). The 375 junction edges, by contrast, are the
  chain's: Open CASCADE built 299 and returned not-done on 76 (step 2's ADR
  reads those 76 before it names what a chain refuses). The question as
  first written: the census's `TangentChain` rows include a blend's own
  contact edges, which the battery's `fillet` sample picks like any edge.
  If Open CASCADE also refuses them, that sample is wrong, not Arris: the
  stage skips an edge no ball can blend and the histogram moves without any
  kernel change, which must be said in step 1's commit and not hidden in
  step 9's numbers. If it blends them, there is a requirement here the plan
  does not yet have, and step 2's ADR takes it or names it for the next plan.
- **Decided (2026-10-02, human): C6's scope is confirmed.** This plan
  takes the census's two largest rows and nothing else; steps 3 to 7 stand.
- **Decided by ADR-0035 (step 2, agent): the junction arc comes from both
  edges.** It and its two vertices are `Generated` from the two edges it
  joins, as a miter's are; the vertex it replaces is `Deleted`. The audit
  and DATA-MODEL §Provenance need no new kind of record (step 4 checks).
- **Decided by ADR-0035 (step 2, agent): the selection follows the chain.**
  As Open CASCADE's spine does, a named edge reaches every edge beyond a
  tangent vertex, each with its own blend face; the cap-edge fixture names
  one edge and its oracle body is the chain of three. Step 4 builds the
  walk with the junction.
- **Moved to step 6 (step 2, agent): the 76 junction edges Open CASCADE
  returned not-done on.** Step 1's probe was not committed, so the ADR
  names the chain's refusals from the closed forms; step 6 re-runs the
  probe over the junction edges and fixes each class it finds as a fixture
  with the oracle's own result beside it.
- **Answered by step 1's counts (agent): one blend face per edge.** Open
  Cascade's stadium has 10 faces (top, bottom, four sides, four blends) and
  the split-rim disc 6 (top, bottom, two sides, two tori): a blend face
  never spans an edge boundary, even over a tangent chain of the same
  surface. Step 5 builds one per edge and the ADR records it; the
  fixtures' counts are `expected.json`'s.
