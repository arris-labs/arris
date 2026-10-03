# Plan: split-rim-vertex

- Started: 2026-10-03
- Milestone: C6 (the blend network, docs/ROADMAP.md), sixth plan
- Idea: docs/ideas/cylinder-plane-two-edge-corner.md (absorbed)
- Idea (verbatim from the human): "/plan option A"

## Goal
A fillet or chamfer of an arc between a plane and a cylinder (or the pairs
ADR-0036 already builds along a circle) runs on through a vertex of exactly
two edges that continue one another on one circle and share both faces, the
second vertex of a split rim, where the seam is not. The two stripes meet on
the ball's cross-section at the vertex; nothing is cut, since there is no
third edge. A rim split into two, three or more arcs, with the seam at one
vertex or none, blends as the closed chain it is, one blend face per edge,
at Open CASCADE's counts. The census's `corner edges that share no face
across, 2 edges` is re-counted, and what is left of it is named by cause. The
fetched tier's `fillet` column (4 of 27) and the committed tier's (5 of 11)
are measured again; `ctc_04` leaves the column only if nothing else refuses
behind it, which step 1 finds before anything is built.

## Non-goals
- Merging the halves into one edge (healing; ADR-0039 rejected it).
- A two-edge vertex that is not a continuation on one circle between the
  same two faces. Step 1 names any such cause found; each is a backlog line.
- The fan (CTC-01), tangent continuation, crossing cylinders, the horn
  torus, the miters in the backlog: each its own line.
- Variable radius, blends over blends, other chamfer modes, NURBS faces,
  healing.

## Design deltas
- **ADR-0041 (step 2), new.** Amends ADR-0035 §1, §3, §5, §6: a vertex of
  exactly two edges `e`, `e′` that share both faces and whose directions run
  on (the same test as a tangent vertex's) is a continuation. The balls are
  one; the *junction arc* is the cross-section at `v` with both contacts of
  each stripe ending on it, so the arc has two ends, `q` on one face's contact
  and `p` on the other's. No edge is cut. Provenance: the junction arc and its
  two new vertices `Generated` from both edges, `v` `Deleted`; each face's
  loop takes one more vertex and edge pair. ADR-0039's rejection of merging
  stands and is cited.
- `arris-ops` `blend.rs`: `corner_of` and `tangent_vertex` take the vertex of
  two edges; the junction (`Miter`) gains the case with no third edge
  (`p`-side cut absent). Expect no new module. No public type or signature
  change; `Reason` gains no variant, so `crates/arris-py` does not change.
- `arris-check`: no change expected; step 4 shows it at `Full`.
- `arris-debug` `census.rs`: `vertex_blend_cause` names the two-edge site's
  causes (a continuation on one circle between one pair of faces, or
  another), by what the vertex's second edge is (step 1).
- Docs: `docs/ARCHITECTURE.md` §blend (the tangent vertex and the
  `VertexBlend` paragraph), ROADMAP §C6.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The two-edge site counted by cause. Extend
  `vertex_blend_cause` so the site splits into: a continuation (both edges
  on one circle, the same two faces, directions running on), and every other
  two-edge vertex, named by what the second edge is. Run
  `real_parts --vertex-blend` on both tiers; print the counts with Open
  CASCADE's verdict (`target/real-parts/vertex-blend.md`). A unit test on a
  small recipe: a cylinder boss whose rim is split into two arcs with no seam
  at one of the vertices. **Gate:** if the continuation is under half the
  site's edges, or frees no sampled edge of either tier, stop and ask the
  human; otherwise record the counts in ROADMAP §C6.
  *(Done: the site is a continuation throughout, 67 of 79 fetched edges and
  all 20 committed, and it holds `ctc_04`'s sampled edge; the gate does not
  fire. No other two-edge cause found, so the first open question is
  answered. `a_split_rims_second_vertex_is_a_continuation` reads `ftc-06`.)*
- [x] Step 2 **[1]** — ADR-0041 and its README row, with ADR-0035's and
  ADR-0039's pointers. Write the fixture's inputs: `regression/split-rim-
  two-edge-vertex-fillet`, a boss or hole whose rim is two half circles with
  the cylinder's seam elsewhere, with Open CASCADE's oracle, `#[ignore]`d.
  *(Done: ADR-0041. The fixture reads FTC-06 in place and fillets one half of a hole's
  rim at r 2, `#[ignore]`d at `VertexBlend` naming the vertex; Open CASCADE's counts: faces
  144 → 146, edges 373 → 377, vertices 250 → 250. The recipe could not build a split rim with no
  seam at a vertex: extruded half circles give two cylinder faces, so the part is read, not
  shrunk (ADR-0026 §4). The unit test of step 1 reads the same part.)*
- [x] Step 3 **[3]** — The walk and the junction: `tangent_vertex` takes the
  two-edge continuation, the junction runs between the two points on the
  shared contacts with no cut, provenance as ADR-0041. The fixture moves to
  `blend/` with its blessed dump, checker green at `Full`; fillet and its
  chamfer twin, convex and concave, at Open CASCADE's counts and probes. A
  closed rim of two arcs with no seam at either vertex, and one of three arcs,
  are fixtures of their own.
  *(Done, with two findings. The seam's vertex was not walked either: its
  third edge is a seam the cylinder uses twice, which ADR-0035 §1 does not
  take, so `tangent_vertex` and the junction take it too, the seam cut at
  `p` (ADR-0041 corrected). A rim with the seam at no vertex cannot occur:
  a seam ends on the rim, and the reader rebuilds a missing one through a
  rim vertex. Fixtures: `blend/split-rim-two-edge-vertex-{fillet,chamfer}`
  (FTC-06, convex), `blend/split-boss-base-{fillet,chamfer}` (concave) and
  `blend/three-arc-hole-rim-fillet`, the last three on plates Open CASCADE
  wrote with its closed edges split, the three-arc rim split once more in
  the file. Open CASCADE matches Arris's counts there; on FTC-06 it builds
  two faces of a whole turn each, genus 11, so those two are held to
  Arris's counts and the closed forms, and its STEP is refused as a face
  wrapping a period. `cancel_counts.txt` blessed for the five here, to keep
  `main` green.)*
- [x] Step 4 **[2]** — Cylinder pair coverage: the same vertex with the
  pairs ADR-0036 builds along a circle (a cone, a sphere or a torus against a
  coaxial plane, cylinder or cone) — each a fixture or its absence named in the
  commit. Where a pair is refused at the junction, `Unsupported` names it, no
  new `Reason`.
  *(Done, with one finding. A rim between two turning faces has a seam of
  each at the vertex, four edges, which neither ADR-0035 §1 nor step 3 took:
  `tangent_vertex` and the junction take it, each seam cut at its own face's
  contact (ADR-0041 corrected). Fixtures `blend/split-{cone-hole-rim-fillet,
  cone-hole-rim-chamfer,dimple-rim-fillet,torus-ring-fillet,pin-shoulder-
  fillet,torus-collar-fillet}`: plane against cone (convex), sphere (convex)
  and torus (concave), cylinder against cone and against torus, all on
  operands Open CASCADE wrote with closed edges split. On the sphere, torus
  and two-seam pairs Open CASCADE builds the ring as one face, so those are
  held to Arris's counts (`counts_differ`, `step_differs`) with measures and
  probes at the oracle's, tolerances widened to its blend's approximation
  (1e-7 to 5e-7). Absent, named: the concave cone (Open CASCADE refuses a
  split concave cone base, `not done`, though it builds the whole circle),
  the pin's chamfer (Open CASCADE's volume of the split ring is 2.2e-4 below
  its own whole-circle chamfer, which Arris matches to 1.3e-7; a closed form
  with all four measures is a fixture of its own, a backlog line), a sphere
  against a cylinder or a cone and a cone against a torus (Open CASCADE
  writes no valid operand: the pole of the sphere breaks its closed-edge
  split, and no revolve of the other pair was drawn). No pair was refused at
  the junction, so no `Unsupported`.)*
- [ ] Step 5 **[2]** — A property over random poses (`blend_prop`): a boss
  or a hole split into two to four arcs at random angles, with the seam at a
  vertex or elsewhere, fillet and chamfer at random size, convex and concave.
  Checks the checker, volume additivity to the closed forms, STEP round-trip,
  deterministic ids. Seeded, sharded through `prop_shards!`; a pose the end
  leaves is rejected by the refusal's name.
- [ ] Step 6 **[1]** — Measure the tiers again (`tools/real-parts.sh`,
  `--census-committed`). Move `fixtures:` expectations whose refusals changed
  (the commit body says why) and bless `cancel_counts.txt` for step 4's
  fixtures (step 3's are blessed). Update ROADMAP §C6's status paragraph with the new `fillet`
  count beside C4's 17 of 38; run the docs-refs tests.

## Acceptance
- Step 1's counts are reproduced, and the §C6 summary line is the number
  `tools/real-parts.sh` prints after step 6.
- Every fixture of steps 3 and 4 passes under `blend/`, checker green at
  `Full`, at Open CASCADE's oracle or the closed form under
  `analytic.measure_differs` with ADR-0015's evidence.
- Step 5's property is green at 256 cases (retirement), 1000 (CI).
- Every part that leaves the column agrees with Open CASCADE at the battery's
  radius, or is refused as another line's. The full profile is green.

## Docs to update on completion
- `docs/ROADMAP.md` §C6: the status paragraph, the new counts, what the
  cycle has left; judge whether it can close, and leave that to `/close-cycle`.
- `docs/BACKLOG.md`: the other two-edge causes step 1 finds; the fan stays.
- `docs/adr/`: ADR-0041, its README row and pointers in 0035 and 0039.
- `docs/ARCHITECTURE.md` §blend: the tangent vertex of two edges, its
  junction, and the `VertexBlend` paragraph.
- `tests/fixtures/README.md`: the `VertexBlend` line if its cases change.
- `CHANGELOG.md` `Unreleased`: a fillet or chamfer of a rim split into
  arcs now builds at every vertex of the split, and what still refuses (no ADR
  numbers or fixture names).
- `AGENTS.md` current state: C6's sixth plan landed, and the column's count.

## Open questions
- Answered at step 1: no cause but a continuation. Was ⚠ OPEN (agent, step 1): whether the two-edge site holds causes other than
  a continuation (a face across, a seam-less tangent junction); the gate
  above fires if the continuation is not the majority.
- Answered at step 3: yes, a junction and two faces, at Open CASCADE's
  counts on the operands it builds whole (ADR-0041 §3). Was ⚠ OPEN (agent,
  step 3): whether a vertex of two edges on one circle needs a junction at
  all where both blends are the same torus (the halves of one ring).
