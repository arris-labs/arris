# Plan: cusp-run-out

- Started: 2026-10-03
- Milestone: C6 (the blend network, docs/ROADMAP.md), seventh plan
- Idea: docs/ideas/tangent-continuation-and-fan.md (option B accepted, then
  absorbed; the fan kept as the parked idea `docs/ideas/blend-fan.md`)
- Idea (verbatim from the human): "I accept B, /plan it"; after step 1
  found the premise wrong: "Do A" (re-plan around the cusp)
- Renamed from `s-bend-walk` at step 1 (see its finding)
- Step 2's gate (verbatim from the human): "(a) Crescent family
  (Recommended)", meaning land the cut with exact surfaces, while FTC-06
  waits for the NURBS cycle

## Finding of step 1 (why this is not an S-bend)
All 21 of FTC-06's `TangentChain` chain ends are **cusps**, none an
inflection (`real_parts --census` names them by site). At each, the blended
edge and the next one reach a vertex of three edges from the *same side*:
two arcs on the floor, tangent at the vertex, one of the big circle and one
of a small circle kissing it, so the outline doubles back and the third
edge (the tangent line between the two walls) is the cusp's spine. The
two edges' dihedrals with the floor read opposite convexities, which is the
test `tangent_vertex` refuses on, correctly: the chain does not run on, it
has a sliver to cross. Open CASCADE alone, at the battery's radius, of
the 13 cusp edges asked: 6 build, 7 are invalid; the battery's sampled edge
(−86.40, 52.19, −234.95) builds. The shrunk case is the crescent prism
`regression/cusp-crescent-fillet`: Open CASCADE builds it at 6 faces, 12
edges and a volume 4.3% of the fillet's corner short of the full-length
removal, i.e. the blend is cut where the gap between the walls is
narrower than the ball and the faces keep their count.

## Finding of step 2 (the gate tripped: two cusps, one of them capped)
There are two cusps, and the crescent is not FTC-06's.
- **The crescent's cusp** has material in the sliver between the walls,
  and both of its floor/top edges are convex. Open CASCADE cuts the stripe
  with the next wall and that's all. A fillet or chamfer of either arc,
  top or floor, at 0.02 to 0.8, always gives 6 faces, 12 edges and 8
  vertices, all valid. The cut is a B-spline from P to Q. P is where the
  contact on the shared face crosses the next edge, a circle against a
  circle in closed form. Q is where the wall contact meets the spine.
  Between them the cut is the blend's torus (or cone) against the next
  wall's cylinder, on parallel axes. It is a graph over the next wall's
  base circle (the height read off the tube at the point's distance from
  the blend's axis), so it is exact pointwise and is traced and fitted as
  in ADR-0037. At Q the torus is tangent to the next wall, through the
  wall it is tangent to: the section has a node there. The branch *ends*
  at the singular point; it does not pass one (ADR-0037 §6 refuses the
  latter). The cone's cut at Q is transverse. Every removed volume
  matches a closed-form integral (the corner section over the face's
  footprint inside the gap) within 5e-7: for the fillet of the big arc,
  r 0.02 → 1.9%, 0.1 → 4.3%, 0.8 → 11.8% short of the full-length
  removal.
- **FTC-06's cusp is an overhang tip.** Its floor (z −234.95, facing
  down) is the underside of a horn. The horn is bounded by a pillar's wall
  (R 22.225, a concave edge whose fillet *adds* material) and a hole's wall
  (R 3.175, a convex edge). The spine runs up into the hole. Each of the 10
  cusp edges Open CASCADE builds at 0.254 closes its stripe with **B-spline
  surface caps**: on the big arcs a torus and two caps, on the small arcs
  a torus, a cylinder and one cap. The cap at the sampled edge rises up
  the spine 1.19 (4.7 r) into the hole. It is a filling, not a cut. The
  other 11 (not 7: step 1 asked a stride, this step asked all 21) are
  invalid at **every** radius from 0.1 to 1.5. The invalid face is always
  the pillar's cylinder, the volume collapses by about 13 139 mm³, and a
  mirror pair splits (x = +131.7 and +143.1 build, −131.7 and −143.1
  are invalid). That is Open CASCADE failing to split the face. It is not
  a radius against a gap. A recipe twin of the overhang (a pillar fused
  with a lip, a hole kissing it) repeats both: B-spline caps, and the
  concave arc invalid with its volume collapsed.
- Where the cut stops: the crescent family has a construction, exact
  surfaces and a traced cut, at Open CASCADE's counts. FTC-06's family has
  a cut that would be the same curve: the next wall's cylinder carried
  down through the stripe's added material. But Open CASCADE answers it
  with fitted cap surfaces, which ADR-0007 and ADR-0037 §1 rule out until
  the NURBS cycle. Arris's cut would disagree with the battery's counts.
  So the plan's goal (FTC-06 leaves the `fillet` column) is out of
  reach, and the human decides how the plan continues (step 2's gate).
  The human chose (a). The goal below is re-scoped, and ADR-0042 records
  the rule.

## Finding of step 3 (the cut is the face end's, and a line edge too)
- The cusp needs no new topology. `corner_of` already finds `n` (on `T`)
  and `w` (on `W_e`) as the corner edges and `W_n` as the face across, so
  the end is an ordinary face end: `corner_of` now returns the spine's
  index where the site holds (`cusp_at`, ADR-0042 §1) instead of refusing,
  and the end takes `P` and `Q` in closed form and cuts both corner edges
  (`cusp_trims`, side inside, nothing lengthened).
- At `Q` the torus's trace against `W_n` is two branches, each a loop
  pinched at the node: it starts and ends at `Q`. The locator found `Q` at
  the branch's start, while the stripe's stretch reaches it at the other
  end. `traced_end` now takes an open branch's end within `tol.linear` of a
  trim point as standing for it, either end. That is ADR-0042 §4, nothing
  more.
- At the cusp the vertex lies on the line through both axes, so
  `Across::meet` cannot read the side from it. The side is read at the
  edge's midpoint (an open arc is under a turn); on a line edge the pierce
  takes the root nearest the midpoint.
- A line edge's cusp builds the same way (ADR-0042 §1 does not restrict
  the site to arcs): `blend/cusp-spandrel-fillet`, the stripe's cylinder
  cut by the arc's, at Open CASCADE's 8/12/6. Added beside the planned
  twins. It is sized 4 so the default mesh bound holds: at 2 its concave
  wall's chord error, an ordinary inscribed one, is 2.0e-3 of a 0.85 volume.
- The census tells the two cusps apart by the spine, not by convexity:
  the walls' outward normals at the tip are opposite where both walls lie
  on one side (a knife-edge sliver, material or void) and equal at an
  overhang tip. FTC-06's sampled edge reads "an end at a cusp, walls on
  either side". A one-side cusp is now refused only where `Q` misses the
  spine or the spine is not a line.
- Outside the step: the render's coarse first pass fails on the pocket's
  top face, before any fillet. A loop with a cusp, meshed at the fewest
  segments per turn, has its two tangent arcs' polygons crossing
  (`regression/crescent-hole-coarse-mesh`, ignored; a backlog line).
- `cancel_counts.txt` gained the five building fixtures here, since the
  hook's cancel test holds every `blend/` fixture to one.

## Finding of step 4 (two failures beside the cut, both fixtures)
- `Profile::edges` refuses a crescent profile unless its small radius is
  half the big one (R 2, ρ 0.9 is `SelfIntersecting`). It validates on
  the fewest-segment polygon, whose chords from the cusp cross: the
  mesher's coarse-chord failure of step 3 in the validator
  (`regression/cusp-profile-off-half`, ignored, on that backlog line).
  So the property builds its crescents as a boolean, the half disc less
  the small cylinder, which gives the same 6/9/5 cusp at every ratio.
- In about one pose in forty of a wide fillet (0.74 of the height, the
  small arc's floor edge), the cut's branch ends at the node 1.07e-7 from
  the closed-form `Q`, a hair past `tol.linear`, and the end is
  `Unsupported`. It builds at rest, and Open CASCADE builds it. Matching a
  trim point at a node is a tolerance decision of ADR-0042 §4, so it is a
  backlog line, not a widening here (`regression/cusp-small-arc-wide-fillet`,
  ignored). The property rejects that refusal alone by name, a torus
  against a cylinder in a fillet of a crescent or a pocket, as
  `end_posed` does for its twin boss.
- The volume closed form needed `u = w²`: the band's length and a fillet
  section's height both have a square root at the blended wall.

## Goal
A fillet or chamfer of an edge that ends at a cusp with both walls on one
side of the shared face is built as Open CASCADE builds it (ADR-0042). The
stripe runs to the cusp and is cut by the next wall, between the point
where its contact leaves the shared face and the point where its other
contact reaches the spine. The result matches Open CASCADE's counts,
measures and probes. The overhang tip, a cusp of opposite senses and
FTC-06's kind, stays `TangentChain`. Open CASCADE caps it with fitted
surfaces, so FTC-06 stays in the `fillet` column and waits for the NURBS
cycle. The census names the two cusps apart, and names a `TangentChain`
end by its site the way it already names `BlendTooLarge` and
`VertexBlend`.

## Non-goals
- The fan (CTC-01) and `blend/five-edge-vertex`: `docs/ideas/blend-fan.md`.
- An edge that is itself a tangent dihedral (ADR-0035 §6). It stays
  refused.
- An inflection (S-bend) chain: no part in either tier has one. If a
  consumer asks, it is its own idea.
- CTC-04's two chain ends at a B-spline-written edge (the NURBS cycle, or
  the backlog's conic-reading line).
- A smooth, seam or collinear extra edge at a vertex of four (backlog).
- The crossing cylinders, a cylinder against a sphere, the horn torus,
  the miters, chamfer modes, variable radius, NURBS faces, healing.

## Design deltas
- **A new rule for the walk's end at a cusp**, ADR-0042 (amending ADR-0035
  §6 and ADR-0037 §6). It applies at a vertex of three edges where the
  blended edge's corner edge is a tangent dihedral, the next edge doubles
  back, is of the same sense and is not blended. There the stripe is cut
  by the next wall: closed-form trim points `P` and `Q`, and the cut is
  the intersector's closed form or ADR-0037's trace and fit, ending at the
  node at `Q`.
- `arris-ops` `blend.rs`: `corner_of` stops refusing that site's corner
  edge as `TangentChain`. No public type or
  signature change expected; `Reason` gains no variant, so
  `crates/arris-py` does not change.
- `arris-debug` `census.rs` (step 1, done): `tangent_chain_cause`, a
  `tangent_chain` field on `SolidCensus`, `ask_the_oracle_tangent` and
  `real_parts --tangent-chain <file>`. In step 3, "an end at a cusp" is
  split into "walls on one side" and "walls on either side".
- `arris-check`: no change expected. Step 3 shows it at `Full`.
- Docs: `docs/ARCHITECTURE.md` §Operations (the tangent vertex paragraph
  and the `TangentChain` one), ROADMAP §C6.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — Count `TangentChain` by site, find why the walk
  stops, and ask the oracle. Done: `census::tangent_chain_cause`,
  `real_parts --census*` prints it, `--tangent-chain` puts it to Open
  CASCADE; a unit test reads FTC-06 and asserts its sampled edge is a cusp;
  `regression/cusp-crescent-fillet` with Open CASCADE's oracle,
  `#[ignore]`d at `TangentChain`. Finding above: the gate (the S-bend under
  half of the refusals) tripped and the human chose to re-plan on the cusp.
- [x] Step 2 **[3]** — The construction, on paper and against the oracle,
  before any code. Dump Open CASCADE's crescent (its faces, edges, the
  fillet face's bounds, volume against the full-length removal) at two or
  three radii, and with the big arc a chamfer; find where the stripe's
  contacts leave the top face and what the curve of the cut is (the blend's
  torus against the small wall's cylinder: a quartic, traced and fitted as
  ADR-0037, or a closed form). Decide which cusps Open CASCADE builds
  invalid and why (the 7 of 13: read them), and whether Arris refuses the
  same by name. Write ADR-0042 with its README row and pointers in ADR-0035
  and ADR-0037, and close the open questions below. **Gate:** if the cut
  has no construction short of a general surface–surface walk, stop and
  ask the human whether FTC-06 waits for the NURBS cycle. Done: the gate
  tripped (finding above), the human chose (a), and ADR-0042 is written.
- [x] Step 3 **[3]** — `corner_of` and the end at a cusp (ADR-0042 §1–§6):
  the site test, `P` and `Q`, the cut, and the topology and provenance of
  §5. The `regression/` fixture moves to `blend/` with its blessed dump,
  checker green at `Full`, at Open CASCADE's counts, measures and probes.
  Add as twins the chamfer, the small arc blended (its cut on the big
  wall), and the pocket (the crescent cut from a block, the floor edge
  concave). Add as `TangentChain` fixtures both arcs blended and the
  overhang tip, with Open CASCADE's oracle and an `analytic` note saying
  why Arris refuses. Split the census's cusp cause in two. Done, with the
  spandrel (a line edge's cusp) beside the twins; finding above.
- [x] Step 4 **[2]** — A property over random poses (`blend_prop`): a
  crescent of random radii whose cusp is one end, or its pocket, or a
  spandrel of random size (the line edge's cusp), its floor
  or top edge filleted and chamfered at random size, small and large
  against the gap, against the checker at `Full`, the closed-form volume
  (ADR-0042, Consequences: the corner section integrated over its band
  inside the gap), STEP
  round-trip, pose independence and deterministic ids. Seeded and sharded
  through `prop_shards!`. A pose Open CASCADE refuses is rejected by the
  refusal's name. Done: `cusps_fillet_as_their_sections` and
  `cusps_chamfer_as_their_sections`, green at 256 and 1000; finding
  below.
- [x] Step 5 **[1]** — Measure the tiers again (`tools/real-parts.sh`,
  `--census-committed`). Move any `fixtures:` expectations whose refusals
  changed (the commit body says why) and bless `cancel_counts.txt` for
  the new fixtures. Update ROADMAP §C6's status paragraph with the
  `fillet` counts beside C4's 17 of 38 and the `TangentChain` census, and
  run the docs-refs tests.
  Done: no part leaves the column (fetched 2 of 27, committed 5 of 11, 0
  failing parts), so no `fixtures:` expectation moved and `cancel_counts.txt`
  was blessed in step 3; the ROADMAP paragraph is written.

## Acceptance
- Step 1's census reproduces: FTC-06's 21 chain ends are named cusps, and
  step 5's numbers are what `tools/real-parts.sh` prints.
- Every fixture of steps 1 and 3 passes under `blend/`, checker green at
  `Full`, at Open CASCADE's oracle, or at the closed form under
  `analytic.measure_differs` with ADR-0015's evidence.
- Step 4's property is green at 256 cases (retirement) and 1000 (CI).
- FTC-06's battery fillet stays refused at an overhang tip by name, and
  the census counts its 21 ends under "walls on either side". The full
  profile is green.

## Docs to update on completion
- `docs/ROADMAP.md` §C6: the status paragraph, the counts, the census of
  `TangentChain`, and what the cycle has left (the crossing cylinders, a
  cylinder against a sphere, a trace that misses, the horn torus). Judge
  whether C6 is ready for `/close-cycle` and leave that call to it.
- `docs/adr/`: ADR-0042 and its README row, with pointers in ADR-0035 and
  ADR-0037.
- `docs/ARCHITECTURE.md` §Operations: the end at a cusp, and what
  `TangentChain` still means (written in step 3; check it at retirement).
- `docs/BACKLOG.md`: any other `TangentChain` end site. The S-bend, the
  overhang tip and the corner patch lines were written in step 3, with the
  coarse-mesh finding.
- `tests/fixtures/README.md`: the `TangentChain` line if its cases change.
- `CHANGELOG.md` `Unreleased`: a fillet or chamfer now ends at a cusp
  where two walls meet tangent, and what still refuses (no ADR numbers or
  fixture names).
- `AGENTS.md` current state: C6's seventh plan landed, and the column's
  count.

## Open questions
- Answered (step 2): the cut is traced and fitted (ADR-0037's machinery)
  between two closed-form trim points. Its branch ends at a node where the
  torus touches the next wall. It keeps the crescent's 6 faces, and the
  contact lines end on it.
- Answered (step 2): Open CASCADE's invalid cusps (11 of 21) do not depend
  on the radius and are not consistent between mirror images: it fails to
  split the pillar's face. Arris would not refuse them by name.
- Answered (human, step 2's gate): (a). Land the crescent family (the
  crescent and the pocket, cut by the next wall, exact surfaces). FTC-06
  stays in the column, its refusal moved to a named overhang-tip site.
