# Plan: cusp-run-out

- Started: 2026-10-03
- Milestone: C6 (the blend network, docs/ROADMAP.md), seventh plan
- Idea: docs/ideas/tangent-continuation-and-fan.md (option B accepted, then
  absorbed; the fan kept as the parked idea `docs/ideas/blend-fan.md`)
- Idea (verbatim from the human): "I accept B, /plan it"; after step 1
  found the premise wrong: "Do A" (re-plan around the cusp)
- Renamed from `s-bend-walk` at step 1 (see its finding)

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

## Goal
A fillet or chamfer of an edge that ends at a cusp is built where Open
CASCADE builds it: the stripe runs to the cusp and is cut by the other wall
where the gap is narrower than the blend (a contact that leaves the top
face runs onto the next wall), at Open CASCADE's counts, measures and
probes. FTC-06 leaves the committed tier's `fillet` column (5 → 4) if its
four sampled edges together agree with Open CASCADE's stage
(`tests/fixtures/real/nist-ftc-06`). The cusps Open CASCADE builds invalid
stay refused, named by site. The census names a `TangentChain` end by its
site, the way it already names `BlendTooLarge` and `VertexBlend`.

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
- **A new rule for the walk's end at a cusp**, decided in step 2 and
  written as ADR-0042 (amending ADR-0035 §6, what `TangentChain` keeps, and
  ADR-0037/0038 if the cut reuses their end machinery): at a vertex of three
  edges where the blended edge's corner edge is a tangent dihedral and the
  next edge leaves the vertex the same way, the stripe is cut by the next
  edge's face where the gap is narrower than the blend. Whether the cut is
  the traced-and-fitted curve of ADR-0037 or a closed form is step 2's
  finding.
- `arris-ops` `blend.rs`: `corner_of` stops refusing a cusp's corner edge
  as `TangentChain` where the rule says it builds. No public type or
  signature change expected; `Reason` gains no variant, so
  `crates/arris-py` does not change.
- `arris-debug` `census.rs` (step 1, done): `tangent_chain_cause`, a
  `tangent_chain` field on `SolidCensus`, `ask_the_oracle_tangent` and
  `real_parts --tangent-chain <file>`.
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
- [ ] Step 2 **[3]** — The construction, on paper and against the oracle,
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
  ask the human whether FTC-06 waits for the NURBS cycle.
- [ ] Step 3 **[3]** — `corner_of` and the end at a cusp: the stripe cut by
  the next wall, the face counts and provenance of Open CASCADE's. The
  `regression/` fixture moves to `blend/` with its blessed dump, checker
  green at `Full`, at Open CASCADE's counts, measures and probes. Add the
  chamfer twin and the twin whose big arc is the concave one.
- [ ] Step 4 **[2]** — A property over random poses (`blend_prop`): a
  crescent of random radii whose cusp is one end, its floor or top edge
  filleted and chamfered at random size, small and large against the gap,
  against the checker at `Full`, the closed-form volume (the corner
  section swept along the arc, less the part cut by the wall), STEP
  round-trip, pose independence and deterministic ids. Seeded and sharded
  through `prop_shards!`. A pose Open CASCADE refuses is rejected by the
  refusal's name.
- [ ] Step 5 **[1]** — Measure the tiers again (`tools/real-parts.sh`,
  `--census-committed`). Move any `fixtures:` expectations whose refusals
  changed (the commit body says why) and bless `cancel_counts.txt` for
  the new fixtures. Update ROADMAP §C6's status paragraph with the
  `fillet` counts beside C4's 17 of 38 and the `TangentChain` census, and
  run the docs-refs tests.

## Acceptance
- Step 1's census reproduces: FTC-06's 21 chain ends are named cusps, and
  step 5's numbers are what `tools/real-parts.sh` prints.
- Every fixture of steps 1 and 3 passes under `blend/`, checker green at
  `Full`, at Open CASCADE's oracle, or at the closed form under
  `analytic.measure_differs` with ADR-0015's evidence.
- Step 4's property is green at 256 cases (retirement) and 1000 (CI).
- FTC-06's battery fillet agrees with Open CASCADE, or the plan names
  what refuses behind it and whose line that is. The full profile is
  green.

## Docs to update on completion
- `docs/ROADMAP.md` §C6: the status paragraph, the counts, the census of
  `TangentChain`, and what the cycle has left (the crossing cylinders, a
  cylinder against a sphere, a trace that misses, the horn torus). Judge
  whether C6 is ready for `/close-cycle` and leave that call to it.
- `docs/adr/`: ADR-0042 and its README row, with pointers in ADR-0035 and
  ADR-0037.
- `docs/ARCHITECTURE.md` §Operations: the end at a cusp, and what
  `TangentChain` still means.
- `docs/BACKLOG.md`: any other `TangentChain` end site, and the S-bend
  (inflection) chain as a line with the reason it waits.
- `tests/fixtures/README.md`: the `TangentChain` line if its cases change.
- `CHANGELOG.md` `Unreleased`: a fillet or chamfer now ends at a cusp
  where two walls meet tangent, and what still refuses (no ADR numbers or
  fixture names).
- `AGENTS.md` current state: C6's seventh plan landed, and the column's
  count.

## Open questions
- ⚠ OPEN (agent, step 2): is the cut of the stripe by the next wall a
  closed form or a traced curve, and does it keep Open CASCADE's face
  count (6 on the crescent) with the contact lines ending on it?
- ⚠ OPEN (agent, step 2): what makes Open CASCADE's 7 of 13 cusps invalid,
  and is that a radius against the gap (refuse by name) or something Arris
  can build?
