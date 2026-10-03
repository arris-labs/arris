# Plan: blend-corners

- Started: 2026-10-03
- Milestone: C6 (the blend network, docs/ROADMAP.md), fifth plan
- Idea: none. `blend-run-over` named the corners (`VertexBlend`) as the next
  plan's, and its census (`fillet-by-part.md`, `run-over.md`) stands in for
  the brainstorm: `VertexBlend` is `ctc_04`'s first refusal on the fetched
  tier and two of the committed tier's six, and 227 edges blended alone
  over six fetched parts. Step 2's gate is the idea's "change my mind"
  line: if step 1 shows the corners are mostly ones Open CASCADE also
  refuses, the plan stops there and the human is asked.
- Idea (verbatim from the human): "/plan blend-corners"

## Goal
`VertexBlend` is counted by sub-cause. Each edge is blended alone at the
battery's radius, and each sampled set is blended together, and every one
is held against Open CASCADE's verdict on it, so a refusal Open CASCADE
shares is told apart from one it builds. The sub-causes that Open CASCADE
builds and that a closed form or ADR-0037's trace can take are built. The
hypothesis step 1 tests first is a blend ending at a vertex of more than
three edges, where the end arc crosses a fan of faces across rather than
the one face ADR-0007 asks for. The surface stays exact and only the end
curve is traced where no closed form exists. The fetched tier's `fillet`
column (6 of 27) and the committed tier's (6 of 11) are measured again
beside C4's 17 of 38 and `blend-run-over`'s line, and what is left of
`VertexBlend` is named by part. *(Rewritten at step 3: the leading site
is not a fan but a chain running on where both faces turn, ADR-0039; the
fan is a backlog line.)*

## Non-goals
- Corners Open CASCADE also refuses. They stay `VertexBlend`, correctly
  (`both refuse` in the battery).
- Tangent continuation (`TangentChain`, STC-09), the horn and spindle torus
  at the axis (`BlendTooLarge`, STC-06), crossing cylinders, the torus off
  its axis, and a trace that misses (`ctc_01` ap242). Each is its own line.
- A corner patch that is not a closed form or an exact surface: no fitted
  N-sided filling (Open CASCADE's `PerformMoreThreeCorner`) and no
  setback vertex blend. ADR-0007's rejected alternative stays rejected.
  Where a sub-cause needs one, it is named and left for a later idea.
- Variable radius, blends over blends, other chamfer modes, NURBS faces as
  operands, healing.
- The miters of several blended edges in the backlog (unequal dihedrals,
  a ruling blend's miter, the oblique three-plane fillet corner). These
  are in scope only if step 1 finds them behind a part's sampled set
  blended together (step 6). Otherwise they stay backlog lines.

## Design deltas
- **ADR-0039 (step 3), written.** The census's leading site is a chain
  junction: a vertex of four edges where both of the blended edge's faces
  turn tangentially and the extra edge is its continuation. The selection
  follows it (amends ADR-0035 §1, §3, §6), and its junction runs between
  points on the two tangent edges, both cut. `tangent_vertex` takes the
  four-edge case. `junction` (step 4) takes runs that share no face:
  `Miter`'s point `q` on the shared face becomes, at such a vertex, a
  second cut edge, so `Miter` gains an optional second `Trim`, and
  `build` cuts both. No public type or signature change; `Reason` gains no
  variant. The bullets below are the plan as opened, kept for the record.
- *(Superseded at step 3.)* **ADR-0039 (step 3), new.** This is the corner the census finds leading.
  For the hypothesis, an end at a vertex of more than three edges: the
  end's section is cut by each face across in turn around the vertex,
  every face across taking its piece of the end curve. The edges between
  those faces are cut where the curve crosses them, and the corner edges
  are cut or lengthened as ADR-0007 and ADR-0038 already rule. It amends
  ADR-0007 (the face across, the `VertexBlend` bound). If step 1 leads
  elsewhere, the ADR records that corner instead, and step 2 rewrites
  steps 3 to 5 in this file.
- **ADR-0040 (step 5), written.** Tangency for a blend: `tangent_normals`
  (the angular precision, or the size times the sine within the faces'
  tolerance) behind `tangent_at`, the stripe's and the ring's own
  dihedral; `tangent_vertex`, `chain` and `corner_of` take the blend's
  size, `Stripe` carries it. No public type or signature change.
- `arris-ops` `blend.rs` `corner_of`: from three edges and one face across
  to the vertex's fan (the corner edges, the faces across and the edges
  between them, in loop order). `face_end`, `End` and `build` take a fan
  of more than one face across. Expect a new module, `blend/fan.rs`, beside
  `mixed.rs` and `traced.rs`. No public type or signature change.
  `Reason::VertexBlend` keeps its variant and loses cases, and the
  `fillet` rustdoc's error list is narrowed to match. `Reason` gains no
  variant, so the binding (`crates/arris-py`) does not change.
- `arris-check`: no change expected (S5 decides each face across by the
  same tracers, ADR-0037 §5). Step 4 shows this at `Full`.
- `arris-debug` `census.rs`: `vertex_blend_cause`, read from the entities
  the refusal names as `run_over_cause` is, plus the sampled set blended
  together, plus Open CASCADE's verdict on each (step 1).
- Docs: `docs/ARCHITECTURE.md` §blend (the end at a corner and the
  `VertexBlend` paragraph) and ROADMAP §C6.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The census of `VertexBlend` by site. Extend
  `arris_debug::census` with `vertex_blend_cause`. It classifies every
  `VertexBlend` edge, blended alone at the battery's radius, by the site
  that refused it:
  - a vertex of more than three edges, split by what the extra edges are:
    a sharp edge between two faces across; a smooth or tangent edge; a
    seam; a degenerate edge; or an edge that continues the corner edge
    collinearly;
  - corner edges that share no face across;
  - a corner edge with no curve (an apex or a pole);
  - a closed edge's vertex carrying more than its seams.

  It also blends each part's sampled set together, and where that alone
  is `VertexBlend`, classifies the miter or corner the edges make. Each
  edge and set gets **Open CASCADE's verdict** through
  `occt_fillet_edges.py` (`ask_the_oracle`, a stride per cause plus every
  sampled edge). The output is `target/real-parts/vertex-blend.md`,
  `part → cause → edges → asked → builds / invalid / refuses`, written
  through `real_parts --vertex-blend` and both tiers. A unit test on a
  small recipe per cause holds the classification: `blend/five-edge-vertex`
  for the fan, a cone apex for the curveless corner edge, and so on.
- [x] Step 2 **[1]** — Record the census in `docs/ROADMAP.md` §C6 and
  `docs/BACKLOG.md`: each cause with its count and Open CASCADE's verdict,
  and the parts its sampled edges would free. Confirm the order of steps 3
  to 6 or rewrite them here. **Gate:** if the edges Open CASCADE builds
  fall in no cause that a closed form or the trace can take, or would
  free no part of either tier's column, stop and ask the human before
  step 3. A change of order is recorded here, not made silently.
- [x] Step 3 **[3]** — *(Rewritten at step 3.)* The decision and the walk.
  The sites were read before building: of the committed tier's 160
  four-edge edges, 142 are a chain running on where both faces turn and 18
  (CTC-01) a fan, and every sampled edge is the former. ADR-0039 written;
  `tangent_vertex` takes the vertex of four edges; the junction there, its
  runs sharing no face, refuses `VertexBlend` naming both edges and the
  vertex until step 4. The census names the blended edge's continuation
  and a junction by its vertex. `fillet.rs`
  `a_chain_runs_on_where_both_faces_turn` (fillet and chamfer, two poses)
  and the census's unit test hold the walk;
  `regression/chamfered-stadium-foot-fillet` (a chamfered stadium's
  chamfer foot filleted, the shrunk CTC-04 wall) holds Open CASCADE's
  oracle, `#[ignore]`d. Counted: the walk takes 112 committed edges (CTC-04
  104, FTC-09 8) and 20 fetched; CTC-04's chamfered walls are tangent only
  to `9e-10` as read and are not walked (open question below).
  `fixtures:` CTC-04's battery fillet now meets `TangentChain` first.
- [x] Step 4 **[3]** — Build the junction at a vertex of four edges, on a
  line meeting an arc (the chamfered stadium's foot). `junction` takes runs
  that share no face: the arc between the points on `w₀` and `w₁`, both
  cut, and each blend face's contacts meeting there. Provenance as ADR-0039
  §3. `regression/chamfered-stadium-foot-fillet` moves to `blend/` with
  its blessed dump. Add a chamfer twin and a concave chain through such a
  vertex (a pocket's floor edge running round a rounded, chamfered corner),
  each with Open CASCADE's oracle. The checker is green at `Full`.
  *(Done: `Miter` gained `q_trim`, `build` cuts it at `q`.
  `blend/chamfered-stadium-foot-fillet`, `-chamfer` and
  `blend/stadium-pocket-chamfer-root-fillet` pass at Open CASCADE's counts
  and probes. Open CASCADE's cylinder-against-cone blend, fillet and
  chamfer, is a walked B-spline, off the closed forms by 5e-9 to 2e-7, so
  each fixture states every closed form (the section swept along the
  stadium's offset curves), `occt_walked`, and measure tolerances widened
  under ADR-0015's 1e-6, as `blend/turned-shoulder-fillet` does; Arris
  matches the closed forms to rounding. `junction` reads either run as a
  stripe or a ring, so two arcs meet the same way: FTC-09's battery
  fillet, held by its pin's split rim, now builds and agrees with Open
  CASCADE, and its `fixtures:` expectation and the committed histogram
  moved here, not at step 8.)*
- [x] Step 5 **[3]** — *(Step 4 found the code already takes it, FTC-09
  agreeing; what is left is the fixture and the tangency question.)* The junction of two arcs: a rim split in half with
  both its faces, the ring of each half meeting the other's at both ends
  (CTC-04's and FTC-09's pins, a cone against a cylinder; a plane against a
  cylinder beside it). A fixture of a split pin with its oracle. Then
  decide the open question on tangency as read, and if it is decided for
  the edge's own tolerance, amend ADR-0035's test with its ADR and cover
  CTC-04's chamfered walls with a fixture read from STEP.
  *(Done: `blend/split-pin-foot-fillet`, two half tori at Open CASCADE's
  counts and probes, the closed forms Pappus's, Open CASCADE's walked
  blend 1e-7 to 4e-7 off them. ADR-0040 decides tangency for the blend:
  its size times the normals' sine within the faces' tolerance (the
  faces', not the edge's: the junction meets contacts at theirs), at every
  tangency a blend reads. `blend/nist-ctc-04-chamfered-wall-fillet` reads
  the part in place and walks a wall's whole outline, twelve edges, at
  Open CASCADE's counts. The committed `VertexBlend` edges go from 63 to
  35; FTC-08 and FTC-10 read written tangencies as tangent too. `fixtures:`
  FTC-08's battery fillet now refuses `TangentChain`, and the committed
  histogram is printed again here, not at step 8. The checker-on census
  found FTC-08's pre-existing fault:
  `regression/nist-ftc-08-fillet-pcurve-jump-checker-fault`. Step 4's
  fixtures' `cancel_counts.txt` lines are blessed here.)*
- [-] Step 6 **[3]** — Struck at step 2 (the census found no part whose sampled set is refused `VertexBlend` while each edge alone builds: in all three sets a single sampled edge is refused alone, so the miter lines stay in the backlog). Was: conditional on step 2: the sampled set blended
  together. Only if step 1 finds a part's set refused `VertexBlend` where
  each edge alone builds, build the miter or corner it names, from the
  backlog's three: unequal dihedrals, a ruling blend's miter, or the
  oblique three-plane fillet corner (a tilted great circle whose pcurve is
  fitted on the sphere). Add a fixture with Open CASCADE's oracle. If the
  census finds none, this step is struck at step 2, and the reason is
  written here.
- [x] Step 7 **[2]** — A property over random poses for every case steps 4
  to 6 build (`blend_prop`): a chain through a vertex of four edges — a
  chamfered stadium or rounded rectangle at random sizes, chamfer and
  radius, and a split rim. *(Rewritten at step 3; was: a fan of two to four
  faces across, a split angle and a lean.)* It checks that the checker is green, that the blended
  and removed material add up in volume, that a STEP round-trip holds,
  and that ids are deterministic. Poses where the end leaves the fan are
  rejected by the refusal's name, not silently. Seeded, and sharded via
  `prop_shards!`.
  *(Done: `fillets_` and `chamfers_through_a_vertex_of_four_edges_match_
  their_closed_forms`, four shards each, a stadium or a split rim at random
  sizes and poses, the volume the corner's section swept along the outline's
  offsets, `2l + 2π(R − depth)` long. No pose left the walk. It found one
  knife-edge pose where S5 leaves a plane against a torus undecided; a
  fixture's axis and angle cannot carry the exact quaternion, so it is the
  `#[ignore]`d `a_walked_chain_in_a_far_pose_is_decided` beside the property,
  which excludes that pair by name until it passes. A backlog line.)*
- [x] Step 8 **[1]** — Measure the fetched and committed tiers again
  (`tools/real-parts.sh`, `--census-committed`). Print the column beside
  C4's 17 of 38 and `blend-run-over`'s line. Move the `fixtures:`
  expectations whose refusals changed (`nist-ctc-04`; `nist-ftc-09` moved
  at step 4, `nist-ftc-08` at step 5), with
  the commit body saying why and `cancel_counts.txt` blessed for the new
  fixtures. Align the battery's tangent-dihedral filter
  (`arris_debug::battery::is_tangent_dihedral`, the angular precision
  alone) with ADR-0040 first, at the battery's radius, so the sample holds
  no edge the kernel names a tangent dihedral, and say in the commit body
  which parts' samples moved. Update `docs/ROADMAP.md` §C6's status paragraph with the new
  count and what each remaining part meets. Run the docs-refs tests: the
  roadmap's histogram is checked against the printout.
  *(Done: `fillet_edges` settles the sample and the radius together, the
  tangent test at the radius as ADR-0040 reads it. The fetched column is 4
  of 27 (from 6), the committed census's 5 of 11 (from 6); no fixture
  expectation moves, the battery operands being frozen data and the
  committed histogram unchanged.)*

## Acceptance
- Step 1's cause table is reproduced, and the column's summary line in §C6
  is the number `tools/real-parts.sh` prints after step 8.
- Every fixture of steps 4 to 6 passes under `blend/`, checker green at
  `Full`. Each is held to Open CASCADE's oracle, or to its closed form
  under `analytic.measure_differs` with ADR-0015's evidence.
  `regression/chamfered-stadium-foot-fillet` is built under `blend/`.
  *(Rewritten at step 3; was `blend/five-edge-vertex` built, now the fan's
  backlog line.)*
- Step 7's property is green at 256 cases (retirement) and 1000 (CI).
- Every part that leaves the column agrees with Open CASCADE's fillet at
  the battery's radius, or is refused as another line's (tangent
  continuation, the horn torus, NURBS, a corner Open CASCADE refuses).
  The full profile is green.

## Docs to update on completion
- `docs/ROADMAP.md` §C6: the status paragraph with the new `fillet` counts
  and the line this plan strikes (the corners). Name what the cycle has
  left: tangent continuation, the crossing cylinders, the torus off its
  axis, a trace that misses, the horn torus. Judge whether the cycle can
  close, and leave that judgement for `/close-cycle`.
- `docs/BACKLOG.md`: what the census leaves. Corners Open CASCADE refuses
  are not backlog; causes needing a filled patch are. Strike or keep the
  three miter lines (65, 66, 68) per step 6.
- `docs/adr/`: ADR-0039, its ADR-0007 and ADR-0035 pointers and its
  README row (written at step 3); ADR-0040, its ADR-0035 and ADR-0039
  pointers and its README row (written at step 5).
- `docs/ARCHITECTURE.md` §blend: the chain through a vertex of four edges
  and its junction (the walk written at step 3) and the `VertexBlend`
  paragraph (what builds now and what is refused).
- `tests/fixtures/README.md`: the `VertexBlend` line, if its cases change.
- `CHANGELOG.md` `Unreleased`: that a fillet or chamfer now runs on
  through a vertex where both of its faces turn tangentially, and which
  corner refusals remain (no ADR numbers or fixture names).
- `AGENTS.md` current state: C6's fifth plan landed, and the column's
  count.

## Open questions
- Step 2's census (2026-10-03): the fan is the leading site and Open CASCADE
  builds it (the four-edge vertex with one sharp extra edge: 60 of 60 asked on
  plane × plane and cone × cylinder, fetched; 46 of 46, committed), so the
  gate does not fire and the order of steps 3 to 5 stands. The fan is two
  faces across, one extra edge, not a larger one: step 7's property takes two
  to four faces across all the same. A second site (a vertex of two edges on a
  cylinder × plane, 77 fetched edges) is a backlog line, not this plan's.
  *(Overturned at step 3: the census's "sharp extra edge" was mostly the
  blended edge's own tangent continuation; see the next line and
  ADR-0039.)*
- Answered at step 3 (ADR-0039): the cause that leads is neither the fan
  nor a curveless corner edge but a chain junction at a vertex of four
  edges; steps 3 to 5 are rewritten for it. The fan is a backlog line.
- Was ⚠ OPEN (agent, step 2), answered above: which cause leads. The fan of faces across is
  the hypothesis. `blend/five-edge-vertex` is a fan, and a real part's face
  split or a fused boss at an edge's end makes one. If step 1 finds that
  the curveless corner edge or a corner with no face across leads, steps 3
  to 5 are rewritten for it at step 2, keeping their grades and
  acceptance.
- Answered at step 3 (ADR-0039 §4): a vertex whose extra edge is smooth,
  a seam or continues a corner edge is neither; it stays `VertexBlend`.
  The junction is the vertex whose corner edges are both tangent and whose
  extra edge continues the blended one.
- Was ⚠ OPEN (agent, step 3), answered above: whether a vertex whose extra
  edge is smooth or a seam is a fan of faces across, or the tangent
  junction of ADR-0035. If
  it is a junction, it is `TangentChain`'s, and the census counts it
  there.
- Answered at step 3 (ADR-0039 §3): each cut tangent edge is `Modified`,
  as ADR-0035 §5 cuts its `w`.
- Was ⚠ OPEN (agent, step 4), answered above: whether an edge between two
  faces across, cut by the end, keeps its id on the piece away from the vertex (`Modified`), or
  is deleted and both pieces generated. Decide it the way ADR-0007 decides
  a shortened corner edge (`Modified`), unless ADR-0009's split order says
  otherwise.
- Not reached: ⚠ OPEN (human, step 2, only if the gate fires): whether to stop the plan
  or go on with a cause Open CASCADE builds that frees no part.
- Answered at step 5 (ADR-0040): tangent for the blend, its size times the
  normals' sine within the faces' tolerance, at every tangency it reads.
- Was ⚠ OPEN (agent, step 5), answered above: whether a corner edge is tangent by the model's
  angular precision (`1e-12`, ADR-0035's test) or by its own tolerance.
  CTC-04's chamfered walls, read from STEP, meet tangentially only to
  `7e-11`–`9e-10` between normals, which moves the ball by `r` times that,
  far inside the edges' `1e-7`; at `1e-12` the walk does not take them and
  they stay `VertexBlend` (24 committed edges, 24 on the fetched AP203
  edition). The change reaches every chain, three edges or four, so it is
  an amendment of ADR-0035 with its own evidence, decided in step 5.
