# Plan: blend-fan

- Started: 2026-10-03
- Milestone: C6, the blend network (docs/ROADMAP.md §C6)
- Idea: docs/ideas/blend-fan.md (absorbed), docs/ideas/crossing-cylinders.md
  (absorbed: option D)
- Idea (verbatim from the human): "/idea blend-fan", "I agree with your
  recommendations"; "/idea crossing-cylinders", then "/plan it"

## Goal
A fillet or a chamfer whose end reaches a vertex where the face across is
not one face is built where Open CASCADE builds it. This covers two sites.
**The fan:** both corner edges are sharp, and their faces across are
different faces, separated at the vertex by one or more sharp extra
edges. The end becomes one piece per face across, each cut as a lone
face across would cut it, and a new vertex on each extra edge it crosses.
This is CTC-01's 12 plane × plane edges on each tier: the hexagonal boss's
top edges and its chamfer facets' edges. **The face across met twice:**
both corner edges lead to the same face, but other edges also stand at
the vertex (`blend/five-edge-vertex`). The end is that face's single
piece, the vertex stays with the edges the end does not reach, and the
face across gains a loop. Separately, the crease between two equal-radius
cylinders whose axes intersect, CTC-01's other refusal, is attributed to
the NURBS cycle by name (`crossing-cylinders`, option D): its fillet is a
pipe about an ellipse, a surface Arris has no exact kind for. When the
plan is done the fetched tier has no `VertexBlend` edge, and CTC-01 sits
in the `fillet` column under NURBS rather than the blend network.

## Non-goals
- The crease's blend itself, fitted or exact (`crossing-cylinders`
  options A/B). That is the NURBS cycle's first ADR. Its `EllipticCylinder`
  chamfer waits for a consumer to ask.
- The 11 `Unsupported end: cylinder × cylinder` edges of CTC-01 (the
  trace that misses), the miters, `ftc-06`'s 3 other `VertexBlend` edges,
  and a vertex whose extra edge is smooth, a seam, or a continuation
  (ADR-0039 §4's other sites).
- A fan at a junction, a miter or a corner end: only a face end
  (`EndKind::Face`) fans.
- No public type or signature of a published crate changes. `Reason`
  gains no variant.

## Design deltas
- **ADR-0043** (step 2): an end across a fan, and an end whose face across
  is met twice. It amends ADR-0039 §4 (what `VertexBlend` keeps) and
  ADR-0007 §"Each end is trimmed by the face across the corner".
  - **Pieces and crossings.** The pieces are the faces met in the
    vertex's star between the two corner edges, on the end's side. Each
    piece's curve is the section a lone face across would give it:
    closed form (ADR-0007) or traced (ADR-0037). A crossing is where the
    blend surface pierces an extra edge, and it cuts that edge
    (`Modified`). The crossing vertex is `Generated` from the blended
    edge.
  - **The surviving vertex.** When the face across is met twice, the
    vertex stays with the edges the end does not reach, and the face
    across's loop splits in two.
  - **Refusals.** A crossing outside the extra edge's range is
    `BlendTooLarge`, naming the edge and the extra edge. A piece the
    section cannot cut is `Unsupported`, naming the blend and that piece.
- `crates/arris-ops/src/blend.rs`: `corner_of` returns the pieces across
  in star order rather than one face, and `End` carries a piece list (an
  arc per piece, a crossing per extra edge) in place of `face` and `arc`.
  These are internal. `rebuild` gives each piece's face its arc in its
  loop.
- `arris-debug` (`publish = false`):
  - `census::vertex_blend_cause` names the two sites apart: "a fan of
    k faces across" and "the face across met twice". The census names a
    cylinder pair by its axes: parallel, an equal-radius crease, or
    crossing.
  - The survey's fillet attribution puts a cylinder pair whose axes are
    not parallel under `Cycle::Nurbs`. That means `blocks_reason` takes
    what it needs to tell them apart (a signature change inside the debug
    crate, named in the commit).
- `docs/ARCHITECTURE.md` §Blends: an end's face across may be several
  pieces.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — **The fan, shrunk and named.**
  - Look at one of CTC-01's 12 fan vertices (`inspect` skill): which
    faces lie across, the angle of the extra edge, and where the end must
    cross it.
  - Shrink it to the smallest case Open CASCADE builds: a hexagonal prism
    with a chamfered top, one top edge filleted. Do the same for a
    chamfer.
  - Commit the two under `tests/fixtures/regression/` with Open CASCADE's
    oracle values, `#[ignore]`d.
  - `census::vertex_blend_cause` names "a fan of k faces across" and "the
    face across met twice" apart. The census test holds both on these
    fixtures and on `blend/five-edge-vertex`.
  - Seen: CTC-01's fan vertex is the foot of a chamfer facet where two
    facets meet over the side faces: corner edges the vertical edge and the
    facet miter, the extra edge the next side face against the next facet,
    k = 2. Open CASCADE builds the shrunk prism at 22 / 35 / 15 / 15 (loops)
    for both fillet and chamfer. The fixtures are
    `regression/hex-chamfer-foot-fan-{fillet,chamfer}`, a circumradius-1,
    height-2 prism, d 0.2, r and c 0.1.
- [x] Step 2 **[1]** — **ADR-0043**, as the design deltas state it, from
  what step 1 saw. It records the counts Open CASCADE reports for each
  site, and the pieces' order rule in the vertex's star.
- [x] Step 3 **[3]** — **The crossing fan built.**
  - `corner_of` reads the pieces, and `face_end` cuts each one between
    its trim point and the crossings.
  - The extra edges are cut at the crossings, provenance as ADR-0043
    says, and `rebuild` places each piece's arc in its face's loop.
  - Fillet and chamfer. Step 1's two fixtures move to `blend/` with their
    blessed dumps, held to the oracle, checker green.
  - A crossing that misses its extra edge is `BlendTooLarge`, tested on a
    radius past it.
  - Seen: both fixtures match Open CASCADE at 22 / 35 / 15 / 15 with
    every stage green, and no other dump in `blend/` moved. The foot's
    crossing lies within a few hundredths of the vertex at any radius
    its facet holds, so the `BlendTooLarge` test is on the hexagon's
    vertical edge instead. Its top is a fan of the two facets with the
    miter as the extra edge. The miter runs out between r 1.4 and 1.5
    for a fillet and between d 0.4 and 0.5 for a chamfer, where the
    arc's and the chord's nearest approach to the edge (0.155 r, 0.5 d)
    passes the miter's top (`a_fan_crossing_past_its_extra_edge_is_too_large`).
    Each piece's section is `section_between`, the cylinder and cone
    branch of a lone end factored out, so a curved piece (step 5) is
    already traced.
- [x] Step 4 **[2]** — **The face across met twice.**
  - The vertex stays with the edges the end does not reach, and the face
    across takes the end arc into a second loop.
  - `blend/five-edge-vertex` goes from `expect_error` to Open CASCADE's
    19 / 28 / 12, loops 13. This is a `fixtures:` commit: the fixture's
    expectation changes with an intentional geometry change.
  - Seen: the fixture matches Open CASCADE at 19 / 28 / 12 / 13 with every
    stage green. `twice_at` walks the star (corner edge, extra, third face,
    extra, the face across again, the other corner edge); the extras may be
    of either sense, as the box's top edges are against its concave
    bottom ones. The surviving vertex is kept, so provenance records
    nothing for it (ADR-0043 §5 says so now). The arc crossing a staying
    edge within both ranges is `BlendTooLarge` naming the edge.
- [x] Step 5 **[2]** — **A curved piece across.** A fan with one piece a
  cylinder, the face `across_of` admits as a post, so the piece is
  traced (ADR-0037). It is shrunk as a fixture with its oracle, so the
  per-piece cut is proven beyond planes. If Open CASCADE does not build
  it, the fixture records a named refusal instead, and the step says so.
  - Seen: Open CASCADE builds it. The fixtures are
    `blend/barrel-ridge-fan-{fillet,chamfer}`: a 3-cube cut to a barrel
    (radius 5 about an axis along x) and a plane z = h + 0.3 x through the
    corner's top, so the vertical edge at the origin ends at a vertex of
    four edges whose faces across are the plane and the cylinder. Both
    match the oracle at 14 / 21 / 9 / 9 with the checker green, and needed
    no code: `section_between` traces the cylinder piece as it does a lone
    one. They moved to `blend/` with their dumps in this step.
- [x] Step 6 **[2]** — **A property over random poses.**
  - The operands are prisms whose top is cut by two planes meeting along
    a sharp edge (k = 2 pieces, random angles), plus k = 3. Fillet and
    chamfer at random size, in random poses.
  - Checker green, and the volume against the closed form: the
    stripe's cross-section area times its centroid's run between the
    piece planes, piecewise across each crossing.
  - Sharded and seeded (`prop_shards!`).
  - Seen: `fans_{fillet,chamfer}_as_their_sections` in
    `blend_prop.rs`, 4 shards each, pass at 256 and at 1000 cases, and a
    volume off by 0.1% fails them. The operand is a block under a convex
    roof `h − max g_i·P` of two or three planes through the rise's top,
    built as a polyhedron; the volume change is minus the roof's height
    integrated over the blend's cross-section, exact in the corner the
    rise stands in, so the closed form does not need the crossings. The
    oracle's answer for `k = 3` is two fixtures,
    `blend/roof-fan-three-{fillet,chamfer}`: Open CASCADE builds both at
    14 / 21 / 9, and Arris matches (ADR-0043 §Consequences says so).
- [ ] Step 7 **[1]** — **The crease attributed to NURBS.**
  - The census names a cylinder pair by its axes. The survey's fillet
    attribution puts a non-parallel pair under `Cycle::Nurbs`, with a
    test on CTC-01's committed edition (`real/nist-ctc-01`) that its
    sample's refusal is the NURBS cycle's.
  - A `docs/BACKLOG.md` line for the crease's blend and its chamfer,
    under the NURBS cycle, carrying `crossing-cylinders`' geometry: the
    spine an ellipse, the contacts ellipses, the chamfer an
    `EllipticCylinder`.
- [ ] Step 8 **[1]** — **The tiers measured.** Run `tools/real-parts.sh`
  and `real_parts --census-committed`, and record the numbers in this
  plan:
  - the fetched and committed `VertexBlend` counts (expected 12 → 0 and
    15 → 3);
  - the `fillet` column by cycle (CTC-01 under NURBS on both tiers);
  - the battery stages (0 failing parts).

## Acceptance
- `cargo nextest run` at `ARRIS_GATE=full`: the fan fixtures (fillet,
  chamfer, curved piece) and `blend/five-edge-vertex` match Open
  CASCADE's oracle within their tolerances, checker green at `Full`.
- Step 6's property passes at 256 cases locally and at CI's 1000.
- `tools/real-parts.sh`: fetched `VertexBlend` 0, CTC-01's `fillet`
  refusal attributed to NURBS on both tiers, 0 failing parts. No part
  leaves the column, because no fan edge is sampled. That is expected and
  stated, not a miss.

## Docs to update on completion
- `docs/ROADMAP.md` §C6:
  - The status paragraph for this plan, with the numbers from step 8.
  - In the `VertexBlend` line, the fan done (ADR-0043).
  - In the **Out** line, "a blend whose surface has no closed form (the
    crease between equal-radius cylinders, the overhang tip's caps): the
    NURBS cycle's".
  - The cylinder × cylinder line split: parallel (ADR-0036 §5, no corpus
    edge) and crossing (NURBS).
- `docs/ARCHITECTURE.md` §Blends: an end across a fan of faces, the
  surviving vertex.
- `docs/adr/0039-…` §4: an *(Amended by ADR-0043)* note.
- `AGENTS.md` current state: the C6 line (the fan, ADR-0043; the crease
  to NURBS), kept under ~15 lines.
- `CHANGELOG.md` `## Unreleased`: "A fillet or chamfer now runs to a
  corner where the face across is split by another sharp edge, or met
  twice; a fillet along the crease where two equal rounds meet is still
  refused."

## Open questions
- Decided (ADR-0043 §2): the pieces' order when the star is not a simple
  fan — refuse `VertexBlend` unless the walk from one corner edge to the
  other, on the side away from the blended edge, is unique.
- Decided (ADR-0043 §4): at the surviving vertex the two edges left there
  are kept, as Open CASCADE does (19 vertices).
- Decided (step 3): a fan is built only where every edge at the vertex,
  the corner edges and the extras, is of the blend's sense. The blend
  then takes the corner from every piece. A fan with an edge of the
  other sense, the mixed corner of ADR-0038 spread over several faces,
  stays `VertexBlend`: no corpus edge has one, and each piece's side
  would have to be read apart. ADR-0043 §6 does not list it, so
  `/retire-plan` adds it there as an amendment note.
- ⚠ OPEN: how the attribution tells a non-parallel cylinder pair apart.
  Either `blocks_reason` reads the model, or the survey passes the census
  cause in. Agent, by step 7. Preferred: the survey passes the cause, so
  `blocks_reason` stays a function of the refusal and its context.
