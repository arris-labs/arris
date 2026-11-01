# Plan: flush-fan-end

- Started: 2026-11-01
- Milestone: M8 (cycle C11, docs/ROADMAP.md §C11, "Corners the closed forms do not cover": the hinge leaf's knuckle joins)
- Idea: none. The roadmap's in-list line, read at the refusal: `/next` → `/plan` on 2026-11-01

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal

A **concave** fillet or chamfer builds when the face across at its end is
split into pieces on one surface. That is what a fuse of flush faces leaves,
since same-domain faces are not merged (C11 Out). The script corpus's last
`fillet` refusal, the hinge leaf's `knuckle_joins`, runs to `ok` at every
stage. The inner edge of an L made by fusing two boxes builds too, as does the
concave edge of a step fused from two boxes. Today each of these is
`VertexBlend` (reproduced 2026-11-01). The convex ends of the same bodies
already build. The end's section is the closed form of a lone face across,
and its measures match Open CASCADE's within the fixture's tolerance.

## Non-goals

- Pieces on a non-planar surface: a flush fan on one cylinder or sphere.
  These stay `VertexBlend`, named. Each gets a backlog line once something
  asks for it.
- Merging same-domain faces, or the pieces at the end (C11 Out). A
  consumer's faces and edges are never merged (ADR-0043, alternatives).
- A flush fan at a junction, a miter or a corner end. Only a face end
  (`EndKind::Face`) takes it, as in ADR-0043 §6.
- The cycle line's other two corners, each a milestone plan after this one:
  - `walked-seam-junction`: a walked chain junction on a seam the loop
    crosses at a slant (`regression/sphere-cylinder-oblique-seam-fillet`).
  - `cusp-corner-patch`: a cusp whose two edges are both blended, as a fitted
    corner patch (`blend/cusp-crescent-both-arcs-fillet`). This is the [3]
    one, and the concave overhang's fitted cap (FTC-06) sits beside it.

## Design deltas

- **The site (ADR-0082, step 1).** The site is a concave face end at a vertex
  `v` of more than three edges. Its corner edges `c1` and `c2` lead to two
  different faces across, `A1` and `Ak`. ADR-0043 §2's walk from `c1` to `c2`
  meets only **smooth** extras, which today `fan_at` refuses. Every piece is
  a plane, and all the pieces are coincident within their tolerance. The
  glossary names this a *flush fan*.
- **The construction (ADR-0082).** A concave end's section `a` lies on the
  pieces' surface, in the empty wedge between `c1` and `c2`. That wedge is
  the side of `v` opposite the walk, so `a` crosses no extra.
  - `a` is ADR-0007's arc or chord for one lone face across.
  - The region that `a`, `c1` from `P1` to `v` and `c2` from `v` to `P2`
    enclose is given to one piece. The intended rule is that `A1` takes it,
    deterministically.
  - `c2` is cut at `P2`. Its part from `v` to `P2` becomes an edge between
    `A1` and `Ak`, the way ADR-0043 §4 cuts and keeps edges.
  - `v` survives with the extras, which stay unchanged.

  Step 1 confirms this choice against Open CASCADE's counts, or reverses it
  with the reason.
- **Provenance.** `v` is kept, with no record. `P1` and `P2` and the arc are
  `Generated` from `e`. `c1` and `c2` are `Modified`, and so are `A1` and
  `Ak`. The middle pieces are not touched.
- **Refusals.** If `a` meets an extra's curve within the extra's range, the
  end is `BlendTooLarge`, naming `e` and that extra. Non-planar or
  non-coincident pieces, a mixed-sense extra, and a convex blend at a smooth
  extra all stay `VertexBlend`, as ADR-0039 §4 keeps them.
- **Types.** None public. `ends.rs` gains `flush_fan_at` beside `fan_at` and
  `twice_at`. `End` carries the cut of `c2` the same way the face met twice
  carries its kept vertex. `Reason` gains no variant, so the binding is
  untouched.
- `docs/ARCHITECTURE.md` §blend ends gets one paragraph. `GLOSSARY.md` gets
  *flush fan*.

## Steps

Grades are tiers (`.agents/tiers.md`). **[1]** routine; **[2]** careful,
a geometric or numeric case within the design; **[3]** unproven.

- [x] Step 1 **[2]** — Fixtures and the decision. Add `regression/` fixtures
  for:
  - `flush-l-inner-{fillet,chamfer}`, the L of two fused boxes;
  - `flush-step-concave-{fillet,chamfer}`;
  - `knuckle-join-flush-end-fillet`, a box fused with a half-buried cylinder
    with one join filleted, the hinge leaf shrunk.

  Each has its Open CASCADE oracle and is `#[ignore]`d. Write ADR-0082 from
  their counts: which piece takes the region, and whether `v` survives.
- [x] Step 2 **[2]** — `flush_fan_at` recognises the site, and `corner_of`
  routes it. A unit test over the three bodies' stars checks the pieces and
  the smooth extras. The census names the site "a flush fan"
  (`census::vertex_blend_cause`).
- [ ] Step 3 **[2]** — Build the two-piece end: `A1` takes the region, `c2`
  is cut at `P2`, `v` is kept, provenance as above. The `flush-l-inner`
  fixtures move to `blend/` and are checker-green at `Full`, at the oracle's
  counts and measures.
- [ ] Step 4 **[2]** — Three pieces, with a middle piece left untouched:
  `knuckle-join-flush-end-fillet` and `flush-step-concave-*` move to `blend/`.
  The hinge leaf's `outcome.json` becomes `ok` at `knuckle_joins` and matches
  its oracle checkpoint.
- [ ] Step 5 **[1]** — Refusals by name, each a `blend/` fixture holding its
  expected refusal:
  - an end whose arc runs past an extra: `BlendTooLarge`;
  - flush pieces on a cylinder: `VertexBlend`;
  - a convex end at a smooth extra: `VertexBlend`.
- [ ] Step 6 **[2]** — Properties: the blend property's generator gains
  flush-fused L and step operands, with 2 and 3 pieces in random poses, for
  fillet and chamfer. Each is held to the closed-form volume of a lone
  concave end (`l-inner-edge`'s formula) and to checker-green.
- [ ] Step 7 **[1]** — Census: `tools/script-corpus.sh` shows 0 `fillet`
  refusals, and the real-parts battery re-run stays at 0 of 38. Commit the
  numbers to the roadmap's §C11 line.

## Acceptance

- `cargo nextest run -p arris-ops` passes at the fast profile. The five new
  `blend/` fixtures meet their oracle counts, volume, area and centroid, and
  each refusal fixture meets its named reason.
- `tools/py-test.sh tests/test_corpus.py -k hinge` is green, with the
  hinge leaf `ok` at every stage against its oracle.
- `tools/script-corpus.sh` shows `fillet` refusals at 0. The roadmap's
  accept asks for "below 3", and the count was 1.
- The real-parts battery's blend row stays at 0 of 38, and no panic.

## Docs to update on completion

- `docs/ROADMAP.md` §C11: tick the hinge leaf in "Corners the closed forms
  do not cover", with the census numbers. The other two corners stay
  unticked, under their plan names.
- `docs/ARCHITECTURE.md` §blend ends: the flush fan, one paragraph.
- `GLOSSARY.md`: *flush fan*.
- `docs/BACKLOG.md`: drop the hinge leaf from line 126's family ("a smooth
  extra edge"), keeping the rest. Add a line for a flush fan on a curved
  surface.
- `CHANGELOG.md` Unreleased: "A concave fillet or chamfer whose end face a
  fuse left in flush pieces now builds".
- `AGENTS.md` current state: M8.

## Open questions

- Settled (step 1, ADR-0082): `A1` takes the end's region. Open CASCADE's
  counts (+4 vertices, +5 edges, +1 face) show no new face and `v` kept, and
  cannot tell `A1` from `Ak`; `A1` keeps the face count and is deterministic.
- Settled (step 2): the coincidence test is the pieces' outward normals the
  same way within `tol.angular` and the vertex within `faces_tolerance` of
  each plane, no new literal. The walk's corner edges are **convex** (the
  walls' edges with the cap) where the blended edge is concave; the smooth
  extras are `tangent_at`. `corner_of` returns `CornerAt { flush: true }`,
  and `end_at` refuses it `VertexBlend` until step 3 builds it.
