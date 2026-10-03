# ADR-0043 — A blend's end across a fan of faces, and across a face met twice

- Status: accepted (2026-10-03)
- Plan: `blend-fan` step 2 (the decision, against the oracle), step 3 (the
  fan built), step 4 (the face across met twice)
- Amends: ADR-0039 §4 (what `VertexBlend` keeps); ADR-0007 §"Each end is
  trimmed by the face across the corner"
- Follows: ADR-0037 (an end the face across cuts in no closed form is
  traced and fitted), ADR-0038 (the face across takes the end)

## Context

ADR-0039 §4 left two sites at a vertex of more than three edges refused
`VertexBlend`: the **fan**, and the vertex where the face across is met
twice. `blend-fan` step 1 looked at both.

- **The fan.** CTC-01's 12 plane × plane edges, on each tier, are the foot
  of a chamfer facet on the hexagonal boss: a vertex `v` of four edges.
  The blended edge `e` runs between a side face `F1` and a facet `F2`. Its
  corner edges are the boss's vertical edge `c1`, between `F1` and the next
  side face `A1`, and the miter `c2`, between `F2` and the next facet `A2`.
  The fourth edge `x` is sharp and separates `A1` from `A2`. The census now
  names it, "a fan of 2 faces across"
  (`census::vertex_blend_cause`). Open CASCADE builds the shrunk case, a
  hexagonal prism with chamfered top and one facet's foot filleted or
  chamfered (`regression/hex-chamfer-foot-fan-{fillet,chamfer}`): 18
  vertices, 30 edges, 14 faces and 14 loops become 22, 35, 15 and 15, for a
  fillet and for a chamfer alike. Both ends are fans of two.
- **The face across met twice.** `blend/five-edge-vertex`: the edge `e` is
  the rise of a box standing on another's top edge, `v` its bottom corner
  on that edge. Both corner edges are bottom edges of the upper box and
  lead to one face, `T`, the lower box's top. The two halves of the top
  edge, `h1` and `h2`, also stand at `v`. Open CASCADE builds it at 19
  vertices, 28 edges, 12 faces and 13 loops (a fillet of either half of the
  top edge it reports not done).

Read in the reference trees: Open CASCADE's `ChFi3d_Builder` trims an end
face by face across a vertex of more than three edges, and keeps the
vertex where an edge still stands at it. Read for the structure; nothing
taken.

## Decision

**1. The site.** The end of a blended edge `e` at a vertex `v` that has
more than three edges, where all of the following hold:

- `e` has two faces `F1` and `F2`, and its corner edges are `c1` in `F1`'s
  loop and `c2` in `F2`'s, each sharp, with a curve.
- Every other edge at `v` (an *extra* edge) is sharp, has two faces, and is
  no seam. A smooth, seam or continuing extra edge stays `VertexBlend`
  (ADR-0039 §4's other sites).
- Let `A1` be `c1`'s face other than `F1` and `A2` be `c2`'s other than
  `F2`. If `A1 = A2`, this is a face met twice (§4). Otherwise it is a fan
  and the **walk** below is unique.

**2. The pieces' order: one walk through the star.** The pieces of a fan are
the faces met on the way from `c1` to `c2` in the vertex's star, **on the
end's side**: the side that does not hold `e`. Start at `c1` in `A1`. In
the loop of the current face, the edge beside the current edge that
touches `v` is the next edge. The face on its other side is the next
piece. Stop at `c2`. The walk is **unique** when at each step the face's
loop has exactly one such edge, the edge has exactly two faces, and it
ends at `c2` in `A2` within as many steps as `v` has edges. The result is
`A1 = A_1, …, A_k = A2` and the extras `x_1, …, x_{k−1}` between them, `k ≥ 2`.
Anything else (a sheet's edge, an edge used twice, a face revisited) is
`VertexBlend`, naming `e` and `v`. This closes the plan's first open
question.

**3. A fan's end is one arc per piece, cut where it crosses the extras.**

- The arc `a_i` on `A_i` is the section a lone face across would give: the
  closed form of ADR-0007 where one exists, the traced and fitted one of
  ADR-0037 otherwise. It joins its two points: `P1` on `c1` and the
  crossing `X_1` for `a_1`, `X_{i−1}` and `X_i` for a middle arc, `X_{k−1}`
  and `P2` on `c2` for `a_k`.
- `P1` and `P2` are the trim points of a lone face across, cutting `c1`
  and `c2` as ADR-0007 does. `X_i` is where the blend's surface pierces
  `x_i`, the root of `x_i`'s curve against the stripe, in the stripe's
  extent (`u` in `[0, u1]`) and in `x_i`'s range. For a line against a
  cylinder or a plane that is a quadratic or linear root, for a circle
  against one it is the intersector's closed form.
- The arc `a_i` and the arc `a_{i+1}` meet at `X_i` within the tolerance.
  For a chamfer the arcs are chords, for a fillet the section of its
  cylinder with each plane.

**4. The face across met twice is one piece, and `v` survives.** The end is
`A`'s single arc `a` from `P1` to `P2`, as for a lone face across. The
extras are not reached. They must not be: if `a` crosses the curve of an
extra edge within its range, the stripe is too wide for the face and the
end is `BlendTooLarge`, naming `e` and that extra edge. `v` is not
consumed. It stays with the edges the end does not reach, and **the two
edges left at it are not merged**: Open CASCADE keeps them (19 vertices),
and a consumer's edges are never merged (ADR-0039, alternatives). This
closes the plan's second open question. `A`'s loop, which met itself at `v`,
splits in two: the outer loop through the extras and `v`, and a loop of
`a`, the cut corner edges and the rest of the footprint. `A` has one loop
more, as Open CASCADE's 13 loops over 12 faces say.

**5. Topology and provenance.**

- A fan: `v` is `Deleted`. The trim points and each crossing are `Generated`
  from `e`. `c1`, `c2` and each `x_i` are `Modified`, each cut where its
  point lies and shortened toward its far vertex. Each arc is `Generated`
  from `e`, and each `A_i` is `Modified` and gains its arc in its loop in
  place of the part of the loop that went round `v`.
- A face met twice: `v` is kept, with the edges that stay at it, which
  are unchanged; it keeps its id, so the record names nothing for it. The trim points and the arc are `Generated` from `e`. `c1`
  and `c2` are `Modified`, and `A` is `Modified`.
- Counts. A fan of `k` pieces at one end adds `k` vertices (two trim points
  and `k − 1` crossings, in place of `v`) and `k` edges (the arcs) to a
  blend's contacts, its face and its far end. The hexagonal prism's two
  ends of two make `+4` vertices and `+5` edges (the two contacts and four
  arcs, less `e`), with the face: 22, 35 and 15, Open CASCADE's.

**6. What is refused, by name.**

- A crossing outside its extra edge's range, or no crossing there:
  `BlendTooLarge`, naming `e` and `x_i`.
- A piece the section cannot cut, the face across a surface the table and
  the tracers do not decide: `Unsupported`, naming the stripe's surface
  kind and the piece's, as ADR-0037 §6 does.
- A walk that is not unique, an extra edge that is smooth, a seam or a
  continuation, a corner edge with no curve: `VertexBlend`, as before.
- A fan at a junction, a miter or a corner end: only a face end
  (`EndKind::Face`) fans.

`Reason` gains no variant, so the binding does not change.

## Consequences

- CTC-01's 12 fan edges build. No part leaves the battery's `fillet`
  column, because no fan edge is among its four sampled edges; the
  census's `VertexBlend` count on the fetched tier goes from 12 to 0.
- `blend/five-edge-vertex` goes from an expected `VertexBlend` to Open
  CASCADE's counts.
- `End` carries a list of pieces in place of one face and one arc
  (internal to `arris-ops`), and a corner edge's cut vertex may now be
  shared with an extra edge's crossing, so each piece's arcs are placed in
  its own loop.
- A fan of three or more faces is not in the corpus. The property of step 6
  covers `k = 3`, and the oracle's answer for it is part of that step.

## Alternatives considered

- **A fan blended piece by piece, each face across a lone end.** The arcs
  would not meet at the crossing: each end would run past the extra edge
  into the next face's plane. The result is not closed.
- **Merging the two edges left at the surviving vertex.** It saves a vertex
  but changes a consumer's edges, which no operation does (ADR-0039,
  alternatives), and Open CASCADE does not.
- **Refusing the fan with a tolerance-sized crossing.** A crossing at an
  extra edge's end is the crossing of the next vertex, a different site.
  It is refused `BlendTooLarge`, as a corner edge cut at its far vertex is.
