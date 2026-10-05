# ADR-0049 — Shell: the offset of every face but the openings, a rim face per opening, a void as a second shell

- Status: accepted (2026-10-05)
- Plan: `shell` step 1 (⚠ OPEN: does Open CASCADE's thick solid give the
  closed void and the adjacent-openings case)
- Idea: `docs/ideas/shell-and-offset.md` (option A), through ADR-0048

## Context

C7's second operation hollows a solid to a wall of constant thickness
(ADR-0047). `offset_faces` (ADR-0048) already moves chosen faces along their
normals with the sharp join and refuses what would change topology; a shell
is that offset applied to every face but the openings, kept beside the
original faces instead of replacing them.

## Decision

**1. The two sides, and no sign.** `thickness` is positive; the side is a
separate argument. `ShellSide::Inward` keeps the body's faces as the outside
and grows the cavity inside them: the inner skin is `offset_faces` of every
face but the openings by `-thickness`, reversed. `ShellSide::Outward` makes
the body's own faces the cavity, reversed, and grows the skin outside them by
`+thickness`. A signed thickness would make the side a convention a caller
has to remember; a named side cannot be misread.

**2. The skin is the offset's.** Each wall face of the skin lies on the offset
of its own surface, the same kind, joined sharp at every edge, the tangent
chain dragged (ADR-0048 §2, §5, §6). An opening is a fixed face of that offset:
the moved walls beside it meet its own surface, so an outward skin's walls run
up to the opening's plane and an inward cavity's walls end in it. Every refusal
the offset raises — `Vanishes`, `VertexSplits`, `NoExactOffset`,
`SurfaceCollapses`, `Gap`, `SelfIntersects` — reaches the caller as the
offset's reason naming the entity, not a second copy under the shell's group.

**3. The rim face.** Each opening becomes one face on the opening's own
surface bounded by the body's rim (its old loop) and the skin's rim (the loop
the moved walls cut on that surface): two loops, an annulus. Where two
openings share an edge, the two rims touch along it, so each opening's face is
one loop running out along the shared edge and back; the edge's middle, across
the mouth, is in no face of the result, and its ends beyond the skin stay as
edges between the two rim faces.

**4. No openings: a void.** With no opening the skin closes on itself: the
result is one body of two shells, the outer face for face the body (inward)
or its offset (outward), the inner reversed and nested inside it
(`ShellNesting`). The second shell is `Generated` from the body.

**5. The shell's own refusals.** `Reason::Shell` holds what only the shell can
get wrong: `NoWalls` (every face an opening), `RepeatedOpening`,
`OpeningNotInBody`, and `OpeningDragged` — an opening tangent to a wall, which
the wall's move would drag (an opening is fixed, a dragged face moves, and the
two cannot both hold). A non-finite or non-positive thickness is
`InputReason::NonFinite` / `NotPositive`.

**6. `Full` in every profile.** As `offset_faces` does (ADR-0048 §7): a skin
that runs into the outer faces — a thickness past the thinnest wall — is
well formed edge by edge and crosses itself only globally, so the result is
checked at `Level::Full` in every build and a global-only report is
`OffsetReason::SelfIntersects`.

**7. The oracle.** `BRepOffsetAPI_MakeThickSolid::MakeThickSolidByJoin` with
the intersection join, the opening faces as its closing faces and the
thickness negated for `Inward`, gives the closed forms exactly — volume, area,
centroid, counts, probes — on step 1's opened fixtures and runs: a box open on
top, inward and outward; open on top and front (sharing an edge), inward and
outward, each rim face one loop as §3 says; open on three faces meeting at a
vertex (run here, step 3's fixture); a square tube; an L-bracket. With no
closing face it does not build a void: it answers with the offset solid alone
(a 6-cube for a 10-cube hollowed in by 2, a reversed 14-cube out). So the
driver builds the void as the body less its inward offset, or its outward
offset less the body (`BRepOffsetAPI_MakeOffsetShape` by join and a cut),
which gives the closed forms of both void fixtures exactly, with two shells.
No fixture is held to its closed forms instead (no
`analytic.measure_differs`). Either construction is tried in a forked child
first, as the offset's is (ADR-0048 §6), and one Open CASCADE refuses or
crashes on is the empty compound, which only a fixture expecting Arris's
refusal accepts.

## Consequences

- `recipe.py` and `arris_debug::fixtures` carry the `shell` op; the step-1
  fixtures wait under `regression/` until `shell` lands (step 2, the adjacent
  openings at step 3), each moving to `shell/` with the step that builds it.
- The round join at a concave edge of an inward shell (a convex one of an
  outward) stays with the offset's backlog line: the shell makes what the
  offset makes.
