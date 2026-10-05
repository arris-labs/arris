# ADR-0048 — Offset faces: a signed distance along the outward normal, the sharp join, topology kept

- Status: accepted (2026-10-05)
- Plan: `offset-faces` step 1 (⚠ OPEN: does Open CASCADE's per-face offset
  give the press-pull answer)
- Idea: `docs/ideas/shell-and-offset.md` (option A)

## Context

C7 opens with `shell` and a face offset (ADR-0047). `shell` is an offset of
every face but the openings, so the offset is built first and on its own: a
consumer's press-pull moves chosen faces of a solid and leaves the rest.

## Decision

**1. The sign.** `distance` is signed along each moved face's outward normal:
positive adds material (a pushed top face makes the body taller, a pushed hole
wall narrows the hole), negative removes it. Zero and non-finite distances are
input refusals. One distance serves every face of a call; a variable distance
per face is the NURBS cycle's (ADR-0047 §1).

**2. The join is sharp.** Two moved faces meet in the intersection of their
offsets, a moved and a fixed face in the moved face's offset met with the fixed
face's own surface (the neighbour extended or trimmed). At a convex edge of an
outward offset the offsets meet at a sharp edge; the round (arc) join is a
backlog line, added when a fixture asks for it.

**3. Topology is kept.** An offset that would make an edge or a face vanish or
reverse, split a vertex, or run the result into itself is refused by name, the
model untouched. The global arrangement (the idea's option B) is raised only if
the real-part corpus asks.

**4. The oracle.** `BRepOffset_MakeOffset` in `BRepOffset_Skin` mode with the
intersection join, a global offset of 0 and `SetOffsetOnFace` for each moved
face, is the press-pull answer: on the corpus's six first fixtures it gives
the closed forms exactly (volume, area, centroid, counts, probes) for a box
face pushed and pulled, the whole box out and in, a cylinder's top pushed and
a through-hole's wall pushed — the whole-body case with the sharp corner of §2.
So no fixture is held to its closed forms instead (no
`analytic.measure_differs`), and no fallback is needed for these. Where a later
fixture finds Open CASCADE wrong or refusing, that fixture is held to its
closed forms with ADR-0015's evidence and the fallback is recorded here. One
fallback is in the driver itself (step 3): where the moved face sits among
concave neighbours — a pocket's floor — Open CASCADE hands back the bare shell
rather than the solid; the driver takes a closed shell as the solid it bounds
(`BRepBuilderAPI_MakeSolid`) and refuses an open one, and that solid gives the
pocket's closed forms exactly, pushed and pulled. The
`offset` recipe op names each face by a point on it (the only face within the
fixture's `probe`), as a `fillet`'s edges are.

**5. Quadric faces (step 4).** A moved face keeps its kind (§ Surface::offset).
Each vertex of a moved face is the point nearest its old one on every surface
around it once moved — Gauss–Newton on signed distances, the first three
independent normals, so three planes meet exactly as before and a vertex on two
slides square to their section; a face that meets itself (a seam) adds the plane
the seam lies in — through the axis for a ruling or a meridian, square to it for
a parallel — so the seam is found as any other edge, the section of the new
surface with that plane. Each edge is the branch of the intersector's section of
its two new surfaces that passes through both new ends, running the old edge's
way, and a degenerate edge at a pole stays at the pole with its pcurve; a
pcurve is placed by whole periods to the old use's `(u, v)` at the edge's
midpoint, which an offset keeps to the period. A cone whose move carries a
vertex to the axis, and a radius driven to zero, are `SurfaceCollapses`; an
elliptic cylinder or a free-form face `NoExactOffset`. An edge between faces
tangent along it, one of which moves, was `Unsupported` until the dragged
chain (§6): the new surfaces there are tangent or apart. Where Open CASCADE
builds an inverted cylinder for a radius driven past zero, or crashes on exactly
zero, the fixture holds Arris to the refusal.

**6. The dragged chain (step 5).** The chosen faces are closed over tangent
edges: a face meeting a moved face tangentially along an edge moves with it,
and so on along the chain, transitively, at the same distance — tangency read
at the edge's midpoint as the blend reads it, for a ball of the offset's
distance. So a filleted edge's blend and the face beyond it are dragged, and a
rounded box moves whole from any one face; a consumer's press-pull selects its
tangent chain the same way. The edge between two tangent moved faces has no
crossing section — the offsets are tangent there — so it is carried: each
point moves by the distance along the normal both faces share, which keeps a
line a line (shifted, where the normal is the same along it) and a circle a
circle (re-radiused and moved along its axis, where the normal keeps one angle
to its plane — every circle of a surface of revolution about its axis); any
other tangent edge is `NoExactOffset` naming it, a circle driven through zero
`Vanishes`. A vertex on a carried edge lies on its curve — the curve's two
surfaces (a line's two planes, a circle's plane and cylinder) lead its
constraints, where its tangent faces alone pin no direction across it. A
dragged fillet moved in past its radius collapses (`SurfaceCollapses`), as a
chosen one does. Where a dragged face no longer meets a face beside it that
stays — a strip of blend, sliced by an oblique flat, pulled clear of the flat
— the offset is `Gap` naming their edge, read where the vertex's faces fail to
meet and those two alone do not, or where no branch of their section passes
through the edge's ends. The oracle's driver closes the named faces over
tangent edges the same way before `SetOffsetOnFace` (normals at each edge's
midpoint within a sine of 1e-6). Open CASCADE's intersection join has nothing
to meet at a gap and kills the process, so the driver tries the offset in a
forked child first and records an offset it refuses or crashes on as the
empty compound — no solid — which only a fixture expecting Arris's refusal
accepts.

## Consequences

- `recipe.py` and `arris_debug::fixtures` carry the `offset` op; the six
  fixtures waited under `regression/` until `offset_faces` landed (step 3), and
  each moves to `offset/` with the step that builds it.
- The open question of a filleted pocket wall pulled from its floor (`Gap` or a
  rebuilt fillet) is answered by §6: the closure is transitive, so the floor,
  tangent to the blend, is dragged too and the pocket deepens with the wall
  moved — neither direction gaps, and nothing is rebuilt. A press-pull that
  keeps the floor and re-blends at the old radius would be the blend's, a
  backlog line if a consumer asks.
