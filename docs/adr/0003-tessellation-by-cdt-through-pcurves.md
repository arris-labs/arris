# ADR-0003 — Tessellation: an own constrained Delaunay triangulation over `robust`, in (u, v), through the pcurves

- Status: accepted (2026-09-06)
- Plan: `m3-tessellation` steps 1 and 2

## Context

M3 turns a body into a closed triangle mesh (`docs/ROADMAP.md` §M3).
Three decisions were open in the plan: whether to write the constrained
Delaunay triangulation (CDT) or take it from a crate (`spade`, `cdt`),
how a face on a periodic surface — a cylinder wall with its seam, later a
sphere with its poles — is triangulated without a tear or a double
surface, and what number a mesh's volume is accepted against, since the
roadmap's `1e-3` relative at chord `0.01` does not hold for the corpus's
cylinder.

Read in the reference trees. Open CASCADE's `BRepMesh`
(`ModelingAlgorithms/TKMesh/BRepMesh`): `BRepMesh_EdgeDiscret`
discretises every edge once and its polygon on triangulation is what
both faces reuse, so the mesh is closed by construction rather than by
matching points afterwards; `BRepMesh_Delaun` triangulates in the face's
parametric space inside a super-mesh of the bounding box, inserts nodes
by re-meshing the polygon around them and recovers the frontier by
re-meshing the polygon to the left of each boundary link
(`frontierAdjust`, `meshLeftPolygonOf`), then removes what is outside the
frontier; `BRepMesh_Deflection` and `BRepMesh_GeomTool` are the
deflection model — a linear deflection with a minimum point count, iso
curves of the surface discretised to it. `truck-meshalgo`
(`tessellation/triangulation.rs`) is what tessellation without pcurves
does: the boundary is sampled from the 3D edges, a periodic parameter is
unwrapped by picking the nearest period per point (`get_mindiff`,
`normalize_range`), coordinates near zero are snapped
(`spade_round`) and the CDT is `spade`'s — the seam is guessed and the
snap is a tolerance in disguise, and both are where its meshes tear.
Shewchuk, *Adaptive Precision Floating-Point Arithmetic and Fast Robust
Geometric Predicates*, for `orient2d` and `incircle` (the `robust` crate,
`SEED.md` §7); Anglada, *An improved incremental algorithm for
constructing restricted Delaunay triangulations* (1997), for segment
insertion by retriangulating the two pseudo-polygons a segment cuts
through the triangles it crosses.

## Decision

**An own CDT, `arris_mesh::cdt`, over `robust`'s predicates and nothing
else.** `triangulate(polygons, interior)` takes the loops of a face as
`Polygon2` rings in (u, v) — an outer counter-clockwise, holes clockwise,
the seam's two copies a period apart exactly as the loop is stored — and
optional interior points, and returns the triangles of the region where
the polygons' winding number is not zero. Points are inserted in input
order into a bounding triangle by locating the containing triangle
(a walk from the last triangle made, a scan of every triangle if the
walk cycles, which only a constrained triangulation lets it), splitting
it or the edge the point lies on, and Lawson's flips with a strict
`incircle`; each polygon segment is then recovered by walking the
triangles it crosses, removing them and retriangulating the pseudo-polygon
on each side by the triangle on the base whose circumcircle holds no
other vertex; interior points go in last. The exterior and the holes are
told from the region by counting the constraints crossed on a breadth-
first walk from the bounding triangle — one more crossing a constraint
from its right to its left, one less the other way — which is an exact
winding number with no centroid to round. Every decision is a predicate
sign; there is no tolerance, no snap and no perturbation anywhere.
Duplicate consecutive points are merged by `Polygon2`; a polygon of
fewer than three distinct points, a point given twice, two segments that
cross or coincide, and a point on a segment it does not end are typed
`CdtError`s naming the polygons, segments and points — what the checker's
L4 and L5 rows promise a valid face never produces.

**Edges once, faces through their pcurves.** Every edge is discretised
once, at parameters chosen for the 3D chord tolerance and for the (u, v)
travel each of its coedges allows, and its samples are one run of mesh
vertices; a face's loop polygon is the *same* parameters through each
coedge's pcurve, so the (u, v) vertices map back to the shared indices
and a mesh of a solid is closed by construction, never by welding. A seam
edge is one run whose indices the wall's triangles use twice, once from
each copy of the pcurve; a degenerate edge's (u, v) segment maps every
vertex to the one pole index and the triangles that collapse are dropped.
The mesh guarantees are `docs/ARCHITECTURE.md` §Tessellation.

**No adaptive refinement.** Where a face's boundary alone leaves a
triangle whose chord deviation would exceed the bound — a doubly curved
surface, or a wide face on a singly curved one — interior points on a
uniform (u, v) grid at the surface's `chord_steps` spacing are inserted;
the grid is sized by a closed-form bound on the second derivative, never
by measuring a triangle's error and splitting it.

**The triangulation is taken in a domain scaled to the surface.** The
empty-circle criterion measures distance, and the parameters are not
distances: a torus runs `R + r cos v` units along `u` per radian and `r`
along `v`, so a Delaunay in the raw (u, v) stretches triangles across
whichever direction is short — several steps wide in one parameter to
gain a little in the other — and the chord bound, which holds for a
triangle one step wide, does not hold for them. Each face's rings and
interior points are therefore scaled by the mean `|∂P/∂u|` and
`|∂P/∂v|` over its region before `triangulate` sees them, which makes
the domain roughly isometric to the surface; the triangles come back as
indices into the same points, so nothing else in the pipeline knows.
Open CASCADE's `BRepMesh` scales its domain the same way.

**The acceptance bound is the closed form of the inscribed polygon.** A
cylinder of radius `r` meshed at sagitta `δ` is an inscribed prism whose
relative volume error is `θ²/6 ≈ 4δ / (3r)`; the unit tests assert that
bound at any `δ`, and the corpus runner's `mesh_volume_rel` default is
`2e-3` at `mesh_chord = 1e-3`, sized by the corpus's smallest radius.
The roadmap's `1e-3` at `0.01` is corrected at the plan's retirement.

## Consequences

- Determinism is by construction: input order, a bounded walk with a
  deterministic fallback, and strict predicates make the same input the
  same index lists on every platform; the `parallel` feature can
  triangulate faces concurrently and collect them in face order with no
  change to the output.
- A face's picture in (u, v) is drawn in the *unscaled* parameters, the
  ones its pcurves are written in; the scaling lives inside
  `tessellate` and is invisible to every caller and every stored value.
- The (u, v) picture of a face — its loops and its triangulation — is
  the debugger for a face that meshes wrong (`render_domain`, step 4),
  because the CDT never sees 3D.
- Cost is `O(n log n)` in practice and `O(n²)` in the worst case, where
  the walk falls back to a scan; a face with tens of thousands of
  boundary samples is fine, and a benchmark is a backlog line.
- A `CdtError` reaching a caller through `MeshError::Face` names a face
  the checker passed at `Fast`: it is a finding about the checker or a
  kernel bug, never something the tessellator should have repaired.

## Alternatives considered

- **`spade` or the `cdt` crate.** Both are sound, but each carries its
  own predicates and insertion order, so determinism and the `robust`
  decision would rest on them, and a seam or a pole could only be
  handled by preprocessing around their API. The algorithm is small and
  the property tests (1000 random stars with holes and interior points:
  every segment an edge, every triangle counter-clockwise, areas exact,
  every other edge shared by two triangles and locally Delaunay) are the
  proof.
- **Ear clipping** of the loop polygon: no Delaunay quality, slivers on
  every long thin face, and interior points impossible.
- **Bowyer–Watson with centroid classification**, the plan's first
  wording: equivalent in output, but the cavity search is no simpler
  than split-and-flip, and a sliver's centroid rounds onto its own edge,
  so classification by a rounded point is a tolerance by another name.
- **Adaptive refinement** by per-triangle error: better meshes for less
  triangles, and a second algorithm to prove; cycle 2, as a backlog line.
- **Unwrapping the periodic parameter** instead of pcurves, as the
  reference tree without pcurves does: guesses the seam and tears the
  face when it guesses wrong; the representation already knows.

## Amendment (2026-11-17, plan `tangency-faults` step 2)

A face's loops may pass through one point twice: a crescent the boolean
keeps where a section circle touches a cap's rim from inside is one loop
through the touch vertex twice, and Open CASCADE writes the same crescent
as an outer bound and a hole meeting there, which the checker's L4 and L5
now admit. The tessellator gives such a vertex the (u, v) the face's loops
first gave it, bit for bit, where its pcurves agree within the parametric
tolerance (a seam vertex, used a period apart, stays two), and the CDT
takes a polygon point equal to an earlier polygon point as that point: it
is inserted once, its segments join there, and its triangles name the
first occurrence. A point given twice is still `CdtError::Duplicate` for
an interior point; a polygon running along another's segments is
`Coincident`. Still no tolerance inside the CDT: the equality is exact.
