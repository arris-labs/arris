# ADR-0056 — A planar section is the split's positive cap, made a sheet

- Status: accepted (2026-10-11)
- Plan: `planar-section` step 3
- Amends: nothing; builds on ADR-0051 (split: one decomposition, the plane
  as a scratch box), ADR-0004 and ADR-0050 (the pave model and the
  selection table), ADR-0055 (the scratch clone), ADR-0030 (cancellation),
  ADR-0053 (the query fixture kind)
- Idea: the roadmap's C8 line "planar section" (no idea file; decisions
  delegated to the agent)
- Reference trees read: Open CASCADE's `BRepAlgoAPI_Section` and
  `BRepAlgoAPI_Common` of a solid and a face, for what each returns on
  flush and tangent planes (the oracle's behaviour is settled by the
  starter fixtures, step 4).

## Context

A section view, a cross-section for mass properties, a drawing's cut: each
asks what a plane cuts out of a solid, without the solid changing. The
answer a consumer fills and hatches is faces, not curves, and the
decomposition `split` already runs puts exactly those faces on the plane.

## Decision

**1. The definition.** The section of a solid by a plane is the plane's
points *interior* to the solid — material on both sides. It is `split`'s
positive cap, turned to face the frame's `z`, less any region where a face
of the body lies on the plane (material on one side only). Where nothing is
flush the section and the split's cap agree to the last digit, and every
plane-against-body case the booleans settle (through an edge, a vertex, a
seam, an apex) is the section's, with their refusals (`TangentContact`,
`BesideSingularity`, `Unsupported`).

**2. `None`, not an error, for an empty answer.** A plane that misses the
body or only touches it — along a face, an edge or a vertex — has no
interior to cut: `Ok(None)`. `BooleanReason::Empty` and
`InputReason::ZeroThickness` from the common are `None`, as `split` maps
them to `NoCrossing`. A section view placed off the part is a question with
an empty answer.

**3. Flush regions are selected after the Common, not by a fourth `Op`.**
Found at step 2: where a body face and the plane box's near face have
opposite normals (the step seen from above, an overhang from below) both
pieces drop out of the Common; where they agree, the coincident row keeps
the **near face's** copy — `Generated` from the near face, beside the true
cap `Modified` from it. So the section takes, among the Common's faces made
from the near face, those whose interior point (`region2::interior_point`
of the face's `FaceDomain` polygons, as a piece's) classifies `Inside` the
body: `On` is flush and dropped, `Outside` is a kernel fault. The selection
table assembles closed shells; this one runs over the cap alone, after it,
so there is no `Op::Section`. None left is `Ok(None)` (a plane flush with a
top, seen from inside).

**4. A scratch clone, a sheet imported alone.** The decomposition and the
cap are made in `Model::clone` (630 ns measured, ADR-0055 §5), the cap
faces gathered into a sheet by `Builder::assemble_sheet` (one open shell
per face — two regions of a section share no edge, so each is its own
edge-connected component) and `Model::import`ed into the model; the clone
is dropped. The body and every entity of the model are untouched; the
sheet's closure is all the call adds. It is an operation on `&mut Model`
returning provenance, not a member of `ops::query` (whose queries take
`&Model` and make nothing), and not a query returning curves: there would
be no faces to fill and nothing for the checker to hold the result to.

**5. The sheet.** Every face lies on a surface equal to the plane itself,
so its (u, v) are the frame's `x` and `y` and its effective normal the
frame's `z`; its outer loop and holes are the section's wires, one face per
connected region. `area` and `centroid` are integrated over each face's
(u, v) region, exact for lines and conics as `measure` is.

**6. Provenance.** The sheet, its shells and faces are `Generated` from the
body. A section edge is `Generated` from each body face it lies on (two for
an edge along a body edge the plane runs through). A section vertex is
`Generated` from the body vertex the plane passes through, else the body
edges it lies on, else the faces. Nothing of the body is `Modified`. No new
`Role`: the plane is no entity and every output has a body origin. Read
from the scratch run's record through `import`'s `IdMap`.

**7. The query form.** `query` fixtures gain `{"section": {"body", "plane":
{"origin", "z", "x"}}}`, expected `none` or the face count, the loop count,
the edge count, the total edge length, the area and the centroid, held to
Open CASCADE: `BRepAlgoAPI_Section` for the edges and their length,
`BRepAlgoAPI_Common` of the solid and a planar face past the body's box for
area, centroid and faces. Where Open CASCADE counts a flush region or
splits a closed section edge at a seam, `analytic.query_differs` records it
(ADR-0053 §8).

## Consequences

- Step 1's builder change is part of this decision: `Builder::
  assemble_sheet` proves open shells and `finish(…, BodyKind::Sheet)` stores
  them; `Wire` and `General` stay `BuildError::Kind`. No operation returned
  a `Sheet` before the section.
- No new error variant: the refusals are the booleans'.
- A plane touching a box along an edge only faults in `split` as in
  `section` (`Fault::Split` Dangling): `regression/box-edge-touching-split`,
  ignored, a backlog line.
- Sections of sheets, shells and wires (`InputReason::NotSolid`), second
  moments of a section, and many parallel sections in one decomposition are
  backlog lines.
