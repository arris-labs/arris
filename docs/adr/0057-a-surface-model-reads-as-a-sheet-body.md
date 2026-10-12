# ADR-0057 — A surface model reads as a sheet body

- Status: accepted (2026-10-12)
- Plan: `step-unit-and-sheet-bodies` step 3
- Amends: ADR-0025 §2 (the list of shapes the reader refuses: the two
  surface models leave it; a faceted B-rep and the tessellated entities
  stay) and ADR-0026's "Construction geometry counts as itself" (a surface
  model only a `CONSTRUCTIVE_GEOMETRY_REPRESENTATION` holds is read like
  any other); ADR-0025 and ADR-0026 are not edited
- Idea: the roadmap's C9 "In" list (no idea file; decisions delegated to
  the agent, the plan's open questions 1 to 3 decided here)
- Reference trees read: Open CASCADE's `StepToTopoDS_Builder` for what a
  `SHELL_BASED_SURFACE_MODEL` and a `FACE_BASED_SURFACE_MODEL` transfer to
  (a shell, a compound of faces) and `BRepGProp::SurfaceProperties` for
  what a surface property is; the oracle settles the rest.

## Context

Six of the fetched tier's parts are held back at `read` by nothing but
surface models: one-face `OPEN_SHELL`s that CATIA writes for its `Fill`,
`Extract` and `Split` surfaces, beside the solid they were made for. Arris
has had the body kind since C8 (`BodyKind::Sheet`, built by
`Builder::assemble_sheet`); the reader was the one input that could not
make one.

## Decision

**1. A surface model reads into a `Sheet` through the solid's own path.**
`SHELL_BASED_SURFACE_MODEL` (its `sbsm_boundary`: `OPEN_SHELL`s,
`CLOSED_SHELL`s, and their `ORIENTED_` forms) and `FACE_BASED_SURFACE_MODEL`
(its `fbsm_faces`: `CONNECTED_FACE_SET`s, which an `OPEN_SHELL` and a
`CLOSED_SHELL` are) take the faces, bounds, edges and pcurves of a
`MANIFOLD_SOLID_BREP`, one Arris shell per file shell, and are assembled by
`Builder::assemble_sheet` and stored as `BodyKind::Sheet`. Nothing about a
face is read differently: its normal is the file face's turned by an
`ORIENTED_` shell or face, which for a sheet is its front. Provenance is
`Role::File` at the shell, face, edge and vertex, and the body at the
surface model. The refusals are the solid's (`Pcurve`, `OpenLoop`, `Gap`,
`Degenerate`, `Topology`, `Invalid`). An open shell whose faces disagree on
orientation is `Topology` — the builder's `SameDirection` — until the
healing cycle's orientation plan repairs it; so is a shell in pieces.

**2. The sheet lives in `Read::solids`.** The plan's open question 1: the
list keeps its type and its one index space, so `ProductTree`'s indices
(`Occurrence::solids`, `FaceColour::solid`) name sheets and solids alike,
and no consumer's occurrence walk changes shape. What changes is the
guarantee — `ReadBody::body` is a `Solid` *or*, for a surface model, a
`Sheet` — which is `### Breaking` in `CHANGELOG.md` with its one-line fix: a
caller that wants solids only filters on `BodyKind`. A separate
`Read::sheets` would have been a public type change and a second index
space in every occurrence.

**3. Construction geometry is read.** The plan's open question 2: a surface
model that only a `CONSTRUCTIVE_GEOMETRY_REPRESENTATION` holds is read as a
sheet like any other. The product tree already tells a body of the part
from a construction surface (no occurrence of the part lists it), and a
refusal hid geometry the file holds. The reader no longer tells the two
apart, so `Refusal::Unsupported`'s `in a CONSTRUCTIVE_GEOMETRY_REPRESENTATION`
suffix is gone; the histogram's arm for it stays, dead, until the survey
step reworks the table.

**4. The oracle's sheets are matched by face count and centroid.** Open
CASCADE keeps no entity id behind a shape (`oracle/step.py`'s `solids`
finds a solid's by content), and a one-face shell has too little content
for that. `expected.py` records the shells of the healed reading that no
solid holds, as `sheet` entries of a part's `solids` with `id` 0, their
counts, area, centroid and the inertia of `BRepGProp::SurfaceProperties`
summed per face; the part runner matches each Arris sheet to the nearest
untaken one of its face count, and a sheet of the oracle's that no Arris
sheet matches fails the run as a solid does. A sheet has no volume, no
Euler line the oracle holds, and no battery: the battery's operations take
solids.

**5. A surface property is unit area density.** The plan's open question 3:
`measure::surface_properties` carries the inertia. The area, the centroid
and the tensor about it are integrals of `1`, `P` and `P Pᵀ` against the
area element `|∂P/∂u × ∂P/∂v|` over each face's region in its own (u, v),
by the quadrature `mass_properties` takes its area by, over the faces of
any body: a solid's boundary or a sheet's faces. `mass_properties` still
refuses a sheet with `InputReason::NotSolid`.

**6. The writer spells a sheet as a surface model.** (Step 6 implements
it.) Each shell of a sheet body is an `OPEN_SHELL` of `ADVANCED_FACE`s —
a `CLOSED_SHELL` where the shell is closed — in a
`SHELL_BASED_SURFACE_MODEL`, listed in a `MANIFOLD_SURFACE_SHAPE_REPRESENTATION`
over the same contexts, and tied to the product's `SHAPE_DEFINITION_REPRESENTATION`
beside the solids' `ADVANCED_BREP_SHAPE_REPRESENTATION` by a
`SHAPE_REPRESENTATION_RELATIONSHIP`.

## Consequences

- Five of the six held-back parts leave the `read` column's surface-model
  row; the five still refused there are all `gap past the cap`, the second
  C9 plan's.
- A consumer that took `Read::solids` for solids must filter; the facade's
  Python `Body.kind` says `"sheet"`.
- The sheet fixtures under `tests/fixtures/real/` are part fixtures: the
  runner holds a sheet to the checker at `Full`, to the oracle's face
  count, area, centroid and inertia and to its dump. Open CASCADE's healed
  face of STC-07's `Fill.1` is 1.6e-7 off the triangle of its three
  vertices, which Arris matches to 2e-12; its fixture widens the area and
  inertia tolerances to 5e-7 and says why.
