# ADR-0079 — A convex overhang tip is cut by the next wall

- Status: accepted (2026-10-31)
- Plan: `overhang-tip-cap` step 1
- Amends: ADR-0042 §1, §6 and its Alternatives
- Reference trees read: none; Open CASCADE run as the oracle

## Context

An overhang tip is a cusp whose two edges have opposite senses: its walls
lie on either side of the face they share. ADR-0042 left it refused because
Open CASCADE adds a B-spline cap, presumed to be a new surface. Measuring
that cap on `blend/cusp-overhang-tip-fillet` corrects that premise.

The cap has corners `P` (the shared-face contact crosses the next edge),
`R` (the other contact at that angle) and `Q` (that contact meets the
spine), using ADR-0042's names. It is tangent to the stripe along `P–R`
and to the blended edge's wall along `R–Q`. Its edge `Q–P` lies on the
next wall within 1.5e-9; their crease falls from 81° at `P` to zero at
`Q`. Across radii 0.05, 0.1, 0.3, 0.6 and 0.9, the cap lies within
4.1e-7 of the fillet's torus and 4.2e-7 of the chamfer's cone. These are
fits to the stripe's own surface. They do not fill a missing surface.

Filleting the pillar's concave edge instead is refused by Open CASCADE
(but see the step 3 amendment: on FTC-06 it builds concave requests).
Its refusal is recorded in `blend/cusp-overhang-tip-pillar-fillet`; the
pillar seam lies on its exposed half so the far end is an ordinary corner.

## Decision

ADR-0042's site also includes `e` convex and `n` concave, with its other
conditions unchanged. The next wall cuts the stripe from `P` to `Q` on
its exact surface, with the same trace, fitted edge, node at `Q`, topology
and provenance as ADR-0042 §2–§5.

The cut lies outside the next wall's old face domain at an overhang tip:
that face extends into the region previously buried by the lip. Its side
check therefore uses `Outside` when the cusp edges differ in sense, and
keeps `Inside` for the same-sense cusp. The trim points must still lie on
the next edge and the spine, each within its range. This corrects the
same-sense assumption in `cusp_trims`, without weakening a domain check.

Arris keeps one stripe face. Open CASCADE divides it at `P–R` and fits
one piece to the same surface; this split has no geometric role. A fixture
uses `analytic.counts_differ`, ADR-0015's representation-convention case,
while its measures remain held to Open CASCADE at the default tolerances.
The counts are 18 vertices, 28 edges and 12 faces against Open CASCADE's
19, 30 and 13: removing the cross-section also joins the two pieces of
the wall contact. Euler remains 2.
The earlier claim in ADR-0042's Alternatives that this cut has a different
volume is withdrawn: the oracle's cap is the same cut within its fit error.

A concave blended edge at an overhang tip remains `TangentChain`, naming
`e`, `w` and `v`. A cusp with both edges blended remains refused: its
corner patch is a different surface. No public type or signature changes.

## Consequences

- A convex fillet at the lip-over-pillar overhang builds, with exact
  surfaces and the cut as its one fitted edge.
- The census's walls-on-either-side cause remains meaningful for the
  concave edge; the convex edge no longer produces that refusal.
- The oracle records a final fillet or chamfer reporting not done as no
  solid only in a recipe declaring `analytic.expect_error`. Errors in
  operands and script checkpoints remain errors, rather than becoming
  accepted empty results.

## Alternatives considered

- **Fit another cap surface.** It introduces fit error where the exact
  stripe already has the required surface.
- **Match Open CASCADE's surface split.** It adds a seam between two
  patches of one surface, with no change in geometry or provenance role.
- **Permit the concave edge too.** Open CASCADE refuses the pillar's, and
  no accepted construction justifies the extension from it; FTC-06's
  concave requests are another matter (step 3 amendment).

## Amendment (2026-10-31, step 2)

The cut also builds a convex chamfer at the lip-over-pillar overhang,
transverse at `Q`. Both hands at fillet radii or chamfer distances 0.05,
0.1 and 0.3 match Open CASCADE's volume, area, centroid and inertia at
the default fixture tolerances, with the same count convention above.

## Amendment (2026-10-31, step 3)

The census of FTC-06's 21 requests that were refused at overhang cusps
(`regression/nist-ftc-06-concave-overhang-fillet/census.json`) corrects
the claim that Open CASCADE refuses a concave edge at an overhang tip at
every radius. That holds for the pillar fixture only. On FTC-06, 13
requests are convex and 8 concave. Open CASCADE builds the 8 concave ones
as valid solids, with two fitted caps per tangent chain (degree 8 x 8, up
to 1.14 from any torus, so not pieces of the stripe). Of the 13 convex
ones, it builds 2 and returns an invalid solid for 11.

The decision stands: the concave edge stays `TangentChain`. The fitted cap
is a different surface and has no construction here yet. An experiment
that extended the spine and cut the exact torus passes the checker but
changes the volume by 1.0325 mm^3 against Open CASCADE's 0.9953, so it is
not accepted. The eight requests are the ignored regression fixture, and
the fitted cap a backlog line. The 11 invalid convex cases build
checker-green in Arris with no accepted oracle (backlog: a second oracle's
record).
