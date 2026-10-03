# ADR-0044 — A miter of unequal dihedrals is two pieces and a trim arc

- Status: accepted (2026-10-04)
- Plan: `blend-miters` step 1 (asked of Open CASCADE), step 2 (this
  decision), step 3 (the fillet built), step 4 (the chamfer built)
- Amends: ADR-0007 §"Each end is trimmed by the face across the corner"
  (its miter has one ellipse and no arc entering any face)

## Context

ADR-0007's miter is the corner of two blended edges at a vertex whose third
edge stays sharp, for two fillets of **equal dihedrals** (or two chamfers of
equal angles with the third edge): the stripes' far contacts then cross the
third edge at one point, the curve between them is the bisecting ellipse (a
line for chamfers), and the third edge is cut where the contacts meet. When
the dihedrals differ the far contacts meet the third edge at two points,
and the corner was refused `VertexBlend`
(`a_miter_of_unequal_dihedrals_is_a_vertex_blend`).

`blend-miters` step 1 asked Open CASCADE for the smallest case: a prism over
the parallelogram (0,0) (2,0) (3,2) (1,2), 2 high, its slanted vertical
edge at (3, 2) and the cap edge at (2, 2, 2) blended in one call at 0.2
(`regression/miter-unequal-dihedrals-{fillet,chamfer}`). Open CASCADE 8.0.1
builds both, valid, at **12 vertices, 18 edges, 8 faces and 8 loops** (the
equal miter's prism has 11, 17, 8, 8), and a concave pocket version too (20,
30, 13). Read from its result: the two blends meet along the usual curve
(the ellipse of the two cylinders; the chamfers' line) from `q`, where the
contacts cross on the shared face, to `m`, a point on one blend's far
contact. From `m` one **trim arc** (a circle, a line) runs to the other
blend's far contact on the third edge, which ends there. Read in the
reference trees: `ChFi3d_Builder`'s corner of two stripes, which ends the
longer one on the other's face; nothing taken.

## Decision

**1. The site.** The corner of `miter` today (two blends of one convexity,
no ruling blend, a third sharp edge between the faces the blends do not
share), where the far contacts meet the third edge at two points further
apart than the blends' tolerance. Mixed convexity stays `VertexBlend`.

**2. The wider blend.** Let `pa` and `pb` be where the two far contacts
meet the third edge. The **wider** blend is the one whose point is the
farther from the vertex; the other is the **narrower**. For fillets it is
the one with the larger contact distance `r / tan(β/2)`, the acute edge's;
for chamfers of one distance it is the one making the smaller angle with the
third edge. In the slanted prism the vertical edge's fillet is the wider
(contacts at 0.324 against 0.2) and the cap edge's chamfer is (its plane
reaches y = 1.8 on the third edge, the vertical's y = 1.82). The wider one
is therefore not a property of the edge but of the pair, and the code finds
it by distance and never by kind.

**3. The corner curve is two pieces.**

- The **miter curve** runs from `q` to `m`, where it meets the narrower
  blend's far contact. It is ADR-0007's curve, cut short.
- The **trim arc** runs from `m` to the wider blend's far contact on the
  third edge, and is the section of the wider blend's surface with the
  narrower blend's **far face** (its face that the other blend does not
  share): a circle or an ellipse where that face is a plane and the blend a
  cylinder, a line where both are planes, so every curve is a conic or a
  line. It is exact on the plane and fitted on the cylinder by the
  oblique-section rule (ADR-0007), at its own tolerance.

**4. Topology.** The far face takes the trim arc in its loop, the third edge
is shortened to the arc's end (`Modified`), and the corner vertex goes. The
count is one vertex and one edge over the equal miter: 12 / 18 / 8 / 8.

**5. Provenance.** The trim arc is `Generated` from the wider blend's edge.
`m` is `Generated` from both edges. The third edge and the far face are
`Modified`. The rest is ADR-0007's.

**6. What is refused, by name.** A trim arc that leaves its face, or a third
edge shorter than the cut, is `BlendTooLarge`. Mixed convexity and the
ruling blend stay `VertexBlend`; no wildcard arm and no fallback.

## Consequences

- The corners line of C6 holds only the ruling miter, which goes to the
  backlog.
- `miter` and `MiterMade` carry the optional trim arc (internal); no public
  type or signature changes and `Reason` gains no variant.
- Equal dihedrals are the case where the trim arc is empty (`m` is `pa`),
  so ADR-0007's miter is this decision's limit and its dumps stay unchanged.

## Alternatives considered

- **A sphere or ball corner for the pair.** Rejected: Open CASCADE builds
  none, and the third edge stays sharp, which a ball would not leave.
- **A fitted patch over the corner.** Rejected for the same reason as
  ADR-0007's: every curve here is a conic or a line, so nothing needs a
  patch.
