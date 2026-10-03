# ADR-0042 — A blend ending at a cusp, with both walls on one side of the shared face, is cut by the next wall

- Status: accepted (2026-10-03)
- Plan: `cusp-run-out` step 2 (the decision, against the oracle), step 3
  (the build)
- Amends: ADR-0035 §6 (what `TangentChain` keeps), ADR-0037 §6 (a singular
  point that ends the stretch is not one between the trim points)
- Follows: ADR-0037 §2–§5 (the trace, the fit, the pcurves, the checker),
  ADR-0040 (tangent within the blend's tolerance)

## Context

The census of `cusp-run-out` step 1 found that all 21 of NIST FTC-06's
`TangentChain` chain ends are cusps. At each one the blended edge `e` and
the next edge `n` leave a vertex `v` of three edges the same way, and the
two walls are tangent along the third edge `w`, the spine. ADR-0035 §1
does not walk on there, and it is right not to: the outline doubles back,
so there is no stripe beyond `v` to join.

Step 2 asked Open CASCADE what it builds there and found two cusps.

- **Both walls on one side of the shared face `T`**: the crescent prism
  (`regression/cusp-crescent-fillet`, material in the sliver, both edges
  convex) and its pocket twin (the sliver void, both edges concave). Open
  CASCADE cuts the stripe with the next wall `W_n`: 6 faces, 12 edges and
  8 vertices for a fillet or a chamfer of either arc, top or floor, at
  every radius from 0.02 to 0.8, and every result valid. The removed or
  added volume matches the closed form within 5·10⁻⁷: the corner section
  integrated over the part of its band that lies in the gap, 1.9 % to
  11.8 % short of the full-length removal.
- **The walls on either side of `T`**, one edge convex and the other
  concave: the tip of an overhang. These are FTC-06's cusps. `T` is the
  underside of a horn, a pillar's wall runs below it and a hole's wall
  above it. Open CASCADE closes every such stripe it builds (10 of 21)
  with B-spline cap surfaces, one or two per blend. The other 11 are
  invalid at every radius from 0.1 to 1.5: the pillar's face loses about
  13 139 mm³, and mirror images disagree. A recipe twin of the overhang
  does the same.

Blending both `e` and `n` at a crescent's cusp also gets a B-spline
corner patch from Open CASCADE.

Read in the reference trees: Open CASCADE trims a stripe on a face it runs
into in `ChFi3d_Builder::PerformIntersectionAtEnd`, and fills what it
cannot trim with a constrained filling (`GeomFill`). The first is the
construction below. The second is a fitted surface. Read for the
structure; nothing taken.

## Decision

**1. The site.** The rule applies at a vertex `v` of exactly three edges
`e`, `n` and `w` where all of the following hold:

- `e` is blended, and `n` is not blended.
- `e` and `n` share exactly one face `T`, and `w` joins `e`'s other face
  `W_e` to `n`'s other face `W_n`.
- `W_e` and `W_n` are tangent at `v` (ADR-0040's reading).
- `n` leaves `v` against the direction in which `e` arrives (the outline
  doubles back).
- `e` and `n` are of one sense, both convex or both concave, so both walls
  lie on the stripe's side of `T`.

There the stripe is **cut by `W_n`**. Its end at `v` is not a cross-section
or a face across. It is a lengthwise cut, from the point where the
stripe's contact on `T` leaves `T` to the point where its contact on
`W_e` reaches the spine.

**2. The two trim points are closed forms.**

- `P` is where the stripe's contact on `T` crosses `n`. In `T`'s plane
  that is a circle or line against a circle or line.
- `Q` is where the stripe's contact on `W_e` meets `w`. On a cylinder or
  plane wall that is a circle or line against the spine's line.

`P` must lie inside `n` and `Q` inside `w`. If it does not, the stripe is
too wide for the gap, and the end is `BlendTooLarge` naming `e` and the
edge it leaves (`n` or `w`), as for a contact that leaves its face.

**3. The cut is the stripe's section with `W_n`, as ADR-0037 takes an
end.** It is the intersector's closed form where one exists. Otherwise it
is traced (`trace_section`), the stretch from `P` to `Q` is fitted
(`fit_branch`), and the pcurves are fitted from the 3D curve (`pcurve_on`),
under ADR-0037 §2–§5 unchanged. For the stripes the table builds against a
cylinder or plane wall parallel to the spine, that means the following.

| Stripe | Against `W_n` | Section |
|---|---|---|
| A torus | A cylinder on a parallel axis | A quartic, a graph over the wall's base circle |
| A torus | A plane | A spiric section |
| A ruling stripe's cylinder | A cylinder across | ADR-0037's quartic |
| A chamfer's plane against a cylinder | | An ellipse, closed form |

**4. At `Q` the fillet's section has a node, and the stretch ends there.**
The stripe is tangent to `W_e` along its contact, and `W_e` is tangent to
`W_n` along `w`. So at `Q` the stripe is tangent to `W_n`, and two
branches of the section cross there. The tracers already end a branch
exactly at a point where the two surfaces come within `tol.linear` of
tangency (ADR-0019's `SectionPoint`). The stretch is the one from `P`
that ends at `Q`, chosen as in ADR-0037 §2, with its midpoint in the
stripe's band. **ADR-0037 §6 is amended:** a singular point *at* a trim
point ends the stretch and is not refused. Only a singular point *between*
the trim points is refused. A chamfer's cut is transverse at `Q`, so this
does not arise there.

**5. Topology and provenance.**

- `v` is deleted. `P` and `Q` are generated from `e`.
- `n` is shortened to end at `P`, and `w` to end at `Q`. Both are
  modified.
- The stripe's face is bounded by its contact on `T` (up to `P`), the cut
  (`P` to `Q`), its contact on `W_e` (from `Q`) and its far end. The cut
  is generated from `e`, as the stripe's other edges are.
- `T`, `W_e` and `W_n` are modified. `T` loses its sliver past the contact
  and `W_n` gains the cut as an edge.

On the crescent: 6 vertices, 9 edges and 5 faces lose `v` and `e`, and gain
`P`, `Q`, the far end's two vertices in place of one, two contacts, the far
end's arc, the cut and one face. That gives 8, 12 and 6, Open CASCADE's
counts.

**6. What is refused, by name.** Each of these stays `TangentChain`,
naming `e`, `w` and `v`, as ADR-0035 §6 has it:

- A cusp whose edges are of opposite sense, the overhang tip. Open
  CASCADE caps it with a fitted surface, which ADR-0007 and ADR-0037 §1
  rule out until the NURBS cycle.
- A cusp where `n` is blended too. That takes a corner patch.
- Any other vertex where a corner edge's faces are tangent but the vertex
  is neither a tangent vertex nor this site.

A trace that does not decide the cut, or a stretch the tracer does not end
at `Q`, is ADR-0037 §6's `Unsupported`, naming the stripe's surface kind
and `W_n`'s. `Reason` gains no variant, so the binding does not change.

## Consequences

- An edge whose blend ends at a cusp with both walls on one side builds at
  Open CASCADE's counts, with exact surfaces and one fitted edge, the cut.
- FTC-06 stays in the battery's `fillet` column. Its cusps are overhang
  tips, and its `TangentChain` is now a named site that the NURBS cycle
  has to take. The census tells the two apart: "a cusp, walls on one
  side" builds, and "a cusp, walls on either side" refuses.
- The cut adds no tracer and no checker row: S5 decides the stripe
  against `W_n` with the tracer that built the cut (ADR-0037 §5).
- The volume of a cut stripe has a closed form for the property test: the
  corner section's area function integrated over its band inside the gap.

## Alternatives considered

- **Tracing the cut as the explicit graph over `W_n`'s base curve**, the
  height read off the stripe's cross-section at each point's distance from
  its axis. Exact pointwise, and it has no node at `Q`. But it is a third
  tracer, used for one family only, and the checker would decide its
  result with a different algorithm than the one that built it. ADR-0037
  rejected a march for the same reason.
- **Capping the overhang tip as Open CASCADE does**, with a fitted filling
  surface. It would take FTC-06 out of the column, but it reverses ADR-0007
  and ADR-0037 §1 for one part, and Open CASCADE's own caps fail on half
  of that part's cusps. The human chose to land the cut and leave the cap
  to the NURBS cycle (`cusp-run-out` step 2's gate).
- **Cutting the overhang tip with `W_n` as well.** It is the same curve
  and needs no new surface, but it would not be Open CASCADE's answer: the
  counts lack the cap, and the volume differs. Acceptance is the oracle's
  counts, so this would be a measured disagreement with no evidence that
  the oracle is wrong (ADR-0015).
