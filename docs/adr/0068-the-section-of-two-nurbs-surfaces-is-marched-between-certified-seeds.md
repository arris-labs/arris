# ADR-0068 — The section of two NURBS surfaces is marched between certified seeds, its loops seeded where they turn

- Status: accepted (2026-10-18)
- Plan: `nurbs-nurbs-section` step 2 (the patch-pair system, the loop
  condition measured and chosen). The curve × surface arm that gives
  the boundary seeds lands in step 4, and the walk in step 5. The
  isolines, poles and flush case come in step 6 and the fit in step 7,
  each an amendment here where it decides something
- Follows: ADR-0061 (a NURBS section traced patch by patch in the NURBS
  surface's (u, v), its seams, isolines, poles and rounding floor),
  ADR-0019 (the turning points a walk starts from)
- Idea: `docs/ideas/nurbs-nurbs.md` (option C: certified seeds and a
  measured walk; Q6: the loop condition measured, the winner recorded
  here)
- Reference trees read: none. Open CASCADE is run as the oracle
  (`GeomAPI_IntSS` at step 7), not read

## Context

Two NURBS surfaces have no implicit form to put one into the other, as
ADR-0061 puts an analytic surface into a NURBS one. Their section is a
curve in four parameters, `(u₁, v₁, u₂, v₂)`. A walk along it needs a
start on every component, and a component it never starts on is a
silent wrong `Ok`. That is the one thing the idea's option A, a plain
marcher, could not rule out.

Components are of two kinds. A branch that reaches an edge or a seam of
either domain is found where that edge line, an exact isocurve, meets
the other surface: the curve × surface arm (step 4), the **boundary
seeds**. A closed loop inside both domains reaches no edge, so a fourth
equation has to put a point on it. That point is a **loop seed**. The
idea left the choice between two conditions to a measurement:

- **Turning**: the section's tangent `N₁ × N₂` along `∂P₁/∂s₁`, which is
  `N₂·∂P₁/∂s₁ = 0`. The section turns in `t₁` there. Every closed loop
  turns in the first surface's `v` at least twice, at its highest and its
  lowest, so this puts a seed on the loop itself.
- **Collinear normals**: `n₁ ∥ n₂ ∥ P₁ − P₂`, four equations. A loop that
  bounds a disc on both surfaces holds such a pair of points inside it
  (Kriezis, Patrikalakis and Prinz, CAD 24, 1992). The pair is off the
  section, and a second isolation would have to carry it onto the
  section. Parallel normals alone are two equations, a surface of
  solutions, and seed nothing. On the section itself the distance's
  critical points are the whole curve, so `n₁ ∥ n₂` and `P₁ − P₂ ∥ n₁`
  have to be written as four separate dot products.

## Decision

**A patch pair's section is three tensor Bernstein polynomials, and its
loop seeds are its turning points in `v₁`.**

1. **The system.** Per pair of rational Bézier patches the section is
   the three components of `w₂·X₁ − w₁·X₂` in `(s₁, t₁, s₂, t₂)`, of
   degree `(p₁, q₁, p₂, q₂)`. Each component is a sum of tensor products
   of a polynomial in one patch's parameters and one in the other's
   (`PolyN::tensor`), so no product is taken in four variables. The
   nets are moved to the middle of the two surfaces' control boxes
   first. Each floor charges the move, at the coordinates' size before
   it, as ADR-0061's does.
2. **The loop condition is the turning one**, `N₂·D₁ = 0`, with
   `D₁ = w₁·X₁,s − w₁,s·X₁` (`w₁²·∂P₁/∂s₁`) and
   `N₂ = w₂·(X₂,s × X₂,t) + w₂,t·(X₂ × X₂,s) + w₂,s·(X₂,t × X₂)`
   (`w₂³` times the normal). A patch whose weights are one constant
   drops the weights' terms. Its floor is first-order: each factor's
   rounding times the other's size. Singular points, where the normals
   are parallel, zero the equation too. Their boxes come back
   uncertified, which is how the walk will know them (step 5).
3. **Pruning.** A closed loop needs parallel normals inside it
   (Sederberg and Meyers, CAGD 5, 1988). So where no patch of one
   surface has a normal cone that meets a cone of the other's, either
   way round, there are no loop seeds to find and no pair is isolated.
   Otherwise a pair is skipped where its control hulls are apart (the
   section needs them to meet). It is also skipped where the cone of
   `D₁` is nowhere square to the cone of `N₂` (the condition needs
   that). A cone is no cone where a coefficient is zero to rounding, at
   a collapsed row, and then nothing is pruned on it.
4. **A pair's zeros are its own**, as ADR-0061's patches' are. A
   certified zero counts within `PATCH_SLACK` of its pair's box, and an
   uncertified one counts where its box reaches in, cut to the box. They
   are merged across knot lines and seams by `bernstein_n::merge`, each
   axis closing where its surface's direction does.

### What step 2 measured, recorded as the decision's grounds

The twins are exact `to_nurbs` twins, posed with every loop inside both
domains, away from seams and poles. The census faces are CTC-02's two
bicubic faces of 16 × 4 spans (`testdata/ctc02-bicubic-faces.txt`), each
against two moved copies of itself. *Crossed* is turned 0.2 about an
oblique tangent, a branch from edge to edge. *Flipped* is turned a
quarter about the normal and over, tipped and lifted 0.45 mm: the saddle
against its flipped copy differs by a definite form, a loop 6 mm across
in the middle. The timings are from the test profile, optimized with debug info,
with one test running.

| Case | Turning: certified (expected), open | boxes, depth | per surface pair | Collinear: certified, open | boxes, depth | per surface pair |
|---|---|---|---|---|---|---|
| sphere × sphere | 2 (2), 0 | 444, 27 | 48 ms | 4, 8 | 78 240, 76 | 9.8 s |
| the same, swapped | 2 (2), 0 | 386, 24 | 44 ms | 4, 8 | 121 434, 118 | 14.7 s |
| crossing cylinders, radii 1 and 0.6 | 4 (4), 0 | 144, 10 | 13 ms | 4, 0 | 1 200, 11 | 0.20 s |
| the same, swapped | 8 (8), 0 | 596, 14 | 50 ms | 4, 0 | 1 120, 11 | 0.20 s |
| spiric torus × plane | 2 (2), 0 | 72, 8 | 3 ms | 4, 0 | 1 017, 12 | 0.10 s |
| CTC-02 f544, crossed | 0 (0), 0 | 96, 0 | 6 ms | 0, 0 | 766, 3 | 0.11 s |
| CTC-02 f544, flipped | 2 (2), 0 | 172, 21 | 11 ms | 1, 0 | 611, 6 | 0.14 s |
| CTC-02 f520, crossed | 0 (0), 0 | 96, 0 | 6 ms | 0, 0 | 766, 3 | 0.12 s |
| CTC-02 f520, flipped | 2 (2), 0 | 172, 21 | 11 ms | 1, 0 | 611, 6 | 0.14 s |

- **Turning gives the closed form's count on every twin**, every seed
  certified and on both surfaces, with the section's tangent along
  `∂P₁/∂u₁` there to 1e-6. Swapping the crossing cylinders doubles the
  count, as the closed form says: on the narrower one each loop turns
  along its axis four times, where `y` is extreme and where `y = 0`. On the census
  faces' loop the two seeds are the lowest and highest `v₁` of the cells
  a sign scan of the copy's side finds the section crossing. The crossed
  copies' branches never turn, and every pair is excluded at depth 0.
- **Collinear costs thirteen to three hundred times as much** and its seeds
  are off the section. On the spheres both poles of each twin zero
  every equation along a row, and eight uncertified boxes come back
  there after up to 121 434 boxes. Its pairs cannot be pruned by hulls,
  since its two points need not be near each other: 547 to 706 census
  pairs isolated against Turning's 24 to 96. On the crossing
  cylinders its four seeds are on the line `x = 0`, nowhere near either
  loop. Those loops wrap the narrower cylinder and bound no disc on it,
  so the theorem says nothing of them. They are found because they
  cross its seam, but that is a boundary seed's work, not the
  condition's.
- **The floor was the finding again.** At first the turning equation's
  floor bounded the normal's rounding by the coordinates' size squared,
  0.076 on the census faces. Near the turning points that is above what
  the equation varies by, so both of f544's seeds came back uncertified
  after 65 halvings. Charged first-order, each factor's rounding times
  the other's size, it is 3.7e-7, and both certify at depth 21. That
  is ADR-0061's lesson, linear in the rounding and not raised to the
  degree, met in four unknowns.
- **The idea's bar is met by three orders of magnitude.** It set loop
  seeds past a second a pair on the census faces as the point to stop
  for the human. They take 11 ms for the whole face pair, 0.47 ms a
  patch pair. The open question it left for this ADR is closed with no
  pruning beyond (3).

The tests hold it: `march_every_twin_loop_carries_its_turning_points`
and `march_the_census_loops_carry_their_turning_points`, each surface
pair within `MEASURED_BOXES` (1 500) and `MEASURED_DEPTH` (40).
`march_the_loop_conditions_compared`, ignored and run by hand, prints
the table above with both conditions.

## Consequences

- The walk (step 5) starts from three kinds of seed: boundary seeds
  (step 4 on every edge line and seam), turning points (here) and the
  uncertified boxes between them, which it treats as singular points.
  A loop has at least two turning points, so a walk from one has to
  reach the other, and every seed it does not reach is
  `SectionFault::UnvisitedSeed`.
- A section along an isocurve of `u₁` turns everywhere. The condition
  vanishes along it and the isolation gives a continuum, which step 6
  decides as ADR-0061 §3 decides an isoline.
- `bernstein_n` gains `PolyN::new` and `PolyN::tensor`. `nurbs::Patch`
  is crate-visible. No public type or signature changes.
- The comparison stays reproducible. The collinear condition lives in
  `march`'s tests beside the survey, not in the kernel.

## Amendment, step 5: the walk, its singular points and its box chain

- **Boundary seeds** are every edge line and seam of both domains
  against the other surface, by the curve × surface arm. A seed on an
  open edge ends a branch; one on a seam is passed, as a turning point
  is. A row collapsed to a pole is not looked at (step 6 decides poles),
  and an isoline on the other surface is refused as `TubeCircle` until
  step 6 returns it. The section touching an open edge is ADR-0061's
  refusal, `UnresolvedTurning`.
- **Singular points** come from the loop condition's uncertified boxes.
  From each, a Gauss–Newton descent on `P₁ − P₂` and `n₂·∂P₁/∂u₁`,
  `n₂·∂P₁/∂v₁` looks for the surfaces meeting tangent. A point within
  `tol.linear` of both and `tol.angular` of tangent is a singular point.
  Otherwise the box is looked at as a turning point the isolation could
  not certify, and one that is neither is `CrowdedSingularity`. That
  includes a turning point of higher order: a cylinder of radius 0.6
  0.6 off the axis of one of radius 1, the narrower walked first, turns
  in its `v` at (−1, 0, 0) as `(u − π/2)⁴`, and the loop condition's
  gradient vanishes there.
- **The reach** of a singular point is one step of the walk at the
  larger principal curvature of either surface there, `√(8·sag/κ)`. Its
  **departures**, the section's points at the reach, are the zeros of
  the section with `|X₁ − w₁·S|² − ρ²w₁²`, isolated over the patch pairs
  whose hulls meet its ball and certified, or the point is
  `CrowdedSingularity`. A branch ends at the singular point through a
  departure, the stretch inside the reach the chord to it. None is an
  isolated point. Turning points inside a reach are the point's own and
  are dropped. A boundary seed inside one, or two reaches that meet, are
  `CrowdedSingularity`.
- **The step** is the chord whose sag at the section's curvature is
  `SECTION_FIT_FRACTION · tol.linear`. The curvature is exact at the
  step's start, the vector in the normals' plane whose components along
  them are the two second fundamental forms along the tangent. The
  longest step is 1/64 of the diagonal of both nets' box. The corrector
  holds both surface points within 1/1024 of `tol.linear` of each other.
- **The box chain**, as built: a step is accepted only where the
  corrector converges, its point is within a quarter step of the
  prediction, the tangent turns by at most four times what the
  curvature at either end allows, and the chord's middle, corrected onto
  the section, is within four sags of it. A step failing any of them is
  halved, forty times at the most. A seed within four sags of a chord,
  past its start and inside the step's parameter box grown by its
  largest move, is where the step ends, exactly at the seed. That
  growth is needed: a turning point lies past the chord's range in the
  parameter it turns in, by the sag. The walks start from the open
  edges' seeds, then the departures, then the passed seeds still not
  reached, each of which must come back to itself. A walk that reaches
  a passed seed a second time, leaves a domain or enters a reach away
  from a seed, or stalls, and any seed left unreached, is
  `SectionFault::UnvisitedSeed`.
- **Measured** on the twins of plane × plane, cylinders crossing, skew
  and tangent inside and outside, and Viviani's sphere × cylinder, both
  orders each. Every case gives the closed form's branches, ends and
  singular points. Each walked point is on both twins and both analytic
  surfaces within 1/1024 of `tol.linear`, and a surface pair takes 2 ms
  to 0.42 s in the test profile, most of it the 11 000 steps a loop of
  radius 1 takes at the default tolerance
  (`march_every_twin_section_has_the_closed_forms_branches`).
