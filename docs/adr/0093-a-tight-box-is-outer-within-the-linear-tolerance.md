# ADR-0093 — A tight box is outer within the linear tolerance

- Status: accepted (2026-11-22)
- Follows: ADR-0002, ADR-0030
- Reference trees read: none.

## Context

A consumer needs a part's extent even when a trimmed face's parameter
rectangle or a B-spline control hull extends beyond the part. Those outer
boxes are inexpensive and useful for broad-phase rejection. The tolerance
envelope is also useful when deciding whether two entities can meet.

## Decision

1. **Expose a separate query.** `tight_bounds` returns the world-axis-aligned
   tight box of a vertex, edge, face, shell or body. Its sides lie within
   the model's linear tolerance of geometry the entity reaches, to rounding.
   The query uses the stored geometry directly. `bounds` supplies the
   tolerance envelope for broad-phase rejection.
2. **Include the closure's boundary geometry.** Vertex points and analytic
   edge ranges give their exact extrema. A NURBS edge keeps the clipped
   Bernstein hulls that cover its range, subdividing where they extend
   past the current box of evaluated points by more than half the linear
   tolerance. Each face also bounds its pcurves evaluated on its surface.
   The checker permits these lifted trims to differ from the shared edge
   curves within the edges' tolerances. Planes and isolines admit exact
   lifts. For other trims, bound the pcurve's distance from its chord and
   lift the chord with bounds on the surface derivatives. A rational
   pcurve's Cartesian controls bound its deviation, and the quotient
   rule and Bernstein control hulls bound a NURBS surface's derivatives.
   On smooth pieces the lift's error shrinks with the square of the
   chord's width. Across a crease the one-sided speeds bound the lift.
3. **Include interior extrema of trimmed faces.** Each coordinate of a
   plane, cylinder, elliptic cylinder or cone is affine along each ruling.
   The endpoints of a ruling inside the trim reach the lifted boundary,
   so their coordinates bound the intervening segment, including a
   singular endpoint on a cone apex. A sphere's six axis poles and a torus's
   stationary points count when the face domain classifies them inside
   or on the trim. A crest circle on a torus has constant height along
   its axis. If only part of it survives the trim, the trim meets it
   at the same height.
4. **Subdivide NURBS faces where their interiors can enlarge the box.**
   Split at a closed, clamped seam and clip each piece in the fundamental
   knot domain. Test the trim in its own translate. Discard patches proved
   outside the loops. For a coordinate strictly monotone along either
   parameter, the boundary supplies its extrema except where extrema can
   occur at a C0 knot or closed, clamped seam. Test the sign of the weighted
   control-point differences at both ends of the coordinate's hull range to
   prove monotonicity. Keep remaining hulls whose open sides
   are within a quarter of the linear tolerance of the box already found
   or of evaluated face points. Lifted trims also have a quarter of the
   tolerance. The edge, trim and interior allowances add to at most one
   linear tolerance. When a midpoint rounds to an endpoint, subdivide along the
   other direction. When subdivision stops, keep an open hull side only
   if the final witness box has reached it within the allowance.
   Otherwise return `BoundsUncertified` naming the face.

## Consequences

The query returns `None` for an unbounded face or when a finite box cannot
be obtained. A caller can interrupt the query through `Control`. Its meter
counts each entity in the closure and each edge, trim or face split. Booleans
and `interference_many` continue to reject candidates with their inexpensive
outer boxes.
