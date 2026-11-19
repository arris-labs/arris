# ADR-0092 — A revolved torus face closes at a pole

- Status: accepted (2026-11-19)
- Plan: `revolve-circle-meets-the-axis` step 2
- Amends: ADR-0080 §3 and §5
- Follows: ADR-0002, ADR-0021 §2
- Reference trees read: none. Open CASCADE was run as the fixture oracle.

## Context

An arc can stay on one side of the revolve axis even when its circle
reaches or crosses the axis. Its sweep lies on the regular band of a horn
or spindle torus and can close at the axis if an arc endpoint lies there.
At that endpoint every value of `u` gives the same point, as at a sphere's
pole. The spindle fixtures include a circle whose centre is 1.3e-4 from
the axis. Its sweep retains that major radius.

## Decision

1. **A torus face reaches the axis only at a torus pole.** A degenerate
   edge joins the pole vertex to itself. Its pcurve is a line of constant
   `v = v*`, where `R + r cos v* = 0`. For a spindle, the roots are
   `±acos(−R/r)` modulo a turn. For a horn, the root is `π` modulo a turn.
   The vertex lies at the point where the surface collapses at that root.
2. **F1 gives the pole its place on the boundary of the face's `v` hull.**
   It includes the degenerate edge's `v*` in the hull. An incident
   non-degenerate pcurve may approach that value on the outer sheet and
   end on the pole vertex. The regular-band margin applies to extents
   apart from such terminal approaches, using the face's tolerance as a
   distance from the axis (ADR-0080 §2). A pcurve touching the root in
   its interior, an endpoint there without the degenerate edge on its
   vertex, or a hull that crosses a root still gives
   `FaceFault::OffTorusBand`.
3. **Revolve uses the torus of the arc's circle when `R > tol`.** An arc
   clear of the axis sweeps a regular-band face. An arc ending on the axis
   closes with a degenerate edge at the torus pole. An arc crossing the
   axis remains `ProfileCrossesAxis`. An arc tangent to the axis in its
   interior is refused by segment. A full profile circle tangent to the
   axis is cut at the touch point, so a partial turn can close its side
   there and a full turn gives `NonManifold`.

## Consequences

The torus pole uses the same topological representation as a sphere's
pole. The outer sheet and its poles determine the supported trim. The
lemon sheet remains outside that domain. Blend contacts at the axis,
offsets at a torus pole and section branches through a pole in a plane
holding the axis retain their existing restrictions.

## F1's coedge walk

The hull alone cannot distinguish a pcurve ending at a pole from one
reaching the same angle in its interior. F1 reads the two neighbours of
each incident coedge to check the shared pole vertex and degenerate
pcurve. It checks the incident pcurve's interior turns against the regular
band margin, then admits the pole as a bound of the face's unwrapped hull.
A ray hit at a torus pole carries `u = 0` and the singular `v`, as at a
cone apex, so classification can test the face's trim without asking the
ordinary surface projection for an azimuth at a collapsed point.

## Tessellation

As at a sphere's pole (ADR-0003), the degenerate edge's samples share
the pole's mesh index, and a triangle collapsed onto that index is
dropped. The surviving fan has normals at each corner's own `u`. On
the outer sheet, `∂P/∂u × ∂P/∂v` is a positive multiple of
`cos(v) radial(u) + sin(v) Z`, so its normalised limit at the pole is
that vector at `v*`, flipped for a reversed face use. A spindle's limit
has both components. A horn's radial scale has a double root and the
limit is `−radial(u)`. The degenerate edge identifies the pole even
when a stored pcurve endpoint rounds past the root.
