# ADR-0080 — A torus face holds the regular band

- Status: accepted (2026-10-31)
- Plan: `horn-torus` step 1
- Amends: ADR-0035, ADR-0036 §3 (the horn and spindle refusals they record)
- Reference trees read: none; Open CASCADE run as the oracle

## Context

`Surface::Torus` carried `r < R`, so a fillet whose ball's centre circle is
not above `r` was refused `BlendTooLarge`: a horn torus (`R = r`, the
surface touches its axis at one point) and a spindle torus (`R < r`, the
surface crosses itself on the axis). Open CASCADE builds both as a torus
face. Nothing in the evaluation needs `r < R`: `P(u, v)` is defined for any
`R, r > 0`, and the face never needs the points on the axis.

## Decision

1. **The invariant is `R, r > 0`.** A torus face lies on the torus's
   **regular band**: the part off its axis, where `R + r cos v` exceeds the
   face's tolerance. On the band, `(u, v)` from `atan2` is the unique
   inverse, so evaluation, projection and the normal are unchanged. The
   normal is undefined only at the axis (`R + r cos v` negligible), as
   before. A point on a spindle's lemon (inner) sheet is not on the band's
   surface and is never the image of a band parameter.
2. **The margin is the face's tolerance as a distance from the axis**, the
   measure the ring's `contacts_on` uses, so the blend and the checker refuse
   at one bound.
3. **A face reaching past the band is a checker violation by name** (step 2,
   `FaceFault::OffTorusBand`). `sample::patch` refuses such a region
   by the same bound.
4. **A section keeps only branches on the band**, split at the singular
   point(s) on the axis, each branch through one ending at a vertex there
   (step 3; the trace, walk and ring modules).
5. A revolve of an arc whose circle crosses the axis stays
   `SweepReason::SpindleTorus`; Open CASCADE's sphere where a stripe closes
   on the axis and a contact at the axis stay `BlendTooLarge`.

## Consequences

The data model admits tori the kernel could not build; every operation that
takes a ring-torus face takes a band face, which steps 2 to 9 of the plan
hold. `sample::torus` (a whole torus as a solid) keeps `r < R`: a whole
torus is not on the band unless it is a ring.

## Alternatives considered

- Keep `r < R` and refuse: the status quo, 98 of 277 sampled edges at the
  census.
- A sphere or a NURBS patch for the horn: a different surface than Open
  CASCADE's and the blend's own, and not an operand of the torus tables.
