# ADR-0076 — A new vertex inside an operand vertex's tolerance may take up to that vertex's tolerance

- Status: accepted (2026-10-30)
- Plan: `tolerant-tangency` step 7 (⚠ OPEN 1, settled)
- Amends: ADR-0060 §1 (`max_tolerance` is the most an operation may grow a
  tolerance to), ADR-0074 §4 (the ceiling of a vanishing end's stop)
- Reference trees read: none

## Context

ADR-0074's stripe stops on its narrowest exact arc at a new vertex `w` on
the edge, whose tolerance is the arc's distance from the edge plus the
surface's share. AP242 edition 1 of CTC-05 writes the same edge with
vertices of `2.05e-2`, healed by the read, and the arc lies `1.97e-2` off
the edge's curve: past `max_tolerance` (`1e-2`) and inside the old
vertex's own tolerance, `1.55e-2` along the edge from it.

The read recorded that the two faces there meet no better than `2.05e-2`.
Cutting the edge at `w` adds no uncertainty of its own: `w` lies within
that ball already.

## Decision

A new vertex whose point lies within an operand vertex's tolerance of it
may take a tolerance up to that vertex's, where `max_tolerance` would
refuse it. The remnant's edge takes the tolerance the arc needs and, on
top of it, as much of its own as the same ceiling leaves, since the
remnant's end is moved onto the contact. A point outside every operand
vertex's tolerance keeps `max_tolerance` as ceiling.

## Consequences

- CTC-05 AP242's battery edge e258 (`r 0.978`) builds, held to the second
  oracle's change. AP242's e256 at `r 0.889`, the same edge, builds too:
  the stripe, the kink and the junction are the AP203 edition's, and the
  second oracle's record of that edition's change holds it (the second
  oracle refuses the AP242 encoding, which proves nothing about the
  geometry: its tolerances, not the edge, are what it cannot hold).
- `OpError::Tolerance` is unchanged elsewhere; no public type changes.
