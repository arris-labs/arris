# ADR-0060 — `max_tolerance` bounds what an operation grows, not what a body holds

- Status: accepted (2026-10-12)
- Plan: `gap-and-orientation-healing` step 3
- Amends: ADR-0058 (its last consequence, "the checker … is not asked to
  change for that alone", was wrong) and ADR-0029 §4 (body bytes refuse a
  tolerance outside `[min_tolerance, max_tolerance]`); neither is edited
- Reference trees read: none

## Context

ADR-0058 §2 decided that the reader's healing ceiling, and not the model's
`max_tolerance`, bounds what a read grows: the corpus's healed parts carry
edges at 0.0392 (CTC-02), 0.0506 (CTC-04) and 0.0182 (CTC-05) in a model
whose default `max_tolerance` is 0.01. It said the checker would not need
to change, since every row judges an entity at its own tolerance.

One row does not. The checker's V1 held a vertex's tolerance to
`[min_tolerance, max_tolerance]`, so every healed body failed it at
`Level::Fast`, which the reader runs on every body: healing would read
nothing. Body bytes (ADR-0029) refused the same range on import, so a
healed body saved in Arris's own format would not load into a model of
the same precision.

## Decision

**1.** `Precision::max_tolerance` is the most an *operation* may grow a
tolerance to: an operation that would exceed it returns
`OpError::Tolerance`, as it always has. It is no longer a bound on what a
body may hold. A body's tolerances are what built it — an operation within
the ceiling, or a read that healed past it and recorded that it did.

**2.** The checker's V1 holds a vertex's tolerance to `min_tolerance` from
below only. `ToleranceBound::AboveMaximum` is removed. No other row
bounded a tolerance from above by the ceiling: E5 and F2 order an entity's
tolerance against its neighbours', which holds a healed body as it holds
any other.

**3.** Body bytes refuse a tolerance below the reading model's
`min_tolerance` only: `BodyError::Precision` loses its `max` field. The
floor stays, for the reason ADR-0029 gave it: a tolerance finer than the
model can hold is not rescaled.

## Consequences

- A healed body passes the checker and round-trips through body bytes in
  the model it was read into.
- An operation on a healed body is unchanged: where its result needs a
  tolerance past the ceiling it refuses, naming the entity. Whether that
  refusal is too eager on a healed body is measured by the battery the
  plan's step 5 runs on every part that now reads, not argued here.
- A body that holds a tolerance past the ceiling because something built
  it wrongly is no longer flagged by V1. The kernel's operations each
  refuse before growing past it, so the only path is a caller's
  hand-built body, whose tolerances the checker still holds against its
  geometry (V2, V3, E4) and its neighbours (E5, F2).
- Breaking: `ToleranceBound::AboveMaximum` and `BodyError::Precision::max`
  are gone, the Python `BodyPrecisionError.max` attribute with them.
