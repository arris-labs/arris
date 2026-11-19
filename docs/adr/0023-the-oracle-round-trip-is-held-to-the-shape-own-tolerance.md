# ADR-0023 — The oracle's own STEP round trip is held to the shape's own tolerance, and a fixture may declare it lossy

- Status: accepted (2026-09-24)
- Plan: none — a CI fix after C3's close (the `oracle` job red on `main`
  since `c3-conic-hits`)
- Follows: ADR-0015 (a fixture may declare the oracle's measurements
  wrong), `docs/ROADMAP.md` §M0 (the oracle round trip)

## Context

`tools/oracle/selftest.py` is M0's proof that the oracle's STEP path can
be trusted: for every fixture it builds Open CASCADE's result, writes
Open CASCADE's own STEP of it, reads that back, and compares the two at
the fixture's tolerances — relative `1e-9` on volume, area and inertia
by default. The corpus runner leans on the same path from the other
side: `compare.py` reads *Arris's* STEP through Open CASCADE's reader.

C3 brought the first fixtures on which Open CASCADE's own round trip
fails, while Arris's STEP of the same results reads back true (the
corpus's compare stage passes on every one). They fail in two ways:

- **Within the shape's tolerance.** `boolean/edge-touching-four-into`,
  `edge-touching-tilted-cut`, `regression/edge-touching-tilted-posed-cut`
  and `regression/pipe-elbow-a-tolerance-off` are results whose vertex
  tolerances Open CASCADE grew to 1.4e-7–5.8e-7 while building them. Its
  reader moves the boundary within those tolerances, and the volume of a
  10-unit cube comes back 9.4e-9 relative off — the boundary moved by
  much less than the tolerance the file itself carries, held to a bound
  that tolerance never promised.
- **Another shape.** `boolean/ball-corner-cut`,
  `boolean/ball-offset-drill-cut` and
  `regression/ball-beside-pole-slice-cut` — sphere faces cut near a pole
  or across the seam — read back with an edge fewer and a loop more, and
  `ball-offset-drill-cut` as another solid altogether: volume 3.447 where
  Open CASCADE built 30.063, four of five probes flipped. Every built
  shape passes `BRepCheck_Analyzer`; the loss is in Open CASCADE's own
  writer and reader.

Neither says anything about Arris, and the job that says it is red on
every commit, which hides the day it says something that does.

## Decision

1. **The self-test holds a round trip to the shape's own tolerance.** A
   reader may move any point of the boundary by up to the shape's largest
   vertex tolerance `t` and still hand back the same shape, so the
   comparison is widened to the first-order change such a move makes —
   `A·t` of volume, `L·t` of area (edges of total length `L`),
   `A·t·R/V` of centroid and `A·t·R²` of inertia, `R` the reach of the
   bounding box from the centroid — wherever that is wider than the
   fixture's (`oracle.measure.within_own_tolerance`). Counts, genus and
   probe classes stay exact.
2. **A fixture may declare Open CASCADE's own STEP of its result lossy**:
   `analytic.step_differs: "why"`, the self-test then records the result
   without the round trip. The text names what the reader hands back
   against what Open CASCADE built, in numbers. The lint refuses it on a
   result Arris does not build.
3. **Nothing Arris is held to changes.** Only the self-test reads either
   rule; the corpus runner and `compare.py` hold Arris's STEP to
   `expected.json` at the fixture's tolerances exactly as before.

## Consequences

When only Open CASCADE's native self-read loses a trim,
`analytic.occt_step_self_read_differs` holds the self-test to the result's
B-spline export. The corpus reads both exports with Arris and checks
Arris's own STEP through Open CASCADE.

- The `oracle` job is green again, and a red one is news.
- The self-test is weaker on shapes whose tolerances Open CASCADE grew:
  it can no longer see a reader error smaller than the shape's own
  tolerance on a result built at `1e-7`. That is the precision the file
  claims, and the corpus's compare stage — Arris's STEP, the fixture's
  tolerances — is where the kernel's precision is held.
- `step_differs` is the third escape beside `counts_differ` and
  `measure_differs`, and like them keeps the oracle's values as the
  record. It is the only one that changes no comparison Arris faces.

## Alternatives considered

- **Loosen the fixture tolerances.** That would loosen the corpus's
  comparison of Arris as well, which has nothing wrong with it.
- **Exempt all seven by name.** It hides the four that are a bound
  question behind the same flag as the three that are a reader bug, and
  the next shape with a grown tolerance needs another flag.
- **Drop the self-test's round trip.** It is what makes `compare.py`'s
  reading of Arris's STEP worth believing, and it still catches every
  fixture Open CASCADE can round-trip — all but three.
