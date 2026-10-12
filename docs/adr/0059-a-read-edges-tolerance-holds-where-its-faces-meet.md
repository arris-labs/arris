# ADR-0059 — A read edge's tolerance holds where its faces meet

- Status: accepted (2026-10-12)
- Plan: `gap-and-orientation-healing` step 2
- Amends: ADR-0025 §4 (what a read edge's tolerance is measured from);
  not edited
- Reference trees read: none. Open CASCADE is run as the oracle.

## Context

The plan's step 2 read CTC-02 (rc) with the model's `max_tolerance`
raised past its gaps, as healing will. The checker at `Full` found its
base plane `f0` meeting two drafted boss cylinders, `f178` and `f331`,
away from their shared edges (S5). The two-face excerpt
`real/edge-off-a-drafted-cylinder` holds it.

The shared edge `#9225` lies on the plane exactly and `0.014950` off the
cylinder. The cylinder's axis is `2°` off the plane's normal. The read
set the edge's tolerance to `0.014950`: the largest distance of its
pcurves' images from its curve (ADR-0025 §4). But the plane and the
cylinder meet in an ellipse, and that ellipse passes `0.014960` from the
edge at its middle. A curve `d` off one surface and on the other is
`d / sin θ` from where they meet, for faces crossing at `θ`, and
`1 / sin 88° = 1.0006`. S5 asks whether every point where the two faces
meet lies within the tolerance of an edge they share. Here one does not.

`docs/DATA-MODEL.md` §Tolerances says what an edge's tolerance is: "the
true geometry this entity stands for lies within distance `t` of the
stored geometry". An edge stands for where its two faces meet. The
checker is right and the read under-measured. The same holds for an
operation on the body: a boolean that takes the edge's tube as where the
faces meet would miss them by `1e-5`.

## Decision

**1.** A read edge between two faces also measures, at each of its
samples (the checker's and the `PCURVE_SAMPLES` a fit is held to, each
peak climbed as the gap from each surface is), the distance from its
curve to the line where the two surfaces' tangent planes at its pcurves'
images meet. Its tolerance is the larger of that distance and its gap
from each surface. For a curve on one surface and `d` off the other,
that distance is `d / sin θ`.

**2.** The raise applies only where the faces cross at an angle whose
`1 / sin θ` is at most `arris_math::SECTION_GROWTH_LIMIT`, `2`: faces
crossing at `30°` or more. Nearer tangent, where the faces meet is
ill-conditioned, and the linear model's answer is not where they meet.
Two G1 blend faces meet along a band as wide as their tolerances allow,
and `d / sin θ` for an edge one rounding off a fillet would be a gap
the read refuses. There the edge keeps its gap from each surface, as
before. A point where a surface has no normal (a pole, an apex) also
keeps it.

**3.** An edge with fewer or more than two uses (a sheet's free edge)
has no pair of faces and is measured as before.

## Consequences

- An exact file is unchanged. Its gaps are rounding, at most twice that
  under the limit, and floored at `default_tolerance` like every other
  read value.
- An edge already at a gap from one face, between faces crossing at
  `θ`, grows by `1 / sin θ`, at most `2×`. This is measured against the
  healing ceiling (ADR-0058) like any gap.
- An edge between near-tangent faces that a file leaves past rounding
  off one of them keeps a tolerance S5 may find too small. If the corpus
  meets one, that is a new row, not a reason to raise the limit silently.
