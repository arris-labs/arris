# ADR-0065 — A loft matches sections by arc length and is exact where a ruled span's kind holds it

- Status: accepted (2026-10-16)
- Plan: `loft` step 2 (the matching, `LoopNurbs`, `LoftError`); the ruled
  loft lands in steps 3 and 4, the smooth one in step 5
- Follows: ADR-0063 (a profile loop as one closed rational quadratic, a
  span exact where a kind holds it), ADR-0062 (the error budget of a
  fitted face), ADR-0020 §1 (closure: a loft is an operand of every
  boolean and blend the NURBS cycle allows)
- Reference trees read: none. Open CASCADE is run as the oracle
  (`BRepOffsetAPI_ThruSections` through `tools/oracle/oracle/loft.py`)

## Context

A loft joins two or more placed profiles, its **loft sections**. Two
questions decide it: which point of one section goes with which point of
the next, and what surface joins them. The plan assumed Open CASCADE
splits a section with fewer segments by arc length and keeps the
consumer's start points. Step 1 measured otherwise:

- With `CheckCompatibility(false)` `ThruSections` splits nothing and
  lofts the first edge of each section (a circle against a square gives
  one face). It has to be on.
- `BRepFill_CompatibleWires` then re-starts every wire at the vertex
  nearest the previous section's start, whatever `WithRotation` says, and
  cuts the wire with fewer edges where the other's vertices *project*
  onto it (a circle against a 12 × 4 rectangle is cut at atan(1/3), the
  corner's direction, not at the corner's arc-length fraction 1/8 of a
  turn). A start point on the other wire's edge adds a vertex there.
- The smooth loft is one polynomial of degree n − 1 through the n
  sections (at most `MaxDegree` 8), edge points matched by edge fraction,
  spans parametrised by chord length; a closed loft repeats its first
  section and is not periodic. A quadratic through the duct's three
  sections at chord-length parameters is within 2e-5 of Open CASCADE's
  volume and area, centripetal 1.6% off; a degree-4 polynomial through the
  closed ring's four circles and the first again is within 3e-5, a
  periodic cubic 10% off.

## Decision

**1. Sections are matched loop by loop, each loop from its start.** Loop
`k` of every section goes with loop `k` (the outer with the outer, hole
`i` with hole `i`); the loop counts must agree. A loop's start is the
first edge `Profile::edges` returns, the consumer's start point; it is
never re-started to the nearest vertex. Open CASCADE's re-start is a
property of its oracle, not of loft: a fixture places its start points
where Open CASCADE keeps them, and a fixture whose sections it would
twist differently is held to the closed form.

**2. Equal segment counts correspond segment for segment; unequal counts
are cut at every vertex, by arc length.** When every section's loop has
the same number of segments, segment `j` goes with segment `j`, whatever
their shapes. Otherwise each loop is cut at the arc-length fractions, from
its start, of every other loop's vertices (their union, two fractions
within the linear tolerance over the shortest perimeter one), so a circle
against a square is cut where the square's corners fall, and a triangle
against a square into six segments each. This differs from Open CASCADE's
projection exactly where the sections are not similar and centred, and
the fixtures hold Arris to the oracle only where the two pair the same
points. Inside a matched pair, points correspond by the segments'
parameters normalised to `[0, 1]`: the same `u` of two compatible NURBS,
so a ruling is the straight line between `C₀(u)` and `C₁(u)`. A pair of
lines or of arcs of one angle pairs as arc length does; a circle's arc
against a square's side does not — the arc's rational parameter is not
its angle — and Open CASCADE pairs them by arc length, which no NURBS
holds (its volume equals the arc-length pairing's integral to 6e-11, which
Arris' own differs from by 1.2e-3, the area by under 2.5e-3 and the
tensor's `zz` entry by 2.5e-3; `loft-circle-to-square` carries those as
its tolerances, measured, not an error budget). Every other fixture,
twisted and ring-to-ring included, agrees to 1e-9.

**3. Matched sections are one knot vector.** Every segment is the rational
quadratic of ADR-0063 (`LoopNurbs`, lifted from `sweep::along` unchanged),
reparametrised to `[0, 1]`, the interior knots of the section pair
unioned (a value as often as the section that has it most often) and
inserted, so the loops of all sections are compatible NURBS with a double
knot at every vertex: what `fit_skin` and the interpolation of step 5 take.
`match_sections` also returns, per matched segment, the consumer's segment
it came from (`MatchedLoop::sources`).

**4. A face's role names the matched segment.** `Role::Loft(PathPart)`
with `Side { segment }` is the matched segment's index in the walk of its
loop (open question 3); a split segment's pieces are numbered in their
own right, and the consumer reads `sources` for where they came from. The
numbers are stable for sections whose counts agree; a change of one
section's segment count renumbers.

**5. Refusals by name.** `LoftError::TooFewSections` (two, three when
closed), `LoopCount { section }`, `Profile { section }` (the profile
validator's own error), `Loop { section, loop_index }` (a curve no
rational quadratic holds), `Coincident { section, previous }` (two
consecutive sections equal control net for control net),
`TooManySections { given, max }` (a smooth loft past §7's bound),
`Smooth { loop_index, segment, source }` (the interpolation of one matched
segment failed: a degree past the NURBS bound, a weight not staying
positive, a singular system; with at most nine sections the first is
unreachable by an honest input, so it is the typed guard, not an `Internal`). Unequal loop
counts, a point as an end section and a NURBS profile segment are
refusals, not extensions.

**6. A ruled span is exact where its kind holds it** (step 4): classified
by check, not by segment kind. Two line segments coplanar within the
tolerance give a `Plane`; two circles coaxial in parallel planes a
`Cylinder` or a `Cone`; anything else a degree-1 NURBS in the span's
direction with no fit. The check reads each matched segment's geometry at
sixteen samples of its parameter: straight within the linear tolerance, or
a circle (centre, normal and radius from three samples, the rest on it)
whose angle only grows. Coaxial means both normals along the line of
centres within the angular tolerance *and* the two arcs at the same
angle about that axis at every sample, so a ruling is a generator; a cone
stands on the smaller circle and points at the larger. A ring edge is the
analytic line or circle only where every face on it is exact, else the
segment's NURBS; a rail is a straight line wherever a face on it is exact,
and a NURBS face meets it by a degree-one pcurve at arc length. Joints between exact spans are the analytic
curves, beside a NURBS span the segment's exact NURBS (ADR-0063).

**7. A smooth loft interpolates the sections' points as Open CASCADE
does** (step 5): per matched segment, the polynomial of degree n − 1
through the n sections, `S(u, v) = Σₖ Lₖ(v) Cₖ(u)`, at chord-length
parameters, two sections the ruled loft (exact spans included); a closed
loft repeats the first section and is not periodic. One loft is one span:
its rings are the end sections' (only the first's when closed), each rail
the surface's control column at the vertex.

- *Points, not nets.* Interpolating the homogeneous control nets is not
  interpolating the points where the sections' weights differ (a line
  against an arc, arcs of different angles): on `loft-duct` it gave a
  volume 4.1e-3 off. `interpolate_skin` first writes the sections of a
  column over one weight function, the product of their distinct ones
  (each weight function divided by its first weight; constant ones
  dropped), segment by segment in Bernstein form, so every section keeps
  its points at every `u` and the homogeneous interpolation is the point
  interpolation, exactly. The degree along `u` grows by two per distinct
  weight function (the duct: four); sections that share their weights,
  scale included, keep their nets. The interpolated weight is that
  common function at every row, so it is positive and the system is
  never singular for increasing parameters.
- *The chord is the control nets'*: the sum over every control point of
  every loop of its distance to the same point of the next section,
  normalised. On the duct it gives 0.35196 against the sampled points'
  0.3518; interpolating sampled points paired by arc length at 0.35196
  lands on Open CASCADE's volume to the sampling's 1e-5, at 0.3518 3.4e-5
  off.
- *What the fixtures hold.* `loft-closed-torus-like` (equal circles, so
  the pairings agree) matches an independent quadrature of the exact
  surface to 1e-14 in volume and area; Open CASCADE, approximating it,
  is 2.9e-9 off, and the fixture holds 1e-8. `loft-duct` differs from
  Open CASCADE by §2's pairing alone (volume 8.5e-5, inertia 1.7e-4) and
  carries that as its tolerances, as `loft-circle-to-square` does.
- More than nine sections, the first counted twice when closed, is
  `LoftError::TooManySections` (the polynomial oscillates; Open CASCADE
  caps the degree at 8 and approximates instead).
- The measure took one Gauss–Legendre interval along the ring's one span
  round a whole turn (1.3e-7 of its area): a NURBS direction with fewer
  spans than its degree is now split into at least its degree's count of
  panels, and a boundary crossing such a line exactly at one of its
  crossing samples, which both open windows beside it missed, is split
  there (`arris_geom::integrate`).

## Consequences

- `arris-geom` gains `LoopNurbs`, `LoopPiece`, `match_sections`,
  `MatchedSection`, `MatchedLoop`, `LoftError`, `LOOP_DEGREE`,
  `interpolate_skin` and `MAX_SMOOTH_SECTIONS`; `sweep`
  uses `LoopNurbs` and its fixtures are unchanged bit for bit.
- A loft's NURBS faces meet only on the columns it builds (the plan's
  non-goal); NURBS against NURBS stays `Unsupported` until the marcher.
- Alternatives rejected: `fit_skin` (needs a continuous family of
  sections, not a few); matching by vertex count only (refuses a circle
  against a square); Open CASCADE's projection rule (not symmetric in the
  sections, and re-starts the consumer's start points).
