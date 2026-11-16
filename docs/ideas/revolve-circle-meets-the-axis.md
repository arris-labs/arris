# Idea: revolve-circle-meets-the-axis

- Status: Open
- Raised: 2026-11-13; consolidated 2026-11-16 from the training harness's
  `spindle-torus-revolve` and item 6 of `fuse-contact-refusals`
- Prompt: none from the human; filed by the training harness (C12,
  ADR-0020's amendment of 2026-11-16) against 0.10.0, each case a fixture on
  its side with a closed form that CadQuery 2.3.0 (Open CASCADE) matches to
  1e-12.

## Problem

`revolve` refuses an arc whose circle crosses or touches the axis, even when
the arc itself stays on one side of it. The refusal is `OpDegenerateError`:
"an arc's circle crosses the revolve axis: a spindle torus". Only the arc is
revolved, so the surface is the outer part of a spindle torus (an apple) or
half a horn torus. Neither intersects itself.

```python
# 1. The arc starts on the axis (32 `rivet` programs).
cq.Workplane("XY").moveTo(0, 4).threePointArc((6, 4), (8, 0)).lineTo(0, 0).close() \
    .revolve(360, (0, 0, 0), (0, 1, 0))
# 2. The arc ends on the axis (36 `dome_cap` programs).
cq.Workplane("XY").moveTo(0, 0).lineTo(8, 0).threePointArc((6, 4), (0, 4)).close() \
    .revolve(360, (0, 0, 0), (0, 1, 0))
# Both: the disc of radius 5 centred on (3, 0), cut to x >= 0, y >= 0, turned
# about y. V = pi (236 + 225 pi - 225 asin(4/5)) / 3 = 768.87,
# A = 10 pi (3 (pi - atan(4/3)) + 4) + 64 pi = 535.42, ybar = 2.0920.
# `dome_cap`'s circle is centred 1.3e-4 off the axis (a sphere's three rounded
# points), so the same check fires at a tiny offset. A circle centred exactly
# on the axis (a sphere) builds.

# 3. A circle that touches the axis (R = r): half a horn torus.
# V = pi^2, A = 2 pi^2 + 2 pi.
```

The harness's fixtures: `cq-spindle-torus-arc-from-axis`,
`cq-spindle-torus-arc-to-axis`, `cq-horn-torus`.

## Constraints it runs into

- C11 made the horn and the spindle torus a face, an operand of every
  operation, and readable from STEP (`CHANGELOG.md` 0.10.0: a fillet's
  rolling ball, the reader). A torus face whose boundary crosses its axis is
  still a checker violation. So the surface exists, and only the revolve's
  check and the face's trim are missing.
- ADR-0014: the revolve's refusals are `SweepReason`s. If any refusal stays,
  it names the arc.

## Options

### A — Build the arc's revolve
Refuse only an arc whose own points reach both sides of the axis (that
solid intersects itself). Build an arc on one side as a torus face trimmed
to the arc's v-range: the outer part for a spindle, half for a horn. About
2–3 steps, mostly the trim at the pole the arc ends on.
### B — A clearer refusal
Keep refusing, but name the arc and the crossing.
### Do nothing
About 70 programs stay refused.

## Recommendation

A. The surface has existed since C11, and two of the corpus's families write
this profile. B only if the trimmed face at the axis pole turns out to be a
case the checker cannot hold.

## Decision for the human

❓ **Q1 — Build an arc whose circle reaches the axis while the arc stays on one side (A)?**
➡️ Yes: C11 already made these faces operands.

## Decision
