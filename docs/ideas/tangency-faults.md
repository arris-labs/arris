# Idea: tangency-faults

- Status: Open
- Raised: 2026-11-13; consolidated 2026-11-16 from the training harness's
  `union-tangent-inside`, `hole-tangent-column`, `overlapping-blends-refusal`
  and items 3 and 5 of `fuse-contact-refusals`
- Prompt: none from the human; filed by the training harness (C12,
  ADR-0020's amendment of 2026-11-16) against 0.10.0, each case a fixture on
  its side with a closed form that CadQuery 2.3.0 (Open CASCADE) matches.

## Problem

Each of these returns one of the two outcomes `.agents/rules/kernel.md`
forbids:

- an `Ok` body that the checker refuses, which a caller cannot see without
  running `check("full")`;
- an `OpInternalError` ("kernel bug: the face arrangement is not a
  subdivision"), which tells the caller nothing about the part.

```python
import arris
import arris.build as b

# 1. A unit cylinder inside a cylinder of radius 2, tangent along x = 0, y = 1
#    (22 `bearing_retainer_cap` programs).
c = b.Solid.cylinder((0, 0, -1), (0, 0, 1), 1, 2)
u = c.fuse(b.Solid.cylinder((0, -1, -0.5), (0, 0, 1), 2, 1, model=c.model))
u.check("full")  # L5 f8, f9: loops 0 and 1 intersect
# V = 5 pi, A = 14 pi, one solid, genus 0.

# 2. A 30 x 5 x 1 bar with a radius-3 boss at each end, tangent to the end
#    face (15 programs, 3 more raise OpInternalError at other sizes).
box = b.Solid.box((-15, -2.5, -0.5), (15, 2.5, 0.5))
one = box.fuse(b.Solid.cylinder((12, 0, -0.5), (0, 0, 1), 3, 1, model=box.model))
two = one.fuse(b.Solid.cylinder((-12, 0, -0.5), (0, 0, 1), 3, 1, model=box.model))
two.check("full")  # L5 f43, f44: loop 0 intersects itself
# V = 154.50155561268, one solid.

# 3. A plate inside a unit cylinder's circle, with its arc tangent along
#    x = 0, y = 1 (25 programs).
c = b.Solid.cylinder((0, 0, -1), (0, 0, 1), 1, 2)
loop = arris.Loop.path((0.5, 0), [
    arris.Segment.line((0.5, 0.5)),
    arris.Segment.arc((-0.5, 0.5), (0, 1)),
    arris.Segment.line((-0.5, 0)),
    arris.Segment.line((0.5, 0)),
])
c.fuse(b.Sketch(b.Workplane.XY(-1.25), loop).extrude(1, model=c.model))
# OpInternalError: a section edge of f57 ends at a node nothing else reaches.
# V = 2 pi + (0.5 + pi / 8) / 4, one solid.

# 4. A bore of diameter 1 cut by a 1 x 1 column it is tangent to along four
#    lines. A bore of 0.9 or 1.1 builds.
box = b.Solid.box((-2, -3, -4), (2, 3, 4))
bored = box.cut(b.Solid.cylinder((0, 0, 4), (0, 0, -1), 0.5, 20, model=box.model))
bored.cut(b.Solid.box((-0.5, -0.5, -10), (0.5, 0.5, 10), model=box.model))
# OpInternalError: a section edge of f9 ends at a node nothing else reaches.
# V = 184, A = 238, one solid, genus 1.

# 5. A vertical cylinder r 1, z 0..3 on x = 5, fused with a horizontal one
#    r 1 along x -6..6 at z = 3, whose end disc is tangent to it (176
#    `pull_handle` programs). Lengthened to 14, it builds.
# OpInternalError: a cycle of f5's arrangement does not turn once.
# V = 15 pi - 8/3, A = 33 pi - 8.

# 6. A cone frustum and a box touching its top rim at one point (6 `bucket`
#    programs). At +x: OpDegenerateError (shells touching at a vertex). At -x:
#    OpInternalError "a hole of f7 lies inside no region". On a cylinder, +x
#    is refused and -x builds, so the fault follows the revolve's seam.

# 7. Two chamfers on the two rims of one planar face, each shorter than the
#    face but together wider (the ring of outer radius 10, bore 6, 8 high,
#    `edges(">Z").chamfer(2.5)`, then a cylinder r 1 cut across at z = 2).
#    The chamfer returns a body check("full") refuses ("hole loop 0 lies
#    outside every outer loop"); the cut then raises OpInternalError. No
#    solid has this shape: Open CASCADE's checker refuses its result too.
```

The harness's fixtures: `cq-union-cylinder-tangent-inside`,
`cq-union-end-cylinders-tangent`, `cq-union-arc-tangent-inside-cylinder`,
`cq-hole-tangent-column`, `cq-tangent-end-disc`, `cq-cone-box-rim-point`,
`cq-cone-box-rim-point-far-side`, `cq-chamfers-overlap-on-a-face`.

## Constraints it runs into

- `.agents/rules/kernel.md`: an `Ok` shape passes the checker, and an
  internal error is a kernel bug. Cases 1, 2 and 7 break the first rule,
  cases 3–6 the second.
- ADR-0015: cases 1–5 have closed forms. Case 7 has no valid result, so
  only a refusal is right there.
- `docs/ideas/touching-operands-fuse-to-several-shells.md` decides what a
  contact from outside becomes. Case 6's +x refusal waits on it.

## Options

### A — Build each one, typed refusal as the fallback
For an internal tangency (1–4), drop the line of tangency from the
arrangement, because one operand lies on the other's material side. Cases 1
and 2 already get the volume right, so the fault is only the imprint left
on the caps. For 5 and 6, find why the arrangement breaks at a tangent
disc or a seam. For 7, refuse two blends that use a face up as
`OpDegenerateError` naming the face and both edges, as one blend past its
faces already is. About 2 plans: the internal tangency (1–4), then the
contacts and the blend refusal (5–7).
### B — Typed refusals only
Each case becomes a modelled `OpDegenerateError`. This is about 3 steps and
gives the floor of A. About 250 programs stay refused, but they say why.
### Do nothing
About 250 programs either return a body that only a full check catches or
raise "kernel bug".

## Recommendation

A, with B as its first step. The refusal floor lands first and is reverted
case by case as each build lands, so no case keeps returning an invalid body
while the builds are worked on. Case 7 is B by nature.

## Decision for the human

❓ **Q1 — Build 1–6 with a typed refusal as the floor, refuse 7 (A)?**
➡️ Yes: an invalid `Ok` body is the worst outcome the kernel has, so the floor goes first.

## Decision
