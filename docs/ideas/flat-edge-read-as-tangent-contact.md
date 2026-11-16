# Idea: flat-edge-read-as-tangent-contact

- Status: Open
- Raised: 2026-11-08; consolidated 2026-11-16 from four of the training
  harness's filings (`union-coplanar-third-operand`, `slots-through-a-cylinder`,
  `interference-split-planar-faces`, and item 8 of `fuse-contact-refusals`)
- Prompt: none from the human; filed by the training harness (C12,
  ADR-0020's amendment of 2026-11-16) against 0.10.0, each case a fixture on
  its side with a closed form that CadQuery 2.3.0 (Open CASCADE) matches.

## Problem

Four booleans that Open CASCADE builds are refused with the same words:
`OpDegenerateError`, "degenerate result: the faces touch along a curve
interior to both result faces". None of them has a tangent contact. Each
one has a face already split along an edge into coplanar pieces, either by
an earlier operation or by another builder. The guess, still unverified, is
that `TangentContact` reads that flat internal edge as a contact. One root
cause would make this one plan.

```python
import math, arris
from arris.build import Solid

# 1. A chained fuse on a shared end plane (all 100 `lobed_knob` programs of
#    the harness's first slice, a third of it). Every order is refused, each
#    pair fuses, and the n-ary call builds.
m = arris.Model()
a, b, c = (Solid.cylinder((x, y, 0), (0, 0, 1), 1.0, 1.0, model=m)
           for x, y in [(0, 0), (1, 0), (0.5, math.sqrt(3) / 2)])
a.fuse(b).fuse(c)   # refused (+f6, +f9, +f10)
a.fuse(b, c)        # ok: V = 3 pi/2 + sqrt(3), A = 6 pi + 2 sqrt(3)
# Moving the third cylinder up by 0.5, or lengthening it to [-0.5, 1.5],
# makes the chain build. One shared end ([0, 2]) is enough to refuse it.
# Three boxes in an L on one plane, [0,2]x[0,2], [1,3]x[0,2], [0.5,2.5]x[1,3],
# z in [0,1], are refused the same way (V = 8, A = 28). Three boxes in a row
# build.

# 2. Three slots through a cylinder's axis, 60 degrees apart (81
#    `wall_anchor` programs). Two slots build; the third cut is refused.
# cq.Workplane("XY").cylinder(10, 4)
#   .cut(box(9, 1, 6)).cut(box(9, 1, 6) turned 60).cut(box(9, 1, 6) turned 120)
# a = 1/2, rho = 4, S = 2 a sqrt(rho^2 - a^2) + 2 rho^2 asin(a/rho):
# V = pi rho^2 10 - 6 (3 S - 3 (8 a^2/sqrt 3) + 2 sqrt 3 a^2), genus 5.

# 3. `interference` of one all-planar part built by Arris against the same
#    part read from Open CASCADE's STEP, whose coplanar faces are merged
#    (100 of 100 `phone_stand` pairs). Against itself the Arris body answers
#    1.0 exactly, and so does Arris against Arris split differently.

# 4. A cut whose tool shares a wall with a hole already in the part. A
#    10 x 8 x 4 box with a through hole of r 1/2 on z, then a cylinder r 1/2
#    from z 0 down plus a cylinder r 1 on z -1/2..0
#    (`.hole(1).cboreHole(1, 2, 0.5)`). Open CASCADE builds the hole with an
#    annular chamber: V = 320 - pi - 0.375 pi. A bore of another width cuts,
#    and so does no bore.
```

The harness's fixtures: `cq-union-three-coplanar-cylinders`,
`cq-three-slots-through-cylinder`, `cq-bore-over-a-bore-of-its-width`. Case 3
is reproduced by its rescore of `phone_stand`.

## Constraints it runs into

- `ExpectError::TangentContact` refuses faces that really do touch along a
  curve, by design. These cases have no such contact, so they are outside
  that refusal's domain.
- ADR-0015: each case has a closed form, so the fixture holds Arris to it.

## Options

### A — Find why the chained case classifies as a contact, and fix it
Shrink the cylinder triple until the contact test that fires is clear. Then
fix how a section edge lying in a coplanar split is classified, so that the
two-call and n-ary paths agree. All four cases become fixtures. If they
share the cause, about 3–4 steps.
### B — Merge coplanar pieces before the section
Fuse coplanar same-domain faces of each operand first. This is the backlog's
same-domain face merging, a larger change that renames faces and so touches
provenance. It also covers case 3 when the STEP side is the one that is
split.
### Do nothing
About 280 of the harness's programs stay refused, and its `phone_stand`
scores stay sampled.

## Recommendation

A. It fixes the classification where it goes wrong and leaves the topology
alone. B is a cycle of its own (same-domain merging is out of C12). If
case 3 does not share the cause, it falls back to exact point
classification (`query::classify_points`, done) and becomes a backlog line.

## Decision for the human

❓ **Q1 — Take the four cases as fixtures and fix the classification (A)?**
➡️ Yes: one cause is likely, and the cylinder triple is the shrink target.

## Decision
