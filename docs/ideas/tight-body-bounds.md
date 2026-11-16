# Idea: tight-body-bounds

- Status: Open
- Raised: 2026-11-14
- Prompt: none from the human; filed by the training harness (C12,
  ADR-0020's amendment of 2026-11-16) against 0.10.0.

## Problem

`Model.bounds(body)` is the union of its faces' boxes, and a face's box takes
its edges' boxes; a NURBS edge is boxed by its control hull (`DATA-MODEL.md`,
`Curve::bounds`: "an outer bound … a tighter, piecewise bound is the
caller's"). The Python binding offers no tighter query, so a caller cannot
get a body's extent where a NURBS edge reaches it. The harness compares a
body's box with Open CASCADE's tight one (`AddOptimal`) to 1e-6, and finds
Arris 1e-3 wide where the geometry is exact.

```python
import arris

m = arris.Model()
cyl = m.primitive_cylinder((0, 0, 0), (0, 0, 1), 2.0, 2.0)[0]
pin = m.primitive_cylinder((0, -3, 1), (0, 1, 0), 0.5, 6.0)[0]
body = m.cut(cyl, pin)[0]
m.bounds(body)
# ((-2.0000002, -2.0012288786, -2e-07), (2.0000002, 2.0012288786, 2.0000002))
# Closed form: (-2, -2, 0) .. (2, 2, 2); a tessellation at a chord of 1e-4
# reaches |y| = 2.000000003. The two NURBS edges the hole leaves on the side
# wall are each boxed to |y| = 2.0012288786.
```

It is the harness corpus's `shaft_collar`: 5 programs, a collar cut across
by a set-screw hole whose apex on the outer wall is the part's extent, boxed
5e-4 to 1e-3 wide (the harness's fixture `cq-cross-hole-bounds`). Volume,
area and centroid are the closed form's to 1e-9.

A second case, in the corpus's `spacer_ring` and `clevis_pin`: a ring or pin chamfered at its rims, then
cut by a box or a cross cylinder. A box slot's two walls cut the chamfer's
cone in two hyperbolas (NURBS), whose hull reaches the outer radius where the
part stops 0.037 short (the harness's `cq-chamfer-slot-bounds`, a
disc of radius 21.19 slotted to |z| < 1.25: x max 21.1900001 against 21.15310).
The 81 `clevis_pin` programs (a cylinder chamfered at both rims, one or two
cross holes) are boxed 1e-3 to 5e-3 wide the same way as the `shaft_collar`.

## Options

### A — `Model.bounds` tight to the body's tolerance
Refine a NURBS edge's (and surface's) box by subdividing it until each
piece's hull is within the tolerance, as `Curve::bounds` already suggests for
callers. Cost: a subdivision per NURBS edge, only where the hull reaches the
running box.

### B — A separate `tight_bounds` query
Leaves `bounds` the cheap reject the booleans use, and gives callers that
measure a part the tight box.

### Do nothing
The harness classes such rows as kernel bugs, which hides that Open
CASCADE's volume is off the closed form on the same parts.

## Recommendation

B. `bounds` is the cheap reject the booleans and `interference_many` rely
on, so it stays an outer bound. `tight_bounds` subdivides only the NURBS
edges and faces whose hull reaches the running box, down to the body's
tolerance. About 2 steps.

## Decision for the human

❓ **Q1 — A separate `tight_bounds` query (B), `bounds` unchanged?**
➡️ Yes: the cheap box has internal callers.

## Decision
