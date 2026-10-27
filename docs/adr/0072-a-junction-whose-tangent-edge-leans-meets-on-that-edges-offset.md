# ADR-0072 — A junction whose tangent edge leans meets on that edge's offset: one ball where the offset of the ruling meets the shared plane's, off the vertex

- Status: accepted (2026-10-27)
- Plan: `three-edge-run-vertex` step 2 (the decision, on step 1's
  fixtures and its reading of Open CASCADE's faces). The closed form
  lands in this step and the runs' re-ending in step 3.
- Amends: ADR-0035 §3 (two stripes meet at a tangent vertex on the ball's
  cross-section) and ADR-0062 §5's step 5b (a walked stripe's junction):
  where the vertex's third edge leans out of the ball's normal plane, the
  shared ball is no longer at the vertex.
- Follows: ADR-0035, ADR-0062, ADR-0039 (the vertex of four, which this
  does not cover)
- Reference trees read: none. Open CASCADE is run as the oracle
  (`BRepFilletAPI_MakeFillet` through `tools/oracle/`, its result written
  by `occt_step.py` and read with `inspect_step`), not read.

## Context

A tangent vertex of three edges joins two runs of a chain: `e` and `e′`
share the face `P`, their far faces `W` and `C` are tangent along the
vertex's third edge `w`. ADR-0035 §3 places both runs' balls in the
vertex's normal plane and finds them one ball, which holds when `w` lies
in that plane: a cylinder's ruling square to the chain.

When `w` leans out of the normal plane, each run's ball at the vertex
touches its own two faces and the two are apart. A walked run's ball
(ADR-0062) is the one that shows it, which is why ADR-0062's step 5b
refused it as `VertexBlend`. That refusal is the last site both CTC-02
AP242 and FTC-07 AP242 stop at in the blend network, a plane top over a
wall that turns into an oblique cylinder. The backlog's "skew tangent
junction" names it: the balls agree to `2.1e-4`, the contacts lie `0.06`
off the ruling.

Step 1 shrank it to `regression/rounded-block-oblique-top-fillet`: a 4 × 2
rectangle with corners rounded at 0.5, extruded 2, its top cut by a plane
tilted 20° about `x`, one top line filleted at 0.2. The chain is the top's
eight edges. At the y walls' ends the ruling is square to the chain; at
the x walls' ends the chain climbs at 20° and the ruling leans by as
much. `regression/rounded-pocket-oblique-floor-fillet` is the concave
twin.

**What Open CASCADE builds.** The block is 18 faces, 40 edges and 24
vertices: the 10 faces of the cut block and one blend face per chain
edge, no face split. Each junction is one circle of radius 0.2. At the x
wall `x = 0`'s end `(0, 0.5)` its centre is `(0.2, 0.5, 1.105179328372)`,
its axis `(0, cos 20°, sin 20°)`, and the ruling runs from the bottom to
`z = 1.105179328372`, where it meets the circle. That centre is at 0.2
from the wall, from the cylinder and from the tilted top: the point where
the ruling moved 0.2 into the solid meets the top moved 0.2 into it, at
`z = 1.5 − 0.5 tan 20° − 0.2 / cos 20°`. It is not at the vertex's height,
`1.318`, but `0.213` below it.

## Decision

**1. Where `w` leans, the shared ball is centred on the offset of `w`.**
Name the faces at the vertex: `P` the shared face, a plane; `W` and `C` the
runs' far faces; `w` the vertex's third edge, along which `W` and `C` are
tangent. When `w` is a line, `W` and `C` share one normal `n` all along it
(a plane's, or a ruled face's along its ruling), so every centre of a ball
touching both lies on `w_r = w + σ r n`, with `σ` −1 for a convex blend
and +1 for a concave one and `n` the faces' outward normal. The shared
ball's centre is

`c* = w_r ∩ offset(P, σ r)`,

the one point at `r` from `P`, `W` and `C` at once. It is `blend::junction::leaning_ball`,
held by its tests to step 1's reading of Open CASCADE to `1e-11`. It lies on both runs'
spines exactly: the table stripe's axis is the section of its two
faces' offsets, and the walked stripe's spine is traced on the same
section.

**2. The junction arc is the great circle from `q` to `p`.** `q = c* −
σ r n_P` is `c*`'s foot on `P`, and `p = c* − σ r n` its foot on `w`. The
arc runs between them in the plane through `c*` square to `n_P × n`, exact
on both blend faces, which meet along it at a tangent dihedral as in
ADR-0035 §3. `w` is cut at `p` (`ends::cut_corner`), the vertex leaves the
body, and on `P` the two contacts meet at `q`.

**3. Both runs end at `c*`, not at the vertex.** The table stripe's `t` is
`c*`'s parameter on its axis, which may lie past its edge's range. A
walked stripe's end `v` is its spine's parameter at `c*`, which must lie
within `REACH_RADII` of the traced piece. Otherwise the junction is
refused as `VertexBlend` naming both edges and the vertex.

**4. Tried only where ADR-0035 §3's meeting fails.** The closed form
replaces the refusal: a junction whose balls already agree in the normal
plane keeps its result and ids. A square `w` puts `c*` at the normal
plane's ball, so the two readings agree where both apply.

**5. What stays `VertexBlend`.** A `w` that is not a line (an annulus
running into a torus, `blend/chain-through-torus-unsupported`); a shared
face that is not a plane; a `w` parallel to `P` (no `c*`); a walked
chamfer's junction at a leaning ruling; the vertex of four where both
faces turn (ADR-0039) with the same lean.

**6. Provenance.** No new kind. The blend faces are `Generated`; `P`, `W`,
`C` and `w` are `Modified`; the vertex is `Deleted`; the arc and its ends
`q` and `p` are `Generated` from both edges, as at ADR-0035's junction.
`Reason` gains no variant.

## Consequences

- The block, the pocket and CTC-02's ellipse chain become buildable at
  each junction of this kind; the blend network's row of real parts is
  expected to fall from 4 of 38 to 2.
- A table stripe may run past its own edge's end, or stop short of it:
  its contact lines end at `c*`'s `t`, not at the vertex's.
- One more condition on a junction (a line `w`, a plane `P`) is read
  before refusing; the faces at the vertex are read once more.

## Alternatives considered

- **Keep refusing.** The real parts stay blocked, and the construction is
  closed form with an oracle that agrees with it to its last digit.
- **A corner patch across the vertex** (a third face between the runs,
  as a three-edge corner's sphere). Open CASCADE builds none: its counts
  are one face per edge, and a patch would part from them and from the
  consumer's one-face-per-edge naming (ADR-0007).
- **Meet at the vertex and widen the tolerance.** The balls agree to
  `2.1e-4` on CTC-02, but the contacts lie `0.06` off the ruling: a
  tolerance that covered it would be a lie about the edge.
