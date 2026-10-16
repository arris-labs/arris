# ADR-0063 — A sweep is exact per span where a kind holds it, fitted elsewhere: the path is a G1 chain, the frame is one of two laws, the field is indexed by arc length

- Status: accepted (2026-10-15)
- Plan: `sweep` step 2 (the path, the laws and the field); the fitted span
  lands in step 3, the refusals in step 4, the exact spans in step 5
- Amends: ADR-0047 §Decision (sweep along a path "needs free-form
  surfaces": a span whose frame is a translation or a rotation about the
  arc's axis is analytic) and ADR-0014 (an oblique circle or ellipse
  swept along a line is the exact `EllipticCylinder`, so `extrude` takes
  any direction off the profile's plane)
- Follows: ADR-0062 (the error budget of a fitted face, `fit_skin`
  through carried sections), ADR-0014 (the elliptic cylinder), ADR-0020 §1
  (closure: a swept body is an operand of every boolean and blend the
  NURBS cycle's first item allows)
- Reference trees read: none. Open CASCADE is run as the oracle
  (`BRepOffsetAPI_MakePipeShell` through `tools/oracle/`), not read

## Context

The consumer's largest open ask is a sweep along a path (pipes, handles,
springs, threads). `extrude` and `revolve` are the two sweeps whose faces
have a kind; a profile carried along anything else has a surface no
analytic kind holds, and a fitted NURBS holds. The idea
(`sweep-and-loft`, option A) is to build each span of the path the
cheapest exact way it can be built, and to fit only where no kind
exists, so that a bent pipe of lines and arcs is all-analytic and an
operand of every quadric boolean today.

Two questions decide the rest. What is a path, so that every kind of span
can be told apart and a corner refused by name? And how does the profile
turn along it? Open CASCADE's `MakePipeShell` is the oracle and offers
corrected Frenet, a fixed binormal, a guide and an auxiliary surface. The
plan's step 1 measured corrected Frenet against a rotation-minimising
frame on `sweep-spline-3d`'s cubic and on a helix of radius 10 and pitch
6: the lateral area differs by 3.6e-7 to 4.3e-7 and the end section
twists by 0.06° to 6.3°. Corrected Frenet is rotation-minimising to Open
CASCADE's own approximation tolerance.

## Decision

**1. The path is a value, a G1 chain** (`arris_geom::Path`): a start
point and segments `LineTo`, `ArcTo { via, to }` (three points, the arc
longer than a half turn when they order that way), `Helix { axis, pitch,
angle }` from the current point (radius its distance from the axis,
sense the sign of `angle`, advance `pitch · angle / 2π`) and `Nurbs`. It
lives in `geom` beside `Profile`, plain data with no orientation or
validity of its own.

**2. Validation is `Path::frames`**, against a `Tolerance`: a segment no
longer than the linear tolerance, an arc whose via point is on its
chord's line within it, a helix on its axis or with no turn, and a NURBS
segment that starts away from where the path has reached
(`Degenerate`, `Gap`, each naming the segment). Every joint must be G1
within the *angular* tolerance — a corner is `Corner { segment, at,
angle }`, never mitered or rounded (the consumer rounds the sketch). A
C0 knot inside a NURBS segment is tested by its two one-sided tangents
and is a corner too. A path that ends within the linear tolerance of its
start is closed, and must be G1 at the closing joint as well (segment 0
names it).

**3. The field is indexed by arc length, on every kind of segment.** Open
question 2 of the plan: a NURBS segment's frames are evaluated at the
arc length, not at the curve's parameter with a speed. The field keeps
a table of the curve's parameter against arc length (accepted steps of
the frame integration, below) and inverts it by Newton on a 16-point
Gauss–Legendre arc length from the entry before. Then a fitted face's
`v` is the path's arc length whatever the segment kind, an exact
segment and its NURBS twin share it, and the twin property compares like
with like. The cost is the inversion per evaluation, microseconds.

**4. Two frame laws** (`FrameLaw`), both giving a right-handed
`(tangent, normal, binormal)` with `tangent × normal = binormal`:
- **Rotation-minimising** (the default): the normal turns no more about
  the tangent than the path's bending forces. Closed forms where they
  exist: a line keeps its normal, an arc rotates it about the arc's axis
  (the exact rotation about the axis), and a helix turns it against the
  Frenet normal by `−τ·s` with `τ = c / (r² + c²)`, `c = pitch / 2π`
  (derived from `U′ = −κ cos ψ T + (ψ′ + τ) V`, `ψ′ = −τ`). Elsewhere
  *double reflection* (Wang, Jüttler, Zheng, Liu 2008), a method of
  fourth order, in steps chosen by step doubling: a step is accepted when
  the angle between one step and two half steps is under the angular
  tolerance (floored at `FRAME_ERROR_FLOOR` (in `arris_math::sweep_steps`), 1e-13, the round-off of a few
  hundred reflections) and the tangent has turned by at most
  `FRAME_MAX_TURN` (0.05 rad). A step under `FRAME_MIN_STEP` of its knot
  interval is a cusp: `FrameStep`. An evaluation between entries is one
  reflection pair from the entry before, so it is as accurate as the
  step it lies in.
- **Fixed binormal**: the binormal is the given direction projected
  perpendicular to the tangent, the normal `binormal × tangent`. A
  pointwise formula: no stepping and no error. On a helix about that
  direction it is the exact screw motion, which is what a thread and a
  spring need. The direction along the tangent anywhere on a span is
  `BinormalAlongTangent { segment, at }`, found exactly on a line, an arc
  and a helix (where `b · T` is a sinusoid in the turn angle plus a
  constant, so its extremes are at the ends and the sinusoid's own) and
  at every table entry on a NURBS segment.

The frame law is the *profile's motion*: the profile is placed where the
consumer put it, and `FrameField::motion_at(s)` is the rigid motion from
the first frame to the frame at `s`. It does not depend on the first
normal the field picked (rotation-minimising transport commutes with a
rotation about the start tangent), so a profile need not be normal to the
path or touch it, as `MakePipeShell` takes it. Other laws — Frenet, a
guide rail, an auxiliary surface, a twist or scale law — are not built.

**5. A closed path closes its frame, or is refused.** Under the
rotation-minimising law the frame returns rotated about the tangent by
the *holonomy*, the solid angle the tangent indicatrix encloses modulo
2π (`FrameField::holonomy`, in `(−π, π]`). It is zero for a planar path.
The sweep (step 4) refuses a closed path whose holonomy exceeds the
angular tolerance as `FrameDoesNotClose { angle }`; the fixed binormal
closes by construction. The trefoil in the geometry tests has a holonomy
of 3.03 rad, matched to the numerical solid angle of its tangent
indicatrix to 1e-5. (A closed curve whose indicatrix is a simple loop has
none: Jacobi's theorem has it bisect the sphere.)

**6. Each span builds its own faces** (steps 3 and 5; recorded here so the
path is designed for them).

| Span | Frame's motion | Faces |
|---|---|---|
| line | a translation | extrude's kinds, the oblique circle or ellipse the exact `EllipticCylinder` |
| arc | a rotation about the arc's axis, the axis in the profile's plane | revolve's kinds (planes, cylinders, cones, tori) |
| everything else | any | `fit_skin` through the carried profile, ADR-0062's budget |

A joint between two spans is the profile carried exactly, shared by the
faces of both. A fold, where the profile reaches past the radius of
curvature of a curved span, is refused (`SweepFolds`); a profile along
its path is `ProfileAlongPath`; a global self-intersection is left to the
checker where it decides the pair.

**7. The error budget of a fitted span** is ADR-0062's split: the carried
section exact (a line, or an exact circle or ellipse), the skin fit under
`SECTION_FIT_FRACTION` of the tolerance, each profile vertex's rail fitted
once and shared by its two faces, the rail's tolerance the measured
deviation. A helix span is cut at quarter turns, finer if the fit asks.

**8. Roles** (`Role::Sweep(PathPart)`, step 3, a break): `PathPart { span,
part }` with the existing `SweepPart`. A joint edge between spans `k − 1`
and `k` is span `k`'s `StartEdge`; the end cap belongs to the last span,
and a closed path has neither cap.

A skin per span, not per run of consecutive fitted spans: decided in
step 3, see the amendment.

## Consequences

- `arris-geom` gains `Path`, `PathSegment`, `FrameLaw`, `FrameField`,
  `PathFrame`, `PathError` and `PathDegeneracy`: public, new, not a break.
- A bent pipe of lines and arcs is all-analytic; springs, threads and
  spline paths carry NURBS faces and refuse only NURBS × NURBS until the
  marcher.
- The oracle is `MakePipeShell` in the two modes the plan's fixtures use;
  where corrected Frenet differs from rotation-minimising by more than
  the area tolerance on some path, the fixture holds volume only, and the
  random properties are held to volume on non-planar paths.
- A joint tolerance of the model's *angular* tolerance (1e-12 by default)
  is strict for a hand-typed path; the sketch's arcs and lines are
  computed in double precision and meet it, and a consumer who types
  endpoints to six digits sets a looser angular tolerance for the model.

## Alternatives considered

- **Everything fitted** (the idea's option B). One code path, but a bent
  pipe of lines and arcs would be NURBS faces, closed to every quadric
  boolean and blend until the marcher, against ADR-0020 §1.
- **The marcher first** (option C). Delays a feature the consumer is
  blocked on by a cycle.
- **Corrected Frenet as the law.** Open CASCADE's own, but it is
  rotation-minimising only to its approximation tolerance, and it has no
  closed form on an arc or a helix. Kept as the oracle.
- **A frame field in the curve's own parameter** (open question 2). No
  inversion cost, but the fitted face's `v` would differ between a
  segment and its NURBS twin, and joints between spans of different kinds
  would reparametrise.

## Amendment (2026-10-15, plan `sweep` step 3)

- **A skin per span** (the ⚠ OPEN of §8). One face per span and profile
  segment keeps `PathPart` exact, and is what `MakePipeShell` builds: it
  too makes a face per spine edge, so the counts match the oracle's. A
  run of NURBS segments would save its inner joints only, and a spline
  path is one segment in every fixture.
- **One skin per loop, split at the vertices.** Each loop of the profile
  is one rational quadratic NURBS: a line raised from degree one, an arc
  or an ellipse its exact arcs, consecutive segments sharing the control
  point at their vertex with a knot of multiplicity two there. The loop is
  carried and skinned once per span (`fit_skin`, quintic along `v`,
  within `SECTION_FIT_FRACTION` of the tolerance), and the net splits at
  those knots exactly into one surface per segment. Two neighbouring side
  faces end on one column of control points, and that column is their
  rail: the isoline of both, with nothing to measure. §7's "the rail's
  tolerance is the measured deviation" is zero by construction, and the
  rail has the edge's default tolerance.
- **A joint's edge is the segment's NURBS carried to it.** The skin
  interpolates its end sections, so the segment's NURBS carried to a
  joint is the isoline there of the faces on both sides. On a cap it is
  planar, so its pcurve is its control points in the cap's (u, v). Every
  pcurve of a fitted span is exact. The analytic curve (§6's "the profile
  carried exactly") would put a fitted pcurve on every side face: a
  rational arc's parameter is not the circle's angle, so `pcurve_on` fits
  `u(θ)` with a hundred spans per quarter turn. That made a four-arc
  ring's mass properties take 19 s, against 0.09 s with the isolines.
  Where an exact span meets a fitted one (step 5), one side of the joint
  has a fitted pcurve either way.
- **A helix span is one face.** §7's cut at quarter turns is not made:
  the quintic skin refines its own knots, and a five-turn spring of
  radius 10 fits in 256 spans, within 1e-12 of the screw's closed-form
  volume.
- **An invalid path is `OpError::Path(PathError)`**, beside
  `OpError::Profile`: like a sketch, a path has no entities to name, and
  `PathError` already names the segment and, for a corner, the arc length
  and the angle. A stop while the frames are built is
  `OpError::Interrupted`. Step 4's `SweepReason::PathCorner` is therefore
  not added: `PathError::Corner` is that refusal.
- **Open CASCADE is the less exact side on a fitted sweep.**
  `MakePipeShell` approximates to its own `SetTolerance` (1e-4), so on the
  spring, the thread and the two splines its volume lies 4e-8 to 6.3e-7
  from the closed forms, where Arris' lies within 1e-11 to 2e-9. Those
  fixtures carry measure tolerances over that miss, under ADR-0062's 1e-5
  for a fitted fixture, and `crates/arris-ops/tests/sweep.rs` holds Arris
  to the closed forms. On `sweep-oblique-circle` Open CASCADE's area is
  5.6% high (90π, the circle's perimeter times the length, against the
  projected ellipse's 267.86322590949, which Arris matches to 1e-15); step
  5 holds that fixture to the closed forms through `measure_differs`.

## Amendment (2026-10-15, plan `sweep` step 4)

- **The fold test is the reach along the curvature normal.** A point
  carried at offset `d` from the path moves along the tangent at speed
  `1 − κ (d · n)`, whatever the frame's twist, so the sweep folds where
  `κ · reach ≥ 1` and `reach` is the outer loop's farthest extent along
  the curvature normal, pulled back to the start by the motion. The
  reach is exact on a loop (the support function of its lines, circles
  and ellipses). The curvature is a central difference of the field's
  tangent; each span is cut into sixteen and an interval whose ends sit
  over half the limit is halved while the ratio still moves, ten times at
  most. The margin is `κ` times the linear tolerance: a profile within
  `tolerance.linear` of the radius of curvature is refused, as revolve
  refuses a profile within it of the axis. Between samples the test is
  sampled, not bounded; a fold narrower than the last interval is left
  to the checker.
- **A hole is not tested**: it lies inside the outer loop.
- **`ProfileAlongPath`** is the start tangent within the angular
  tolerance of the profile's plane. **`FrameDoesNotClose { angle }`** is
  the holonomy's absolute value, for a closed path under the
  rotation-minimising law only.
- **No Python class is added.** The binding carries a `Degenerate`
  refusal's reason as text, so the three variants reach Python through
  `Display` with no new class or stub line.


## Amendment (2026-10-15, plan `sweep` step 5)

- **A span is exact where its motion is one of the table's two and the
  faces then need no fit; otherwise it is fitted whole.** A span is
  classified by its segment (`LineTo`, `ArcTo`; a helix or a NURBS segment
  is always fitted, so a NURBS twin of a line or an arc is the fitted
  control) and by a check, not an assumption: the field's motion from the
  carried start to the carried end must be the translation along the
  tangent, or the rotation about the arc's circumcircle axis by its
  length over its radius, within the angular tolerance and the linear
  tolerance at the profile's origin. Under a fixed binormal that is not an
  arc's axis the check fails and the arc is fitted.
- **A line span** is `extrude`'s faces on the carried profile: a plane per
  line, a `Cylinder` for a circle normal to the path, an
  `EllipticCylinder` for an ellipse normal to it. A circle or ellipse
  *oblique* to the path gives an `EllipticCylinder` of the section across
  the path: the conic projected along the tangent is an ellipse whose axes
  are the singular directions of the 2 × 2 projection, and the cylinder's
  `X` is its major axis. Its seam is wherever the vertex's rise stands, not
  necessarily at `u = 0`, the ring edge's pcurve on it a fitted NURBS
  (`pcurve_on`'s oblique section), so the exact kind holds for any tilt.
- **An arc span** is `revolve`'s faces on the carried profile when the
  carried plane holds the arc's axis and every point of every loop is off
  it by more than the linear tolerance, and no segment is elliptic
  (`EllipticRevolve`): planes, cylinders, cones, spheres and tori by
  `revolve`'s own surface selection, a vertex running a circle about the
  axis. Otherwise — a tilted profile, a profile touching the axis, an
  elliptic segment — the span is fitted.
- **A joint (ring) is the profile's own segment carried, an analytic
  curve, only when every span it joins is exact** (and for an open path's
  end ring, its one span). Beside a fitted span it is the segment's exact
  NURBS, the isoline the fitted faces end on; the exact span's faces then
  take a fitted pcurve for it. The analytic ring with fitted pcurves
  stays rejected (step 3's 19 s).
- **`sweep-oblique-circle` is held to the closed forms** through
  `measure_differs`: Open CASCADE's pipe takes the lateral area as the
  circle's perimeter times the length. Arris' own reading of Open
  CASCADE's STEP of it, the oblique cylinder, measures the closed form to
  4e-14, so it is the *measurement* of its own shape that is wrong, not
  the shape; the new `analytic.occt_file_measure_differs` holds the
  read-back stages to the closed forms where every other
  `measure_differs` blames a shape its file carries.
