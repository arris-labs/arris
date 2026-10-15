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
  tolerance (floored at `FRAME_ERROR_FLOOR`, 1e-13, the round-off of a few
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

⚠ OPEN: a skin per span, or per run of consecutive fitted spans (fewer
faces, no joint edges inside a spline, a coarser `PathPart`). Step 3
decides it and amends this ADR.

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
