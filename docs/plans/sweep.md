# Plan: sweep

- Started: 2026-10-15
- Milestone: C10, the NURBS cycle, second item's first half (docs/ROADMAP.md §C10)
- Idea (verbatim from the human): "sweep and loft", decided as "sweep-and-loft option A"
- Idea: `docs/ideas/sweep-and-loft.md` (the sweep half absorbed here; the
  file stays, `Status: Accepted (A)`, until the loft plan absorbs the rest)

## Goal

`ops::sweep` carries a placed `Profile` along a `geom::Path`, a G1 chain
of 3D lines, arcs, helix arcs and NURBS curves, under one of two frame
laws: **rotation-minimising** (double reflection; the default) or
**fixed binormal** (a direction, for springs and threads). The profile is
taken where the consumer placed it, so it need not be normal to the path
or touch it, as Open CASCADE's `MakePipeShell` takes it. Each span of the
path builds its own faces. A span whose frame is a translation (a line)
gives extrude's kinds, and an oblique circle or ellipse gives the exact
`EllipticCylinder`. A span whose frame is a rotation about an arc's axis,
with that axis in the profile's plane, gives revolve's kinds. Every other
span is skinned by `fit_skin` through the carried profile under ADR-0062's
budget. A bent pipe of lines and arcs is therefore all-analytic and an
operand of every quadric boolean and blend today; springs, threads and
spline paths carry NURBS faces, and refuse only NURBS × NURBS.
A closed path sweeps a body with no caps when the frame closes. A path
corner, a fold where the profile reaches past the path's radius of
curvature, a profile along its path, and a frame that does not close are
each refused by name. `extrude` builds an oblique direction through the
line span, and `SweepReason::DirectionNotNormal` is gone. Every sweep
fixture matches `MakePipeShell`, and the twin property holds: a path
given as its exact NURBS twin sweeps fitted faces that agree with the
exact spans' bodies.

## Non-goals

- **Loft.** It is the next plan in the main slot, from the idea's option A
  (section matching by arc length, ruled or smooth, closed).
- **Other frame laws**: Frenet, a guide rail, an auxiliary surface, a
  twist or scale law. Only the two above (the idea's decision 2).
- **Mitering or rounding a path corner.** Refused; the consumer rounds
  the sketch (decision 3).
- **Global self-intersection**, a path that comes back through its own
  tube. The local fold is refused; the global case is left to the
  checker where it decides the face pair (S5), and a backlog line records
  it. Open CASCADE does not detect it either.
- **NURBS × NURBS.** Two fitted side faces of one sweep meet along a rail
  the sweep builds itself; a blend on that rail, or a boolean of two
  fitted sweeps, stays `Unsupported` naming the pair until the marcher.
- **A revolve of an elliptic segment**, `SweepReason::EllipticRevolve`:
  an arc span with an elliptic profile segment is fitted, and `revolve`
  keeps its refusal.
- **A profile with a NURBS segment.** `ProfileSegment` has none; adding
  one is its own backlog line.

## Design deltas

- **ADR-0063 (step 2): a sweep is exact per span where a kind holds it,
  fitted elsewhere.** It records the path value, the two frame laws and
  the double-reflection step bound, the exact-span table, the closed-path
  rule (the frame's holonomy against the angular tolerance), the fold
  test, the roles, and the error budget of a fitted span (ADR-0062's
  split: carried section exact, skin fit, rail fit). Alternatives
  recorded: everything fitted (the idea's option B), and the marcher
  first (option C). It amends ADR-0047 with a line: the oblique extrude
  is built.
- **`arris-geom`, public, new (not a break):**
  - `Path { start: Point3, segments: Vec<PathSegment> }` beside
    `Profile`, with `PathSegment::{LineTo(Point3), ArcTo { via, to },
    Helix { axis: Axis, pitch: f64, angle: f64 }, Nurbs(NurbsCurve)}`.
    A helix arc starts at the current point; its radius is that point's
    distance from the axis and its sense is the sign of `angle`.
  - `FrameLaw::{RotationMinimizing, FixedBinormal(UnitVec3)}`.
  - `Path::frames(&self, law, tol, meter) -> Result<FrameField,
    PathError>`: a frame per arc length, exact on a line and on an arc
    under either law, double reflection elsewhere at a step bounded by
    the angular tolerance.
  - `PathError::{Gap, Corner, Degenerate, BinormalAlongTangent, ...}`,
    each naming the segment. The final variant list is ADR-0063's.
- **`arris-topo::provenance`, a break:** `Role::Sweep(PathPart)` appended
  after `PlanarFace`, with `pub struct PathPart { pub span: usize, pub
  part: SweepPart }`. A joint edge between spans `k − 1` and `k` is span
  `k`'s `StartEdge`; `EndEdge`, `EndVertex` and `EndCap` belong to the
  last span only, and a closed path has neither cap. `SweepPart` is
  unchanged. The binding's `role.rs` and `_arris.pyi` change in that
  commit.
- **`arris-ops`, public:**
  - `sweep(m, profile: &Profile, path: &Path, law: FrameLaw, control) ->
    Result<(Body, Provenance), OpError>`, re-exported by the `arris`
    facade.
  - `SweepReason` gains `SweepFolds { span, at }`, `ProfileAlongPath`
    and `FrameDoesNotClose { angle }`, and loses `DirectionNotNormal`
    (step 6). Each is a break: binding and `### Breaking` in that commit.
  - `OpError::Path(PathError)` (step 3, a break): an invalid path,
    beside `OpError::Profile`; Python `OpPathError` with `detail`. It is
    the corner's refusal too, so no `SweepReason::PathCorner`
    (ADR-0063's amendment).
  - `extrude`'s signature is unchanged; it takes an oblique direction.
- **`arris-ops::sweep` internals:** the file is split on the way in, as
  the backlog's kernel-seams line asks: one skeleton for extrude, revolve
  and the span builder.
- **Fixture grammar:** a `sweep` op in both interpreters (`recipe.py` and
  `arris-debug`'s recipe types), with the path and the law. `MakePipeShell`
  runs it, in `SetMode(false)` (corrected Frenet) or `SetMode(gp_Dir)`
  (binormal). `prop::recipe` draws it.
- **`docs/DATA-MODEL.md`:** §Profiles gains the path; §Provenance gains
  `Role::Sweep`.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — **The oracle and the fixture list.** Add the
  `sweep` op to the fixture grammar. On the Python side it runs
  `BRepOffsetAPI_MakePipeShell`, with a helix built as a 2D line on a
  cylindrical surface. On the Rust side the recipe types parse it and
  the interpreter refuses it as `Unsupported` until step 3. Commit these
  under `regression/`, each `#[ignore]`d with its Open CASCADE values:
  - `sweep-pipe-l-bend`: a circle along line, arc, line (planar);
  - `sweep-rect-s-bend`: a rectangle with a hole along line, arc, arc,
    line;
  - `sweep-oblique-circle`: a circle tilted 30° along a line (the
    `EllipticCylinder`);
  - `sweep-arc-tilted-profile`: a square whose plane misses the arc's
    axis (fitted);
  - `sweep-spring`: a circle along five helix turns, binormal the axis;
  - `sweep-thread`: a triangle along a helix, binormal the axis;
  - `sweep-spline-handle`: a rounded slot along a planar cubic;
  - `sweep-spline-3d`: a circle centred on a non-planar cubic;
  - `sweep-ring`: a closed planar path of four arcs, no caps.

  Also measure, for `sweep-spline-3d` and an unrestricted helix, how far
  corrected Frenet sits from rotation-minimising: the twist at the end and
  the area difference. Record it in this plan. It answers open question 1.

  **Measured (step 1).** A 6 × 2 rectangle centred on the path and normal
  to it, swept by `MakePipeShell` in `SetMode(false)`, against a double
  reflection frame sampled at 20 001 points (40 001 for the 5-turn helix)
  and the lateral area integrated numerically over it. Twist is the angle
  between the rectangle's long side in Open CASCADE's last section and the
  carried frame there:

  | path | twist at the end | area gap (relative) |
  |---|---|---|
  | `sweep-spline-3d`'s cubic | 0.06° | 3.6e-7 |
  | helix, radius 10, pitch 6, 1 turn | −1.16° | 3.7e-7 |
  | the same, 5 turns | −6.32° | 4.3e-7 |

  Corrected Frenet is rotation-minimising to the approximation Open
  CASCADE's pipe makes of it (its own `SetTolerance` default is 1e-4): the
  area gap sits at 1e-7, three orders under any fixture tolerance a fitted
  face holds. Checked beside it against closed forms: the spring's volume
  is within 4e-8 of `2π·turns·πr²·R`, the thread's within 6e-7, and
  `sweep-spline-3d`'s volume and area within 6e-7 and 3e-7 of the round
  tube's `A·L` and `2πrL + 2A` over the cubic's numeric length.
- [x] Step 2 **[2]** — **ADR-0063, the path and the frame laws.** Write
  the ADR. Add `Path`, `PathSegment`, `FrameLaw`, `FrameField` and
  `PathError` to `arris-geom`, with validation: the gap and the G1 test
  at every joint against the model's tolerances, and closure. Each segment
  is evaluated by arc length; the helix is in closed form, a NURBS segment
  by its own arc-length map. Double reflection is stepped so that the
  frame's error stays under the angular tolerance. Tests against closed
  forms:
  - on a circle the rotation-minimising frame is the rotation about its
    axis;
  - on a helix it turns against Frenet's by `−τ·s`;
  - a fixed binormal frame on a helix about that binormal is a screw
    motion;
  - the holonomy of a closed planar path is zero, and of a closed
    non-planar one it matches the enclosed solid angle on the tangent
    sphere.

  Rustdoc examples on every public item.
- [x] Step 3 **[3]** — **The fitted span, for every span.** Add
  `ops::sweep` and `Role::Sweep(PathPart)`. The role change is a break,
  so `role.rs`, the stub and `### Breaking` go in this commit. In this
  step every span is fitted; step 5 specialises. For each span:
  - Each profile segment becomes a NURBS section (a line, or an exact
    circle or ellipse), carried by the frame and skinned by `fit_skin`
    under ADR-0062's budget. A helix span is cut at quarter turns, or
    finer if the fit asks for it.
  - Each profile vertex's rail is fitted once and shared by its two
    faces. The rail's pcurves are the faces' isolines, and the rail's
    tolerance is the measured deviation.
  - Joint edges are the profile segment carried exactly; caps are planar
    faces.
  - A closed path whose frame closes needs no caps and joins its last
    span to its first.

  Add `Meter` checkpoints and `cancel_counts.txt` entries. Tests:
  - the checker is green on every case;
  - a straight degree-1 NURBS path's sweep agrees with `extrude`, and an
    exact NURBS arc path's with `revolve`, in volume, area and centroid
    within the fixture tolerance;
  - `sweep-spring`, `sweep-thread`, `sweep-arc-tilted-profile` and
    `sweep-spline-handle` leave `regression/` for `sweep/`, matching Open
    CASCADE.

  The unknowns established here: a fit that converges over five turns,
  and rails that the two faces' isolines agree with inside the edge
  tolerance.

  **Built (step 3).** Each loop is one rational quadratic NURBS, skinned
  once per span (quintic along `v`) and split at its vertex knots, so a
  rail is the shared column of both faces' nets and a joint's edge is the
  segment's NURBS carried there: every pcurve on a fitted span is an
  isoline, exact (ADR-0063's amendment). The analytic joint edge with
  fitted pcurves was tried first and dropped: a four-arc ring's mass
  properties took 19 s against 0.09 s. A five-turn spring is one face of
  256 spans, its volume within 1e-12 of the closed form; no quarter-turn
  cut. An invalid path is `OpError::Path`. `sweep-spline-3d` left
  `regression/` here too, being all-fitted. Open CASCADE's pipe misses the
  closed forms by 4e-8 to 6.3e-7 on these fixtures, so each carries
  measure tolerances over that miss, and `tests/sweep.rs` holds Arris to
  the closed forms. The spring meshes at a chord of 1e-2: 620 000
  triangles at the default take half a minute a mesh.
- [x] Step 4 **[2]** — **The refusals.** Add the new `SweepReason`
  variants. The binding carries a reason as text, so no Python class or
  stub line follows (ADR-0063's amendment).
  A corner is already `OpError::Path(PathError::Corner { segment, at,
  angle })` from step 3; its fixture is added here with the others.
  - `ProfileAlongPath`: the profile's plane holds the start tangent
    within the angular tolerance;
  - `SweepFolds { span, at }`: along a curved span, the profile's
    farthest reach along the curvature normal is at least the radius of
    curvature. The test samples the span with a bound on the curvature
    between samples, and on an arc it coincides with revolve's
    `ProfileCrossesAxis` and `SpindleTorus`;
  - `FrameDoesNotClose { angle }`: a closed path whose holonomy exceeds
    the angular tolerance under the rotation-minimising law.

  For each, a fixture under `sweep/` asserting the refusal, its fields
  and no panic, plus `sweep-fold-near-limit` just inside the bound,
  which builds and is checker-green.

  **Built (step 4).** `ops/src/sweep/refuse.rs`; fixtures
  `sweep-path-corner`, `-profile-along-path`, `-fold`,
  `-fold-near-limit`, `-frame-does-not-close`, each with its
  `expect_error` (four new `ExpectError` kinds; the fields are held in
  `tests/sweep.rs`). The oracle records no genus for the closing case,
  whose pipe has an odd Euler characteristic, so `fixture.py` relaxes the
  manifold check for it as it does for `non-manifold`. The skew loop's
  fixture numbers are full f64: rounded to 12 digits a joint's tangent
  was 6.6e-12 off, above the angular tolerance of 1e-12.
- [x] Step 5 **[2]** — **The exact spans.** Two cases leave the fitted
  path:
  - **A line span** builds extrude's faces through the shared skeleton.
    The skeleton's oblique case is new: a circle or ellipse gives
    `EllipticCylinder`, with its pcurves, and a line gives a plane.
  - **An arc span** builds revolve's faces when the frame law's motion
    is a rotation about the arc's axis and the profile's plane holds that
    axis.

  Joints between an exact span and a fitted one share the exact carried
  curve. `sweep-pipe-l-bend`, `sweep-rect-s-bend`, `sweep-oblique-circle`
  and `sweep-ring` leave `regression/` (all but the oblique circle already
  build fitted and match; their dumps are blessed here). Open CASCADE's
  area for `sweep-oblique-circle` is 5.6% high, so that fixture moves with
  `analytic.measure_differs` and the closed forms (ADR-0063's amendment). The twin test
  becomes a property here: the same path with each line and arc given as
  its exact NURBS sweeps fitted faces that agree with the exact build in
  volume, area and centroid.

  **Built (step 5).** `ops/src/sweep/exact.rs` classifies each span by its
  segment and checks the field's motion against the translation or the
  rotation it must be (ADR-0063's amendment); `along.rs` builds an exact
  span's sides from its surfaces and rails and a fitted one's as before. A
  ring is the profile's analytic segment only where every span it joins is
  exact, else the segment's NURBS. Arc spans reuse `revolve`'s surface
  selection (its helpers are `pub(super)` now) and are fitted when the
  carried plane misses the axis, a point lies on it, or a segment is
  elliptic. An oblique circle or ellipse gives an `EllipticCylinder` for
  every tilt: its seam is the vertex's rise wherever that stands, and the
  ring edge's pcurve on it is `pcurve_on`'s fitted NURBS, so no tilt falls
  back. The first `measure_differs` fixture whose *measure*, not its
  shape, Open CASCADE gets wrong needed `analytic.occt_file_measure_differs`
  to hold the read-back stages to the closed forms (the plan did not
  foresee it). `sweep-fold-near-limit` now builds a torus of minor radius
  0.95 on major 1, and `mass_properties` integrates its inertia 4.3e-9
  off the closed form (a `revolve` of the same disc does too), so that
  fixture's `inertia_rel` is 1e-8 (docs/BACKLOG.md). The twin property is
  `tests/sweep_prop.rs`, planar paths of lines and arcs with a rectangle
  or a disc; step 8 extends it.
- [x] Step 6 **[1]** — **The oblique extrude.** `extrude` builds any
  direction off the profile's plane through the line span. A direction
  whose normal component times the length is under the linear tolerance
  is `InputReason::ZeroThickness`. `SweepReason::DirectionNotNormal` is
  removed: binding, stub and `### Breaking` in this commit. The existing
  extrude fixtures are unchanged. Add `sweep/extrude-oblique-slot` with
  its oracle values, then retire the backlog line and M5's ⚠ OPEN 3.

  **Built (step 6).** `extrude` routes side surfaces through `exact::line_span`:
  an oblique extrude of an arc or ellipse sweeps an exact `EllipticCylinder`,
  and in-plane or near-zero-thickness directions return `InputReason::ZeroThickness`.
  `SweepReason::DirectionNotNormal` removed; `sweep/extrude-oblique-slot` fixture
  committed with oracle values, closed-form area and `measure_differs`. Backlog line
  and M5 ⚠ OPEN 3 retired.
- [ ] Step 7 **[2]** — **The swept body as an operand.** Fixtures with
  Open CASCADE values, each a chain from a sweep:
  - fillet the rail edges of `sweep-rect-s-bend` (all analytic);
  - fuse two L-bend pipes that cross;
  - cut `sweep-spring` with a box and split it by a plane (NURBS ×
    plane);
  - section `sweep-spline-handle`;
  - chamfer a rail of `sweep-thread`, refused `Unsupported` naming the
    two NURBS faces.

  Each builds and matches, or is refused by name. A failure found here
  becomes a `regression/` fixture and a backlog line, not a fix in this
  step, unless the sweep built the faulty input.
- [ ] Step 8 **[2]** — **Properties.** `sweep_prop` with `prop_shards!`
  over random G1 paths (lines and arcs, planar and not, random helix
  spans) and random profiles from the existing strategy:
  - the checker is green, or the sweep is a named refusal;
  - rigid motion: move the profile and path, then sweep, equals sweep,
    then move;
  - splitting the path at an interior point adds volumes;
  - a line path agrees with `extrude`;
  - the exact and NURBS twins agree;
  - the STEP round trip keeps volume and counts.

  `prop::recipe` and the differential draw `sweep`; record the 1000-draw
  tally in the roadmap's harness paragraph at retirement.
- [ ] Step 9 **[1]** — **The binding and the facade.**
  - `arris.sweep(profile, path, law=...)`, with `Path`, `PathSegment` and
    `FrameLaw` classes, `PathError` mapped, stubs and docstring examples;
    pytest for a pipe and a spring.
  - The facade re-exports `sweep`, `Path`, `PathSegment` and `FrameLaw`,
    with a facade-level doc example.

## Acceptance

`ARRIS_GATE=full` green, with:

- the nine sweep fixtures, the refusal fixtures and
  `extrude-oblique-slot` passing under `tests/fixtures/sweep/`, each
  matched to `MakePipeShell` within its tolerance;
- step 7's operand fixtures building and matched, or refused by name;
- `sweep_prop`, the twin property and the differential green at 256
  cases;
- no `sweep` fixture left in `regression/`;
- the Python job's pytest, docstrings and stubtest green.

## Docs to update on completion

- `docs/DATA-MODEL.md` §Profiles: the path value, its validation and the
  frame laws. §Provenance: `Role::Sweep(PathPart)` and the joint-edge
  naming.
- `docs/ARCHITECTURE.md` §Operations: `sweep`, the span table (exact or
  fitted), the fold and closure refusals, the oblique extrude. §Errors:
  the new `SweepReason`s.
- `docs/ROADMAP.md` §C10: the sweep half of the second item done, with
  the date and the fixture counts. M5's ⚠ OPEN 3 closed by ADR-0063. The
  harness paragraph gets the differential tally with `sweep` drawn.
- `docs/BACKLOG.md`: retire the oblique-extrusion line. Add global
  self-intersection of a sweep, a NURBS profile segment, and other frame
  laws, if anything asks for them.
- `docs/adr/0047-…`: the amendment line, if step 2 did not write it.
- `docs/ideas/plugin-cad-consumer-asks.md` status line: A7's sweep
  landed, loft open.
- `docs/ideas/sweep-and-loft.md`: status note that the sweep landed. The
  loft plan deletes the file.
- `tools/oracle/README.md`: the `sweep` op and its modes.
- `CHANGELOG.md` `Unreleased`: sweep along a path, the oblique extrude,
  the refusals. The `### Breaking` bullets are already written by
  steps 3, 4 and 6.
- `AGENTS.md` current state: sweep landed; next is loft.

## Open questions

- ✔ OPEN 1 (answered by step 1: the measured gap is 4e-7, so
  `sweep-spline-3d` and the random non-planar properties keep the area,
  and no `analytic.measure_differs` is needed). Asked as: on a non-planar path, Open CASCADE's
  corrected Frenet is not exactly rotation-minimising. If step 1
  measures an area gap above the fixture tolerance, hold
  `sweep-spline-3d` and the random non-planar properties to volume only
  (the profile is centred on the path, so the volume does not depend on
  twist). Record the area through `analytic.measure_differs` with
  ADR-0015's evidence, or switch the fixture to the binormal law.
- ✔ OPEN 2 (answered by step 2, ADR-0063 §3: the field carries the arc-length
  map, a table inverted by Newton, so a fitted span's `v` is the path's arc
  length). Asked as: does the frame field carry the arc-length
  map of a NURBS path segment, or the curve's own parameter with a speed?
  This decides what a fitted span's `v` is. ADR-0063 records the choice.
- ✔ OPEN 3 (answered by step 3, ADR-0063's amendment: one skin per
  span, as `MakePipeShell` builds a face per spine edge). Asked as: one
  skin per span, or one per run of
  consecutive fitted spans. One per run means fewer faces and no joint
  edges inside a spline. It also makes span-level roles coarser, so
  `PathPart` would name the run's first span.
