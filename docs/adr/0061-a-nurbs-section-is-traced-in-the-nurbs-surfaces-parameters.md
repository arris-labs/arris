# ADR-0061 — The section of a NURBS surface with an analytic one is traced in the NURBS surface's (u, v), patch by patch, as a torus's is

- Status: accepted (2026-10-13)
- Plan: `nurbs-analytic-section` step 2 (the decision and the isolation);
  the walk (step 3, amended below), the isolines and the intersector's
  arms land in its steps 3 to 5, and step 7 adds the pcurve's source
  here as an amendment
- Follows: ADR-0019 (the torus traced in its own (u, v): the isolation,
  the cells, the singular-point bump, the tube circle divided out), whose
  machinery this reuses unchanged
- Closes, once step 5 lands: the `Unsupported` arms of
  `docs/DATA-MODEL.md` §Curves for a NURBS surface against a plane, a
  cylinder, an elliptic cylinder, a cone, a sphere and a torus
- Reference trees read: none. Open CASCADE is run as the oracle
  (`GeomAPI_IntSS`, through `tools/oracle/`), not read

## Context

Step 1's census (the plan's table) puts four of the real-part battery's
refused stages within reach: CTC-02's box cut and drill and CTC-05's box
cut in both editions. Each is a NURBS face against a plane or a cylinder.
Every surface pair the stages ask is NURBS × analytic, and one curve pair
is too, a circle against a NURBS face. The faces are bicubic B-splines of
up to 16 × 4 spans and a rational degree-(3, 2) patch. One face the
box reaches has a collapsed row.

ADR-0019 traced a torus in its own (u, v): the other surface's implicit
`F` is put into the torus's rational parametrisation, patch by patch, and
the zero set `f(u, v) = 0` is walked between certified turning and
critical points. Nothing in that machinery is particular to a torus
except three things: the patches (sixteen quarter turns), the chart from
a patch's `(s, t)` to the angles, and the closure (both directions close
over `2π`). A NURBS surface is a set of rational Bézier patches already.

## Decision

**The section of a NURBS surface with an analytic surface is traced in
the NURBS surface's own (u, v)**, by putting each of its Bézier patches
into the other surface's implicit form: `Implicit::along` over the
patch's homogeneous net moved into the other surface's frame. The result
is a tensor Bernstein polynomial of bidegree `(d·p, d·q)`, with `d` the
other surface's degree: (3, 3) to (12, 12) for a bicubic patch. The
turning points, the critical points, the cells and the singular-point bump
of ADR-0019 then run on it unchanged. `trace_torus::PatchedSection` holds
the patches of either kind behind a `Chart`. A torus's chart is the
quarter turns, `4·qu + qv`, its `s` an angle through the half-angle map.
A NURBS surface's chart is its knot spans, row-major over `u`, its `s`
linear over the span.

Four things a torus never had to decide:

1. **The domain's edge.** A NURBS surface is not extrapolated past its
   domain. A branch that reaches an edge of a direction that does not
   close ends there. Its end is the root of `f` along that edge line.
   That is the same equation the edge's own curve meets the other surface
   by, so the boolean's pave and the section agree on the vertex. Every
   root on an edge line seeds a march, as `u = 0`'s roots seed a torus's
   winding components. `seam_candidates` is the first row of `u` on
   either chart. The walk takes the other three edges as well (step 3).
2. **Closure per direction**, read from `NurbsSurface::closure`. A
   periodic direction and a clamped one whose ends meet (a full turn of a
   revolution) both close, over the closure's length from the start of
   the domain. A torus has two seams; a NURBS surface has zero, one or
   two. `bernstein2::merge_wrapped` merges two patches' finds across a
   seam per direction, from the domain's start rather than from zero.
   `merge` is that with both directions closing over the same period
   from zero, the torus's case, bit for bit.
3. **An isoline on the other surface**, the analogue of the tube circle:
   a whole row `u = u₀` or column `v = v₀` on which `f` vanishes within
   `near`. It is found first and returned as the exact isocurve (knot
   insertion, no fit). Then it is divided out of the patches it crosses,
   as `deflate` divides out a tube circle, and crossing branches end on
   it. The candidates are the patch edges and the sign changes along a
   row. A NURBS surface on which `f` vanishes over a whole patch is
   coincident with the other surface over an area, and is refused as
   `Unsupported` (step 4).
4. **A collapsed row**, one point in space (a pole, an apex), is **in
   scope** (step 1: CTC-02's box reaches one, and a loft to a point makes
   one). On the row `P_s ≡ 0`, so the critical points' first numerator
   `w^(d+1)·∂F/∂s` vanishes all along it. Where the point is off the
   other surface, `f` stays clear of `near` there and the gate drops the
   row. Where it is on it, the row is an isoline of `f`, divided out as
   (3) divides one. It is not returned as a curve: it is one point, a
   singular point of the section at which every branch that reaches the
   row ends, as a sphere's pole is (step 5).

**The rounding floor charges the move into the other frame.** A torus's
patches are built from its frame in the other frame, one rigid offset, so
its floor is `BERNSTEIN_ROUNDING` times the polynomial's magnitude at the
patch's reach. A NURBS net is moved point by point, and each point keeps a
rounding of `MOVE_ROUNDING` (4ε) times its terms before they cancelled:
`|R·X| + w·|shift|`, the net's distance from the other frame's origin and
back. Every coefficient of `f` is a sum of terms each bounded by a
monomial of the coordinates' bound. So the floor adds what the magnitude
grows by when the reach grows by that rounding: linear in it, not raised
to the degree. `steep`, what `tol.linear` makes of `f`, is the other
surface's steepness at the farthest control point times the heaviest
weight to the degree, since `f = wᵈ·F`.

### What step 2 measured, recorded as the decision's grounds

- **The twins turn where their analytic surfaces do.** The exact
  `to_nurbs` twin of a torus against the spiric planes gives the closed
  form's four turning points to 1e-12. The figure eight's crossing and
  the outer touch, both on the twin's seam `u = 0`, are one certified
  singular point each, merged across the seam of a full turn that closes
  without being periodic. A cylinder's twin against an off-axis sphere
  turns at `z = 0`, `cos θ = (1 + e² − R²)/2e`, posed a kilometre away,
  and its internal tangency is one singular point. A sphere's twin
  against a plane and a cylinder turns on the equator where the closed
  form says and nowhere near its poles. A bicubic saddle `z = u² − v²`,
  its saddle the corner four patches share, against the plane at its
  height is one certified singular point at the corner, merged from four
  patches. Above it the hyperbola turns at `u = ±√h`. Every patch
  of every twin, against a plane, a cone and a torus, equals `wᵈ·F` at
  the surface's point within four floors.
- **Degree blow-up does not need a prefilter.** CTC-02's two largest
  faces the census reached, 64 bicubic patches each
  (`crates/arris-geom/testdata/ctc02-bicubic-faces.txt`), against the
  plane and the drill that meet them and against a torus whose tube cuts
  each in two loops, bidegree (12, 12): every turning point is certified
  and within 1e-14 of the other surface. The worst isolation is 292 boxes
  to a depth of 12, about 5 ms in a test build. A patch the other surface
  misses is excluded at depth 0 on its coefficients' signs, so a box
  test on the control hull first would save only the substitution, which
  is microseconds. That is within ADR-0019's measured bound for a
  torus's sixteen patches, 740 boxes and 25 halvings, and the torus's
  `MEASURED_BOXES` (2000) and `MEASURED_DEPTH` (32) hold it in the test.
- **The floor is the finding.** With the move's rounding charged as the
  coordinate's size instead — the quartic of a kilometre's reach — the
  torus's turning points on CTC-02's faces came back as four uncertified
  boxes off the section by up to 0.23 mm. The floor (0.9) was above
  what `f_t` varies by near a turning point, so the isolation could
  neither exclude nor certify there. Charged linearly it is 2.6e-4, and
  all four certify.
- **The torus is unchanged.** Its census and its tests pass as they
  were, and so do the 27 corpus fixtures named for a torus, an elbow, a
  bead, a dome or a ring, against their blessed dumps.

### Step 3: the walk on a bounded domain (amendment, 2026-10-13)

- **The walk's parameters.** Each direction of a NURBS surface's domain
  (its closure where it closes) is mapped linearly onto `[0, 2π]`
  (`trace_torus::Rescale`), so a closed direction closes over `2π` as a
  torus's angles do, and every size the torus walk measures in radians —
  its first cell, its longest step, its slacks — is the same share of any
  surface's domain whatever its knots' units. The torus's own angles are
  walked as they are, bit for bit.
- **Edge cells** decide (1). Every root of `f` along an edge line of a
  direction that does not close is proven alone in a bracket on the
  edge's own polynomial and given a cell with one arm into the domain:
  on an edge of `u` a march's cell (`∂f/∂t` of one sign, `f` of opposite
  signs along its lower and upper edges); on an edge of `v` the half
  inside, with `∂f/∂t` and `∂f/∂s` of one sign and `f` of one sign along
  its inner edge, so the arm leaves through the side where `f` on the
  edge has the other sign. A march starts from an edge cell's arm or
  ends in one as at a turning point's arm, and stops at the domain's
  edge otherwise only as a fault. The branch ends there as
  `BranchEnd::Clipped`, pinned to the root on the edge line.
- **A turning point near an edge of `v`** has its cell cut at the edge,
  and an arm that reaches the edge inside the cell — through the cut
  side, or the cut edge — is a short graph from the point to the edge,
  where a branch ends. One within `64·ANGLE_SLACK` of the edge is on it:
  the section crosses the edge square to it there, the arm beyond is
  none, and a branch ends at the point. A hemisphere's twin cut by a
  vertical plane ends its arc so, on the equator.
- **A patch's zeros are its own.** A patch's polynomial goes on past the
  patch, where the surface is the next patch's or none, and its
  isolation certifies zeros there by boxes that reach across — a whole
  span across, on a random B-spline. A torus's quarter turns are one
  surface and never showed it. On a NURBS surface a certified zero counts
  only within `PATCH_SLACK` (`1e-9` of the span) of its own patch, one
  that is not certified only where its box reaches in, each box cut to
  the patch and that slack: a zero on a knot line found from both sides
  is still one, and a box no longer holds the next patch's zeros. The
  walk drops what is left past the domain's edges.
- **Refused by name** (`SectionFault::UnresolvedTurning`): the section
  touching an edge, a turning point on an edge of `u`, a crossing at a
  corner of the domain, and a **turn at a kink**. Across a knot of `v`
  where the surface is only continuous the section's tangent jumps, and
  it may turn back in `u` with `∂f/∂t` of one sign below the knot line
  and of the other above, and no turning point to seed a march from: a
  loop whose turns are all at kinks would be missed. So a root of `f` on
  such a line where `∂f/∂t` changes sign across it, beyond its rounding,
  is refused (`PatchedSection::turns_at_a_kink`). Following the turn
  instead needs a cell at a corner of the section, which a fitted curve
  would have to break at too; a C0 face is not in the census.
- **What the step measured.** Five properties over random poses — the
  exact twins of a cylinder, a cone, a sphere and a torus over random
  parts of their domains and over whole turns, and random rational
  B-splines of degree one to three, against a plane, a cylinder, a cone,
  a sphere and a torus — at 1000 cases on two seeds, 10 000 poses: every
  sample of every branch on the other surface to rounding and at `uv` on
  the NURBS surface, every clipped end on its edge line, the arguments
  in either order the same trace, and a dense scan of the domain finding
  no change of side away from a branch. All 7 543 twins traced. Of the
  2 457 random B-splines 221 were refused at a kink and 3 by name
  elsewhere, a fold or a net two hundred across against a torus a few
  across, where `f64` does not certify a turning point; none was
  answered wrong. A B-spline's net is held to a hundred of the other
  surface's sizes across, the torus tracer's `R/r` bound. The twin of a
  cylinder against a plane is the closed form's ellipse within
  `SECTION_FIT_FRACTION` once fitted; a torus's twin traces the torus's
  spiric sections. On CTC-02, f544 is cut by its box's plane in two
  mirrored arcs from edge to edge, each turning once short of the lower
  edge of `v` with the arm past the turn leaving through it; the drill
  the census paired with f520 passes it by, 42 mm off (step 2's note
  that it crossed the face was wrong; its census of no turning point
  stands). `Surface::project` onto a kinked rational B-spline missed a
  point of the surface by 2.9e-6; that is the projection's, an ignored
  test in `tests/nurbs_project.rs` and a backlog line, and the
  properties hold a branch to the NURBS surface by its `uv`.

### Step 4: isolines and a flush surface (amendment, 2026-10-13)

- **An isoline is a tube circle.** It is looked for as a torus's tube
  circle is (`Tracer::tube_circles`, now either surface's): candidates on
  the knot lines of `u`, the domain's edges among them, and at the sign
  changes of `f` and of its slope along a line of constant `v` through
  every patch; a candidate held to `2·tol.linear` from one edge of `v` to
  the other is refined on the lines where the distance beside it is
  largest, and accepted in length, every point within `tol.linear` of the
  other surface and `F` along it within what `tol.linear` makes of `F`
  (`Implicit::firmness`, the cone's apex measured by projecting it onto
  the NURBS surface). It is a tangency, order two, where `F`'s slope
  across it is within that bound too. One within `EDGE_SNAP` of a knot
  line is put on it, and where `u` does not close one past an edge is
  put on the edge: a face's boundary edge on the other surface.
- **It is returned as the surface's own line**: each row of the net
  across `u` summed at `u₀` in homogeneous coordinates
  (`NurbsSurface::isocurve`), which is knot insertion to full
  multiplicity, over the other direction's knots and degree. No fit; its
  parameter is the surface's other one.
- **It is divided out of the patches whose span holds it**, by `s − s₀`
  — both patches where it is on a knot, and where `u` closes the one
  holding it a closure on. The others keep their polynomial and take the
  quotient's sign: what is left is `F / gᵐ`, `g = (u − u₀)/2π` where `u`
  does not close and `sin((u − u₀)/2)` where it does, never larger than
  one, so the quotient over `|∇F|` is never less than the distance, and
  for an odd order it changes sign with a whole turn as a tube circle's
  does. **No correction** is made, unlike the torus's first circle: that
  subtracts `F` along the circle from every column as one polynomial,
  and a B-spline's other pieces carried to `u₀` are not the surface
  there. What the division is off by is dropped; the isoline was
  accepted only with it within the tolerance's share of `F`.
- **The walk evaluates the quotient on the patches** (`Quotient`, an
  owned copy of them after the division): a divided patch's polynomial
  times the smooth ratio `|s − s₀| / |g|`, and an undivided one's over
  `|g|ᵐ`, over `wᵈ·|∇F|`. The torus's quotient is a closed form along the
  chord; a NURBS surface has none, and the patches' own polynomial makes
  the walk's zeros the isolation's to rounding by construction.
- **A line of constant `v`** is one of `u` with the parameters swapped
  (`NurbsSurface::transposed`): the walk runs on the swapped surface and
  `SectionBranch::uv` swaps them back. Isolines of both directions are
  refused (`SectionFault::TubeCircle`), and so are two or more of one
  direction with more of the section besides, as two tube circles are;
  with nothing besides they are all returned.
- **A branch that crosses an isoline ends on it** at a `SectionPoint`
  that is not isolated, as at a tube circle. One that reaches an isoline
  on an edge of the domain ends there as `BranchEnd::Clipped`: it is the
  edge's end.
- **Flush over an area is refused before anything else** as
  `GeomError::Unsupported`: a Bézier patch of the surface every point of
  a nine-by-nine grid over which, corners and edges included, is within
  `tol.linear` of the other surface. The boolean's coincident arm is the
  analytic kinds' own; a NURBS face flush with another is a backlog line.
- **A collapsed row on the other surface** is found as an isoline and
  divided out, as §4 says, but is still returned as a line one point
  long; step 5 returns it as the singular point it is.
- **What the step measured.** A cylinder's twin flush with a plane at
  its rim, and through its middle; a ruled patch whose boundary ruling
  lies on a cylinder, with the arc beside it held to `u = 3 / (9/4 +
  (3/2 − 2v)²)` to `1e-12`; a cubic patch `z = uv − u³` on the plane `z =
  0`, its line `u = 0` returned and the parabola `v = u²` cut there into
  two branches ending at the origin; a plane along a cylinder twin's
  seam ruling (a tangency on a closed seam), a plane through its axis
  (two rulings) and a sphere about it (two rims); and the torus's twin,
  over a whole turn and over part of one, against the elbow, the bead,
  the ball and the plane through the axis of `tests/trace_torus.rs`,
  each tube circle at `u = 0` and `0.7` the twin's isoline, and over the
  whole turn the same branches and points as the torus's to `1e-9`. A
  property poses a cylinder's twin over a random part of it, anywhere,
  against a plane across it or on its rim, through its axis or along a
  ruling, and a sphere about it: at 1000 cases on two seeds every
  isoline is the twin's line to `1e-12` and on the other surface, in
  either argument order the same. Step 3's properties pass unchanged at
  1000 cases on two seeds, and the torus's tests and corpus against
  their blessed dumps.

### Step 7: the boolean's pcurve on the NURBS face (amendment, 2026-10-13)

The boolean's section edge carries, on a NURBS face, the pcurve
`pcurve_on` projects (`Surface::project`, patch by patch), not the
`(u, v)` the walk traced. The decision is the plan's open question, and
the measure that decided it is the checker at `Full` on every result of
step 7's fixtures, whose pcurve-to-curve deviation is the L2 row: the
box cut of CTC-02 (bicubic faces of 64 patches), CTC-05's rational degree (3, 2) face, a plate drilled by a
cylinder, a NURBS cylinder drilling a box, three bores in one call, a
free-form bicubic dome fused with a box and a split of a NURBS cylinder,
all checker green at `Full`. The fit's span count was not recorded;
step 9's stage timings are where a slow fit would show.

The traced `(u, v)` stays out for a reason that does not need the
measure: `intersect_surfaces` returns the branches' 3D curves and
`MeetPoint`s, and a pcurve on each surface for a branch would be a new
field of `SurfaceIntersection`, a public change that every consumer of
the intersector pays for to save a projection the fixtures show is not
the weak part. If a corpus ever shows an L2 above the band on a NURBS
face (a section edge a projection folds across a patch edge), the
branch's traced `(u, v)` is the remedy and this amendment is the place
to reopen.

## Consequences

- `PatchedSection::on_nurbs` builds the section of a NURBS surface. Its
  turning and critical points are certified over the knot grid and
  merged across shared patch edges and closed seams. `sign_over` cuts a
  box at the knots it straddles and takes a closed direction a closure
  either way.
- `nurbs::project::Patch` and `NurbsSurface::patches` are crate-visible,
  so the projection and the section share one Bézier split.
- The tube-circle search serves both surfaces; its division is the
  torus's closed-form correction on a torus and a plain division of the
  patches that hold the line on a NURBS surface (step 4). A NURBS
  surface's isolines are returned exact, as `SectionTrace::circles`.
- No public type or signature changes until step 5, which removes the
  `Unsupported` arms. From step 3 `trace_section` traces a NURBS
  surface against an analytic one; `BranchEnd::Clipped` is also a
  branch's end on the edge of a NURBS surface's domain, and
  `SectionFault::UnresolvedTurning` names the refusals above.
- The floor of a NURBS section grows with the net's distance from the
  other surface's frame. A part a kilometre from the origin isolates
  less sharply than the same part at the origin, by a factor of that
  distance over the part's size. That is a statement about `f64`.

## Alternatives considered

- **Marching on the NURBS surface from seed points** (Open CASCADE's
  `IntPatch_ImpPrmIntersection` family, by its reputation and the
  oracle's walked lines; not read). It makes "every part of the section
  is on a branch" a sampling argument, which ADR-0018 and ADR-0019 refused
  for the analytic pairs. A torus's machinery is already certified and
  costs no more here.
- **Putting the analytic surface into the NURBS surface's parameters
  the other way round**, as a ruled pair is walked along its rulings.
  A NURBS surface has no implicit form, so there is nothing to put the
  analytic parametrisation into.
- **One polynomial for the whole surface, by degree elevation across
  knots.** A B-spline is only piecewise polynomial, so it has no single
  Bernstein form. The patches are the polynomial pieces, and a zero on
  their shared edge is merged, as the torus's quarter-turn edges are.
- **A control-hull box test before the substitution** (the plan's open
  question on degree blow-up). It is not needed at the census's largest
  faces (above). If a corpus ever shows otherwise, it is a local
  addition to `on_nurbs`.
