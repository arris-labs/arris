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

### A walked blend's junctions: a saddle and a touch on an edge (amendment, 2026-10-14)

`plans/nurbs-face-blend` step 6 put the checker's S5 on a walked blend
(ADR-0062) and its neighbours. Against the face it is tangent to along a
contact the pair was decided already: the contact is an isoline of the
blend, divided out as §3 divides one. The slot plate's chain
(`blend/nurbs-slot-plate-rim-fillet`) left four pairs refused, all at the
junctions where the walked stripe meets the table's stripes on the ball's
arc. The blend's surface runs past the junction by the spine's reach, so
the arc is an interior isoline, and the refusals were two this ADR named:

- **A crossing on an edge of `u`.** The slot's flat wall is tangent to the
  blend at the arc's contact on the round end, which lies on the blend's
  domain edge `u = 1`. That is a saddle of the section on the edge, two
  arms into the domain, and its cell could not fit inside. The cell is
  now cut at the edge. Below it the certificate is a whole cell's, line
  by line of constant `u` (`Fvv` of one sign, `f` of one sign above and
  below, the separator), and the arms beyond the edge are none. Where no
  isoline fixes the orientation, a saddle on an edge of `v` is traced with
  the parameters swapped, so it is on an edge of `u` there.
- **A touch of an edge that does not enter the domain.** The table
  stripe's cylinder shares the arc with the blend and is tangent to it
  along it. With that isoline divided out, the quotient is positive all
  round the junction's vertex but at the vertex, on the blend's contact
  edge `u = 0`, where the contact on the top face is tangent to the
  cylinder's straight contact and curves away: the cylinder and the
  contact agree to fourth order there. This is now an `EdgeTouch`. Its
  cell is certified on the section's polynomial: `f` moves one way into
  the domain all over it, and has the sign it moves to along a line half
  the tolerance in. Along the edge it has that sign beyond a stretch about
  the touch, halved to the stretch's own size where the coefficients do
  not say. Every zero in the cell is then within half the tolerance of
  the stretch, and every point of the stretch is within the tolerance of
  the other surface. A stretch shorter than the tolerance is an isolated
  `MeetPoint`; a longer one is the edge's own line over it, a tangent
  `SectionCircle`. A turning point on an edge of `u`, or a box of them
  reaching it, is taken the same way, so the touch is found in either
  orientation. One that enters the domain, tangent to the edge from
  inside, is still refused. A convex junction has one: a bar's wall into
  a twinned quarter cylinder, where the table's cylinder carried on past
  the arc passes inside the walked blend beside its wall contact, and the
  branch runs within rounding of the contact edge near the corner. That
  pair stays unchecked (`walked_fillet.rs`, ignored, and a backlog line).

What it measured. The stretch is what rounding leaves unsigned, not the
tolerance's reach: `2.1e-7` either side for `z = x² + y` on `z = 0`,
`8.3e-4` for `z = x⁴ + y`, and `4.1e-4` along the slot's contact at each
junction, where the edge is within `5e-9` of the cylinder `1e-2` away.
All four junction pairs decide in 4 to 11 ms. The slot's check at `Full`
takes 2.7 s against 0.4 s when the pairs were refused. Every walked
fixture (the slot, the cap, the bicubic box, the tee and the drilled
torus) leaves only its pair of two NURBS faces unchecked, and the corpus
holds a blend's result to that. Other results with read B-spline faces
still leave pairs the tracer refuses (the bicubic dome's section and fuse,
CTC-05's box cuts), and so do Open CASCADE's walked blends read back. The step 3 and 4 properties pass at 64
cases.

### A walked blend's pole, and a saddle past an edge (amendment, 2026-10-14)

`plans/nurbs-face-blend` step 9b ended a walked stripe on a pole (ADR-0062
§5): where CTC-01's two equal cylinders turn tangent, the blend's row at
that end is collapsed onto the vertex. The checker's S5 left three of the
blend's pairs undecided, two refusals this ADR named:

- **Isolines both ways, one way's poles.** Against each cylinder the
  contact is an isoline of `u` and the pole an isoline of `v` the same
  face holds, and isolines both ways were refused as tube circles. A pole
  is no line. The polynomial vanishes all along its edge, but the section
  has one point there. Where one direction's isolines are all poles (one
  point to rounding all along, `Tracer::is_pole`), the other direction is
  traced with its own isolines divided out, beside the poles. Each pole is
  divided out of the patches along its edge (`PatchedSection::deflate_pole`)
  and returned as its own isoline, which the intersector makes a point.
  Its order is local: how many of the leading rows of coefficients at the
  edge are within what `tol.linear` makes of `f` on every patch along it,
  at most two. The isoline search's slope test, over a whole unit of the
  walk's parameter, called the blend's tangent pole a crossing: the fit's
  derivative error over that reach is past the tolerance, and over the
  patch it is not. A pole alone (a hemisphere's tangent plane at its pole)
  is divided out as before. Dividing every pole out unconditionally lost
  that point and was not kept.
- **A saddle a hair past an edge.** At the crease's junction the wall
  plane is tangent to the blend where the arc's contact meets the
  cylinder's tangent edge, on the blend's domain edge `u = 0`: the
  slot's saddle on an edge again. But the fit put this one `1.1e-8` of
  the walk's parameter past the edge (`5e-9` in length), outside every
  patch's square. The census never saw it. Its near miss, `5.4e-10` off
  the plane, turned `4.7e-5` inside, and those turning points stood alone,
  uncertified. The critical points are now looked for in a slab past each
  open edge, an eighth of each patch's span deep (`EDGE_SLAB`, the margin
  the isolation extrapolates by to certify), the polynomial carried on by
  de Casteljau (`PatchedSection::critical_points_past`). One whose point
  lies within `tol.linear` of the edge's beside it is put on the edge,
  since the edge's place is not resolved within the tolerance. It is then
  the section's singular point there, and its bump holds the near miss,
  as for a saddle inside. One the census inside found already is not
  found twice.

What it measured. The crease's three pairs decide, and the walked blend
leaves nothing unchecked. The half dome's twin against the plane through
its pole and a meridian (refused before) returns the meridian and the
pole. The patch `z = (u − v)(u + 2v) − 5e-10` over `u ∈ [1e-8, 1]`, its
saddle past the edge, returns the singular point on the edge and the arm
cut there (before: one arm, no point). Each test fails with its change
taken out. The crate's tests, the corpus and every walked fixture pass
unchanged at 32 cases.

### A saddle at a corner of the domain (amendment, 2026-10-14)

`plans/nurbs-face-blend` step 10c. FTC-10's walked blend is an open
5 × 12 patch whose four corners are real, and eight cylinders and spheres
beside it are tangent to it there within the tolerance: the section's
saddle is at a corner, on an edge of `u` and an edge of `v` at once, and
`F ≈ A·dv² − B·du²` has one of its four arms inside the quadrant. The
cell was cut at an edge of `u` only, and every cell the corner needs
reaches past an edge of `v` too, so none fitted at any halving, whichever
way the parameters were swapped.

- **The cell is cut at both edges.** A saddle within `EDGE_TOUCH` of an
  edge of `v` that does not close is put on it, as one on an edge of `u`
  is, and the cell reaches only above it (or below, on `v = 2π`). The
  certificate holds on the rows the cell reaches: `f` of one sign on each,
  and the separator searched between the edge and the cell's top. Of the
  four arms the cut leaves the one inside the quadrant; the march and the
  assembly already take a saddle's arms singly.
- **A box of turning points is the saddle's when its edge is the saddle.**
  An uncertified box whose edge the saddle sits on, to `EDGE_TOUCH`, is
  absorbed by it. The boundary case was decided by rounding, and sent the
  box to the touch of an edge, which it is not.

The patch `z = 4v² − u²` over `[0, 1]²` against its plane returns the one
arm `v = u/2` from the corner to `u = 1`, in either order of the surfaces
(before: refused). FTC-10 then decides all eight pairs and the checker
passes at `Full`.

### A branch at the foot of a junction's arc (amendment, 2026-10-14)

`plans/nurbs-face-blend` step 10d. FTC-11's walked blend meets the two
table stripes' cylinders at the chain's ends along its end arcs: an
isoline of the blend on the cylinder, whose foot is on the blend's contact
edge with the face both roll on. The section's other branch runs down to
that foot along the contact edge, nearer it than the polynomial's floor
signs, and the junction is a 7.3° kink, so the branch is tangent to the
edge there to second order. Four things kept the pairs undecided:

- **A turning point within its fold's rounding of an edge of `v` is on
  it.** Along the contact edge the blend and the cylinder share the face's
  tangent plane, so `∂f/∂v` vanishes on the whole edge and the section
  crosses it at a turning point; the isolation put it `1.6e-11` inside,
  past `EDGE_TOUCH`, and the cell's sign test on the edge row was
  rounding's. A point is on the edge also where `c·d²`, the fold's move in
  `u` over the way `d` to the edge, is under `ANGLE_SLACK`.
- **A crossing that is one side of a tangency is the tangency.** At the
  other end the fit crosses the cylinder twice along the end arc, `8.4e-4`
  of `u` apart and within `1.5e-9` between: one candidate's bracket saw
  one crossing, and the isoline was divided out at order one, leaving the
  second in the quotient, a line of turning points beside it. An isoline
  accepted as a crossing whose slope-zero line beside it, a Newton step on
  the slope away, is held to the tolerance and tangent there is divided
  out there at order two, as two candidates of one group already were.
- **Each patch keeps its own rounding floor.** Dividing a tangency out of
  the patch that holds it raises that patch's floor; the section's floor
  was the largest, and the patch beside, whose polynomial falls with the
  square of the way to the line, kept no sign above it. A patch's floor is
  now its own, raised only where it is divided, and the sign tests,
  candidates and isolations each take the patch's. Dividing the patch
  beside as well, as a torus's column within `DIVIDED_WITHIN` is, was
  tried and rejected: a patch's polynomial past its span is not the
  surface, does not vanish on the line, and the division put the torus
  twin's branches `0.14` off.
- **A march that cannot step within the tolerance of an edge of `v` runs
  aground on it.** Where no step from the march's point is proven and the
  line of constant `u` from it to the edge is within `tol.linear` of the
  other surface all the way, the branch is not the tracer's to resolve
  there, as a point within the tolerance of an edge is put on it. It
  follows the edge line while that is within the tolerance, to the first
  of an edge cell whose arm points back at it (captured as a march is), the
  isoline's foot, or a root along the line by bisection, and ends there
  (`BranchEnd::Clipped`); short of all three it ends on the edge where it
  stuck. A branch clipped so ends on the edge within the tolerance of the
  other surface, at its root to rounding where it found one. At FTC-11's
  start arc the two arms end at the edge's two roots, `0.36270` (an edge
  cell) and `0.36389` (a root the candidate for both did not split), and
  at its end arc both meet at the foot.

The checker then decides both pairs at `Full`; FTC-11's body holds to
Open CASCADE's measures as before, and its fixture test fails with any
of the four taken out.

### Two isolines of one direction with more besides (amendment, 2026-10-27)

`plans/blend-runs-onto` step 4. A run-over's rolled face shares an arc
with the stripe's cylinder at each crossing (ADR-0071, amended). Both are
rows of its `v` on that one cylinder, and the section has more besides
them, near each row's foot on the face both roll on. Either row divided
out alone leaves the other in the quotient, so the walk has no one
factor to divide by and the pair was refused (`TubeCircle`).

- **Such a surface is traced a piece at a time.** Where the walk refuses
  `TubeCircle` and the isolines found are two or more of one direction
  and none of the other, the surface is cut midway between each two
  (`NurbsSurface::segment`, the control net cut at those parameters as
  `clipped_points` cuts it). Each piece is traced on its own with the
  one isoline it holds, and the traces are joined. The pieces keep the
  surface's parameters, so every branch's `uv` is the surface's own. A
  branch that crosses a cut ends there on both sides, `Clipped`. Every
  pose the walk decided before is decided as before.

`z = (y − 0.2)(y − 0.8)((x − ½)² + (y − ½)² − 0.04)` over `[0, 1]²`
against `z = 0` is the rows `y = 0.2` and `y = 0.8` with a circle of
`0.2` between them. It now returns the two rows and the circle in two
halves cut at `y = ½`. Before, it was refused.

### A touch as deep as rounding reaches (amendment, 2026-10-27)

`plans/blend-runs-onto` step 8. A ring's contact runs off a plate's edge
14 degrees off the edge's tangent (the pi mount plate's front
standoffs). At every crossing the rolled face's foot on the boss leaves
the torus's contact tangent: the pipe about the crossed edge is tangent
to the plane of the ring's centres. So the foot agrees with the torus to
fourth order at the corner where the row the two share meets the rolled
face's domain edge. At this angle the foot stays within `1e-7` of the
torus from `−0.2` to `+0.46` of the walk's `u` about that row. Along the
edge, the quotient with the row divided out stays within its floor
(`9.2e-9`, which the division by `(s − s₀)²` carries) for as far. Its
sign there is the fit's: it dips below zero, by `3.5e-11` in distance,
between two sign changes `0.06` apart, a sliver `4.5e-5` deep. The slope
could not prove the candidate at the first change to be a lone crossing,
and it was no touch either, since `f` half a tolerance in has the
sliver's sign.

- **A touch's inner line goes as deep as rounding leaves `f` unsigned.**
  Where `f` along the line half a tolerance in does not have the sign it
  rises to, the tracer tries lines twice as deep, and twice again, within
  the cell. The first signed line bounds the touch's zone. Every point of
  a grid over the zone, the stretch's samples across as many lines out
  to the inner one, must be within the tolerance of the other surface,
  or there is no touch. Every zero in the cell is then within the
  tolerance of the other surface, as the stretch is. A touch found half
  a tolerance in comes out as before, its zone the edge line alone.

The shrunk plate (`blend/standoff-root-fillet-runs-off-at-a-shallow-angle`)
is decided at `Full`. In each piece, the touch's inner line is found at
`1.75e-4`, nine doublings in and past the sliver, and its zone is within
`5.8e-10` of the torus.

### The foot of an isoline two tangent faces leave unsigned (amendment, 2026-10-28)

`plans/three-edge-run-vertex` step 3. A walked corner blend meets the
table stripe's cylinder beside it on one arc of the ball, a column of the
walked face tangent to the cylinder, divided out at order two. Both
faces are tangent to the top they roll on, along contacts that meet
tangent at the arc's end. So along the walked face's contact edge the
two agree to fourth order about the column's foot: on the rounded block
under an oblique top, within `4e-14` of each other `8.8e-4` of the walk's
`u` before the foot. The quotient along that edge is its rounding over
the stretch, and the pair was refused two ways:

- a march came to the edge a stretch short of the foot and went on
  proving steps there, each smaller than the last, none getting anywhere,
  until it ran out of cells;
- a candidate root on the edge `3.4e-5` from the foot was neither a lone
  crossing nor a touch the edge's stretch could bound.

- **A march whose steps shrink below the least cell runs aground.**
  Where the step a march takes from a point falls below `HALF_MIN`, no
  cell having captured it, it tries the aground rule above as if no step
  had been proven. It ends at the isoline's foot when the edge line to it
  is held to the tolerance, as at FTC-11's end arc. Where the rule does
  not apply, the march goes on as before.
- **An edge candidate the edge line joins to the foot is the foot.** A
  candidate on an edge of `v` from which the edge line to the divided
  isoline's foot is within `tol.linear` of the other surface all the way
  is dropped before its root is sought. The foot is a point of the
  isoline already, and a branch that comes to it runs aground there.
- **A box of turning points beside the isoline, held to the tolerance
  out to it, is the isoline.** On a rounded box split by a tilted plane
  and its cap's rim filleted, the walked skin's fitted row sat `1.6e-4`
  of the walk's `u` off the exact arc, the distance `3e-10` there. The
  quotient kept a near-double zero along the whole column, an
  uncertified box of turning points the whole domain long in `v`, which
  was refused. Where every point of a grid over the strip from such a box
  to the isoline, as long as the box, is within `tol.linear` of the other
  surface, the box is dropped: the isoline stands for the tangency within
  the tolerance, as a touch stands for its zone. A singular point the
  same strip joins to the isoline, its cell's reach out to it, is the
  isoline's too.

A walked skin that reaches past a leaning junction's ball rolls on along
the far face's extension, tangent to the wall along the ruling, and the
tracer met two singular points there crowded (`split_prop`'s rounded box
at every tilt it shrank to). ADR-0072's amendment ends the skin at the
arc instead, refitted so that the arc is its end row. The surface's
corner then lies on the ruling at `p`, where the wall, the far face and
the skin are tangent, and beside the table stripe's cylinder along the
arc:

- **A critical point past both edges at a corner is the corner's.** A
  critical point found past one open edge and put on it is put on the
  other direction's edge too where the corner's point is within
  `tol.linear` of its own, and is no point of the surface where it lies
  past that edge and is not. It is not found twice.
- **Two extrema of one kind within each other's reach are one touch.**
  The census finds the corner's touch a hair inside the domain, and the
  edges' slab finds it on the corner; both maxima, or both minima, each
  within twice the other's reach, with values whose signs are rounding.
  The one on more open edges is kept, the corner's on the corner. Two
  crossings, or a crossing and an extremum, stay crowded.
- **A box of critical points at a corner is the corner's touch where it
  is an extremum.** An uncertified box near the other surface that
  reaches both open edges of one corner, short both ways, is read at the
  corner. Its Hessian there definite, it is that touch; otherwise it is
  crowded, as before.
- **A box of turning points beside a touch, held to the tolerance out
  to it, is the touch's.** The corner touches the wall exactly, its value
  zero and its reach none, and the census leaves uncertified boxes of
  turning points beside it where the distance is below its rounding.
- **An edge candidate the edge line joins to a touch on it is the
  touch.** Along the skin's edges out of that corner the distance is
  below its rounding too; a candidate root there, the edge line from it
  to an isolated singular point on the same line within `tol.linear` of
  the other surface, is dropped as the isoline's foot is.

Every pair of the block's and the pocket's table cylinders and walked
corners, square junctions and leaning ones, now traces as the isoline
and two branches, and every pair of the walls with the skin ended at the
arc as its corner's touch; the checker decides them at `Full`, as it
decides both split boxes'.

### A saddle within its reach of an edge (amendment, 2026-10-28)

`plans/walked-saddle-cell` step 2. Where a walked skin ends at a leaning
junction's arc (ADR-0072), its corner lies on the ruling, and the face
across the ruling — the S-bend's second arc's cylinder, CTC-02's planes
— meets it there in a saddle, not a touch. The census finds the saddle a
hair inside the domain: on the S-bend's skin on its edge of `u` and
`3.5e-7` of the walk's `v` inside the other edge, its value `2e-14` and
its reach `4.2e-7` that way; on CTC-02's excerpt `1.6e-8` of `u` and
`4.2e-7` of `v` inside, the reach `9e-6` and `4e-6`. Both distances
exceed `EDGE_TOUCH`, so the saddle cell was whole that way, and a whole
cell, at least twice the reach across, does not fit inside the domain at
any halving: the five pairs were refused `CrowdedSingularity`.

- **A saddle within its reach of an open edge is on it.** Where the
  saddle's distance to an edge of `u` or `v` that does not close is
  within its reach in that direction, and the edge's point beside it (the
  corner's, where the other direction is cut too) is within `tol.linear`
  of the other surface, the saddle cell is cut at that edge as at one
  within `EDGE_TOUCH`. No saddle decided before moves: a whole cell
  never fitted for one that near an edge.

Measured against the rule taken out: the S-bend's walked skin against
its cylinders refused, and with the rule on `v` only CTC-02's four walked
× plane pairs still refused (each in both orders). With it, every walked
face of both against every cylinder, and of the excerpt against every
plane, traces in both orders, its samples within the pair's tolerance of
both surfaces (`tests/intersect_nurbs.rs`).

### A knot line the tolerance does not crease (amendment, 2026-10-28)

`plans/walked-saddle-cell` step 3. A walked corner skin's section across
the stripe is a rational quadratic arc of two spans joined at a double
knot: tangent continuous, its speed alone jumping. Against the table
stripe's cylinder, which the skin meets on the junction's arc, the
section has a root on that knot line where `∂f/∂t` is `+5.1e-9` below
and `−3.9e-9` above, against floors of `1e-11`, and the tracer took it
for a turn at a kink: `UnresolvedTurning`, in 7 of 56 built cases of
the oblique blend property at 64 cases and on the committed pocket's
recipe at 30°. The skin's normals either side part by `1.6e-8` rad over
a patch whose `∂P/∂t` is `0.2` long, so the surface leaves the other
side's tangent continuation by `3e-9`, far inside `tol.linear`.

- **A turn at a kink needs a crease the tolerance resolves.** The sign
  change of `∂f/∂t` across a knot line is refused only where the
  normals either side part by an angle that, times the longer of the two
  patches' `∂P/∂t`, exceeds `tol.linear` (`creased` in
  `trace_torus.rs`); where one side has no normal it is still refused.
  Elsewhere the root is a turning point on the knot line, which the
  isolation finds from either patch within `PATCH_SLACK`. A C0 knot of
  the random B-splines above parts its normals by far more, and is
  refused as before.

Measured against the rule taken out: the 30° pocket's walked skins
against their cylinders refused. With it, every walked face of that
pocket against every cylinder traces in both orders, its samples within
the pair's tolerance of both surfaces (`tests/intersect_nurbs.rs`), and
a 400 × 401 scan of each walked face's domain finds every sign change
of the section within one step of a branch or the isoline the tracer
divided out. The oblique property at 64 cases leaves no S5 row
undecided; the random B-spline properties pass unchanged.


### A tangency's slope over the patches it is divided from (amendment, 2026-10-29)

`plans/walked-saddle-cell` step 6. With its S5 allowance removed, the
oblique blend property at 1000 cases built a rounded block under a top
tilted 31° (posed 89 below the origin) whose walked corner skins the
tracer refused against the table stripes' cylinders as
`UnresolvedTurning`. ADR-0072's skin continues the stripe's cylinder
across the junction's arc: along the arc's row the two are tangent, `F`
`≈ −0.2·(v − v₀)²` across the whole stripe, `v₀` `3.7e-6` inside the
domain's edge. The isoline finder found the line (`F` along it `3e-13`)
and refused it order two: the tangency's own curve, where the slope
across vanishes, wanders `6e-7` of the walk's `u` either side of the
isoline, and the slope across reached `5.6e-8` against what `tol.linear`
makes of `F`, `4.5e-8`. Divided out once, the line left its other half
in the quotient, a line of zeros beside the edge that no edge cell
certifies.

- **The slope a tangency drops is bounded over its reach.** The gate
  took `F` plus its slope across, as if what the division drops reached
  `|g| ≤ 1` over the whole domain. On a NURBS surface an isoline is
  divided out of the patches whose span holds it alone, and the others
  keep their own polynomial (§3), so the dropped `F + s·(u − u₀)` is
  within `|F| + |s|·reach`, the reach the largest `|u − u₀|` over those
  patches (`PatchedSection::divided_reach`). The gate is that sum against
  the same bound: tighter than before for a surface of one wide patch,
  looser for a fine one. A torus's walk divides the closed form round the
  whole tube and keeps `|g| ≤ 1`.

Measured against the rule taken out: the posed block's recipe
(`tests/intersect_nurbs.rs`, the committed block reshaped to the
property's 3 × 1.5 and posed) refuses 4 of its 32 walked × cylinder
traces; with it all trace in both orders within the pair's tolerance of
both surfaces, the property's case one branch within `4e-14` of each.
The skins' patches holding the line are `0.1` of the walk's `u` wide.
Unposed, the same block passes either way, its slope inside the old
bound. The geom tests pass unchanged.

### A branch that comes to a touch on an edge (amendment, 2026-10-29)

`plans/walked-saddle-cell` step 7. At 1000 cases the oblique blend
property left, in shard 1, a rounded block under a top tilted 5.1° whose
walked corner skins the tracer refused against two planes as
`UnresolvedTurning`. ADR-0072's skin touches the plane across the ruling
at its corner `(0, 0)` exactly: a singular point on the edge `u = 0`, not
a crossing (cell `4.6e-7` by `1.5e-7`). The section is a sliver round it:
a turning point `4e-5` in `u`, one arm leaving the cell's side at
`(1.7e-5, 2.6e-5)` and coming to the edge at `v = 2.79e-5`, `f` along the
way no more than `2e-11` with the slope across `8e-7`, all of it within
`tol.linear` of the plane.

`edge_cells` already leaves that root to the touch: the edge line from the
candidate to the singular point on it is within the tolerance
(`at_touch`), so it is "a point of the section already" and gets no cell.
The march that came to the edge found no cell to be captured by and
`march` refused it.

- **A branch that comes to such a stretch ends there.** At an edge of `u`
  with no cell of its own, the march ends aground (`ArcEnd::Aground`, an
  open end at the edge as at the foot of an isoline) where `at_touch`
  holds of its point, the same test that left the root cellless. Anywhere
  else the edge stays a refusal.

Measured against the rule taken out: the posed block's recipe at 5.1°
(`tests/intersect_nurbs.rs`) refuses the pair in both orders, twice
(4 traces); with it each is one branch within the pair's tolerance of
both surfaces. The geom tests pass unchanged.

### A box of critical points beside the isoline (amendment, 2026-10-29)

`plans/walked-saddle-cell` step 8. The same 1000 cases left the block of
the amendment above posed 90 above the origin, and a block at 24.9°
posed `(0, 77, −69)`, refused against the table stripes' cylinders as
`CrowdedSingularity`. The skin is tangent to the cylinder along the
junction's arc and the isoline is divided out at order two (the
amendment before last). The census past the edge `v = 2π`
(`critical_past_edges`) leaves an uncertified box of the quotient's
critical points `2e-4` to `3e-4` of the walk's `u` beside the isoline,
`1e-4` wide or less and up to the slab's depth. Over the first block's
box `∂f/∂t` is positive, but its least coefficient, `4e-12`, is under
the rounding floor the pose gives it, `1.8e-11`, and `∂f/∂s` runs from
`−3e-14` to `2.5e-8`. Put on the edge, within the tolerance at a corner
of it, the box was refused as too flat to decide. Unposed, the same
block passes the checker at `Full`.

- **A box of critical points beside the isoline is the isoline's.** On a
  NURBS surface an uncertified box of critical points, found inside the
  domain or past its edges, whose strip to the isoline divided out is
  within `tol.linear` of the other surface at every point of a grid
  (`beside_isoline`), is dropped before the singular points are decided.
  It is the quotient's double zero a hair off a fitted row tangent to an
  exact arc, as a box of turning points beside the isoline is (above),
  and the section there is the isoline.

Dropping a box over which one partial keeps its sign, the hypothesis the
plan began from, is not this case: neither partial keeps its sign beyond
its floor, which is why the box is uncertified.

Measured against the rule taken out: the two posed blocks' recipes
(`tests/intersect_nurbs.rs`) refuse 4 and 2 of their 64 walked ×
cylinder traces; with it all trace in both orders within the pair's
tolerance of both surfaces. The geom tests pass unchanged.

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
