# Plan: nurbs-analytic-section

- Started: 2026-10-13
- Milestone: C10, the NURBS cycle (docs/ROADMAP.md §C10), its first item
- Idea (verbatim from the human): "first thing from C10"

## Goal

A NURBS surface meets every analytic kind: `intersect_surfaces` returns
the section of a `Surface::Nurbs` against a plane, a cylinder, an elliptic
cylinder, a cone, a sphere or a torus. Each section is traced exactly in
the NURBS surface's own `(u, v)`, the way a torus's is traced in its own
(ADR-0019), and fitted under `SECTION_FIT_FRACTION`. A circle and an
ellipse meet a NURBS surface as a line already does. With both in place,
`cut`, `fuse`, `common`, their multi-tool forms, `split` and `section`
take a body with NURBS faces against analytic ones. A pair still out of
reach is refused by name, never with a panic. What the plan proves:
- A body whose analytic faces are replaced by their exact `to_nurbs`
  twins gives the same result as the analytic body, within the fixture's
  tolerance.
- The real parts' NURBS-blocked `box_cut` and `drill_*` stages either
  agree with Open CASCADE or are refused by name.

## Non-goals

- **NURBS against NURBS** (surface pairs, and a NURBS curve against a
  NURBS surface). That is C10's third item. Those pairs keep their
  `Unsupported`.
- **Blends on a NURBS face.** A fillet needs the offset of the NURBS face,
  and that is the next plan in this cycle. `blend/ends.rs` keeps its
  refusal.
- **Sweep and loft.** They are C10's second item and consume this plan.
- **A NURBS face flush with an analytic one** (the surfaces coincide over
  an area). The intersector detects it and refuses it as
  `Unsupported`; the boolean's coincident arm stays same-kind only. This
  is a backlog line.
- **NURBS against analytic in `extrema_*`** beyond what the plane and the
  sphere already take. The distance's backlog line stands.
- **Trimming curves on a NURBS face** other than the section's own and
  `pcurve_on`'s projection. No new pcurve kind is added.
- **Performance work** beyond the meter and the budget. Timing is
  recorded at step 9; a slow part becomes a backlog line.

## Design deltas

- **ADR-0061** (step 2): the section of a NURBS surface against an
  analytic one is traced in the NURBS surface's `(u, v)`. Each rational
  Bézier patch of the surface is put into the other surface's implicit
  form (`Implicit::along`, generic over two variables since C3). The
  result is a tensor Bernstein polynomial of bidegree `(d·p, d·q)`. The
  torus walk's turning points, critical points, cells and singular-point
  bump then run on it unchanged. The ADR decides four things the torus
  never had to:
  1. **The domain's edge.** A branch that reaches an edge of a non-periodic
     direction ends there. Its end is the root of `f` along the edge line,
     which is the same equation the edge's own curve meets the other
     surface by, so the boolean's pave and the section agree on the
     vertex. Every root on an edge line seeds a march, the way `u = 0`'s
     roots seed the torus's winding components. The surface is not
     extrapolated past its domain.
  2. **Periodicity per direction**, read from `NurbsSurface::closure`.
     The torus's two seams become zero, one or two.
  3. **The isoparametric line on the other surface**, the analogue of the
     tube circle. A whole row or column `u = u₀` (or `v = v₀`) on which
     `f` vanishes is found first. It is returned as the exact isocurve
     (knot insertion, no fit) and divided out of its patch column, as
     `deflate` divides out a tube circle. The candidates are the patch
     edges and the sign changes along the rows. A surface on which `f`
     vanishes over a whole patch is coincident and is refused.
  4. **A collapsed row** (a pole, `NurbsSurface::collapsed_rows`). The
     section through it is handled as the sphere's pole is, or refused by
     name. Step 1's census decides which; see the open question.
- **`arris-geom` internals.** `trace_torus::PatchedSection` and
  `torus_walk::Walker` stop being torus-only: they work on a patched
  parametrisation, either the torus's sixteen quarters or a NURBS
  surface's Bézier patches. The torus keeps its fixtures and blessed
  dumps bit for bit; any change there fails step 3. The quotient by a
  tube circle stays the torus's own closed form.
- **Public API.** `intersect_surfaces`, `trace_section` and
  `intersect_curve_surface` keep their signatures and lose the
  `Unsupported` arms for NURBS × analytic and for circle or ellipse ×
  NURBS. No new public type or error variant. `GeomError::Unsupported`
  stays the refusal for NURBS × NURBS and for the flush case. This
  changes behaviour, not types, so `CHANGELOG.md` gets a bullet under
  `Unreleased` and nothing under `Breaking`.
- **`arris-ops`.** The boolean's pave already maps `GeomError` and fits
  pcurves with `pcurve_on`, which projects onto a NURBS face. A branch
  traced in the NURBS surface's `(u, v)` carries its pcurve there exactly.
  Step 7 decides whether the pave takes that pcurve instead of the
  projection; the deciding measure is the checker's L2 on the
  `(u, v)` jumps.
- **`arris-debug`.** `histogram::blocks_reason_in` already attributes
  NURBS × analytic to the NURBS cycle. No change, except that the rows
  fall.
- **`arris-py`.** No signature or error changes, so nothing to rebind. One
  docstring example (a cut through a NURBS face) is added at step 8.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[1]** — **Census of the NURBS-blocked stages.** Run the
  battery on the 6 parts that C9's histogram puts in the NURBS row, and on
  the committed tier's NURBS fillet. For each refused stage, record the
  face pair, the NURBS face's degrees, span counts, closure and collapsed
  rows, and whether the stage is reachable without NURBS × NURBS or a
  blend. Shrink each reachable stage to a
  `tests/fixtures/regression/<slug>/` fixture with its oracle value and an
  `#[ignore = "…"]`. Write the table into this plan; it fixes step 9's
  acceptance list and answers the collapsed-row question.

  **Census (2026-10-13, `real_parts --nurbs`, `arris_debug::nurbs_census`).**
  The six parts are 11 refused stages. Only the booleans are reachable:
  each tool is analytic (planes and lines for the box; a cylinder, planes,
  circles and lines for a drill), so every pair the stage asks is NURBS ×
  analytic, or a circle or a line against a NURBS face. Every fillet is a
  blend.

  | Part | Solid | Stage | Refused pair | NURBS face | Reachable | Fixture |
  |---|---|---|---|---|---|---|
  | CTC-02 AP242 e2 | #14911 | box_cut | NURBS × plane | bicubic, 16×4 spans, polynomial, open/open; the box reaches 3 more bicubic faces (8×2, 2×6, 6×2), one with a collapsed row at `u = 0` | yes | `regression/nist-ctc-02-ap242-box-cut-nurbs-plane` |
  | CTC-02 AP242 e2 | #14911 | drill_x | NURBS × cylinder | bicubic, 16×4, polynomial, open/open; reaches one more of the same | yes (circles too) | `regression/nist-ctc-02-ap242-drill-x-nurbs-cylinder` |
  | CTC-05 AP203 | #249 | box_cut | NURBS × plane | degree (3, 2), 2×1 spans, rational, open/open, no pole; the only one reached | yes | `regression/nist-ctc-05-ap203-box-cut-nurbs-plane` |
  | CTC-05 AP242 e1 | #1205 | box_cut | NURBS × plane | the same face as one rational Bézier patch (1×1) | yes | `regression/nist-ctc-05-ap242-box-cut-nurbs-plane` |
  | CTC-02 AP242 e2 | #14911 | fillet | NURBS curve × plane | none: a B-spline edge between a cylinder and a plane | no, a blend | — (backlog: B-spline edges where the faces meet in a conic) |
  | FTC-11 AP242 e2 | #1299 | fillet | NURBS curve × plane | none: a B-spline edge between a plane and a cylinder | no, a blend | — (same line) |
  | FTC-10 (committed) | #6434 | fillet | NURBS curve × cylinder | none: a B-spline edge between two cylinders | no, a blend | — (same line) |
  | CTC-05 AP203 | #249 | fillet | NURBS × torus | the (3, 2) face above | no, a blend on a NURBS face | — (the next plan) |
  | CTC-05 AP242 e1 | #1205 | fillet | NURBS × torus | the same | no, a blend on a NURBS face | — (the next plan) |
  | CTC-01 AP203 | #335 | fillet | cylinder × cylinder | none: the crossing cylinders' crease | no, a blend | — (backlog: the crease) |

  Step 9's acceptance list is the four fixtures: CTC-02's `box_cut` and
  `drill_x` and CTC-05's two `box_cut`s. The fillet column's NURBS rows
  stay. Three of them carry no NURBS face at all: they are the backlog's
  B-spline-edge line, which this cycle's blend plan can take. The largest
  substituted bidegree the census asks is (6, 6), a bicubic patch against
  the drill's cylinder. No torus meets a NURBS face outside a fillet.
  Collapsed row: **in scope** (answered below).
- [x] Step 2 **[3]** — **ADR-0061, then isolation on a NURBS patch
  grid.** Write the ADR (design delta above). Generalise `PatchedSection`
  over a patched parametrisation, using `Implicit::along` on each Bézier
  patch (`NurbsSurface`'s patch split, shared with `nurbs::project`).
  Isolate the turning and critical points over the grid and merge them
  across patch edges and any seam. Test with the exact `to_nurbs` twin of
  a cylinder, a sphere and a torus patch, each against a plane and a
  quadric: the census has the turning and singular points the closed form
  predicts. A bicubic bump against a plane at its saddle's height gives a
  certified saddle. The torus's own census is unchanged.

  **Landed (2026-10-13).** `PatchedSection::on_nurbs` behind a `Chart`
  (the torus's quarters, or a NURBS surface's knot spans), merged per
  direction by `bernstein2::merge_wrapped`. Every twin and the saddle
  match the closed form (ADR-0061 §grounds). On CTC-02's two largest
  faces (64 bicubic patches each, against their plane and drill and a
  torus at bidegree (12, 12)), every turning point certifies within 292
  boxes to depth 12, about 5 ms. One finding: the floor must charge the
  move of the net into the other frame linearly. As the quartic of a
  kilometre's reach it was 0.9, and the torus's turning points came back
  uncertified.
- [x] Step 3 **[3]** — **The walk on a bounded domain.** `Walker` and
  `Tracer` run on the NURBS parametrisation. Branches end on the domain's
  edges at the roots of `f` along each edge line, and every such root
  seeds a march. Periodicity is per direction. `trace_section` dispatches
  NURBS × analytic here and keeps `within` out of it. Properties over
  random poses: every point of every branch lies on both surfaces within
  `tol.linear`; an edge end lies on the edge line; the twin of a cylinder
  against a plane traces what `plane_cylinder` gives in closed form,
  within the fit fraction. The torus fixtures, dumps and properties pass
  unchanged.

  **Landed (2026-10-13).** `torus_walk::trace_nurbs`, behind
  `trace_section`. The walk runs on the NURBS parameters mapped onto
  `[0, 2π]` each way (`Rescale`), so the torus's constants carry over.
  Every root on an edge line gets an edge cell with one arm, and a turning
  point's cell is cut at an edge of `v` (ADR-0061, amended). Findings:
  a patch's isolation certifies zeros past the patch, a span past it on
  a random B-spline, which were phantom turning points: a zero now
  counts only on its own patch. A B-spline only continuous across a knot
  of `v` can turn there with no turning point, and is refused by name.
  Over 10 000 random poses on two seeds every twin traced, and the random
  B-splines were refused only at kinks but for 3 of 2 457, by name.
  `Surface::project` misses a point of a kinked rational B-spline by
  2.9e-6: an ignored test in `tests/nurbs_project.rs`, and a backlog line
  at retirement. Step 2's note that the drill crosses CTC-02's f520 was
  wrong: it passes it by 42 mm. The torus's tests, properties and its 27
  corpus fixtures pass against their blessed dumps.
- [x] Step 4 **[3]** — **Isolines on the other surface, and coincidence.**
  Find a row or column of `f ≡ 0` within `near`. Return it as the exact
  isocurve and deflate it out, and end crossing branches on it as on a
  tube circle. A surface coincident with the other over a patch is
  `Unsupported`. Fixtures: a cylinder twin whose rim row lies on a plane
  (the cut flush with the rim), a ruled NURBS whose boundary ruling lies
  on a cylinder, and a planar NURBS on its plane (refused).

  **Landed (2026-10-13).** The torus's tube-circle search finds a NURBS
  surface's isolines of `u`; a line of `v` is found and walked on the
  surface transposed. Each is returned as `NurbsSurface::isocurve`, the
  net summed at `u₀`, and divided out of the patches whose span holds
  it, with no correction: a B-spline's other pieces carried to `u₀` are
  not the surface there. The walk evaluates the quotient on the divided
  patches themselves (`trace_torus::Quotient`), so it has the
  isolation's zeros by construction. Isolines of both directions, and
  two or more with more of the section, are refused as `TubeCircle`; a
  patch flush with the other surface is `Unsupported` (ADR-0061,
  amended). Beyond the three fixtures: a branch crossing an isoline (`z
  = uv − u³` on its plane), a tangent isoline on a closed seam, two
  rulings and two rims alone, the torus twin against the tube-circle
  holders of `tests/trace_torus.rs`, and a property over a cylinder twin
  in any pose (1000 cases, two seeds). A collapsed row on the other
  surface is divided out but still returned as a line one point long:
  step 5's.
- [x] Step 5 **[2]** — **`intersect_surfaces` over NURBS × analytic.**
  The six arms in both orders go through `section::traced`: fitted
  branches, exact isocurves and singular points as `MeetPoint`s. Update
  the doc comment and add an example. Fixtures under
  `tests/fixtures/geom/`, each held to Open CASCADE's `GeomAPI_IntSS`
  through the oracle (curve length, endpoints, count) and to the closed
  form where the NURBS is a twin. Covers the collapsed-row case if step 1
  kept it in scope: an isoline on a collapsed row (step 4 finds and
  divides it out) is returned as the singular point it is, not as a
  line one point long.

  **Landed (2026-10-13).** The six arms in both orders, the elliptic
  cylinder's included, go through `section::traced`; two NURBS surfaces
  stay `Unsupported`. An exact isoline whose control points are one point
  (a collapsed row) is a `MeetPoint` instead of a curve, `Touch` or
  `Crossing` as the isoline was, and dropped if the tracer's own singular
  point is within `2·tol.linear` of it. The oracle script walks a NURBS
  pair in `GeomAPI_IntSS` (no `IntAna` form), and
  `geom/c10-nurbs-analytic-pairs` holds the bicubic against a plane, two
  cylinders (a loop and four clipped arms), a sphere, a cone and a torus,
  the octant against a plane and a cylinder, and the extruded quadratic
  against a plane (two exact rulings) and a sphere, to it; the twins and
  the pole are held to closed forms in `tests/intersect_nurbs.rs`.
  `every_other_pair_is_unsupported` now holds only NURBS × NURBS to
  `Unsupported`; a NURBS × analytic pair is `Ok`, a named
  `DegenerateSection`, or a flush `Unsupported`. Random NURBS patches
  traced against a plane take minutes at 256 cases, so the property
  `tests/nurbs.rs` held to the old refusal no longer traces one: that
  is step 9's timing line.

  Three corpus refusals fell with the arm, all NURBS × plane on the STEP
  cylinder whose faces are all B-splines: the interference of a box
  through its wall now answers `overlapping` as Open CASCADE does
  (`query/interference-nurbs-cylinder`, renamed from `-refused`; the
  distance of a far box stays refused, its line against a NURBS face
  being the `extrema` backlog line), its section by a plane answers
  (`query/section-nurbs-cylinder`, new, held to the circle's closed
  form: Open CASCADE's area is 4.5e-5 below it, `analytic.query_differs`),
  and the checker decides a NURBS face against the planes it meets
  (`a_nurbs_face_against_planes_is_decided_and_not_a_violation`; the S5
  row stays unchecked for two NURBS faces only). The boolean's own pave
  is step 7's.
- [x] Step 6 **[2]** — **A circle and an ellipse against a NURBS
  surface.** The curve is the meet of its plane and its cylinder (an
  elliptic cylinder for the ellipse). On each patch, the plane's offset
  and the cylinder's implicit form are two Bernstein polynomials whose
  common zeros `common_zeros_with` certifies, as `line_nurbs` does. A
  touch is decided in length on the distance. Each hit's `t` comes from
  the circle's angle at the surface point. Fixtures: a circle through a
  twin cylinder (four hits, closed form), a circle tangent to a bump (a
  touch), and a circle missing by `2·tol.linear`.

  **Landed (2026-10-13).** `nurbs::line`'s `Line` became a `Cut`, a line
  or a conic, and the isolation, the descents, the poles and the touch
  search are shared; the line's tests, its fixtures and its blessed
  hits are unchanged. A conic's two offsets are its plane's and its
  cylinder's `E / |∇E|` (the distance in the plane to first order), the
  polynomials `h₁` and `h₂ = (min(a, b) / 2)·E` of the patch's degrees
  and twice them; the critical-point system is `W² h₁ D₁ + h₂ D₂` with
  `Dₖ = hₖ,ₓ W − eₖ hₖ Wₓ`, `e = (1, 2)`. The second-order model of a touch
  adds the conic's bend, `κ N·n`, to the surface's sag. `Coincident` is a
  conic that meets the surface and lies within the tube of it at 64
  points round (`CONIC_SAMPLES`), the line's test of touches along it
  being a straight line's. Held to `GeomAPI_IntCS`
  (`geom/c10-conic-nurbs-hits`: three circles and an ellipse through the
  bicubic, a circle across the pole, a tangent circle, the same two
  tolerances clear, one touching at the pole, one across the crest) and,
  at 3000 cases, to the analytic arms through the exact twin of every
  surface kind, an ellipse and a circle centred on a point of the
  surface (`tests/conic_nurbs.rs`); the oracle's tangent hit, which it
  reports twice a little way apart, is matched by ours where it lies
  within the tolerance of the surface beside it.
- [x] Step 7 **[2]** — **The boolean over NURBS × analytic.** Run the pave,
  face splitting on the NURBS face's `(u, v)` domain, and classification
  end to end. Decide the pcurve source (traced `(u, v)` or `pcurve_on`) on
  the checker's L2 and record it in the ADR. Fixtures under
  `tests/fixtures/boolean/`, each with its Open CASCADE value:
  - a box cut by the NURBS twin of a cylinder;
  - a NURBS twin of a block drilled by a cylinder;
  - a bicubic dome fused with a box;
  - the multi-tool cut of three cylinders through a NURBS-faced plate;
  - `split` and `section` of a NURBS-faced body by a plane.

  The NURBS operands are written by the oracle's `occt_step.py --nurbs`
  and read through Arris's reader, so both kernels see the same surfaces.

  **Landed (2026-10-13).** The boolean, multi-tool cut, `split` and
  `section` run end to end on NURBS × analytic with no change to `arris-ops`:
  the pave's `pcurve_on` projection stays the pcurve source (ADR-0061,
  step 7 amendment). New fixtures, each checker green and held to Open
  CASCADE (or the closed form where it is the quadrature that is off):
  `boolean/nurbs-cylinder-tool-through-hole-cut`, `nurbs-plate-drilled-cut`,
  `nurbs-plate-three-bores-cut-many` (counts differ: Open CASCADE splits a
  circle on a NURBS plane in two), `nurbs-bicubic-dome-fuse`,
  `split/nurbs-cylinder-oblique` and `query/section-nurbs-bicubic-dome`
  (closed form 27.25; Open CASCADE 2.9e-6 low). The dome is a free-form
  bicubic solid from `tools/oracle/occt_bicubic_dome.py`: the plan's
  sphere-cap dome is unreadable (backlog, ADR line 102). Step 1's
  fixtures: CTC-05's two moved to `boolean/` and pass, with Open CASCADE's
  healing counts as `counts_differ`. CTC-02's `box_cut` passes every
  Arris stage and the oracle but stays ignored: the reader refuses Open
  CASCADE's own STEP of the result (backlog). CTC-02's `drill_x` stays
  ignored: Open CASCADE's cut is wrong by half the removed volume (its
  own classifier, 40 000 samples; backlog), so step 9's acceptance for it
  is Arris's `common` and `cut` agreeing, not the oracle. The CTC-02
  and CTC-05 fixtures carry measured per-fixture tolerances: the oracle's
  quadrature on the part's B-spline faces is 6e-6 off Arris's exact volume
  of the uncut part. Stage times in the debug-optimised test profile:
  CTC-02 `box_cut` 680 s, `drill_x` 140 s, CTC-05 50 to 80 s: step 9's
  timing line.
- [ ] Step 8 **[2]** — **The twin property.** Add a recipe transform to
  `prop::recipe`: replace one operand's faces by their exact `to_nurbs`
  twins (through `Builder`, keeping the topology). The property: for every
  boolean recipe over plane, cylinder, cone, sphere and torus operands,
  the twin's result is checker green and matches the analytic result's
  volume, area and counts within the fixture tolerance, or both refuse.
  A disagreement is shrunk to a `regression/` fixture. Add the Python
  docstring example (a cut through a NURBS face, read from a STEP).
- [ ] Step 9 **[1]** — **The real parts.** Re-run the battery and the
  histogram over both tiers. Move step 1's fixtures that now pass into
  their areas with their blessed dumps; keep the rest ignored, each
  refusal named. Re-bless the committed histogram block in
  `docs/ROADMAP.md` (`fixtures:` commit). Record the NURBS row before and
  after, and the stage timings, in this plan.

## Acceptance

- `cargo nextest run --workspace` at 256 cases (`real_*` included) is
  green. The full gate (`ARRIS_GATE=full`) is green.
- Every fixture from steps 5–7 is checker green and matches Open CASCADE
  (or the closed form, for twins) within its tolerance.
- The torus's fixtures and blessed dumps are unchanged, bit for bit.
- The twin property (step 8) and the trace properties (step 3) are green
  at 256 cases. CI's run at 1000 cases is green before retirement.
- The real-part histogram's NURBS rows fall at `box_cut` and `drill_*`.
  Every remaining NURBS refusal names a NURBS × NURBS pair or a blend; none
  is a fault or a panic. The number is printed in the roadmap.

## Docs to update on completion

- `docs/DATA-MODEL.md` §Surfaces and §Curves (the surface–surface and
  curve–surface tables): NURBS × analytic is traced in the NURBS surface's
  `(u, v)`, isolines are exact, a flush NURBS face is refused, and a
  circle or an ellipse meets a NURBS surface. §NURBS: patches into an
  implicit form.
- `docs/ARCHITECTURE.md` §Operations: the boolean, `split` and `section`
  take NURBS × analytic face pairs; the refusal list names what is left.
- `docs/ROADMAP.md` §C10: the status line for this first item, with the
  histogram's NURBS row. The committed histogram block is re-blessed at
  step 9.
- `docs/BACKLOG.md`: drop the "NURBS operands in booleans" line, or narrow
  it to NURBS × NURBS. Add the flush NURBS face line and any slow-part
  timing line. Update the `extrema` line if this plan touched it. Add
  step 3's refusals: a section turning at a kink of a B-spline (a knot
  of `v` where it is only continuous), and one touching an edge of the
  domain, turning on an edge of `u` or crossing at a corner. Add
  `Surface::project` missing a point of a kinked rational B-spline
  (`tests/nurbs_project.rs`, ignored). Add step 4's refusal of isolines
  of both directions — a face with a boundary edge of `u` and one of `v`
  on the other surface, `z = uv` on `z = 0` — which the walk does not
  divide out together.
- `docs/adr/README.md`: ADR-0061's line.
- `CHANGELOG.md` `Unreleased`: booleans, split and section now take bodies
  with NURBS faces against analytic ones, which covers most imported
  B-spline parts. The refusals that are left: two NURBS faces, a NURBS
  face flush with another face, and fillets on NURBS faces.
- `AGENTS.md` current state: C10's first item landed.

## Open questions

- Answered at step 1: a collapsed row (a pole on a NURBS surface) is **in
  this plan**. Both of the rule's conditions hold. CTC-02's box reaches a
  bicubic face whose `u = 0` row is one point, and a loft to a point, this
  cycle's next item, makes one on every section it lofts. Step 5 holds it
  with a fixture.
- ⚠ OPEN: is the pcurve on the NURBS face the traced `(u, v)` or
  `pcurve_on`'s projection? The **agent** decides at step 7, on the
  checker's L2 and the fit's span count, and records the answer in
  ADR-0061.
- Answered at step 2: degree blow-up needs **no prefilter**. On the
  census's largest faces, against a torus at bidegree (12, 12), the
  isolation takes 292 boxes at the most, inside the torus's own bound.
  A patch the other surface misses is excluded at depth 0 (ADR-0061).
