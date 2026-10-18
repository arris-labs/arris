# Plan: nurbs-nurbs-section

- Started: 2026-10-18
- Milestone: C10, the NURBS cycle (docs/ROADMAP.md §C10), its third item, M1 of two
- Idea: docs/ideas/nurbs-nurbs.md (M1 absorbed; settled there: option C,
  two plans, the twin property as oracle, flush faces refused, one ADR,
  the loop condition measured here, sub-resolution spans merged)

## Goal

Two NURBS geometries meet in all three intersectors. `intersect_surfaces`
and `trace_section` return the **marched section** of two `Surface::Nurbs`:
its **seeds** certified by Bernstein isolation, the walk between them in
both surfaces' (u, v) at once, the 3D curve and both pcurves fitted from
the walk under `SECTION_FIT_FRACTION`. `intersect_curve_surface` takes a
NURBS curve against a NURBS surface, and `intersect_curves` two NURBS
curves. A section whose seeds the walk does not all visit is refused by
name, never returned. What the plan proves: the exact `to_nurbs` twins of
two analytic surfaces give the analytic section, over every analytic pair
in random poses; fitted pairs agree with Open CASCADE's `GeomAPI_IntSS`.

## Non-goals

- **M2, `nurbs-nurbs-operands`**: the pave and booleans on two NURBS
  faces vetted against Open CASCADE, checker S5 deciding two NURBS faces
  at `Full`, the walked stripe between two NURBS faces (ADR-0062's row),
  profiling the NURBS × plane walk first (BACKLOG §79), the lumps finding.
  The next plan, from `docs/ideas/nurbs-nurbs.md`.
- **Two NURBS surfaces flush over an area**: refused `Unsupported`, as
  NURBS × analytic is (BACKLOG §76).
- **A fully certified section** (the idea's option B): only seeds are
  certified. Also out: NURBS × NURBS in `extrema_*`, and performance work
  beyond a measured bound.

## Design deltas

- **ADR-0068** (step 2): the section of two NURBS surfaces is marched
  between certified seeds. Per pair of rational Bézier patches the system
  is `w₂·P₁ − w₁·P₂ = 0`: three tensor Bernstein polynomials in
  `(s₁, t₁, s₂, t₂)` of degree `(p₁, q₁, p₂, q₂)`. *Boundary seeds* are
  each edge line of either domain (an exact isocurve) against the other
  surface, by the new curve × surface arm, seams merged as ADR-0061 §2
  does. *Loop seeds* solve the condition step 2 measures (normals
  parallel, or the tangent parallel to a fixed direction, each behind
  normal-cone pruning), its degenerate boxes the *singular points*. The
  walk predicts along `n₁ × n₂`, corrects onto both surfaces, and accepts
  a step only inside its seeds' box chain; every seed is visited or the
  section is refused. Isolines and poles as ADR-0061 §3–4; the 3D curve
  fitted under `SECTION_FIT_FRACTION`, each pcurve from the walk's own
  parameters, not by projection.
- **`arris-geom` internals.** A new crate-private `bernstein_n`: tensor
  polynomials in three and four variables, exclusion, Krawczyk, polish
  and merge, as `bernstein2` does for two (which stays as it is). Step 1
  found exclusion one polynomial at a time too weak in four unknowns
  (the live boxes grow as the condition number to the fourth); a box is
  also excluded by a row of `Y·F`, `Y` the inverse of its middle
  Jacobian, which took the posed systems from 10 263 boxes and seven
  continua of 31 to 989 boxes and none. A new
  `march` module holds the seeds and the walk, and `trace::Walk` gains a
  `Marched` arm. Knot spans narrower than the parameter resolution are
  merged before any isolation, the NURBS × analytic trace included (Q7).
- **Public API.** `intersect_surfaces`, `trace_section`,
  `intersect_curve_surface` and `intersect_curves` keep their signatures
  and lose their NURBS × NURBS `Unsupported` arms. **Breaking:**
  `SectionFault::UnvisitedSeed` (a seed no walk reached), bound in
  `crates/arris-py` and `_arris.pyi` in the same commit (step 5).
- **`arris-ops`, `arris-check`**: no code change. With no guard before the
  intersector, booleans and S5 reach the new arm at once (open question).

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[3]** — **`bernstein_n`: isolation in three and four
  unknowns.** Exclusion, Krawczyk on the grown box, polish, merge, the
  front and depth bounds of `bernstein2` carried to n variables. Tests:
  square systems with known simple roots certify each alone; a double
  root comes back uncertified, never lost; measured boxes and depth
  recorded as the module's constants.
- [ ] Step 2 **[3]** — **The loop condition, measured; ADR-0068.** Build
  the patch-pair systems and both candidate fourth equations with
  normal-cone pruning. Run them on twins with interior loops (sphere ×
  sphere, crossing cylinders of unequal radii, spiric torus × plane) and
  on the CTC-02 faces against moved copies of themselves. Certified count
  and time per pair go in the ADR, which picks the winner.
- [ ] Step 3 **[2]** — **Sub-resolution knot spans merged.** Before every
  isolation, NURBS × analytic included. The
  `curve-surface-hit-off-the-surface-on-a-degenerate-knot-vector` crash
  input joins `fuzz/corpus/` and gets a passing fixture, and the finding
  leaves BACKLOG §Findings.
- [ ] Step 4 **[3]** — **NURBS curve × NURBS surface.** Per span × patch,
  three equations in `(t, s, u)`: hits certified, a touch from the
  uncertified boxes as the line arm reads one, a curve on the surface
  `Coincident`. Held to the twins of line, circle and ellipse × each
  analytic kind against the closed forms, plus `tests/fixtures/geom/`
  fixtures against `GeomAPI_IntCS`.
- [ ] Step 5 **[3]** — **The marcher.** Seeds (step 4 on every edge
  line, step 2's loop condition, singular points), the walk with its box
  chain, seams wrapped, `SectionFault::UnvisitedSeed` with its Python
  binding. Held to twins of plane × plane, cylinder × cylinder (crossing,
  skew, tangent) and sphere × cylinder: branch count, ends and singular
  points equal the closed form's, every walked point within `near` of
  both surfaces.
- [ ] Step 6 **[2]** — **Isolines, poles and the flush case.** An
  isocurve of one surface on the other returned exact (a plane's twin
  through a ruled face's row), a pole on the other surface as a singular
  point (a sphere's twin against a cone's twin through it), a patch flush
  with the other surface `Unsupported`. A test each.
- [ ] Step 7 **[2]** — **The arm and its fit.** The 3D curve and both
  pcurves fitted from the walk. `intersect_surfaces` and `trace_section`
  dispatch NURBS × NURBS to it. `tests/fixtures/geom/` fixtures against
  `GeomAPI_IntSS` on fitted pairs: a sweep face against a loft face, a
  CTC-02 face against a bent copy, two skins crossing at a seam. Tests
  that asserted the refusal are re-pointed.
- [ ] Step 8 **[2]** — **Two NURBS curves.** `intersect_curves`: the
  distance's critical pairs per span pair (two equations in `bernstein2`
  as it is), a hit where the distance is within `tol.linear`, an overlap
  `Coincident`. Held to twins of line, circle and ellipse pairs, plus a
  `GeomAPI_ExtremaCurveCurve` fixture.
- [ ] Step 9 **[2]** — **The twin property, the fuzz target and the
  bound.** `nurbs_nurbs_prop`: random poses of every analytic pair,
  both twinned, against the analytic section. The `intersect_surfaces`
  fuzz target draws NURBS × NURBS. A meter budget is measured on the
  CTC-02 faces and held by a test.

## Acceptance

- `cargo nextest run -p arris-geom -E 'test(nurbs_nurbs) | test(bernstein_n) | test(march)'`
  at 256 cases. `nurbs_nurbs_prop` holds every analytic pair's twin
  section to the closed form: branch count, ends and singular points,
  Hausdorff within `SECTION_FIT_FRACTION · tol.linear`.
- `tests/fixtures/geom/` NURBS × NURBS fixtures pass `tests/oracle.rs`
  against `GeomAPI_IntSS`, `GeomAPI_IntCS` and `GeomAPI_ExtremaCurveCurve`.
- The degenerate-knot fixture passes, and the fuzz targets run clean for
  10 minutes each on NURBS × NURBS input.
- The full gate is green, the re-pointed ops and check tests included.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §`arris-geom` row and the intersector paragraph:
  the marched section; two NURBS faces leave the `Unsupported` list.
- `docs/DATA-MODEL.md` §Curves: the three tables' NURBS × NURBS arms.
- `docs/ROADMAP.md` §C10 third item and `AGENTS.md` current state: M1 done.
- `docs/BACKLOG.md`: drop the two-NURBS-faces line's intersector half
  (its operands half is M2's).
- `CHANGELOG.md` `Unreleased`: what a caller can now intersect, and the
  `UnvisitedSeed` break. `GLOSSARY.md`: sharpen *Marched section* or
  *Seed* if ADR-0068 changes either.

## Open questions

- ⚠ OPEN (agent, by step 7): the tests that asserted the refusal
  (`arris-ops/tests/boolean.rs:516`, `arris-check/tests/full.rs:183`,
  `twin_prop.rs`'s S5 exemption, the corpus's two-NURBS cases). One whose
  new result is checker-green and matches its oracle passes; any other
  becomes a `regression/` fixture for M2.
- ⚠ OPEN (agent, in ADR-0068 at step 2): loop seeds past a second a pair
  on the CTC-02 faces mean pruning by normal cones before the walk is
  built, or stopping for the human.
