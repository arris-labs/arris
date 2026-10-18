# Idea: nurbs-nurbs

- Status: Accepted 2026-10-18 (C, two plans); to be absorbed by
  plans/nurbs-nurbs-section, then plans/nurbs-nurbs-operands
- Raised: 2026-10-18
- Prompt (verbatim from the human): "/idea nurbs-nurbs"; the roadmap's line
  is "NURBS against NURBS: a marcher with explicit seam handling,
  `intersect_curves` for two NURBS curves, and the boolean and blends the
  real parts ask of them" (docs/ROADMAP.md §C10, third item)

## Problem

Two NURBS geometries meeting is the one arm left without a form in each
of the three intersectors: `intersect_surfaces` (`intersect.rs:405`),
`intersect_curve_surface` (`intersect_curve.rs:417`, a NURBS curve against
a NURBS surface) and `intersect_curves` (`intersect_curves.rs:154`). Each
is an explicit `Unsupported`. What that refuses today:

- **C10's own output against itself.** A sweep cut by a sweep, a loft
  through a loft, a fitted blend face against a fitted sweep face: every
  sweep or loft is an operand of every boolean the *analytic* kinds allow
  (ADR-0063, ADR-0065 §non-goals), not of each other. By ADR-0020's
  closure-before-breadth, the cycle's output is not yet fully operable.
- **Blends on an edge between two fitted faces.** ADR-0062's table refuses
  "two B-spline faces"; `sweep/sweep-thread-chamfer-unsupported` holds it
  (the thread's crest rail, which Open CASCADE chamfers). A walked
  stripe's offsets of two NURBS faces are two NURBS surfaces, so the blend
  needs the surface marcher first.
- **The checker.** S5 leaves a pair of NURBS faces undecided at `Full`
  (`arris-check/tests/full.rs:183`, `twin_prop.rs:55`): every sweep, loft
  and twin body with two fitted faces side by side is checked short of
  `Full`. "An `Ok` shape passes `arris-check`" (`.agents/rules/kernel.md`)
  holds with that exemption.
- **Nesting of lumps** with NURBS faces on both shells
  (`regression/nurbs-cylinder-slab-cut-two-lumps`, BACKLOG §Findings).
  The finding names the ray fire over NURBS faces; whether that waits on
  this or only on the classifier is the plan's to read.

What does *not* ask for it: the refusal histograms. The fetched tier's
NURBS row is 0 of 38 at every column, and the script corpus's 4 refusals
are the blend network's 3 and shell's 1. Demand is the cycle's own
closure and the checker's gap, not a refused part. Imported parts whose
B-spline faces meet each other (car-body, consumer-product STEP) would
show it; the corpus has none yet.

## Constraints it runs into

- **SEED.md §9**: analytic kinds are first-class; NURBS × NURBS is the last
  pair, named in §"not in M0" as its own cycle's work. Exhaustive dispatch
  (no `_ =>`) means three arms change, nothing else.
- **ADR-0061** traces NURBS × analytic in the NURBS surface's (u, v)
  because the analytic side has an implicit form. Two NURBS surfaces have
  none (implicitising a bicubic patch is degree 18, and rational patches
  worse), so ADR-0061's polynomial-in-two-unknowns machinery does not
  carry over directly: the problem is three equations in four unknowns,
  `S₁(u₁, v₁) = S₂(u₂, v₂)`. Its four decisions — the domain's edge,
  closure per direction, isolines, collapsed rows — all recur, and its
  seam merge (`bernstein2::merge_wrapped`) and closure reading
  (`NurbsSurface::closure`) are reused.
- **ADR-0018**: one traced curve per surface pair for the whole boolean;
  the pave reads crossings of two traced curves from the tracer's points.
  That is why `intersect_curves` of two NURBS curves has stayed refused
  without hurting. It is still needed where an *edge* on a NURBS curve
  meets a section curve that is not its own pair's.
- **ADR-0062**: a walked stripe traces its offsets' section; two NURBS
  faces' offsets (`nurbs/offset.rs`, fitted) are a NURBS × NURBS section.
  Its tolerance budget (the stripe within the model's tolerance of the
  exact ball path) has to absorb a second fit.
- **kernel.md**: no literal tolerances, deterministic iteration, typed
  refusals naming entities. A marcher's step control is where literals
  creep in; each must be a named constant in `arris-math` or the model's.
- **Performance**: BACKLOG §79 says the NURBS × plane walk is already slow
  (CTC-02's box cut 680 s in the test profile). A four-unknown march over
  patch pairs multiplies the patch count by itself.

## Options

### A — A plain marcher: seed by subdivision, walk by Newton

Subdivide patch pairs until their control hulls are apart or small,
Newton each small overlapping pair onto a point, march from each point in
both directions with a step from the curvature, stop at the domain's
edges. This is Open CASCADE's `IntPatch_WLine` shape and most kernels'.
Cheapest (about 5 steps for the surface arm). It **misses closed loops**
inside a patch pair whose hulls never subdivided small enough, and
**jumps branches** near a tangency, with no way to know it did. Both fail
silently, which is the one thing kernel.md forbids: a missed loop is a
wrong `Ok`. It forecloses certification later without a rewrite.

### B — Fully certified: 4-variable Bernstein isolation

Extend `bernstein2` to tensor polynomials in four variables
(`S₁ − S₂`, per patch pair, three components of bidegree (p, q, p, q)),
subdivide in 4D with exclusion and Krawczyk, as ADR-0019 and ADR-0061 do
in 2D. The section is certified as a whole: every box either excluded or
holding a proven piece. The project's existing standard, carried through.
The cost is large: subdivision in 4D to a curve's width is the curse of
dimensions (a bicubic pair has 256 coefficients per equation per box, and
covering a curve to `tol.linear` takes boxes in proportion to its length
over that width). 10–14 steps, and the measured bound ADR-0061 gives
(292 boxes, 5 ms) would not hold; minutes per pair on CTC-02-sized faces
is likely.

### C — Certify the topology, march the geometry (recommended)

Certify only the points that decide the section's topology, in few
unknowns, and march between them as A does, with the walk held to them:

1. **Boundary seeds**: every point where a section reaches an edge of
   either domain. Each is a NURBS isocurve against the other NURBS
   surface: three equations in three unknowns — `intersect_curve_surface`'s
   new NURBS × NURBS arm, certified by a 3-variable Bernstein isolation.
   Seams are edges of the domain that a closed direction merges across
   (ADR-0061 §2, reused).
2. **Loop seeds**: a closed loop wholly inside both domains carries a
   point where the surfaces' normals are parallel *and* — tighter — on any
   loop, a point where the section's tangent is parallel to a chosen
   direction (the Sederberg–Meyers / Hohmeyer loop condition). The plan
   picks the formulation; either is four equations in four unknowns,
   isolated and certified with Krawczyk. A patch pair whose normal cones
   are disjoint is excluded at once without it, which is most pairs.
3. **Singular points** (tangencies, crossings of two branches) are the
   loop condition's degenerate solutions, found by the same isolation;
   branches end on them as they do in ADR-0061.
4. **The walk** goes from seed to seed in all four parameters, a
   predictor along `n₁ × n₂` and a corrector onto both surfaces. A step
   is accepted only if it stays within its certified box chain, so a jump
   to another branch is detected and the step halved, not taken. Every
   seed must be visited: a seed left unvisited after all walks is a
   typed `SectionFault`, never an `Ok`.
5. **Output**: the 3D curve and both pcurves fitted from the walk's own
   `(u₁, v₁)` and `(u₂, v₂)` (no projection back), each fit's deviation
   measured and carried as the edge's tolerance (ADR-0059's rule).

`intersect_curves` for two NURBS curves comes along almost free: two
equations' worth in two unknowns, Bernstein in 2D, `bernstein2` as it is.

Trade-offs: certification is where it decides correctness (topology: how
many branches, which seeds they join) and absent where it is cheap to
verify after the fact (each point's distance to both surfaces). The walk
can still be slow; it can no longer be silently wrong. It does not
foreclose B: a pair C refuses can be the corpus that motivates B.

Cost: about 14–16 steps, split into two milestone plans:

- **M1 `nurbs-nurbs-section`** (geom only, ~8 steps): the 3- and
  4-variable isolation, the three arms, the seam merge, the coincident
  refusal, the twin property as oracle (the exact twins of two analytic
  surfaces give the closed-form section — every analytic pair is an
  oracle for free), a fuzz target, and a measured performance bound on
  the CTC-02 faces against their own twins.
- **M2 `nurbs-nurbs-operands`** (~7 steps): the pave and booleans on two
  NURBS faces, checker S5 decided at `Full`, a walked stripe between two
  NURBS faces (the thread chamfer, a loft edge's fillet), sweep × sweep
  and loft × sweep fixtures against Open CASCADE, `twin_prop` without
  its S5 exemption, the lumps finding read.

### Do nothing

Keep the three arms refused and close C10 without them. The histograms
would not notice. What it costs: sweep and loft stay operands of
analytic bodies only, the checker's `Full` keeps an exemption for every
fitted body, ADR-0062's two-B-spline row stays a refusal, and the next
imported B-spline-heavy part hits the wall on its first boolean. It also
leaves the roadmap's cycle goal ("a NURBS face meets … other NURBS faces")
unmet; the cycle would close with its goal rewritten.

## Recommendation

**C, as two milestone plans, M1 first.** A loses on the rule that a
wrong `Ok` is worse than a refusal: a missed loop is exactly that, and
it would be found by a consumer, not by us. B is the right shape and the
wrong cost: 4D subdivision buys certainty over every point where C buys
it over the few points that decide the answer, and checks the rest by
measurement. Do nothing loses on closure: ADR-0020 is why C10 holds sweep
and loft to "an operand of every boolean", and the checker's exemption
is an untested promise in every fitted body.

What would change my mind: if M1's step measuring the CTC-02 faces shows
the loop-seed isolation alone past a second a pair, the loop condition is
reformulated (normal-cone pruning first) before the walk is built; if
the twin property shows the walk jumping branches where certified boxes
should have stopped it, the box chain is tightened toward B.

**The words.** The glossary has no entry for the marcher or its seeds;
the roadmap and SEED use "marcher". Proposed:

- **Marched section**: the section of two NURBS surfaces, walked in both
  surfaces' (u, v) at once between certified seeds; its edges and both
  pcurves are fitted from the walk. _Avoid_: SSI curve, marched curve.
- **Seed**: a certified point of a marched section that decides its
  topology — a boundary seed where it reaches an edge or a seam of either
  domain, a loop seed inside both, a singular point where branches meet.
  Every seed is visited, or the section is refused.

## Decision for the human

All rounds answered (see *Decision*). Next: `/plan nurbs-nurbs-section`.

## Decision

Round 1, 2026-10-18:

- Q1: C, certified seeds and a measured walk.
- Q2: two plans, M1 `nurbs-nurbs-section`, then M2 `nurbs-nurbs-operands`.
- Q3: A, the twin property over every analytic pair, with Open CASCADE's
  `GeomAPI_IntSS` on fitted pairs.
- Q4: yes, two NURBS faces flush over an area stay refused `Unsupported`.
- Q5: yes, M1 writes the ADR; M2 amends ADR-0062's two-B-spline row.
- Glossary: *Marched section* and *Seed* accepted, added to `GLOSSARY.md`.

Round 2, 2026-10-18:

- Q6: A, the loop condition is measured in M1 (normals parallel against
  section tangent parallel to a fixed direction, each behind normal-cone
  pruning, on the CTC-02 faces and the analytic twins); the ADR records
  the winner and its numbers.
- Q7: A, the intersectors merge knot spans narrower than the parameter
  resolution before isolating, with no API change; the
  `curve-surface-hit-off-the-surface-on-a-degenerate-knot-vector`
  finding's fixture lands in M1.
- Q8: yes, profiling the NURBS × plane walk (BACKLOG §79) is M2's first
  step.
- Q9: yes, M2 includes the walked stripe between two NURBS faces and
  amends ADR-0062's two-B-spline row.
- Q10: no. Under the developer-first positioning (ADR-0067), the side
  slot goes to the developer launch items ahead of the plugin CAD's
  asks: the credibility plan (already landed, per
  `ideas/developer-first-positioning.md`), then `arris.build`'s first
  milestone, which shares no files with the marcher. Draft and thicken
  are queued after it.
