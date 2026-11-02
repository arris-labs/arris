# Plan: cusp-corner-patch

- Started: 2026-11-02
- Milestone: M10 (cycle C11, docs/ROADMAP.md §C11 — the blend residue)
- Request, verbatim: "$plan cusp-corner-patch"; no idea file exists.
- Basis: C11's next-plan line; ADR-0042 §6 and ADR-0079 leave this case
  refused. ADR-0062 permits fitted blend surfaces; the corner law is not settled.

## Goal
A cusp of three edges, with both same-sense edges selected at one fillet
radius or symmetric chamfer distance, builds where the acceptance corpus's
oracle builds: the two stripes end on a fitted corner patch, with shared
edges, complete provenance and a body decided at `Full`. Cover the crescent
prism, its concave pocket and a line–arc spandrel, with a planar shared face
and plane/cylinder walls tangent along a line. The result remains an operand
of booleans and blends, meshes and round-trips through STEP.

## Non-goals
Mixed-sense cusps, including FTC-06's concave overhang caps (ADR-0079),
arbitrary free-form walls, general three-or-more-blend corners, variable
radius and other chamfer modes. The cusp profile/coarse-mesh and wide-fillet
node-matching defects retain their existing backlog entries; unrelated
failures get fixtures and backlog entries, not silent scope expansion.

## Design deltas
- `docs/ARCHITECTURE.md` §Operations (blends): add a distinct two-edge cusp
  join before `build::vertex_joins`'s refusal, separate from a tangent chain,
  miter and three-edge `corner::Corner`. `ring` and `Stripe` ends must admit
  this join before lone-cusp trimming runs; assemble each shared boundary once.
- New ADR (next free number, currently ADR-0084), amending ADR-0042 §6 and
  ADR-0079's both-edges refusal: derive the cut-back locations, boundary
  curves and tangent constraints from the oracle's geometry, then define an
  independent deterministic surface law. A generic smooth fill is not a
  sufficient definition: its interior and measures must agree with the oracle.
  Step 1 settles the law; step 2 proves its numerical construction before
  routing a public operation through it. Name any reference-tree modules read.
- Keep the construction private to `arris-ops::blend`, using `NurbsSurface`,
  `NurbsCurve`, `fit_skin` where its constraints suffice, and the existing
  section/pcurve machinery. No new crate dependency or geometry kind. No
  planned public type/signature change: `fillet(m: &mut Model, body: Body,
  edges: &[Edge], radius: f64, control: &Control<'_>) ->
  Result<(Body, Provenance), OpError>` and `chamfer` with `distance: f64`
  retain their contracts and Python methods. A necessary public fitting API
  must first be named with its exact signature in this plan and the ADR.
- `docs/DATA-MODEL.md` §NURBS, §Tolerances, §Invariants: document the patch's
  parametrisation (a B-spline graph over the shared plane, trimmed; no
  collapsed row, ADR-0084 §5) and positional/tangent error bounds.
  Partition the entity tolerance among the surface, boundary curves and
  pcurves; refinement is bounded and metered. Check regularity in the patch
  interior and compatible limits at corners; no tolerance growth to hide a
  failed fill, no checker exemption. Existing typed errors name the site on
  an unsupported construction, an overlarge trim or a failed fit.
- §Provenance: patch face generated from both selected edges; stripe faces
  keep their individual origins. Patch boundaries/vertices shared by both
  blends have both origins; stripe-only contacts retain their edge's origin.
  Consumed edges are deleted; retain the cusp vertex where the patch returns
  to it. Surviving shortened edges and affected faces are modified once. Fix split order in the ADR and assert `generated_pair`.
- `GLOSSARY.md`: **Corner patch**, the fitted face joining two stripes at a
  cusp, distinct from a junction's shared arc. No existing `⚠ OPEN:` closes.

## Steps
Grades are tiers (`.agents/tiers.md`): **[1]** routine, design fixed and tests
mechanical; **[2]** careful, geometry within a given design; **[3]** unproven,
an algorithm's robustness or bound must be established. Each step is a commit.

- [x] Step 1 **[3]** — Establish the corner law and record the ADR. Inspect
  the crescent oracle's patch, trims, normals and interior over several sizes;
  compare pocket, spandrel and chamfer twins. Commit reproducible measurements
  and desired-result fixtures (ignored in `regression/` until built), retaining
  the existing refusal fixture until step 3. Resolve the first open question.
- [ ] Step 2 **[3]** — Construct the bounded patch privately from step 1's
  boundary data: ADR-0084 §5's least-bending height graph, trimmed. Test boundary positions, tangent planes, regularity, refinement
  convergence and cancellation against independent checks and oracle samples;
  reject incompatible constraints. Record measured error budgets in the ADR.
- [ ] Step 3 **[3]** — Route the convex crescent's two selected arcs through
  the new join; trim both stripes and assemble the patch with shared pcurves
  and provenance. Convert `blend/cusp-crescent-both-arcs-fillet` from refusal
  to success, reconcile its regression twin, bless its dump, and require
  `Full` with no undecided pair and the stored oracle measures. Add changelog
  and rustdoc coverage of the newly supported case in this commit.
- [ ] Step 4 **[2]** — Build concave pocket and line–arc spandrel fillets,
  both hands and top/bottom placements. Promote step 1's fixtures as they pass;
  verify shared-face rewrites, spine retention/trimming, orientation and provenance.
  Add oracle-held oversized-trim and unsupported mixed-sense refusal guards.
- [ ] Step 5 **[3]** — Build the symmetric chamfer counterpart using the
  measured chord/boundary law (the oracle adds no patch in the three families).
  Promote the oracle-built chamfer fixtures; hold their measures, boundaries,
  counts and `Full` reports. Record oracle refusals separately from successes.
- [ ] Step 6 **[3]** — Prove closure on the new face: a box cut crossing the
  patch, fuse/common with that tool, split/section through it, and a second
  blend on a surviving edge; mesh and STEP round trip. Commit oracle-valued
  fixtures and resolve new patch-related S5/B1 undecided pairs through the
  shared intersectors, without exclusions; inspect each failed result.
- [ ] Step 7 **[2]** — Add `cusp_corner` properties in `blend_prop`: safe
  sizes/scales and random poses, convex/concave, fillet/chamfer. Assert rigid
  motion and mirror covariance, selection-order and repeated-run determinism,
  provenance stability under small edits, and cancellation/refusal rollback.
  Derive a safe size domain in the ADR; do not reject failures inside it.
- [ ] Step 8 **[2]** — Re-run both real-part tiers and the script corpus,
  print the histograms, and census cusp pairs explicitly (single-edge requests
  cannot exercise this join). Store recipes/oracle records for newly covered
  sites and named residuals in BACKLOG. Hold the existing one-edge cusp,
  overhang, flush-fan and seam-junction results; record C11's M10 outcome.

## Acceptance
`blend/cusp-crescent-both-arcs-fillet` must succeed: its stored oracle has
volume `4.6938986018778355`, area `20.451369829102774`, V/E/F `11/17/8`,
one solid and Euler characteristic 2. Compare all stored measures and probes
at fixture tolerances; any count convention needs geometric evidence under
ADR-0015. Oracle fit error is measured, not absorbed by arbitrary tolerance
widening (ADR-0062, ADR-0073). Pocket, spandrel, chamfer and closure fixtures
each carry their own oracle values; all new successes are decided at `Full`.

```sh
cargo test -p arris --test corpus blend_cusp
cargo test -p arris --test corpus cusp_corner
cargo test -p arris --test corpus_lint
ARRIS_PROPTEST_CASES=256 cargo test -p arris-ops --test blend_prop cusp_corner
tools/real-parts.sh
tools/py-test.sh
tools/script-corpus.sh
```

Name new corpus tests with `cusp_corner` so the closure run includes them.
Run the Python corpus before aggregating its outcomes. The blend row remains
0 of 38 real parts and script `fillet` refusals 0; report the new pair census
separately. Long runs are background jobs; the full gate runs once at retirement
on its commit's own tree (ADR-0081), with the existing unrelated waits named.

## Docs to update on completion
- `docs/ARCHITECTURE.md` §Operations, §The checker — construction and limits.
- `docs/DATA-MODEL.md` §NURBS, §Tolerances, §Provenance — fit and origins.
- `docs/adr/README.md` and the new ADR — evidence, bounds, refusals.
- `GLOSSARY.md` — Corner patch; blend rustdoc and `CHANGELOG.md` — support.
- `docs/ROADMAP.md` §C11, `AGENTS.md` current state — M10 and measured counts.
- `docs/BACKLOG.md` — retire the both-edges cusp line, retain measured residue.

## Open questions
Evidence: `tests/fixtures/regression/cusp-corner-crescent-fillet/README.md`
and `law.json`, `graph.json` beside it.
- *Settled* (step 1, ADR-0084): the cut-back is the ball at the walls'
  offsets' meeting, both stripes ending on its arcs; a fillet's corner patch
  runs from those arcs to the retained cusp vertex along the walls'
  (u, v)-straight curves, G1 to the ball and G0 on the walls, its interior
  the least-bending height graph over the shared plane. A square Coons fill
  folds at the cusp corner, and the miter's area falls outside the oracle's
  own bound, so both are rejected. A chamfer adds no patch: its faces meet on
  their exact section and the spine is cut at `h − σd`, so step 5 builds an
  exact section, not a patch. Fixtures are held within the oracle's own bound
  of its patch. Step 2 proves the graph's fit against the oracle.
- ⚠ OPEN (step 2, for the human: it changes the acceptance's measures): the
  least-bending graph misses the oracle's own bound. Converged at two
  resolutions it lies 0.14–0.31 r below the oracle's patch on average, its
  volume 1.9–4.5 and its area 0.86–2.0 bounds off in all nine runs (crescent
  at 0.1: 2.4–2.6 and 1.1); the G1 Hermite fill over a chart that does not
  fold lies 0.65–5.9 and 2.7–4.7 bounds the other way (`graph.json`). Only
  the folding square fill lands inside, so the oracle's interior is its own
  construction's, not a law's. Recommended: a second oracle's crescent
  (ADR-0073); if it sides with the graph, hold the fillets' measures to it
  and keep §5, else §5 changes. Step 2's construction waits on this.
  Decided (a), and run (`second.json`): the second oracle builds no patch
  in all nine runs. Its stripes run on to the cusp and meet on their own
  intersection, ADR-0084's rejected miter, 9/14/7 against the acceptance's
  11/17/8; Open CASCADE's mutual trim of the two single-edge fillets
  matches it to 1.3·10⁻⁹ at 0.1. The graph is 1.9–5.1 bounds off on
  volume and 1.5–3.2 on area, Open CASCADE's patch 2.6–4.2 on area. Neither
  agrees, so §5 is not kept on this evidence; whether the fillet becomes
  the miter (which changes the acceptance's counts) is the human's.
- ⚠ OPEN: What size domain, fit budget and regularity test admit each family
  without folding? Agent fixes the bounds by step 2, extends them for chamfers
  by step 5 and records evidence. A change to C11 scope belongs to the human.
