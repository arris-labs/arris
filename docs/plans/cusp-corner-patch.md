# Plan: cusp-corner-patch

- Started: 2026-11-02
- Milestone: M10 (cycle C11, docs/ROADMAP.md §C11 — the blend residue)
- Request, verbatim: "$plan cusp-corner-patch"; no idea file exists.
- Basis: C11's next-plan line; ADR-0042 §6 and ADR-0079 leave this case
  refused. ADR-0084 as amended (the mutual trim, the human's decision at
  step 2) settles the law; the slug keeps its first name.

## Goal
A cusp of three edges, with both same-sense edges selected at one fillet
radius or symmetric chamfer distance, builds as the stripes' **mutual
trim** (`GLOSSARY.md`): each stripe runs on past the ball at the offsets'
meeting and is cut by the other along their section, from `T*` down to
the spine at `S = (cusp, h − σr)`; the cusp vertex goes, no face is added,
provenance is complete and the body is decided at `Full`. Cover the
crescent prism, its concave pocket and a line–arc spandrel. The result
remains an operand of booleans and blends, meshes and round-trips STEP.

## Non-goals
Mixed-sense cusps, including FTC-06's concave overhang caps (ADR-0079),
arbitrary free-form walls, general three-or-more-blend corners, variable
radius and other chamfer modes; any fitted corner face. The cusp
profile/coarse-mesh and wide-fillet node-matching defects keep their
backlog entries; unrelated failures get fixtures and backlog entries.

## Design deltas
- `docs/ARCHITECTURE.md` §Operations (blends): a two-edge cusp join before
  `build::vertex_joins`'s refusal, distinct from a tangent chain, a miter
  and the three-edge `corner::Corner`: ADR-0042's cut by the next wall
  with the other stripe as the cutter, between the closed-form trim
  points `T*` and `S` (ADR-0084 §4′). `ring` and `Stripe` ends admit it
  before lone-cusp trimming runs; the section edge is assembled once and
  shared by both stripes. The section is the intersector's closed form or
  traced and fitted (`trace_section`, `fit_branch`, `pcurve_on`) under
  ADR-0042 §3–§4, its branch ending at the tangency at both trim points.
- Construction private to `arris-ops::blend`. No new crate dependency,
  geometry kind, surface fit or public type/signature: `fillet(m: &mut
  Model, body: Body, edges: &[Edge], radius: f64, control: &Control<'_>)
  -> Result<(Body, Provenance), OpError>` and `chamfer` keep their
  contracts and Python methods. `Reason` gains no variant.
- §Provenance: ADR-0084 §7′. Stripe-only entities keep their edge's
  origin; the section edge, `T*` and `S` are `Generated` from both, after
  each edge's own records; `generated_pair(e, n)` is exactly those three.
  `e`, `n`, `v` deleted; `w` modified (cut at `S`); walls and `T` once.
- `GLOSSARY.md`: **Mutual trim** (done at step 2's reshape; *corner patch*
  under its _Avoid_). No `docs/DATA-MODEL.md` delta: nothing is fitted.

## Steps
Grades are tiers (`.agents/tiers.md`): **[1]** routine, design fixed and tests
mechanical; **[2]** careful, geometry within a given design; **[3]** unproven,
an algorithm's robustness or bound must be established. Each step is a commit.

- [x] Step 1 **[3]** — Establish the corner law and record the ADR. Inspect
  the crescent oracle's patch, trims, normals and interior over several sizes;
  compare pocket, spandrel and chamfer twins. Commit reproducible measurements
  and desired-result fixtures (ignored in `regression/` until built), retaining
  the existing refusal fixture until step 3. Resolve the first open question.
- [x] Step 2 **[3]** — The mutual section, privately: given two stripes at
  a cusp, the closed-form trim points `T*` and `S` and the section of the
  two stripe surfaces between them (torus/torus on parallel axes, torus/
  ruling cylinder), its branch ending at the tangency at both ends.
  Unit-test the points on both surfaces and both trim points within
  tolerance, the branch chosen, and agreement with the section edge of
  Open CASCADE's `common` of its two single-edge fillets (sampled by an
  oracle script); refuse a branch that leaves either band, by name.
- [x] Step 3 **[3]** — Route the convex crescent's two arcs through the
  new join: cut both stripes on the shared section, shorten `w` to `S`,
  provenance per §7′. Convert `blend/cusp-crescent-both-arcs-fillet` from
  refusal to success, taking its regression twin's `second_oracle.json` and
  `analytic` (same recipe), bless its dump and require `Full` with no
  undecided pair (the stripes are tangent at both ends of their edge).
  Settle Open CASCADE's STEP read-back (its patch's gap, the twin's
  README) by the existing keys. Update `build.rs`'s and `corpus.rs`'s
  refusal comments; changelog and rustdoc for the new case.
- [x] Step 4 **[2]** — Pocket and spandrel fillets: import each one's
  second-oracle run at 0.1 into its fixture (`second_oracle.py`) with
  `measure_differs`/`counts_differ`, build them, promote as they pass;
  check shared-face rewrites, spine cut, orientation, provenance. Add
  oracle-held `BlendTooLarge` and mixed-sense refusal guards.
- [ ] Step 5 **[2]** — Chamfers: the same join with the chamfer faces'
  exact conic section (ADR-0084 §3, Open CASCADE's counts already 9/14/7).
  Promote the three chamfer fixtures held to Open CASCADE at the default
  tolerances, counts and `Full`; a missing closed form is an intersector
  case, not a trace.
- [ ] Step 6 **[2]** — Closure: a box cut crossing the section edge,
  fuse/common with that tool, a split through it, a second blend on a
  surviving edge; mesh and STEP round trip. Oracle-valued fixtures; new
  undecided pairs resolved in the shared intersectors, no exclusions.
- [ ] Step 7 **[2]** — `cusp_corner` properties in `blend_prop`: safe
  sizes/scales and random poses, convex/concave, fillet/chamfer; rigid
  motion and mirror covariance, selection-order and repeated-run
  determinism, provenance stability, cancellation and refusal rollback.
  Derive the safe size domain (ADR-0084 §5′) in the ADR.
- [ ] Step 8 **[2]** — Re-run both real-part tiers and the script corpus,
  print the histograms, census cusp pairs explicitly (single-edge requests
  cannot reach this join). Recipes and records for newly covered sites,
  named residuals in BACKLOG; hold the one-edge cusp, overhang, flush-fan
  and seam-junction results; record C11's M10 outcome.

## Acceptance
`blend/cusp-crescent-both-arcs-fillet` must succeed, held to the second
oracle's record (ADR-0073, `measure_differs`): volume `4.693817087520078`,
area `20.411586052772922` and its centroid and inertia, at the fixture's
tolerances; counts V/E/F/loops `9/14/7/7` (`counts_differ`; Open
CASCADE's `11/17/8` stay the record), one shell, one solid, genus 0, Euler
characteristic 2, its two probes. Pocket and spandrel fillets carry their
own records (the pocket 12 faces, 26 edges); chamfers and closure
fixtures their Open CASCADE values. All new successes are decided at `Full`.

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
- `docs/ARCHITECTURE.md` §Operations, §The checker — the join and its limits.
- `docs/adr/README.md` and ADR-0084 — the safe size domain (step 7).
- `CHANGELOG.md` and blend rustdoc — the newly supported case.
- `docs/ROADMAP.md` §C11, `AGENTS.md` current state — M10 and measured counts.
- `docs/BACKLOG.md` — retire the both-edges cusp line, retain measured residue.

## Open questions
Evidence: `tests/fixtures/regression/cusp-corner-crescent-fillet/README.md`
and `law.json`, `graph.json`, `second.json` beside it.
- *Settled* (step 1, ADR-0084 §1–§3): the site, `Q` and `T*` at the
  offsets' meeting, a chamfer's exact section; square fill rejected for
  folding at the cusp.
- *Settled* (step 2, the human, ADR-0084's amendment): the least-bending
  graph missed Open CASCADE's own bound in all nine runs; a second oracle
  builds no patch, its stripes meeting on their own intersection
  (9/14/7), which the mutual trim of Open CASCADE's single-edge fillets
  matches to 1.3·10⁻⁹ of the body at 0.1, ≤ 0.24 bounds at worst. A
  fillet is that mutual trim, held to the second oracle's record, and the
  acceptance's counts and measures are its.
- *Settled* (step 2): no fit budget or graph regularity is needed; the
  size domain is §5′'s, derived by step 7. A change to C11 scope belongs
  to the human.
- *Settled* (step 3): the join is recorded as a miter (`Miter`, `q = T*`,
  `p3 = S`, the spine its third edge) and each ring's end at the cusp as a
  junction's, `T*` read off the two contacts before the rings are built.
  It takes two open arcs of one sense, so the pocket's concave arcs build
  too (12 faces, 26 edges, the second oracle's counts); step 4 holds them
  to its record. `regression/cusp-corner-crescent-fillet` stays, ignored,
  as the evidence directory the oracle scripts read the recipe from; the
  area fixture runs that recipe.
- *Settled* (step 4): the pocket and the spandrel are promoted as
  `blend/cusp-{pocket,spandrel}-both-arcs-fillet`, their regression twins
  kept ignored as the crescent's is. The spandrel's cusp is a line and an
  arc: `cusp_of` takes an arc among the rings or a closed-form line stripe
  (its wall a plane normal to `T`), each side's contact on `T` read before
  the rings are built, and `cusp_join` takes the two sides as `Run`s, a
  stripe's contacts ending at the trim points' parameters on its lines.
  The second oracle's record of the spandrel is 6.2e-9 above Arris's volume
  and Arris matches a 20-digit quadrature of the mutual trim's height field
  to 1e-11, so the fixture's tolerances are the record's (1e-8, 3e-8
  centroid and inertia), said in its `measure_differs`. Open CASCADE builds
  a mixed-sense pair of the overhang tip, so that guard is a test pinning
  `TangentChain` (`fillet.rs`), not an oracle-held refusal; the
  oracle-held `BlendTooLarge` is the crescent at r 1 (both kernels refuse).
