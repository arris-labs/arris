# Plan: split-by-plane

- Started: 2026-10-06
- Milestone: C7, prismatic features (docs/ROADMAP.md §C7)
- Idea: none of its own. The roadmap's C7 line "Split by a plane" is the
  scope, scheduled by ADR-0047. The decisions are made here and in ADR-0051.
- Idea (verbatim from the human): "yes, /plan it" (the next plan after
  multi-tool-boolean)

## Goal

`arris_ops::split(m, body, plane, control)` cuts a solid by a plane into
the solids on either side and keeps both. It returns `Split { positive,
negative, provenance }`. `positive` is the body on the side the plane
frame's `z` points to and `negative` is the other side. Either may hold
several lumps, as a U-bracket split across both arms does. Both come from
**one** decomposition, the General Fuse of ADR-0004 and ADR-0050, with the
plane as an operand. The body's faces are split once by the section, and
each piece is kept on its side. The section regions become cap faces on the
plane, one copy per side with opposite orientation. Each side is
self-contained, with its own section edges and vertices. Both sides pass
the checker. Every entity of the body that the plane does not touch keeps
its id on its side. The provenance names, for every output, the input it
came from and, through the `Split` it arrives in, the side. The plane does
not cross the body when it misses or only touches it, and that case is a
typed refusal naming the body. The results match Open CASCADE's
`BRepAlgoAPI_Splitter` on a fixture corpus, and the API is bound in the
facade and in Python.

## Non-goals

- A split by a surface other than a plane, by a face of the body, or by
  another body, and keeping a split's pieces as one compound: these are out
  of C7 by the roadmap, and each gets a backlog line.
- Splitting a face or a sheet (a face split by a curve, a surface trimmed by
  a plane): the healing cycle's sheet bodies.
- NURBS faces in the body: refused as the booleans refuse them
  (`OpError::Unsupported` naming the face). They belong to the NURBS cycle.
- A planar section of a body as a query (its curves or its area without
  making solids): the query cycle's (A6). This plan's cap faces are not
  exposed as a query.
- Per-face tessellation (A9) is the side plan.

## Design deltas

- **`arris-ops`, public**: `pub fn split(m: &mut Model, body: Body, plane:
  &Frame, control: &Control<'_>) -> Result<Split, OpError>` and `pub
  struct Split { pub positive: Body, pub negative: Body, pub provenance:
  Provenance }`, re-exported beside the booleans. It is the first operation
  whose output is not `(Body, Provenance)`. ARCHITECTURE §Operations states
  that shape and gains the exception. The plane is a `Frame`, as
  `query::project_to_plane` takes it: the origin and `z` say where the
  plane is and which side is positive, and `x` fixes the cap surface's
  parameterisation, so a consumer's sketch plane gives caps whose (u, v)
  are the sketch's. The new refusal is `SplitReason::NoCrossing { body }`,
  for a plane that misses the body or only touches it along a face, an edge
  or a vertex. A tangent touch with crossing elsewhere follows the
  booleans' `TangentContact` rule. Each of these is a `CHANGELOG.md`
  `### Breaking` bullet (`Reason` and `OpError` grow) and gets its binding
  arm in the same commit.
- **`arris-topo`, public, provenance**: `Role::Split(SplitPart)` is
  appended last (ADR-0028's rule), with `SplitPart::Cap(PlaneSide)` and
  `PlaneSide { Positive, Negative }`. A cap face is `Generated` from its
  side's cap role, since the plane is no entity, so a consumer's split
  feature can name "the cut face of the lid". The body's faces, edges,
  shell and body are `Modified` into their pieces on each side. A section
  edge is `Generated` from the body face it lies on and from the cap role
  of its side, and a section vertex from the edge it splits and that role.
  The input body is `Modified` into both `positive` and `negative`. The
  split order (ADR-0009) is positive side first. This is a breaking
  change: `Role` is exhaustive. The Python `Role` class gets the variant
  (`crates/arris-py/src/role.rs`).
- **`arris-ops`, crate-private, the decomposition**: one `Build` gets two
  selections, cut and common against the plane's operand, assembled into
  two bodies. How the plane enters the decomposition is ADR-0051's call
  (see Open questions). Either way, no entity of a plane operand is left in
  the model, and the output ids depend only on the input.
- **Recipe grammar** (both interpreters, `arris_debug::fixtures` and
  `tools/oracle/oracle/recipe.py`): `split <name>, plane {origin, normal,
  x}` yields `<name>.positive` and `<name>.negative`. A fixture's result
  names one of them, and a `side` variant names the other, so each fixture
  has an `expected.json` and a dump per side. Open CASCADE is driven by
  `BRepAlgoAPI_Splitter` with a planar face bounded past the body's box,
  and its solids are sorted to a side by their centroid's signed distance.
- **Corpus**: a new area `split/`, which the corpus lint holds to passing
  fixtures with dumps like the other areas, and a row in the coverage
  table.
- **Facade and binding**: `arris::ops::{split, Split}` through the
  re-export; `Model.split(body, origin, normal, x=None)` in `arris-py`
  returns `(positive, negative, provenance)` with cancel and budget, plus
  its stub and docstring example. `NoCrossing` becomes a Python class.
- **ADR-0051**, the split by a plane: one decomposition with two
  selections rather than a cut and a common run apart; how the plane is an
  operand; the `Split` return shape; self-contained sides; `Role::Split`;
  `NoCrossing` (written in step 1).

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]**: The oracle and the grammar. Add the `split` op and
  `side` variants in `recipe.py` (`BRepAlgoAPI_Splitter`) and in
  `arris_debug::fixtures`, with the Rust runner refusing it until step 3.
  Fixtures go under `regression/` with `expected.json` per side and their
  closed forms. Each also records the cut and the common by a half-space
  box, so any disagreement on Open CASCADE's side shows up here. The
  fixtures:
  - a box split mid-height
  - a box split obliquely across a corner
  - a cylinder split across its axis
  - a cylinder split along a plane containing its axis, both through its
    seam and off it
  - a tube split across (annulus caps)
  - a sphere split off-centre
  - a plate with a row of holes split through the row
  - a U-bracket split across both arms (a two-lump side)
  - a filleted box split through its fillets (cylinder and torus sections)
  - a shelled box split across its walls (a cap with a hole)
  - the two refusals: a plane that misses, and a plane tangent to a
    cylinder

  ADR-0051 is written here and settles the operand question below.
- [x] Step 2 **[2]**: Two selections from one decomposition, with no public
  change. `result` assembles a cut and a common of the same `Build` into
  two bodies in one pass, with self-contained topology per side. Tested
  against `cut` and `common` run separately on the existing two-operand
  `boolean/` fixtures: equal volume, area and counts, both checker-green,
  and every existing dump unchanged bit for bit.
- [x] Step 3 **[3]**: `ops::split` in general position, where the plane
  meets no vertex, edge or face of the body within tolerance. This step adds
  the plane operand as ADR-0051 decides it, `Split`, `Role::Split`,
  `SplitReason::NoCrossing`, the provenance above, and the binding's arms
  for the role and the refusal. The general-position fixtures move to
  `split/` and are blessed: the box, oblique box, cylinder across, tube,
  sphere, plate with holes, U-bracket, filleted box and shelled box. The
  missed plane becomes a refusal fixture.
- [ ] Step 4 **[3]**: The plane through the body's own entities. Cases:
  - coincident with a face (an L-block split at its step: no cap on that
    stretch, and the face goes to the side its normal says)
  - containing an edge (a box split diagonally through two opposite edges)
  - through a vertex
  - through a cylinder's seam or along a ruling (the axial cylinder split)
  - tangent to a cylinder with crossing elsewhere (`TangentContact`, as the
    booleans rule it)
  - through a cone's apex or a sphere's pole (`BesideSingularity` where the
    booleans refuse it, otherwise built)

  Each case gets a fixture with its oracle, and the refusals are named.
- [ ] Step 5 **[1]**: The facade and the binding: `Model.split` with cancel
  and budget, the stub, the docstring example, a pytest per refusal, and
  the `Role.split` accessor. Add the rustdoc example on `split` and record
  the split fixtures' cancel step counts.
- [ ] Step 6 **[2]**: Properties, sharded and seeded, over random planes
  through random bodies (boxes, cylinders, rounded boxes, multi-tool cuts,
  shelled and filleted bodies):
  - the two sides' volumes sum to the body's
  - each side equals the body's common with its half-space box (volume,
    area, counts)
  - the sides' areas sum to the body's plus twice the section's (the caps)
  - `fuse` of the two sides gives back the body's volume
  - `split` commutes with `transform`
  - each side is an operand of fillet, chamfer, `offset_faces`, `shell` and
    every boolean, with the checker green
  - provenance passes `audit` with the body as the input and both sides as
    outputs

  `prop::recipe` and the differential also draw `split`, as a separate
  strategy so `recipe()`'s seeded stream does not move.

## Acceptance

The plan closes when all of the following pass:
- `ARRIS_GATE=full` is green, and every existing fixture's dump is
  unchanged.
- The `split/` fixtures are checker-green at `Full` on both sides. They
  match Open CASCADE's splitter on volume, area, centroid, inertia, counts
  and probes per side, or their closed forms where ADR-0015's evidence is
  recorded.
- Every refusal fixture fails with its named reason.
- Step 6's properties pass at 256 cases.
- The differential passes at 1000 recipes with `split` drawn and no new
  unexplained disagreement.
- The `python` job's pytest, docstring examples and `mypy.stubtest` are
  green.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Operations: `split`, its two-output shape as the
  exception to `op(&mut Model, …) -> (Body, Provenance)`, the plane operand
  and the two selections. §Errors: `NoCrossing`. The capability table gets
  a "Split a body by a plane" row.
- `docs/DATA-MODEL.md` §Provenance: `Role::Split(SplitPart::Cap(_))`, what
  a split records, the split order by side, and self-contained sides.
- `docs/ROADMAP.md` §Fixtures: `split/` in the list of linted areas. §C7:
  the status line says split landed.
- `tests/fixtures/README.md`: the `split` op and the `side` variant.
- `tools/oracle/README.md`: how the splitter is driven.
- `docs/BACKLOG.md`: split by a face or a body, split of a sheet, and
  whatever refusal the corpus shows is common.
- `CHANGELOG.md` `Unreleased`: what a consumer can now do (split a body in
  two, name the cut faces) and the refusals they will hit. `### Breaking`
  holds the bullets the steps wrote.
- `AGENTS.md` current state: split done, per-face tessellation and C7's
  close next.

## Open questions

- Settled in ADR-0051 (step 1): the plane enters as (a), a scratch box in
  the plane's frame past the body's extent, run through the unchanged
  decomposition and freed afterwards by a new `Model::discard(body, keep)`
  in `arris-topo` (a transaction cannot, since the results are appended
  after the box; a scratch model cannot, since the body's untouched
  entities would lose their ids). **Design delta for step 3**: `discard` is
  a public `arris-topo` method, named in its commit body.
- Found in step 1: the recipe grammar's `side` is an expression on the
  `split` step (positive where above zero), so a variant names the other
  side by overriding a param `side`, and a closed form mixes the two sides
  by `(1 + side) / 2`. No recipe key is added, so no hash moves. Open
  CASCADE's splitter equals its half-space common and cut on all 24 sides
  of the twelve non-refusal fixtures (counts equal, measures to 3e-15).
- Found in step 3: `NoCrossing` is a unit variant, the body in
  `OpError::Degenerate`'s `entities` as every other reason names its
  entities, not a `{ body }` field. `audit` gains `audit_many(model, inputs,
  outputs, provenance)` in `arris-topo` (public, additive) for one record
  of two bodies, and the corpus runner's `Made` gains `outputs`, so the
  provenance stage audits a split against both sides; a later step's
  `<name>.positive` and `.negative` resolve through it. The scratch box is
  `primitive_box`'s Euler sequence (`euler_box`) over the plane's frame, its
  near face the plane `Frame` itself used `Reversed`, its far faces past the
  body's box by the box's own diagonal. A plane past the body's box is
  refused before the box is built; one inside the box that misses the body
  (a cylinder's box corner) reaches the booleans, whose `Empty` and
  `ZeroThickness` become `NoCrossing`. The fixtures moved to `split/`
  without the `-split` in their slugs. All eleven passed on the first run,
  and so did both axial cylinder fixtures in every variant, off-seam
  counts included: step 4 decides whether that stays the convention and
  moves them. The tangent plane fails with `Fault::Split` (a section edge
  ends at a node nothing else reaches), step 4's.
- ⚠ OPEN: whether a side whose pieces are only a sliver within tolerance of
  the plane counts as a crossing or as `NoCrossing`. Agent decides at step
  4, matching what Open CASCADE's splitter builds on the touch fixtures.
