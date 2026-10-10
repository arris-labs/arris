# Plan: interference

- Started: 2026-10-11
- Milestone: C8, the query cycle (docs/ROADMAP.md §C8)
- Idea (verbatim from the human): "interference"
- Idea: `docs/ideas/interference.md` (absorbed). Option A plus D's pair
  loop was accepted; the decisions are delegated to the agent and settled
  below, recorded by ADR-0055 in step 2.

## Goal
A consumer asks whether two solids interfere and gets one verdict:
**apart** (with `distance`'s answer, farther than the model's linear
tolerance), **touching** (the contact is a point,
a curve or a face, with a point of it), or **overlapping** (the shared
volume, and whether either solid lies inside the other). The verdict
agrees with `ops::query::distance` on every pose — distance 0 exactly
when the verdict is not apart — and the volume *is* `common`'s, measured
on a scratch clone of the model, so the model is never written. A second
call screens a list of bodies (an assembly) and returns the verdict of
every pair that is not apart, in ascending pair order. Answers are held to
Open CASCADE (`BRepAlgoAPI_Common` + `GProp`, `BRepExtrema_DistShapeShape`)
by an `interference` form of the `query` fixture kind, and the Python
binding exposes both.

## Non-goals
- **Returning the intersection body.** That is `common`; a viewer that
  wants to highlight it calls `common` (idea option C, rejected).
- **A volume without building the common** (idea option B). A backlog line,
  taken only when a measured clash loop says the scratch common is too
  slow.
- **A verdict-only screen that builds no common.** Telling touching from
  overlapping without the common needs a transversality test on the
  section; a backlog line unless step 7's measurement asks for it.
- **NURBS operands' volume.** `common` refuses a NURBS face meeting the
  other operand (backlog, the NURBS cycle's); interference refuses the same
  way, naming the face. Not even an apart pose answers: `distance` refuses a NURBS face
  against an edge (step 5).
- **Shells, sheets, penetration depth, minimum translation to separate.**
  A shell is `QueryReason::NotMeasurable`, as in `distance`.
- **The planar section** — the next and last C8 plan.

## Design deltas
- **`arris-ops`, `ops::query::interference`** (new file
  `query/interference.rs`):
  - `interference(&Model, a: Body, b: Body, &Control) ->
    Result<Interference, OpError>`, a long query (ADR-0030).
  - `enum Interference { Apart(Distance), Touching { contact: Contact,
    at: Point3 }, Overlapping { volume: f64, a_in_b: bool, b_in_a: bool } }`
    and `enum Contact { Point, Curve, Face }`. *Refines the idea's four
    variants:* `Contains` is `Overlapping` with a flag, so two equal solids
    (each inside the other) need no fifth case.
  - **How**: `distance(a, b)` first, pruned by `FaceTree` as it is. A
    positive distance is `Apart`. At zero, on `m.clone()`: `pave::build`
    once, then the `Common` selection over it (`result::boolean`), then
    `measure::mass_properties` of the result. The clone and everything on
    it is dropped. `BooleanReason::Empty` is touching at a point or a
    curve, `InputReason::ZeroThickness` touching at a face; the contact's
    dimension comes from the pave model already built (flush pairs with
    area, section curves and common blocks of positive length, else the
    hits). `a_in_b` / `b_in_a` come from the selection's piece
    classification (no piece of `a`'s boundary `Outside` `b`), exposed
    `pub(crate)` from `boolean/result.rs`.
  - `interference_many(&Model, &[Body], &Control) ->
    Result<Vec<Clash>, OpError>`, `struct Clash { operands: [usize; 2],
    verdict: Interference }`: body boxes through a tree, then
    `interference` on every candidate pair whose distance is zero; apart
    pairs are omitted; pairs ascending as `interferences_many` orders them.
  - No new error variant: the refusals are `distance`'s and `common`'s.
    ARCHITECTURE §Operations gains the query; `boolean::interferences`
    (the pave model) keeps its name.
- **`arris-debug` / `tools/oracle`**: an `interference` query form,
  `{"interference": {"a": {"step": …}, "b": {"step": …}}, "expect":
  "apart" | "touching" | "overlapping"}`. `expected.py` answers with
  `BRepExtrema_DistShapeShape` (distance, `InnerSolution`),
  `BRepAlgoAPI_Common` + `GProp_GProps` (volume), and the containment by
  the common's volume against each operand's. Contact dimension from
  `BRepAlgoAPI_Section`'s result; where it cannot tell a face contact from
  its boundary curve, `analytic.query_differs` records it (ADR-0053 §8).
  `tests/fixtures/README.md` §Query fixtures extended.
- **`arris-py`**: `Model.interference(a, b)` and
  `Model.interference_many(bodies)`, `Interference` / `Contact` / `Clash`
  classes, stub and docstring examples.
- **ADR-0055**: *interference is distance's contact and common's volume*:
  the three verdicts, touching vs overlapping read as `common` reads it
  (under the tolerance a sliver is zero thickness), the contact dimension,
  containment from the selection, the scratch clone and its measured cost,
  the many-body screen, the query form. Reference trees read: Open
  CASCADE's `BRepExtrema_ShapeProximity` and `BOPAlgo` interference
  reporting, for what a clash report holds.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — `ops::query::interference` for two solids with the
  public types: distance first, the scratch clone, pave + `Common` +
  `mass_properties`, `Empty`/`ZeroThickness` to `Touching`, `a_in_b` /
  `b_in_a` from the selection. `Contact` is provisional here (`Face` from
  `ZeroThickness`, else `Point`); step 3 makes it exact.
  - **The step that settles the method**: measure, in a release build,
    the clone (first append copies `Indices`) and the common on the real
    tier's largest part against a pin through it, for the ADR. If the
    clone dominates, copy only the two bodies' closures into a fresh
    model instead, decided here. *Measured (release, a 400 mm plate with
    400 holes, 406 faces, against a press-fit pin): the clone 630 ns — a
    list of chunk pointers — against 2.5 s for the pave and common; the
    scratch clone stays, nothing is copied by closure.*
  - Unit tests, named poses: two boxes apart, on a face, on an edge, at a
    corner, crossing, one inside the other, equal; a ball inside a block
    with no vertex inside it (ADR-0054 §1's case); a pin in a hole with
    clearance (apart) and press-fit (overlapping). Rustdoc with an example.
- [x] Step 2 **[1]** — ADR-0055 (above), with step 1's measured cost and
  the containment rule.
- [x] Step 3 **[2]** — The contact dimension from the pave model: `Face`
  for a flush pair whose overlap has area, `Curve` for a section curve or
  common block of positive length (two cylinders side by side, a box on a
  cylinder's ruling), `Point` otherwise (two balls), and `at` a deterministic point of it:
  `distance`'s own closest point, already the least by entity id and
  parameter (ADR-0054 §2). Tests for each, both operand orders. *Step 3
  found that `common` refuses a ball on a plane and a cone's apex on a face
  (a point tangency), so those poses became a backlog line and stay out of
  the tests; the face contact is the `ZeroThickness` of step 1, which
  needs no pave-model reading.*
- [x] Step 4 **[2]** — The `interference` query form in `arris-debug` and
  `tools/oracle` (`expected.py`, `query.py`, `selftest.py`), the runner's
  comparison (verdict exact, volume within the fixture's tolerance scaled
  to volume as `measure` fixtures are, containment flags, the contact
  dimension unless `query_differs`), `tests/fixtures/README.md`
  §Query fixtures. *Landed with one starter fixture,
  `query/interference-primitives` (the cube poses and two cylinders along a
  ruling, both orders), which step 5 extends. The open question below is
  settled: the section alone cannot tell, the pairwise common of faces can.*
- [x] Step 5 **[1]** — The corpus: `query/interference-primitives` (the
  step 1 and step 3 poses, sphere, cylinder, cone and torus among them),
  `query/interference-parts` (a drilled real-tier plate against a pin at
  clearance, line-to-line and press fit; a bracket against its bolt),
  `query/interference-nurbs-refused` (a NURBS face crossing the other
  operand: `Unsupported` naming it; the same pair apart: answered).
  *Step 5 found the plan wrong on the apart pose: `distance` itself
  refuses a NURBS face against a box's edge (the backlog's NURBS extrema
  line), so the apart pose is refused too and the fixture says so with the
  new `expect_refused: "unsupported"` query field. The NURBS cylinder is
  never answered but by a point query. The primitives fixture gained a ball
  with no vertex inside the block, a ball, a torus and a frustum through a
  face; two tangent balls stay out (step 3's backlog line).*
- [ ] Step 6 **[2]** — Properties (`prop_shards!`, `prop::body` operands
  in random poses): not apart ⇔ `distance ≤ tolerance.linear`; the volume equals
  `common`'s `mass_properties` volume bit for bit; swapping operands swaps
  the flags and keeps the rest; translating both keeps the verdict;
  `vol(a) = volume + vol(cut(a, b))` within the fixture tolerance where
  overlapping; the model unchanged (`dump` before and after equal).
- [ ] Step 7 **[2]** — `interference_many` and `Clash`: body-box pruning,
  ascending pairs, `Control` polled per pair. Tests: the result equals
  `interference` over every pair by brute force on a random assembly of
  `prop::body` bodies; a 20-body assembly's release cost measured and
  written into the ADR's consequences (whether the verdict-only screen
  stays a backlog line).
- [ ] Step 8 **[1]** — Python: `Model.interference`,
  `Model.interference_many`, the classes, `_arris.pyi`, docstring
  examples, pytest cases; `CHANGELOG.md` `Unreleased` bullet (new API,
  nothing under `Breaking`).

## Acceptance
`cargo nextest run --workspace` with the `query/interference-*` fixtures
green against the oracle (verdict, volume, containment, contact), checker
green on every scratch common in debug builds, step 6's properties at the
gate's case count, step 7's brute-force equality, and CI's `python` job
(pytest, docstrings, `mypy.stubtest`). This is the roadmap's C8 acceptance
line "interference agreeing with the boolean's common", held as an
identity.

## Docs to update on completion
- `docs/ARCHITECTURE.md` §Operations — `ops::query::interference` and
  `interference_many` beside `distance`; the crate table's query list
  (line 24).
- `docs/ROADMAP.md` §C8 — status line: interference landed (ADR-0055);
  the planar section remains.
- `docs/BACKLOG.md` — the volume without assembling the common (idea
  option B); the verdict-only screen, unless step 7 took it.
- `tests/fixtures/README.md` §Query fixtures — the `interference` form
  (step 4 writes it; check it at retirement).
- `CHANGELOG.md` — the bullet (step 8).
- `AGENTS.md` current state — C8 line: interference landed.

## Open questions
- Settled in step 1: scratch `Model::clone`, by its measured 630 ns.
- Settled in step 4: `BRepAlgoAPI_Section` alone reports only a face
  contact's boundary curves, so the oracle tells `face` by the
  `BRepAlgoAPI_Common` of face pairs having area; `query_differs` is not
  needed for contact.
