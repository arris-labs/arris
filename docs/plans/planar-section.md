# Plan: planar-section

- Started: 2026-10-11
- Milestone: C8, the query cycle (docs/ROADMAP.md §C8) — its last "in" line
- Idea (verbatim from the human): "the planar section of C8"
- No idea file: the roadmap's line and ADR-0051 settle the shape; the
  decisions below are delegated to the agent and recorded by ADR-0056 in
  step 3.

## Goal
A consumer cuts a solid by a plane and gets what the plane cuts out of it
without the solid changing: a **sheet body** of planar faces lying on the
plane — one face per connected region of the section, its outer loop and
its holes being the section's wires — plus the section's area and
centroid, and a provenance record naming, for every section edge, the
body face it lies on and, for every section vertex, the body edge it lies
on. The body and every entity of the model are untouched; the sheet is the
only thing the call adds. The sheet is the cap `split` would make on its
positive side (ADR-0051) less any region flush with a face of the body,
turned to face the plane frame's `z`, so a section and a split at the
same plane agree to the last digit where nothing is flush, and every
plane-against-body case the booleans settle (through an edge, a vertex, a
seam, an apex) is the section's too. A plane that misses the body or only
touches it — along a face, an edge or a vertex — answers `None`, not an
error: a section view placed off the part is a question with an empty
answer. Answers are held to Open CASCADE (`BRepAlgoAPI_Section` for the
wires, `BRepAlgoAPI_Common` of the solid and a planar face for the region)
by a `section` form of the `query` fixture kind, and the Python binding
exposes it.

## Non-goals
- **Sections of sheets, shells and wire bodies** (open wires, points).
  The booleans take solids; a non-solid is `InputReason::NotSolid` here as
  there. Backlog line.
- **Sectioning by a surface other than a plane**, drawings, hatching,
  hidden lines, dimensions (roadmap §C8 out list). The sheet's faces
  tessellate per face like any face (C7); hatching them is the consumer's.
- **Second moments of the section** (`I_x`, `I_y`, `J` — engineering
  section properties). Area and centroid only; a backlog line.
- **Many parallel sections in one decomposition** (a slicer, a stack of
  drawing views). Each call runs its own; a backlog line with step 7's
  measured cost.
- **Lifting the booleans' refusals.** A plane tangent to a face along a
  curve while crossing elsewhere is `TangentContact`, a NURBS face crossing
  the plane `Unsupported`, beside an apex or a pole `BesideSingularity` —
  as `split` refuses them. They stay the booleans' backlog lines.
- **A section without building the faces** (wires only, cheaper): not
  asked for; the faces are what a section view fills.

## Design deltas
- **`arris-topo`, the builder makes sheets** (found at step 1: no
  operation returned a `Sheet`, and `Builder::finish` refused the kind
  with `BuildError::Kind`, `assemble` every edge not used twice).
  `Builder::assemble_sheet(&Model, tolerance, Assembly)` (new) proves
  open shells — an edge used once or twice, each shell's Euler line closing
  once its boundary cycles are capped — and `finish(m, BodyKind::Sheet)`
  stores them; `Wire` and `General` stay `BuildError::Kind`. No signature
  changes and no new variant. DATA-MODEL §Body kinds and §Euler operators.
- **`arris-ops`, `ops::section`** (new file `boolean/section.rs`, beside
  `split.rs`, exported from `lib.rs` as `section` and `Section`). It adds a
  body, so it is an operation on `&mut Model` with provenance, not a member
  of `ops::query` (whose queries take `&Model` and make nothing,
  ARCHITECTURE §Operations); it is a long call (ADR-0030).
  - `section(m: &mut Model, body: Body, plane: &Frame, control: &Control)
    -> Result<Option<Section>, OpError>`.
  - `struct Section { sheet: Body, area: f64, centroid: Point3, provenance:
    Provenance }`. `sheet` is a `BodyKind::Sheet` body of one open shell
    per face (`assemble_sheet` holds a shell to one edge-connected
    component sharing no vertex with another, and two regions of a
    section share no edge), every
    face on a surface equal to `plane` itself so its (u, v) are the
    frame's `x` and `y`, its effective normal the frame's `z`.
  - **How**: on `m.clone()` (interference's scratch clone, 630 ns measured,
    ADR-0055): `split`'s scratch box (`primitive::plane_box`, factored out
    of `split` into a shared helper), `pave::build` once,
    `result::boolean` with `Op::Common` alone; the faces `Generated` from
    the box's near face are the section (the cap of the positive side).
    They are gathered into a sheet in the scratch model
    (`Builder::assemble_sheet` over `FaceSpec::Keep` of each, reversed to
    face `z`), and `Model::import`ed into `m`; the scratch model is dropped.
    `BooleanReason::Empty` and `InputReason::ZeroThickness` are `Ok(None)`,
    as `split` maps them to `NoCrossing`.
  - **Flush regions** (a stepped block cut at the step's height, an
    overhang whose underside lies on the plane): the section is the plane's
    points *interior* to the solid — material on both sides — so a region
    where a body face lies on the plane is not section. *Settled at step
    2*: the Common's cap does not exclude it. Where the body's face and the
    near face have opposite normals (the step seen from above, the
    overhang from below) both pieces drop, but where they agree the
    coincident row keeps the **near face's** copy — `Generated` from it,
    beside the true cap `Modified` from it — not the body's as this plan
    supposed. So the section selects, among the Common's faces made from
    the near face, those whose interior point (`region2::interior_point`
    of the face's `FaceDomain` polygons, as a piece's) classifies
    `Inside` the body; `On` is flush and dropped, `Outside` a kernel fault.
    No fourth `Op`: the selection table assembles closed shells, and this
    selection is over the cap alone, after it. None left is `Ok(None)` (a
    plane flush with a top, seen from inside).
  - **Provenance**: the sheet's faces `Generated` from the body
    (`Origin::Entity(body)`); a section edge `Generated` from the body face
    it lies on (two for a section edge along a body edge the plane runs
    through: both faces); a section vertex `Generated` from the body edge it
    lies on, or from the body vertex the plane passes through
    (`Modified`-free: nothing of the body changes). Read from the scratch
    run's record through `import`'s `IdMap`. No new `Role`: the plane is
    no entity, and the outputs all have a body origin.
  - `area` and `centroid` by `integrate::region_integral` over each face's
    (u, v) region — the frame's own coordinates, so exact for lines and
    conics as `measure` is.
  - No new error variant.
- **`arris-debug` / `tools/oracle`**: a `section` query form,
  `{"section": {"body": {"step": …}, "plane": {"origin": …, "z": …, "x":
  …}}}`, expected: `none` or the face count, the loop count, the edge
  count, the total edge length, the area and the centroid. `expected.py`:
  `BRepAlgoAPI_Section(solid, gp_Pln)` for the edges and their length,
  `BRepAlgoAPI_Common(solid, a planar face past the body's box)` +
  `GProp_GProps` for area, centroid and faces. Where Open CASCADE counts a
  flush region or splits a closed section edge at a seam,
  `analytic.query_differs` records it (ADR-0053 §8).
  `tests/fixtures/README.md` §Query fixtures extended.
- **`arris-py`**: `Model.section(body, plane)` returning a `Section`
  (`sheet`, `area`, `centroid`, `provenance`) or `None`; the stub and a
  docstring example.
- **ADR-0056**: *a planar section is the split's positive cap, made a
  sheet*: the definition (interior points, flush regions excluded), `None`
  for no crossing, the scratch clone and `import`, why not a query on
  `&Model` returning curves (no faces to fill, no checker), the
  provenance, the shell layout, the query form. Reference trees read: Open
  CASCADE's `BRepAlgoAPI_Section` and `BRepAlgoAPI_Common` of a solid and
  a face, for what each returns on flush and tangent planes.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The builder makes sheets: `Builder::assemble_sheet`
  and `finish(…, BodyKind::Sheet)` (design delta above). Tests: kept faces
  as open shells — a box less its top, a cylinder's wall (an annulus),
  two faces as two shells, a closed shell — finished as `Sheet`s that pass
  the checker at `Full`; the `patch` sample re-assembled `New` is the same
  sheet up to ids; an open shell refused by `assemble` and by
  `finish(Solid)`. *Inserted at step 1, when the section's first line
  found the builder refusing every kind but `Solid`; the section moved to
  step 2 and every later step down one.*
- [x] Step 2 **[3]** — `ops::section` and `Section`: the plane box
  factored out of `split`, the scratch clone, pave + `Common`, the cap
  faces gathered into a sheet, reversed to face `z`, imported, `None` on
  `Empty`/`ZeroThickness`, `area` and `centroid`, the provenance through
  the `IdMap`. Checker green on the sheet in debug builds.
  - **The step that settles the definition**: the stepped block cut at
    its step, an overhang on the plane, a box whose top the plane is
    flush with (`None`). If the Common cap holds a flush region, add
    `Op::Section` (above) here and say so in the plan. *It does where the
    normals agree; the selection is a classification of the cap pieces,
    not an `Op` (design delta, flush regions).* A plane touching a box
    along an edge only faults in `split` too (`Fault::Split` Dangling):
    `regression/box-edge-touching-split`, ignored, a backlog line.
  - Unit tests, named poses with closed forms: a box mid-height and across
    a corner (a triangle), a cylinder across (a disc) and along its axis
    through the seam (a rectangle), a tube (a ring: one face, two loops),
    a sphere off-centre (a disc of `π(R²−d²)`), a U-bracket across both
    arms (two faces), a plane missing the body (`None`); the model's dump
    unchanged but for the sheet's closure; two runs give the same ids.
    Rustdoc with an example.
- [x] Step 3 **[1]** — ADR-0056 (above), with step 1's builder change
  and step 2's flush finding (the coincident row keeps the near face's
  copy where normals agree; the cap pieces classified `Inside`); ARCHITECTURE §Operations gains `section` beside `split` in the
  same commit.
- [ ] Step 4 **[2]** — The `section` query form in `arris-debug` and
  `tools/oracle` (`expected.py`, `query.py`, `selftest.py`), the runner's
  comparison (`none` exact, counts exact unless `query_differs`, length,
  area and centroid within the fixture's tolerance scaled as `measure`
  fixtures are, the sheet checker-green at `Full`), `tests/fixtures/README.md`
  §Query fixtures; one starter fixture, `query/section-primitives` with
  the box poses.
- [ ] Step 5 **[1]** — The corpus: `query/section-primitives` extended
  (step 2's poses, a cone across and through its apex, a torus across its
  hole — two rings — and through its axis — two discs), `query/section-flush`
  (the stepped block, the overhang, the flush top, each against Open
  CASCADE with `query_differs` where it counts the flush region),
  `query/section-parts` (a real-tier drilled plate across its holes and
  along them; the filleted and the shelled box), `query/section-refused`
  (a plane tangent to a cylinder's ruling while crossing a box fused to
  it: `TangentContact`; across a NURBS face: `Unsupported`).
- [ ] Step 6 **[2]** — Properties (`prop_shards!`, `prop::body` in random
  poses, random planes through the body's box): every section edge lies on
  the plane and on each body face it names, within its tolerance; the
  section's area equals the area of `split`'s positive cap bit for bit
  (a random plane is flush with no face)
  (and the negative cap's within tolerance) where the split answers; the
  plane's `z` reversed gives the same area and centroid and the opposite
  normal; translating body and plane together translates the centroid;
  the model is unchanged but for the sheet (`dump` of the body's closure
  before and after equal); `None` exactly when `split` refuses
  `NoCrossing`.
- [ ] Step 7 **[1]** — Python: `Model.section`, the `Section` class,
  `_arris.pyi`, docstring example, pytest cases; `CHANGELOG.md`
  `Unreleased` bullet (new API, nothing under `Breaking`). Measure a
  20-section stack of the real tier's largest part in release for the
  backlog line on parallel sections.

## Acceptance
`cargo nextest run --workspace` with the `query/section-*` fixtures green
against the oracle (none, counts, edge length, area, centroid), the
checker green on every sheet the section returns, step 6's properties at
the gate's case count, and CI's `python` job (pytest, docstrings,
`mypy.stubtest`). This is the roadmap's C8 acceptance line "a section's
wires lying on the plane and the body's surface", held as a property, and
its "each query a corpus … `BRepAlgoAPI_Section`" for the section.

## Docs to update on completion
- `docs/ARCHITECTURE.md` §Operations — `section` beside `split` (step 3
  writes it; check it at retirement); the crate table's operation list
  (line 24).
- `docs/ROADMAP.md` §C8 — status line: the planar section landed
  (ADR-0056); every "in" line done, the cycle ready for `/close-cycle`.
- `docs/BACKLOG.md` — sections of non-solids; second moments of a
  section; many parallel sections in one decomposition (with step 7's
  number).
- `tests/fixtures/README.md` §Query fixtures — the `section` form (step 4
  writes it; check it at retirement).
- `CHANGELOG.md` — the builder's bullet (step 1), the section's (step 7).
- `AGENTS.md` current state — C8 line: the planar section landed, the
  cycle's "in" list done.

## Open questions
- Settled in step 2: the `Common` cap keeps a flush region where the
  normals agree; the section keeps the cap pieces whose interior point is
  `Inside` the body, no `Op::Section`.
- Settled in step 1: one open shell per face, as `assemble_sheet` holds
  shells to edge-connected components that share no vertex.
- ⚠ OPEN: does Open CASCADE's `Common` of a solid and a face keep a flush
  region, and does its `Section` split a closed section curve at a seam?
  Agent, settled in step 4 by the starter fixtures; each difference a
  `query_differs` entry.
