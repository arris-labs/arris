# Plan: step-unit-and-sheet-bodies

- Started: 2026-10-12
- Milestone: C9, the healing cycle (docs/ROADMAP.md §C9), its first part
- Idea (verbatim from the human): "C9 (or first part of it if it does not fit into one plan)"
- Ask: A12, `docs/ideas/plugin-cad-consumer-asks.md` (its row is the decision; that file stays for its other open asks)

## Goal

The plugin CAD can export STEP in metres: `step::write_with` and
`step::write_products_with` declare the caller's `LengthUnit` as the file's
`LENGTH_UNIT`, convert no coordinate, and round-trip bit-exact through
`ReadOptions` with the same unit; `write` and `write_products` stay
millimetres. Sheet bodies are first-class in the places C9's "In" list
names: `ops::planar_face` turns a `Profile` into a one-face sheet;
`SHELL_BASED_SURFACE_MODEL` and `FACE_BASED_SURFACE_MODEL` read as
checker-green `Sheet` bodies where they are refused today; the STEP writer
writes a sheet back as a surface model; `measure::surface_properties` gives a
sheet's (or any body's) area, centroid and inertia matched to Open CASCADE's
`BRepGProp::SurfaceProperties`; transform, mirror, tessellation, body bytes,
the native format and the checker take a sheet, each proven by a test. The
fetched tier's histogram, re-run, prints its `read` column: the 6 parts held
back only by surface models (CTC-02 AP242, CTC-04 AP203, FTC-09, FTC-10,
STC-07, STC-08; every one a one-face `OPEN_SHELL`: CATIA's `Fill`,
`Extract` and `Split` surfaces) leave it, 11 → 5, the 5 left all `gap past the cap`
— the second C9 plan's (sewing and orientation).

## Non-goals

- Sewing, `Refusal::Gap`, orientation healing, `ShapeFix` — the second C9
  plan, written after this one retires.
- A sheet as an operand of the boolean, split, section, blends, shell,
  offset, the multi-tool boolean or the query cycle's operations beyond what
  already accepts one (`docs/ROADMAP.md` §C9 Out); each keeps or gets its
  typed `NotSolid` refusal, and a test says so where none exists.
- Thicken (a sheet grown into a solid) — the backlog's A7 line.
- Wire bodies, faceted solids (`FACETED_BREP`, `TESSELLATED_*` stay
  `Refusal::Unsupported`), IGES.
- Scaling a model between units: the writer declares, never converts.

## Design deltas

- **ADR-0057** (step 3): *a surface model reads as a sheet body.* Decides
  where a read sheet sits in `step::Read` (recommended: in the same list,
  `Read::solids`, its `ReadBody::body` a `Sheet`, so `ProductTree`'s indices
  keep one space — see Open questions 1), what a surface model held only by
  a `CONSTRUCTIVE_GEOMETRY_REPRESENTATION` becomes (Open question 2), how
  the writer spells a sheet (`SHELL_BASED_SURFACE_MODEL` of `OPEN_SHELL`s in
  a `MANIFOLD_SURFACE_SHAPE_REPRESENTATION`, tied to the product's shape
  representation beside the solids' `ADVANCED_BREP_SHAPE_REPRESENTATION`),
  and what a surface property is (unit area density over the faces of any
  body). Amends ADR-0025 §2's list of refused shapes; ADR-0025 is not
  edited.
- **`arris-io` public API (additive):** `step::WriteOptions { length_unit:
  LengthUnit }` with `Default` = millimetres; `step::write_with(model,
  bodies, &WriteOptions)`; `step::write_products_with(model, bodies, tree,
  &WriteOptions, control)`. `write`/`write_products` call them with the
  default. `LengthUnit` gains the writer side of its mapping (an SI prefix,
  or a `CONVERSION_BASED_UNIT` for inch and foot).
- **`arris-io` guarantee change (Breaking):** `step::read`'s "every `Ok`
  body is a `Solid`" becomes "a `Solid` or, for a surface model, a `Sheet`";
  a caller that wants solids only filters on `BodyKind`. `step::write`
  accepts `Sheet` bodies; `StepError::Unsupported`'s "not a solid" narrows
  to wire and general bodies. Under `### Breaking` in `CHANGELOG.md` in the
  commit that makes it (step 3, step 6).
- **`arris-ops` public API (additive):** `ops::planar_face(m, &Profile,
  control) -> Result<(Body, Provenance), OpError>` (the sweeps' private
  `cap_face` lifted into it, M5 `⚠ OPEN` 1 closed); `measure::
  surface_properties(m, body, control) -> Result<SurfaceProperties,
  OpError>` with `SurfaceProperties { area, centroid, inertia }`.
  `mass_properties` keeps refusing a sheet with `InputReason::NotSolid`.
- **`arris` facade and `arris-py`:** each new function re-exported and
  bound in the step that adds it — `Model.write_step(..., length_unit=)`,
  `planar_face`, `surface_properties`, a read sheet arriving as a `Body`
  whose `kind` says `"sheet"` — with its `_arris.pyi` line and docstring
  example (kernel rules §API).
- **`arris-debug` survey and the oracle:** the survey reads sheets, holds
  each to Open CASCADE's surface properties of the same file entity, and runs
  the battery's `measure` (as surface properties) and `write_read` stages on
  it, never the solid-only stages; `tools/oracle` learns a `sheet` result
  (area, centroid, surface inertia; no volume) and a recipe that ends in a
  `profile`.
- **Fixtures:** a new area, `tests/fixtures/sheet/`, for planar faces,
  read surface models and their operands, held to the oracle's sheet
  result; `tests/fixtures/real/` gains surface-model excerpts.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

A12 goes first although it is not the riskiest: it is the one open blocking
ask of the first consumer and the roadmap opens the cycle with it; the
surface-model reader, the plan's real unknown, follows as soon as there is a
measure to hold it to.

- [x] Step 1 **[1]** — A12. `WriteOptions`, `write_with`,
  `write_products_with`; `LengthUnit`'s write mapping (`SI_UNIT` with its
  prefix; inch and foot as `CONVERSION_BASED_UNIT` over the millimetre, as
  the reader parses them); the uncertainty written in the same unit.
  Python: `length_unit=` keyword on `write_step` and the products writer,
  default `"mm"`. Tests: for every `LengthUnit`, a body written with it and
  read back with `ReadOptions` of the same unit is bit-identical in every
  coordinate (property over random bodies and poses); read back in
  millimetres it scales by the unit's factor; Open CASCADE reads the metre
  file of a corpus fixture to 10⁹ × its volume. `CHANGELOG.md` bullet.
- [x] Step 2 **[2]** — `measure::surface_properties`: area, centroid and
  inertia about the centroid at unit area density, over every face of any
  body, integrated as `mass_properties` integrates area (the same
  quadrature, the centroid taken about the vertices' centre for the same
  reason). Oracle: `tools/oracle/oracle/measure.py` gains
  `SurfaceProperties`; on every solid fixture its area equals
  `mass_properties`' to the fixture's `area_rel`; closed forms for a sphere's,
  a cylinder's lateral and a torus's surface; Open CASCADE's centroid and
  inertia on a battery of solid fixtures. Bound in Python.
- [x] Step 3 **[2]** — **ADR-0057**, and surface models read as sheets:
  `SHELL_BASED_SURFACE_MODEL` (its `OPEN_SHELL`s and `CLOSED_SHELL`s) and
  `FACE_BASED_SURFACE_MODEL` (its `CONNECTED_FACE_SET`s) through the solid
  reader's face, bound and pcurve path into `Builder::assemble_sheet`, with
  `Role::File` provenance and the same refusals (`Pcurve`, `OpenLoop`,
  `Gap`, `Topology` — an open shell whose faces disagree on orientation is
  `Topology` until the second plan heals it). Fixtures in `real/`: excerpts
  (`tools/oracle/excerpt.py`) of FTC-10's one-face `Extract`, STC-07's
  `Fill`, CTC-02's `Split` and a `FACE_BASED_SURFACE_MODEL` built by hand,
  each held to Open CASCADE's face count and surface properties of the same
  entity. Python: a read sheet is a `Body` of kind `"sheet"`. `CHANGELOG.md`
  under `### Breaking` (the guarantee).
  *Landed with these changes to the plan:* the fixtures are part fixtures
  (`real/surface-model-{extract,fill,split}`, `real/face-based-surface-model`)
  and the part runner and `expected.py` learned sheets (matched by face
  count and centroid, since Open CASCADE keeps no entity id; a sheet its
  reader leaves out, CTC-04's construction surfaces, carries `oracle_skips`);
  nist-ctc-02, -04, -05 and slow-gap-refusal now list their surface models
  as `read`; Python says `Model.body_kind(body)`, not a `Body.kind`; open
  questions 1 and 2 decided in ADR-0057 as recommended.
- [x] Step 4 **[1]** — `ops::planar_face`: the sweeps' `cap_face` lifted
  out, a `Profile` (holes included) to a one-face `Sheet` with provenance
  naming each profile edge's edge. Oracle: a `planar_face` recipe op ending
  in the oracle's existing `profile`; `sheet/` fixtures (a rectangle, a
  disc, a slot with two holes, an arc-and-line profile) matched in area,
  centroid and inertia; property: area equals the profile's closed-form
  area in random poses. Bound in Python. Closes M5 `⚠ OPEN` 1.
- [ ] Step 5 **[1]** — Sheet operands, each a test over a planar face and a
  read surface model: `transform` and `mirror` (kind kept, surface
  properties carried by the motion, a mirror's front turned as its faces
  are), tessellation (every triangle's normal on the sheet's front, mesh
  points on their faces within tolerance), `body::write`/`read` and the
  native format (round trip identical), the checker at `Full` (S2's
  once-used edges). Fix what refuses; the operations out of scope get a
  test naming their `NotSolid` refusal where none exists. Property: surface
  properties invariant under random rigid motions and mirrors.
- [ ] Step 6 **[2]** — The STEP writer writes sheets, as ADR-0057 spells
  them, beside solids in one file and in `write_products`. Tests: write →
  read through Arris is identical in counts and surface properties; Open
  CASCADE reads the written sheet to the fixture's area, centroid and
  inertia; every `sheet/` and surface-model `real/` fixture round-trips; a
  wire body is still `StepError::Unsupported`. `CHANGELOG.md` (the writer
  accepts sheets) in `### Breaking` beside step 3's.
- [ ] Step 7 **[2]** — The survey and the histogram: `arris_debug::survey`
  reads sheets, holds each to Open CASCADE's surface properties of its file
  entity, runs `measure` and `write_read` on it; `tools/real-parts.sh`
  re-run over the fetched tier and the committed one; the numbers written
  into `docs/ROADMAP.md` §C9 and the `fillet-by-part`/`failures` reports
  checked for a sheet that met a kernel bug (a `regression/` fixture and a
  `real-parts.waits` line if one did, per the nightly triage rule).

## Acceptance

- The full gate (`ARRIS_GATE=full`) green, the `sheet/` area and every
  `real/` surface-model fixture matched to Open CASCADE's surface properties
  within its tolerance, checker green at `Full` on every read and built
  sheet.
- `tools/real-parts.sh` over the fetched tier: the healing row's `read`
  column 5 parts (from 11), every one `gap past the cap`; no part whose
  surface model reads fails a battery stage it passed before; the number of
  the 38-part total printed (from 14).
- The A12 round trip bit-exact for every `LengthUnit`, and the plugin CAD's
  ask closed in `docs/ideas/plugin-cad-consumer-asks.md`.
- `crates/arris-py`'s pytest, doctests and `mypy.stubtest` green locally
  for the bound functions.

## Docs to update on completion

- `docs/ROADMAP.md` §C9 — status line: A12 and sheet bodies landed, the
  `read` column's numbers; what is left (sewing, orientation) named.
- `docs/ARCHITECTURE.md` "Units" row — the writer declares the caller's
  unit; §Formats and tools — surface models read as sheets, sheets written;
  the operations a sheet is an operand of, and those that refuse it.
- `docs/DATA-MODEL.md` — `BodyKind::Sheet`'s paragraph: what builds one
  (`planar_face`, the reader, the section) and what measures one.
- `docs/COMPARISON.md` — the "Sheet and non-manifold bodies" row.
- `docs/BACKLOG.md` — drop the `ops::planar_face` line and the surface-model
  half of the faceted/surface-model line (faceted stays).
- `docs/ideas/plugin-cad-consumer-asks.md` — A12 landed in the status line.
- `docs/adr/README.md` — ADR-0057's line.
- `CHANGELOG.md` — bullets checked against what the steps wrote.
- `AGENTS.md` current state — C9's first part landed; next the sewing and
  orientation plan.

## Open questions

- Decided (1): **a read sheet's place in `step::Read`** — in `Read::solids`
  with a `Sheet` body (one index space for `ProductTree`, no type change,
  the guarantee change under Breaking), or a separate `Read::sheets`
  (a public type change and a second index space in every occurrence).
  Recommendation: the same list. Agent decides in ADR-0057, step 3.
- Decided (2): **a surface model only a `CONSTRUCTIVE_GEOMETRY_REPRESENTATION`
  holds** — read it as a sheet like any other (the caller sees it in no
  occurrence of the product tree and may skip it), or keep refusing it as
  construction geometry. Recommendation: read it; the product tree already
  tells a body of the part from a construction surface, and a refusal hides
  geometry the file holds. Agent decides in ADR-0057, step 3.
- Decided (3): **whether `surface_properties` carries the inertia** or only
  area and centroid. Recommendation: carry it — Open CASCADE and Parasolid
  both give it and the quadrature is the same pass. Agent, step 2.
