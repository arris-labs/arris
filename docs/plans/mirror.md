# Plan: mirror

- Started: 2026-09-30
- Milestone: C5 — the consumer's API, its fourth line (docs/ROADMAP.md §C5, "Mirror (A11)")
- Idea: the ask is `docs/ideas/plugin-cad-consumer-asks.md` A11 (the idea
  stays until C5's last plan absorbs it); no idea of its own — the roadmap
  fixed the shape, the ADR in step 1 fixes the rest
- Idea (verbatim from the human): "mirror"

## Goal

A consumer can reflect a body in a plane: `ops::mirror(model, body, &plane,
&control)` returns a new solid that is the body's mirror image, checker-green,
with every face's orientation flipped where the geometry demands it so the
solid still has its material inside, and a `Provenance` that records every
vertex, edge, face, shell and the body itself as `Modified` one to one from
the entity it mirrored. A mirrored corpus fixture matches Open CASCADE's
mirror in volume, area, centroid, counts and point classifications; a
mirrored body reflects back to the original's measures; and it survives what
any body survives — booleans, tessellation, STEP and body bytes.

## Non-goals

- No general affine map: no scale, no shear, no non-uniform scale. A
  reflection is the one improper isometry a mirrored component needs;
  everything else stays out.
- `transform` keeps its signature and its rigid `Isometry`. A mirror is not
  smuggled through it (an `Isometry` is a proper rotation by construction).
- No mirror of a sub-body (a face, a shell), of a sheet, or of an assembly;
  a body in a model, as `transform` takes one. Mirror-and-fuse (a symmetric
  part from a half) is the consumer's composition of `mirror` and `fuse`.
- No `Frame` of either handedness: frames stay right-handed (DATA-MODEL
  §Conventions). The reflection is absorbed by a parametrisation change,
  not by weakening that invariant.
- No pattern operations (linear or circular), which would sit beside it.

## Design deltas

- **ADR-0031, mirror** (step 1, written). Decisions, with the alternatives
  weighed there:
  1. **A reflection is its own type**, `arris_math::Reflection`, not an
     `Isometry` variant.
  2. **Frames stay right-handed; a quadric's `u` is reflected**
     (`u ↦ 2π − u`, frame `X′ = R X`, `Y′ = −R Y`, `Z′ = R Z`); a plane
     and a NURBS surface keep their parameters (plane frame `Z′ = −R Z`,
     control net reflected). A curve needs no map at all: its image is
     the same curve in the same parameter.
  3. **Pcurves** are reused where the map is the identity and reflected
     (`Curve2::reflected`) where it is not.
  4. **Orientation and loops.** The effective loop of an image is the
     reverse of the mirror image of the original's. A plane's or NURBS
     face toggles its use orientation and keeps its stored loop; a
     quadric face keeps its use orientation and has its stored loop
     reversed, each coedge use toggled.
  5. **Provenance** is `Modified`, one to one.
  6. **The op** is built over `Assembly::of_body` and `Builder::assemble`
     like `transform`, with two provided hooks on `GeometryRemap`.
- **New public API, `arris-math`:** `Reflection` (`new(origin, normal)`,
  `plane_through(origin, normal)`, `apply`, `apply_vec`, `apply_unit`,
  `apply_frame`) and its `Degenerate` refusal for a zero or non-finite
  normal. `Frame` gains no mirrored constructor — the frame it returns
  from `apply_frame` is right-handed by construction.
- **New public API, `arris-geom`:** `Curve::mirrored(&Reflection) ->
  Curve`, `Surface::mirrored(&Reflection) -> (Surface, ParamMap)` and
  `NurbsCurve`/`NurbsSurface::mirrored`; `Curve2::reflected()` (the
  `u ↦ 2π − u` image); `ParamMap::{Identity, ReflectU}`.
- **New public API, `arris-topo`:** `GeometryRemap` gains two provided
  hooks, `pcurve(model, p, surface) -> Curve2Id` and `face(model, surface)
  -> FaceRemap { toggle_use, reverse_loops }` (defaults are the identity,
  so `transform` and `KeepGeometry` are untouched), and `Assembly::of_body`
  applies them: it toggles the face use and copies a loop backwards, each
  coedge use toggled, where told to. This is a
  change to a public trait, named in the step's commit and under
  `CHANGELOG.md` `### Breaking` (a consumer with its own `GeometryRemap`
  gets a default method, so it compiles; the entry says so).
- **New public API, `arris-ops`:** `pub fn mirror(m: &mut Model, body: Body,
  plane: &Reflection, control: &Control<'_>) -> Result<(Body,
  Provenance), OpError>`, re-exported by `arris`, with a doc example.
- **Corpus grammar (`arris-debug`):** a `mirror` step (`of`, `plane:
  {origin, normal}`) in the fixture recipe, in the property recipe's draw
  (the twelfth operation of the eleven the differential draws from —
  ROADMAP §Beside the cycles gets its count updated), and in
  `tools/oracle/oracle/recipe.py` as `BRepBuilderAPI_Transform` with
  `gp_Trsf.SetMirror(gp_Ax2)` and copy on.
- **`docs/ARCHITECTURE.md`** §Operations (the op), §Errors and the
  operation contract (nothing new: an existing error set), provenance
  table. **`docs/DATA-MODEL.md`** §Conventions (the note on why frames stay
  right-handed and where the reflection went), §Orientation (the rule of
  decision 4).
- **Crate boundary:** unchanged; `arris-math` gains `Reflection`.
- **Callers:** none change, but every dispatch on `GeometryRemap`
  implementors is checked (`arris-io`'s import path uses `KeepGeometry`).

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[1]** — ADR-0031, mirror: the six decisions above, the
  alternatives (a left-handed frame, an `Isometry` widened to improper
  maps, a per-face reflection flag, mirror as a `transform` option), the
  per-surface parameter-map table stated (plane, cylinder, cone, sphere,
  torus, elliptic cylinder, NURBS) and the per-kind orientation rule; the
  ADR index updated. Docs only.
- [x] Step 2 **[3]** — the riskiest unknown first, the geometry alone:
  `Reflection`; `Curve::mirrored`, `Surface::mirrored`, NURBS `mirrored`,
  `ParamMap`, `Curve2::reflected`. No topology yet. Tests, as
  properties over random frames, radii, planes and parameters: for every
  analytic kind and NURBS, `mirrored.eval(map(u, v)).point ==
  reflection.apply(orig.eval(u, v).point)` (a curve has no map) to rounding; the surface normal is `R n` for a quadric and `−R n` for a plane and a
  NURBS surface, as ADR-0031's table says; the mirrored frame is right-handed and `Frame::from_orthonormal`
  accepts it; a periodic range keeps its convention (`[0, 2π)`); a pcurve
  reflected by the map, evaluated on the mirrored surface, is the mirror of
  the original's point in 3D. Degenerate cases named as fixtures of the
  step: a plane through the surface's axis, one parallel to it, and one
  normal to it; a sphere's poles and a torus's seam.
- [x] Step 3 **[2]** — `ops::mirror` over `Assembly::of_body`: the
  `pcurve` and `face` hooks on `GeometryRemap` with
  `of_body` applying them (the loop reversal is the step's careful part),
  the op, its provenance one to one,
  `OpError::Interrupted` under a `Control` (ticks per face like `transform`),
  the doc example. Tests: the two `transform/` fixtures mirrored in three
  planes each (axis-aligned, oblique, through the body), the checker at
  `Full` green, volume equal, centroid reflected, every face's counts
  equal, provenance one to one with every entity of the input listed once;
  a mirror of a mirror in the same plane matches the original's dump
  within the fixture's tolerance; the same three interrupt assertions as
  `cancellation` (budget `k < N` rolls back, `N` gives the dump).
- [x] Step 4 **[2]** — the corpus: `Step::Mirror` in the fixture
  recipe, `mirror` in `recipe.py` (and `selftest.py`), the oracle's
  measures cached like any other. Fixtures under `tests/fixtures/transform/`
  with oracle values from Open CASCADE: a box, a cylinder posed off-axis
  (`posed-cylinder`), a hollow ring (`moved-hollow-ring`), a sphere and a
  torus segment, a blended body, and a body with an NURBS face (a fitted
  section from `boolean/`), each mirrored in a plane through, beside and
  across it. Area, volume, centroid, counts, point classifications match
  within each fixture's tolerance; the corpus lint holds the area to
  passing fixtures.
- [x] Step 5 **[2]** — a mirrored body is a first-class body: booleans on
  a mirrored operand (`cut`, `fuse`, `common` of a body and its own mirror,
  through a symmetric plane and across it) against the oracle;
  `tessellate` watertight and outward, mass properties; the STEP round
  trip (write, read, checker-green, same measures) and body bytes
  round-trip. Fixtures in `boolean/` and `transform/` with oracle values.
  Anything the mirror exposes that does not pass becomes a
  `tests/fixtures/regression/` fixture, `#[ignore]`d with the desired
  assertion, and a backlog line — not a workaround inside `mirror`.
- [ ] Step 6 **[2]** — the properties: `mirror` in the property recipe's
  draw (`prop::recipe`) so the differential, the algebraic identities
  (volume preserved, `mirror ∘ mirror` = identity, mirror of a fuse = fuse
  of the mirrors) and step 7 of `cancellation`'s interrupt property cover
  it; sharded with `prop_shards!`. `arris` re-exports `Reflection` and
  `mirror`.

## Acceptance

- `cargo nextest run --workspace` green with `parallel` on and off, the
  corpus included: every existing fixture still matches its oracle and its
  dump (no fixture expectation changes, save the ones step 4 adds).
- The mirrored corpus fixtures (steps 4 and 5) match Open CASCADE's
  volume, area, centroid, counts and point classifications — ROADMAP
  §C5's "a mirrored corpus fixture matches the oracle's mirror".
- Step 2's parameter-map properties green at their configured case count,
  for every analytic kind and NURBS.
- Step 6's properties green: differential agreement or a named
  exclusion, `mirror ∘ mirror` identity, every drawn operation interrupted
  at a random step leaves the model as it was.
- `cargo build --target wasm32-unknown-unknown` for every crate.
- `tools/semver-gate.sh` green: the `GeometryRemap` change is under
  `CHANGELOG.md` `### Breaking`.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Operations — `mirror` beside `transform`, its
  provenance; the `GeometryRemap` hooks in the builder section;
  crate table (`arris-math` holds `Reflection`).
- `docs/DATA-MODEL.md` §Conventions (frames stay right-handed; a mirror is
  a parametrisation change) and §Orientation (the rule of ADR-0031
  decision 4).
- `docs/ROADMAP.md` §C5 — mirror's line to done with ADR-0031, the status
  line; §Beside the cycles — the differential's operation count (eleven →
  twelve) and the histogram numbers if the recipe draw changes them.
- `docs/BACKLOG.md` — the lines steps 4–6 find; "patterns" if a consumer
  would compose them from mirror.
- `CHANGELOG.md` `Unreleased` — the feature bullet (mirror a body in a
  plane, orientation and provenance handled); the `Breaking` bullet for
  `GeometryRemap` is written by step 3.
- `AGENTS.md` current state — C5's "Next" drops mirror; STEP product
  structure stays.

## Open questions

- Found in step 4: a NURBS-faced result cannot be a `transform/` fixture —
  `check_stage` holds the checker's NURBS face pairs to "nothing
  unchecked" and the corpus has no fixture with a NURBS result that is not
  a refusal — so the body with an NURBS face is
  `crates/arris-ops/tests/mirror.rs`'s test over `nurbs-box.step` against
  the oracle's volume and area. The ring torus's fixture is
  `regression/mirror-torus-ring`: Arris's mirror matches the oracle in
  every stage, and only its reader's refusal of Open CASCADE's STEP of the
  mirrored ring (a left-handed torus frame) fails (backlog line). The
  torus, cone, sphere, elliptic cylinder and blend mirrors are held
  clean and as their own mirror images by `ops/tests/mirror.rs`, and the
  cone, sphere, elliptic cylinder and blend by fixtures as well.

- Decided in step 1 (ADR-0031 §2, §6): a quadric's `u ↦ 2π − u` with
  `Z′ = R Z`, a plane and NURBS on the identity; the hooks are
  `pcurve(model, p, surface)` and `face(model, surface) -> FaceRemap`.
  Working it out found that the loops of a quadric face must be reversed,
  which the plan's first draft did not have: step 3 carries it.
- Found in step 5: the boolean fixtures of a body and its mirror are
  `boolean/mirror-{box,cylinders}-{fuse,cut,common}` and
  `mirror-ball-corner-cut-fuse`, and `transform/mirror-fused-boss` is the
  mirror of a fuse. Three deviations, none of them `mirror`'s. (a) A cut of
  a body by its own oblique mirror image is refused by Arris as
  `Degenerate` (the pieces meet along the plane's edges) where Open
  CASCADE returns a solid of shells touching along edges; so the cut
  fixtures have no oblique variant, and the oblique cylinders' fuse and
  common are fixtures of their own, since Open CASCADE cuts one more
  section arc at an ellipse's parameter origin (`counts_differ`, the
  convention an earlier fixture records). (b) A ball united with its own
  mirror through its centre is coincident spheres under frames a half turn
  apart: the split fails with a dangling face, and the same without any
  mirror (a ball and its half-turned copy), so that is
  `regression/coincident-spheres-rotated-frame` and a backlog line, not a
  `mirror` workaround. (c) Measures of the fuses through fitted or
  quadric-pair sections agree with the oracle to 3e-9 to 1e-8, not 1e-9:
  their fixtures state `tolerances` of 5e-9 and 2e-8. The plan's
  tessellation, STEP and body-bytes round trips are
  `ops/tests/mirror.rs` over the same bodies as step 3 (and the runner
  does them for every fixture).
- Resolved in step 5: Arris's own STEP of a mirrored body (every quadric
  kind the tests hold, torus included) reads back clean at `Full` with the
  same measures, so the writer needs no fix for a left-handed conic pcurve;
  only Open CASCADE's left-handed torus frame is refused by the reader
  (the `regression/mirror-torus-ring` backlog line).
- `⚠ OPEN:` whether the checker, the tessellator and the STEP writer
  read a seam pair as `(0, 2π)` in that order. ADR-0031 says they take it
  by use orientation and the period. Agent, by step 3: a mirrored full
  cylinder wall at `Full` is the test; a reader that assumes otherwise
  becomes a fixture and, if small, a fix in its own commit.
- `⚠ OPEN:` whether the STEP writer already writes a mirrored body's
  left-handed conic pcurves correctly (ARCHITECTURE §STEP notes one case
  for a left-handed pcurve conic). Agent, by step 5: if it does not, the
  fix is a step of its own before 5's acceptance, not an exclusion.
