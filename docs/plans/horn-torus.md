# Plan: horn-torus

- Started: 2026-10-31
- Milestone: M6 (cycle C11, docs/ROADMAP.md §C11, "`BlendTooLarge` where Open CASCADE builds: the horn torus on an arc")
- Idea: none; verbatim from the human: "the horn torus (`BlendTooLarge`)", with the backlog line "A horn torus on a blend's arc"

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
A fillet whose ball's centre circle is not above `r` — a horn torus
(`R = r`, `regression/tangent-chain-horn-torus`) or a spindle torus (`R < r`,
`blend/cylinder-rim-spindle-fillet`) — builds as Open CASCADE builds it:
a `Surface::Torus` face held to the torus's **regular band**, where the
surface point is off the axis (`R + r cos v` above the face's tolerance).
The data model admits any torus with `R, r > 0`; a face reaching past
the band is a checker violation by name. The new faces are operands of
every operation a ring-torus face is (section, boolean, tessellation,
offset, STEP), so the blend's output stays operable (closure before
breadth). What stays `BlendTooLarge` at the ring is a contact at the axis
or a stripe whose arc of the tube runs past the band, each naming the face.

## Non-goals
- A revolve of an arc whose circle crosses the axis stays
  `SweepReason::SpindleTorus` (a backlog line: the revolve of an arc on
  the band).
- Open CASCADE's sphere where the stripe closes on the axis, and a contact
  at the axis: stay `BlendTooLarge` naming the face.
- The ellipse run-over and walked run-overs (backlog, ADR-0071).
- Any torus face on the lemon (inner) sheet of a spindle torus.

## Design deltas
- **ADR-0080, a torus face holds the regular band** (step 1): the
  `Surface::Torus` invariant goes from `r < R` to `R, r > 0`; a face's
  domain keeps `R + r cos v > tol_face`. On the band `(u, v)` from
  `atan2` is the unique inverse, so evaluation and inversion are
  unchanged; a point on a spindle's lemon is not on the band's surface. A
  section keeps only branches on the band, split at the singular point(s)
  on the axis. Names the trace, walk and ring modules it touches.
- `docs/DATA-MODEL.md` torus row (`R > r` → the band) and the meridian
  table row at line ~498; `Surface::Torus` rustdoc (`r < R in cycle 1`).
- **Public type change (break):** `arris_check::FaceFault::OffTorusBand {
  loop_index, coedge }` (F1), mapped in `crates/arris-py` with its class,
  attributes and `_arris.pyi` line; `CHANGELOG.md` `### Breaking`.
- `arris-ops` `blend/ring.rs`: the `c.x <= radius + tolerance` refusal
  becomes the band test over the stripe's `[v_a, v_b]` (its minimum is at
  `v = π` when the arc crosses it, else at a contact); `ARCHITECTURE.md`
  §blends (line ~895) and the `TooLarge` row of the error table reworded.
- `arris-io` STEP reader: `torus()` takes a non-ring torus;
  `Refusal::SelfIntersectingTorus` is narrowed to `major <= 0` (no variant
  change, no break).
- `arris-debug` `sample.rs` samples a band face; the census's horn/spindle
  cause keeps its name for what the band still refuses.

## Steps
Complexity grades; a step's grade is its tier, the model that executes it
(`.agents/tiers.md`), and `**[1, tier 3]**` overrides the tier: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — ADR-0080 and the geometry: the `Surface::Torus`
  invariant and rustdoc, the data model's rows, `sample.rs`; a property
  test over random `R/r` in `(0.3, 3)` with `R = r` drawn on purpose:
  `inverse(eval(u, v)) = (u, v)` and a unit normal for every `v` on the band.
- [x] Step 2 **[2]** — The checker's F1 `OffTorusBand`: a torus face whose
  pcurves or `v` span reach `R + r cos v ≤ tol` is refused by name; a
  hand-built horn face over its axis point names it, every existing
  corpus fixture unchanged. Py binding, stub and `Breaking` bullet in the
  same commit.
- [x] Step 3 **[3]** — Sections on a horn and a spindle torus: plane,
  cylinder, cone, sphere, torus and NURBS partners keep only band branches,
  split at the axis point(s). The six-partner tables in `trace_torus.rs`
  (its `ring_of`) and the walk's run over horn and spindle; property:
  every section point within tolerance of both surfaces and on the band,
  each branch through the singular point ending at a vertex there.
- [x] Step 4 **[2]** — The ring builds the band: the band test replaces
  the ring test in `ring.rs`; `regression/tangent-chain-horn-torus` moves to
  `blend/` with its blessed dump against its Open CASCADE values;
  `blend/cylinder-rim-spindle-fillet` flips from refusal to build
  (`fixtures:` commit, the oracle values unchanged); a stripe past the band
  stays `BlendTooLarge` naming the face (a fillet test).
- [x] Step 5 **[2]** — The output is an operand: a box cut through the horn
  blend's face and a cylinder fused across the spindle blend's, each a
  fixture with its Open CASCADE oracle; the mesh corpus tessellates both
  blends watertight with normals off the axis.
- [x] Step 6 **[1]** — STEP: the reader takes a non-ring torus, the
  refusal narrowed to `major <= 0`; the horn and spindle blends written and
  read back with equal measures (an `io` round-trip test).
- [x] Step 7 **[2]** — Offset of a band face: `r ± d` that keeps the face
  on the band offsets; one that drives it past the band is
  `SurfaceCollapses` naming the face; `offset_prop` draws horn and spindle
  faces.
- [x] Step 8 **[2]** — Blend properties: a cylinder rim filleted at random
  `r` from a ring torus through horn to spindle, short of the contact at
  the axis, its volume held to the Pappus closed form; the cap-edge chain
  at random `r2/r` in `(0.3, 0.7)`, checker-green or refused by name.
- [ ] Step 9 **[2]** — The census: the run-over census re-run over the
  four fetched parts; the horn/spindle cause's edges (98 of 277 at the
  finding) and STC-06's two sampled edges each built and matching Open
  CASCADE's measures, or refused by a name; a `debug` test holding the
  counts, the numbers printed for the roadmap.

## Acceptance
`cargo nextest run` over the `blend/` areas with
`blend_tangent_chain_horn_torus` and `blend_cylinder_rim_spindle_fillet`
building, checker-green at `Full`, volume, area, centroid and counts within
each fixture's tolerance of Open CASCADE's; step 5's cut and fuse fixtures
the same; the band properties (steps 1, 3, 7, 8) at 256 cases in the full
gate; step 9's census counts printed, every horn or spindle edge built or
refused by name; the real parts' `fillet` row still 0 of 38, the script
corpus's `fillet` refusals not above 1; no panic.

## Docs to update on completion
- `docs/DATA-MODEL.md` §Surfaces torus row, §meridians row — the band (if
  not landed in step 1).
- `docs/ARCHITECTURE.md` §blends (ring refusals) and the error table's
  `BlendReason::TooLarge` and checker F1 entries.
- `docs/ROADMAP.md` §C11 — tick the horn torus line with the census counts.
- `docs/BACKLOG.md` — delete the horn torus line (line ~80) and trim the
  `BlendTooLarge` clause of line ~71; add the revolve-on-the-band line.
- `docs/adr/0035`, `0036` — an amendment line pointing at ADR-0080 for the
  horn and spindle refusals they record.
- `GLOSSARY.md` — **regular band** (of a torus): the part of a torus off its
  axis, where a face may lie; **horn torus**, **spindle torus** if missing.
- `CHANGELOG.md` `Unreleased` — fillets whose torus is horn or spindle now
  build (`/retire-plan`); the `Breaking` bullet lands in step 2.
- `AGENTS.md` current state — M6 done.

## Open questions
- *Settled* (step 5): the cut is `blend/horn-blend-box-cut` (box
  `[1.9, -1, 1.95]-[3, 0.1, 3]`, planes `x = 1.9`, `y = 0.1`, `z = 1.95` off
  the axis) and the fuse `blend/spindle-blend-cylinder-fuse` (a radius 0.3
  cylinder along `y` through `x = 1.5`, `z = 2.6`); both build at once, as
  Open CASCADE does. The mesh half is `arris-mesh/tests/band.rs`: the four
  blends closed, torus-face vertices off the axis, triangles facing outward.
- *Settled* (step 4): the reader's narrowing (step 6's `torus()` change,
  `SelfIntersectingTorus` at `major <= 0`) was pulled into step 4, since the
  built spindle and horn fixtures are written and read back by the corpus;
  step 6 keeps its box and the horn/spindle blend round-trip test.
  `blend/tangent-chain-horn-torus` moved to `blend/` with Open CASCADE's
  values. No fillet test for a stripe past the band exists to build: an
  arc crossing `v = pi` reaches `R - r`, which for a rim, a boss root or a
  hole's rim is the wall's own radius, never at the tolerance but for a
  wall of no radius; the contact at the axis (`R = r + 0`) is the
  too-large case and is held by
  `a_fillet_on_the_band_builds_and_one_at_the_axis_is_too_large`.
- *Settled* (step 3): spindle in scope. The section is traced on the
  torus's polynomial whole — the walk's distance to a torus covers the
  lemon too (`Implicit::sectioned`), so its roots are the patches' — and
  the band is decided after (`torus_walk/band.rs`): an axis point is a
  singular point of the trace, so every branch is on the band or off it
  whole and is kept or dropped whole, the points on an axis merged to one.
  The shared-axis arm drops its lemon and axis meetings the same way.
  A surface tangent along a horn's tube circle — every blend junction of
  a horn (step 4's finding) — is tangent to the axis at the axis point the
  circle passes through, so the quotient by the circle vanishes twice
  along the collapsed row `v = π`: the row is divided out too, on the
  patches by `(1 − t)²` and `t²` of the quarter turns meeting there and in
  the walk by dropping the closed form's leading `ρ`; the rest keeps one
  sign along the row and stays off the axis point, refused
  (`CrowdedSingularity`) where it does not. Refused by name, a backlog
  line: a plane holding a horn's or a spindle's axis (`TubeCircle` in the
  tracer; the shared-axis arm answers a horn's two tube circles whole and
  refuses a spindle's, which cross into the lemon, `Unsupported`), a
  surface crossing a horn along a tube circle (`TubeCircle`: the quotient
  by it double along the row), and one touching a horn's axis point alone,
  tangent to its axis (`TangentAlongCurve`).
- ⚠ OPEN: the band's margin — agent, by step 1. Pick: the face's tolerance
  as a distance from the axis, the same measure `contacts_on` uses, so the
  ring and the checker refuse at one bound.
- ⚠ OPEN: `Refusal::SelfIntersectingTorus` — agent, step 6. Pick: kept and
  narrowed to `major <= 0` rather than removed, so the reader's refusal
  enum does not break.
