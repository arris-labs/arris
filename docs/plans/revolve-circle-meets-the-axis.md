# Plan: revolve-circle-meets-the-axis

- Started: 2026-11-19
- Milestone: C12, the training harness's refusals (docs/ROADMAP.md §C12 **In**)
- Idea: docs/ideas/revolve-circle-meets-the-axis.md (absorbed; option A, the
  harness's three programs and their closed forms)

## Goal

`revolve` builds a profile arc whose circle reaches or crosses the axis while
the arc stays on one side of it. An arc clear of the axis sweeps a band face of
a horn or spindle torus. An arc ending on the axis sweeps a torus face that
closes at a **torus pole**, one degenerate edge at the singular `v`, as a
sphere's face closes at its pole. A full circle tangent to the axis gets its
vertex at the touch point, so a partial turn builds half a horn torus and a
full turn is `NonManifold`. The harness's `cq-spindle-torus-arc-from-axis`,
`cq-spindle-torus-arc-to-axis` (with `dome_cap`'s centre 1.3e-4 off the axis)
and `cq-horn-torus` build checker-green at `Full` to their closed forms, mesh
closed, read back from STEP and are operands of the boolean. The only refusal
left is an arc that touches the axis inside its range, and it names the arc.

## Non-goals

- A section by a plane holding the axis, through a pole: the trace's
  `TubeCircle` hole stays the backlog line it is (a cut there is the typed
  refusal it already returns).
- Splitting a path's arc at an interior tangency with the axis (refused, named).
- The lemon (inner) sheet of a spindle: an arc reaching both sides of the axis
  stays `ProfileCrossesAxis`.
- Blends and offsets at a torus pole: `offset_faces` keeps `SurfaceCollapses`,
  a fillet there its existing refusal.

## Design deltas

- **ADR-0092, a revolved torus face closes at a pole on the axis** (amends
  ADR-0080 §3 and §5). A torus face may reach the axis only at a vertex on
  it, through a degenerate edge whose pcurve is the line `v = v*`, where
  `R + r cos v* = 0` (`v* = acos(−R/r)`, `π` for a horn). Every other
  pcurve's `v` extent stays clear of `v*` by the band's margin except at the
  endpoint on that vertex. The hull test of F1 counts the pole's `v` as held.
- **`arris-check` F1** (`topology.rs::f1_malformed`): the band test over a
  pcurve's extent excludes an endpoint on a degenerate edge at `v*`.
  `FaceFault::OffTorusBand` stays and fires everywhere else.
- **`arris-ops` revolve** (`sweep/revolve.rs`): `orient` drops the
  `centre − radius ≤ tol` test. `swept_surface` keeps a torus for any
  `R > tol`. `Turn::closes_at` adds `Surface::Torus` to the faces that close at
  a vertex on the axis. `apex_pcurve` already gives the degenerate edge's line
  at the side's own end `v`. A `ProfileLoop::Circle` within `tol` of tangency
  to the axis gets its edge's range re-seated so the vertex is the touch
  point, then the existing full-turn `NonManifold` rule applies.
- **`SweepReason::SpindleTorus` → `SweepReason::ArcTouchesAxis { loop_index,
  segment }`** (breaking, `### Breaking` bullet in its commit). It names the
  first path arc, in the consumer's order, that touches the axis at a point
  inside its range. `arris-py` maps it: class, attributes, `_arris.pyi` line.
  `arris-debug::histogram` renames its row.
- **`arris-mesh` tessellate**: `singular_normal` and the pole fan take a torus
  at `v*`. The limit normal there is `±Z` (a spindle's apex cone) or the
  horn's tangent plane, from the side the face lies on.
- **`arris-io` STEP**: the pole's degenerate edge is written and read as a
  sphere pole's is; the reader holds a toroidal face to the band but there.
- `GLOSSARY.md`: **Torus pole** (new); **Regular band** and **Horn torus**
  sharpened ("no face reaches" → "a face reaches only at a torus pole").

## Steps

Complexity grades; a step's grade is its tier (`.agents/tiers.md`):
**[1]** routine; **[2]** careful, a geometric or numeric case within a given
design; **[3]** unproven, robustness or a bound established here.

- [x] Step 1 **[2]** — Fixtures: `regression/revolve-spindle-arc-from-axis`,
  `-arc-to-axis` (variant: centre 1.3e-4 off the axis),
  `revolve-spindle-arc-clear-of-axis`, `revolve-horn-torus-half-turn` and
  `revolve-horn-arc-touches-axis-inside` (refusal). Each has its closed form
  (`V = π(236 + 225π − 225 asin(4/5))/3`, `A = 10π(3(π − atan(4/3)) + 4) +
  64π`; `V = π²`, `A = 2π² + 2π`) and Open CASCADE's record from
  `tools/oracle/`, with probes and `#[ignore]`.
- [x] Step 2 **[1]** — ADR-0092 as in *Design deltas*; ADR-0080 gets an
  "Amended by" line; GLOSSARY **Torus pole**.
- [x] Step 3 **[3]** — The pole: F1 admits a degenerate edge at `v*`, and the
  revolve closes a torus face there (`closes_at`). Arc-from-axis and
  arc-to-axis (both variants) build `Full`-green and match their closed forms
  in `measure`. The fixtures move to `sweep/`. A hand-built face whose
  non-degenerate pcurve touches `v*` still reports `OffTorusBand`
  (`crates/arris-check/tests/`).
- [x] Step 4 **[1]** — `orient` drops the circle test. The clear-of-axis arc
  builds on the outer sheet, and its fixture moves to `sweep/`.
- [ ] Step 5 **[2]** — Property (`crates/arris-ops/tests/revolve_prop.rs`):
  random arcs on circles with `R ≤ r`, from, to or clear of the axis, revolved
  by a random angle, are `Full`-green with Pappus's volume (the profile's area
  and centroid, closed form) and an area within the fixture tolerance, or are
  `ProfileCrossesAxis`.
- [ ] Step 6 **[2]** — Mesh at a torus pole: the fan closes, the normal is the
  limit from the face's side, and the fixtures mesh closed to their volume
  (`crates/arris-mesh/tests/band.rs` takes the revolve fixtures).
- [ ] Step 7 **[2]** — STEP: write and read back a pole face. Arris reads Open
  CASCADE's STEP of each fixture, Open CASCADE reads Arris's, and the corpus
  read-back stage passes. Lift the expected pole refusals in the spindle and
  horn arc fixtures and in `crates/arris-io/tests/step_read.rs` once the files read.
- [ ] Step 8 **[3]** — The boolean on a pole body, three fixtures from the
  oracle: the apple fused to a coaxial shank on its flat side (the `rivet`
  program), cut by a box across the pole clear of the axis plane, and the
  common with a ball centred on the pole. Each is `Full`-green to the oracle,
  or the refusal its kind already has (`TubeCircle` named in the fixture).
- [ ] Step 9 **[2]** — Horn circle: re-seat a tangent `Circle` loop's vertex
  at the touch point. `revolve-horn-torus-half-turn` builds to `π²` and
  `2π² + 2π`, and its full turn is `NonManifold` (a unit test).
- [ ] Step 10 **[1]** — `ArcTouchesAxis { loop_index, segment }` replaces
  `SpindleTorus`: error docs, `arris-py` class and stub, the histogram row,
  `tests/revolve.rs`, the refusal fixture moved to `sweep/`, a `### Breaking`
  bullet.

## Acceptance

- `cargo nextest run -p arris --test corpus -E 'test(/revolve_(spindle|horn)/)'`
  and the step 8 boolean fixtures: `Full`-green, counts and genus against Open
  CASCADE, volume, area and centroid to the closed forms within the fixture
  tolerance, STEP read back by the oracle and by Arris, provenance accounted.
- `revolve_prop` passes at 256 cases in the full gate; `band.rs` meshes every
  revolve fixture closed.
- `tools/py-test.sh tests/test_errors.py` (or the file mapping the sweep
  reasons) passes with `ArcTouchesAxis`, along with `mypy.stubtest`.
- `grep -rn SpindleTorus crates docs/ARCHITECTURE.md docs/DATA-MODEL.md
  .agents` is empty. At planning its hits are `error.rs`, `sweep/refuse.rs`,
  `sweep/revolve.rs`, `tests/revolve.rs`, `histogram.rs` (step 10), the two
  design docs and `inspect/SKILL.md` (retirement).
- The real-part histogram and the script corpus are re-run with no row above
  C11's close and no panic.

## Docs to update on completion

- `docs/DATA-MODEL.md` torus row: a face reaches the axis only at a torus pole
  (ADR-0092); the revolve refusal's new name.
- `docs/ARCHITECTURE.md` §revolve: spindle and horn arcs build; the pole;
  `ArcTouchesAxis`.
- `docs/ROADMAP.md` §C12: the revolve line **Done** with its numbers;
  `.agents/skills/inspect/SKILL.md:119`: the refusal's new name.
- `CHANGELOG.md` `Unreleased`: consolidate what builds now in the
  `/retire-plan` commit. The `Breaking` bullet is step 10's.
- `AGENTS.md` current state: the revolve line, and the next step of C12.

## Open questions

Decided here by the agent, recorded:

- An arc tangent to the axis inside its range is refused (`ArcTouchesAxis`),
  not split. No harness program writes one, a split would give two faces for
  one consumer segment in `Role::Revolve` provenance, and the refusal names
  the segment, as ADR-0014's `EllipticRevolve` does.
- `dome_cap`'s centre 1.3e-4 off the axis stays a torus (`R = 1.3e-4`), not
  snapped to a sphere. The consumer's points define it, Open CASCADE keeps the
  same surface, and a snap would change the surface by a tolerance the model
  does not carry.
- Settled in step 3 and recorded in ADR-0092: F1 reads neighbouring
  coedges to verify a pole endpoint before allowing it in the `v` hull.
  The extent alone cannot establish a shared pole vertex or distinguish
  an interior touch at the same `v`. A path arc tangent at an endpoint on
  the axis uses equal major and minor radii when the reconstructed radii
  agree within the model tolerance. The endpoint's tangency establishes
  the double root independently of their computed ordering.
- Settled in step 4: the signed centre-distance guard still refuses the
  unsupported inner sheet. For an arc on the outer sheet, the arc's actual
  radial extent determines whether it can build clear of the axis. For
  partial turns, S5 checks axial cap planes against the spindle's two
  meridian arcs clipped to the outer sheet. The surface section keeps
  its existing refusal.
- ⚠ OPEN: in step 7, diagnose Open CASCADE's own STEP of
  `revolve-spindle-arc-to-axis`. The direct build is valid and matches the
  closed forms. Its self-read loses the pole and trim, reporting zero
  volume and only the base's area for both variants. The from-axis
  recipe for the same solid survives Open CASCADE's round trip. Both
  handed profile frames, both axis directions, cyclic loop starts and
  an alternate point on the arc leave the to-axis failure unchanged.
