# Plan: c6-blend-pairs

- Started: 2026-10-02
- Milestone: C6, the blend network (docs/ROADMAP.md §C6) — its second plan: the battery's radius, then the next face pairs the residue ranks
- Idea (verbatim from the human): "fix BlendTooLarge at the battery's radius (blend running out of its face onto the neighbour, 3 fetched parts where Open CASCADE builds it), then the next pair families from the residue (cylinder × cylinder, torus × cylinder, torus × plane, cone pairs)"

## Goal

The fetched tier's `fillet` column is read on edges the kernel is meant to
blend, and the pairs it ranks first are built. The battery stops choosing a
radius no face can hold, so the three `BlendTooLarge` parts (827-9999-906,
-908, CTC-03) either agree with Open CASCADE or surface the pair they were
hiding behind it. Then, in the order step 1's probe ranks them, a blend of
a cone against a plane or a coaxial cylinder along a circle (a torus for a
fillet, a cone for a chamfer: `ring`'s construction with a cone face), and
of two cylinders with parallel axes along a ruling (a cylinder), fillet
and chamfer, open or closed, in a chain, checker-green at `Full`, matching
Open CASCADE on the committed fixtures and the real parts the probe names.
What stays with later C6 plans: the torus pairs (torus × cylinder, torus ×
plane), the blend that really runs over its neighbour (a radius past the
face's width, which Open CASCADE builds by consuming the face), the
corners refused as `VertexBlend`, the other chamfer modes, variable
radius, blends over blends. The cycle stays open when this one retires.

## Non-goals

- The torus pairs. The residue holds them (torus × cylinder 2 parts, torus
  × plane 1), and a rolling ball between a torus and a plane is a closed
  form only on planes through the axis or square to it; whether a part's
  edges are that case is step 1's finding, and a family that ranks first
  there is a decision for the human (open question 1), not an addition.
- A blend that runs over its neighbour: the face it leaves is consumed, a
  topological change (a face deleted, a loop rewritten) that no stripe of
  ADR-0007's shape makes. It stays the roadmap's line, with the parts that
  show it (step 1 names them).
- Cylinder × cylinder on crossing or skew axes (a quartic, ADR-0007's
  "nothing is fitted but a pcurve") and sphere pairs.
- Corners: a vertex of other than three edges, unequal-dihedral miters.
- Variable radius, the other chamfer modes, a blend over a blend.
- The first consumer's side-by-side run: ranking input only (ADR-0020).

## Design deltas

- **The battery** (`arris-debug`, `battery.rs`): `FILLET_FRACTION` of the
  shortest sampled edge becomes the smaller of that and a fraction of the
  narrowest face across the sampled edges, so a stage never asks a radius a
  face of the part cannot hold. The cases are Open CASCADE's too (the oracle
  runs the same radius), so a part's stage moves from one class to another
  and the histogram is re-measured; docs/ARCHITECTURE.md §Formats and tools
  states the rule. No public kernel type changes.
- **ADR-0036 (step 2): the cone and parallel-cylinder rows of the table.**
  Plane × cone, cone × coaxial cylinder along a circle: the ball's centre is
  on the plane's offset by `r` and on the cone's offset surface (a cone of
  the same axis and half-angle, its apex moved along the axis by
  `r / sin α`), a centre circle coaxial with the cone, so a fillet is a
  torus of minor radius `r` and a chamfer the cone through the two contact
  circles; the contact on the cone is the foot of the normal from the
  centre, a circle at the cone's own `v` of its (u, v). Two cylinders with
  parallel axes along a ruling: the centre is on the two offset cylinders
  (radii `R ∓ r`), a line parallel to both axes (two lines, the edge's side
  chosen as ADR-0007 does for plane × cylinder), a cylinder of radius `r`
  about it, both contacts rulings; the chamfer is the plane through the two
  contact rulings. The ADR decides the cone's seam and apex handling (a
  contact reaching the apex is `BlendTooLarge`), which sign of the cone's
  half-angle puts the ring torus a ring torus, and extends ADR-0035's
  chain junction to a cone stripe.
- `docs/ARCHITECTURE.md` §Operations (blends): the two rows, the refusals
  that narrow; the paragraph that says plane × cone, cylinder × cone and
  cylinder × cylinder are `Unsupported` shrinks to what still is.
  §Errors' `Unsupported` row follows.
- `docs/ROADMAP.md` §C6 line 2 and the histogram numbers; `docs/BACKLOG.md`.
- **Public types.** None expected: `fillet` and `chamfer` take the same
  edges, and the new refusals are `Unsupported`/`BlendTooLarge` with their
  existing reasons. A step that needs a `Reason` variant says so in its
  commit body, adds the `### Breaking` bullet and updates `arris-py`'s
  exhaustive `match` and `_arris.pyi` in the same commit
  (`.agents/rules/kernel.md` §API).
- Crate boundaries and layers: none. The kernel half is `arris-ops`'s
  `blend.rs`; a split of `ring` into a surface-agnostic part and a
  per-face-kind part is a refactor step 3 names if the cone row wants one.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [ ] Step 1 **[2]** — The battery's radius, and the probe that ranks the
  residue. Fix the radius rule above, rerun `tools/real-parts.sh` and read
  what the three `BlendTooLarge` parts do now: each agrees, or is refused
  by the pair it was hiding, or really runs over a neighbour at a radius no
  smaller face-bounded rule can avoid (then it is named with its fixture
  under `regression/`, `#[ignore]`d with the oracle's body, and stays the
  roadmap's line). Then a throwaway probe, not committed, as C6's first
  plan's was: every edge of the fetched tier's solids with two distinct
  faces and a curve, blended alone at a tenth of the narrowest adjacent
  face, tangent dihedrals and tangent-vertex edges set aside, grouped by
  refusal with the edge's curve kind — the census the residue is chosen
  from. Test: the battery's unit tests; the histogram's before and after in
  the commit body; the probe's table goes into this plan's open questions.
- [ ] Step 2 **[3]** — ADR-0036 against the probe: the cone row, the
  parallel-cylinder row and which of the two the probe ranks first (the
  order of steps 3 to 6 follows it; a pair the probe finds absent from the
  parts but present in the census stays a backlog line). Fixtures first, as
  `tests/fixtures/regression/` entries with Open CASCADE's oracle values and
  `#[ignore]`d with the desired assertion: a frustum's rim against its cap
  plane (fillet and chamfer, convex and concave), a cone against a coaxial
  cylinder (a step of a turned part), and a bar with two parallel round
  ends. Test: the fixtures run ignored and fail with the refusals the
  probe names (`cargo nextest run -p arris --run-ignored only -E
  'test(/regression_/)'`).
- [ ] Step 3 **[3]** — The cone against a plane along a closed circle:
  `ring` generalises its cylinder face to a cone face (the offset cone, the
  foot-of-normal contact, the torus and the chamfer cone), the cone's seam
  kept or shortened as the cylinder's, a contact reaching the apex or a
  torus that is not a ring torus `BlendTooLarge`. Fixtures move to `blend/`
  with their blessed dumps: volume to the oracle and to Pappus' closed
  form for the removed section swept about the axis, checker green at
  `Full` with nothing unchecked. Test: those fixtures; the frustum rim's
  `Full` check names no unchecked pair.
- [ ] Step 4 **[2]** — The cone row's open arcs, chains and cone ×
  cylinder: an arc of such a circle (a split rim) with its ends trimmed on
  a plane through the axis or square to it (step 3 of the first plan's
  rule, on a cone), and a chain whose junction is a cone stripe meeting a
  line or a cylinder stripe on the ball's cross-section (ADR-0035 §2, an
  existing construction). Fixtures: the split frustum rim, a turned step
  (cone into cylinder at a shoulder) and its chamfer, against the oracle.
- [ ] Step 5 **[3]** — Two parallel cylinders along a ruling: a new row in
  `stripe` (the ruling ball), the contacts rulings on each face, the
  cylinder or plane between them, the ends trimmed by the face across
  (planes square to the axes, the usual three faces) and the miter of two
  of them at a corner whose third edge stays sharp refused as
  `VertexBlend`. Fixtures: the bar with two round ends, convex and
  concave, fillet and chamfer. Test: the oracle's volume, area and
  counts; the closed form of the removed section (`(1 − π/4) r²` for a
  right-angled edge does not hold between curved faces; the section is
  computed from the two offset circles and written in the fixture's
  `analytic` block).
- [ ] Step 6 **[2]** — The refusals the rows leave, each named, as the
  first plan's step 6 did: a cone whose offset reaches its apex, a cone ×
  plane edge that is not a coaxial circle (a plane oblique to the axis
  gives an ellipse and a quartic centre locus: `Unsupported` naming the
  pair), a cylinder pair with crossing or skew axes, a chain through a
  torus. Each a committed fixture with `expect_error`, Open CASCADE's body
  beside it where it builds one.
- [ ] Step 7 **[2]** — The property. `prop::recipe` gains a turned part
  with a coned shoulder (a revolve of a profile with a line at an angle)
  and a bar with two round ends, one edge or the whole outline blended in
  one call at a random radius below the bound, fillet and chamfer. Checks:
  the checker at `Full`, volume against the closed form (Pappus about the
  axis for the cone row, the swept section along the ruling for the
  cylinder row), deterministic, the record complete, and the differential
  against Open CASCADE counting these recipes among the agreeing.
- [ ] Step 8 **[1]** — Measure and close the plan's loop. Rerun
  `tools/real-parts.sh` and the committed tier: the `fillet` column and
  `docs/ROADMAP.md` §C6 get the numbers beside step 1's baseline, each
  part that left the column agreeing with Open CASCADE or refused as a
  named later C6 line. The residue (the torus pairs, the run-over, the
  corners) becomes the next plan's sizing, in the roadmap and
  `docs/BACKLOG.md`.

Each step is one commit-sized unit with its own test, fixture or oracle
comparison. Steps 3 and 5 may split if the construction and its ends need
more than a commit each; say so in that commit.

## Acceptance

- `cargo nextest run -p arris -E 'test(/^blend_/)'`: every blend fixture
  green, the new ones against Open CASCADE's volume, area, centroid and
  counts, the checker at `Full` with nothing unchecked, and no fixture left
  under `regression/` for a line of this plan (the torus pairs' and the
  run-over's keep their `#[ignore]` and reason).
- The cone and parallel-cylinder properties at 256 cases (the retirement
  run) and 1000 in CI, seeded; the differential counts those recipes among
  the agreeing.
- `tools/real-parts.sh`: each part of the fetched tier's `fillet` column
  either agrees with Open CASCADE's fillet within its fixture's tolerance
  or is refused as a named later C6 line; the committed tier's column
  printed beside it and beside step 1's baseline.
- `cargo test --workspace --doc`, `cargo clippy --workspace -D warnings`,
  `tools/check-layers.sh`, the wasm build, the semver gate and the
  `python` CI job's inputs (`arris-py` compiles, `_arris.pyi` unchanged or
  updated) pass.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Operations (blends) — the cone and parallel-
  cylinder rows, the refusals that remain; §Errors' `Unsupported` row;
  §Formats and tools — the battery's radius rule.
- `docs/DATA-MODEL.md` §Provenance — only if a row's record differs from
  ADR-0007's (a cone stripe's seam, a cylinder pair's contacts): checked
  against the audit.
- `docs/adr/0036-….md` and `docs/adr/README.md` — written at step 2.
- `docs/ROADMAP.md` §C6 — status line "second plan landed", the line-2
  pairs marked done within the section, the new histogram numbers.
- `docs/BACKLOG.md` — drop the lines this plan covers (cone and
  cylinder × cylinder in the `Blends on face pairs outside ADR-0007's
  table` line); add the residue and any run-over part step 1 names.
- `CHANGELOG.md` `## Unreleased` — a fillet or chamfer of a cone's rim
  against a plane or coaxial cylinder, and of two parallel round faces
  along a ruling, and which refusals remain; a `### Breaking` bullet only
  if `Reason` gained a variant.
- `AGENTS.md` current state — C6's second plan landed.

## Open questions

- ⚠ OPEN: **Which pair families, in which order** — human decides at step 2,
  from step 1's probe (agent writes the table). The plan assumes the cone
  row (700 edges in the first plan's census, the largest `Unsupported`
  left) and the parallel-cylinder ruling (CTC-01's refusal) rank first and
  the torus pairs wait. If the probe puts torus × cylinder or torus ×
  plane above them, steps 3 to 5 are replaced by that family's rows (a
  meridian or parallel circle of the torus on a plane through the axis or
  square to it, as the first plan's open arc) and the ADR says so.
- ⚠ OPEN: **The battery's face bound** — agent decides at step 1: the
  fraction of the narrowest adjacent face (the sampled edges' faces, or
  every face of the part), and whether the stage skips an edge whose faces
  are narrower than a floor. The rule is recorded in the commit body with
  the before/after histogram.
- ⚠ OPEN: **Whether the run-over is a line of its own** — human decides
  after step 1. Open CASCADE consumes a face a radius outgrows; if the
  corpus's parts need that at their sampled radius, it becomes C6's next
  plan rather than a harness fix.
- ⚠ OPEN: **Cone half-angle range** — agent decides in ADR-0036: a cone
  past a right angle from its axis is a plane's neighbour, one whose offset
  apex crosses the face is `BlendTooLarge`; the bounds are written, not
  assumed, and held by fixtures.
