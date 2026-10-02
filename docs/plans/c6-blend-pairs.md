# Plan: c6-blend-pairs

- Started: 2026-10-02
- Milestone: C6, the blend network (docs/ROADMAP.md §C6) — its second plan: the battery's radius, then the next face pairs the residue ranks
- Idea (verbatim from the human): "fix BlendTooLarge at the battery's radius (blend running out of its face onto the neighbour, 3 fetched parts where Open CASCADE builds it), then the next pair families from the residue (cylinder × cylinder, torus × cylinder, torus × plane, cone pairs)"

## Goal

The fetched tier's `fillet` column is read on edges the kernel is meant to
blend, and the pairs it ranks first are built. The three
`BlendTooLarge` parts (827-9999-906, -908, CTC-03) either agree with Open
CASCADE or surface the pair they were hiding behind it; step 1 found their
refusal is not the battery's radius but a convex blend ending at a concave
corner, which step 2 builds. Then, in the order step 1's probe ranks them, a blend of
a cone against a plane or a coaxial cylinder along a circle (a torus for a
fillet, a cone for a chamfer: `ring`'s construction with a cone face), then
the same meridian construction for a sphere's or a torus's parallel against
a coaxial plane, cylinder or cone (ADR-0036, which step 3's census ordered
so), fillet and chamfer, open or closed, in a chain, checker-green at
`Full`, matching Open CASCADE on the committed fixtures and the real parts
the probe names. What stays with later C6 plans: the blend ends with no
closed form (step 3's census: an open arc's torus on a plane off its axis,
a ruling stripe on a curved face across), the parallel-cylinder row
(decided in ADR-0036, 29 edges), a torus against a cylinder off its axis,
the blend that really runs over its neighbour (a radius past the
face's width, which Open CASCADE builds by consuming the face), the
corners refused as `VertexBlend`, the other chamfer modes, variable
radius, blends over blends. The cycle stays open when this one retires.

## Non-goals

- The torus pairs off the axis: a torus against a cylinder off its axis,
  and a torus against a plane through its axis along a meridian circle.
  The coaxial parallels are the meridian row's (step 6, ADR-0036 §6).
- Two cylinders with parallel axes along a ruling: decided (ADR-0036 §5),
  built when a census ranks it; its step-3 fixtures stay under
  `regression/`.
- Blend ends that are no closed form (ADR-0036 §6): the next plan's idea.
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

- **The battery** (`arris-debug`): unchanged. Step 1 tried bounding
  its radius by the narrowest adjacent face and dropped it: the committed
  tier's stored operands are held equal to the derived ones, so the rule
  moved a committed part's radius (FTC-11, 4.49 to 0.15) and its oracle
  values, and the three `BlendTooLarge` parts refused at any radius anyway.
- **ADR-0036 (step 3): the cone and parallel-cylinder rows of the table.**
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
  per-face-kind part is a refactor step 4 names if the cone row wants one.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The three `BlendTooLarge` parts, and the probe that
  ranks the residue. Try a face-bounded radius, rerun `tools/real-parts.sh`
  and read what the three parts do now: each agrees, or is refused
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
- [x] Step 2 **[2]** — A convex blend ending at a concave corner (found at
  step 1). `face_end` takes the end arc's side of the face across from the
  blend's own convexity alone; where both corner edges are concave (a rib's
  root on the plate round it, a boss on a plate) the arc lies in the hole
  the footprint leaves and the face gains the corner, so the side flips,
  and likewise for a concave blend at concave corners. Move
  `regression/rib-corner-fillet-to-plate` to `blend/` with its blessed dump
  and add its siblings: the 45° sloped rib end (NIST 827-9999-906's edge),
  the same corner chamfered, a concave blend at a pocket's concave corners,
  and a boss's rim edge. Re-run step 1's probe: the 1561 `BlendTooLarge`
  edges are the baseline, 959 after the rule. Test: those fixtures against
  the oracle at `Full` with nothing unchecked; the three parts' `fillet`
  stage agrees or names the pair it reaches; and the 514 edges the probe
  turned from `BlendTooLarge` to `ok` are held to the checker (a throwaway
  run, its count in the commit body), since the probe built them without
  checking.
- [x] Step 3 **[3]** — ADR-0036 against the probe: the cone row, the
  parallel-cylinder row and which of the two the probe ranks first (the
  order of steps 4 to 7 follows it; a pair the probe finds absent from the
  parts but present in the census stays a backlog line). Fixtures first, as
  `tests/fixtures/regression/` entries with Open CASCADE's oracle values and
  `#[ignore]`d with the desired assertion: a frustum's rim against its cap
  plane (fillet and chamfer, convex and concave), a cone against a coaxial
  cylinder (a step of a turned part), and a bar with two parallel round
  ends. Test: the fixtures run ignored and fail with the refusals the
  probe names (`cargo nextest run -p arris --run-ignored only -E
  'test(/regression_/)'`).
- [x] Step 4 **[3]** — The cone against a plane along a closed circle:
  `ring` generalises its cylinder face to a cone face (the offset cone, the
  foot-of-normal contact, the torus and the chamfer cone), the cone's seam
  kept or shortened as the cylinder's, a contact reaching the apex or a
  torus that is not a ring torus `BlendTooLarge`. Fixtures move to `blend/`
  with their blessed dumps: volume to the oracle and to Pappus' closed
  form for the removed section swept about the axis, checker green at
  `Full` with nothing unchecked. Test: those fixtures; the frustum rim's
  `Full` check names no unchecked pair.
- [ ] Step 5 **[2]** — The cone row's open arcs, chains and cone ×
  cylinder: an arc of such a circle (a split rim) with its ends trimmed on
  a plane through the axis or square to it (step 3 of the first plan's
  rule, on a cone), and a chain whose junction is a cone stripe meeting a
  line or a cylinder stripe on the ball's cross-section (ADR-0035 §2, an
  existing construction). Fixtures: the split frustum rim, a turned step
  (cone into cylinder at a shoulder) and its chamfer, against the oracle;
  `regression/turned-shoulder-fillet` moves to `blend/` with its closed
  forms and `measure_differs`, Open CASCADE's blend there being a walked
  B-spline (step 3).
- [ ] Step 6 **[3]** — The meridian row's circle meridians (ADR-0036 §1,
  §6; replaces the parallel cylinders at step 3): a sphere centred on the
  axis or a torus against a coaxial plane, cylinder or cone along a
  parallel — the meridian offset is a concentric circle, the centre the
  root on the edge's side, the contact on the torus or the sphere a line
  at constant `v`, the chamfer at the chord. Fixtures first under
  `regression/` with Open CASCADE's oracle, then moved to `blend/`: a
  cylinder capped by a coaxial spherical dome meeting it at an angle, a
  torus ring cut square to its axis, convex and concave, fillet and
  chamfer. Where Open CASCADE walks rather than placing a torus, the
  closed forms and `measure_differs` (ADR-0015), as at step 5.
- [ ] Step 7 **[2]** — The refusals the rows leave, each named, as the
  first plan's step 6 did: a cone whose offset reaches its apex, a centre
  circle that is no ring torus, a cone × plane edge that is not a coaxial
  circle (a plane oblique to the axis gives an ellipse and a quartic
  centre locus; a plane through the apex along a ruling, 64 edges at step
  3: `Unsupported` naming the pair), a cylinder pair with crossing axes, a
  torus against a cylinder off its axis. Each a committed fixture with `expect_error`, Open CASCADE's body
  beside it where it builds one.
- [ ] Step 8 **[2]** — The property. `prop::recipe` gains a turned part
  with a coned shoulder (a revolve of a profile with a line at an angle)
  and a domed or toroidal one (an arc in the profile meeting a line at an
  angle), one edge or the whole outline blended in one call at a random
  radius below the bound, fillet and chamfer. Checks: the checker at
  `Full`, volume against Pappus about the axis, deterministic, the record complete, and the differential
  against Open CASCADE counting these recipes among the agreeing.
- [ ] Step 9 **[1]** — Measure and close the plan's loop. Rerun
  `tools/real-parts.sh` and the committed tier: the `fillet` column and
  `docs/ROADMAP.md` §C6 get the numbers beside step 1's baseline, each
  part that left the column agreeing with Open CASCADE or refused as a
  named later C6 line. The residue (the torus pairs, the run-over, the
  corners) becomes the next plan's sizing, in the roadmap and
  `docs/BACKLOG.md`.

Each step is one commit-sized unit with its own test, fixture or oracle
comparison. Steps 4 and 6 may split if the construction and its ends need
more than a commit each; say so in that commit.

## Acceptance

- `cargo nextest run -p arris -E 'test(/^blend_/)'`: every blend fixture
  green, the new ones against Open CASCADE's volume, area, centroid and
  counts, the checker at `Full` with nothing unchecked, and no fixture left
  under `regression/` for a line of this plan (the torus pairs' and the
  run-over's keep their `#[ignore]` and reason).
- The meridian-row properties at 256 cases (the retirement
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

- `docs/ARCHITECTURE.md` §Operations (blends) — the meridian row (line
  and circle meridians), the refusals that remain; §Errors' `Unsupported`
  row.
- `docs/DATA-MODEL.md` §Provenance — only if a row's record differs from
  ADR-0007's (a cone stripe's seam, a cylinder pair's contacts): checked
  against the audit.
- `docs/adr/0036-the-meridian-row-and-the-parallel-cylinder-row.md` and
  `docs/adr/README.md` — written at step 3.
- `docs/ROADMAP.md` §C6 — status line "second plan landed", the line-2
  pairs marked done within the section, the new histogram numbers.
- `docs/BACKLOG.md` — drop the lines this plan covers (the cone and the
  coaxial sphere and torus pairs in the `Blends on face pairs outside
  ADR-0007's table` line); keep step 3's lines (the ends, the parallel
  cylinders, B-spline-written edges, the edge off its face).
- `CHANGELOG.md` `## Unreleased` — a fillet or chamfer of a circle where
  a cone, a sphere or a torus meets a coaxial plane, cylinder or cone, and
  which refusals remain; a `### Breaking` bullet only
  if `Reason` gained a variant.
- `AGENTS.md` current state — C6's second plan landed.

## Open questions

- **Decided at step 3 (agent, delegated, 2026-10-02): which pair families,
  in which order** — the cone row (steps 4–5), then the circle meridians
  (step 6), the parallel cylinders waiting (ADR-0036 §6, the census below).
  The question as it stood: The plan assumes the cone
  row (700 edges in the first plan's census, the largest `Unsupported`
  left) and the parallel-cylinder ruling (CTC-01's refusal) rank first and
  the torus pairs wait. If the probe puts torus × cylinder or torus ×
  plane above them, steps 4 to 6 are replaced by that family's rows (a
  meridian or parallel circle of the torus on a plane through the axis or
  square to it, as the first plan's open arc) and the ADR says so.
- **Answered at step 1 (no for these parts), closed at step 3: whether
  the run-over is a line of its own.** Open CASCADE consumes a face a radius outgrows; if the
  corpus's parts need that at their sampled radius, it becomes C6's next
  plan rather than a harness fix.
- **Decided at step 3 (ADR-0036 §3): cone half-angle range** — none beyond
  the data model's `(0, π/2)`: the construction sees only the corner's
  angle, and what bounds a blend is where it lands. The question as it
  stood: a cone
  past a right angle from its axis is a plane's neighbour, one whose offset
  apex crosses the face is `BlendTooLarge`; the bounds are written, not
  assumed, and held by fixtures.
- **Found at step 1 (agent, 2026-10-02): the three `BlendTooLarge` parts are not
  a radius.** A battery radius of a tenth of the narrowest length the blend
  must fit in (the sampled edge's, and the shortest edge on either of its
  faces' boundaries) moved only a NURBS stage of the fetched tier's 11
  parts, to both-refuse, and was dropped (it moves a committed part's
  stored operands); the three kept `BlendTooLarge` — 827-9999-906, -908 and CTC-03 — at any radius down to
  0.01. 906's edge is the sloped corner of a 63.5-long rib (1.27 thick,
  45° end) standing on a plate: a convex blend whose lower end meets the
  plate's top, which surrounds the rib, so the corner with it is concave
  and the end arc lies in the footprint, a hole of that face. `face_end`
  asked for it inside. It is not Open CASCADE running over a neighbour:
  the answer to the run-over question is no for these parts, and the
  roadmap's run-over line stays only for what a larger probe shows. Shrunk
  to `regression/rib-corner-fillet-to-plate` (a rib box on a plate box,
  one vertical corner edge, r 0.5; the oracle builds it, volume to the
  closed form `(1 − π/4) r² · 6` off the fused body); step 2 fixes it.
- **The census at step 1 (agent): every blendable edge of the fetched tier's
  solids, 14066, filleted alone at a tenth of the narrowest bound, with
  step 2's rule applied** (a throwaway probe, not committed; edges
  with their parts out of 29): built 4169, `TangentChain` 5106 (tangent
  dihedrals and ends at tangent corners, 29), `BlendTooLarge` 959 (plane ×
  plane lines 632 in 25 parts, cylinder × plane lines 191, circles 124;
  before the rule 1561 — cause of the remaining 959 not yet classified:
  step 2's re-run reads them), `VertexBlend` 372, and `Unsupported` by the
  pair the edge or its chain reaches: cylinder × cylinder 614 (a line 477
  in 22 parts, a circle 137), cone × cylinder 490 (446 circles in 21
  parts), plane × torus 582 (361 circles in 22 parts, 221 lines), cylinder
  × torus 250, cone × plane 309 (187 circles in 12 parts, 122 lines),
  cylinder × sphere 183, NURBS 600+, cone × torus 48, elliptic 80. So the
  plan's order holds for the cone (799 edges across the two cone pairs)
  and the cylinder pair, but the torus pairs together (832) outrank either
  family and are one family (a part's own blends met by a second edge):
  the human's decision at step 3 is whether they replace the cylinder
  pair in this plan or open the next one. One fault: 2 edges answer
  `Internal(Geometry(NotOnSurface))` — a finding with its own fixture when
  step 2 re-runs the probe.
- **Found at step 2 (agent, 2026-10-02): the rule is the corner's convexity
  against the blend's, and 906 and 908 now build.** `end_side` (blend.rs)
  puts the end arc inside the face across where blend and both corner edges
  share a convexity and outside where they differ (a concave blend at a
  convex corner, a pocket's top rim, gains the face; at a concave corner,
  the floor, loses it); corner edges of unlike convexity are `VertexBlend`,
  which no committed fixture or property met. `tools/real-parts.sh`: the
  `BlendTooLarge` line fell from 3 parts to 1 (CTC-03, cause not yet
  classified); 827-9999-906 and -908 report no refusal. The step's census
  re-run was **not** done: step 1's probe was a throwaway and was not kept,
  so the 514-edge checker pass and the 959 classification wait for a probe
  rewritten at step 3, which needs it for the ranking anyway. The boss's
  rim edge of the step's text is a square boss's corner here, a circle's
  rim being a ring, whose end rule is the same function.
- **The census at step 3 (agent, 2026-10-02): the probe rewritten**, still
  throwaway and not committed, now naming for each `Unsupported` the two
  faces and how they sit, or the edge and the face across its end. 9567
  edges over the 18 parts of the fetched tier that read (step 1's 14066
  counted the committed tier's files too): built 2932 (2248 checker-green
  at `Full`; 684 on the 4 parts whose own reading fails S5,
  `regression/nist-*-s5`, so every blend there inherits the failure —
  that is step 2's check of the edges it turned to `ok`, with no failure
  of a blend's own), `TangentChain` 3325, `Unsupported` of a pair 1269,
  `Unsupported` of an end 1093, `BlendTooLarge` 670 (plane × plane lines
  432 in 17 files, cylinder × plane lines 142, arcs 76, circles 12; their
  cause is still unclassified, read at step 9), `VertexBlend` 276,
  `Internal` 2. The pairs: the cone coaxial with a cylinder or a plane
  411 edges in 15 parts, a sphere or a torus coaxial 218 in 6, crossing
  cylinders 186 in 8, NURBS faces 250 in 6, a torus against a cylinder
  off its axis 82 in 4, a plane through a cone's apex 64 in 2, parallel
  cylinders 29 in 5. The ends: an open arc's torus on a plane parallel to
  its axis 321 in 12, a ruling stripe on a curved face across 422 in 14,
  an edge written as a B-spline 287 in 5. Step 1's "torus pairs 832" was
  mostly the first of those ends, the pair sorted so the ring's own torus
  read as a face. The two `Internal` edges (FTC-06) are
  `regression/edge-off-its-plane-fillet`, a cube with one edge lifted
  1.3e-7 off its plane in the file.
- **Found at step 3 (agent): Open CASCADE walks the cone against a
  cylinder.** Its known parts stop at the plane against a cone; the turned
  shoulder's blend is a B-spline up to 8e-6 off the torus and its volume
  2.7e-8 relative under the Pappus form, so
  `regression/turned-shoulder-fillet` has no probe on the blend and no
  `analytic.volume`. Step 5 writes its closed forms (volume, area,
  centroid, inertia) and `measure_differs` when it moves the fixture. The
  four plane × cone fixtures match their Pappus volumes to 1e-9.
- **Found at step 4 (agent, 2026-10-02): `ring` is the meridian
  construction, no split needed.** Each face is a `Meridian`, a line in
  the half-plane bounded by the axis; the centre, the feet, the torus's
  `v`s and the chamfer's chord come from the two lines, and the plane
  against a cylinder reproduces its old numbers exactly (no blessed dump
  moved). The seam, ends and junctions were already face-agnostic. Open
  CASCADE's own STEP of the concave boss fillet is refused on read
  (`not a closed shell`): its writer keeps a reversed torus face's bounds,
  as on `blend/boss-base-fillet`, so the fixture names it
  `occt_step_refused`. The unit test that held plane × cone `Unsupported`
  now holds a cone's ruling on a plane through its apex (step 7's
  refusal). An open arc of the cone row goes through the same code
  unproven until step 5's fixtures.
