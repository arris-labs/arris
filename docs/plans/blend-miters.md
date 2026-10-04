# Plan: blend-miters

- Started: 2026-10-04
- Milestone: C6, the blend network (docs/ROADMAP.md §C6)
- Idea: docs/ideas/blend-miters.md (absorbed: option C, the re-filing step,
  and the tangent cylinder–sphere misread its check found)
- Idea (verbatim from the human): "/idea on the miters, the last VertexBlend
  site (ftc-06's 3 edges). …", then "I agree with recommendations"

## Goal
Two fillets or two chamfers meeting at a vertex whose third edge stays
sharp are built when their dihedrals differ (for chamfers: when their edges
make unequal angles with the third edge), wherever Open CASCADE builds
them. Today the slanted prism's corner
(`a_miter_of_unequal_dihedrals_is_a_vertex_blend`) is refused. The corner
becomes the existing miter curve (the cylinders' bisecting ellipse, or the
chamfers' line) up to where it reaches the narrower blend's far contact.
From there the wider blend is trimmed by the face across: a plane against
a cylinder or a plane, so every curve is a conic or a line. The third face
takes that arc, and the third edge is shortened to its end.

Two misattributions are corrected beside it:
- **ftc-06's 3 `VertexBlend` edges.** They are one straight run whose
  second face jumps at a four-edge vertex. The census names them "a dihedral
  jump on a collinear run" and attributes them to the NURBS cycle, since the
  patch has no closed form and Open CASCADE builds none of them.
- **The 9 tangent cylinder–sphere edges.** These are ctc-04's 7 and ftc-08's
  2: a ball against its rounding of the same radius. They are refused
  `TangentChain`, as the ~50 like them already are, and no longer as an
  unsupported pair.

When the plan is done, the corners line of C6 holds only the ruling miter,
and it goes to the backlog.

## Non-goals
- **The ruling miter** (a plane–cylinder ruling blend meeting another blend
  at a vertex). Its corner is probably traced. It stays `VertexBlend` and
  stays on the backlog.
- **ftc-06's setback patch itself**, and the overhang tip. Both are the
  NURBS cycle's (ADR-0042 §6). ftc-06's part-level attribution
  (`TangentChain`) does not change.
- **The trace that misses** (`Unsupported end: cylinder × cylinder`). No
  sampled edge has it.
- **Miters of mixed convexity** (one blend convex, the other concave). They
  stay `VertexBlend`.
- A corner of three blended edges. That is ADR-0007's sphere, unchanged.
- No public type or signature of a published crate changes, and `Reason`
  gains no variant. The Python binding is untouched.

## Design deltas
- **ADR-0044** (step 2): a miter of unequal dihedrals. It amends ADR-0007's
  miter, which today has one ellipse, no arc entering any face, and the
  third edge cut at the one point where the far contacts meet.
  - **Corner curve.** The miter curve runs from `q`, where the contacts
    cross on the shared face, to `m`, where it meets the narrower blend's
    far contact. The trim arc then runs from `m` to the wider blend's far
    contact on the third edge, as the section of the wider blend with the
    narrower blend's far face.
  - **Third edge and faces.** The third edge is shortened to the trim
    arc's end. The face across takes the arc.
  - **Provenance.** The trim arc is `Generated` from the wider blend's edge.
    `m` is `Generated` from both edges. The third edge is `Modified`.
  - **What step 1 settles.** Which blend is the wider one, which face takes
    the arc, and the counts. Step 1 reads them off Open CASCADE's result,
    and the ADR records them.
- `crates/arris-ops/src/blend.rs`:
  - `miter` builds the two-piece corner where it now refuses `unequal`.
  - `Miter` and `MiterMade` carry the optional trim arc. That is internal.
  - `rebuild` gives the face across its arc and shortens the third edge.
  - The fillet and chamfer rustdoc's `VertexBlend` list loses the unequal
    miter.
- **The cylinder–sphere misread** (step 6) is a refusal order. Where it is
  raised is found first. The fix makes a tangent dihedral `TangentChain`
  wherever a pair is checked. No ADR, since ADR-0040 already says it.
- `arris-debug` (`publish = false`):
  - `census::vertex_blend_cause` names "a dihedral jump on a collinear run"
    where an extra edge continues the blended one and the second face
    changes.
  - The census attributes that site to `Cycle::Nurbs`. Any signature change
    inside the debug crate is named in its commit.
- `docs/ARCHITECTURE.md` §Blends: the miter paragraph and the `VertexBlend`
  list. `docs/DATA-MODEL.md` §provenance at a miter.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — **The unequal miter, asked of Open CASCADE and
  shrunk.**
  - Put the slanted prism's corner (the parallelogram prism of
    `a_miter_of_unequal_dihedrals_is_a_vertex_blend`, the vertical edge and
    its cap edge) to `tools/oracle/`, for both fillet and chamfer. Also
    check a convex-up and a concave pocket version.
  - Read its result with the `inspect` skill: the corner's curves and
    their kinds, which face takes the arc, where the third edge is cut, and
    the counts.
  - Commit `regression/miter-unequal-dihedrals-{fillet,chamfer}` with the
    oracle's values, `#[ignore]`d.
  - If Open CASCADE refuses or builds it invalid, stop here: that is
    ⚠ OPEN 1.
  - **Found** (Open CASCADE 8.0.1, checker-valid, 12 vertices, 18 edges,
    8 faces, genus 0; the equal miter has 11/17/8): the corner is the two
    blends' meeting curve (the ellipse of the two cylinders, the chamfers'
    line) from `q` on the shared face to `m`, then one trim arc (a circle,
    a line) from `m` to the third edge, which ends there. The wider blend
    differs by kind: the vertical edge's fillet at 0.2 (contacts at
    r/tan(θ/2) = 0.324 on the acute edge) is the wider and its trim is the
    circle on the top face (the cap blend's far face); the cap chamfer is
    the wider (its plane cuts the third edge at y = 1.8, the vertical's at
    1.82) and its trim is the line on the slanted side face (the vertical
    chamfer's far face). A concave pocket version builds too (20/30/13);
    mixed convexity stays out of scope. Arris refuses both fixtures
    `VertexBlend`.
- [x] Step 2 **[1]** — **ADR-0044**, as the design deltas say, from what
  step 1 saw. Include the counts, which blend is wider, and the arc's face.
- [x] Step 3 **[3]** — **The fillet miter built.**
  - `miter` computes `m` and the trim arc (the existing plane–cylinder
    section, exact on the plane, the pcurve on the cylinder fitted by the
    oblique-section rule).
  - `rebuild` places the arc and cuts the third edge.
  - Provenance as ADR-0044 says.
  - The fillet fixture moves to `blend/` with its blessed dump, held to
    the oracle, checker green. `blend/fillet-miter` and every other dump
    in `blend/` stay unchanged.
  - The old test becomes the built case. Mixed convexity is still
    `VertexBlend`, tested.
  - A trim arc leaving its face is `BlendTooLarge`, tested at a radius past
    it.
  - **Found:** the fixture is held to closed forms where Open CASCADE
    drifts. Its volume is 1.7e-9 and its area 2.8e-9 (relative) under the
    closed forms, which Arris matches to about 1e-11. The volume is the
    two fillets' regions less their overlap; the area is summed face by
    face. Both are now in `analytic`. The fixture expressions gain `atan`
    for them. That drift is far short of ADR-0015's 1e-6, so this is not
    `measure_differs`. The volume, area and inertia tolerances are 1e-8,
    as `blend/turned-shoulder-fillet` widened its own.
  - **Found:** on a convex far face the trim arc cannot leave alone. It is
    tangent to the third edge at its end, and at `m` it is square to the
    narrower blend's contact. So it stays in the corner those two lines
    bound, and it leaves only after the third edge or a contact has run
    out. The check stays as a guard. The test at r = 1.5 is refused by
    whichever bound fails first. A third edge of the other sense is
    `VertexBlend`, since no fixture shows it.
- [x] Step 4 **[2]** — **The chamfer miter built.** The same with lines:
  the chamfers' meeting line to `m`, then the wider chamfer's chord in the
  face across. The fixture moves to `blend/`, held to the oracle.
  - **Found:** `m` is the narrower chamfer's far contact through the wider
    chamfer's plane; the trim chord is `section_between`'s plane–plane
    chord. Arris matches Open CASCADE's counts and volume with no closed
    form added; the cap chamfer is the wider and cuts the third edge at
    (2.9, 1.8, 2).
- [x] Step 5 **[2]** — **A property over random slanted corners.**
  - Draw prisms over random parallelograms in random poses, fillet and
    chamfer at random size, two edges at one corner.
  - Check: checker green, the same ids for either edge order, determinism,
    and the volume. Where the volume oracle comes from is ⚠ OPEN 2.
  - Shard it with `prop_shards!` as the other blend properties are.
  - **Found (OPEN 2 decided):** the closed form. The prism less the corner
    section over the rise, less the top edge's section over its length
    (the slant's gain at one end paid back at the other), plus the overlap
    `∫ L(n) g(n) dn` over the wall strip, `g(n) = r − √(2rn − n²)`; chamfers
    in a polynomial. It reproduces the fixture's closed form at the slanted
    prism and Arris's volume at 2000 cases per shard, acute and obtuse
    corners, either top edge.
- [ ] Step 6 **[2]** — **The tangent cylinder–sphere edges read as
  tangent.**
  - **Blocked (2026-10-04), ⚠ OPEN 3.** The premise is wrong: no refusal
    order is at fault. The 7 edges (ctc-04's NURBS-written circles between a
    sphere and a cylinder of one radius, centre on the axis; the three plane
    × cylinder line edges beside them name the same pair from their ends)
    read as a *crease*, not as tangent, under ADR-0040 §1: `|n₁ × n₂|` is
    `7.5e-7` to `1.2e-5` (the file writes its coordinates to about 1e-6) and
    the ball's move `r · sine` is `5.9e-7` or more against the faces'
    tolerance `1e-7`, while the edges' own tolerances are `8e-6` to `1.5e-4`.
    ADR-0040 §3 chose the faces' tolerance over the edge's on purpose, so
    the stripe's contacts stay within what a junction accepts. Their
    "twins" are tangent within it.
  - Find where ctc-04's 7 edges are refused as a pair while their twins are
    `TangentChain`.
  - Shrink to the smallest body that shows it (a rounding ending in its
    corner ball). Commit it as a `regression/` test asserting `TangentChain`,
    and fix it in the same step, moving the test into its area.
  - Re-run ctc-04's battery and the committed census. Settle why the
    fixture records the sample as `TangentChain` while the census names
    this pair. If the fixture is stale, re-bless it in a `fixtures:` commit
    whose body says why.
- [ ] Step 7 **[1]** — **ftc-06's 3 edges re-filed.**
  - `census::vertex_blend_cause` names "a dihedral jump on a collinear run".
  - The census test holds that name on ftc-06, and its attribution is the
    NURBS cycle's.
  - `regression/` gains no fixture: Open CASCADE builds none of the three
    (2 invalid, 1 refused), so there is no oracle.
- [ ] Step 8 **[1]** — **The tiers measured.**
  - Run `tools/real-parts.sh` and `--census-committed`.
  - Record the expected outcome in the plan: `VertexBlend` 0 edges in C6 on
    both tiers (ftc-06's 3 under NURBS), the cylinder–sphere edges counted
    under `TangentChain`, and whether ctc-04 leaves the `fillet` column.
  - 0 failing parts.

## Acceptance
- `blend/miter-unequal-dihedrals-{fillet,chamfer}` match Open CASCADE's
  oracle within their tolerance, checker green, with every `blend/` dump
  unchanged.
- The step 5 property is green at 256 cases (`ARRIS_GATE=full`).
- The step 6 regression test is green in its area.
- The census test holds ftc-06's cause and attribution.
- `tools/real-parts.sh` shows 0 failing parts and no `VertexBlend` edge
  attributed to the blend network on either tier.

## Docs to update on completion
- `docs/ARCHITECTURE.md` §Blends: the miter paragraph (unequal dihedrals
  and chamfer angles built, the trim arc, the third edge, the face across).
  Also the `VertexBlend` refusal list (drop the unequal miter, keep mixed
  convexity and the ruling miter) and the tangent dihedral read before any
  pair.
- `docs/DATA-MODEL.md` §provenance at a miter (around "At a miter the two"):
  the trim arc and `m`.
- `docs/ROADMAP.md` §C6:
  - The eighth plan's paragraph gains a ninth, with the measured tiers.
  - The corners line marks the unequal miter done (ADR-0044), the ruling
    miter left to the backlog, ftc-06's run and the tangent cylinder–sphere
    attributed.
  - "What the cycle has left" is restated.
- `docs/BACKLOG.md`:
  - Drop the unequal-dihedral miter line.
  - Keep the ruling miter line, noting C6 left it.
  - Add ftc-06's dihedral jump to the "smooth, seam or collinear extra
    edge" line as the NURBS cycle's.
- `docs/adr/README.md`: ADR-0044's row.
- `CHANGELOG.md` `Unreleased`: two fillets or chamfers at a corner of a
  slanted part now build, and a tangent edge between a rounding and its ball
  is refused as a tangent dihedral.
- `AGENTS.md` current state: C6's line gains the miter (ADR-0044).

## Open questions
- ⚠ OPEN 3 — Step 6's tangent cylinder–sphere edges. Whether the blended
  edge's own dihedral (a stripe's and a ring's `TangentChain`, the one place
  no stripe is built) reads tangency at the larger of the faces' tolerance
  and the edge's, an amendment of ADR-0040 §3 (an ADR-0045) that the
  junctions and corner edges keep; or the 7 edges stay `Unsupported pair`
  and step 6 shrinks to a census re-attribution (the NURBS cycle's, the file
  being loose) with no kernel change. Preferred: the amendment, if a corpus
  run shows no edge that builds today turns `TangentChain`; otherwise the
  re-attribution. **Agent decides, but widening a tolerance is a design
  change** (`/work`'s rules), so it waits for a word from the human before
  steps 6 and 8.
- ⚠ OPEN 1 — If Open CASCADE refuses or builds the slanted miter invalid
  (step 1), is it held to closed forms under `analytic.measure_differs`
  (ADR-0015), or does the plan drop steps 2–5 and keep 6–8? **Agent
  decides at step 1.** Preferred: closed forms if the corner's volume has
  one, otherwise drop.
- ⚠ OPEN 2 (decided at step 5: the closed form) — The step 5 property's volume oracle. Options are a closed form
  for the corner's removed volume (the stripes' sections × length,
  corrected by the miter wedge and the trim cap), or the differential's
  cached Open CASCADE run per drawn pose. **Agent decides at step 5.**
  Preferred: the closed form, as `blend_prop` corrects its equal miters.
