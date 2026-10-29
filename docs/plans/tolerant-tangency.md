# Plan: tolerant-tangency

- Started: 2026-10-29
- Milestone: M4 (cycle C11, docs/ROADMAP.md §C11, first **In** item)
- Idea: docs/ideas/blend-residue-real-parts.md (Accepted A; its M4 absorbed)

## Goal
The real parts' `fillet` row is **0 of 38**. Both of its remaining blocks
build, checker-green at `Full`. CTC-05's blocks are **vanishing ends**
whose narrowest exact arc lies off the vertex. The stripe ends on that arc,
on a degenerate edge, and the edge runs on from there to its vertex, so
the surface stays on the exact balls. The cases are AP203's e256 and
AP242's battery edge e258 (r 0.978). Their oracle is the second oracle's
valid solid (ADR-0073), held as a change against each kernel's own
operand, since the change is 1e-7 to 3e-9 of the part's area. AP242's
e256 at r 0.889, which the second oracle refuses too, stays `TangentChain`.
FTC-11's e24 chains through v24, a 3.9° kink tangent within the vertex's
tolerance, where the end across never meets the hole. The two stripes meet
in a junction across the kink, within the vertex's tolerance, and the
result is held to Open CASCADE. v2 of the same slot stays the end across
it is today.

## Non-goals
- M5 `overhang-tip-cap` (the idea's last milestone, the script corpus's one
  refusal), the horn torus, the parallel-cylinder row, a cusp with both
  edges blended: later C11 plans.
- Open CASCADE's fixed 0.1 rad G1 allowance (rejected by ADR-0040). The
  vertex's tolerance is read only where the end across meets nothing.
- A stripe running on past a vanishing end, its faces crossing at
  2.2e-4 rad (backlog line 67's other reading), and a face turning tangent
  inside an edge: both stay refused.
- A local two-sided surface distance as an oracle measure (the second
  oracle's report proposes it): a backlog line if the deltas cannot tell.
- STC-07 AP242's undecided cylinder pairs (its own `regression/` wait).

## Design deltas
- **ADR-0074, a vanishing end leaves the rest of its edge** (amends
  ADR-0062 §5 and step 9b). The stripe ends on its narrowest exact arc, and
  the skin's row there stays exact, not collapsed. The loop crosses that row
  on a degenerate edge at a new vertex `w`. `w` is the edge's point nearest
  the arc, and its tolerance is the larger of the edge's and the arc's
  distance to `w` plus the surface's share. The edge is cut at `w`, and its remnant runs from `w` to
  the old vertex, kept on both faces as the tangent dihedral it is
  (ADR-0045). *Settled at step 1:* growing the kept vertex instead would
  need 9.94e-3 on AP203, at the ceiling, and would re-end every kept edge
  there. The remnant needs only the edge's own 7.74e-3, because AP203's arc
  lies 7.39e-3 and 7.26e-3 off e256's curve, 6.6e-3 along it from v84
  (E6 bounds the row's 1.98e-4 by `w`'s tolerance; no row bounds an edge's
  length). AP242's arc lies 1.97e-2 off its curve (edge 2.4e-4, vertex
  2.05e-2), past `max_tolerance`: ⚠ OPEN 1.
- **ADR-0075, a chain through a vertex tangent within its tolerance**
  (amends ADR-0040 §2–3, ADR-0035 §1). Where an end at a vertex would be an
  end across a face and the trace finds no crossing, the walk reads the
  vertex's corner edge tangent at `t = max(faces', vertex's)`. The two
  runs' end balls are then `2r·sin(θ/2)` apart (7.2e-4 at v24). The
  junction's shared edge is the arc of their mean ball, its pcurves each
  face's own end isocurve. The edge and its junction vertices take the
  distance to either face, bounded by the vertex's tolerance. §3's reason
  ("contacts further apart than the junction accepts") is met by the
  vertex's own tolerance.
- **ADR-0073 amendment, a record of changes.** `second_oracle.json` may
  hold the operand's measures beside the result's. With them, the fixture
  holds Arris's `result − part` to the record's `result − part`. Area is
  held first, at a tolerance from the stripe (⚠ OPEN 2), and volume looser.
  Each kernel measures its own exact bodies, never a read through STEP
  (the report's 1–2 mm² drift on CTC-05). `tools/oracle/second_oracle.py`
  writes the operand's values, and `arris-debug`'s measure stage reads
  them (private).
- No public type or signature changes. `Reason` gains no variant.
  `BlendReason::TangentChain` stays the refusal past `τₙ` and at a kink the
  vertex does not cover.
- `GLOSSARY.md`: **Vanishing end** (new). **Junction** sharpened (across a
  kink, within the vertex's tolerance).

## Steps
Complexity: **[1]** routine, the design says exactly what to write and
the tests are mechanical; **[2]** careful, a geometric or numeric case
to get right within a given design; **[3]** unproven, an algorithm whose
robustness or bound has to be established here.

- [x] Step 1 **[2]** — The second oracle's records, at accuracy 1.0, kept
  with the runs and imported at steps 6–7 once step 3 takes the operand.
  AP203 e256 at r 0.889: builds, faces 156 → 157, dA −0.08592, dV −0.26313.
  AP242 e256 at r 0.889: refuses ("reduce the radius"), while Open CASCADE
  builds it invalid, so both refuse it. The battery's AP242 edge is e258
  at r 0.978 (the fixture's e256 mirrored): builds, 156 → 158, dA −0.00233,
  dV −0.00416, while Open CASCADE builds it invalid. FTC-11's slot chained
  at 5.73°: refuses ("Create Blend error"), as at 0.5°. Open CASCADE builds
  both the slot fixture's e24 (r 0.0105) and the battery's edge at
  (−24.40, −5.23) (r 0.0255) valid (⚠ OPEN 3).
- [x] Step 2 **[3]** — ADR-0074: a vanishing end off its vertex stops
  short, and the edge's remnant runs on. AP203's e256 gets past v84. A
  walked end whose contact misses the face across beside a crease within
  that edge's own tolerance is `TangentChain` (`near_tangent_corner`).
  *Settled:* AP203 then stops at v115, where f109 runs on into the cylinder
  f19 7.0e-6 rad off tangent, which is within the vertex's 7.74e-3 and not
  the faces'. That is ADR-0075's chain, so AP203 builds only after it
  (step 6), and the second oracle's record is its tangent chain's: rerun,
  3 edges, faces 156 → 159, dA −0.11878, dV −0.23780 (AP242's e258:
  dA −0.03770, dV −0.02452). The remnant's assembly (`remnant_edges`, the
  faces' split pieces) builds no body until then: step 6 is its test.
- [ ] Step 3 **[2]** — The ADR-0073 amendment: a record of changes, the
  tool and the measure stage, and `compare.py`. Test: the four existing
  records unchanged, and a change-record fixture held and failing when its
  delta is perturbed.
- [ ] Step 4 **[3]** — The chain walk through a vertex tangent within its
  tolerance (ADR-0075's selection; write ADR-0075): FTC-11's v24 and
  CTC-05's v115. Test: each chain names the edges up to where it next ends.
  `blend/nist-ftc-11-ap242-bspline-circle-fillet` is unchanged, its v2
  still the end across.
- [ ] Step 5 **[3]** — The junction across the kink: the mean ball's arc,
  tolerances bounded by the vertex's. The FTC-11 slot fixture moves to
  `blend/`, held to Open CASCADE's measures, checker green at `Full`.
- [ ] Step 6 **[2]** — CTC-05 AP203 built through v115, checker green at
  `Full`, held to the second oracle's tangent-chain changes under
  `measure_differs` (`fixtures:` commit).
- [ ] Step 7 **[2]** — CTC-05 AP242's e258 past the ceiling (⚠ OPEN 1,
  ADR-0060 amended), a new fixture `blend/nist-ctc-05-ap242-battery-edge-
  fillet`, held likewise. AP242's e256 stays `TangentChain`, its
  description naming both oracles' refusals.
- [ ] Step 8 **[2]** — The census: `real_parts --tangent-chain --census`
  over both tiers with `paranoid`. Every edge that built still builds with
  the same counts. Each edge that newly chains or joins is checker-green or
  becomes a `regression/` fixture. The count goes in ADR-0074/0075's
  consequences.
- [ ] Step 9 **[2]** — Blend properties. A plane-and-cylinder chain kinked
  by θ under the vertex's tolerance, in random poses, held to the unkinked
  chain's area within `r·θ·length`. A near-tangent end (two cylinders
  turned tangent within a widened edge tolerance) is held to the same
  vanishing end built exactly tangent.
- [ ] Step 10 **[1]** — The battery re-run: `tools/real-parts.sh` with the
  `fillet` row at 0 of 38, the waits file current, and the roadmap's C11
  counts written. The backlog lines 67 and 68 are removed.

## Acceptance
`tools/real-parts.sh` prints the blend network row at **0 of 38**, every
other row unchanged. In the corpus, `blend/nist-ctc-05-ap203-nurbs-torus-
fillet` and `-ap242-battery-edge-fillet` are built and checker-green at
`Full`, held to the second oracle's changes; `-ap242-nurbs-torus-fillet`
is `TangentChain`. `blend/nist-ftc-11-ap242-slot-chain-fillet`
is built, held to Open CASCADE's volume, area and centroid, and
`blend/nist-ftc-11-ap242-bspline-circle-fillet` is unchanged. The step-8
properties pass at 256 cases. The census shows no edge that built now
refusing, and no panic. `tools/full-gate.sh` is green.

## Docs to update on completion
- `docs/ARCHITECTURE.md` blend section (the tangency paragraph after
  "The selection follows chains"): the vertex's tolerance at a kink, the
  junction across it, and the vanishing end joined at its vertex.
- `docs/DATA-MODEL.md` §Tolerances: a healed entity grown by an operation
  (ADR-0060 as amended).
- `docs/ROADMAP.md` §C11: the first **In** item ticked, the row count,
  and Accept's numbers.
- `docs/ideas/blend-residue-real-parts.md`: status set to M4 done.
- `docs/BACKLOG.md`: remove lines 67 and 68, and add any step-7 findings.
- `GLOSSARY.md`: Junction (Vanishing end written at step 2).
- `CHANGELOG.md` Unreleased: fillets that end where two faces run
  tangent within a file's tolerance, and through a slight kink a file
  writes as tangent, now build.
- `AGENTS.md` current state.

## Open questions
- ⚠ OPEN 1 (agent, step 7): a new vertex past `max_tolerance`. AP242's `w`
  needs 1.97e-2, stands 1.55e-2 along e258 from v80, and v80 holds 2.05e-2
  from the read's healing. I'd amend ADR-0060 §1 so a new vertex inside an
  operand vertex's tolerance may take up to that vertex's tolerance: the
  read already recorded that the faces meet only that well there, and the
  operation adds no uncertainty of its own.
- ⚠ OPEN 2 (agent, step 3): the tolerance on a change. I'd use area within
  the larger of 1e-2 of the change and `τ × the stripe's length`, and
  volume within 10× that, the report's area-first finding. The pick is
  recorded in the amendment.
- ⚠ OPEN 3 — *Settled at step 1:* the second oracle refuses FTC-11 at
  5.73° as well, but Open CASCADE's slot results pass `BRepCheck`, so it
  still blocks (ADR-0026) and steps 4–5 stay. Step 4 starts from the
  battery's edge at (−24.40, −5.23), r 0.0255, as well as the fixture's e24.
