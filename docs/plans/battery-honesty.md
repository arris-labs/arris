# Plan: battery-honesty

- Started: 2026-10-26
- Milestone: C11's first plan (cycle C11, docs/ROADMAP.md §C11)
- Idea: docs/ideas/blend-residue-real-parts.md (its milestone 1 absorbed;
  decisions Q1–Q4 settled there)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
The blend network's row of the refusal histogram counts only blends that
Open CASCADE really builds and Arris refuses. The battery's `fillet` sample
leaves out an edge that the blend reads as a tangent dihedral (ADR-0045's
test, the edge's tolerance included). The oracle reports a `fillet` or
`chamfer` that silently left out an asked edge, and the battery counts that
stage as Open CASCADE refusing. The committed tier's operands are
re-derived by today's battery. Both tiers are re-run, and the row is
printed, expected at 6 of 38 (CTC-04, FTC-07 AP242, FTC-08, STC-07 AP242,
CTC-01 and FTC-10 no longer blocked). C11's Accept is restated against the
corrected count. No kernel code changes.

## Non-goals
- Any blend construction. The residue is the idea's later milestones, each
  its own plan: M2 `blend-run-over` (FTC-06 AP242 and the script corpus's 2),
  M3 `three-edge-run-vertex` (CTC-02 AP242), M4 `tolerant-tangency` (FTC-11,
  CTC-05 ×2), M5 `overhang-tip-cap` (FTC-06 committed).
- The battery's other stages, its radius rule (`FILLET_FRACTION` of the
  shortest sampled edge) and its stride.
- Keying the oracle cache by the oracle's own source; this plan clears it
  once by hand (Design deltas). One backlog line if it bites again.
- The script corpus, whose stages are not compared with Open CASCADE.

## Design deltas
- **The oracle (`tools/oracle/oracle/recipe.py`, `fillet` and `chamfer`).**
  After `Build`, each asked edge is looked up with `Contour(edge)`. An edge
  that no contour took is a **dropped edge**: the result is still recorded,
  and `expected.json` records the asked points that were dropped beside it
  (`dropped_edges`), written only when there are any, so no other fixture's
  bytes change. No exception is raised, because an oracle refusal deletes
  `expected.json` and would strip numbers from fixtures that hold them.
- **The corpus (`arris-debug::fixtures`).** The expected reading gains the
  optional field. The corpus lint refuses a fixture whose oracle dropped an
  edge unless the fixture expects an error, so an existing blend fixture
  that compares against a body missing a blend comes to light in step 3.
- **The battery (`arris-debug::battery`).** A `fillet` stage whose oracle
  dropped an edge is `OracleRefuses`, or `BothRefuse` when Arris refuses.
  That is ADR-0026's "only a stage Open CASCADE builds" read as "builds what
  was asked". `is_tangent_dihedral` takes `t` as the largest of the
  default, the faces' and the edge's tolerance (ADR-0045 §1), so
  `blendable_edges` and the census's sample match the blend.
- **The oracle cache.** Answers are keyed by `fixture.json` bytes, so
  cached `fillet` answers predate `dropped_edges`. Step 5 clears the cache
  before the fetched tier's run.
- **ADR-0070** (amends ADR-0026 §5 and the amendment of step 9): a dropped
  edge counts as Open CASCADE refusing the stage, the sample uses
  ADR-0045's test, and the evidence of step 1.
- **No public type or signature** of a published crate changes;
  `arris-debug` is `publish = false`. No `CHANGELOG.md` bullet: nothing a
  consumer sees.
- **`docs/ROADMAP.md` §C11**: the count and Accept as the idea's Q3 decided.

## Steps
Complexity grades the human uses to pick the agent for a step: **[1]**
routine: the design says exactly what to write and the tests are
mechanical; **[2]** careful: a geometric or numeric case to get right
within a given design; **[3]** unproven: an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]**: The evidence, and ADR-0070. Read
  `ChFi3d_Builder`'s add path in the reference trees to see where a G1
  edge is left out. On CTC-04, FTC-07 AP242, FTC-08 and STC-07 AP242,
  compare Open CASCADE's sample volume with its volume for the sample less
  the tangent edge (a scratch script, not committed), and check
  `Contour(edge) == 0`. Write ADR-0070 with the numbers. If the volumes
  differ, stop: that part is the blend's, and the plan is rescoped
  (Open questions).
- [x] Step 2 **[1]**: The oracle records `dropped_edges` for `fillet` and
  `chamfer`, and `tools/oracle/selftest.py` covers both kinds of answer.
  A fixture, `blend/tangent-edge-beside-a-crease-fillet`, asks for a crease
  and a tangent edge of one body (a stadium's flat side against its round
  end), so Open CASCADE drops one and Arris refuses `TangentChain`. Its
  oracle records the drop.
- [x] Step 3 **[2]**: The corpus reads `dropped_edges`, and the lint
  refuses a dropped edge outside an `expect_error` fixture. Re-run the
  oracle over every `fillet` and `chamfer` fixture (about 174). Every
  `expected.json` should be byte-identical except the new fixture's; any
  that is not is named in the commit and settled there (Open questions).
  *Settled at step 2:* a part fixture's battery answers carry `dropped_edges`
  too, so the two `regression/` part fixtures whose fillet stage dropped an
  edge (nist-ctc-04-ap203-drill-geometry-fault, nist-ftc-07-ap242-box-cut-split-fault)
  are re-run here, additions only; the committed tier's are step 5's.
- [x] Step 4 **[1]**: The battery counts a dropped edge as Open CASCADE
  refusing, and `is_tangent_dihedral` reads ADR-0045's tolerance. Unit
  tests: CTC-04's e961 leaves the sample, and a scratch stage with a drop
  is `BothRefuse`.
- [ ] Step 5 **[1]**: `fixtures:` re-derive the committed tier
  (`battery --write`, `tools/oracle/expected.py`, `battery --record`) on
  its 9 battery parts. The body names each part's `fillet` stage, before
  and after, and every oracle value that moved, and why: operands changed,
  the oracle did not.
- [ ] Step 6 **[1]**: Clear the oracle cache and run `tools/real-parts.sh`.
  Update `tools/real-parts.waits` if a wait clears. Re-run the
  `--tangent-chain`, `--vertex-blend` and `--run-over` census over the
  blocked parts. Update `docs/ROADMAP.md` §C11 with the printed count, the
  remaining parts by refusal, and the restated Accept.

## Acceptance
- `tools/real-parts.sh` plus the committed tier's histogram print the blend
  network row at 6 of 38 or below. Any part newly blocked by a re-derived
  sample is counted, named in §C11, and given a backlog line, never left
  out.
- No sampled edge of a blocked part is refused as "the edge itself, a
  tangent dihedral" (the census of step 6).
- Every `fillet` and `chamfer` fixture's `expected.json` is unchanged but
  the new one's; the corpus lint holds the dropped-edge rule.
- `tools/full-gate.sh` green, once, at `/retire-plan`.

## Docs to update on completion
- `docs/ROADMAP.md` §C11: the count, the parts left by refusal, Accept as
  restated (step 6 writes it; retirement checks it).
- `docs/adr/0026-…`: a pointer line to ADR-0070 under its amendments.
- `tests/fixtures/README.md`: `dropped_edges` in the expected reading, and
  the battery's `oracle-refuses` / `both-refuse` for a dropped edge.
- `tools/oracle/README.md`: the fillet and chamfer record their dropped
  edges.
- `docs/ideas/blend-residue-real-parts.md`: milestone 1's evidence and
  re-run numbers, for M2's plan.
- `AGENTS.md` current state: C11's first plan done, the corrected count.
- `GLOSSARY.md`: *Battery* sharpened by "builds what was asked"; *dropped
  edge* if the term reaches ADR-0070's text.

## Open questions
- ⚠ OPEN: if step 1 shows Open CASCADE does blend the tangent edge on a
  part (the volumes differ), that part is the blend's. The agent rescopes:
  the dropped-edge steps stay for the parts that hold, the part goes to
  the idea's M4, and the expected count rises by one. Decided at step 1.
- ⚠ OPEN: an existing fixture whose oracle turns out to have dropped an
  edge (step 3). The agent decides per fixture: the recipe moves the edge
  to the one Open CASCADE took, or the fixture expects Arris's refusal.
  No oracle value is edited. Decided at step 3.
- ⚠ OPEN: a part that the re-derived sample newly blocks is recorded, not
  avoided. The agent decides whether it joins an existing residue
  milestone or is a backlog line. Decided at step 6.
