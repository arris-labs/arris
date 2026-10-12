# Plan: gap-and-orientation-healing

- Started: 2026-10-12
- Milestone: C9, the healing cycle (docs/ROADMAP.md §C9), its second and last part
- Idea (verbatim from the human): "next thing in c9"

## Goal

The STEP reader heals the two things C9's "In" list names, and says so. A
solid whose edges lie past the cap off the faces they bound reads, its
tolerances grown to the gaps it measured, up to a healing ceiling. A face
the file walks against its neighbours on every shared edge is turned back.
Both are on by default. Each heal is recorded on the `ReadBody`, naming
every entity it grew or turned, with the measured gap, and nothing is
healed silently. Past the healing ceiling the solid is still
`Refusal::Gap`. A face that disagrees on some shared edges and not all is
still `Topology`. Every healed body passes the checker and matches Open
CASCADE's healed reading (`ShapeFix`) of the same entity in volume and
centroid within the part's tolerance. The real-part histogram's `healing`
row falls from 7 parts to 0 at `read`. The parts that now read run through
the battery, and whatever they meet there lands in its own cycle's row.
The three blend fixtures whose Open CASCADE STEP is `occt_step_refused`
for the turned torus face read it back.

**Evidence the plan is built on** (measured 2026-10-12 while planning, with
a throwaway probe: the existing reader in a `Model` whose `max_tolerance`
was raised to 0.2):

| Part | Gap refused today | Read with the ceiling raised | vs Open CASCADE |
|---|---:|---|---|
| CTC-02 AP203 (fetched) | 0.0105 | refused next: a NURBS band whose seam has no exact curve | — |
| CTC-02 rc (committed `nist-ctc-02`) | 0.0191 | reads, checker **S5**: f0 meets f178 and f331 away from shared edges | volume 6.7e-6 rel |
| CTC-04 AP242 (fetched) | 0.0506 | reads, checker green | — |
| CTC-05 AP242 (fetched) | 0.0207 | reads, checker green | — |
| CTC-05 rd (committed `slow-gap-refusal`) | 0.0182 | reads, checker green | volume 3.2e-7 rel |
| FTC-07 AP242 (fetched) | 0.0346 | reads, checker **E8/L5**: e279, e283 self-intersect at t ≈ 5e-4 and 1.9e-3 | volume within 4e-7 of the rd edition's |
| FTC-11 AP242 (fetched) | 0.0140 | reads, checker green | — |

Every gap is an edge or a vertex lying 0.01 to 0.05 mm off its surfaces, on
parts 100 to 400 mm across. In every case the surfaces themselves do not
meet within the cap, so no edge curve, re-fitted or projected, closes the
gap: closing it would mean moving a surface, and moving a surface is not
healing. Open CASCADE carries the same parts at vertex and edge
tolerances of up to 0.108. "Sewing" in ADR-0025 §4's sense is therefore
growing the tolerance the file needs. The decision and its limits are
ADR-0058's, written at step 1.

## Non-goals

- Free-edge sewing: faces that share no edge in the file, matched and
  merged into one shell (`BRepBuilderAPI_Sewing` proper). No part of the
  corpus needs it: every surface model read so far is a one-face open
  shell. It becomes a backlog line in the step 1 commit.
- Moving geometry to close a gap: re-intersecting faces, re-fitting edge
  curves, or snapping vertices.
- Healing as a model operation on a body already in the model
  (`ops::heal`). The bodies these gaps belong to never exist without the
  reader's healing, and a body the kernel built carries its tolerances
  already. Healing is a stage of the read, and the read records it.
- Shape fixing beyond gaps and orientation: small-edge removal, face
  removal, merging same-domain faces (docs/ROADMAP.md §C9 Out). Step 2 may
  find that FTC-07 needs a small-edge fix. If so, that fix is a refusal
  with a backlog line, not a step here.
- The reader's subset beyond healing: CTC-02 AP203's NURBS band without an
  exact seam, and the `VERTEX_LOOP` sphere and walked-B-spline open loops
  of Open CASCADE's STEP (BACKLOG lines 45, 99). They stay refusals, filed
  in their own histogram rows.
- Sheets as boolean operands, wire bodies, faceted B-reps, IGES.

## Design deltas

- **ADR-0058** (step 1): *a read gap is closed by a grown tolerance, never
  by moved geometry, and the read records it.* It amends ADR-0025 §4
  (whose "closing it would be sewing, which is healing" now has an
  answer) and does not edit it. It decides:
  1. **What healing is.** An entity's tolerance is raised to its measured
     gap past the read cap (`READ_GAP_FRACTION`). The ceiling is a named
     constant `HEAL_GAP_FRACTION` in `arris-math`, relative to the part
     like the cap, and its value is set from the gap census below with its
     evidence. The model's `max_tolerance` stays an absolute ceiling.
     Whether `Precision::DEFAULT.max_tolerance` (1e-2 today, under every
     gap of the table) rises, or whether healing is bounded by the healing
     ceiling alone, is decided here (Open question 1).
  2. **The orientation rule.** A face whose every shared edge is walked
     the same way as the neighbour across it, in a shell whose other faces
     agree with each other, has its loops turned. Partial disagreement is
     still `Refusal::Topology`. A face on a closed shell with no agreeing
     majority (two faces, each disagreeing with the other) is refused, not
     guessed.
  3. **The record.** `ReadBody` gains `healing: Healing { grown:
     Vec<Grown>, turned: Vec<FaceId> }`, with `Grown { entity: AnyId,
     file: u64, gap: f64 }`. Each `#id` is the file entity, as provenance
     names it. Provenance itself is unchanged: every entity stays
     `Generated` from its file entity, because healing changes no
     topology (Open question 2).
  4. **The switch.** `ReadOptions::heal: Heal`, an enum with `Heal::Off`
     (today's refusals, bit for bit) and `Heal::GapsAndOrientation`, the
     default. The default is on because Open CASCADE heals every read and
     a consumer's part file is expected to open. A write-then-read of a
     healed body must also read back under default options.
- **ADR-0060** (step 3): *`max_tolerance` bounds what an operation grows,
  not what a body holds.* The checker's V1 held a vertex's tolerance to
  `max_tolerance`, so every healed body failed the reader's own check,
  and body bytes refused the same range: V1 keeps its floor only,
  `ToleranceBound::AboveMaximum` and `BodyError::Precision::max` go
  (Breaking, Python's `BodyPrecisionError.max` with them). ADR-0058's
  "the checker is not asked to change" was wrong; this step found it.
- **`arris-io` public API (Breaking):** `ReadOptions` gains `heal` and
  `ReadBody` gains `healing`. `Grown::entity` is an `EntityId`, ADR-0058
  §3's `AnyId` narrowed: healing grows only vertices and edges. Neither struct is `#[non_exhaustive]`, so a
  struct literal breaks. The fix is `..Default::default()` for
  `ReadOptions`, and constructing `ReadBody` is not a consumer's business.
  The `CHANGELOG.md` bullet goes under `### Breaking` in step 3.
  `step::read`'s guarantee "every `Ok` body passes the checker" is
  unchanged.
- **`arris-math`:** `HEAL_GAP_FRACTION` beside `READ_GAP_FRACTION`, its doc
  carrying the census.
- **`arris-io` reader internals:** `topology.rs`'s gap measurement keeps
  measuring past the first gap, so that every gap is known and recorded
  rather than only the first refused. The orientation pass runs after the
  shell's faces are bound and before the checker.
- **`arris-check`** (step 2): E8 counts a crossing only where the curve
  between leaves the tolerance ball; L5 lets a loop cross itself inside
  a vertex's tolerance. Held to every existing fixture staying green.
- **ADR-0059** (step 2): a read edge's tolerance holds where its two
  faces meet, `d / sin θ`, up to `arris_math::SECTION_GROWTH_LIMIT`.
- **`arris` facade and `arris-py`:** `Heal` re-exported. Python's
  `read_step(..., heal=True)` returns each read body's healing record as
  `healing.grown` (a list of `(entity, file_id, gap)`) and
  `healing.turned`. The `_arris.pyi` lines and docstring examples land in
  the step that adds each.
- **`arris-debug` survey:** the part report records each solid's healing
  (count grown, largest gap, count turned), and the histogram's summary
  prints a `healed` count beside `read`. A part fixture's `fixture.json`
  can say `"healed": {...}` for a solid whose read heals, and the part
  runner holds the read's record to it.
- **Fixtures:** `real/nist-ctc-02` and `real/slow-gap-refusal` flip from
  `refused` to `read` with their healing. New `real/` excerpts
  (`tools/oracle/excerpt.py`) hold one gap of each fetched part and the
  turned face. `blend/{hole-rim-fillet,boss-base-fillet,cone-boss-base-fillet}`
  lose `occt_step_refused`.

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

The ADR goes first because steps 2 and 3 both read its ceiling. The two
checker failures go second: they are the one thing the probe did not
answer, and whether they belong to the checker or to the files decides
whether 4 or all 6 of the healable parts leave the row.

- [x] Step 1 **[2]** — **The gap census and ADR-0058.** A survey mode
  `real_parts --gaps <file.stp>...` (and `--gaps-committed`). It reads
  each file with `Heal::Off` semantics but measures *every* gap, not just
  the first. For each solid it prints the entity kind, the file `#id`,
  the gap, the part's size, the surface pair, and Open CASCADE's tolerance
  at that entity (`tools/oracle`: `BRep_Tool::Tolerance` after
  `ShapeFix_Shape`, matched by position). Run over the 38 parts. The
  output, the gaps as fractions of each part's size beside Open CASCADE's
  carried tolerances, sets `HEAL_GAP_FRACTION`'s value. ADR-0058 decides
  §1–4 above and closes Open questions 1 and 2. The backlog gains the
  free-edge sewing line, and line 61 is rewritten to point at this plan.
  Test: a docs test holds the census table in the ADR to the committed
  tier's two parts.
- [x] Step 2 **[3]** — **The two healed parts the checker rejects.** Read
  `nist-ctc-02` (S5) and FTC-07 AP242 (E8, L5) in a model whose
  `max_tolerance` covers their gaps, as the probe did. Shrink each to a
  `regression/` excerpt fixture: the S5 face pair with its neighbours, and
  the self-intersecting edges with their faces. Commit them `#[ignore]`
  with the desired assertion: checker green, Open CASCADE's area and
  volume of the excerpt. Classify each cause with the `inspect` skill:
  either a checker test that ignores the grown tolerance (the fix is in
  `arris-check`, judging at the entity's tolerance), or a real defect of
  the file that a tolerance cannot cover (a curve that doubles back, faces
  that truly cross). A file defect beyond gaps and orientation stays a
  typed refusal (`Refusal::Invalid` naming the check) with a backlog
  line, and the fixture holds the refusal. Either way both fixtures leave
  `regression/` with an answer. Existing fixtures stay green under any
  checker change.
  *Done:* none was a file defect, and each was fixed in the step, so the
  excerpts went straight to `real/` (sheets of the faces involved,
  `excerpt.py --faces`). FTC-07's E8 and L5 were the checker's and occur
  without healing: an edge bending through its tolerance's length, whose
  chord is shorter than its arc, and a loop zigzagging inside a vertex's
  tolerance (`real/edge-bending-within-its-tolerance`,
  `real/loop-crossing-inside-a-vertex`). CTC-02's S5 was the reader's:
  an edge on the plane and `d` off a drafted cylinder is `d / sin θ` from
  where they meet, and the read measured only `d`. ADR-0059 raises a
  read edge's tolerance to that, between faces crossing at `30°` or more
  (`real/edge-off-a-drafted-cylinder`, read with `max_tolerance` raised
  until step 3 heals it). Open CASCADE's healing moves these faces'
  boundaries further than Arris's read does, so each fixture's
  tolerances are that band, derived in its description.
- [x] Step 3 **[2]** — **Gap healing in the reader.** `ReadOptions::heal`
  and `Heal`, `ReadBody::healing` and `Grown`. Every gap from the read cap
  up to `HEAL_GAP_FRACTION` of the part (and within the model's ceiling as
  ADR-0058 sets it) is grown and recorded; one past the ceiling is still
  `Refusal::Gap` with the gap and the ceiling in its message. Fixtures:
  `real/slow-gap-refusal` and `real/nist-ctc-02` (if step 2 cleared it)
  flip to `read`, held to Open CASCADE's volume and centroid with a
  `healed` expectation (count grown, largest gap). New `real/` excerpts of
  one gap each from CTC-04, CTC-05 and FTC-11 AP242 carry the same, with
  their files committed. Properties: under `Heal::Off` every committed
  part and every solid fixture's STEP reads bit-identically to today; a
  healed body written and read back under default options has the same
  counts and volume, and a healing record no larger than the first; a body
  with no gap past the cap has an empty record. The Python binding gets
  `heal=` and `healing`. `CHANGELOG.md` gets a `### Breaking` bullet
  (`ReadOptions` and `ReadBody` gained a field) and a feature bullet ("a
  part whose edges lie a little off its faces now reads, and says which").
  *Done:* the checker's V1 refused every healed body, so ADR-0060 drops
  V1's ceiling and body bytes' with it. A pcurve is fitted under the
  read cap first and under the healing cap only where that fails, so a
  body that needs no healing reads bit for bit as with `Heal::Off`
  (`real_heal_off_reads_every_committed_part_as_before`). The record
  names every vertex and edge whose tolerance passes the read cap. The
  committed tier's `real/nist-ctc-05` (CTC-05's file again) flipped with
  the two named, and the parts' sheets' dumps moved by ids only, the
  healed solid now standing first in the model. The excerpts are
  `real/healed-gap-{cylinder-meets-sphere,nurbs-meets-cylinder,on-an-engraved-plane}`,
  each held to the band its healed edges leave the sheet's area. The
  committed histogram's `healing` row is 0. The fetched tier is step 5's.
- [x] Step 4 **[2]** — **Orientation healing.** The rule of ADR-0058 §2
  in the reader, for solids and sheets, recorded in `Healing::turned`.
  Fixtures: the three blend fixtures' Open CASCADE STEP reads back, so
  their `occt_step_refused` is removed and the read-back stage matches
  the oracle. A hand-written `real/` excerpt of a two-face open shell
  with one face reversed reads as a sheet with that face turned, held to
  Open CASCADE's surface properties after `ShapeFix_Shell`. A closed
  shell with one face turned in Arris's own STEP of a corpus box (a
  property over random poses and the face chosen) reads back to the box,
  with that face named. Partial disagreement stays `Topology`, with a
  test. Python: `healing.turned`. `CHANGELOG.md` bullet. BACKLOG line 98
  is removed.
  *Done:* the rule turns a face's walk, its surface and normal kept,
  before any pcurve is fitted; that is Open CASCADE's writer's defect,
  and the three blends' files read back to the oracle. Three guards keep
  it from guessing. A face sharing fewer than two edges is no evidence of
  which use was turned: a cylinder whose side walks one circle backwards
  would otherwise turn its cap. A solid with a loop that does not chain
  is inconsistent, not reversed. A face turned back that the checker then
  rejects (a face whose front *and* bounds the file turned, left bounding
  no region) is `Topology`, not `Invalid`, as `step_sheets` holds. The
  two-face case is a closed dome and its disc (`step_read`). Partial
  disagreement is a through-hole face with only its hole's bound turned
  (`step_orient`). The sheet excerpt is hand-written,
  `real/turned-face-in-an-open-shell`, three faces so that one is
  outvoted, and its dump is the untouched corner's. The part runner's
  `healed` counts `turned`. The backlog's orientation line was 99, not
  98.
- [x] Step 5 **[1]** — **The histogram re-run.** The survey records
  healing per solid and the summary prints the `healed` count. The fetched
  tier is re-run, and the committed tier's histogram block in
  `docs/ROADMAP.md` is updated by its docs test. Every part that now reads
  runs through the battery, and each new refusal there gets its cycle's
  row and, where it is a fault rather than a refusal, a `regression/`
  fixture and a backlog line (light triage: no investigation). The
  `read` column's fall is printed in the commit body. BACKLOG line 61 is
  removed.
  *Done:* the fetched tier went from 46 read and 8 refused to 50 read (5
  healed) and 4 refused. Its `healing` row went from 5 parts to 0, and
  with the committed tier's 2, the acceptance's 7 → 0 holds. CTC-02
  AP203 now refuses at its NURBS band (`itself: outside the subset`).
  Four S5 waits passed, fixed by step 2, and their fixtures moved to
  `real/`. Of the parts that now read, four meet faults, each shrunk to
  `regression/` with a wait and a backlog line, uninvestigated. CTC-02
  AP203's read takes 176 s in two B-spline fits the unhealed read never
  reached. Its oracle hung for two hours in Open CASCADE's Gauss–Kronrod
  volume integration. A solid a fixture records only as refused now keeps
  the plain adaptive one, since none of its measures are compared.
  CTC-04 AP242 meshes a point off a face. FTC-07 and FTC-11
  AP242's box cut (and FTC-07's drills) are `Internal(Split)`, the one
  finding that may be healing's own (a grown tolerance above
  `max_tolerance`, ADR-0060). The battery's Arris refusals went from 2 to
  12 under the NURBS and blend-network rows.

## Acceptance

- `cargo nextest run --workspace` (the full gate, `/retire-plan`) green,
  including `real_*`.
- `real_parts --committed`: `nist-ctc-02` and `slow-gap-refusal` read,
  checker green, volume and centroid matching Open CASCADE within the
  part's tolerance, `healed` records matched. (If step 2 classified
  CTC-02's S5 as a file defect, CTC-02 reads to its typed refusal instead
  and the plan says so.)
- The fetched tier re-run: the `healing` row at `read` is 0 parts, from 7.
  CTC-02 AP203 moves to the reader-subset row. The number is printed in
  the roadmap's C9 status line.
- `blend/hole-rim-fillet`, `blend/boss-base-fillet` and
  `blend/cone-boss-base-fillet` read Open CASCADE's STEP and match.
- Properties: `Heal::Off` is bit-identical to the pre-plan reader; a
  healed read round-trips through write and read; a turned face in a
  random pose is turned back and named.
- No panic on any input: `step_read` fuzz target, 15 minutes on the
  committed seeds, with healing on.

## Docs to update on completion

- `docs/ROADMAP.md` §C9: the status line (healing's `read` 7 → 0, the
  `healed` count, sewing and orientation landed), and the cycle ready for
  `/close-cycle`. The committed histogram block is held by its docs test.
- `docs/ARCHITECTURE.md` §Formats and tools (the reader): healing as a
  read stage, the cap and the healing ceiling, the record. Also the
  sentence "Healing is an operation — the healing cycle's" (near "The
  checker never repairs"), corrected to "a stage of the read, recorded on
  what it returns".
- `docs/DATA-MODEL.md` §Tolerances: an imported entity's tolerance may
  be raised to its measured gap by healing, up to `HEAL_GAP_FRACTION`,
  and the read names it.
- `tests/fixtures/README.md` §Part fixtures: the `healed` expectation.
- `docs/BACKLOG.md`: lines 61 and 98 removed (steps 5 and 4), the
  free-edge sewing line added (step 1).
- `AGENTS.md` current state: C9's second part landed; the cycle is ready
  to close.
- `CHANGELOG.md`: the bullets land in steps 3 and 4. `/retire-plan`
  checks they read for a consumer.

## Open questions

1. Closed at step 1 (ADR-0058 §2, option (b)), with ADR-0060 at step 3: **The absolute ceiling.** Every gap in the table is above
   `Precision::DEFAULT.max_tolerance` (1e-2). Option (a): raise the
   default ceiling to cover what healing grows (Open CASCADE has no
   ceiling, and 0.108 is what it carries here). That touches every
   operation's refusal threshold. Option (b): the reader's healing
   ceiling (`HEAL_GAP_FRACTION`) is the only bound on a read entity's
   tolerance, and the model's `max_tolerance` keeps bounding what
   *operations* may grow. Option (c): healing never passes the model's
   ceiling, and a caller who wants these parts raises it. Recommended:
   (b), since operations keep their guard and the read keeps its own,
   relative to the part, but its effect on an operation run on a healed
   body (a fillet next to a 0.05 edge) must be measured. Who: the agent,
   in ADR-0058 at step 1, on the census. Step 3 is blocked until it is
   decided.
2. Closed at step 1 (ADR-0058 §3, as recommended): **Provenance for a healed entity.** Recommended: leave it
   `Generated` from its file entity and carry healing on `ReadBody`
   only. The roadmap's accept line ("a healed body's provenance names
   every face sewn or turned") is then met by the record, not by a new
   `Role` or a `Modified` entry, which would claim an earlier state that
   never existed in the model. Who: the agent, ADR-0058, step 1. If it
   goes the other way, step 3 gains a `Role::Healed` and a provenance
   change under `### Breaking`.
3. Closed at step 2: **FTC-07 and CTC-02's checker failures** may be file defects
   no tolerance covers. If so, the healing row ends at 2 parts, not 0,
   and the acceptance line is rewritten in the step 2 commit with the
   classification. Who: the agent, step 2. *Answer:* none is; two were
   the checker's (E8, L5) and one the read's measure (ADR-0059), all
   fixed, so the acceptance line stands at 0.
