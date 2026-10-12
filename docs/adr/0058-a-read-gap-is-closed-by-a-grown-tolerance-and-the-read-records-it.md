# ADR-0058 — A read gap is closed by a grown tolerance, and the read records it

- Status: accepted (2026-10-12)
- Plan: `gap-and-orientation-healing` step 1
- Amends: ADR-0025 §4 ("closing it would be sewing, which is healing":
  this is the answer) and ADR-0057 §1 (an open shell whose faces disagree
  on orientation is `Topology` "until the healing cycle's orientation
  plan repairs it": §2 below is that repair); neither is edited
- Idea: the roadmap's C9 "In" list (no idea file; decisions delegated to
  the agent, the plan's open questions 1 and 2 decided here)
- Reference trees read: none. Open CASCADE is run as the oracle
  (`tools/oracle/occt_tolerances.py`: `BRep_Tool::Tolerance` after its
  reader's `ShapeFix_Shape`), and what its healing does is what it carries.

## Context

Seven of the fetched tier's parts are held back at `read` by a gap:
`Refusal::Gap`, an edge or a vertex lying 0.01 to 0.05 mm off the faces it
meets, on parts 100 to 1200 mm across. The gap census (`real_parts --gaps`,
`arris_debug::gaps`) reads each file into a model whose `max_tolerance` is
raised to 1, so that only the part-relative cap
(`arris_math::READ_GAP_FRACTION`, 2e-4 of the part) bounds the read, and
prints every vertex and edge whose tolerance the read grew past the default
`max_tolerance` (1e-2), beside the tolerance Open CASCADE's healed reading
carries at the same vertex or edge midpoint.

The committed tier's two parts (the table is held to the committed
fixtures by `crates/arris/tests/docs_refs.rs`; Open CASCADE's column is
`BRep_Tool::Tolerance` after its reader's `ShapeFix_Shape`):

<!-- gaps: committed -->
### nist-ctc-02 #20700[0] — size 1167.4, Open CASCADE carries up to 0.1077 (vertex) and 0.1077 (edge)

| gap | of size | vertices | edges | surfaces | file entities | Open CASCADE |
|---:|---:|---:|---:|---|---|---:|
| 0.0392 | 3.36e-5 | 16 | 8 | cylinder × plane | #8910, #8911, #8914, … | 0.0385 |
| 0.0332 | 2.85e-5 | 16 | 8 | cylinder × plane | #8715, #8718, #8722, … | 0.0330 |
| 0.0330 | 2.83e-5 | 32 | 16 | cylinder × plane | #8660, #8661, #8666, … | 0.0328 |
| 0.0286 | 2.45e-5 | 4 | 4 | NURBS × plane | #8046, #8047, #8048, … | 0.0283 |
| 0.0267 | 2.29e-5 | 16 | 8 | cylinder × plane | #8432, #8436, #8440, … | 0.0265 |
| 0.0212 | 1.82e-5 | 2 | 1 | NURBS × NURBS | #8644, #8645, #13720 | 0.0105 |
| 0.0211 | 1.81e-5 | 0 | 1 | NURBS × NURBS | #13622 | 0.0105 |
| 0.0191 | 1.63e-5 | 8 | 8 | cylinder × plane | #7766, #7767, #7768, … | 0.0190 |
| 0.0150 | 1.28e-5 | 16 | 8 | cylinder × plane | #8462, #8463, #8473, … | 0.0386 |
| 0.0101 | 8.61e-6 | 2 | 1 | NURBS × NURBS | #8632, #8633, #11631 | 0.0053 |

### slow-gap-refusal #6534[0] — size 907.3, Open CASCADE carries up to 0.0860 (vertex) and 0.0860 (edge)

| gap | of size | vertices | edges | surfaces | file entities | Open CASCADE |
|---:|---:|---:|---:|---|---|---:|
| 0.0182 | 2.01e-5 | 2 | 1 | NURBS × NURBS | #2781, #2783, #4444 | 0.0199 – 0.0261 |

<!-- /gaps -->

The table is re-measured as the reader changes. Since ADR-0059 an edge
between two crossing faces also covers where they meet, which raised
the first row from `0.0386` (`3.31e-5`) to `0.0392` and the second's
fraction from `2.84e-5`; the prose below quotes the first measure.

The fetched tier's other healable parts, same census (run 2026-10-12, not
held by a test):

| Part | Largest gap | Part size | Of size | Surface pair | Open CASCADE |
|---|---:|---:|---:|---|---:|
| CTC-04 AP242 | 0.0506 | 935.7 | 5.4e-5 | cylinder × sphere | 0.0493 |
| CTC-05 AP242 | 0.0207 | 907.2 | 2.3e-5 | NURBS × cylinder | 0.0169 – 0.0174 |
| FTC-07 AP242 | 0.0346 | 407.0 | 8.5e-5 | cylinder × sphere | 0.0345 |
| FTC-11 AP242 | 0.0141 | 96.5 | 1.5e-4 | cylinder × plane | 0.0140 |
| CTC-02 AP203 | — | 1167.2 | — | refused at a NURBS band whose seam has no exact curve, not at a gap | — |

Three facts the census settles:

- **Every gap is within the read cap already.** The largest, FTC-11's, is
  1.5e-4 of its part; `READ_GAP_FRACTION` is 2e-4. What refuses the seven
  parts is the *absolute* `min(…, max_tolerance)` of `topology.rs`'s cap,
  1e-2 by default, under every gap in the table.
- **Open CASCADE carries the same gaps.** At the gaps themselves its
  tolerance is Arris's within 3% on every quadric pair (0.0386 and 0.0385,
  0.0346 and 0.0345, 0.0141 and 0.0140). It differs on the NURBS pairs —
  CTC-02's 0.0212 against 0.0105 and 0.0101 against 0.0053, CTC-05's
  0.0207 against 0.0169 — where it carries less, its `FixShape` having
  re-fitted an edge Arris keeps, and on CTC-02's 0.0150 cylinder pairs,
  where it carries 0.0386. A tolerance per entity, never a model-wide one.
- **No curve closes them.** In every row the two surfaces do not meet
  within the gap, so no edge curve, re-fitted or projected, can lie on
  both; healing a gap is growing a tolerance or moving a surface, and the
  second is not healing.

## Decision

**1. What healing is.** An entity's tolerance is raised to the gap
measured at it. The reader's cap, today `READ_GAP_FRACTION` of the part
and at most the model's `max_tolerance`, becomes with healing on
`arris_math::HEAL_GAP_FRACTION` of the part, 5e-4: `3.4×` the largest gap
Arris measured over the corpus (FTC-11's 1.5e-4), `1.9×` the largest
Open CASCADE carries relative to its part (FTC-07's 0.104 on 407, 2.6e-4),
and under the 1e-3 of the size healing tools commonly default their sewing
tolerance to. The constant lands with this ADR, unused until step 3. A
gap past the ceiling is still `Refusal::Gap`, naming the gap and the
ceiling.

**2. The model's `max_tolerance` bounds operations, not the read** (the
plan's open question 1, option (b)). A healed entity's tolerance may be
above `max_tolerance`; the reader's own ceiling is the only bound on what
it grew. An operation on such a body is unchanged: it refuses, as it
always has, an entity whose tolerance it would have to grow past the
model's ceiling, and the refusal names that entity. Option (a), raising
`Precision::DEFAULT.max_tolerance`, would move every operation's refusal
threshold for the sake of one stage; option (c) leaves the seven parts
where they are and tells the caller to raise a ceiling that is also an
operation's guard. What a fillet next to a 0.05 edge does is *measured*,
not argued: step 5 runs the battery on every part that now reads, and each
refusal there gets its row. If a stage's guard turns out to be wrong for a
healed body, that is the stage's backlog line.

**3. The record.** `ReadBody` gains `healing: Healing { grown: Vec<Grown>,
turned: Vec<FaceId> }` with `Grown { entity: AnyId, file: u64, gap: f64 }`:
each entity the read grew, the file entity it came from (the `#id` of its
`VERTEX_POINT` or `EDGE_CURVE`) and the gap it measured; each face it
turned. Provenance is unchanged (the plan's open question 2): every entity
stays `Generated` from its file entity, because healing changes no
topology — a `Modified` entry would claim an earlier state the model never
held, and a new `Role` would make every consumer's match on roles name a
state of the read. The roadmap's accept line, "a healed body's provenance
names every face sewn or turned", is met by the record, which names them.

**4. The orientation rule.** A face whose every shared edge is walked the
same way as the neighbour across it — both faces use the edge in the same
direction, which for a manifold shell means one of them is reversed — and
whose neighbours agree with each other, has its loops turned. A face that
disagrees on some of its shared edges and not all is still
`Refusal::Topology`: the file is inconsistent, not reversed. A closed
shell with no agreeing majority (two faces each disagreeing with the
other) is refused, not guessed. Applies to a shell of a solid and of a
sheet (ADR-0057 §1).

**5. The switch.** `ReadOptions::heal: Heal`, an enum, `Heal::Off` (today's
refusals, bit for bit) and `Heal::GapsAndOrientation`, the default: Open
CASCADE heals every read, and a consumer's part file is expected to open.
A body Arris writes after healing must also read back under default
options. `ReadOptions` and `ReadBody` gain a field each: `### Breaking` in
`CHANGELOG.md`, in step 3.

**6. Sewing in the sense of free edges is not done.** Faces that share no
edge in the file, matched and merged into one shell
(`BRepBuilderAPI_Sewing` proper), are a backlog line: no part of the
corpus needs it, every surface model read so far is a one-face open shell.

## Consequences

- Six of the seven `healing` parts can leave the row by step 3 if the
  checker accepts what they read as: CTC-02 AP203 refuses next at a NURBS
  band (the reader's subset), and the plan's step 2 classifies the two
  parts the checker rejected under a raised ceiling.
- The census mode stays: `real_parts --gaps` is how the next ceiling is
  chosen if a new part's gaps run past 5e-4.
- A healed body's edge may carry a tolerance above `max_tolerance`; the
  checker already judges at each entity's own tolerance, so it is not
  asked to change for that alone.
