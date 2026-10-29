# Idea: blend-residue-real-parts

- Status: Accepted (A); milestone 1 done (plans/battery-honesty, retired 2026-10-26); M2 done (plans/blend-runs-onto, retired 2026-10-27; the order of M3–M5 below); M3 done (plans/three-edge-run-vertex, retired 2026-10-28); M4 absorbed by plans/tolerant-tangency; M5 unplanned
- Raised: 2026-10-26
- Prompt (verbatim from the human): "blend residue: the 12 parts blocked at fillet"

## Problem

C11 opened on one number: the blend network blocks 12 of the 38 real parts
at `fillet` (`docs/ROADMAP.md` §C11). Ten of them are `TangentChain`, one
`BlendTooLarge`, one `VertexBlend`. A census run on 2026-10-26 against
`main` (`real_parts --tangent-chain`, `--vertex-blend`, `--run-over`,
`--census`) put each part's refusing sampled edge to Open CASCADE alone.
It splits the 12 into two halves that need different work:

| part | tier | refusing sampled edge | Open CASCADE alone | whose |
|---|---|---|---|---|
| CTC-04 | committed | e961 cylinder × sphere, the edge itself a tangent dihedral | refuses | the battery's |
| FTC-07 AP242 | fetched | e425 NURBS × cylinder, tangent dihedral | refuses | the battery's |
| FTC-08 | committed | e600 cylinder × plane, tangent dihedral | refuses | the battery's |
| STC-07 AP242 | fetched | e170 NURBS × cylinder, tangent dihedral | refuses | the battery's |
| CTC-01 | committed | none: today's sample builds | — | the battery's (stale operands) |
| FTC-10 | committed | none: today's sample builds | — | the battery's (stale operands) |
| FTC-06 AP242 | fetched | `BlendTooLarge`: the contact leaves its plane face into a neighbour | builds | the blend's |
| CTC-02 AP242 | fetched | `VertexBlend`: two edges of the run meet at a vertex of 3 edges, plane × plane | builds | the blend's |
| FTC-11 AP242 | fetched | e24 end at an inflection, its corner edge tangent within the vertex's tolerance | builds | the blend's |
| CTC-05 AP203, AP242 | fetched | e256 vanishing end within the edge's tolerance | builds invalid | the blend's, oracle contested |
| FTC-06 | committed | e0 end at a cusp, walls on either side (overhang tip) | builds | the blend's |

**The battery's half (6 parts).** The battery leaves out a tangent edge before it samples (`blendable_edges`, `crates/arris-debug/src/battery.rs:299`). It reads tangency at the
faces' tolerance, which is ADR-0040's test. Since ADR-0045 the blend reads
an edge's own dihedral at the **edge's** tolerance as well. That filter
dates from `c6-blend-network` step 1 (`5bbd2b71`) and was never moved to
ADR-0045's test. So an edge whose healed tolerance is wide gets sampled,
and the kernel then correctly refuses it as a tangent dihedral. Open
CASCADE refuses each of these four edges on its own too ("There are no
suitable edges"). Yet with the other three in the same `MakeFillet`, the
stage *builds* (`tools/oracle/oracle/recipe.py:1011` `Add`s every edge and
checks only the result). Most likely Open CASCADE drops the tangent edge
without saying so. ADR-0026 says a stage blocks a part only where Open
CASCADE builds and Arris refuses, and here Open CASCADE did not build what
was asked. CTC-01 and FTC-10 are the same in an older form: their
operands were written on 2026-09-26 (`real-part-corpus` step 6), before
the battery left out any tangent edge, and today's sample of either part
builds.

**The blend's half (6 parts)** is the real residue: four different
constructions, all of them already backlog lines.

The roadmap's other C11 items are the horn torus, the parallel-cylinder
row, a cusp with both edges blended, and the hinge leaf's knuckle. None of
them is the refusing sampled edge of any of the 12. They count only in the
census's each-edge-alone columns and in the script corpus, so they belong
to a later idea, not to this one.

## Milestone 1's result (2026-10-26)

The battery's half held, and a little more: re-derived and re-run, the
row is **4 of 38**, not the expected 6, all fetched: CTC-02 AP242 and
FTC-07 AP242 at `VertexBlend`, CTC-05 AP242 and FTC-11 at `TangentChain`.
Open CASCADE's volume with and without each tangent edge was identical, so
the drop is real (ADR-0070). FTC-06 (both tiers) and CTC-05 AP203 no
longer block (the sample, or a dropped edge). So:

- **M2 `blend-run-over`** loses FTC-06 AP242: it is the script corpus's 2
  (standoff roots, phone lip) alone.
- **M3** (CTC-02 AP242) now also holds FTC-07 AP242's `VertexBlend`.
- **M4** holds FTC-11 and CTC-05 AP242 (CTC-05 AP203 cleared).
- **M5 `overhang-tip-cap`** has no real part left (FTC-06 committed is
  both-refuse); it stays only for the script corpus or a later consumer.
- STC-07 AP242's fillet builds and the checker leaves four cylinder pairs
  undecided: a kernel bug, not residue, held as an ignored `regression/`
  fixture and a backlog line.

The table above is the census before the re-derivation.

## Constraints it runs into

- **ADR-0026** §5 and the amendment of step 9: what blocks a part, and
  how the battery is derived. Changing the sample or the oracle's verdict
  amends it.
- **ADR-0040 / ADR-0045**: the tangency tests the battery has to match.
- **ADR-0020**, closure before breadth: C11 was chosen from this
  histogram. If half the row is the measurement's fault, the histogram
  has to be corrected before it can rank anything.
- **C11's Accept** ("below 12 of 38"): fixing the battery alone gets there
  without building a single blend. That meets the letter of Accept and
  misses its goal (Q3).
- **git rules**: re-derived committed operands change `fixture.json` and
  `expected.json` (Open CASCADE re-run), so it is a `fixtures:` commit that
  says why. The fetched tier's operands are derived on every run, so
  nothing is committed for it.
- **ADR-0007 / ADR-0037 §1 / ADR-0042 §6**: these kept the overhang tip's
  B-spline cap out until NURBS faces existed. Since C10 they do (ADR-0062).
- Backlog lines 65, 67, 68 and the run-over family hold the residue's
  findings. Line 67 already notes that CTC-05's oracle is contested.

## Options

### A: battery honesty first, then the residue part by part
Milestone 1, `battery-honesty`, is planned (`docs/plans/battery-honesty.md`).
It is expected to take the count from 12 to 6, plus whatever the new
samples turn up on other parts.

Then the residue, one milestone plan each, ranked by what it unblocks:
- **M2 `blend-run-over`**: a contact leaving its face into a neighbour
  that is not the corner's. FTC-06 AP242 and the script corpus's 2
  (standoff roots, phone lip). 3 refusals.
- **M3 `three-edge-run-vertex`**: two edges of one run through a vertex of
  3 edges. CTC-02 AP242; Open CASCADE builds all 13 plane × plane edges
  asked. 1 part, 142 edges.
- **M4 `tolerant-tangency`**: the vertex's tolerance in the chain walk
  (FTC-11) and the vanishing end (CTC-05 ×2, oracle chosen per backlog
  line 67). 3 parts.
- **M5 `overhang-tip-cap`**: the fitted cap at a cusp whose walls lie on
  either side (FTC-06 committed). 1 part, and the hardest construction.

Cost: 4 + roughly 4 × 6–8 steps. That is five plans, the slot holding at
most two at a time.

### B: the residue only, battery left as it is
Build M2–M5 and leave the six artefacts in the histogram. That is cheaper
by one plan, but the histogram goes on reporting six blocks that no blend
construction can clear. C11's Accept can't be reached honestly, and
ADR-0020's ranking at C11's close would rest on a miscount.

### C: battery honesty only, then close C11
Fixing the battery alone gives 6 of 38, below 12. Close C11 on that and
let the next histogram pick again. This meets the number without
improving any blend. It also leaves the script corpus's 3 refusals, which
are half of why C11 was chosen.

### Do nothing
The first C11 plan would chase four tangent dihedrals that both kernels
refuse, and two stale samples.

## Recommendation

**A.** The measurement is fixed before anything is built on it, and the
residue is then ordered by refusals cleared, with run-over first because
it clears one real part and two scripts. B builds on a miscount. C reads
Accept's number and ignores its goal. If step 1 shows Open CASCADE really
blends the tangent edge, I'd change my mind: those four parts go back to
the blend's half (M4 would grow to hold them) and A shrinks to the
stale-operand fix.

An ADR-0026 amendment is needed (a dropped edge counts as Open CASCADE
refusing; the sample uses ADR-0045's test); milestone 1 writes it.

**The words.** The glossary doesn't define *battery* or *blend residue*,
though the docs use both everywhere. Proposed:
- **Battery**: the fixed operations each real-part solid is put through
  after its read (`measure`, `write_read`, `box_cut`, the three drills,
  `fillet`), each compared with Open CASCADE's; a stage Open CASCADE
  builds and Arris refuses blocks the part (ADR-0026). _Avoid_: test
  suite (for this).
- **Blend residue**: the blends Open CASCADE builds and Arris refuses
  once the blend network has been built: what C11 works through. A
  sample's own artefact is not residue.

## Decision for the human

Round 1:

❓ **Q1: Which shape?** A, battery honesty and then the residue; B, the
residue only; C, battery honesty and then close C11.
➡️ A: the residue's order is only worth something on a correct histogram.

❓ **Q2: The oracle's dropped edge.** When Open CASCADE builds a stage but
leaves out an asked edge, should the stage count as (A) Open CASCADE
refusing, so "both refuse"; or (B) only be kept out of the sample, the
oracle unchanged?
➡️ A: both. B alone would let the next stale or edge-case sample hide a
drop again.

❓ **Q3: C11's Accept.** Should the threshold read (A) "below 6 of 38
after the battery's correction, and the script corpus below 3"; or (B)
stay "below 12"?
➡️ A: otherwise milestone 1 meets Accept by itself.

❓ **Q4: The glossary entries.** Add *battery* and *blend residue* as
worded above?
➡️ Yes.

*Later*: the order of M3–M5 after M2 (it hangs on Q1 and on what
milestone 1's re-run shows); CTC-05's oracle (backlog line 67: the two
editions agreeing, or the volume less a deficit along the spine), when M4
is planned.

## Decision

Round 1, 2026-10-26: "yes to all".

- **Q1: A.** Battery honesty first (milestone 1), then the residue as
  M2 `blend-run-over`, then M3–M5.
- **Q2: A.** An asked edge that no Open CASCADE contour took counts as
  Open CASCADE refusing the stage, and the sample also uses ADR-0045's
  test. Milestone 1 writes the ADR-0026 amendment.
- **Q3: A.** C11's Accept becomes "below 6 of 38 after the battery's
  correction, and the script corpus below 3". Milestone 1 edits
  `docs/ROADMAP.md` §C11 with the corrected count.
- **Q4: yes.** *Battery* and *blend residue* are in `GLOSSARY.md`.

The *Later* questions are left to the plans: CTC-05's oracle is chosen by
M4's plan. M3–M5's order, set at M2's retirement (2026-10-27) from the
re-run (the real parts' `fillet` row still 4 of 38, the script corpus's
refusals 1): **M3, then M4, then M5**. M3 unblocks two parts (CTC-02 and
FTC-07 AP242, one construction), M4 two (FTC-11, CTC-05 AP242, with an
oracle question), and M5 has no real part left.
