# ADR-0026 — The real-part corpus: NIST's parts committed and fetched, the oracle reading healed, and every refusal mapped to the cycle it blocks

- Status: accepted (2026-09-26)
- Plan: `real-part-corpus` step 1
- Follows: ADR-0025 §6 and §7 (the corpus's two tiers, the battery and a
  coarse refusal table, left to this plan), ADR-0024 §2 (the
  differential's classes and its `Reason` count), ADR-0020 (the histogram
  picks the cycle after the reader)

## Context

ADR-0025 §6 decided that C4 closes on a refusal histogram over real parts,
in two tiers: a committed tier of public-domain parts, named as the NIST
MBE PMI test models, and a fetched tier, a pinned sample of a large public
dataset whose licence is confirmed before the plan names it. Its table from
refusal to cycle is coarse, and it leaves four rows open: `Topology`,
`Pcurve`, `Invalid` and `Degenerate`. It also leaves how the operations'
tangent and non-manifold reasons count. Three facts had to come from the
sources themselves, not from memory.

### The licences, as the sources state them (read 2026-09-26)

- **NIST MBE PMI test models** (nist.gov, "Download Free CAD Models, STEP
  Files, and Test Results", updated 2025-12-30): *"The test cases, CAD
  models, and STEP files can be used without any restrictions. Their use in
  other software or hardware products does not imply a recommendation or
  endorsement of those products by NIST. We would appreciate
  acknowledgement if any of the test cases, CAD models, STEP files, or
  screenshots of the models are used, however, the use of the NIST logo
  seen at the top of this page is not allowed in promotional materials."*
  The page links the STEP archive (`NIST-PMI-STEP-Files.zip`) and the D2MI
  project's CAD files (`NIST-D2MI-Models.zip`) under the same disclaimer.
  The archive's `README.txt` adds that the files *"are NOT reference STEP
  files without any errors"*, and that the exporting systems' names have
  been removed.
- **ABC dataset** (deep-geometry.github.io/abc-dataset): *"The copyright of
  the CAD models is owned by their creators. For licensing details, see
  Onshape Terms of Use 1.g.ii."* Onshape's terms (effective 2020-07-15)
  grant a licence *"to use the intellectual property contained in
  Customer's Public Document without restriction, including without
  limitation the rights to use, copy, modify, merge, publish, distribute,
  sublicense, and/or sell copies"*. The grant covers *"any Public Document
  owned by a Free Plan User created on or after August 7, 2018, or any
  Public Document created prior to that date without a LICENSE tab"*. For
  a document before that date that *"contained a tab called LICENSE
  reserving rights greater than the foregoing, those greater reserved
  rights will continue to apply"*, and likewise for any document owned by
  a user who is not on the free plan. The dataset's per-model metadata
  records the creation date (the page's sample model: 2014-08-19). It
  records neither the owner's plan nor whether the document had a LICENSE
  tab.
- **Fusion 360 Gallery Dataset** (its `LICENSE.md`, updated 11/2021):
  *"You may access, use, reproduce and modify the Dataset, in each case,
  only for non-commercial research purposes."* Redistribution of a
  portion is allowed only under the same restriction.

### What Open CASCADE's reader does on transfer

`STEPControl_Reader` runs `ShapeProcess::Operation::FixShape` on every
transfer by default (`STEPControl_Reader::GetDefaultShapeProcessFlags`).
The flag setter takes a `std::bitset` that the Python binding cannot pass.
Healing can still be turned off: `SetShapeFixParameters` with every
`FixShape.*Mode` set to `0`, **after** `ReadFile`, which is when the
transfer actor exists. Before it, the call is silently overwritten.
`read.step.sequence` no longer has any effect. Measured on
`nist_ctc_03_asme1_rc.stp`, which has 15 cylindrical faces each bounded by
two circles and no seam, as ISO 10303-42 allows:

| Reading | Cylinder faces with two wires | Seam uses | `BRepCheck` | Volume |
|---|---|---|---|---|
| healed (default) | 0 | 30 | valid | 331 938.27 |
| healing off | 15 | 0 | **invalid** | **−346 282.18** |

Unhealed, Open CASCADE's reading of a valid file is not a solid: its data
model, like Arris's, needs the seam, and the reader alone does not add it.

### First look: the reader over NIST's files

`inspect_step` and a timing harness ran `step::read` in a release build
over every STEP file of both archives, with nothing fixed. Per solid
instance:

| File (AP203 geometry only) | Solids | Outcome |
|---|---|---|
| `nist_ctc_01_asme1_rd` | 1 | `Gap` #1864: 0.02 past the cap of 0.01 |
| `nist_ctc_02_asme1_rc` | 2 | `Gap` #11056 (0.0112); `Unsupported` `SHELL_BASED_SURFACE_MODEL` |
| `nist_ctc_03_asme1_rc` | 1 | `Invalid`: L4 zero signed area, 12 faces |
| `nist_ctc_04_asme1_rd` | 4 | `Malformed` #7319, no frame; 3 × `SHELL_BASED_SURFACE_MODEL` |
| `nist_ctc_05_asme1_rd` | 2 | `Gap` #4444 (0.0182, after 66.6 s); `SHELL_BASED_SURFACE_MODEL` |
| `nist_ftc_06_asme1_rd` | 1 | `OpenLoop` #351 |
| `nist_ftc_07_asme1_rd` | 1 | `Gap` #1963: 0.02 (after 20.3 s) |
| `nist_ftc_08_asme1_rc` | 1 | `Malformed` #3672, no frame |
| `nist_ftc_09_asme1_rd` | 1 | **read**, 158 faces, `Full` clean (0.81 s) |
| `nist_ftc_10_asme1_rb` | 1 | `Invalid`: L4 no outer loop (after 9.5 s) |
| `nist_ftc_11_asme1_rb` | 1 | `Invalid`: L4 zero signed area |

The fetched tier's files, the same way (an instance per placement):

| File | Instances | Outcome |
|---|---|---|
| `nist_ctc_01_asme1_ap242-e1` | 1 | `Malformed` #16: a `CAMERA_MODEL_D3` where a placement belongs |
| `nist_ctc_02_asme1_ap242-e2` | — | **did not finish** in 900 s |
| `nist_ctc_03_asme1_ap242-e2` | 2 | **read**, 120 faces each, `Full` clean |
| `nist_ctc_04_asme1_ap242-e1` | 4 | 2 × `Gap` #11276 (0.0106, 15.6 s); 2 × `SHELL_BASED_SURFACE_MODEL` |
| `nist_ctc_05_asme1_ap242-e1` | 1 | `Unsupported` #1645: a `VERTEX_LOOP` whose seam crosses its face's bound |
| `nist_ftc_06_asme1_ap242-e2` | 2 | **read, and `Full` fails**: S5, faces meeting away from their shared edges |
| `nist_ftc_07_asme1_ap242-e2` | 51 | 17 × `Gap` #1534: 0.02; 34 × `SHELL_BASED_SURFACE_MODEL` (108 s) |
| `nist_ftc_08_asme1_ap242-e1-tg` | 0 | nothing: its `TESSELLATED_SOLID` is not seen |
| `nist_ftc_08_asme1_ap242-e2` | 13 | `Invalid`: L4 zero signed area |
| `nist_ftc_09_asme1_ap242-e1` | 6 | **read**, 158 faces, `Full` clean; 5 × `SHELL_BASED_SURFACE_MODEL` |
| `nist_ftc_10_asme1_ap242-e2` | 247 | 19 × **read**, 264 faces, `Full` clean (the same body 19 times); 228 × `SHELL_BASED_SURFACE_MODEL` (790 s) |
| `nist_ftc_11_asme1_ap242-e2` | 11 | `Gap` #875: 0.02 |
| `nist_stc_06_asme1_ap242-e3` | 13 | `Invalid`: L4 zero signed area |
| `nist_stc_07_asme1_ap242-e3` | 7 | 5 × `Malformed` #1219, no frame; 2 × `SHELL_BASED_SURFACE_MODEL` |
| `nist_stc_08_asme1_ap242-e3` | 4 | **read**, 270 faces each, `Full` clean; 2 × `SHELL_BASED_SURFACE_MODEL` |
| `nist_stc_09_asme1_ap242-e3` | 1 | **read**, 125 faces, `Full` clean |
| `nist_stc_10_asme1_ap242-e2` | 1 | `Gap` #4962: 0.02 |
| `nist_ctc_01_asme1_ap203` | 1 | `Gap` #2018: 0.02 |
| `nist_ctc_02_asme1_ap203` | 1 | `Gap` #5805: 0.02 |
| `nist_ctc_03_asme1_ap203` | 1 | `Invalid`: L4 zero signed area |
| `nist_ctc_04_asme1_ap203` | 2 | `SHELL_BASED_SURFACE_MODEL`; `Unsupported` #1954, the `VERTEX_LOOP` seam |
| `nist_ctc_05_asme1_ap203` | 1 | `Unsupported` #774, the `VERTEX_LOOP` seam |
| D2MI `827-9999-904 rev c` | — | the file does not parse: a string holds a `\` that starts no directive (line 45 751) |
| D2MI `827-9999-905` to `-908` | 1 each | `Invalid`: L4 zero signed area |

No file panicked. The first look shows that a refusal is not always the
file's fault:

- **A cylinder band bounded by two circles** (`ctc_03`, `ftc_11`, and the
  same faces in `ftc_08`'s AP242 edition) is valid STEP. The reader
  leaves it seamless, and the reader's own checker then refuses it as
  `Invalid`. Open CASCADE adds the seam and reads it.
- **An `AXIS2_PLACEMENT_3D` with no reference direction** along the x
  axis (`ctc_04`, `ftc_08`) is refused as `Malformed`, where ISO 10303-42
  defines the default direction.
- **Every `Gap` of exactly `0.02`** is not a measured distance. It is the
  fit tolerance after one growth past the cap (`GAP_GROWTH`). The reader
  reports it when a curve measured off its surface by less than the cap
  still has no pcurve fit at the cap (`ctc_01`'s #1864 is a cubic
  B-spline edge). Open CASCADE's reading of `ctc_01` carries no vertex
  or edge tolerance above `0.0065`.
- **A `VERTEX_LOOP` on a cone or sphere** whose seam the reader draws
  across the face's other bound (`ctc_04` and `ctc_05` PMI editions) is
  refused as `Unsupported`. The limit is the reader's choice of seam, not
  anything in the file.
- **Placements that are not there.** Every AP242 edition with more than
  one instance of a solid (`ctc_03`, `ctc_04`, `ftc_07`, `ftc_08`,
  `ftc_10`, `ftc_11`, `stc_06`, `stc_07`, `stc_08`: 2 to 19 each) holds
  one solid by Open CASCADE's reading, which transfers the product
  structure.
  Arris's flattening takes some other relationship for a placement, and
  where the instances read, it returns bodies the part does not have.
  That is a wrong solid, not a count.
- **A read that fails the checker at `Full`** (`ftc_06` AP242, S5): the
  reader's `Fast` check passed a body whose faces meet away from their
  shared edges. It is a wrong solid until shown otherwise.
- **A presentation entity** (`CAMERA_MODEL_D3`, `ctc_01` AP242) is
  followed where a placement belongs, and a `TESSELLATED_SOLID`
  (`ftc_08` `-tg`) reads as no solid at all, where ADR-0025 §2 counts the
  tessellated relatives of a faceted B-rep as a refusal.
- **Time.** Three refused reads take 9–67 s. `ftc_10` AP242 takes 790 s,
  and `ctc_02` AP242 does not finish in 900 s.

## Decision

### 1. The committed tier: NIST's AP203 geometry-only files

The eleven files of `NIST-PMI-STEP-Files.zip`'s `AP203 geometry only/`,
one per test case (CTC 01–05, FTC 06–11), are committed as `real/`
fixtures (`tests/fixtures/real/nist-<case>/`), one per file.

- **Budget: 5 MB of STEP text, committed uncompressed.** The eleven are
  4.43 MB raw and 0.80 MB after `gzip -9`, which is what git's zlib
  stores. A compressed file would put a decompressor in
  `arris-debug`'s dependencies, and it would save the repository less
  than git's own compression already does. A later part that would
  exceed the budget goes to the fetched tier.
- **Each fixture records** its source URL, the archive's `sha256`
  (`8fa78429e6d8d9b0d7681d223b6aa9ec98c3772185c55b1a0e3679b21c181911`),
  the file's own `sha256`, and the licence line quoted above. NIST's
  acknowledgement is given in `tests/fixtures/README.md`.
- These are the geometry-only editions. The PMI editions carry the same
  test cases from other exporters, 2–5 times larger, and they go to the
  fetched tier.

### 2. The fetched tier: NIST's other editions; ABC and Fusion 360 dropped

- **ABC is dropped.** Its licence per model depends on two facts the
  dataset does not record, the owner's plan and a LICENSE tab, and its
  sample model predates 2018-08-07. A licence that cannot be confirmed is
  never assumed (ADR-0025 §6). If Onshape's API is ever shown to expose a
  document's LICENSE tab, a filter on it could admit a confirmed subset.
  That is a backlog line, not this plan's work.
- **Fusion 360 Gallery is dropped.** Its licence restricts use to
  non-commercial research. Arris is a library whose consumers include
  commercial ones, and its CI serves them.
- **The fetched tier is the rest of NIST's public-domain STEP.** From
  `NIST-PMI-STEP-Files.zip` it takes the AP242 editions (e1, e2, e3, the
  tessellated `-tg`) and the AP203-with-PMI editions, 22 files and
  40 MB. From `NIST-D2MI-Models.zip` (`sha256`
  `f20e36fb68633129dfedadf209ba4836bb0e42e18f12e7a2c28140ecdfb26cf3`) it
  takes five STEP files. `tools/real-parts.sh` fetches both archives into
  `target/real-parts/` and checks the archive hashes and each file's hash
  against a committed list. It never commits the files. Their licence
  would allow committing them, so a failure found there may be committed
  as an excerpt, and no recipe is needed.
- It is smaller than a dataset sample, and it is kept for what it adds:
  other exporters, three AP242 editions, PMI and presentation entities
  around the same geometry, and files too large for the budget. The
  script and the nightly job stay ready for a larger source whose licence
  is confirmed later.

### 3. The oracle reads healed, and records what healing did

- **The oracle's measures come from the healed reading**, which is Open
  CASCADE's default. Unhealed, its reading of a valid file can be an
  invalid shape with a negative volume, so holding Arris to it would hold
  Arris to a wrong answer.
- **The oracle also reads with healing off** (§Context's parameters) and
  records `occt_heals: true` for a solid whose unhealed reading fails
  `BRepCheck_Analyzer`, or whose counts differ from the healed ones.
  Such a solid is one that Open CASCADE had to repair or complete before it
  was a solid.
- **Arris against a healed solid:**
  - An Arris read is held to the healed measures within the fixture's
    tolerances, widened to the read body's own as the read-back stage
    widens them (ADR-0023).
  - An Arris refusal is recorded as a refusal and counted, never as a
    disagreement.
  - A healed solid Arris reads to other measures is a wrong solid, as
    anywhere else.
- **Counts.** A read solid's counts are held to the healed reading's only
  where `occt_heals` is false. Healing adds seams and splits edges, and
  that is not a disagreement about the part.

### 4. A refusal is recorded only when it is right

Step 4 records a refusal only where its `why` can say why it is right:
the file holds an entity outside the subset, or it describes no solid by
ISO 10303-42. A refusal of a valid entity that the subset maps, like the
seamless band, the default reference direction, the misreported gap,
the `VERTEX_LOOP` seam or the followed camera, is a reader bug. The kernel rules treat it as they
treat a wrong solid: it is shrunk to a `regression/` fixture, its part
waits under a named exclusion, and plan step 5 fixes it. So is a read that
does not finish within the corpus's time budget.

### 5. The table: every refusal to the cycle it blocks

The histogram's rows are the named cycles of `docs/ROADMAP.md`, plus
*counted as itself* for a refusal that no named cycle removes. The table
is code with exhaustive matches (plan step 7). It takes the refusal
itself rather than its kind, because one kind splits by the entity it
names.

**The reader's `Refusal`:**

| Refusal | Blocks | Why |
|---|---|---|
| a file that does not parse (`ReadError::Parse`), counted once per file | itself: unparsed | the file breaks Part 21; a lone `\` in a string is the one met |
| `NoLengthUnit` | itself | the file's omission; nothing to build |
| `Malformed` | itself | the file breaks the schema |
| `Offset` | sweep | an offset surface or curve is the sweep cycle's variant decision |
| `Composite` | sweep | composite curves and surfaces arrive with the sweep cycle's paths |
| `CurveBounded` | itself | a representation of trimming that no named cycle adds |
| `DegenerateTorus`, `SelfIntersectingTorus` | itself | the data model holds `R > r` only; a variant decision on evidence |
| `Unsupported` naming `SHELL_BASED_SURFACE_MODEL` or `FACE_BASED_SURFACE_MODEL` | healing | sheet bodies are the healing cycle's |
| `Unsupported` naming `FACETED_BREP` or a tessellated solid or shell | itself: faceted | ADR-0025 §6 |
| any other `Unsupported` | itself: outside the subset | ADR-0025 §6 |
| `Degenerate` | itself | the file's geometry describes nothing |
| `Topology` | healing | sewing, splitting and turning loops to close a shell are healing (ADR-0025's amendment of steps 14 and 15) |
| `Pcurve` | NURBS | a projection or fit that fails on a free-form face is the NURBS cycle's robustness; a curve off its surface is a `Gap`, not this |
| `OpenLoop` | healing | ADR-0025 §6 |
| `Gap` | healing | ADR-0025 §4: closing it is sewing |
| `Invalid` | healing | a body its own file makes invalid needs repair |

**The operations' `OpError`, per battery stage:**

| Error | Blocks |
|---|---|
| `Unsupported` with a `Nurbs` surface or curve in the pair | NURBS |
| `Unsupported` without one, at the fillet stage | blend network (a pair outside ADR-0007's table) |
| `Unsupported` without one, at any other stage | itself: an unsupported pair (none is expected since C3) |
| `Degenerate` with `BlendTooLarge`, `TangentChain` or `VertexBlend` | blend network |
| `Degenerate` with `DirectionNotNormal` or `EllipticRevolve` | sweep |
| `Degenerate` with `NotSolid` | healing |
| `Degenerate` with `TangentContact` or `NonManifold` | itself: non-manifold. Only a `General` body holds it, and no named cycle builds one |
| `Degenerate` with `BesideSingularity` | itself: beside a singularity, a limit of ADR-0021 |
| `Degenerate` with `Empty` or `ZeroThickness` | itself: a result with no material, the battery's operands missing the part |
| `Degenerate` with any other `Reason` | itself, by the `Reason`'s name. These are argument errors (`NonFinite`, `NotPositive`, `NoEdges`, `RepeatedEdge`, `EdgeNotInBody`, `NotProjectable`, `DegenerateEdge`, `ProjectionCollapses`, `NotPlanar`, `OutOfDomain`, `Singular`) and sweep-profile errors (`ProfileCrossesAxis`, `AxisNotInProfilePlane`, `AngleAboveTurn`, `SpindleTorus`). The battery should never raise one, so a count there is a battery bug to read |
| `InvalidInput`, `Profile`, `Tolerance`, `NotFound` | itself, by name |
| `Internal` | **never counted**: a kernel bug, like a panic (ADR-0024 §2), shrunk to `regression/` |

- **The stage names are the battery's**: `read`, `measure`, `write_read`,
  `box_cut`, `drill_x`, `drill_y`, `drill_z`, `fillet`. The histogram
  counts a part once per cycle, at the first stage that cycle blocks it,
  so *cycle → parts blocked* counts parts and not refusals. It also prints
  the count per stage beneath.
- **The mapping is data about cycles, not a judgement of the file.** A
  row changes only by an ADR, the way a new variant changes a `match`: a
  new `RefusalKind` or `Reason` fails to compile until the table has its
  row.

### 6. Time

The first look's slowest reads are refusals that spend their time in
repeated pcurve fits. The corpus runner therefore times each part's read,
and plan step 4 settles the nextest group (the plan's open question) from
those timings. A read that does not finish within 60 s in a release build
is a reader bug under §4, not a timing to accommodate.

## Consequences

- `tests/fixtures/real/` holds eleven NIST parts, 4.4 MB. The repository
  records NIST's disclaimer and acknowledgement beside them.
- The oracle's reader gains a second, unhealed reading per `part` fixture,
  and `expected.json` gains `occt_heals` per solid.
- The histogram covers the committed tier and NIST's other editions. It
  does not cover a large third-party sample, and C4's number says so.
- Before C4 can close, plan step 5 grows to the reader bugs this ADR
  lists: the seamless band, the default reference direction, the
  misreported gap of a fit that fails at the cap, the `VERTEX_LOOP` seam
  across a bound, the followed presentation entity, tessellated solids
  not counted, the slow reads, the placements that are not there, and
  `ftc_06` AP242's read that fails `Full`.
- Backlog gains a line for an ABC subset filtered by a confirmed licence.

## Alternatives considered

- **ABC under the Onshape grant, assumed.** Most of its models may well
  be covered. But "may" is not "confirmed", and ADR-0025 §6 forbids naming
  a tier on an assumed licence.
- **Fusion 360 Gallery for CI only**, as "research". A kernel meant for
  commercial consumers, tested for them, is not the non-commercial
  research the licence names.
- **Dropping the fetched tier and committing every NIST file.** 45 MB of
  STEP in the repository is 10× the budget, for editions of the same test
  cases.
- **The oracle unhealed**, as the plan first proposed. It is an invalid
  shape exactly where a file is valid and Open CASCADE's model needs more
  than the file gives, so Arris would be held to a wrong answer.
- **The oracle healed and nothing recorded.** Arris's refusal of a
  seamless band and of a real gap would look the same beside it. The
  `occt_heals` flag is what tells them apart.
- **`blocks_refusal(RefusalKind)`**, as the plan's delta first wrote it.
  `Unsupported` covers a shell-based model (healing) and a faceted B-rep
  (itself), so the kind alone cannot place it. Splitting the kind would
  change `arris-io`'s public enum, which this plan does not do.
- **Recording every refusal as the reader returns it.** It would count
  the reader's own bugs as the healing cycle's demand and rank the next
  cycle on them.

## Amendment (2026-09-26, plan `real-part-corpus` step 3)

- **Counts are held wherever healing changed no topology**, not only
  where `occt_heals` is false. The first part read, `nist_ftc_09_asme1_rd`,
  is healed: unhealed, Open CASCADE's reading of it fails
  `BRepCheck_Analyzer` with a volume of 902 599 against 136 445. Yet its
  counts are the same either way (300/454/158/228). §3 skips counts
  because healing adds seams and splits edges, and it has done neither
  here. So `expected.json` records the unhealed reading's counts beside
  `occt_heals`, and a read solid's counts are held wherever the two
  readings' counts agree.

## Amendment (2026-09-26, plan `real-part-corpus` step 4)

- **A part waits under an exclusion, and is not run.** A part meeting a
  reader bug records the outcomes it is to have, and names the
  `regression/` part fixture the bug was shrunk to in `waits_on`. The
  runner skips it until then, and the lint fails once that fixture leaves
  `regression/`, as ADR-0024 §2's exclusions do. Asserting the part's
  present, wrong outcome instead would run CTC-05's 88 s read on every
  commit, to prove a failure its regression fixture already proves.
- **The committed tier stays in the hook.** With the waiting parts
  skipped, the `real_` tests take 2.7 s in the test profile, FTC-09's
  checker at `Full` the most of it, so no nextest group of their own is
  needed. The plan's open question on time is settled on that number.
- **A time budget is a fixture's field, set on one fixture.** A slow read
  is shrunk to a part fixture with `read_seconds: 60`. It runs in the
  test profile's optimised build, which came to 88 s where release took
  66. No other fixture carries a budget, since the time of a test that
  shares the machine with a thousand others is no assertion.
- **The committed tier's first outcome.** Of the eleven parts, two run:
  FTC-09 reads, within 1e-14 of Open CASCADE's measures, and CTC-02's
  solid is refused for a real gap. Nine wait on five bugs. Three are the
  step-1 findings: the seamless band (CTC-03, FTC-10, FTC-11), the axis
  without a reference direction (CTC-04, FTC-08) and the fit reported as a
  gap (CTC-01, FTC-07). One is new: an edge through a sphere's pole
  (FTC-06, its "open loop" a half sphere bounded by a meridian circle).
  The last is the slow read (CTC-05). Every `SHELL_BASED_SURFACE_MODEL` of
  the tier is a one-face open shell, and its refusal is right.


## Amendment (2026-09-26, plan `real-part-corpus` step 5)

- **An edge's measured gap is the fitted pcurve's worst, not its worst at
  the checker's samples.** CTC-04 and FTC-08 read once their placements
  did, and then failed at the mesh. A corner on a fitted pcurve stood
  1.2e-7 to 9.3e-6 off its shared position. The edge's tolerance, measured
  at the checker's 23 samples, fell short of the fit's deviation between
  them by up to 15%. The reader now samples each use at the checker's
  samples and at the `PCURVE_SAMPLES` the fit is held to, and climbs each
  peak past the model's default to its top by golden section. An exact
  pcurve's gap is rounding, below the default, and is not climbed.
- **A mesh corner is held to its face's boundary tolerance.** The corpus
  runner held every corner to the face's tolerance, which only a kernel
  built model satisfied. It now holds each to the largest tolerance of
  the face and the edges and vertices bounding it (ADR-0012's amendment).
- **An unset reference direction along the axis is to rounding.** CTC-04
  and FTC-08 place circles on the axis `(-1, -6.1e-17, 0)` with no
  reference direction. ISO 10303-42's `first_proj_axis` swaps world `X`
  for world `Y` when the axis is world `X`, and the reader now makes that
  swap wherever world `X`'s part perpendicular to the axis is rounding —
  the test `Frame::new` makes of any hint. A written reference direction
  along the axis is still `Malformed`. Both parts read, within the
  oracle's measures, and their shrunk fixture moves to `real/`.
- **The `real_` tests stay in the hook at 17 s.** CTC-04 takes 17 s in
  the test profile and FTC-08 6 s, against step 4's 2.7 s for the whole
  set. Under nextest each runs beside the property shards of 40–60 s, so
  the hook's wall time does not move, and the step 4 settlement stands.
- **A band is joined by a seam, splitting an edge where it must.** A face
  whose two loops each wrap one period of its surface — a cylinder's side
  bounded by its two circles, as ISO 10303-42 allows — is one loop in
  Arris: the two joined by a rebuilt seam along the surface's isocurve
  (a ruling, a meridian, a torus's circle), walked out and back, as a
  `VERTEX_LOOP` is joined to its apex. Where no vertex of the one loop
  faces one of the other — CTC-03's circles start a quarter turn apart —
  the other loop's edge is split where the first vertex faces it, and
  every use of that edge on every face with it, each use keeping its
  pcurve over its part of the range. A NURBS surface's band is refused,
  since its isocurve has no exact form, as is an edge a second band
  would split again. CTC-03 and FTC-11 read. FTC-10 then meets eight
  cones bounded by their base circle alone, their apex implicit: the
  next cause, shrunk to `regression/cone-face-without-its-apex`.
- **The seam-crossing test moves each chord whole.** It moved each end of
  a sampled chord to its own period, so a loop passing the seam's far
  side made a chord across the whole domain, which crossed the seam
  (FTC-10). A crossing within the parametric band of the seam's ends is
  the loop meeting it at its vertex, and is not one either.
- **The oracle measures a part's spline-bounded solid by Gauss–Kronrod.**
  Healing gives every seamless face a B-spline seam pcurve, so every NIST
  reading is spline-bounded, and the plain adaptive integration it took
  split FTC-11's Ixx and Iyy by 3.3 in 1.6e6 and made Ixy −1.1, where the
  part is a solid of revolution. Its fixed-order and Gauss–Kronrod
  integrations both keep the symmetry to 1e-13 and agree with Arris
  there. A part now takes Gauss–Kronrod, as an extrusion's result does.
  Every other part's numbers moved by 1e-12 at most, and are regenerated
  with it. A recipe's result keeps its integration and its numbers.
- **A face of one wrapping loop is closed at its apex.** A cone bounded by
  its base circle alone, its apex written nowhere, as FTC-10's drill
  points are, is one loop wrapping a period. The reader adds a vertex at
  the singular point on the side of the loop the face lies on, and joins
  it by a seam as it joins a `VERTEX_LOOP` (a ruling to a cone's apex, a
  meridian to a sphere's pole). A face with no singular point on that
  side would be unbounded, and is refused. FTC-10 reads, in 20 s in the
  test profile.
- **An edge through a singular point is split there.** FTC-06 bounds a
  half sphere by one meridian circle whose vertex is at the north pole and
  which runs through the south. No pcurve runs through a pole, and each
  side of it has one (ADR-0021), so before any face is walked, the reader
  splits every edge whose curve passes a singular point of a face it
  bounds, away from its own ends, with a vertex on the point. Every use of
  it on every face walks its pieces, and the walk then adds the poles'
  degenerate edges as it does for any edge ending there. FTC-06 reads.
- **A fit's tolerance may pass the cap; the gap it leaves may not.** A fit
  holds its image to a fraction of the tolerance it is asked for, and the
  image is never nearer the curve than the curve is to its surface. CTC-01's
  edge #1864 lies 0.0065 off its cylinders, so no fit at the cap of 0.01
  could pass, and the reader reported the fit tolerance one growth past
  the cap as a gap of 0.02. The tolerance a fit is asked for was always a
  search step, and the gap the pcurve leaves is measured afterwards. So
  the search now runs one growth past the cap, the cap judges the
  measured gap, and a fit that fails even there is refused as a fit, with
  the fit's own error, never as a gap. CTC-01 reads. FTC-07 then meets
  fitted pcurves past the knot domain of eight B-spline faces, shrunk for
  now to its own file, `regression/nurbs-pcurve-leaves-domain`.
- **A fit that must miss is not tried.** A fit holds its image to half the
  tolerance it is asked for, so one asked for less than twice the curve's
  farthest distance from its surface misses, and on a B-spline face a
  miss is a fit refined to thousands of spans, seconds each. CTC-05's 66 s
  and FTC-07's 170 s were those misses, the tolerance climbing to what the
  curve needed one miss at a time. After the first miss, the reader now
  measures the curve's farthest distance at the fit's samples, each peak
  climbed, refuses it as a gap if it is past the cap, and asks next for
  four times it. CTC-05 reads to its right refusals in 1 s. A fit that
  passes on its first try is as it was, and the committed dumps moved
  only in pcurves fitted after a miss, and the tolerances they carry.
- **A fitted pcurve on a B-spline face is held inside its knot domain.**
  With its fits fixed, FTC-07 read to eight faces whose fitted pcurves
  dipped 6e-7 past a unit knot domain, fitted between projections the
  domain clamps, and the checker refused it (F1). The reader moves a
  pcurve's control points into the domain in each direction the surface
  does not close in, which holds the curve there, and measures the gap
  afterwards as for any pcurve. FTC-07 then reads in 19 s, but the checker
  at `Full` cannot decide S5 for a fillet torus beside a plane nearly
  tangent to its rim: `intersect_surfaces` refuses the section. That is
  the intersector's limit, not the reader's. It is a backlog line, its
  operands an ignored test in `arris-geom`, and FTC-07 waits on
  `regression/torus-plane-section-undecided`.
- **A seam root beside a tangency is the tangency.** FTC-07's plane is not
  near its torus but tangent to it, on the rim, on the torus's seam. The
  tracer's root search along the seam landed 6.8e-7 from the singular
  point, outside the cell the point's own precision sizes, and could not
  prove the root alone: the distance is quadratic there, within the
  tolerance over a run some 1e-3 wide. A root joined to a singular point
  on the seam by a run within the tolerance is now that point. The pair
  is one touch, S5 decides it, and FTC-07 reads whole, in 35 s in the test
  profile with the checker at `Full`. Every part of the committed tier
  runs, and none waits.

## Amendment (2026-09-26, plan `real-part-corpus` step 7)

- **A boolean that meets a NURBS face in classifying a piece refuses the
  pair.** Every cut and drill of FTC-07 and FTC-10 returned
  `Internal(Classify)`: a piece is classified by a ray, and a line has no
  closed form against a NURBS surface. It is now `Unsupported`, naming
  the piece's face and the other operand's first NURBS face, the one the
  ray meets. That is §5's NURBS cycle, and the eight stages record
  `arris-refuses`.
- **A cusp in a face's loop turns by +π or −π as the curvature says.**
  CTC-04's plane f393 and FTC-06's f0 are bounded by B-spline edges and
  blends that leave a line tangent to it: two cusps in one, ten in the
  other. The turn at a cusp is ±π, and `turn` picked the sign by
  rounding, so the loop summed to 0 or −4 turns and every face of the
  part failed to split. The split now signs it by the curvature the
  node's order already used. A square less a quarter disc, extruded and
  cut (`boolean/spandrel-cavity-cut`), is the shrunk case: both cusps
  rounded to −π, and its cap read as a hole. All eight cuts and drills of
  both parts now agree with Open CASCADE.
- **A recipe on a real part is held as its battery holds it.** The two
  NIST fixtures, moved to `boolean/`, met the corpus's defaults: CTC-04's
  centroid was 1.5e-7 off at 360 mm from the origin, against the
  `centroid_abs` of 1e-7 sized for primitives, and FTC-06's counts were
  those of Open CASCADE's healed reading (§3). A recipe with a `step`
  operand now has its measures widened to the result's own tolerance, as
  the part's reading and battery have them. A healed part's counts go in
  `counts_differ` with the reason. The runner also skips the B-spline
  read-back for such a recipe: reading Open CASCADE's conversion of the
  cut part took 300 s for FTC-06 and 520 s for CTC-04, which is the
  reader's cost on a large all-B-spline file (the fetched tier's slow
  reads, step 9). That cost is a backlog line.

## Amendment (2026-09-26, plan `real-part-corpus` step 8)

- **The table is `arris_debug::histogram`.** `blocks_refusal`,
  `blocks_parse` and `blocks_reason` are §5's rows as exhaustive matches
  with no wildcard arm, over `Refusal`, `ReadError`, `OpError` and every
  `Reason`. A `Reason` counted "by its name" is the variant's name.
- **Construction geometry counts as itself.** A surface model that only a
  `CONSTRUCTIVE_GEOMETRY_REPRESENTATION` holds — the exporter's
  'supplemental geometry', two of CTC-04's three — is no body of the
  part, and healing would not make it one. The reader now says so in the
  refusal's `name` (`SHELL_BASED_SURFACE_MODEL in a
  CONSTRUCTIVE_GEOMETRY_REPRESENTATION`), and the table counts it as
  *itself: supplemental geometry*. No type changes: the refusal is still
  `Unsupported`, and only the text of its `name` grows. A surface model
  any other representation holds stays the healing cycle's.
- **A part fixture records the cycle of each refusal it holds** (`blocks`,
  by stage: `read`, or a battery stage recorded `arris-refuses`), and the
  part runner holds the record to the table on every run. So the
  committed tier's histogram counts from its fixtures in milliseconds,
  while `cargo test` proves each count against the kernel. The battery
  example's `--record` writes it.
- **Only a stage Open CASCADE builds and Arris refuses blocks a part.** A
  stage both kernels refuse is counted as a class, never under a cycle:
  the fillet whose radius neither can fit says nothing about which cycle
  Arris lacks.

## Amendment (2026-09-26, plan `real-part-corpus` step 9)

- **The fetched tier runs by script.** `tools/real-parts.sh` fetches both
  archives, checks their hashes and each file's against
  `tools/real-parts.sha256` (27 files), and surveys each file in its own
  process of the `real_parts` example, under a timeout
  (`arris_debug::survey`). A survey holds the file as the part runner
  holds a committed part, but counts instead of stopping. It writes a
  scratch part fixture, has Open CASCADE read it through the oracle's
  cache, holds every solid Arris reads to it, and runs the battery. The
  histogram and the failure list go to `target/real-parts/`. A failure
  is excluded only by a line of `tools/real-parts.waits` naming a
  fixture still under `regression/`, and the corpus lint counts those
  lines among the part exclusions. The nightly runs it as one job,
  with the archives cached.
- **The battery runs on the first placement of each solid.** Another
  placement is the same solid moved, and FTC-10 AP242 would otherwise run
  it nineteen times.
- **A file whose every solid Arris refuses is not read by the oracle.**
  Nothing in it is held to the oracle's reading. Open CASCADE's
  Gauss–Kronrod measure of CTC-02 AP203's healed solid ran past twenty
  minutes, where its read takes 0.6 s. A file the reader returns neither
  a solid nor a refusal of is a failure, since that is a solid it does
  not see.
- **A presentation places nothing.** The phantom placements and the
  followed camera were one cause. AP242's saved views are
  `DRAUGHTING_MODEL`s, some of them complex instances that are also a
  `REPRESENTATION`, and each maps the part's shape into itself by a
  `MAPPED_ITEM`, from the origin or from a `CAMERA_MODEL_D3`. The
  flattening took each map for a placement. A `DRAUGHTING_MODEL` or a
  `…_PRESENTATION_REPRESENTATION` is now no node of it. Every AP242
  edition reads one instance per solid, as Open CASCADE does, and CTC-01
  AP242 reads (`real/saved-view-draughting-model`).
- **A `TESSELLATED_SOLID` or `TESSELLATED_SHELL` is refused where it
  stands**, as a faceted relative (ADR-0025 §2). FTC-08's `-tg` edition
  read to nothing before.
- **The slow reads are gone.** Step 5's fit fixes, and one instance read
  where there were nineteen, bring FTC-10 AP242 from 790 s to 0.5 s in a
  release build, and CTC-02 AP242 from past 900 s to 8 s. Every read of
  the tier is under §6's 60 s. The whole script takes 94 s on six jobs
  with the oracle's cache warm.
- **Six failures are excluded, each committed as an excerpt.** The first
  full run met them after the two fixes above.
  `tools/oracle/excerpt.py` cuts the failing solid out of its file, with
  a product for Open CASCADE to transfer from, as a `regression/` part.
  Such a part commits no dump, and its runner holds none.
  - The checker at `Full` finds faces meeting away from their shared
    edges (S5) on four reads that Open CASCADE holds valid: mitred cylinders
    of one radius (CTC-02 AP242, FTC-06 AP242), a torus elbow on
    cylinders of its minor radius (CTC-05 AP203), and holes into a bore
    with crossing axes (STC-10 AP242). Each is a wrong solid until shown
    otherwise. Which side is wrong, the reader's body or the checker's
    section, is the first thing their fix has to find.
  - FTC-10 AP242's mesh puts a point of one face 1.4e-4 from where its
    (u, v) evaluates.
  - CTC-04 AP203's drill along its second axis is
    `Internal(Fault::Geometry)`.
  Each is a backlog line. Open CASCADE's `STEPControl_Reader` holds all
  of them, so none is a refusal to count, and the histogram counts their
  other stages as they are.

## Amendment (2026-10-26, plan `battery-honesty` step 1)

- A stage in which Open CASCADE left out an asked edge counts as Open CASCADE refusing it, and the sample reads ADR-0045's tangency tolerance: ADR-0070.
