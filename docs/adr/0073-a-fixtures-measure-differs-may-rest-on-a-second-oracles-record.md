# ADR-0073 — A fixture's `measure_differs` may rest on a second oracle's record: a second kernel run as a black box, its measurements imported by a tool

- Status: accepted (2026-10-28)
- Plan: `three-edge-run-vertex` step 3
- Amends: ADR-0015 (a fixture may declare the oracle's measurements
  wrong, and is then held to its closed forms), whose *Alternatives*
  left "a second oracle" open "should a band appear with no closed form"
- Follows: ADR-0072 (the leaning junction, whose fixtures need it)

## Context

ADR-0015 lets a fixture overrule Open CASCADE's measurements only with
closed forms of the volume, area, centroid and inertia. That covers a
body built of quadrics and their blends. It does not cover a walked
blend, which has no closed form.

`blend/rounded-block-oblique-top-fillet` (plan `three-edge-run-vertex`,
then under `regression/`) fillets a chain round a rounded block's tilted
top. Four of its blend faces are a tilted plane against a corner
cylinder, walked. Every other face of Arris's result matches Open
CASCADE's face area to about 1e-6, the leaning junctions included. The
volume does not: Arris 11.563825145, Open CASCADE 11.564260401, 3.8e-5
relative. The disagreement is local to the four corners:

- At the middle of a lower corner, Arris's contacts lie on the exact
  rolling ball to 3e-10. Open CASCADE's B-spline blend face misses it.
  A ball centre taken along its normal sits up to 7e-4 off the top's
  offset and 1.5e-3 off the corner cylinder's, at radius 0.2. Its
  contact edges pass 3.4e-4 and 2.0e-3 from the exact contacts.
  Tightening `BRepFilletAPI_MakeFillet::SetParams` does not move its
  volume.
- A second commercial kernel, run as a black box on the same operand at
  scale 100, builds the fillet. Its blend faces lie on the rolling ball:
  on a 16 × 16 grid over each, the centre `p ± r·n` is at `r` from both
  supports to 9e-14 at scale 100. Open CASCADE's, measured the same way
  on its own result, miss by up to 0.29 there.
- That kernel's volume, area, centroid and centroidal inertia agree with
  Arris's to 9e-10 or better, each within the fixture's default
  tolerance. Its junction at (0, 0.5) is ADR-0072's circle about `c*`.

So the oracle is wrong at those corners, and there is local evidence
that says so. ADR-0015 asks for that evidence, but it can only take the
numbers from closed forms, and there are none.

## Decision

**1. The record.** A fixture may carry `second_oracle.json` beside
`expected.json`. It holds the recipe hash the run's operand was built
from, and per variant:

- `source`: the second kernel's version and build;
- `run`: the commit of the script that ran it;
- `accuracy`: the mass-property accuracy it was asked for;
- `scale`: the run's length over the fixture's;
- `volume`, `area`, `centroid` and `inertia` (about the centroid, unit
  density, products negated as `expected.json` writes them), in the
  fixture's units.

It is written only by `tools/oracle/second_oracle.py`, never typed by
hand. The tool reads the run's neutral record, divides each value by the
right power of `scale`, and stamps the recipe hash. The second kernel
measures its exact body, before any export: a file it writes carries its
blend faces as fitted splines, and measuring that would compare the fit.

**2. What it changes.** Under `analytic.measure_differs`, a fixture is
held either to closed forms of all four measurements (ADR-0015) or to the
record for every variant. It is never held to both, so that only one
source is held. With a record, the measure stage, the mesh's volume
check and `tools/oracle/compare.py` take the four numbers from it, at
the fixture's tolerances. Everything else in ADR-0015 holds: Open
CASCADE still decides counts, genus, probes and the read-back of Arris's
STEP, and its numbers stay in `expected.json` as the record.

**2a. Open CASCADE's file is held to Open CASCADE's reading of it.**
ADR-0015 holds Arris's reading of Open CASCADE's STEP to Open CASCADE's
in-memory measurements: its fixtures blame Open CASCADE's boolean, and
the file carries that boolean's shape. A walked corner does not survive
the file that way. Open CASCADE's corner faces carry tolerances up to
4.7e-4, and its pcurves lie that far from the 3D edges it writes, while a
STEP file carries the edges and one global uncertainty. Read back,
Open CASCADE itself measures its own block as 11.5639516, 2.7e-5 off its
in-memory 11.5642604 and 1.1e-9 from Arris's reading of the same file.
So under a second oracle's record, the read-back stages (plain and
B-spline) hold Arris's reading of the file to Open CASCADE's reading of
it (`occt_step.py --measured`, `oracle::occt_step_measured`), within
the read body's own tolerance as before. Counts, genus and the checker
are held as for any fixture. This compares the two readers on one file,
which is what the stage is for.

**3. The lint.** A record without `measure_differs` is a problem. So is
a record whose recipe hash is stale, a variant it lacks, or a record
beside closed forms. As with closed forms, at least one of the four
measurements must differ from Open CASCADE's by more than `ANALYTIC_REL`.
A record that agrees with the oracle claims nothing.

**4. The evidence.** The text of `measure_differs` and the fixture's
description name a local measure on which the second kernel is right
and Open CASCADE wrong (here, the ball residual of the blend faces), and
the size of Open CASCADE's error. Agreement in volume is not that
evidence. A second kernel that agrees with Open CASCADE means Arris has
the bug. The record is never a way to choose whichever oracle Arris
happens to match.

**5. Scope.** Only fixtures Arris authors, whose operand both sides
build from the committed recipe. A fetched-tier part, or a committed
real part (CTC-05 AP242), is a separate decision on the same record: M4
`tolerant-tangency` makes it.

**6. Per-face areas** stay in the run's own record. The runner compares
no per-face measure for any oracle, and a face's area alone does not say
which kernel is right.

## Consequences

- `arris_debug::fixtures::Fixture` gains `second_oracle:
  Option<SecondOracle>`, loaded when the file is present.
  `SecondOracle` and `SecondMeasured` are new. `arris-debug` is
  `publish = false`, so no consumer sees the change.
- `measure_target` takes the chain, which now carries its variant
  (`Chain::variant`), and names its source "the second oracle's".
- `occt_step.py --measured` and `oracle::occt_step_measured` (with
  `FileMeasure`) give Open CASCADE's reading of its own file. The
  compare cache keys `second_oracle.json` where a fixture has one.
- `.agents/rules/kernel.md` §Testing and `tests/fixtures/README.md` say
  where the record may stand in for closed forms. `GLOSSARY.md` gains
  *Second oracle*.
- The first user is `blend/rounded-block-oblique-top-fillet`, which
  moves out of `regression/` with its dump.
- Should a future Open CASCADE build the corners on the ball, the lint
  fails the claim, as ADR-0015's does, and the record is removed.

## Alternatives considered

- **Closed forms for the corner.** There are none: the blend of a
  tilted plane against a cylinder is walked (ADR-0062).
- **A mesh integrator over Arris's own faces.** It measures Arris against
  itself, and so cannot decide between the two kernels.
- **Keep the fixture under `regression/`, held to Open CASCADE.** A
  correct result would then fail for good, and no fixed Arris could ever
  pass it.
- **Widen the volume tolerance to 4e-5.** Rejected as in ADR-0015: a
  fixed Arris and one that learned Open CASCADE's corner would both pass.
- **Type the second kernel's numbers into `analytic`.** That would be a
  closed form read off a run, which ADR-0015 forbids. A hand-typed copy
  also loses the version, the run and the scale it came from.
