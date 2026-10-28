# ADR-0015 — A fixture may declare the oracle's measurements wrong, and is then held to its closed forms

- Status: accepted (2026-09-18)
- Plan: `seam-parametrisation-faults` step 1
- Amended by: ADR-0073 (the measurements may come from a second
  oracle's record where no closed form exists)
- Follows: `SEED.md` §9 (the test oracle), ADR-0004 (`counts_differ`'s
  first use)

## Context

`SEED.md` §9 makes Open CASCADE the differential oracle on every fixture,
and `.agents/rules/kernel.md` §Testing holds Arris to its numbers:
"every fixture has an oracle … Arris must match within the fixture's
stated tolerance". The recipe's `analytic` block is the author's closed
form, and the corpus lint holds it to the oracle at 1e-6 — a cross-check
of conventions, on the assumption that where the two disagree, the
closed form was typed wrong. One escape exists already:
`analytic.counts_differ`, for a *convention* Open CASCADE follows and
Arris does not (it imprints a tangent ruling on a face; Arris keeps the
face whole). The oracle's counts stay in `expected.json` as the record,
and the recipe's own are what Arris is held to.

There is no escape for a measurement, because until now there was no
case where the oracle's volume was simply wrong. There is one now.

Take `boolean/cross-cylinders-fuse`: a cylinder of radius `R = 1` and
length `L = 6` along z, fused with the same cylinder along x turned by
`turn` about its own axis. Turning a cylinder about its own axis is the
identity on the solid — it moves only the seam — so every `turn` gives
the same body, the Steinmetz union, exactly centrally symmetric, whose
volume `2πR²L − 16R³/3`, area `4πRL + 4πR² − 16R²`, centroid (the
origin) and inertia tensor are closed forms. Open CASCADE 8.0.1
(`BRepAlgoAPI_Fuse`, measured by `GProp`) agrees with every one of them
to 1e-10 at `turn` 30°, −90°, −89.98°, −90.049°, −90.05° and −90.1°. At
−90° the turned seam runs through a crossing vertex `(0, ±R, 0)`; just
past it, it runs beside one, meeting the other wall a distance
`R·sin δ` away where `δ` is the turn past −90°. There Open CASCADE
returns:

| `turn` | V/E | volume − exact | area − exact | centroid, x |
|---|---|---|---|---|
| −90.01° | 9/15 | +7.31e-4 | +2.19e-3 | 1.88e-6 |
| −90.02° | 9/15 | +1.46e-3 | +4.39e-3 | 3.77e-6 |
| −90.03° | 9/15 | +2.19e-3 | +6.58e-3 | 5.65e-6 |
| −90.04° | 9/15 | +2.92e-3 | +8.77e-3 | 7.53e-6 |
| −90.045° | 9/15 | +3.29e-3 | +9.87e-3 | 8.47e-6 |

against 11/17 at a generic turn. The volume, the area and the
centroid's offset are each **linear in `δ`**, with the same slope in
every row — a sliver the size of the gap between the seam and the
crossing vertex, counted twice or left unclosed — and the error vanishes
by −90.049°. The vertex tolerance is 1e-7 and the gap is 1.7e-4 at the
smallest turn in the table, three orders of magnitude apart, so this is
not a tolerance merge the closed form ought to forgive; the counts say
the same, two vertices gone. A body that is centrally symmetric by
construction cannot have its centroid off the origin, and a body that is
the same set of points at every turn cannot have a volume that depends
on the turn. The oracle is wrong in this band; the closed forms are
exact.

Arris fails in the same band (`Fault::Seam`, or `Unsupported`, plan
`seam-parametrisation-faults`), and the fixture that records it cannot
be written under the current rules: the lint rejects a closed form that
disagrees with the oracle, and the runner would hold a fixed Arris to
Open CASCADE's sliver.

## Decision

**A fixture may say `analytic.measure_differs: "why"`, the measurement
twin of `counts_differ`.** Then:

- `analytic.volume`, `area`, `centroid` and **`inertia`** are all
  required, and they — not the oracle's — are what the corpus runner's
  measure stage, its mesh-volume check and `tools/oracle/compare.py`
  hold Arris to, at the fixture's usual tolerances.
- The oracle still runs on the fixture and its numbers stay in
  `expected.json` as the record of what Open CASCADE builds; counts,
  genus, probes, the STEP read-back and the Euler line are compared
  exactly as for any fixture, and the oracle's own STEP round trip in
  `selftest.py` still has to agree with the oracle.
- The corpus lint skips the 1e-6 comparison of those four fields
  against the oracle, and **fails if none of them differs** in a
  variant, as `counts_differ` fails when the counts agree: the key
  claims a disagreement, and a claim the oracle does not bear out is
  stale.
- `analytic.inertia` is new: the tensor about the centroid at unit
  density, rows of expressions, products of inertia negated as
  `expected.json` carries them. It may be stated on any fixture, and the
  lint then cross-checks it against the oracle like the other closed
  forms; under `measure_differs` it is required.

**Inertia is required, not skipped.** The alternative was to skip the
inertia comparison on such a fixture, since no fixture yet states one.
That leaves nothing checking Arris's inertia on exactly the bodies where
the oracle cannot, and the closed form is not hard where the other three
are known: for the Steinmetz union it is the two cylinders' tensors less
the bicylinder's, `∫x² = ∫z² = 64R⁵/45`, `∫y² = 16R⁵/15` over the
common, and it matches the oracle's generic-turn tensor to 1e-11. A
fixture that cannot state its inertia cannot claim the oracle wrong.

**What a fixture must show to use it.** The key's text, and the
fixture's `description`, carry:

1. a closed form for every field the runner compares, derived rather
   than read off a run — Arris's own output is never the closed form;
2. why the oracle's answer is *wrong* rather than a convention: an
   invariance the oracle breaks (a symmetry, a motion that is the
   identity on the solid), or agreement with the closed form just
   outside the band — the error is a function of something the solid
   does not depend on;
3. the oracle's error named in size, so a reader can see it is not the
   last digits of a tolerance.

A difference that is a convention stays `counts_differ`; a difference
that is within a fixture's tolerance is not a difference; and a closed
form that merely disagrees with the oracle, with no invariance showing
which is right, is a closed form to re-derive, not an oracle to
overrule.

## Consequences

- `arris_debug::fixtures::Analytic` gains `measure_differs:
  Option<String>` and `inertia: Option<[[Num; 3]; 3]>`, both
  `#[serde(default)]`, so every existing `fixture.json` stays valid; a
  caller constructing `Analytic` literally breaks, which pre-1.0 is
  allowed and is named in the commit that makes the change.
- The rule "every fixture has an oracle" stands: the oracle still runs,
  still decides every classification and the genus, every count a
  recipe does not overrule with `counts_differ`, and still reads Arris's
  STEP. `kernel.md` §Testing gains a pointer here for the
  measurement case.
- The corpus now carries a fixture that holds Arris to a number Open
  CASCADE disagrees with. Should a future OCCT pin fix its sliver,
  `selftest.py` reports the changed `expected.json`, and the lint then
  fails the `measure_differs` claim until the key is removed — so the
  escape cannot outlive the fault it records.
- The first user is `boolean/seam-beside-crossing-fuse` (plan
  `seam-parametrisation-faults` step 3), whose counts also differ: Open
  CASCADE's 9/15 merges the seam's hits into the crossing vertices,
  where Arris keeps them, 10/16 as at a generic turn. It carries both keys.

## Alternatives considered

- **Tolerances wide enough to cover the sliver** (`volume_rel` 1e-4 on
  that fixture). Rejected: it holds Arris to *either* answer, so a fixed
  Arris and one that learned Open CASCADE's sliver pass alike, and it
  widens a tolerance to make a case pass, which `/work` forbids.
- **Leave the band out of the corpus**, testing it only by a property
  on volume. Rejected: every failure becomes a fixture
  (`kernel.md` §Testing), and the property alone gives no blessed dump,
  no probes and no provenance check of the case.
- **A second oracle** (another kernel, or a mesh integrator) for such
  fixtures. Rejected as disproportionate for one band: the closed forms
  are exact, a second kernel would need its own environment and pin,
  and it may share the fault. It stays open should a band appear with
  no closed form.
- **Skip inertia under `measure_differs`.** Rejected above: the hole
  lands exactly where the oracle cannot fill it.

## Amendment (2026-09-18, plan `seam-parametrisation-faults` step 3)

The Decision's second point said the oracle's own STEP round trip in
`selftest.py` "still has to agree with the oracle". It cannot, and that
is more of the same evidence: Open CASCADE's body in the band does not
measure the same after its own STEP write and read — at −90.02° its
volume goes from 32.367241 to 32.366737, its centroid from
(3.8e-6, −1.5e-6, −3.8e-7) to (5.7e-6, 3.1e-6, −1.0e-7). A correct body
survives the round trip to 1e-10, as every other fixture's does. Under
`measure_differs` the round trip is therefore held to what the corpus
still takes from the oracle — counts, genus and probes, which do survive
it — and not to the measurements, which the corpus no longer takes from
the oracle at all. Nothing else in the decision changes.
