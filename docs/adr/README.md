# Architecture decision records

One file per decision, numbered, never edited after acceptance — a change of
mind is a new ADR that supersedes the old one. Format: Context, Decision,
Consequences, Alternatives considered. An ADR whose design was informed by a
reference implementation (Open CASCADE, truck, Fornjot) names the module read,
so provenance is auditable (`SEED.md` §8).

The decisions taken at kickoff live in `SEED.md` §9 and are not repeated
here; the first ADR is the first decision taken *after* the seed.

| # | Title | Status |
|---|---|---|
| [0001](0001-nalgebra-types-by-alias.md) | `arris-math` exposes `nalgebra`'s types by alias | accepted |
| [0002](0002-euler-operators-explicit-pcurves-role-provenance.md) | Euler operators over a staging builder; explicit pcurves; provenance rooted in roles | accepted |
| [0003](0003-tessellation-by-cdt-through-pcurves.md) | Tessellation: an own constrained Delaunay triangulation over `robust`, in (u, v), through the pcurves | accepted |
| [0004](0004-booleans-by-general-fuse-over-coedges.md) | Booleans by a General Fuse over coedges: shared paves, faces split in (u, v), the result assembled with kept ids | accepted |
| [0005](0005-ruled-direction-flattened-for-the-triangulation.md) | A ruled direction is flattened before the triangulation | accepted |
| [0006](0006-lumps-in-one-solid.md) | Lumps in one `Solid`: several shells, nested by B1, derived and never stored | accepted |
| [0007](0007-blends-as-rolling-ball-stripes-on-analytic-pairs.md) | Blends are rolling-ball stripes on analytic face pairs, built in closed form and assembled with kept ids | accepted |
| [0008](0008-coaxial-surfaces-of-revolution-meet-through-their-meridians.md) | Coaxial surfaces of revolution meet through their meridians: one arm over the meridian sections, `Points` on the axis, the boolean's quadric guard | accepted |
| [0009](0009-no-name-grammar-a-guaranteed-split-order.md) | Arris owns no name grammar: a consumer names from `Provenance`, and the kernel guarantees the split order | accepted |
| [0010](0010-retain-keeps-slots-sparse.md) | `Model::retain` keeps slots sparse: a live id never moves, a dead one never aliases | accepted |
| [0011](0011-the-tessellation-boundary-is-f64.md) | The tessellation boundary is `f64`: the cast to `f32` is the consumer's, at its own boundary | accepted |
| [0012](0012-the-render-buffer-beside-the-watertight-one.md) | The render buffer beside the watertight one: an optional face-local corner block on the same `TriMesh` | accepted |
| [0013](0013-mesh-formats-live-in-arris-io.md) | Mesh formats live in `arris-io`, which depends on `arris-mesh` | accepted |
| [0014](0014-elliptic-profile-segments-sweep-an-elliptic-cylinder.md) | Elliptic profile segments sweep an elliptic cylinder; a revolve refuses them | accepted |
| [0015](0015-a-fixture-may-declare-the-oracle-wrong.md) | A fixture may declare the oracle's measurements wrong, and is then held to its closed forms | accepted |
| [0016](0016-a-touch-off-every-vertex-is-resolved-through-the-section-curves.md) | A touch off every vertex is resolved through the section curves; the intersector's touch stays a verdict on depth | accepted |
| [0017](0017-the-application-gate-closes-on-the-corpus.md) | The application gate closes on the corpus; the swap is the consumer's | accepted |
| [0018](0018-quadric-sections-are-fitted-nurbs-and-one-meets-result.md) | Quadric sections are fitted NURBS under the faces' tolerance, traced by ruling families in a region; one `Meets` result | accepted |
| [0019](0019-torus-sections-are-traced-in-the-torus-parameter-plane.md) | Torus sections are traced in the torus's parameter plane: a second tracer beside the ruling families, ADR-0018's storage unchanged | accepted |
| [0020](0020-the-direction-after-c3-closure-then-measurement.md) | The direction after C3: closure before breadth, measured then chosen, cycles named not numbered; a consumer blocked on missing API ranks first (amended 2026-09-26) | accepted |
| [0021](0021-a-pcurve-never-runs-through-a-singular-point.md) | A pcurve never runs through a surface's singular point: fitted by projection, split at the apex and the pole, refused beside them | accepted |
| [0022](0022-the-same-within-a-tolerance-is-an-equivalence.md) | The same within a tolerance is an equivalence, decided once per level: points by closure, fits held to the branch, curves by their surfaces, the pinch refused | accepted |
| [0023](0023-the-oracle-round-trip-is-held-to-the-shape-own-tolerance.md) | The oracle's own STEP round trip is held to the shape's own tolerance, and a fixture may declare it lossy (`step_differs`) | accepted |
| [0024](0024-the-measuring-harness.md) | The measuring harness: a cached oracle, a differential over recipes, three property tiers, an in-house timer and fuzzing outside the workspace | accepted |
| [0025](0025-the-step-reader-converts-refuses-and-flattens.md) | The STEP reader: what it converts, what it refuses by name, and what it flattens | accepted |
| [0026](0026-the-real-part-corpus-and-the-refusal-table.md) | The real-part corpus: NIST's parts committed and fetched, the oracle reading healed, and every refusal mapped to the cycle it blocks | accepted |
| [0027](0027-the-changelog-is-written-when-the-change-lands.md) | The changelog is written when the change lands, under `Unreleased`; a release versions it, and a break is announced before it can ship | accepted |
