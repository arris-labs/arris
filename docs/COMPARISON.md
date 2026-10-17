# Arris and Open CASCADE

A comparison for someone choosing between the two. Open CASCADE (OCCT) is
the mature kernel most open-source CAD builds on; Arris is a narrower one.
Arris is not aiming at feature parity with OCCT, and this file does not
claim it has any. It lists what each one does today, where they differ in
approach, and what has and has not been measured.

Maintained with the code: each row names the version of `CHANGELOG.md`
that made its Arris column true, and `/close-cycle` reviews the whole
file against the code. A row that is no longer true is fixed in the
commit that makes it untrue. The Open CASCADE column describes the
version the oracle pins (`cadquery-ocp` 8.0.1 in `tools/oracle/pyproject.toml`),
not the latest.

## Where each one stands

| Area | Open CASCADE | Arris (0.9.0) |
|---|---|---|
| Surfaces | Plane, cylinder, cone, sphere, torus, NURBS, offsets, and more | Plane, cylinder, elliptic cylinder, cone, sphere, torus; NURBS surfaces read, checked, sectioned against analytic surfaces (ADR-0061), and swept/lofted (ADR-0063, ADR-0065) |
| Curves | Lines, circles, ellipses, parabolas, hyperbolas, NURBS | Lines, circles, ellipses, NURBS (for sections, extrema, sweep paths, and lofts) |
| Booleans (cut, fuse, common) | Over all of the above, NURBS included | Over every analytic face in any pose, multi-tool in one call; NURBS against analytic surfaces (ADR-0061); NURBS against NURBS not yet |
| Sections | Full general sections | Exact for conic pairs; traced in NURBS parameter plane and fitted elsewhere; planar section of a body as a sheet |
| Fillet and chamfer | Constant and variable radius, on most face pairs | Constant radius, on chains of edges between planes and cylinders (coaxial pairs included), cone, sphere and torus rims, plus walked stripes on NURBS faces (ADR-0062); no variable radius |
| Shell and offset | Yes, including on NURBS | Offset faces and shell on analytic faces (refuses elliptic cylinder and NURBS for offset and shell) |
| Sweep, loft, pipe | Yes | Sweep along a path (lines, arcs, helix arcs, B-spline curves, ADR-0063); loft across sections by arc-length matching (ruled and smooth polynomial, ADR-0065); pipe not yet |
| Distance, interference, clash | Yes (`BRepExtrema`, `BRepAlgoAPI`) | Yes, with a typed refusal for pairs without a formula |
| Ray fire | Yes | Yes, on every face kind including NURBS |
| Healing | `ShapeFix` and sewing, on by default when reading | Two repairs, on by default when reading and each named on the body: a gap up to 5e-4 of the part's size closed by growing that entity's tolerance, and a face the file walks against all its neighbours turned back. Faces that share no edge are not sewn: refused, naming why |
| Sheet and non-manifold bodies | Yes | Sheet bodies (open shells): from a planar profile, a planar section and STEP surface models, which read and write; measured by area and surface inertia; moved, mirrored and tessellated; booleans, blends and shells refuse one. A surface model whose faces do not meet is refused until sewing lands. Non-manifold is refused |
| STEP | Read and write, IGES too | Read (every solid or surface model of a file is a checked body or a named refusal), write AP214 in a chosen length unit; assemblies both ways |
| Other formats | IGES, STL, glTF, and more | STL, OBJ, a deterministic native format in JSON and bytes |
| Provenance | `BRepTools_History` (modified, generated, deleted) | Every operation returns generated, modified and deleted entities for every entity touched |
| Tolerances | Per shape | Per vertex, edge and face; grown by operations and checked |
| Errors on bad input | Often an exception or an invalid shape | A typed error naming the entities; the checker runs after every operation in debug builds |
| Determinism | Not promised across platforms | Same input, same output and entity ids on every platform; no `HashMap` iteration reaches a decision |
| Language and targets | C++; Python through bindings; WebAssembly builds exist but are not its focus | Pure Rust, `#![forbid(unsafe_code)]`, builds for `wasm32`; Python binding on PyPI |
| Licence | LGPL 2.1 with an exception | MIT or Apache-2.0 |
| Speed | Mature and tuned | The corpus is timed and a 3× regression against a saved run is flagged; no comparison with OCCT has been run |

## Capability matrix

The matrix of what each operation does and refuses for each face kind, held
cell for cell to `crates/arris/tests/capability.txt`:

<!-- matrix: capability -->
| operation | plane | cylinder | elliptic cylinder | cone | sphere | torus | NURBS |
|---|---|---|---|---|---|---|---|
| cut | ok | ok | ok | ok | ok | ok | ok |
| fuse | ok | ok | ok | ok | ok | ok | ok |
| common | ok | ok | ok | ok | ok | ok | unchecked S5 |
| split | ok | ok | ok | ok | ok | ok | unchecked S5 |
| section | ok | ok | ok | ok | ok | ok | ok |
| fillet | ok | ok | refused Unsupported(plane surface, elliptic cylinder surface) | ok | ok | - | unchecked S5 |
| chamfer | ok | ok | ok | ok | ok | - | unchecked S5 |
| offset | ok | ok | refused Degenerate(Offset(NoExactOffset)) | ok | ok | ok | refused Degenerate(Offset(NoExactOffset)) |
| shell | ok | ok | refused Degenerate(Offset(NoExactOffset)) | ok | ok | refused Degenerate(Shell(NoWalls)) | refused Degenerate(Offset(NoExactOffset)) |
| sweep operand | ok | ok | ok | ok | ok | ok | ok |
| loft operand | ok | ok | ok | ok | ok | unchecked S5 | refused Unsupported(NURBS surface, NURBS surface) |
| ray | ok | ok | ok | ok | ok | ok | ok |
| distance | ok | ok | ok | ok | ok | ok | ok |
| STEP round trip | ok | ok | ok | ok | ok | ok | ok |
<!-- /matrix -->


## Where the approach differs

These are design choices, not claims about who is better at a task.

- **Refusals instead of guesses.** A pair Arris cannot do returns
  `OpError::Unsupported` naming both faces, or a `Reason` variant such as
  `TangentContact` or `BlendTooLarge`. OCCT more often builds a shape that
  the checker would reject, or fails without a named cause. The
  refusal list is in `CHANGELOG.md`.
- **Provenance by construction.** Every operation records what each output
  entity came from, rather than reconstructing it from history afterwards.
  OCCT's history records are a fair point of comparison, and Arris's
  advantage is that the record is required, not optional.
- **Tolerance per entity.** Both kernels carry per-shape tolerances. Arris
  holds every section edge to its faces' tolerance, so a grazing contact
  does not widen; `boolean/grazing-ball-bar-cut` shows the same case with
  an edge at 2.8e-6 from Open CASCADE and at 1e-7 from Arris.
- **Determinism.** Entity ids and iteration order are part of the contract,
  so the same model rebuilt gives the same ids. No measurement of OCCT's
  ordering has been run against this.

## Measured against Open CASCADE

Every fixture in `tests/fixtures/` has an expected value computed by
Open CASCADE (`tools/oracle/`), and the corpus runner checks Arris against
it within the fixture's tolerance. Where a closed form exists, the
fixture holds both to it.

Cases where Open CASCADE's own result was measurably off and Arris matched
the closed form:

- `boolean/elliptic-operand-cut`: OCCT's area is 1.5e-7 relative off the
  closed form, and its inertia 9.3e-7; Arris matches both to better than
  1e-10.
- `boolean/pole-slice-beside-seam-cut`: OCCT loses a sliver at a sphere's
  seam, 4.5e-6 relative in volume; Arris finds the second seam crossing.
- `boolean/coaxial-fuse-four-off`: OCCT closes a 4e-7 crescent on one side
  of a shape and not on its mirror image; Arris gives the same answer for
  both.
- `boolean/grazing-ball-bar-cut`: OCCT's section edge is at 2.8e-6, a
  model tolerance of 1e-7 widened after the fact; Arris's is at 1e-7.
- `regression/singular-bore-cut`: OCCT builds a shell pinched to a point;
  Arris refuses with `Reason::NonManifold`, naming the vertex.

These are specific cases, not a failure rate. The tolerance-band survey
(`crates/arris-ops/tests/tolerance_band.rs`) perturbs contacts a quarter tolerance to
sixteen tolerances off flush, coaxial or tangent. When last run (September
2026) Arris failed about a third of the booleans it generated. Open CASCADE has not been run over the
same band, so no comparison of rates exists. Do not read the cases above
as "more robust than Open CASCADE".

Cases Open CASCADE builds and Arris does not yet (each is a refusal today):
NURBS faces meeting another NURBS face in a boolean (NURBS against analytic
lands in C10), variable-radius blends, sewing the free faces of a STEP file,
and offset/shell on elliptic cylinder or NURBS surfaces. The fixtures that
hold these are in `tests/fixtures/` and `docs/ROADMAP.md`.

## Reproducing the comparison

```sh
uv sync --project tools/oracle                     # once; downloads the OCCT wheel
uv run --project tools/oracle tools/oracle/selftest.py
cargo nextest run --workspace                      # the corpus runs Arris against the oracle
```

`tools/oracle/README.md` lists each script. Oracle values are cached under
`target/oracle-cache/`; `ARRIS_ORACLE_CACHE=off` runs them afresh.

## What is not claimed

- No speed claim. The corpus is timed to catch regressions, not to rank
  kernels, and neither kernel's speed has been measured against the other.
- No breadth parity with OCCT, and none is a goal.
- No robustness rate. The fixtures are a corpus, not a sample of the
  shapes users model. "Checked against Open CASCADE on every shape it
  ships" is the claim; "robust" is not.
- No production-readiness claim. The API breaks between minor versions
  until 1.0.
- No claim that Arris "solves topological naming": provenance is what a
  naming scheme needs, and the scheme is the application's (ADR-0009).
- No WebAssembly size compared with another kernel: never measured against
  opencascade.js or others.
- No claim about the share of mechanical parts covered by analytic kinds:
  no source exists for such a percentage.
