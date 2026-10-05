# The Open CASCADE oracle

Ground truth for the fixture corpus (`SEED.md` §7, `docs/ROADMAP.md`
§Fixtures). Open CASCADE is **run** here through the `cadquery-ocp` wheels;
no crate links it. Every number in a `tests/fixtures/**/expected.json` was
written by `expected.py` in this directory, never by hand.

## Environment

A [`uv`](https://docs.astral.sh/uv/) project pinned to Python 3.12 and
`cadquery-ocp==8.0.1.*` (`pyproject.toml`, `uv.lock`). The venv is created
on first use and lives in `tools/oracle/.venv` (gitignored):

```sh
uv sync --project tools/oracle          # once; downloads the OCCT wheel
uv run --project tools/oracle tools/oracle/selftest.py
```

Running a script outside the environment fails on the first line with the
command above, never with a silent skip. The OCCT version is recorded in
every `expected.json`; a pin change that moves a number is a `fixtures:`
commit that says so (`.agents/rules/git.md`).

## Scripts

| Script | Does |
|---|---|
| `expected.py [--own] <fixture-dir>...` | Builds each recipe (every variant), measures it, writes `expected.json`, prints one summary line per result; for a geometry fixture, evaluates, projects and intersects instead; for a part fixture, reads its STEP file twice — healed, as Open CASCADE's reader heals by default, for the measures, and with every `FixShape` switch off for `occt_heals` and `unhealed_counts` (ADR-0026 §3) — and records every solid with its `#id` (`fixture.compute_part`), then builds each case of its battery over the `step` operand and records what `measure` gives with its `own` block, or `{"refused": why}` where Open CASCADE raises (`fixture.compute_battery`; `tests/fixtures/README.md` §The battery). A recipe it cannot build prints `<dir>: ERROR <why>` on one line of stderr and the rest are still built, exit 1: `arris_debug::oracle::expected_batch` hands it many scratch fixtures at once — the differential's whole draw in one process — and reads each directory's answer apart. `--own` adds each solid result's `own` block (below), which only the differential asks for, and a part's battery results carry; a solid fixture of the corpus is never written with it. Every solid result also records `nurbs_counts`, the counts of the result passed through `BRepBuilderAPI_NurbsConvert` (conversion can add seams), or `nurbs_fails` with Open CASCADE's reason where the conversion fails: what Arris's reading of `occt_step.py --nurbs` is held to |
| `compare.py <fixture-dir> <file.step> [--variant NAME]` | Reads a STEP file (Arris's output), measures it, compares against `expected.json` within the fixture's tolerances, prints a table; exit 1 on mismatch, 2 on a stale `expected.json` or an environment error. Solid fixtures only: a geometry fixture is compared by `crates/arris-geom/tests/oracle.rs`. `arris_debug::oracle::compare` is the Rust seam to it, and the corpus runner (`arris_debug::corpus::run`) calls it on every fixture |
| `occt_step.py <fixture-dir> <out.step> [--variant NAME] [--nurbs]` | Builds the recipe (one variant) and writes Open CASCADE's own STEP of the result — with `--nurbs`, passed through `BRepBuilderAPI_NurbsConvert` first — for Arris's STEP reader to read back (ADR-0025). Exit 2 with `occt_step: ERROR <why>` on stderr for a recipe it cannot build. `arris_debug::oracle::occt_step` is the Rust seam to it, cached like the others, and the corpus runner's read-back stage calls it on every fixture that does not say `analytic.step_differs` |
| `occt_assembly.py <fixture-a> <fixture-b> <out.step>` | Builds both recipes, makes a named XCAF assembly of the first placed once and a sub-assembly placed once that holds the second placed twice (each a turn and a shift), colours the parts and one face, writes it with `STEPCAFControl_Writer` and prints as JSON each placed instance's volume and centroid, the expected product tree (names, placements, colours) and the coloured face: what Arris's reader of an assembly is held to (ADR-0025 §5, ADR-0033). `arris_debug::oracle::occt_assembly` is the Rust seam, cached under both `fixture.json`s |
| `occt_read_assembly.py <in.step>` | Reads a STEP file with `STEPCAFControl_Reader` (names and colours on) and prints as JSON the XCAF document's free shapes as occurrences (name, placement in the parent, part colour, children) and each leaf part's volume and centroid at its composed placement: what Arris's writer of an assembly is held to (ADR-0033). `arris_debug::oracle::occt_read_assembly` is the Rust seam, cached under the file's text |
| `excerpt.py <file.stp> <solid-id> <out.stp> [description]` | One solid of a STEP file alone: the entity and everything it refers to, a new `ADVANCED_BREP_SHAPE_REPRESENTATION` over its representation's context, and a new product for Open CASCADE's reader to transfer from. The file's own product structure, placements and PMI are left out. What a fetched part's failure is committed as under `regression/`, where its licence allows (ADR-0026 §2) |
| `occt_fillet_edges.py <file.stp> <solid-id> <radius> <probe> <points.json>` | One solid of a STEP file, each edge named by a point of the JSON list (in the model's own frame, within `probe` of that edge alone) filleted alone at `radius`; prints one verdict per point as JSON: `builds` (done, passes `BRepCheck_Analyzer`), `invalid`, `refuses` with Open CASCADE's reason, or `no-edge`. A solid placed more than once is the placement nearest the points. What the run-over census holds Arris's `BlendTooLarge` against. `arris_debug::oracle::fillet_edges` is the Rust seam, cached under the file and the request |
| `mesh.py <file.stl>` | Reads an STL file (Arris's output, ASCII or binary) through Open CASCADE's `RWStl` and prints its triangle count, area and signed volume as JSON — an independent reader of the bytes, not a fixture comparison; exit 2 on a read error. `arris_debug::oracle::compare_stl` is the Rust seam to it |
| `selftest.py [fixture-dir...]` | `tests/fixtures/expr-cases.json`'s expression grammar cases (the same ones `arris_debug::fixtures::expr`'s own test evaluates); inline smoke recipes covering every op and the geometry kind against closed forms; then for each fixture: a fresh `expected` must equal the committed one, and for a solid OCCT's own STEP of the result must compare clean — on its counts, genus and probes only where the recipe says `analytic.measure_differs` (ADR-0015) |

## The cache

The Rust seam (`arris_debug::oracle`) keeps every answer the oracle
settles — a `compare.py` table that said `MATCH`, a scratch fixture's
`expected.json`, a `mesh.py` reading, an `occt_step.py` file — in `target/oracle-cache/`, one file
per key (ADR-0024). The key is sha256 over the script's name, the bytes
of every file it reads (the STEP or STL text, `fixture.json`,
`expected.json`), the variant, and a digest of the oracle itself: every
`*.py` in this directory outside `.venv/` and `__pycache__/`, with
`pyproject.toml` and `uv.lock`. Editing any script here therefore misses
every entry, and nothing needs clearing by hand. A mismatch or an
environment error is never kept. `ARRIS_ORACLE_CACHE=off` bypasses the
cache; `ci.yml` sets it, so CI always runs the oracle.

## Package

- `oracle/recipe.py` — the recipe interpreter: `box`, `cylinder`, `profile`
  (lines, three-point arcs, circles, holes), `extrude`, `revolve`,
  `transform`, `mirror`, `fuse`, `common`, `cut`, `fillet`, `chamfer` and `offset`
  (`BRepFilletAPI_MakeFillet`, `MakeChamfer` with one distance, each edge
  the nearest to a recipe point by `BRepExtrema`, which must be
  the only edge within the fixture's `probe`; `offset` is
  `BRepOffset_MakeOffset` with offset 0 and the intersection join, each
  moved face given its distance by `SetOffsetOnFace` and named by a point
  the same way, and the bare closed shell it returns beside concave
  neighbours made the solid it bounds, ADR-0048 §4), and `step` (a solid of a
  STEP file beside the recipe, by `step.solids`, its SHA-256 checked),
  chained by step name; `params`
  with string expressions and `variants` overriding them. The grammar is
  the module docstring and `tests/fixtures/README.md`. `fixture.load_fixture`
  records the recipe's directory under `recipe.DIR_KEY`, outside the hash,
  which is where a `step` operand's file is found.
- `oracle/measure.py` — volume, area, centroid and the inertia tensor
  (`GProp`: its fixed-order integration, but the adaptive overloads to
  `SPLINE_EPS` for a shape trimmed by a many-span B-spline pcurve — under
  a quadric section walked or fitted, or under an exact conic on a cone,
  a sphere or a torus — over which the fixed order is 1e-6 off, and
  `VolumePropertiesGK` for a shape with a surface-of-extrusion face,
  whose area is the Green integral of its basis arc length instead, and
  for a part's spline-bounded solid, over which the plain adaptive one is
  1e-6 off in the inertia tensor — NIST's FTC-11, a solid of revolution
  whose symmetry it breaks),
  counts by unique
  sub-shape (a seam edge once, an edge `BRep_Tool::Degenerated` names not
  at all), loops, shells, solids — of the boundary only: an edge oriented
  INTERNAL or EXTERNAL, a wire of nothing else and a vertex on nothing
  else are left out, as a STEP round trip leaves them — the Euler
  characteristic `V − E + 2F − L` and the genus it implies, in/out/on
  classification of probe points (`BRepClass3d`), and the comparison with
  its tolerances.
- `oracle/geometry.py` — the geometry kind: `Geom_Plane`,
  `Geom_CylindricalSurface`, `Geom_ConicalSurface`,
  `Geom_SphericalSurface`, `Geom_ToroidalSurface`, `Geom_Line`,
  `Geom_Circle`, `Geom_Ellipse` and `Geom_BSplineCurve` (a `nurbs`
  curve: flat knots, Cartesian control points, weights) from named
  specs; `D2` at every
  parameter; `GeomAPI_ProjectPointOnSurf` / `OnCurve` for every point;
  `IntAna_QuadQuadGeo` for surface pairs (`unsolved` where it reports
  `NoGeometricSolution`, with `GeomAPI_IntSS`'s walked lines sampled and
  polished onto both surfaces by Newton steps as `section` curves;
  `point` where it reports `IntAna_Point`, a parabola's and a
  hyperbola's branches sampled at their own parameters, through the
  overload each kind pair has) and `IntAna_IntConicQuad` for a
  curve against a surface — `IntAna_IntLinTorus` for a line against a
  torus, `GeomAPI_IntCS` for a `nurbs` curve — hits deduplicated within
  `Precision::Confusion` and dropped (counted) when off either operand;
  and for two curves with a `nurbs` one among them, which Open CASCADE
  has no intersector for, `GeomAPI_ExtremaCurveCurve` span by span of
  the B-spline, a line within `LINE_REACH` of its origin, every extremum
  within `Precision::Confusion` a hit.
- `oracle/step.py` — STEP AP214 write and read, with OCCT's transfer
  banner silenced; and `solids`, every solid a file's reading transfers,
  each with the `#id` of the solid entity behind it. The reader heals on
  transfer as Open CASCADE does by default (`FixShape`, ADR-0026 §3): its
  unhealed reading of a valid file with a seamless cylinder band is no
  solid. The model this build reads keeps no `#id` (`Number` and
  `IdentLabel` answer 0) and drops instances nothing refers to, so the
  `#id` is found by content: `EntityFromShapeResult` of the solid
  unplaced (placed, it answers the assembly occurrence), then the face
  count and first vertex of its outer shell, matched against the same read
  from the file's own text (`instances`, a scan of the DATA sections that
  skips strings and comments), which must name one solid alone.
- `oracle/mesh.py` — STL read through `RWStl`, and its triangle count,
  area and signed volume by the divergence theorem.
- `oracle/fixture.py` — fixture directories, `expected.json` layout,
  the recipe hash.

## `expected.json`

```json
{
  "occt": "8.0.1.0.0",
  "recipe_sha256": "<hash of params, variants, steps, result, probes>",
  "results": {
    "default": {
      "degenerate": false,
      "counts": {"vertices": 10, "edges": 15, "faces": 7, "loops": 9, "shells": 1, "solids": 1},
      "volume": 11497.345175425633, "area": 3950.796447372311, "centroid": [20.0, 15.0, 5.0],
      "inertia": [[993800.5904969159, -1.3969838619232178e-09, -4.656612873077393e-10],
                  [-1.3969838619232178e-09, 1693800.5904969191, -2.3283064365386963e-10],
                  [-4.656612873077393e-10, -2.3283064365386963e-10, 2495978.761403409]],
      "euler_characteristic": 0, "genus": 1,
      "probes": [{"label": "inside", "point": [5, 5, 5], "class": "in"}]
    }
  }
}
```

`inertia` is OCCT's `GProp_GProps::MatrixOfInertia` of the volume
properties: the 3×3 tensor about the centre of mass at unit density, in
the physical convention — the diagonal holds the moments of inertia and
the off-diagonal the *negated* products — which is what
`arris_ops::measure::MassProperties` states and the runner's `measure`
stage compares within the fixture's `inertia_rel`.

The hash covers only what the oracle evaluates, so editing a fixture's
`analytic` or description does not stale it; editing a step does, and
`compare.py` refuses to run until `expected.py` is rerun. A degenerate
result (no solid — the common of two flush boxes) has counts only.

With `--own`, each solid result also carries what the shape declares of
itself (`oracle.measure.own_measures`): `"own": {"tolerance", "edge_length",
"reach", "removable_vertices"}`. These are its largest vertex tolerance, the
total length of its edges, the farthest corner of its bounding box from the
centroid, and the count of vertices that only split an edge between the same
two faces into two. The differential holds Arris's result to the
first-order bound of a boundary moved within both shapes' tolerances, which
is the bound `within_own_tolerance` holds the oracle's STEP round trip to.
It also compares counts net of removable vertices (ADR-0024, amendment of
step 5b).

A geometry fixture's `expected.json` has `"kind": "geometry"` and, instead
of `results`, one entry per recipe sample and pair:

```json
{
  "occt": "8.0.1.0.0", "recipe_sha256": "…", "kind": "geometry",
  "samples": [
    {"of": "wall",
     "evaluations": [{"at": [0.0, 0.0], "point": [2.0, 0.0, 0.0], "du": […], "dv": […], "duu": […], "duv": […], "dvv": […]}],
     "projections": [{"point": [0.0, -5.0, 3.0], "uv": [4.71238898038469, 3.0], "nearest": [0.0, -2.0, 3.0], "distance": 3.0}]},
    {"of": "ring",
     "evaluations": [{"at": 3.141592653589793, "point": […], "d1": […], "d2": […]}],
     "projections": [{"point": [-4.0, 0.0, 0.0], "t": 3.141592653589793, "nearest": […], "distance": 1.0}]}
  ],
  "pairs": [
    {"a": "cap", "b": "wall", "type": "circle", "curves": [{"type": "circle", "points": [[…], …]}]},
    {"a": "ring", "b": "wall", "type": "points", "hits": [{"point": […], "t": 0.8410686705679302}, …]}
  ]
}
```

A surface pair's `type` is `empty`, `coincident`, `point` (with its
`points`: a plane tangent to a sphere, two spheres touching, a plane
through a cone's apex), `line`, `circle`, `ellipse` or `unsolved` (no
curves: `IntAna_NoGeometricSolution`), with every result curve sampled
(five points along a line, eight around a closed curve); a curve–surface
pair's is `coincident` or `points`, hits ascending by the conic
parameter, plus `dropped` when the intersector reported a point that
lies on neither operand; a curve–curve pair's is `points`, each hit
carrying the second curve's parameter as `tb` beside the first's `t`. `IntAna_QuadQuadGeo` has one overload per
unordered kind pair, in its own operand order and with its own tolerance
signature (an angle and a distance, a distance, or none);
`geometry.py` swaps a pair into that order, since the result does not
depend on it.

## Conventions the interpreter mirrors

Open CASCADE's, which Arris follows too (`docs/DATA-MODEL.md`
§Conventions): a full revolve and a cylinder have one seam edge; a fuse of
flush boxes drops the shared face and keeps coplanar neighbours unmerged;
a common with no volume is an empty result. A fixture's `analytic` block
is the cross-check that catches a convention mismatch on either side.
