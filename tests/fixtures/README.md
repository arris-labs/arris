# The fixture corpus

The unit of acceptance (`docs/ROADMAP.md` §Fixtures). One directory per
fixture, `<area>/<slug>/`, of one of four kinds — a **solid** (the
default: a recipe built and measured), **geometry** (`"kind": "geometry"`,
under `geom/`: analytic surfaces and curves evaluated, projected onto and
intersected; §Geometry fixtures below) or a **part** (`"kind": "part"`,
under `real/`: a STEP file Arris did not write, read; §Part fixtures
below) or a **query** (`"kind": "query"`, under `query/`: questions asked
of a solid; §Query fixtures below) — holding:

| File | Written by | Holds |
|---|---|---|
| `fixture.json` | a person or the agent | the **recipe**: operands and operations both sides evaluate, probe points, tolerances, and the closed-form `analytic` values |
| `expected.json` | `tools/oracle/expected.py`, never by hand | the **oracle's answer** per variant: volume, area, centroid, the inertia tensor about the centroid, counts, Euler characteristic and genus, probe classifications, plus the OCCT version and the recipe hash |
| `dump.txt` | Arris, through the corpus runner under `ARRIS_BLESS=1`, once the fixture passes | the text dump of the result, the regression guard for ids and provenance, committed once the fixture passes and required by the lint of every fixture the runner compares; `dump.<variant>.txt` for a variant other than `default` |

The Rust reading of both files is `arris_debug::fixtures`; the Python one
is `tools/oracle/oracle/recipe.py` and `fixture.py`. The corpus lint
(`crates/arris/tests/corpus_lint.rs`, run by `cargo test`) checks every
directory: both files present and parseable, `expected.json` not stale, the
Euler line zero, every `analytic` value matching the oracle to 1e-6
relative, counts and probe expectations exactly — and every solid under
`primitive/`, `build/`, `transform/`, `boolean/`, `sweep/`,
`loft/`, `provenance/`, `blend/`, `offset/`, `shell/`, `split/` or `sheet/` that the
runner compares (the oracle built a solid, the recipe expects no refusal)
carrying its committed dump per variant, which a fixture only has once it
passed and was blessed. An `#[ignore]`d fixture in those areas therefore
fails the lint: they hold zero ignored fixtures by test, and CI runs the
corpus's ignored tests as well, all but the `regression_*` ones. A
fixture under `regression/` — a failure waiting for its fix, below — needs
no dump and must have none: one with a dump passes, belongs in its area,
and fails the lint until it moves. The oracle's own self-test
(`uv run --project tools/oracle tools/oracle/selftest.py`) reproduces every
committed `expected.json` and round-trips each result through STEP.

The corpus **runner** (`arris_debug::corpus::run(dir, variant)`, one
`#[test]` per fixture in `crates/arris/tests/corpus.rs`) is the fixture
test itself: it builds the recipe in Arris, runs the checker at `Full`
(nothing violated, nothing undecided but the face pairs with a NURBS
face in them, which the surface intersector has no form for — an
operand read from a file of B-splines), compares counts — `solids` as the
result's lumps, `arris_check::lumps` — and genus against
`expected.json`, writes STEP under `target/inspect/`, parses it back as
Part 21 (`arris_io::step::part21`, every instance the writer defined kept
once) and has the oracle read it back (`compare.py`), has the oracle write
its own STEP of the recipe (`occt_step.py`) and reads that back through
Arris's reader (`corpus::read_back_stage`, ADR-0025): every solid read,
the checker at `Full` clean, the oracle's own counts and genus — the file
is Open CASCADE's topology, so `counts_differ` does not apply — and the
oracle's own volume, area, centroid and inertia within the fixture's
tolerances widened to the read body's own (ADR-0023); then the same again
for the result converted to B-splines (`occt_step.py --nurbs`,
`corpus::read_back_nurbs_stage`), held to `expected.json`'s `nurbs_counts`
— the converted shape's, since conversion can add seams — with the rows
the checker cannot decide on a NURBS face (S5's and B1's face pairs with
one in them, B1's nesting of a shell no ray is cast from) left
unchecked and nothing else, and skipped where `expected.json` records
`nurbs_fails`, Open CASCADE's own conversion failing; measures it over the B-Rep
(`ops::measure::mass_properties`) and holds its volume, area, centroid
and inertia tensor to the oracle's within `volume_rel`, `area_rel`,
`centroid_abs` and `inertia_rel`, tessellates the result at `mesh_chord` and
holds the mesh closed with a positive signed volume within
`mesh_volume_rel` of the oracle's, classifies every probe point against
the result (`arris_check::classify::classify_point`) and holds it to the
oracle's class *exactly* — both sides have their own tolerance for "on",
the fixture's `probe` for the oracle and the entities' own for Arris, and
a probe is placed so the two agree, so a disagreement is a finding and
never something a band is widened to cover — asserts every step's
provenance accounting, and diffs the dump against `dump.txt`. A result
the oracle recorded no solid for (`degenerate` in `expected.json`) must
fail with `OpError::Degenerate` at its result step, and one whose recipe
says `analytic.expect_error` must fail with that typed refusal; either
ends the run there, nothing later compared, no `dump.txt`. Every op of
the recipe grammar has its operation; a fixture that does not pass yet —
a failure shrunk to one — lives under `regression/<slug>/`, outside the
lint's areas, with its test `regression_<slug>` `#[ignore = "why"]`d in
`corpus.rs`; it fails at the stage that differs under `--include-ignored`,
so the day it passes the test says so, and the commit that fixes it moves
the directory into its area, blesses its dump and renames the test
`<area>_<slug>`. `ARRIS_BLESS=1 cargo test -p arris --test corpus
<name>` writes the dump instead of diffing it; commit the file as part
of the step that made the fixture pass, and a later change to it is a
`fixtures:` commit that says why the ids or the geometry moved.

## `fixture.json`

```json
{
  "description": "what the fixture is for",
  "params":   {"R": 35, "r": 3, "t": 10},
  "variants": {"thicker": {"t": 12}},
  "steps": [
    {"name": "plate", "op": "box", "min": [0, 0, 0], "max": [100, 100, "t"]},
    {"name": "h0", "op": "cylinder", "base": ["50 + R * cos(radians(0))", 50, -1],
     "axis": [0, 0, 1], "radius": "r", "height": "t + 2"},
    {"name": "result", "op": "cut", "target": "plate", "tool": "h0"}
  ],
  "result": "result",
  "probes": [{"label": "inside", "point": [50, 50, 5], "expect": "in"}],
  "tolerances": {"volume_rel": 1e-9, "area_rel": 1e-9, "centroid_abs": 1e-7, "probe": 1e-7},
  "analytic": {
    "volume": "100 * 100 * t - pi * r * r * t",
    "area": "...",
    "centroid": [50, 50, 5],
    "counts": {"vertices": 10, "edges": 15, "faces": 7, "loops": 9, "shells": 1},
    "genus": 1
  }
}
```

- **Numbers** anywhere in `steps`, `probes` and `analytic` may be a JSON
  number or a string expression over `params`: `+ - * / ^`, parentheses,
  `pi`, and `sin cos tan atan sqrt radians degrees abs`. `^` is power,
  binding tighter than `* /` and a unary minus (`d^2 / 4` is `d²/4`,
  `-2 ^ 2` is `−4`) and right-associative (`2 ^ 3 ^ 2` is `512`); `**` is
  rejected, not read as power, on both sides. Both sides evaluate the same grammar: proven by
  `expr-cases.json` (in this directory), which
  `arris_debug::fixtures::expr`'s own test and the oracle's
  `selftest.py` both evaluate.
- **`variants`** override params; every variant gets its own result in
  `expected.json`, and `default` (the base params) always exists. A probe's
  `expect` is checked in every variant, so leave it out (`null`) when the
  answer changes between variants. The `provenance/` fixtures are recipes
  whose variants are the point: the same recipe under several parameter
  sets, and `crates/arris/tests/provenance.rs` holds the records equal
  across them — the bolt pattern's chains, `consumer-rebuild`'s ending at
  the consumer's own keys (ADR-0028), and for the `split-*` fixtures the
  split order of every origin's pieces (ADR-0009).
- **Steps** are chained by `name`; `result` names the fixture's result.
  Angles are degrees; directions and plane axes are normalised by the
  interpreter.

| `op` | Fields |
|---|---|
| `box` | `min`, `max` |
| `cylinder` | `base` (centre of the base cap), `axis`, `radius`, `height` |
| `profile` | `plane` `{origin, x, y}`; `outer` and `holes` as loops: `{"circle": {"center": [u, v], "radius": r}}`, `{"ellipse": {"center": [u, v], "major": [du, dv], "minor_radius": b}}` or `{"start": [u, v], "segments": [{"line_to": [u, v]}, {"arc_to": [u, v], "via": [u, v]}, {"ellipse_to": [u, v], "center": [u, v], "major": [du, dv], "minor_radius": b, "ccw": true}, …]}` (the last segment ends at `start`; loop orientation is irrelevant) |
| `planar_face` | `profile` — a one-face sheet; a result that is a sheet is measured by its surface properties (area, centroid, inertia at unit area density), recorded with `"sheet": true`, no volume, probes or Euler line, held to the `sheet/` area's dumps, written to STEP and read back by the oracle and by Arris |
| `extrude` | `profile`, `direction`, `length` |
| `revolve` | `profile`, `axis` `{origin, direction}`, `angle_deg` |
| `sweep` | `profile`, `path` `{start, segments}` (each segment `{line_to}`, `{arc_to, via}`, `{helix: {axis, pitch, angle_deg}}` from the current point, or `{nurbs: {degree, knots, control_points, weights}}`), `law`: `"rotation_minimizing"` or `{"binormal": [x, y, z]}`. The oracle is `BRepOffsetAPI_MakePipeShell` (corrected Frenet, or the fixed binormal), the profile added where it stands; it takes wires, so a face's outer wire and its holes' are swept apart and sewn with the two caps, and a helix is a 2D line on a cylinder built to 3D. Arris builds it with `ops::sweep` (ADR-0063) |
| `loft` | `sections` (profile step names, two or more, the same number of loops), `kind`: `"ruled"` or `"smooth"`, optional `closed` (the last section joins the first, no caps). The oracle is `BRepOffsetAPI_ThruSections` per loop (outer wire, then each hole's), compatibility checking on (off, it lofts only the first edge of a section with fewer edges), sewn with the first and last profile's faces as caps; a closed loft repeats the first section. Open CASCADE re-starts each wire at the vertex nearest the previous section's start, so a fixture's sections start where it keeps them. Arris builds both kinds with `ops::loft` (ADR-0065) |
| `transform` | `of`, optional `translate`, optional `rotate` `{axis, origin, angle_deg}`; rotation first |
| `mirror` | `of`, `plane` `{origin, normal}`: the reflection in it (`ops::mirror`; `gp_Trsf::SetMirror` in the oracle) |
| `fuse`, `common` | `a`, `b`; a `fuse` may instead take `bodies`, a list of two or more step names (one General Fuse, `arris_ops::fuse_many`; ADR-0050) |
| `cut` | `target`, `tool`; or `tools`, a list of step names cut away in one call (`arris_ops::cut_many`; ADR-0050) |
| `split` | `of`, `plane` `{origin, normal, x?}` (the frame `arris_ops::split` takes; `x` fixes the caps' (u, v)), and `side`, an expression positive where the step's own name is the positive side (the default) and negative where it is the other: a variant names a side by overriding a param `side`. The step's name is that side; `<name>.positive` and `<name>.negative` are both, for a later step. The oracle is `BRepAlgoAPI_Splitter` with a planar face past the body's box, its solids sorted to a side by their centroid's signed distance; `expected.json` records beside each result the same side as the common (positive) or cut (negative) of the body with a half-space box, under `half_space`, so a disagreement of the splitter with Open CASCADE's own booleans is on record. A side with no solid is a result with no volume (`analytic.degenerate`), which is how a plane that misses is a refusal fixture |
| `pattern` | `of`, `step` (`[x, y, z]`), `count`, and for a grid `step2` and `count2`: the copies of `of` translated by `i * step` (plus `j * step2`), a list a `tools` or `bodies` field takes by the pattern's name. `count`s are plain integers; each copy is a `transform` step named `name[i]` or `name[i,j]`, expanded when the recipe is read (`fixtures::expand_patterns`, `recipe.py`'s `expand_patterns`) and not part of the recipe hash |
| `fillet` | `of`, `edges` (a list of points, one on each edge to blend), `radius` |
| `chamfer` | `of`, `edges` (as a `fillet`'s), `distance` (one, measured on both faces from the edge) |
| `offset` | `of`, `faces` (a point on each face to move, within `probe` of that face alone — Arris takes the face the point classifies `On`, so a point on an edge or a seam names none; every face tangent to a named one moves with it, ADR-0048 §6), `distance` (signed along the outward normal, positive adds material; ADR-0048) |
| `shell` | `of`, `openings` (a point on each face to open, as an `offset`'s faces; none or absent for a closed void), `thickness` (positive), `side` (`"inward"`: the body's faces kept, the cavity inside them; `"outward"`: the body's faces the cavity, a skin outside them; ADR-0049) |
| `polyhedron` | `points` (a list of `[x, y, z]`), `faces` (each a list of loops, each a list of indices into `points`: the outer loop counter-clockwise seen from outside the solid, a hole's clockwise), `namespace` (Arris's key space) |
| `step` | `file` (beside `fixture.json`), `sha256` (of the file), `id` (the `#id` of its `MANIFOLD_SOLID_BREP` or `BREP_WITH_VOIDS`), `near` (a point; only where the file places that solid more than once) |

- **A `fillet`'s or a `chamfer`'s edges are named by a point each**, so a selection
  survives a transform and a second blend, which a role does not: Arris
  takes the edge `classify_point` answers `On(Edge)` for, the oracle the
  nearest edge by `BRepExtrema`, and both refuse a point that is within
  `probe` of two edges or on none — a vertex, a face, the inside or the
  outside (`CorpusError::EdgePoint`).
- **A `polyhedron` is a consumer's own topology** (ADR-0028): a solid
  of planar faces, each on the plane of its outer loop. Arris describes
  it to `Builder::assemble` — a vertex per point, a line edge per pair of
  points a loop joins, from the lower index to the higher, a planar face
  per face with the lines as pcurves — and finishes it through
  `ops::build`, keyed by `arris_debug::polyhedron`'s convention: point
  `i` is key `i`, face `j` key `j`, the edge between points `a < b` key
  `a << 32 | b`, the shell and the body `0`, all in `namespace`. The
  oracle makes each loop a polygon wire, each face a planar face over its
  wires, sews them and makes the shell a solid. The `build/` fixtures
  are its corpus.
- **A `step` operand is a solid read from a file** (ADR-0026): Arris
  through `arris_io::step::read`, the oracle through Open CASCADE's
  reader, which heals on transfer by default (ADR-0026 §3). Both refuse a
  file whose SHA-256 is not the recipe's, and since the hash is in the
  recipe, a changed file stales `expected.json` and misses the oracle's
  cache. The solid is named by its file entity, not by either reader's
  order of instances. Arris keeps it as the reader numbers it; Open
  CASCADE's model keeps no `#id`, so `oracle/step.py` finds it by the
  face count and first vertex of its outer shell. Where an assembly
  places the solid more than once, the placement whose centroid is
  nearest `near` is taken, and a tie within `probe` is refused. A
  refusal of the solid by Arris's reader is the step's error
  (`CorpusError::Refused`); a file that is missing, does not match its
  hash, or has no such solid is `CorpusError::StepFile`.
  `boolean/step-operand-cut` cuts Arris's own STEP of
  `boolean/through-hole`'s result. The runner holds a recipe with a
  `step` operand as a part's reading and battery hold it. Its measures
  are within the fixture's tolerances widened to the result's own
  (`corpus::within_own_tolerance`, ADR-0023), since a real part carries
  the file's tolerances, which a primitive's defaults do not size. The
  B-spline read-back (`read_back_nurbs_stage`) is skipped: the
  conversion of a whole real part costs minutes to read, which
  measures the reader on a file of that size and not the operation
  (ADR-0026, amendment of step 7).
- **An ellipse** (ADR-0014) is its centre, `major` — from the centre to a
  major vertex, so its length is the major radius and its direction the
  axis — and `minor_radius`; an `ellipse_to` segment runs from the
  previous point to `ellipse_to`, both on the ellipse, turning
  counter-clockwise about the plane's normal when `ccw`. A `minor_radius`
  longer than `major` names the same point set with the axes swapped, on
  both sides (`gp_Elips` insists on `a ≥ b`); radii within the linear
  tolerance of each other are a circle in Arris. The oracle's area of a
  face on the `Geom_SurfaceOfLinearExtrusion` an ellipse extrudes into
  is the contour integral of its basis arc's length
  (`GCPnts_AbscissaPoint`, to 1e-13) against `dv` around the face's own
  (u, v) loops — Green's theorem over an area element that is the basis
  curve's speed alone — since Open CASCADE's fixed-order integration is
  2e-5 off there and its adaptive one worse. An untrimmed such face is a
  rectangle and the integral is its arc length times its height, as it
  always was; a boolean that trims one with planes parallel to the axis
  (rulings) or perpendicular to it (arcs) is exact too, and a curved
  pcurve there — a section of an elliptic prism and a revolved body — is
  integrated by composite Gauss–Legendre over the pcurve, its halvings
  held to agree to a tenth of `area_rel`, and refused by name where they
  do not (`regression/mirrored-revolve-common-seam-fault`). A shape with an extrusion face also
  takes `VolumePropertiesGK` for its volume, centroid and inertia, which
  the plain adaptive integration is 9e-7 off in
  (`boolean/elliptic-operand-cut`, against its closed forms); a shape of
  elementary faces alone keeps the fixed order over the whole shape and
  its committed numbers bit for bit. A shape trimmed by a B-spline
  pcurve of many spans — under a walked or fitted section edge
  (ADR-0018), or under an exact conic on a cone, a sphere or a torus,
  which has no exact 2D form there (ADR-0021) — is measured by the
  adaptive integration to 1e-12 instead, volume and area both: over such
  a pcurve the fixed order is 1e-6 off where the shape is right to 1e-10
  against a quadrature of the exact section.
- **A `profile` plane's `x` and `y`** must be orthogonal (each normalised
  first): refused on both sides, by the same named tolerance
  (`arris_math::Precision::DEFAULT.angular_tolerance`, Open CASCADE's
  `Precision::Angular`, mirrored as a literal in `recipe.py` with a
  comment pointing back here).
- **`analytic`** is the author's closed form, the cross-check that catches a
  convention mismatch on either side (profile orientation, seam counting,
  which faces a fuse keeps). Every field is optional; `degenerate: true`
  says the result has no volume (`boolean/flush-common`,
  `boolean/swallow-cut`, `boolean/disjoint-common`), and then nothing
  else is compared.
  `expect_error: "tangent-contact" | "non-manifold" | "blend-too-large" |
  "tangent-chain" | "vertex-blend" | "elliptic-revolve" | "vanishes" |
  "vertex-splits" | "no-exact-offset" | "surface-collapses" | "gap" | "self-intersects" |
  "opening-dragged" | "no-crossing" | "path-corner" | "profile-along-path" |
  "sweep-folds" | "frame-does-not-close" | "loft-too-few-sections" |
  "loft-loop-count" | "loft-coincident" | "unsupported"` says Open CASCADE
  builds a result Arris refuses by design (the tangent cases,
  `BooleanReason::TangentContact` — ADR-0004, whose contact Open CASCADE carries
  as an edge of four faces, so where that makes the Euler characteristic
  odd, as in `boolean/tangent-hole`, the oracle records no genus
  and the recipe states none; `boolean/edge-touching-fuse`, two
  solids sharing an edge, `InputReason::NonManifold` — ADR-0006, whose
  compound has an odd Euler characteristic, so the oracle records no
  genus for it and the recipe states none; a blend it builds and Arris
  refuses as `BlendReason::TooLarge`, `TangentChain` or `VertexBlend` —
  ADR-0007; a revolve of an elliptic profile segment, which it sweeps
  into a surface of revolution with an elliptic meridian and Arris
  refuses as `SweepReason::EllipticRevolve` — ADR-0014; a sweep whose
  path has a corner (`path-corner`, `OpError::Path`), whose profile lies
  along the path (`ProfileAlongPath`), folds past a curved span's radius
  of curvature (`SweepFolds`) or whose closed path's frame comes back
  turned (`FrameDoesNotClose`, where the pipe's closing face is not a
  closed surface and the oracle records no genus) — ADR-0063; a loft with too few sections or unequal loop counts (`OpError::Loft` with `TooFewSections` or `LoopCount`), where the oracle is not asked and records the empty result, and two coincident consecutive sections (`Coincident`), where `ThruSections` reports not done — ADR-0065; an offset whose
  move it carries through a change of topology, dropping a face or
  splitting a vertex into an edge, which Arris refuses as
  `OffsetReason::Vanishes` or `VertexSplits`, a distance that drives a surface's
  radius through zero (where Open CASCADE builds an inverted surface or
  crashes) `SurfaceCollapses`, and the face of an elliptic cylinder
  `NoExactOffset` — ADR-0048 §3; a face dragged along a tangent chain that
  pulls clear of a fixed neighbour, `Gap`, where Open CASCADE makes nothing
  and the oracle records the empty result — ADR-0048 §6; an `expect_error`
  recipe's oracle may be degenerate without the recipe saying
  `degenerate`, which the two together never do, and the runner still
  asserts the named error, not any `Degenerate`; a push whose
  built result has faces running into each other away from their shared
  edges, `SelfIntersects`, from the checker's global level — ADR-0048 §7; a
  shell opening tangent to a wall, which Open CASCADE answers with the body
  unhollowed and Arris refuses as `ShellReason::OpeningDragged` — ADR-0049; a split whose plane misses the body or only touches it, where Open CASCADE makes one side empty and Arris refuses as `SplitReason::NoCrossing` — ADR-0051 §7; a blend through a face pair or an end with no closed form yet, `OpError::Unsupported` whatever the pair, ADR-0035 §6): the oracle's numbers are
  recorded and the lint still
  cross-checks them against the other `analytic` values, but the runner asserts the typed error and compares
  nothing — and the oracle's self-test records the result without
  round-tripping it through STEP, since nothing ever reads it back
  (`boolean/tangent-hole`'s slit carries the tangent ruling as an edge of
  four faces, which Open CASCADE's own reader does not give back as a
  closed surface). `counts_differ: "why"` says Arris builds the result with
  counts that differ from Open CASCADE's by a stated convention
  (`boolean/tangent-outside-cut`: Open CASCADE imprints the tangent
  ruling on the touched face, Arris keeps the face whole); `counts` is
  then required and is Arris's, and so is `genus` where the convention
  changes it (`boolean/coaxial-fuse-four-off`: Open CASCADE closes a
  crescent hole 4e-7 wide), the runner and `compare.py` hold the result
  to them, the oracle's counts stay in `expected.json` as the record, and
  the lint holds both to the Euler line. `inertia` is the
  tensor about the centroid at unit density, rows of expressions in
  `expected.json`'s convention (products of inertia negated), checked
  against the oracle's relative to its largest component.
  `measure_differs: "why"` says Open CASCADE's *measurements* of the
  result are wrong and the closed forms right (ADR-0015): `volume`,
  `area`, `centroid` and `inertia` are then all required and are what
  the runner (measure stage and mesh volume) and `compare.py` hold
  Arris to; the oracle's values stay in `expected.json` as the record,
  and the lint fails a variant in which none of them differs from its
  closed form. Before using it an author shows, in the key's text and
  the description, a closed form derived rather than read off a run,
  why the oracle is wrong rather than following a convention — an
  invariance it breaks, such as a motion that is the identity on the
  solid changing its volume — and the size of its error.
  `step_differs: "why"` says Open CASCADE's *own* STEP of the result does
  not read back as the result — other counts, or another solid — while
  Arris's does (ADR-0023); the text gives both sides in numbers. Only the
  oracle's self-test reads it, and skips that fixture's round trip; the
  runner skips its read-back stage, `compare.py` compares exactly as
  without it, and the lint refuses it on a result Arris does not build.
  `occt_step_refused: {"kind": …, "why": …}` says Arris's reader refuses
  Open CASCADE's own STEP of the result with that refusal kind (as
  `RefusalKind` prints it): a file that describes no solid by ISO
  10303-42 — its boolean leaving a face whose walk does not close. (Its
  writer turning a face's bounds is turned back by the read's
  orientation healing, ADR-0058 §4.) The read-back stage asserts
  that refusal and fails once the file reads, so the entry is lifted with
  the change that reads it. `occt_nurbs_refused: {"kind": …, "why": …}`
  does the same for the converted file alone, where the plain one reads
  or is refused otherwise: a gap of the reader's on Open CASCADE's
  conversion, held by a regression fixture of its own.
  `occt_step_unread: "why"` skips both read-backs of Open CASCADE's own
  STEP where reading it measures the reader on a walked blend rather than
  the operation — minutes of fitting, then a refusal — and names the
  regression fixture that holds that reading instead; Arris's own STEP is
  still read back by Open CASCADE and compared. The runner holds a `measure_differs` fixture's
  read-back to the oracle's own measurements: all but one blame Open
  CASCADE's boolean, whose shape its file carries. `occt_file_measure_differs:
  "why"` (needs `measure_differs`) is that one: Open CASCADE's measurement
  of its own, right, shape is the wrong one (`sweep/sweep-oblique-circle`),
  and the read-back stages hold Arris's reading of its file to the closed
  forms. Every other round trip in
  the self-test is held to the result's own tolerance where that is wider
  than the fixture's: a reader may move the boundary by its largest vertex
  tolerance. `genus` is what the
  Euler line is checked with:
  `V − E + F − (L − F) − 2(S − G) = 0` with the oracle's counts.
- **Tolerances** are the fixture's; absent ones take the defaults shown.
  Counts and classifications are always exact. Three keys are Arris's
  alone and never reach the oracle: `mesh_chord` (default `1e-3`), the
  chord tolerance the runner tessellates the result at, and
  `mesh_volume_rel` (default `2e-3`), how far the mesh's signed volume may
  be from the oracle's volume — sized by the closed form of an inscribed
  prism, `4δ / (3r)` at that chord on the corpus's smallest radius
  (ADR-0003); and `inertia_rel` (default `1e-9`), read by the runner for
  the `measure` stage's inertia tensor.
- **`mesh_keeps`** is a list of step names, Arris's alone and outside the
  recipe hash like `tolerances`: after the mesh stage the runner meshes
  each named step's body at `mesh_chord` too, and every edge that body
  shares by id with the result must have the same polyline — the same
  count, the same positions bit for bit — in both meshes
  (`CorpusError::Mesh`). An edge's discretisation is a function of the
  edge, so a face an operation keeps keeps its mesh (ADR-0010; the
  per-face tessellation plan).
- **`precision`** is the `arris_math::Precision` the model is created
  with: every field the recipe names, the rest `Precision::DEFAULT`
  (`{"precision": {"default_tolerance": 1e-6}}`). Arris carries no unit,
  so this is what makes a recipe's numbers mean metres rather than
  millimetres — a consumer in metres sets `default_tolerance` at the
  micrometre scale (`docs/ARCHITECTURE.md` §Units), and the corpus's
  `probe-*-m` fixtures are the proof that the operations hold there. It is
  Arris's alone and never reaches the oracle, whose `Precision::Confusion`
  is a constant of its build, so it is outside the recipe hash like
  `tolerances` — and a fixture in another unit therefore states a
  `mesh_chord` of its own, the default `1e-3` being a chord for a model of
  order 1–1000 units. `Model::new` refuses an inconsistent one and the
  runner reports it as `CorpusError::Precision`.

## Geometry fixtures (`"kind": "geometry"`)

`"kind"` absent or `"solid"` is the recipe above; anything else that is
not `"geometry"` is an error on both sides (`fixture_kind` in the
oracle, `arris_debug::fixtures::kind_of` in Rust) — never silently read
as a solid.

The M1 oracle: no solid, no STEP. `fixture.json` names analytic surfaces
and curves by their frames and radii, the parameters to evaluate them at,
the points to project onto them, and the pairs to intersect; the oracle
writes what Open CASCADE's `Geom_*::D2`, `GeomAPI_ProjectPointOn*`,
`IntAna_QuadQuadGeo` and `IntAna_IntConicQuad` say, and
`crates/arris-geom/tests/oracle.rs` holds Arris to it: evaluations and
projected parameters to 1e-9 relative, intersection types exactly, the
oracle's sampled points on Arris's curves to 1e-9.

```json
{
  "kind": "geometry",
  "description": "...",
  "params": {"R": 2},
  "surfaces": {
    "wall": {"type": "cylinder", "origin": [0, 0, 0], "z": [0, 0, 1], "x": [1, 0, 0], "radius": "R"},
    "cap":  {"type": "plane", "origin": [0, 0, 5], "z": [0, 0, 1], "x": [1, 0, 0]}
  },
  "curves": {
    "ring": {"type": "circle", "origin": [0, 0, 0], "z": [0, 1, 0], "x": [1, 0, 0], "radius": 3}
  },
  "samples": [
    {"of": "wall", "params": [[0, 0], ["pi / 2", 3]], "points": [[0, -5, 3]]},
    {"of": "ring", "params": ["pi"], "points": [[-4, 0, 0]]}
  ],
  "pairs": [{"a": "cap", "b": "wall"}, {"a": "ring", "b": "wall"}]
}
```

| `type` | Fields |
|---|---|
| `plane` | `origin`, `z`, `x` |
| `cylinder`, `sphere` | `origin`, `z`, `x`, `radius` |
| `elliptic_cylinder` | `origin`, `z` (the axis), `x` (the section's major axis), `major_radius`, `minor_radius` |
| `cone` | `origin`, `z`, `x`, `radius` (at `v = 0`), `half_angle_deg` |
| `torus` | `origin`, `z`, `x`, `major_radius`, `minor_radius` |
| `line` | `origin`, `direction` |
| `circle` | `origin`, `z`, `x`, `radius` |
| `ellipse` | `origin`, `z`, `x` (the major axis), `major_radius`, `minor_radius` |
| `nurbs` (curve) | `degree`, `knots` (flat: each as often as it repeats), `control_points` (Cartesian), `weights` |
| `nurbs` (surface) | `degree` (`[p, q]`), `knots` (`[u, v]`, flat), `control_points` (a row per control point of `u`, each row a point per control point of `v`, Cartesian), `weights` (the same shape). Evaluated and projected onto (`GeomAPI_ProjectPointOnSurf`); paired with a line only (`GeomAPI_IntCS`) |

- A frame is `origin`, `z`, `x` as `Frame::new` and `gp_Ax3` build it: `x`
  made perpendicular to `z`, both normalised, `y = z × x`. Numbers may be
  expressions as in a solid recipe.
- A sample's `params` are `[u, v]` pairs for a surface and `t` values for
  a curve; `points` are projected. A pair is two surfaces, a curve `a`
  against a surface `b`, or two curves with a `nurbs` one among them or
  two conics.
- The fixtures here are written by `geom/generate.py` (closed forms at
  full precision in committed poses) — edit and rerun it, then
  `expected.py`, rather than the numbers: `analytic-eval`,
  `c1-intersections` (the plane and plane–cylinder table),
  `c2-cylinder-pairs` (every pose of the cylinder–cylinder table) and
  `c2-quadric-pairs` (the coaxial pairs with a cone, a sphere or a torus
  in them, and the pairs any sphere makes: crossings, touches, a plane
  through an apex, two spheres touching; a plane through a cone's and a
  torus's axis, two rulings and two tube circles; and lines against the
  three — chords, grazes, a ruling, a line through the apex, the poles,
  a torus's four crossings and its inner and outer equators grazed) and
  `c3-cylinder-pairs` (the cylinder pairs that meet in a quartic:
  crossing axes of unequal radii, skew axes within the radii breaking
  out of the larger cylinder or staying inside it, one pair swapped) and
  `c3-quadric-pairs` (the pairs with a cone or a sphere in them that
  share no axis: a plane off a cone's axis in an ellipse, a parabola and
  a hyperbola, through the apex in the apex, two rulings or a touching
  one; a pipe, a ball and a narrow cone against a cone, a post through
  the ball) and `c3-torus-pairs` (a plane, a cylinder, a cone, a sphere
  and a second torus around one torus that shares no axis with any of
  them: the spiric sections of a plane parallel to the axis through the
  hole, tangent to it and through the tube, and of an oblique plane; a
  drill through the tube and a pipe of the tube's radius tangent to the
  centre circle, which shares that tube circle exactly; a cone and a
  sphere off the axis; an interlocked torus and a larger one) and
  `c3-conic-hits` (circles and ellipses against the four
  surfaces a conic has no exact section of — a cone, a sphere, an
  elliptic cylinder and a torus — each placed where the surface's trace
  in its plane is a closed form: the cone's two lines through its apex
  crossed and touched, the elliptic cylinder's two lines at its major
  radius crossed, touched and cut obliquely, the torus's equators crossed
  eight times by an equatorial ellipse, its inner equator touched and one
  tube circle drilled; and coplanar conic pairs with an ellipse among
  them — an ellipse against a circle between its radii, against a circle
  of its minor radius (a touch at both minor vertices), against a second
  ellipse about its centre, and one pair swapped) and
  `c3-nurbs-hits` (an ellipse as four rational quadratic
  arcs, a quintic of three spans and a rational cubic with a double
  knot, each against a plane, a cylinder, a cone, a sphere and a torus)
  and `c3-nurbs-crossings` (the same three against lines, circles and
  ellipses: through them across their planes, a chord, a tangent and a
  concentric circle in the ellipse's plane, a line beside them) and
  `c4-nurbs-projections` (points projected onto three free-form NURBS
  surfaces: a bicubic saddle, a rational bump of degrees (2, 3), and a
  surface folded back on itself so a point in the pocket has a local
  minimum on each layer). A point whose nearest point is not unique is
  left out of it (Arris reports the tie, `tests/nurbs_project.rs`), and
  so is one whose nearest point is on a boundary Open CASCADE's projection
  stops short of: its distance was above the true minimum by up to 2.4e-4
  off the saddle's edge, the bump's corner and the fold's edge, and those
  are held to a grid instead.
  `c8-line-nurbs-hits` (lines against NURBS surfaces: a bicubic of two
  spans each way crossed inside, on its knot line `u = 1`, on `v = 1`
  and where the two meet; an octant of a sphere as a rational
  biquadratic crossed inside, along its axis through its pole and grazed
  along a parallel; a quadratic profile extruded, met along a ruling, and
  passed by a line two tolerances above its crest) is answered by
  `GeomAPI_IntCS` too, which reports the line along the ruling as a cloud
  of points on it and no segment: the oracle test holds each to the line
  Arris calls `Coincident`, as it holds a cone's ruling, and pins every
  pair's count, touches and `Coincident` by name.
  `c8-curve-extrema` asks its pairs for the stationary pairs of the
  distance instead: two skew lines and two parallel ones, a circle
  against its axis, a circle linked through it, a circle of its radius on
  the tube about it (one family at distance 2 with no common axis), a
  coaxial circle and one a thousandth of a radian off it, an ellipse
  against a circle standing over its centre, a second ellipse and a line,
  and two NURBS curves against a circle, a line and each other;
  `c8-curve-surface-extrema` asks a curve and a surface instead: lines
  along and skew to a cylinder, an elliptic cylinder, a plane and a sphere,
  circles coaxial with a torus and a cone, linked with a torus's tube and
  tipped round an elliptic cylinder, an ellipse clear of a cone, and a
  NURBS curve against a plane and an elliptic cylinder; and
  `c8-surface-extrema` asks two surfaces: a bead on a pipe's axis, two pipes
  skew at their common normal, a pin off a hole's axis, a ring over a floor,
  a pearl above a ring, parallel planes, a wall along an elliptic cylinder,
  a pipe through a ring and two elliptic cylinders with parallel axes.
  `c4-closed-curve-hits` is a clamped closed cubic B-spline, its ends one
  point, against a plane and a cylinder through that join and a plane
  clear of it: the join is one hit, at the start parameter, as Open
  CASCADE's general intersector counts it.
  An oracle parabola or hyperbola, sampled at its own
  parameters in `[−2, 2]` a branch per curve, is held against Arris's
  exact rational quadratic NURBS. An `elliptic_cylinder` is the section
  ellipse extruded along the axis, which is what Open CASCADE carries it
  as and the same point set with the same `(u, v)`; it is no `gp`
  quadric, so it takes no *surface* pair — those are the property tests'
  — and the general intersector finds nothing on an extrusion's
  unbounded parametric range, so the oracle trims it to `EXTRUSION_REACH`
  either way along its axis and a recipe keeps its hits well inside
  that.
- A `nurbs` curve is `NurbsCurve::new`'s arguments and the oracle's
  `Geom_BSplineCurve` over the same knots, so the two share a parameter
  and its hits are compared in it. Against a surface it goes through
  `GeomAPI_IntCS`, Open CASCADE's general curve–surface intersector,
  whose hits are held to both operands and deduplicated as the conics'
  are; it has no `coincident`, and a segment of the curve on the surface
  is an error of the recipe. Against a line, a circle or an ellipse,
  Open CASCADE has no 3D curve–curve intersector, so the oracle is
  `GeomAPI_ExtremaCurveCurve` — which is also what decides two coplanar
  conics, the only other curve pair a recipe may ask for — span by span
  of the B-spline (over its
  whole domain the search misses crossings): every extremum within
  `Precision::Confusion` is a hit with both parameters, `t` and `tb`,
  and the oracle test holds Arris's hits to them as it holds a curve
  against a surface, a touch to 1e-6.
- A pair with `"extrema": true` is two curves asked for the stationary
  pairs of their distance, not their intersection. The oracle is
  `Extrema_ExtCC` over each curve's whole domain (a line's `LINE_REACH`
  either side of its origin). It answers `"type": "parallel"` with the
  `distance` where it sees a family, or `"type": "extrema"` with each pair's
  `ta`, `tb`, points `a` and `b` and `distance`. A pair it reports at a
  range end that is not stationary there is dropped and counted under
  `dropped`. Its answer is not complete: two circles may come back with one
  pair of four, and a family at one distance with no common axis as points
  sampled along it. So `oracle.rs` holds every oracle pair to one of
  Arris's isolated pairs (to 1e-6 in both points, or to 1e-4 where the
  distances agree to 1e-9, since in a valley nearly flat along a family
  Open CASCADE places the pair only to about 4e-6), or to the distance of
  one of Arris's continua; it holds Arris's nearest to the oracle's nearest
  to 1e-9, and `parallel` to a continuum at its distance. Completeness is
  the property tests' (`tests/extrema.rs`), against a dense grid.
- A pair with `"extrema": true` of a curve `a` and a surface `b` is asked
  for the stationary pairs of their distance. The oracle is
  `GeomAPI_ExtremaCurveSurface` over the whole curve and surface (the
  elliptic cylinder trimmed to `EXTRUSION_REACH`; a NURBS surface has no
  form). It answers `"type": "parallel"` with the `distance` of the nearer
  family where it sees the curve along the surface, or `"type": "extrema"`
  with each pair's `t`, `u`, `v`, points and `distance` under
  `curve_surface`. It is not complete, so `oracle.rs` holds it as it holds
  the curve pairs: each pair of the oracle to one of Arris's or to a
  continuum at its distance, Arris's nearest to its nearest. Completeness
  is the property tests' (`tests/extrema_surface.rs`): every local minimum
  of a dense sample of the curve, the surface eliminated by its
  projection, is a pair of the answer.
- A pair with `"extrema": true` of two surfaces asks for their distance's
  stationary pairs: `GeomAPI_ExtremaSurfaceSurface` over each surface's
  whole range (an unbounded direction within `EXTRUSION_REACH`). It answers
  `"type": "parallel"` with the `distance`, or `"type": "extrema"` with each
  pair's `ua`, `va`, `ub`, `vb`, points and `distance` under
  `surface_surface`. It reports pairs that are not along both surfaces'
  normals (a grid point at a range end), which are dropped and counted
  under `dropped`; what is left is incomplete. `oracle.rs` holds every pair
  of the oracle farther than REL from zero to one of Arris's or to a
  continuum at its distance, and Arris's nearest to at most the oracle's:
  a meeting is `intersect_surfaces`'s, and the oracle finds the nearest
  only sometimes. Completeness is the property tests'
  (`tests/extrema_surfaces.rs`): every local minimum of a dense sample of
  one surface, the other eliminated by its projection, is a pair of the
  answer, and every grid extreme of a NURBS surface's height or distance is
  a critical point.
- A surface pair Open CASCADE finds no conic for is `"type": "unsolved"`
  (`IntAna_NoGeometricSolution`), with the lines `GeomAPI_IntSS` walks
  for it as curves of type `section`: 17 samples each, every one polished
  onto both surfaces by Newton steps, since the walked line is only
  within about 1e-8 of them; a line along which the surfaces touch has
  nothing to polish onto and is dropped and counted under `dropped`. The
  oracle test holds Arris to `Unsupported` or to a closed-form `Empty`
  against it, or holds Arris's curves to both surfaces — a fitted one to
  `SECTION_FIT_FRACTION` of the tolerance (ADR-0018) — and every walked
  sample to one of them within that fraction, each curve carrying some
  and none reaching past them; `oracle.rs` pins which one each pair is
  by name. The walk splits a loop into one line or two, so its count is
  not compared. Open CASCADE answers two ellipses for skew cylinders of
  equal radii (below), so the corpus has no such pair. A pair meeting in isolated points is `"type": "point"` with
  its `points`; Arris's points of a meeting in points alone are held to
  them exactly.
- The corpus lint checks presence, hash and shape for this kind (every
  name resolves, every spec builds, one result per sample and pair with
  the counts asked for); the values are the oracle test's to compare.
- What the oracle cannot answer stably is recorded as it is: the
  conic–quadric intersector has no tolerance, so a curve built exactly
  tangent to a cylinder comes back as none, one or two points by rounding,
  and Arris's single `tangent` hit is compared against whatever it
  reported to 1e-6 (a touch is conditioned as the square root of the
  rounding); a line parallel to the axis yields a hit at `t ≈ 1e16` that
  the oracle drops as off both operands and counts under `dropped`. Two
  parallel cylinders touching come back as one ruling or as two about
  1e-7 apart, by the same rounding (`c2-cylinder-pairs`' inside touch is
  two), and each is compared against Arris's single touching ruling to
  1e-6. The
  cylinder–cylinder intersector has no case for skew axes: it answers
  `unsolved` for unequal radii whether or not the cylinders can meet,
  and two ellipses for equal radii whatever the gap between the axes. So
  `c2-cylinder-pairs` gives its skew pairs unequal radii, and Arris's
  `Empty` for the pair further apart than the radii stands against
  `unsolved`. The plane–sphere case decides a touch at machine epsilon,
  so a plane built tangent at a pole comes back `empty` (or a circle of
  rounding radius) where Arris says one touching point, which the oracle test
  accepts within 1e-6 of Arris's point; the cylinder–sphere case wants
  the sphere's own axis on the cylinder's exactly, so `c2-quadric-pairs`
  has the same sphere twice — its frame turned across the axis, which
  comes back `unsolved` and holds Arris's circles to both surfaces, and
  along the axis, which is answered. A line against a torus is
  `IntAna_IntLinTorus`, not `IntAna_IntConicQuad`, whose quadric has no
  torus. A ruling of a cone in the `tilt` pose comes back as two points
  of the ruling rather than in the quadric, the quadratic's vanishing
  coefficients being rounding there, so the oracle test holds those
  points to the line Arris calls `Coincident`.

## Part fixtures (`"kind": "part"`)

A real part (ADR-0026): a STEP file Arris did not write, in the directory
beside `fixture.json`, `expected.json` and one `dump.<id>.<instance>.txt`
per solid read. The committed tier is NIST's MBE PMI test models, which
NIST lets anyone use without restriction; NIST asks for acknowledgement,
and this is it: the parts under `real/nist-*` are NIST's.

```json
{
  "kind": "part",
  "description": "what the part is",
  "file": "nist_ftc_09_asme1_rd.stp",
  "source": "the URL and archive it came from",
  "licence": "the licence, quoted from the source",
  "sha256": "<of the file>",
  "solids": [
    {"id": 5384, "instance": 0, "outcome": "read"},
    {"id": 6073, "instance": 0, "outcome": {"refused": {"kind": "unsupported entity", "why": "…"}},
     "blocks": {"read": "healing"}}
  ]
}
```

- **`solids`** lists every solid instance Arris's reader returns, in its
  order (`FileEntity`'s `#id` and placement), with the outcome it is held
  to. A change of outcome is a `fixtures:` commit. A refusal that starts
  to read fails its test until the fixture says so.
- **A solid's `blocks`** records the cycle each of its refusals blocks,
  as `arris_debug::histogram::Cycle` prints it (ADR-0026 §5): under
  `read` for a refused solid, and under each battery stage recorded
  `arris-refuses`. The runner holds it to the table, and the refusal
  histogram (`cargo run -p arris-debug --example real_parts --
  --committed`) counts from it. The battery example's `--record` writes
  it.
- **A solid's `healed`** is what its read heals (ADR-0058): `{"grown": N,
  "largest": gap, "turned": M}`, the count of vertices and edges the
  read grew past the read cap, the largest gap to six significant
  digits, and the count of faces it turned; a zero is left out. It is
  absent where the read heals nothing, and the runner holds the read's
  record to it either way, so no part of the corpus heals silently.
- **A refusal's `why` says why the refusal is right**, not which it is: the
  file holds an entity outside the subset, or describes no solid. A
  refusal of something the reader should map is a reader bug. It is
  shrunk to `regression/` like a wrong solid (ADR-0026 §4), and is never
  recorded here.
- **`expected.json`** is the oracle's reading of the file (`expected.py`),
  healed as Open CASCADE's reader heals by default. It holds one entry
  per solid, with the `#id` it came from, what a solid result records,
  `occt_heals` (the unhealed reading of that entity is not a valid solid,
  has other counts, or is missing), and `unhealed_counts`, and its
  answer to the battery (below). The hash covers `kind`, `file`,
  `sha256` and `battery`, so a changed file or operand stales it.
- **The runner** (`arris_debug::part::run`, `real_<slug>` in
  `crates/arris/tests/corpus.rs`) reads the file into a model of the
  fixture's `precision`, holds the list of instances and each refusal's
  kind exactly, and puts every `read` solid through these stages:
  - its healing record, held to `healed`;
  - the checker at `Full`, what it cannot decide on a NURBS face left
    unchecked;
  - matched to the oracle's solid of its `#id` nearest by centroid, each
    taken once; a read with no such solid is a solid the file does not
    have;
  - counts and genus, where healing changed no topology (`unhealed_counts`
    equal to `counts`);
  - volume, area, centroid and inertia within `tolerances`, widened to
    the body's own (ADR-0023);
  - the mesh at `mesh_chord`, closed and within `mesh_volume_rel`;
  - the dump.

  A solid the oracle reads and Arris neither reads nor refuses fails too.
- **The lint** holds the file to its `sha256`, `expected.json` to the
  hash, the oracle's solids to the Euler line, every refusal to a reason,
  and every `read` solid to its committed dump — none under `regression/`.
- **`waits_on`** lists the part fixtures under `regression/` a part waits
  on. Each is a reader bug it meets, shrunk to the smallest file that
  still fails, with the desired outcome, and its test `regression_<slug>`
  is `#[ignore]`d. While any is listed, the runner does not read the part;
  its `solids` record the outcomes it is to have once the bugs are fixed,
  so it needs no dump yet. The lint fails once a named fixture has left
  `regression/`, so the fix that moves it lifts the exclusion. The part
  exclusions and the part fixtures open under `regression/` are the same
  set, by test (`corpus_lint.rs`), counting the fetched tier's waits in
  `tools/real-parts.waits`. A part under `regression/` commits no dump,
  and its runner holds none.
- **A fetched part's failure** is committed as an excerpt, the failing
  solid alone (`tools/oracle/excerpt.py`), where its licence allows
  (ADR-0026 §2): a `regression/` part named by the fetched part's line in
  `tools/real-parts.waits`.
- **`read_seconds`** caps the read in the test profile's optimised build.
  It is set only on the fixture a slow read is shrunk to
  (`real/slow-gap-refusal`, once it passed), since timing on a shared machine is no
  assertion anywhere else (ADR-0026 §6).
- **The committed tier** is NIST's eleven AP203 geometry-only files,
  `real/nist-ctc-01` to `-05` and `real/nist-ftc-06` to `-11`. All eleven
  run, none waits, and every solid reads: CTC-02's and CTC-05's healed,
  their gaps grown and named (ADR-0058), every surface body as a sheet.
- A bug fixed moves its part into `real/` beside the NIST parts:
  `real/axis-placement-along-x-without-reference`, a cylinder whose
  circles are placed on an axis along `-X` to rounding with no reference
  direction; `real/seamless-cylinder-band`, a cylinder whose side is
  bounded by its two circles and no seam;
  `real/cone-face-without-its-apex`, a cone bounded by its base circle
  alone; `real/edge-through-sphere-pole`, a half ball bounded by one
  meridian circle through both poles; and
  `real/pcurve-fit-reported-as-gap`, CTC-01's file, whose edge #1864 lies
  0.0065 off its cylinders; `real/slow-gap-refusal`, CTC-05's file,
  held to its time budget, now read healed; and `real/torus-plane-section-undecided`,
  FTC-07's file, whose B-spline pcurves overshot their domain and whose
  torus is tangent to a plane on its seam. Six are faces of a part
  excerpted as a sheet (`excerpt.py --faces`):
  `real/edge-off-a-drafted-cylinder`, CTC-02's base plane and a drafted
  boss whose edge lies off the cylinder, read healed (ADR-0059);
  `real/healed-gap-cylinder-meets-sphere`,
  `real/healed-gap-nurbs-meets-cylinder` and
  `real/healed-gap-on-an-engraved-plane`, one gap each of CTC-04,
  CTC-05 and FTC-11 AP242, grown and named (ADR-0058), each held to the
  band its healed edges' tolerances leave the sheet's area;
  `real/edge-bending-within-its-tolerance`, an
  FTC-07 AP242 edge turning within its tolerance's length (E8); and
  `real/loop-crossing-inside-a-vertex`, an FTC-07 AP242 sphere face whose
  loop zigzags inside a vertex's tolerance (L5). Open CASCADE's healing
  moves those faces' boundaries further from the file's curves than
  Arris's read does, and each fixture's tolerances are the band that
  moves, derived in its description.
  `real/turned-face-in-an-open-shell` is written by hand:
  `real/face-based-surface-model`'s corner with one face's bounds turned
  as Open CASCADE's writer turns them, read with the face turned back
  (ADR-0058 §4) to the corner's own dump.
- Four fetched-tier excerpts waited under `regression/` on S5 until a
  read edge's tolerance came to hold where its faces meet (ADR-0059):
  `real/nist-ctc-02-ap242-mitred-pipes-s5`,
  `real/nist-ftc-06-ap242-mitred-pipes-s5`,
  `real/nist-ctc-05-ap203-torus-elbow-s5` (read healed) and
  `real/nist-stc-10-ap242-holes-into-bore-s5`.
- `real/nist-ftc-09-offset` is `real/nist-ftc-09` with one plane wrapped
  in an `OFFSET_SURFACE` by hand: the refusal path under test.

### The battery

Every solid of the committed tier that reads goes through a fixed
battery of operations (ADR-0025 §6, `arris_debug::battery`), and each is
sorted into the differential's classes (ADR-0024 §2):

| Stage | What |
|---|---|
| `write_read` | the body written to STEP and read back by Arris: every solid read, the checker green, the same counts, the mass properties within the body's own tolerance. Arris's alone |
| `box_cut` | cut by a box with a corner at the oracle's centroid, its edges along the oracle's principal axes, reaching past the part |
| `drill_x`, `drill_y`, `drill_z` | cut by a cylinder through the centroid along each principal axis, smallest moment first, of a tenth of the smallest radius of gyration |
| `fillet` | at most four of the solid's edges between two faces, in id order at a fixed stride, each named by its curve's midpoint, blended at a tenth of the shortest one's length — an edge whose two faces are tangent along it is not sampled: no rolling ball blends one, and Open CASCADE refused all 2479 of the fetched tier's |

```json
"solids": [
  {"id": 189, "instance": 0, "outcome": "read",
   "battery": {"box_cut": "agree", "fillet": "oracle-refuses", "drill_x": {"arris-refuses": "Degenerate(TangentChain)"}, …},
   "blocks": {"drill_x": "blend network"}}
],
"battery": {
  "#189[0]": {
    "box_cut": {"steps": [{"op": "step", "name": "part", …}, …], "result": "result"},
    …
  }
}
```

- **`battery`** holds the operand stages' recipes by solid
  (`#<id>[<instance>]`) and stage, each over the `step` operand naming
  the solid. `cargo run -p arris-debug --release --example battery --
  --write <dir>` derives them from the file and `expected.json`'s oracle
  reading, once; after that they are data, and a kernel change that would
  derive others does not move the fixture. They are in the hash, and
  `expected.py` builds each: `expected.json`'s `battery` holds what
  `measure` records of the result with its `own` tolerance and sizes, or
  `{"refused": why}`.
- **A solid's `battery`** records each stage's class: `agree`,
  `both-refuse`, `oracle-refuses`, or `{"arris-refuses": name}` with
  the differential's name of the `OpError` (the reader's refusal kind for
  `write_read`). `--record <dir>` runs the battery and writes them. The
  runner holds every stage to its class. A result is held as the
  differential holds one: the checker at `Full`, NURBS pairs left
  unchecked; counts net of removable vertices, only where healing changed
  none of the part's topology; the mass properties within both shapes'
  own tolerances; a closed mesh; the provenance audit.
- **A disagreement, a checker violation, a panic or an internal fault is
  never recorded.** It is a kernel bug, shrunk to `regression/` as a
  reader bug is.
- A part shrunk from another carries no battery: its battery is its
  source's.

## Query fixtures (`"kind": "query"`)

A solid recipe and the questions a consumer asks of its result, each held
to Open CASCADE's answer (ADR-0053 §8). One kind for every query of the
query cycle: ray fire is the first form, and the distance, interference
and section plans add theirs to the same `queries` list. They sit under
`query/`.

```json
{
  "kind": "query",
  "params": {}, "variants": {}, "steps": [ … ], "result": "<step>",
  "queries": [
    {"label": "up through the middle",
     "ray": {"origin": [20, 15, -5], "direction": [0, 0, 1]},
     "hits": "all", "expect_hits": 2},
    {"label": "over the top face",
     "distance": {"a": {"point": [10, 10, 15]}, "b": {"step": "box"}},
     "expect_distance": 5}
  ]
}
```

- The recipe is the solid kind's, interpreted unchanged (`params`,
  `variants`, `steps`, `result`, `precision`, `tolerances`); every number
  of a ray may be an expression over the params.
- A **ray query** is a half-line from `origin` along `direction` (any
  length but zero; both sides normalise it), so `t` is the distance along
  it and a hit at a negative `t` is never reported. `hits` is `"first"`
  (the smallest `t`) or `"all"`. Labels are unique within a fixture.
- A **distance query** (ADR-0054) is `"distance": {"a": <operand>, "b":
  <operand>}`. An operand is `{"point": [x, y, z]}`, `{"step": "<step>"}`
  (the body that recipe step built) or a step narrowed to its `face`,
  `edge` or `vertex` nearest a point, `{"step": …, "face": {"near": [x, y,
  z]}}`: ids differ between kernels, so an entity is picked by place.
  `expect_distance` is the distance the author works out by hand, held to
  the oracle's by the lint. Exactly one of `ray` and `distance` per query.
- `expect_hits` is the count the author works out by hand for what
  `hits` asks. The lint holds it to the oracle's, as `analytic` is held;
  it is optional.
- `expected.json` records, per variant and query, every hit of
  `IntCurvesFace_ShapeIntersector` on the half-line by ascending `t`
  whatever `hits` asks (`first` takes the first): `t`, `point`, the
  face's `uv`, its outward `normal` there (zero at a pole), the
  `face_centroid`, the line's `transition` (`in`, `out`, `tangent`) and
  the point's `state` on its face (`in`, or `on` its boundary).
- The runner builds the result under every variant, holds it to the
  checker at `Full`, and compares each query's hits to the oracle's: the
  count, `t`, the point and the normal within the fixture's tolerances,
  and the face a hit lands on **by its centroid, never by id**, since ids
  differ between kernels. Open CASCADE reports a ray through an edge once
  per face; Arris reports one hit naming the edge (ADR-0053 §1), which
  the fixture records with `analytic.query_differs` and its evidence: the
  runner then merges the oracle's hits of one `t` before comparing, holds
  the hit's normal to any of the merged faces' (it is the lowest-id
  face's), and the lint requires that some answer has such a pair, and
  refuses a pair without the entry. Matching faces by centroid trusts the
  oracle's quadrature, which is 7e-3 off on a B-spline cylinder wall; a
  fixture whose faces' centroids are far apart widens `centroid_abs` and
  says why.
- A distance query's `expected.json` answer is
  `BRepExtrema_DistShapeShape`'s: the `distance`, every solution it lists
  (both points, the kind each lies on — `vertex`, `edge` or `face` — and a
  face's centroid) and `contained` from `InnerSolution`. The runner
  compares the distance within `probe`, `contained` exactly, and the two
  feet to *any* listed solution (a family of equal solutions is listed by
  its sampled ends only). A foot's support kind is compared exactly and a
  face by its centroid within `centroid_abs`, except on a `point` operand,
  which Open CASCADE sees as a vertex, and when `contained`. `unique` must
  equal "the oracle lists one solution" unless `analytic.query_differs`
  records why, which then also lets a foot match none of the listed
  solutions.
- An **interference query** (ADR-0055) is `"interference": {"a": {"step":
  …}, "b": {"step": …}}`, whole bodies only, with the hand-worked `expect`
  (`apart`, `touching` or `overlapping`), `expect_volume` (an overlapping
  pair) and `expect_contact` (`point`, `curve` or `face`, a touching pair),
  each optional and held to the oracle's by the lint, and
  `expect_refused: "unsupported"` where the kernel refuses a pair (a NURBS
  face meeting the other solid in `common`, a NURBS face against an edge
  in `distance`) and Open CASCADE answers: the runner holds the refusal
  and skips the comparison. The two operands
  must be different bodies: a step against itself is not a pair. The
  oracle's answer is `verdict`, the `distance` of
  `BRepExtrema_DistShapeShape` (apart when it exceeds `probe`), and for
  an overlapping pair the volume of `BRepAlgoAPI_Common` (a pair is
  overlapping when that is above 1e-9 of the smaller operand's) with
  `a_in_b` / `b_in_a`, true when the common's volume equals that operand's;
  for a touching pair the `contact`: `face` when some pair of faces has a
  `BRepAlgoAPI_Common` with area, else `curve` when
  `BRepAlgoAPI_Section` holds an edge of positive length, else `point`
  (the section alone reports only the boundary of a face contact, so the
  pairwise common tells face from curve). The runner compares the verdict
  exactly, an apart pair's distance within `probe`, the volume relative
  to `volume_rel`, the containment flags exactly and the contact exactly
  unless `analytic.query_differs` records why it differs, which the lint
  accepts when some answer is a touching pair.
- A **section query** (ADR-0056) is `"section": {"body": {"step": …},
  "plane": {"origin": [x, y, z], "z": [x, y, z], "x": [x, y, z]}}`, the
  plane's `x` optional, with the hand-worked `expect` (`none` or `sheet`),
  `expect_faces`, `expect_area` and `expect_centroid`, each optional and
  held to the oracle's by the lint. The oracle's answer is `none` when
  `BRepAlgoAPI_Common` of the solid and a planar face past its box holds
  no face; else that common's `faces`, `loops` (its wires), `area` and
  `centroid`, and the distinct `edges` of `BRepAlgoAPI_Section(solid,
  plane)` with their total `edge_length`. The runner makes the section on
  a clone of the model, holds its sheet to the checker at `Full`, and
  compares none-or-sheet exactly, then the face, loop and edge counts
  exactly, the edge length and area relative to `area_rel`, the centroid
  within `centroid_abs`. Open CASCADE keeps a region of the plane flush
  with a face of the solid, which is no section (the plane's points with
  material on both sides): a fixture with such a pose records it with
  `analytic.query_differs`, and the runner then holds the verdict to
  `expect` and a sheet to `expect_area` (required) and `expect_centroid`,
  the closed forms, instead of the oracle's counts and measures.
  `expect_refused` (`unsupported` or `tangent_contact`) holds a section the
  kernel refuses as the booleans do where Open CASCADE answers: the runner
  holds the refusal and skips the comparison. Open CASCADE's length of an
  elliptic edge is 1.7e-4 relative off the exact, so an oblique cut of a
  cylinder is held to its closed form under `query_differs`.
- The hash covers `kind`, `params`, `variants`, `steps`, `result` and
  `queries`.
- No `dump.txt`: the result's own dump is the solid kinds'.

## `expected.json`

Layout in `tools/oracle/README.md`. Regenerate with

```sh
uv run --project tools/oracle tools/oracle/expected.py tests/fixtures/<area>/<slug>
```

and commit the change as `fixtures: …` saying why the numbers moved
(`.agents/rules/git.md`). The hash covers `params`, `variants`, `steps`,
`result` and `probes` of a solid, and `kind`, `params`, `surfaces`,
`curves`, `samples` and `pairs` of a geometry fixture, and `kind`, `params`,
`variants`, `steps`, `result` and `queries` of a query fixture; a change to any of
them makes the old `expected.json` stale and the lint says so. Both sides
parse a coordinate to the same `f64` (serde_json's `float_roundtrip`), so
a full-precision number hashes and evaluates identically.

## Property-test failures

Property tests run through `arris_debug::prop::check`: a seeded
`proptest` runner, `ARRIS_PROPTEST_CASES` cases (default 256) from
`ARRIS_PROPTEST_SEED` (default fixed, so CI is deterministic). A failure
prints the shrunk input and the `ARRIS_PROPTEST_SEED=…` that reproduces
it.

An expensive property is split across shards by `prop_shards!` and runs as
`property::shard_3`. Its failure names the shard, but the seed it prints is
still the *base* seed and the total case count — that recipe reproduces the
whole run, and the shard alone is re-run by its test name. The fixture a
sharded failure becomes records all three in its commit body: the base
seed, the case count and the shard. proptest's own `proptest-regressions/` files are gitignored and never
committed: the regression is a fixture under `regression/` (or a
hand-picked test beside the property) with the *desired* assertion,
`#[ignore]`d until it passes, and the seed and case count in the commit
body — so the original case stays reproducible after the shrinker or the
strategy changes (`.agents/rules/kernel.md` §Testing, `inspect` skill).

A finding of the differential (`arris_debug::differential`,
`crates/arris/tests/differential.rs`) takes the same path. The run prints
each failing case shrunk as a `fixture.json`, with the tolerances the case
was judged at. Those are the fixture defaults, widened to what both
shapes' own tolerances support (ADR-0024, amendment of step 5b). That
recipe is committed under `regression/<slug>/`, and its `expected.json`
comes from plain `expected.py`. If the draw keeps reaching the failure,
a named exclusion in `differential::EXCLUSIONS` covers its symptom and
cites the slug. The corpus lint then fails once the fixture leaves
`regression/`, so the commit that fixes it also lifts the exclusion.

The property tests over the same booleans hold that same list:
`differential::exclusion_of_panic` covers the checker guard's panic, and
`exclusion_of_error` covers a typed `OpError::Internal`. The shrunk case
then waits as an ignored test beside its property, so the fix that lifts
the exclusion lifts it everywhere.

A fuzz finding (`fuzz/`, ADR-0024 §5) is an intersector's answer on two
operands, not a recipe. Its operands become a test beside that
intersector's own, ignored until the fix lands. Once it passes they can
join a `geometry` fixture under `geom/`: every geometry fixture is held to
the oracle, so none waits under `regression/`.

## Conventions the numbers assume

Open CASCADE's, which Arris matches (`docs/DATA-MODEL.md`
§Conventions): a full revolve or a cylinder has one seam edge, counted
once; a degenerate edge (a cone's apex, a sphere's pole) is not counted
at all, on either side, so the Euler line closes on its genus; a fuse of flush boxes drops the shared face and does not merge the
coplanar neighbours; a common with no volume is degenerate.
