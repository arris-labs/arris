# 03 — Roadmap

Every milestone ends with a corpus run that prints numbers, and each retires
the scariest remaining unknown first. A milestone's "out" list is as binding
as its "in" list. One section per cycle; a finished cycle compresses to its
status line (`/close-cycle`), and the next is appended below.

Spine: **C1** M0 → M1 → M2 → M3 → M4 → M5 (the vertical slice, done), then
**C2** (the application gate, done 2026-09-19), then **C3** (every quadric
pair — closure: what Arris builds, Arris takes as an operand; done
2026-09-24), then **C4** (the reader: the STEP reader and a real-part
corpus; done 2026-09-26), then **C5** (the consumer's API: what a
plugin-based CAD cannot start without, ADR-0020's amendment; done
2026-10-02), then **C6** (the blend network: fillets and chamfers on the
face pairs real parts ask for; done 2026-10-05), then **C7** (prismatic
features: shell, offset faces, the multi-tool boolean, split by a plane;
opened 2026-10-05, ADR-0047). An unopened cycle carries
a name, not a number: it takes its number when `/close-cycle` opens its
section (ADR-0020).

---

## Fixtures

The unit of acceptance. A fixture is a directory
`tests/fixtures/<area>/<slug>/` with:

- `fixture.json` — the **recipe**: operands as primitives, profiles and
  poses, the operations applied to them in order, the classification probe
  points, the comparison tolerances, the `precision` the model is built
  with — `Precision::DEFAULT` unless the fixture is in another unit — and
  the closed-form `analytic` values where a formula exists. Both sides evaluate the recipe: the oracle in
  Open CASCADE, the test in Arris. A recipe, not a STEP file, so the corpus
  never depends on a reader that does not exist yet and so a change to an
  operand is a one-line diff.
- `expected.json` — the **oracle's answer**, one per recipe variant:
  volume, area, centroid, the inertia tensor about the centroid, counts
  (vertices, edges, faces, loops, shells), the Euler characteristic and
  genus, the in/out/on result for each probe point, whether Open CASCADE
  built a solid at all, the OCCT version and the recipe's hash. Written by
  `tools/oracle/expected.py`, never by hand, and committed. A change to it is a `fixtures:` commit that says why
  (`.agents/rules/git.md`).
- `dump.txt` — Arris's text dump of the result once the fixture passes,
  the regression guard for ids and provenance (`dump.<variant>.txt` for a
  variant other than `default`). Committed once the fixture passes; the
  corpus lint fails a fixture the runner compares under `primitive/`,
  `transform/`, `boolean/`, `sweep/`, `provenance/` or `blend/` without
  one, so those areas hold no ignored fixture by test. A failure waiting for its fix is
  a fixture under `regression/`, outside them, and moves into its area in
  the commit that makes it pass.

The test for a fixture (`arris_debug::corpus::run`, one `#[test]` per
fixture in `crates/arris/tests/corpus.rs`) builds the recipe in Arris,
runs the checker at `Level::Full` — nothing violated, nothing left
`unchecked` — compares counts and genus against `expected.json`, writes
STEP and runs `tools/oracle/compare.py` on it (volume, area, centroid
and every probe, read back by Open CASCADE), tessellates the result at
the fixture's `mesh_chord` and holds the mesh closed with its signed
volume within `mesh_volume_rel` of the oracle's, measures it over the
B-Rep (`ops::measure::mass_properties`) and holds volume, area, centroid
and inertia to the oracle's within `inertia_rel`, classifies every probe
point against the result itself (`arris_check::classify::classify_point`)
and holds it to the oracle's class exactly, asserts each step's
provenance accounting (data-model §Provenance), and diffs the dump —
written instead under `ARRIS_BLESS=1`.
A fixture whose result is no solid does not reach those stages: one the
oracle recorded none for (`degenerate`) must fail with
`OpError::Degenerate`, and one whose recipe says `analytic.expect_error`
must fail with that typed refusal — `tangent-contact`, `non-manifold`,
`blend-too-large`, `tangent-chain`, `vertex-blend`,
`elliptic-revolve`, `vanishes` or `vertex-splits`, the oracle's
numbers kept as the record of what Open CASCADE builds instead. A recipe may also say `analytic.counts_differ: "why"` and carry
its own counts, for the one place Arris's convention is deliberately not
Open CASCADE's (a tangent ruling left unimprinted); every other fixture
mirrors the oracle's counts exactly. Likewise `analytic.measure_differs:
"why"` with closed forms for the volume, area, centroid and inertia, for
a result Open CASCADE measurably builds wrong (ADR-0015), and
`analytic.step_differs: "why"` for a result whose own STEP Open CASCADE
does not read back as itself, which only the oracle's self-test skips
(ADR-0023).
Tolerances are the fixture's: relative 1e-9 on volume and area for
analytic results, exact on counts and classifications. The oracle is run,
never linked (`SEED.md` §7).

A third kind, `part` (ADR-0026), is the one fixture that is a file and
not a recipe: a STEP file Arris did not write, under `real/`, its licence
and SHA-256 in `fixture.json`. It records each solid's outcome, `read` or
the `RefusalKind` it is refused as, and for each solid read the battery's
operands and the class each stage lands in: tessellate and measure,
write and read back, a box cut, three drills along the principal axes,
and a fillet. Each refusal it holds records the cycle it blocks. A read
solid is held to Open CASCADE's healed reading of the same file and has
a dump. A part waiting on a kernel bug names its `regression/` fixture
and is not read until that fixture moves. The format is
`tests/fixtures/README.md` §Part fixtures.

The committed tier's refusal histogram (`cargo run -p arris-debug
--example real_parts -- --committed`), which a docs test holds to its
fixtures:

<!-- histogram: committed -->
11 parts, 16 solids: 9 read, 7 refused. 54 battery stages: 38 agree, 3 both refuse, 0 Open CASCADE refuses, 13 Arris refuses.

| Cycle | Parts blocked | read | measure | write_read | box_cut | drill_x | drill_y | drill_z | fillet |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| blend network | 4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 4 |
| healing | 3 | 3 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| NURBS | 2 | 0 | 0 | 0 | 2 | 0 | 0 | 0 | 0 |
| itself: supplemental geometry | 1 | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| sweep | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

| Refusal | Stage | Count | Blocks |
|---|---|---:|---|
| Degenerate(TangentChain) | fillet | 4 | blend network |
| unsupported entity | read | 3 | healing |
| gap past the cap | read | 2 | healing |
| Unsupported(plane surface × NURBS surface) | box_cut | 2 | NURBS |
| Unsupported(plane surface × NURBS surface) | drill_x | 2 | NURBS |
| Unsupported(plane surface × NURBS surface) | drill_y | 2 | NURBS |
| Unsupported(plane surface × NURBS surface) | drill_z | 2 | NURBS |
| unsupported entity | read | 2 | itself: supplemental geometry |
| Unsupported(NURBS curve × cylinder surface) | fillet | 1 | NURBS |
<!-- /histogram -->

---

## C1 — the vertical slice

*Goal: box − cylinder through every crate. A blind hole, an 8-hole bolt
pattern by repeated cut, and a flush box ∪ box union: checker green,
volumes and counts matching Open CASCADE, provenance naming every hole wall
stably across parameter changes (`SEED.md` §6).*

**Status: done 2026-09-12, tag `c1`.** Retired the representation:
analytic surfaces with explicit seams, per-entity tolerances and a pcurve
on every coedge make a plane/cylinder boolean one decomposition and one
selection table, proven by the corpus and property tests without a human
looking at a screen. ADR-0001 to ADR-0005.

### M0 — the harness

*Goal: everything the agent needs to see and to judge exists before the
first line of geometry.*

**Status: done 2026-09-05, tag `m0`.** Retired the oracle round trip
(Open CASCADE's own STEP of every fixture reads back clean) and the corpus
lint. No ADRs.

- The workspace of architecture, with the layer rule checked by CI and
  the hook.
- Ids, handles, `Orientation` and `Precision` (the bookkeeping types of
  data-model, no entities yet).
- `arris-check`'s skeleton: `Level`, `Report`, and a `Violation` variant per
  invariant in data-model §Invariants, with a test that the doc and the
  enum list the same set.
- `TriMesh` and `Polyline` in `arris-mesh`, with signed volume, area and
  closedness.
- `arris-debug`: the software rasteriser to PNG, `View`s and highlights;
  the property-test configuration and numeric strategies.
- The oracle: `tools/oracle/` as a `uv` project over `cadquery-ocp`,
  `expected.py`, `compare.py`, the recipe interpreter for every operation
  C1 will have, and a self-test that round-trips OCCT's own STEP.
- The fixture corpus laid out with every C1 recipe and its `expected.json`
  generated, plus a corpus lint that checks the Euler line and the
  `analytic` values against the oracle.

**Out:** any curve, surface, entity or operation.

**Accept:** `cargo test --workspace` green including the corpus lint;
`compare.py` matches OCCT's own STEP on every fixture; the layer check and
the wasm build pass; a PNG of a hand-built cube mesh rendered and read by
the agent.

### M1 — math and analytic geometry

*Goal: every curve and surface of C1 evaluates, projects and intersects,
proven by property tests against closed forms.*

**Status: done 2026-09-06, tag `m1`.** Retired the plane–cylinder case
table and the parametrisation agreement with Open CASCADE. ADR-0001.

- `arris-math`: points, vectors, unit vectors and frames over `nalgebra`;
  `Interval`; exact 2D orientation and in-circle predicates over `robust`;
  polynomial roots to quartic and interval-guarded Newton; tolerance types.
- `arris-geom`: `Surface` and `Curve` with the parametrisations of
  data-model; evaluation and first/second derivatives; point projection
  onto every variant; `Curve2` and the pcurve of every analytic curve on
  the plane and the cylinder; NURBS evaluation, knot insertion and
  least-squares fitting of a `Curve2::Nurbs` to a sampled curve.
- Intersections: plane–plane (line), plane–cylinder (circle, ellipse, one
  or two lines, or tangent), line–plane, line–cylinder, circle–plane,
  circle–cylinder — each an exhaustive match arm, everything else
  `Unsupported`.
- Geometric property-test strategies in `arris-debug`: random frames,
  poses, radii.

**Out:** cone, sphere, torus and cylinder–cylinder intersections; NURBS
surfaces beyond evaluation.

**Accept:** property tests at 1000 cases each — projection is idempotent
and lands on the surface to 1e-12·scale; every intersection point lies on
both operands to 1e-12·scale; pcurve image matches the 3D curve to the
fitting tolerance; the plane–cylinder case table agrees with the closed
forms in random poses.

### M2 — topology, the checker, primitives, formats

*Goal: a box and a cylinder exist as bodies in the arena, pass every
invariant, and Open CASCADE reads them back with the right numbers.*

**Status: done 2026-09-06, tag `m2`.** Retired the seam, through the
checker and through Open CASCADE's reader, and Euler operators over
immutable entities. ADR-0002.

- `arris-topo`: the chunked arena, entities, orientation composition,
  adjacency indices, deterministic iteration, transactions, `import`,
  `retain` (sparse), the raw insert API for tests.
- Euler operators (Mäntylä's set, adapted to coedges and seams) as the only
  way an operation builds topology.
- `arris-check` complete at `Level::Fast`, plus L5/S5/B1/B2 at `Full`; a
  violation test per invariant.
- `ops::primitive_box`, `ops::primitive_cylinder` (with its seam), each
  returning `Generated` provenance for every entity.
- `io::step` writer; `io::native` round trip; `arris_debug::dump_text`.

**Out:** any operation that takes a body as input.

**Accept:** the `primitive/*` fixtures pass end to end (oracle reads the
STEP: volume, area, counts exact); every invariant's violation test
reports its violation and nothing else; native round trip dumps
identically; ids identical across two runs.

### M3 — tessellation and measurement

*Goal: the agent can look at a body, and the kernel can measure one.*

**Status: done 2026-09-07, tag `m3`.** Retired a constrained Delaunay
triangulation of our own and mass properties as an exact flux integral
over the B-Rep. ADR-0003.

- `arris-mesh::tessellate`: edges discretised once at the model's
  tolerance and shared by both faces; faces triangulated in (u, v) by
  constrained Delaunay over `robust` predicates, seams and periods handled
  through the pcurves; `FaceRange`/`EdgeRange` in iteration order.
- `arris_debug::render_png` over bodies, faces coloured by id, one
  `Highlight` per call; `arris_debug::rerun` (feature) for the human.
- `ops::measure::mass_properties`: volume, area, centroid, inertia by
  Gauss over the faces with quadrature in (u, v).

**Out:** adaptive or curvature-driven refinement beyond a chord tolerance;
`f32` output.

**Accept:** the mesh of every `primitive/*` fixture is closed and its
signed volume matches the oracle within the closed form of an inscribed
polygon's error, `4δ/(3r)` at the runner's `mesh_chord` (`mesh_volume_rel
= 2e-3`, sized by the corpus's smallest radius); `measure` matches the
oracle to 1e-9 relative; a PNG of the cylinder shows one wall, two caps
and a seam, read by the agent.

### M4 — booleans on plane and cylinder (the risk milestone)

*Goal: box − cylinder, then everything C1's corpus asks of it.*

**Status: done 2026-09-11, tag `m4`.** Retired the risk the cycle is
named for: a plane–cylinder boolean is one decomposition over shared
paves and one selection table, and what a manifold `Solid` cannot hold is
a typed refusal rather than a tolerance accident. ADR-0004, ADR-0005.

- The General Fuse decomposition in `ops::boolean`: intersect every face
  pair (M1's table), split faces by the intersection edges with pcurves on
  both sides, share the split edges between operands (the pave model, read
  in the reference tree and rebuilt for coedges), classify each piece
  in/out/on with tolerance-aware point classification, assemble the result
  for `fuse`, `common`, `cut`; `boolean::interferences` is the same
  decomposition as a printable value.
- Coincident planar faces (flush) and tangent cylinder–plane contact as
  explicit cases, not tolerance accidents.
- Provenance built inside the algorithm: `Modified` for split pieces,
  `Generated` for intersection edges and tool-face images, `Deleted` for
  the swallowed rest; `Provenance::then` for chains.
- `ops::transform`.

**Out:** cone, sphere, torus, NURBS operands; two operands whose result has
two shells (enclosed cavity); merging of same-domain faces after a fuse.

**Accept:** every `boolean/*` fixture passes end to end, property tests
hold (volume additivity `V(A ∪ B) + V(A ∩ B) = V(A) + V(B)`,
`V(A − B) + V(A ∩ B) = V(A)`, commutativity of `fuse` and `common` up to
ids, cut-then-fuse restores the volume) at 200 random poses of a box
and a cylinder, and `provenance/bolt-pattern-rebuild` reports 8 of 8 walls
under the same origin across three parameter sets.

### M5 — sweeps and the cycle's corpus

*Goal: a sketched profile becomes a solid, and C1 closes on its numbers.*

**Status: done 2026-09-12, tag `m5`.** Retired the sketch as topology: a
profile is a `geom::Profile` value, and a sweep's faces enter the builder
through `Builder::assemble`, held to Pappus's theorems at a thousand
random profiles. No ADRs.

- A `geom::Profile` of lines and arcs with holes (no `ops::planar_face`:
  a one-face sheet is the healing cycle's); `ops::extrude` (planes and
  cylinders);
  `ops::revolve` (planes, cylinders, and cones/spheres/tori as *surfaces*
  where the profile demands them — their booleans are C2–C3's).
- Extrude and revolve provenance: side faces `Generated` from profile
  edges, caps from the profile face.
- The full C1 corpus run in CI, the corpus's ignored tests included, and
  zero ignored fixtures in `primitive/`, `transform/`, `boolean/`, `sweep/`,
  `provenance/` held by the corpus lint.

**Out:** sweep along a path, loft, draft; a revolve profile touching its
axis (C2).

**Accept:** every fixture of the C1 corpus passes every stage;
`/close-cycle` drift review empty; tag `m5` and `c1`.

### C1 acceptance corpus

The fixtures themselves: every directory under `tests/fixtures/`
`primitive/`, `transform/`, `boolean/`, `sweep/` and `provenance/`, its
recipe, purpose and closed forms in `fixture.json` (`description`,
`analytic`) and the oracle's answer in `expected.json`, run as one
`#[test]` each by `crates/arris/tests/corpus.rs`. The grammar and the
conventions the numbers assume are `tests/fixtures/README.md`'s.

---

## C2 — the application gate

*Goal: Arris covers what the first consumer's facade uses, and its probe
corpus — the recorded kernel failures with their `#[ignore]`d twins — is
green as fixtures. The gate is a corpus run, not a judgement call; the swap
itself is the consumer's, on its own schedule (ADR-0017).*

**Status: done 2026-09-19, tag `c2`, released as `v0.1.1`.** Retired the
application gate: the first consumer's probe shapes pass every corpus
stage in its own units, over blends built in closed form on analytic face
pairs, multi-shell results as lumps of one solid, and quadric faces decided
through their meridians on a shared axis, with the facade's three open
decisions taken. ADR-0006 to ADR-0017.

- Fillet and chamfer, several edges in one call: plane–plane edges with
  miters and three-blend corners, plane–cylinder edges along a ruling and
  around a rim, a second blend on a blended body (ADR-0007).
- Cone, sphere and torus in the intersector wherever two surfaces share an
  axis or a plane holds it, and a line against each (ADR-0008); the
  checker's S5 and B1 and `classify_point` over them, so M5's quadric-faced
  revolves are corpus fixtures.
- Cylinder–cylinder booleans: coaxial, parallel, tangent inside and out,
  equal radii crossing, whatever the tool's seam (ADR-0015, ADR-0016).
- Results of more than one shell as lumps of one `Solid` (ADR-0006); a
  revolve profile touching its axis.
- The facade's three decisions: no name grammar and a guaranteed split
  order (ADR-0009), `Model::retain` never renumbers (ADR-0010), `TriMesh`
  stays `f64` (ADR-0011).
- Plane projection, face frames, inertia matched to an independent
  integrator, the mesh corner block (ADR-0012), STL and OBJ export
  (ADR-0013).
- Elliptic profile segments, swept to an elliptic cylinder (ADR-0014); the
  workspace published from a tag.

**Out:** NURBS–NURBS intersection, sweep along a path, loft, shell, healing,
the STEP reader.

**Accept:** the consumer's probe corpus in its own units — through hole
(8.7434e-5), blind hole (1.8743e-4), enclosed cavity (9.36e-4, two shells),
cylinder − cylinder transversal (2.2079e-5), flush union (2.0), revolve
touching the axis (2π), a fillet on a filleted body, a vertical and a cap
edge filleted in one call.
Every one of them is a fixture in metres at the micrometre default
tolerance a metre model carries (`docs/ARCHITECTURE.md` §How a consumer's
kernel facade maps on, *Units*): the six
`probe-*-m` fixtures under `boolean/`, `blend/` and `sweep/`
(**done 2026-09-17**, the recipe's own `precision`), the enclosed cavity
as `boolean/enclosed-cavity` and the transversal as
`boolean/parallel-cylinders-cut`;
`sweep/revolve-frustum`, `revolve-barrel` and `revolve-ring` passing every
corpus stage with nothing left unchecked; every row of the facade table
(`docs/ARCHITECTURE.md` §How a consumer's kernel facade maps on) present.
The consumer's naming fixtures, its facade over Arris and its twins
un-ignored are the consumer's own acceptance, not this cycle's (ADR-0017).

---

## C3 — every quadric pair

*Goal: a boolean takes a cone, sphere, torus or elliptic-cylinder face as
an operand in any pose, because every quadric pair meets in the
intersector — closure: a body Arris built is a body Arris takes
(ADR-0020). Faces that meet within a tolerance, touching or coincident,
are a corpus of their own and not left to luck.*

**Status: done 2026-09-24, tag `c3`, released as `v0.2.0`.** Retired
closure: every pair of analytic surfaces meets in the intersector in
every pose, conics exact and the rest traced and fitted to NURBS under
one `Meets` result, `Unsupported` left only where a `Surface::Nurbs` is
in the pair; every quadric face is a boolean operand, a section through
an apex or a pole included; and "the same within a tolerance" is an
equivalence decided once per level. ADR-0018 to ADR-0022.

- The general quadric pairs in `intersect_surfaces`: the ruled pairs
  traced by their rulings and fitted inside a region (ADR-0018), every
  pair with a torus traced in the torus's parameter plane and fitted
  whole, a tube circle the other surface holds returned exact (ADR-0019).
- Conics against a cone, a sphere or a torus in `intersect_curve_surface`,
  two coplanar conics in `intersect_curves`, and a fitted `Curve::Nurbs`
  against every analytic surface, a line and a conic.
- A fitted pcurve on every analytic surface over its own projection,
  split at an apex or a pole; a section beside one refused by name
  (ADR-0021).
- The pave model's quadric guard lifted: the frustum, ball, ring,
  elliptic, filleted and chamfered `boolean/*` fixtures, and
  `quadric_operands_obey_every_identity`.
- S5 and B1 over every pair of analytic surfaces.
- Features a tolerance apart (ADR-0022): section vertices by closure,
  fits held to the exact branch, a section edge known by its surfaces, a
  block along an operand edge that edge's piece, a pinch refused as
  `InputReason::NonManifold`; `tolerance_band.rs` holding the band where it
  holds and ratcheting the rest, whose failures are `regression/`
  fixtures and `docs/BACKLOG.md` lines.

**Out:** NURBS operands and NURBS–NURBS intersection (the NURBS cycle's);
blends on quadric face pairs (the blend-network cycle's); a spindle
torus; a revolve of an elliptic segment; the STEP reader.

**Accept:** property tests at random poses of every quadric pair — every
intersection point on both surfaces within the tolerance, the curve's
image matching both surfaces at its samples; the boolean identities of M4
(volume additivity, cut-then-fuse, commutativity) over a quadric operand
against a box and a cylinder; every new `boolean/*` quadric fixture
passing every corpus stage against Open CASCADE;
`crates/arris-ops/tests/blend_prop.rs` with nothing unchecked in a pose;
`regression/seam-a-tolerance-from-crossing-fuse` moved into `boolean/`;
`docs/DATA-MODEL.md` with no `⚠ OPEN` left.

---

## C4 — the reader and the real-part corpus

*Goal: Arris reads a part it did not design, and every refusal it returns
over a public corpus of real parts is counted. That count, beside the
first consumer's side-by-side regressions, is what picks the cycle after
this one (ADR-0020). Chosen by rule, not measured into: nothing could be
measured until the reader existed. An accepted ADR that cites `C4` means
the NURBS cycle, read through ADR-0020's table, not this one.*

**Status: done 2026-09-26, tag `c4`, released as `v0.3.0`.** Retired the
unranked refusal: `arris_io::step::read` returns a checker-green body or
a counted `Refusal` per solid and placement; Open CASCADE's STEP of
every corpus fixture, and its B-spline conversion, read back to the
oracle's measures (168 corpus tests each, three files refused by name as
describing no solid, one variant Open CASCADE cannot convert); and 38 NIST
parts, 70 solids, read to 29 bodies within the oracle's measures and 41
refusals by kind, with no panic, each body read put through the battery. Six
fetched parts wait on kernel bugs under `regression/`. ADR-0023 to
ADR-0026.

- A Part 21 parser with a typed error carrying the entity id; the
  AP203/214/242 B-Rep subset onto Arris's own geometry, B-splines as
  `Nurbs`, provenance `Generated` from the file's entity (`Role::File`);
  assemblies flattened, AP242's saved views ignored (ADR-0025, ADR-0026).
- Pcurves and degenerate edges rebuilt rather than read, closed form on
  the analytic surfaces and fitted on a NURBS one, held inside its knot
  domain; each entity's tolerance measured from its own gaps, a gap past
  `READ_GAP_FRACTION` refused.
- A typed refusal for everything outside the subset, `RefusalKind::ALL`
  counted by the histogram.
- The real-part corpus (ADR-0026): the `part` fixture kind under `real/`
  (§Fixtures), the battery, `tools/real-parts.sh` over the fetched tier
  in the nightly, and the table mapping every refusal to the cycle it
  blocks, as exhaustive matches.

**Out:** sewing and repair, and open shells — healing is its own cycle;
booleans on NURBS faces (the NURBS cycle's); IGES; writing anything the
writer does not write today.

**Accept:** write → read round trip as a property, over the corpus's
shapes in random poses, to the entities' own tolerances; Open CASCADE's
STEP of every corpus fixture read back to the same counts, volume, area
and centroid the fixture asserts, `step_differs` fixtures skipped
(ADR-0023); a public corpus of real parts read
either to checker-green — mass properties within the fixture's tolerance
of the oracle's — or to a typed refusal, with no panic and no wrong
solid; and the refusal histogram over that corpus printed.

---

## C5 — the consumer's API

*Goal: a plugin-based CAD can start on Arris. A plugin's feature names
what it builds; a body and its provenance cross a process boundary and
survive in a user's file; an edit can stop an operation that is running,
on wasm too; a part can be mirrored; and an assembly keeps its products,
instances and names through STEP. Each is public API the consumer
designs its document model around, so it has to exist before that
consumer's first cycle rather than be found as its regressions later.*

*Chosen by ADR-0020's amendment of 2026-09-26: a consumer blocked on
missing API ranks first, as a consumer waiting on a swap does. The asks
are `docs/ideas/plugin-cad-consumer-asks.md`'s A1–A4 and A11. The
histogram's first line, the blend network (17 of 38 parts), became C6.*

**Status: done 2026-10-02, tag `c5`, released as `v0.4.0`.** Retired the
consumer's missing API: a plugin's feature names what it builds
(`Role::Consumer`, finished by `ops::build`), a body and its provenance
survive in a user's file (`arris_io::body`, versions frozen and migrated),
every operation stops on a poll or a step budget and rolls the model back,
ids included (the thin-elliptic section stops within 3.9 ms of a poll),
`ops::mirror` reflects a body in a plane, and `step::read` returns a
`ProductTree` that `step::write_products` writes back. ADR-0028 to
ADR-0033, ADR-0033 an amendment of ADR-0025 §Instances.

- Consumer roles (A1), body bytes (A2), cancellation (A3), mirror (A11) and
  the STEP product structure (A4): the asks of
  `docs/ideas/plugin-cad-consumer-asks.md` this cycle took.

**Out:** the rest of that idea — `region2` as public API, multi-tool
booleans and per-face tessellation are backlog lines for when the
consumer reaches them; the query and sweep cycles are its ranking input,
recorded and not applied. The first-party binding stands beside this
cycle; its body-bytes interop no longer waits on A2.

**Accept:** a body built under a consumer role, written as body bytes
and imported into a fresh model, is checker-green with the same counts,
measures and provenance, and the previous version's bytes read and
migrate; every operation interrupted at random points returns
`Interrupted` and leaves the model as it was, as a property; a mirrored
corpus fixture matches the oracle's mirror; an assembly's product tree
reads from Open CASCADE's XCAF STEP with the names and placements it
wrote, and Arris's own written assembly reads back to the same tree.

---

## C6 — the blend network

*Goal: a fillet or chamfer lands where a real part asks for one, not only
on ADR-0007's table of face pairs. Chosen by ADR-0020's rule, the
histogram over the real-part corpus first (no consumer waits on a swap
past the asks C5 closed): at C4's measurement (2026-09-26,
`tools/real-parts.sh`'s `both.md`: 38 parts, 70 solids, 29 read and 41
refused; 143 battery stages, 107 agree, 7 both refuse, 29 Arris refuses)
the blend network blocks 17 of the 38 parts, every one at the `fillet`
stage, and healing 14 at `read`, so the blend network is the larger by
three. The committed tier today (§Fixtures) blocks 6 of its 11.*

| Cycle | Parts blocked | read | measure | write_read | box_cut | drill_x | drill_y | drill_z | fillet |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| blend network | 17 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 17 |
| healing | 14 | 14 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| NURBS | 3 | 0 | 0 | 0 | 3 | 0 | 0 | 0 | 0 |
| itself: faceted | 1 | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| itself: supplemental geometry | 1 | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| itself: unparsed | 1 | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| sweep | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

**Status: done 2026-10-05, tag `c6`, released as `v0.5.0`.** Retired the
blend network: a fillet or chamfer follows a chain of line and arc edges
(ADR-0035); the cone, sphere and torus pairs along a coaxial circle blend
(ADR-0036); an end the face across cuts in no closed form is traced and
fitted, the surface exact (ADR-0037); a blend into a step lengthens its
corner edge (ADR-0038); a chain runs on through a four-edge vertex where both
faces turn, tangency read within the blend's size times the normals' sine
(ADR-0039, ADR-0040, ADR-0045); a split rim's two-edge vertex is a
continuation (ADR-0041); an end at a cusp with both walls on one side is cut
by the next wall (ADR-0042); an end across a fan, or a face met twice, is
built (ADR-0043); a miter of unequal dihedrals is two pieces and a trim arc
(ADR-0044). Measured with `tools/real-parts.sh` and `--census-committed`
(2026-10-04): the fetched tier's `fillet` column holds 2 of 27 parts, both
the NURBS cycle's (CTC-01's crossing cylinders, STC-07's NURBS surface), the
committed tier's 5 of 11; 0 failing parts; the fetched battery's 89 stages
79 agree, 4 both refuse, 6 Arris refuses; over both tiers the blend network
blocks 3 at `fillet`, beside C4's 17. Each plan's census is in its ADR. What
is left went to `docs/BACKLOG.md`: the pairs off the axis, a cylinder against
a sphere, the parallel-cylinder row, the overhang tip, the horn and spindle
torus at the axis, the ruling miter, the other chamfer modes, variable
radius and blends over blends.

- ~~Tangent edge chains blended as one (5 `Degenerate(TangentChain)`
  refusals).~~ Done: a stripe follows a chain of line and arc edges
  (ADR-0035), open or closed; the chain's refusals are named
  (`Unsupported` for a pair or an oblique end, `BlendTooLarge`,
  `TangentChain`).
- Blends on face pairs outside ADR-0007's table: ~~a circular edge where
  the table asks a line (circle × plane, 9)~~ done for a plane against a
  cylinder, an open arc or a closed circle, fillet and chamfer; plane ×
  ~~plane × cone, cone × cylinder or cone, a sphere or a torus against a
  coaxial plane, cylinder or cone~~ done (ADR-0036); cylinder × cylinder
  with parallel axes (ADR-0036 §5, no corpus edge), and crossing axes (the
  NURBS cycle's), torus × cylinder off its axis and torus × plane off its
  axis remain, with the elliptic cylinder: moved to the backlog.
- ~~A blend running into a step (`BlendTooLarge`, a mixed corner)~~ done
  (ADR-0038); what stays `BlendTooLarge` is the ring's contact at the axis
  (a horn or spindle torus) and radii Open CASCADE refuses; and a tangent
  corner the walk stops at (`TangentChain`, STC-09).
- The corners refused as `VertexBlend`: ~~a chain through a vertex of four
  edges~~ done (ADR-0039, ADR-0040); ~~the vertex of two edges of a split
  rim~~ done (ADR-0041); ~~the fan (CTC-01) and the face across met twice~~
  done (ADR-0043); ~~the miter of unequal dihedrals~~ done (ADR-0044);
ftc-06's dihedral jump on a collinear run and the tangent cylinder–sphere
edges are attributed (the NURBS cycle's; `TangentChain`, ADR-0045); the
ruling miter remains, on the backlog.
- The remaining chamfer modes (two distances, a distance and an angle):
  moved to the backlog.
- Variable radius, and blends over blends: moved to the backlog.

**Out:** NURBS faces as blend operands (the NURBS cycle's), a blend whose
surface has no closed form (the crease between equal-radius cylinders, the
overhang tip's caps: the NURBS cycle's), healing (the reader's 14 parts),
shell and offset (C7's, ADR-0047).

**Accept:** the committed tier's `fillet` column and the fetched tier's
printed beside C4's 17 of 38, every part leaving it either agreeing with
Open CASCADE's fillet within its fixture's tolerance or refused as another
cycle's; each new face pair a corpus fixture with its oracle, checker
green, and a property over random poses for every pair the intersector
now takes to a blend.

---

## C7 — prismatic features

*Goal: a hobbyist's prismatic part can be modelled on Arris end to end:
hollowed to a wall, its faces pushed and pulled, a pattern of holes cut in
one go, the body split in two. Chosen by ADR-0020's amendment (a consumer
blocked on missing API ranks first) over the histogram: at C6's close the
real-part histogram ranks healing first (14 of 38 parts at `read`), the
NURBS cycle 3 at `box_cut`, the `fillet` column 2 of 27 fetched and 5 of 11
committed; the first consumer, the plugin-based CAD, has no users because
it cannot model an ordinary mechanical part, and its recorded asks are
shell and offset (A7) and the multi-tool boolean (A8) of
`docs/ideas/plugin-cad-consumer-asks.md`. ADR-0047 split the sweep cycle to
take shell and offset without sweep and loft.*

**Status: opened 2026-10-05; scope set by the human the same day. The preparatory refactor landed the same day: the blend in `blend/` by phase over a crate-private `body_view`, the pave model in `pave/`, one periodic-parameter toolkit, direct crate dependencies, `Reason` grouped by operation, and the corpus as one table with a coverage lint.**

- Shell: a solid hollowed to a thickness, inward or outward, with the
  chosen faces removed as openings, or none for a closed void; the offset
  faces met by the intersectors every quadric pair already has, a vertex of
  three or more offset faces closed by their meeting.
- Offset faces (press-pull): chosen faces moved along their normals by a
  distance, their neighbours extended or trimmed to meet them; the whole
  body's offset is the case where every face moves.
- A boolean with many tools: `cut` and `fuse` of one body by N tools in one
  general fuse, one decomposition rather than N chained ones, provenance
  naming each tool (A8).
- Split by a plane: a body cut by a plane into the solids on either side,
  both kept, provenance naming the side each piece came from.
- Beside the cycle, a side plan: per-face incremental tessellation (A9), an
  edge's discretisation a pure function of the edge and the chord and
  `tessellate_faces` over a subset, so faces meshed at different times stay
  watertight (ADR-0010).

**Out:** sweep along a path and loft (the NURBS cycle's, ADR-0047); the
shell or offset of a NURBS face, and variable thickness (no exact kind
holds them: the NURBS cycle's); draft and thicken (backlog lines); a split
by a surface other than a plane, or by a body; healing; C6's blend residue
(backlog lines).

**Accept:** each operation a corpus of fixtures with Open CASCADE's answer
(its thick solid, its offset shape, its boolean with several tools, its
splitter), checker green, provenance complete; properties over random poses
and operands, blended bodies among them: a shell's volume against the body
less its inner offset where a closed form has one, a multi-tool cut against
the chained cuts (volume, area, counts), the pieces of a split summing to
the body and each equal to its common with the half-space; every result an
operand of fillet, chamfer and every boolean (ADR-0020 §1); an offset that
would drive a radius through zero or a face out of existence refused by
name. The side plan: faces meshed in any subsets and order give the same
edge polylines bit for bit as one call, and their union is closed.

---

## Beside the cycles

Two lines of work that are not cycles. Neither changes a public type or a
signature, so neither earns a minor version (`.agents/rules/git.md`
§Tags); each stands beside whatever cycle is open (ADR-0020).

**The lean gate** (ADR-0032). The pre-commit hook is a path-scoped `fast`
slice, about 110 s at worst against 312 s; the full suite runs once per plan
at retirement, in CI, nightly, at cycle close and at release.

**The measuring harness.** It measures what decides the cycle after C4
(ADR-0024). The oracle answers from a cache keyed on every input it
reads (`target/oracle-cache/`, bypassed by `ARRIS_ORACLE_CACHE=off` in CI
and nightly): the corpus binary takes 10.3 s cold and 1.3 s warm, and a
warm run starts no Python. Random recipes over the twelve operations
`prop::recipe` draws from (every op both interpreters carry but `step`,
a solid read from a file) run through both kernels in the
differential. At 1000 draws of the fixed seed (measured 2026-10-02), 792 reach a
comparison and agree, 156 both refuse, 7 are refused by Open CASCADE, and
39 by Arris (all `Degenerate(Empty)`). 6 are under named exclusions, each
waiting on a `regression/` fixture, and none fails. That is 0.165 s of
wall clock per recipe. The hook runs 32 cases over what a commit reaches (ADR-0032), a plan's retirement 256 and CI 1000, all on the fixed seed;
depth comes from `nightly.yml`, which runs every property at 5000 cases
on a seed drawn from the date, 99,600 CPU-seconds split over six jobs,
plus the differential at 1000 recipes on the same seed. The corpus
benchmark times 316 cases on the reference machine: 282 from 141
fixtures, build 4.56 s and mesh 3.25 s, and 34 from 17 real parts' files,
read 21.63 s (docs/ARCHITECTURE.md §Formats and tools). Each night compares against the
last, flagging a case past 3×. Three fuzz targets over the intersectors
(`fuzz/`, outside the workspace) are seeded from every geometry pair.
Their first hour, on 24 cores after the three faults a triage run
found were fixed, reached 25, 42 and 28 executions per second with no
crash, slowed by sections against very thin elliptic cylinders. Each night runs 30
minutes per target from the corpus the nights before grew; the first
night found a fourth, two planes a hair from parallel meeting in a line
with a NaN origin. A night takes about 5 hours: its longest property
job, `workspace-rest`, took 4 h 46 min on 2026-10-01 and 5 h 19 min on
the three nights before, against the workflow's 350-minute timeout. A red night is a finding to triage, not a gate; CI's
fixed seed is the gate (ADR-0024, amendment of 2026-09-25).
A fourth, `step_read`, runs the STEP reader on Arris's and Open
CASCADE's STEP of every solid fixture: over the parser alone its first
minutes found a page directive that swallowed a line break, and its
first hour after the fix, on 24 cores, ran 298 million inputs with no
crash; over the whole reader, fifteen minutes on sixteen cores ran 11.4
million with none. A fifth, `body_read`, runs `body::read` on any bytes,
seeded from the guard's files: every input is a checker-green body or a
typed error, and the model it reads into is unchanged on an error.

**The first-party binding.** Code-first and agent-driven modelling is one
of the consumers `SEED.md` §1 names, and a binding in this repository is
how that consumer exists before an application is written on top of it.
**Landed (2026-10-02):** `crates/arris-py`, a thin 1:1 layer above the
facade (ADR-0034), published to PyPI as `arris` in lockstep from the same
`v*` tag. It binds the operations with cancel, budget and Ctrl-C, profiles,
measures, the checker's report, adjacency walks, tessellation as mesh
bytes, and STEP, body bytes, native, STL and OBJ; every error is a typed
exception and a handle carries its model. What it waits on: the human's
`pypi` environment and pending-publisher registration, the first upload,
and the platforms of the wheel beyond Linux x86_64. Its own backlog lines
(corner blocks, enumerating a model's bodies, a record naming foreign
inputs) are in `docs/BACKLOG.md`.

---

## Named cycles, unordered

None is scheduled, and the order below carries no meaning; an entry
says so where a ranking has already put it next. The next cycle is picked from two numbers: the refusal histogram over
the real-part corpus, and the first consumer's side-by-side run of its
suite against both backends (ADR-0017). Where they disagree, the
consumer's regressions rank first while there is a consumer waiting on a
swap, and the histogram after (ADR-0020). Each cycle earns its own
section, with an acceptance corpus and a number, when `/close-cycle`
opens it.

- **The NURBS cycle** — NURBS–NURBS surface intersection (a marcher with
  explicit seam handling); NURBS operands in booleans; the operations that
  build free-form faces, sweep along a path and loft (ADR-0047 split them
  from shell and offset, which are C7's).
- **The healing cycle** — healing; sheet and wire bodies in every
  operation.
- **The query cycle** — distance, clash, ray fire, selection; none of
  them has its machinery yet, since nothing in the kernel holds a
  hierarchy over face boxes or fires a ray at a surface outside
  `classify_point`.
- **The attribute cycle** — attributes a consumer attaches to entities,
  carried through every operation by declared rules over the split order
  ADR-0009 fixes, for a consumer with no naming scheme of its own
  (Parasolid's attribute definitions).
- **The breadth-and-speed cycle** — IGES, read and write; same-domain face
  merging; the performance pass the design reserved room for.

## What not to spend agent time on

A sketch constraint solver, rendering, UI, physics, drawings, mesh
processing beyond tessellation, formats beyond STEP and the native one,
and speed as a headline (`SEED.md` §4 non-goals). A backlog line that
lands in one of these is rejected with that reference.
