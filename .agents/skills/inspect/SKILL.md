---
name: inspect
description: See a shape, an intersection curve or a failed operation without a GUI — dump it as text, render it to a PNG you can read, run the invariant checker on it, compare it against the Open CASCADE oracle, or stream it to Rerun for the human. Use when working on anything geometric and you need to check the result rather than reason about it; when a fixture or property test fails; when asked what a shape looks like; and to turn a failure into a fixture.
---

# Seeing geometry without a GUI

You can look at a shape. Do that instead of reasoning about coordinates from
source. Never ask the human to describe what a shape looks like — the human
has no better tool than you do until Arris sits behind a CAD application.

## The four questions, and which tool answers each

| Question | Tool | Output |
|---|---|---|
| Is it valid? | `arris-check` — `arris_check::check(&model, body, Level::Fast)` after every step, `Level::Full` before calling a body done (every row of `docs/DATA-MODEL.md` §Invariants) | Every violated invariant with the entity that violates it, one line each, sorted by entity; `Report::unchecked()` lists the `Full` rows the kernel could not decide (printed with a `?`), and `Report::euler()` the Euler line. Read this first; most "wrong picture" bugs are a checker line. |
| What is it, exactly? | text dump — `arris_debug::dump_text(&model, body)` | Precision, then shells, faces, loops and coedges in iteration order with effective orientations (`+f0`, `-e1`; a seam edge appears twice in its loop), surfaces and pcurves written out, then edges with curves and ranges, vertices, and the Euler line `euler V/E/F/L/S g<G> = <residual>`; 12 decimals, deterministic, diffable, what a fixture stores as `dump.txt`. `arris_debug::sample::{cuboid, cuboid_nurbs, cylinder, frame}` are valid bodies to compare a suspect one against (`frame` is genus 1 with two-loop faces, built through the Euler operators). |
| What does it look like? | PNG — `arris_debug::render_body(&model, body, View::Iso, highlight, "name")` for a body with topology, `render_png(&mesh, &polylines, view, highlight, "name")` over a bare `TriMesh` and `Polyline`s otherwise (`arris_debug::polyline_of(&curve, range, n)` and `wireframe_of(&surface, domain, n)` give the polylines of geometry that has no body yet — an intersection curve, a surface under test) | Orthographic 800×600 render to `target/inspect/<name>.png`, one flat colour per face id (a curved face's own shading splits it into several — highlight it to see its extent, don't count colours), edges black with hidden parts hidden, dots at edge ends, `Some(Highlight::Face(id) / Edge(id) / Point(p))` in red. `View::{Iso, Top, Front, Right}`; `Iso` looks from (+1, −1, +1) so +x, −y and +z faces are visible. `render_body` meshes through `arris_debug::mesh_of(&model, body)`, at a chord found from the body's own bounding-box diagonal — never pass it a model tolerance. When a face's *mesh* is the suspect, not its position, `arris_debug::render_domain(&model, face, "name")` draws its (u, v) loops and triangulation instead — the CDT never sees 3D, so this is the debugger for a face that comes out wrong. **Read the PNG with the Read tool** — it renders as an image. `arris_debug::render` returns the pixel buffer for a test that counts colours. |
| Where does a boolean go wrong? | pave model — `arris_ops::boolean::interferences(&model, a, b)` (a query: `&Model`, no body, no provenance) | The decomposition every boolean is a selection over (ADR-0004), as a value with a `Display`: every face pair whose boxes overlap with its `SurfaceIntersection`, every edge-on-face hit with its `Landing` (interior, or which edge or vertex of the face it crossed) and the section vertex it merged into, a touch that landed on no vertex listed beside the crossings the section curves resolved it into (ADR-0016), every section crossing (two section curves of one pair crossing each other on both faces — the two ellipses of equal crossing cylinders — with its vertex), every triple point (a section curve of two operands' faces crossing a face of a third — two overlapping holes' circles on a plate's top — with its vertex), the section vertices with their tolerances and sources (`singular` for an apex or a pole a section runs through where no edge pierces, ADR-0021), the paves on every edge and every section curve — a degenerate edge's at each `u` a section arrives at its vertex with, the section edges with their range, ends and a pcurve on each face, the contacts of every touching curve (a block of it interior to both faces, with its midpoint and (u, v) on each — the segment the boolean decides by the curvature rule, and refuses as `TangentContact` when both pieces through it would survive), the coincident edge–face pairs, and for every `Coincident` face pair its edge–edge crossings, the images (a piece of one face's edge inside the other face, with its pcurve there) and the common blocks (a piece of `b`'s edge that is a piece of `a`'s, held once). Print it with `{}` and read the pave, not the result: a missing section edge is a block whose midpoint was not `Inside` both faces, a wrong count is a hit the face's polygons decided differently from the other face's; a flush face that survived is a piece whose interior point classified `Inside` or `Outside` where it should have been `On` — an image missing from that face's list — and a `BuildError::EdgeUses` after a flush operation is a common block that was not found (`Fault::CommonBlock` when the paves disagree). To draw it, `polyline_of(&i.curves[s.curve].curve, s.range, n)` over each section edge `s` and `render_png` them over `mesh_of` one operand — the section curves lie on the operand's faces where the other one crosses them. A pair or an edge–face pair with no closed form is `OpError::Unsupported` naming both; a section edge crossing a seam without a pave is `Fault::Seam`, a kernel bug. |
| Is it right? | oracle — `arris_debug::oracle::compare("<area>/<slug>", &step_text, variant, tag)` from Rust, or `uv run --project tools/oracle tools/oracle/compare.py <fixture-dir> <file.step> [--variant NAME]` on a file from `arris_io::step::write(&model, &[body])` | Arris's volume, area, centroid, counts, genus and probe classifications (read from the STEP it wrote) against Open CASCADE's `expected.json`, within the fixture's tolerances; a table, exit 1 on mismatch (`OracleError::Mismatch` with the table from Rust). `expected.py <dir>` regenerates the oracle's answer, `selftest.py` proves the oracle against itself. The whole chain for one fixture — checker at `Full`, counts, oracle, provenance accounting, dump diff — is `cargo test -p arris --test corpus <name>`; `ARRIS_BLESS=1` writes `dump.txt` instead of diffing it. |

A STEP file — a consumer's, Open CASCADE's (`target/inspect/occt-*.step`
after a corpus run), a fuzz crash — is seen the same way:
`cargo run -p arris-debug --example inspect_step -- <file.step> [name]`
reads every solid (an assembly's at each placement), prints its checker
report at `Full` and its dump, and renders it to
`target/inspect/<name>-<id>-<instance>.png`; a solid the reader refuses
prints its `Refusal` and kind instead. From Rust it is
`arris_debug::step_file::inspect(&mut model, path, name)`.

A real part is seen by its fixture name. `cargo test -p arris --test
corpus real_<slug>` (dashes as underscores: `real_nist_ctc_01`) runs
`tests/fixtures/real/<slug>/` whole: every
solid held to its recorded outcome, then its battery. To look at one,
run `inspect_step` on the STEP file its `fixture.json` names, beside it
in that directory, with `<slug>` as the name. Its `expected.json` holds
Open CASCADE's healed reading of each solid by `#id`, and `occt_heals`
says where healing changed the topology. A fetched part is seen the same
way from `target/real-parts/files/`, and its survey's log is
`target/real-parts/work/<stem>.log`.

A sketch that `Profile::edges` refuses has no edges to dump or render, and
its `ProfileError` names a loop and a segment in the consumer's own
indices (`loop_index` 0 the outer, holes from 1). Draw the raw loops in
the sketch's own plane: `Profile`'s fields are public, so walk
`outer` and `holes`, map each (u, v) to `[u, v, 0.0]` — a line segment
its two ends, an arc its start, `via()` and `end()`, a circle loop a
ring sampled about its centre — build one `Polyline::new(points)` per loop,
and `render_png(&TriMesh::new(), &polylines, View::Top,
Some(Highlight::Point([u, v, 0.0])), "name")` with the named segment's
start as the highlight. `View::Top` looks down the sketch's normal, so the
picture is the sketch as it was drawn. For a revolve's axis refusal
(`SweepReason::ProfileCrossesAxis`, `SpindleTorus`, a full turn's `NonManifold`) add
the axis as one more polyline, its origin and a far point taken into the
plane's (u, v) by dotting their offset from `plane` with its `X` and `Y`.

For the human: `arris_debug::rerun::{spawn, log}` (the `rerun` feature)
spawns a Rerun viewer and streams a body's mesh to it — a `Mesh3D` per
face under `<body>/faces/<face>`, a `LineStrips3D` per edge under
`<body>/edges/<edge>`, one `Points3D` of every mesh vertex under
`<body>/vertices` — as separate entity paths so each layer toggles in the
viewer. Use it when the human asks to see something; do not use it to
convince yourself — the PNG is what you can read.

## Reading a picture

A picture says *something* is off; a number says what. After a PNG shows a
missing face or a spike:

1. `dump_text` the shape and grep the entity the picture points at — the
   face's surface parameters, its wire's edges, each edge's tolerance.
2. Render again with that entity highlighted, and the input shapes beside
   it, to see whether the entity is wrong or its neighbour is.
3. Run the checker on the *inputs*. A boolean on an invalid input produces
   an invalid output with an unhelpful picture.

## From a failure to a fixture

Every geometry failure becomes a fixture (`.agents/rules/kernel.md`):

1. **Reproduce** in a test with the operands built from primitives or
   loaded from STEP.
2. **Shrink**: fewer faces, rounder numbers, an axis-aligned pose if the
   failure survives it. Stop when one more simplification makes it pass —
   that boundary is the bug's description.
3. **Save** under `tests/fixtures/regression/<slug>/`, not in its area —
   the corpus lint holds `primitive/`, `build/`, `transform/`, `boolean/`, `sweep/`,
   `provenance/` and `blend/` to passing, blessed fixtures: the recipe
   (`fixture.json`, format in `tests/fixtures/README.md`), the expected
   values (`expected.json`, from `expected.py`), and the test
   `regression_<slug>` in `crates/arris/tests/corpus.rs` with the
   *desired* assertion, `#[ignore = "<what fails>"]`.
4. **Commit** it as `test(<scope>): fixture <slug>` with the failure's
   shape in the body. The fix is a plan step or a backlog line; the
   fixture is committed either way. The commit that makes it pass moves
   the directory into its area, blesses its dump (`ARRIS_BLESS=1`) and
   renames and un-ignores its test (`<area>_<slug>`).

## Property-test failures

`arris_debug::prop::check` prints the shrunk case and the
`ARRIS_PROPTEST_SEED=…` that reproduces the run. Turn it into a fixture the
same way; the seed goes into the fixture's commit body so the original
case is reproducible even after the shrinker or the strategy changes.
A nightly failure prints the night's `ARRIS_PROPTEST_SEED=…` in its job
summary. Run the failing test locally at that seed and the job's
`ARRIS_PROPTEST_CASES`.

A **differential** failure (`crates/arris/tests/differential.rs`) prints
its case already shrunk as a `fixture.json`, with the tolerances it was
judged at. Commit that under `regression/<slug>/`, and write its
`expected.json` with plain `expected.py`. When the draw keeps reaching the
failure, add a named exclusion to `differential::EXCLUSIONS` that covers
its symptom and cites the slug. The property tests hold the same list,
for a panic (`exclusion_of_panic`) and for a typed fault
(`exclusion_of_error`), so one fix lifts both.

A **fuzz** failure (`fuzz/`, ADR-0024 §5) is a crash file under
`fuzz/artifacts/<target>/`; the nightly uploads it as an artifact.
`cargo run --manifest-path fuzz/Cargo.toml --example show -- <target>
<file>` prints the operands it decodes to and the intersector's answer.
`cargo +nightly fuzz tmin -s none <target> <file>` shrinks it. The
operands then go into a test beside the intersector's own tests, as the
first fuzz findings did (`Frame::new`'s, the knot rule's, the oblique
elliptic section's), `#[ignore]`d until the fix lands, and once it
passes they can join a `geometry` fixture under `geom/`, so the oracle
holds them too. `crates/arris-geom/tests/oracle.rs` reads every
geometry fixture, so there is no waiting area for a failing one.

## Limits

- The PNG renderer is a software rasteriser of the *tessellation*. A shape
  the tessellator cannot mesh renders as its edges only — that is itself a
  finding, not a rendering bug.
- The oracle needs `tools/oracle/.venv` (`uv sync --project tools/oracle`;
  `tools/oracle/README.md`). Run outside it, every script fails on its
  first line with that command, never skips.
