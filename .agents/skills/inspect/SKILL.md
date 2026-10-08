---
name: inspect
description: See a shape, an intersection curve or a failed operation without a GUI — dump it as text, render it to a PNG you can read, run the invariant checker on it, compare it against the Open CASCADE oracle, or stream it to Rerun for the human. Use when working on anything geometric and you need to check the result rather than reason about it; when a fixture or property test fails; when asked what a shape looks like; and to turn a failure into a fixture.
---

# Seeing geometry without a GUI

Look at the shape instead of reasoning about coordinates from source. Never
ask the human to describe one — they have no better tool than you.

## Is it valid? — the checker

`arris_check::check(&model, body, Level::Fast)` after every step,
`Level::Full` before calling a body done (`docs/DATA-MODEL.md`
§Invariants). One line per violated invariant, sorted by entity;
`Report::unchecked()` lists `Full` rows it could not decide (`?`),
`Report::euler()` the Euler line. **Read this first** — most "wrong
picture" bugs are a checker line.

## What is it, exactly? — the dump

`arris_debug::dump_text(&model, body)`: precision, shells, faces, loops and
coedges with effective orientations (`+f0`, `-e1`; a seam edge appears
twice), surfaces and pcurves, edges with curves and ranges, vertices, and
`euler V/E/F/L/S g<G> = <residual>`. 12 decimals, deterministic, diffable —
a fixture's `dump.txt`. Valid bodies to compare against:
`arris_debug::sample::{cuboid, cuboid_nurbs, cylinder, frame}` (`frame` is
genus 1 with two-loop faces).

## What does it look like? — the PNG

- `arris_debug::render_body(&model, body, View::Iso, highlight, "name")`
  for a body; `render_png(&mesh, &polylines, view, highlight, "name")` for a
  bare `TriMesh` and `Polyline`s. Geometry without a body (an intersection
  curve, a surface under test): `polyline_of(&curve, range, n)`,
  `wireframe_of(&surface, domain, n)`.
- Output: orthographic 800×600 at `target/inspect/<name>.png`. **Read it
  with the Read tool.** One flat colour per face id (shading can split a
  curved face — highlight it, don't count colours), edges black with hidden
  parts hidden, dots at edge ends, `Some(Highlight::Face(id) | Edge(id) |
  Point(p))` in red. `View::{Iso, Top, Front, Right}`; `Iso` looks from
  (+1, −1, +1).
- `render_body` meshes via `mesh_of(&model, body)` at a chord from the
  bounding-box diagonal — never pass it a model tolerance.
- A face whose *mesh* is the suspect: `render_domain(&model, face, "name")`
  draws its (u, v) loops and triangulation — the CDT never sees 3D.
- `arris_debug::render` returns the pixel buffer for a test that counts
  colours.

## Where does a boolean go wrong? — the pave model

`arris_ops::boolean::interferences(&model, a, b)` — a query, no body, no
provenance — is the decomposition every boolean selects over (ADR-0004).
Print it with `{}` and read the pave, not the result. It lists: face pairs
with their `SurfaceIntersection`; edge-on-face hits with their `Landing`
and section vertex; touches resolved into crossings (ADR-0016); section
crossings and triple points with their vertices; section vertices with
tolerances and sources (`singular`, ADR-0021); paves on every edge and
section curve; section edges with range, ends and a pcurve per face (`p<pair>
on <face>` where pairs share a block); touching-curve contacts; coincident
edge–face pairs; and for `Coincident` face pairs the edge–edge crossings,
images and common blocks.

Reading it:

- a missing section edge — a block whose midpoint was not `Inside` both
  faces; a wrong count — a hit two faces' polygons decided differently;
- a flush face that survived — a piece classified `Inside`/`Outside` that
  should be `On`: an image missing from that face's list;
- `BuildError::EdgeUses` after a flush operation — a common block not found
  (`Fault::CommonBlock` when paves disagree);
- `TangentContact` — a contact segment where both pieces would survive;
- `OpError::Unsupported` — a pair with no closed form, naming both;
  `Fault::Seam` — a section edge crossing a seam without a pave, a kernel
  bug.

To draw it: `polyline_of(&i.curves[s.curve].curve, s.range, n)` per section
edge `s`, `render_png` over `mesh_of` one operand.

## Is it right? — the oracle

`arris_debug::oracle::compare("<area>/<slug>", &step_text, variant, tag)`,
or `uv run --project tools/oracle tools/oracle/compare.py <fixture-dir>
<file.step> [--variant NAME]` on `arris_io::step::write(&model, &[body])`
output: volume, area, centroid, counts, genus and probe classifications
against Open CASCADE's `expected.json`; exit 1 / `OracleError::Mismatch`
with the table. `expected.py <dir>` regenerates the answer, `selftest.py`
proves the oracle. The whole chain for one fixture — checker at `Full`,
counts, oracle, provenance, dump diff — is `cargo test -p arris --test
corpus <name>`; `ARRIS_BLESS=1` writes `dump.txt` instead of diffing. Needs
`tools/oracle/.venv` (`uv sync --project tools/oracle`); outside it every
script fails on its first line, never skips.

## Other inputs

- **A STEP file** (a consumer's, `target/inspect/occt-*.step`, a fuzz
  crash): `cargo run -p arris-debug --example inspect_step -- <file.step>
  [name]` prints each solid's `Full` report and dump and renders
  `target/inspect/<name>-<id>-<instance>.png`; a refused solid prints its
  `Refusal`. From Rust: `arris_debug::step_file::inspect(&mut model, path,
  name)`.
- **A real part**: `cargo test -p arris --test corpus real_<slug>` (dashes
  → underscores) runs `tests/fixtures/real/<slug>/`; `inspect_step` its
  STEP file to look. `expected.json` holds Open CASCADE's healed reading per
  `#id`, `occt_heals` where healing changed topology. Fetched parts:
  `target/real-parts/files/`, logs in `target/real-parts/work/<stem>.log`.
- **A refused sketch** (`ProfileError` names `loop_index`, 0 = outer, and a
  segment): `Profile`'s fields are public — map each loop's (u, v) to `[u,
  v, 0.0]` (line ends; arc start, `via()`, `end()`; a sampled circle), one
  `Polyline` per loop, `render_png(&TriMesh::new(), &polylines, View::Top,
  Some(Highlight::Point(start)), "name")`. For a revolve axis refusal
  (`ProfileCrossesAxis`, `SpindleTorus`, full-turn `NonManifold`) add the
  axis, projected by dotting offsets from `plane` with its `X` and `Y`.
- **For the human**: `arris_debug::rerun::{spawn, log}` (`rerun` feature)
  streams faces, edges and vertices as separate entity paths. Only when
  they ask — the PNG is what you can read.

## Reading a picture

A picture says something is off; a number says what.

1. `dump_text` and grep the entity the picture points at.
2. Re-render with it highlighted, inputs beside it — is it wrong, or its
   neighbour?
3. Check the *inputs*: an invalid operand gives an invalid result.

A shape the tessellator cannot mesh renders as edges only — a finding, not
a rendering bug.

## From a failure to a fixture

1. **Reproduce** in a test, operands from primitives or STEP.
2. **Shrink**: fewer faces, rounder numbers, axis-aligned if it still
   fails. Stop when one more simplification passes — that boundary
   describes the bug.
3. **Save** under `tests/fixtures/regression/<slug>/` (areas hold passing,
   blessed fixtures only): `fixture.json` (format in
   `tests/fixtures/README.md`), `expected.json` from `expected.py`, and
   `regression_<slug>` in `crates/arris/tests/corpus.rs` with the desired
   assertion and `#[ignore = "<what fails>"]`.
4. **Commit** `test(<scope>): fixture <slug>` with the failure's shape in
   the body; the fix is a plan step or backlog line. The fixing commit
   moves it into its area, blesses the dump, renames the test
   `<area>_<slug>` and un-ignores it.

**Property tests**: `arris_debug::prop::check` prints the shrunk case and
`ARRIS_PROPTEST_SEED=…`; the seed goes in the fixture's commit body. A
nightly prints its seed in the job summary — rerun locally at it and the
job's `ARRIS_PROPTEST_CASES`.

**Differential** (`crates/arris/tests/differential.rs`): prints the case
already shrunk as a `fixture.json`; commit it under `regression/<slug>/`
with plain `expected.py`. If the draw keeps hitting it, add a named
exclusion to `differential::EXCLUSIONS` citing the slug — the property
tests share it (`exclusion_of_panic`, `exclusion_of_error`).

**Fuzz** (`fuzz/`, ADR-0024 §5): a crash under `fuzz/artifacts/<target>/`
(uploaded by the nightly). `cargo run --manifest-path fuzz/Cargo.toml
--example show -- <target> <file>` decodes it; `cargo +nightly fuzz tmin -s
none <target> <file>` shrinks it. The operands become an `#[ignore]`d test
beside the intersector's own; once passing they may join a `geom/`
geometry fixture, which `crates/arris-geom/tests/oracle.rs` reads.
