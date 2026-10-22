<p align="center">
  <img src="https://raw.githubusercontent.com/arris-labs/arris/main/docs/assets/arris.svg" width="160" alt="arris logo">
</p>

<h1 align="center">arris</h1>

<p align="center">
  <a href="https://github.com/arris-labs/arris/actions/workflows/ci.yml"><img src="https://github.com/arris-labs/arris/actions/workflows/ci.yml/badge.svg?branch=main" alt="CI"></a>
  <a href="https://crates.io/crates/arris"><img src="https://img.shields.io/crates/v/arris.svg" alt="crates.io"></a>
  <a href="https://pypi.org/project/arris/"><img src="https://img.shields.io/pypi/v/arris.svg" alt="PyPI"></a>
  <a href="#licence"><img src="https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg" alt="License: MIT OR Apache-2.0"></a>
  <img src="https://img.shields.io/badge/rustc-1.85%2B-orange.svg" alt="rustc 1.85+">
</p>

<p align="center">
  <b>A B-Rep geometric kernel in pure Rust, for code-CAD scripts, AI agents and CAE pipelines.</b>
</p>

An *arris* is the sharp edge where two surfaces meet — the thing a B-Rep
kernel exists to compute.

Arris is a library, not an application. It builds solids, cuts and fuses
them, blends their edges, measures, tessellates and exports them, from Rust
or from Python. Four things set it apart:

- **Every operation says what it did.** It returns the new body and a
  provenance record — the faces, edges and vertices it generated, modified
  and deleted — so a script can pick "the edge the extrude made" instead of
  an index that shifts when the model changes.
- **It refuses rather than guesses.** Geometry it cannot do is a typed error
  naming the entities involved, never a panic and never an invalid shape;
  every result passes the invariant checker. Every vertex, edge and face
  carries its own tolerance instead of one global epsilon.
- **It is deterministic.** Same input, same output, same entity ids, on
  every platform: a body built for `wasm32` is byte-identical to the native
  one.
- **It is pure Rust.** No `unsafe`, no C++ underneath, one crate to depend
  on, and a Python wheel built from the same code.

> Pre-1.0: the API breaks between minor versions. [CHANGELOG.md](CHANGELOG.md)
> lists every change, and every break with its fix.

## Install

**Python** — a prebuilt wheel, no Rust toolchain needed:

```sh
pip install arris
```

**Rust** — one crate, `arris`, re-exports the whole kernel; the API is on
[docs.rs](https://docs.rs/arris):

```sh
cargo add arris
```

## Example: a bracket in Python

<p align="center">
  <img src="https://raw.githubusercontent.com/arris-labs/arris/main/docs/assets/showcase.png" width="480" alt="Motor-mount bracket modelled in Arris, faces coloured by provenance">
</p>

```python
# A motor-mount bracket: extruded L profile, fused boss, bore and bolt pattern cut
# in one call, fillets on the bend and hole rims, and a chamfer on the boss.
# Full script with per-operation face colouring: crates/arris-py/examples/bracket.py

import arris
from arris.build import Solid, Workplane

# 1. Extrude the L plate, then fuse a boss on the upright leg
plate = Workplane.XZ().polygon([(0, 0), (60, 0), (60, 8), (8, 8), (8, 50), (0, 50)]).extrude(-40)
boss = Solid.cylinder((8, 20, 32), (1, 0, 0), 14, 12)
boss = boss.tag("boss", boss.faces())  # a tag follows its faces through every later operation
part = plate.fuse(boss)

# 2. Cut bore, bolt holes and counterbores in one multi-tool call
bore = Solid.cylinder((-1, 20, 32), (1, 0, 0), 8, 23)
holes = [Solid.cylinder((40, y, -1), (0, 0, 1), 3.5, 10) for y in (8, 32)]
wells = [Solid.cylinder((40, y, 5), (0, 0, 1), 6, 4) for y in (8, 32)]
part = part.cut(bore, *holes, *wells)

# 3. Fillets and chamfer: edges selected by provenance, never by index
bend = part.edges().from_role(arris.Role("extrude", "Rise", 0, 3)).one()
part = part.fillet([bend], 4)
skin = part.faces().kind("plane").normal((0, 0, 1)) - part.faces().created_by(wells[0]) - part.faces().created_by(wells[1])
walls = (part.faces().created_by(wells[0]) | part.faces().created_by(wells[1])).kind("cylinder")
rims = walls.edges() & skin.edges()
part = part.fillet(rims, 0.8)
crown = part.faces().tagged("boss")
lip = (crown.kind("plane").edges() & crown.kind("cylinder").edges()).one()
part = part.chamfer([lip], 1.5)
```

### The binding beneath

`arris.build` is pure Python over the binding, `arris.Model`, which stays
available for anything the layer does not wrap:

```python
# A motor-mount bracket: extruded L profile, fused boss, bore and bolt pattern cut
# in one call, fillets on the bend and hole rims, and a chamfer on the boss.
# Every face is coloured by the operation that made it, followed through provenance.
# The same part on the raw binding; the layer above it is the first snippet.

import arris

model = arris.Model()

# 1. Extrude the L plate
profile = arris.Profile(
    arris.Loop.polygon([(0, 0), (60, 0), (60, 8), (8, 8), (8, 50), (0, 50)]),
    origin=(0, 0, 0), normal=(0, -1, 0), x_axis=(1, 0, 0),
)
plate, plate_rec = model.extrude(profile, (0, 1, 0), 40)

# 2. Fuse a boss on the upright leg
boss, boss_rec = model.primitive_cylinder((8, 20, 32), (1, 0, 0), 14, 12)
body, fuse_rec = model.fuse(plate, boss)

# 3. Cut bore, bolt holes, and counterbores in one multi-tool call
wells = [model.primitive_cylinder((40, y, 5), (0, 0, 1), 6, 4) for y in (8, 32)]
tools = [
    model.primitive_cylinder((-1, 20, 32), (1, 0, 0), 8, 23)[0],
    *(model.primitive_cylinder((40, y, -1), (0, 0, 1), 3.5, 10)[0] for y in (8, 32)),
    *(w for w, _ in wells),
]
body, cut_rec = model.cut_many(body, tools)
history = plate_rec.then(fuse_rec).then(cut_rec)

# 4. Fillets and chamfer: edges selected by provenance
[bend] = [e for e in history.generated_from(arris.Role("extrude", "Rise", 0, 3)) if isinstance(e, arris.Edge)]
body, _ = model.fillet(body, [bend], 4)

rims = [
    e for _, w_rec in wells
    for f in w_rec.generated_from(arris.Role("cylinder", "Wall"))
    for e in cut_rec.generated_from(f)
    if isinstance(e, arris.Edge)
]
body, _ = model.fillet(body, rims, 0.8)

[lip] = [e for e in boss_rec.generated_from(arris.Role("cylinder", "TopRim")) if isinstance(e, arris.Edge)]
body, _ = model.chamfer(body, [lip], 1.5)
```

## What it does

| Area | What you get |
|---|---|
| **Modelling** | Primitives, rigid transforms and mirroring; extrude (in any direction) and revolve of profiles of lines, arcs and elliptic arcs; offset faces, shell to a wall, split by a plane; a profile as a one-face sheet body |
| **Sweep and loft** | Sweep a profile along lines, arcs, helix arcs and B-spline curves, with rotation-minimising or fixed-binormal frames; loft through two or more profiles, ruled or smooth |
| **Booleans** | Cut, fuse and common, one tool or many in one call, in any pose, over planes, cylinders, elliptic cylinders, cones, spheres, tori and NURBS faces — free-form against analytic and free-form against free-form; results with cavities, split pieces and disjoint shells |
| **Fillet and chamfer** | Constant radius along chains of edges, with mitres, corners, steps and hole rims; on free-form faces and between two of them, the blend a NURBS surface within your tolerance of the exact one |
| **Queries** | Ray hits, minimum distance, interference with the shared volume, clash screening of an assembly, the planar section of a body; volume, area, centroid and inertia; face frames and plane projection. None of them changes the body |
| **Meshing** | Tessellation to a chord tolerance, per face or whole, faces meshed at different times welded together; NumPy arrays from Python |
| **STEP** | Every solid or surface model in a file read as a checked body or a typed refusal naming why; small gaps closed and reversed faces turned back, each repair named on the body; assemblies; AP214 export in the unit you choose |
| **Other formats** | STL and OBJ export; a deterministic native format in JSON and bytes, which every later release reads |

## Not yet

Each of these is a typed refusal today, never a wrong answer:

- variable-radius fillets and chamfers;
- offset and shell of elliptic-cylinder and NURBS faces;
- a NURBS face lying flush against another face over an area;
- sewing the faces of a STEP file that share no edge;
- non-manifold bodies.

Arris is narrower than Open CASCADE and is not aiming for feature parity.
[docs/COMPARISON.md](docs/COMPARISON.md) sets out what each does today and
where they differ, and its [capability matrix](docs/COMPARISON.md#capability-matrix),
held cell for cell by a test, gives what each operation does and refuses
for each face kind.

## How it is tested

- **An oracle for every fixture.** Each fixture in the test corpus carries
  values computed by Open CASCADE — volume, area, centroid, counts, point
  classifications — that Arris must match within the fixture's tolerance.
  Open CASCADE runs as the oracle through Python and is never a build or
  runtime dependency ([PROVENANCE.md](PROVENANCE.md)).
- **A checker after every operation.** Every result passes the invariant
  checker, which runs after every operation in debug builds.
- **Properties over random operands.** Volume additivity, cut-then-fuse,
  commutativity, STEP round trips and more, over random operands in random
  poses from a fixed seed; nightly at five times CI's case count.
- **Real parts.** A corpus of real STEP parts is read, checked, cut and
  blended, and a differential runs random recipes through Arris and Open
  CASCADE side by side.

## Workspace

`crates/arris` is the facade a consumer depends on. It re-exports the
layered crates beneath it, and a crate never depends on one above it
(`tools/check-layers.sh` holds that in CI and in the pre-commit hook):

| Crate | Holds |
|---|---|
| `arris-math` | Points, frames, intervals, tolerances, the deterministic transcendentals |
| `arris-geom` | Curves and surfaces, NURBS, intersections, sections and fits |
| `arris-topo` | The B-Rep in an arena: bodies, shells, faces, edges, vertices, provenance |
| `arris-check` | The invariant checker |
| `arris-ops` | Primitives, booleans, blends, sweeps, lofts, queries |
| `arris-mesh` | Tessellation |
| `arris-io` | STEP, STL, OBJ and the native format |
| `arris-py` | The Python binding, published to PyPI |

`arris-debug` holds the developer tools — the rasteriser, the fixtures, the
property-test strategies — and is not published. The layout and its reasons
are in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Prior work

Arris stands on four open projects, and it reads them, not copies them.
Arris contains no code, comments or identifiers from any of them, and no
line-by-line translation. Each one was read for its algorithms and for the
mistakes it made, and the approach was reimplemented for Arris's own
representation. [PROVENANCE.md](PROVENANCE.md) sets out the rule.

- **[Open CASCADE](https://github.com/Open-Cascade-SAS/OCCT)** (LGPL 2.1, with
  an exception). The mature C++ kernel, and the test oracle. Arris runs it
  through Python to produce expected values; no crate links it, at build or
  run time. See [docs/COMPARISON.md](docs/COMPARISON.md).
- **[truck](https://github.com/ricosjp/truck)**. The original pure-Rust NURBS
  B-Rep. Arris learns from its NURBS evaluation, its tessellation and its
  marching intersections, and from why they fail at seams.
- **[monstertruck](https://github.com/virtualritz/monstertruck)**. An active
  fork of truck, with fillets, healing and more STEP. Arris learns from its
  fillet and healing work.
- **[Fornjot](https://github.com/hannobraun/fornjot)**. A validation-first
  B-Rep kernel, which never reached booleans. Arris learns from its layered
  validation and its per-object checks.

## Licence

MIT or Apache-2.0, at your option. See `LICENSE-MIT` and `LICENSE-APACHE`.
