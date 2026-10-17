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
  <img src="https://raw.githubusercontent.com/arris-labs/arris/main/docs/assets/showcase.png" width="480" alt="Motor-mount bracket modelled in Arris, faces coloured by provenance">
</p>

```python
# A motor-mount bracket: extruded L profile, fused boss, bore and bolt pattern cut
# in one call, fillets on the bend and hole rims, and a chamfer on the boss.
# Every face is coloured by the operation that made it, followed through provenance.
# Full script: crates/arris-py/examples/bracket.py

import arris

model = arris.Model()

# 1. Extrude the L plate
profile = arris.Profile(
    arris.Loop.polygon([(0, 0), (60, 0), (60, 8), (8, 8), (8, 50), (0, 50)]),
    origin=(0, 0, 0), normal=(0, -1, 0), x_axis=(1, 0, 0),
)
plate, record = model.extrude(profile, (0, 1, 0), 40)

# 2. Fuse a boss on the upright leg
boss, _ = model.primitive_cylinder((8, 20, 32), (1, 0, 0), 14, 12)
body, record = model.fuse(plate, boss)

# 3. Cut bore, bolt holes, and counterbores in one multi-tool call
tools = [model.primitive_cylinder((-1, 20, 32), (1, 0, 0), 8, 23)[0]]
for y in (8, 32):
    tools.append(model.primitive_cylinder((40, y, -1), (0, 0, 1), 3.5, 10)[0])
    tools.append(model.primitive_cylinder((40, y, 5), (0, 0, 1), 6, 4)[0])
body, record = model.cut_many(body, tools)

# 4. Fillets and chamfer: edges selected by provenance
[bend] = [e for e in record.generated_from(arris.Role("extrude", "Rise", 0, 3)) if isinstance(e, arris.Edge)]
body, record = model.fillet(body, [bend], 4)
body, record = model.fillet(body, rim_edges, 0.8)
body, record = model.chamfer(body, [boss_lip], 1.5)
```

A B-Rep geometric kernel written from scratch in Rust, built for code-CAD script
authors, AI agents, and CAE pipelines.

An *arris* is the sharp edge where two surfaces meet — the thing a B-Rep kernel exists to compute.

Arris is a library, not an application: it builds solids, cuts and fuses
them, blends their edges, measures, tessellates and exports them. It carries
a tolerance on every entity rather than one global epsilon, returns a typed
error naming the entities involved rather than panicking on geometry it
cannot do, and returns a provenance record from every operation so that a
parametric application can name the faces an operation made. Same input,
same output, same entity ids, on every platform. Pure Rust, no `unsafe`,
builds for `wasm32`.

## Features

Pre-1.0 and under development. The public API breaks between minor versions,
and each change, with each break and its fix, is listed in
[CHANGELOG.md](CHANGELOG.md).

- **Modelling:** primitives, rigid transforms, and extrude and revolve of a
  profile of lines, arcs and elliptic arcs; prismatic features (offset
  faces, shell to a wall, split by a plane); a profile as a one-face sheet
  body, moved and mirrored like a solid and measured by area.
- **Sweeps and lofts:** sweep along a path (lines, arcs, helix arcs, B-spline
  curves) with rotation-minimising or fixed-binormal frames; loft across
  sections matched by arc length (ruled or smooth polynomial).
- **Booleans:** cut, fuse and common, single or multi-tool, over every
  analytic face (plane, cylinder, elliptic cylinder, cone, sphere, torus)
  and NURBS against analytic surfaces, in any pose, with multi-shell results
  (cavities, split cuts, disjoint fuses). Sections are exact where they are
  conics and traced in the parameter plane and fitted to NURBS elsewhere.
- **Blends:** constant-radius fillet and chamfer along chains of edges
  between planes and cylinders, coaxial pairs included, with miters,
  corners, steps, cusps, hole rims, and walked stripes on NURBS faces.
- **Queries**, which leave the body unchanged: ray hits, minimum distance
  between points, vertices, edges, faces and solids, interference between
  solids with the shared volume, clash screening of an assembly, and the
  planar section of a body.
- **Output:** tessellation to a chord tolerance, per face or whole, with
  faces meshed at different times welded together; mass properties (volume,
  centroid, inertia); plane projection and face frames.
- **Formats:** a STEP reader that returns a checked body, or a typed refusal
  naming why, for every solid or surface model in a file, closing small gaps
  and turning back reversed faces and naming each repair; STEP AP214 export
  in the unit you choose, STL and OBJ export; a deterministic native format
  in JSON and bytes; and one body, with its provenance, as bytes that any
  later release reads.

Not yet: booleans where both operands carry NURBS faces meeting each other,
variable-radius blends, non-manifold bodies, offset/shell on elliptic cylinder
or NURBS surfaces, and sewing faces that a STEP file leaves unconnected.
Each of those is a typed refusal today, never a wrong answer.

Arris is narrower than Open CASCADE and is not aiming for feature parity.
[docs/COMPARISON.md](docs/COMPARISON.md) sets out what each does today, where
the two differ in approach, and what has been measured against it. See the
test-held [capability matrix](docs/COMPARISON.md#capability-matrix) for what each
operation does and refuses per face kind.

## From Python

`pip install arris` gives a binding over the same kernel: build a body from
primitives and profiles, fillet and cut it, ask what it measures and what the
checker says, tessellate it, and write and read STEP, STL, OBJ and body
bytes, without compiling a crate. Every operation returns the body and its
provenance, and every error is a typed exception.

```python
import arris

model = arris.Model()
plate, _ = model.primitive_box((0, 0, 0), (100, 100, 10))
hole, _ = model.primitive_cylinder((50, 50, -1), (0, 0, 1), 5, 12)
part, record = model.cut(plate, hole)
print(model.mass_properties(part).volume, bool(model.check(part, "full")))
```

## Correctness

Every operation returns a shape that passes the invariant checker, and the
checker runs after every operation in debug builds. Every fixture in the
test corpus carries an oracle value — volume, area, centroid, counts, point
classifications — computed by Open CASCADE, which Arris must match within
the fixture's stated tolerance. Open CASCADE is run as the oracle through
Python; it is never a build or runtime dependency, and `PROVENANCE.md` says
how it is used and read. Properties (volume additivity, cut-then-fuse,
commutativity, STEP round-trip) are tested over random operands in random
poses from a fixed seed.

## Workspace

Arris is a Cargo workspace. `crates/arris` is the facade a consumer depends
on; it re-exports the layered crates beneath it — `arris-math`, `arris-geom`,
`arris-topo` (the representation), `arris-check` (the invariant checker),
`arris-ops`, `arris-mesh`, `arris-io` (the algorithms) and `arris-debug`
(dev-facing: rasteriser, fixtures, property-test strategies; it is not
published). `crates/arris-py` is the Python binding, above the facade and
published to PyPI rather than crates.io. A crate never depends on one above it; `tools/check-layers.sh`
enforces that in CI and in the pre-commit hook. The layout and the reasons
are in `docs/ARCHITECTURE.md`; `tools/oracle/` is the Open CASCADE test
oracle, run through Python and never linked.

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
