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

A B-Rep geometric kernel written from scratch in Rust.

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
  faces, shell to a wall, split by a plane).
- **Booleans:** cut, fuse and common, single or multi-tool, over every
  analytic face (plane, cylinder, elliptic cylinder, cone, sphere, torus)
  in any pose, with multi-shell results (cavities, split cuts, disjoint
  fuses). Sections are exact where they are conics and traced and fitted to
  NURBS elsewhere.
- **Blends:** constant-radius fillet and chamfer along chains of edges
  between planes and cylinders, coaxial pairs included, with miters,
  corners, steps, cusps and hole rims.
- **Queries**, which leave the body unchanged: ray hits, minimum distance
  between points, vertices, edges, faces and solids, interference between
  solids with the shared volume, clash screening of an assembly, and the
  planar section of a body.
- **Output:** tessellation to a chord tolerance, per face or whole, with
  faces meshed at different times welded together; mass properties (volume,
  centroid, inertia); plane projection and face frames.
- **Formats:** a STEP reader that returns a checked body, or a typed refusal
  naming why, for every solid in a file; STEP AP214, STL and OBJ export; a
  deterministic native format in JSON and bytes; and one body, with its
  provenance, as bytes that any later release reads.

Not yet: sweeps and lofts, variable-radius blends, non-manifold bodies,
healing of what a STEP file leaves open, and booleans where a NURBS face
meets a face of the other operand. Each of those is a typed refusal today,
never a wrong answer.

Arris is narrower than Open CASCADE and is not aiming for feature parity.
[docs/COMPARISON.md](docs/COMPARISON.md) sets out what each does today, where
the two differ in approach, and what has been measured against it.

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
