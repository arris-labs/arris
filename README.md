# arris

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

## State

Pre-1.0 and under development. The public API breaks between minor versions,
and each break is listed, with its fix, in `CHANGELOG.md`. What is in today:

- primitives, rigid transforms, and extrude and revolve of a profile of
  lines, arcs and elliptic arcs;
- booleans — cut, fuse, common — over every analytic face (plane,
  cylinder, elliptic cylinder, cone, sphere, torus) in any pose, with
  multi-shell results (cavities, split cuts, disjoint fuses); sections are
  exact where they are conics and traced and fitted to NURBS elsewhere;
- constant-radius fillet and chamfer on plane–plane and plane–cylinder
  edges, with miters, corners and hole rims;
- tessellation to a chord tolerance, mass properties (volume, centroid,
  inertia), plane projection and face frames;
- a STEP reader that returns a checked body, or a typed refusal naming
  why, for every solid in a file; STEP AP214, STL and OBJ export; a
  deterministic native format in JSON and bytes; and one body, with its
  provenance, as bytes that any later release reads.

Not in yet: booleans with a NURBS face as an operand, blends between faces
outside the plane and cylinder pairs, variable-radius blends, sheet and
non-manifold bodies, and healing of what a STEP file leaves open. Each of
those is a typed refusal today, never a wrong answer.

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

## Licence

MIT or Apache-2.0, at your option. See `LICENSE-MIT` and `LICENSE-APACHE`.
