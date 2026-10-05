# 01 — Architecture

Arris is a library. It owns a *model* — an arena of geometry and topology —
and a set of *operations* that append to it and return handles. Everything a
consumer does is a call of the shape `op(&mut Model, inputs…) ->
Result<(Body, Provenance), OpError>`, followed by queries on the handle it
got back. There is no session object, no builder with hidden state, no
global. This document holds the crate layout, the arena and handle model,
the operation and error contract, where the checker runs, the threading and
wasm rules, and how a consumer's kernel facade maps onto the API. The
entities and geometry themselves are in [data-model](DATA-MODEL.md).

## Crates and the layer rule

One Cargo workspace, `crates/arris-*`, plus the facade crate `arris` that
re-exports the public API. Lower crates never name types from upper ones.

| Crate | Owns | External deps | Layer |
|---|---|---|---|
| `arris-math` | `Point3`/`Vec3`/`UnitVec3` (over `nalgebra`, ADR-0001), `Frame`, `Frame2`, `Axis`, `Isometry`, `Interval`, `Aabb`, the periodic-parameter toolkit (`wrap_angle`, `wrap_into`, `shift_nearest`, …), exact orientation predicates (over `robust`), polynomial and interval-guarded Newton root finding, `Precision` and `Tolerance`, `Control` (a poll and a budget of steps), the `Meter` that counts against it, `Stop` and `Interrupted` (ADR-0030) | `nalgebra`, `robust`, `serde` (feature) | 0 — representation |
| `arris-geom` | `Surface`, `Curve`, `Curve2` (analytic + NURBS): evaluation, derivatives, point projection, curve/curve, curve/surface and surface/surface intersection (`trace_quadrics`, the exact section of two quadrics by the rulings of one, and `trace_torus`, the exact section of a torus in its own parameter plane — both fitted by the intersector), bounding boxes over a parameter range, pcurves and the NURBS fit behind them; the (u, v) toolkit `region2` and `integrate` shared by the checker, tessellation, mass properties and classification; `Profile`, the planar sketch of lines, arcs and elliptic arcs a sweep takes, validated and oriented by `Profile::edges`; `GeomError` | `arris-math`, `thiserror`, `serde` (feature) | 0 — representation |
| `arris-topo` | `Model` (the arena), typed ids, `Shape`/`Body`/`Face`/… handles, orientation, entities, pcurves, per-entity tolerances, Euler operators including the assembly seam (`Assembly::of_body`, `effective_uses`, `AssemblySlots`), the Euler line (`euler::EulerLine`), adjacency and iteration, `Provenance` and its audit | `arris-geom`, `arris-math`, `thiserror`, `serde` (feature) | 0 — representation |
| `arris-check` | The invariant checker: `check(&Model, Body, Level) -> Report` and the `Violation` list of data-model §Invariants; the shared face domain (`domain::FaceDomain`), point classifier (`classify::Classifier`, `classify_point`) and region flux (`flux::face_flux`) every `Full` row, the boolean and tessellation read a face through | `arris-math`, `arris-geom`, `arris-topo`, `serde` (feature, forwarded to `arris-topo`) | 1 |
| `arris-ops` | Primitives, extrude and revolve of a `Profile`, transform, mirror, booleans, the blends, each returning `Provenance`; the queries `measure` (mass properties) and `query` (projection onto a plane, a face's outward frame) | `arris-math`, `arris-geom`, `arris-topo`, `arris-check`, `thiserror`, `rayon` (feature) | 2 — algorithms |
| `arris-mesh` | `TriMesh`, `Polyline`, the constrained Delaunay triangulation in (u, v) (`cdt`, ADR-0003), tessellation of faces and edges with shared edge discretisation; re-exports `arris-math`'s `Aabb` and `Interval`, which its signatures use | `arris-math`, `arris-geom`, `arris-topo`, `arris-check`, `thiserror`, `rayon` (feature) | 2 — algorithms |
| `arris-io` | STEP AP214 Part 21 writer and reader (`step::write`, `step::read`, ADR-0025) over the Part 21 parser (`step::part21`), the native format (`native`), body bytes (`body`, ADR-0029), STL and OBJ mesh writers (`stl`, `obj`, ADR-0013) | `arris-math`, `arris-geom`, `arris-topo`, `arris-check`, `arris-mesh`, `thiserror`, `serde`, `serde_json`, `postcard` (the last three behind the `serde` feature) | 2 — algorithms |
| `arris-debug` | Text dump, the hand-built sample bodies (`sample`), PNG render (own software rasteriser over `image`), Rerun stream (feature), the fixture loader and corpus lint, the corpus runner (`corpus`), the part fixtures' runner and lint (`part`), the battery run on every part's solid (`battery`), the refusal histogram and ADR-0026's table (`histogram`), a fetched part surveyed (`survey`), a STEP file seen solid by solid (`step_file`) and the oracle seam (`oracle`), the seeded property-test runner and strategies (`prop`, `prop::recipe` among them), the differential over both kernels (`differential`), the benchmark timer (`bench`) | `arris-ops`, `arris-mesh`, `arris-io`, `arris-topo`, `arris-geom`, `arris-math`, `image`, `serde`, `serde_json`, `sha2`, `thiserror`, `proptest` (not on `wasm32`), `rerun` (feature) | 3 — dev-facing |
| `arris` | Facade: re-exports | `math` through `io`; `debug` as a dev-dependency only | 4 |
| `arris-py` | The Python binding (PyPI `arris`, ADR-0034): a thin 1:1 layer over the facade, `publish = false`, a `cdylib` + `rlib`, empty on `wasm32` | `arris`, `pyo3` (not on `wasm32`) | 5 — binding |

`math`, `geom` and `topo` are the representation: they change rarely and a
change there is a design delta named in a plan. Everything from `check` up
is an algorithm crate that can be rewritten without touching its
neighbours. `ops` and `mesh` are siblings: neither depends on the other.
`io` depends on `mesh` — STL and OBJ each write one or several
`TriMesh`es, and every format a consumer looks for lives under the one
name `io` (ADR-0013) — but not on `ops`, which no format needs. Mass
properties live in
`ops::measure` because they integrate over the B-Rep, not over a mesh; a
consumer that wants mesh-based inertia integrates `arris-mesh`'s output
itself.

The rule is enforced, not remembered: `tools/check-layers.sh` walks the
declared edges of `cargo metadata` with a layer number per crate — its
place in the chain `math` ← `geom` ← `topo` ← `check` ←
`ops`/`mesh` ← `io` ← `debug` ← `arris` ← `arris-py`, finer than the table's tiers — and
fails on any edge that does not go strictly downward; dev-dependencies
are exempt so a lower crate's tests may use `arris-debug`, which is itself a
dev-dependency of the facade and never reaches a consumer. CI runs the
script and its self-test (a scratch copy with a forbidden edge must fail);
the pre-commit hook runs the script.

Every crate has `#![forbid(unsafe_code)]` and `#![warn(missing_docs)]`.
Feature flags are few and named the same in every crate that has them:
`serde` (on by default in `topo` and `io`; off by default in `math` and
`geom`, where `topo`'s feature turns both on because `Precision` and the
geometry are part of the native format; `check` carries the same feature
forwarded to `arris-topo/serde` and nothing of its own, the one path `io`
has there without depending on `topo`'s own default features), `parallel`
(`rayon` inside `ops` and `mesh`; never enabled on
`wasm32`), `paranoid` (`ops`: run the checker after every operation in
release builds too, §The checker), `rerun` (`debug` only). Every internal
workspace dependency is declared `default-features = false` at
`[workspace.dependencies]`, so `default = [...]` on a member (`topo`,
`io`, `arris`) is what a plain path or version dependency on it actually
gets; a consumer that wants the layer-legal path without a crate's own
defaults depends on it directly with `default-features = false` and
forwards the feature itself, as `arris`'s own `serde` feature and
`cargo check -p arris --no-default-features` do (`arris-ops`, `-mesh`
and `-io` reach no serde type at all with it off — `cargo tree -p arris
--no-default-features -e normal` has neither `serde` nor `serde_json`;
CI asserts the second). The facade forwards `serde`, `parallel` and
`paranoid`.

## The model, the arena and handles

`Model` is the arena. It holds every vertex, edge, face, shell, body,
curve, surface and pcurve ever created in it, each behind a typed
generational id (`VertexId`, `EdgeId`, `FaceId`, `ShellId`, `BodyId`,
`CurveId`, `SurfaceId`, `Curve2Id`): a `u32` slot index and a `u32`
generation. Ids are allocated sequentially in creation order, and a slot is
reused only after a compaction (below) at a new generation, so in the
common case an id is also a creation timestamp, and iteration in id order
is deterministic on every platform. An accessor (`model.face(id)`) returns `NotFound` for an index
past the arena, a freed slot or a stale generation — never the slot's
current occupant. Geometry is inserted by value (`add_curve`,
`add_surface`, `add_curve2`) and never deduplicated: two faces share a
`SurfaceId` because the operation that split them handed both the same
id, not because the arena matched two equal surfaces.

**An id names no model.** It is a slot and a generation and nothing else, so
a Rust caller who gives one model another's id gets whatever entity sits in
that slot, or `NotFound`. The Python binding closes that for its callers
without a kernel change: its `Model` owns the arena behind a lock and a
serial, and every handle it returns carries that serial, so a handle from
another model raises `ForeignHandleError` before the id reaches the kernel
and a freed one surfaces the kernel's `NotFound` as `StaleHandleError`
(`crates/arris-py`, ADR-0034 §6).

**Entities are immutable.** An operation never edits an entity in place; it
appends new ones and returns a handle to a new body that references the
untouched old entities by id. Two bodies that share a face share its id,
its geometry and its tolerance — structural sharing is the default, not an
optimisation. This is what makes provenance cheap (an untouched entity keeps
its id; a modified one has a new id and a record), undo free (an older body
handle is still valid), and background evaluation safe (a clone of the model
sees the same entities).

**The arena is chunked and the chunks are shared.** Storage is a list of
fixed-size chunks (`arris_topo::CHUNK_SIZE` slots) behind `Arc`, one list
per entity and geometry kind; `Model::clone` copies the lists of `Arc`s,
and the first append after a clone copies only the tail chunk. Cloning a
model for a background evaluation is therefore O(number of chunks), not
O(entities), and two clones that diverge share every chunk they both leave
untouched.

**A handle is an id plus an orientation.** `Shape { id: EntityId,
orientation: Orientation }` is the uniform handle used by provenance,
iteration and errors; `Body`, `Shell`, `Face`, `Edge`, `Vertex` are typed
newtypes over the same pair and convert to `Shape` for free. Copying a
handle is copying two integers. Orientation composes down the hierarchy
(data-model §Orientation); a handle never carries geometry.

**Failed operations leave the model as it was.** Because the arena only
grows and refills slots `retain` freed, an operation runs inside
`Model::transaction(|m| …)`, which marks every arena at entry — its
length and its free set — and on `Err` drops everything appended since:
the tail of every arena and the freed slots the transaction filled,
emptied again at the generation they had, with the adjacency indices and
the next id rolled back with them — so the model, its ids and its clones
are exactly as before. Transactions nest; an inner `Err` undoes only the
inner appends. A consumer never sees half-built entities.

**Bodies move between models by import.** `Model::import(&mut self, &other,
body) -> Result<(Body, IdMap), TopoError>` deep-copies a body's closure —
sorted id order per kind, geometry first — inside a transaction and
returns the new handle with the id map; a reference in the other model
that does not resolve is `NotFound` and nothing is appended. This is how
independent evaluations on clones are merged, how a consumer holds several
documents, and how provenance across models is translated
(`Provenance::mapped`).

**Compaction.** `Model::retain(&mut self, keep: &[Body]) -> Result<usize,
NotFound>` frees every entity and geometry value not reachable from `keep`
— the slot's value dropped, its generation bumped so every handle to it
stops resolving instead of aliasing — rebuilds the adjacency indices, and
returns the count. A freed slot keeps its place and is filled by a later
append, lowest index first, at the bumped generation (ids `v3g1`, then
`v3g2`…), so a long-lived model does not grow without bound and ids stay
deterministic; a transaction that fails after filling freed slots empties
them again, and one that fails after a `retain` does not undo it. It is
the only operation that invalidates handles and it is never called by
the kernel itself. **Slots are never renumbered** (ADR-0010): every entity
reachable from `keep` keeps its id, so a consumer's stored ids — a
selection, an undo entry, a name derived from provenance — survive a
compaction untouched, and a handle to a freed entity stays `NotFound` for
ever rather than resolving to whatever refilled its slot. A *dense* copy
is `Model::import` into a fresh model, which returns the `IdMap` with it,
or the native format, which writes freed slots as freed.

## Operations

Every operation in `arris-ops` has the same shape:

```rust
pub fn cut(m: &mut Model, target: Body, tool: Body, control: &Control) -> Result<(Body, Provenance), OpError>;
```

- Inputs are handles into `m`. The operation reads them, appends, and
  returns a new handle plus the provenance record (data-model
  §Provenance) that says which output entity came from which input entity
  and how. An operation without a provenance record is unfinished.
- The operation never mutates its inputs, never panics on geometry, never
  contains a numeric literal that stands for a tolerance, and never iterates
  a hash map where the order can reach a geometric decision or an id.
- Parameters that are geometry (`Axis`, `Frame`, `Profile`) are plain
  values from `arris-math`/`arris-geom`, not handles: a primitive is built
  from numbers, and only the result lives in the model. `arris_math::Axis
  { origin, direction }` is a point and a unit direction, normalised by
  `Axis::new` (`Axis::z_at` for the common case); `primitive_cylinder(m,
  axis, radius, height)` places its frame by `Frame::from_z` of it, so it
  seams where the oracle's cylinder does, and `primitive_box(m, min,
  max)` takes two corners. Both build through the Euler operators
  (data-model §Euler operators) and return every entity `Generated`
  from a `Role`.
- Same input, same output, same ids, on every platform. The tests assert
  this by dumping twice and diffing.
- Every operation on a model, and every long query beside one, takes a
  trailing `&Control` (ADR-0030): a poll the consumer answers from
  whatever its platform has, and an optional budget of steps.
  `Control::NONE` runs to the end. A stop is `Interrupted` in the
  operation's own error type, carrying the cause (`Stop::Poll` or
  `Stop::Budget`) and the steps taken, and the model is as it was, ids
  included. A step is one iteration of a loop whose trip count the input's
  entity count does not bound (a tracer's march, a fit's refinement, a
  boolean pair, a face split, a blend corner, a CDT insertion, a reader's
  entity). The writers and the constant-work queries take no `Control`,
  and the checker is not ticked.

`ops::build(m, builder, &BuildKeys)` finishes a `Builder` the consumer
filled itself — through the Euler operators or `Builder::assemble` —
into a `Solid`, and records every entity `Generated` from
`Role::Consumer` of the key `BuildKeys` gives its slot (data-model
§Provenance, ADR-0028). It is the one operation whose output is the
consumer's input rather than the kernel's work, so it checks the body at
`Level::Full` in every build profile — not only in debug builds, as the
other operations check theirs — and refuses a body that fails. The
other operations that create from nothing take no key: their roles
already name each part, and `Provenance::rerooted` puts the consumer's
key in their place.

`ops::transform(m, body, motion: &Isometry)` moves a body rigidly. Every
curve and surface is appended transformed and every pcurve id is reused as
it stands — a rigid motion carries the parametrisation with it, so
parameter space does not move — and every vertex, edge, face, shell and
the body itself is appended new in the body's own iteration order through
`Builder::assemble`, each `Modified` one-to-one from the entity it moved.
The body's kind is kept and nothing of the input is shared, so the moved
body is an operand a boolean can take beside the original. It reaches
exactly as far as `assemble` does — shells that share nothing, each one
edge-connected, carried shell by shell in the body's stored order, each
`Modified` from the one it moved — and it is how the property tests put
their operands in random poses. It is `arris_topo::builder::Assembly::
of_body(m, body, &mut remap) -> Result<(Assembly, BodyIndex), NotFound>`
(data-model §Euler operators) plus a `GeometryRemap` that moves a point,
a curve and a surface by the motion: the walk over the body's closure, the
`Assembly` it describes and the `BodyIndex` provenance is built from are
shared with a boolean's own assembly, so `transform`'s own work is the
remap alone. `GeometryRemap` has two provided hooks beside the three
it must answer: `pcurve(model, p, surface)` (the pcurve to give a copied
use; the original surface it lay on) and `face(model, surface) ->
FaceRemap { toggle_use, reverse_loops }`; `of_body` applies them, toggling
the face's use and copying its stored loops backwards with each coedge use
toggled where told to. Their defaults are the identity, which is all a
rigid motion needs.

`ops::mirror(m, body, plane: &Reflection, control)` is `transform`'s
sibling over the same seam (ADR-0031): the image of a body in a plane,
frames kept right-handed. Every curve and surface is appended as its
`mirrored` image — a quadric's `u` is reflected (`u ↦ 2π − u`), a plane's
and a NURBS surface's parameters are kept and their normal turns, a curve
keeps its parameter — a pcurve is reused where the surface's parameters
are kept and `Curve2::reflected` where they are not, and the `face` hook
answers `toggle_use` for a plane or NURBS face and `reverse_loops` for a
quadric one, so every image's effective loop is the reverse of the mirror
image of the original's. Provenance is `Modified` one-to-one; nothing of
the input is shared, even with the plane through the body.

A **query** has a different shape: it takes `&Model`, makes no body and
records no provenance, because there is nothing for a later operation to
name. `ops::measure::mass_properties(&Model, Body) ->
Result<MassProperties, OpError>` is the first — volume, area, centroid
and the inertia tensor about the centroid at unit density, in the
physical convention, with `MassProperties::inertia_about(point)` for any
other point. Every quantity is a flux integral over the body's faces by
Green's theorem in each face's own (u, v) (`geom::integrate`), as the
checker's B2 row already computes an enclosed volume: nothing is
discretised, so the numbers are the geometry's and not a mesh's, and the
corpus holds them to the oracle's within each fixture's tolerance. The
volume and first moments are integrated about the centre of the body's
vertices, and the second moments about the centroid itself rather than
carried there by the parallel-axis theorem. Both are for a body far from
the origin: the parallel-axis theorem would pay for its distance in
cancellation, and faces whose pcurves of one section are fitted apart
close only to the fit, so every gap leaks flux in proportion to its
distance from the point the integral is taken about. A body that is not a `Solid`
is `OpError::Degenerate` with `InputReason::NotSolid`; an invalid one
`InvalidInput`, as an operation's input is.

`ops::query::project_to_plane(&Model, &[Shape], &Frame) ->
Result<Vec<Projection>, OpError>` answers a consumer's sketch: each
vertex or edge named, in the order given, projected orthogonally onto the
plane in its frame's `(x, y)` — a vertex to a `Point2`, an edge to
`geom::project_to_plane` of its curve with the edge's range **carried
through the projected curve's own parameter** (data-model §Pcurves), so
`curve.point(range.lerp(s))` is the projected edge at
`edge.range().lerp(s)` and the piece covers the edge and no more. The
projection is the edge's own curve in its own direction; a `Reversed`
handle does not reverse it. It refuses, as `Degenerate` naming the
shape, a face, shell or body (`QueryReason::NotProjectable` — which edges a
view shows is the caller's to decide), a degenerate edge
(`QueryReason::DegenerateEdge`), and a curve seen edge-on
(`QueryReason::ProjectionCollapses`).

`ops::query::face_frame(&Model, Face) -> Result<Frame, OpError>` and
`ops::query::frame_at(&Model, Face, Point2) -> Result<Frame, OpError>`
answer the outward-oriented frame a facade needs at a face: `face_frame`
is a planar face's own surface frame, `Z` its outward normal — negated
together with `X` (`Y` kept, so the flip stays right-handed) when the
handle's use is `Reversed` — refused, naming the face, for a curved
surface (`QueryReason::NotPlanar`), since only a plane has one frame for its
whole domain. `frame_at` is any face's frame at a `(u, v)` its own domain
contains (`check::domain::FaceDomain`): `Z` is `Surface::normal` there
composed with the use exactly as `face_frame`'s is, `X` the surface's own
`∂P/∂u`, `Y = Z × X` — so on a plane it agrees with `face_frame`'s at
every `(u, v)`. It refuses, naming the face, a `(u, v)` outside the
domain (`QueryReason::OutOfDomain`) and a singular one — a sphere's pole, a
cone's apex — where the surface has no normal (`QueryReason::Singular`).

`ops::boolean::interferences(&Model, a, b) -> Result<Interferences,
OpError>` is another query: the boolean decomposition of ADR-0004 as
a value, computed without building anything. It is the two-operand case
of `interferences_many(&Model, &[Body])`: the decomposition is indexed by
operand (`Interferences::operands`, `FacePair::operands`,
`EdgeImage::operand` and `on`), one build over every pair of operands
(ADR-0050). **No guard stands before
the intersector**: every face pair, and every edge against a face, whose
boxes overlap is asked about, whatever analytic surface either lies on,
and `OpError::Unsupported` names a pair only where the intersector
itself has no arm — a NURBS operand. A cone, an elliptic cylinder, a
sphere and a torus face are paved like any other, each with its corpus
(`boolean/frustum-*`, `boolean/chamfered-boss-slot-cut`,
`boolean/elliptic-*`, `boolean/ball-*`, `boolean/ring-*`,
`boolean/filleted-*`), so a body `ops::revolve`, `ops::fillet` or
`ops::chamfer` returned is a body a boolean takes — the closure ADR-0020
asks for. A whole sphere face, closed on its two degenerate edges, and a
whole torus face, periodic both ways between its two seams, needed
nothing of the pave model but one conversion: the slack a section
edge's pcurve is allowed past the face's (u, v) box is the 3D tolerance
converted *where the pcurve is* and in the direction held
(`check::domain::bands`), not once at the block's midpoint — a section
loop round a pole crosses the seam where a tolerance is a far wider `u`
than it is at the loop's far side (`boolean/ball-polar-drill-cut`). A
pair meeting in curves of both kinds is read as both: its crossing
curves are its sections and its touching curves its contacts, whatever
else the `Meets` holds — a pipe's bend and the straight run it joins,
tangent along the tube circle and crossing in a quartic beside it
(`boolean/pipe-elbow-fuse`).
Every face pair is
intersected in one region for the whole boolean — the overlap of the
operands' boxes grown by its own diagonal — so pairs on the same two
surfaces get the same traced curve (ADR-0018). It holds every face
pair whose boxes overlap with its `SurfaceIntersection` — its crossing
curves the *section curves*, its touching curves the *contact curves*,
one pair holding either or both; its points matter only as a traced
section's singular points, where its branches end, or as an apex or a
pole an operand's own vertex already stands on; every point where
an edge of one operand pierces a face of the other, kept when the
parameter is in the edge's range and the (u, v) is on the face
(`region2::point_side` on the face's loops, a `Boundary` verdict
resolved to the edge or vertex of the face within its own tolerance,
the edge's own end vertex when the hit is within its ball); those hits
merged into section vertices by *closure*, not by arrival — every
candidate point (the hits, the edge–edge crossings, the section
crossings and singular vertices below, the operand vertices they name)
is a ball of the tolerance of the entities that made it, two candidates
are the same point when their balls meet, `|p − q| ≤ tp + tq`, or they
name one operand vertex, and a section vertex is a connected component
of that relation, so three points 1.7e-7 and 2.4e-7 apart at 1e-7 are
one vertex in whatever order they were found; a component is numbered
by its first member in the order the candidates are made and carries
that member's source, its point is its first operand vertex's, else its
first member's on an operand edge — a hit's or a crossing's, the edge
being cut there exactly — else its first member's, and its tolerance is
the largest of the entities
merged plus the spread of the points about it (data-model
§Tolerances); a component holding two vertices of one operand would
collapse what lies between them and is `OpError::Tolerance`; the
*section crossings* — two section curves of one
crossing pair intersected with each other (`intersect_curves`), a
crossing on both faces being a section vertex by the same merge, since
two curves of one pair meet where the surfaces are tangent to each
other, the two ellipses of equal cylinders with crossing axes at `±R`
along the axes' common perpendicular, and no edge of either operand is
there to make a hit — for a traced pair, whose fitted branches meet
only at its singular points and end there exactly, the crossings are
those points on both faces, with the branches that end at each, and no
two fitted curves are intersected; a *touch* — a hit where the edge meets the surface
without crossing it — makes no vertex of its own, but one whose
component holds a vertex made by the hits and crossings joins it, since the edge passes
through that vertex (a seam ruling or a rim circle through the crossing
of two ellipses, tangent to the other wall there because the walls are),
and one that lands on none is *resolved through the section curves*
(ADR-0016): the edge is intersected with every section curve of the
touched face and a face of its own, and each crossing on the face is a
hit like any other, with its vertex and its pave — because the
intersector's touch is a verdict on depth, the edge within the tolerance
of the surface, and a chord `h` deep is `2√(2Rh)` long, so beside a
point where the two surfaces are tangent to each other a seam `6e-8`
inside the other wall is one touch whose two crossings are `7e-4`
apart, both ellipses cut it, and neither would find a pave there; the
edge against the section curve is two curves of one surface crossing at
an angle, exact where the edge against the surface is a square root of
rounding, so the intersector's verdict is left as it is and a designed
tangency in any pose stays one touch. Where the edge and the section
curve are themselves one touch by depth — two conics of one surface
crossing twice at a shallow angle, a small circle through a sphere's
pole 2e-4 of a radian off the seam and back across it 1.8e-4 on, within
9e-9 of it between — their common points are taken along the line
their two planes meet in (`conic_crossings`), a quadratic whose double
root is decided by its own rounding and never by a tolerance;
an operand face's *singular vertex* — a cone's apex, a sphere's pole,
held by a degenerate edge that has no curve and is hit by nothing — that a
section curve of one of the face's pairs runs through, within
`pcurve_on`'s own band and on the other face, is a section vertex over
that operand vertex: the one a seam ending there made by piercing the
other face, or one of its own (`VertexSource::Singular`) where the seam
only touches it, so no block has the point inside it; a section that
passes beside the point, nearer than the face's (u, v) polygons resolve,
is refused by name (`BooleanReason::BesideSingularity`, ADR-0021);
the paves each vertex puts on the edge that hit or touched it — none
where the vertex holds one of that edge's own ends — and on
every section curve it projects onto within its tolerance — at the
parameter of its own section crossing on a curve that crossing names,
so two section curves end on the point where they cross whatever the
vertex's point is; an open
section curve at each of its ends a vertex lies on, both ends of a
traced branch that leaves a singular point and comes back to it; and the
section edges — the blocks between consecutive paves whose midpoint is
inside both faces, a closed curve with no pave seeded at the start of
its domain when it is interior to both, and a periodic one's last block
wrapping round to its first pave, one period on — a traced loop's
period being its own length, not a turn — each with a pcurve on each
face by `pcurve_on`, translated by whole periods into the copy of the
domain the face's loops are written in and ending on its vertex's own
(u, v) on the face — where an edge of the face is paved at the vertex
or ends on an operand vertex it holds, that edge's pcurve there, else
the vertex point's — wherever it lies further from it than half the
band L2 holds a junction to, the end control points of a clamped
spline moved: a vertex whose members lie further apart than a face's
tolerance has its edges' pcurves ending that far apart, which no exact
curve can close, so the section edge's pcurve moves and never the
operand's; its tolerance the larger of the faces' raised to the
pcurves' residual, the move included, with rounding at the positions'
scale above it so a motion of the body keeps it; a traced loop's stays at its
faces' — the fit's quarter of the tolerance leaves the pcurves room
under their half (data-model §Tolerances); and a pave on a degenerate
edge for every section edge that ends on its vertex, at the `u` the
section arrives with, since the vertex is a whole line of (u, v) and the
face's arrangement needs a node where the section meets it (ADR-0021);
an arrival past the face's (u, v) box — the section crossing the seam
inside the vertex's ball, nearer the pole than its tolerance — is no
length there, and the pcurve ends on the box's edge instead, the seam's
own corner, where the degenerate edge already meets it. A `Coincident` pair — two faces on one
surface, the flush case — is decided by the same arrangement (ADR-0004):
the two faces' edges are intersected with one another and every
crossing is a section vertex — at an end vertex of either edge when that
end lies within the tolerance of the other edge and so does its own
edge from the crossing to it: two edges on one surface crossing at a
shallow angle stay that near over a stretch longer than the tolerance,
the intersector's point may lie anywhere along it, and where an earlier
operation made the vertex at that crossing it is that vertex, not a
second one a hair past its tolerance; every edge of either face is paved by
every section vertex on it but its own ends — a vertex holding one of
the edge's end vertices, a crossing merged into it even a hair past
that end's tolerance, paves nothing there, which would leave a sliver
block; and each piece of each edge between its
paves is matched to the piece of that face's edge it coincides with
as a *common block* when an edge of that face is `Coincident` with its
curve and the two pieces overlap (the same piece then, with the same
ends, or `Fault::CommonBlock`, a kernel bug: the vertices the edges
share paved them differently), placed on the other face as an *image*
with a pcurve there when it lies inside that face by its polygons, and
dropped when it lies outside — the coincidence is the curves' verdict,
never the polygon band's, which two fitted pcurves of one curve can
straddle. For the same reason a block of a section curve that is a
piece of an operand edge of either face is that edge and not a section
edge. Whether an edge runs along the curve is asked of the surfaces
first: an edge of one face whose other face in its own operand lies on
a surface `Coincident` with the pair's other face is on the pair's
section to its own tolerance, and along the branch its midpoint lies on
— a restoring fuse's section and the cut's section edge are two fits of
one traced section over different regions, which no closed form
compares, and each is held to the exact branch. The same holds of the
edge running along a section curve a touch is resolved through, and of
two edges of a coincident pair whose other faces lie on one surface. Only
without such a face is it `curves_coincide`, the verdict without the
points, so a rim circle in the plane of the ellipse its own cap plane
cuts from the other wall — two short cylinders crossing steeply — needs
no closed form for where the two conics would meet.
Beside that whole-curve verdict is a block's own: a block every point of
which the model checks lies within the tolerance of a piece of an
operand edge of either face between the same two vertices is that
piece, asked before the block's midpoint is asked to lie inside both
faces — along an edge of one it lies on that face's boundary — and the
piece is placed on the pair's other face as an image where the section
edge would have been. When it lies within the tolerance of an edge of
the other face as well, wherever that edge is paved, it is on both
boundaries and is neither a section edge nor an image: each edge keeps
its own piece. That happens where two boxes touch along an edge and one
is turned a tolerance about the edge's middle, and the two edges' crossing
is hit at points scattered along them by more than a tolerance. It is the seam's last stretch beside a pole where
the small circle crosses it twice, or a rim circle a fraction of a
tolerance from the ellipse a turned face cuts from the wall; built as a
section edge, the block would bound a sliver of zero area on the edge's
face. An
edge that lies in a face of the other operand
(`Interferences::coincident`) is paved and placed the same way whether
or not a face of its own is coincident with that face: a seam on the
ruling two parallel walls cross along splits the other wall as an image,
since no coincident neighbour is there to place it — unless the piece
runs along that face's own boundary, a pipe's cap circle on the bend it
joins: then it is the face's edge, and the coincident caps beside it
hold the common block. A section vertex nothing ends at — two traced
branches crossing on the faces' boundary and leaving both — paves no
edge of a coincident face. A contact curve — a plane and a cylinder, or
two parallel cylinders, touching along a ruling; a ball in a bore of its
radius, along a circle — contributes no section edge and no pave on any
operand edge: it is paved by the *touches*, the hits of either face's
edges on the other face that lie on it (every curve in a face tangent to
the other surface is tangent to it there, so these are where the curve
leaves one face inside the other), and each block between consecutive
touches whose midpoint is inside both faces is a *contact*, the segment
the two faces share; on a closed curve the last block wraps round to the
first touch, and a closed curve no edge reaches is one block
(`boolean/ball-in-bore-cut`, refused as `BooleanReason::TangentContact`). Every list is in a deterministic order and `Display` prints the
whole model, which is what the `inspect` skill reads when a boolean is
wrong. The property tests build their
operands through `arris_debug::prop::body` — a box and a cylinder whose
axis passes through the box, both under one random motion — and, for
the identities whose outcome has to be known in advance, its
`piercing_pair`: the cylinder clears every edge of the box, so `fuse`,
`common` and `box − cylinder` are one shell each and `cylinder − box` is
exactly two, two lumps of one solid that hold the identities like the
rest. The other pairs — the wall crossing an edge, a corner sliced off —
stay in `overlapping_pair`, where `cut` is held to the identity whatever
number of lumps it makes. Two cylinders come from `parallel_pair` — axes
apart between the two tangent distances, some with a seam on a ruling or
flush caps — and `crossing_pair` — equal radii crossing at 30° to 90°,
each through the other, some with a seam through a crossing vertex, its
`common` held to `16R³/(3 sin ψ)` and either `cut` to
`InputReason::NonManifold`. `bar_cut` is the split-order property's operand
(ADR-0009): a box cut by one or two slabs that cross it, drawn together
with a second set of slabs and a resized, re-posed box, so the same
recipe is built twice under a parameter edit that never changes which
entities bound which piece.

`ops::fuse(m, a, b)`, `ops::common(m, a, b)` and `ops::cut(m, target,
tool)` are three selections over that decomposition (ADR-0004), one
algorithm and one table — and `ops::cut_many(m, target, &tools)` and
`ops::fuse_many(m, &bodies)` the same table over N operands in one build
(ADR-0050): `cut` and `fuse` are their one-tool and two-body cases. A
piece of operand `k` is classified against every other operand whose box
holds its interior point (a point outside a box is outside the operand,
with no ray cast) and kept by the operand's row of the table: a fuse keeps
what is inside no other operand, a cut the target outside every tool and a
tool inside the target and outside every other tool, reversed. Tools that
meet one another are refused (`OpError::Unsupported`, naming the two
faces) until their own sections and the triple points on a third operand's
face are decomposed. Every
face of both operands is split in its own (u, v): the pieces of its
loops between consecutive paves and the section edges on it make a
planar arrangement — half-edges ordered around each node by the pcurves'
tangent angle, a tie within the angular tolerance by the signed
curvature, a tie of both `BooleanReason::TangentContact` — whose regions are
walked by taking the next half-edge clockwise from the direction one
arrived from; a cycle turning once counter-clockwise bounds a piece, one
turning clockwise is a hole, assigned by winding to the innermost piece
around it. A cusp, where the walk leaves back the way it arrived within
the angular tolerance, as a blend leaving a line tangent to it does, turns
by ±π as the node's order decided it by curvature: `+π` round a spike when
the leaving curve bends right of the arriving one walked back, never by
rounding. Each piece is classified at `region2::interior_point`
carried to 3D by `classify_point` against the other operand, and the
table decides. A ray has no closed form against a NURBS face, so a piece
whose ray reaches one is `OpError::Unsupported` naming the piece's face
and the first NURBS face of the other operand — the face the ray met —
the NURBS cycle's refusal, never an internal fault (ADR-0026 §5):

| Piece of | `fuse` | `common` | `cut` |
|---|---|---|---|
| A (the target) | kept when outside B | kept when inside B | kept when outside the tool |
| B (the tool) | kept when outside A | kept when inside A | kept when inside the target, reversed |
| a coincident face | once, from A, when the normals agree | once, from A, when they agree | once, from A, when they oppose |

The coincident row is read at a piece classified `On` a face of the
other operand that its own face is coincident with: the two effective
normals at the piece's interior point agree or oppose, and the piece is
kept from A alone, in A's orientation, `Modified` from A's face and
`Generated` from B's, or dropped by both. The images of B's edges split
A's face along B's boundary and B's images split A's, so the piece is
exactly the overlap; a common block is one edge of the result, A's
piece, and every use of B's piece is rewritten to it with a pcurve
fitted to A's curve in that use's translate of the domain (a seam's two
uses get two), B's piece `Modified` into A's. A piece classified `On` a
face its own face is tangent to has its interior point on the ruling
and lies to one side of the other operand everywhere else; which side is
the *curvature rule*. With `n` the other face's effective outward normal
at the contact, each surface leaves the shared tangent plane across the
contact curve as `κ s² / 2` along `n`, `κ` its normal curvature across
the curve (`Surface::normal_curvature`, the second fundamental form over
the first) signed against `n`; the other body lies on the side of its
surface away from `n`, so a piece is inside it exactly when its own `κ`
is below the other's. The two surfaces agree along the curve to second
order, so any direction across it decides the same, and each surface is
read along its own normal crossed with the curve's tangent. For a plane
and a cylinder it is the plane outside the cylinder's surface and the
cylinder on its axis's side of the plane; two parallel cylinders touching
are `−1/R₁` against `+1/R₂` outside and two distinct radii inside, never
equal, since equal radii touching inside share their axis and are
`Coincident`. Equal curvatures, compared exactly, are a touch of higher
order the rule cannot decide, `OpError::Unsupported` naming the pair.
Before any face is
split, every contact is decided at its midpoint by the same rule and the
table: a contact whose two pieces would both survive is two result faces
touching along a curve interior to both, and the operation is
`Degenerate` with `BooleanReason::TangentContact` naming the pair — a hole wall
tangent to a side face, or a `fuse` of two solids that touch along a
line — because the manifold `Solid` cannot carry the slit (ADR-0004). A
touch from outside in a `cut` or a `common` passes: the tool's piece is
dropped, the target's kept whole, and the ruling is no edge — Open
CASCADE imprints it, and `boolean/tangent-outside-cut` states that
convention. A piece `On` an edge or a vertex, or on a face its own is
neither coincident nor tangent with, lies within the tolerance of the
other operand at its interior point — a sliver between a seam and two
section curves beside the point where they cross is within it
throughout — and is decided by the *transversal rule* at a section edge
it has instead: at the edge's midpoint, the direction into the piece is
its surface's normal crossed with the edge's tangent as the piece's loop
walks it, and the piece is inside the other operand exactly when that
direction is against the other face's effective outward normal. The
piece crosses no face of the other operand inside itself, so every such
edge decides the same, and the one read is the one whose two surfaces
are furthest from tangent; nothing in it measures a distance, so no
tolerance decides it. A piece with no section edge of a crossing
pair, or whose surfaces are tangent within the angular tolerance along
every one, is `OpError::Unsupported` naming the pair. The survivors are
grouped into shells by shared edges — none is
`Degenerate` with `BooleanReason::Empty`, or with `InputReason::ZeroThickness` when
what was dropped lay on the other operand (two solids sharing only a
face); an edge piece used by more than two faces, or a vertex two shells
reach, is two lumps touching and `InputReason::NonManifold` naming it, and so
is a vertex whose face uses close into more than one fan — the corners
there, a use arriving and the next leaving, joined where two share an
edge piece — one shell touching itself at a point, as a wall is left
pinched at a singular point of a section; both before anything is
assembled — and several shells are ordered into lumps, each outer shell then
the voids inside it (ADR-0006), by assembling them once into a clone of
the model and reading `arris_check::lumps` of that body, so the order is
B1's own; a split target, a disjoint `fuse` and a cavity are each one
solid. The shells are then assembled through
`Builder::assemble` with every untouched entity of a kept-by-id operand
`Keep`: a face whose loops changed at all, even only by a split edge or
a re-tolerated vertex, is a new face `Modified` from the old; the tool
of a `cut` keeps nothing, every entity of it `Deleted` and each
surviving piece `Generated` from its parent (data-model §Provenance).
A `fuse` and a `common` have no tool: both operands are kept by id, so
an untouched face of either keeps it, and the result's body is `Modified`
from both operands' where a `cut`'s is `Modified` from the target's
alone; a result shell is `Modified` from the operand shells its pieces
came from, and a cavity made of a cut tool's pieces alone is `Generated`
from the tool's shell. Tolerances follow data-model §Tolerances' growth rule and a
piece keeps its parent's.

The keep-by-id assembly and the provenance writer above are
`arris-ops`'s own `rebuild` module (ADR-0004), not the boolean's: a
`Policy` per operand (`Reuse`, an untouched entity kept by id and a piece
`Modified` from its parent; `Regenerate`, every entity `Deleted` and a
surviving piece `Generated` from it, the tool of a `cut`) turns the
pieces a boolean has already decided on into an
`Assembly` and, once `Builder::assemble` returns it, writes the generic
half of their provenance — every operand entity kept, modified or
deleted, the shell reconciliation, a coincident piece's stand-in — from
the same `AssemblySlots` `transform` reads its own outputs through. The
boolean layers its own two relations on top — a section vertex's or
edge's `Generated` from the face pair that made it, meaningless without
`Interferences` — inline, over the `Provenance` the writer returns.
`boolean()` is `rebuild`'s first caller; a blend is its second, through
`rebuild::rewrite`: one operand, its faces kept by id unless their loops
change, its edges and vertices kept unless a new edge is a piece of one
or a face's new loops no longer reach them, the blend faces added after
each shell's own, and the same generic provenance — kept unrecorded, a
piece or a replacement `Modified`, the rest of what is gone `Deleted`,
each shell and the body `Modified` — over which the blend adds its
`Generated` records.

A body is read for such an edit through `body_view::BodyView` (crate-private
in `arris-ops`, the seam shell and offset share with the blend): each face's
effective orientation and shell, every edge's uses and every vertex's edges
in the body's own order, with `outward`, `convex` and `tangent_at` asked
of them.

`ops::fillet(m, body, edges: &[Edge], radius)` blends the listed edges
with a rolling ball, one stripe per edge built in closed form from the
edge's two faces (ADR-0007). The table: two planes blend to a cylinder
of the radius on the line where the faces' offset planes meet — its
frame's `X` at one contact ruling and `Z` along the edge, so the contacts
sit at `u = 0` and `u = π − φ` for normals `φ` apart and `v` is the
edge's own parameter — with the contact on each plane the line at
`r tan(φ/2)` from the edge, a `Line` pcurve there and a ruling on the
cylinder. A plane against a cylinder along a ruling blends to a cylinder
too: the ball's centre is on the plane's offset by `r` and on the
cylinder coaxial with the face at `R − r` or `R + r` — the ball inside
the face's cylinder or outside it — the one of their two lines on the
edge's side of the axis, and its contact with the face is the ruling
through that centre, a `Line` pcurve at fixed `u` in the translate of
the face's own loop; the contacts sit at `u = 0` and at the angle
between them about the blend's axis, no longer `π − φ`. That pair has no
chamfer in the table, and no miter: its contact on the cylinder misses
the other blend's on the third edge, so a corner with a ruling blend is
`VertexBlend`. A plane against a cylinder along a circle — a hole's rim,
a boss's base — is a closed edge and blends with no ends: a torus
coaxial with the cylinder, its centre circle at `R + sσr` for `s` `−1`
on a convex edge and `σ` the side of the axis the cylinder's outward
normal points to, minor radius `r`, one quarter of its tube between the
contacts; it chamfers to the 45° cone through the same two circles at
`distance`. Each contact is the edge's own circle moved along the axis
or widened, so it keeps the edge's frame and range. The blend's frame
has the cylinder's `Z` and its `X` at the edge's vertex, so its `u` seam
— a tube circle of the torus, a ruling of the cone — runs between the
two contacts' vertices in the half-plane of the cylinder's own seam,
which is shortened to the contact on the cylinder; that vertex must
carry no other edge, else `VertexBlend`. A plane against a cone along a
circle coaxial with it — a frustum's rim, a conical boss's base — is the
same construction read in the half-plane bounded by the axis (ADR-0036):
each face's meridian is a line there, the ball's centre is where the two
lines offset by `r` toward the ball cross, and each contact is the foot
of the centre on its meridian, a parallel of its face; the fillet is the
torus of the centre's distance from the axis and minor radius `r`, the
chamfer the cone through the two circles at `distance` along each
meridian, at the chord's angle to the axis. The plane against a cylinder
is the case of one meridian square to the axis and one parallel to it, and
a cone against a coaxial cylinder or cone — a turned part's shoulder — is
the case of neither a plane's: the same two lines, the torus coaxial with
both, or a chamfer cone through the two contacts. A sphere centred on the
axis or a coaxial torus is a circle meridian — the sphere's great circle,
the torus's tube circle — against a plane, a cylinder, a cone or another
of them along a parallel (a dome on a cylinder, a toroidal bead on a
disc, a torus ring cut square to its axis): its offset is the concentric
circle `r` toward the ball, the centre the crossing of the two offsets
nearest the edge, its contact the foot along the radius, a line at
constant `v` on the sphere or the torus; a chamfer's contact on it is at
the chord `distance` from the edge, as Open CASCADE measures it. A sphere
or a torus whose axis is not the edge's (a torus's meridian circle, a
sphere's tilted circle) is `Unsupported` naming the pair, and so is a
chamfer whose chord lands square to the axis or along it, which a circle
meridian can make and no fixture holds yet. Each curved face's seam
is shortened to its own contact as the cylinder's is (a vertex with a seam
of each, and no other edge, is the closed edge's), and a contact at the
axis or past the cone's apex, an offset circle shrunk to nothing or two
offsets that do not cross is `BlendTooLarge`. An open arc of such a circle —
a rim split where a file put its vertices, a D-shaped notch — blends to
the same torus or cone over the arc's own range (ADR-0035), with the
blend's `X` at the arc's start vertex and no seam: its loop is a
rectangle in (u, v). Each end is a corner of three edges trimmed by the
face across. A plane through the cylinder's axis meets the torus in the
tube circle at the vertex's angle and the cone in its ruling, exact on
the plane and a line at constant `u` on the blend; the corner's radial
line on the plane and ruling on the cylinder are shortened to the
contacts, and the face across takes the section between them. A plane
parallel to the axis and off it, or a cylinder about a parallel axis off
it (a second boss), is met by each contact circle at its own angle, in
closed form in the circle's plane — the crossing on the vertex's side —
so each contact keeps its own range; the end is the section of the
blend's exact surface with that face, traced by `trace_section` and
fitted between the two trim points by `fit_branch` (`blend/traced.rs`,
ADR-0037), its pcurves fitted from it, and the face across takes it as
an edge. A cone's end on a plane off the axis (a hyperbola) is a pair the
tracers do not take and stays `Unsupported`, as does any other face
across, naming the blend's surface and that face; an arc that meets
another blended edge anywhere but at a junction (below) is `VertexBlend`.
A torus that would not
be a ring torus, a contact that reaches the axis, a contact or an end
section that leaves its face, or a seam or a corner edge shorter than the
trim is `BlendTooLarge`. `ops::chamfer(m, body, edges, distance)` is the same
operation cut flat: two planes chamfer to the plane through the lines at
`distance` from the edge along each face — its frame's origin on one
contact, `X` across to the other and `Y` along the edge — and every end
segment and corner line is the chord between two points of the
construction, exact on every plane it lies on, never the intersector's.
Convex or concave is read from the
dihedral, and the contact curves come from the construction, never from
the intersector. Each end is trimmed by the face across the corner — at a
vertex of three edges, the plane the corner's other two edges share:
the section is a circle when that plane is perpendicular to the edge
and an ellipse when oblique, exact on the plane and a `Line` or a
fitted `Nurbs` on the cylinder by the oblique-section rule, at the arc's
own tolerance. A cylinder or a cone across — a rib running into a boss —
is pierced by each contact line at the root nearest the vertex, in closed
form; a fillet's cylinder meets it in a quartic, traced along the rulings
by `trace_section` and fitted between the two trim points on the blend's
band (`blend/traced.rs`, ADR-0037), and a chamfer's plane in the conic
the intersector writes exactly, the stretch between the two points on the
chamfer's band; that face's pcurve is placed in its loop's translate at
the corner edge. The corner vertex goes, the corner's other two edges are
shortened on their own curves to the arc's ends, and the face across
takes the arc in its loop. At a *mixed* corner — its two edges of unlike
convexity, a blend running into a step — the trim on the edge of the
blend's own convexity lies past the vertex, and that edge is lengthened
along its own analytic curve to it instead (`blend/mixed.rs`,
ADR-0038): one edge, `Modified`, its pcurves derived again over the new
range, the stretch checked inside the face it runs down; the face across
grows by the region the arc bounds. The face across an end may be several
pieces (ADR-0043): at a vertex of more than three edges whose extra edges
are all sharp and of the blend's sense, and the faces between the two corner
edges a fan walked through the vertex's star, the end is one arc per piece,
each the section a lone face across would give, with a vertex where the
blend's surface crosses each extra edge, which is cut there; the vertex is
`Deleted`. Where both corner edges lead to the one face and other edges
also stand at the vertex (a box set on another's top edge), the end is that
face's single arc, the vertex stays with the edges it does not reach, and
the face's loop splits in two. Two blends meeting at a vertex whose third
edge stays sharp meet in a miter: the two equal-radius cylinders' axes
cross at the ball's one centre, and the miter is the ellipse of the
plane through it bisecting the axes — minor radius `r` toward the shared
face, major radius `r / sin(ψ/2)` for the edges `ψ` apart — from the
point where the two contacts on the shared face cross to the point on
the third edge where the other two contacts meet it, written from the
construction and fitted as a `Nurbs` pcurve on each cylinder; the third
edge is shortened to that point, no arc enters any face, and the miter
edge belongs to both blend faces. The two far contacts meet the third
edge at one point exactly when the two edges' dihedrals are equal (a box
corner, any right-angled prism). Where they differ (a slanted prism's
vertical edge and its cap edge) the corner is two pieces (ADR-0044): the
miter curve runs from the point where the contacts cross on the shared face
to `m`, where it meets the narrower blend's far contact, and a trim arc
then runs from `m` to the wider blend's far contact on the third edge — the
section of the wider blend with the narrower blend's far face, a circle or
line, exact on the plane and fitted on a cylinder. The wider blend is the
one whose far contact reaches farther along the third edge (a pair's
property, not an edge's), the far face takes the arc in its loop, and the
third edge ends at the arc's end; equal dihedrals are the case of an empty
arc. Two chamfers at such a corner meet in the line between the same two
points, and unequal angles with the third edge give the same two pieces
with a chord for the arc. Three blended edges at
a vertex of three planes, every blend convex or every one concave, meet
in a corner, and no corner edge is cut: each face's two contacts cross at
one point, the corner's three points, and each blend ends on the
corner face between the points on its two faces. Three fillets' axes meet
at the ball's one centre, and the corner is the sphere of the radius
about it, tangent to each cylinder along the great circle through the
centre square to its axis — no equal-dihedral condition, as a miter
needs. The sphere's frame has `Z` toward the point of a face square to
the other two, so the side between those two is the equator and the other
two sides meridians, every pcurve on the sphere a line; the meridians
meet at the pole, that point, crossed by a degenerate edge as a
revolve's sphere closes at its axis. A fillet corner with no face square
to the other two would put a side on a tilted great circle with a fitted
pcurve and stays refused, a backlog line. Three chamfers meet in the triangle of the three
points, each side a chord in one chamfer's plane, at any such corner.
The selection follows chains (ADR-0035): at a *tangent vertex* — three
edges, the two faces of the third tangent there, the next edge open, not
itself a tangent dihedral, sharing exactly one face with the blended one,
convex where it is convex and concave where it is concave, and running on
within a right angle — or a vertex of four edges where both of the
blended edge's faces turn (ADR-0039): the next edge sharing no face with
it, and each of the two others tangent there and between one face of each
— or a vertex where the blended edge runs on with nothing turning
(ADR-0041): the next edge sharing both its faces, the vertex carrying no
other edge (the second vertex of a rim split in arcs), only a seam of one
of those faces (the first), or a seam of each where both turn (a cylinder
against a cone) — the blend runs on into the next edge with the same
kind and size, and on from there until a vertex that is not one, so naming
one edge of a chain or all of them gives one result with the same ids.
Two faces are *tangent* for a blend — at a vertex the walk reads, along
the blended edge itself, and at a corner edge — where their outward
normals agree to the angular precision, or where the blend's radius or
distance times the sine between them is within the faces' tolerance
(the default where theirs is smaller): a ball touching one face then
touches the other within the tolerance the blend is built to, so the two
stripes' contacts meet as one ball's would (ADR-0040). A file writes a
tangency to a few `1e-10`, which the angular precision alone reads as a
sharp edge.
Each edge keeps its own stripe and its own blend face. At a tangent vertex
the two stripes are one ball's, and they meet in a *junction*, recorded as
a miter is: the ball's great circle square to the edges' common direction
for fillets, the chord of the same two points for chamfers, from the point
`q` where the two contacts on the shared face meet to the point `p` where
the other two meet on the third edge — a line at constant `v` on a
stripe's cylinder and at constant `u` on a ring's torus or cone, every
pcurve exact; the third edge is shortened to `p`, the vertex goes, and no
face across takes an arc. At a vertex of four edges the runs share no
face, and `q` lies on the second tangent edge instead: each contact meets
the other run's contact across the tangent edge between their faces, and
both tangent edges are cut, `Modified`, at their points (ADR-0039 §2).
Where the runs share both faces (ADR-0041 §2) each contact meets the other
run's on its own face, `q` on one face and `p` on the other; nothing is cut
at a vertex of two edges, and at the seam's vertex the seam is shortened to
`p`, as a closed edge's is; where both faces turn, each seam is cut at the
contact on its own face. Every vertex of a rim split in arcs is one of
these two, since a seam ends on the rim, so the rim blends as the ring it
is, one face per arc.
A chain that reaches a pair outside the table is
refused naming that pair and the edge the walk reached. The edges are
blended in the body's iteration order, whatever order they are listed in, so the result
and its ids are the same for any order of one set; disjoint blends share
nothing but the faces across their ends, where a corner edge between two
of them is cut at both its ends in one edge. A contact line or an end arc that
would leave its face through an edge that is not the corner's own, or a
corner edge shorter than the trim — decided in the face's own (u, v)
through `FaceDomain::side` at `check_samples` interior parameters, a
contact inside its face and an end arc inside the face across where the
blend and both corner edges share a convexity — a convex blend between two
convex corner edges — and outside it where they differ, a convex blend at
concave corner edges (a rib's root on its plate) or a concave blend at
convex ones (a pocket's rim), the arc lying in the hole the footprint
leaves or the corner gained, and outside it at a mixed corner; a trim
past the vertex anywhere but on a mixed corner's edge of the blend's
convexity, on a fitted curve, or along a stretch that leaves its face —
the same at a face across of any kind the end takes (a plane, a cylinder
or a cone, ADR-0037) and at a ring's open arc end — is
`BlendReason::TooLarge` naming the edge and the face or edge the blend
runs out of. An end at a cusp — a vertex of three edges where the
next edge leaves the way the blended one does, the two walls tangent
along the third edge, the spine, both edges of one sense so that both
walls lie on one side of the face they share (a crescent's sliver of
material, or its pocket) — is cut by the next wall (ADR-0042): the
contact on the shared face is trimmed where it crosses the next edge,
in closed form, and the contact on the blended edge's wall where it
meets the spine, at the vertex's own angle on a ring and where the two
lines cross on a stripe, the next edge and the spine shortened to those
points, and the cut between them is the stripe's section with the next
wall, traced and fitted as an end on a curved face across is, its
stretch ending at the node where a fillet touches that wall on the spine
— a branch that starts and ends at the node reaches it at either end. A
tangent dihedral, or an end at a vertex where a corner edge's two faces
are tangent but which is neither a tangent vertex nor such a cusp — an
overhang tip, a cusp whose edges are of opposite senses, its walls on
either side of the shared face, which Open CASCADE caps with fitted
surfaces; a cusp whose next edge is blended too, which takes a corner
patch; the next edge turning back with no cusp, or itself a tangent
dihedral — is `BlendReason::TangentChain`; a
vertex of
other than three edges (but the chain junctions of two and four above), a miter
of blends not both convex or both concave, a miter with a ruling
plane–cylinder blend, or a corner of three blended
edges whose faces are not all planes, whose blends are mixed, or — three
fillets — none of whose faces is square to the other two, is
`BlendReason::VertexBlend` naming the vertex; a surface pair
outside the table — two cylinders, a torus against a cylinder off its axis
or a plane through its axis, a plane against a cone that is not a coaxial
circle (an oblique plane's ellipse, a plane through the apex along a
ruling), a NURBS face — or a face across an end the closed forms and the
tracers do not take, is
`OpError::Unsupported` naming the kinds and the faces; an empty list,
an edge listed twice and an edge of another body are `BlendReason::NoEdges`,
`RepeatedEdge` and `EdgeNotInBody`. A blend that meets a third face while
its contacts stay inside their faces is not detected by the operation:
S5 catches it in the corpus. Provenance is rooted at the edge with no new
`Role` (data-model §Provenance).

`ops::offset_faces(m, body, faces: &[Face], distance, control)` moves the
chosen faces of a solid along their outward normals by a signed distance —
positive adds material — and returns the result with its provenance
(ADR-0048). It works as `blend` does, over `body_view` and
`rebuild::rewrite`, by phase in `offset/`. The chosen faces are closed over
tangent edges (`chain`), so a filleted edge's blend and the face beyond it
move with the one chosen, as a consumer's press-pull selects its chain; an
edge between two moved faces that are tangent is carried along their shared
normal, a line shifted and a circle re-radiused and moved along its axis,
and any other such curve is `NoExactOffset`. Each moved face lies on
`Surface::offset` of its own surface, the same kind (a plane's plane, a
coaxial cylinder or cone, a concentric sphere, a torus of the same major
radius). Each vertex is then the point nearest its old one on every surface
around it once moved (Gauss–Newton on signed distances, a seam adding the
plane it lies in), and each edge the branch of the section of its two new
surfaces, from the intersectors every quadric pair already has, that passes
through both new ends, running the old edge's way: the join is sharp, a
moved and a fixed face meeting where the moved face's offset meets the fixed
face's own surface. Topology is kept, so what would change it is refused by
name, the model untouched: a face with no exact offset, a surface driven
through zero, an edge or face that vanishes or turns inside out, a vertex
that splits, a dragged face that no longer meets a fixed neighbour
(`Gap`). The result is checked at `Level::Full` in every profile (§Checker),
a crossing far from what moved being `SelfIntersects`.

`ops::shell(m, body, openings: &[Face], thickness, side: ShellSide, control)`
hollows a solid to a wall of constant thickness (ADR-0049), by phase in
`shell/` over `offset/`: `offset::pieces` is the offset's phases up to the
rewrite (the chain, the moves, the vertices, the edges, the new faces),
which `offset_faces` rewrites in place and `shell` assembles into a second
skin, so no phase is copied. `thickness` is positive and `ShellSide` says
which way: `Inward` keeps the body's faces as the outside and puts the skin,
reversed, inside them; `Outward` makes the body's faces the cavity,
reversed, and grows the skin outside them. The skin is the offset of every
face but the openings, each on the offset of its own surface and joined
sharp, so what the offset refuses reaches the caller as `Reason::Offset`
naming the entity. The result goes through `Builder::assemble`, not
`rewrite`, since it adds faces and a shell (`shell/assemble.rs`): the body's
faces, the skin, and for each opening one rim face on the opening's own
surface bounded by the body's loop and the skin's. Where two openings share
an edge, or one meets itself across its seam, the rims cancel along it
(`shell/rim.rs`): what is left at each end is a piece of the one curve
shared by both rims, one loop each, and the edge's middle, across the mouth,
is in no face. With no openings the skin closes on itself and the body has
two shells, the void inside the outer (`ShellNesting`). `Reason::Shell`
holds what only the shell can get wrong: `NoWalls` (every face an
opening), `RepeatedOpening`, `OpeningNotInBody` and `OpeningDragged` (an
opening tangent to a wall, which the wall's move would drag). The result is
checked at `Level::Full` in every profile, as the offset's is.

Sweeps take a planar `geom::Profile` — an outer loop and holes of lines
and arcs in a plane's own (u, v), validated and oriented by
`Profile::edges` (data-model §Profiles) — so a consumer's sketch never has
to become topology before it becomes a solid. A sweep's faces are known
outright, so it enters the builder through `Builder::assemble` as
`transform` does, in one fixed order — vertices per loop in walking order
(the start ring, then the end ring), edges (start, end, rises), faces
(start cap, end cap, sides per loop per segment) — so the ids are a
function of the profile alone; every tolerance is `default_tolerance`,
and every entity is `Generated` from a `Role` naming the part of the
sketch it came from (data-model §Provenance, `SweepPart`).

`ops::revolve(m, &profile, axis: Axis, angle)` sweeps the profile about
an axis lying in its plane within the tolerances
(`SweepReason::AxisNotInProfilePlane` otherwise): a partial turn with two flat
ends — the profile face, its outward normal against the turn, and its
copy rotated by `angle` — or a full turn with seams when `angle` is within
`angular_tolerance` of `2π` (`SweepReason::AngleAboveTurn` above it,
`NotPositive` at or below zero). The profile lies wholly on one side of
the axis (`SweepReason::ProfileCrossesAxis` across it, `ZeroThickness` within
`default_tolerance` of it everywhere), and an arc whose centre is off the
axis and nearer it than its radius is `SweepReason::SpindleTorus`. An
elliptic segment or a full-ellipse loop is `SweepReason::EllipticRevolve`
naming the first of them in the sketch's own order, the surface it would
sweep having no variant (ADR-0014); an ellipse whose radii agree within
`default_tolerance` is a circle edge and sweeps as one. It may
touch the axis: a vertex within `default_tolerance` of it is *on* it and
sweeps no rise — one vertex, shared by both flat ends of a partial turn
and not made in a full turn, since no face keeps it there — and a line
segment with both ends on it lies *along* it and sweeps no face: in a
partial turn the one edge both flat ends share, in a full turn nothing,
so a rectangle with a side on the axis turns into a solid cylinder of
three faces. A face closing at a vertex on the axis — a cone's apex, a
sphere's pole — holds a degenerate edge there in place of the rise, its
pcurve the line at the singular `v` over the rise's range, one per face
closing there, and a full turn keeps the vertex for it: the first
degenerate edges an operation makes, which `Builder::assemble` takes used
once and the Euler line leaves out. A full turn touching the axis at a
vertex with no segment along it is `InputReason::NonManifold`, since the
surface would touch itself there; a partial turn's flat ends make that
vertex manifold. Every other segment sweeps one face: a segment
parallel to the axis a cylinder, perpendicular a plane (an annulus, or a
sector of one), oblique a cone with its apex on the axis; an arc centred
on the axis a sphere, elsewhere a torus of `R` its centre's distance and
`r` its radius. Every pair of faces a revolve makes shares its axis or
has a plane through it, so S5 and B1 decide them by the meridian arm and
`classify_point` casts against them by the line arms (ADR-0008); a
revolve's faces are all boolean operands, its spheres and tori
included (§Operations). The
surfaces of revolution share one frame: origin on the axis, `X` the unit
radial from the axis into the profile's plane — so `u = 0` *is* the
profile plane and every seam lies in it — `Z` the axis direction, except
a cone whose radius shrinks along the axis, which takes `Z = −axis` since
the data model's cone grows along `+Z`. Every vertex sweeps a circular
*rise* about the axis over `[0, angle]`; each side face's loop is start
edge, rise, end edge, rise — the end edge the start edge's second use
across the seam in a full turn, a rise at a vertex on the axis left out,
so a side reaching the axis closes there — walked that way when the material sweeps
along the profile's normal and the other way otherwise, and the face's
use orientation is the surface normal against the segment's outward
in-plane normal at its midpoint (material on the loop's left), uniform
over a face by construction. Every pcurve is exact through `pcurve_on`,
then translated by whole periods into the copy of the domain the loop is
written in (the profile plane at `u = 0`, a seam's second use one period
on), since `pcurve_on` reports a periodic parameter in `[0, 2π)`. A full
turn closes the profile into one lump (ADR-0006) of a shell per *chain* —
a maximal run of a loop's segments off the axis, a loop that never lies
along it being one chain: the chain whose ends span every other's along
the axis is the lump's outer shell, stored first, and every other chain —
a hole, a notch cut in from the axis — a void directly inside it, stored
by loop and lowest segment and `Generated` from `SweepPart::Cavity {
loop_index, segment }`, `segment` the lowest index the consumer wrote
among its segments; a partial turn's flat ends join everything into one
shell.

`ops::extrude(m, &profile, direction: Vec3, length)` sweeps the profile
along its plane's normal, either way: `direction` is the normal or its
opposite within `angular_tolerance` (`SweepReason::DirectionNotNormal`
otherwise — an oblique extrusion of an arc is a cylinder of elliptical
section, which `Surface::EllipticCylinder` (ADR-0014) could hold, but no
sweep builds it yet: a backlog line, ADR-0047), `length` finite and above
`default_tolerance` (`NotPositive` at or below zero, `ZeroThickness`
within the tolerance). The sweep is the plane's exact normal, never the
caller's rounding of it. The profile face keeps its plane's frame
whichever way the sweep goes — a planar face's frame *is* the answer a
consumer reads back — and is the cap whose outward normal opposes the
sweep; the other cap is its copy translated by the sweep. A line segment
sweeps a plane whose `X` is the segment and `Y` the sweep, an arc a
cylinder whose frame is the arc's centre with `Z` the sweep and `X` the
profile plane's, so a circle loop's seam stands at its vertex's rise as
Open CASCADE's does; every vertex sweeps a straight rise. Each side
face's loop is start edge, rise, end edge, rise — a circle loop's one
rise its seam, used twice — walked that way when the sweep runs along the
profile's normal and the other way otherwise, with the face use and the
pcurves decided as for a revolve. Both sweeps share the cap, side-face
and provenance construction. There is no `planar_face` operation — a
sheet of one face is the healing cycle's sheet bodies, and the cap construction is the
sweeps' private helper.

### Errors

`OpError` is a `thiserror` enum and every variant names the entities
involved, so the message a consumer shows — or the agent reads — says
*which* face pair, *which* edge, not "boolean failed":

| Variant | When | Carries |
|---|---|---|
| `InvalidInput` | an input body fails the checker (checked in debug builds before the operation starts, and in release when the `paranoid` feature is on) | `Body`, the `Report` |
| `Unsupported` | the exhaustive dispatch reached a surface or curve pair the kernel has no formula for yet — a boolean's face pair with a NURBS face in it, a piece whose classifying ray reaches a NURBS face, a tangent contact or an `On` piece that neither the curvature rule nor the transversal rule decides, a blend's face pair outside its table or the face across a blend's end | the two `GeomKind`s with their entities |
| `Degenerate` | the requested result has no valid representation: a parameter that makes no geometry (`InputReason::NonFinite`, `InputReason::NotPositive` naming it — a zero radius, a box whose `min` is not below its `max`, a revolve angle at or below zero, a zero extrude direction; `SweepReason::AngleAboveTurn` past `2π`), a zero-thickness intersection or an extrude of zero length (`InputReason::ZeroThickness`), a revolve whose axis is off the profile's plane (`SweepReason::AxisNotInProfilePlane`), whose profile crosses its axis (`SweepReason::ProfileCrossesAxis`) or lies within the tolerance of it everywhere (`InputReason::ZeroThickness`), or whose arc's circle crosses it (`SweepReason::SpindleTorus`), or whose profile has an elliptic segment (`SweepReason::EllipticRevolve` naming the loop and segment, ADR-0014); an extrude off its plane's normal (`SweepReason::DirectionNotNormal`); a boolean that selects no material (`BooleanReason::Empty`: a target inside its tool, a `common` of disjoint operands); a `cut_many` with no tool or a `fuse_many` of fewer than two bodies (`BooleanReason::NoTools`) and a body twice among a boolean's operands (`BooleanReason::RepeatedOperand`, naming it, ADR-0050 §8); result shells that would touch along an edge or at a vertex, a shell that would touch itself at a vertex whose faces close into more than one fan — a wall pinched at a singular point of a section — or a full revolve touching its axis at a vertex with no segment along it (`InputReason::NonManifold`, naming the shared edges or vertices, none for a sweep); faces touching along a curve interior to both result faces (`BooleanReason::TangentContact`); a section passing a face's apex or pole without running through it, nearer than the face's (u, v) polygons resolve (`BooleanReason::BesideSingularity`, naming the two faces and the vertex, ADR-0021); a blend asked for no edges (`BlendReason::NoEdges`), for an edge twice (`BlendReason::RepeatedEdge`) or for an edge of another body (`BlendReason::EdgeNotInBody`), one that leaves its face, outruns a corner edge or a seam, or around a closed edge would need a torus that is not a ring torus or a contact reaching the axis (`BlendReason::TooLarge`), one at a tangent dihedral or ending on a tangent corner edge (`BlendReason::TangentChain`), or one at a corner the closed forms do not cover (`BlendReason::VertexBlend`, ADR-0007); an offset of no faces, of a face twice or of another body's face (`OffsetReason::NoFaces`, `RepeatedFace`, `FaceNotInBody`), one that makes an edge or face vanish or turn inside out (`Vanishes`) or splits a vertex into an edge (`VertexSplits`), moves a face with no exact offset of its kind (`NoExactOffset`) or drives a surface through zero (`SurfaceCollapses`), pulls a dragged face clear of a fixed neighbour (`Gap`), or builds a body whose faces run into each other (`SelfIntersects`, found by the checker's global level), ADR-0048 §3, §6, §7; a query on a body that is not a `Solid` (`InputReason::NotSolid`); a projection handed a face, shell or body (`QueryReason::NotProjectable`), a degenerate edge (`QueryReason::DegenerateEdge`), or a curve that projects to a point or a segment (`QueryReason::ProjectionCollapses`); `face_frame` on a curved face (`QueryReason::NotPlanar`); `frame_at` at a `(u, v)` outside the face's domain (`QueryReason::OutOfDomain`) or at a singular one with no normal (`QueryReason::Singular`) | the entities (none for a primitive or a sweep) and a `Reason`, grouped by the operation that raises it: `Input` (`NonFinite`, `NotPositive`, `ZeroThickness`, `NonManifold`, `NotSolid`), `Sweep`, `Boolean`, `Blend`, `Offset` and `Query`, each its own enum. `Reason::name()` is the leaf's stable name (`BlendTooLarge`), which the refusal histograms key on; `Display` is the leaf's own message |
| `Profile` | a sweep's sketch is not a valid profile: `Profile::edges` refused it (data-model §Profiles). An invalid profile has no entities to name, so it is neither `InvalidInput` nor `Degenerate` | the `ProfileError`, naming the loop and segment |
| `Tolerance` | the result would need an entity tolerance above `Precision::max_tolerance` | the entity, the tolerance it wanted |
| `NotFound` | an id does not resolve in this model (wrong model, or compacted away) | the `AnyId` that failed to resolve itself, never an entity that merely holds it |
| `Unkeyed` | `build` was handed a live slot of its builder that its `BuildKeys` gives no key: the record would have an output with no origin | the `BuildSlot` (a vertex, edge or face slot, or a shell index) |
| `Rejected` | `build` refuses the consumer's topology as a solid: `Builder::finish` refused it (`Rejection::Builder`), a slot is an entity `assemble` kept from another body (`Rejection::Kept`), or the finished body fails the checker at `Full`, in every build profile (`Rejection::Checker`). The input's fault, never `Internal` | a `Rejection` — the `BuildError`, the `BuildSlot` or the `Report` |
| `Interrupted` | the consumer's `Control` stopped the operation: its poll answered true (`Stop::Poll`) or its budget of steps ran out (`Stop::Budget`), ADR-0030 | the `Interrupted` value: the cause and the steps taken |
| `Internal` | a kernel bug the operation caught: the checker rejected its own output, the builder refused a step of its fixed sequence, a frame could not be placed from inputs it had validated, a point it had to classify could not be, a geometry query failed on validated input for a reason other than a missing closed form, a section edge crossed a seam the seam's own hit should have paved, a piece of a coincident face pair's edge matched no piece of the edge it lies along, the (u, v) arrangement of a face was not the subdivision the pave model promised (`SplitFault`: a dangling section edge, a cycle not turning once, a hole inside no piece, a piece with no interior point, a pave at an edge's end), the shells a boolean kept did not nest into lumps, an operation's own fixed sequence broke an invariant it should have kept — an internal lookup by index or key, never a model id, found nothing (`Fault::Invariant { what }`), a sweep's own later step needed an entity its earlier step did not make for a segment (`Fault::Unmade { segment }`), a surface had no normal at a point on a face an operation needed one at, every partial derivative degenerate where the checker's own tolerances should have ruled that out (`Fault::NoNormal { face }`), or a profile edge's curve was not one of the kinds `Profile::edges` makes (`Fault::ProfileCurve(GeomError)`) | a `Fault` — the `Report`, the `BuildError`, the `FrameError`, the `ClassifyError`, the `GeomError`, the two faces of the seam crossing, the edge and face of the unmatched common block, the `SplitFault` naming the face, the `LumpError`, or one of the four bookkeeping variants above |

The STEP reader's failures are its own and never an `OpError`: a file
that is not Part 21 is `ReadError::Parse(Part21Error)`, naming the line,
the column and the instance, and fails the whole file; past parsing,
every solid is `Ok` or a `Refusal` naming the `#id` where it stopped —
the geometry outside the subset (`Offset`, `Composite`, `CurveBounded`,
`DegenerateTorus`, `SelfIntersectingTorus`, `Unsupported`, `Degenerate`),
the file's units (`NoLengthUnit`) or references (`Malformed`), its
topology (`Topology`, `Pcurve`, `OpenLoop`), a gap past the cap (`Gap`)
or a body the checker rejects (`Invalid`, carrying the `Report`).
`Refusal::kind` is the fieldless `RefusalKind` a histogram counts, and
`RefusalKind::ALL` lists every kind, held to the enum by an exhaustive
`index`.

`Internal(Fault::Checker)` is returned only in release builds with
`paranoid` on, since a debug build panics on the same report (below) —
except for `build`, whose failing body is `Rejected` in every build;
every other fault is returned as `Internal` in any build. A degenerate *result* that the
consumer might reasonably want anyway (the flush intersection that is a
face, not a solid) is `Degenerate` with a reason, never a silently empty
body: the kernel does not decide what fail-soft means. Every operation
runs inside `Model::transaction`, so on any `Err` the model — its ids
included — is as it was, `Interrupted` (the consumer's poll or budget,
in `OpError`, `GeomError`, `MeshError`, `ReadError` and `BodyError`
alike) included. It is a stop the consumer asked for, so it is never
`Internal` and the operation names no entity in it.

## The checker

`arris-check` is its own crate so that no algorithm crate can skip it by
accident and so that its dependency list stays at exactly `arris-topo`.
`arris_check::check(&model, body, Level) -> Report` (`topo` sits below the
checker, so it is a free function over the model, not a method) returns
every violation with the entity that violates it; `Report::is_ok()` is
what every test asserts and what every operation asserts on its own
output. A body handle that does not resolve is one M1 line; a reference
that does not resolve is reported under M1 and skipped by every other
row; adjacency is read off the body's own entities, so every row stands
without the arena's indices and M2 alone speaks for them. Every report also carries the body's Euler–Poincaré line,
`Report::euler()`, which is a line and not a violation. The invariants are
listed in data-model §Invariants; each has a `Violation` variant, a test
that constructs it and sees it reported, and a level:

- `Level::Fast` — combinatorial and local geometric checks (ids resolve,
  loops close, orientations compose, tolerances are ordered, pcurves match
  their 3D curves at sample points). Linear in the body. This is what runs
  after every operation.
- `Level::Full` — adds the global checks: an edge does not cross itself,
  the loops of a face do not cross, faces of a shell intersect only at
  shared edges, shells nest, a solid encloses positive volume. Not
  linear. Runs on demand, in the fixture corpus and in `/close-cycle`.

`arris_check::domain::FaceDomain::of(&model, face, tolerance) ->
Result<FaceDomain, NotFound>` is the one answer to "where is this (u, v)
point on this face": a face's loops read once as polygons within a chord
of their pcurves, its (u, v) and 3D boxes, and `FaceDomain::side(uv) ->
(Side, Vec2)`, which tries every period translate of the surface
(`domain::shifts`) before answering `Outside`, each try answered by the
domain's `region2::SideIndex` in the segments near the point rather than
a walk over tens of thousands of them, so a periodic face's loops
need only be written in one translate and S5, B1, the boolean and
tessellation can never disagree about a point past a period —
`FaceDomain::winds_around` and `FaceDomain::boundary_entity` resolve a
point on the boundary to its vertex, else its edge, the same way, also
trying periods on a closed edge; the free `domain::boundary_entity(model,
edges, point)` does the same over a whole body's edges, which the
classifier below asks of one. It replaced `Checker::face_side` and
`faces_fine`, the classifier's own polygons, `check::uv_bounds`, the
boolean's `FaceInfo`'s domain part and mesh's padded (u, v) box — one
definition, not five (ADR-0004). The checker's `Full` rows and the
classifier build it at the model's parametric tolerance; a boolean builds
it at the pair's own face tolerance — S5's `regions_overlap` and
`curve_is_interior_to_both`, and the classifier's boundary and ray tests,
take the larger of the two faces' own tolerances at a pair, not the
model's, so a face modelled looser than the default is judged by its own
tolerance there too. It lives in `check` because the classifier and B1
are already there, and `ops` and `mesh` already depend on `check`.

`arris_check::classify::Classifier` is B1's ray cast made public and
complete, built once and asked of many points: `Classifier::of_body(model,
body)` reads the body's faces — their `FaceDomain`s — once, and
`.classify(point) -> Result<Classification, ClassifyError>` answers
`Inside`, `Outside`, or `On(Shape)` naming the most specific entity the
point is within the tolerance of — the vertex, else the edge, else the
face. `classify_point(&model, body, point)` is the one-shot wrapper over
a fresh `Classifier`. The boundary test comes first, by the entities' own
tolerances; only a point that is on nothing is cast for, and then the
eight fixed directions are tried in order, a direction abandoned on a
boundary, tangent or coincident hit, with all eight abandoned reported as
`ClassifyError::Undecided` naming the body and the point. A ray meets
every analytic surface by closed form (ADR-0008), so only a NURBS face is
`ClassifyError::Geometry`; a hit at a cone's apex or a sphere's pole lands
on that face's degenerate edge, a boundary like any other, and abandons
its direction. B1 is the same
code over one shell's faces — `Checker::shell_classifier` and `nesting`
build one `Classifier` per shell rather than one per ray, and a boolean's
piece selection builds one per operand rather than one per piece it
classifies — so the row that proves a shell nesting and the predicate
that decides which piece of a split face a boolean keeps can never
disagree about a point (ADR-0004). It lives in `check` because that is
where B1 already was, and `ops` depends on `check`; the facade re-exports
it.

`arris_check::flux::face_flux(&model, face, integrand) -> Result<f64,
FluxError>` is the one `∬ integrand(P, ∂P/∂u × ∂P/∂v) du dv` over a
face's region, by Green's theorem through its loops
(`geom::integrate::region_integral` on the surface's own `surface_grid`):
B2's enclosed volume, `arris_check::lumps`'s per-shell volume (both
`P/3`'s flux, whose divergence is one) and `ops::measure`'s mass
properties (the same integral with the density's and the moments' fields)
are this integral with a different field each time, so the checker and a
measurement can never read one face's region differently.
`ops::measure::face_area` stays in `ops`: it integrates the surface's own
area element, not the flux of a vector field.

`arris_check::lumps(&model, body) -> Result<Vec<Lump>, LumpError>` is
B1's nesting as a value (ADR-0006): each outer shell of a solid with the
voids whose innermost container it is, in the order the body stores the
outer shells — what the STEP writer writes a solid entity per and the
corpus counts as `solids`. It runs B1's own code over the body and nothing
else, so it is `Ok` exactly where B1 passes and decides, and otherwise
names B1's first fault or its undecided rows; a body of one shell is one
lump and casts no ray. Lumps are derived, never stored on the body.

A `Full` row the kernel has no closed form for — a face pair whose
surfaces the intersector cannot intersect, a shell no containment ray
could be classified against — is never guessed at and never quietly
passed: it is listed by `Report::unchecked()`, printed with a `?` after
its row number, and left out of `is_ok()`. An operation that cannot afford
an undecided row asks for the list.

**In debug builds every operation runs `Level::Fast` on its output before
returning `Ok`, and panics with the report if it fails.** A checker failure
after an operation is a kernel bug, and the kernel's own invariants are the
one place a panic is allowed (`.agents/rules/kernel.md`). A test that needs
to build an invalid body — every checker test does — constructs it through
`arris-topo`'s raw insert, which the checker does not guard, and says so by
name. `ops::build` is the exception: the body it finishes is the
consumer's topology, not the kernel's work, so it runs `Level::Full` in
every build and refuses a failure as `OpError::Rejected` instead of
panicking.
`ops::offset_faces` and `ops::shell` are the others: a push can build a body that is well
formed edge by edge and crosses itself far from the face it moved, which
no local step sees, so it checks its own result at `Level::Full` in every
build (skipping the debug guard, which `Full` includes) and refuse what
the global level finds as `OffsetReason::SelfIntersects`, naming the faces
the report names, the model untouched (ADR-0048 §7, ADR-0049 §6). A violation the
global level does not own is still `Internal`.

**In release builds nothing runs unless asked.** `arris_check::check` is
public and cheap enough for a consumer to run after every feature; the
`paranoid` feature turns the debug behaviour on in release, returning
`OpError::Internal` instead of panicking.

The checker never repairs. Healing is an operation — the healing cycle's
— and it returns provenance like any other.

## Geometry dispatch

`Surface` and `Curve` are open enums, not trait objects (`SEED.md` §9).
Every intersection, projection and classification is an exhaustive `match`
over the pair of variants, so adding a variant makes every dispatch fail to
compile until it is handled. A pair without a closed form is traced
exactly and fitted (below), and a pair nothing decides — a NURBS surface
against any surface, two NURBS curves — is a `GeomError::Unsupported` arm
naming both kinds (`OpError::Unsupported` once an operation wraps it with
the entities) — never a wildcard falling back to a generic marcher where
a closed form exists. The results are
enums too: `SurfaceIntersection::{Empty, Coincident, Meets { curves,
points }}` for a surface pair — each curve and isolated point of a
`Meets` a `Crossing` or a `Touch`, the two kinds together in one result
(ADR-0018) —
`CurveSurfaceIntersection::{Points, Coincident}` for a curve against a
surface and `CurveIntersection::{Points, Coincident}` for two curves, so
a caller matches the case rather than counting curves or points. A pair
may be supported in part: two cylinders meet by closed form when their
axes are parallel (rulings, or `Coincident` or `Empty` when coaxial),
when their axes cross at equal radii (two ellipses), and when skew axes
are further apart than the two radii (`Empty`); crossing axes of unequal
radii and skew axes within the radii meet in a quartic no variant
carries, which `trace_quadrics` traces exactly by the rulings of one
cylinder and the intersector fits to `Curve::Nurbs` within a fraction of
the tolerance, clipped to the region the caller passes (ADR-0018,
`docs/DATA-MODEL.md` §Curves has the table). Every pair
with a cone, a sphere or a torus in it is supported where the two share
an axis — a plane perpendicular to it, a cylinder, cone or torus on it, a
sphere centred on it, and every plane–sphere and sphere–sphere pair — by
one arm over the meridian sections in the plane through the axis rather
than a table of pairwise closed forms (ADR-0008): circles about the axis,
points on it, `Coincident` or `Empty` — and where a plane holds a cone's
or a torus's axis, its meridian: two rulings through the apex, or two
tube circles, a partial revolve's flat ends. In general position a
plane meets a cone in an exact conic — an ellipse, or a parabola's or a
hyperbola's branches as rational quadratic NURBS over the region — and
a cylinder, a cone or a sphere meets a cone or a sphere in a traced and
fitted section. A torus sharing no axis with the other surface has no
ruling to walk, so its section is traced by the second tracer,
`trace_torus`, in the torus's own parametrisation — the torus put into
the other surface's implicit polynomial in Bernstein form over sixteen
quarter-turn patches, the branches proven by isolated turning and
singular points rather than sampled, a tube circle the other surface
holds returned as an exact `Curve::Circle` — and fitted by the same
code path as a ruled pair's, over the whole torus rather than a region
(ADR-0019). Both tracers are reached through `trace_section`, and a
branch, or a stretch of one, is fitted by `fit_branch`: the intersector's
sections and a blend's end that the face across cuts in no closed form
share the one dispatch and the one fit rule (ADR-0037). The elliptic cylinder an extruded elliptic segment sweeps
(ADR-0014) is decided against a plane in every pose and against a
cylinder or another elliptic cylinder with a parallel axis by closed
form, through the two sections in the plane across the axes — the
pairs an extrude's faces make — against those on other axes, a cone or
a sphere by the ruled tracer, and against a torus by the torus's; the
pave model takes a face on it as an operand face like any other, its
pcurves fitted over its own projection (ADR-0021). A circle or an ellipse — the edge
an operand face already carries — is decided against every analytic
surface: by the closed forms of the table where there is one; against a
cone, a sphere or an elliptic cylinder in any plane by the quadric's
polynomial along the conic, a trigonometric polynomial of degree two
whose extrema leave the signed distance monotone in between; and against
a torus by the conic's four exact rational quarter arcs put into the
torus's polynomial, degree eight in Bernstein form — the verdict taken
on that distance as everywhere else (`docs/DATA-MODEL.md` §Curves). Two
coplanar conics that are not the same conic meet at the roots of one
quartic in the second's plane (`conic2`), and `curves_coincide` is that
verdict alone. A
`Curve::Nurbs` — the fitted section edge the next boolean meets — is
decided against every analytic surface by its spans put into the
surface's implicit polynomial in Bernstein form, the extrema isolated by
subdivision and the verdict taken on the exact distance, as the closed
forms take it (ADR-0018, `docs/DATA-MODEL.md` §Curves), and against a
line, a circle or an ellipse through the same arm on a plane — the
conic's, or two through the line — and in a conic's plane on the
cylinder the conic is the section of; two fitted curves are
`Unsupported`, the tracer's points saying where one pair's meet. The
NURBS surface variant is one arm like the others; a NURBS–NURBS marcher, when it
comes, is what that arm calls, and analytic pairs never route through
it.

## Tessellation

`arris_mesh::tessellate(&Model, Body, chord, &Control) -> Result<TriMesh,
MeshError>` turns a body into a triangle mesh with a `FaceRange` per
face and an `EdgeRange` per edge, both in the body's iteration order
(data-model §Adjacency and iteration). The chord tolerance is the
consumer's request — a number like a render's resolution, not a model
tolerance — validated finite and positive (`MeshError::Chord`); in debug
builds the body passes the checker at `Level::Fast` first, as every
operation's input does (`MeshError::InvalidInput`), which is why `mesh`
depends on `check`. The mesh guarantees (ADR-0003):

- Positions are `f64` and exact evaluations of the geometry: a topo
  vertex's point, an edge's curve at a sampled parameter, a surface at
  an interior grid point. Nothing is welded or snapped.
- A topo vertex is one mesh vertex and an edge's samples are one index
  run, so a mesh of a `Solid` is closed by construction. Every edge is
  discretised once, at `n` uniform parameters — the largest of its
  curve's `chord_segments` at the chord and, per coedge, the count that
  keeps each step's (u, v) travel under the face's surface's
  `chord_steps` — and its `EdgeRange` is the polyline from its start
  vertex to its end vertex along its curve's parameter.
- A face's loops are the *same* parameters through each coedge's
  pcurve, triangulated in (u, v) by the constrained Delaunay
  triangulation of `arris_mesh::cdt` and mapped back to the shared
  indices. A seam edge is discretised once and its indices appear in the
  wall's triangles twice, once from each copy of the pcurve; a
  degenerate edge's (u, v) segment maps to one index and the triangles
  that collapse are dropped.
- Triangles are counter-clockwise seen from outside, by the face use's
  effective orientation against the surface normal.
- Same body, same chord, same mesh on every platform, with the
  `parallel` feature on or off.
- A face whose surface curves in both directions — a sphere, a torus, a
  NURBS surface — carries interior points on a uniform (u, v) lattice at
  its `chord_steps` spacing, those its loops wind around; a ruled
  direction has an infinite step, so a plane, a cylinder and a cone take
  none and their loops' own samples bound the chord. The rings and the
  lattice are scaled by the surface's mean speeds before the
  triangulation, so Delaunay's criterion measures distance on the
  surface and not in the parameters (ADR-0003), and a ruled direction is
  then flattened to a thin ribbon — an eighth of a chord step of the
  curved parameter — so the criterion is left to the parameter the chord
  bound is written in (ADR-0005). No triangle of a face on a cylinder
  travels more than one chord step of the turn, whatever the shear of its
  region: the wall of a hole drilled at an angle is a strip oblique to
  the ruling, and in an isometric domain Delaunay would join its boundary
  across the hole rather than column by column.
- A face whose loops are not the simple nested polygons the checker
  promises is `MeshError::Face` with the `CdtError` naming the segments;
  a corner block that does not fit the mesh it is offered to, and a face
  no point of which has a surface normal, are `MeshError::Corners`.
- The interior lattice is capped by its total point count, not per
  direction: a face whose curvature varies enormously over its domain — a
  torus with a minor radius far smaller than its major one, at a fine
  chord — that would need more than `MAX_INTERIOR_POINTS` points is
  `MeshError::GridTooLarge { face, points }` naming how many it would
  need, never an allocation the machine cannot make. `MeshError::Internal`
  is tessellation's own bookkeeping breaking on already-validated input —
  never a property of the body or the chord, and never the CDT's own
  fault, so never a `MeshError::Face`.
- `MeshError::Interrupted` is the caller's poll or budget (ADR-0030). A
  step is an edge, a face (in the domain pass and in the loop pass), an
  interior lattice point, and a CDT insertion or recovered segment
  (`cdt::triangulate_metered`). The parallel face pass splits a meter per
  face and charges them in face order, so a budget stops at the same step
  with `parallel` on or off. The model is only read, so nothing is undone.

`arris_mesh::tessellate_with(&Model, Body, &MeshRequest, &Control)` is the same
mesh with the **corner block** beside it when `MeshRequest::corners`
asks for one: the render buffer, and `tessellate(m, body, chord, control)` is
`tessellate_with` of `MeshRequest::new(chord)` (ADR-0012). Asked for,
`TriMesh::corners()` is `Some(&Corners)` and every position, triangle
and range above is unchanged. A *face-local vertex* is one input point
of one face's triangulation — a loop sample or an interior lattice point
— shared within its face by every triangle that uses it and shared
across faces by none:

- `Corners::positions` gives the shared `TriMesh` position each one
  stands on, `normals` the **outward** unit normal there — the surface's
  own, flipped where the face is used `Reversed`, never averaged with a
  neighbouring face's — and `uvs` the surface's *own* parameters, never
  normalised. `Corners::triangles` is parallel to `TriMesh::triangles`
  and `Corners::faces` to `TriMesh::faces`, one `CornerFace` per face
  with its vertex run and the tightest `uv_box` over it, so a consumer
  that wants `[0, 1]` normalises by that box itself.
- A sharp edge keeps both faces' normals, a seam's two copies differ by
  exactly one period in the parameter they straddle, and a pole or apex
  carries one face-local vertex per triangle of its fan. Where the
  parametrisation is singular the normal is the limit approached along
  the parameter the surface still moves in, from inside the face's own
  (u, v) box — `±Z` at a sphere's pole for every corner, and at a cone's
  apex one normal per corner's own `u` (ADR-0012). The limit is the
  first-order term of `∂P/∂u × ∂P/∂v` out of `Surface::eval`, so no
  surface kind is matched on and nothing is divided by zero.
- The corpus runner asks for the block on every fixture and holds every
  face-local vertex to both invariants — `surface.point(u, v)` is the
  position it stands on within the face's tolerance, and its normal is
  the outward one there — so the whole corpus covers it, at no
  measurable cost to the run.

No adaptive refinement: interior points, where a face needs them, lie on
a uniform (u, v) grid sized by the chord bound. No `f32` output
(ADR-0011: positions are `f64`, the corner block is `f64`, and the cast
is the consumer's, one line over `TriMesh::positions` at its own
boundary), no smooth shading — a corner's normal is its own face's, and
averaging across a tangent edge is a consumer's pass over this block —
and no mesh-based mass properties (`ops::measure` integrates the
B-Rep).

## Threading and wasm

- Every public type is `Send + Sync`. There is no global mutable state, no
  thread-local cache, no interior mutability in the representation crates.
- Operations take `&mut Model`: one operation at a time per model. That is
  the contract, not a limitation to be worked around with locks. Parallelism
  *inside* an operation is `rayon` behind the `parallel` feature and must
  give identical output with the feature off — the tests run both ways.
  Today that is `arris-mesh`'s faces after the sequential edge pass, and
  `arris-ops`'s two read-only passes over a boolean: the intersection of
  every candidate face pair, and the splitting of every face in its own
  (u, v). Each collects its results in the sequential order before
  anything mutable sees them, which is what makes a result byte-identical
  either way — its error included: every item is evaluated and the first
  error *in order* is returned, since a `Result` collected straight from
  `rayon` is whichever error a thread met first. Parallelism *across* operations is clone-evaluate-import,
  above.
- A running operation is stopped through the `&Control` it was handed and
  nothing else: no clock, no thread, no callback beyond the poll, so wasm
  has it. The poll is `&(dyn Fn() -> bool + Sync)` — `Sync` because the
  parallel passes poll from several threads — and reads whatever the
  consumer has (an atomic, a `SharedArrayBuffer`, its own clock for a
  timeout). The budget is deterministic: a parallel pass hands each item a
  meter split off the budget left when the pass starts, and afterwards
  sums the items' steps in sequential order and stops at the first item
  whose running sum crosses the cap. The sequential build runs the same
  rule, so both stop on the same item at the same count and only a
  budget's result is exact; where a poll lands under `parallel` is the
  schedule's, and only its rollback is promised.
- `wasm32-unknown-unknown` builds every crate with default features; CI
  checks it. `arris-py` is empty there (pyo3 does not build for it) and
  its dependency is gated, so the job is unchanged. No kernel crate touches the filesystem, the clock, threads or
  randomness; `arris-debug` is the only crate that writes files, and the
  oracle is not a crate at all.
- `f64` everywhere inside, the tessellation boundary included (ADR-0011):
  `TriMesh` positions are `f64` and there is no `f32` accessor, because
  the mesh is measured against the oracle as well as drawn, and a
  renderer's cast belongs where it knows its buffer layout and its local
  origin. The corner block beside it is `f64` for the same reason
  (ADR-0012): the corpus checks it on every fixture.

## Formats and tools

- **STEP AP214** (`arris_io::step::write(&model, &[bodies]) -> Result<
  String, StepError>`): the writer came first, in C1, because the
  oracle reads Arris's output through it. One product whose shape
  representation lists a solid entity per lump of each solid body
  (`arris_check::lumps`, ADR-0006): a `MANIFOLD_SOLID_BREP` over its
  `CLOSED_SHELL`, or for a lump with voids a `BREP_WITH_VOIDS` adding an
  `ORIENTED_CLOSED_SHELL` of orientation false per void, whose
  `CLOSED_SHELL` holds the void's faces turned — the reversed shell the
  reference tree's writer emits and its reader turns back into a hole;
  per face an `ADVANCED_FACE` whose `same_sense` is the shell's use
  of it, a `FACE_OUTER_BOUND` for the loop of positive winding and
  `FACE_BOUND`s for the rest, each with the same flag as `same_sense`
  (the stored loop is counter-clockwise about the surface normal, STEP's
  bound about the face's effective one); per edge an `EDGE_CURVE` over a
  `SURFACE_CURVE` — a `SEAM_CURVE` when both uses are in one loop — holding
  the 3D curve and one `PCURVE` per use, so a reader takes the model's own
  trimming; the analytic surfaces and curves on `AXIS2_PLACEMENT_3D`
  (origin, `Z`, `X`, as `gp_Ax3` reads them) and the `B_SPLINE_*` entities,
  rational ones as complex entities, so the writer is exhaustive over the
  geometry enums. An edge on a periodic NURBS — a traced section loop —
  is written on its own piece of the curve, clamped over the edge's range
  (`NurbsCurve::segment`): an `EDGE_CURVE` carries no range and a reader
  finds it from the vertices, which on a closed curve put the block that
  wraps past the knots' end on the other side of them. Millimetres and radians, since the reader scales to
  millimetres by default and Arris carries no unit; the uncertainty is
  `default_tolerance`; the time stamp is empty and every real is the
  shortest round-trip decimal, so two writes are byte-identical. What
  STEP cannot hold: a degenerate edge's coedge is left out of its loop (as
  Open CASCADE's writer does; a loop of nothing else is
  `StepError::Unsupported`), and a left-handed pcurve conic is written on
  the direct placement STEP has, losing the traversal sense (the reader
  reprojects, and ignores plane pcurves anyway). Sheet, wire and general
  bodies are `Unsupported` until an operation produces them, and a solid
  whose shells do not nest into lumps is `StepError::Lumps`.
  `arris_io::step::write_products(&model, &[bodies], &ProductTree,
  &Control)` writes an assembly (ADR-0033): one `PRODUCT` per distinct
  product, however many occurrences share it (`Some(product)`), each
  placement a `NEXT_ASSEMBLY_USAGE_OCCURRENCE` with its
  `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` and
  `ITEM_DEFINED_TRANSFORMATION`, names as product names and colours as
  `STYLED_ITEM`s (a face's an `OVER_RIDING_STYLED_ITEM`). The bodies are
  their products' own geometry and the writer never moves them; a tree it
  cannot write is `StepError::Tree`, and it ticks per occurrence and per
  body (`StepError::Interrupted`). `write` is the one-product case, byte
  for byte.
- **Part 21** (`arris_io::step::part21::parse(&str) -> Result<Exchange,
  Part21Error>`): the exchange structure below any
  schema, the first layer of the reader (ADR-0025 §3). It keeps the
  header's entities (`FILE_DESCRIPTION`, `FILE_NAME`, `FILE_SCHEMA`
  required) and every instance of every `DATA` section in a `BTreeMap` by
  id, simple or complex (the partial entities in the order written), with
  its parameters as written — integers, reals in every spelling the grammar
  allows, strings with their encodings decoded, enumerations, binaries,
  typed parameters, references, lists, `$` and `*` — resolving nothing.
  The first place the text leaves the grammar is a `Part21Error` naming
  its line, its column in characters and the instance it was in: a
  duplicate id, a malformed token, an unterminated string or comment, a
  missing header entity, and the edition-3 `ANCHOR`, `REFERENCE` and
  `SIGNATURE` sections and value instances, which are not read. It never
  panics, runs in time linear in the text, bounds nesting at 64, and the
  corpus runner parses every file the writer makes before the oracle
  reads it, each instance kept once.
- **The STEP reader** (`arris_io::step::read(&mut Model, &str,
  &ReadOptions, &Control) -> Result<Read, ReadError>`, ADR-0025): the AP203/214/242
  B-Rep subset onto the variants Arris has, never a new one. Layered under
  `step/reader/`: `assembly` flattens the product structure
  (`SHAPE_DEFINITION_REPRESENTATION`, `NEXT_ASSEMBLY_USAGE_OCCURRENCE`,
  `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`, a relationship with an
  `ITEM_DEFINED_TRANSFORMATION`, `MAPPED_ITEM`) to every path from a root
  to a solid, each path one *instance* with its placements composed and
  numbered in the order of their ids; `products` builds the `ProductTree`
  from the same walk — an `Occurrence` per site, with its product's name,
  its placement in its parent in `ReadOptions::length_unit` (or the
  `Refusal` of it), its `solids` as indices into `Read::solids` and its
  children, so the tree and the bodies agree on what instance *k* is —
  and `colours` resolves a `STYLED_ITEM` through its style chain to the
  plain RGB of an occurrence's solids or of a face
  (`ProductTree::faces`), skipping any other colour model rather than
  refusing for it (ADR-0033); `units` reads each representation
  context's length unit (`SI_UNIT` with its prefix, or a
  `CONVERSION_BASED_UNIT`) and plane-angle unit and converts to
  `ReadOptions::length_unit` — millimetres by default, the unit the writer
  declares — as an exact ratio where one is representable, the placement's
  motion riding the conversion so geometry is read in place; the file's
  `UNCERTAINTY_MEASURE_WITH_UNIT` is kept beside the result
  (`ReadSolid::uncertainty`), never used as a tolerance; `geometry` maps
  each curve and surface exactly (ADR-0025 §1): the analytic ones as
  themselves, a `SURFACE_OF_LINEAR_EXTRUSION` or `SURFACE_OF_REVOLUTION`
  that is a plane or a quadric as that quadric, every B-spline subtype, a
  parabola, a hyperbola, a polyline and every other swept surface as an
  exact `Nurbs` (data-model §NURBS), a trimmed curve or surface as its
  basis, and a normal's sense kept beside the variant as a `reversed`
  flag the topology turns `same_sense` by; `topology` reads one
  `MANIFOLD_SOLID_BREP` or `BREP_WITH_VOIDS` through
  `Builder::assemble`, edge ranges found by projecting the vertices onto
  the curve, an edge used twice in a loop a seam, every pcurve rebuilt
  by `pcurve_on` and never read, each loop's pcurves walked to be
  continuous and placed in the surface's own period, a degenerate edge
  the file left out rebuilt along the singular row a loop's walk jumps
  on (`Surface::singularities`), a `VERTEX_LOOP` joined to its face's
  other bound by the exact iso-line, and each entity's tolerance measured
  from its own gaps and capped at `READ_GAP_FRACTION` of the part's size
  (data-model §Tolerances). The body is checked at `Level::Fast` in
  every build, since a violation there is the file's and not the
  kernel's. Every solid comes back as a `ReadSolid` carrying its
  `FileEntity` and either a `ReadBody` — the body and its provenance, each
  entity `Generated` from its file entity (`Role::File`, data-model
  §Provenance) — or a `Refusal`; a refused solid leaves nothing in the
  model and hides no other, and only a `Part21Error` fails the file
  (§Errors). Deterministic: the same text reads to the same ids. What
  it does not read: layers, PMI, materials, properties, and the
  edition-3 sections; a faceted B-rep or a shell-based surface model
  stands where a solid would and is counted as refused.
- **Cancellation of the readers** (ADR-0030): `step::read` runs in one
  `Model::transaction`, ticking per solid placement and, inside a solid,
  per file edge, face and pcurve fitted; a stop is
  `ReadError::Interrupted` for the whole call — the solids already read
  are dropped, since a refusal is a solid's and an interrupt is the
  caller's. A stop that an attempt-and-fall-back site inside a solid
  swallowed still wins: the solid reads the first stop from its meter
  (`Meter::stopped`) and returns it in place of whatever came after.
  `body::read` and `body::from_json` tick per stage and per vertex, edge
  and face of the tolerance check, and return `BodyError::Interrupted`
  with the model as it was.
- **Native format** (`arris_io::native::{to_json, from_json, to_bytes,
  from_bytes}`): `serde` of the model under a version header, JSON for
  diffs and `postcard` bytes for storage; data-model §Native format.
- **Body bytes** (`arris_io::body::{write, to_json, read, from_json}`,
  ADR-0029): one body and its `Provenance` under an eight-byte magic and
  `BODY_VERSION`, `postcard` for storage and JSON for diffs, both
  deterministic; data-model §Native format. A write imports the body into
  a fresh model under the writer's `Precision`, so the bytes are the same
  whatever holes the writer's model had. A read migrates every earlier
  version, refuses a newer one and a tolerance the reading model's
  precision cannot hold, imports inside a transaction and checks at
  `Level::Full` in every build profile; the record comes back with the
  map from the writer's ids to the caller's, and what it names outside
  the body is foreign. A version's files are frozen under
  `crates/arris-io/tests/body/v<N>/` and read in the suite forever.
- **STL** (`arris_io::stl::{write_ascii, write_binary}`, ADR-0013): one
  or several `TriMesh`es in — one per body, as `step::write` takes
  several bodies of one `Model` — `MeshWriteError` the only failure —
  binary's `u32` triangle count, summed over every mesh. STL has no
  shared vertex index, so several meshes become one file by writing every
  mesh's facets in argument order under a single `solid`/`endsolid`
  (ASCII) or a single header and triangle count (binary); no
  renumbering needed. A facet's normal is the
  triangle's own winding, never the corner block's — never the point of
  asking `RWStl` to compute normals a flat facet does not have a use
  for; ASCII writes every real as the shortest round-trip decimal so two
  writes are byte-identical, binary writes IEEE 754 little-endian `f32`,
  the one narrowing the kernel ships (`.agents/rules/kernel.md`),
  computed in `f64` and cast only at the writer. `tools/oracle/mesh.py`
  reads both forms back through Open CASCADE's `RWStl` — an independent
  reader of the bytes, not a fixture comparison — and
  `arris_debug::oracle::compare_stl` is the Rust seam to it, held to the
  meshes' own summed triangle count and area and to
  `measure::mass_properties`'s volume within the corpus's own
  `mesh_volume_rel`.
- **OBJ** (`arris_io::obj::write`, ADR-0013): the same one-or-several
  `TriMesh`es in. `v` and `f` always, one `g` per `FaceRange`
  partitioning the triangles by face id in iteration order; `vt` and `vn`
  only when a mesh carries a `Corners` block, one of each per face-local
  vertex, `f` then `v/vt/vn` — the same index for `vt` and `vn`, since a
  face-local vertex has exactly one of each and OBJ has no way to name
  one without the other. Unlike STL, OBJ's `v`/`vt`/`vn` indices are
  shared across the whole file, so several meshes are written one after
  another, each mesh's own `f` indices offset by the running total of
  the meshes before it. Every real is the shortest
  round-trip decimal, `f64` throughout, so two writes of the same meshes
  are byte-identical; `write` never fails.
- **Text dump** (`arris-debug::dump_text`): the deterministic, diffable
  rendering of a body that fixtures store and tests compare. Not a format:
  it has no reader.
- **The docs-refs lint** (`crates/arris/tests/docs_refs.rs`): a plan file
  is deleted on retirement (`.agents/rules/docs-lifecycle.md`), so a
  citation of `docs/plans/<slug>` or `plans/<slug>` — the commit-message
  form `(plans/<slug> step N)` included — left behind under `crates/`,
  `tools/`, `.githooks/` or the root `Cargo.toml` after that outlives the
  file it points at. Proven the way `corpus_lint.rs` proves its own
  rules: against a scratch tree with a citation deliberately left
  dangling next to one that still resolves, not just by running clean
  against the real tree.
- **The gate's tiers** (ADR-0032): `.config/nextest.toml` names the `fast`
  profile (the default minus `real_*` and four single tests over ~30 s;
  `tools/profile-test.sh` holds the difference to that set) and `full`
  (the default, named). `tools/gate.sh` turns a path set into the filterset,
  case counts and whether `cargo doc` is due, and `tools/gate-test.sh`
  asserts it; the pre-commit hook runs what it says, `/retire-plan` the
  full profile at 256 cases, CI at 1000.
- **`tools/test-timings.sh`**: the suite's wall clock, per test binary and
  whole, at whatever `ARRIS_PROPTEST_CASES` is set to; `--profile` times one
  nextest profile and lists its slowest tests. Sharding the boolean
  properties and moving the suite onto `cargo nextest` took
  `cargo nextest run --workspace` from 445.68 s to 70.92 s at 256 cases and
  from 1656.26 s to 254.13 s at 1000, measured with this script on a
  16-core machine — 6.3×, within a tenth of the floor that machine's total
  work over its cores allows. Not a benchmark harness: it times binaries,
  not operations.
- **The corpus benchmark** (`crates/arris/benches/corpus.rs`, `harness =
  false`, ADR-0024 §4): for every fixture under `boolean/`, `sweep/` and
  `blend/` whose recipe builds, the build and the tessellation at its
  `mesh_chord`, timed apart by `arris_debug::bench`. For every `real/`
  part that does not wait, the reader's read of its file, and the checker
  at `Fast` alone on each solid read. The reader runs that check in every
  build, so the read less the check is the reader without it. A part
  shrunk to a file another part already has is skipped. The timer takes a
  warm-up and five timed runs, their median and median absolute
  deviation, and writes a JSON `Report`. `--save` writes a report,
  `--compare` prints each case's ratio against a saved one, flagging
  those past `bench::RATIO_FLAG` (3×), and `--table` writes that
  comparison as markdown. Time is never a gate. `tools/bench-compare.sh`
  runs it against `target/bench/baseline.json` (`--bless` sets it) or a
  report named, and the nightly runs the same script against the last
  night's report. On the reference machine it is 316 cases: 282 from 141
  fixtures (build 4.56 s, mesh 3.25 s), and 34 from 17 parts' files (read
  21.63 s).
- **The reader's cost** in a release build (the benchmark's read cases,
  ADR-0025 §Consequences). The committed tier's files read in
  0.3 ms to 0.29 s each, 14 of 16 solid instances read. Two files are
  slower: CTC-05 in 0.95 s, the fits of a solid refused for a gap, and
  FTC-07 in 19.8 s, its fitted pcurves on B-spline faces. The checker at
  `Fast` is 2.7% (FTC-07) to 83% of a read, 36% to 83% on every other
  part it checks: 0.96 s of the 21.63 s, where it was 0.074 s of 20.99 s
  when this was first measured; the rise is not yet attributed.
- **The real-part corpus** (ADR-0026). Its committed tier is the
  `part` fixtures under `real/`, run by `cargo test`
  (`tests/fixtures/README.md` §Part fixtures). Its fetched tier is
  `tools/real-parts.sh`: NIST's archives fetched into
  `target/real-parts/`, every file held to `tools/real-parts.sha256`,
  and each surveyed in its own process under a timeout
  (`arris_debug::survey`: Arris's read held to Open CASCADE's through
  the oracle's cache, then the battery). A failure is a panic, a
  checker-rejected `Ok`, an `Ok` outside the oracle's measures or a
  battery stage that is a kernel fault. It writes `histogram.md`,
  `failures.md` and `both.md`, the histogram over both tiers, and
  `fillet-by-part.md`, the fillet column by part (`arris_debug::census`:
  for each solid whose fillet stage Arris refuses, its first refusal at
  the battery's radius and what every blendable edge alone meets;
  `--example real_parts -- --census-committed` for the committed tier),
  and exits
  1 on a failure no line of `tools/real-parts.waits` excludes by a
  fixture still under `regression/`. `cargo run -p arris-debug
  --example real_parts -- --committed` prints the committed tier's
  histogram from its fixtures, which record the cycle of each refusal
  (`arris_debug::histogram`). `--example battery -- --write|--record`
  derives a part's battery operands and records each stage's class.
- **The fuzz targets** (`fuzz/`, ADR-0024 §5): a crate outside the
  workspace (`exclude = ["fuzz"]`), unpublished, on nightly under
  `cargo fuzz` with `libfuzzer-sys` and `arbitrary`, none of them
  workspace dependencies. There are five targets. `intersect_surfaces`,
  `intersect_curve_surface` and `intersect_curves` each decode analytic
  operands in a pose from bytes, including NURBS curves given by their
  control points and the fitted curves of a section of two decoded
  surfaces. A number is folded into its range, so a mutation still
  decodes, and one in range decodes as itself, so `fuzz/seed.rs` writes
  every `tests/fixtures/geom/` pair as a seed. Each target asserts no
  panic, every hit on both operands within the tolerance (a surface
  pair's curves inside the region asked for), and the same answer twice.
  `step_read` runs `step::read` on any text into two fresh models,
  seeded from Arris's and Open CASCADE's STEP of every solid fixture's
  result, and asserts no panic, the same answer twice and a parse error
  placed inside the text. `body_read` runs `body::read` on any bytes and
  `body::from_json` on any that are UTF-8, seeded from the guard's files
  and read into a model that already holds a body, and asserts no panic,
  the same answer twice, a checker-green body on every read, and the
  model unchanged on every refusal. They run without ASan
  (`-s none`), which finds nothing in `forbid(unsafe_code)` crates and
  costs twentyfold. `fuzz/show.rs`
  decodes a crash.
- **`nightly.yml`** (ADR-0024 §3), on a schedule and on
  `workflow_dispatch`, with `ARRIS_ORACLE_CACHE=off` throughout. Its
  `seed` job draws one seed per run, `sha256` of the UTC date, printed as
  `ARRIS_PROPTEST_SEED=…`; a dispatch may replay a given one. On that
  seed it runs every property at 5000 cases, five times CI's, in six
  jobs split by nextest filterset along the suite's per-test timings. It also runs the differential at 1000 recipes with the
  corpus's ignored tests, the corpus benchmark against the last night's
  report, each fuzz target for 30 minutes from a corpus kept in the
  Actions cache, and `tools/real-parts.sh` with the NIST archives cached
  by the manifest's hash, its histogram and failure list uploaded. A
  failure is a red run and nothing else.
- **The oracle** (`tools/oracle/`, Python 3.12, Open CASCADE through the
  `cadquery-ocp` wheels in a `uv` environment): `expected.py` builds each
  fixture's recipe in OCCT and writes `expected.json`; `compare.py` reads
  an Arris STEP file and compares it against that within the fixture's
  tolerances; `selftest.py` proves the oracle against closed forms and its
  own STEP. A second fixture kind, `geometry` (`tests/fixtures/geom/`),
  has no solid: named analytic surfaces and curves that the oracle
  evaluates, projects onto and intersects, and that
  `crates/arris-geom/tests/oracle.rs` compares Arris against — the
  parametrisation's ground truth. It is run, never linked; no crate
  depends on it. The fixture format is `tests/fixtures/README.md`; its
  role is roadmap §Fixtures.
- **`arris-debug`** is dev-facing: the text dump (`dump_text`), the
  sample bodies built by hand through the raw insert with explicit
  pcurves (`sample::{cuboid, cuboid_nurbs, unit_box, cylinder, sphere,
  torus, patch}` — what the checker's tests start from, since they
  cannot use `arris-ops`; `sphere` is the body with a seam and two
  degenerate pole edges, `torus` the genus-1 one with two seams and no
  pole, and `patch` a rectangular sheet of any surface kind, the
  tessellation's property tests' operand) and through the Euler
  operators (`sample::frame`, the genus-1 twin of `boolean/frame-cut`), the rasteriser (`render_png`) and the samplers
  that feed it a curve or a surface without a body (`polyline_of`,
  `wireframe_of`), the Rerun stream, the fixture loader and corpus lint
  (`fixtures`; a solid fixture the runner compares under `primitive/`,
  `transform/`, `boolean/`, `sweep/`, `provenance/`, `blend/`, `offset/` or `shell/` without its
  committed dump per variant fails the lint, so an ignored fixture there does; a
  failure waiting for its fix sits under `regression/`, and fails the lint
  once it has a dump), the corpus runner (`corpus::run`, the fixture test of
  roadmap §Fixtures, one function per `corpus::Stage`, the six that read
  no file — checker, counts, measure, mesh, probes, provenance — run
  together by `corpus::stages` before the STEP round trip and the dump — a `profile` step built into a `geom::Profile` kept
  beside the bodies for the sweep steps that name it, no body and no
  accounting of its own; a `fillet` or `chamfer` step's edges named by a point each,
  the edge `classify_point` answers `On(Edge)` for when no second edge of
  the body passes within the fixture's `probe` of the point, a
  `CorpusError::EdgePoint` otherwise; a `step` step's solid read by
  `arris_io::step::read` from the file beside the recipe, held to the
  recipe's SHA-256 of it and named by its `#id` — and, where an assembly
  places it more than once, by the placement whose centroid is nearest a
  point — the reader's refusal of it `CorpusError::Refused` (ADR-0026);
  a `part` fixture's file read whole by `part::run`, every solid held to
  the outcome the fixture records, each read one matched to the oracle's
  healed reading of its `#id` by centroid and put through the checker,
  counts where healing changed no topology, mass properties, mesh and
  dump (`tests/fixtures/README.md` §Part fixtures);
  checker, counts — a solid per lump — and genus,
  the oracle's reading
  of the STEP, the mass properties against the oracle's within the
  fixture's tolerances, the mesh closed and within `mesh_volume_rel`,
  every probe classified as the oracle classifies it
  (`classify_point`, exactly: both sides have their own tolerance for
  "on" and a probe is placed so the two agree, so a disagreement is a
  finding and never something a band is widened to cover), provenance
  accounting, the dump; `ARRIS_BLESS=1` writing `dump.txt` (`dump.<variant>.txt` for another
  variant); a result the
  oracle recorded no solid for must fail with `OpError::Degenerate`, and
  one the recipe marks `analytic.expect_error` with that typed refusal,
  the run ending there with the oracle's numbers kept as the record of
  what Open CASCADE builds; one whose recipe states a convention Arris
  does not follow, `analytic.counts_differ`, held to the recipe's own
  counts with the oracle's kept as the record; one whose recipe states
  the oracle's measurements wrong, `analytic.measure_differs`, held to
  its closed forms of volume, area, centroid and inertia in the measure
  stage, the mesh check and `compare.py` alike — ADR-0015)
  over the
  oracle seam (`oracle::compare`: STEP under `target/inspect/`, which the runner
  names `<area>-<slug>-<variant>`, plus a digest of the directory for a
  scratch copy of a fixture, so two runs of a recipe never write one file;
  then `compare.py` through `uv`, a missing environment a loud error;
  `oracle::scratch_fixture`: a test's own recipe written under
  `target/inspect/<name>/` with its `expected.json` from `expected.py`
  — through `oracle::expected_batch`, which answers many such
  directories in one `expected.py` process, a recipe the oracle refuses
  an answer of its own and the rest still built —
  for a body the corpus does not hold — the revolves closing at a cone's
  apex or a sphere's pole, and the STEP tests' own bodies — held to the
  oracle's reading of its STEP all the same; every settled answer —
  a `MATCH`, a scratch `expected.json`, an STL reading — kept in
  `oracle::cache` under `target/oracle-cache/` by a key over every input
  the script reads and the oracle's own sources, so an unchanged call
  starts no Python, and `ARRIS_ORACLE_CACHE=off`, which CI sets, bypasses
  it — ADR-0024), and
  the seeded property-test runner and strategies (`prop`,
  with every analytic surface and curve in a random pose and random
  clamped NURBS curves and surfaces under `prop::geom`; sketches under
  `prop::profile` — `star`, a polygon with arcs and holes in either
  orientation; `rectilinear`, a staircase of segments parallel and
  perpendicular to an axis beside it or, half the time, reaching it —
  sides along the axis and notches cut in from it — and `general`, a
  convex polygon beside an axis or, one time in three, with a side along
  it, the chords beside that side closing at a cone's apex or a sphere's
  pole, whose segments sweep cones both ways, spheres and tori,
  the last two each given as a `Sweep` with the axis, a revolve angle and an extrude
  length; `prop::sweep`, Pappus's
  theorems as the oracle a sweep's volume and area are held to, taken
  in the profile's plane by `region_integral` and a quadrature over its
  boundary, an independent path from `measure`'s flux; and
  `prop::recipe`, whole corpus recipes — two to four boxes, cylinders and
  swept sketches, a box or cylinder edge blended, one face offset or the
  whole hollowed by `shell` first, each placed near
  the others under a shared pose and chained by one to three booleans,
  probed at every operand's centre and just in and out of its faces —
  which `corpus::build` builds without a directory or an oracle answer,
  and which the differential runs through both kernels — `differential::run`,
  ADR-0024 §2: `ARRIS_DIFF_CASES` recipes drawn from the property seed
  under `target/inspect/differential/`, answered by one
  `expected_batch`, judged on every core against `corpus::stages` with
  no dump and no STEP round trip, and sorted into `Agree`, `BothRefuse`,
  `ArrisRefuses` counted per refusal name (`differential::refusal`: a
  `Degenerate`'s or an `Unsupported`'s), `OracleRefuses`, and the four
  that fail the run — `Disagree { stage }` (measurements held to the
  first-order bound of a boundary known to `t_arris + t_occ`, counts net
  of vertices that only split an edge), `CheckerViolation` (the checker
  at `Full`, an input it rejects, or the debug build's guard caught as a
  panic on the test side), `Internal` (a caught kernel fault, named by
  it) and `Panic` — unless a named exclusion
  (`differential::EXCLUSIONS`, each citing the `regression/` fixtures it
  waits on, which the corpus lint holds present) covers the symptom, when
  it is counted `Excluded` under its name; the property tests over the
  same booleans hold the same list (`exclusion_of_panic`,
  `exclusion_of_error`); each failing case shrunk through its
  `ValueTree` for at most `ARRIS_DIFF_SHRINK` candidates, the oracle run
  per candidate through its cache, and printed as a `fixture.json`;
  `crates/arris/tests/differential.rs` runs it and prints the
  histogram). `prop` runs a
  property whole through `check`, or split across `k` shards through
  `prop_shards!`, which writes one `#[test]` per shard over a body given
  once so libtest's pool runs them at once instead of one property holding
  one thread. A shard draws from `shard_seed(base, i) = sha256(base ‖ i)`,
  which does not take `k`: raising a property's shard count shortens every
  existing shard's stream to a prefix of what it was rather than re-rolling
  the corpus, and `cases_per_shard` rounds up, so `k` shards run at least
  the configured cases and never fewer. A failure names its shard and
  prints the base seed and total that reproduce the whole run. It is a
  dev-dependency of the workspace's crates and never of a consumer. For
  the crates below it (`math`, `geom`, `topo`) that dev-dependency is a
  cycle, so their property tests are integration tests under
  `crates/<crate>/tests/`, where the crate is linked once and its types
  unify; a `#[cfg(test)]` module would see two copies), and `testing`
  (dev-only, not built for wasm32 since it takes `prop`'s
  `TestCaseError`: the helpers that used to be copied across test files —
  a relative-tolerance and period-aware numeric comparison (`close`,
  `close_param`), the provenance accounting a generated body's record
  owes (`entities_of`, `recorded_parts`, data-model §Provenance), and
  central differences against a curve's or surface's own analytic
  derivatives (`central_differences_curve`, `central_differences_surface`)
  — now imported once by `ops`, `geom`, `mesh` and `io`'s tests instead of held
  per file).

- **The Python wheel** (`crates/arris-py`, PyPI `arris`, ADR-0034): the
  binding is built by `maturin` into one `abi3` wheel (Python 3.10 and up)
  and an sdist, with the version read from the workspace (`0.5.0-dev` is
  `0.5.0.dev0`). It ships hand-written stubs (`_arris.pyi`, `__init__.pyi`,
  `_numpy.pyi`, `py.typed`) and every public class, function and `Model`
  method carries a runnable example in its doc comment. CI's `python` job
  builds it with `maturin develop`, runs the pytest — the oracle's numbers
  for corpus fixtures scripted in Python, every docstring example, and
  `mypy.stubtest` against the stubs, so a stub that drifts fails — at the
  oldest and newest supported Python, and builds the release wheel and sdist
  for `twine check`. The hook only compiles the crate; the Python suite
  never runs there. The wheel carries no rasteriser and no numpy
  dependency: a `Mesh` crosses as little-endian `f64` and `u32` bytes, with
  `to_numpy()` importing numpy only when called. `release.yml` publishes it
  from the same `v*` tag, behind its own `pypi` environment.

## How a consumer's kernel facade maps on

The first consumer programs against a facade trait of its own and can swap
the backend behind it, on its own schedule (ADR-0017). Its surface maps onto
Arris method for method, and nothing in the facade needs Arris types above
it. What a method covers is a separate question, answered case by case: a
blend between faces outside ADR-0007's table is refused where the old
backend may build it, and the STEP reader refuses by name what it does
not map (ADR-0025 §2). The consumer's run of its suite on both
backends is what measures this — and, beside the real-part corpus's
refusal histogram, what picks the cycle after the reader's.

| Facade needs | Arris provides |
|---|---|
| Primitives (box, cylinder) | `ops::primitive_box`, `ops::primitive_cylinder` |
| Extrude / revolve of a sketched profile with holes | `ops::extrude`, `ops::revolve` over `Profile` (lines, arcs and elliptic arcs; a revolve refuses an elliptic segment, ADR-0014) |
| Boolean union / intersect / cut | `ops::fuse`, `ops::common`, `ops::cut` |
| Cut or fuse many tools at once (a pattern of holes or bosses) | `ops::cut_many`, `ops::fuse_many` (ADR-0050): one decomposition over the body and every tool, provenance naming each tool; tools that stay clear of one another until the next step lifts that |
| Transform (geometry only, topology and index order preserved) | `ops::transform` — new ids, provenance `Modified` one-to-one in iteration order |
| Mirror a body in a plane | `ops::mirror` — new ids, provenance `Modified` one-to-one, the image a solid with its material inside (ADR-0031) |
| Fillet / chamfer of named edges, one call for all edges | `ops::fillet`, `ops::chamfer` (ADR-0007) |
| Press-pull: chosen faces moved along their normals, neighbours extended or trimmed | `ops::offset_faces` (ADR-0048) — each moved face `Modified` from itself, the whole body's offset when every face moves |
| Hollow a solid to a wall of constant thickness, inward or outward, faces opened or none (a closed void) | `ops::shell` (ADR-0049) — each wall face `Modified` from itself with its inner copy `Generated` from it, each opening `Modified` into its rim face, a void's second shell `Generated` from the body |
| Tessellation into a render mesh with per-face and per-edge ranges | `arris_mesh::tessellate` → `TriMesh` with `FaceRange`/`EdgeRange` keyed by `FaceId`/`EdgeId`; `arris_mesh::tessellate_with` of a `MeshRequest::with_corners` adds the render buffer beside it — face-local vertices with outward normals and the surface's own (u, v), which a renderer uploads as they stand (ADR-0012) |
| A face's outward-oriented frame | `ops::query::face_frame(&model, face)` for a plane (stable across re-evaluation, since a primitive's frame or a sweep's profile plane is), `ops::query::frame_at(&model, face, uv)` for any face at a `(u, v)` its domain contains |
| Mass properties (volume, area, centroid, inertia) | `ops::measure::mass_properties` → `MassProperties` (exact over the B-Rep, the tensor about the centroid); or the consumer's own integrator over `TriMesh` |
| STEP export of several bodies, or of an assembly | `io::step::write(&model, &[bodies])`; `io::step::write_products(&model, &[bodies], &tree, &control)` for products, placements, names and colours (ADR-0033) |
| STEP import | `io::step::read(&mut model, &text, &ReadOptions { length_unit })` → one `ReadSolid` per solid and placement, and `Read::products` the product tree over them (names, placements, colours): a checker-clean body with provenance naming its file entity (`Role::File`), or a typed `Refusal` whose `RefusalKind` a histogram counts (`RefusalKind::ALL`); only a parse error fails the file (ADR-0025) |
| A render mesh as STL or OBJ of several bodies, beside STEP | `io::stl::write_ascii`/`write_binary(&meshes, name)`, `io::obj::write(&meshes)` — one `TriMesh` per body, `vt`/`vn` written when a mesh carries the corner block (ADR-0012, ADR-0013) |
| Projecting an edge or vertex onto a sketch plane | `ops::query::project_to_plane(&model, &[shapes], &plane)` → a `Projection` per shape: a vertex's `Point2`, an edge's `Curve2` (a line stays a line, a circle becomes a circle or an ellipse, an ellipse stays an ellipse, a NURBS a `Curve2::Nurbs`) with the edge's range carried into that curve's own parameter, so the piece is the edge's and no more (data-model §Pcurves) |
| Persistent topological names (origin-based) | Emitted by the consumer from `Provenance`: an output face is named after the input face it was `Modified` from, `Split(k)` when one input yields several outputs, and after the tool face when `Generated`; edges and vertices derive from their faces exactly as today. No centroid matching. Arris ships no name grammar (ADR-0009): the words are the application's. What the kernel guarantees is the **split order** — an origin's outputs in `generated_from` and `modified_from` are the pieces in an order that holds under every parameter edit keeping which entities bound which piece (a face's by the origins bounding each piece, an edge's along its curve; data-model §Provenance), so `Split(k)` means the same piece after the edit. A feature that builds topology itself roots its chains at its own keys through `ops::build`, and re-roots a primitive's, sweep's or file's record at them with `Provenance::rerooted` (`Role::Consumer`, ADR-0028) |
| Memoising shapes by content, dropping unreferenced ones | Memoisation stays in the consumer (it is about features, not geometry); dropping is `Model::retain`, which frees slots without moving a surviving id (§The model, ADR-0010) |
| Units | Arris is unit-agnostic. The consumer sets `Precision` for its unit (metres: `default_tolerance` at the micrometre scale) when it creates the `Model`. A fixture says which unit it is in the same way, through its recipe's `precision`: the corpus's `probe-*-m` fixtures are the consumer's probe shapes in metres at that tolerance |

What the facade has today that Arris will not have: a tolerance nudge
(there is none; a degenerate boolean is `OpError::Degenerate`), and a
"which surface came from which" matcher (provenance replaces it).

## Open questions

Collected from this document; each closes with an ADR.

None. The three this document carried were C2's facade line, closed
together: ADR-0009 (Arris owns no name grammar, and the split order is a
contract), ADR-0010 (compaction keeps slots sparse) and ADR-0011 (the
tessellation boundary is `f64`). `docs/DATA-MODEL.md` carries none
either: its last, the quadric intersection curves, is ADR-0018.

