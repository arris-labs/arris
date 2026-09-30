# Changelog

What changed for a consumer of the `arris` crates, newest first. Pre-1.0,
a minor version breaks the API and a patch does not; every break is listed
under **Breaking** with its one-line fix. A change lands under
`Unreleased` in the commit that makes it, and a release turns that section
into its version (ADR-0027).

## Unreleased

- `arris_ops::mirror` reflects a body in a plane (`arris_math::Reflection`):
  the image is a new solid with its material inside, checker-clean, and
  every vertex, edge, face, shell and the body is recorded as modified
  from the one it mirrors. It stops under a `Control` like every operation.
- A consumer that builds topology itself — through the builder's Euler
  operators or `Builder::assemble` — finishes it into a solid with
  `arris_ops::build`, giving each vertex, edge, face, shell and the body
  a key of its own (`BuildKeys`). Every entity of the result is recorded
  as generated from `Role::Consumer` of its key, so provenance chains
  through later operations end at the consumer's own names. Keys need
  not be unique.
- `build` checks the body fully in every build profile, release
  included, and refuses: `OpError::Unkeyed` naming a slot with no key,
  and `OpError::Rejected` for topology the builder refuses, a slot kept
  from another body, or a body the checker rejects (faces that cross, a
  body inside out) with the checker's report. The model is left as it
  was.
- `Provenance::rerooted` renames every role of a record — a primitive's,
  a sweep's, a file's — through a function the consumer gives, so those
  chains can end at the consumer's own keys too, with no change to the
  operations that made them.
- One body and its provenance can leave a model as bytes and enter
  another, in another process or under a later release:
  `arris_io::body::{write, read}`, with `to_json`/`from_json` beside them
  for diffs. A body is written through a fresh model under the writer's
  precision, so the bytes do not depend on the holes in the model it came
  from, and both encodings are deterministic byte for byte.
- A body read from bytes keeps its record. Origins that name entities
  outside the body — a boolean's operands — come back as
  `Imported::foreign`, `Imported::translated` puts the whole record in the
  reader's ids in one pass, and `arris_topo::IdMap::inverse` translates
  the reader's own operands back.
- Every earlier version of body bytes reads and is migrated. A newer
  version (`BodyError::Version`), a tolerance the reading model's
  precision cannot hold (`Precision`, never rescaled), bytes that are not
  body bytes (`Magic`), a truncated or malformed stream (`Decode`) and a
  body the checker rejects (`Rejected`) are typed refusals that leave the
  reader's model as it was. The whole-model native format still refuses
  another version.
- `Provenance` and `IdMap` now have a JSON form, their relations and maps
  written as ordered pairs; the `postcard` encoding is unchanged.
- A running operation can be stopped, on native and on wasm. Every
  operation on a model, and every long query beside one, takes a
  `Control`: a poll the consumer answers from its own flag, buffer or
  clock, and optionally a budget of steps. When either says stop, the call
  returns `Interrupted` in its own error type — with the cause and the
  steps taken — and the model is as it was, ids included. A budget is
  deterministic: the same input and budget stop at the same step on every
  platform and with `parallel` on or off, so an evaluation can be capped
  reproducibly. `Control::NONE` runs to the end. A section of a torus
  against a very thin elliptic cylinder, which takes minutes, now stops
  within milliseconds of the poll turning true. Reading a STEP file stops
  as a whole: the solids already read are dropped. Writers and
  constant-work queries take no `Control`, and the checker runs to the
  end once the work is done.

### Breaking

- `arris_topo::builder::GeometryRemap` gains two provided methods,
  `pcurve` and `face` (with the new `FaceRemap`), which `Assembly::of_body`
  calls: an implementation of your own compiles as it is, its defaults
  being the identity.
- `arris_topo::provenance::Role` gains `Consumer(ConsumerKey)`, a
  consumer's own key: a `match` over `Role` needs a `Consumer(_)` arm.
- `arris_ops::OpError` gains `Unkeyed { slot }` and `Rejected(Rejection)`,
  the refusals of the new `arris_ops::build`: a `match` over `OpError`
  needs both arms.
- `arris_geom`'s `intersect_surfaces`, `intersect_curve_surface`,
  `intersect_curves`, `curves_coincide`, `trace_quadrics`, `trace_torus`,
  `pcurve_on`, `pcurve_ending_on`, `fit_curve`, `fit_curve2` and
  `fit_curve_periodic` take a trailing `&mut arris_math::Meter`, the
  counter a caller can stop them through: pass `&mut Meter::default()` to
  run to the end. `GeomError` and `FitError` gain `Interrupted(Interrupted)`:
  a `match` over either needs an arm.
- `arris_ops`'s `cut`, `fuse`, `common`, `interferences`, `extrude`,
  `revolve`, `fillet`, `chamfer`, `transform`, `build`, `primitive_box`,
  `primitive_cylinder` and `measure::mass_properties` take a trailing
  `&arris_ops::Control` (also `arris_math::Control`), the poll and budget
  of steps a caller stops them through: pass `&Control::NONE` to run to
  the end. `OpError` gains `Interrupted(Interrupted)`: a `match` over it
  needs an arm.
- `arris_mesh`'s `tessellate` and `tessellate_with` take a trailing
  `&Control` (`&Control::NONE` runs to the end): a poll or a budget of
  steps stops the mesh, and `MeshError` gains `Interrupted(Interrupted)`.
  `cdt::triangulate_metered` is `cdt::triangulate` under a `Meter`, and
  `CdtError` gains `Interrupted(Interrupted)`: a `match` over any of the
  three needs an arm.
- `arris_io`'s `step::read`, `body::read` and `body::from_json` take a
  trailing `&Control` (`&Control::NONE` runs to the end). A stop is
  `ReadError::Interrupted` or `BodyError::Interrupted`, and the model is
  as it was: `step::read` is one transaction, so an interrupt drops the
  solids already read rather than returning a partial `Read`. Both error
  enums gain the variant: a `match` over either needs an arm.
- `arris` re-exports `Control`, `Stop` and `Interrupted` at its root; its
  documentation shows an operation stopped by a poll over an `AtomicBool`
  and by a budget of steps.

## 0.3.0 — 2026-09-26

- STEP files from other systems are read: `arris_io::step::read` returns,
  for every solid in the file, a body that passes the checker, or a typed
  refusal naming why it was not read. The refusals a real part is most
  likely to hit are fillets between faces outside the blend table, surface
  models (shells that bound no solid), gaps too wide to close without
  sewing, and booleans on B-spline faces.
- A body read from a file carries its provenance: its entities name the
  file entities they stand for, as `Role::File`.
- A point projects onto a B-spline surface, and a curve on one gets its
  pcurve. A closed B-spline surface wraps a parameter past its closed
  direction instead of extrapolating, and reports that closure as its
  period.
- Mass properties are integrated about the body rather than the origin,
  which makes a body far from the origin measure more accurately. The last
  bits of rounding move.

### Breaking

- `arris_topo::provenance::Role` gains `File(FileEntity)`: a `match` over
  `Role` needs a `File(_)` arm.
- `arris_geom::AmbiguousLocus` gains `MedialAxis`: a `match` over it needs
  a `MedialAxis` arm.
- `arris_geom::integrate::inner_step(s)` becomes `surface_grid(s)`, and
  `region_integral(pieces, step, f)` becomes `region_integral(pieces,
  &grid, f)`.

## 0.2.0 — 2026-09-24

- A boolean takes a cone, sphere, torus or elliptic-cylinder face as an
  operand, in any pose. Every pair of analytic surfaces meets: conic
  sections exactly, the rest traced and fitted to a B-spline within the
  faces' tolerance.
- A cut through a cone's apex or a sphere's pole works.
- Faces that touch or overlap within their tolerance are decided the same
  way every time, and the one case that can't be (a pinch) is refused by
  name.
- A boolean with a B-spline face in either operand is still refused, as
  `OpError::Unsupported` naming the face.

### Breaking

- `arris_geom::SurfaceIntersection` loses `Transversal`, `Tangent` and
  `Points` for `Meets { curves, points }` (with `MeetCurve`, `MeetPoint`
  and `MeetKind`): a `match` reads the curves and points of the one
  variant.
- `arris_geom::intersect_surfaces` takes a `within: &Aabb` region: pass the
  box the answer is wanted in.
- New variants in public enums, each needing an arm in an exhaustive
  `match`: `SectionFault`, `BranchEnd`, `GeomError::ThroughSingularity`,
  `VertexSource::Singular`, `Reason::BesideSingularity`,
  `BuildError::Geometry`.
- `MAX_FIT_SPANS` is 4096 (was 1024).

## 0.1.1 — 2026-09-18

- Two boolean fixes: operands that touch away from any vertex, and a piece
  lying within the other operand's tolerance, are now decided correctly.

## 0.1.0 — 2026-09-17

The first release.

- Box and cylinder primitives, rigid transforms, and extrude and revolve of
  a profile of lines, arcs and elliptic arcs.
- Cut, fuse and common over planar and cylindrical faces, with multi-shell
  results.
- Constant-radius fillet and chamfer on plane–plane and plane–cylinder
  edges.
- Cone, sphere and torus faces, checked and measured.
- Tessellation to a chord tolerance, mass properties, plane projection and
  face frames.
- STEP AP214, STL, OBJ and the native format.
- Every operation returns provenance, and every refusal is a typed error.
