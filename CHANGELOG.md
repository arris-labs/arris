# Changelog

What changed for a consumer of the `arris` crates, newest first. Pre-1.0,
a minor version breaks the API and a patch does not; every break is listed
under **Breaking** with its one-line fix. A change lands under
`Unreleased` in the commit that makes it, and a release turns that section
into its version (ADR-0027).

## Unreleased

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

### Breaking

- `arris_topo::provenance::Role` gains `Consumer(ConsumerKey)`, a
  consumer's own key: a `match` over `Role` needs a `Consumer(_)` arm.
- `arris_ops::OpError` gains `Unkeyed { slot }` and `Rejected(Rejection)`,
  the refusals of the new `arris_ops::build`: a `match` over `OpError`
  needs both arms.

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
