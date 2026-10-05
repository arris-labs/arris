# Changelog

What changed for a consumer of the `arris` crates, newest first. Pre-1.0,
a minor version breaks the API and a patch does not; every break is listed
under **Breaking** with its one-line fix. A change lands under
`Unreleased` in the commit that makes it, and a release turns that section
into its version (ADR-0027).

## Unreleased

- `arris_math` gains the periodic-parameter toolkit beside `wrap_angle`:
  `wrap_into` (into `[lo, lo + period)`), `wrap_signed`, `wrap_offset`,
  `shift_nearest`, `shift_nearest_uv` and `shift_into_range`, each taking
  the period from the caller, so a period that is not a turn is wrapped
  correctly.
- `arris_geom::Surface::offset(distance)` returns the surface moved along
  its own normal as a surface of the same kind (a plane's parallel plane,
  a coaxial cylinder or cone, a concentric sphere, a torus of the same
  major radius), and `None` for an elliptic cylinder, a NURBS surface, a
  non-finite distance and a radius driven to or through zero.
- `arris_ops::offset_faces(model, body, faces, distance, control)` moves
  chosen faces of a solid along their outward normals by a signed distance
  (positive adds material), the press-pull of a CAD: a pushed top face makes
  the body taller, a pushed hole wall narrows the hole, and every face
  moving is the body's offset. A moved plane, cylinder, cone, sphere or
  torus stays its own kind, its neighbours are extended or trimmed to meet
  it at a sharp edge, and a face tangent to a moved one (a fillet and the
  face beyond it) moves with it. The result is checked at the full level in
  every build, provenance names every moved face, and it is an operand of
  fillet, chamfer and every boolean. `Model.offset_faces` is the same call
  in Python. It refuses, naming the face or edge and leaving the model
  untouched: no faces, a face twice or of another body, a NURBS or
  elliptic-cylinder face, a radius driven through zero, an edge or face that
  would vanish or turn inside out, a vertex that would split, a moved face
  that no longer meets a fixed neighbour, and a result that runs into
  itself. The round join at an outward convex edge is not offered.

### Breaking

- A lower crate no longer re-exports the crates beneath it:
  `arris_topo::{arris_geom, arris_math}`, `arris_check::arris_topo`,
  `arris_io::{arris_check, arris_mesh}` and `arris_ops::arris_check` are
  gone. Depend on each crate directly, or use the facade's
  `arris::{math, geom, topo, …}`.
- `arris_ops::Reason` is grouped by the operation that raises it, and each
  group is its own enum, so a match names the operation first:
  `Reason::Input(InputReason)`, `Reason::Sweep(SweepReason)`,
  `Reason::Boolean(BooleanReason)`, `Reason::Blend(BlendReason)` and
  `Reason::Query(QueryReason)`. Messages are unchanged, and the new
  `Reason::name()` returns the leaf's name as it was spelt before (so a
  refusal count keyed on `BlendTooLarge` still reads `BlendTooLarge`).
  The old and the new:

  | Was | Is |
  |---|---|
  | `NonFinite`, `NotPositive`, `ZeroThickness`, `NonManifold`, `NotSolid` | `Reason::Input(InputReason::…)`, the same names |
  | `ProfileCrossesAxis`, `AxisNotInProfilePlane`, `AngleAboveTurn`, `SpindleTorus`, `EllipticRevolve`, `DirectionNotNormal` | `Reason::Sweep(SweepReason::…)`, the same names |
  | `Empty`, `TangentContact`, `BesideSingularity` | `Reason::Boolean(BooleanReason::…)`, the same names |
  | `NoEdges`, `RepeatedEdge`, `EdgeNotInBody`, `TangentChain`, `VertexBlend` | `Reason::Blend(BlendReason::…)`, the same names |
  | `BlendTooLarge` | `Reason::Blend(BlendReason::TooLarge)` |
  | `NotProjectable`, `DegenerateEdge`, `ProjectionCollapses`, `NotPlanar`, `OutOfDomain`, `Singular` | `Reason::Query(QueryReason::…)`, the same names |
- `arris_ops::Reason` gains the group `Reason::Offset(OffsetReason)` —
  `NoFaces`, `RepeatedFace`, `FaceNotInBody`, `Vanishes`, `VertexSplits`,
  `NoExactOffset`, `SurfaceCollapses`, `Gap`, `SelfIntersects` —
  the refusals of `offset_faces`. An exhaustive `match` on `Reason` adds
  its arm.

## 0.5.0 — 2026-10-05

- `NurbsSurface::project` finds the nearest point on a free-form surface
  that is thin, collapsed to a pole or ruled: from a patch beside a corner
  the search crossed into a worse basin and returned a local minimum that
  was not the nearest, and a pole found a rounding's width off its row was
  reported with its parameters at the end of the knots. The search now
  descends inside its patch first, and a point on a pole's axis is
  reported on the pole.

- A boolean whose result holds a sliver of a lump far from the origin is no
  longer refused as "the shells do not nest: void is inside no shell": a
  pipe cut from an elliptic prism it grazes, posed 86 from the origin, left
  a lens of 3e-13 whose enclosed volume, integrated about the origin, lost
  its sign to the gaps between fitted edges. The checker now integrates a
  shell's volume about the centre of its own vertices, as `mass_properties`
  already did, and the lens is a lump of its own.

- The STEP reader reads an edge that ends at the seam of a closed B-spline
  curve: a rational degree-14 circle whose first and last poles are one
  point, with the edge running from a vertex on it to that point, was
  refused as "the edge's end is not past its start" because the end vertex
  projects onto the curve's start; the edge now ends at the domain's end.
  Open CASCADE's STEP of a revolve cut by a cylinder, of a prism cut from a
  common and of a hexagon revolved about its edge read back as the same
  solid.

- The intersection of two cones of one radius and half-angle on parallel
  axes no longer returns two isolated touch points that lie on neither the
  other cone nor the section: a ruling parallel to a generator of the other
  cone has its double root at infinity, and the quotient of two roundings
  that stood for it landed inside the region. The two branches are returned
  alone.

- A closed NURBS curve whose first knot span is vanishingly narrow no
  longer reports a plane crossing that is not on the plane: a clamped cubic
  whose second knot was 3.6e-304 jumped from its first control point to its
  second inside that span, and a crossing in it came back at a parameter
  of 9e-16, 0.3 off the surface. `intersect_curve_surface` now solves each
  side of the join of a closed curve alone and finds it at 1e-304.

- A boolean no longer misses where a closed section curve's edge crosses
  the other operand when the edge was cut at the curve's seam: the edge's
  bounding box covered only the part of its range inside the curve's
  domain, so a face it crossed was skipped and the result's hole loop
  crossed its outer loop (the checker's loop-intersection row). A fused
  hexagon-with-a-hole revolve, cut and fused again, was refused as an
  invalid body; `Curve::bounds` of a periodic NURBS now wraps a range that
  runs past its domain.

- A curve that passes a cone's apex or a sphere's pole closer than a 256th
  of its length but farther than the tolerance now has a pcurve on the
  surface: the section of a cone and a cylinder that passed the apex at
  0.0136 failed in the fit ("the normal equations are singular") because
  the angle's swing there was narrower than the sampling that unwraps it.

- `fillet` and `chamfer` build two blends meeting at a corner whose third
  edge stays sharp when the two edges' dihedrals differ (two fillets of a
  slanted prism's vertical and cap edges) or, for chamfers, make unequal
  angles with the third edge. The blends meet along their usual curve, the
  wider one is trimmed by the other's far face, and the third edge ends at
  that trim, as Open CASCADE builds it. A tangent edge between a rounding
  and its ball (a cylinder and a sphere of one radius) is now refused as
  `TangentChain` rather than as an unsupported pair. Refusals you will
  meet: a miter of one convex and one concave blend, and one with a
  plane–cylinder ruling blend, are still `VertexBlend`; a trim that runs
  off its face is `BlendTooLarge`.

- `fillet` and `chamfer` run to a corner where the face across is split by
  another sharp edge, or is met twice: a facet's foot on a chamfered boss,
  a rise under a roof of several planes, a box standing on another's top
  edge. The end is one arc on each face across, with a vertex where it
  crosses the edge between them, or, where the face is met twice, the
  face's one arc with its loop split in two, as Open CASCADE builds it.
  Refusals you will meet: a blend wide enough to run past the edge between
  the faces is `BlendTooLarge`; a fan with an edge of the other
  convexity, and a fillet along the crease where two equal rounds cross,
  are still refused.

- `fillet` and `chamfer` blend an edge that ends at a cusp with both walls on
  one side of the face they share: a crescent's tip, a crescent-shaped pocket,
  the sliver between a line and an arc tangent to it. The stripe runs to the
  cusp and is cut by the next wall, with exact surfaces, as Open CASCADE cuts
  it. Refusals you will meet: an overhang tip, a cusp whose two edges are of
  opposite senses and whose walls lie on either side of the face (Open CASCADE
  caps it with fitted surfaces), a cusp whose next edge is blended too, and a
  wide blend whose cut ends a hair from the cusp's node, stay refused
  (`TangentChain`, or `Unsupported` for the last).

- `fillet` and `chamfer` blend a rim split into arcs: where a hole's or a
  boss's circle is written as two, three or more edges (a file that splits
  its closed edges, or a part cut in halves), naming one arc blends the whole
  ring, one blend face per arc, whether the cylinder's seam is at one of the
  splits or not. The same holds for a cone, a sphere or a torus against a
  coaxial plane, cylinder or cone. Refusals you will meet: a vertex of two
  edges that are not one circle between the same two faces stays
  `VertexBlend`.

- `fillet` and `chamfer` run on through a vertex of four edges where both of
  the blended edge's faces turn tangentially (a chamfered stadium's foot, a
  pin's rim split in half by its cylinder and cone, a wall turning round a
  rounded corner): the blend follows the chain and the two tangent edges are
  cut at the junction. Faces a file writes as tangent to a few `1e-10` now read
  as tangent for a blend within its tolerance, so such walls build and a
  tangent cap reports `TangentChain` rather than another refusal.

- `fillet` and `chamfer` blend an edge that runs into a step: where the
  blended edge meets a corner whose other two edges differ in convexity (a
  low block against a taller one, a rib meeting a shoulder, leaning either
  way), the blend lengthens the corner edge of its own convexity past the
  vertex and ends on the face across, which may be a plane, a cylinder or a
  cone, and likewise at an open arc's ends. The blend's surface stays exact.
  Refusals you will meet: a stretch that would leave its face, or a ball
  that finds no place on either face, is `BlendTooLarge`, as is a ring's
  contact at the axis (a horn or spindle torus).

- `fillet` and `chamfer` blend an edge that ends on a face the blend does not
  meet square: an open arc ending on a plane parallel to its axis and off it,
  or on a cylinder across (a rib's foot against a round or conical boss, a
  twin boss's foot), and a straight edge along a ruling ending on a
  cylinder or a cone. The blend's surface stays exact and its end curve is
  fitted within the edge's tolerance. Refusals you will meet: a chamfer's
  cone ending on a plane parallel to its axis, two crossing cylinders and a
  torus against a cylinder off its axis are `Unsupported` naming the pair;
  a blend that runs out of its face onto the next is `BlendTooLarge`, so
  several parts that cleared the end now meet that.

- `fillet` and `chamfer` follow an outline: naming one edge of a chain of
  lines and arcs that meet tangentially (a slot's rim, a stadium, a D-shaped
  notch) blends the whole chain, open or closed, one blend face per edge,
  and a plane against a cylinder blends along an open arc as well as a
  closed circle, so a hole's rim split in two half circles blends. The
  result matches Open CASCADE's on the committed outlines. Refusals you
  will meet: an edge that is itself a tangent dihedral, and a chain turning
  from convex to concave, are `TangentChain`; a chain reaching a pair no
  blend takes (a torus off its axis, a NURBS face) is `Unsupported` naming
  the pair; a blend that runs out of its face is `BlendTooLarge`.

- `fillet` and `chamfer` blend the circles of a turned part: a cone against a
  plane, a cylinder or another cone (a frustum's rim, a conical boss's base,
  a shoulder between a cone and a cylinder), and a sphere or a torus against
  a coaxial plane, cylinder or cone (a dome on a cylinder, a toroidal bead
  on a disc), open arc or closed circle, in a chain with the lines and arcs
  beside them. A convex blend that ends at a concave corner (a rib's root
  on its plate) now builds. Refusals you will meet: a ball whose contact
  would pass a cone's apex, or whose centre circle is no ring torus, is
  `BlendTooLarge`; a plane meeting a cone obliquely or through its apex,
  two cylinders, and a torus against a cylinder off its axis are
  `Unsupported` naming the pair.

- A Python package, `arris`, over the same kernel: `pip install arris` and,
  in a script, build a body from primitives, sketches (lines, arcs and
  elliptic arcs, with holes) and their extrusions and revolutions; cut, fuse
  and intersect, fillet and chamfer, transform and mirror; read volume,
  area, centroid and inertia and the checker's report; walk a body's
  faces, edges and vertices; tessellate to little-endian mesh bytes (with
  `to_numpy()` when numpy is installed); and write and read STEP (every
  solid of a file its own result or a named refusal, with the product
  structure), STL, OBJ, body bytes and the native format. Every operation
  returns the body and its provenance, takes `cancel=` and `budget=` and
  stops on Ctrl-C leaving the model as it was, and every error is a typed
  exception that names the entities it concerns. A handle carries its model:
  one from another model raises `ForeignHandleError`. Not in the package: a
  PNG renderer, a way to write an assembly, and the kernel's per-corner
  normals.

## 0.4.0 — 2026-10-02

- `step::read` returns the file's product structure beside the flattened
  bodies: `Read::products`, a `ProductTree` of occurrences — each a
  product's name, its placement in its parent (or the refusal of it), the
  indices of the solids it holds in `Read::solids`, and its children.
  Open CASCADE's XCAF assemblies read to their names, nesting and
  placements. A colour a file paints a solid or a face with — plain RGB,
  or one of the eight named colours — comes with the occurrence
  (`Occurrence::colour`) or in `ProductTree::faces`; anything else is
  skipped, never a reason to lose a body.
- `step::write_products` writes an assembly: a `ProductTree` of occurrences over
  a slice of bodies, each product once however many times it is placed
  (occurrences sharing a `Some(product)`), each placement with its
  transformation, names as product names and colours as styled items,
  from the bodies in their products' own frames. Open CASCADE's XCAF
  reader reads it to the same names, placements and colours. It stops
  under a `Control`, and refuses a tree it cannot write with
  `StepError::Tree`.
- `arris_ops::mirror` reflects a body in a plane (`arris_math::Reflection`):
  the image is a new solid with its material inside, checker-clean, and
  every vertex, edge, face, shell and the body is recorded as modified
  from the one it mirrors. It stops under a `Control` like every operation.
  Two refusals to expect: a ball united with a copy of itself turned half
  a turn about its axis (so a body fused with its own mirror in a plane
  through its centre) is refused as a kernel bug where Open CASCADE
  returns the ball, and the STEP reader refuses the torus face of Open
  CASCADE's mirrored ring, whose surface frame is left-handed.
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
- The checker no longer rejects a valid face whose hole runs close to the
  rim of a circular boundary: it reported "hole loop lies outside every
  outer loop" for a boolean's correct output, a debug build's guard
  panicked on it, and `build` refused it. Loops are now compared as finely
  as a face's domain is, so a cut, common or fuse of a tilted cylinder or
  disc by a slab or pin, which tripped it, returns its body.

### Breaking

- `arris_io::step::StepError` gains `Tree(TreeError)` and
  `Interrupted(Interrupted)`: a `match` over it needs the two arms.
- `arris_io::step::Read` gains the field `products`: a struct pattern
  needs `products: _` or `..`.
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
