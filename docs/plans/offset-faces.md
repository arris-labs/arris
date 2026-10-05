# Plan: offset-faces

- Started: 2026-10-05
- Milestone: C7, prismatic features (docs/ROADMAP.md §C7)
- Idea: docs/ideas/shell-and-offset.md (absorbed; option A in two plans,
  this one first, `shell` on top of it; the idea's four defaults taken —
  the human delegates them)
- Idea (verbatim from the human): "/idea shell-and-offset", then
  "/plan offset-faces"

## Goal

`arris_ops::offset_faces(m, body, faces, distance, control)` moves the
chosen faces of a solid along their outward normals by a signed distance
(positive adds material) and returns a checker-green solid with complete
provenance: each moved face on the offset of its own surface, the same
kind (a plane's plane, a cylinder's coaxial cylinder, a cone's coaxial
cone, a sphere's concentric sphere, a torus's torus of the same major
radius), every edge it touched recomputed — between two moved faces the
intersection of their offsets, between a moved and a fixed face the
moved face's offset met with the fixed face's own surface (the neighbour
extended or trimmed), between two tangent moved faces the edge carried
along their shared normal — and every vertex the meeting of its new
edges. A face tangent to a moved face moves with it (the chain is
dragged). The whole body's offset is every face moving; at a convex edge
of an outward offset the join is sharp (the offsets meet). Where the
offset cannot be held exactly or locally — a NURBS or elliptic-cylinder
face, a radius driven through zero, an edge or face that would vanish or
reverse, a vertex whose faces no longer meet in one point, a dragged
face that no longer meets its fixed neighbour, a result that runs into
itself — it is refused by name, the model untouched. The result passes
`Level::Full` in every build profile, and is an operand of fillet,
chamfer and every boolean. It is bound in the facade and in Python, and
matched against Open CASCADE on a fixture corpus.

## Non-goals

- **Shell** — the next plan, `shell`, on top of this one: the inner faces
  are this plan's offset of every face but the openings, each opening's
  rim joined to the inner rim by a face on the opening's own surface, a
  closed void when there are none.
- The round (arc) join at an outward convex edge; added when a fixture
  asks for it (a backlog line at retirement).
- An offset that changes topology (a face that vanishes and its
  neighbours then meeting, a degree-four vertex splitting into an edge):
  refused; the global arrangement (the idea's option B) is a later idea,
  raised only if the real-part corpus asks.
- Offsets of NURBS faces, of elliptic cylinders (an ellipse's parallel
  curve is no ellipse), variable distance per face: refused, the NURBS
  cycle's (ADR-0047 §1).
- The histogram's `Cycle::Sweep` split (ADR-0047 §4): this plan does not
  touch `arris_debug::histogram`.
- The multi-tool boolean, split by a plane, per-face tessellation: their
  own plans.

## Design deltas

- **`arris-geom`**: `Surface::offset(&self, distance) -> Option<Surface>`
  along the surface's own normal (`None` for `EllipticCylinder`, `Nurbs`
  and a radius driven to or through zero, a torus whose minor radius would
  reach its major); a pure function, so it lives in geom (ADR-0013).
  docs/DATA-MODEL.md §Surfaces gains each kind's offset. Step 2 found a
  cone's apex is not the surface's to judge: the offset cone always
  exists, and a point whose move crosses the axis is on its other nappe, so
  "the offset passes the apex" is `SurfaceCollapses` decided per face in
  step 4, not a `None`.
- **`arris-ops`, public**: `pub fn offset_faces(m: &mut Model, body: Body,
  faces: &[Face], distance: f64, control: &Control<'_>) -> Result<(Body,
  Provenance), OpError>`. New `Reason::Offset(OffsetReason)` group:
  `NoFaces`, `RepeatedFace`, `FaceNotInBody`, `NoExactOffset` (face),
  `SurfaceCollapses` (face), `Vanishes` (the edge or face), `VertexSplits`
  (vertex), `Gap` (the edge between a dragged face and a fixed one),
  `SelfIntersects` (the faces the `Full` check names); a non-finite or
  zero distance is `InputReason::NonFinite` / `NotPositive` on `|distance|`.
  Each a `CHANGELOG.md` `### Breaking` bullet (`Reason` grows a group) and
  a Python exception class in the commit that adds it (kernel.md §API).
- **`arris-ops`, crate-private**: `offset/` by phase — the move set and
  its tangent closure, the new surfaces, edges, vertices, then
  `rebuild::rewrite` over `body_view::BodyView`, as `blend/` does.
- **Check level**: the operation runs `Level::Full` on its own result in
  every profile, as `build` does, and turns a global violation into
  `SelfIntersects`; docs/ARCHITECTURE.md §Operations says so beside
  `build`.
- **Provenance** (docs/DATA-MODEL.md §Provenance): a moved face is
  `Modified` from itself (new surface, same id), a fixed face whose loop
  changed is `Modified`, a recomputed edge or vertex `Modified` from its
  old one; nothing is generated or deleted while topology is kept.
- **Recipe grammar** (both interpreters, `arris_debug::fixtures` and
  `tools/oracle/oracle/recipe.py`): `offset of <name>, faces [[x,y,z],
  ...], distance` — faces named by a point on them, as a fillet's edges
  are; `prop::recipe` and the differential draw it.
- **Facade and binding**: re-exported from `arris`; `Model.offset_faces`
  in `arris-py`, its stub and docstring example.
- **ADR-0048** — offset faces: sharp join, the tangent chain dragged, the
  refusals that keep topology fixed, `Full` in every profile, the sign
  convention, and the oracle's driving (step 1).

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The oracle and the grammar. `offset` in
  `recipe.py` driving Open CASCADE's offset with per-face values
  (`BRepOffset_MakeOffset`, distance 0 on the fixed faces,
  `SetOffsetOnFace` on the moved ones, the intersection join) and in
  `arris_debug::fixtures`, the Rust runner refusing it until step 3. Six
  fixtures under `regression/` with `expected.json` and closed forms:
  a box face pushed and pulled, the whole box out and in, a cylinder's top
  pushed, a through-hole's wall offset. Where Open CASCADE's per-face
  offset is shown wrong or refuses, the fixture is held to its closed
  forms (`analytic.measure_differs`, ADR-0015) and the fallback is
  recorded. ADR-0048 written here with what the oracle does.
- [x] Step 2 **[1]** — `Surface::offset` in geom for every analytic kind,
  `None` for the rest and for every collapse; geometry tests per kind
  (a point of the offset at exactly `|d|` from the original along its
  normal, at sampled (u, v)), and DATA-MODEL §Surfaces.
- [ ] Step 3 **[3]** — The planar core: `offset_faces` for bodies of
  planes, any subset moving. The move set, each edge recomputed (two
  moved: their offsets' line; moved and fixed: the line on the fixed
  plane; two fixed: re-ranged), each vertex the meeting of its new edges,
  `rewrite`, provenance, `Full` in every profile; `Reason::Offset` with
  the input reasons, `Vanishes` (an edge whose ends cross, a face whose
  loop turns inside out) and `VertexSplits` (a vertex of four or more
  faces not meeting within tolerance), with the binding's classes.
  Fixtures: the four box ones of step 1 move to `offset/` and are
  blessed; new: an L-bracket's inner face pushed, a wedge's face pushed
  until a face vanishes (refused), a pyramid apex (refused), a pocket's
  floor pushed and pulled.
- [ ] Step 4 **[2]** — Quadric faces: cylinders, cones, spheres and tori
  moving or fixed, edges from the existing intersectors with the branch
  nearest the old edge, seams carried, `NoExactOffset` and
  `SurfaceCollapses`. Fixtures: step 1's cylinder and hole ones moved and
  blessed; a cone frustum's side, a sphere-capped boss, a revolved torus
  ring, the whole cylinder out and in; a hole shrunk past its radius and
  an elliptic-extrusion face (refused).
- [ ] Step 5 **[3]** — The dragged chain: the move set closed over
  tangent edges (`BodyView::tangent_at`), an edge between two tangent
  moved faces carried along their normal (a line shifted, a parallel
  circle re-radiused and moved along its axis; any other curve
  `NoExactOffset`), `Gap` where a dragged face no longer meets its fixed
  neighbour. Fixtures: a box with every edge filleted, offset whole out
  and in (closed form, the corner spheres included), and in past its
  radius (refused); a filleted box's face pushed and pulled with its
  fillets dragged; a filleted pocket wall pushed (the direction that
  meets the floor) and pulled (`Gap`, unless step 1's oracle shows a
  sharp answer for it — see Open questions).
- [ ] Step 6 **[2]** — Global refusal: a push that runs into a distant
  face (a thin wall pushed through a parallel one, a boss pushed into the
  body's other side) refused as `SelfIntersects` from the `Full` report,
  the model untouched; fixtures for both.
- [ ] Step 7 **[1]** — The facade and the binding: `arris` re-export,
  `Model.offset_faces` with cancel and budget, its stub and docstring
  example, a pytest per refusal group, the rustdoc example on
  `offset_faces`.
- [ ] Step 8 **[2]** — Properties, sharded and seeded: the whole-body
  offset of a box, a cylinder and a rounded box against their closed
  forms in random poses; offset by `d` then `−d` returns the body's
  volume, area and counts; a perpendicular-walled face pushed by `d`
  changes the volume by its area times `d`; offset commutes with
  `transform`; the result (blended bodies among the operands) is an
  operand of fillet, chamfer and a cut, checker green; `prop::recipe` and
  the differential draw `offset`.

## Acceptance

`ARRIS_GATE=full` green; the corpus's `offset/` fixtures (at least the
seventeen named above that build) checker-green at `Full`, matched to Open
CASCADE's volume, area, centroid, inertia, counts and probes, or to their
closed forms where ADR-0015's evidence is recorded; every refusal fixture
fails with its named `OffsetReason`; step 8's properties at 256 cases;
the differential at 1000 recipes with `offset` drawn and no new
unexplained disagreement; the `python` job's pytest, docstring examples
and `mypy.stubtest` green.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Operations — `offset_faces`: its phases over
  `body_view` and `rewrite`, the sharp join, the dragged chain, the
  `Full` check in every profile beside `build`'s; §Errors — the
  `Offset` group.
- `docs/DATA-MODEL.md` §Surfaces — each kind's offset (step 2 writes it;
  checked here); §Provenance — what an offset records.
- `tests/fixtures/README.md` — the `offset/` area in the lint's list and
  the `offset` op in the grammar.
- `tools/oracle/README.md` — how the offset is driven and its fallback.
- `docs/ROADMAP.md` §C7 — status line: offset faces landed.
- `docs/BACKLOG.md` — the round join; whatever refusal the corpus shows
  is common.
- `CHANGELOG.md` `Unreleased` — what a consumer can now do (press-pull,
  whole-body offset) and the refusals they will hit; `### Breaking` holds
  the bullets the steps wrote.
- `AGENTS.md` current state — offset faces done, shell next.

## Open questions

- ⚠ OPEN: Does Open CASCADE's per-face offset (zero on the fixed faces)
  give the press-pull answer, or only the whole-shape offset? Agent
  decides at step 1 from its runs; the fallback is closed forms, recorded
  in ADR-0048.
- ⚠ OPEN: A filleted pocket wall pulled away from the floor: refuse as
  `Gap` (the default here) or rebuild the fillet at its radius between
  the moved wall and the floor? Agent decides by step 5 from what Open
  CASCADE and the consumer's press-pull do; a rebuild that needs the
  blend is a backlog line, not this plan.
