# Plan: shell

- Started: 2026-10-05
- Milestone: C7, prismatic features (docs/ROADMAP.md §C7)
- Idea: none of its own — `docs/ideas/shell-and-offset.md` (option A in two
  plans) was absorbed by `offset-faces`, whose non-goals fence this one.
  The decisions are taken here and in ADR-0049.
- Idea (verbatim from the human): "/plan shell"

## Goal

`arris_ops::shell(m, body, openings, thickness, side, control)` hollows a
solid to a wall of constant thickness and returns a checker-green solid with
complete provenance. The wall is on the inside (`ShellSide::Inward`, the
body's outer faces kept and the cavity grown inside them) or the outside
(`ShellSide::Outward`, the body's own faces become the cavity and a skin
grows outside them). The inner skin is `offset_faces`' offset of every face
but the openings, by `thickness` — each face on the offset of its own
surface, the same kind, the sharp join at every edge, a face tangent to a
moved one moving with it — and each opening is a face on the opening's own
surface bounded by the body's rim and the skin's rim, one loop where two
openings share an edge. With no openings the cavity is a closed void: the
result has two shells, the outer face-for-face the body (or its offset) and
the inner reversed. Where the wall cannot be held — a NURBS or
elliptic-cylinder face, a radius driven through zero, an edge or face that
would vanish or reverse, a vertex whose faces no longer meet in one point,
an opening tangent to a wall, every face an opening, a thickness that runs
the skin into the outer faces — it is refused by name, the model untouched.
The result passes `Level::Full` in every build profile, is an operand of
fillet, chamfer, `offset_faces` and every boolean, is bound in the facade and
in Python, and is matched against Open CASCADE on a fixture corpus.

## Non-goals

- Variable thickness per face, an offset of a NURBS face: refused, the NURBS
  cycle's (ADR-0047 §1).
- The round (arc) join at a concave edge of an inward shell, or a convex
  edge of an outward one; the sharp join is what `offset_faces` makes, and
  the arc is one backlog line shared with it.
- A shell that changes topology (a wall thinner than the thickness at a
  face that then vanishes and its neighbours meeting): refused.
- Thicken (a sheet grown to a solid) and draft: backlog lines, ADR-0047.
- The multi-tool boolean, split by a plane, per-face tessellation: their own
  plans.

## Design deltas

- **`arris-ops`, public**: `pub fn shell(m: &mut Model, body: Body, openings:
  &[Face], thickness: f64, side: ShellSide, control: &Control<'_>) ->
  Result<(Body, Provenance), OpError>`; `pub enum ShellSide { Inward,
  Outward }` (a thickness has no sign of its own: `thickness` is positive,
  and the sign `offset_faces` takes is the side's). New `Reason::Shell(
  ShellReason)` group: `NoWalls` (every face an opening), `RepeatedOpening`,
  `OpeningNotInBody`, `OpeningDragged` (an opening tangent to a wall, which
  the wall's move would drag). The offset's own refusals reach the caller as
  `Reason::Offset(..)`, naming the entity, since they are the offset's: no
  second copy of each. A non-finite or non-positive thickness is
  `InputReason::NonFinite` / `NotPositive`. Each a `CHANGELOG.md`
  `### Breaking` bullet (`Reason` grows a group, an exhaustive `match`
  adds its arm) and the binding's arm in the commit that adds it
  (kernel.md §API).
- **`arris-ops`, crate-private**: `shell/` by phase over `offset/`.
  `offset::build` is split so the phases up to the rewrite — the chain, the
  moves, the vertices, the edges, the new faces — are one function returning
  the offset's pieces (`Offset`), which `offset_faces` rewrites in place and
  `shell` assembles into a second skin; no phase is copied.
- **Topology** (docs/DATA-MODEL.md §Topology): the shell's result is built
  through the assembler, not `rewrite`, because it adds faces and a shell.
  Faces: the body's, `Modified` where a loop changed; the offset's moved
  faces as the inner skin, reversed for `Inward` and not for `Outward`; for
  each opening one face on its surface whose loops are the body's rim and
  the skin's rim, or one merged loop where two openings share an edge (that
  edge is in no face of the result). One shell when any opening exists, two
  when none, the void's inside the outer's (`ShellNesting` holds).
- **Check level**: `Level::Full` on the result in every profile, as
  `offset_faces` runs it, a global violation mapped to
  `OffsetReason::SelfIntersects` (the skin ran into the outer faces).
- **Provenance** (docs/DATA-MODEL.md §Provenance): the outer face `Modified`
  from itself (same role as before, new id as `offset_faces` gives); its
  inner copy, edges and vertices `Generated` from it, so a consumer names
  the inner face after the face it came from; each opening's rim face
  `Modified` from the opening; the body `Modified` and, for a void, the
  second shell `Generated` from the body.
- **Recipe grammar** (both interpreters, `arris_debug::fixtures` and
  `tools/oracle/oracle/recipe.py`): `shell of <name>, openings [[x,y,z],
  ...], thickness, side` — openings named by a point on them, as `offset`'s
  faces are; `prop::recipe` and the differential draw it.
- **Facade and binding**: `arris::ops::shell` by the existing re-export;
  `Model.shell` in `arris-py` with `side` as `"inward"` / `"outward"`, its
  stub and docstring example.
- **ADR-0049** — shell: the two sides and the thickness's sign, the
  skin as the offset of every face but the openings, the rim face and the
  shared-edge merge, the void as a second shell, the refusals, `Full` in
  every profile, and the oracle's driving (step 1).

## Steps

Complexity grades the human uses to pick the agent for a step: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[2]** — The oracle and the grammar. `shell` in `recipe.py`
  driving Open CASCADE (`BRepOffsetAPI_MakeThickSolid` by join, the
  intersection join, openings named by a point, a negative offset for
  `Inward`) and in `arris_debug::fixtures`, the Rust runner refusing it
  until step 2. Where Open CASCADE refuses or crashes (no opening is the
  likely one) the driver builds the void as the body less its inward
  offset, or the fixture is held to its closed forms
  (`analytic.measure_differs`, ADR-0015), the fallback recorded. Fixtures
  under `regression/` with `expected.json` and closed forms: a box open on
  top, inward and outward, a closed box (a void), a box open on top and one
  side (adjacent openings), a box open on two opposite faces (a square tube),
  an L-bracket open on one face. ADR-0049 written here.
- [x] Step 2 **[3]** — The planar core with no shared edge between openings:
  split `offset::build` into the offset's pieces and its rewrite, then
  `shell` over bodies of planes, none or any non-adjacent openings, both
  sides. The assembly: outer faces, the inner skin, each opening's rim
  face with its two loops, the void's second shell; provenance as above;
  `Full` in every profile; `ShellSide`, `Reason::Shell` with `NoWalls`,
  `RepeatedOpening`, `OpeningNotInBody` and the binding's arms. Fixtures
  move to `shell/` and are blessed: the box open on top (both sides), the
  closed box, the square tube, the L-bracket. Checked at this step: the
  assembler and `mass_properties` take a body of two shells.
- [x] Step 3 **[3]** — Adjacent openings: two openings sharing an edge, the
  rim faces' outer and inner loops touching along it, merged into one loop
  with the shared edge in no face; the same where three openings meet at a
  vertex, and an opening whose neighbour is an opening across a seam.
  Fixtures: the box open on top and one side (both sides), a box open on
  three faces, a wedge open on both its slanted faces.
- [x] Step 4 **[2]** — Quadric walls: cylinders, cones, spheres and tori
  hollowed, seams and poles carried by `offset_faces`' phases. Fixtures: a
  tube (cylinder open at both ends), a cup revolved from a profile with an
  arc and a cone, a hemispherical bowl open on its flat, a closed sphere
  (void) and a torus ring (void), each inward and outward where it
  holds; a cylinder of radius below the thickness
  (`SurfaceCollapses`) and an elliptic-extrusion face (`NoExactOffset`),
  refused.
- [ ] Step 5 **[3]** — The dragged chain and the global refusals:
  `OpeningDragged` where an opening is tangent to a wall (a face beside a
  fillet that is itself an opening), the blend and its far face dragged
  where the opening is not tangent; `Vanishes`, `VertexSplits` and `Gap`
  as the offset raises them, and `SelfIntersects` where the skin crosses
  the outer faces (a thickness past the thinnest wall). Fixtures: a
  filleted box open on top (the fillets dragged, both sides), a filleted
  box with its top fillet's neighbour opened (refused), a thin plate hollowed
  past its half-thickness (refused), a boss's shell past the boss's
  radius (refused), a pyramid hollowed (apex splits, refused).
- [ ] Step 6 **[1]** — The facade and the binding: `Model.shell` with cancel
  and budget, `side` as a string, its stub and docstring example, a pytest
  per refusal group, the rustdoc example on `shell`.
- [ ] Step 7 **[2]** — Properties, sharded and seeded: a shell's volume plus
  the volume of `offset_faces` of every face but its openings by the signed
  thickness equals the body's (inward) in random poses on a box, a cylinder
  and a rounded box; a closed void's volume against the closed form; shell
  commutes with `transform`; the result is an operand of fillet, chamfer, a
  cut and `offset_faces`, checker green; every moved wall's thickness
  against `thickness` by the distance between a face and its inner copy;
  `prop::recipe` and the differential draw `shell` (an opening or none on a
  box or cylinder operand with no blend).

## Acceptance

`ARRIS_GATE=full` green; the corpus's `shell/` fixtures (the twenty-odd
named above that build) checker-green at `Full`, matched to Open CASCADE's
volume, area, centroid, inertia, counts and probes, or to their closed forms
where ADR-0015's evidence is recorded; every refusal fixture fails with its
named reason; step 7's properties at 256 cases; the differential at 1000
recipes with `shell` drawn and no new unexplained disagreement; the `python`
job's pytest, docstring examples and `mypy.stubtest` green.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Operations — `shell`: its phases over `offset/`,
  the two sides, the rim face and the merge, the void; §Errors — the `Shell`
  group; the corpus areas and the capability table (a "Hollow a solid" row).
- `docs/DATA-MODEL.md` §Topology — a body of two shells from a shell;
  §Provenance — what a shell records (the inner copy `Generated`).
- `tests/fixtures/README.md` — the `shell/` area in the lint's list, the
  `shell` op in the grammar and the new `expect_error` names.
- `tools/oracle/README.md` — how the shell is driven and its fallback.
- `docs/ROADMAP.md` §C7 — status line: shell landed.
- `docs/BACKLOG.md` — the round join at a concave edge of an inward shell
  (with offset's), whatever refusal the corpus shows is common.
- `CHANGELOG.md` `Unreleased` — what a consumer can now do (hollow a part,
  open faces, a closed void, grow a skin) and the refusals they will hit;
  `### Breaking` holds the bullets the steps wrote.
- `AGENTS.md` current state — shell done, the multi-tool boolean next.

## Open questions

- Answered at step 1 (ADR-0049 §7): Open CASCADE's thick solid gives every
  opened case's closed forms exactly, adjacent openings included (each rim
  face one loop); with no openings it returns the offset solid alone, so the
  driver builds the void as the body less its inward offset (or its outward
  offset less the body), which gives the closed forms. No fixture needs
  `analytic.measure_differs`.
- Answered at step 2: the assembler takes a body of several shells and a
  kept face used reversed (`FaceSpec::Keep` with either orientation), the
  checker nests the void at `Full`, and `mass_properties` integrates every
  face use of the body, so a reversed second shell subtracts: no change to
  either. The shell builds its `Assembly` itself rather than through
  `rebuild::rewrite`, whose stored→effective walk it shares
  (`rebuild::stored_to_spec`).
- Found at step 2: a wall kept by id that also has its skin copy
  `Generated` from it is recorded `Modified` into itself, or the audit finds
  it recorded with no origin — what the plan's provenance line meant by
  "Modified from itself". `OpeningDragged` landed with the other
  `ShellReason`s at step 2, read off the offset's moved set, since a dragged
  opening would otherwise reach the checker as `Internal`; step 5 keeps its
  fixtures. Until step 3, two openings sharing an edge (or one across its
  own seam) are `OpError::Unsupported` naming both.
- Done at step 3: the merge is a cancellation, not a geometric search. An
  edge between two openings and the skin's copy of it lie on one curve (the
  offset re-ranges an edge between fixed faces on its own curve) and are
  walked opposite ways by the rim, so each rim's stored loop and walked-back
  skin loop are spliced, position by position, where they cancel: what is
  left at each end is a piece of that curve, one edge shared by both rims
  (`shell/rim.rs`) — on the body's edge inward (`Modified` from it), beyond
  it on the skin's copy outward (`Generated`, the edge `Deleted`). The
  skin's copy is in no face and is never built. "Across a seam" is the
  same splice on an opening's own seam: a cylinder open on its side and
  top leaves a disc, the top's rim cancelling whole (`Deleted`). Where the
  splice leaves several loops they are grouped into faces by winding on a
  plane, and refused `Unsupported` on any other surface; so is a shared
  closed edge whose vertex moves. Fixtures moved or added under `shell/`:
  the box open on top and front, on three faces at a corner (both sides
  each), a wedge open on both slants (both sides), all Open CASCADE's.
- Done at step 4: the quadric walls needed no code. `offset_faces`' phases
  carry the seams and poles, and the splice of step 3 is only reached
  where two openings share an edge. Twelve fixtures under `shell/`: tube,
  cup (floor, torus, cone), hemispherical bowl, sphere and torus voids,
  each both sides; a pin past its radius (`SurfaceCollapses`) and an
  elliptic prism (`NoExactOffset`), refused. Both sphere voids carry
  `occt_step_refused`: Open CASCADE writes the whole sphere of its cut as
  one `VERTEX_LOOP` with no seam, which the reader does not map (a backlog
  line). A two-shell torus void reports genus 2, one per shell.
