# Plan: touching-lumps-along-a-curve

- Started: 2026-11-17
- Milestone: C12, the training harness's refusals (`docs/ROADMAP.md`)
- Idea: `docs/ideas/touching-operands-fuse-to-several-shells.md` (the ADR
  and the curve touches absorbed; point touches and shared-topology
  contacts stay there for the next two plans)

## Goal

Two solids that touch from outside along a curve fuse to one `Solid` of
**touching lumps**. Their volumes and areas add up, the full checker
passes the result, and each lump keeps its own edges and vertices. This
covers the training harness's `cotter_pin` and `hinge` programs (about 220)
and the fixture `boolean/tangent-cylinders-fuse`. ADR-0088 amends ADR-0006,
and B1 admits a touch between shells while still refusing a crossing. A
body of touching lumps can be measured, meshed, classified, written to
STEP and used as an operand. `TangentContact` remains only where the
result's *one* shell would touch itself.

## Non-goals

- Point touches, where two surfaces meet at isolated points: the torus and
  box, a ball on a disc, the `cut` and `common` of a box edge tangent to a
  wall, backlog line "`common` refuses solids that touch at one point", and
  the nesting `OpInternalError` of the off-centre disc. These are the next
  plan, `touching-lumps-at-a-point`.
- Contacts that share topology: an edge of four faces, or a vertex two
  shells reach (`edge-touching-fuse`, `cross-cylinders-cut`,
  `pockets-touching-edge-cut-many`, the frustum rim-point pair). They stay
  `NonManifold`; they are the third plan (idea, Q2).
- A `cut` or `common` whose result splits into lumps that touch along a
  contact. That needs the contact to split a face, and it stays
  `TangentContact`.
- A shell touching itself (the pinch), `General` bodies, and `interference`
  reporting a touch.

## Design deltas

- **ADR-0088, "Lumps of one solid may touch"** (step 1), amending ADR-0006.
  The interiors of a `Solid`'s lumps are disjoint. Their boundaries may
  meet along curves where their surfaces touch (`MeetKind::Touch`) and at
  isolated points, but never across each other or over an area.
  "Shells share nothing" stays as it is. The sentence "two lumps meeting
  at a point or along a line are a `General` body" goes. Its argument (idea
  §Constraints): the multi-lump `Solid` already plays the role of Open
  CASCADE's compound.
- **Checker, B1** (`arris-check/src/full.rs::faces_meet`, `lumps.rs`). A
  face pair from two different shells is a violation only where it crosses
  (a `Crossing` curve or point interior to both) or overlaps (`Coincident`
  regions). A `Touch` curve or point is admitted. S5, inside one shell,
  keeps its rule. The nesting ray starts at a vertex of the shell clear of
  the other shell by its tolerance, or else at an interior point of a face.
  This way a seam vertex lying on the contact never leaves the nesting
  undecided. Touching from inside is still a nesting violation (an outer
  shell inside an outer shell).
- **Boolean, the slit** (`boolean/result.rs::contacts`, `shells`).
  `contacts()` stops refusing. It records each contact both of whose pieces
  survive, as a slit. Once the kept pieces are grouped into shells, a slit
  is looked up by the kept piece of each face that holds its midpoint:
  - pieces in two different shells: touching lumps, built;
  - pieces in one shell: `TangentContact`, with the same entities as today.

  The paves at a contact's ends (a box edge, or a rim circle, touching the
  other face) make no vertex that two shells share (ADR-0088).
- **Public API.** No type or signature changes. These docs change:
  - `BooleanReason::TangentContact`'s doc: one shell touching itself;
  - `InputReason::NonManifold`'s doc and its `Display` text: an edge or
    vertex the shells would *share*, no longer just any touch;
  - `fuse`'s and `cut`'s error paragraphs (`boolean/mod.rs`);
  - the Python stubs' docstrings where they quote these.
- **Design docs.**
  - `docs/DATA-MODEL.md`: §Entities (the body bullet on lumps) and
    §Invariants (B1).
  - `docs/ARCHITECTURE.md` §Operations: the contact paragraph ("Before
    any face is split, every contact is decided…").
- **Glossary.** Add **Touching lumps**, as worded in the idea.

## Steps

Grades: **[1]** routine, the design says exactly what to write and the
tests are mechanical; **[2]** careful, a geometric or numeric case within
the design; **[3]** unproven, an algorithm whose robustness is established
here.

- [x] Step 1 **[3]** — ADR-0088, `GLOSSARY.md`, DATA-MODEL B1 and
  §Entities, and B1 in the checker as the deltas say. Tests in
  `arris-check` use bodies assembled from two shells:
  - two cylinders tangent from outside: passes;
  - a ball on a box face: passes;
  - two cylinders crossing: B1 violation;
  - a cylinder tangent inside another as a second outer shell: nesting
    violation;
  - a seam vertex on the contact line: decided, not unchecked.
- [x] Step 2 **[3]** — The slit decided by shells (deltas). The fixture
  `boolean/tangent-cylinders-fuse` builds `Full`-green to its closed form,
  with Open CASCADE's values re-taken. `tangent-hole` and
  `ball-in-bore-cut` still refuse `TangentContact` (one shell). The error
  docs and Display text change in this commit, and `boolean.rs`'s two held
  `TangentContact` asserts are reviewed.
- [x] Step 3 **[2]** — The harness's poses as fixtures, with Open CASCADE's
  values, built checker-green:
  - `boolean/fuse-cylinders-tangent-equal-height` (`cotter_pin`): the flush
    end discs' rims touch at a point;
  - `boolean/fuse-box-cylinder-tangent-side` (`hinge`, CadQuery-centred):
    the box's edges touch the wall at the ends of the contact.

  Python tests in `test_tangency.py` drive both through `arris.build`
  and hold the two shells, volume and area.
- [x] Step 4 **[2]** — Rename the property to
  `a_cylinder_tangent_to_a_box_face_fuses_to_touching_lumps`. Its `fuse`
  arm landed in step 2 and holds two shells, additive volume and area,
  a clean `Full` report and the box's faces by id. Remove the external
  touch exception in `coplanar_prop.rs` and keep the same-lump refusals
  in `multi_tool.rs`.
- [x] Step 5 **[2]** — Re-bless `tolerance_band.txt`. Each `fuse` outcome
  that flips from `TangentContact` is checked built, additive and
  checker-green. A flip to anything else is a finding, which becomes a
  regression fixture and a backlog line.
- [x] Step 6 **[2]** — A touching-lumps body downstream (`arris/tests`):
  - `mass_properties`, `mesh` (each lump watertight), and
    `classify_points` (`on` at the contact);
  - STEP written and read by the oracle as two solids, and the native
    format round-tripped;
  - as an operand: a hole cut in one lump builds; a plate fused across both
    lumps is `TangentContact` (one shell touching itself); `common` with a
    box over the contact builds.

  Each is checker-green, or the named refusal.

## Acceptance

- `tangent-cylinders-fuse` and the two step-3 fixtures build `Full`-green
  to Open CASCADE's volume, area and lump count within their tolerances.
  Every other `TangentContact` fixture keeps its refusal.
- Step 4's property passes at 256 cases.
- `tolerance_band` passes against its re-blessed outcomes.
- The refusal histogram and the script corpus are re-run and printed: the
  `fuse` `TangentContact` rows go down, no row goes above C11's close, and
  there is no panic.
- The full gate is green.

## Docs to update on completion

- `docs/ROADMAP.md` §C12 **In**: the touching-operands line gets "curve
  touches done", with the next two plans named.
- `docs/ARCHITECTURE.md` §Operations: confirm step 2's paragraph still
  reads true after steps 3–6.
- `CHANGELOG.md` `Unreleased`: solids that touch along a line now fuse to
  one solid of two lumps, where before they were refused.
- `AGENTS.md` current state: one clause.
- `GLOSSARY.md`: **Touching lumps**, already done in step 1.

## Open questions

- **Settled (step 3)**: a box edge touching the wall off every vertex
  already makes no section vertex. Where two flush rims touch from
  outside at a wall contact's end, a seam crossing the opposite cap
  plane would make a shared vertex. The hits locate the contact and are
  left out of the vertex merge (ADR-0088).
- **Settled (step 2)**: a slit is decided by *lump*, not shell. A cavity
  touching the wall round it (`ball-in-bore-cut`, `pin-in-bore-cut`) is two
  shells of one lump, its material touching itself, and stays
  `TangentContact`; only two shells of different lumps are touching lumps.
  A face with pieces in a lump the other face's are in is refused too (the
  contact cannot be placed among them without a region test).
- **Settled (step 5)**: the six `TangentContact` outcomes that build are
  the exact `TangentCylinders` and `TangentOutside` contacts, each
  repeated at zero perturbation for all three motions. Both build to
  two lumps with additive volume and area and a clean `Full` report.
  The 22 other changes are built outcomes whose counts now match the
  exact contact. No `FlushBoxes` outcomes changed.
- **Settled (step 6)**: the pave compares faces across operands, so a
  contact inherited from two shells of one operand could survive after a
  plate joined those shells. After assembly, check the surviving face
  regions from such shells within one result shell for a touching curve,
  using the checker's face-pair predicate via the doc-hidden
  `first_touching_face_pair`. The refusal names the original faces.
  A plate that fills the whole contact builds one lump.
