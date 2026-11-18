# Plan: touching-lumps-with-shared-topology

- Started: 2026-11-18
- Milestone: C12, the training harness's refusals (docs/ROADMAP.md §C12 **In**,
  "Next are contacts that share topology")
- Idea: docs/ideas/touching-operands-fuse-to-several-shells.md (absorbed:
  Q2, option A, decided 2026-11-17); ADR-0088 §6 names this plan

## Goal
Operands whose result lumps would share an edge used by four faces, or a
vertex, build one `Solid` of touching lumps: each lump gets its own copy
of the edge or vertex, so shells still share nothing (ADR-0006, ADR-0088 §1).
`edge-touching-fuse`, `cross-cylinders-cut`, `fuse-frustum-box-rim-point`
and its `-far-side` twin build `Full`-green to Open CASCADE's volume, area
and centroid. Their counts differ by the copies, which `counts_differ`
states. Where the material of one lump would touch itself through such an
entity, the result is still `NonManifold`, naming it: `pockets-touching-edge-cut-many`,
the pinch, a void that shares an edge with its outer shell, and a later
boolean that joins two touching lumps while leaving the contact exposed.
A new checker row catches the same case in any body. `weld` keeps copies
apart, so every lump's mesh is closed on its own.

## Non-goals
- A shell touching itself (`regression/singular-bore-cut`, the pockets): a
  `General` body, still the backlog line it is.
- Faces tangent at a shared edge (a seam on the ruling two cylinders touch
  along): the radial order has no gap to pair across, so it stays
  `NonManifold`, as one backlog line.
- Undoing the split a touch leaves, such as the box edge the rim point
  splits: a two-valent vertex stays, and `counts_differ` states it.
- A shared contact edge in an Open CASCADE-like compound (ADR-0088 rejected it).

## Design deltas
- **ADR-0090, "a shared contact is copied per shell"** (follows ADR-0088 §6):
  1. *Radial pairing.* For an edge piece used more than twice,
     `Build::shells` sorts its uses around the edge's tangent at the
     middle of its range. It reads each use's direction into its face
     (`n × t`, effective orientation) and pairs each use with its neighbour
     across the material side. The union-find unions only within a pair. A
     tie, two directions within the angular tolerance, stays
     `NonManifold` naming the edge.
  2. *Copies per shell.* `rebuild::assembly` keys new edge and vertex specs
     by `(ERef, shell)` and `(VRef, shell)` where an entity is reached by
     several shells. The first shell, in shell order, keeps the original
     key (and a kept id); each later shell gets a new entity with the same
     curve, range, point and tolerance. Pcurves are per use already
     (`PieceUse`), so no copy needs a new one. `Plan::new_edges` and
     `new_vertices` carry the shell. Ids are deterministic by shell order.
  3. *One lump refuses.* `NonManifold` names the entity when both pairs of
     an edge fall in one shell (the pockets), or when two shells that
     share an entity are placed in one lump by `lump_order` (a void sharing
     an edge with its outer shell). The same applies to an operand's
     coincident copies that end in one lump after a later boolean; this
     joins `inherited_contact` (ADR-0089).
  4. *Provenance.* A source edge, vertex or section maps to every copy:
     `modified` (or `generated`) lists them all.
     `rebuild::write_provenance`'s `edge_id` / `vertex_id` return a list.
  5. *Weld and mesh.* `TriMesh::weld` keys on `EdgeId` (ADR-0052), never on
     position, so copies are never merged. Each lump's mesh is closed, and
     a contact's positions appear once per lump. A consumer that merges
     positions gets the non-manifold mesh edge that Open CASCADE's
     compound also gives. This answers the idea's open `weld` question.
  6. *The checker row.* **V4**: no two distinct vertices of one shell lie
     within the larger of their tolerances (Fast; a sweep sorted on x).
     Different points at the ends of one non-degenerate edge are allowed
     (ADR-0074). The boolean result guard runs it as a safety net for an
     edge or vertex copied into the wrong lump.
- **Public types:** `arris_check::Violation` gains V4's variant
  (`CoincidentVertices { a, b }`, breaking: `Violation` is exhaustive).
  The changelog lists it under `Breaking`, and the binding maps it where
  it maps violations. The boolean result guard uses the doc-hidden
  `arris_check::check_boolean_result`. `InputReason::NonManifold`'s
  doc and text narrow to "would touch itself".
- **Crate boundaries:** none (`arris-ops::rebuild`, `boolean/result.rs`,
  `arris-check`). Design docs: see *Docs to update*.

## Steps
Grades are tiers (`.agents/tiers.md`): **[1]** routine, **[2]** careful
(a geometric or numeric case within a given design), **[3]** unproven (an
algorithm whose robustness has to be established here).

- [x] Step 1 **[2]** — Prove the representation before the boolean makes
  it. Write ADR-0090. Add `arris-check/tests/full.rs` cases with hand-built
  bodies (`solid_of`): two cuboids on one line with their own coincident
  edges, two at one corner, and a cuboid touching `cylinder_in`'s top rim
  at one point. Each is `Full`-green with nothing unchecked, gives two
  lumps from `lumps`, and has a volume that is the sum. Each lump's
  tessellation is closed, and `weld` of the faces is closed with the
  contact's positions duplicated (delta 5).
- [x] Step 2 **[2]** — V4 in `arris-check`: the row, its `Violation`
  variant, the binding, and a hand-built shell through two coincident
  vertices refused by it. Run the full corpus (`real_*` included). A V4
  hit on a body this plan does not make is decided as in Open questions 1.
- [x] Step 3 **[3]** — Radial pairing in `Build::shells` (delta 1). Unit
  tests in `result.rs`: two boxes on an edge give two shells; the pockets
  give one shell with both pairs and are refused `NonManifold` naming the
  edge; and a tie is refused. The step still ends `NonManifold` on the
  shared entity, since copies come in step 4.
- [x] Step 4 **[2]** — Copies per shell, provenance, and the one-lump
  refusal (deltas 2–4). `edge-touching-fuse` builds, with `counts_differ`
  and a re-blessed dump. The edge half of
  `boxes_touching_along_an_edge_or_at_a_corner_are_non_manifold` becomes
  a two-lump body with audited provenance. A new fixture,
  `three-wedges-one-edge-fuse-many`, has three 60° prisms round one edge,
  three lumps, and Open CASCADE's values. The pockets keep `NonManifold`,
  with their description saying why (one lump).
- [ ] Step 5 **[2]** — Point contacts. `cross-cylinders-cut`, both
  `fuse-frustum-box-rim-point` fixtures and the boxes' corner half build to
  Open CASCADE's values. Turn the frustum test in `boolean.rs`, and
  `test_tangency.py`'s, into two-lump assertions. The `boolean_prop`
  crossing cylinders' cuts hold `V(a) − V(a ∩ b)` and two lumps.
  Re-measure the `tolerance_band.rs` header table.
- [ ] Step 6 **[2]** — Touching lumps as an operand. A chain fixture from
  `edge-touching-fuse` covers three cases: a hole cut in one lump away from
  the contact (two lumps); a bridge over the whole contact (one lump, the
  copies inside it dropped); and a bridge over part of it, refused
  `NonManifold` naming the operand's copies (delta 3). The first two use
  Open CASCADE's values, and the third is held to the refusal.

## Acceptance
- The corpus binary passes. The four fixtures and the two new ones are
  checker-green within tolerance, against Open CASCADE's volume, area and
  centroid. Counts are held where `counts_differ` does not excuse them, and
  the oracle's reading of Arris's STEP is held too. The pockets and the
  partial bridge are the named `NonManifold` refusal.
- `grep -rl --include=fixture.json '"non-manifold"' tests/fixtures` names only
  `boolean/pockets-touching-edge-cut-many` and step 6's partial bridge
  (run 2026-11-18: five fixtures today, each in step 4 or 5's files).
- `crossing_cylinders_obey_every_identity` passes at 256 cases with the
  cuts held to their identities. `tools/py-test.sh tests/test_tangency.py`
  passes.
- The full gate is green. The refusal histogram and the script corpus are
  re-run and printed: the `NonManifold` row falls, and no row rises above
  C11's close.

## Docs to update on completion
- `docs/DATA-MODEL.md` §Entities: shells share nothing, and a contact
  that would share topology is copied per shell. §Invariants: V4, and B1's
  mention of copies.
- `docs/ARCHITECTURE.md` §Booleans (radial pairing, copies, one-lump
  refusal); §Errors `Degenerate` row (`NonManifold` now means "a shell
  or a lump would touch itself at a shared edge or vertex");
  §Tessellation (`weld` keeps copies apart).
- `docs/ROADMAP.md` §C12: the touching-lumps line says that shared
  contacts build.
- `docs/BACKLOG.md`: the tangent-at-a-shared-edge line.
- `tests/fixtures/README.md`: `non-manifold`'s meaning.
- `AGENTS.md` current state; `CHANGELOG.md` `Unreleased` (fuses and cuts
  that touch along an edge or at a corner now build; V4 under `Breaking`).
- `GLOSSARY.md` **Touching lumps**: each lump keeps its own copy of a
  contact edge or vertex.

## Open questions
1. *Settled in step 2.* V4 runs on boolean results. Both OCCT STEP
   crescent read-backs require loop splicing as well as merging vertex
   ids (`docs/ideas/sew-step-crescent-vertices.md`). V4 allows the
   distinct ends of a short edge, preserving the CTC-05 topology of
   ADR-0074 (ADR-0090 amendment).
2. *Settled in step 3.* The radial order uses `Precision::angular_tolerance`
   to decide ties. The rays come from local unit normals and the unit edge
   tangent, so the threshold has no dependence on the edge's trim length.
3. *Settled in step 4.* The wedge fixture matches three source edges
   to one common-block representative. A face-boundary check accepts
   that representative when two of the matched edges come from later
   operands. Each source still gets its own same-parameter pcurves on
   the representative before the build pairs the uses.
