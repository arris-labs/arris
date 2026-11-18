# ADR-0088 — Lumps of one solid may touch

- Status: accepted (2026-11-18)
- Amends: ADR-0006 (the sentence "Two lumps meeting at a point or along a
  line are a non-manifold body, C7's `General`, not a `Solid` with a
  special case", and B1's "no two shells of the body meet")
- Follows: ADR-0008 (the ray cast B1 places shells by), ADR-0018 and
  ADR-0061 (the sections B1's meeting test reads)

## Context

About 530 programs of the training harness fuse operands that touch only
along a line or at a point: two tangent cylinders, a box and a cylinder
tangent to its side, a torus and a box at one point, a ball on a disc.
Open CASCADE builds each as a compound of two solids; Arris refuses each
as `TangentContact` or `Unsupported`, and its own checker agrees with the
refusal: ADR-0006 made touching lumps a `General` body, because in the
reference trees touching belongs to a compound and never to a solid.

ADR-0006 also gave the reason a `Solid` of several lumps exists at all:
the multi-lump `Solid` plays the role of Open CASCADE's compound.
`BRepCheck_Analyzer` rejects a `TopoDS_Solid` of two outer shells whether
they are apart or touch; Arris accepts two that are apart. Two lumps that
touch are the compound's other case, so the line ADR-0006 drew between
"apart" and "touching" is the arbitrary one. A `General` body would also
cost the consumer what ADR-0006 refused to cost it: no `measure`, no mesh,
no operand for the next operation.

Today's code already crosses that line without saying so: a torus's tracer
does not see an isolated touch, so a fuse of a torus and a box at one point
that sits off the torus's seams returns two lumps touching, and the full
checker passes them.

## Decision

1. **The interiors of a `Solid`'s lumps are disjoint. Their boundaries may
   meet.** They meet along curves where their surfaces touch
   (`MeetKind::Touch`: tangent along the curve, the surfaces not crossing)
   and at isolated points where they touch, but never across each other
   and never over an area. **Shells share nothing** stays as ADR-0006 has
   it: no edge and no vertex is used by two shells of one body. Each lump
   keeps its own edges and vertices, and a contact is where two surfaces
   are within tolerance of each other, never an entity both shells use.
   The sentence that made a touch a `General` body goes.
2. **B1 admits a touch between two shells and refuses the rest.** A pair of
   faces of two different shells is a violation where their surfaces
   *cross* (`MeetKind::Crossing`, a crossing through a singular point
   included) at a point interior to both, and where *coincident* surfaces
   carry faces whose regions overlap. A `Touch` curve or point is admitted.
   S5, inside one shell, keeps its rule: a touch interior to both faces of
   one shell is still a violation. The test is `faces_meet`, given what
   the meeting is held to; B1 and S5 differ in that one argument, so a
   touch and a crossing are told apart by the intersector's own verdict and
   by nothing the checker adds.
3. **A shell is placed by a point clear of the other shell.** B1's nesting
   asks a ray cast whether one shell lies inside another, at a vertex of
   it. A vertex on the contact has no parity: a cylinder's two seam
   vertices lie on the ruling another cylinder touches it along. The
   shell is placed instead by the first point of it that is on none of the
   other shell's faces or edges (by the classifier's own `on`, at the
   faces' tolerances), its vertices first, then interior points of its
   faces on a grid over each (u, v) box. A shell none of whose first 64
   points is decided leaves the row unchecked, as a shell no ray could be
   classified against does today.
4. **Touching from inside is still a violation.** A lump inside another
   lump, tangent to its wall, is an outer shell whose innermost container
   is an outer shell: `OuterInOuter`, placed by an interior point since
   its vertices may be on the contact. Touching from inside a cavity is a
   void in a void's or a lump in a void's nesting, decided the same way.
5. **A fuse can build a curve contact between lumps.** The result's pieces
   are grouped into shells and the shells into lumps. A curve contact whose
   two faces' pieces lie in different lumps is touching lumps. Where they lie in one lump — one shell
   touching itself, or a cavity touching the wall round it, a ball in a
   bore of its radius — the lump's material would touch itself and it stays
   `TangentContact`. B1 itself still admits a void touching its outer
   shell (it judges shells, not material); only the boolean refuses to
   make one. `InputReason::NonManifold` is then
   an edge or vertex two shells would *share*, no longer any touch.
6. **Contacts that share topology** (an edge of four faces, a vertex two
   shells reach) are admitted by this rule too and are built by a later
   plan, which gives each lump its own copy of the edge or vertex. They
   stay `NonManifold` until then; this ADR draws no arbitrary line between
   the two kinds of touch.

   ADR-0090 implements the copies. A shared edge or vertex can belong to
   distinct touching lumps, and contact within one lump is still refused.

### Contact ends at coincident caps

Two externally tangent walls with flush caps can meet at a rim tangency
on a wall's seam. The seam crosses the opposite cap plane at its own
vertex. The rims touch from outside, so the hit describes an end of the
wall contact and splits neither cap. The rim-on-wall and seam-on-cap
hits stay in the pave model for contact detection and are left out of the
section-vertex merge. A box edge tangent to a wall away from a vertex
already makes no section vertex. Both results keep each lump's topology
whole.

## Consequences

- `arris_check`'s B1 changes with no public type changing; the doc of
  `ShellNestingFault::Overlap` says "cross or overlap", and
  `docs/DATA-MODEL.md` §Entities and B1 are rewritten. `lumps()` is
  the same code and follows.
- A test body of two shells touching is built from `sample::cylinder_in`
  (a placed cylinder, added to `arris-debug`'s samples for the purpose).
- The intersector supplies the same surface verdict to B1 and S5. The
  checker tries extra placing points when the first point leaves nesting
  undecided. For S5, trim intersections subdivide touching curves so a
  short interior contact still has a test point.
- A solid of touching lumps is a consumer's single shape where Open CASCADE
  has a compound: STEP writes a `MANIFOLD_SOLID_BREP` per lump, which the
  oracle reads as that many solids. The native format round-trips the
  touching lumps as one solid.

## Alternatives considered

- **A `General` body** (ADR-0006's answer): the data model holds it, but no
  operation takes it, so a fused hinge could not be measured, meshed or
  used as an operand. Open CASCADE's compound does share the contact edge,
  which is this option's one advantage, and nobody has asked for it.
- **Two bodies per operation**: an API break; ADR-0006 refused the shape
  already.
- **A refusal with a clearer name**: the harness counts every refusal as a
  failure whatever it is called.
- **B1 admitting every meeting of two shells**: an overlap or a crossing is
  two lumps sharing material, which is no lump at all.
