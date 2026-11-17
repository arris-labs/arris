# Idea: touching-operands-fuse-to-several-shells

- Status: Accepted (A); the ADR and the curve touches (cases 1 and 2's
  `fuse`) done under ADR-0088. Still to plan:
  point touches (`touching-lumps-at-a-point`), then shared-topology
  contacts (Q2). Rewritten 2026-11-17 after reproducing the four cases on
  `0.12.0-dev`: they are three different failures, not one, and two of them
  hit `cut` and `common` as well as `fuse`.
- Raised: 2026-11-13; consolidated 2026-11-16 from items 1, 2, 4 and 7 of
  the training harness's `fuse-contact-refusals` (items 3, 5 and 8 went to
  `tangency-faults` and `flat-edge-read-as-tangent-contact`, item 6 to
  `revolve-circle-meets-the-axis`).
- Prompt: none from the human; filed by the training harness (C12,
  ADR-0020's amendment of 2026-11-16) against 0.10.0. Each case is a fixture
  on the harness's side, with a closed form that CadQuery 2.3.0 (Open
  CASCADE) matches. The harness reads a body of several lumps as Open
  CASCADE's compound of solids.

## Problem

About 530 harness programs fuse operands that touch only along a line or at
a point. Open CASCADE builds each one as a compound of two solids. Arris
refuses all of them. Rerun on today's tip, with the same poses:

| Case (programs) | `fuse` | `cut` / `common` |
|---|---|---|
| 1. two unit cylinders tangent along a ruling (159 `cotter_pin`) | `TangentContact` | builds / `Empty` |
| 2. box 4×4×1 and a cylinder tangent to its side (61 `hinge`) | `TangentContact` | `Unsupported` cylinder against a box edge (a line tangent to it) |
| 3. torus and box touching at one point (80 `torus_link`) | `Unsupported` plane against a vertex | same |
| 4. sphere r 2 on a cylinder's end disc at the pole (231 `ball_knob`) | `Unsupported` plane against sphere | same |

The table shows three failures:

- **(a) A touch along a curve, where both pieces survive** (cases 1, 2).
  The selection names it `TangentContact` (`boolean/mod.rs`, the
  fuse/cut doc). This is a deliberate refusal, held by the fixture
  `boolean/tangent-cylinders-fuse`.
- **(b) A piece classified on a point touch** (cases 3, 4, and case 2's
  `cut` and `common`). The intersector already has the touch:
  `MeetKind::Touch`, where a plane meets a sphere at one point. But a face
  the touch does not split is classified at its own sample point, and in
  these symmetric poses that point *is* the touch: the disc's centre, the
  box face's centre on the torus's seam vertex. The piece then lands `On`
  something it is not tangent along, and `result.rs::unsupported` reports
  this as "no closed form". That message is wrong. Move the disc off the
  centre (0.3) and the `fuse` becomes an `OpInternalError` instead: "no ray
  from s2 could be classified", B1's nesting ray failing at the touch. Turn
  the torus so the contact sits on its equator seam and the `fuse` becomes
  `Unsupported` plane against circle.
- **(c) The invariant.** B1 says that no face of one shell meets a face of
  another (ADR-0006, `docs/DATA-MODEL.md` §Invariants). Today's code
  already breaks it. Shift the box off the torus's seams and the `fuse`
  returns two lumps touching at one point, and the *full* checker passes
  them: the torus tracer does not see an isolated touch. So an `Ok` body
  already breaks a written invariant without the checker noticing.

The same family includes four contacts that share topology: an edge or a
vertex the result's two shells would share. Each is `NonManifold` today,
and Open CASCADE builds each one: `edge-touching-fuse`, `cross-cylinders-cut`,
`pockets-touching-edge-cut-many`, and `fuse-frustum-box-rim-point` with
its `-far-side` twin. The harness has not filed this kind, but it comes up
in cuts as well as fuses.

## Constraints it runs into

- **ADR-0006** decided that "Two lumps meeting at a point or along a line
  are a non-manifold body, C7's `General`", because in the reference trees
  touching belongs to a compound and never to a solid. Building these cases
  in one `Solid` reverses that sentence, which needs a new ADR that amends
  it. The argument for reversing it comes from the ADR itself: Arris's
  `Solid` of several lumps already plays the role of Open CASCADE's compound.
  Two lumps that are apart are one `Solid` here, where `BRepCheck` rejects a
  `TopoDS_Solid` of two outer shells. Letting those lumps touch is the
  compound's other case.
- ADR-0006's **"shells share nothing"** (no edge and no vertex used by two
  shells) is what keeps every walk manifold, shell by shell. Option A keeps
  it as it is.
- **B1 and S5** (`docs/DATA-MODEL.md` §Invariants): B1's face-pair test
  between shells is `faces_meet` (`arris-check/src/full.rs`). It treats a
  touch and a crossing the same way, so it would need to tell them apart.
- `InputReason::NonManifold`'s doc and message ("which a solid does not
  hold"), and the `TangentContact` clause of the boolean's doc.
- **Closure before breadth** (ADR-0020): a body of touching lumps has to
  work as an operand of every later operation, or be refused by name.
- STEP already writes one `MANIFOLD_SOLID_BREP` per lump, and the oracle
  reads them as that many solids (checked: a disjoint `fuse` round-trips
  as two). Nothing changes in the native format. Python has no public type
  or signature change.

## Options

### A — Touching lumps in one `Solid`, with no shared topology
The lumps' interiors are disjoint, and their boundaries may meet in two
ways: along curves where their surfaces touch (`MeetKind::Touch`), and at
isolated points. They never meet across each other and never over an area.
Each lump keeps its own edges and vertices.

What changes:

- The checker: B1 admits a touch between outer shells and still refuses a
  crossing. The nesting ray starts from a point away from every touch.
  Touching from inside, a lump inside another, stays a nesting violation.
- The boolean: `TangentContact` is kept only where the result's *one*
  shell would touch itself. A piece is classified at a sample point away
  from a point touch, or its touch with a line is decided by the touch.

These are the fuse's consequences; `cut` and `common` get (b)'s fix for
free. Case 1's `cut` already builds this way. Backlog line "`common`
refuses solids that touch at one point" is the same fix. The
checker's silent pass in (c) becomes a valid body.

Cost: two milestone plans of about five steps each.

- **Plan 1** (done): the ADR,
  B1, and touches along a curve (cases 1 and 2's fuse).
  `tangent-cylinders-fuse` flips.
- **Plan 2:** point touches in all three operations (cases 3 and 4, case 2's
  `cut` and `common`, the nesting fault), and a property test that uses a
  touching pair as an operand.

What it forecloses: nothing. The `General` body is still there for a
shell that touches itself (the pinch, backlog).

### B — A `General` body, the contact as shared topology
This is the representation Open CASCADE builds: the compound's two solids
share the contact edge. The data model holds it, but no operation builds
or takes it. `measure`, the mesh, the STEP writer and every boolean's
operand would refuse it as `NotSolid`, so a fused hinge could not be
measured. That is ADR-0006's own reason against `General`. Cost: a cycle.

### C — `fuse` returns the operands as two bodies
This is an API break, and the "union" loses its meaning for the next
operation. ADR-0006 rejected the several-bodies shape already.

### D — A clearer refusal
Cases 3 and 4 are named `TangentContact` or `NonManifold` instead of
`Unsupported`, and the nesting `OpInternalError` is fixed. Cost: two
steps. All 530 programs stay refused.

### Do nothing
The largest refused group stays refused. "No closed form" stays a
misleading name, there is an internal error one pose away, and an `Ok`
body that B1 should reject still passes the checker.

## Recommendation

**A**, in two plans: the curve touches first, because their refusal is the
deliberate one and the ADR has to come before them. Then the point touches.
D loses because the harness counts a refusal as a failure whatever its name
is, and A fixes D's two faults on the way. B costs a cycle and makes every
fused part unmeasurable. C breaks the API.

One thing would change my mind: a consumer that needs the contact as an
edge, for example to mesh both lumps on one shared seam. That is B's case,
and nobody has asked for it.

## The words

The slug says "several shells", but the glossary's word is **lump**: a
shell can also be a void. Also, the glossary's **Contact** is a blend's
contact curve and explicitly not the boolean's, so the boolean's word stays
*touch* (`MeetKind::Touch`). Proposed entry, written in the plan's ADR
commit:

> **Touching lumps**: Lumps of one solid whose boundaries meet along curves
> where their surfaces touch, or at isolated points, never across or over
> an area; each keeps its own edges and vertices (ADR-00NN). Open CASCADE's
> compound of touching solids.
> _Avoid_: compound, non-manifold solid

## Decision for the human

❓ **Q1 — Touching lumps in one `Solid` (A), with no shared topology, behind an ADR that amends ADR-0006's "two lumps meeting at a point or along a line are a `General` body"; or B, C, D?**
➡️ Yes, A: it builds the 530 programs as the harness reads them, keeps every shell manifold, and turns today's silent B1 gap into a valid body.

Round 2 (Q1 settled as A):

❓ **Q2 — Contacts that share topology** (an edge of four faces, a vertex
two shells reach: `edge-touching-fuse`, `cross-cylinders-cut`,
`pockets-touching-edge-cut-many`, the two frustum rim-point fixtures):
(A) the ADR admits them under the same rule, and a third plan, after the
harness's two, builds them by giving each lump its own copy of the edge or
vertex; or (B) they stay `NonManifold`, and the ADR says so.
➡️ A: one rule for every touch, so the ADR does not have to draw an
arbitrary line. The checker already passes duplicated edges, because two
faces meeting on their own boundaries are no B1 violation. Only the
boolean's grouping by shared edge has to learn the split. The third plan
also has to settle one open point: `weld` would merge the copies'
coincident positions into a non-manifold mesh edge.

❓ **Q3 — The glossary entry**: add **Touching lumps** as worded in *The
words*, in the commit that writes the ADR.
➡️ Yes: the term names what the ADR introduces, and *Avoid* keeps
Open CASCADE's *compound* out of Arris's docs.

## Decision

- **Q1** (2026-11-17): yes — A. Touching lumps in one `Solid`, no shared
  topology, behind an ADR amending ADR-0006; two plans, curve touches
  first, then point touches.
- **Q2** (2026-11-17, delegated to the agent): A. The ADR admits
  shared-topology contacts under the same rule. A third plan, after the
  harness's two, builds them with a copy of the edge or vertex per lump,
  and settles what `weld` does with the copies.
- **Q3** (2026-11-17, delegated to the agent): yes. **Touching lumps** goes
  into `GLOSSARY.md` as worded, in the commit that writes the ADR, because
  the docs lifecycle puts a term with the first doc that uses it.
