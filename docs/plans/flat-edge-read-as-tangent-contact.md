# Plan: flat-edge-read-as-tangent-contact

- Started: 2026-11-16
- Milestone: C12, the training harness's refusals (docs/ROADMAP.md), its
  second in-list item
- Idea: docs/ideas/flat-edge-read-as-tangent-contact.md (absorbed; option A,
  decided by the agent under the charter: fix the classification, leave the
  topology alone)

## Goal

The four refusals the harness filed build checker-green, matching their
closed forms and Open CASCADE: a chained fuse on a shared end plane
(cylinders and the L of boxes), a third slot across a cylinder,
`interference` of a body against the same solid split another way, and a
bore cut over a bore of its width as two tools of one cut. The face
arrangement never sees one curve twice, and when it would, the result is
a named kernel fault, never `TangentContact`. That refusal is again said
only of two faces that touch along a curve.

## Non-goals

- Same-domain face merging (C12's out list): coplanar pieces stay split.
- The tangency-faults idea's cases (`docs/ideas/tangency-faults.md`), the
  next item on the C12 list.
- Changing the domain of `TangentContact` at `result.rs`'s contact check.
- The fused `cboreHole` tool (a bore fused with its counterbore): it
  already cuts to the closed form at the venv's build.

## Design deltas

**What planning found.** Every case was reproduced with Arris alone. Each
refusal comes from `boolean/pieces.rs` (`Arrangement::sorted`): two
half-edges leave one node at the same angle and curvature. The errors name
three entities, which is that site's shape, and the faces they name are two
coplanar pieces of one operand.
- *Cases 1–3, one cause.* `pave/coincident.rs` (`Build::coincident`) adds
  an image for each (pair, edge, block). An edge shared by two coplanar
  faces of one operand (the a/b edge on the fused end plane, a slot floor
  split by an earlier slot, the box's halves) lies inside one face of the
  other operand. That face is coincident with both pieces, so it makes two
  pairs, and the edge is imaged on it once per pair. `edges_by_face` then
  enters it twice. The `along` loop of the same function already holds an
  image once per (edge, block, face). The first loop has no such check.
- *Case 4, a second source.* The fused tool cuts. The two-tool cut, with the
  tools' tops at z = 0 inside the part, is refused with three entities.
  The guess, which step 4 tests: one curve on the hole wall is entered
  twice, once as the bore tool's rim image (its wall is coincident with
  the hole wall) and once as the counterbore top's section edge. `along`
  catches an edge that lies along a section of its own pair. Here the edge
  belongs to a third operand.
- *The idea was wrong on one point.* `interference` of a body split at
  x = 1 against the same box split at y = 0.7 is refused too. Only a body
  against itself answers.

**Decomposition** (`arris-ops::boolean`, no public change):
- An edge piece is placed on a face once, whichever pair meets it first.
  One helper, `held(images, edge, index, face)`, serves both loops of
  `coincident`.
- A section block that lies along a piece of an operand edge already
  placed on that face is that piece, as `along` does inside a pair, for
  every operand of an n-ary boolean (step 4).
- Guard: in `Arrangement::sorted`, a full tie between two half-edges with
  the same two end nodes is one curve entered twice. It is
  `OpError::Internal(Fault::Invariant { what: "one curve entered twice on
  a face" })` and names both edges. A tie whose ends differ is still the
  contact. No new error variant, so the binding and its stubs are
  unchanged.

**No ADR.** It restores what ADR-0004 and ADR-0050 already require ("every
face is split once by every section on it"). Step 4 writes ADR-0087 only
if its fix changes which operand owns a shared piece.

**New term:** *Image*, defined in `GLOSSARY.md` under Operations.

## Steps

Grades: **[1]** routine; **[2]** careful, a geometric or numeric case
within a given design; **[3]** unproven, an algorithm whose robustness or
bound is established here.

- [x] Step 1 **[2]** — The five cases as `tests/fixtures/regression/`
  fixtures, each `#[ignore]`d with the desired `ok`, Open CASCADE's values
  from `tools/oracle/`, and the closed form in the note.
  `chained-fuse-coplanar-cylinders` (V = 3π/2 + √3, A = 6π + 2√3).
  `chained-fuse-coplanar-boxes-l` (V = 8, A = 28).
  `three-slots-through-cylinder` (the harness's closed form, genus 5).
  `interference-box-against-its-halves` (whole~halves and halves~thirds,
  overlap 4). `cut-many-bore-over-bore-of-its-width` (V = 320 − 1.375π).
- [x] Step 2 **[3]** — An image held once per (edge, block, face) in both
  of `coincident`'s loops. The cylinder chain, the box L and the slots
  build checker-green to their oracle values and move into `boolean/` with
  blessed dumps. The risk is the flush selection of a face that an image
  splits on one operand and an operand's own edge splits on the other.
- [x] Step 3 **[2]** — The interference fixture moves into `query/`
  (as `query/interference-box-against-its-halves`; `interference-parts` is
  another recipe) green, both pairs at overlap 4 and both containment flags.
  Its cause was a second one, found here: `same_section` read two edges of
  coplanar pieces as one curve when one's midpoint lay on the other's line
  (a seam crossing another at its midpoint, thirds against halves), so no
  crossing was recorded and `matching_block` found a half-overlap
  (`Fault::CommonBlock`). Two faces with no section (a face beside the edge
  lies on the other face's surface) are no section.
- [x] Step 4 **[3]** — The bore over a bore: find the second duplicate,
  extend `along` across operands as the deltas say, and move the fixture
  to `boolean/` green. Cutting the bore and the counterbore as two tools
  and as one fused tool gives the same measures. The guess held: the
  counterbore top × hole wall section (a circle at z = 0) lies along the
  bore's rim, imaged on both faces through the bore's own faces; the
  section edge beside the images doubled the curve on each.
  `along_third` drops such a block when the third operand's edge lies on
  both faces' surfaces and each face has a pair with one of its faces,
  placing the piece there through `along` (held once).
- [ ] Step 5 **[2]** — The guard in `Arrangement::sorted`, with a unit
  test that enters one section twice on a face and expects the fault
  named. Rerun every fixture that expects `TangentContact`; none may flip.
- [ ] Step 6 **[2]** — Property test (`arris-debug/src/prop`): n random
  boxes or cylinders standing on one plane, n in 3..5. The chained fuse
  equals `fuse_many` in volume and area or both refuse alike. A random box
  split by a random plane and fused back has `interference` with the
  whole box equal to its volume.
- [ ] Step 7 **[1]** — `crates/arris-py/tests/test_coplanar_chain.py`
  runs the harness's four programs as the binding sees them: the three
  cylinders' chain, three slots, the interference of the box against its
  halves, and `cut` with a bore and a counterbore. Each matches its closed
  form.

## Acceptance

- The five fixtures pass in their areas with Open CASCADE's values: the
  `regression/` lint shows none of them left.
- Every fixture that expected `TangentContact` before this plan still
  expects it and passes: `rg -l '"tangent-contact"' tests/fixtures` lists
  the same 14 files before and after.
- The step 6 property passes at 1000 cases in CI. The full gate is green.
- Rerun and print the histogram and the script corpus. No row may be above
  C11's close and nothing may panic.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Operations, the coincident-pair paragraph: an
  edge piece is placed on a face once, whichever pair meets it, and a
  section block along an edge already placed there, of any operand, is
  that edge.
- `docs/ARCHITECTURE.md` §Operations, the arrangement: a tie with the same
  ends is the invariant fault, not the contact.
- `GLOSSARY.md`: **Image**.
- `docs/ROADMAP.md` C12: the in-list line marked done with its counts.
- `CHANGELOG.md` `Unreleased`: chained fuses on a shared plane, slots
  across slots, and cuts over an existing bore now build; `interference`
  of differently split bodies answers.
- `AGENTS.md` current state, the **Next** line: the tangency-faults idea.

## Open questions

- Settled (step 4): case 4's cause is the cross-operand `along` gap, as
  guessed; the section block along a third operand's edge imaged on both
  faces is that edge. The edge keeps its own operand, so no ADR-0087.
- Decided (agent): the guard is a `Fault::Invariant`, an `OpInternalError`
  in Python. C12 forbids that error for harness programs, but the guard
  fires only on a kernel bug. Reporting such a bug as a contact sent four
  filings down the wrong trail.
