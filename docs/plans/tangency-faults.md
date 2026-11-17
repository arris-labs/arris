# Plan: tangency-faults

- Started: 2026-11-17
- Milestone: C12, the training harness's refusals (docs/ROADMAP.md), its
  third in-list item
- Idea: docs/ideas/tangency-faults.md (absorbed; option A, decided by the
  agent under the charter, its refusal floor narrowed as the deltas say)
- Second oracle: not needed; every case that builds has a closed form.

## Goal

None of the harness's tangency programs returns an `Ok` body the checker
refuses, or an `OpInternalError`. Five cases build `Full`-green to their
closed forms: a cylinder tangent inside a cylinder, a bar with a boss
tangent to each end, a plate whose arc is tangent inside a cylinder, a bore
cut by a column tangent to it along four lines, and a cylinder whose end
disc is tangent to another's side. A frustum and a box that touch at one
point of its rim are refused `NonManifold` on both sides of the revolve's
seam. Two chamfers that together use up the face between them are refused
`BlendTooLarge`, which names both edges and the face.

## Non-goals

- Building the frustum and box as two shells that touch at a point. That
  belongs to `docs/ideas/touching-operands-fuse-to-several-shells.md`.
- Building overlapping blends as one: no solid has that shape, and Open
  CASCADE refuses it too.
- A near-tangency a hair off within the tolerance (the backlog line about
  surfaces a hair off parallel or tangent). Same-domain face merging.

## Design deltas

**What planning found** (on `d707e68b`, with the harness's own programs):
- Case 1 builds `V = 5π` and drops the tangent ruling from the walls.
  Each outer cap is a crescent, the rim and the inner section circle
  touching at `(0, 1, ±0.5)`. It is written as an outer loop and a hole
  that meet at no vertex (L5). It should be one loop through the touch
  vertex twice, which L5 allows.
- Case 2: one boss fused to the bar is green. The second fuse leaves a
  self-crossing loop on the top and the bottom. The lead is what the first
  fuse leaves at `(15, 0, ±0.5)`.
- Cases 3 and 4 fail with `SplitFault::Dangling`: a ruling where a face
  touches a wall from inside ends at a node nothing else reaches. Bores of
  0.45 and 0.55 build.
- Case 5 fails with `SplitFault::Turn` on the end disc, which touches the
  wall along `x = 6, y = 0, z ∈ [2, 3]`. A longer cylinder builds.
- Case 6: at `+x` the touch is the rim's seam vertex, refused
  `NonManifold`. At `−x` it lies inside the rim edge, and the cap's
  arrangement places the coplanar box face as a hole of no region.
- Case 7: the contact circles fall at radii 7.5 and 8.5 on the 4-wide face
  and swap order. The chamfer returns a body that `Full` refuses (L4, S5),
  and the cut then faults on it.

**The rule each build restores** (ADR-0004, ADR-0050): a touch between a
section and a face's boundary, or between two sections, is a vertex of
both. A touching ruling with one operand's material on both its sides is
no section on either face. No ADR is planned. A step that changes which
side owns a tangent line or a touch vertex writes ADR-0088.

**The refusal floor, narrowed.** Option B first would need one of two
things. One is `Full`'s L5 after every release boolean, which costs every
caller. The other is remapping `SplitFault`s to a modelled reason, which
hides kernel bugs behind a name (`.agents/rules/kernel.md`). So the builds
go first, worst outcome first: the invalid `Ok` bodies, cases 1–2. Typed
refusals are only where a refusal is the right answer: case 6 at `−x`, and
case 7.

**Case 7 reuses `BlendReason::TooLarge`** ("a blend does not fit its
faces"), so the binding is unchanged. Its doc gains: "or two blends'
contacts on one face cross or swap order; the error's entities are both
edges and the face". The check runs before any geometry is cut, pairwise
over each face's contacts in (u, v). This doc comment is the plan's only
public API change.

## Steps

Grades: **[1]** routine; **[2]** careful, a geometric or numeric case
within a given design; **[3]** unproven, an algorithm whose robustness or
bound is established here.

- [x] Step 1 **[2]** — Nine `tests/fixtures/regression/` fixtures, each
  with Open CASCADE's values and its closed form in the note:
  - `fuse-cylinder-tangent-inside` (V = 5π, A = 14π);
  - `fuse-bar-end-bosses-tangent` (V = 154.50155561268);
  - `fuse-arc-plate-tangent-inside-cylinder` (V = 2π + (0.5 + π/8)/4);
  - `cut-bore-tangent-column` (V = 184, A = 238, genus 1);
  - `fuse-end-disc-tangent-to-wall` (V = 15π − 8/3, A = 33π − 8);
  - `fuse-frustum-box-rim-point` and `-far-side` (`NonManifold`);
  - `chamfer-two-rims-use-up-face` and `-then-cut` (`BlendTooLarge`).

  The near side already passes and goes straight into `boolean/`.
- [ ] Step 2 **[3]** — Case 1. The section circle's touch on the rim
  becomes a vertex of both, and the crescent becomes one loop through that
  vertex twice. The fixture moves to `boolean/`, `Full`-green. The risk is
  finding the touch: it is a double root, and no crossing marks it.
- [ ] Step 3 **[3]** — Case 2. Find what the first fuse leaves at the
  tangent point that the second breaks, and fix it by step 2's rule. Both
  orders and `fuse_many` give the closed form, and the fixture moves green.
- [ ] Step 4 **[3]** — Cases 3 and 4, the dangling ruling. Apply the
  material-side rule on both faces. Both fixtures move green. Bores of
  0.45, 0.5 and 0.55 each match their closed forms in a unit test.
- [ ] Step 5 **[3]** — Case 5. The end disc's arrangement turns once and
  the fixture moves green. Lengths 12, 12.5 and 14 match their closed
  forms.
- [ ] Step 6 **[2]** — Case 6's far side. A coplanar touch at a point
  inside an edge becomes the vertex contact the seam side already finds,
  refused `NonManifold` with that vertex named. Both sides and a lug turned
  90° about the axis refuse alike.
- [ ] Step 7 **[2]** — Case 7. The crossing-contacts check refuses
  `BlendTooLarge` for chamfers and fillets, on either pair of rims. A
  chamfer of 1.9 on both rims, or of 3.9 on one rim, still builds green.
  Both fixtures move to `blend/`, and `TooLarge`'s doc is extended.
- [ ] Step 8 **[2]** — Property tests (`arris-debug/src/prop`), each in a
  random rigid pose: a cylinder tangent inside a larger one, a bar end
  tangent to a boss, and a column tangent to a bore. `fuse`, `cut` and
  `common` match the closed form `Full`-green or refuse by a modelled
  reason, never `Internal`. `prop::body::standing` takes back the tangent
  draws it leaves out (backlog, flat-edge evidence line). Each fault found
  becomes a regression fixture and a step.
- [ ] Step 9 **[1]** — `crates/arris-py/tests/test_tangency.py` runs the
  harness's nine programs through the binding. Each matches its closed
  form or refusal kind, and every `Ok` body passes `check("full")`.

## Acceptance

- The nine fixtures pass in their areas. Cases 1–5 match Open CASCADE's
  values within tolerance and are `Full`-green. Both sides of case 6 are
  `NonManifold`, and both forms of case 7 are `BlendTooLarge`.
- Step 8's properties pass at 16 cases (fast profile) and at 256 (full
  gate).
- Every fixture that expects `TangentContact` or `NonManifold` still
  does, in the full gate.
- The script corpus and the real-part histogram are re-run and printed,
  with no row above C11's close and no panic.
- `tools/py-test.sh tests/test_tangency.py` and the full gate are green on
  the plan's tip.

## Docs to update on completion

- `docs/ROADMAP.md` §C12: the tangency-faults line is **Done**, with what
  builds and the two refusals.
- `docs/DATA-MODEL.md` §the boolean's face split: a touch is a vertex of
  both curves, a crescent is one loop through it twice, and the
  material-side rule for a touching ruling.
- `docs/ideas/touching-operands-fuse-to-several-shells.md`: the frustum
  and box at a rim point join its cases, and its `tangency-faults`
  reference reads as done.
- `docs/BACKLOG.md`: the flat-edge evidence line loses its tangent clause.
  Any finding from step 8 that this plan does not fix goes here.
- `CHANGELOG.md` `Unreleased`; `AGENTS.md` current state and **Next**.
- `GLOSSARY.md`: no new term (*touch* holds).

## Open questions

- ⚠ OPEN 1: at a touching ruling, drop it from both faces or enter it on
  both. The agent decides at step 4, keeping whichever leaves the walls
  `Full`-green. The pick goes in the commit, and in ADR-0088 if ownership
  changes.
- ⚠ OPEN 2: step 8 may find a family no step covers. The agent adds steps
  up to the ~10 limit; past that, one backlog line with its fixtures.
