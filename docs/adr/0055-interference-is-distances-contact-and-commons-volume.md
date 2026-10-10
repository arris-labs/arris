# ADR-0055 — Interference is distance's contact and common's volume, on a scratch clone

- Status: accepted (2026-10-11)
- Plan: `interference` step 2
- Amends: nothing; builds on ADR-0054 (distance, point-set semantics),
  ADR-0004 and ADR-0050 (the pave model and the selection table),
  ADR-0030 (cancellation), ADR-0053 (the query fixture kind)
- Idea: "interference" (option A plus D's pair loop; its decisions
  delegated to the agent)
- Reference trees read: none. Open CASCADE is the test oracle only
  (`BRepExtrema_DistShapeShape`, `BRepAlgoAPI_Common`, `GProp`).

## Context

A consumer checking a fit or an assembly asks one question of two solids:
do they interfere, and how. The kernel already holds both halves of the
answer — `distance` knows when two solids meet, `common` knows the shared
volume — and a third algorithm would be a third place for them to disagree.

## Decision

**1. Three verdicts, one rule.** `interference(a, b)` is `Apart(Distance)`
when `distance(a, b)` exceeds the model's linear tolerance,
`Touching { contact, at }` when the solids meet and share no volume, and
`Overlapping { volume, a_in_b, b_in_a }` when they share volume. "Not
apart" is "distance within the linear tolerance" — the same threshold the
boolean uses for a piece lying on the other operand — so the identity with
`distance` holds as an inequality at the tolerance, not as `== 0`.

**2. Touching versus overlapping is read as `common` reads it.** The
common is built; `BooleanReason::Empty` (nothing survives) is touching at
a point or a curve, `InputReason::ZeroThickness` (what was dropped lay on
the other operand) is touching along a face. A sliver thinner than the
tolerance is therefore zero thickness, by the boolean's own rule, and any
other refusal of `common` (`Unsupported` for a NURBS face meeting the other
solid, `TangentContact`, `NonManifold`) is interference's refusal too.
Nothing new is decided about geometry the boolean refuses.

**3. The volume is `common`'s.** `mass_properties` of the common body,
so the identity "volume equals the common's, bit for bit" is a property
test, not a tolerance.

**4. Containment from the selection.** The build records, per operand,
whether any piece of its boundary lay outside the other operand (a piece
classified outside, or on a face whose normal opposes). An operand with no
such piece is inside the other: `a_in_b` / `b_in_a`. Two equal solids are
each inside the other, so `Contains` is a flag of `Overlapping`, not a
fourth variant. A ball in a block with no vertex inside it needs nothing
special: its pieces are all inside.

**5. A scratch clone, never the model.** The common is built on
`Model::clone`, dropped afterwards, so the query takes `&Model` like the
other queries and the model is never written. Measured in release on a
406-face plate (400 holes) against a press-fit pin: the clone 630 ns (it
copies a list of chunk pointers), the pave and common 2.5 s. Copying the
two bodies' closures into a fresh model would save nothing.

**6. The contact dimension** comes from the pave model already built, for
the `Empty` case only (a `ZeroThickness` is a face): a section edge, a
tangent contact block, a common edge block or an edge image is a curve; none
of them is a point. `at` is the closest point `distance` returned, which
`distance` already chooses deterministically (ADR-0054 §2) and which lies
on both solids within the tolerance.

**7. Many bodies.** `interference_many` boxes every body (the union of its
faces' boxes, grown by the linear tolerance), runs `interference` on every
pair whose boxes overlap, and returns the pairs
that are not apart in ascending pair order. No verdict-only screen that
skips the common: telling touching from overlapping without it needs a
transversality test on the section, a backlog line until a measured clash
loop asks for it. The boxes are tested pair by pair, not through a
hierarchy: a box test is nanoseconds against tens of milliseconds for the
cheapest candidate pair, so a tree would save nothing until an assembly has
thousands of bodies. A body given twice is a solid with itself: inside
itself, its own volume (the pave model of a body with itself is
meaningless, so `interference` answers it without one).

**8. The query form.** `query` fixtures gain `{"interference": {"a", "b"},
"expect": "apart" | "touching" | "overlapping"}`, held to Open CASCADE.

## Consequences

- Measured in release on a 20-body assembly (ten cubes and ten cylinders in
  a 5 × 4 grid, neighbours and diagonals overlapping): 43 clashes in 2.6 s,
  about 60 ms per overlapping pair, the apart pairs free. A clash loop over
  an assembly of that size is interactive-slow but usable, so the
  verdict-only screen stays a backlog line.

- No new error variant: the refusals are `distance`'s and `common`'s.
- The cost of a clash is a distance plus a full pave and common; the
  volume without assembling the common (idea option B) is a backlog line.
- `result::boolean_each_witnessed` and `boolean::common_probe` are
  crate-private additions to the boolean; the public booleans are
  unchanged.
