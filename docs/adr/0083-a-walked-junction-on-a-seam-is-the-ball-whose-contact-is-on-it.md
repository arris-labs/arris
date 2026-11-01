# ADR-0083 — A walked junction on a seam is the ball whose contact is on it

- Status: accepted (2026-11-01)
- Plan: `walked-seam-junction` step 1 (the decision, against the oracle)
- Amends: ADR-0062 step 9 (the slanted seam), ADR-0072 (the off-vertex junction)
- Reference trees read: none; Open CASCADE run as the oracle

## Context

A walked chain through a vertex `v` that lies on a seam of a curved face
`F` ends each run on the ball at `v` (ADR-0062 §4). The ball is in `v`'s
normal plane, and there the contact on `F` is on the seam only where the
loop crosses the seam square. At a slant the contact is off the seam by
`r·cot θ`, and `ends::on_corner_edge` refuses the vertex as `VertexBlend`
(ADR-0062 step 9; it was a `kernel bug` before).

Three fixtures reproduce it, and Open CASCADE builds all of them (`expected.json`,
vertices / edges / faces):

| fixture | seam vertices that refuse | crossing | Open CASCADE |
|---|---|---|---|
| `regression/sphere-cylinder-oblique-seam-fillet` | the ball's two (`v4`, `v5`) | 33° from the seam plane | 9 / 15 / 8 |
| `regression/sphere-cylinder-oblique-seam-chamfer` | the ball's two | the same | 9 / 15 / 8 |
| `regression/sphere-cylinder-pin-seam-slant-fillet` | the pin's one (`v6`), the ball's two cross square | 80° from the ruling | 9 / 15 / 8 |

In each the loop has three vertices, so three junctions. Before the blend
the body has 6 / 9 / 5. Each junction contributes two vertices (its two
contacts) and the three vertices of the loop go, so the three untouched
ones (`v0`, `v1`, `v3`) plus 6 are the 9. A junction that kept `v` and added
a vertex where the contact crosses the seam would count 11 at least. The
pin's seam, crossed 10° off square, refuses the same way as the ball's.

## Decision

**1. The site.** A tangent vertex `v` of a walked chain whose two runs share
both faces `F1` and `F2`, every other edge at `v` a seam of `F1` or `F2`
used twice by its face. The two runs trace one offsets' section, their
spines (each carried `REACH_RADII` past its edge's ends, ADR-0062 §1)
holding the same balls to the tolerance.

**2. The junction is the ball on the spine whose contact on the seamed face
is on its seam.** `B(t*)`, with `t*` the root of the contact's signed
distance to the seam's isocurve in the face's `(u, v)`, bracketed within the
reach on either side of `v`, then bisection and Newton as `reach_past` is
found. A square crossing gives back the normal plane's ball, `t* = t(v)`.
When both faces are seamed at `v` the two roots must agree within the
tolerance, else `VertexBlend`.

**3. The construction is ADR-0072's off-vertex junction.** Both runs end at
`B(t*)`; one skin is cut short of `v`, the other past it on its spine's
carried-on stretch. The arc (the chord, for a chamfer) is `B(t*)`'s, from
contact to contact; each seam is cut at its contact. `v` is deleted, with
the chain edge's piece between `v` and `t*`.

**4. Provenance.** `v` and that piece are `Deleted`; the arc and its two
vertices `Generated` from both edges; each seam, `F1` and `F2`, `Modified`.

**5. Refusals.** No root in reach, or a root past the other run's far end:
`BlendTooLarge` naming the edge and the seam. A seam vertex that also carries
another blended edge, a non-seam extra or a kink stays `VertexBlend` or
`TangentChain`. A table chain's seam vertex is unchanged. `Reason` gains no
variant.

## Consequences

- The three fixtures move to `blend/` at the counts above and their measures.
- ADR-0062 step 9 stays as written; this supersedes its refusal.
- The root is a numeric search on a spine already held, no new literal.
