# Idea: touching-operands-fuse-to-several-shells

- Status: Open
- Raised: 2026-11-13; consolidated 2026-11-16 from items 1, 2, 4 and 7 of
  the training harness's `fuse-contact-refusals`. Items 3, 5 and 8 went to
  `tangency-faults` and `flat-edge-read-as-tangent-contact`, item 6 to
  `revolve-circle-meets-the-axis`.
- Prompt: none from the human; filed by the training harness (C12,
  ADR-0020's amendment of 2026-11-16) against 0.10.0, each case a fixture on
  its side with a closed form that CadQuery 2.3.0 (Open CASCADE) matches.

## Problem

`fuse` refuses operands that touch only along a line or at a point. Open
CASCADE builds these as one compound of two solids. This is the largest
group the harness filed, about 530 programs:

```python
# 1. Two unit cylinders, height 1, axes on x = 1 and x = -1, touching along
#    a line (159 `cotter_pin` programs). OpDegenerateError.
#    Two shells, V = 2 pi, A = 8 pi.
# 2. A box 4x4x1 on the origin and a cylinder r 1, h 2 on x = 3, tangent to
#    one of the box's faces (61 `hinge` programs). Same error. V = 16 + 2 pi.
# 3. A torus and a box touching at one point (80 `torus_link` programs).
#    OpUnsupportedError "no closed form for +f6 (plane surface) against +v0
#    (point)". V = 6 pi^2 + 8, two shells.
# 4. A sphere r 2 at the origin and a cylinder r 1 on z -5..-2, whose end
#    disc touches the sphere's pole (231 `ball_knob` programs, 153 in the
#    harness's subset). OpUnsupportedError "+f1 (plane surface) against +f3
#    (sphere surface)", or "... against +v0 (point)" where floating point
#    puts the disc a hair off the pole. V = 41 pi / 3, A = 24 pi, two shells.
```

The harness's fixtures: `cq-tangent-cylinders`,
`cq-tangent-cylinder-and-box`, `cq-torus-box-point-contact`,
`cq-sphere-pole-on-disc` (with `-sphere-first`, `-corpus`, `-cylinder`). The
harness reads a body of several shells as a compound.

## Constraints it runs into

- The boolean refuses deliberately where "the result's shells would touch
  along an edge or at a vertex" (a non-manifold result, data-model
  §Invariants). Building these cases means one of two things. Either a body
  may hold several solids whose shells touch, which is an invariant change
  (the checker's shell rows, `common`/`cut` taking such a body back,
  STEP's writer). Or the fuse returns two bodies, which changes the
  operation's signature.
- Cases 3 and 4 also need a point contact between surfaces that have no
  section (plane–torus at a point, plane–sphere at a pole). These are
  closed forms the intersector does not have yet.
- An ADR is needed whichever option is taken: it amends what a body is
  allowed to be.

## Options

### A — A body of several solids touching at a curve or a point
The fuse's result keeps both shells in one body. The checker accepts shells
that touch only on a curve or at a point, and every operation takes such a
body as an operand. This matches what Open CASCADE and the harness read.
Cost: an ADR, the checker's shell rows, the point and line contact
sections, and STEP write and read. About 2 plans.
### B — `fuse` returns the operands unchanged, as two bodies
No invariant change. The result type grows a "touching, not joined" outcome
the caller must handle. That is an API break, and later operations on the
"union" lose their meaning.
### C — A clearer refusal
`OpDegenerateError` names the contact in every case, never `Unsupported`
(cases 3 and 4 today). About 1–2 steps. All 530 programs stay refused, but
the harness can tell the reason apart.
### Do nothing
The largest refused group stays refused, and cases 3 and 4 claim that no
closed form exists where the issue is really a contact.

## Recommendation

A, after `tangency-faults` and `flat-edge-read-as-tangent-contact`, which
fix wrong outcomes rather than refusals. C first, as one step, if A slips
past C12.

## Decision for the human

❓ **Q1 — A body may hold solids touching on a curve or at a point, and `fuse` returns that (A), behind an ADR?**
➡️ Yes: it is what Open CASCADE builds and what the harness reads.

*Later*: the ADR's number and the checker rows it amends, once Q1 is settled.

## Decision
