# Idea: CAD-Recode sketch prism contacts

- Status: Open
- Raised: 2026-11-21 in arris-gym's `lift-sketches` step 7 (ARRIS-GYM ADR-0023).
- Pin: arris 0.11.1, CAD-Recode validation revision
  `17d554bbc50a23de6cf6a742a3e0ed867f000153` (CC-BY-NC-4.0).

## Problem

The arris-gym sketch lift reaches all 968 validation programs. On the
pinned wheel 75 produce typed refusals and one returns a body that the
full checker rejects. The oracle builds each. In `arris-gym` at commit
`90051d8c7f3f08ebb36f6a6a1c92ac60a042d84e`, the generated report
`tests/fixtures/c2/cadrecode/arris-report.md` keeps the refusal messages
and the checker row's program hash. The reductions below include a desired
result by construction and a report for the exact fixture on CadQuery 2.3.0
and Open CASCADE 7.9.3.0. Fixture names in the table refer to directories
under `tests/fixtures/regression/` in the same `arris-gym` commit.

| Rows | What the wheel reports | Reproducer |
|---:|---|---|
| 36 | Faces touch along a curve interior to both result faces | Stem `119` reduces to a circular cut tangent to the bottom edge (`cq-sketch-tangent-cut-from-bottom`). |
| 15 | Result shells would touch along an edge or at a vertex | The `cq-sketch-corner-contact` fixture holds the contact class. Stem `14` first refuses at its second union. Determine which of the 15 inputs have two shells with only an edge or a point in common. |
| 11 | Boolean builder finds a one-coedge edge | Stem `11` reduces to a circular cut tangent to the left edge (`cq-sketch-tangent-cut-from-left`). |
| 6 | Face arrangement has a dangling section edge | Stem `236` reduces to a slanted prism fused with a taller box (`cq-sketch-fuse-at-profile-vertex`). |
| 7 | Profile loop crosses itself | Stem `226` reduces to a notched circular segment (`cq-sketch-assemble-notched-arc`). The line the checker names lies within the disc and is disjoint from its arc. |
| 1 | Full checker reports intersecting cap loops | Stem `239` reduces to a circular post on a cap (`cq-sketch-post-tangent-to-cap`). |

The counts group the recorded outcomes by message. They do not establish
that each message has a single cause.

The post case reproduces directly with the wheel:

```python
import arris
from arris.build import Workplane

m = arris.Model()
xy = Workplane.XY()
base = xy.rect(31, 16, center=(-7.5, 5)).extrude(-25, model=m)
post = xy.circle(3, center=(-7, 0)).extrude(19, model=m)
fused = base.fuse(post)
print([str(v) for v in m.check(fused.body, level="full").violations])
# ['L5 f9: loops 0 and 1 intersect']
arris.Model().read_body(m.write_body(fused.body, fused.provenance))
# BodyRejectedError: the body read is not valid:
# L5 f7: loops 0 and 1 intersect
```

The post footprint is tangent at `(-7, -3, 0)`. The desired volume is
`31 * 16 * 25 + 19 * 9 * pi`, and the post's side wall is the net addition
to the box's surface area.

## Constraints

Keep the full checker and the manifold solid rule. A genuine shell
contact may be a refusal. The examples give the geometry and measures
to check when triaging each operation.

## Options

### A — Start with the reduced cases

Reproduce the tangent post, the two tangent cuts, the prism join and the
notched arc in the kernel's fixtures. The post has a direct wheel
reproducer and returns a body that fails the full checker. Triage it first.
Split the other cases by the actual cause after reduction.

### B — Defer the cases

The corpus retains the typed refusals and the checker failure. They
continue to count in its all-program figures.

## Recommendation

A. Re-run the five reductions on the next wheel. Use the full checker,
the expected measures and the original corpus rows to verify each fix.

## Decision for the human

Start with the reduced cases and triage the tangent post first (A),
or defer this group of failures (B)?

## Decision
