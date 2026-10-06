# ADR-0052 — An edge's samples are a pure function of the edge; a frustum takes rings

- Status: accepted (2026-10-06)
- Plan: `per-face-tessellation` step 2
- Amends: ADR-0003 ("a plane, a cylinder and a cone are ruled and take
  none"; the edge count read against the face's surface steps over the
  face's box), ADR-0005 (a cone face's interior)
- Idea: ask A9 of the plugin CAD's consumer asks, the roadmap's C7 side
  plan; builds on ADR-0010 (stable ids)

## Context

A consumer that caches a mesh per `FaceId` across edits needs a kept
face's mesh to meet its re-meshed neighbours bit for bit. Today an edge's
sample count is the largest count any of its coedges asks for, and a
coedge asks for `travel / steps`, where `steps` is its face's
`Surface::chord_steps` over that face's whole (u, v) box. For a plane, a
cylinder, an elliptic cylinder, a sphere and a torus the steps ignore the
box. For a cone (the radius at the box's far `v` end) and a NURBS surface
(curvature sampled over the box) they do not, so trimming such a face
changes the samples of every edge it has, including edges a boolean keeps
by id beside an untouched face. `boolean/cone-boss-cut-kept-edge-mesh`
is that failure: the base circle of a conical boss, kept by a cut of its
top, goes from 28 points to 24. The operations themselves keep a kept
edge's pcurves bit for bit (`kept_pcurve_prop.rs`), so the sampling is the
only dependence left.

Reading a coedge's requirement over its own pcurve's box makes the count a
function of the edge. On a cone it also changes what a face's chains look
like: a frustum's narrow circle is sampled for its own radius, a coarser
step than its wide circle's. The plan this ADR comes from estimated the
excess of a triangle standing on the narrow chain with its apex on the
wide one as `δ·max_t (1 + t(k − 1))(1 − t)²`, past `δ` beyond a ratio of
`k = 3`, and proposed interior rings for it. That estimate is
pessimistic: it took the triangle's cross-section at height `t` for a
chord of the cone at that height's radius, whose ends are on the cone.
They are not. The excess is real, but it comes from somewhere else.

## Decision

**1. A coedge's requirement is read along its own pcurve.** Its steps are
the surface's over the pcurve's (u, v) box across the edge's range, not
over the face's box. For a cone that is the radii the edge reaches; for a
NURBS surface the curvature over its whole domain, which is never finer
than any face's and depends on the surface alone. A degenerate edge — a
cone's apex, a sphere's pole — keeps its face's count, per coedge. Its
`EdgeRange` is one index, so its count never reaches a neighbour, and its
row of (u, v) samples is the face's own business (§3).

**2. The bound on a cone.** To second order, a point `Σ λᵢ Pᵢ` of a flat
triangle whose corners lie on a cone of half-angle `α`, at radii `ρᵢ` and
angles `uᵢ`, is off the cone by at most

    (cos α / 2) · Σ λᵢ ρᵢ (uᵢ − ū)²,   ū = Σ λᵢ ρᵢ uᵢ / Σ λᵢ ρᵢ,

the `λρ`-weighted variance of the angles. Over a segment between two
samples, that peaks at `(cos α / 2) · ρ₁ρ₂ Δu² / (√ρ₁ + √ρ₂)²`, a fraction
`1 / (1 + √(ρ₂/ρ₁))` of the way from the narrow end. With each chain
sampled at its own radius's step `hᵢ = √(8δ / (ρᵢ cos α))`, a segment
offset by `(h₁ + h₂) / 2` is exactly `δ` off, whatever the ratio. A
Delaunay circle's chords on two constant-`v` lines are concentric. So a
triangle with its base on one chain has its apex within half the other
chain's step of the base's middle, and two such chains need nothing
between them at any ratio. Frusta of ratio 1.5 to 100 meshed with each
edge at its own radius came out at 0.82 chords at worst, with no ring.

**3. A frustum whose radii span more than `RING_RATIO = 2` takes rings.**
That structure breaks where a chain runs oblique to the ruling. A narrow
chain at its own, coarser step leaves room between its samples for a
circumcircle whose triangle cuts across a wider chain's bulge. A frustum
of ratio 22 drilled across its wall at a slight tilt came out 1.49 chords
off. The fix is face-local Steiner points, like the sphere's lattice:
lines of constant `v` at the radii `ρ_max / 2^i` strictly above the
region's narrow end, each sampled uniformly across the face's `u` box at
its own radius's step and strictly inside it, kept where the loops wind
around them and off every loop segment. They count against
`MAX_INTERIOR_POINTS` with the lattice and are metered as interior points
(ADR-0030). They change no edge, so the mesh stays closed by
construction.

The ratio is the plan's open question, decided by measurement. Over a
thousand random split and drilled frusta of ratio up to 100, with every
edge at its own radius, rings at a ratio of 3 left 1.02 chords. A face of
ratio 3.0 exactly, which takes no ring at 3, left 1.0035. That is the
ribbon's `√(1 + 4/64)` (ADR-0005) on the apex offset, which bounds the
two-chain case at 1.04 at any ratio. At a ratio of 2, two thousand such
frusta stayed within 0.99997 chords. With today's face-wide edge sampling
and rings at 2, six hundred stayed within 0.9987.

**A face reaching its apex takes no ring.** Its degenerate edge is a row
of (u, v) samples that all map to one position. In the flattened ribbon a
ring sparser than that row is no wall. A sample beyond the ring meets the
row on both sides of a ring point, so the apex's fan stops being one
cycle and the mesh opens. Rings opened 113 of 144 drilled cones and 231
of 400 split or drilled ones. Such a face needs none: its narrow end is
the apex, and every segment from the apex is a ruling. With every edge at
its own radius and no ring, 944 random split and drilled cones stayed
closed and within 0.9996 chords.

**4. The weld contract.** Meshes of different faces are joined by edge
id, and the shared runs must agree bit for bit. A mismatch is refused by
name, never snapped: there is no tolerance in a weld (ADR-0003's
positions are exact evaluations, and a snap would make one).

## Consequences

- A frustum's mesh grows. A ratio-10 patch goes from 154 to 264
  positions, a ratio-100 one from 481 to 958, at a chord of 1e-3, while
  edges are still sampled at the face's widest radius. Sampling the
  narrow edges at their own radius gives part of that back. Chamfers,
  countersinks and drafts span ratios under 2 and change nothing.
- The cancellation record (`tests/cancel_counts.txt`) has no cone and is
  unchanged. A cone fixture's step count grows by its ring points.
- `RING_RATIO` is a shape constant with the measurement above behind it,
  not a tolerance. It never reaches a coordinate.
- The probes that hold a cone face to its chord sample each triangle
  densely. The midpoint and centroid miss a segment's worst point between
  two radii, which lies at `1 / (1 + √k)` of the way, not at the middle.

## Alternatives considered

- **No rings**, on the strength of the two-chain identity. Correct for
  every face whose chains run along `v`, and it fails at 1.49 chords on
  the first oblique one.
- **Rings at a ratio of 3**, the largest the plan's estimate allowed:
  fewer points, and 2% over the chord where the ribbon's factor bites.
- **Rings on a cone reaching its apex**, with the run stopped short of
  the apex. Any ring sparser than the apex row opens the fan, even a
  single one at half the base radius.
- **Keeping the face's box for the requirement.** It is not a function
  of the edge, and it is what A9 cannot cache across.
- **Reading a NURBS requirement over the knot-span cells the pcurve
  crosses** rather than the whole domain. Also pure, and cheaper on a
  face trimmed from a large surface. Measured at the step that applied
  §1 (`Surface::chord_steps_along`): the real-part tier at a chord of 1e-3
  came to 977,217 positions in 3.74 s against 977,451 in 3.60 s before,
  so nothing needed the cheaper read.
