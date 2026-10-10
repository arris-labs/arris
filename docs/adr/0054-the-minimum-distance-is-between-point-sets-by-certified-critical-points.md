# ADR-0054 — The minimum distance is between point sets, by certified critical points and a boundary recursion

- Status: accepted (2026-10-10)
- Plan: `minimum-distance` step 2
- Amends: nothing; builds on ADR-0053 (the face hierarchy, the query fixture
  kind, the most specific entity), ADR-0004 (point classification),
  ADR-0013 (layers), ADR-0030 (cancellation)
- Idea: "minimum distance, the next C8 slice" (option C, its four decisions
  delegated to the agent)
- Reference trees read: Open CASCADE's `BRepExtrema` package
  (`BRepExtrema_DistShapeShape`, `BRepExtrema_DistanceSS`) for what is
  reported and the solutions it lists, and its `Extrema` package
  (`Extrema_ExtCC`, `Extrema_ExtPS`, `Extrema_GenExtSS`) for where it
  samples: the generic pairs start Newton from a grid and so promise no
  completeness. Nothing is copied.

## Context

C8's distance asks for the least distance between two operands, each a
point, a vertex, an edge, a face or a body. A consumer wants the feet (to
snap, to dimension, to check a clearance), not only the number, and wants
to know when the answer is not the only one. Open CASCADE's answer is a
list of solutions found from a sampled grid; Arris can do better on its
own geometry, whose kinds are closed under the reductions below.

## Decision

**1. Point-set semantics.** A body operand is its solid, a point set. A
point inside it, or an operand with a vertex `classify_point` finds
`Inside`, is at distance 0 with `contained: true`. A shell is not a point
set and is refused as `QueryReason::NotMeasurable`, as is a body that is
not a solid. Open CASCADE's `InnerSolution` is the same test on vertices,
so a crossing that puts a vertex of one inside the other is `contained`;
the walk adds one more, for an operand with no vertex inside that lies
wholly in the other (a ball in a block): at a positive distance the two
cannot cross, so its closest point classifies it.

**2. One pair per answer.** The pair returned is the least by `(a's entity
id, b's entity id, a's parameters, b's parameters)`, so the answer is
deterministic (ADR-0010). `unique` is false when the minimum came from a
continuum, or when a second candidate within `tol.linear` of it has feet
farther apart than the tolerance. Every equal-distance solution, as Open
CASCADE lists them, is not returned.

**3. Certified critical points, never sampling.** The distance between two
trimmed entities is the least of (a) the stationary pairs of their
*untrimmed* carriers whose feet lie inside both trims and (b) the same
question asked of their boundaries: a face against the other operand's
edges, an edge against its vertices. `arris_geom::extrema` supplies (a):
`extrema_curve_curve`, `extrema_curve_surface`, `extrema_surface_surface`
and the feet of a point, each reporting every stationary pair, isolated or
a continuum.

**4. The reduction table.** Each family comes down to at most two unknowns,
which the certified solver takes (`bernstein2::common_zeros`, Krawczyk, as
ADR-0053 §7 chose for a line against a NURBS surface):

- an offset surface is its base surface at a shifted distance: a sphere is
  its centre, a cylinder its axis, a torus its centre circle (the critical
  points of `|f − r|` away from `f = r` are those of `f`);
- against a plane, the extremum of the height above it;
- against a cone, the signed distance in its meridian plane, a root of a
  squared trigonometric polynomial, the spurious roots removed against the
  unsquared equation;
- against an elliptic cylinder, the projection onto its section plane
  against the ellipse;
- a bounded curve against a circle is one unknown, the circle's
  projection eliminating its parameter (step 1's finding below).

Three surface-interior pairs have no reduction below three unknowns: cone
against cone, cone against an elliptic cylinder, and an elliptic cylinder
against an elliptic cylinder. A NURBS surface against anything but a plane
or a sphere, and a curve against a NURBS surface's interior, belong to the
NURBS cycle. All of them are `GeomError::Unsupported`.

**5. An unsupported pair refuses only if it could matter.** A body walks
`FaceTree::nearest` nearest first with the bound shrinking to the best
distance found. A face pair whose box distance is not below the bound is
skipped, and the unsupported pair refuses (as `OpError::Unsupported`
naming both faces) only if the walk reaches it. A body–body query with cones
on both sides still answers whenever its minimum lies elsewhere. The
alternative, a solver in three or four unknowns, would make the plan's
riskiest step riskier; the pairs are rare (a countersink is a cone against
a plane, a cylinder or a sphere, all covered). A backlog line keeps the
solver.

**6. A continuum needs no interior candidate.** A curve of critical points
is a level set of the distance. Inside the trims it reaches a boundary
because every closed family of the analytic kinds crosses a periodic seam,
which is an edge, so a boundary term attains the same distance. The
argument fails for a closed level set that lies inside both trims and
crosses no seam (a face whose domain is a disk around a pole); the plan's
open question held that; step 7 settled it. A closed level set inside
both trims that crosses no seam needs a face whose boundary is not a seam
and lies off the level set, and the kernel's periodic faces carry their
seams, so the level set of every family of the analytic kinds crosses one.
The representative a continuum returns is also a candidate wherever it lies
inside both trims, which covers a level set wholly inside them. What stays
unseen is a second, separate component at the same distance, inside the
trims and crossing no boundary, while the representative lies on another;
the answer's `unique` is false for the continuum either way. A continuum
makes `unique` false.

**7. Contact is decided by the section.** Before the walk, the face pairs
whose boxes overlap are intersected (`intersect_surfaces`, the section the
boolean uses). A section that crosses a boundary of either face is already a
meeting of an edge with the other face; one lying inside both is a closed
loop or an isolated touching point, and a point of each decides it. That
point inside both domains gives distance 0 there. This is
also what interference will reuse.

**8. The `distance` query form**: `{"distance": {"a": <operand>, "b":
<operand>}}` in the query fixture kind (ADR-0053 §8). An operand is a
point, a recipe step (a body), or an entity of one picked by the point
nearest it, since ids differ between kernels. `expected.py` answers with
`BRepExtrema_DistShapeShape`. The runner compares the distance within the
fixture's `linear` tolerance, the feet to *any* oracle solution, a support
face by centroid and the support kind exactly, and `unique` to "the oracle
has one solution" unless `analytic.query_differs` records why.

**9. What step 1 measured**, which settled the method:

- Two unknowns stalled for 130–400 ms on nearly coaxial circles (a pin in a
  hole), and the isolation returned thousands of near-stationary points
  when quartered. A bounded curve against a circle is therefore one unknown,
  by the circle's projection, in microseconds. Where the isolation gives up
  or cannot certify, the region is searched in one variable along its
  longer side.
- A continuum is read as the tolerance reads it: the distance varies along
  the family by at most `tol.linear` (for a line against a circle's axis,
  `2(δ + R sin θ) ≤ tol.linear`), never as an angle. A family needs no common
  axis: a circle on the tube about another is one.
- A pair is solved in one canonical order of its curves (kind, then their
  defining numbers) and swapped back. Otherwise, in a valley flat to the
  tolerance, the two orders' polishes settled 1e-4 apart.
- A piece's two ends are always candidates: a symmetric pose puts a stationary
  pair on a conic arc's end.
- Release cost per pair: two cubic NURBS curves of four spans 2 ms, degree 5
  at six spans 6 ms, degree 15 38 ms, degree 25 128 ms (`MAX_DEGREE`); any of
  them against a circle under 5 ms; a near-continuum on the two-unknown
  route up to 20 ms.
- Open CASCADE's `Extrema_ExtCC` is incomplete (one pair of four for two
  circles, a family at one distance as points along it), so the fixtures
  hold each oracle pair to one of ours, or to a continuum at its distance,
  and our nearest to its nearest. Completeness is a property test's, against
  a dense grid.

## Consequences

- `QueryReason` gains `NotMeasurable`: a breaking change, announced under
  `### Breaking`, with the binding's error mapping and stub.
- A consumer that wants every equal-distance solution asks again with the
  operand narrowed to an entity.
- `unique: false` is a tolerance reading, not a proof of a continuum.
- Overlapping operands are at distance 0 and `contained` says whether one
  is inside the other; how deep they overlap is interference's to report.

## Alternatives considered

- **Sampling or a mesh**: gives no certificate and loses a narrow minimum,
  the pin in a hole.
- **Every equal-distance solution, as Open CASCADE**: its list depends on
  its sampling, and a continuum is an infinite set; one pair and a flag
  say what a consumer can use.
- **A solver in three or four unknowns for the three refused pairs**: see §5.
- **A shell as its sheet's point set**: a distance to an open shell has no
  inside, and its use is a healing cycle's.
