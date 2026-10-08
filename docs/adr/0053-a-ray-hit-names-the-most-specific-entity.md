# ADR-0053 — A ray hit names the most specific entity; the query corpus is one fixture kind

- Status: accepted (2026-10-09)
- Plan: `ray-fire` step 2
- Amends: nothing; builds on ADR-0004 (point classification), ADR-0013
  (layers), ADR-0018 (touches are decided along the line)
- Idea: "first slice of C8", the query cycle's roadmap entry
- Reference trees read: Open CASCADE's `IntCurvesFace` package
  (`IntCurvesFace_ShapeIntersector`, `IntCurvesFace_Intersector`) for what a
  ray fire reports, and its `BVH` package (`BVH_Tree`, the median builder)
  for the hierarchy's shape. Nothing is copied.

## Context

C8 adds four queries (distance, ray fire, interference, planar section).
They share two things no earlier cycle built: a spatial index over a body's
faces, and a fixture kind that holds a question rather than a recipe's
result. Ray fire is the query nearest to code that exists, since the
classifier already casts a ray at every face of a body, so it goes first and
fixes both. A consumer picks with the ray (a viewport pick, a snap, a sight
line), so what a hit *names* matters as much as where it is.

## Decision

**1. A hit names the most specific entity**: a vertex, else an edge, else
the face, exactly as `Classification::On` does and by the same rule
(`boundary_entity`, each entity within its own tolerance). A ray through a
box's edge is one hit naming the edge, not two hits on its two faces; a
ray through a corner is one hit naming the vertex. Hits are deduplicated
along the entity they share, so a closed body is crossed an even number of
times from outside whatever the ray passes through.

**2. The normal of an edge or vertex hit** is the outward normal of the hit
face with the lowest `FaceId`, and the docs say so. The alternative, no
normal and both faces' on request, was rejected: snapping wants one normal
and a section view wants none, and the lowest id is deterministic
(ADR-0010). The hit also carries the `(u, v)` on that face.

**3. A tangent hit** is a hit with `tangent: true`, at the point of closest
approach, as the intersector reports it (ADR-0018). It counts as a hit for
`All` and `First`, and it never counts as a crossing for a parity.

**4. A ray lying in a face** (in a plane face, or along a cylinder's
ruling) is refused as `QueryReason::RayInFace`, naming the face. The
alternative, the face's entry and exit on its boundary as two hits, would
be a guess about what the consumer meant: a pick ray in a face is measure
zero, and a sight line along a wall wants to know it is along it. Open
CASCADE gives no answer to build on either. For a ruling of a NURBS surface
`GeomAPI_IntCS` returns scattered points of the line and no segment (83
points for the fixture of step 1). A zero direction is
`QueryReason::ZeroDirection`.

**5. A ray from inside a solid** reports its hits from `t = 0` as from
anywhere: `t` is the parameter along the unit direction, the origin is not
a hit unless it is on the boundary (then it is a hit at `t = 0` naming the
entity), and `First` is the smallest `t >= 0`. A hit at a negative `t` is
never reported: a ray is a half-line.

**6. The hierarchy** is `arris_check::tree::FaceTree`, over each face's
`FaceDomain::bounds` (already inflated by the face's tolerance). It lives in
`arris-check`, beside `classify`, because `Classifier` uses it and `check` is
below `ops` (ADR-0013). A face with no bounded box is in every query's
result. The build is a deterministic median split on the longest axis, the
order of the split being the box centre's and ties broken by `FaceId`;
`along` returns the faces a segment enters in ascending entry order, ties by
`FaceId`; `near` returns by `FaceId`. No hash iteration reaches an order.
`Classifier` casts through the tree and its answers do not change.

**7. A line against a NURBS surface is subdivision, not marching**: Bézier
extraction, a hull-box rejection, and the common zeros of two Bernstein
polynomials certified by Krawczyk (`nurbs/line.rs`). The cost of step 1,
one release-build Bézier patch hit by 20 lines, half of them exact grazes,
per line: 11 ms at degree 3, 24 ms at degree 9 at a weight ratio of 4, 18 ms
at 9 and ratio 100, 44 ms at 15 and ratio 10, 133 ms at 25 (`MAX_DEGREE`) at
ratio 10 or 1000. A transversal line alone is 0.1–28 ms over the same range.
The weight ratio does not move it, so no subdivision bound is needed.

**8. The query fixture kind**: `"kind": "query"`, a solid recipe (the
existing interpreter) and a `queries` list; here each query is `{"ray":
{origin, direction}, "hits": "first" | "all"}`, and the distance,
interference and section plans add their own query forms to the same list.
`expected.py` answers with `IntCurvesFace_ShapeIntersector`. A hit is
compared by count, `t`, point and normal within the fixture's tolerance, and
the face it lands on **by the face's centroid, not by id**, since ids differ
between kernels. Where Open CASCADE reports an edge hit twice (once per
face), `analytic.query_differs` records why and holds Arris to one hit (§1),
with the evidence, as `measure_differs` does (ADR-0015).

## Consequences

- `QueryReason` gains `RayInFace` and `ZeroDirection`: a breaking change,
  announced under `### Breaking`, with the binding's error mapping.
- A consumer that wants every face along an edge hit asks the model's
  edge-to-faces adjacency; the hit names the edge.
- The classifier and ray fire share the tree, so a later cached tree on a
  `Model` serves both.
- A fixture's face by centroid cannot tell two faces with one centroid
  apart (two coaxial disks). The corpus avoids that; the lint does not.

## Alternatives considered

- **Both faces as two hits on an edge**: matches Open CASCADE, doubles the
  crossings a parity needs, and leaves the consumer to merge them.
- **A ray in a face as its entry and exit**: see §4.
- **Marching for line × NURBS**: needs a start, loses a hit at a fold, and
  gives no proof that a crossing was not missed.
- **A hierarchy in `arris-ops`**: the classifier below it could not use it.
- **A second fixture kind per query**: four comparators for one shape.
