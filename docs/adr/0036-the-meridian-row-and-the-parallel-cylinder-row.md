# ADR-0036 — The meridian row and the parallel-cylinder row: a coaxial pair along a parallel blends to a torus found in the meridian half-plane, two parallel cylinders along a ruling to a cylinder found in the cross-section

- Status: accepted (2026-10-02)
- Plan: `c6-blend-pairs` step 3; the cone against a plane along a circle
  lands in step 4, the row's open arcs, chains and the cone against a
  cylinder in step 5, the sphere's and the torus's parallels in step 6,
  the refusals in step 7; the parallel cylinders are decided here and
  built when a census ranks them
- Amends: ADR-0007 §"One stripe per edge" (the table gains two rows) and
  §"Everything outside the table" (what `Unsupported` still names)
- Follows: ADR-0007 (closed-form stripes, nothing fitted but a pcurve),
  ADR-0035 (chains, junctions on the ball's cross-section, an open arc
  trimmed on its meridian)

## Context

The plan's step 1 counted the fetched tier's blendable edges, each
filleted alone at a tenth of the narrowest bound, and this step rewrote
that throwaway probe to say, for every `Unsupported`, which faces it
names and how they sit: coaxial, a plane through the axis, parallel or
crossing axes — and, where it names an edge and the face across its end,
that it is an *end* the table cannot trim rather than a pair it cannot
blend. Over the 18 parts of the fetched tier that read, 9567 edges:

| Outcome | Edges | Parts |
|---|---:|---:|
| built, checker green at `Full` (2248), or built on a part whose own reading already fails S5 (684, 4 parts, `regression/nist-*-s5`) | 2932 | 16 |
| `TangentChain` (tangent dihedrals, which Open CASCADE refuses too, and ends at tangent corners) | 3325 | 18 |
| `Unsupported`, a pair | 1269 | 18 |
| `Unsupported`, an end | 1093 | 14 |
| `BlendTooLarge` | 670 | 18 |
| `VertexBlend` | 276 | 16 |
| `Internal` (`regression/edge-off-its-plane-fillet`) | 2 | 1 |

The pairs, by how the two faces sit:

| Family | Edges | Parts |
|---|---:|---:|
| a cone against a coaxial cylinder or a plane square to its axis, along a parallel (arc 274, circle 121, B-spline-written 16) | 411 | 15 |
| a sphere or a torus against a coaxial plane, cylinder or cone, along a parallel | 218 | 6 |
| two cylinders with crossing axes (a quartic edge) | 186 | 8 |
| a NURBS face against anything | 250 | 6 |
| a cylinder against a torus off its axis | 82 | 4 |
| a plane through a cone's apex, along a ruling | 64 | 2 |
| two cylinders with parallel axes, along a ruling | 29 | 5 |

And the ends, each an edge of a pair the table builds whose end the face
across cannot trim in closed form: an open arc's torus on a plane
parallel to the axis and off it (321 edges, 12 parts — a spiric section),
a ruling stripe on a cylinder or a cone across whose axis is square to it
(422, 14 — a quartic), and an edge written as a B-spline where its faces
meet in a circle or a line (287, 5).

Read in the reference trees. Open CASCADE's known parts
(`ChFiKPart_ComputeData.cxx`) cover the plane against a plane, a
cylinder and a cone, and nothing else: `ChFiKPart_ComputeData_FilPlnCon`
works in the half-plane through the cone's axis, takes the angle between
the plane's trace and the cone's ruling, sets the centre circle back from
the edge by `r / tan(θ/2)` along each trace, and builds a torus — or a
sphere where the centre circle closes on the axis; `ChPlnCon` is its
chamfer. A cone against a cylinder and two cylinders go through its
general walker, and the oracle shows it: the turned shoulder's blend
(`regression/turned-shoulder-fillet`) is a B-spline surface whose
distance from the exact torus reaches 8e-6, its volume 2.7e-8 relative
under the Pappus form; the ogive bar's chamfer is a B-spline standing in
for a plane. The plane against a cone is a torus to 1e-16, and its four
fixtures match their Pappus forms to 1e-9. The same oracle settles how a
chamfer's distance is measured on a curved face: on the ogive at `d =
0.2` each contact is 0.2000000 from the edge in a straight line and
0.2000834 along the arc, so the distance is the chord.

## Decision

**1. The meridian row: two faces of revolution about one axis, along a
parallel.** A circular edge where two faces meet, each a plane square to
the circle's axis, a cylinder, a cone, a sphere centred on the axis or a torus, every axis
the circle's within the tolerances (ADR-0008's *coaxial*), blends by its
section. In the half-plane bounded by the axis through any point of the
edge each face is a curve, its *meridian*: a line square to the axis for
the plane, parallel to it for the cylinder, through the apex at the
half-angle for the cone, a circle for the sphere and the torus. The
ball's centre is where the two meridians, offset by `r` toward the ball's
side, cross — the root the edge's own point is the limit of as `r` goes
to `0`, as `ruling_ball` picks it — and each contact is the foot of the
centre on its meridian. Swept about the axis: a fillet is the torus of
the centre's distance from the axis as major radius and `r` as minor,
its frame on the axis level with the centre; each contact is a parallel
of its face, a line at constant `v` on a cylinder and a cone (a cone's
`v` is its ruling's length, so the contact sits at the edge's `v` plus or
minus `r cot(θ/2)`, `θ` the corner's angle in the section) and at
constant `v` on the torus. The plane against a cylinder along a circle,
ADR-0007's ring, is the case of a line square and a line parallel, and
keeps its frames, parameters and ids exactly: the generalisation moves no
blessed dump.

**2. A chamfer takes the chord.** Its two contacts are where the circle
of radius `d` about the edge's point meets each meridian in the section,
on the face's side — along a line meridian that is `d` along it, which is
ADR-0007's rule on planes and on the ring; on a circle meridian it is the
chord, as Open CASCADE measures it. The chamfer is the surface the chord
sweeps about the axis: a cone of the chord's angle to the axis, its `Z`
toward the wider contact as the ring's 45° cone is, or a cylinder or a
plane annulus when the chord is parallel or square to the axis within
`tol.angular` — which two lines of the row make only when they are
tangent, but two cones or a cone and a torus can.

**3. The cone's bounds are read, not assumed.** The data model's cone
has `α ∈ (0, π/2)` and widens along its `Z`; which side of it the
material is on is the face's orientation, as for every face, so no sign
of `α` needs choosing and no half-angle in that interval is refused for
its own sake: the construction sees only the corner's angle `θ ∈ (0, π)`
in the section, and a tangent corner is `TangentChain` before it. What
bounds a blend is where it lands: a centre circle whose radius is not
above `r` by the tolerance — a horn or spindle torus, or Open CASCADE's
sphere where it closes on the axis — is `BlendTooLarge` naming the edge
and the face whose contact runs inward, as the ring's is today
(ADR-0035's amendment); a contact at a radius within the tolerance of
the axis, or past the cone's apex, or outside its face by
`FaceDomain::side`, is `BlendTooLarge` naming that face. A cone face's
apex is a degenerate edge of its loop and is never a corner: an edge
ending there is not a blend this row builds.

**4. Seams, ends and junctions are the ring's.** A closed edge has one
vertex, on each curved face's seam through it — the cone's, the
cylinder's, or both where two seams meet it, all in the one half-plane at
the vertex's angle — and each seam is shortened to its own contact; the
blend's `u` seam is its section in that half-plane, from one contact's
vertex to the other's. A vertex with any other edge is `VertexBlend`. An
open arc of the row is ADR-0035 §4 with "the cylinder" read as either
face of revolution: each end a junction, or trimmed by a plane through
the axis on the torus's meridian or the chamfer's ruling there, any other
face across `Unsupported` naming the blend's kind and the face across. A
junction is ADR-0035 §3 unchanged: the ball's cross-section at a tangent
vertex is the torus's meridian circle at the vertex's angle.

**5. The parallel-cylinder row: two cylinders with parallel axes, along
a ruling.** In the plane square to the axes each face is a circle; offset
each by `r` toward the ball's side (radius `R_i + sσ_i r`, `s` and `σ_i`
as `ruling_ball` reads them) and the ball's centre is where the two
offsets cross, the root on the edge's side of the line through the two
axes; offsets that do not cross, or one of no radius, are
`BlendTooLarge`. A fillet is the cylinder of radius `r` about the line
through the centre parallel to the axes; each contact is the ruling at
the foot of the centre on its face, a line at constant `u` on the face
and on the blend, whose frame is a plane-cylinder ruling stripe's. A
chamfer is the plane through the two rulings at chord `d` (§2). The ends
are a ruling stripe's: trimmed by the face across, a plane square to the
axes giving the blend's section, an oblique plane `Unsupported` as for
a plane against a cylinder; two of these stripes meeting at a corner
whose third edge stays sharp are `VertexBlend` until a census asks for
their miter. Two cylinders whose axes cross or are skew meet along a
quartic, and stay `Unsupported` (ADR-0007: nothing is fitted but a
pcurve).

**6. The order: the line meridians, then the circle meridians; the
parallel cylinders wait.** The census puts the cone's coaxial pairs first
among pairs, in 15 of the 18 parts, so the plan builds them first. The
circle meridians, a sphere's or a torus's parallel against a coaxial
face, come second (218 edges, 6 parts): they are this row with one more
meridian kind, so they replace the parallel-cylinder row in the plan,
which the census finds on 29 edges. §5 stands as the decision for when a
census asks for it; until then those edges stay `Unsupported` naming the
two cylinders. The torus pairs the plan's non-goals named are, by the
census, two families: the coaxial parallels, which are this row, and a
torus against a cylinder off its axis (82 edges), which is not and stays
`Unsupported`. The ends outrank every pair, and need what ADR-0007 rules
out — an end curve with no closed form, traced and fitted — so they are a
decision for C6's next plan, not a row of this one.

**7. Provenance and refusals.** No new record and no new `Role`: a
stripe of either row is recorded as the ring or a ruling stripe is, its
blend face, contacts, ends and trim vertices `Generated` from the edge,
every shortened seam and corner edge `Modified`. `Reason` gains no
variant: what a row cannot build is `BlendTooLarge`, `VertexBlend`,
`TangentChain` or `Unsupported` as above, each naming its entities. The
checker decides a blend against its faces by the arms it already has:
the coaxial meridian arm (ADR-0008) for the first row, the parallel-axes
row of the cylinder pairs' table (`docs/DATA-MODEL.md` §Curves) for the
second.

## Consequences

- Every surface the two rows build is exact, and every pcurve on them is
  a line; nothing is fitted. Where Open CASCADE walks instead (a cone
  against a cylinder, two cylinders) its oracle values sit 1e-8 relative
  from the closed forms, so those fixtures are held to the closed forms
  by `analytic.measure_differs` with the distances above as the evidence
  (ADR-0015), written by the step that builds each pair.
- `ring` stops being the plane-against-cylinder function: its section is
  the meridian construction and its ends, seams and junctions are
  shared. A split of `ring` into the section and the rest is the refactor
  the plan's step 4 may name.
- A torus against a plane through its axis, along a meridian circle, is
  a different construction (the centre circle lies in a plane parallel to
  the meridian plane) and is not in this row; nor is a torus against a
  cylinder off its axis.
- The two end families (a torus ending on a plane parallel to its axis,
  a ruling stripe ending on a curved face across) are C6's largest
  refusal after this plan, in 12 and 14 parts. Building them means an end
  curve traced on the blend and fitted, as C3's sections are (ADR-0019),
  which ADR-0007 rules out for a blend; that is the next plan's idea,
  with the census as its input.

## Alternatives considered

- **A row per pair**, as Open CASCADE's known parts are written (one for
  the plane against the cone, another for each pair after). Each would
  repeat the offset, the root choice and the frames; the meridian
  section is one construction with the pairs as its inputs, and it is
  the one Open CASCADE's plane–cone part already uses inside.
- **A chamfer measured along the face** (arc length on a circle
  meridian, or on a cylinder's section). Equal to the chord on every
  line meridian, so the plane, cylinder and cone rows do not tell them
  apart; on a curved section it disagrees with the oracle by 4e-4 of `d`
  at `d / R = 0.1`, and the consumer's old backend and every STEP part
  dimension a chamfer by its legs.
- **The torus pairs first.** Step 1's sorted census counted 832 torus
  refusals over the cone's 799. Read by the faces each refusal names,
  most of them were not torus faces of the part at all: they were the
  ring's own torus at an open arc's end, on a plane off its axis, which
  is an end family (§6). The coaxial torus parallels left over are part
  of this row's circle meridians.
- **The parallel cylinders before the circle meridians**, as the plan
  first ordered them. That costs a second construction (the ruling ball
  in the cross-section, the stripe's ends and miters) for 29 edges, where
  the circle meridians reuse the first one for 218.
