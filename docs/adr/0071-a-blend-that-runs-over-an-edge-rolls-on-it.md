# ADR-0071 — A blend that runs over an edge rolls on it: between the two crossings the ball touches the other face and the edge, a walked rolled face

- Status: accepted (2026-10-27)
- Plan: `blend-runs-onto` step 2 (the decision, on step 1's fixtures and a
  probe of the census). The spine lands in step 3, the hole's rim in step
  4, the outer edge in step 5.
- Amends: ADR-0007 §"The `BlendTooLarge` bound": a contact that leaves its
  face through an edge that is not the corner's own is a run-over where
  §3 below holds. Before, it was always refused.
- Follows: ADR-0062 (the walked stripe), ADR-0037 (traced sections),
  ADR-0038 (the last amendment of the bound)
- Reference trees read: none. Open CASCADE is run as the oracle
  (`BRepFilletAPI_MakeFillet` through `tools/oracle/`, its result written
  by `occt_step.py` and read with `inspect_step`), not read.

## Context

The script corpus has two refusals with the same reason, *the blend leaves
its face through an edge that is not the corner's own*:

- the pi mount plate's standoff roots, a concave ring 0.6 round a boss
  that stands 0.5 inside the plate's edge;
- the phone stand's lip, a convex blend of 2 whose contact on the lip's
  face runs into the port's hole.

Open CASCADE builds both. Step 1 shrank them to
`regression/standoff-root-fillet-runs-off-the-plate` (a boss of radius 2
whose axis is 2.3 from a plate's front face, root filleted at 0.6) and
`regression/lip-fillet-runs-into-a-hole` (a slab's top edge filleted at 2
over a cross hole of radius 3.5 whose top is 0.5 below it). It then read
Open CASCADE's results face by face.

**Neither result cuts the blend by the face it runs onto**, the plan's
working guess:

- **Plate.** The volume added is 0.9962. The full ring would add 1.0359,
  and the ring cut at the front face's plane 1.0274. The front face is
  untouched. Between the crossings at `x = 5 ± √(2.6² − 2.3²)`, a rational
  patch of degree 2 across runs from the boss's contact down to the
  plate's front edge itself. Its section at `x = 5` is the arc of radius
  0.6 centred at `(5, −0.3, 2 + √0.27)`, within 1.5e-6. That centre lies
  at 2.6 from the boss's axis and at 0.6 from the edge's line. The torus
  is left either side of the patch, in two faces (V/E/F 15/24/11).
- **Slab.** Between the crossings at `y = 10 ± √(3.5² − 2²)`, a rational
  patch in two faces (split at the rim's top) runs from the top face's
  contact down to the rim's upper arc. Its section at `y = 10` is the arc
  of radius 2 centred at `(12 − √1.75, 10, 6)`: at 2 below the top, and
  at 2 from the rim's circle. The hole's wall is untouched up to the rim.
  The blend's cylinder is left in two faces, and the end face's hole loop
  has joined its outer loop (17/27/11).

In both, the ball **rolls on the crossed edge**. It keeps touching the
blend's other face, and touches the edge's curve where it would have
touched the face it left.

A probe at every contact refusal printed the edges of the face left that
the contact crosses, and how often. It ran on both fixtures and on the
run-over census of the four parts the blend network blocks (CTC-02 AP242,
CTC-05 AP242, FTC-07 AP242, FTC-11 AP242; 36 `BlendTooLarge` edges, each
alone):

- **Both fixtures** cross one convex edge twice: a line (plate) or a
  circle (slab). No end of the contact is on it.
- **12 edges, CTC-02's rings**, cross one B-spline edge once and end on a
  line edge. Their contact runs off at the end, not through the middle.
  Open CASCADE refuses all 12.
- **1 edge** is an end arc leaving the face across (ADR-0038's test).
- **23 edges** are refused before any contact is placed: the ring's
  contact at the axis and the meridian's foot, ADR-0038's *contact leaves
  a face* label for the horn and spindle torus.

No real part's refusal is a run-over of this kind. The construction is
the scripts', and the shape a consumer's root and edge fillets take.

## Decision

**1. A run-over rolls the ball on the crossed edge.** The stripe's contact
on face `F` crosses `E`, an edge of `F` that is not the corner's own. Let
`H` be the blend's other face and `G` the other face of `E`. Between the
two crossings, the ball of radius `r` touches `H` and `E`'s curve. Its
centre runs on the section of `H`'s offset by `r` (toward the ball) with
the pipe of radius `r` about `E`. That pipe is a cylinder for a line and a
torus for a circle. Both are analytic, so the section is traced
(ADR-0037). The face the ball sweeps there, the **rolled face**, is a
walked stripe (ADR-0062). Its sections are arcs of radius `r` from the
spine's foot on `H` to its foot on `E`, fitted within ADR-0062's budget.
`E` stands in for a face whose offset is the pipe and whose foot is the
nearest point of the curve.

**2. The closed-form stripe ends where the rolled face begins.** At each
crossing the ball touches `F`, `E` and `H` at once. The stripe's section
there is one arc, shared by both faces and tangent across it. The stripe
is cut at those two arcs:

- An open edge's stripe becomes two faces.
- A ring's torus becomes one face open at the rolled face. Open CASCADE
  splits it in two, and the fixture states that as `counts_differ`.

`E` is split at the crossings. Its stretch between them bounds the rolled
face and `G`, and `G` is unchanged. `F`'s loop runs the contact to each
crossing and then along `E`'s outer pieces. Where `E` is a hole's rim,
the hole's loop joins `F`'s outer loop.

**3. Accepted only where:**

- `E` is convex (the ball rides an edge it cannot pass inside);
- `E` is a line or a circle, so the pipe is analytic;
- the contact crosses `E` exactly twice, inside `E`'s range, neither
  crossing within the trim of a corner end;
- it crosses no other edge of `F`;
- the spine is traced between the two crossings, and each foot on `H`
  lies inside `H` by `on_side_of_face`.

Anything else stays `BlendTooLarge`, naming the blended edge and `E`.
`Reason` gains no variant.

**4. Provenance.**

- `Generated`: the rolled face, the stripe's faces and the crossing
  vertices.
- `Modified`: `F`, `H`, `G` (its loop gains the two crossing vertices)
  and `E`'s two outer pieces.
- `E`'s middle stretch keeps `E`'s id as one more `Modified` piece, since
  it is the same curve between the same two faces, `G` and now the
  rolled face.

## Consequences

- The script corpus's pi mount plate and phone stand are expected to
  build at every stage (plan steps 4, 5, 8). No real part's count moves.
- The rolled face is a fitted surface beside the closed forms. S5 decides
  it against `G`, `H` and the stripe by the walked stripe's arms, or the
  step that meets the gap adds one. Arris's checker leaves Open CASCADE's
  own patches undecided against the torus and the hole's wall, which
  says the gap is real.
- A concave run-over adds less material than a ring cut by the face it
  runs onto. Arris's measure follows Open CASCADE's.
- A contact that crosses an edge once and ends on another (CTC-02's 12)
  is still refused, as Open CASCADE refuses it.
- A run-over of a walked stripe, and a chamfer's, stay refused
  (`blend/split-cylinder-rim-chamfer-past-a-hole`) until a part or a
  script asks.

## Alternatives considered

- **Cut the blend by the face it runs onto** (the plan's working
  decision). It is exact for a convex blend into a hole, but it is not
  what Open CASCADE builds. On the plate, a point on Open CASCADE's patch
  classifies `in` against the cut ring, and the fillet adds 3% more
  material (1.0274 against 0.9962). A consumer comparing bodies would see a different shape.
- **A boolean of the blend's wedge** (cut or fuse the region between the
  faces and the blend surface). It is simple and robust for a convex
  blend, but for the same reason it builds the clipped shape. It also
  needs the wedge's ends to meet the corner ends' trims.
- **Keep refusing.** That leaves the scripts' two commonest fillets, a
  boss's root near an edge and a lip over a hole, unbuilt where Open
  CASCADE builds them.
