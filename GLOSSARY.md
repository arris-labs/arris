# Arris glossary

The words this project uses for its own concepts, one meaning each. A term
here is used as written in code, docs, plans and commits; the words under
_Avoid_ are the synonyms it replaces. Definitions say what a thing **is**;
how it works lives in the design doc section named beside it. Standard
B-Rep and NURBS vocabulary (face, loop, coedge, pcurve, knot) means what
the literature means and is defined in `docs/DATA-MODEL.md`, not here.

## The model

**Model**:
The arena that owns every geometric and topological entity, with one
`Precision`.
_Avoid_: document, scene, world

**Body**:
A set of shells in a model that an operation takes and returns: a solid, a
sheet or a wire.
_Avoid_: shape, part (a part is a consumer's)

**Sheet body**:
A body of open shells with no inside, how a surface model reads.
_Avoid_: surface body, open solid

**Lump**:
An outer shell with the voids it contains, derived from geometry and never
stored (ADR-0006).

**Precision**:
The model-wide tolerance configuration: the default, the floor, and the
maximum an operation may grow to.

**Tolerance**:
The distance within which an entity's geometry is held to be where its
topology says, carried by each vertex, edge and face. Algorithms use the
entity's tolerance or a named constant, never a literal.
_Avoid_: epsilon, eps

**Provenance**:
The generated / modified / deleted record every operation returns for
every entity it touched, its keys an entity or a role.
_Avoid_: history, lineage

**Role**:
A name for an output piece that no input entity generated (a sweep's cap,
a blend face), so a consumer can name it.

**Split order**:
The guarantee that the `k` of `Split(k)` means the same piece after an edit
that keeps which entities bound which piece (ADR-0009).

**Path**:
A G1 chain of lines, arcs, helix arcs and NURBS curves that a profile is
carried along by `sweep` (ADR-0063); a corner is refused, never rounded.
_Avoid_: spine, rail (a rail is the edge a sweep builds, below)

**Loft section**:
A placed profile a loft passes through; the loft joins two or more, loop for
loop (ADR-0065).
_Avoid_: bare "section" (the boolean's word for the curve where two faces
cross)

**Ruled span**:
The part of a loft between two consecutive loft sections when it is straight
between them, a plane, cylinder or cone where its kind holds it and a
degree-1 NURBS otherwise (ADR-0065).

**Frame law**:
The rule that turns a path into a frame at every arc length:
rotation-minimising (the default) or fixed binormal. The `FrameField` is
its result, and the profile's motion is read from it.
_Avoid_: orientation mode, Frenet (a different law, not offered)

**Rail**:
The edge a profile vertex sweeps along a span, shared by the two side
faces that meet there.

**Ring**:
A profile's vertices and edges carried to one station of a sweep: the
start ring, the end ring, a joint between spans. A ring has one entry per
vertex or segment of every loop, in walking order.

**Rise**:
A vertex's rail in `extrude` (a straight line along the sweep) and
`revolve` (a circle about the axis); `SweepPart::Rise` names it in
provenance for every sweep, rails of `sweep` and `loft` included.

**Holonomy**:
The rotation about the tangent by which a rotation-minimising frame comes
back after a closed path; a sweep along a closed path has no caps only
when it is zero within the angular tolerance.

**Checker**:
`arris-check`: the invariant list a valid body satisfies, run after every
operation in debug builds, at `Fast` or `Full`.
_Avoid_: validator

**Refusal**:
A typed error naming the entities for something Arris does not do or
will not guess at: `Unsupported`, a `Reason` such as `TangentContact` or
`BlendTooLarge`, a reader's `Refusal`.
_Avoid_: failure (a failure is a wrong answer or a fault), crash

**Fault**:
An internal inconsistency an operation detected in its own work: a kernel
bug, never a refusal.

## Operations

**Pave**:
A parameter on an edge or section curve where something meets it; paves
are shared between operands, never matched (ADR-0004).

**Pave model**:
`boolean::interferences`, the decomposition every boolean selects from:
face pairs, section edges, paves and common blocks.
_Avoid_: intersection graph

**Section edge**:
An edge a boolean makes where two faces cross.

**Marched section**:
The section of two NURBS surfaces, walked in both surfaces' (u, v) at once
between certified seeds; its edges and both pcurves are fitted from the
walk.
_Avoid_: SSI curve, marched curve

**Seed**:
A certified point of a marched section that decides its topology: a
boundary seed where it reaches an edge or a seam of either domain, a loop
seed where it turns in the first surface's `v` (every closed loop has
two), a singular point where branches meet, each branch reaching it
through a *departure*, the section's point at the singular point's reach.
Every seed is visited, or the section is refused.

**Saddle cell**:
The certified cell of a crossing, a saddle of the section where its two
arms meet: inside it the arms are graphs either side of a separator, and
out of it each leaves as a branch (ADR-0061). At an edge or a corner of an
open domain it is cut there and holds only the arms inside.

**Common block**:
A piece of edge two operands share within tolerance, kept as one edge.

**Stripe**:
The blend surface rolled along one edge between its two faces, in closed
form where the face pair has one.

**Walked stripe**:
A stripe with no closed form: the ball's centre traced along the section of
the two faces' offsets and fitted (ADR-0062).
_Avoid_: marched fillet, numeric fillet

**Spine**:
The curve a stripe's ball centre runs along. Not the roadmap's `Spine:`
line.

**Run-over**:
A stripe whose contact leaves its face through a convex edge that is not
the corner's own, an outer edge or a hole's rim. Between the two
crossings each section, in the stripe's own plane, is the circle of the
radius tangent to the other face through the point where that edge
crosses the plane (ADR-0071, amended).
_Avoid_: cliff edge, overflow

**Rolled face**:
The face a run-over's sections sweep between the crossings, from the
other face to the crossed edge: skinned through them as a walked stripe
is (ADR-0071, amended).

**Chain**:
Edges a blend continues across because both faces turn where they meet
(ADR-0035, ADR-0039).

**Tangent vertex**:
A vertex of two edges that continue one another, where two stripes meet
on one ball (ADR-0041).

**Kink**:
A chain end's vertex of three edges whose corner edge is tangent within
the vertex's tolerance though not within its faces'. A chain runs on
through it only where the end across meets nothing (ADR-0075).

**Junction**:
Where two stripes of a chain meet on one ball's arc, at a tangent vertex
or, where the tangent edge leans, on that edge's offset away from it
(ADR-0035, ADR-0072). Across a kink the two balls stand apart, and they
meet on their mean ball's arc within the vertex's tolerance (ADR-0075).

**Vanishing end**:
An end of a walked stripe where the edge's two faces turn tangent within
the edge's tolerance: the stripe narrows to a pole at the vertex, or stops
short on its narrowest exact arc with the edge's remnant running on
(ADR-0062, ADR-0074).

**Cusp**:
A vertex of three edges where the blended edge and the next edge leave the
same way, the two walls tangent along the third edge (ADR-0042). That third
edge is the cusp's own "spine", not the stripe's **Spine** above. An edge's
_sense_ is whether it is convex or concave.

**Overhang tip**:
A cusp whose two edges have opposite senses, its walls on either side of
the face they share (ADR-0079).

**Healing**:
What the STEP reader repairs in a file it reads (grown tolerances, turned
faces), recorded on the body (ADR-0058).
_Avoid_: fixing, cleanup

## Testing and measuring

**Fixture**:
A directory under `tests/fixtures/<area>/<slug>/`: a recipe
(`fixture.json`), the oracle's answer (`expected.json`) and, once passing,
the blessed dump. The unit of acceptance.
_Avoid_: test case, sample (a sample is `arris_debug::sample`'s hand-built
body)

**Area**:
A fixture's directory by topic (`boolean/`, `blend/`, …), which holds
passing fixtures only; `regression/` holds failures waiting for their fix.

**Bless**:
Writing a passing fixture's `dump.txt` (`ARRIS_BLESS=1`), after which the
dump is the regression guard for ids and provenance.

**Oracle**:
Open CASCADE run through Python (`tools/oracle/`) to produce each fixture's
expected values. Never a build or runtime dependency.
_Avoid_: reference implementation, ground truth (the closed form is, when
there is one: `measure_differs`, ADR-0015)

**Second oracle**:
A second commercial kernel run as a black box on an Arris-authored
recipe, its measurements imported into a fixture's `second_oracle.json`
by a tool (ADR-0073). It stands in for closed forms under
`measure_differs` where the result has none. A **record of changes**
holds the operand's measures beside the result's, and Arris's
`result − operand` is held to its own (a real part, ADR-0073 as amended).

**Corpus**:
All fixtures, run by the corpus runner one test each.

**Real-part corpus**:
STEP files Arris did not write: the **committed tier** under `real/`, run by
`cargo test`, and the **fetched tier**, NIST's archives surveyed by
`tools/real-parts.sh` (ADR-0026).

**Script corpus**:
About 15 hobby parts and 2–3 CAE parts, each a Python script through the
public binding the way a script author or an agent writes it, each stage's
refusal recorded; its histogram ranks the next cycle beside the real-part
corpus's.
_Avoid_: DIY part corpus

**Determinism corpus**:
The recipes whose body dump digests are held equal on every target: native
and `wasm32-wasip1` match `digests.json` byte for byte
(`crates/arris/tests/determinism.rs`, ADR-0064).

**Refusal histogram**:
Every typed refusal over the real-part corpus, counted by kind; with the
first consumer's regressions it picks the next cycle (ADR-0020).
_Avoid_: refusal table (the code that counts it is `histogram`)

**Battery**:
The fixed stages each real-part solid is put through after its read
(`measure`, `write_read`, `box_cut`, the three drills, `fillet`), each
held to Open CASCADE's; a stage Open CASCADE builds what was asked of and
Arris refuses blocks the part (ADR-0026). A **dropped edge**, an asked
edge of a `fillet` or `chamfer` that no Open CASCADE contour took, makes
the stage Open CASCADE refusing (ADR-0070).
_Avoid_: test suite (for this)

**Census**:
A breakdown of one operation's refusals over the real parts, edge by edge
(`arris_debug::census`, the fillet column).

**Blend residue**:
The blends Open CASCADE builds and Arris refuses once the blend network
was built: what C11 works through. A refusal the battery's own sample
causes is not residue.

**Differential**:
Random recipes run through Arris and the oracle, disagreements shrunk to a
fixture (`differential::run`).

**Property test**:
Random operands in random poses held to an identity (volume additivity,
round trip, commutativity); a hand-picked case is a fixture instead.

**Cost** (of a property):
The divisor `c` of a property whose single case costs minutes: it runs the
tier's case count over `c` (`prop_shards!`'s `/ c`), so one count holds
every tier to its wall clock.

**Twin property**:
The property that geometry given as its exact NURBS twin gives a result
agreeing with the analytic one.

**Finding**:
A defect a nightly, fuzz target or the differential measured, written in
`docs/BACKLOG.md` §Findings with what replays it.
_Avoid_: bug report, issue

**Gate**:
The checks a change passes, in three halves: the **static half** (layer
rule, tolerance literals, fmt, clippy) the pre-commit hook runs at a code
commit; the **test half** (the `fast` slice, doctests, `cargo doc` when due)
that runs after the commit on its own tree, or inline at the next hook when
the last commit is not verified; and the full gate once per plan and at a
release.

**Verified**:
A commit whose test half passed: on its own tree after it was committed, or
inline at a later commit's hook over the range that holds it; a prose or
version commit takes its parent's state. The next code commit and any push
wait on it.
_Avoid_: checked (the checker's word), green (a CI run's)

## Consumers and process

**Consumer**:
An application that builds on Arris; **the first consumer** is the
parametric CAD whose kernel facade Arris replaces.
_Avoid_: client, user (a user is the consumer's)

**Facade**:
The consumer's own interface over its kernel, behind which Arris is
swapped in.

**Modelling layer**:
The Python API over the binding — workplanes, selectors, chained
operations — for a script author; the binding beneath it stays 1:1 with
the facade.
_Avoid_: DSL, wrapper

**Tag**:
A name a modelling-layer script puts on a set of a solid's faces, carried
onto each later operation's faces through its provenance; the layer's, in
Python only, and not a kernel attribute (ADR-0009).

**Workplane**:
A frame that a modelling-layer script draws on: sketches are drawn in its
`x` and `y`, and what they make grows along its `z`. A planar face's is
outward; the standard ones are `XY`, `YZ` and `XZ`, right-handed.

**Selection**:
Some of a solid's faces or edges in the modelling layer, as a set in
ascending handle order, picked by provenance, role or tag first and by
geometry after; never by position, so one is narrowed to a single member,
not indexed.

**Probe corpus**:
The first consumer's recorded kernel failures, each a fixture Arris must
pass before the swap (the application gate, ADR-0017).

**Cycle**:
A roadmap section (`C1`, …) with goal, in, out and accept; an unopened one
has a name and no number until `/close-cycle` opens it.
_Avoid_: sprint, milestone (a milestone is a cycle's part, `m1`)

**Closure**:
The rule that what Arris builds, Arris takes as an operand, before the
kernel takes new input (closure before breadth, ADR-0020).

**Conductor**:
The session that runs a plan's steps, each as a fresh subagent on its tier,
and retires the plan itself (`/work <slug>` with no range;
`.agents/skills/work/SKILL.md`).
_Avoid_: orchestrator, driver, runner

**Tier**:
A step's grade read as the model class that executes it: 1 routine, 2
careful, 3 unproven (`.agents/tiers.md`). A plan names tiers, never models.
_Avoid_: level, model (a tier is mapped to a model by a harness's table)
