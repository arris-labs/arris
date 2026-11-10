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

**Regular band** (of a torus):
The part of a torus off its axis, where `R + r cos v` exceeds the face's
tolerance; the only part a face may lie on (ADR-0080).

**Horn torus**:
A torus with `R = r`: it touches its axis at one point, which no face reaches.

**Spindle torus**:
A torus with `R < r`: it crosses itself on its axis, and a face lies on the
outer sheet of its regular band, never on the lemon (inner) sheet.

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

**Trace**:
A face's curve in a blend's cross-section, the plane square to a line
edge: a line for a plane, a circle for a cylinder about a parallel axis.
The rolling ball's centre is where the two traces, offset toward it,
cross (ADR-0036 §5).

**Spine**:
The curve a stripe's ball centre runs along. Not the roadmap's `Spine:`
line.

**Contact**:
The curve where a stripe's ball touches one of its two faces, an edge of the
blend; a stripe has two, and an end arc joins them (ADR-0007). Not the
boolean's contact curve, where two surfaces touch without crossing.
_Avoid_: bare "contact curve" for this (the boolean's word)

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
meet on their mean ball's arc within the vertex's tolerance (ADR-0075), or,
at a vertex on a seam, at the ball whose contact is on it (ADR-0083).

**Miter**:
Where two stripes of equal radius meet at a vertex whose third edge stays
sharp: their end arcs are one edge, the intersection of the two blend
surfaces, shared by both and recorded as generated from both blended edges
(ADR-0007, ADR-0044).

**Remnant**:
The stretch of a blended edge a vanishing end leaves between the vertex and
the stripe's narrowest exact arc: a new edge on the same curve, with the
edge's tolerance, running on to the vertex (ADR-0074).

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

**Mutual trim**:
How two blends end at a cusp where both edges are blended: each stripe
runs on past the ball at the offsets' meeting and is cut by the other
along their section, from the face they share down to the spine, which is
cut there; the cusp vertex goes and no face is added (ADR-0084).
_Avoid_: corner patch (a fitted face, rejected there)

**Flush fan**:
The face across a concave blend's end when a fuse left it as several pieces on
one plane: the seam between two pieces is a smooth edge at the end's vertex.
The end is one lone face's section, in that plane (ADR-0082).

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
(`arris_debug::census`, the fillet column), and of the cusp pairs the parts
hold (`cusp_pairs`, ADR-0084).

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

**Budget** (of a property shard):
The wall clock a property shard spends at the nightly tier: it runs chunks
of cases, each seeded from the base seed, the shard and the chunk's index,
until the next chunk would pass the budget, the tier's case count a
ceiling. How many cases ran depends on the runner; which case failed does
not.
_Avoid_: timeout (nextest's cap, which ends a test unreported)

**Chunk** (of a budgeted property shard):
The unit a budget is spent in and a failure replays by: `CHUNK_CASES` (8)
cases, seeded by the sha256 of the shard seed and the chunk's index, so the
index named by `ARRIS_PROPTEST_CHUNK` reruns alone. The last chunk of a
shard is cut at the tier's ceiling.

**Twin property**:
The property that geometry given as its exact NURBS twin gives a result
agreeing with the analytic one.

**Finding**:
A defect a nightly, fuzz target or the differential measured, written in
`docs/BACKLOG.md` §Findings with what replays it.
_Avoid_: bug report, issue

**Signature** (of a failure):
A failure's (source, operation, class, key): the source the property's
`binary::property` (shard suffix dropped), a fuzz target or the
differential; the class and key, first match wins: `timeout`, `abort`,
`panic` at its `file:line`, `checker` at its row, `differential` at its
`case N` class, `variant` at a typed token, `assert` at the message's stem
(the text up to its first digit, trailing punctuation cut). Never a number,
an id or the rest of a message's text: the stem is the test's format string,
which no draw changes. A *known* signature names an open finding and goes
when the finding does; a new one is what turns a nightly red.
_Avoid_: fingerprint

**Gate**:
The checks a change passes, in three halves: the **static half** (layer
rule, tolerance literals, fmt, clippy) the pre-commit hook runs at a code
commit; the **test half** (the `fast` slice, doctests, `cargo doc` when due)
that runs after the commit on its own tree, or inline at the next hook when
the last commit's verification is red or missing; and the full gate once per plan and at a
release.

**Verified**:
A commit whose test half passed: on its own tree after it was committed, or
inline at a later commit's hook over the range that holds it; a prose or
version commit takes its parent's state. A push waits on it; the next code
commit does not (its own verification is queued over a run still going).
_Avoid_: checked (the checker's word), green (a CI run's)

**Covered**:
A commit whose own verification was replaced by a later commit's, run
over a range that includes it: it takes that run's verdict, and a red
names the range, not the commit.
_Avoid_: skipped (a commit with no verification at all)

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
