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

**Chain**:
Edges a blend continues across because both faces turn where they meet
(ADR-0035, ADR-0039).

**Tangent vertex**:
A vertex of two edges that continue one another, where two stripes meet
on one ball (ADR-0041).

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

**Corpus**:
All fixtures, run by the corpus runner one test each.

**Real-part corpus**:
STEP files Arris did not write: the **committed tier** under `real/`, run by
`cargo test`, and the **fetched tier**, NIST's archives surveyed by
`tools/real-parts.sh` (ADR-0026).

**Determinism corpus**:
The recipes whose body dump digests are held equal on every target: native
and `wasm32-wasip1` match `digests.json` byte for byte
(`crates/arris/tests/determinism.rs`, ADR-0064).

**Refusal histogram**:
Every typed refusal over the real-part corpus, counted by kind; with the
first consumer's regressions it picks the next cycle (ADR-0020).
_Avoid_: refusal table (the code that counts it is `histogram`)

**Census**:
A breakdown of one operation's refusals over the real parts, edge by edge
(`arris_debug::census`, the fillet column).

**Differential**:
Random recipes run through Arris and the oracle, disagreements shrunk to a
fixture (`differential::run`).

**Property test**:
Random operands in random poses held to an identity (volume additivity,
round trip, commutativity); a hand-picked case is a fixture instead.

**Twin property**:
The property that geometry given as its exact NURBS twin gives a result
agreeing with the analytic one.

**Finding**:
A defect a nightly, fuzz target or the differential measured, written in
`docs/BACKLOG.md` §Findings with what replays it.
_Avoid_: bug report, issue

**Gate**:
The checks a change passes: the pre-commit hook's slice sized to what is
staged, and the full gate once per plan and at a release.

## Consumers and process

**Consumer**:
An application that builds on Arris; **the first consumer** is the
parametric CAD whose kernel facade Arris replaces.
_Avoid_: client, user (a user is the consumer's)

**Facade**:
The consumer's own interface over its kernel, behind which Arris is
swapped in.

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
