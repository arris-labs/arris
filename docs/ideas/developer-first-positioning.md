# Idea: developer-first-positioning

- Status: Accepted 2026-10-17 (A, L-A); credibility and ADR-0067 landed; script corpus done (plans/script-corpus); the friction list kept for the layer plan. Still to be absorbed, in order: the modelling layer's first milestone, the MCP server
- Raised: 2026-10-17
- Prompt (verbatim from the human): "I think that positioning arris target audience as DIY enthusiasts was wrong. Read @docs/notes/positioning.md - do you think that direction is bettter for arris?" — then, on the verdict: "I agree with you on all points, so yes, /idea that"

## Problem

Since 2026-10-05 the work has been ranked for one audience: hobbyists who
model DIY parts in the plugin CAD. The plugin CAD has no users until it is
a usable desktop CAD. A usable desktop CAD means years of UI work that
agents do badly, because a UI has no oracle. So the kernel's feedback loop
waits on the part of the system least suited to how it is built. A hobbyist
who hits a failed boolean leaves. A developer who calls the kernel from a
script sends the five lines that fail, and that is the input every rule in
`.agents/rules/kernel.md` turns into a fixture.

`SEED.md` §1 already names "code-first modelling tool" among the consumers,
and ADR-0020 §2 put a binding "for code-first and agent-driven modelling" in
this repository. The DIY framing was never written into an ADR. It lives in
C10's "In" list (the DIY part corpus) and in how plans were ranked. What
the repository lacks for a developer audience:

- **Something to look at.** `docs/assets` holds the logo, and no example
  script ships. The README's "Not yet" line is stale: it still lists sweeps
  and lofts, which landed in ADR-0063 and ADR-0065.
- **A modelling layer.** The wheel is 1:1 with the facade (ADR-0034 §1):
  no workplanes, no selectors, no chained calls. Next to build123d or
  CadQuery it is the layer below what a script author expects to write.
- **A way in from a browser or from an agent's tool list.** CI builds
  `wasm32-unknown-unknown`, but no crate exports anything to JavaScript.
  No MCP server exists.

## Constraints it runs into

- **ADR-0034 §1**: the ergonomic modelling layer "belongs in another
  repository on top of this one". Option L-A below amends this.
- **ADR-0020 §2**: the consumer's regressions rank first "while there is a
  consumer waiting on a swap". The plugin CAD keeps that for its
  regressions. Its *feature asks* stop ranking above the histogram.
- **`SEED.md` §4 non-goals and the roadmap's "What not to spend agent time
  on"**: rendering and UI. A browser playground's viewer is UI. It would
  live outside the kernel crates (a `site/`, or its own repository) or not
  be built at all. A modelling layer must not grow a sketch solver (§9).
- **`docs/COMPARISON.md` §What is not claimed**, and its untracked evidence
  file: speed, breadth parity, robustness rate, production readiness,
  adoption. The review the human brought adds three claims to that list:
  - "solves topological naming": provenance is what a naming scheme
    needs, and the scheme is the application's (ADR-0009);
  - a wasm size against opencascade.js: never measured;
  - a share of mechanical parts the analytic kinds cover: no source.
  `SEED.md` §9's "~95 %" is a design guess, not a claim.
- **ADR-0017 / the first consumer**: unchanged. The swap stays on its own
  schedule.

## Options

### Positioning

**A — developer-first.** Code-CAD script authors and AI agents first, CAE
pipelines second (provenance through a boolean lets tags on faces survive
a cut; per-face tessellation (ADR-0052) carries them to triangles). The
plugin CAD becomes one consumer and a showcase. The DIY part corpus keeps
its parts and changes its harness (below). Cost: no kernel change. The
ranking of the side slot changes, and the launch work is new.

**B — DIY through the plugin CAD**, as now. Its users come once the CAD is
usable. Until then the kernel's feedback comes from the nightly and the
real-part corpus alone.

**Do nothing**: B by default. Treating both audiences as equal is B with
extra work, because no slot is freed for A's items.

### The modelling layer (only under A)

**L-A — in the wheel, pure Python** (`arris.<name>` over `arris._arris`):
workplanes, profile builders, chained operations, and selectors over
provenance and geometry. It ships and versions with the binding, and a
kernel break edits it in the same commit, the reason ADR-0020 kept the
binding in-repo. It amends ADR-0034 §1 and adds no Rust crate. Tested by
the CI `python` job. Cost: a milestone plan of about 8 steps. A second one
follows if selectors need their own.

**L-B — a separate repository**, as ADR-0034 says. The kernel stays lean.
The layer lags every pre-1.0 break by a release, which ADR-0020 already
rejected for the binding on the same grounds.

**L-C — a build123d backend.** It reuses an existing API and its users.
build123d is written against OCP's classes throughout, so this is a port of
its internals, not an adapter. We would chase another project's API, whose
selectors are positional, exactly what provenance makes unnecessary.
Rejected.

**L-D — none.** Developers get the raw binding. It is honest but loses the
comparison a code-CAD user makes in the first five minutes.

### The corpus

The DIY part corpus (ROADMAP C10, "In") becomes the **script corpus**:
the same ~15 hobby parts plus two or three CAE parts (a flow domain with
tagged inlet and outlet faces cut by an obstacle). Each part is written as
a Python script through the public binding, the way a script author or an
agent would write it, with each stage's refusal recorded. It is written
against the raw binding now, so it ranks kernel work without waiting for
the layer. Where the raw API is clumsy, the scripts show it, and that is
the evidence for the layer's scope. They are ported when the layer lands.

#### Evidence from the script corpus: friction in the raw API

The 18 script-corpus parts (`crates/arris-py/corpus/`) reveal where the raw
binding makes scripts verbose and clumsy. This per-part record is the direct
input for the modelling layer (`arris.build`):

1. **`enclosure`**:
   - Selecting 4 vertical edges for filleting required calling `edge_points(body)`
     (tessellating the B-rep!) and checking polyline endpoints for $\Delta z > 1$.
   - Selecting the open top face for `shell` required calling `face_frame()`
     on all faces to test normal $z > 0.5$ and origin $z = 20$.
   - Floor posts had to be positioned starting at $z = \text{WALL}/2$ to guarantee
     volumetric overlap and avoid coplanar face issues in booleans.
   - Root fillet edges were found by checking radial distance $\sqrt{(x-x_0)^2 + (y-y_0)^2} \approx r$
     on tessellated polylines.
2. **`hinge-leaf`**:
   - Manual coordinate placement and positioning for knuckle cylinder, pin hole,
     and screw holes.
   - Join fillet edges identified by inspecting edge polylines near the knuckle root.
3. **`phone-stand`**:
   - Profile boilerplate: extruding an angled profile required constructing
     `Profile(Loop.polygon(...), origin, normal, x_axis)` with explicit 3D vectors.
   - Edge selection for front lip and back support fillets required coordinate inspection.
4. **`pi-mount-plate`**:
   - 4 corner standoffs, 4 holes, and center slot required manual coordinate loops
     and repeated `fuse_many` / `cut_many`.
   - Corner edge selection required bounding-box checks on polylines.
5. **`wall-hook`**:
   - Profile chaining: hook profile and stiffener rib required separate 2D sketches
     in different planes, manual extrusions, and boolean fusion.
   - Inner curve edge selection required checking polyline coordinates.
6. **`knob`**:
   - Polar patterning: 6 finger flutes cut around the cylinder required manual
     angle loops with `math.cos`/`math.sin` and individual cylinder primitives.
7. **`spool-holder`**:
   - Hub and flange profile required explicit 2D loop coordinates rather than
     a sketch builder with step/offset helpers.
8. **`hose-adapter`**:
   - Repeated barbs required manual loop coordinate calculations for every sawtooth.
9. **`bottle-cap`**:
   - Helical sweep: required explicit `Path` with `helix` segment, aligning
     the triangular thread profile normal to the path start tangent.
10. **`drawer-handle`**:
    - Path construction: required manual 3D segments (`line_to`, `arc_to`, `line_to`),
      and placing the profile in the path's starting plane.
11. **`cable-clip`**:
    - Arc segments in profiles required calculating arc via-points by hand.
12. **`funnel`**:
    - Revolved thin-walled profile required computing all inner and outer wall
      coordinates manually.
13. **`vase`**:
    - 4 loft sections at different heights required 4 separate `Profile` objects
      with identical normal/x_axis vectors.
14. **`spur-gear-blank`**:
    - Circular pattern: 24 gear teeth required an explicit Python loop rotating
      a box primitive around the axis and fusing 25 bodies.
    - 6 lightening holes required trigonometric coordinate calculations.
15. **`gopro-mount`**:
    - Prong pattern: 3 prongs translated and fused manually, then a transverse
      hole cut through all three.
16. **`cantilever`**:
    - CAE boundary condition tagging: finding fixed ($x=0$) and load ($x=L$)
      faces required calling `face_frame()` across all faces and checking normals.
    - Tip fillet edges selected by polyline coordinate filtering at $x=L$.
17. **`duct-obstacle`**:
    - CFD boundary tagging: inlet ($x=0$), outlet ($x=L$), and wall faces
      manually classified by inspecting face frames.
    - Obstacle cylinder had to be manually oversized across duct walls to ensure
      a clean through-cut.
18. **`heat-sink`**:
    - Fin array: 5 fins created in a manual loop with intentional $z$-overlap into
      the base plate to avoid coplanar boolean issues.
    - 4 mounting holes placed by coordinate list.

#### Summary of friction themes for `arris.build`

1. **Selectors over provenance and geometry (Q6)**:
   - Selecting edges for fillets/chamfers currently forces scripts to call
     `edge_points()` (tessellating the B-rep!) to filter by 3D coordinates.
   - Selecting faces for shells, cuts, or CAE tags forces scripts to query
     `face_frame()` on every face.
   - *Scope for `arris.build`:* Filter chains like `part.faces().filter(normal=Z).highest()`,
     `part.edges().vertical()`, and provenance selectors like `part.generated(feature)`.
2. **Workplanes and sketches (Q7)**:
   - Creating any profile currently requires explicit `Profile(Loop(...), origin, normal, x_axis)`.
   - *Scope for `arris.build`:* `wp = part.face(...).workplane()`, `wp.circle(r).extrude(h)`.
3. **Linear and polar patterns**:
   - Every patterned feature (gear teeth, flutes, fin arrays, bolt circles)
     currently requires Python loops with trigonometry and manual `fuse_many` / `cut_many`.
   - *Scope for `arris.build`:* `pattern_linear(n, step)`, `pattern_polar(n, angle, axis)`.
4. **Holes and pockets (automatic through-sizing)**:
   - Scripts currently have to manually oversize cutting tools (e.g. $z - 5$ to $z + 5$)
     and offset starting positions to ensure clean booleans.
   - *Scope for `arris.build`:* Semantic `hole(depth="through")` and `pocket()`.
5. **Chained immutable operations (Q7)**:
   - Every raw operation returns `(body, record)`. Scripts have to unpack
     tuples on every line.
   - *Scope for `arris.build`:* `body.cut(...).fillet(...)` returning new values.
6. **First-class CAE face tags**:
   - Tracking boundary conditions currently requires manual `Labelled` provenance
     inheritance loops.
   - *Scope for `arris.build`:* Tags that stick to faces and survive booleans automatically.


### Credibility, cheap

Landed: the showcase part, the capability matrix, the README update, and ADR-0067.

### Later, as their own plans

- **The MCP server**: a thin Python package over the wheel (build, check,
  measure, render, export). About 3 steps.
- **A JS binding**: wasm-bindgen under `forbid(unsafe_code)`, checked the
  way ADR-0034 checked pyo3. It is an ADR plus about 6 steps. The
  playground beside it is UI (see Constraints).

Each is ranked against draft and thicken when a slot frees.

## Recommendation

Positioning A, the layer L-A, the script corpus in the DIY corpus's place
in C10, and the credibility plan in the side slot ahead of draft and
thicken. B loses because its feedback waits on the one component agents
build worst. L-B loses for the reason ADR-0020 kept the binding here. L-C
costs a port and gives up the selectors provenance makes possible. L-D
hands the first impression to build123d.

What would change my mind: the plugin CAD reaching usable within a cycle
or two. Its users would then outrank any launch audience. Another trigger
is the script corpus showing that the raw binding is enough for agents,
which would demote L-A below the MCP server.

## Decision for the human

Round 1:

❓ **Q1 — Positioning**: who Arris is built for first: (A) developers, with
code-CAD script authors and agents first and CAE pipelines second, the
plugin CAD one consumer whose regressions still rank first (ADR-0020) and
whose feature asks no longer do; (B) DIY hobbyists through the plugin CAD,
as now.
➡️ A. The kernel's feedback loop stops waiting on a UI.

❓ **Q2 — Where the modelling layer lives**: (A) in the wheel as pure
Python, amending ADR-0034 §1; (B) a separate repository, as ADR-0034 says;
(C) a build123d backend; (D) none.
➡️ A. It breaks in the same commit as the kernel, the reason the binding
is in-repo.

❓ **Q3 — The script corpus**: it replaces the DIY part corpus in C10, with
~15 hobby parts plus 2–3 CAE parts as Python scripts on the raw binding,
its histogram ranking C11 beside the real-part one.
➡️ Yes. Same parts, a harness a developer recognises, and evidence for
the layer's scope.

❓ **Q4 — The credibility plan**: the README synced, a capability matrix,
one showcase part with a provenance-coloured render and an oracle-read STEP,
and three more claims under "not claimed". It takes the side slot ahead of
draft and thicken.
➡️ Yes. About 4 steps, and nothing else here is useful until it exists.

❓ **Q5 — The ADR**: one ADR, written by the first plan that needs it,
recording the audience and (with Q2 A) amending ADR-0034 §1.
➡️ Yes.

Round 2:

❓ **Q6 — How the modelling layer selects**: (A) by provenance and role
first (`result.generated`, the faces a box's role names, the walls a
cut made), with geometric filters (kind, normal, axis, extreme along a
direction) returning sets in a deterministic order, and no positional
index selectors; (B) build123d-style geometric and positional selectors
only; (C) both as equals.
➡️ A. Positional selectors break when a parameter changes, and this is
the one place a script author can see what provenance buys.

❓ **Q7 — The layer's shape and name**: (A) immutable values with
chained methods that each return a new value (`plate.cut(holes)`), the
way the kernel's operations return a new body and its provenance, with
workplanes as frames, in `arris.build`; (B) build123d's context managers
(`with BuildPart():`), whose builder state is implicit.
➡️ A. It mirrors the kernel's contract, and an agent reads explicit
values more reliably than implicit builder state. "Part" stays out of the
name (the glossary keeps it the consumer's).

❓ **Q8 — The launch gate**: what exists before you post anywhere. (A)
the credibility plan, the layer's first milestone and the script corpus
committed with its histogram; (B) the credibility plan alone; (C) A plus
a browser playground. Where and when it is posted is yours, and the agent
never posts.
➡️ A. The first thing a code-CAD reader tries is writing a part, and the
histogram is what makes "it refuses instead of lying" a number.

❓ **Q9 — The MCP server**: (A) in this repository, as an optional extra
of the wheel (`pip install arris[mcp]`), planned after the layer's first
milestone: run a script and return the body's measures, the checker's
report and any typed refusal; (B) a separate repository; (C) not
planned.
➡️ A. It breaks with the layer it calls. A rendered image is left out
because ADR-0034 §5 keeps PNG out of the wheel. Adding one later is its
own amendment.

❓ **Q10 — The JS binding and the playground**: (A) the JS binding as a
backlog line, ranked after the MCP server, with its ADR checking
wasm-bindgen under `forbid(unsafe_code)` as ADR-0034 checked pyo3; the
playground not built in this repository (UI is a non-goal); (B) both
before the launch; (C) both rejected.
➡️ A. The browser audience is real but second, and its viewer belongs
to a consumer.

*Later*: none. Venue and timing of the launch are the human's, at launch.

## Decision

Round 1 (2026-10-17), as given: "Accept Q1=A, Q2=A, Q3=Yes, Q4=Yes,
Q5=Yes, and accept both Glossary entries (Script corpus and Modelling
layer)."

- Q1: A. Developers first: code-CAD script authors and agents, then CAE
  pipelines. The plugin CAD is one consumer. Its regressions still rank
  first under ADR-0020, and its feature asks no longer do.
- Q2: A. The modelling layer ships in the wheel as pure Python, amending
  ADR-0034 §1.
- Q3: yes. The script corpus replaces the DIY part corpus in C10.
- Q4: yes. The credibility plan takes the side slot ahead of draft and
  thicken.
- Q5: yes. One ADR, written by the first plan that needs it.
- Glossary: *Script corpus* and *Modelling layer* added.

Round 2 (2026-10-17), as given: "Accept Q6=A, Q7=A, Q8=A, Q9=A, Q10=A."

- Q6: A. The layer selects by provenance and role first. Geometric filters
  return sets in a deterministic order. No selector picks by index.
- Q7: A. Immutable values with chained methods that each return a new
  value; workplanes are frames; the module is `arris.build`.
- Q8: A. The launch waits on the credibility plan, the layer's first
  milestone, and the script corpus committed with its histogram. The
  human posts it.
- Q9: A. The MCP server is `arris[mcp]` in this repository, planned
  after the layer's first milestone, and returns no rendered image under
  ADR-0034 §5.
- Q10: A. The JS binding is a backlog line ranked after the MCP server.
  The playground is rejected for this repository (`docs/BACKLOG.md`
  §Rejected), and so is the build123d backend (Q2).

