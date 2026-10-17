# Idea: developer-first-positioning

- Status: Accepted 2026-10-17 (A, L-A); to be absorbed by its plans, in order: the credibility plan (side slot), the script corpus (C10), the modelling layer's first milestone, the MCP server. Each plan absorbs its part and cuts this file down
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

### Credibility, cheap

About 4 steps, one plan:
- the README synced with what landed;
- a capability matrix (works / refuses, per operation and face kind) drawn
  from COMPARISON.md;
- one showcase part: a short Python script, its render from the `inspect`
  renderer with faces coloured by provenance, its STEP read back by the
  oracle, at the top of the README;
- the three claims above added to "What is not claimed".

Fixtures and the oracle give this its numbers. No one judges a picture.

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

