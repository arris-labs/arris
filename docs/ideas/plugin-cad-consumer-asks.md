# Idea: plugin-cad-consumer-asks

- Status: Accepted 2026-09-26 — A1–A4 and A11 are C5 (docs/ROADMAP.md, ADR-0020's amendment); A5, A8 and A9 are backlog lines; A6 and A7 are recorded as ranking input; A10 stays with `python-binding`. A1 landed (ADR-0028); A2 landed (ADR-0029); A3 (ADR-0030), A11 (ADR-0031) and A4 (ADR-0033) landed 2026-10-01. Kept for A6, A7 and A10, which are not C5's; deleted when they are decided
- Raised: 2026-09-26
- Prompt (verbatim from the human): "during formulating SEED.md, you can suggest feature/architecture change requests for arris to implement in future to better fit our goal of plugin-based FOSS CAD system" — and: "Write requests to arris as new idea file using its idea skill."

## Problem

A general-purpose parametric CAD is being seeded on Arris, called the plugin CAD below. It has a small core, and every domain (robotics, drawings, sheet metal, FEA, part libraries) is a plugin in Rust or Python. Plugins can add history features. A document must open without the plugins that made it, so a plugin feature stores its last result in the file. Python plugins run out of process. The app runs natively and in a browser.

That design asks things of the kernel that no consumer asked while the first consumer was a single application behind a facade. Four of them block the plugin CAD's first cycle (the vertical slice with a plugin feature in its history). The rest decide which of Arris's named cycles the plugin CAD needs first. Under ADR-0020 the ranking of cycles after C4 comes from the refusal histogram and the consumer's regressions. The plugin CAD is the consumer now, so its needs should be on record before C4 closes rather than found as regressions later.

## The asks

| # | Ask | Why the plugin CAD needs it | Its cycle blocked | Size |
|---|---|---|---|---|
| A1 | **Consumer roles**: `Role::Consumer { namespace: u32, key: u64 }`, accepted by `Builder` and by any op that creates from nothing | A plugin feature that builds topology directly (not through `extrude`/`revolve`, whose `SweepPart` already carries the consumer's indices) has no role to root its chains at, so its outputs have no stable name | C1 | 2–3 steps |
| A2 | **Body bytes**: one body's closure plus its `Provenance` serialised and imported into another model, with a compatibility policy (read N−1, migrate) | Frozen plugin results live in user files, and bodies cross the Python plugin's process boundary. `io::native` is the whole model and refuses another version | C1 | 4–5 steps + ADR |
| A3 | **Cancellation**: an interrupt token (a flag the consumer sets, checked at loop boundaries) → `OpError::Interrupted`, rolled back by the transaction; optionally a deterministic step budget | An edit supersedes a running evaluation, and on wasm nothing can kill the thread. Backlog: a thin-elliptic-cylinder section takes 6 min | C1 | 3–5 steps + ADR |
| A11 | **`ops::mirror`**: a reflection in a plane, orientation flipped, provenance one to one. `ops::transform` takes an `Isometry` (rigid only) | Mirror is a core feature; mirrored components | C1 | 2–3 steps |
| A5 | **`region2` as public API** over `Curve2`, exact predicates | The sketcher shades regions from its own arrangement. Sharing Arris's makes "what the sketch shades is what extrude accepts" true by construction | C1–C2 | 2 steps |
| A4 | **STEP product structure**: the reader returns products, instances, placements, names and colours beside the flattened bodies; the writer writes instances, names and colours | Assembly exchange is core to the plugin CAD. Flattening loses instancing and the part names users navigate by | C2 | 5–7 steps + ADR |
| A8 | **Multi-tool boolean**: `cut`/`fuse` with N tools in one General Fuse | A pattern of 100 holes should be one decomposition, not 100 chained ones | C2 | 3–4 steps |
| A9 | **Per-face incremental tessellation**: an edge's discretisation a pure function of the edge and the chord; `tessellate_faces` over a subset | Kept entities keep ids (ADR-0010), so the plugin CAD caches meshes per face; that needs faces meshed at different times to stay watertight | C2 | 3 steps |
| A6 | **The query cycle ranked early**: minimum distance, ray fire, interference, planar section of a body | Measure, precise snapping, interference checks, section views, the drawings plugin | C2–C3 | a cycle |
| A7 | **The sweep and healing cycles**: sweep along a path, loft, shell, offset, draft/tapered extrude, thicken; sheet bodies and `ops::planar_face` | Core features of a general CAD; surface work needs sheets | after C2 | two cycles |
| A10 | **`python-binding` option A**, plus body-bytes interop (A2) and handles that carry their model | It is the geometry library inside an out-of-process Python plugin: a body in, ops locally, a body plus provenance out | C3 | the idea's 9 steps |

**What the plugin CAD does not need**, as ranking signal: the attribute cycle (the plugin CAD names from provenance, and plugin data points at persistent names), API stability before 1.0 (the plugin CAD confines Arris types to one crate and puts its own stable plugin API above), and IGES.

## Constraints it runs into

- **SEED.md §4 non-goals**: none crossed. A4 stays within STEP, and A10 is the binding ADR-0020 §2 already placed in scope. Rendering, drawings and a solver stay out: A6's section returns wires and faces, not a drawing.
- **ADR-0002**: `Role` is exhaustive, so A1 is a breaking change to `arris-topo`, the way `Role::File` was (ADR-0025).
- **ADR-0009**: Arris ships no name grammar. A1 keeps to that: the kernel carries an opaque key and the words stay the consumer's.
- **DATA-MODEL §Native format**: "for the whole model a file of another version is `NativeError::Version`". A2's read-N−1 policy amends that for body bytes; the model format can keep its refusal.
- **ARCHITECTURE §Threading and wasm**: no clock, threads or randomness in a kernel crate. A3's token is a flag and a counter, with no clock; a budget in steps keeps results deterministic.
- **ADR-0025 §Instances**: flattening was chosen because "Arris has no product structure". A4 amends that. The product tree is an `arris-io` value beside the bodies, not topology, and flattened bodies stay available for the histogram.
- **Backlog "Narrow `pub` internals"** pulls the other way from A5. A5 wants one deliberate public 2D API, not the internals exposed.
- **ADR-0020**: the consumer's regressions rank first while a consumer waits. A6 and A7 are that ranking, stated before the regressions exist.
- **Two active plans**: `real-part-corpus` holds one slot, so at most one of these starts before C4 closes.

## Options

### A — The four C1 blockers as one plan beside C4; the rest as backlog lines and a ranking input
A1, A2, A3 and A11 as a plan in the free slot (about 12–15 steps, two ADRs: body bytes and cancellation). A5, A8 and A9 as backlog lines, each picked up when the plugin CAD's C1 or C2 reaches it. A4 as the first plan once C4 closes, since it extends the reader C4 just built. A6 and A7 recorded as the plugin CAD's ranking input for the cycle after C4 (ADR-0020). A10 stays with the `python-binding` idea, gaining A2 as a dependency. Cost: one plan now. The plugin CAD's C1 is never blocked for long.

### B — A consumer cycle after C4 holding all of them
One cycle with a corpus: everything above, before the NURBS/query/sweep choice. It is coherent and reviewable as one unit, but it delays the plugin CAD's C1 by the rest of C4 and bundles small API changes with two whole cycles (A6, A7) that ADR-0020 says should be ranked, not scheduled.

### C — Wait for the plugin CAD's regressions
Take nothing now; let the plugin CAD hit each gap and file it. That is ADR-0020's measuring rule applied literally. But A1–A3 are not regressions. They are API shapes the plugin CAD's document model is designed around from day 0, and discovering them late means redesigning the plugin CAD's C1.

### Do nothing
The plugin CAD's C1 works around a missing role, body format and cancellation in the application. That is the habit both projects' rules forbid ("a gap becomes an Arris fixture, never a workaround").

## Recommendation

**A.** The four blockers are small, public-API-shaped and known now. Doing them beside C4 costs one plan slot and keeps the plugin CAD from building on workarounds. A4 belongs right after the reader, while its code is fresh. The two big cycles stay under ADR-0020's ranking, with the plugin CAD's needs recorded as the consumer's input instead of being scheduled by fiat.

What would change my mind: if `real-part-corpus` needs the second plan slot (reader-cycle scope), A's plan waits for C4 to close. Then B's ordering is the same thing in practice, and the choice is only about labelling.

## Decision for the human

1. Accept the four C1 blockers (A1 consumer roles, A2 body bytes, A3 cancellation, A11 mirror) as one plan beside C4? *Preferred: yes, when a plan slot is free.*
2. STEP product structure (A4) as the first plan after C4 closes, amending ADR-0025 §Instances? *Preferred: yes.*
3. Record the plugin CAD's ranking input (query and sweep cycles first, attribute cycle last) in `docs/ROADMAP.md`'s named-cycles list, as ADR-0020's consumer input? *Preferred: yes, as one line each.*
4. A5, A8 and A9 as backlog lines now? *Preferred: yes.*
5. ADRs: two with the plan (body-bytes compatibility policy, cancellation), and one with A4 (amending ADR-0025). A1 and A11 need none: they extend existing enums and ops the way ADR-0025 did.
