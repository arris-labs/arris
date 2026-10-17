# ADR-0067 — The developer-first audience and the modelling layer in the wheel

- Status: accepted (2026-10-18)
- Plan: `showcase-and-matrix` step 5
- Amends: ADR-0034 (§1: the modelling layer ships in the wheel as pure Python,
  `arris.build`, amending "belongs in another repository on top of this one")
- Follows: ADR-0009 (no name grammar, provenance is the primitive), ADR-0020
  (consumer regressions rank first, binding in-repo)

## Context

Since 2026-10-05 the kernel's user-facing priorities were ranked for hobbyists
modelling DIY parts in the plugin CAD. The plugin CAD has no users until it is a
usable desktop CAD, meaning years of UI work that agents do badly because UI
has no oracle. The kernel's feedback loop was waiting on the component least
suited to how it is built: a hobbyist who hits a failed boolean leaves,
whereas a developer who hits a failure sends the five lines of script that
reproduce it, which `.agents/rules/kernel.md` turns into a fixture.

`SEED.md` §1 already names "code-first modelling tool" among consumers, and
ADR-0020 §2 put a binding "for code-first and agent-driven modelling" in this
repository. ADR-0034 §1 relegated an ergonomic modelling layer to "another
repository on top of this one", which would make it lag every pre-1.0 kernel
break by a release — the very reason ADR-0020 kept the binding in-tree.

## Decision

1. **The audience is developers first**: code-CAD script authors and AI agents
   first, CAE pipelines second (where provenance through booleans lets boundary
   tags survive cuts and carry into tessellation). The plugin CAD is one
   consumer: its regressions rank first (ADR-0020 §2), its feature asks do not
   outrank the refusal histogram.
2. **The modelling layer ships in the wheel as pure Python (`arris.build`)**,
   amending ADR-0034 §1. It adds no Rust crate and versions in lockstep with
   the binding. A kernel break edits the layer in the same commit.
3. **Selection is by provenance and role first**: selectors query
   `result.generated`, the faces an operation's role names, and walls an
   operation created. Geometric filters (surface kind, normal, axis, extreme
   along a direction) return sets in a deterministic order. No selector picks
   by positional index.
4. **Values are immutable and chained**: methods return new values
   (`plate.cut(holes)`), matching the kernel's functional contract and making
   agent-driven generation reliable without hidden builder state. Workplanes
   are frames.
5. **The launch gate**: public posting waits on the credibility plan (showcase
   part, capability matrix, and non-claims), the modelling layer's first
   milestone, and the script corpus committed with its histogram. Where and
   when to post is the human's decision; the agent never posts.
6. **Claims never made**:
   - Arris does not "solve topological naming": provenance is what a naming
     scheme needs, and the scheme is the application's (ADR-0009);
   - no WASM size comparison against other kernels: never measured;
   - no claimed percentage of mechanical parts covered by analytic kinds: no
     source.

## Consequences

- The DIY part corpus in C10 becomes the script corpus: ~15 hobby parts plus
  2–3 CAE parts written as Python scripts against the raw binding.
- `arris.build` lives in `crates/arris-py/python/arris/build/` and is exercised
  by CI's `python` job.
- The MCP server (`arris[mcp]`) and JS binding are ranked in the roadmap behind
  the layer's first milestone.

## Alternatives considered

- **Keep DIY positioning**: leaves kernel feedback gated on CAD UI maturity.
- **Separate repository for the modelling layer (ADR-0034 §1)**: rejected; lags
  kernel breaks.
- **build123d backend**: rejected; build123d is tightly coupled to OCP and its
  positional selectors abandon the advantages of provenance.
- **Raw binding only**: rejected; loses the first five minutes for a code-CAD
  user.
