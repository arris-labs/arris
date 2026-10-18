# Idea: nurbs-nurbs

- Status: Accepted (C); M1, the section, absorbed by plans/nurbs-nurbs-section;
  M2, the operands, waits for the next plan, `nurbs-nurbs-operands`
- Raised: 2026-10-18
- Prompt (verbatim from the human): "/idea nurbs-nurbs"; the roadmap's line
  is "NURBS against NURBS: a marcher with explicit seam handling,
  `intersect_curves` for two NURBS curves, and the boolean and blends the
  real parts ask of them" (docs/ROADMAP.md §C10, third item)

## Problem (what M1 leaves)

M1 gives the three intersectors their NURBS × NURBS arms, the marched
section between certified seeds (ADR-0068, written by M1). With no guard
before the intersector, booleans and the checker reach it at once, but no
plan has vetted them. What is still to do:

- **C10's output against itself.** A sweep cut by a sweep, a loft through
  a loft, a fitted blend face against a fitted sweep face, held to Open
  CASCADE. ADR-0020's closure is why: a sweep or loft is an operand of
  every boolean the analytic kinds allow (ADR-0063, ADR-0065), not yet of
  each other.
- **Blends on an edge between two fitted faces.** ADR-0062's table refuses
  "two B-spline faces"; `sweep/sweep-thread-chamfer-unsupported` holds it
  (Open CASCADE builds it). A walked stripe's offsets of two NURBS faces
  are a marched section.
- **The checker.** S5 leaves two NURBS faces undecided at `Full`
  (`arris-check/tests/full.rs:183`, `twin_prop.rs:55`).
- **Nesting of lumps** with NURBS faces on both shells
  (`regression/nurbs-cylinder-slab-cut-two-lumps`).
- **Speed.** The NURBS × plane walk is slow already (BACKLOG §79: CTC-02's
  box cut 680 s in the test profile); two-NURBS booleans run it beside the
  marcher.

No refusal histogram asks for any of this: the fetched tier's NURBS row
is 0 of 38, and the script corpus's 4 refusals are elsewhere. Demand is
the cycle's closure and the checker's gap.

## M2's scope, as decided

About 7 steps: profile the NURBS × plane walk first (Q8); the pave and
booleans on two NURBS faces; S5 decided at `Full`; the walked stripe
between two NURBS faces, amending ADR-0062's row (Q9); sweep × sweep and
loft × sweep fixtures against Open CASCADE; `twin_prop` without its S5
exemption; the lumps finding read. Two faces flush over an area stay
refused (Q4). Whatever M1's step 7 turns into `regression/` fixtures joins
this list.

## Decision

Round 1, 2026-10-18:

- Q1: C, certified seeds and a measured walk.
- Q2: two plans, M1 `nurbs-nurbs-section`, then M2 `nurbs-nurbs-operands`.
- Q3: A, the twin property over every analytic pair, with Open CASCADE's
  `GeomAPI_IntSS` on fitted pairs.
- Q4: yes, two NURBS faces flush over an area stay refused `Unsupported`.
- Q5: yes, M1 writes the ADR; M2 amends ADR-0062's two-B-spline row.
- Glossary: *Marched section* and *Seed* accepted, added to `GLOSSARY.md`.

Round 2, 2026-10-18:

- Q6: A, the loop condition is measured in M1; the ADR records the winner.
- Q7: A, the intersectors merge knot spans narrower than the parameter
  resolution before isolating, with no API change (M1 step 3).
- Q8: yes, profiling the NURBS × plane walk is M2's first step.
- Q9: yes, M2 includes the walked stripe between two NURBS faces.
- Q10: no. Under the developer-first positioning (ADR-0067), the side
  slot goes to the developer launch items ahead of the plugin CAD's
  asks: the credibility plan (already landed), then `arris.build`'s first
  milestone, which shares no files with the marcher. Draft and thicken
  are queued after it.

The options weighed (A, a plain marcher; B, 4D certified isolation; C; do
nothing) are in git history, in `e978e054`.
