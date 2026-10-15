# Idea: sweep-and-loft

- Status: Accepted 2026-10-15 (option A, every preferred answer); the sweep
  half absorbed by `docs/plans/sweep.md` (its decisions are ADR-0063).
  This file keeps only the loft half, which its own plan absorbs and
  deletes.
- Raised: 2026-10-15
- Prompt (verbatim from the human): "sweep and loft"

## Problem

Loft is the other half of the plugin CAD's largest open ask, A7
(`docs/ideas/plugin-cad-consumer-asks.md`), and C10's next item after
sweep. The DIY audience reaches for it to make a transition duct, a
nozzle, or a grip from a circle to a rounded square.

What exists already: `fit_skin` (`crates/arris-geom/src/nurbs/skin.rs`)
skins a surface through compatible sections, and its doc names loft as its
next consumer; the exact NURBS kinds (`nurbs/exact.rs`); and, from the
sweep plan, the placed `Profile` and the oracle script's pattern. What does
not exist: a section-matching rule, provenance roles for a loft, and an
oracle for `BRepOffsetAPI_ThruSections`.

## Constraints it runs into

- **ADR-0020 §1, closure.** Adjacent fitted side faces meet as NURBS ×
  NURBS, refused until C10's marcher; keep the fitted set as small as the
  geometry allows.
- **Exhaustive enums (kernel rules).** New `Role` and reason variants are
  breaking: `### Breaking`, and the Python binding in the same commit.
- **ADR-0013, layers.** Section matching belongs in `geom`, the operation
  in `ops`.
- **Oracle.** The acceptance is "match `ThruSections`", so the matching rule
  is the one Open CASCADE can be asked for.

## Option A, the loft half (decided)

**Loft** takes two or more placed `Profile`s with matching loop counts. It
can be ruled or smooth, and it can be closed. A section with fewer segments
is split by arc length to match the others, as `ThruSections` does. The
profile's start point fixes the correspondence between sections. Ruled
spans between coplanar edges are planes, coaxial circles give cones and
cylinders, and everything else is skinned by `fit_skin`.

Cost: about 6 plan steps (section matching, ruled and smooth spans, the
exact cases, roles and binding, oracle script, corpus and properties). It
forecloses nothing; the NURBS × NURBS refusals lift when the marcher lands.

## Decision

Answered 2026-10-15, every preferred answer:

- Q5: sweep and loft are two plans in sequence in the main slot, leaving
  the second slot for draft and thicken. *Yes.*
- Q6: the loft gets its own ADR (section matching, ruled versus smooth),
  written by its plan. *Yes.*

(Q1–Q4 were the sweep's, recorded in ADR-0063.)
