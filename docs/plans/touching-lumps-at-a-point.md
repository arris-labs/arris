# Plan: touching-lumps-at-a-point

- Started: 2026-11-18
- Milestone: C12, the training harness's refusals ([roadmap](../ROADMAP.md))
- Idea: [touching-operands-fuse-to-several-shells](../ideas/touching-operands-fuse-to-several-shells.md)
  (point touches and the remaining hinge operations absorbed, Q1 and Q3
  settled in [ADR-0088](../adr/0088-lumps-of-one-solid-may-touch.md))

## Goal

The harness's torus against a box (`torus_link`) and ball on a cylinder's
end disc (`ball_knob`) fuse to one `Solid` of two touching lumps, each with
its own edges and vertices. The two point-touch pairs and the hinge pair
from the curve milestone cut to the target and have `BooleanReason::Empty`
as their common. Their measures and probe classes agree with the oracle,
their built results pass `Full` with no unchecked findings, and
`interference` reports the dimension of each touch. A point-touch fuse
can be used in a later boolean.

## Non-goals

- Shared-topology contacts stay with Q2 in the idea, including both
  frustum rim-point fixtures, edge-touching boxes and the touching cuts.
- General bodies, building a pinched lump, and new STEP or native
  format representations.

## Design deltas

- **Piece classification.** In ARCHITECTURE §Operations, an initial
  sample at a touch can give an `On` verdict for a piece that otherwise
  lies to one side. In `arris-ops` (`boolean/pieces.rs` and
  `boolean/result.rs::select`),
  retry against the implicated operand from deterministic points on the
  original surface and strictly inside the same trimmed piece. Use the
  piece's own loops and seam translate. Use the classifier's tolerances
  to establish clearance and account for retries in the operation's
  meter. A certified inside/outside verdict feeds the existing selection
  table, including the bookkeeping that distinguishes `Empty` from
  `ZeroThickness`. Keep coincident, curve-curvature and transversal
  decisions for their cases, and keep the typed error when the retry
  establishes no side. Keep the original interior point for provenance
  ordering. Record the retry rule and its evidence in a follow-up ADR.
- **Paves and nesting.** A point touch can be on a seam, pole or trimmed
  boundary. Keep the evidence available to selection and keep each
  shell's topology distinct. Preserve the paves required by crossing branches
  and coincident regions. B1 and S5 retain their rules (DATA-MODEL
  §Invariants). Exercise B1's existing clear-point nesting on the shifted
  disc. If it still fails, fix point placement in `arris-check` with a
  direct two-shell test of the failing pose and an outer-in-outer control.
  If the torus trace needs a correction, use `arris-geom`'s existing
  `MeetPoint` with a geometry fixture that distinguishes touch, gap and
  crossing (ADR-0019), and document it in DATA-MODEL §Curves.
- **Surviving touches.** Apply ADR-0088 to the kept regions. A touch
  between separate lumps is allowed. Where a new or inherited touch
  would leave one lump pinched, return `TangentContact` naming the source
  faces. Use the existing checker/ops division of responsibility and
  test the output trims, since a later operand may cover the touch.
- **API.** Rust and Python keep their public types and signatures.
  Update Rustdoc on `fuse`, `cut`, `common` and
  `BooleanReason::TangentContact` as their behavior lands, and the
  corresponding Python docstrings where they describe these cases.
  Existing crate dependency directions and the glossary's **Touching
  lumps** definition apply.

## Steps

Grades: **[1]** routine, the design specifies the work and the tests are
mechanical. **[2]** careful, a geometric or numeric case within the design.
**[3]** unproven, robustness or a bound is established here.

- [x] Step 1 **[3]** — Reproduce the radius-2 ball on the disc at its pole
  with centred and 0.3-offset discs. Record the section, sample and nesting
  failures in regression fixtures with Open CASCADE's values. Make the
  piece selection and any nesting correction in the deltas, write the ADR,
  and graduate the fixtures when `fuse` builds two lumps, both cuts keep
  their target and `common` is `Empty`. Hold a clear gap and a shallow
  overlap beyond tolerance on either side of the touch. Hold the
  surviving-touch rule for the cases this step opens, with the spherical
  cavity against a plane as the same-lump control.
- [x] Step 2 **[3]** — Build the torus and box at the outer point of the
  ring in the harness pose, on the equator seam away from its vertex, and
  off both seams. Trace any hit that wrongly splits a piece or changes
  the dimension of the touch. Graduate oracle-backed fixtures for fuse,
  cut and common. Drive the two harness fuses through Python
  `arris.build` in `test_tangency.py`.
- [ ] Step 3 **[2]** — Extend the hinge recipe
  (`boolean/fuse-box-cylinder-tangent-side`) with sibling cut and common
  fixtures. The external cut keeps the 4×4×1 plate, volume 16 and area 48,
  and the common is `Empty`. Add query probes for the harness pairs,
  the ball on a plate, sphere against a sphere's seam, and cone apex on a
  face from BACKLOG. Check `interference` in both orders against the
  oracle's point or curve dimension.
- [ ] Step 4 **[3]** — Extend the surviving-region rule to a later fuse
  that joins a point-touch pair away from the touch. Hold the named refusal while a
  join covering the touch builds. Add oracle-backed fixtures and a
  seeded property using a point-touch fuse as an operand: vary dimensions
  and rigid pose, cut a tool through one lump clear of the other and the
  touch, and check the volume subtraction, two lumps, disjoint edge and
  vertex ids, the untouched lump's face ids and a clean `Full` report.

## Acceptance

- Register the graduated recipes in `crates/arris/tests/corpus.rs`. Use
  `uv run --project tools/oracle tools/oracle/expected.py <fixture-dirs>`
  to record Open CASCADE's values. Fuses have additive volume and area
  and the volume-weighted centroid of the operands. Cuts have the
  target's measures, and commons have no solid and return exactly
  `Empty`. Use the oracle's contact and off-contact probe classes and
  document count differences caused by imprinting.
- `cargo nextest run -p arris --test corpus` covers the new recipes and
  the existing curve-touch, coincident, crossing and `NonManifold`
  controls. Run `cargo nextest run -p arris-ops --test interference --test
  tangency_prop --test tolerance_band` with `PROPTEST_CASES=256` and
  `tools/py-test.sh tests/test_tangency.py`.
- Run `tools/real-parts.sh` and `tools/script-corpus.sh` and print the
  histograms. Hold C11's close as the ceiling for each row and check for
  panics. Run `tools/full-gate.sh <log>` in the background.

## Docs to update on completion

- ARCHITECTURE §Operations and §Errors, and DATA-MODEL §Invariants or
  §Curves if a placement or intersection implementation detail changes.
- ROADMAP §C12 **In** and AGENTS current state: mark point touches done
  and name the shared-topology milestone next for this family.
- Update the remaining idea's status to cite the completed ADR when
  retiring this plan.
- BACKLOG: reconcile the point-touch `common` and `interference` line
  against the probes that now pass. Record any distinct holding failure.
- CHANGELOG `Unreleased`: write the consumer-visible bullets in the
  commits that land the behavior.

## Open questions

- **Settled in step 1:** [ADR-0089](../adr/0089-resample-a-piece-at-an-isolated-touch.md)
  records the bounded retry, the clearance in both poses and the gap and
  overlap controls. B1 places both shells in both poses with its existing
  clear-point rule.
- **Settled in step 2:** The torus tracer returns an isolated touch in
  all three poses. The seam vertex keeps its own hits, with no section
  edge. The equator hit away from that vertex adds no pave. Off both
  seams, the face pair supplies the only hit at the contact. The
  selection retry from ADR-0089 builds all three, with no tracer or
  pave change. Open CASCADE imprints the equator contact away from its
  vertex, adding one vertex and one edge.
