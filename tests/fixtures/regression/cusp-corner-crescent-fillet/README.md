# Cusp corner measurements — construction unresolved

Observed 2026-11-02, Open CASCADE 8.0.1.0.0. This is evidence for the
active cusp-corner-patch plan, not an accepted construction ADR.

Reproduce the 18 observations with:

```sh
uv run --project tools/oracle tools/oracle/cusp_corner.py
```

The record is `tests/fixtures/regression/cusp-corner-crescent-fillet/measurements.json`.
Six desired-result fixtures under `regression/cusp-corner-*` cover crescent,
pocket and spandrel, each filleted and chamfered. Their `expected.json`
files come from `expected.py`, not Arris. At sizes 0.02, 0.1 and 0.3 all
18 oracle bodies are valid and neither selected edge is dropped.

Fillets add one B-spline face; chamfers add none. The crescent fillet's
patch is degree 8 by 8. At radius 0.1 its boundary has four curves:

- A stripe cross-section from (-1.7894736842, -0.8931875131, 0.9)
  to the common top contact (-1.7, -0.8485281374, 1).
- The other cross-section from that contact to
  (-1.6363636364, -0.7713892158, 0.9).
- One curve on each cylinder from those lower contacts back to the
  original cusp (-2, 0, 1). The original cusp survives.

The top contact is the intersection of the two offset contact circles:
centres (0,0), (-1,0), radii 2-r and 1+r. The stripe cross-sections there
supply the two lower contacts. Samples of the wall curves interpolate
height linearly with angle; this observation still needs a residual bound
and confirmation on the other families. Sampling a B-spline's entire
rectangular carrier is insufficient: much lies outside the trimmed face.

Reference trees read: `ChFi3d_Builder_CnCrn` and `ChFi3d_FilBuilder_C2`.
The former contains a third-order plate solve followed by degree-at-most-8
approximation; boundary orders vary between G0 and G1. This is a candidate
explanation, not proof of which branch the recipe takes. No implementation
or identifiers were copied. Arris's independent surface law and interior
comparison remain unresolved. A generic Coons fill is not established by
these boundary observations, nor does matching them establish measures.

The oracle STEP cannot yet serve as an Arris-rendered inspection:

```sh
uv run --project tools/oracle tools/oracle/occt_step.py \
  tests/fixtures/regression/cusp-corner-crescent-fillet target/inspect/cusp-corner.step
cargo run -p arris-debug --example inspect_step -- \
  target/inspect/cusp-corner.step cusp-corner
```

The reader reports #608's gap 0.005603629779155641 past the cap
0.00229128784747792. This is a read observation, not evidence by itself
that the oracle's body or its mass properties are wrong. Diagnose the
boundary/pcurve discrepancy before choosing an oracle-fit allowance;
no fixture tolerance or acceptance condition has been changed.
