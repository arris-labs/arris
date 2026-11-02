#!/usr/bin/env python3
"""cusp_corner_closure.py <fixture-dir>... — the mutual-trim reference of a
cusp-corner fillet closure fixture, imported as its `second_oracle.json`.

Run with `uv run --project tools/oracle tools/oracle/cusp_corner_closure.py`.
For each fixture (plan `cusp-corner-patch` step 6, ADR-0084 §4′, ADR-0073):
the recipe up to its two-edge fillet is built with each edge filleted alone
by Open CASCADE (an exact rolling-ball stripe cut by the next wall), the
two combined by `common`, which is the stripes' mutual trim; the recipe's
later steps are then applied to that body and its volume, area, centroid
and inertia written as a neutral record for `second_oracle.py`. A
construction by the mutual-trim law from an independent kernel's
single-edge stripes, which `cusp_corner_second.py` holds to the second
oracle's runs (`second.json`, at most 0.24 of Open CASCADE's own bound),
not a run of a second kernel on the recipe itself.
"""
import copy
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from oracle import OracleError, occt_version, require_ocp

require_ocp()
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from oracle.fixture import load_fixture  # noqa: E402
from oracle.measure import measure  # noqa: E402
from oracle.recipe import _build_step, _split, build, expand_patterns, resolve_params, vector  # noqa: E402

HERE = Path(__file__).resolve().parent


def reference(fixture):
    steps = fixture["steps"]
    k = next(i for i, s in enumerate(steps) if s["op"] == "fillet")
    fillet = steps[k]
    singles = []
    for edge in fillet["edges"]:
        alone = copy.deepcopy(fixture)
        alone["steps"] = steps[: k + 1]
        alone["steps"][k]["edges"] = [edge]
        alone["result"] = fillet["name"]
        alone.pop("analytic", None)
        singles.append(build(alone, "default", [])[0])
    op = BRepAlgoAPI_Common(singles[0], singles[1])
    op.Build()
    trim = op.Shape()
    if not BRepCheck_Analyzer(trim).IsValid():
        raise OracleError("the mutual trim is not valid")
    shapes = {fillet["name"]: trim}
    params = resolve_params(fixture, "default")
    probe = float(fixture.get("tolerances", {}).get("probe", 1e-7))
    for step in expand_patterns(steps[k + 1:]):
        name, kind = step["name"], step["op"]
        if kind == "split":
            sides = _split(shapes[step["of"]], step["plane"], params)
            from oracle.recipe import number
            side = "positive" if number(step.get("side", 1), params) > 0.0 else "negative"
            shapes[name] = sides[side]
        else:
            shapes[name] = _build_step(step, kind, params, lambda n: shapes[n], probe, None, None)
    return shapes[fixture["result"]]


def main(argv):
    for arg in argv:
        directory = Path(arg)
        fixture = load_fixture(directory)
        shape = reference(fixture)
        m = measure(shape, [], 1e-7)
        record = {"source": f"Open CASCADE {occt_version()} common of the two single-edge fillets (mutual trim)",
                  "run": "cusp_corner_closure.py", "accuracy": 1.0, "scale": 1.0,
                  **{key: m[key] for key in ("volume", "area", "centroid", "inertia")}}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "record.json"
            path.write_text(json.dumps(record))
            subprocess.run([sys.executable, str(HERE / "second_oracle.py"), str(path), str(directory)], check=True)
        print(f"{directory}: counts {m['counts']}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
