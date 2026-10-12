#!/usr/bin/env python3
"""occt_tolerances.py <file.stp>  — read a STEP file as Open CASCADE's
reader heals it by default (ADR-0026 §3, `oracle.step.solids`) and print,
as JSON, one object per solid: `id` (the `#id` of its `MANIFOLD_SOLID_BREP`
or `BREP_WITH_VOIDS`), `size` (the diagonal of its bounding box) and
`vertices` and `edges`, the entities carrying a tolerance above `FLOOR`
(`point` for a vertex, `midpoint` of the curve for an edge, and `tolerance`),
with `max_vertex` and `max_edge`, the largest of all. What the gap census
(ADR-0058) sets Arris's healing ceiling beside: the tolerances Open CASCADE
carries the same part at. Exits 2 with `occt_tolerances: ERROR <why>` on
stderr when the file does not read. Run as
`uv run --project tools/oracle tools/oracle/occt_tolerances.py`.
"""

import json
import sys
from pathlib import Path

from oracle import require_ocp

require_ocp()

from OCP.Bnd import Bnd_Box  # noqa: E402
from OCP.BRep import BRep_Tool  # noqa: E402
from OCP.BRepAdaptor import BRepAdaptor_Curve  # noqa: E402
from OCP.BRepBndLib import BRepBndLib  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE, TopAbs_VERTEX  # noqa: E402
from OCP.TopExp import TopExp  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as IndexedMapOfShape  # noqa: E402

from oracle import step  # noqa: E402

# Entities below this are not listed: Open CASCADE's own default is 1e-7 and
# an exporting kernel's usual claim 1e-5 to 1e-4.
FLOOR = 1e-3


def entities(shape, kind):
    found = IndexedMapOfShape()
    TopExp.MapShapes_s(shape, kind, found)
    return [found.FindKey(k) for k in range(1, found.Extent() + 1)]


def measure(shape) -> dict:
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    lo, hi = box.CornerMin(), box.CornerMax()
    vertices, edges = [], []
    max_vertex = max_edge = 0.0
    for v in entities(shape, TopAbs_VERTEX):
        vertex = TopoDS.Vertex(v)
        tol = BRep_Tool.Tolerance_s(vertex)
        max_vertex = max(max_vertex, tol)
        if tol > FLOOR:
            p = BRep_Tool.Pnt_s(vertex)
            vertices.append({"point": [p.X(), p.Y(), p.Z()], "tolerance": tol})
    for e in entities(shape, TopAbs_EDGE):
        edge = TopoDS.Edge(e)
        tol = BRep_Tool.Tolerance_s(edge)
        max_edge = max(max_edge, tol)
        if tol > FLOOR and not BRep_Tool.Degenerated_s(edge):
            curve = BRepAdaptor_Curve(edge)
            p = curve.Value(0.5 * (curve.FirstParameter() + curve.LastParameter()))
            edges.append({"midpoint": [p.X(), p.Y(), p.Z()], "tolerance": tol})
    return {
        "size": lo.Distance(hi),
        "vertices": vertices,
        "edges": edges,
        "max_vertex": max_vertex,
        "max_edge": max_edge,
    }


def main(argv: list[str]) -> int:
    if len(argv) != 1:
        print(__doc__)
        return 2
    try:
        out = [{"id": label, **measure(solid)} for label, solid in step.solids(Path(argv[0]))]
    except Exception as e:  # OracleError, or Open CASCADE's own
        message = " ".join(str(e).split()) or type(e).__name__
        print(f"occt_tolerances: ERROR {message}", file=sys.stderr)
        return 2
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
