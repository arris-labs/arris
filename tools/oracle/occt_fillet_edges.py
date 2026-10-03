#!/usr/bin/env python3
"""occt_fillet_edges.py <file.stp> <solid-id> <radius> <probe> <points.json>  — one
solid of a STEP file, each of the edges named by a point of the JSON list in
`points.json` (`[x, y, z]`, in the model's own frame) filleted alone at `radius`; prints, as
JSON, one verdict per point, in order: `builds` (the builder is done and the
result passes `BRepCheck_Analyzer`), `invalid` (done, the result fails the
analyzer), `refuses` (not done, or the builder raises) with `why`, or
`no-edge` where the point is not within `probe` of exactly one edge. A solid
placed more than once is the placement nearest the points. What the run-over
census holds `BlendTooLarge` against: whether Open CASCADE builds the same
blend. Exits 2 with `occt_fillet_edges: ERROR <why>` on stderr for a file or
a solid it cannot read. Run as
`uv run --project tools/oracle tools/oracle/occt_fillet_edges.py`.
"""

import json
import sys
from pathlib import Path

from oracle import OracleError, require_ocp

require_ocp()

from OCP.Bnd import Bnd_Box  # noqa: E402
from OCP.BRepBndLib import BRepBndLib  # noqa: E402
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from OCP.BRepExtrema import BRepExtrema_DistShapeShape  # noqa: E402
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE  # noqa: E402
from OCP.TopExp import TopExp  # noqa: E402
from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as IndexedMapOfShape  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from OCP.gp import gp_Pnt  # noqa: E402

from oracle import step  # noqa: E402


def edges_of(shape):
    mapped = IndexedMapOfShape()
    TopExp.MapShapes_s(shape, TopAbs_EDGE, mapped)
    out = []
    for i in range(1, mapped.Extent() + 1):
        edge = TopoDS.Edge(mapped.FindKey(i))
        box = Bnd_Box()
        BRepBndLib.Add_s(edge, box)
        out.append((edge, box))
    return out


def near_edges(edges, point, probe):
    p = gp_Pnt(*point)
    vertex = BRepBuilderAPI_MakeVertex(p).Vertex()
    found = []
    for edge, box in edges:
        if box.Distance(_box_of(p)) > probe:
            continue
        d = BRepExtrema_DistShapeShape(edge, vertex)
        if d.IsDone() and d.Value() <= probe:
            found.append(edge)
    return found


def _box_of(p):
    box = Bnd_Box()
    box.Add(p)
    return box


def total_distance(shape, points, probe):
    edges = edges_of(shape)
    total = 0.0
    for pt in points:
        p = gp_Pnt(*pt)
        vertex = BRepBuilderAPI_MakeVertex(p).Vertex()
        best = float("inf")
        for edge, box in edges:
            if box.Distance(_box_of(p)) > best:
                continue
            d = BRepExtrema_DistShapeShape(edge, vertex)
            if d.IsDone():
                best = min(best, d.Value())
        total += best
    return total


def main() -> int:
    if len(sys.argv) != 6:
        print("usage: occt_fillet_edges.py <file.stp> <solid-id> <radius> <probe> <points.json>", file=sys.stderr)
        return 2
    path, wanted, radius, probe = Path(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    points = json.loads(Path(sys.argv[5]).read_text())
    try:
        candidates = [s for label, s in step.solids(path) if label == wanted]
        if not candidates:
            raise OracleError(f"{path} has no solid #{wanted} that Open CASCADE reads")
        if len(candidates) > 1:
            candidates.sort(key=lambda s: total_distance(s, points[:8], probe))
        shape = candidates[0]
    except OracleError as e:
        print(f"occt_fillet_edges: ERROR {e}", file=sys.stderr)
        return 2
    edges = edges_of(shape)
    out = []
    for pt in points:
        near = near_edges(edges, pt, probe)
        if len(near) != 1:
            out.append({"verdict": "no-edge", "why": f"{len(near)} edges within {probe}"})
            continue
        try:
            mf = BRepFilletAPI_MakeFillet(shape)
            mf.Add(radius, near[0])
            mf.Build()
            if not mf.IsDone():
                out.append({"verdict": "refuses", "why": "not done"})
            elif BRepCheck_Analyzer(mf.Shape()).IsValid():
                out.append({"verdict": "builds"})
            else:
                out.append({"verdict": "invalid"})
        except Exception as e:  # noqa: BLE001 - OCCT raises its own exception types
            out.append({"verdict": "refuses", "why": str(e).splitlines()[0] if str(e) else type(e).__name__})
    json.dump(out, sys.stdout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
