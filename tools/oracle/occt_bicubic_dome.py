#!/usr/bin/env python3
"""occt_bicubic_dome.py <out.step>  —  write a free-form solid as Open
CASCADE's STEP: a bicubic B-spline dome over the square [0, 10]², two
spans each way, its boundary poles on z = 0 so that its four edges are
straight lines in that plane, closed by the plane's square. Two faces and
four edges; the dome rises to 4.78 at its centre. The operand of the
boolean fixtures `nurbs-bicubic-dome-fuse` and `section-nurbs-bicubic-dome` (plans/nurbs-analytic-section
step 7), which commit the file. Run as `uv run --project tools/oracle
tools/oracle/occt_bicubic_dome.py`.
"""

import sys
from pathlib import Path

from oracle import OracleError, require_ocp

require_ocp()

from OCP.BRepBuilderAPI import (  # noqa: E402
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_Sewing,
)
from OCP.Geom import Geom_BSplineSurface  # noqa: E402
from OCP.ShapeFix import ShapeFix_Solid  # noqa: E402
from OCP.collections import Array1_double, Array1_int, Array2_double, Array2_gp_Pnt  # noqa: E402
from OCP.TopAbs import TopAbs_SHELL  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from OCP.gp import gp_Pnt  # noqa: E402

from oracle import step  # noqa: E402

# Heights of the 5 x 5 poles: the border on the plane, a bump inside.
HEIGHTS = [
    [0, 0, 0, 0, 0],
    [0, 3, 4, 3, 0],
    [0, 4, 6, 4, 0],
    [0, 3, 4, 3, 0],
    [0, 0, 0, 0, 0],
]


def dome() -> object:
    poles = Array2_gp_Pnt(1, 5, 1, 5)
    for i in range(5):
        for j in range(5):
            poles.SetValue(i + 1, j + 1, gp_Pnt(2.5 * i, 2.5 * j, HEIGHTS[i][j]))
    knots = Array1_double(1, 3)
    for k, v in enumerate((0.0, 0.5, 1.0)):
        knots.SetValue(k + 1, v)
    mults = Array1_int(1, 3)
    for k, m in enumerate((4, 1, 4)):
        mults.SetValue(k + 1, m)
    weights = Array2_double(1, 5, 1, 5)
    for i in range(5):
        for j in range(5):
            weights.SetValue(i + 1, j + 1, 1.0)
    surface = Geom_BSplineSurface(poles, weights, knots, knots, mults, mults, 3, 3)
    top = BRepBuilderAPI_MakeFace(surface, 1e-7).Face()
    polygon = BRepBuilderAPI_MakePolygon()
    for x, y in ((0, 0), (0, 10), (10, 10), (10, 0)):
        polygon.Add(gp_Pnt(x, y, 0))
    polygon.Close()
    bottom = BRepBuilderAPI_MakeFace(polygon.Wire(), True).Face()
    sewing = BRepBuilderAPI_Sewing(1e-6)
    sewing.Add(top)
    sewing.Add(bottom)
    sewing.Perform()
    explorer = TopExp_Explorer(sewing.SewedShape(), TopAbs_SHELL)
    if not explorer.More():
        raise OracleError("the sewing made no shell")
    solid = BRepBuilderAPI_MakeSolid(TopoDS.Shell(explorer.Current())).Solid()
    fix = ShapeFix_Solid(solid)
    fix.Perform()
    return fix.Solid()


def main(argv: list[str]) -> int:
    if len(argv) != 1:
        print(__doc__)
        return 2
    try:
        step.write(dome(), Path(argv[0]))
    except Exception as e:
        print(f"occt_bicubic_dome: ERROR {' '.join(str(e).split())}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
