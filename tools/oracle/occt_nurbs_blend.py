#!/usr/bin/env python3
"""occt_nurbs_blend.py <operand> <out.step>  —  write one operand of the
walked-blend fixtures (plans/nurbs-face-blend step 5b) as Open CASCADE's
STEP, a solid with free-form faces for the fillet and chamfer fixtures
that commit the file. `<operand>` is one of:

- `bicubic-top-box`: the box [0, 10] × [0, 6] × [0, 3…4.2] whose top is a
  bicubic B-spline (5 × 5 poles, two spans each way), its boundary poles
  in the four wall planes, so that each wall meets it along a B-spline
  isocurve and is a plane bounded by it; six faces.
- `nurbs-cap-cylinder`: a cylinder of radius 4 capped at height 6 by the
  sphere of radius 8 through that rim, the cap's sphere converted to its
  exact B-spline by `BRepBuilderAPI_NurbsConvert` and the faces sewn back:
  a NURBS face against a cylinder along a circle.
- `nurbs-slot-plate`: the plate [0, 10] × [0, 6] × [0, 2] with a U slot
  2 wide cut in from x = 10, its round end the half cylinder of radius 1
  about (5, 3), that face converted to its exact B-spline and sewn back:
  the slot's rim a line, a circle on the NURBS face and a line, tangent
  at two vertices.
- `nurbs-box`: the box [0, 3] × [0, 2] × [0, 2] with every face converted
  to its B-spline: each edge between two NURBS faces.

Run as `uv run --project tools/oracle tools/oracle/occt_nurbs_blend.py`.
"""

import math
import sys
from pathlib import Path

from oracle import OracleError, require_ocp

require_ocp()

from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse  # noqa: E402
from OCP.BRepBuilderAPI import (  # noqa: E402
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_NurbsConvert,
    BRepBuilderAPI_Sewing,
)
from OCP.BRepAdaptor import BRepAdaptor_Surface  # noqa: E402
from OCP.BRepCheck import BRepCheck_Analyzer  # noqa: E402
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeSphere  # noqa: E402
from OCP.Geom import Geom_BSplineSurface  # noqa: E402
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Sphere  # noqa: E402
from OCP.ShapeFix import ShapeFix_Solid  # noqa: E402
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain  # noqa: E402
from OCP.TopAbs import TopAbs_FACE, TopAbs_SHELL  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402
from OCP.collections import Array1_double, Array1_int, Array2_double, Array2_gp_Pnt  # noqa: E402
from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt  # noqa: E402

from oracle import step  # noqa: E402

# The sewing tolerance: the faces are built from the same curves, so they
# meet to rounding; a few ulps of the parts' size.
SEWING = 1e-7

# Heights of the box's top poles, `[i][j]` at (2.5 i, 1.5 j): the border
# gently varying so that each wall's top edge is a curve, a bump inside.
TOP = [
    [3.0, 3.2, 3.3, 3.2, 3.0],
    [3.1, 3.6, 3.8, 3.6, 3.1],
    [3.2, 3.8, 4.2, 3.8, 3.2],
    [3.1, 3.6, 3.8, 3.6, 3.1],
    [3.0, 3.2, 3.3, 3.2, 3.0],
]


def _faces(shape):
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        yield TopoDS.Face(explorer.Current())
        explorer.Next()


def _solid(faces) -> object:
    """The solid the faces bound, sewn and fixed, checked valid."""
    sewing = BRepBuilderAPI_Sewing(SEWING)
    for face in faces:
        sewing.Add(face)
    sewing.Perform()
    explorer = TopExp_Explorer(sewing.SewedShape(), TopAbs_SHELL)
    if not explorer.More():
        raise OracleError("the sewing made no shell")
    solid = BRepBuilderAPI_MakeSolid(TopoDS.Shell(explorer.Current())).Solid()
    fix = ShapeFix_Solid(solid)
    fix.Perform()
    solid = fix.Solid()
    if not BRepCheck_Analyzer(solid).IsValid():
        raise OracleError("the sewn solid is not valid")
    return solid


def _twin(shape, kind) -> object:
    """`shape` with each face on a surface of `kind` replaced by its exact
    B-spline, the faces sewn back into a solid."""
    faces = []
    for face in _faces(shape):
        if BRepAdaptor_Surface(face).GetType() == kind:
            converter = BRepBuilderAPI_NurbsConvert(face, True)
            if not converter.IsDone():
                raise OracleError("BRepBuilderAPI_NurbsConvert failed on a face")
            face = TopoDS.Face(converter.Shape())
        faces.append(face)
    return _solid(faces)


def bicubic_top_box() -> object:
    poles = Array2_gp_Pnt(1, 5, 1, 5)
    for i in range(5):
        for j in range(5):
            poles.SetValue(i + 1, j + 1, gp_Pnt(2.5 * i, 1.5 * j, TOP[i][j]))
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
    top = BRepBuilderAPI_MakeFace(surface, 1e-9).Face()
    corners = {(i, j): (10.0 * i, 6.0 * j) for i in (0, 1) for j in (0, 1)}

    def line(a, b):
        return BRepBuilderAPI_MakeEdge(gp_Pnt(*a), gp_Pnt(*b)).Edge()

    def wall(iso, a, b):
        # The wall under the top's boundary isocurve `iso` from corner `a`
        # to corner `b`: down, along the floor, up, back along the curve.
        (xa, ya), (xb, yb) = corners[a], corners[b]
        za, zb = TOP[4 * a[0]][4 * a[1]], TOP[4 * b[0]][4 * b[1]]
        wire = BRepBuilderAPI_MakeWire()
        wire.Add(BRepBuilderAPI_MakeEdge(iso).Edge())
        wire.Add(line((xb, yb, zb), (xb, yb, 0.0)))
        wire.Add(line((xb, yb, 0.0), (xa, ya, 0.0)))
        wire.Add(line((xa, ya, 0.0), (xa, ya, za)))
        return BRepBuilderAPI_MakeFace(wire.Wire(), True).Face()

    walls = [
        wall(surface.VIso(0.0), (0, 0), (1, 0)),
        wall(surface.VIso(1.0), (0, 1), (1, 1)),
        wall(surface.UIso(0.0), (0, 0), (0, 1)),
        wall(surface.UIso(1.0), (1, 0), (1, 1)),
    ]
    floor = BRepBuilderAPI_MakeWire()
    for a, b in (((0, 0), (1, 0)), ((1, 0), (1, 1)), ((1, 1), (0, 1)), ((0, 1), (0, 0))):
        floor.Add(line((*corners[a], 0.0), (*corners[b], 0.0)))
    bottom = BRepBuilderAPI_MakeFace(floor.Wire(), True).Face()
    return _solid([top, *walls, bottom])


def nurbs_cap_cylinder() -> object:
    radius, height, sphere = 4.0, 6.0, 8.0
    centre = height - math.sqrt(sphere**2 - radius**2)
    cylinder = BRepPrimAPI_MakeCylinder(radius, 2.0 * height).Shape()
    ball = BRepPrimAPI_MakeSphere(gp_Pnt(0.0, 0.0, centre), sphere).Shape()
    common = BRepAlgoAPI_Common(cylinder, ball)
    common.Build()
    if not common.IsDone():
        raise OracleError("the cylinder's common with the sphere failed")
    return _twin(common.Shape(), GeomAbs_Sphere)


def nurbs_slot_plate() -> object:
    plate = BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(10.0, 6.0, 2.0)).Shape()
    channel = BRepPrimAPI_MakeBox(gp_Pnt(5.0, 2.0, -1.0), gp_Pnt(11.0, 4.0, 3.0)).Shape()
    # The round end's seam on the far side, inside the channel.
    end = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(5.0, 3.0, -1.0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)), 1.0, 4.0).Shape()
    tool = BRepAlgoAPI_Fuse(channel, end)
    tool.Build()
    cut = BRepAlgoAPI_Cut(plate, tool.Shape())
    cut.Build()
    if not tool.IsDone() or not cut.IsDone():
        raise OracleError("the slot's cut failed")
    unify = ShapeUpgrade_UnifySameDomain(cut.Shape(), True, True, True)
    unify.Build()
    return _twin(unify.Shape(), GeomAbs_Cylinder)


def nurbs_box() -> object:
    box = BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(3.0, 2.0, 2.0)).Shape()
    return step.nurbs(box)


OPERANDS = {
    "bicubic-top-box": bicubic_top_box,
    "nurbs-cap-cylinder": nurbs_cap_cylinder,
    "nurbs-slot-plate": nurbs_slot_plate,
    "nurbs-box": nurbs_box,
}


def main(argv: list[str]) -> int:
    if len(argv) != 2 or argv[0] not in OPERANDS:
        print(__doc__)
        return 2
    try:
        step.write(OPERANDS[argv[0]](), Path(argv[1]))
    except Exception as e:
        print(f"occt_nurbs_blend: ERROR {' '.join(str(e).split())}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
