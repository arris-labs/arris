"""A profile swept along a path: `BRepOffsetAPI_MakePipeShell`.

The `sweep` op of the recipe grammar (`recipe.py` documents it) reaches
here with every number already evaluated. The path is a spine wire of one
edge per segment:

    line_to   a line edge from the current point
    arc_to    a circular arc through `via` (`GC_MakeArcOfCircle`)
    helix     a 2D line on the cylinder about `axis` through the current
              point, built to 3D by `BRepLib.BuildCurves3d_s`: the angle
              turns about the axis (its sign is the sense) and the point
              advances `pitch * angle / 2 pi` along it
    nurbs     a `Geom_BSplineCurve` from degree, flat knots, control points
              and weights

The frame law is `SetMode(False)`, the corrected Frenet trihedron, for
`rotation_minimizing`, and `SetMode(gp_Dir)`, the fixed binormal, for
`{"binormal": d}`. The profile is added where it stands: no contact and no
correction, so a profile off the path or off the normal plane is taken as
it is placed. `MakePipeShell` takes a wire, so a face profile is swept as
its outer wire and its holes' and sewn with the two caps. A closed spine
sweeps to a solid with no caps.
"""

import math

from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_Sewing,
)
from OCP.BRepLib import BRepLib
from OCP.BRepTools import BRepTools
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakePipeShell
from OCP.GC import GC_MakeArcOfCircle
from OCP.GeomAbs import GeomAbs_C1
from OCP.Geom import Geom_BSplineCurve, Geom_CylindricalSurface
from OCP.Geom2d import Geom2d_Line
from OCP.ShapeFix import ShapeFix_Solid
from OCP.TopAbs import TopAbs_SHELL, TopAbs_WIRE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape
from OCP.collections import Array1_double, Array1_gp_Pnt, Array1_int
from OCP.gp import gp_Ax3, gp_Dir, gp_Pnt, gp_Pnt2d, gp_Dir2d

from . import OracleError


def _pnt(v: list[float]) -> gp_Pnt:
    return gp_Pnt(v[0], v[1], v[2])


def _helix_edge(start: list[float], axis: dict, pitch: float, angle: float):
    """The edge of a helix arc from `start`: radius the point's distance
    from the axis, `angle` radians about it, `pitch` per turn."""
    origin, direction = axis["origin"], axis["direction"]
    ax = gp_Ax3(_pnt(origin), gp_Dir(*direction))
    rel = [s - o for s, o in zip(start, origin)]
    x, y, z = ax.XDirection(), ax.YDirection(), ax.Direction()
    px = rel[0] * x.X() + rel[1] * x.Y() + rel[2] * x.Z()
    py = rel[0] * y.X() + rel[1] * y.Y() + rel[2] * y.Z()
    pz = rel[0] * z.X() + rel[1] * z.Y() + rel[2] * z.Z()
    radius = math.hypot(px, py)
    if radius == 0.0 or angle == 0.0:
        raise OracleError("a helix arc needs a point off its axis and a nonzero angle")
    surface = Geom_CylindricalSurface(ax, radius)
    rise = pitch / (2.0 * math.pi)
    sign = 1.0 if angle > 0.0 else -1.0
    line = Geom2d_Line(gp_Pnt2d(math.atan2(py, px), pz), gp_Dir2d(sign, sign * rise))
    length = abs(angle) * math.hypot(1.0, rise)
    edge = BRepBuilderAPI_MakeEdge(line, surface, 0.0, length)
    edge.Build()
    if not edge.IsDone():
        raise OracleError("helix: Open CASCADE reports not done")
    shape = edge.Edge()
    BRepLib.BuildCurves3d_s(shape, 1e-9, GeomAbs_C1, 14, 0)
    return shape


def _bspline(spec: dict) -> Geom_BSplineCurve:
    """The curve of a `nurbs` segment, as a geometry fixture's curve is
    given: degree, flat knots, control points and weights."""
    degree = int(spec["degree"])
    flat = [float(k) for k in spec["knots"]]
    points = spec["control_points"]
    weights = [float(w) for w in spec["weights"]]
    if len(flat) != len(points) + degree + 1 or len(weights) != len(points):
        raise OracleError("nurbs path segment: knots must number poles + degree + 1, one weight each")
    values: list[float] = []
    counts: list[int] = []
    for k in flat:
        if values and values[-1] == k:
            counts[-1] += 1
        else:
            values.append(k)
            counts.append(1)
    poles = Array1_gp_Pnt(1, len(points))
    wts = Array1_double(1, len(points))
    for i, (p, w) in enumerate(zip(points, weights)):
        poles.SetValue(i + 1, _pnt(p))
        wts.SetValue(i + 1, w)
    knots = Array1_double(1, len(values))
    mults = Array1_int(1, len(values))
    for i, (k, m) in enumerate(zip(values, counts)):
        knots.SetValue(i + 1, k)
        mults.SetValue(i + 1, m)
    return Geom_BSplineCurve(poles, wts, knots, mults, degree)


def _end_point(shape: TopoDS_Shape) -> list[float]:
    curve = BRepAdaptor_Curve(shape)
    pt = curve.Value(curve.LastParameter())
    return [pt.X(), pt.Y(), pt.Z()]


def spine(path: dict) -> TopoDS_Shape:
    """The wire of `path`: `{"start": [x, y, z], "segments": [...]}`."""
    wire = BRepBuilderAPI_MakeWire()
    here = list(path["start"])
    for seg in path["segments"]:
        if "line_to" in seg:
            to = seg["line_to"]
            made = BRepBuilderAPI_MakeEdge(_pnt(here), _pnt(to))
            made.Build()
            edge = made.Edge()
        elif "arc_to" in seg:
            to = seg["arc_to"]
            arc = GC_MakeArcOfCircle(_pnt(here), _pnt(seg["via"]), _pnt(to))
            if not arc.IsDone():
                raise OracleError("arc path segment: the three points define no arc")
            made = BRepBuilderAPI_MakeEdge(arc.Value())
            made.Build()
            edge = made.Edge()
        elif "helix" in seg:
            h = seg["helix"]
            edge = _helix_edge(here, h["axis"], h["pitch"], math.radians(h["angle_deg"]))
            to = _end_point(edge)
        elif "nurbs" in seg:
            curve = _bspline(seg["nurbs"])
            made = BRepBuilderAPI_MakeEdge(curve)
            made.Build()
            edge = made.Edge()
            to = _end_point(edge)
        else:
            raise OracleError(f"unknown path segment {seg!r}")
        wire.Add(edge)
        here = list(to)
    wire.Build()
    if not wire.IsDone():
        raise OracleError("sweep path: the edges do not make a wire")
    return wire.Wire()


def _wires(face: TopoDS_Shape) -> tuple[TopoDS_Shape, list[TopoDS_Shape]]:
    """The outer wire of a face and its holes' wires."""
    outer = BRepTools.OuterWire_s(TopoDS.Face(face))
    holes = []
    explorer = TopExp_Explorer(face, TopAbs_WIRE)
    while explorer.More():
        wire = explorer.Current()
        if not wire.IsSame(outer):
            holes.append(wire)
        explorer.Next()
    return outer, holes


def _pipe_of_wire(wire: TopoDS_Shape, path: dict, law) -> BRepOffsetAPI_MakePipeShell:
    """The pipe shell of one wire along the spine, built."""
    mk = BRepOffsetAPI_MakePipeShell(spine(path))
    if law == "rotation_minimizing":
        mk.SetMode(False)
    elif isinstance(law, dict) and "binormal" in law:
        d = law["binormal"]
        if math.hypot(*d) == 0.0:
            raise OracleError("sweep binormal is zero")
        mk.SetMode(gp_Dir(*d))
    else:
        raise OracleError(f"sweep law must be \"rotation_minimizing\" or {{\"binormal\": d}}, not {law!r}")
    mk.Add(wire, False, False)
    mk.Build()
    if not mk.IsDone():
        raise OracleError("sweep: Open CASCADE reports not done")
    return mk


def pipe(profile: TopoDS_Shape, path: dict, law) -> TopoDS_Shape:
    """`profile`, a face, swept along `path` under `law` to a solid.
    `MakePipeShell` takes wires, not faces: the outer wire and each hole's
    are swept separately, and the shells and the two caps (the profile
    itself and its last section) are sewn and closed to a solid."""
    outer, holes = _wires(profile)
    mk_outer = _pipe_of_wire(outer, path, law)
    closed = spine(path).Closed()
    if closed and not holes:
        if not mk_outer.MakeSolid():
            raise OracleError("sweep: Open CASCADE could not close the swept shell to a solid")
        return mk_outer.Shape()
    if closed:
        raise OracleError("sweep: a closed path with a holed profile is not a fixture the oracle builds")
    pipes = [mk_outer] + [_pipe_of_wire(h, path, law) for h in holes]
    sewing = BRepBuilderAPI_Sewing(1e-6)
    for mk in pipes:
        sewing.Add(mk.Shape())
    sewing.Add(profile)
    end = BRepBuilderAPI_MakeFace(TopoDS.Wire(mk_outer.LastShape()), True)
    for mk in pipes[1:]:
        end.Add(TopoDS.Wire(mk.LastShape()))
    end.Build()
    if not end.IsDone():
        raise OracleError("sweep: the last section makes no planar face")
    sewing.Add(end.Face())
    sewing.Perform()
    shells = []
    explorer = TopExp_Explorer(sewing.SewedShape(), TopAbs_SHELL)
    while explorer.More():
        shells.append(TopoDS.Shell(explorer.Current()))
        explorer.Next()
    if len(shells) != 1:
        raise OracleError(f"sweep: the sewn pieces make {len(shells)} shells, not one")
    solid = BRepBuilderAPI_MakeSolid(shells[0]).Solid()
    fix = ShapeFix_Solid(solid)
    fix.Perform()
    return fix.Solid()
