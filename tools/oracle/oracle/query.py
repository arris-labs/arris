"""The query fixture kind: Open CASCADE's answers to the questions a
consumer asks of a solid, for `tests/fixtures/query/`. The recipe is the
solid kind's (params, variants, steps, result) plus the questions (also in
`tests/fixtures/README.md`):

    {
      "kind": "query",
      "steps": [...], "result": "<step>",
      "queries": [
        {"label": "through the middle",
         "ray": {"origin": [x, y, z], "direction": [dx, dy, dz]},
         "hits": "first" | "all",
         "expect_hits": 2}                              # optional
      ]
    }

A ray is a half-line from `origin` along `direction` (normalised), so `t`
is the distance along it and a hit at a negative `t` is never reported
(ADR-0053 §5). The answer per variant and query is every hit of
`IntCurvesFace_ShapeIntersector` on the half-line, sorted by `t`, whatever
`hits` asks: the runner takes the first of them for `"first"`. Each hit
records `t`, `point`, the face's `uv`, the face's outward `normal` there,
the `face_centroid` (the key a hit is matched by across kernels, since ids
differ), the line's `transition` (`in`, `out`, `tangent`) and the point's
`state` on its face (`in`, or `on` its boundary). Open CASCADE reports a
ray through an edge once per face and a ray through a vertex once per face
at it; the runner holds Arris to one (`analytic.query_differs`, ADR-0053
§8). Every number may be an expression over `params`.
"""

from typing import Any

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.IntCurveSurface import IntCurveSurface_In, IntCurveSurface_Out
from OCP.Precision import Precision
from OCP.TopAbs import TopAbs_IN, TopAbs_REVERSED
from OCP.TopoDS import TopoDS_Shape
from OCP.gp import gp_Dir, gp_Lin, gp_Pnt, gp_Vec

from . import OracleError
from .recipe import build, resolve_params, variant_names, vector


def compute_query(fixture: dict) -> dict:
    """The `results` of a query fixture: per variant, per query, the hits."""
    queries = fixture.get("queries")
    if not queries:
        raise OracleError("a query fixture needs a non-empty `queries`")
    tolerance = float(fixture.get("tolerances", {}).get("probe", 1e-7))
    results: dict[str, Any] = {}
    for variant in variant_names(fixture):
        shape, _ = build(fixture, variant)
        params = resolve_params(fixture, variant)
        results[variant] = {"queries": [_answer(shape, query, params, tolerance) for query in queries]}
    return {"results": results}


def _answer(shape: TopoDS_Shape, query: dict, params: dict[str, float], tolerance: float) -> dict:
    label = query.get("label", "?")
    if query.get("hits") not in ("first", "all"):
        raise OracleError(f"query {label!r}: `hits` is `first` or `all`")
    ray = query.get("ray")
    if ray is None:
        raise OracleError(f"query {label!r}: no `ray`")
    origin = vector(ray["origin"], params)
    direction = vector(ray["direction"], params)
    if not any(direction):
        raise OracleError(f"query {label!r}: the ray's direction is zero")
    line = gp_Lin(gp_Pnt(*origin), gp_Dir(*direction))
    intersector = IntCurvesFace_ShapeIntersector()
    intersector.Load(shape, tolerance)
    intersector.Perform(line, 0.0, Precision.Infinite_s())
    intersector.SortResult()
    hits = []
    for i in range(1, intersector.NbPnt() + 1):
        face = intersector.Face(i)
        u, v = intersector.UParameter(i), intersector.VParameter(i)
        point = intersector.Pnt(i)
        hits.append(
            {
                "t": intersector.WParameter(i),
                "point": [point.X(), point.Y(), point.Z()],
                "uv": [u, v],
                "normal": _normal(face, u, v),
                "face_centroid": _centroid(face),
                "transition": _transition(intersector.Transition(i)),
                "state": "in" if intersector.State(i) == TopAbs_IN else "on",
            }
        )
    return {"label": label, "hits": hits}


def _transition(transition) -> str:
    if transition == IntCurveSurface_In:
        return "in"
    if transition == IntCurveSurface_Out:
        return "out"
    return "tangent"


def _centroid(face: TopoDS_Shape) -> list[float]:
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(face, props)
    c = props.CentreOfMass()
    return [c.X(), c.Y(), c.Z()]


def _normal(face: TopoDS_Shape, u: float, v: float) -> list[float]:
    """The face's outward normal at `(u, v)`: the surface's, flipped for a
    reversed face. A pole, where the partials are parallel, has none and
    is recorded as the zero vector, which the runner does not compare."""
    surface = BRepAdaptor_Surface(face)
    point, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
    surface.D1(u, v, point, du, dv)
    n = du.Crossed(dv)
    if n.Magnitude() < 1e-12:
        return [0.0, 0.0, 0.0]
    n.Normalize()
    if face.Orientation() == TopAbs_REVERSED:
        n.Reverse()
    return [n.X(), n.Y(), n.Z()]
