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

A distance query is `{"label", "distance": {"a": <operand>, "b": <operand>}}`
(ADR-0054); an operand is `{"point": [x, y, z]}`, `{"step": "<step>"}`
(the body that step built) or a step narrowed to the face, edge or vertex
nearest a point, `{"step": …, "face": {"near": [x, y, z]}}` (ids differ
between kernels). The answer is `BRepExtrema_DistShapeShape`'s: the
`distance`, every solution's two points with the kind each lies on
(`vertex`, `edge`, `face`) and a face's `_face_centroid`, and `contained`
from `InnerSolution`. A point operand is a vertex to Open CASCADE.

An interference query is `{"label", "interference": {"a": <operand>, "b":
<operand>}}` with whole `{"step": …}` operands (ADR-0055). The answer holds
the `distance` (`BRepExtrema_DistShapeShape`), the `verdict` (`apart` when
the distance exceeds the fixture's `probe`; else `overlapping` when
`BRepAlgoAPI_Common` has a volume above 1e-9 of the smaller operand's;
else `touching`), the common's
`volume` and the containment flags `a_in_b` / `b_in_a` (the common's
volume equals the operand's), and for a touching pair the `contact`
dimension: `face` when some face pair's `BRepAlgoAPI_Common` has area,
else `curve` when `BRepAlgoAPI_Section` holds an edge of positive length,
else `point`. `BRepAlgoAPI_Section` alone reports only the boundary of a
face contact, so the face test is the pairwise common.

A section query is `{"label", "section": {"body": {"step": …}, "plane":
{"origin": [x, y, z], "z": [x, y, z], "x": [x, y, z]}}}` (ADR-0056; `x` is
optional). The answer: `none` when `BRepAlgoAPI_Common` of the solid and a
planar face past its box holds no face; else the common's `faces`, `loops`
(its wires), `area` and `centroid` (`BRepGProp`), and from
`BRepAlgoAPI_Section(solid, plane)` the distinct `edges` and their total
`edge_length`.

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
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Section
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakeVertex
from OCP.BRepExtrema import (
    BRepExtrema_DistShapeShape,
    BRepExtrema_IsInFace,
    BRepExtrema_IsOnEdge,
    BRepExtrema_IsVertex,
)
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.IntCurveSurface import IntCurveSurface_In, IntCurveSurface_Out
from OCP.Precision import Precision
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_IN, TopAbs_REVERSED, TopAbs_VERTEX, TopAbs_WIRE
from OCP.TopExp import TopExp
from OCP.TopoDS import TopoDS_Shape
from OCP.collections import IndexedMap_TopoDS_Shape_TopTools_ShapeMapHasher as IndexedMapOfShape
from OCP.gp import gp_Ax3, gp_Dir, gp_Lin, gp_Pln, gp_Pnt, gp_Vec

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
        shape, shapes = build(fixture, variant)
        params = resolve_params(fixture, variant)
        results[variant] = {"queries": [_answer(shape, shapes, query, params, tolerance) for query in queries]}
    return {"results": results}


def _answer(
    shape: TopoDS_Shape, shapes: dict[str, TopoDS_Shape], query: dict, params: dict[str, float], tolerance: float
) -> dict:
    label = query.get("label", "?")
    if "distance" in query:
        if "ray" in query:
            raise OracleError(f"query {label!r}: a ray or a distance, not both")
        return {"label": label, "distance": _distance(shapes, query["distance"], params, label)}
    if "interference" in query:
        if "ray" in query:
            raise OracleError(f"query {label!r}: a ray or an interference, not both")
        return {"label": label, "interference": _interference(shapes, query["interference"], params, label, tolerance)}
    if "section" in query:
        if "ray" in query:
            raise OracleError(f"query {label!r}: a ray or a section, not both")
        return {"label": label, "section": _section(shapes, query["section"], params, label)}
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


def _operand(shapes: dict[str, TopoDS_Shape], spec: dict, params: dict[str, float], label: str) -> TopoDS_Shape:
    """A distance operand: `{"point": [x, y, z]}`, or `{"step": name}`
    whole or narrowed by `"face"`, `"edge"` or `"vertex"` to the entity
    nearest `{"near": [x, y, z]}`."""
    if "point" in spec:
        return BRepBuilderAPI_MakeVertex(gp_Pnt(*vector(spec["point"], params))).Vertex()
    step = spec.get("step")
    if step not in shapes:
        raise OracleError(f"query {label!r}: no step {step!r}")
    whole = shapes[step]
    narrowed = [(kind, spec[kind]) for kind in ("face", "edge", "vertex") if kind in spec]
    if not narrowed:
        return whole
    if len(narrowed) > 1:
        raise OracleError(f"query {label!r}: narrow to one of face, edge, vertex")
    kind, where = narrowed[0]
    near = BRepBuilderAPI_MakeVertex(gp_Pnt(*vector(where["near"], params))).Vertex()
    best, best_distance = None, float("inf")
    for entity in _entities(whole, {"face": TopAbs_FACE, "edge": TopAbs_EDGE, "vertex": TopAbs_VERTEX}[kind]):
        d = BRepExtrema_DistShapeShape(near, entity)
        if d.IsDone() and d.Value() < best_distance:
            best, best_distance = entity, d.Value()
    if best is None:
        raise OracleError(f"query {label!r}: the step {step!r} has no {kind}")
    return best


def _entities(shape: TopoDS_Shape, kind) -> list[TopoDS_Shape]:
    """The distinct sub-shapes of `kind`, in the explorer's order."""
    found = IndexedMapOfShape()
    TopExp.MapShapes_s(shape, kind, found)
    return [found.FindKey(i) for i in range(1, found.Extent() + 1)]


_SUPPORT = {BRepExtrema_IsVertex: "vertex", BRepExtrema_IsOnEdge: "edge", BRepExtrema_IsInFace: "face"}


def _distance(shapes: dict[str, TopoDS_Shape], spec: dict, params: dict[str, float], label: str) -> dict:
    """`BRepExtrema_DistShapeShape` between the two operands: the distance,
    every solution it lists with the kind each point lies on and the
    centroid of a face, and whether it found one operand inside the other's
    solid (`InnerSolution`)."""
    a = _operand(shapes, spec["a"], params, label)
    b = _operand(shapes, spec["b"], params, label)
    extrema = BRepExtrema_DistShapeShape(a, b)
    if not extrema.IsDone():
        raise OracleError(f"query {label!r}: BRepExtrema_DistShapeShape failed")
    solutions = []
    for i in range(1, extrema.NbSolution() + 1):
        solution: dict[str, Any] = {}
        for side, point, kind, on in (
            ("a", extrema.PointOnShape1(i), extrema.SupportTypeShape1(i), extrema.SupportOnShape1(i)),
            ("b", extrema.PointOnShape2(i), extrema.SupportTypeShape2(i), extrema.SupportOnShape2(i)),
        ):
            solution[side] = [point.X(), point.Y(), point.Z()]
            solution[f"{side}_support"] = _SUPPORT[kind]
            if kind == BRepExtrema_IsInFace:
                solution[f"{side}_face_centroid"] = _centroid(on)
        solutions.append(solution)
    return {"distance": extrema.Value(), "solutions": solutions, "contained": bool(extrema.InnerSolution())}


def _volume(shape: TopoDS_Shape) -> float:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, props)
    return props.Mass()


def _area(shape: TopoDS_Shape) -> float:
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(shape, props)
    return props.Mass()


def _interference(shapes: dict[str, TopoDS_Shape], spec: dict, params: dict[str, float], label: str, probe: float) -> dict:
    """The verdict on two solids; see the module docstring."""
    a = _operand(shapes, spec["a"], params, label)
    b = _operand(shapes, spec["b"], params, label)
    extrema = BRepExtrema_DistShapeShape(a, b)
    if not extrema.IsDone():
        raise OracleError(f"query {label!r}: BRepExtrema_DistShapeShape failed")
    distance = extrema.Value()
    out: dict[str, Any] = {"distance": distance}
    if distance > probe:
        out["verdict"] = "apart"
        return out
    common = BRepAlgoAPI_Common(a, b)
    if not common.IsDone():
        raise OracleError(f"query {label!r}: BRepAlgoAPI_Common failed")
    volume = _volume(common.Shape())
    va, vb = _volume(a), _volume(b)
    if volume > 1e-9 * min(va, vb):
        out.update(
            {
                "verdict": "overlapping",
                "volume": volume,
                "a_in_b": abs(volume - va) <= 1e-9 * va,
                "b_in_a": abs(volume - vb) <= 1e-9 * vb,
            }
        )
        return out
    out["verdict"] = "touching"
    out["contact"] = _contact(a, b, probe)
    return out


def _section(shapes: dict[str, TopoDS_Shape], spec: dict, params: dict[str, float], label: str) -> dict:
    """The plane's section of a solid; see the module docstring."""
    solid = _operand(shapes, spec["body"], params, label)
    plane = spec["plane"]
    origin = gp_Pnt(*vector(plane["origin"], params))
    z = gp_Dir(*vector(plane["z"], params))
    axes = gp_Ax3(origin, z, gp_Dir(*vector(plane["x"], params))) if "x" in plane else gp_Ax3(origin, z)
    pln = gp_Pln(axes)
    # A square of the plane that covers the solid's box from the origin.
    box = Bnd_Box()
    BRepBndLib.Add_s(solid, box)
    lo, hi = box.CornerMin(), box.CornerMax()
    x0, y0, z0, x1, y1, z1 = lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z()
    centre = gp_Pnt((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    reach = 2.0 * (origin.Distance(centre) + ((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2) ** 0.5) + 1.0
    face = BRepBuilderAPI_MakeFace(pln, -reach, reach, -reach, reach).Face()
    common = BRepAlgoAPI_Common(solid, face)
    if not common.IsDone():
        raise OracleError(f"query {label!r}: BRepAlgoAPI_Common failed")
    region = common.Shape()
    faces = _entities(region, TopAbs_FACE)
    area = _area(region) if faces else 0.0
    if not faces or area <= 0.0:
        return {"none": True}
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(region, props)
    c = props.CentreOfMass()
    section = BRepAlgoAPI_Section(solid, pln)
    if not section.IsDone():
        raise OracleError(f"query {label!r}: BRepAlgoAPI_Section failed")
    edges = _entities(section.Shape(), TopAbs_EDGE)
    length = 0.0
    for edge in edges:
        line = GProp_GProps()
        BRepGProp.LinearProperties_s(edge, line)
        length += line.Mass()
    return {
        "none": False,
        "faces": len(faces),
        "loops": len(_entities(region, TopAbs_WIRE)),
        "edges": len(edges),
        "edge_length": length,
        "area": area,
        "centroid": [c.X(), c.Y(), c.Z()],
    }


def _contact(a: TopoDS_Shape, b: TopoDS_Shape, probe: float) -> str:
    """`face`, `curve` or `point`: the dimension of what two touching
    solids share."""
    boxes = []
    for shape in (a, b):
        boxes.append([(f, _box(f, probe)) for f in _entities(shape, TopAbs_FACE)])
    for fa, ba in boxes[0]:
        for fb, bb in boxes[1]:
            if ba.IsOut(bb):
                continue
            common = BRepAlgoAPI_Common(fa, fb)
            if common.IsDone() and _area(common.Shape()) > probe * probe:
                return "face"
    section = BRepAlgoAPI_Section(a, b)
    for edge in _entities(section.Shape(), TopAbs_EDGE):
        props = GProp_GProps()
        BRepGProp.LinearProperties_s(edge, props)
        if props.Mass() > probe:
            return "curve"
    return "point"


def _box(shape: TopoDS_Shape, gap: float) -> Bnd_Box:
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    box.SetGap(gap)
    return box


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
