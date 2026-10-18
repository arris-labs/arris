"""The geometry fixture kind: Open CASCADE's evaluations, projections and
intersections of named analytic surfaces and curves, for
`tests/fixtures/geom/`. The recipe grammar (also in
`tests/fixtures/README.md`):

    {
      "kind": "geometry",
      "params":   {...},                                # optional numbers
      "surfaces": {"<name>": {"type": "plane",    "origin", "z", "x"},
                   "<name>": {"type": "cylinder", "origin", "z", "x", "radius"},
                   "<name>": {"type": "elliptic_cylinder", "origin", "z", "x", "major_radius", "minor_radius"},
                   "<name>": {"type": "cone",     "origin", "z", "x", "radius", "half_angle_deg"},
                   "<name>": {"type": "sphere",   "origin", "z", "x", "radius"},
                   "<name>": {"type": "torus",    "origin", "z", "x", "major_radius", "minor_radius"},
                   "<name>": {"type": "nurbs",    "degree": [p, q], "knots": [[...], [...]],
                              "control_points": [[[x, y, z], ...], ...], "weights": [[w, ...], ...]}},
      "curves":   {"<name>": {"type": "line",     "origin", "direction"},
                   "<name>": {"type": "circle",   "origin", "z", "x", "radius"},
                   "<name>": {"type": "ellipse",  "origin", "z", "x", "major_radius", "minor_radius"},
                   "<name>": {"type": "nurbs",    "degree", "knots", "control_points", "weights"}},
      "samples":  [{"of": "<name>", "params": [[u, v], ...] | [t, ...], "points": [[x, y, z], ...]}],
      "pairs":    [{"a": "<name>", "b": "<name>"},
                   {"a": "<curve>", "b": "<curve>", "extrema": true},
                   {"a": "<curve>", "b": "<surface>", "extrema": true},
                   {"a": "<surface>", "b": "<surface>", "extrema": true}]
    }

Frames are `gp_Ax3(origin, z, x)`: `x` is projected perpendicular to `z`
and both are normalised, `y = z × x`. A pair is two surfaces, a curve
`a` against a surface `b`, or two curves with a `nurbs` one among them
or two conics. An `elliptic_cylinder` is the section ellipse extruded
along `z` (`Geom_SurfaceOfLinearExtrusion`), Open CASCADE having no
analytic surface for it, which is the same point set; it is not a `gp`
quadric, so it takes no surface pair, and it is trimmed to
`EXTRUSION_REACH` either way along its axis, since the general
intersector finds nothing on an unbounded parametric range. Every number
may be an expression over `params` as in the solid kind.

The result per sample: `Geom_*::D2` at every parameter and
`GeomAPI_ProjectPointOnSurf` / `OnCurve` for every point (nearest point,
its parameters, the distance). Per pair: `IntAna_QuadQuadGeo` for two
surfaces — the type (`empty`, `coincident`, `point`, `line`, `circle`,
`ellipse`, `parabola`, `hyperbola` — one branch per curve — or `unsolved`
where it finds no conic,
`IntAna_NoGeometricSolution`) and, for every curve it returns, its type
and sampled points, or for a `point` result its points; an `unsolved`
pair also carries the lines `GeomAPI_IntSS` walks, as curves of type
`section`, each sampled and every sample polished onto both surfaces by
Newton steps; a line along which the surfaces are tangent at a sample
has no crossing to polish onto, and the walk of a touch is no ground
truth (1e-4 off the touching circle of a sphere in a cylinder), so it is
dropped and counted under `dropped` — or
`IntAna_IntConicQuad` for a conic against a plane or a quadric, or
`IntAna_IntLinTorus` for a line against a torus: `coincident`, or
`points` with every hit's point and conic parameter, duplicates within
`Precision::Confusion` reported once (a tangent touch comes back twice).
A conic against a torus or an elliptic cylinder has no `IntAna` form and
goes through `GeomAPI_IntCS` below, which has no `coincident`: it reports
a conic lying on one of those two as a cloud of hundreds of points, so a
fixture asks it only where the two cross or touch.

A `nurbs` surface is a `Geom_BSplineSurface` of the recipe's degrees and
knots (flat, as a curve's: each written as often as it repeats),
control net (a row per control point of `u`) and weights, so both
parameters are the recipe's. It is evaluated at parameters and points are
projected onto it, by `GeomAPI_ProjectPointOnSurf`, whose lowest distance
is the nearest point; it is paired with a line only, through
`GeomAPI_IntCS` below — `points` as for a NURBS curve, or `coincident`
where the intersector reports a segment of the line on the surface. A
line along a ruling comes back as a cloud of points along it rather
than a segment, which the oracle test holds to the line Arris calls
`Coincident`.

A `nurbs` curve is a `Geom_BSplineCurve` of the recipe's degree, knots
(flat: each written as often as it repeats), Cartesian control points and
weights, so its parameter is the recipe's. Against a surface it goes
through `GeomAPI_IntCS`, the general curve–surface intersector: `points`
with every hit's point and curve parameter, held to both operands and
deduplicated as the conics' are. It has no `coincident`: a segment of the
curve on the surface is an error here, since no fixture means one.

A pair may also be two curves — one of them a `nurbs` one, or two conics:
Open CASCADE has
no 3D curve–curve intersector, so its `GeomAPI_ExtremaCurveCurve` over
the two curves' domains — a line's over `LINE_REACH` either side of its
origin, a B-spline's span by span, since over the whole domain the search
misses crossings — stands in, every extremum within `Precision::Confusion` a hit:
`points` with each hit's point (the first curve's), its parameter `t` on
the first curve and `tb` on the second, deduplicated as above. It has no
`coincident`: extrema say nothing of two curves that are one, and no
fixture means one.

A pair with `"extrema": true` asks for the stationary pairs of the distance
between two curves instead (`extrema_curves`): `Extrema_ExtCC` over the
same ranges. A curve `a` against a surface `b` with `"extrema": true` asks
for the stationary pairs of the distance between them
(`extrema_curve_surface`): `GeomAPI_ExtremaCurveSurface` over the whole
curve and the whole surface (the elliptic cylinder trimmed to
`EXTRUSION_REACH`), `parallel` with the distance it gives where it sees a
curve along the surface, else `curve_surface` with each pair's curve
parameter `t`, surface `(u, v)`, both points and distance. A NURBS surface
has no form. Two surfaces with `"extrema": true` ask for the stationary
pairs of their distance (`extrema_surfaces`): `GeomAPI_ExtremaSurfaceSurface`
over each surface's whole parameter range (an unbounded direction within
`EXTRUSION_REACH`), `parallel` with the distance where it sees a family,
else `surface_surface` with each pair's `(ua, va)` and `(ub, vb)`, both
points and distance. It reports pairs that are not stationary (a grid
point at a range end), which are dropped and counted.
"""

import math
from typing import Any

from OCP.Geom import (
    Geom_BSplineCurve,
    Geom_BSplineSurface,
    Geom_Circle,
    Geom_ConicalSurface,
    Geom_CylindricalSurface,
    Geom_Ellipse,
    Geom_Hyperbola,
    Geom_Line,
    Geom_Parabola,
    Geom_Plane,
    Geom_RectangularTrimmedSurface,
    Geom_SphericalSurface,
    Geom_SurfaceOfLinearExtrusion,
    Geom_ToroidalSurface,
)
from OCP.Extrema import Extrema_ExtCC, Extrema_POnCurv
from OCP.GeomAdaptor import GeomAdaptor_Curve
from OCP.collections import Array1_double, Array1_gp_Pnt, Array1_int, Array2_double, Array2_gp_Pnt
from OCP.GeomAPI import (
    GeomAPI_ExtremaCurveCurve,
    GeomAPI_ExtremaCurveSurface,
    GeomAPI_ExtremaSurfaceSurface,
    GeomAPI_IntCS,
    GeomAPI_IntSS,
    GeomAPI_ProjectPointOnCurve,
    GeomAPI_ProjectPointOnSurf,
)
from OCP.gp import gp_Ax2, gp_Ax3, gp_Dir, gp_Pnt, gp_Vec
from OCP.IntAna import IntAna_IntConicQuad, IntAna_IntLinTorus, IntAna_QuadQuadGeo, IntAna_Quadric, IntAna_ResultType
from OCP.Precision import Precision

from . import OracleError
from .recipe import number, resolve_params, vector

SURFACE_TYPES = ("plane", "cylinder", "elliptic_cylinder", "cone", "sphere", "torus", "nurbs")
CURVE_TYPES = ("line", "circle", "ellipse", "nurbs")

# How far either side of its origin a line is searched for its extrema
# with another curve: past every curve of a fixture's box.
LINE_REACH = 100.0

# How far either way along its axis an `elliptic_cylinder` is built.
# Open CASCADE has no analytic surface for one, so it is the section
# ellipse extruded along the axis — and the general intersector finds
# nothing at all on an extrusion's unbounded parametric range (±2e100),
# whatever the pose, so the extrusion is trimmed to this band. Its
# parametrisation is the extrusion's, so an evaluation or a projection
# inside the band is unchanged; a fixture keeps its hits well inside it,
# as it keeps a line's within LINE_REACH.
EXTRUSION_REACH = 100.0

# Where a result curve is sampled: lines at these parameters from the
# curve's own origin, closed curves at this many even steps of a turn.
LINE_SAMPLES = (-3.0, -1.0, 0.0, 1.0, 3.0)
TURN_SAMPLES = 8

# An open conic (a parabola, a hyperbola's branch) is sampled at these
# parameters of its own: `Geom_Parabola`'s is the distance along the
# axis of symmetry's normal, `Geom_Hyperbola`'s the `u` of `cosh u`.
OPEN_CONIC_SAMPLES = (-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0)

# A walked section line of `GeomAPI_IntSS` is sampled at this many even
# steps of its parameter, both ends included, and each sample is polished
# onto both surfaces by at most `POLISH_STEPS` Newton steps, until it is
# within `POLISHED` of each (the walked line itself is only within its
# approximation tolerance, about 1e-8 here, which is no ground truth for a
# curve held to a fraction of 1e-7).
SECTION_SAMPLES = 17
POLISH_STEPS = 20
POLISHED = 1e-14
# `1 − (nₐ·n_b)²` at or below which the normals are parallel: `sin²` of an
# angle of 1e-6, where a Newton step would move a point by the distances
# over 1e-12.
TANGENT = 1e-12


def _pnt(v: list[float]) -> gp_Pnt:
    return gp_Pnt(v[0], v[1], v[2])


def _dir(v: list[float], what: str) -> gp_Dir:
    if math.hypot(*v) == 0.0:
        raise OracleError(f"{what}: zero direction")
    return gp_Dir(v[0], v[1], v[2])


def _ax3(spec: dict, params: dict[str, float], what: str) -> gp_Ax3:
    return gp_Ax3(
        _pnt(vector(spec["origin"], params)),
        _dir(vector(spec["z"], params), f"{what} z"),
        _dir(vector(spec["x"], params), f"{what} x"),
    )


def _positive(spec: dict, key: str, params: dict[str, float], what: str) -> float:
    v = number(spec[key], params)
    if not v > 0.0:
        raise OracleError(f"{what}: {key} must be positive")
    return v


def build_surface(name: str, spec: dict, params: dict[str, float]):
    """The `Geom_Surface` of a surface spec."""
    kind = spec.get("type")
    if kind == "nurbs":
        return _bspline_surface(name, spec, params)
    ax = _ax3(spec, params, name)
    if kind == "plane":
        return Geom_Plane(ax)
    if kind == "cylinder":
        return Geom_CylindricalSurface(ax, _positive(spec, "radius", params, name))
    if kind == "cone":
        angle = math.radians(number(spec["half_angle_deg"], params))
        if not 0.0 < angle < math.pi / 2:
            raise OracleError(f"{name}: half_angle_deg must be inside (0, 90)")
        return Geom_ConicalSurface(ax, angle, _positive(spec, "radius", params, name))
    if kind == "elliptic_cylinder":
        # Open CASCADE has no elliptic cylinder: the section ellipse
        # extruded along the axis is the same point set, and the general
        # intersector takes it where `IntAna` has no quadric for it.
        big = _positive(spec, "major_radius", params, name)
        small = _positive(spec, "minor_radius", params, name)
        if small > big:
            raise OracleError(f"{name}: minor_radius must not exceed major_radius")
        section = Geom_Ellipse(gp_Ax2(ax.Location(), ax.Direction(), ax.XDirection()), big, small)
        extrusion = Geom_SurfaceOfLinearExtrusion(section, ax.Direction())
        return Geom_RectangularTrimmedSurface(extrusion, 0.0, 2.0 * math.pi, -EXTRUSION_REACH, EXTRUSION_REACH, True, True)
    if kind == "sphere":
        return Geom_SphericalSurface(ax, _positive(spec, "radius", params, name))
    if kind == "torus":
        big = _positive(spec, "major_radius", params, name)
        small = _positive(spec, "minor_radius", params, name)
        if small >= big:
            raise OracleError(f"{name}: minor_radius must be below major_radius")
        return Geom_ToroidalSurface(ax, big, small)
    raise OracleError(f"{name}: unknown surface type {kind!r}")


def build_curve(name: str, spec: dict, params: dict[str, float]):
    """The `Geom_Curve` of a curve spec."""
    kind = spec.get("type")
    if kind == "line":
        return Geom_Line(_pnt(vector(spec["origin"], params)), _dir(vector(spec["direction"], params), name))
    if kind == "nurbs":
        return _bspline(name, spec, params)
    ax = _ax3(spec, params, name)
    ax2 = gp_Ax2(ax.Location(), ax.Direction(), ax.XDirection())
    if kind == "circle":
        return Geom_Circle(ax2, _positive(spec, "radius", params, name))
    if kind == "ellipse":
        big = _positive(spec, "major_radius", params, name)
        small = _positive(spec, "minor_radius", params, name)
        if small > big:
            raise OracleError(f"{name}: minor_radius must not exceed major_radius")
        return Geom_Ellipse(ax2, big, small)
    raise OracleError(f"{name}: unknown curve type {kind!r}")


def _bspline(name: str, spec: dict, params: dict[str, float]):
    """The `Geom_BSplineCurve` of a `nurbs` spec: the flat knots folded into
    distinct values and multiplicities, which is how it takes them."""
    degree = spec["degree"]
    flat = [number(k, params) for k in spec["knots"]]
    points = [vector(p, params) for p in spec["control_points"]]
    weights = [number(w, params) for w in spec["weights"]]
    if not isinstance(degree, int) or degree < 1:
        raise OracleError(f"{name}: degree must be a positive integer")
    if len(weights) != len(points) or len(flat) != len(points) + degree + 1:
        raise OracleError(f"{name}: {len(flat)} knots and {len(weights)} weights for {len(points)} control points of degree {degree}")
    if any(b < a for a, b in zip(flat, flat[1:])) or any(not w > 0.0 for w in weights):
        raise OracleError(f"{name}: knots must not decrease and weights must be positive")
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


def _fold_knots(flat: list[float]) -> tuple[list[float], list[int]]:
    """Flat knots as distinct values and multiplicities."""
    values: list[float] = []
    counts: list[int] = []
    for k in flat:
        if values and values[-1] == k:
            counts[-1] += 1
        else:
            values.append(k)
            counts.append(1)
    return values, counts


def _bspline_surface(name: str, spec: dict, params: dict[str, float]):
    """The `Geom_BSplineSurface` of a `nurbs` surface spec."""
    degree = spec["degree"]
    net = [[vector(p, params) for p in row] for row in spec["control_points"]]
    wts = [[number(w, params) for w in row] for row in spec["weights"]]
    flats = [[number(k, params) for k in ks] for ks in spec["knots"]]
    if not (isinstance(degree, list) and len(degree) == 2 and all(isinstance(d, int) and d >= 1 for d in degree)):
        raise OracleError(f"{name}: degree must be two positive integers")
    n, m = len(net), len(net[0]) if net else 0
    if any(len(row) != m for row in net) or [len(r) for r in wts] != [m] * n:
        raise OracleError(f"{name}: the control net and the weights must both be {n} x {m}")
    if [len(k) for k in flats] != [n + degree[0] + 1, m + degree[1] + 1]:
        raise OracleError(f"{name}: {len(flats[0])} and {len(flats[1])} knots for a {n} x {m} net of degrees {degree}")
    if any(b < a for ks in flats for a, b in zip(ks, ks[1:])) or any(not w > 0.0 for row in wts for w in row):
        raise OracleError(f"{name}: knots must not decrease and weights must be positive")
    poles = Array2_gp_Pnt(1, n, 1, m)
    weights = Array2_double(1, n, 1, m)
    for i in range(n):
        for j in range(m):
            poles.SetValue(i + 1, j + 1, _pnt(net[i][j]))
            weights.SetValue(i + 1, j + 1, wts[i][j])
    folded = [_fold_knots(ks) for ks in flats]
    arrays = []
    for values, counts in folded:
        k, c = Array1_double(1, len(values)), Array1_int(1, len(values))
        for i, (v, mult) in enumerate(zip(values, counts)):
            k.SetValue(i + 1, v)
            c.SetValue(i + 1, mult)
        arrays.append((k, c))
    return Geom_BSplineSurface(poles, weights, arrays[0][0], arrays[1][0], arrays[0][1], arrays[1][1], degree[0], degree[1])


def _xyz(p) -> list[float]:
    return [p.X(), p.Y(), p.Z()]


def evaluate_surface(surface, u: float, v: float) -> dict:
    p, du, dv, duu, dvv, duv = gp_Pnt(), gp_Vec(), gp_Vec(), gp_Vec(), gp_Vec(), gp_Vec()
    surface.D2(u, v, p, du, dv, duu, dvv, duv)
    return {"at": [u, v], "point": _xyz(p), "du": _xyz(du), "dv": _xyz(dv), "duu": _xyz(duu), "duv": _xyz(duv), "dvv": _xyz(dvv)}


def evaluate_curve(curve, t: float) -> dict:
    p, d1, d2 = gp_Pnt(), gp_Vec(), gp_Vec()
    curve.D2(t, p, d1, d2)
    return {"at": t, "point": _xyz(p), "d1": _xyz(d1), "d2": _xyz(d2)}


def project_surface(surface, point: list[float]) -> dict:
    proj = GeomAPI_ProjectPointOnSurf(_pnt(point), surface)
    if proj.NbPoints() == 0:
        raise OracleError(f"projection of {point} found no point")
    u, v = proj.LowerDistanceParameters()
    return {"point": point, "uv": [u, v], "nearest": _xyz(proj.NearestPoint()), "distance": proj.LowerDistance()}


def project_curve(curve, point: list[float]) -> dict:
    proj = GeomAPI_ProjectPointOnCurve(_pnt(point), curve)
    if proj.NbPoints() == 0:
        raise OracleError(f"projection of {point} found no point")
    return {"point": point, "t": proj.LowerDistanceParameter(), "nearest": _xyz(proj.NearestPoint()), "distance": proj.LowerDistance()}


def _sample_line(line) -> dict:
    loc, d = line.Location(), line.Direction()
    return {"type": "line", "points": [[loc.X() + t * d.X(), loc.Y() + t * d.Y(), loc.Z() + t * d.Z()] for t in LINE_SAMPLES]}


def _sample_closed(kind: str, conic) -> dict:
    curve = Geom_Circle(conic) if kind == "circle" else Geom_Ellipse(conic)
    return {"type": kind, "points": [_xyz(curve.Value(2.0 * math.pi * i / TURN_SAMPLES)) for i in range(TURN_SAMPLES)]}


def _sample_open(kind: str, curve) -> dict:
    return {"type": kind, "points": [_xyz(curve.Value(u)) for u in OPEN_CONIC_SAMPLES]}


_QUADRIC_OF = {
    "plane": lambda s: s.Pln(),
    "cylinder": lambda s: s.Cylinder(),
    "cone": lambda s: s.Cone(),
    "sphere": lambda s: s.Sphere(),
    "torus": lambda s: s.Torus(),
}

# The overloads of `IntAna_QuadQuadGeo`, by the kinds in the order it
# takes them and the tolerances each takes: an angle and a distance
# (`angular`), a distance alone (`linear`), or none. A pair the other way
# round is swapped before the call; the result does not depend on the order.
_OVERLOADS = {
    ("plane", "plane"): "angular",
    ("plane", "cylinder"): "angular",
    ("plane", "cone"): "angular",
    ("plane", "sphere"): "none",
    ("plane", "torus"): "linear",
    ("cylinder", "cylinder"): "linear",
    ("cylinder", "cone"): "linear",
    ("cylinder", "sphere"): "linear",
    ("cylinder", "torus"): "linear",
    ("cone", "cone"): "linear",
    ("cone", "torus"): "linear",
    ("sphere", "cone"): "linear",
    ("sphere", "sphere"): "linear",
    ("sphere", "torus"): "linear",
    ("torus", "torus"): "linear",
}


def intersect_surfaces(name_a: str, kind_a: str, a, name_b: str, kind_b: str, b) -> dict:
    """`IntAna_QuadQuadGeo` on the two `gp` quadrics, and `GeomAPI_IntSS` on
    the two surfaces where it finds no conic."""
    if "nurbs" in (kind_a, kind_b):
        # `IntAna` has no form for a NURBS surface: the general intersector
        # walks it, as it walks a quadric pair with no conic.
        sections, dropped = _walked_sections(name_a, a, name_b, b)
        out = {"a": name_a, "b": name_b, "type": "unsolved"}
        if sections:
            out["curves"] = sections
        if dropped:
            out["dropped"] = dropped
        return out
    if kind_a not in _QUADRIC_OF or kind_b not in _QUADRIC_OF:
        raise OracleError(f"{name_a} vs {name_b}: no gp quadric for {kind_a}–{kind_b}")
    qa, qb = _QUADRIC_OF[kind_a](a), _QUADRIC_OF[kind_b](b)
    if (kind_a, kind_b) in _OVERLOADS:
        tolerances = _OVERLOADS[(kind_a, kind_b)]
    elif (kind_b, kind_a) in _OVERLOADS:
        tolerances = _OVERLOADS[(kind_b, kind_a)]
        qa, qb = qb, qa
    else:
        raise OracleError(f"{name_a} vs {name_b}: no IntAna_QuadQuadGeo for {kind_a}–{kind_b}")
    try:
        if tolerances == "angular":
            r = IntAna_QuadQuadGeo(qa, qb, Precision.Angular_s(), Precision.Confusion_s())
        elif tolerances == "linear":
            r = IntAna_QuadQuadGeo(qa, qb, Precision.Confusion_s())
        else:
            r = IntAna_QuadQuadGeo(qa, qb)
    except TypeError as e:
        raise OracleError(f"{name_a} vs {name_b}: no IntAna_QuadQuadGeo for {kind_a}–{kind_b}: {e}") from e
    if not r.IsDone():
        raise OracleError(f"{name_a} vs {name_b}: IntAna_QuadQuadGeo not done")
    t = r.TypeInter()
    out: dict[str, Any] = {"a": name_a, "b": name_b}
    if t == IntAna_ResultType.IntAna_Empty:
        out["type"] = "empty"
    elif t == IntAna_ResultType.IntAna_Same:
        out["type"] = "coincident"
    elif t == IntAna_ResultType.IntAna_Point:
        out["type"] = "point"
        out["points"] = [_xyz(r.Point(i + 1)) for i in range(r.NbSolutions())]
    elif t == IntAna_ResultType.IntAna_Line:
        out["type"] = "line"
        out["curves"] = [_sample_line(r.Line(i + 1)) for i in range(r.NbSolutions())]
    elif t == IntAna_ResultType.IntAna_Circle:
        out["type"] = "circle"
        out["curves"] = [_sample_closed("circle", r.Circle(i + 1)) for i in range(r.NbSolutions())]
    elif t == IntAna_ResultType.IntAna_Ellipse:
        out["type"] = "ellipse"
        out["curves"] = [_sample_closed("ellipse", r.Ellipse(i + 1)) for i in range(r.NbSolutions())]
    elif t == IntAna_ResultType.IntAna_Parabola:
        out["type"] = "parabola"
        out["curves"] = [_sample_open("parabola", Geom_Parabola(r.Parabola(i + 1))) for i in range(r.NbSolutions())]
    elif t == IntAna_ResultType.IntAna_Hyperbola:
        out["type"] = "hyperbola"
        out["curves"] = [_sample_open("hyperbola", Geom_Hyperbola(r.Hyperbola(i + 1))) for i in range(r.NbSolutions())]
    elif t == IntAna_ResultType.IntAna_NoGeometricSolution:
        out["type"] = "unsolved"
        sections, dropped = _walked_sections(name_a, a, name_b, b)
        if sections:
            out["curves"] = sections
        if dropped:
            out["dropped"] = dropped
    else:
        raise OracleError(f"{name_a} vs {name_b}: unexpected IntAna result {t}")
    return out


def _walked_sections(name_a: str, a, name_b: str, b) -> tuple[list[dict], int]:
    """`GeomAPI_IntSS` on the two surfaces: every line it walks, sampled at
    `SECTION_SAMPLES` parameters and each sample polished onto both
    surfaces, as a curve of type `section`; and how many lines were
    dropped for a touch along them."""
    r = GeomAPI_IntSS(a, b, Precision.Confusion_s())
    if not r.IsDone():
        raise OracleError(f"{name_a} vs {name_b}: GeomAPI_IntSS not done")
    out = []
    dropped = 0
    for i in range(r.NbLines()):
        line = r.Line(i + 1)
        t0, t1 = line.FirstParameter(), line.LastParameter()
        if not (math.isfinite(t0) and math.isfinite(t1)):
            raise OracleError(f"{name_a} vs {name_b}: an unbounded walked line")
        walked = []
        for k in range(SECTION_SAMPLES):
            p = line.Value(t0 + (t1 - t0) * k / (SECTION_SAMPLES - 1))
            walked.append([p.X(), p.Y(), p.Z()])
        polished = [_polished(name_a, a, name_b, b, x) for x in walked]
        if any(x is None for x in polished):
            dropped += 1
        else:
            out.append({"type": "section", "points": polished})
    return out, dropped


def _signed(surface, x: list[float]) -> tuple[float, list[float]]:
    """The signed distance of `x` from `surface` along its unit normal at
    the nearest point, and that normal."""
    proj = GeomAPI_ProjectPointOnSurf(_pnt(x), surface)
    if proj.NbPoints() == 0:
        raise OracleError(f"projection of {x} found no point")
    u, v = proj.LowerDistanceParameters()
    p, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
    surface.D1(u, v, p, du, dv)
    n = du.Crossed(dv)
    n.Normalize()
    normal = [n.X(), n.Y(), n.Z()]
    return sum((x[i] - _xyz(p)[i]) * normal[i] for i in range(3)), normal


def _polished(name_a: str, a, name_b: str, b, x: list[float]) -> list[float] | None:
    """`x` moved onto both surfaces by Newton steps on their signed
    distances: each step the smallest move that zeroes both to first
    order, `δ = −[nₐ n_b] G⁻¹ [fₐ f_b]` with `G` the normals' Gram
    matrix. `None` where the normals are parallel: the surfaces touch
    there, and a touch has no crossing to converge onto."""
    for _ in range(POLISH_STEPS):
        fa, na = _signed(a, x)
        fb, nb = _signed(b, x)
        if abs(fa) <= POLISHED and abs(fb) <= POLISHED:
            return x
        c = sum(na[i] * nb[i] for i in range(3))
        det = 1.0 - c * c
        if det <= TANGENT:
            return None
        ka = (fa - c * fb) / det
        kb = (fb - c * fa) / det
        x = [x[i] - ka * na[i] - kb * nb[i] for i in range(3)]
    raise OracleError(f"{name_a} vs {name_b}: a walked point did not polish onto both surfaces: {x}")


def intersect_curve_surface(name_c: str, kind_c: str, curve, name_s: str, kind_s: str, surface) -> dict:
    """`IntAna_IntConicQuad` of the `gp` conic against the plane or the
    quadric, and `IntAna_IntLinTorus` of a line against a torus."""
    out: dict[str, Any] = {"a": name_c, "b": name_s}
    # `IntAna` has no form for a NURBS curve, nor for a conic against a
    # torus or an elliptic cylinder; the general intersector has. It
    # reports a curve *lying on* the surface as a cloud of points rather
    # than a segment, so a fixture asks it only where the two cross or
    # touch — a conic on one of those two is pinned in the Rust tests.
    general = (
        kind_c == "nurbs"
        or kind_s in ("elliptic_cylinder", "nurbs")
        or (kind_c != "line" and kind_s == "torus")
    )
    if general:
        r = GeomAPI_IntCS(curve, surface)
        if not r.IsDone():
            raise OracleError(f"{name_c} vs {name_s}: GeomAPI_IntCS not done")
        if r.NbSegments() != 0:
            # A line along a ruling of a NURBS surface: the intersector
            # reports the stretch on it as a segment.
            if kind_s == "nurbs":
                out["type"] = "coincident"
                return out
            raise OracleError(f"{name_c} vs {name_s}: a segment of the curve on the surface")
        out["type"] = "points"
        found = []
        for i in range(r.NbPoints()):
            _u, _v, t = r.Parameters(i + 1)
            found.append((_xyz(r.Point(i + 1)), t))
        return _hits(out, curve, surface, found)
    conic = {"line": lambda c: c.Lin(), "circle": lambda c: c.Circ(), "ellipse": lambda c: c.Elips()}[kind_c](curve)
    if kind_s == "torus":
        if kind_c != "line":
            raise OracleError(f"{name_c} vs {name_s}: no intersector for {kind_c}–torus")
        r = IntAna_IntLinTorus(conic, surface.Torus())
        if not r.IsDone():
            raise OracleError(f"{name_c} vs {name_s}: IntAna_IntLinTorus not done")
        out["type"] = "points"
        return _hits(out, curve, surface, [(_xyz(r.Value(i + 1)), r.ParamOnLine(i + 1)) for i in range(r.NbPoints())])
    try:
        if kind_s == "plane":
            r = IntAna_IntConicQuad(conic, surface.Pln(), Precision.Angular_s(), Precision.Confusion_s())
        else:
            r = IntAna_IntConicQuad(conic, IntAna_Quadric(_QUADRIC_OF[kind_s](surface)))
    except TypeError as e:
        raise OracleError(f"{name_c} vs {name_s}: no IntAna_IntConicQuad for {kind_c}–{kind_s}: {e}") from e
    if not r.IsDone():
        raise OracleError(f"{name_c} vs {name_s}: IntAna_IntConicQuad not done")
    if r.IsInQuadric():
        out["type"] = "coincident"
        return out
    out["type"] = "points"
    found = [] if r.IsParallel() else [(_xyz(r.Point(i + 1)), r.ParamOnConic(i + 1)) for i in range(r.NbPoints())]
    return _hits(out, curve, surface, found)


def _ranges(kind: str, curve) -> list[tuple[float, float]]:
    """Where a curve is searched: a line `LINE_REACH` either side of its
    origin, a closed conic one turn, a B-spline span by span — over its
    whole domain at once the extrema search misses crossings."""
    if kind == "line":
        return [(-LINE_REACH, LINE_REACH)]
    if kind == "nurbs":
        knots = [curve.Knot(i + 1) for i in range(curve.NbKnots())]
        return list(zip(knots, knots[1:]))
    return [(curve.FirstParameter(), curve.LastParameter())]


def intersect_curves(name_a: str, kind_a: str, a, name_b: str, kind_b: str, b) -> dict:
    """`GeomAPI_ExtremaCurveCurve` of two curves, one of them a B-spline:
    the extrema within `Precision::Confusion` as `points`."""
    out: dict[str, Any] = {"a": name_a, "b": name_b}
    conics = {"circle", "ellipse"}
    if "nurbs" not in (kind_a, kind_b) and not {kind_a, kind_b} <= conics:
        raise OracleError(f"{name_a} vs {name_b}: a curve pair needs a nurbs curve or two conics in it")
    found = []
    for a_lo, a_hi in _ranges(kind_a, a):
        for b_lo, b_hi in _ranges(kind_b, b):
            r = GeomAPI_ExtremaCurveCurve(a, b, a_lo, a_hi, b_lo, b_hi)
            for i in range(r.NbExtrema()):
                if r.Distance(i + 1) > Precision.Confusion_s():
                    continue
                ta, tb = r.Parameters(i + 1)
                pa, pb = gp_Pnt(), gp_Pnt()
                r.Points(i + 1, pa, pb)
                found.append((_xyz(pa), ta, tb))
    hits: list[dict] = []
    for p, ta, tb in found:
        if any(math.dist(p, h["point"]) <= Precision.Confusion_s() for h in hits):
            continue
        hits.append({"point": p, "t": ta, "tb": tb})
    hits.sort(key=lambda h: h["t"])
    out["type"] = "points"
    out["hits"] = hits
    return out


def extrema_curves(name_a: str, kind_a: str, a, name_b: str, kind_b: str, b) -> dict:
    """`Extrema_ExtCC` of two curves, an `"extrema": true` pair: `parallel`
    with the distance where it reports the curves parallel (two lines,
    two coaxial circles, a line along a circle's axis), else `extrema`
    with every stationary pair it finds — each one's distance, both
    parameters and both points — searched over each curve's `_ranges`,
    duplicates within `Precision::Confusion` on both curves reported once,
    ascending by `ta` then `tb`. It is not complete: for two circles it
    may report one pair of the four or more there are, and for a family
    of pairs at one distance that is no common axis it reports points
    sampled along it, so the oracle test holds every pair it reports to
    one of Arris's and Arris's nearest to its, never the count."""
    out: dict[str, Any] = {"a": name_a, "b": name_b}
    found = []
    whole = lambda kind, c: [(-LINE_REACH, LINE_REACH)] if kind == "line" else [(c.FirstParameter(), c.LastParameter())]
    dropped = 0
    for a_lo, a_hi in whole(kind_a, a):
        for b_lo, b_hi in whole(kind_b, b):
            ext = Extrema_ExtCC(GeomAdaptor_Curve(a, a_lo, a_hi), GeomAdaptor_Curve(b, b_lo, b_hi))
            if not ext.IsDone():
                raise OracleError(f"{name_a} vs {name_b}: Extrema_ExtCC did not finish")
            if ext.IsParallel():
                out["type"] = "parallel"
                out["distance"] = math.sqrt(ext.SquareDistance(1))
                return out
            for i in range(ext.NbExt()):
                pa, pb = Extrema_POnCurv(), Extrema_POnCurv()
                ext.Points(i + 1, pa, pb)
                if not (_stationary(a, pa.Parameter(), pb.Value()) and _stationary(b, pb.Parameter(), pa.Value())):
                    dropped += 1
                    continue
                found.append({
                    "ta": pa.Parameter(),
                    "tb": pb.Parameter(),
                    "a": _xyz(pa.Value()),
                    "b": _xyz(pb.Value()),
                    "distance": math.sqrt(ext.SquareDistance(i + 1)),
                })
    kept: list[dict] = []
    for e in found:
        if any(math.dist(e["a"], k["a"]) <= Precision.Confusion_s() and math.dist(e["b"], k["b"]) <= Precision.Confusion_s() for k in kept):
            continue
        kept.append(e)
    kept.sort(key=lambda e: (e["ta"], e["tb"]))
    out["type"] = "extrema"
    out["extrema"] = kept
    if dropped:
        out["dropped"] = dropped
    return out


def extrema_curve_surface(name_a: str, kind_a: str, a, name_b: str, kind_b: str, b) -> dict:
    """`GeomAPI_ExtremaCurveSurface` of a curve and a surface, an
    `"extrema": true` pair: `parallel` with the distance where it reports
    the curve along the surface (it gives the nearer family only), else
    `extrema` with every stationary pair it finds, `curve_surface` — the
    curve's `t`, the surface's `u` and `v`, both points and the distance,
    duplicates within `Precision::Confusion` once, ascending by `t`. Like
    the curve pairs' it is not complete."""
    out: dict[str, Any] = {"a": name_a, "b": name_b}
    if kind_b == "nurbs":
        raise OracleError(f"{name_a} vs {name_b}: no extrema against a NURBS surface")
    r = GeomAPI_ExtremaCurveSurface(a, b)
    if r.IsParallel():
        out["type"] = "parallel"
        out["distance"] = r.LowerDistance()
        return out
    kept: list[dict] = []
    for i in range(r.NbExtrema()):
        t, u, v = r.Parameters(i + 1)
        pa, pb = gp_Pnt(), gp_Pnt()
        r.Points(i + 1, pa, pb)
        e = {"t": t, "u": u, "v": v, "a": _xyz(pa), "b": _xyz(pb), "distance": r.Distance(i + 1)}
        if any(math.dist(e["a"], k["a"]) <= Precision.Confusion_s() and math.dist(e["b"], k["b"]) <= Precision.Confusion_s() for k in kept):
            continue
        kept.append(e)
    kept.sort(key=lambda e: (e["t"], e["u"], e["v"]))
    out["type"] = "extrema"
    out["curve_surface"] = kept
    return out


def _surface_range(kind: str) -> tuple[float, float, float, float]:
    """The `(u0, u1, v0, v1)` a surface of this kind is searched over."""
    turn = 2.0 * math.pi
    reach = EXTRUSION_REACH
    return {
        "plane": (-reach, reach, -reach, reach),
        "cylinder": (0.0, turn, -reach, reach),
        "elliptic_cylinder": (0.0, turn, -reach, reach),
        "cone": (0.0, turn, -reach, reach),
        "sphere": (0.0, turn, -math.pi / 2, math.pi / 2),
        "torus": (0.0, turn, 0.0, turn),
    }[kind]


def _normal_stationary(surface, u: float, v: float, other: gp_Pnt) -> bool:
    """The offset from `other` to the surface's point at `(u, v)` has no
    component along either tangent beyond `Precision::Confusion`: a pair
    of a grid search's that is not along the normals is no extremum."""
    p, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
    surface.D1(u, v, p, du, dv)
    r = gp_Vec(other, p)
    for d in (du, dv):
        n = d.Magnitude()
        if n > 0.0 and abs(r.Dot(d)) / n > Precision.Confusion_s():
            return False
    return True


def extrema_surfaces(name_a: str, kind_a: str, a, name_b: str, kind_b: str, b) -> dict:
    """`GeomAPI_ExtremaSurfaceSurface` of two surfaces, an `"extrema": true`
    pair: `parallel` with the distance where it reports them parallel, else
    `extrema` with every stationary pair it finds as `surface_surface` — the
    first surface's `(ua, va)`, the second's `(ub, vb)`, both points and
    the distance, duplicates within `Precision::Confusion` once, ascending
    by `(ua, va, ub, vb)`. A pair that is not along both surfaces' normals
    is dropped and counted under `dropped`. Like the others it is not
    complete."""
    out: dict[str, Any] = {"a": name_a, "b": name_b}
    if "nurbs" in (kind_a, kind_b):
        raise OracleError(f"{name_a} vs {name_b}: no surface extrema against a NURBS surface")
    r = GeomAPI_ExtremaSurfaceSurface(a, b, *_surface_range(kind_a), *_surface_range(kind_b))
    if r.IsParallel():
        out["type"] = "parallel"
        out["distance"] = r.LowerDistance()
        return out
    kept: list[dict] = []
    dropped = 0
    for i in range(r.NbExtrema()):
        ua, va, ub, vb = r.Parameters(i + 1)
        pa, pb = gp_Pnt(), gp_Pnt()
        r.Points(i + 1, pa, pb)
        if not (_normal_stationary(a, ua, va, pb) and _normal_stationary(b, ub, vb, pa)):
            dropped += 1
            continue
        e = {"ua": ua, "va": va, "ub": ub, "vb": vb, "a": _xyz(pa), "b": _xyz(pb), "distance": r.Distance(i + 1)}
        if any(math.dist(e["a"], k["a"]) <= Precision.Confusion_s() and math.dist(e["b"], k["b"]) <= Precision.Confusion_s() for k in kept):
            continue
        kept.append(e)
    kept.sort(key=lambda e: (e["ua"], e["va"], e["ub"], e["vb"]))
    out["type"] = "extrema"
    out["surface_surface"] = kept
    if dropped:
        out["dropped"] = dropped
    return out


def _stationary(curve, t: float, other: gp_Pnt) -> bool:
    """The offset from `other` to the curve's point at `t` has no component
    along the curve's tangent there beyond `Precision::Confusion`: what a
    range end the search reports as an extremum is not, unless it is one."""
    p, d1 = gp_Pnt(), gp_Vec()
    curve.D1(t, p, d1)
    n = d1.Magnitude()
    if n == 0.0:
        return True
    r = gp_Vec(other, p)
    return abs(r.Dot(d1)) / n <= Precision.Confusion_s()


def _hits(out: dict, curve, surface, found: list) -> dict:
    """`found` as the pair's `hits`: each `(point, t)` on both operands
    within `Precision::Confusion` and not a duplicate of one already kept,
    ascending by `t`; the rest dropped and counted."""
    hits: list[dict] = []
    dropped = 0
    for p, t in found:
        # The line–quadric intersector has no angular tolerance: a line
        # parallel to a cylinder's axis to rounding solves a quadratic with
        # a vanishing leading coefficient and reports a root at 1e16. A hit
        # that is not on both operands within Precision::Confusion is that
        # artefact, dropped and counted.
        on_curve = GeomAPI_ProjectPointOnCurve(_pnt(p), curve)
        on_surface = GeomAPI_ProjectPointOnSurf(_pnt(p), surface)
        if (
            on_curve.NbPoints() == 0
            or on_surface.NbPoints() == 0
            or on_curve.LowerDistance() > Precision.Confusion_s()
            or on_surface.LowerDistance() > Precision.Confusion_s()
        ):
            dropped += 1
            continue
        if any(math.dist(p, h["point"]) <= Precision.Confusion_s() for h in hits):
            continue
        hits.append({"point": p, "t": t})
    hits.sort(key=lambda h: h["t"])
    out["hits"] = hits
    if dropped:
        out["dropped"] = dropped
    return out


def compute_geometry(fixture: dict) -> dict:
    """The `samples` and `pairs` results of a geometry recipe."""
    params = resolve_params(fixture, "default")
    surfaces = fixture.get("surfaces", {})
    curves = fixture.get("curves", {})
    if set(surfaces) & set(curves):
        raise OracleError(f"names used for both a surface and a curve: {sorted(set(surfaces) & set(curves))}")
    built_s = {n: (spec["type"], build_surface(n, spec, params)) for n, spec in surfaces.items()}
    built_c = {n: (spec["type"], build_curve(n, spec, params)) for n, spec in curves.items()}

    samples = []
    for i, sample in enumerate(fixture.get("samples", [])):
        name = sample.get("of")
        result: dict[str, Any] = {"of": name, "evaluations": [], "projections": []}
        if name in built_s:
            _, s = built_s[name]
            for at in sample.get("params", []):
                u, v = vector(at, params, 2)
                result["evaluations"].append(evaluate_surface(s, u, v))
            for p in sample.get("points", []):
                result["projections"].append(project_surface(s, vector(p, params)))
        elif name in built_c:
            _, c = built_c[name]
            for at in sample.get("params", []):
                result["evaluations"].append(evaluate_curve(c, number(at, params)))
            for p in sample.get("points", []):
                result["projections"].append(project_curve(c, vector(p, params)))
        else:
            raise OracleError(f"sample {i}: {name!r} is neither a surface nor a curve")
        samples.append(result)

    pairs = []
    for i, pair in enumerate(fixture.get("pairs", [])):
        a, b = pair.get("a"), pair.get("b")
        if pair.get("extrema"):
            if a in built_c and b in built_s:
                (ka, ca), (kb, sb) = built_c[a], built_s[b]
                pairs.append(extrema_curve_surface(a, ka, ca, b, kb, sb))
                continue
            if a in built_s and b in built_s:
                (ka, sa), (kb, sb) = built_s[a], built_s[b]
                pairs.append(extrema_surfaces(a, ka, sa, b, kb, sb))
                continue
            if not (a in built_c and b in built_c):
                raise OracleError(f"pair {i}: an extrema pair is two curves, a curve and a surface, or two surfaces")
            (ka, ca), (kb, cb) = built_c[a], built_c[b]
            pairs.append(extrema_curves(a, ka, ca, b, kb, cb))
            continue
        if a in built_c and b in built_c:
            (ka, ca), (kb, cb) = built_c[a], built_c[b]
            pairs.append(intersect_curves(a, ka, ca, b, kb, cb))
            continue
        if b not in built_s:
            raise OracleError(f"pair {i}: b {b!r} is not a surface")
        kb, sb = built_s[b]
        if a in built_s:
            ka, sa = built_s[a]
            pairs.append(intersect_surfaces(a, ka, sa, b, kb, sb))
        elif a in built_c:
            ka, ca = built_c[a]
            pairs.append(intersect_curve_surface(a, ka, ca, b, kb, sb))
        else:
            raise OracleError(f"pair {i}: a {a!r} is neither a surface nor a curve")
    return {"samples": samples, "pairs": pairs}
