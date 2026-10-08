#!/usr/bin/env python3
"""generate.py — write the geometry fixtures' `fixture.json` under this
directory: `analytic-eval` (every analytic variant in three poses,
parameters on and off the seam, projections from both sides),
`c1-intersections` (every case of the cycle-1 intersection table in one
committed general pose), `c2-cylinder-pairs` (every pose of the
cylinder–cylinder table in that pose), `c2-quadric-pairs` (the coaxial
pairs with a cone, a sphere or a torus in them, and the pairs any sphere
makes, in that pose), `c3-cylinder-pairs` (the cylinder pairs that
meet in a quartic, in that pose), `c3-quadric-pairs` (the pairs with
a cone or a sphere in them that share no axis, in that pose),
`c3-torus-pairs` (the pairs a torus makes with a surface sharing no axis
with it, in that pose) and
`c3-conic-hits` (circles and ellipses against the four surfaces a conic
has no exact section of, and coplanar conic pairs with an ellipse among
them, in that pose),
`c3-nurbs-hits` (NURBS curves against every analytic surface the grammar
has, in that pose), `c3-nurbs-crossings` (the same curves against
lines, circles and ellipses, in that pose), `c4-nurbs-projections`
(points projected onto free-form NURBS surfaces with no closed form, in
that pose), `c4-closed-curve-hits` (a closed B-spline met at its join)
and `c8-line-nurbs-hits` (lines against NURBS surfaces: crossings on and
off knot lines, a pole, a graze, a ruling and a miss, in that pose).
Plain Python, no Open CASCADE: the coordinates
are the closed forms of `docs/DATA-MODEL.md` §Geometry written out in
world space at full precision, so both sides read identical numbers.
Rerun after editing, then `tools/oracle/expected.py` on each directory.

The poses: `world` (the identity), `swap` (`z` along +y, `x` along +z,
moved), and `tilt`, a rotation with rational entries (rows of the
orthogonal matrix `[[2, 3, 6], [3, −6, 2], [6, 2, −3]] / 7`) at an
off-centre origin, so nothing is aligned and every number is committed.
"""

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
TAU = 2.0 * math.pi


# --- small vector helpers -----------------------------------------------------


def add(a, b):
    return [x + y for x, y in zip(a, b)]


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def mul(s, a):
    return [s * x for x in a]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    return mul(1.0 / norm(a), a)


class Frame:
    """origin, x, y, z with y = z × x; x is made perpendicular to z."""

    def __init__(self, origin, z, x):
        self.origin = list(map(float, origin))
        self.z = unit(z)
        x = sub(x, mul(dot(x, self.z), self.z))
        self.x = unit(x)
        self.y = cross(self.z, self.x)

    def to_world(self, p):
        return add(self.origin, add(mul(p[0], self.x), add(mul(p[1], self.y), mul(p[2], self.z))))

    def vec(self, v):
        return add(mul(v[0], self.x), add(mul(v[1], self.y), mul(v[2], self.z)))

    def spec(self, **rest):
        return {"origin": self.origin, "z": self.z, "x": self.x, **rest}

    def around(self, phase):
        """The direction at `phase` about z, and the one a quarter turn on."""
        a = add(mul(math.cos(phase), self.x), mul(math.sin(phase), self.y))
        return a, cross(self.z, a)


POSES = {
    "world": Frame([0, 0, 0], [0, 0, 1], [1, 0, 0]),
    "swap": Frame([1, 2, 3], [0, 1, 0], [0, 0, 1]),
    "tilt": Frame([-2.5, 1.75, 0.5], [2 / 7, 3 / 7, 6 / 7], [3 / 7, -6 / 7, 2 / 7]),
}


# --- closed forms (docs/DATA-MODEL.md §Geometry) -----------------------------


def surface_point(kind, f, dims, u, v):
    su, cu, sv, cv = math.sin(u), math.cos(u), math.sin(v), math.cos(v)
    if kind == "plane":
        local = [u, v, 0.0]
    elif kind == "cylinder":
        r = dims["radius"]
        local = [r * cu, r * su, v]
    elif kind == "cone":
        r, a = dims["radius"], math.radians(dims["half_angle_deg"])
        rho = r + v * math.sin(a)
        local = [rho * cu, rho * su, v * math.cos(a)]
    elif kind == "sphere":
        r = dims["radius"]
        local = [r * cv * cu, r * cv * su, r * sv]
    elif kind == "torus":
        big, small = dims["major_radius"], dims["minor_radius"]
        rho = big + small * cv
        local = [rho * cu, rho * su, small * sv]
    else:
        raise ValueError(kind)
    return f.to_world(local)


def surface_normal(kind, f, dims, u, v):
    su, cu, sv, cv = math.sin(u), math.cos(u), math.sin(v), math.cos(v)
    radial = [cu, su, 0.0]
    if kind == "plane":
        local = [0.0, 0.0, 1.0]
    elif kind == "cylinder":
        local = radial
    elif kind == "cone":
        a = math.radians(dims["half_angle_deg"])
        local = [math.cos(a) * cu, math.cos(a) * su, -math.sin(a)]
    elif kind == "sphere":
        local = [cv * cu, cv * su, sv]
    elif kind == "torus":
        local = [cv * cu, cv * su, sv]
    else:
        raise ValueError(kind)
    return f.vec(local)


def curve_point(kind, f, dims, t):
    st, ct = math.sin(t), math.cos(t)
    if kind == "circle":
        r = dims["radius"]
        return f.to_world([r * ct, r * st, 0.0])
    if kind == "ellipse":
        a, b = dims["major_radius"], dims["minor_radius"]
        return f.to_world([a * ct, b * st, 0.0])
    raise ValueError(kind)


def curve_in_plane_normal(kind, f, dims, t):
    """The outward in-plane normal of a circle or an ellipse at t."""
    st, ct = math.sin(t), math.cos(t)
    if kind == "circle":
        return f.vec([ct, st, 0.0])
    a, b = dims["major_radius"], dims["minor_radius"]
    return unit(f.vec([b * ct, a * st, 0.0]))


# --- analytic-eval --------------------------------------------------------------

SURFACE_DIMS = {
    "plane": {},
    "cylinder": {"radius": 2.0},
    "cone": {"radius": 2.0, "half_angle_deg": 30.0},
    "sphere": {"radius": 2.5},
    "torus": {"major_radius": 4.0, "minor_radius": 1.5},
}
CURVE_DIMS = {
    "line": {},
    "circle": {"radius": 2.0},
    "ellipse": {"major_radius": 3.0, "minor_radius": 2.0},
}
# Parameters: on the seam, just past it, mid-way, near the far end of the
# turn; v across the domain of every variant (the sphere's latitude for
# all, plus a pole for the sphere).
U_SAMPLES = [0.0, 0.7, 2.9, TAU - 0.3]
V_SAMPLES = {
    "plane": [-3.0, 0.0, 1.5, 4.0],
    "cylinder": [-3.0, 0.0, 1.5, 4.0],
    "cone": [-1.0, 0.0, 1.5, 4.0],
    "sphere": [-1.2, 0.0, 0.8, math.pi / 2],
    "torus": [0.0, 1.0, 2.9, TAU - 0.4],
}
# Projections: a point at (u, v) moved along the normal by ± this, both
# sides of the surface, at parameters off every ambiguous locus.
OFFSETS = [0.5, -0.5]
PROJECTION_PARAMS = {
    "plane": [(0.0, 0.0), (1.3, -2.1), (-4.0, 3.5)],
    "cylinder": [(0.0, 1.0), (1.3, -2.1), (5.2, 3.5)],
    "cone": [(0.0, 1.0), (1.3, 2.5), (5.2, 3.5)],
    "sphere": [(0.0, 0.3), (1.3, -0.6), (5.2, 1.1)],
    "torus": [(0.0, 0.6), (1.3, 2.2), (5.2, 4.4)],
}
T_SAMPLES = [0.0, 0.7, 2.9, TAU - 0.3]
CURVE_PROJECTION_PARAMS = [0.0, 1.1, 4.0]


def analytic_eval():
    surfaces, curves, samples = {}, {}, []
    for kind, dims in SURFACE_DIMS.items():
        for pose, f in POSES.items():
            name = f"{kind}_{pose}"
            surfaces[name] = {"type": kind, **f.spec(**dims)}
            params = [[u, v] for v in V_SAMPLES[kind] for u in U_SAMPLES]
            points = [
                add(surface_point(kind, f, dims, u, v), mul(d, surface_normal(kind, f, dims, u, v)))
                for (u, v) in PROJECTION_PARAMS[kind]
                for d in OFFSETS
            ]
            samples.append({"of": name, "params": params, "points": points})
    for kind, dims in CURVE_DIMS.items():
        for pose, f in POSES.items():
            name = f"{kind}_{pose}"
            if kind == "line":
                curves[name] = {"type": "line", "origin": f.origin, "direction": f.z}
                params = [-3.0, 0.0, 1.5, 4.0]
                points = [f.to_world([a, b, t]) for (a, b, t) in [(0.0, 0.0, 1.0), (1.5, -2.0, -3.0), (-0.5, 4.0, 2.5)]]
            else:
                curves[name] = {"type": kind, **f.spec(**dims)}
                params = T_SAMPLES
                points = [
                    add(curve_point(kind, f, dims, t), add(mul(d, curve_in_plane_normal(kind, f, dims, t)), mul(h, f.z)))
                    for t in CURVE_PROJECTION_PARAMS
                    for (d, h) in [(0.5, 1.0), (-0.5, -2.0)]
                ]
            samples.append({"of": name, "params": params, "points": points})
    return {
        "kind": "geometry",
        "description": "every analytic surface and curve in three poses (world, swap, tilt): D2 at parameters on and off the seam and across v, and projections of points displaced from the surface along its normal to both sides, off every ambiguous locus; written by generate.py",
        "surfaces": surfaces,
        "curves": curves,
        "samples": samples,
        "pairs": [],
    }


# --- c1-intersections --------------------------------------------------------------


def c1_intersections():
    f = POSES["tilt"]
    R = 2.0
    z = f.z
    surfaces = {"cyl": {"type": "cylinder", **f.spec(radius=R)}}
    curves = {}
    pairs = []

    def plane(name, origin, normal, x_hint):
        surfaces[name] = {"type": "plane", **Frame(origin, normal, x_hint).spec()}

    def line(name, origin, direction):
        curves[name] = {"type": "line", "origin": origin, "direction": direction}

    def circle(name, origin, normal, x_hint, radius):
        curves[name] = {"type": "circle", **Frame(origin, normal, x_hint).spec(radius=radius)}

    def ellipse(name, origin, normal, x_hint, major, minor):
        curves[name] = {
            "type": "ellipse",
            **Frame(origin, normal, x_hint).spec(major_radius=major, minor_radius=minor),
        }

    def pair(a, b):
        pairs.append({"a": a, "b": b})

    a, across = f.around(0.6)
    on_axis = f.to_world([0.0, 0.0, 1.5])
    # Plane–cylinder: the case table of step 4.
    plane("cap", add(on_axis, mul(0.7, a)), z, a)
    tilt = 0.5
    plane("oblique", add(on_axis, mul(0.3, across)), add(mul(math.cos(tilt), z), mul(math.sin(tilt), a)), across)
    plane("chordal", add(add(on_axis, mul(0.4 * R, a)), mul(2.0, across)), a, z)
    plane("touching", add(add(on_axis, mul(R, a)), mul(-1.5, across)), a, z)
    plane("clear", add(add(on_axis, mul(1.6 * R, a)), mul(0.5, across)), a, z)
    for name in ["cap", "oblique", "chordal", "touching", "clear"]:
        pair(name, "cyl")
    # Plane–plane: crossing, parallel apart, the same plane flipped.
    plane("cap_lifted", add(on_axis, mul(2.5, z)), z, across)
    plane("cap_flipped", add(on_axis, mul(-1.3, across)), mul(-1.0, z), a)
    pair("cap", "oblique")
    pair("cap", "cap_lifted")
    pair("cap", "cap_flipped")
    # Line–plane against the cap: crossing, parallel above, in the plane.
    cap_point = add(on_axis, mul(-0.8, across))
    d = unit(add(mul(math.cos(0.9), a), mul(math.sin(0.9), z)))
    line("skewer", sub(cap_point, mul(2.2, d)), d)
    line("hover", add(cap_point, mul(1.1, z)), unit(add(a, across)))
    line("flat", cap_point, unit(sub(a, mul(0.5, across))))
    for name in ["skewer", "hover", "flat"]:
        pair(name, "cap")
    # Line–cylinder: two hits, a touch, a miss, a ruling, parallel and off.
    b, bcross = f.around(2.3)
    d = unit(add(mul(math.cos(0.4), bcross), mul(math.sin(0.4), z)))
    line("chord", sub(add(on_axis, mul(0.45 * R, b)), mul(3.0, d)), d)
    line("grazing", sub(add(on_axis, mul(R, b)), mul(-1.7, d)), d)
    line("miss", sub(add(on_axis, mul(1.8 * R, b)), mul(0.6, d)), d)
    line("ruling", add(f.to_world([0.0, 0.0, -2.0]), mul(R, b)), z)
    line("offside", add(f.to_world([0.0, 0.0, -2.0]), mul(0.5 * R, b)), z)
    for name in ["chord", "grazing", "miss", "ruling", "offside"]:
        pair(name, "cyl")
    # Circle–plane against the cap: reach M = r sin(tilt); centre height h.
    r, tilt = 3.0, 0.8
    c_axis = add(mul(math.cos(tilt), z), mul(math.sin(tilt), a))
    reach = r * math.sin(tilt)
    cap_origin = surfaces["cap"]["origin"]
    for name, h in [("crossing_ring", 0.35 * reach), ("kissing_ring", -reach), ("hovering_ring", 1.4 * reach)]:
        circle(name, add(cap_origin, add(mul(h, z), mul(0.9, across))), c_axis, across, r)
        pair(name, "cap")
    circle("lying_ring", add(cap_origin, mul(-1.2, a)), z, across, r)
    circle("floating_ring", add(cap_origin, add(mul(0.9, z), mul(1.1, a))), z, across, r)
    pair("lying_ring", "cap")
    pair("floating_ring", "cap")
    # Circle–cylinder: a parallel, a meridional circle (four), and in the
    # plane across the axis two crossings, a touch from outside and from
    # inside, a miss outside and one inside.
    c, ccross = f.around(4.0)
    high = f.to_world([0.0, 0.0, 2.75])
    circle("parallel", high, z, c, R)
    circle("meridional", high, ccross, c, 1.6 * R)
    small = 1.2
    for name, delta in [
        ("two", abs(small - R) + 0.55 * (small + R - abs(small - R))),
        ("outside_touch", small + R),
        ("inside_touch", R - small),
        ("outside_miss", 1.3 * (small + R)),
        ("inside_miss", 0.4 * (R - small)),
    ]:
        circle(name, add(high, mul(delta, c)), z, c, small)
    for name in ["parallel", "meridional", "two", "outside_touch", "inside_touch", "outside_miss", "inside_miss"]:
        pair(name, "cyl")
    # Ellipse–plane and ellipse–cylinder, around the oblique section of
    # the cylinder — the ellipse a boolean's section edge on a cylinder
    # wall actually is. Its closed form is docs/DATA-MODEL.md §Curves:
    # centred at the piercing point of the axis, minor axis R across the
    # axis, major axis R / |n·Z| along the axis's projection on the plane.
    n = surfaces["oblique"]["z"]
    cos = abs(dot(n, z))
    piercing = add(f.origin, mul(dot(n, sub(surfaces["oblique"]["origin"], f.origin)) / dot(n, z), z))
    ellipse("section", piercing, n, z, R / cos, R)
    section = Frame(piercing, n, z)
    pair("section", "cyl")
    pair("section", "oblique")
    # Three planes normal to the section's major axis: through the centre
    # (the two ends of the minor axis), at the end of the major axis (one
    # touch), and clear of it.
    for name, along, x_hint in [
        ("cut_minor", 0.0, section.z),
        ("touch_major", R / cos, section.z),
        ("clear_major", 1.5 * R / cos, section.z),
    ]:
        plane(name, add(piercing, mul(along, section.x)), section.x, x_hint)
        pair("section", name)
    # An ellipse in a plane through the axis: four crossings when its
    # reach across the axis exceeds R, two touches when it equals R.
    e, ecross = f.around(5.2)
    mid = f.to_world([0.0, 0.0, -0.75])
    ellipse("meridional_ellipse", mid, ecross, e, 1.6 * R, 1.2 * R)
    ellipse("grazing_ellipse", mid, ecross, e, R, 0.6 * R)
    # And one across the axis, entirely inside the wall.
    ellipse("inner_ellipse", mid, z, e, 0.9 * R, 0.5 * R)
    for name in ["meridional_ellipse", "grazing_ellipse", "inner_ellipse"]:
        pair(name, "cyl")
    return {
        "kind": "geometry",
        "description": "every case of the cycle-1 intersection table (plane–plane, plane–cylinder, line–plane, line–cylinder, conic–plane, conic–cylinder) around one cylinder in the tilt pose: circle, ellipse, two rulings, a tangent ruling and empty; crossing, parallel and coincident planes; one hit, parallel and coincident lines; two hits, a touch, a miss, a ruling; two, four, touches, misses, a parallel; the oblique section ellipse against the cylinder and the plane it lies in, cut, touched and missed by three planes normal to its major axis, and ellipses across the axis with four crossings, two touches and none; written by generate.py",
        "surfaces": surfaces,
        "curves": curves,
        "samples": [],
        "pairs": pairs,
    }


# --- c2-cylinder-pairs --------------------------------------------------------------


def c2_cylinder_pairs():
    f = POSES["tilt"]
    R = 2.0
    z = f.z
    surfaces = {"cyl": {"type": "cylinder", **f.spec(radius=R)}}
    pairs = []

    def cylinder(name, origin, axis, x_hint, radius):
        surfaces[name] = {"type": "cylinder", **Frame(origin, axis, x_hint).spec(radius=radius)}

    def pair(a, b):
        pairs.append({"a": a, "b": b})

    a, across = f.around(0.6)
    on_axis = f.to_world([0.0, 0.0, 1.5])
    # Parallel axes, `d` apart along `a`, each origin slid along the axis
    # and one axis reversed: two rulings, a touch outside and inside
    # (`d = R + r` and `R − r`), apart, nested.
    r = 1.5
    for name, d, radius, flip in [
        ("two", 2.0, r, False),
        ("outside", R + r, r, True),
        ("inside", R - r, r, False),
        ("apart", R + r + 1.0, r, False),
        ("nested", 0.8, 0.5, True),
    ]:
        axis = mul(-1.0, z) if flip else z
        cylinder(name, add(add(on_axis, mul(d, a)), mul(-0.7, z)), axis, across, radius)
        pair("cyl", name)
    # Crossing axes through a point of the first, equal radii at 90° and
    # 50°: two ellipses each. Unequal radii: a quartic, unsolved.
    psi = math.radians(50.0)
    cylinder("cross_90", add(on_axis, mul(0.7, a)), a, z, R)
    cylinder("cross_50", add(on_axis, mul(-1.1, add(mul(math.cos(psi), z), mul(math.sin(psi), a)))), add(mul(math.cos(psi), z), mul(math.sin(psi), a)), across, R)
    cylinder("cross_unequal", add(on_axis, mul(0.4, across)), across, z, 1.2)
    for name in ["cross_90", "cross_50", "cross_unequal"]:
        pair("cyl", name)
    # Skew axes along `a`, their common perpendicular along `across`:
    # further apart than the radii (empty by the triangle inequality) and
    # within them (a quartic). Unequal radii: Open CASCADE reports two
    # ellipses for equal radii whatever the gap.
    cylinder("skew_apart", add(on_axis, mul(R + 1.2 + 0.8, across)), a, z, 1.2)
    cylinder("skew_close", add(on_axis, mul(1.5, across)), a, z, 1.2)
    for name in ["skew_apart", "skew_close"]:
        pair("cyl", name)
    # Two pairs the other way round: the rulings from the second operand's
    # origin, the ellipses with the axes swapped.
    pair("two", "cyl")
    pair("cross_50", "cyl")
    return {
        "kind": "geometry",
        "description": "every case of the cylinder–cylinder table around one cylinder in the tilt pose: parallel axes giving two rulings, a tangent ruling outside and inside, apart and nested; crossing axes of equal radii at 90° and 50° giving two ellipses; crossing axes of unequal radii and skew axes within the radii, which Open CASCADE leaves unsolved; skew axes apart, which it leaves unsolved and Arris decides empty; two pairs swapped; written by generate.py",
        "surfaces": surfaces,
        "curves": {},
        "samples": [],
        "pairs": pairs,
    }


# --- c3-cylinder-pairs --------------------------------------------------------------


def c3_cylinder_pairs():
    f = POSES["tilt"]
    R = 2.0
    z = f.z
    surfaces = {"cyl": {"type": "cylinder", **f.spec(radius=R)}}
    pairs = []

    def cylinder(name, origin, axis, x_hint, radius):
        surfaces[name] = {"type": "cylinder", **Frame(origin, axis, x_hint).spec(radius=radius)}
        pairs.append({"a": "cyl", "b": name})

    a, across = f.around(1.1)
    on_axis = f.to_world([0.0, 0.0, -0.8])

    def tilted(deg):
        t = math.radians(deg)
        return add(mul(math.cos(t), z), mul(math.sin(t), a))

    # Crossing axes of unequal radii: a smaller cylinder through the first
    # at 90° and at 40°, and a larger one around it — two loops each.
    cylinder("cross_90", add(on_axis, mul(0.6, a)), a, z, 1.2)
    cylinder("cross_40", add(on_axis, mul(-0.9, tilted(40.0))), tilted(40.0), across, 1.5)
    cylinder("cross_larger", add(on_axis, mul(1.3, tilted(70.0))), tilted(70.0), across, 3.0)
    # Skew axes within the radii, the common perpendicular along `across`:
    # the smaller one breaking out of the first (one loop) and staying
    # inside it (two loops), at 90° and at 55°.
    cylinder("skew_out", add(on_axis, mul(1.5, across)), a, z, 1.2)
    cylinder("skew_in", add(on_axis, mul(-0.5, across)), a, z, 1.2)
    cylinder("skew_55", add(on_axis, mul(1.1, across)), tilted(55.0), across, 1.0)
    # One the other way round: the walked cylinder is the smaller either way.
    pairs.append({"a": "skew_out", "b": "cyl"})
    return {
        "kind": "geometry",
        "description": "the cylinder pairs that meet in a quartic around one cylinder in the tilt pose, which Arris traces and fits (ADR-0018) and Open CASCADE leaves unsolved in IntAna_QuadQuadGeo and walks in GeomAPI_IntSS: crossing axes of unequal radii at 90° and 40° and around a larger cylinder, two loops each; skew axes within the radii breaking out (one loop), staying inside (two loops) and at 55°; one pair swapped; written by generate.py",
        "surfaces": surfaces,
        "curves": {},
        "samples": [],
        "pairs": pairs,
    }


# --- c3-quadric-pairs --------------------------------------------------------------


def c3_quadric_pairs():
    f = POSES["tilt"]
    alpha = math.radians(30.0)
    surfaces = {"cone": {"type": "cone", **f.spec(radius=1.0, half_angle_deg=30.0)}}
    pairs = []
    apex = [0.0, 0.0, -1.0 / math.tan(alpha)]

    def local_frame(origin, z, x):
        return Frame(f.to_world(origin), f.vec(z), f.vec(x))

    def plane(name, origin, tilt):
        # The normal `tilt` off the cone's axis, towards its local x.
        n = [math.sin(tilt), 0.0, math.cos(tilt)]
        surfaces[name] = {"type": "plane", **local_frame(origin, n, [0.0, 1.0, 0.0]).spec()}
        pairs.append({"a": name, "b": "cone"})

    # A plane off the axis and the apex: steeper than the cone an ellipse,
    # as steep a parabola, parallel to the axis a hyperbola's two branches.
    plane("steep", [0.0, 0.0, 1.0], math.radians(20.0))
    plane("parabolic", [0.0, 0.0, 1.0], math.pi / 2 - alpha)
    plane("parallel", [0.5, 0.0, 0.0], math.pi / 2)
    # Through the apex: the apex alone, two rulings, one touching ruling.
    plane("apex_steep", apex, math.radians(20.0))
    plane("apex_shallow", apex, math.radians(80.0))
    plane("apex_touch", apex, math.pi / 2 - alpha)

    def other(name, spec, a="cone"):
        surfaces[name] = spec
        pairs.append({"a": a, "b": name})

    # A pipe across the cone's axis through one nappe: two loops.
    other("pipe", {"type": "cylinder", **local_frame([0.0, 0.0, 2.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]).spec(radius=0.4)})
    # A ball straddling the cone off its axis: one loop.
    ball = [1.8, 0.0, 1.5]
    other("ball", {"type": "sphere", **local_frame(ball, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]).spec(radius=1.0)})
    # A post along the axis's direction through the ball, off its centre.
    other("post", {"type": "cylinder", **local_frame([2.3, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]).spec(radius=0.6)}, a="ball")
    # A narrow cone, its apex inside the first's upper nappe and its axis
    # 8° off: its lower nappe leaves the first's upper nappe and enters
    # its lower one, a loop on each.
    t = math.radians(8.0)
    zb = [math.sin(t), 0.0, math.cos(t)]
    spike_apex = [0.0, 0.0, 1.0]
    beta = math.radians(10.0)
    origin = add(spike_apex, mul(0.3 / math.tan(beta), zb))
    other("spike", {"type": "cone", **local_frame(origin, zb, [0.0, 1.0, 0.0]).spec(radius=0.3, half_angle_deg=10.0)})
    # Two the other way round.
    pairs.append({"a": "cone", "b": "parallel"})
    pairs.append({"a": "spike", "b": "cone"})
    return {
        "kind": "geometry",
        "description": "the quadric pairs with a cone or a sphere in them that share no axis, around one cone in the tilt pose: a plane off the axis in an ellipse, a parabola and a hyperbola's two branches, exact; through the apex the apex alone, two rulings and one touching ruling; a pipe across the axis, a ball straddling the cone, a post through the ball and a narrow cone inside the first, which Arris traces and fits (ADR-0018) and Open CASCADE walks in GeomAPI_IntSS; two pairs swapped; written by generate.py",
        "surfaces": surfaces,
        "curves": {},
        "samples": [],
        "pairs": pairs,
    }


# --- c3-torus-pairs --------------------------------------------------------------


def c3_torus_pairs():
    f = POSES["tilt"]
    R, r = 2.0, 0.5
    surfaces = {"ring": {"type": "torus", **f.spec(major_radius=R, minor_radius=r)}}
    pairs = []

    def local_frame(origin, z, x):
        return Frame(f.to_world(origin), f.vec(z), f.vec(x))

    def other(name, spec, a="ring"):
        surfaces[name] = spec
        pairs.append({"a": a, "b": name})

    # A plane parallel to the axis: through the hole two ovals, tangent to
    # the hole a figure eight with one singular point, through the tube
    # alone one oval. Oblique to the axis, two ovals again.
    def wall(name, d):
        other(name, {"type": "plane", **local_frame([d, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]).spec()})

    wall("hole", 1.0)
    wall("eight", R - r)
    wall("tube", R)
    other("oblique", {"type": "plane", **local_frame([0.0, 0.0, 0.1], [0.3, 0.1, 1.0], [0.0, 1.0, 0.0]).spec()})
    # A drill of a fifth of the tube's radius straight through the tube:
    # two loops. A pipe of the ring's own tube radius, its axis tangent to
    # the centre circle — the elbow — shares that tube circle, exactly,
    # and crosses the ring in two arms besides.
    other("drill", {"type": "cylinder", **local_frame([R, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]).spec(radius=0.2)})
    other("elbow", {"type": "cylinder", **local_frame([R, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]).spec(radius=r)})
    # A cone and a sphere off the axis, each through the tube.
    other("cone", {"type": "cone", **local_frame([1.6, 0.4, -1.2], [0.15, 0.2, 1.0], [0.0, 1.0, 0.0]).spec(radius=0.6, half_angle_deg=30.0)})
    other("ball", {"type": "sphere", **local_frame([1.7, 0.4, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]).spec(radius=0.8)})
    # A second torus interlocked with the first, its axis tangent to the
    # centre circle, and a larger one the ring stands in at an angle.
    other("link", {"type": "torus", **local_frame([R, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]).spec(major_radius=1.0, minor_radius=0.3)})
    other("hoop", {"type": "torus", **local_frame([0.4, 0.0, 0.3], [0.25, 0.0, 1.0], [1.0, 0.0, 0.0]).spec(major_radius=2.2, minor_radius=0.6)})
    # Two the other way round: the walked torus is the ring either way
    # (`link` is smaller, so it is walked against the ring).
    pairs.append({"a": "hole", "b": "ring"})
    pairs.append({"a": "link", "b": "ring"})
    return {
        "kind": "geometry",
        "description": "a plane, a cylinder, a cone, a sphere and a second torus around one torus in the tilt pose, sharing no axis with it, which Arris traces in the torus's parameter plane and fits (ADR-0019) and Open CASCADE leaves unsolved in IntAna_QuadQuadGeo and walks in GeomAPI_IntSS: a plane parallel to the axis through the hole (two ovals), tangent to it (a figure eight through one singular point) and through the tube alone (one oval), an oblique plane (two ovals); a drill through the tube (two loops) and a pipe of the tube's radius tangent to the centre circle, which shares a tube circle with it exactly; a cone and a sphere off the axis; a torus interlocked with the ring and a larger one it stands in; two pairs swapped; written by generate.py",
        "surfaces": surfaces,
        "curves": {},
        "samples": [],
        "pairs": pairs,
    }


# --- c2-quadric-pairs --------------------------------------------------------------


def c2_quadric_pairs():
    f = POSES["tilt"]
    z = f.z
    surfaces = {}
    pairs = []

    def plane(name, origin, normal, x_hint):
        surfaces[name] = {"type": "plane", **Frame(origin, normal, x_hint).spec()}

    def cylinder(name, origin, axis, x_hint, radius):
        surfaces[name] = {"type": "cylinder", **Frame(origin, axis, x_hint).spec(radius=radius)}

    def cone(name, origin, axis, x_hint, radius, half_angle_deg):
        surfaces[name] = {"type": "cone", **Frame(origin, axis, x_hint).spec(radius=radius, half_angle_deg=half_angle_deg)}

    def sphere(name, centre, axis, x_hint, radius):
        surfaces[name] = {"type": "sphere", **Frame(centre, axis, x_hint).spec(radius=radius)}

    def torus(name, origin, axis, x_hint, big, small):
        surfaces[name] = {"type": "torus", **Frame(origin, axis, x_hint).spec(major_radius=big, minor_radius=small)}

    def pair(a, b):
        pairs.append({"a": a, "b": b})

    a, across = f.around(0.6)
    on_axis = lambda h: f.to_world([0.0, 0.0, h])
    # The three surfaces of revolution on the axis: a cone of radius 2 at
    # 30° whose apex is 2 / tan 30° below its origin, a sphere whose own
    # frame is turned across the axis, a torus of radii 4 and 1.5.
    cone("cone", on_axis(1.5), z, a, 2.0, 30.0)
    apex = on_axis(1.5 - 2.0 / math.tan(math.radians(30.0)))
    sphere("sphere", on_axis(-0.5), a, z, 2.5)
    torus("torus", on_axis(3.0), z, across, 4.0, 1.5)
    # Planes perpendicular to the axis, their origins off it: crossing the
    # cone, the sphere and the torus (one circle, one circle, two circles),
    # touching the sphere at its pole (a point) and the torus at the top of
    # its tube (a tangent circle of radius 4), and through the cone's apex,
    # normal reversed (a point).
    plane("cap_cone", add(on_axis(2.5), mul(0.7, a)), z, a)
    plane("cap_sphere", add(on_axis(0.5), mul(-0.4, across)), z, across)
    plane("cap_torus", add(on_axis(3.6), mul(1.1, a)), z, across)
    plane("touch_sphere", add(on_axis(2.0), mul(0.9, a)), z, a)
    plane("touch_torus", add(on_axis(4.5), mul(-0.6, a)), z, a)
    plane("apex_plane", add(apex, mul(0.3, a)), mul(-1.0, z), across)
    for name, other in [
        ("cap_cone", "cone"),
        ("cap_sphere", "sphere"),
        ("cap_torus", "torus"),
        ("touch_sphere", "sphere"),
        ("touch_torus", "torus"),
        ("apex_plane", "cone"),
    ]:
        pair(name, other)
    # Cylinders on the axis, slid along it: a bore of radius 1.2 through
    # the cone (two circles, one per nappe) and the sphere (two circles),
    # clear of the torus's tube (empty); a sleeve of radius 3 through the
    # torus (two circles); a sleeve of radius 5.5 around it (a tangent
    # circle at the outer equator); a hoop of the sphere's radius (a
    # tangent circle at its equator).
    # Open CASCADE's cylinder–sphere case wants the sphere's own axis on
    # the cylinder's exactly: the turned sphere is `unsolved` there, and
    # the orb, the same sphere with its frame along the axis, is answered.
    cylinder("bore", on_axis(-2.0), z, a, 1.2)
    cylinder("sleeve", on_axis(6.0), mul(-1.0, z), across, 3.0)
    cylinder("touch_sleeve", on_axis(1.0), z, a, 5.5)
    cylinder("hoop", on_axis(-4.0), z, across, 2.5)
    sphere("orb", on_axis(-0.5), z, a, 2.5)
    for name, other in [
        ("bore", "cone"),
        ("bore", "sphere"),
        ("bore", "orb"),
        ("bore", "torus"),
        ("sleeve", "torus"),
        ("touch_sleeve", "torus"),
        ("hoop", "sphere"),
        ("hoop", "orb"),
    ]:
        pair(name, other)
    # A second cone on the axis at the same angle with another apex (one
    # circle), and one at another angle opening the other way (two
    # circles); a larger sphere on the axis through the torus's tube (two
    # circles).
    cone("cone_same", on_axis(3.5), z, across, 2.0, 30.0)
    cone("cone_steep", on_axis(4.0), mul(-1.0, z), a, 1.0, 50.0)
    sphere("ball", on_axis(2.0), across, a, 4.2)
    pair("cone", "cone_same")
    pair("cone", "cone_steep")
    pair("ball", "torus")
    # A plane oblique to the sphere's axis, in general position (a circle);
    # two spheres crossing (a circle), touching from outside and from
    # inside (a point each).
    n = unit(add(mul(math.cos(0.5), z), mul(math.sin(0.5), a)))
    plane("oblique", add(add(on_axis(-0.5), mul(1.0, n)), mul(0.4, across)), n, across)
    d = unit(add(add(mul(0.3, z), mul(0.8, a)), mul(-0.5, across)))
    sphere("pebble", add(on_axis(-0.5), mul(2.5 + 0.8, d)), across, z, 0.8)
    e = unit(add(add(mul(-0.7, z), mul(0.2, a)), mul(0.6, across)))
    sphere("marble", add(on_axis(-0.5), mul(2.5 - 0.6, e)), z, a, 0.6)
    pair("oblique", "sphere")
    pair("sphere", "ball")
    pair("sphere", "pebble")
    pair("marble", "sphere")
    # Planes through the axis, their origins slid along it and across it in
    # the plane: through the cone (its two rulings through the apex) and
    # through the torus (its two tube circles) — a partial revolve's caps.
    plane("meridian_cone", add(on_axis(-0.7), mul(1.3, across)), a, z)
    plane("meridian_torus", add(on_axis(2.2), mul(-0.9, a)), mul(-1.0, across), z)
    pair("meridian_cone", "cone")
    pair("torus", "meridian_torus")
    # Lines against the three: what B1's containment ray and a probe's
    # classification cast. The sphere crossed on a chord, grazed at its
    # radius, and the orb along its own axis through both poles; the cone
    # skewered across one nappe, along a ruling, through its apex inside
    # the nappes, across a ruling in the tangent plane there, and parallel
    # to a ruling in a plane through the axis (one hit); the torus along a
    # diameter of its equator (four hits), grazing the outer equator,
    # grazing the inner one (a touch between two crossings), down through
    # the tube, and along its axis (none).
    curves = {}

    def line(name, origin, direction):
        curves[name] = {"type": "line", "origin": origin, "direction": unit(direction)}

    alpha = math.radians(30.0)
    ruling = add(mul(math.sin(alpha), a), mul(math.cos(alpha), z))
    line("chord_sphere", add(on_axis(-0.5), mul(1.2, a)), add(across, mul(0.3, z)))
    line("graze_sphere", add(on_axis(-0.5), mul(2.5, a)), across)
    line("pole_line", on_axis(-6.0), z)
    line("skewer_cone", add(on_axis(3.0), add(mul(1.0, across), mul(-5.0, a))), a)
    line("ruling", add(apex, mul(2.0, ruling)), ruling)
    line("apex_line", apex, add(z, mul(0.2, a)))
    tilt = 0.7
    line("tangent_cone", add(apex, mul(2.0, ruling)), add(mul(math.cos(tilt), ruling), mul(math.sin(tilt), across)))
    line("parallel_ruling", add(apex, mul(0.8, z)), ruling)
    line("through_tube", on_axis(3.0), a)
    line("graze_outer", add(on_axis(3.0), mul(5.5, across)), a)
    line("graze_inner", add(on_axis(3.0), mul(2.5, across)), a)
    line("vertical_tube", add(on_axis(0.0), mul(4.0, a)), z)
    for name, other in [
        ("chord_sphere", "sphere"),
        ("graze_sphere", "sphere"),
        ("pole_line", "orb"),
        ("pole_line", "torus"),
        ("skewer_cone", "cone"),
        ("ruling", "cone"),
        ("apex_line", "cone"),
        ("tangent_cone", "cone"),
        ("parallel_ruling", "cone"),
        ("through_tube", "torus"),
        ("graze_outer", "torus"),
        ("graze_inner", "torus"),
        ("vertical_tube", "torus"),
    ]:
        pair(name, other)
    # Two pairs the other way round.
    pair("cone", "cap_cone")
    pair("torus", "sleeve")
    return {
        "kind": "geometry",
        "description": "the coaxial pairs with a cone, a sphere or a torus in them, and the pairs any sphere makes, around one axis in the tilt pose: planes perpendicular to the axis crossing a cone, a sphere and a torus, touching the sphere at a pole and the torus at its tube's top, and through the cone's apex; cylinders on the axis through the cone, the sphere (its frame turned, which Open CASCADE leaves unsolved, and along the axis, which it answers) and the torus, clear of the torus, around it at its outer equator and at the sphere's equator; two cones on the axis at the same and at another angle; a sphere through the torus; a plane oblique to a sphere; two spheres crossing, touching outside and touching inside; planes through the cone's and the torus's axis, giving two rulings through the apex and two tube circles; lines against the three: a chord and a graze of the sphere, the poles of the orb, a skewer, a ruling, a line through the apex, a tangent and a line parallel to a ruling of the cone, a diameter of the torus's equator, both equators grazed, a line down the tube and one along the axis; two pairs swapped; written by generate.py",
        "surfaces": surfaces,
        "curves": curves,
        "samples": [],
        "pairs": pairs,
    }


# --- c3-conic-hits -------------------------------------------------------------------


def c3_conic_hits():
    """Circles and ellipses against the four surfaces a conic has no exact
    section of, and coplanar conic pairs with an ellipse among them —
    everything `c3-conic-hits` adds to the intersector, in the tilt pose.
    Every case is built so that its crossings and touches are a closed
    form of the surface's trace in the conic's plane."""
    f = POSES["tilt"]
    z = f.z
    a, across = f.around(0.6)
    on_axis = lambda h: f.to_world([0.0, 0.0, h])
    surfaces = {
        "cone": {"type": "cone", **Frame(on_axis(1.5), z, a).spec(radius=2.0, half_angle_deg=30.0)},
        "ball": {"type": "sphere", **Frame(on_axis(-0.5), a, z).spec(radius=2.5)},
        "oval": {"type": "elliptic_cylinder", **Frame(on_axis(-2.0), z, a).spec(major_radius=2.6, minor_radius=1.4)},
        "ring": {"type": "torus", **Frame(on_axis(3.0), z, across).spec(major_radius=4.0, minor_radius=1.5)},
    }
    curves = {}
    # The cone, in the plane of its axis and `a`, where its trace is the
    # two lines through the apex: the axis is `gap` above the apex, so
    # each line is `gap·sin α` from the centre on it. A circle wider than
    # that crosses both twice, and one exactly that wide touches each.
    gap = 2.0 / math.tan(math.radians(30.0))
    reach = gap * math.sin(math.radians(30.0))
    curves["cone_cross"] = {"type": "circle", **Frame(on_axis(1.5), across, z).spec(radius=2.0)}
    curves["cone_touch"] = {"type": "circle", **Frame(on_axis(1.5), across, z).spec(radius=reach)}
    # The sphere, about its centre: an ellipse in a tilted plane whose
    # radii straddle the radius crosses it four times, and a circle whose
    # far point is exactly on it touches there, a quarter turn from its
    # own seam.
    ball = on_axis(-0.5)
    lean = unit(add(z, mul(0.6, a)))
    in_plane = cross(lean, across)
    curves["ball_cross"] = {"type": "ellipse", **Frame(ball, lean, across).spec(major_radius=3.5, minor_radius=2.0)}
    curves["ball_touch"] = {"type": "circle", **Frame(add(ball, across), lean, in_plane).spec(radius=1.5)}
    # The elliptic cylinder, in the plane of its axis and its major axis,
    # where its trace is the two lines at `±a`: an ellipse reaching past
    # them crosses each twice, a circle of the major radius touches each,
    # and a circle in a plane oblique to the axis crosses four times.
    oval = on_axis(-2.0)
    curves["oval_cross"] = {"type": "ellipse", **Frame(oval, across, a).spec(major_radius=3.4, minor_radius=1.2)}
    curves["oval_touch"] = {"type": "circle", **Frame(oval, across, z).spec(radius=2.6)}
    curves["oval_slice"] = {"type": "circle", **Frame(oval, unit(add(z, mul(0.8, a))), across).spec(radius=3.2)}
    # The torus: an ellipse in its equatorial plane reaching past the
    # outer equator and inside the inner one leaves and re-enters the
    # tube eight times; a circle of the inner radius in the plane of the
    # axis touches the tube at both inner equator points; and a circle
    # about a point off the core circle, in that plane, crosses one tube
    # circle twice and comes nowhere near the other.
    ring = on_axis(3.0)
    curves["ring_cross"] = {"type": "ellipse", **Frame(ring, z, a).spec(major_radius=6.05, minor_radius=2.25)}
    curves["ring_touch"] = {"type": "circle", **Frame(ring, across, z).spec(radius=2.5)}
    curves["ring_drill"] = {"type": "circle", **Frame(add(ring, add(mul(4.0, a), mul(1.0, z))), across, z).spec(radius=2.2)}
    # Two conics in one plane with an ellipse among them, which no plane
    # decides: a circle between the ellipse's radii crosses it four
    # times, a circle of its minor radius touches it at both minor
    # vertices, and a second ellipse about the same centre, wider one way
    # and narrower the other, crosses it four times.
    flat = Frame(f.to_world([0.0, 0.0, 6.0]), unit(add(z, mul(0.3, across))), a)
    turned = Frame(flat.origin, flat.z, add(mul(math.cos(0.7), flat.x), mul(math.sin(0.7), flat.y)))
    curves["flat_oval"] = {"type": "ellipse", **flat.spec(major_radius=3.0, minor_radius=1.6)}
    curves["flat_round"] = {"type": "circle", **flat.spec(radius=2.2)}
    curves["flat_minor"] = {"type": "circle", **flat.spec(radius=1.6)}
    curves["flat_turned"] = {"type": "ellipse", **turned.spec(major_radius=2.6, minor_radius=1.9)}
    # The elliptic cylinder is new to the oracle here, so its own
    # parametrisation is checked too: `P(u, v) = O + a cos u·X + b sin
    # u·Y + v·Z`, which is the extrusion the oracle builds.
    samples = [
        {"of": "oval", "params": [[0.0, 0.0], [0.7, 1.3], [math.pi, -2.0], [5.9, 0.4]],
         "points": [f.to_world([3.5, 1.0, -1.0]), f.to_world([0.2, -0.3, -2.0])]},
    ]
    pairs = [
        {"a": "cone_cross", "b": "cone"},
        {"a": "cone_touch", "b": "cone"},
        {"a": "ball_cross", "b": "ball"},
        {"a": "ball_touch", "b": "ball"},
        {"a": "oval_cross", "b": "oval"},
        {"a": "oval_touch", "b": "oval"},
        {"a": "oval_slice", "b": "oval"},
        {"a": "ring_cross", "b": "ring"},
        {"a": "ring_touch", "b": "ring"},
        {"a": "ring_drill", "b": "ring"},
        {"a": "flat_oval", "b": "flat_round"},
        {"a": "flat_round", "b": "flat_oval"},
        {"a": "flat_oval", "b": "flat_minor"},
        {"a": "flat_oval", "b": "flat_turned"},
    ]
    return {
        "kind": "geometry",
        "description": "Circles and ellipses against a cone, a sphere, an elliptic cylinder and a torus, and coplanar conic pairs with an ellipse among them, in the tilt pose (ADR-0018): each conic placed in a plane where the surface's trace is a closed form — the cone's two lines through its apex, the elliptic cylinder's two lines at its major radius, the torus's two tube circles — so its crossings and its touches are known before the oracle is asked; the elliptic cylinder is the section ellipse extruded along the axis, which Open CASCADE meets through its general intersector, and the coplanar pairs through its curve\u2013curve extrema; written by generate.py",
        "surfaces": surfaces,
        "curves": curves,
        "samples": samples,
        "pairs": pairs,
    }


# --- c3-nurbs-hits -------------------------------------------------------------------


def c3_nurbs_hits():
    f = POSES["tilt"]
    z = f.z
    a, across = f.around(0.6)
    on_axis = lambda h: f.to_world([0.0, 0.0, h])
    surfaces = {
        "floor": {"type": "plane", **Frame(add(on_axis(0.5), mul(0.7, a)), z, a).spec()},
        "wall": {"type": "cylinder", **Frame(on_axis(-2.0), z, a).spec(radius=2.0)},
        "cone": {"type": "cone", **Frame(on_axis(1.5), z, a).spec(radius=2.0, half_angle_deg=30.0)},
        "sphere": {"type": "sphere", **Frame(on_axis(-0.5), a, z).spec(radius=2.5)},
        "torus": {"type": "torus", **Frame(on_axis(3.0), z, across).spec(major_radius=4.0, minor_radius=1.5)},
    }
    # An ellipse of radii 3.5 and 2 as four rational quadratic arcs, its
    # plane standing along the axis and leaning off it: exact, clamped and closed, its seam
    # clear of every surface.
    centre = f.to_world([1.0, 0.5, 1.0])
    ex = unit(f.vec([1.0, 0.2, 0.3]))
    ey = unit(cross(ex, f.vec([0.2, 1.0, 0.1])))
    w = math.sqrt(0.5)
    corners = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1), (1, 0)]
    ring = {
        "type": "nurbs",
        "degree": 2,
        "knots": [0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 4.0, 4.0, 4.0],
        "control_points": [add(centre, add(mul(3.5 * x, ex), mul(2.0 * y, ey))) for x, y in corners],
        "weights": [1.0 if (x == 0) != (y == 0) else w for x, y in corners],
    }
    # A quintic of three spans weaving through all five, as a fitted
    # section curve is a quintic of many.
    wave = {
        "type": "nurbs",
        "degree": 5,
        "knots": [0.0] * 6 + [1.0, 2.0] + [3.0] * 6,
        "control_points": [
            f.to_world(p)
            for p in [[-5.0, -1.0, -1.0], [-3.0, 2.0, 3.0], [-1.0, -3.0, 0.0], [0.0, 3.0, 4.0], [2.0, -2.0, -2.0], [3.0, 1.0, 5.0], [5.0, -1.0, 1.0], [6.0, 2.0, 3.0]]
        ],
        "weights": [1.0] * 8,
    }
    # A rational cubic with a double knot inside, where it is only C¹.
    skein = {
        "type": "nurbs",
        "degree": 3,
        "knots": [0.0] * 4 + [1.0, 1.0, 2.0] + [3.0] * 4,
        "control_points": [
            f.to_world(p)
            for p in [[4.0, -4.0, 5.0], [1.0, -1.0, 6.0], [-2.0, 2.0, 2.0], [3.0, 3.0, -1.0], [-3.0, -2.0, 1.0], [0.0, 4.0, 3.0], [-4.0, 1.0, 3.0]]
        ],
        "weights": [1.0, 0.6, 1.8, 0.9, 1.4, 0.7, 1.0],
    }
    curves = {"ring": ring, "wave": wave, "skein": skein}
    # Each curve evaluated on its knots and between them, and projected
    # onto from a point beside it: the parameter the hits are compared in.
    samples = [
        {"of": "ring", "params": [0.0, 0.37, 1.0, 2.5, 3.99], "points": [f.to_world([3.0, 1.0, 2.5])]},
        {"of": "wave", "params": [0.0, 0.5, 1.0, 1.75, 2.0, 3.0], "points": [f.to_world([0.5, 0.2, 1.4])]},
        {"of": "skein", "params": [0.25, 1.0, 1.5, 2.0, 2.9], "points": [f.to_world([0.0, 1.5, 3.0])]},
    ]
    pairs = [{"a": c, "b": s} for c in curves for s in surfaces]
    return {
        "kind": "geometry",
        "description": "NURBS curves against a plane, a cylinder, a cone, a sphere and a torus around one axis in the tilt pose (ADR-0018): an ellipse as four rational quadratic arcs, a quintic of three spans, and a rational cubic with a double knot — each evaluated on and between its knots, projected onto, and met with every surface through Open CASCADE's general curve–surface intersector; written by generate.py",
        "surfaces": surfaces,
        "curves": curves,
        "samples": samples,
        "pairs": pairs,
    }


# --- c3-nurbs-crossings --------------------------------------------------------------


def nurbs_point(spec, t):
    """A `nurbs` spec's point at `t`: de Boor on the homogeneous control
    points (*The NURBS Book* A4.1), on the span holding `t`, the last one
    at the domain's end."""
    p, knots = spec["degree"], spec["knots"]
    n = len(spec["control_points"])
    k = max(i for i in range(p, n) if knots[i] <= t)
    d = [mul(w, pt) + [w] for pt, w in zip(spec["control_points"][k - p : k + 1], spec["weights"][k - p : k + 1])]
    for r in range(1, p + 1):
        for j in range(p, r - 1, -1):
            i = k - p + j
            alpha = (t - knots[i]) / (knots[i + p + 1 - r] - knots[i])
            d[j] = [(1.0 - alpha) * x + alpha * y for x, y in zip(d[j - 1], d[j])]
    return mul(1.0 / d[p][3], d[p][:3])


def c3_nurbs_crossings():
    f = POSES["tilt"]
    hits = c3_nurbs_hits()
    ring, wave, skein = (hits["curves"][n] for n in ("ring", "wave", "skein"))
    # The ring's own frame: the ellipse of radii 3.5 and 2 that c3-nurbs-hits
    # built as four rational arcs, at its own angle.
    centre = f.to_world([1.0, 0.5, 1.0])
    ex = unit(f.vec([1.0, 0.2, 0.3]))
    ey = unit(cross(ex, f.vec([0.2, 1.0, 0.1])))
    normal = cross(ex, ey)
    on_ring = lambda th: add(centre, add(mul(3.5 * math.cos(th), ex), mul(2.0 * math.sin(th), ey)))
    ring_tangent = lambda th: unit(add(mul(-3.5 * math.sin(th), ex), mul(2.0 * math.cos(th), ey)))
    curves = {"ring": ring, "wave": wave, "skein": skein}
    pairs = []

    def line(name, through, direction, back=1.5):
        direction = unit(direction)
        curves[name] = {"type": "line", "origin": sub(through, mul(back, direction)), "direction": direction}

    def conic(name, kind, through, z, x_hint, radii, phase):
        """A circle or an ellipse through `through`, at its own angle
        `phase` there, in the plane of normal `z`."""
        frame = Frame([0.0, 0.0, 0.0], z, x_hint)
        a, b = radii
        offset = add(mul(a * math.cos(phase), frame.x), mul(b * math.sin(phase), frame.y))
        origin = sub(through, offset)
        if kind == "circle":
            curves[name] = {"type": "circle", "origin": origin, "z": frame.z, "x": frame.x, "radius": a}
        else:
            curves[name] = {"type": "ellipse", "origin": origin, "z": frame.z, "x": frame.x, "major_radius": a, "minor_radius": b}

    # The ring against a line through it off its plane (one crossing), a
    # chord in its plane (two), a tangent in its plane (one touch) and the
    # tangent lifted off the plane (none); a circle and an ellipse through
    # it across its plane (one each), and a circle in its plane about its
    # centre between the radii (four). Every hit is clear of the seam, at
    # the major vertex.
    line("pierce", on_ring(1.0), add(normal, mul(0.4, ex)))
    line("chord", on_ring(0.8), sub(on_ring(2.9), on_ring(0.8)))
    line("tangent", on_ring(2.2), ring_tangent(2.2), back=2.0)
    curves["lifted"] = dict(curves["tangent"], origin=add(curves["tangent"]["origin"], mul(0.05, normal)))
    conic("hoop", "circle", on_ring(4.0), add(ex, mul(0.6, normal)), ey, (1.3, 1.3), 2.0)
    conic("oval", "ellipse", on_ring(5.2), add(ey, mul(0.8, normal)), add(ex, mul(0.3, normal)), (2.2, 0.9), 0.7)
    curves["round"] = {"type": "circle", "origin": centre, "z": normal, "x": ex, "radius": 2.7}
    for other in ["pierce", "chord", "tangent", "lifted", "hoop", "oval", "round"]:
        pairs.append({"a": "ring", "b": other})
    # The quintic against a line, a circle and an ellipse through it, and
    # the line moved off it (none); the rational cubic against a line
    # through its double knot and a circle through it.
    line("wave_line", nurbs_point(wave, 1.3), f.vec([0.3, -1.0, 0.6]))
    curves["wave_miss"] = dict(curves["wave_line"], origin=add(curves["wave_line"]["origin"], unit(f.vec([1.0, 0.0, 0.2]))))
    conic("wave_hoop", "circle", nurbs_point(wave, 0.6), f.vec([0.2, 0.5, 1.0]), f.vec([1.0, 0.0, 0.0]), (1.8, 1.8), 4.1)
    conic("wave_oval", "ellipse", nurbs_point(wave, 2.4), f.vec([1.0, -0.4, 0.3]), f.vec([0.0, 1.0, 0.0]), (2.5, 1.1), 1.9)
    line("skein_line", nurbs_point(skein, 1.0), f.vec([1.0, 1.0, -0.5]))
    conic("skein_hoop", "circle", nurbs_point(skein, 2.5), f.vec([0.6, -0.3, 1.0]), f.vec([0.0, 1.0, 0.0]), (1.4, 1.4), 0.4)
    for a, b in [("wave", "wave_line"), ("wave", "wave_miss"), ("wave", "wave_hoop"), ("wave", "wave_oval"), ("skein", "skein_line"), ("skein", "skein_hoop")]:
        pairs.append({"a": a, "b": b})
    # Two pairs the other way round.
    pairs.append({"a": "hoop", "b": "ring"})
    pairs.append({"a": "wave_line", "b": "wave"})
    return {
        "kind": "geometry",
        "description": "NURBS curves against lines, circles and ellipses in the tilt pose (ADR-0018): the ellipse of c3-nurbs-hits as four rational arcs against a line through it off its plane, a chord and a tangent in its plane, the tangent lifted off it, a circle and an ellipse through it across its plane and a circle in its plane crossing it four times; the quintic against a line, a circle and an ellipse through it and a line beside it; the rational cubic against a line through its double knot and a circle through it; two pairs swapped — met through Open CASCADE's curve–curve extrema; written by generate.py",
        "surfaces": {},
        "curves": curves,
        "samples": [],
        "pairs": pairs,
    }


# --- c4-nurbs-projections ------------------------------------------------------------


def c4_nurbs_projections():
    """Free-form surfaces, none with a twin in the analytic kinds, and the
    points the nearest of which is asked: a saddle whose two directions
    curve opposite ways; a rational bump of several spans in both
    directions; and a surface folded back on itself, so that a line
    through the fold meets it three times and a point in the pocket has
    a local minimum on each layer. Each is projected from a point near
    it, off its edge, above and below, and in the fold; the parameters
    are the oracle's (`Geom_BSplineSurface` over the same knots)."""
    f = POSES["tilt"]

    def net(rows):
        return [[f.to_world(p) for p in row] for row in rows]

    # A bicubic Bezier patch on a 4 x 4 grid, z = (x^2 - y^2) / 4 at the
    # nodes: control values, not the surface's own, so it is free-form.
    grid = [-3.0, -1.0, 1.0, 3.0]
    saddle = {
        "type": "nurbs",
        "degree": [3, 3],
        "knots": [[0.0] * 4 + [1.0] * 4, [0.0] * 4 + [1.0] * 4],
        "control_points": net([[[x, y, (x * x - y * y) / 4.0] for y in grid] for x in grid]),
        "weights": [[1.0] * 4 for _ in grid],
    }
    # A rational surface of degrees (2, 3): a hill whose centre control
    # point is pulled hard, over two spans in u and two in v.
    xs = [-3.0, -1.0, 1.0, 3.0]
    ys = [-3.0, -1.5, 0.0, 1.5, 3.0]
    height = [[0.2, 0.4, 0.5, 0.4, 0.2], [0.5, 1.6, 2.4, 1.6, 0.5], [0.5, 1.6, 2.4, 1.6, 0.5], [0.2, 0.4, 0.5, 0.4, 0.2]]
    pull = [[1.0, 1.0, 1.0, 1.0, 1.0], [1.0, 1.4, 2.5, 1.4, 1.0], [1.0, 1.4, 2.5, 1.4, 1.0], [1.0, 1.0, 1.0, 1.0, 1.0]]
    bump = {
        "type": "nurbs",
        "degree": [2, 3],
        "knots": [[0.0] * 3 + [1.0] + [2.0] * 3, [0.0] * 4 + [1.0] + [2.0] * 4],
        "control_points": net([[[x, y, height[i][j]] for j, y in enumerate(ys)] for i, x in enumerate(xs)]),
        "weights": pull,
    }
    # A cubic profile in (x, z) that goes right, back and right again — a
    # Z — swept along y by a quadratic that bulges in the middle.
    profile = [[-4.0, 0.0], [-1.0, 1.0], [3.0, 2.0], [-3.0, 3.0], [1.0, 4.0], [4.0, 5.0]]
    across = [(-3.0, 0.0), (0.0, 1.5), (3.0, 0.0)]
    fold = {
        "type": "nurbs",
        "degree": [3, 2],
        "knots": [[0.0] * 4 + [1.0, 2.0] + [3.0] * 4, [0.0] * 3 + [1.0] * 3],
        "control_points": net([[[x, y, z + bulge] for y, bulge in across] for x, z in profile]),
        "weights": [[1.0] * 3 for _ in profile],
    }
    surfaces = {"saddle": saddle, "bump": bump, "fold": fold}
    at = lambda points: [f.to_world(p) for p in points]
    samples = [
        {
            "of": "saddle",
            "params": [[0.0, 0.0], [0.5, 0.5], [0.9, 0.2], [1.0, 1.0]],
            "points": at([[0.0, 0.0, 1.2], [1.5, -1.0, 2.0], [5.0, 0.0, 0.5], [0.3, 0.2, 0.05]]),
        },
        {
            "of": "bump",
            "params": [[0.0, 0.0], [0.7, 1.3], [1.5, 0.4], [2.0, 2.0]],
            "points": at([[0.4, 0.3, 3.0], [2.6, 1.1, 0.7], [-1.0, 2.0, 1.0], [-0.9, -0.6, 2.61], [0.3, 0.2, 40.0]]),
        },
        {
            "of": "fold",
            "params": [[0.0, 0.0], [1.0, 0.5], [1.7, 0.25], [3.0, 1.0]],
            "points": at([[0.7, 0.4, 2.2], [-2.0, 0.5, 4.1], [2.2, -1.0, 1.5], [0.0, 2.9, 3.0], [-0.6, -0.4, 1.9]]),
        },
    ]
    return {
        "kind": "geometry",
        "description": "Points projected onto free-form NURBS surfaces with no analytic twin, in the tilt pose (ADR-0025 §1, plan step-reader step 3): a bicubic saddle, a rational bump of degrees (2, 3) over two spans each way, and a cubic-by-quadratic surface folded back on itself so a point in the pocket has a local minimum on each layer — each evaluated and projected from points near it, one off an edge, one far above and one inside the fold, held to GeomAPI_ProjectPointOnSurf's lowest distance. No query sits on a mirror plane of a symmetric surface, where two points tie and Arris reports the tie rather than choosing (tests/nurbs_project.rs), and none has its nearest point on a boundary that Open CASCADE's projection misses: it stopped short of the true minimum, by up to 2.4e-4 in distance, for a point off the saddle's edge, off the bump's corner and off the fold's edge, so those are held to a grid there instead; written by generate.py",
        "surfaces": surfaces,
        "curves": {},
        "samples": samples,
        "pairs": [],
    }


# --- c4-closed-curve-hits --------------------------------------------------------------


def c4_closed_curve_hits():
    """A clamped cubic B-spline whose two ends are one point, crossing a
    plane and a cylinder at that join and elsewhere (plan step-reader
    step 5): the join is one hit, not one at each end."""
    f = POSES["tilt"]
    # The join at (2, 0, 0), left and arrived at along (0.5, 1.5, 0.8), so
    # the curve is G1 there and crosses both surfaces through it.
    local = [[2.0, 0.0, 0.0], [2.5, 1.5, 0.8], [0.0, 3.0, 1.2], [-2.5, 1.5, 0.4], [-2.5, -1.5, -0.6], [0.0, -3.0, -1.0], [1.5, -1.5, -0.8], [2.0, 0.0, 0.0]]
    loop = {
        "type": "nurbs",
        "degree": 3,
        "knots": [0.0] * 4 + [1.0, 2.0, 3.0, 4.0] + [5.0] * 4,
        "control_points": [f.to_world(p) for p in local],
        "weights": [1.0] * 8,
    }
    join = f.to_world([2.0, 0.0, 0.0])
    surfaces = {
        # Through the join, its normal across the curve's tangent there.
        "sheet": {"type": "plane", **Frame(join, f.vec([0.0, 1.0, 0.3]), f.vec([1.0, 0.0, 0.0])).spec()},
        # The join on its wall, the curve leaving outward.
        "drum": {"type": "cylinder", **Frame(f.to_world([0.0, 0.0, -3.0]), f.z, f.vec([1.0, 0.0, 0.0])).spec(radius=2.0)},
        # Clear of the join: a plane the curve crosses twice elsewhere.
        "shelf": {"type": "plane", **Frame(f.to_world([0.0, 0.0, 0.5]), f.z, f.vec([1.0, 0.0, 0.0])).spec()},
    }
    samples = [{"of": "loop", "params": [0.0, 0.5, 2.5, 4.5, 5.0], "points": [f.to_world([2.5, 0.5, 0.0])]}]
    pairs = [{"a": "loop", "b": s} for s in surfaces]
    return {
        "kind": "geometry",
        "description": "A clamped closed cubic B-spline in the tilt pose, its two ends one point and the curve G1 there, against a plane and a cylinder through that join and a plane clear of it (plan step-reader step 5): at the join Arris reports one hit, at the start parameter, not one at each end; held to the count of Open CASCADE's general curve-surface intersector, whose two parameters of the join name one point; written by generate.py",
        "surfaces": surfaces,
        "curves": {"loop": loop},
        "samples": samples,
        "pairs": pairs,
    }


# --- c8-line-nurbs-hits ----------------------------------------------------------------


def nurbs_surface_point(spec, u, v):
    """A `nurbs` surface spec's point at `(u, v)`: de Boor on the
    homogeneous net along `v` in every row of control points, then along
    `u` (*The NURBS Book* A4.3)."""
    p, q = spec["degree"]
    ku, kv = spec["knots"]

    def boor(degree, knots, points, t):
        n = len(points)
        k = max(i for i in range(degree, n) if knots[i] <= t)
        d = [list(x) for x in points[k - degree : k + 1]]
        for r in range(1, degree + 1):
            for j in range(degree, r - 1, -1):
                i = k - degree + j
                alpha = (t - knots[i]) / (knots[i + degree + 1 - r] - knots[i])
                d[j] = [(1.0 - alpha) * x + alpha * y for x, y in zip(d[j - 1], d[j])]
        return d[degree]

    rows = [
        boor(q, kv, [mul(w, pt) + [w] for pt, w in zip(row, weights)], v)
        for row, weights in zip(spec["control_points"], spec["weights"])
    ]
    h = boor(p, ku, rows, u)
    return mul(1.0 / h[3], h[:3])


def c8_line_nurbs_hits():
    """Lines against free-form and rational NURBS surfaces (plan ray-fire
    step 1): transversal crossings of a bicubic of four spans, one of them
    on a knot line, one on the other and one where the two meet; an
    octant of a sphere as a rational biquadratic, crossed inside, through
    its pole along the axis, and grazed; a quadratic profile extruded,
    with a line along one of its rulings; and a line one tolerance clear
    of the bicubic's crest, which misses it. Every line is drawn through
    a point of the surface it is paired with, at parameters committed
    here, so the hits are where they were put."""
    f = POSES["tilt"]

    def net(rows):
        return [[f.to_world(p) for p in row] for row in rows]

    # A bicubic over two spans each way: z a saddle with a tilt, on a 5 x 5
    # grid, so the knot lines u = 1 and v = 1 run through its middle.
    grid = [0.0, 1.0, 2.0, 3.0, 4.0]
    knots = [0.0] * 4 + [1.0] + [2.0] * 4
    hills = {
        "type": "nurbs",
        "degree": [3, 3],
        "knots": [knots, knots],
        "control_points": net([[[x, y, 0.3 * (x - 2.0) * (y - 2.0) + 0.1 * x] for y in grid] for x in grid]),
        "weights": [[1.0] * 5 for _ in grid],
    }
    # An octant of a sphere of radius 2 about the pose's origin: a quarter
    # circle in u times a quarter meridian in v, the row v = 1 the pole.
    r = 2.0
    s2 = math.sqrt(0.5)
    arc = [(1.0, 0.0, 1.0), (1.0, 1.0, s2), (0.0, 1.0, 1.0)]
    octant = {
        "type": "nurbs",
        "degree": [2, 2],
        "knots": [[0.0] * 3 + [1.0] * 3, [0.0] * 3 + [1.0] * 3],
        "control_points": net([[[r * x * a, r * y * a, r * b] for a, b, _ in arc] for x, y, _ in arc]),
        "weights": [[wu * wv for _, _, wv in arc] for _, _, wu in arc],
    }
    # A quadratic profile in (x, z) swept straight along y: a ruled surface,
    # each of its rulings a line along y.
    profile = [[-2.0, 0.0], [0.0, 2.0], [2.0, 0.5]]
    trough = {
        "type": "nurbs",
        "degree": [2, 1],
        "knots": [[0.0] * 3 + [1.0] * 3, [0.0, 0.0, 1.0, 1.0]],
        "control_points": net([[[x, y, z] for y in (-3.0, 3.0)] for x, z in profile]),
        "weights": [[1.0, 1.0] for _ in profile],
    }
    surfaces = {"hills": hills, "octant": octant, "trough": trough}
    curves = {}

    def through(name, spec, uv, direction, back=4.0):
        p = nurbs_surface_point(spec, *uv)
        d = unit(f.vec(direction))
        curves[name] = {"type": "line", "origin": sub(p, mul(back, d)), "direction": d}

    # The bicubic's normal at (u, v) is about (−∂z/∂x, −∂z/∂y, 1) in the
    # pose's frame; each line leans off it so it crosses at an angle.
    through("hills_inside", hills, (0.6, 1.4), [0.2, -0.1, 1.0])
    through("hills_on_u_knot", hills, (1.0, 0.4), [0.3, 0.2, 1.0])
    through("hills_on_v_knot", hills, (1.6, 1.0), [-0.2, 0.3, 1.0])
    through("hills_on_both_knots", hills, (1.0, 1.0), [0.25, -0.15, 1.0])
    through("octant_inside", octant, (0.4, 0.3), [-0.6, -0.5, -0.4])
    # Along the axis through the pole, from outside.
    curves["octant_pole"] = {"type": "line", "origin": f.to_world([0.0, 0.0, 4.0]), "direction": unit(f.vec([0.0, 0.0, -1.0]))}
    # Tangent at the point at (0.5, 0.5): along its parallel, which is
    # perpendicular to the radius there.
    graze_at = nurbs_surface_point(octant, 0.5, 0.5)
    radial = unit(sub(graze_at, f.origin))
    along = unit(cross(f.z, radial))
    curves["octant_graze"] = {"type": "line", "origin": sub(graze_at, mul(3.0, along)), "direction": along}
    # Along the trough's ruling at u = 0.3.
    ruling = nurbs_surface_point(trough, 0.3, 0.0)
    curves["trough_ruling"] = {"type": "line", "origin": sub(ruling, mul(2.0, f.vec([0.0, 1.0, 0.0]))), "direction": unit(f.vec([0.0, 1.0, 0.0]))}
    # Along the ruling on the trough's crest, two default tolerances (1e-7
    # each) above it along the normal there: a miss. The profile's height
    # is z(u) = 4u − 3.5u², greatest at u = 4/7, where its normal is z.
    crest = nurbs_surface_point(trough, 4.0 / 7.0, 0.0)
    curves["trough_clear"] = {"type": "line", "origin": add(sub(crest, mul(2.0, f.vec([0.0, 1.0, 0.0]))), mul(2e-7, f.vec([0.0, 0.0, 1.0]))), "direction": unit(f.vec([0.0, 1.0, 0.0]))}
    pairs = [{"a": c, "b": c.split("_")[0]} for c in curves]
    samples = [
        {"of": "hills", "params": [[0.6, 1.4], [1.0, 1.0], [2.0, 2.0]], "points": []},
        {"of": "octant", "params": [[0.0, 0.0], [0.5, 0.5], [1.0, 1.0]], "points": []},
        {"of": "trough", "params": [[0.3, 0.0], [0.5, 1.0]], "points": []},
    ]
    return {
        "kind": "geometry",
        "description": "Lines against NURBS surfaces in the tilt pose (plan ray-fire step 1): a bicubic of two spans each way crossed inside, on its knot line u = 1, on v = 1 and where the two meet; an octant of a sphere as a rational biquadratic crossed inside, along its axis through its pole and grazed along a parallel; a quadratic profile extruded along y, met along one of its rulings, and passed by a line two tolerances above its crest — through Open CASCADE's general curve-surface intersector, GeomAPI_IntCS; written by generate.py",
        "surfaces": surfaces,
        "curves": curves,
        "samples": samples,
        "pairs": pairs,
    }


def write(name, recipe):
    directory = HERE / name
    directory.mkdir(exist_ok=True)
    (directory / "fixture.json").write_text(json.dumps(recipe, indent=2) + "\n")
    print(f"{directory}: {len(recipe['surfaces'])} surfaces, {len(recipe['curves'])} curves, {len(recipe['samples'])} samples, {len(recipe['pairs'])} pairs")


if __name__ == "__main__":
    write("analytic-eval", analytic_eval())
    write("c1-intersections", c1_intersections())
    write("c2-cylinder-pairs", c2_cylinder_pairs())
    write("c2-quadric-pairs", c2_quadric_pairs())
    write("c3-cylinder-pairs", c3_cylinder_pairs())
    write("c3-quadric-pairs", c3_quadric_pairs())
    write("c3-torus-pairs", c3_torus_pairs())
    write("c3-conic-hits", c3_conic_hits())
    write("c3-nurbs-hits", c3_nurbs_hits())
    write("c3-nurbs-crossings", c3_nurbs_crossings())
    write("c4-nurbs-projections", c4_nurbs_projections())
    write("c4-closed-curve-hits", c4_closed_curve_hits())
    write("c8-line-nurbs-hits", c8_line_nurbs_hits())
