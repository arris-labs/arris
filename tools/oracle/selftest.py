#!/usr/bin/env python3
"""selftest.py [fixture-dir...]  — prove the oracle agrees with itself.

For an inline smoke recipe and then for every fixture directory (default:
all under tests/fixtures/): build the recipe, compute its expected values,
check they equal the committed expected.json (if any), write Open CASCADE's
own STEP of the result to a temporary file and run the comparison on it.
A result with no solid (`degenerate`) and one Arris refuses by design
(`analytic.expect_error`) are recorded but not round-tripped: neither is
ever read back through STEP by the corpus. One whose recipe says the
oracle measures it wrong (`analytic.measure_differs`) is round-tripped
on its counts, genus and probes only. One whose recipe says Open
CASCADE's own STEP of it does not read back as itself
(`analytic.step_differs`, ADR-0023) is recorded but not round-tripped.
Every other result is held to the tolerances it declares itself, the
fixture's where they are wider (`measure.within_own_tolerance`).
Exits 1 on the first disagreement. Run as
`uv run --project tools/oracle tools/oracle/selftest.py`.
"""

import json
import math
import sys
import tempfile
from pathlib import Path

from oracle import OracleError, require_ocp

require_ocp()

from oracle import step  # noqa: E402
from oracle.fixture import compute_expected, corpus_root, fixture_dirs, load_expected, load_fixture  # noqa: E402
from oracle.measure import DEFAULT_TOLERANCES, compare, format_table, measure, within_own_tolerance  # noqa: E402
from oracle.recipe import build, fixture_kind, number, probes, variant_names  # noqa: E402

PI = 3.141592653589793
MEASUREMENTS = ("volume", "area", "centroid", "inertia")

# Inline recipes exercising every op the interpreter has, each with the
# closed form it must reproduce. The fixture corpus (tests/fixtures/) holds
# the real ones; these run even on a checkout with no corpus yet.
SMOKES = [
    {
        "name": "box − cylinder, params and a variant",
        "recipe": {
            "params": {"r": 4},
            "variants": {"wider": {"r": 5}},
            "steps": [
                {"name": "plate", "op": "box", "min": [0, 0, 0], "max": [40, 30, 10]},
                {"name": "hole", "op": "cylinder", "base": [20, 15, -1], "axis": [0, 0, 1], "radius": "r", "height": 12},
                {"name": "result", "op": "cut", "target": "plate", "tool": "hole"},
            ],
            "result": "result",
            "probes": [
                {"label": "inside", "point": [5, 5, 5]},
                {"label": "in_hole", "point": [20, 15, 5]},
                {"label": "on_top", "point": [5, 5, 10]},
            ],
        },
        "analytic": {
            "default": {"volume": 12000 - PI * 16 * 10, "area": 3800 - 2 * PI * 16 + 2 * PI * 4 * 10, "counts": (10, 15, 7, 9), "genus": 1},
            "wider": {"volume": 12000 - PI * 25 * 10, "genus": 1},
        },
        "probes": {"inside": "in", "in_hole": "out", "on_top": "on"},
    },
    {
        "name": "profile with a hole, extruded (same numbers as the cut)",
        "recipe": {
            "steps": [
                {
                    "name": "sketch",
                    "op": "profile",
                    "plane": {"origin": [0, 0, 0], "x": [1, 0, 0], "y": [0, 1, 0]},
                    "outer": {"start": [0, 0], "segments": [{"line_to": [40, 0]}, {"line_to": [40, 30]}, {"line_to": [0, 30]}, {"line_to": [0, 0]}]},
                    "holes": [{"circle": {"center": [20, 15], "radius": 4}}],
                },
                {"name": "result", "op": "extrude", "profile": "sketch", "direction": [0, 0, 2], "length": 10},
            ],
            "result": "result",
        },
        "analytic": {"default": {"volume": 12000 - PI * 16 * 10, "area": 3800 - 2 * PI * 16 + 2 * PI * 4 * 10, "counts": (10, 15, 7, 9), "genus": 1}},
    },
    {
        "name": "rectangle revolved into a tube, then a quarter",
        "recipe": {
            "params": {"angle": 360},
            "variants": {"quarter": {"angle": 90}},
            "steps": [
                {
                    "name": "rect",
                    "op": "profile",
                    "plane": {"origin": [0, 0, 0], "x": [1, 0, 0], "y": [0, 0, 1]},
                    "outer": {"start": [1, -1], "segments": [{"line_to": [2, -1]}, {"line_to": [2, 1]}, {"line_to": [1, 1]}, {"line_to": [1, -1]}]},
                },
                {"name": "result", "op": "revolve", "profile": "rect", "axis": {"origin": [0, 0, 0], "direction": [0, 0, 1]}, "angle_deg": "angle"},
            ],
            "result": "result",
        },
        "analytic": {
            "default": {"volume": 2 * PI * (4 - 1) / 2 * 2, "area": 2 * PI * (1 + 2) * 2 + 2 * PI * (4 - 1), "counts": (4, 6, 4, 6), "genus": 1},
            "quarter": {"volume": (2 * PI * (4 - 1) / 2 * 2) / 4, "area": (2 * PI * (1 + 2) * 2 + 2 * PI * (4 - 1)) / 4 + 2 * 2, "counts": (8, 12, 6, 6), "genus": 0},
        },
    },
    {
        "name": "arc profile: half disc extruded",
        "recipe": {
            "steps": [
                {
                    "name": "half",
                    "op": "profile",
                    "plane": {"origin": [0, 0, 0], "x": [1, 0, 0], "y": [0, 1, 0]},
                    "outer": {"start": [-2, 0], "segments": [{"line_to": [2, 0]}, {"arc_to": [-2, 0], "via": [0, 2]}]},
                },
                {"name": "result", "op": "extrude", "profile": "half", "direction": [0, 0, 1], "length": 3},
            ],
            "result": "result",
        },
        "analytic": {"default": {"volume": PI * 4 / 2 * 3, "area": PI * 4 + (PI * 2 + 4) * 3, "counts": (4, 6, 4, 4), "genus": 0}},
    },
    {
        "name": "mirror of an off-origin box in x = 0",
        "recipe": {
            "steps": [
                {"name": "a", "op": "box", "min": [1, 2, 3], "max": [3, 5, 9]},
                {"name": "result", "op": "mirror", "of": "a", "plane": {"origin": [0, 0, 0], "normal": [1, 0, 0]}},
            ],
            "result": "result",
        },
        "analytic": {"default": {"volume": 36.0, "area": 2 * (2 * 3 + 2 * 6 + 3 * 6), "centroid": [-2.0, 3.5, 6.0], "counts": (8, 12, 6, 6), "genus": 0}},
    },
    {
        "name": "transform then fuse, common and cut of two corner cubes",
        "recipe": {
            "params": {"which": 0},
            "steps": [
                {"name": "a", "op": "box", "min": [-1, -1, -1], "max": [1, 1, 1]},
                {"name": "b0", "op": "box", "min": [-1, -1, -1], "max": [1, 1, 1]},
                {"name": "b", "op": "transform", "of": "b0", "translate": [1, 1, 1], "rotate": {"axis": [0, 0, 1], "angle_deg": 90}},
                {"name": "result", "op": "fuse", "a": "a", "b": "b"},
                {"name": "inter", "op": "common", "a": "a", "b": "b"},
                {"name": "diff", "op": "cut", "target": "a", "tool": "b"},
            ],
            "result": "result",
        },
        "analytic": {"default": {"volume": 15.0, "area": 42.0, "counts": (20, 30, 12, 12), "genus": 0}},
        "others": {"inter": {"volume": 1.0, "area": 6.0, "counts": (8, 12, 6, 6)}, "diff": {"volume": 7.0, "area": 24.0, "counts": (14, 21, 9, 9)}},
    },
    {
        "name": "flush union keeps coplanar neighbours; flush common is degenerate",
        "recipe": {
            "steps": [
                {"name": "a", "op": "box", "min": [0, 0, 0], "max": [40, 30, 10]},
                {"name": "b", "op": "box", "min": [40, 0, 0], "max": [80, 30, 10]},
                {"name": "result", "op": "fuse", "a": "a", "b": "b"},
                {"name": "flush", "op": "common", "a": "a", "b": "b"},
            ],
            "result": "result",
        },
        "analytic": {"default": {"volume": 24000.0, "area": 7000.0, "counts": (12, 20, 10, 10), "genus": 0}},
        "degenerate": ["flush"],
    },
    {
        "name": "one edge of a cube filleted, named by a point on it",
        "recipe": {
            "params": {"r": 0.2},
            "steps": [
                {"name": "cube", "op": "box", "min": [0, 0, 0], "max": [2, 2, 2]},
                {"name": "result", "op": "fillet", "of": "cube", "edges": [[2, 2, 1]], "radius": "r"},
            ],
            "result": "result",
            "probes": [
                {"label": "inside", "point": [1, 1, 1]},
                {"label": "corner_gone", "point": [1.95, 1.95, 1]},
                {"label": "on_blend", "point": ["2 - r + r / sqrt(2)", "2 - r + r / sqrt(2)", 1]},
            ],
        },
        "analytic": {"default": {"volume": 8 - (1 - PI / 4) * 0.04 * 2, "area": 24 - 0.8 - 2 * (1 - PI / 4) * 0.04 + PI * 0.2, "counts": (10, 15, 7, 7), "genus": 0}},
        "probes": {"inside": "in", "corner_gone": "out", "on_blend": "on"},
    },
    {
        "name": "one edge of a cube chamfered, named by a point on it",
        "recipe": {
            "params": {"d": 0.2},
            "steps": [
                {"name": "cube", "op": "box", "min": [0, 0, 0], "max": [2, 2, 2]},
                {"name": "result", "op": "chamfer", "of": "cube", "edges": [[2, 2, 1]], "distance": "d"},
            ],
            "result": "result",
            "probes": [
                {"label": "inside", "point": [1, 1, 1]},
                {"label": "corner_gone", "point": [1.95, 1.95, 1]},
                {"label": "on_chamfer", "point": ["2 - d / 2", "2 - d / 2", 1]},
            ],
        },
        "analytic": {"default": {"volume": 8 - 0.04, "area": 24 - 0.8 - 0.04 + 2 * math.sqrt(2) * 0.2, "counts": (10, 15, 7, 7), "genus": 0}},
        "probes": {"inside": "in", "corner_gone": "out", "on_chamfer": "on"},
    },
    {
        "name": "a triangular prism from points and faces of indices",
        "recipe": {
            "params": {"h": 3},
            "steps": [
                {
                    "name": "result",
                    "op": "polyhedron",
                    "namespace": 0,
                    "points": [[0, 0, 0], [2, 0, 0], [0, 2, 0], [0, 0, "h"], [2, 0, "h"], [0, 2, "h"]],
                    "faces": [[[0, 2, 1]], [[3, 4, 5]], [[0, 1, 4, 3]], [[1, 2, 5, 4]], [[2, 0, 3, 5]]],
                },
            ],
            "result": "result",
            "probes": [
                {"label": "inside", "point": [0.5, 0.5, 1]},
                {"label": "beyond_slant", "point": [1.5, 1.5, 1]},
                {"label": "on_slant", "point": [1, 1, 1]},
            ],
        },
        "analytic": {"default": {"volume": 6, "area": 4 + (4 + math.sqrt(8)) * 3, "counts": (6, 9, 5, 5), "genus": 0}},
        "probes": {"inside": "in", "beyond_slant": "out", "on_slant": "on"},
    },
]


# A geometry recipe against closed forms: the world-frame cylinder of the
# fixtures above evaluated, projected onto, and cut by a cap, an oblique
# plane, a ruling, a ring and a NURBS arc, and the arc against the chord
# — every kind of result the geometry oracle writes
# (tests/fixtures/README.md, "kind": "geometry").
S2 = math.sqrt(0.5)
GEOMETRY_SMOKE = {
    "name": "geometry: cylinder r 2 evaluated, projected onto and cut",
    "recipe": {
        "kind": "geometry",
        "params": {"R": 2},
        "surfaces": {
            "wall": {"type": "cylinder", "origin": [0, 0, 0], "z": [0, 0, 1], "x": [1, 0, 0], "radius": "R"},
            "cap": {"type": "plane", "origin": [0, 0, 5], "z": [0, 0, 1], "x": [1, 0, 0]},
            "oblique": {"type": "plane", "origin": [0, 0, 0], "z": [1, 0, 1], "x": [0, 1, 0]},
            "side": {"type": "plane", "origin": ["R", 0, 0], "z": [1, 0, 0], "x": [0, 0, 1]},
        },
        "curves": {
            "ruling": {"type": "line", "origin": ["R", 0, 1], "direction": [0, 0, 1]},
            "chord": {"type": "line", "origin": [0, 0, 1], "direction": [1, 0, 0]},
            "ring": {"type": "circle", "origin": [0, 0, 0], "z": [0, 1, 0], "x": [1, 0, 0], "radius": 3},
            # A quarter of the circle of radius 3 in the xz plane, as one
            # rational quadratic arc.
            "arc": {
                "type": "nurbs",
                "degree": 2,
                "knots": [0, 0, 0, 1, 1, 1],
                "control_points": [[3, 0, 0], [3, 0, 3], [0, 0, 3]],
                "weights": [1, "sqrt(0.5)", 1],
            },
        },
        "samples": [
            {"of": "wall", "params": [[0, 0], ["pi / 2", 3]], "points": [[0, -5, 3], [1, 0, 2]]},
            {"of": "ring", "params": ["pi"], "points": [[-4, 0, 0]]},
            {"of": "arc", "params": [0.5], "points": [[4, 0, 4]]},
        ],
        "pairs": [
            {"a": "cap", "b": "wall"},
            {"a": "oblique", "b": "wall"},
            {"a": "side", "b": "wall"},
            {"a": "cap", "b": "side"},
            {"a": "ruling", "b": "wall"},
            {"a": "chord", "b": "wall"},
            {"a": "ring", "b": "wall"},
            {"a": "chord", "b": "cap"},
            {"a": "arc", "b": "wall"},
            {"a": "arc", "b": "chord"},
        ],
    },
    "evaluations": {"wall": [[2, 0, 0], [0, 2, 3]], "ring": [[-3, 0, 0]], "arc": [[3 * S2, 0, 3 * S2]]},
    "derivatives": {"wall": [{"du": [0, 2, 0], "dv": [0, 0, 1], "duu": [-2, 0, 0]}]},
    "projections": {"wall": [{"uv": [3 * PI / 2, 3], "distance": 3}, {"uv": [0, 2], "distance": 1}], "ring": [{"t": PI, "distance": 1}], "arc": [{"t": 0.5, "distance": 4 / S2 - 3}]},
    "pairs": [
        ("circle", 1),
        ("ellipse", 1),
        ("line", 1),
        ("line", 1),
        ("coincident", 0),
        ("points", 2),
        ("points", 4),
        ("points", 0),
        ("points", 1),
        ("points", 1),
    ],
    "ellipse_axes": (2 / S2, 2),
    # Where the arc crosses the wall: x = R on the circle of radius 3.
    "arc_hit": [2, 0, math.sqrt(5.0)],
    # Where the arc crosses the chord, at z = 1, and the chord's parameter
    # there, the distance from its origin on the axis.
    "arc_chord": ([math.sqrt(8.0), 0, 1], math.sqrt(8.0)),
}


def _close(a, b, tol=1e-9) -> bool:
    return all(abs(x - y) <= tol * max(1.0, abs(x), abs(y)) for x, y in zip(a, b))


def check_geometry_smoke() -> bool:
    smoke = GEOMETRY_SMOKE
    print(f"smoke: {smoke['name']}")
    expected = compute_expected(smoke["recipe"])
    ok = expected.get("kind") == "geometry"
    by_name = {s["of"]: s for s in expected["samples"]}
    for name, points in smoke["evaluations"].items():
        for e, p in zip(by_name[name]["evaluations"], points):
            if not _close(e["point"], p):
                print(f"  {name} at {e['at']}: {e['point']} differs from {p}")
                ok = False
    for name, rows in smoke["derivatives"].items():
        for e, row in zip(by_name[name]["evaluations"], rows):
            for key, value in row.items():
                if not _close(e[key], value):
                    print(f"  {name} at {e['at']}: {key} {e[key]} differs from {value}")
                    ok = False
    for name, rows in smoke["projections"].items():
        for e, row in zip(by_name[name]["projections"], rows):
            got = e["uv"] if "uv" in row else [e["t"]]
            want = row["uv"] if "uv" in row else [row["t"]]
            if not _close(got, want) or abs(e["distance"] - row["distance"]) > 1e-9:
                print(f"  {name} projection of {e['point']}: {got} at {e['distance']} differs from {want} at {row['distance']}")
                ok = False
    for pair, (kind, count) in zip(expected["pairs"], smoke["pairs"]):
        got = len(pair.get("curves", pair.get("hits", [])))
        if pair["type"] != kind or got != count:
            print(f"  {pair['a']} vs {pair['b']}: {pair['type']} with {got} differs from {kind} with {count}")
            ok = False
    # The oblique section's sampled points lie on the ellipse of the
    # closed form: minor radius R, major R / cos 45°, centred at the origin.
    big, small = smoke["ellipse_axes"]
    for p in expected["pairs"][1]["curves"][0]["points"]:
        # Major axis along (−1, 0, 1)/√2 (or its opposite), minor along y.
        along = (-p[0] + p[2]) * S2
        if abs((along / big) ** 2 + (p[1] / small) ** 2 - 1.0) > 1e-9 or abs(p[0] + p[2]) > 1e-9:
            print(f"  oblique section point {p} is off the closed-form ellipse")
            ok = False
    hits = expected["pairs"][8].get("hits", [])
    if len(hits) != 1 or not _close(hits[0]["point"], smoke["arc_hit"]):
        print(f"  arc vs wall: {hits} differs from the closed form {smoke['arc_hit']}")
        ok = False
    hits = expected["pairs"][9].get("hits", [])
    point, tb = smoke["arc_chord"]
    if len(hits) != 1 or not _close(hits[0]["point"], point) or not _close([hits[0]["tb"]], [tb]):
        print(f"  arc vs chord: {hits} differs from the closed form {point} at tb = {tb}")
        ok = False
    return ok


def check_analytic(name: str, result: dict, analytic: dict) -> bool:
    ok = True
    for key in ("volume", "area"):
        if key in analytic:
            e, a = analytic[key], result[key]
            if abs(a - e) > 1e-9 * abs(e):
                print(f"  {name}: {key} {a!r} differs from analytic {e!r}")
                ok = False
    if "counts" in analytic:
        c = result["counts"]
        got = (c["vertices"], c["edges"], c["faces"], c["loops"])
        if got != tuple(analytic["counts"]):
            print(f"  {name}: counts V/E/F/L {got} differ from analytic {analytic['counts']}")
            ok = False
    if "genus" in analytic and result["genus"] != analytic["genus"]:
        print(f"  {name}: genus {result['genus']} differs from analytic {analytic['genus']}")
        ok = False
    return ok


def check_degenerate_edge_smoke(tmp: Path) -> bool:
    """Open CASCADE's own cone and sphere carry degenerate edges at the apex
    and the poles. Left out of the edge count, both derive genus 0 with an
    even characteristic, before and after a STEP round trip."""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCone, BRepPrimAPI_MakeSphere

    print("smoke: degenerate edges are not counted (BRepPrimAPI cone and sphere)")
    ok = True
    shapes = {
        # V apex and rim, E seam and rim (the apex's degenerate edge not
        # counted), F wall and base, one wire each.
        "cone": (BRepPrimAPI_MakeCone(1.0, 0.0, 1.0).Shape(), (2, 2, 2, 2)),
        # V the poles, E the seam (two degenerate pole edges not counted).
        "sphere": (BRepPrimAPI_MakeSphere(1.0).Shape(), (2, 1, 1, 1)),
    }
    for name, (shape, counts) in shapes.items():
        path = tmp / f"degenerate-{name}.step"
        step.write(shape, path)
        for label, s in (("built", shape), ("read back", step.read(path))):
            try:
                result = measure(s, [], DEFAULT_TOLERANCES["probe"])
            except OracleError as e:
                print(f"  {name} {label}: {e}")
                ok = False
                continue
            c = result["counts"]
            got = (c["vertices"], c["edges"], c["faces"], c["loops"])
            if got != counts or result["genus"] != 0 or result["euler_characteristic"] % 2 != 0:
                print(f"  {name} {label}: V/E/F/L {got} genus {result['genus']} differ from {counts} genus 0")
                ok = False
    return ok


def check_step_solids_smoke(tmp: Path) -> bool:
    """`step.solids` names each solid it reads by the `#id` of its
    `MANIFOLD_SOLID_BREP`, which Open CASCADE's model does not keep: a box
    and a cylinder written into one file come back each under the `#id`
    whose shell has its own face count. And the instance scan skips a `;`
    or a `#id =` inside a string or a comment."""
    from OCP.BRep import BRep_Builder
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.TopoDS import TopoDS_Compound
    from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt

    print("smoke: the solids of a STEP file named by their #id")
    ok = True
    text = "DATA;\n#1=A('x;#9=B',/* ;#8=C */ 2);\n/* #6=F; */ #2 = B(''';#7=D''');\nENDSEC;\nDATA;#3=E();ENDSEC;"
    found = step.instances(text)
    if sorted(found) != [1, 2, 3]:
        print(f"  the instance scan found {sorted(found)}, not [1, 2, 3]")
        ok = False
    compound = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound)
    builder.Add(compound, BRepPrimAPI_MakeBox(2.0, 3.0, 4.0).Shape())
    builder.Add(compound, BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(10, 0, 0), gp_Dir(0, 0, 1)), 1.0, 2.0).Shape())
    path = tmp / "two-solids.step"
    step.write(compound, path)
    bodies = step.instances(path.read_text())
    faces = {}
    for label, body in bodies.items():
        if body.lstrip().startswith("MANIFOLD_SOLID_BREP"):
            shell = int(body.split("#")[-1].rstrip(") "))
            faces[label] = bodies[shell].count("#")
    read = {label: measure(solid, [], DEFAULT_TOLERANCES["probe"])["volume"] for label, solid in step.solids(path)}
    expect = {label: 24.0 if n == 6 else PI * 2.0 for label, n in faces.items()}
    if sorted(read) != sorted(expect) or any(not _close([read[k]], [expect[k]]) for k in expect):
        print(f"  solids by #id {read} differ from {expect}")
        ok = False
    return ok


def check_expr_cases() -> bool:
    """`tests/fixtures/expr-cases.json`: the same grammar cases
    `arris_debug::fixtures::expr`'s test evaluates too, so `^`, `**` and
    the rest mean the same thing on both sides."""
    print("smoke: the expression cases both sides evaluate")
    cases = json.loads((corpus_root() / "expr-cases.json").read_text())
    ok = True
    for case in cases:
        expr, params = case["expr"], case["params"]
        if case.get("error"):
            try:
                number(expr, params)
            except OracleError:
                continue
            print(f"  {expr!r}: expected an error, got a value")
            ok = False
            continue
        try:
            got = number(expr, params)
        except OracleError as e:
            print(f"  {expr!r}: {e}")
            ok = False
            continue
        expect = case["expect"]
        if abs(got - expect) > 1e-6:
            print(f"  {expr!r}: {got} != {expect}")
            ok = False
    return ok


def run_smokes(tmp: Path) -> bool:
    ok = check_geometry_smoke()
    ok &= check_expr_cases()
    ok &= check_degenerate_edge_smoke(tmp)
    ok &= check_step_solids_smoke(tmp)
    for smoke in SMOKES:
        recipe = smoke["recipe"]
        print(f"smoke: {smoke['name']}")
        expected = compute_expected(recipe)
        for variant, analytic in smoke["analytic"].items():
            ok &= check_analytic(f"[{variant}]", expected["results"][variant], analytic)
        if "probes" in smoke:
            probes = {p["label"]: p["class"] for p in expected["results"]["default"]["probes"]}
            if probes != smoke["probes"]:
                print(f"  probes {probes} differ from {smoke['probes']}")
                ok = False
        _, shapes = build(recipe)
        for step_name, analytic in smoke.get("others", {}).items():
            ok &= check_analytic(step_name, measure(shapes[step_name], [], DEFAULT_TOLERANCES["probe"]), analytic)
        for step_name in smoke.get("degenerate", []):
            if not measure(shapes[step_name], [], DEFAULT_TOLERANCES["probe"])["degenerate"]:
                print(f"  {step_name}: expected a degenerate (no-solid) result")
                ok = False
        ok &= round_trip("smoke", recipe, expected, tmp)
    return ok


def round_trip(name: str, fixture: dict, expected: dict, tmp: Path) -> bool:
    tol = {**DEFAULT_TOLERANCES, **fixture.get("tolerances", {})}
    # A result the recipe marks `analytic.expect_error` is recorded only as
    # what Open CASCADE builds for a case Arris refuses by design: the
    # corpus runner stops at the typed refusal and never reads that result
    # back through STEP, so a round trip of it proves nothing the corpus
    # uses. `boolean/tangent-hole` could not survive one in any case — its
    # slit carries the tangent ruling as an edge of four faces, and Open
    # CASCADE's own reader does not give that back as a closed surface.
    refused = fixture.get("analytic", {}).get("expect_error")
    # A result the recipe says Open CASCADE measures wrong
    # (`analytic.measure_differs`, ADR-0015) is compared on what the corpus
    # still takes from the oracle — counts, genus, probes: the runner and
    # compare.py hold Arris's measurements to the closed forms, and a body
    # the oracle built wrong need not even measure the same after its own
    # STEP round trip (`boolean/seam-beside-crossing-fuse` does not).
    measured_wrong = fixture.get("analytic", {}).get("measure_differs")
    # A result the recipe says Open CASCADE's own STEP loses (ADR-0023):
    # its reader hands back other topology or another solid for its own
    # file, while Arris's STEP of the same result reads back true — which
    # is what the corpus runner's compare.py stage checks, and still does.
    lossy = fixture.get("analytic", {}).get("step_differs")
    ok_all = True
    for variant in variant_names(fixture):
        shape, _ = build(fixture, variant)
        if expected["results"][variant]["degenerate"]:
            print(f"  {name}[{variant}]: degenerate, no STEP round trip")
            continue
        if refused:
            print(f"  {name}[{variant}]: {refused}, refused by Arris, no STEP round trip")
            continue
        if lossy:
            print(f"  {name}[{variant}]: Open CASCADE's own STEP differs (analytic.step_differs), no round trip")
            continue
        path = tmp / f"{name.replace('/', '_')}-{variant}.step"
        step.write(shape, path)
        back = step.read(path)
        actual = measure(back, probes(fixture, variant), tol["probe"])
        # The reader may move the boundary within the tolerances the file
        # carries (ADR-0023); the fixture's own tolerance where it is wider.
        rows = compare(expected["results"][variant], actual, within_own_tolerance(tol, shape, expected["results"][variant]))
        if measured_wrong:
            rows = [r for r in rows if not r[0].startswith(MEASUREMENTS)]
        ok = all(r[3] for r in rows)
        ok_all &= ok
        print(f"  {name}[{variant}]: STEP round trip {'ok' if ok else 'MISMATCH'}")
        if not ok:
            print(format_table(rows))
    return ok_all


def check_committed(name: str, fresh: dict, committed: dict | None) -> bool:
    if committed is None:
        print(f"  {name}: no expected.json committed yet")
        return True
    a = json.dumps(fresh, sort_keys=True)
    b = json.dumps(committed, sort_keys=True)
    if a == b:
        print(f"  {name}: expected.json matches a fresh run")
        return True
    print(f"  {name}: expected.json DIFFERS from a fresh run (rerun expected.py, commit as fixtures:)")
    return False


def main(argv: list[str]) -> int:
    dirs = [Path(a) for a in argv] if argv else fixture_dirs()
    ok = True
    with tempfile.TemporaryDirectory(prefix="arris-oracle-") as tmpdir:
        tmp = Path(tmpdir)
        ok &= run_smokes(tmp)
        print(f"{len(dirs)} fixture director{'y' if len(dirs) == 1 else 'ies'}:")
        for directory in dirs:
            name = str(directory.relative_to(directory.parents[1])) if len(directory.parents) > 1 else str(directory)
            try:
                fixture = load_fixture(directory)
                fresh = compute_expected(fixture)
                ok &= check_committed(name, fresh, load_expected(directory))
                if fixture_kind(fixture) == "geometry":
                    print(f"  {name}: geometry, no STEP round trip")
                elif fixture_kind(fixture) == "part":
                    print(f"  {name}: a part, read from its own STEP file; no round trip")
                else:
                    ok &= round_trip(name, fixture, fresh, tmp)
            except OracleError as e:
                print(f"  {name}: ERROR {e}")
                ok = False
    print("selftest: " + ("ok" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
