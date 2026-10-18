"""A vase: four circles (radius 18, 26, 14, 20 mm at heights 0, 20, 50, 80)
lofted ruled, the base rim rounded, then hollowed to a 2 mm wall open at
the top."""

import arris

SECTIONS = [(0.0, 18.0), (20.0, 26.0), (50.0, 14.0), (80.0, 20.0)]
WALL = 2.0
BASE_ROUND = 1.0


def build(run):
    model = run.model

    circles = [
        arris.Profile(arris.Loop.circle((0, 0), r), origin=(0, 0, z), normal=(0, 0, 1), x_axis=(1, 0, 0))
        for z, r in SECTIONS
    ]
    body, _ = run.stage("body", lambda: model.loft(circles, "ruled"))

    # The base rim: the circle on the plane z = 0.
    rim = [
        edge
        for edge, points in run.edge_points(body).items()
        if all(abs(p[2]) < 1e-6 for p in points) and abs(points[0][0] ** 2 + points[0][1] ** 2 - SECTIONS[0][1] ** 2) < 1e-3
    ]
    rounded = run.stage("base_round", lambda: model.fillet(body, rim, BASE_ROUND), finish=True)
    if rounded:
        body = rounded[0]

    top = SECTIONS[-1][0]
    [mouth] = [
        face
        for face in model.faces(body)
        if (plane := run.plane(face)) and abs(plane[0][2] - top) < 1e-6
    ]
    run.stage("hollow", lambda: model.shell(body, [mouth], WALL, "inward"))
