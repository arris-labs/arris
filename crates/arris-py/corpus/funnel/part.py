"""A funnel: a ruled loft from a 10 mm stem (15 mm long) to a 60 mm mouth at
40 mm, the crease between stem and cone rounded, then hollowed to a 1.5 mm
wall open at both ends."""

import arris

SECTIONS = [(0.0, 5.0), (15.0, 5.0), (40.0, 30.0)]
NECK_ROUND = 3.0
WALL = 1.5


def build(run):
    model = run.model

    circles = [
        arris.Profile(arris.Loop.circle((0, 0), r), origin=(0, 0, z), normal=(0, 0, 1), x_axis=(1, 0, 0))
        for z, r in SECTIONS
    ]
    body, _ = run.stage("body", lambda: model.loft(circles, "ruled"))

    # The crease: the circle of the stem's radius at the stem's top.
    crease = [
        edge
        for edge, points in run.edge_points(body).items()
        if all(abs(p[2] - SECTIONS[1][0]) < 1e-6 for p in points)
    ]
    rounded = run.stage("neck_round", lambda: model.fillet(body, crease, NECK_ROUND), finish=True)
    if rounded:
        body = rounded[0]

    # The two openings: the planar faces at the stem's foot and the mouth.
    ends = [
        face
        for face in model.faces(body)
        if (plane := run.plane(face)) and min(abs(plane[0][2]), abs(plane[0][2] - SECTIONS[-1][0])) < 1e-6
    ]
    run.stage("hollow", lambda: model.shell(body, ends, WALL, "inward"))
