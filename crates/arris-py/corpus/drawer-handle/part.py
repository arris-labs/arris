"""A drawer pull: an 8 mm round bar swept along a U path (two legs, two
quarter bends, a 40 mm grip), a tapped hole in each foot, the feet chamfered."""

import math

import arris

BAR_RADIUS = 4.0
SPAN, LEG, BEND = 60.0, 15.0, 10.0
FEET = [(0.0, 0.0), (SPAN, 0.0)]
TAP_RADIUS, TAP_DEPTH = 1.5, 8.0
CHAMFER = 0.5


def build(run):
    model = run.model

    bar = arris.Profile(arris.Loop.circle((0, 0), BAR_RADIUS), normal=(0, 0, 1), x_axis=(1, 0, 0))
    c = math.sqrt(0.5)
    top = LEG + BEND
    path = arris.Path(
        (0, 0, 0),
        [
            arris.PathSegment.line((0, 0, LEG)),
            arris.PathSegment.arc((BEND, 0, top), (BEND - BEND * c, 0, LEG + BEND * c)),
            arris.PathSegment.line((SPAN - BEND, 0, top)),
            arris.PathSegment.arc((SPAN, 0, LEG), (SPAN - BEND + BEND * c, 0, LEG + BEND * c)),
            arris.PathSegment.line((SPAN, 0, 0)),
        ],
    )
    handle, _ = run.stage(
        "handle", lambda: model.sweep(bar, path, arris.FrameLaw.rotation_minimizing())
    )

    taps = [
        model.primitive_cylinder((x, y, -1), (0, 0, 1), TAP_RADIUS, TAP_DEPTH + 1)[0] for x, y in FEET
    ]
    body, _ = run.stage("tapped", lambda: model.cut_many(handle, taps))

    # Each foot's rim: the circle of the bar's radius on the plane z = 0.
    rims = [
        edge
        for edge, points in run.edge_points(body).items()
        if all(
            abs(p[2]) < 1e-6
            and any(abs(math.hypot(p[0] - x, p[1] - y) - BAR_RADIUS) < 1e-6 for x, y in FEET)
            for p in points
        )
    ]
    run.stage("foot_chamfer", lambda: model.chamfer(body, rims, CHAMFER), finish=True)
