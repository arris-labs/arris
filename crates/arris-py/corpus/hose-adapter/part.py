"""A hose adapter: a 24 mm tube, 10 mm bore, two barbs on one end and a
collar with wrench flats; the collar's two rims chamfered."""

import math

import arris

COLLAR_RADIUS, COLLAR_FROM, COLLAR_TO = 12.0, 10.0, 14.0
FLAT_AT = 10.0
CHAMFER = 1.0


def build(run):
    model = run.model

    section = arris.Profile(
        arris.Loop.polygon(
            [
                (5, 0), (7, 0), (8.5, 4), (7, 4), (8.5, 8), (7, 8), (7, 10),
                (COLLAR_RADIUS, COLLAR_FROM), (COLLAR_RADIUS, COLLAR_TO),
                (7, 14), (7, 24), (5, 24),
            ]
        ),
        normal=(0, -1, 0),
        x_axis=(1, 0, 0),
    )
    body, _ = run.stage("body", lambda: model.revolve(section, (0, 0, 0), (0, 0, 1), 2 * math.pi))

    # The collar's rims: the circles of its radius at its two ends.
    rims = [
        edge
        for edge, points in run.edge_points(body).items()
        if abs(points[0][2] - points[-1][2]) < 1e-6
        and all(
            abs(math.hypot(p[0], p[1]) - COLLAR_RADIUS) < 1e-6
            and min(abs(p[2] - COLLAR_FROM), abs(p[2] - COLLAR_TO)) < 1e-6
            for p in points
        )
    ]
    chamfered = run.stage("chamfered", lambda: model.chamfer(body, rims, CHAMFER), finish=True)
    if chamfered:
        body = chamfered[0]

    # Two flats, a box each on opposite sides of the collar.
    boxes = [
        model.primitive_box((FLAT_AT, -13, 9), (13, 13, 15))[0],
        model.primitive_box((-13, -13, 9), (-FLAT_AT, 13, 15))[0],
    ]
    run.stage("flats", lambda: model.cut_many(body, boxes))
