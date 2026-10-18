"""A hinge leaf: a 40 x 30 x 2 plate with a knuckle along one edge, the
pin's hole through the knuckle, three screw holes, and a fillet where the
knuckle meets the plate."""

import math

import arris

LEAF = ((0.0, 0.0, 0.0), (40.0, 30.0, 2.0))
KNUCKLE_RADIUS = 4.0
PIN_RADIUS = 1.6
SCREWS = [(14.0, 6.0), (14.0, 15.0), (14.0, 24.0)]
SCREW_RADIUS = 2.0
JOIN_RADIUS = 0.8


def build(run):
    model = run.model

    leaf, _ = run.stage("leaf", lambda: model.primitive_box(*LEAF))

    # The knuckle's axis runs along y through the leaf's mid-thickness, on its
    # edge; half of it stands in the leaf.
    knuckle = model.primitive_cylinder((0, 0, 1), (0, 1, 0), KNUCKLE_RADIUS, 30)[0]
    body, _ = run.stage("knuckle", lambda: model.fuse(leaf, knuckle))

    pin = model.primitive_cylinder((0, -1, 1), (0, 1, 0), PIN_RADIUS, 32)[0]
    body, _ = run.stage("pin_hole", lambda: model.cut(body, pin))

    screws = [
        model.primitive_cylinder((x, y, -1), (0, 0, 1), SCREW_RADIUS, 4)[0] for x, y in SCREWS
    ]
    body, _ = run.stage("screw_holes", lambda: model.cut_many(body, screws))

    # The two long edges where the knuckle's cylinder meets the leaf's faces
    # (z = 0 and z = 2), at x = sqrt(r^2 - 1).
    joins = [
        edge
        for edge, points in run.edge_points(body).items()
        if all(
            abs(math.hypot(p[0], p[2] - 1) - KNUCKLE_RADIUS) < 1e-3
            and p[0] > 1
            and min(abs(p[2]), abs(p[2] - 2)) < 1e-6
            for p in points
        )
        and abs(points[-1][1] - points[0][1]) > 1
    ]
    run.stage("knuckle_joins", lambda: model.fillet(body, joins, JOIN_RADIUS), finish=True)
