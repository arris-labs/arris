"""A mounting plate for a single-board computer: a 85 x 56 x 3 plate with
rounded corners, four standoffs on the board's hole pattern with a hole
through each, a stadium cable slot, and a fillet at each standoff's root."""

import math

import arris

SIZE = (85.0, 56.0, 3.0)
CORNER_RADIUS = 3.0
HOLES = [(14.0, 3.5), (72.0, 3.5), (14.0, 52.5), (72.0, 52.5)]
STANDOFF_RADIUS, STANDOFF_HEIGHT = 3.0, 5.0
HOLE_RADIUS = 1.375
ROOT_RADIUS = 0.6


def build(run):
    model = run.model

    plate, _ = run.stage("plate", lambda: model.primitive_box((0, 0, 0), SIZE))
    corners = [
        edge
        for edge, points in run.edge_points(plate).items()
        if abs(points[-1][2] - points[0][2]) > 1
    ]
    plate, _ = run.stage("rounded", lambda: model.fillet(plate, corners, CORNER_RADIUS))

    # Standoffs start inside the plate, so they fuse to it.
    standoffs = [
        model.primitive_cylinder((x, y, SIZE[2] / 2), (0, 0, 1), STANDOFF_RADIUS, STANDOFF_HEIGHT)[0]
        for x, y in HOLES
    ]
    body, _ = run.stage("standoffs", lambda: model.fuse_many([plate, *standoffs]))

    holes = [
        model.primitive_cylinder((x, y, -1), (0, 0, 1), HOLE_RADIUS, SIZE[2] + STANDOFF_HEIGHT + 2)[0]
        for x, y in HOLES
    ]
    body, _ = run.stage("holes", lambda: model.cut_many(body, holes))

    # The cable slot: a stadium 30 long and 8 wide about the plate's middle,
    # extruded through it.
    stadium = arris.Profile(
        arris.Loop.path(
            (31.5, 24),
            [
                arris.Segment.line((53.5, 24)),
                arris.Segment.arc((53.5, 32), (57.5, 28)),
                arris.Segment.line((31.5, 32)),
                arris.Segment.arc((31.5, 24), (27.5, 28)),
            ],
        ),
        origin=(0, 0, -1),
        normal=(0, 0, 1),
        x_axis=(1, 0, 0),
    )
    slot, _ = model.extrude(stadium, (0, 0, 1), SIZE[2] + 2)
    body, _ = run.stage("slot", lambda: model.cut(body, slot))

    # A standoff's root: the circle it stands on the plate's top face in.
    roots = [
        edge
        for edge, points in run.edge_points(body).items()
        if any(
            all(
                abs(math.hypot(p[0] - x, p[1] - y) - STANDOFF_RADIUS) < 1e-3 and abs(p[2] - SIZE[2]) < 1e-3
                for p in points
            )
            for x, y in HOLES
        )
    ]
    run.stage("standoff_roots", lambda: model.fillet(body, roots, ROOT_RADIUS), finish=True)
