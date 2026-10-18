"""A wall hook: a flat back plate with two screw holes and a J-shaped arm
fused to its face, the arm's tip rounded."""

import math

import arris

PLATE = (30.0, 5.0, 60.0)  # x, y (the wall's normal), z
ARM_X = (8.0, 22.0)
SCREWS = [(15.0, 40.0), (15.0, 54.0)]
SCREW_RADIUS = 2.5
TIP_RADIUS = 1.5


def build(run):
    model = run.model

    plate, _ = run.stage("plate", lambda: model.primitive_box((0, 0, 0), PLATE))

    # The arm's section, drawn in the yz plane: a bar that leaves the plate
    # and turns up through a half circle, an inner radius of 7 mm.
    section = arris.Profile(
        arris.Loop.path(
            (4, 8),
            [
                arris.Segment.line((30, 8)),
                arris.Segment.arc((30, 30), (41, 19)),
                arris.Segment.line((30, 26)),
                arris.Segment.arc((30, 12), (37, 19)),
                arris.Segment.line((4, 12)),
                arris.Segment.line((4, 8)),
            ],
        ),
        origin=(ARM_X[0], 0, 0),
        normal=(1, 0, 0),
        x_axis=(0, 1, 0),
    )
    arm, _ = run.stage("arm", lambda: model.extrude(section, (1, 0, 0), ARM_X[1] - ARM_X[0]))
    body, _ = run.stage("hook", lambda: model.fuse(plate, arm))

    screws = [
        model.primitive_cylinder((x, -1, z), (0, 1, 0), SCREW_RADIUS, PLATE[1] + 2)[0]
        for x, z in SCREWS
    ]
    body, _ = run.stage("screw_holes", lambda: model.cut_many(body, screws))

    # The arm's outer arc meets its two ends in an edge each: radius 11
    # about (y, z) = (30, 19), on the planes x = 8 and x = 22.
    tips = [
        edge
        for edge, points in run.edge_points(body).items()
        if any(all(abs(p[0] - x) < 1e-6 for p in points) for x in ARM_X)
        and all(abs(math.hypot(p[1] - 30, p[2] - 19) - 11) < 1e-3 for p in points)
    ]
    run.stage("tip_round", lambda: model.fillet(body, tips, TIP_RADIUS), finish=True)
