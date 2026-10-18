"""A spur gear blank: a 60 mm disc and a 20 mm hub, 24 teeth (boxes turned
about the axis), a 10 mm bore with six lightening holes and a keyway, the
hub's rim chamfered."""

import math

import arris

DISC_RADIUS, DISC_THICKNESS = 30.0, 8.0
HUB_RADIUS, HUB_HEIGHT = 10.0, 14.0
TEETH, TOOTH_WIDTH = 24, 3.0
BORE_RADIUS = 5.0
LIGHTENING, LIGHTENING_CIRCLE, LIGHTENING_RADIUS = 6, 19.0, 4.0
KEY_WIDTH, KEY_DEPTH = 2.0, 1.5
CHAMFER = 0.8


def build(run):
    model = run.model

    disc = model.primitive_cylinder((0, 0, 0), (0, 0, 1), DISC_RADIUS, DISC_THICKNESS)[0]
    hub = model.primitive_cylinder((0, 0, 0), (0, 0, 1), HUB_RADIUS, HUB_HEIGHT)[0]
    blank, _ = run.stage("blank", lambda: model.fuse(disc, hub))

    # One tooth on the x axis, rooted a millimetre inside the rim, then
    # turned to each of the 24 places, half a step off the cylinders' seams.
    tooth = model.primitive_box(
        (DISC_RADIUS - 1, -TOOTH_WIDTH / 2, 0), (DISC_RADIUS + 4, TOOTH_WIDTH / 2, DISC_THICKNESS)
    )[0]
    step = 2 * math.pi / TEETH
    teeth = [model.transform(tooth, axis=(0, 0, 1), angle=(i + 0.5) * step)[0] for i in range(TEETH)]
    body, _ = run.stage("toothed", lambda: model.fuse_many([blank, *teeth]))

    bore = model.primitive_cylinder((0, 0, -1), (0, 0, 1), BORE_RADIUS, HUB_HEIGHT + 2)[0]
    holes = [
        model.primitive_cylinder(
            (
                LIGHTENING_CIRCLE * math.cos(2 * math.pi * i / LIGHTENING),
                LIGHTENING_CIRCLE * math.sin(2 * math.pi * i / LIGHTENING),
                -1,
            ),
            (0, 0, 1),
            LIGHTENING_RADIUS,
            DISC_THICKNESS + 2,
        )[0]
        for i in range(LIGHTENING)
    ]
    body, _ = run.stage("bored", lambda: model.cut_many(body, [bore, *holes]))

    key = model.primitive_box(
        (-KEY_WIDTH / 2, BORE_RADIUS - 1, -1), (KEY_WIDTH / 2, BORE_RADIUS + KEY_DEPTH, HUB_HEIGHT + 1)
    )[0]
    body, _ = run.stage("keyway", lambda: model.cut(body, key))

    # The hub's rim: the circle of the hub's radius on its top face.
    rim = [
        edge
        for edge, points in run.edge_points(body).items()
        if all(
            abs(p[2] - HUB_HEIGHT) < 1e-6 and abs(math.hypot(p[0], p[1]) - HUB_RADIUS) < 1e-6
            for p in points
        )
    ]
    run.stage("hub_chamfer", lambda: model.chamfer(body, rim, CHAMFER), finish=True)
