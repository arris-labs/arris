"""A spool holder: a 50 mm flange screwed to a wall, a hub with a retaining
lip at its end, a blind bore from the wall side, four screw holes in the
flange and a fillet where the hub meets it."""

import math

import arris

FLANGE_RADIUS, FLANGE_THICKNESS = 25.0, 4.0
HUB_RADIUS, HUB_TOP = 8.0, 50.0
BORE_RADIUS, BORE_TOP = 3.0, 30.0
SCREW_CIRCLE, SCREW_RADIUS = 18.0, 2.5
ROOT_RADIUS = 2.0


def build(run):
    model = run.model

    section = arris.Profile(
        arris.Loop.polygon(
            [
                (0, 0),
                (FLANGE_RADIUS, 0),
                (FLANGE_RADIUS, FLANGE_THICKNESS),
                (HUB_RADIUS, FLANGE_THICKNESS),
                (HUB_RADIUS, HUB_TOP),
                (11, 53),
                (11, 56),
                (0, 56),
            ]
        ),
        normal=(0, -1, 0),
        x_axis=(1, 0, 0),
    )
    body, _ = run.stage("body", lambda: model.revolve(section, (0, 0, 0), (0, 0, 1), 2 * math.pi))

    bore = model.primitive_cylinder((0, 0, -1), (0, 0, 1), BORE_RADIUS, BORE_TOP + 1)[0]
    body, _ = run.stage("bored", lambda: model.cut(body, bore))

    screws = []
    for i in range(4):
        angle = math.radians(45 + 90 * i)
        centre = (SCREW_CIRCLE * math.cos(angle), SCREW_CIRCLE * math.sin(angle), -1)
        screws.append(model.primitive_cylinder(centre, (0, 0, 1), SCREW_RADIUS, FLANGE_THICKNESS + 2)[0])
    body, _ = run.stage("screw_holes", lambda: model.cut_many(body, screws))

    # The hub's root: the circle of the hub's radius on the flange's face.
    root = [
        edge
        for edge, points in run.edge_points(body).items()
        if all(
            abs(p[2] - FLANGE_THICKNESS) < 1e-6 and abs(math.hypot(p[0], p[1]) - HUB_RADIUS) < 1e-6
            for p in points
        )
    ]
    run.stage("hub_root", lambda: model.fillet(body, root, ROOT_RADIUS), finish=True)
