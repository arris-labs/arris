"""A control knob: a 28 mm revolved cap with an arc shoulder, a blind bore
for the shaft, eight grip flutes down its side and the top edge rounded."""

import math

import arris

SHOULDER = 11.0
BORE_RADIUS, BORE_DEPTH = 3.0, 8.0
FLUTES, FLUTE_CENTRE, FLUTE_RADIUS, FLUTE_HEIGHT = 8, 14.5, 1.5, 10.0
# The shoulder is a quarter-ish arc about (10, 11), radius 4, cut off at 60 degrees.
TOP = 11 + 2 * math.sqrt(3)
TOP_ROUND = 1.0


def build(run):
    model = run.model

    # The section in the xz plane, turned about z.
    section = arris.Profile(
        arris.Loop.path(
            (0, 0),
            [
                arris.Segment.line((14, 0)),
                arris.Segment.line((14, SHOULDER)),
                arris.Segment.arc((12, TOP), (10 + 2 * math.sqrt(3), 13)),
                arris.Segment.line((0, TOP)),
                arris.Segment.line((0, 0)),
            ],
        ),
        normal=(0, -1, 0),
        x_axis=(1, 0, 0),
    )
    body, _ = run.stage("body", lambda: model.revolve(section, (0, 0, 0), (0, 0, 1), 2 * math.pi))

    bore = model.primitive_cylinder((0, 0, -1), (0, 0, 1), BORE_RADIUS, BORE_DEPTH + 1)[0]
    body, _ = run.stage("bored", lambda: model.cut(body, bore))

    # Flutes start half a step round, off the revolve's seam.
    flutes = []
    for i in range(FLUTES):
        angle = math.radians((i + 0.5) * 360 / FLUTES)
        centre = (FLUTE_CENTRE * math.cos(angle), FLUTE_CENTRE * math.sin(angle), -1)
        flutes.append(model.primitive_cylinder(centre, (0, 0, 1), FLUTE_RADIUS, FLUTE_HEIGHT)[0])
    body, _ = run.stage("fluted", lambda: model.cut_many(body, flutes))

    # The top face's rim: the circle at the top, drawn on the top plane.
    rim = [
        edge
        for edge, points in run.edge_points(body).items()
        if all(abs(p[2] - TOP) < 1e-6 and abs(math.hypot(p[0], p[1]) - 12) < 1e-6 for p in points)
    ]
    run.stage("top_round", lambda: model.fillet(body, rim, TOP_ROUND), finish=True)
