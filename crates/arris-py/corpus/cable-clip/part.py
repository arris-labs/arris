"""A cable clip: a 10 x 1.6 mm strap swept along a 20 mm straight, a half
circle of 8 mm radius and a straight back, a screw hole through both legs
and the free ends rounded."""

import arris

WIDTH, THICKNESS = 10.0, 1.6
LEG, GAP = 20.0, 16.0
SCREW = (4.0, 0.0)
SCREW_RADIUS = 1.6
END_ROUND = 0.5


def build(run):
    model = run.model

    # The strap's section is normal to the first leg: width along y,
    # thickness along z.
    half_w, half_t = WIDTH / 2, THICKNESS / 2
    section = arris.Profile(
        arris.Loop.polygon([(-half_w, -half_t), (half_w, -half_t), (half_w, half_t), (-half_w, half_t)]),
        normal=(1, 0, 0),
        x_axis=(0, 1, 0),
    )
    path = arris.Path(
        (0, 0, 0),
        [
            arris.PathSegment.line((LEG, 0, 0)),
            arris.PathSegment.arc((LEG, 0, GAP), (LEG + GAP / 2, 0, GAP / 2)),
            arris.PathSegment.line((0, 0, GAP)),
        ],
    )
    strap, _ = run.stage("strap", lambda: model.sweep(section, path, arris.FrameLaw.rotation_minimizing()))

    screw = model.primitive_cylinder((*SCREW, -5), (0, 0, 1), SCREW_RADIUS, GAP + 10)[0]
    body, _ = run.stage("screw_hole", lambda: model.cut(strap, screw))

    # The free ends: the four edges along y on the end plane x = 0.
    ends = [
        edge
        for edge, points in run.edge_points(body).items()
        if all(abs(p[0]) < 1e-6 for p in points) and abs(points[-1][1] - points[0][1]) > 1
    ]
    run.stage("ends_round", lambda: model.fillet(body, ends, END_ROUND), finish=True)
