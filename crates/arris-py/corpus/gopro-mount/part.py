"""A three-prong action-camera mount: a base plate, three 3 mm prongs
6 mm apart (one rounded-top extrusion translated twice), a pin hole across
them, two screw holes in the plate and its corners rounded."""

import arris

PLATE = ((-5.0, -3.0, 0.0), (20.0, 18.0, 3.0))
PRONGS, PRONG_THICKNESS, PRONG_PITCH = 3, 3.0, 6.0
PIN_RADIUS, PIN_HEIGHT = 2.5, 12.0
SCREWS = [-2.5, 17.5]
SCREW_RADIUS = 1.5
CORNER_RADIUS = 2.0


def build(run):
    model = run.model

    plate = model.primitive_box(*PLATE)[0]

    # A prong's outline in the yz plane: 15 mm wide, a half circle on top,
    # its foot buried a millimetre in the plate.
    outline = arris.Profile(
        arris.Loop.path(
            (0, 2),
            [
                arris.Segment.line((15, 2)),
                arris.Segment.line((15, PIN_HEIGHT)),
                arris.Segment.arc((0, PIN_HEIGHT), (7.5, 19.5)),
                arris.Segment.line((0, 2)),
            ],
        ),
        normal=(1, 0, 0),
        x_axis=(0, 1, 0),
    )
    prong = model.extrude(outline, (1, 0, 0), PRONG_THICKNESS)[0]
    prongs = [model.transform(prong, (i * PRONG_PITCH, 0, 0))[0] for i in range(PRONGS)]
    body, _ = run.stage("mount", lambda: model.fuse_many([plate, *prongs]))

    last = (PRONGS - 1) * PRONG_PITCH + PRONG_THICKNESS
    pin = model.primitive_cylinder((-1, 7.5, PIN_HEIGHT), (1, 0, 0), PIN_RADIUS, last + 2)[0]
    body, _ = run.stage("pin_hole", lambda: model.cut(body, pin))

    screws = [model.primitive_cylinder((x, 7.5, -1), (0, 0, 1), SCREW_RADIUS, 5)[0] for x in SCREWS]
    body, _ = run.stage("screw_holes", lambda: model.cut_many(body, screws))

    # The plate's four vertical edges.
    (x0, y0, _), (x1, y1, z1) = PLATE
    corners = [
        edge
        for edge, points in run.edge_points(body).items()
        if abs(points[-1][2] - points[0][2]) > 1
        and any(abs(points[0][0] - x) < 1e-6 and abs(points[0][1] - y) < 1e-6 for x in (x0, x1) for y in (y0, y1))
    ]
    run.stage("plate_round", lambda: model.fillet(body, corners, CORNER_RADIUS), finish=True)
