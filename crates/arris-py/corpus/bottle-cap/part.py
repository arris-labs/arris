"""A bottle cap: a 32 mm cup with a 2.5-turn internal thread of pitch 3 mm,
a triangular tooth swept along a helix and fused to the wall, the top rim
rounded."""

import math

import arris

WALL_BORE, OUTER, HEIGHT, CEILING = 13.5, 16.0, 14.0, 12.0
PITCH, TURNS, START = 3.0, 2.5, 2.0
RIM_ROUND = 1.0


def build(run):
    model = run.model

    cup = arris.Profile(
        arris.Loop.polygon(
            [(WALL_BORE, 0), (OUTER, 0), (OUTER, HEIGHT), (0, HEIGHT), (0, CEILING), (WALL_BORE, CEILING)]
        ),
        normal=(0, -1, 0),
        x_axis=(1, 0, 0),
    )
    cap, _ = run.stage("body", lambda: model.revolve(cup, (0, 0, 0), (0, 0, 1), 2 * math.pi))

    # The tooth, in the axial plane where the helix begins: its base buried
    # half a millimetre in the wall, its crest pointing at the axis.
    tooth = arris.Profile(
        arris.Loop.polygon([(0.5, -1.2), (-1.2, 0), (0.5, 1.2)]),
        origin=(WALL_BORE, 0, START),
        normal=(0, -1, 0),
        x_axis=(1, 0, 0),
    )
    coil = arris.Path(
        (WALL_BORE, 0, START),
        [arris.PathSegment.helix((0, 0, START), (0, 0, 1), PITCH, TURNS * 2 * math.pi)],
    )
    thread, _ = run.stage(
        "thread", lambda: model.sweep(tooth, coil, arris.FrameLaw.fixed_binormal((0, 0, 1)))
    )
    body, _ = run.stage("threaded", lambda: model.fuse(cap, thread))

    rim = [
        edge
        for edge, points in run.edge_points(body).items()
        if all(abs(p[2] - HEIGHT) < 1e-6 and abs(math.hypot(p[0], p[1]) - OUTER) < 1e-6 for p in points)
    ]
    run.stage("top_round", lambda: model.fillet(body, rim, RIM_ROUND), finish=True)
