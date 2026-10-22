"""The spur gear blank on `arris.build`: the teeth and the lightening
holes are polar patterns, the keyway a through pocket."""

import math

from arris.build import Solid, Workplane

DISC_RADIUS, DISC_THICKNESS = 30.0, 8.0
HUB_RADIUS, HUB_HEIGHT = 10.0, 14.0
TEETH, TOOTH_WIDTH = 24, 3.0
BORE_RADIUS = 5.0
LIGHTENING, LIGHTENING_CIRCLE, LIGHTENING_RADIUS = 6, 19.0, 4.0
KEY_WIDTH, KEY_DEPTH = 2.0, 1.5
CHAMFER = 0.8
TOOTH_ROOT = 1.0  # a tooth is rooted this far inside the rim
TOOTH_HEIGHT = 4.0  # and stands this far out of it


def build(run):
    model = run.model
    plane = Workplane.XY()

    disc = Solid.cylinder((0, 0, 0), (0, 0, 1), DISC_RADIUS, DISC_THICKNESS, model=model)
    hub = Solid.cylinder((0, 0, 0), (0, 0, 1), HUB_RADIUS, HUB_HEIGHT, model=model)
    blank = run.stage("blank", lambda: disc.fuse(hub))

    # One tooth drawn on a plane whose x points out along its radius,
    # half a step off the cylinders' seams.
    step = 2 * math.pi / TEETH
    places = plane.polar(TEETH, DISC_RADIUS, start=step / 2)
    tooth = (TOOTH_ROOT + TOOTH_HEIGHT, TOOTH_WIDTH)
    body = run.stage(
        "toothed",
        lambda: blank.boss(
            [p.rect(*tooth, center=(TOOTH_HEIGHT / 2 - TOOTH_ROOT / 2, 0)) for p in places], DISC_THICKNESS
        ),
    )

    holes = plane.polar(LIGHTENING, LIGHTENING_CIRCLE)
    cuts = [plane.circle(BORE_RADIUS), *(p.circle(LIGHTENING_RADIUS) for p in holes)]
    body = run.stage("bored", lambda: body.pocket(cuts))

    key = plane.rect(KEY_WIDTH, KEY_DEPTH + 1, center=(0, BORE_RADIUS + (KEY_DEPTH - 1) / 2))
    body = run.stage("keyway", lambda: body.pocket(key))

    # The hub's rim: the outer circle of its top face, which the bore's
    # circles (and the keyway's arcs) lie inside.
    top = body.faces().normal((0, 0, 1)).extreme((0, 0, 1))
    inside = ((-HUB_RADIUS + 1, -HUB_RADIUS + 1, 0), (HUB_RADIUS - 1, HUB_RADIUS - 1, HUB_HEIGHT + 1))
    rim = top.edges() - top.edges().within(inside)
    run.stage("hub_chamfer", lambda: body.chamfer(rim, CHAMFER), finish=True)
