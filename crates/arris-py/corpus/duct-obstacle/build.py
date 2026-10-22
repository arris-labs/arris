"""The duct on `arris.build`: the inlet, outlet and walls are tags that
follow a through-cut and a blind port into the mesh."""

import harness
from arris.build import Solid, Workplane

LENGTH = 100.0
HALF_WIDTH = 20.0
HALF_HEIGHT = 20.0
OBSTACLE_X = 30.0
OBSTACLE_RADIUS = 8.0
TAP_X = 60.0
TAP_RADIUS = 2.0
TAP_DEPTH = 5.0


def build(run):
    model = run.model

    duct = run.stage(
        "duct",
        lambda: Solid.box((0, -HALF_WIDTH, -HALF_HEIGHT), (LENGTH, HALF_WIDTH, HALF_HEIGHT), model=model),
    )
    inlet = duct.faces().normal((-1, 0, 0))
    outlet = duct.faces().normal((1, 0, 0))
    part = duct.tag("inlet", inlet).tag("outlet", outlet).tag("walls", duct.faces() - inlet - outlet)
    harness.assert_tagged_solid(part, ["inlet", "outlet", "walls"])

    # The obstacle: a cylinder across the flow, through the whole duct.
    part = run.stage("cavity", lambda: part.hole(Workplane.XY().at([(OBSTACLE_X, 0)]), 2 * OBSTACLE_RADIUS))
    part = harness.tag_rest(part, "obstacle")
    harness.assert_tagged_solid(part, ["inlet", "outlet", "walls", "obstacle"])

    # The pressure tap: a blind port in from the side wall at y = -HALF_WIDTH.
    wall = Workplane.XZ(HALF_WIDTH)
    part = run.stage("tapped", lambda: part.hole(wall.at([(TAP_X, 0)]), 2 * TAP_RADIUS, TAP_DEPTH))
    part = harness.tag_rest(part, "tap")
    harness.assert_tagged_solid(part, ["inlet", "outlet", "walls", "obstacle", "tap"])

    run.tagged = part
