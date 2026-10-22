"""The cantilever on `arris.build`: the fixed and load faces are tags that
follow every cut, and the holes are a placement."""

import harness
from arris.build import Solid, Workplane

LENGTH, WIDTH, HEIGHT = 100.0, 20.0, 20.0
HOLE_RADIUS = 5.0
HOLE_X = [35.0, 70.0]
FILLET_RADIUS = 2.0


def build(run):
    model = run.model

    beam = run.stage("beam", lambda: Solid.box((0, 0, 0), (LENGTH, WIDTH, HEIGHT), model=model))
    fixed = beam.faces().normal((-1, 0, 0))
    load = beam.faces().normal((1, 0, 0))
    part = beam.tag("fixed", fixed).tag("load", load).tag("beam", beam.faces() - fixed - load)
    harness.assert_tagged_solid(part, ["fixed", "load", "beam"])

    # Two lightening holes through y, in the plane that looks along it.
    spots = Workplane.XZ().at([(x, HEIGHT / 2) for x in HOLE_X])
    part = run.stage("lightened", lambda: part.hole(spots, 2 * HOLE_RADIUS))
    part = part.tag("holes", part.faces().last() - part.faces().tagged("beam"))
    harness.assert_tagged_solid(part, ["fixed", "load", "beam", "holes"])

    # Rounded top and bottom edges of the load tip.
    tip = part.faces().tagged("load").edges().parallel((0, 1, 0))
    part = run.stage("filleted", lambda: part.fillet(tip, FILLET_RADIUS))
    part = harness.tag_rest(part, "fillet")
    harness.assert_tagged_solid(part, ["fixed", "load", "beam", "holes", "fillet"])

    run.tagged = part
