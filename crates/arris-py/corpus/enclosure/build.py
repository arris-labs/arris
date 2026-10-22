"""The enclosure on `arris.build`: the same stages as `part.py`, the edges
and faces chosen by selection and the posts and holes by placement."""

from arris.build import Solid, Workplane

LENGTH, WIDTH, HEIGHT = 60.0, 40.0, 20.0
CORNER_RADIUS = 3.0
WALL = 2.0
POSTS = [(8.0, 8.0), (52.0, 8.0), (8.0, 32.0), (52.0, 32.0)]
POST_RADIUS, POST_HEIGHT = 3.5, 15.0
HOLE_RADIUS, HOLE_FLOOR = 1.5, 6.0
ROOT_RADIUS = 0.8


def build(run):
    model = run.model

    body = run.stage("body", lambda: Solid.box((0, 0, 0), (LENGTH, WIDTH, HEIGHT), model=model))
    body = run.stage("rounded", lambda: body.fillet(body.edges().parallel((0, 0, 1)), CORNER_RADIUS))
    lid = body.faces().normal((0, 0, 1)).extreme((0, 0, 1))
    body = run.stage("shelled", lambda: body.shell(lid, WALL, "inward"))

    # Posts start inside the floor so they fuse to it and not touch it.
    floor = Workplane.XY(WALL / 2).at(POSTS)
    body = run.stage("posts", lambda: body.boss([plane.circle(POST_RADIUS) for plane in floor], POST_HEIGHT))

    # Blind holes down from the posts' tops.
    tops = Workplane.XY(WALL / 2 + POST_HEIGHT).at(POSTS)
    depth = WALL / 2 + POST_HEIGHT - HOLE_FLOOR
    body = run.stage("drilled", lambda: body.hole(tops, 2 * HOLE_RADIUS, depth))

    # A post's root is the circle it stands on the floor in.
    circles = body.faces().normal((0, 0, 1)).extreme((0, 0, -1)).edges().kind("circle")
    roots = circles - circles
    for x, y in POSTS:
        around = ((x - POST_RADIUS, y - POST_RADIUS, 0), (x + POST_RADIUS, y + POST_RADIUS, WALL + 1))
        roots |= circles.within(around)
    run.stage("post_roots", lambda: body.fillet(roots, ROOT_RADIUS), finish=True)
