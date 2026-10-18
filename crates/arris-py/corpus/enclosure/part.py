"""A project box: a 60 x 40 x 20 block with rounded corners, hollowed to a
2 mm wall and open at the top, four screw posts on its floor, a blind hole
down each post and a fillet at each post's root."""

import math

LENGTH, WIDTH, HEIGHT = 60.0, 40.0, 20.0
CORNER_RADIUS = 3.0
WALL = 2.0
POSTS = [(8.0, 8.0), (52.0, 8.0), (8.0, 32.0), (52.0, 32.0)]
POST_RADIUS, POST_HEIGHT = 3.5, 15.0
HOLE_RADIUS, HOLE_FLOOR = 1.5, 6.0
ROOT_RADIUS = 0.8


def build(run):
    model = run.model

    body, _ = run.stage("body", lambda: model.primitive_box((0, 0, 0), (LENGTH, WIDTH, HEIGHT)))

    # The four vertical edges: the ones that rise.
    corners = [
        edge
        for edge, points in run.edge_points(body).items()
        if abs(points[-1][2] - points[0][2]) > 1
    ]
    body, _ = run.stage("rounded", lambda: model.fillet(body, corners, CORNER_RADIUS))

    # The opening is the face looking up at the top of the block.
    [lid] = [
        face
        for face in model.faces(body)
        if (plane := run.plane(face)) and plane[1][2] > 0.5 and abs(plane[0][2] - HEIGHT) < 1e-6
    ]
    body, _ = run.stage("shelled", lambda: model.shell(body, [lid], WALL, "inward"))

    # Posts start inside the floor so they fuse to it and not touch it.
    posts = [
        model.primitive_cylinder((x, y, WALL / 2), (0, 0, 1), POST_RADIUS, POST_HEIGHT)[0]
        for x, y in POSTS
    ]
    body, _ = run.stage("posts", lambda: model.fuse_many([body, *posts]))

    holes = [
        model.primitive_cylinder((x, y, HOLE_FLOOR), (0, 0, 1), HOLE_RADIUS, POST_HEIGHT)[0]
        for x, y in POSTS
    ]
    body, _ = run.stage("drilled", lambda: model.cut_many(body, holes))

    # A post's root is the circle it stands on the floor in.
    roots = [
        edge
        for edge, points in run.edge_points(body).items()
        if any(
            all(
                abs(math.hypot(p[0] - x, p[1] - y) - POST_RADIUS) < 1e-3 and abs(p[2] - WALL) < 1e-3
                for p in points
            )
            for x, y in POSTS
        )
    ]
    run.stage("post_roots", lambda: model.fillet(body, roots, ROOT_RADIUS), finish=True)
