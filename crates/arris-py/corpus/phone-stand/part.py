"""A phone stand: a base, a lip at its front and a leaning back, drawn as one
side section and extruded 70 mm; a cable hole through the lip, and the
lip's and the back's top edges rounded."""

import arris

WIDTH = 70.0
SECTION = [(0, 0), (90, 0), (90, 14), (78, 14), (78, 6), (60, 6), (28, 80), (16, 80), (46, 6), (0, 6)]
PORT = ((76.0, 35.0, 10.0), 3.5)
LIP_ROUND, BACK_ROUND = 2.0, 3.0


def build(run):
    model = run.model

    # The section is drawn in the xz plane and extruded along y.
    section = arris.Profile(
        arris.Loop.polygon(SECTION), origin=(0, 0, 0), normal=(0, -1, 0), x_axis=(1, 0, 0)
    )
    body, _ = run.stage("stand", lambda: model.extrude(section, (0, 1, 0), WIDTH))

    (x, y, z), radius = PORT
    port = model.primitive_cylinder((x, y, z), (1, 0, 0), radius, 16)[0]
    body, _ = run.stage("port", lambda: model.cut(body, port))

    # Top edges: those running along y at the lip's top and the back's top.
    def along_y_at(points, x_values, z_value):
        return (
            abs(points[-1][1] - points[0][1]) > 1
            and any(all(abs(p[0] - x) < 1e-6 for p in points) for x in x_values)
            and all(abs(p[2] - z_value) < 1e-6 for p in points)
        )

    lip = [e for e, points in run.edge_points(body).items() if along_y_at(points, (90, 78), 14)]
    if made := run.stage("lip_round", lambda: model.fillet(body, lip, LIP_ROUND), finish=True):
        body = made[0]

    back = [e for e, points in run.edge_points(body).items() if along_y_at(points, (16, 28), 80)]
    run.stage("back_round", lambda: model.fillet(body, back, BACK_ROUND), finish=True)
