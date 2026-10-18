"""A cantilever beam: a 100 x 20 x 20 bar with its fixed face (x=0) and load
face (x=100) tagged, two transverse weight-relief holes cut through its web, and
filleted load-end edges; tags followed through every cut and verified in the
mesh."""

import harness

LENGTH, WIDTH, HEIGHT = 100.0, 20.0, 20.0
HOLE_RADIUS = 5.0
HOLE_X = [35.0, 70.0]
FILLET_RADIUS = 2.0


def build(run):
    model = run.model

    body, record = run.stage("beam", lambda: model.primitive_box((0, 0, 0), (LENGTH, WIDTH, HEIGHT)))
    part = harness.Labelled(model, body, record)
    for face in model.faces(body):
        frame = model.face_frame(face)
        if frame.origin[0] < 1e-6 and frame.z[0] < -0.5:
            part.tag(face, "fixed")
        elif abs(frame.origin[0] - LENGTH) < 1e-6 and frame.z[0] > 0.5:
            part.tag(face, "load")
        else:
            part.tag(face, "beam")
    part.assert_tagged(["fixed", "load", "beam"])

    # Two lightening holes through y.
    holes = []
    hole_tools = []
    for x in HOLE_X:
        h, r = model.primitive_cylinder((x, -5, HEIGHT / 2), (0, 1, 0), HOLE_RADIUS, WIDTH + 10)
        holes.append(h)
        hole_tools.append(harness.Labelled(model, h, r, "holes"))

    body, record = run.stage("lightened", lambda: model.cut_many(part.body, holes))
    part.step(body, record, "holes", hole_tools)
    part.assert_tagged(["fixed", "load", "beam", "holes"])

    # Rounded top and bottom edges of the load tip.
    edges = [
        e
        for e, pts in run.edge_points(body).items()
        if all(abs(p[0] - LENGTH) < 1e-4 for p in pts)
        and (all(abs(p[2] - HEIGHT) < 1e-4 for p in pts) or all(abs(p[2]) < 1e-4 for p in pts))
    ]
    body, record = run.stage("filleted", lambda: model.fillet(body, edges, FILLET_RADIUS))
    part.step(body, record, "fillet")
    part.assert_tagged(["fixed", "load", "beam", "holes", "fillet"])

    run.labelled = part
