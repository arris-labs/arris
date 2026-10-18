"""A flow duct with an obstacle: a rectangular channel with inlet (x=0),
outlet (x=100) and wall faces tagged, an obstacle cylinder cut across the flow,
and a wall pressure tap port; tags asserted through every cut and into the mesh."""

import harness

LENGTH = 100.0
HALF_WIDTH = 20.0
HALF_HEIGHT = 20.0
OBSTACLE_X = 30.0
OBSTACLE_RADIUS = 8.0
TAP_X = 60.0
TAP_RADIUS = 2.0


def build(run):
    model = run.model

    body, record = run.stage(
        "duct",
        lambda: model.primitive_box((0, -HALF_WIDTH, -HALF_HEIGHT), (LENGTH, HALF_WIDTH, HALF_HEIGHT)),
    )
    part = harness.Labelled(model, body, record)
    for face in model.faces(body):
        frame = model.face_frame(face)
        if frame.origin[0] < 1e-6 and frame.z[0] < -0.5:
            part.tag(face, "inlet")
        elif abs(frame.origin[0] - LENGTH) < 1e-6 and frame.z[0] > 0.5:
            part.tag(face, "outlet")
        else:
            part.tag(face, "walls")
    part.assert_tagged(["inlet", "outlet", "walls"])

    # Obstacle across the duct.
    obs, obs_rec = model.primitive_cylinder((OBSTACLE_X, 0, -HALF_HEIGHT - 5), (0, 0, 1), OBSTACLE_RADIUS, 2 * HALF_HEIGHT + 10)
    body, record = run.stage("cavity", lambda: model.cut(part.body, obs))
    part.step(body, record, "obstacle", [harness.Labelled(model, obs, obs_rec, "obstacle")])
    part.assert_tagged(["inlet", "outlet", "walls", "obstacle"])

    # Side wall pressure tap port.
    tap, tap_rec = model.primitive_cylinder((TAP_X, -HALF_WIDTH - 5, 0), (0, 1, 0), TAP_RADIUS, 10)
    body, record = run.stage("tapped", lambda: model.cut(part.body, tap))
    part.step(body, record, "tap", [harness.Labelled(model, tap, tap_rec, "tap")])
    part.assert_tagged(["inlet", "outlet", "walls", "obstacle", "tap"])

    run.labelled = part
