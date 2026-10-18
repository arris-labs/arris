"""A finned heat sink: a 60 x 60 x 6 mm base plate, five cooling fins fused to
it, and four corner mounting holes; base and fin faces tagged through every
operation and reaching the mesh."""

import harness

BASE_SIZE = 60.0
BASE_THICKNESS = 6.0
N_FINS = 5
FIN_WIDTH = 4.0
FIN_PITCH = 12.0
FIN_Y_MIN, FIN_Y_MAX = 5.0, 55.0
FIN_Z_MIN, FIN_Z_MAX = 4.0, 26.0
MOUNT_RADIUS = 2.0
MOUNT_HOLES = [(4.0, 4.0), (56.0, 4.0), (4.0, 56.0), (56.0, 56.0)]


def build(run):
    model = run.model

    body, record = run.stage("base", lambda: model.primitive_box((0, 0, 0), (BASE_SIZE, BASE_SIZE, BASE_THICKNESS)))
    part = harness.Labelled(model, body, record, "base")
    part.assert_tagged(["base"])

    # Five fins, slightly overlapping the base to fuse cleanly.
    fins = []
    fin_tools = []
    for i in range(N_FINS):
        x = 5.0 + i * FIN_PITCH
        f, r = model.primitive_box((x, FIN_Y_MIN, FIN_Z_MIN), (x + FIN_WIDTH, FIN_Y_MAX, FIN_Z_MAX))
        fins.append(f)
        fin_tools.append(harness.Labelled(model, f, r, "fins"))

    body, record = run.stage("finned", lambda: model.fuse_many([part.body, *fins]))
    part.step(body, record, "fins", fin_tools)
    part.assert_tagged(["base", "fins"])

    # Four corner mounting holes.
    mounts = []
    mount_tools = []
    for x, y in MOUNT_HOLES:
        h, r = model.primitive_cylinder((x, y, -1), (0, 0, 1), MOUNT_RADIUS, BASE_THICKNESS + 2)
        mounts.append(h)
        mount_tools.append(harness.Labelled(model, h, r, "mounts"))

    body, record = run.stage("mounted", lambda: model.cut_many(part.body, mounts))
    part.step(body, record, "mounts", mount_tools)
    part.assert_tagged(["base", "fins", "mounts"])

    run.labelled = part
