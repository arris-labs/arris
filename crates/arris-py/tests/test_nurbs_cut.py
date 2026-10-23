"""A cut through NURBS faces, read from a STEP file: the plate of
`boolean/nurbs-plate-drilled-cut` (every face a rational Bézier patch, the
exact twin of a box) drilled by an analytic cylinder, held to Open
CASCADE's volume and area (ADR-0061).
"""

import arris
import pytest

from conftest import CORPUS, oracle, tolerances

FIXTURE = "boolean/nurbs-plate-drilled-cut"


def test_a_cylinder_cuts_a_plate_read_from_a_step_of_b_spline_faces():
    model = arris.Model()
    text = (CORPUS / FIXTURE / "box.step").read_text()
    [solid] = model.read_step(text).solids
    plate = solid.body
    tool, _ = model.primitive_cylinder((20, 15, -1), (0, 0, 1), 4, 12)
    drilled, record = model.cut(plate, tool)
    expected, tol = oracle(FIXTURE), tolerances(FIXTURE)
    measured = model.mass_properties(drilled)
    assert measured.volume == pytest.approx(expected["volume"], rel=tol["volume_rel"])
    assert measured.area == pytest.approx(expected["area"], rel=tol["area_rel"])
    assert record.generated
