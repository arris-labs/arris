//! `Model`'s features: fillet, chamfer, offset faces and shell.

use super::*;

#[pymethods]
impl Model {
    /// `body` with the given `edges` rounded to `radius`.
    ///
    /// An edge on a free-form face, or between analytic faces that no closed
    /// form holds, rolls the ball along a path found from the faces' offsets;
    /// two free-form faces are refused.
    ///
    /// Raises `OpDegenerateError`, `OpUnsupportedError` or `OpToleranceError`
    /// for an edge set the blend network cannot build, and
    /// `ForeignHandleError` for an edge of another model.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// cube, record = model.primitive_box((0, 0, 0), (2, 2, 2))
    /// edge = record.generated_from(arris.Role("box", "Edge", "Z", "Max", "Max"))
    /// rounded, _ = model.fillet(cube, [e for e in edge if isinstance(e, arris.Edge)], 0.2)
    /// removed = (1 - math.pi / 4) * 0.2**2 * 2
    /// assert abs(model.mass_properties(rounded).volume - (8 - removed)) < 1e-9
    /// ```
    #[pyo3(signature = (body, edges, radius, *, cancel=None, budget=None))]
    fn fillet(
        &self,
        py: Python<'_>,
        body: &Body,
        edges: Vec<PyRef<'_, Edge>>,
        radius: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let body = body.resolve(&self.shared)?;
        let edges = self.edges_list(&edges)?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::fillet(m, body, &edges, radius, control)
        })
    }

    /// `body` with the given `edges` cut back by `distance` on each side,
    /// measured in the edge's normal plane. Takes the edges `fillet` takes;
    /// errors as for `fillet`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// cube, record = model.primitive_box((0, 0, 0), (2, 2, 2))
    /// edge = record.generated_from(arris.Role("box", "Edge", "Z", "Max", "Max"))
    /// cut, _ = model.chamfer(cube, [e for e in edge if isinstance(e, arris.Edge)], 0.2)
    /// assert abs(model.mass_properties(cut).volume - (8 - 0.5 * 0.2**2 * 2)) < 1e-9
    /// ```
    #[pyo3(signature = (body, edges, distance, *, cancel=None, budget=None))]
    fn chamfer(
        &self,
        py: Python<'_>,
        body: &Body,
        edges: Vec<PyRef<'_, Edge>>,
        distance: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let body = body.resolve(&self.shared)?;
        let edges = self.edges_list(&edges)?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::chamfer(m, body, &edges, distance, control)
        })
    }

    /// `body` with the given `faces` moved along their outward normals by
    /// `distance`: positive adds material, negative removes it. Each moved
    /// face stays on the offset of its own surface, the faces beside it are
    /// extended or trimmed to meet it, and a face tangent to a moved one
    /// moves with it.
    ///
    /// Raises `OpDegenerateError` with the reason's name in `reason` for a
    /// move the kernel refuses (`NoFaces`, `RepeatedFace`, `FaceNotInBody`,
    /// `Vanishes`, `VertexSplits`, `NoExactOffset`, `SurfaceCollapses`,
    /// `Gap` or `SelfIntersects`) or a zero or non-finite `distance`,
    /// `OpUnsupportedError` for a neighbouring face it cannot place, and
    /// `ForeignHandleError` for a face of another model. The model is left
    /// as it was on every refusal.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// cube, record = model.primitive_box((0, 0, 0), (2, 2, 2))
    /// top = record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    /// taller, _ = model.offset_faces(cube, [f for f in top if isinstance(f, arris.Face)], 0.5)
    /// assert abs(model.mass_properties(taller).volume - 10) < 1e-9
    /// ```
    #[pyo3(signature = (body, faces, distance, *, cancel=None, budget=None))]
    fn offset_faces(
        &self,
        py: Python<'_>,
        body: &Body,
        faces: Vec<PyRef<'_, Face>>,
        distance: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let body = body.resolve(&self.shared)?;
        let faces = faces
            .iter()
            .map(|f| f.resolve(&self.shared))
            .collect::<Result<Vec<topo::Face>, BindError>>()?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::offset_faces(m, body, &faces, distance, control)
        })
    }

    /// `body` hollowed to a wall of `thickness`, open at `openings`. With
    /// `side="inward"` the body's faces stay the outside and the cavity
    /// grows inside them; with `side="outward"` the body's faces become the
    /// cavity and a skin grows outside them. The inner skin is every face
    /// but the openings moved by `thickness`, each on the offset of its own
    /// surface; each opening becomes a rim face between the two skins. No
    /// openings leaves a closed void: a body of two shells. Each skin face
    /// is generated from the face it copies.
    ///
    /// Raises `ValueError` for a `side` other than `"inward"` or
    /// `"outward"`; `OpDegenerateError` with the reason's name in `reason`
    /// for a shell the kernel refuses — the openings themselves
    /// (`RepeatedOpening`, `OpeningNotInBody`, `NoWalls`), an opening
    /// tangent to a wall (`OpeningDragged`), the walls' offset (`Vanishes`,
    /// `VertexSplits`, `NoExactOffset`, `SurfaceCollapses`, `Gap`, or
    /// `SelfIntersects` for a thickness past the thinnest wall) — or a
    /// non-positive or non-finite `thickness`; `OpUnsupportedError` for
    /// openings meeting in a way it cannot splice; and `ForeignHandleError`
    /// for a face of another model. The model is left as it was on every
    /// refusal.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// cube, record = model.primitive_box((0, 0, 0), (10, 10, 10))
    /// top = record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    /// cup, _ = model.shell(cube, [f for f in top if isinstance(f, arris.Face)], 1.0)
    /// assert abs(model.mass_properties(cup).volume - (1000 - 8 * 8 * 9)) < 1e-9
    /// assert len(model.faces(cup)) == 11
    /// ```
    #[pyo3(signature = (body, openings, thickness, side="inward", *, cancel=None, budget=None))]
    #[allow(clippy::too_many_arguments)] // the keywords are the Python signature
    fn shell(
        &self,
        py: Python<'_>,
        body: &Body,
        openings: Vec<PyRef<'_, Face>>,
        thickness: f64,
        side: &str,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let side = shell_side_of(side)?;
        let body = body.resolve(&self.shared)?;
        let openings = openings
            .iter()
            .map(|f| f.resolve(&self.shared))
            .collect::<Result<Vec<topo::Face>, BindError>>()?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::shell(m, body, &openings, thickness, side, control)
        })
    }
}
