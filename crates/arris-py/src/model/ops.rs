//! `Model`'s operations: primitives, transforms, the booleans, split, section
//! and the sweeps.

use super::*;

#[pymethods]
impl Model {
    /// A box with corners `min` and `max`, every face, edge and vertex
    /// named by a `Role` in the returned `Provenance`.
    ///
    /// Raises `OpDegenerateError` for a corner that is not finite or an
    /// extent that is not positive.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// assert abs(model.mass_properties(body).volume - 6) < 1e-12
    /// assert record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    /// ```
    #[pyo3(signature = (min, max, *, cancel=None, budget=None))]
    fn primitive_box(
        &self,
        py: Python<'_>,
        min: [f64; 3],
        max: [f64; 3],
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::primitive_box(m, min, max, control)
        })
    }

    /// A cylinder whose bottom cap is centred on `origin`, rising `height`
    /// along `axis` with the given `radius`.
    ///
    /// Raises `OpDegenerateError` for a radius or height that is not
    /// positive or an axis that is not finite or is zero.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_cylinder((0, 0, 0), (0, 0, 1), 2, 5)
    /// assert abs(model.mass_properties(body).volume - math.pi * 4 * 5) < 1e-9
    /// ```
    #[pyo3(signature = (origin, axis, radius, height, *, cancel=None, budget=None))]
    #[allow(clippy::too_many_arguments)] // the keywords are the Python signature
    fn primitive_cylinder(
        &self,
        py: Python<'_>,
        origin: [f64; 3],
        axis: [f64; 3],
        radius: f64,
        height: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let axis = finite("axis origin", origin)
            .and_then(|()| direction_of("axis direction", axis))
            .and_then(|d| Axis::new(Point3::from(origin), d).map_err(OpError::from))
            .map_err(|e| self.refuse(py, e))?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::primitive_cylinder(m, axis, radius, height, control)
        })
    }

    /// A copy of `body` turned by `angle` radians about the line through
    /// the origin along `axis`, then moved by `translation`.
    ///
    /// Raises `OpDegenerateError` for a number that is not finite or an
    /// axis of zero length; `ForeignHandleError` for a body of another model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// moved, _ = model.transform(body, (10, 0, 0))
    /// assert abs(model.mass_properties(moved).centroid[0] - 10.5) < 1e-12
    /// ```
    #[pyo3(signature = (body, translation=[0.0; 3], *, axis=[0.0, 0.0, 1.0], angle=0.0, cancel=None, budget=None))]
    #[allow(clippy::too_many_arguments)] // the keywords are the Python signature
    fn transform(
        &self,
        py: Python<'_>,
        body: &Body,
        translation: [f64; 3],
        axis: [f64; 3],
        angle: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let body = body.resolve(&self.shared)?;
        let motion = finite("translation", translation)
            .and_then(|()| direction_of("rotation axis", axis))
            .and_then(|a| {
                if angle.is_finite() {
                    let turn = UnitQuaternion::from_axis_angle(&UnitVec3::new_normalize(a), angle);
                    Ok(Isometry::new(turn, Vec3::from(translation)))
                } else {
                    Err(refused(Reason::Input(InputReason::NonFinite {
                        what: "angle",
                    })))
                }
            })
            .map_err(|e| self.refuse(py, e))?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::transform(m, body, &motion, control)
        })
    }

    /// A copy of `body` reflected in the plane through `origin` with the
    /// given `normal`; its faces face outward again.
    ///
    /// Raises `OpDegenerateError` for a coordinate that is not finite or a
    /// normal of zero length.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// mirrored, _ = model.mirror(body, (0, 0, 0), (1, 0, 0))
    /// assert abs(model.mass_properties(mirrored).centroid[0] + 0.5) < 1e-12
    /// assert abs(model.mass_properties(mirrored).volume - 6) < 1e-12
    /// ```
    #[pyo3(signature = (body, origin, normal, *, cancel=None, budget=None))]
    fn mirror(
        &self,
        py: Python<'_>,
        body: &Body,
        origin: [f64; 3],
        normal: [f64; 3],
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let body = body.resolve(&self.shared)?;
        let plane = finite("mirror origin", origin)
            .and_then(|()| direction_of("mirror normal", normal))
            .and_then(|n| {
                Reflection::new(Point3::from(origin), n).map_err(|_| {
                    refused(Reason::Input(InputReason::NonFinite {
                        what: "mirror plane",
                    }))
                })
            })
            .map_err(|e| self.refuse(py, e))?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::mirror(m, body, &plane, control)
        })
    }

    /// `target` with `tool` removed. The tool's faces that bound the new
    /// cavity are generated from; see the returned `Provenance`.
    ///
    /// Raises `OpUnsupportedError` (naming both operands) for a pair of
    /// surfaces with no closed form yet, `OpDegenerateError` when nothing
    /// would remain, and `Interrupted` when `cancel` or `budget` stops it.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// plate, _ = model.primitive_box((0, 0, 0), (10, 10, 10))
    /// tool, _ = model.primitive_cylinder((5, 5, -1), (0, 0, 1), 2, 12)
    /// holed, record = model.cut(plate, tool)
    /// volume = model.mass_properties(holed).volume
    /// assert abs(volume - (1000 - math.pi * 4 * 10)) < 1e-9
    /// assert record.generated
    /// ```
    #[pyo3(signature = (target, tool, *, cancel=None, budget=None))]
    fn cut(
        &self,
        py: Python<'_>,
        target: &Body,
        tool: &Body,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let (target, tool) = (target.resolve(&self.shared)?, tool.resolve(&self.shared)?);
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::cut(m, target, tool, control)
        })
    }

    /// The union of `a` and `b`. Errors as for `cut`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// a, _ = model.primitive_box((0, 0, 0), (2, 2, 2))
    /// b, _ = model.primitive_box((1, 0, 0), (3, 2, 2))
    /// union, _ = model.fuse(a, b)
    /// assert abs(model.mass_properties(union).volume - 12) < 1e-9
    /// ```
    #[pyo3(signature = (a, b, *, cancel=None, budget=None))]
    fn fuse(
        &self,
        py: Python<'_>,
        a: &Body,
        b: &Body,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let (a, b) = (a.resolve(&self.shared)?, b.resolve(&self.shared)?);
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::fuse(m, a, b, control)
        })
    }

    /// `target` with every body of `tools` removed, in one decomposition: the
    /// result of cutting them one after another, made by one pass over all of
    /// them, with one provenance record that names each tool. A pattern of
    /// holes is one call. The tools may overlap one another.
    ///
    /// Raises `OpDegenerateError` for an empty `tools` or a body named twice
    /// among the target and the tools, `OpUnsupportedError` for tools that
    /// touch one another where no result is defined (flush on one face,
    /// tangent), and otherwise as for `cut`.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// plate, _ = model.primitive_box((0, 0, 0), (40, 30, 10))
    /// tools = [
    ///     model.primitive_cylinder((x, 15, -1), (0, 0, 1), 3, 12)[0]
    ///     for x in (10, 20, 30)
    /// ]
    /// holed, record = model.cut_many(plate, tools)
    /// volume = model.mass_properties(holed).volume
    /// assert abs(volume - (12000 - 3 * math.pi * 9 * 10)) < 1e-9 * 12000
    /// assert record.generated
    /// ```
    #[pyo3(signature = (target, tools, *, cancel=None, budget=None))]
    fn cut_many(
        &self,
        py: Python<'_>,
        target: &Body,
        tools: Vec<PyRef<'_, Body>>,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let target = target.resolve(&self.shared)?;
        let tools = tools
            .iter()
            .map(|t| t.resolve(&self.shared))
            .collect::<Result<Vec<_>, _>>()?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::cut_many(m, target, &tools, control)
        })
    }

    /// The union of every body of `bodies`, in one decomposition: the result
    /// of fusing them one after another, with one provenance record that
    /// names each. Errors as for `cut_many`; fewer than two bodies is
    /// `OpDegenerateError`.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// plate, _ = model.primitive_box((0, 0, 0), (60, 30, 10))
    /// bosses = [
    ///     model.primitive_cylinder((x, 15, 5), (0, 0, 1), 4, 15)[0]
    ///     for x in (15, 45)
    /// ]
    /// fused, _ = model.fuse_many([plate, *bosses])
    /// volume = model.mass_properties(fused).volume
    /// assert abs(volume - (18000 + 2 * math.pi * 16 * 10)) < 1e-9 * 18000
    /// ```
    #[pyo3(signature = (bodies, *, cancel=None, budget=None))]
    fn fuse_many(
        &self,
        py: Python<'_>,
        bodies: Vec<PyRef<'_, Body>>,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let bodies = bodies
            .iter()
            .map(|b| b.resolve(&self.shared))
            .collect::<Result<Vec<_>, _>>()?;
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::fuse_many(m, &bodies, control)
        })
    }

    /// The intersection of `a` and `b`. Errors as for `cut`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// a, _ = model.primitive_box((0, 0, 0), (2, 2, 2))
    /// b, _ = model.primitive_box((1, 0, 0), (3, 2, 2))
    /// overlap, _ = model.common(a, b)
    /// assert abs(model.mass_properties(overlap).volume - 4) < 1e-9
    /// ```
    #[pyo3(signature = (a, b, *, cancel=None, budget=None))]
    fn common(
        &self,
        py: Python<'_>,
        a: &Body,
        b: &Body,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let (a, b) = (a.resolve(&self.shared)?, b.resolve(&self.shared)?);
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::common(m, a, b, control)
        })
    }

    /// `body` cut by the plane through `origin` with the given `normal` into
    /// the solid on each side: `(positive, negative, record)`, `positive`
    /// the one the normal points to. Either may have several lumps. The cut
    /// faces are generated from `Role("split", "Cap", "Positive")` and
    /// `Role("split", "Cap", "Negative")`; their (u, v) are the plane's `x`
    /// direction (any perpendicular of the normal when `x` is `None`) and its
    /// cross product with the normal. Every entity the plane does not touch
    /// keeps its id on its side.
    ///
    /// Raises `OpDegenerateError` for a coordinate that is not finite, a
    /// normal of zero length or an `x` with no part across it, and for a plane that
    /// misses the body or only touches it; `OpUnsupportedError` and
    /// `OpDegenerateError` as for `cut` where the plane meets the body as a
    /// boolean would refuse; `Interrupted` when `cancel` or `budget` stops it.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// block, _ = model.primitive_box((0, 0, 0), (10, 10, 10))
    /// above, below, record = model.split(block, (0, 0, 4), (0, 0, 1))
    /// assert abs(model.mass_properties(above).volume - 600) < 1e-9
    /// assert abs(model.mass_properties(below).volume - 400) < 1e-9
    /// assert record.generated_from(arris.Role("split", "Cap", "Positive"))
    /// ```
    #[pyo3(signature = (body, origin, normal, x=None, *, cancel=None, budget=None))]
    #[allow(clippy::too_many_arguments)] // the keywords are the Python signature
    fn split(
        &self,
        py: Python<'_>,
        body: &Body,
        origin: [f64; 3],
        normal: [f64; 3],
        x: Option<[f64; 3]>,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<(Body, Body, Provenance)> {
        let body = body.resolve(&self.shared)?;
        let plane = plane_of("split", origin, normal, x).map_err(|e| self.refuse(py, e))?;
        let limits = Limits::new(cancel, budget);
        let halves = self.run(py, &limits, move |m, control| {
            arris::ops::split(m, body, &plane, control)
        })?;
        Ok((
            Body::minted_by(self, halves.positive),
            Body::minted_by(self, halves.negative),
            Provenance::minted_by(self, halves.provenance),
        ))
    }

    /// What the plane through `origin` with the given `normal` cuts out of
    /// the solid `body`, without changing it: a `Section`, or `None` when the
    /// plane misses the body or only touches it (along a face, an edge or a
    /// point). The section's sheet is a body of planar faces on the plane,
    /// one per connected region, with the region's holes as inner loops; its
    /// (u, v) are the plane's `x` direction (any perpendicular of the normal
    /// when `x` is `None`) and its cross product with the normal, and the
    /// faces face the normal. A region where a face of the body lies on the
    /// plane is not section: the section is the plane's points with
    /// material on both sides.
    ///
    /// Raises `OpDegenerateError` for a coordinate that is not finite, a
    /// normal of zero length, an `x` with no part across it, and a body that
    /// is not a solid; `OpUnsupportedError` and `OpDegenerateError` as for
    /// `split` where the plane meets the body as a boolean would refuse;
    /// `Interrupted` when `cancel` or `budget` stops it.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// block, _ = model.primitive_box((0, 0, 0), (10, 6, 4))
    /// cut = model.section(block, (0, 0, 1.5), (0, 0, 1))
    /// assert abs(cut.area - 60) < 1e-9
    /// assert all(abs(c - h) < 1e-9 for c, h in zip(cut.centroid, (5, 3, 1.5)))
    /// assert len(model.faces(cut.sheet)) == 1
    /// assert model.section(block, (0, 0, 9), (0, 0, 1)) is None
    /// ```
    #[pyo3(signature = (body, origin, normal, x=None, *, cancel=None, budget=None))]
    #[allow(clippy::too_many_arguments)] // the keywords are the Python signature
    fn section(
        &self,
        py: Python<'_>,
        body: &Body,
        origin: [f64; 3],
        normal: [f64; 3],
        x: Option<[f64; 3]>,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Option<Section>> {
        let body = body.resolve(&self.shared)?;
        let plane = plane_of("section", origin, normal, x).map_err(|e| self.refuse(py, e))?;
        let limits = Limits::new(cancel, budget);
        let made = self.run(py, &limits, move |m, control| {
            arris::ops::section(m, body, &plane, control)
        })?;
        Ok(made.map(|s| Section::new(self, s)))
    }

    /// The solid `profile` sweeps along `direction` for `length`: caps from
    /// the profile's face and a side face for each of its segments, every
    /// entity named by the part of the sketch it came from
    /// (`Role("extrude", "Side", loop, segment)`).
    ///
    /// Raises `OpProfileError` for an invalid sketch and `OpDegenerateError`
    /// for a length that is not positive or finite or a direction that is
    /// not along the profile's normal.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// sketch = arris.Profile(arris.Loop.polygon([(0, 0), (4, 0), (4, 3), (0, 3)]))
    /// body, record = model.extrude(sketch, (0, 0, 1), 2)
    /// assert abs(model.mass_properties(body).volume - 24) < 1e-9
    /// assert record.generated_from(arris.Role("extrude", "Side", 0, 0))
    /// ```
    #[pyo3(signature = (profile, direction, length, *, cancel=None, budget=None))]
    fn extrude(
        &self,
        py: Python<'_>,
        profile: &Profile,
        direction: [f64; 3],
        length: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let direction =
            direction_of("extrude direction", direction).map_err(|e| self.refuse(py, e))?;
        let (profile, limits) = (profile.kernel.clone(), Limits::new(cancel, budget));
        self.operate(py, &limits, move |m, control| {
            arris::ops::extrude(m, &profile, direction, length, control)
        })
    }

    /// The sheet body of `profile`'s face: one planar face on the profile's
    /// plane, its front along the plane's normal, every entity named by the
    /// part of the sketch it came from
    /// (`Role("planar_face", "StartEdge", loop, segment)`; the face is
    /// `Role("planar_face", "StartCap")`).
    ///
    /// Raises `OpProfileError` for an invalid sketch.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// sketch = arris.Profile(arris.Loop.polygon([(0, 0), (4, 0), (4, 3), (0, 3)]))
    /// sheet, record = model.planar_face(sketch)
    /// assert model.body_kind(sheet) == "sheet"
    /// assert abs(model.surface_properties(sheet).area - 12) < 1e-9
    /// assert record.generated_from(arris.Role("planar_face", "StartCap"))
    /// ```
    #[pyo3(signature = (profile, *, cancel=None, budget=None))]
    fn planar_face(
        &self,
        py: Python<'_>,
        profile: &Profile,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let (profile, limits) = (profile.kernel.clone(), Limits::new(cancel, budget));
        self.operate(py, &limits, move |m, control| {
            arris::ops::planar_face(m, &profile, control)
        })
    }

    /// The solid `profile` sweeps turning by `angle` radians (a full turn
    /// is `2π`) about the line through `origin` along `axis`; the axis must
    /// lie in the profile's plane.
    ///
    /// Raises `OpProfileError` for an invalid sketch and `OpDegenerateError`
    /// for an axis off the plane, a profile across the axis, or an angle
    /// that is not in `(0, 2π]`.
    ///
    /// ```python
    /// import math
    ///
    /// import arris
    ///
    /// model = arris.Model()
    /// sketch = arris.Profile(arris.Loop.polygon([(1, 0), (2, 0), (2, 1), (1, 1)]))
    /// body, _ = model.revolve(sketch, (0, 0, 0), (0, 1, 0), 2 * math.pi)
    /// assert abs(model.mass_properties(body).volume - 3 * math.pi) < 1e-9
    /// ```
    #[pyo3(signature = (profile, origin, axis, angle, *, cancel=None, budget=None))]
    #[allow(clippy::too_many_arguments)] // the keywords are the Python signature
    fn revolve(
        &self,
        py: Python<'_>,
        profile: &Profile,
        origin: [f64; 3],
        axis: [f64; 3],
        angle: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let axis = finite("axis origin", origin)
            .and_then(|()| direction_of("axis direction", axis))
            .and_then(|d| Axis::new(Point3::from(origin), d).map_err(OpError::from))
            .map_err(|e| self.refuse(py, e))?;
        let (profile, limits) = (profile.kernel.clone(), Limits::new(cancel, budget));
        self.operate(py, &limits, move |m, control| {
            arris::ops::revolve(m, &profile, axis, angle, control)
        })
    }

    /// The solid `profile` sweeps along `path`: the profile is taken where
    /// it is placed (it need not touch the path or be normal to it), turned
    /// by `law` (`FrameLaw.rotation_minimizing()` when omitted). A line
    /// span gives `extrude`'s faces, an arc span about an axis in the
    /// profile's plane gives `revolve`'s, and any other span a fitted NURBS
    /// face. Every entity is named by the span and the part of the sketch it
    /// came from (`Role("sweep", "Side", span, loop, segment)`); a closed
    /// path whose frame closes gives a body with no caps.
    ///
    /// Raises `OpProfileError` for an invalid sketch, `OpPathError` for a
    /// path with a gap, a corner or a degenerate segment, and
    /// `OpUnsupportedError` or `OpDegenerateError` for a profile along its
    /// path, a path bent tighter than the profile is wide, or a frame that
    /// does not close.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// sketch = arris.Profile(
    ///     arris.Loop.circle((0, 0), 1), origin=(0, 0, 0), normal=(0, 0, 1)
    /// )
    /// spine = arris.Path((0, 0, 0), [arris.PathSegment.line((0, 0, 10))])
    /// body, record = model.sweep(sketch, spine)
    /// assert abs(model.mass_properties(body).volume - 10 * 3.141592653589793) < 1e-9
    /// assert record.generated_from(arris.Role("sweep", "Side", 0, 0, 0))
    /// ```
    #[pyo3(signature = (profile, path, law=None, *, cancel=None, budget=None))]
    fn sweep(
        &self,
        py: Python<'_>,
        profile: &Profile,
        path: &Path,
        law: Option<&FrameLaw>,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let law = law.map(|l| l.kernel).unwrap_or_default();
        let (profile, path) = (profile.kernel.clone(), path.kernel.clone());
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::sweep(m, &profile, &path, law, control)
        })
    }

    /// The solid through two or more placed `sections` (profiles with the
    /// same number of loops), in order. `kind="ruled"` makes each span
    /// between consecutive sections straight: planes, cylinders and cones
    /// where the sections allow, a degree-1 NURBS face otherwise;
    /// `kind="smooth"` makes one surface through every section (at most
    /// nine). A section with fewer segments is cut by arc length to match
    /// the other, each loop's start point fixing the correspondence. Open,
    /// the first and last sections are planar caps; `closed=True` joins the
    /// last section to the first and builds no caps. Every entity is named
    /// by its span and the part of the sketch it came from
    /// (`Role("loft", ...)`).
    ///
    /// Raises `ValueError` for a `kind` other than `"ruled"` or `"smooth"`;
    /// `OpLoftError` (its `detail` names the section) for too few sections,
    /// unequal loop counts, two coincident consecutive sections or too many
    /// for a smooth loft. A section that is not a valid profile is
    /// an `OpLoftError` too.
    ///
    /// ```python
    /// import arris
    ///
    /// def square(z, half):
    ///     corners = [(half, half), (-half, half), (-half, -half), (half, -half)]
    ///     return arris.Profile(
    ///         arris.Loop.polygon(corners), origin=(0, 0, z), normal=(0, 0, 1)
    ///     )
    ///
    /// model = arris.Model()
    /// body, _ = model.loft([square(0, 2), square(3, 1)])
    /// # A frustum: h/3 (A1 + A2 + sqrt(A1 A2)) with areas 16 and 4.
    /// assert abs(model.mass_properties(body).volume - 28) < 1e-9
    /// ```
    #[pyo3(signature = (sections, kind="ruled", closed=false, *, cancel=None, budget=None))]
    fn loft(
        &self,
        py: Python<'_>,
        sections: Vec<PyRef<'_, Profile>>,
        kind: &str,
        closed: bool,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Made> {
        let kind = match kind {
            "ruled" => arris::ops::LoftKind::Ruled,
            "smooth" => arris::ops::LoftKind::Smooth,
            other => {
                return Err(pyo3::exceptions::PyValueError::new_err(format!(
                    "{other:?} is not a loft kind (\"ruled\" or \"smooth\")"
                )));
            }
        };
        let sections: Vec<_> = sections.iter().map(|p| p.kernel.clone()).collect();
        let limits = Limits::new(cancel, budget);
        self.operate(py, &limits, move |m, control| {
            arris::ops::loft(m, &sections, kind, closed, control)
        })
    }
}
