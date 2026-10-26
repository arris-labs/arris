//! `Model`'s queries: properties, geometry, frames, rays, distance,
//! interference, the checker and tessellation.

use super::*;

#[pymethods]
impl Model {
    /// The volume, area, centroid and inertia of the solid `body`, at unit
    /// density.
    ///
    /// Raises `OpDegenerateError` for a body that is not a solid and
    /// `Interrupted` when `cancel` or `budget` stops it.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// props = model.mass_properties(body)
    /// assert abs(props.volume - 6) < 1e-12 and abs(props.area - 22) < 1e-12
    /// assert props.centroid == (0.5, 1.0, 1.5)
    /// ```
    #[pyo3(signature = (body, *, cancel=None, budget=None))]
    fn mass_properties(
        &self,
        py: Python<'_>,
        body: &Body,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<MassProperties> {
        let body = body.resolve(&self.shared)?;
        let limits = Limits::new(cancel, budget);
        let kernel = self.run(py, &limits, move |m, control| {
            arris::ops::measure::mass_properties(m, body, control)
        })?;
        Ok(MassProperties::new(kernel))
    }

    /// The area, centroid and inertia of the faces of `body` — a solid's
    /// boundary or a sheet's faces — at unit area density.
    ///
    /// Raises `OpDegenerateError` for a body whose faces have no area and
    /// `Interrupted` when `cancel` or `budget` stops it.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// props = model.surface_properties(body)
    /// assert abs(props.area - 22) < 1e-12
    /// assert props.centroid == (0.5, 1.0, 1.5)
    /// ```
    #[pyo3(signature = (body, *, cancel=None, budget=None))]
    fn surface_properties(
        &self,
        py: Python<'_>,
        body: &Body,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<SurfaceProperties> {
        let body = body.resolve(&self.shared)?;
        let limits = Limits::new(cancel, budget);
        let kernel = self.run(py, &limits, move |m, control| {
            arris::ops::measure::surface_properties(m, body, control)
        })?;
        Ok(SurfaceProperties::new(kernel))
    }

    /// The point of `vertex`, as stored (the true point lies within the
    /// vertex's tolerance of it).
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// points = sorted(model.point(v) for v in model.vertices(body))
    /// assert points[0] == (0.0, 0.0, 0.0) and points[-1] == (1.0, 2.0, 3.0)
    /// ```
    fn point(&self, vertex: &Vertex) -> PyResult<(f64, f64, f64)> {
        let vertex = vertex.resolve(&self.shared)?;
        let p = self
            .shared
            .lock()?
            .vertex(vertex.id)
            .map_err(BindError::from)?
            .point();
        Ok((p.x, p.y, p.z))
    }

    /// The curve under `edge`: its kind, its ends in the direction the edge
    /// was reached, and a line's or a conic's parameters; see `CurveInfo`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// [edge, *_] = model.edges(body)
    /// line = model.curve(edge)
    /// assert line.kind == "line" and line.radius is None
    /// start, end = model.vertices_of(edge)
    /// assert (line.start, line.end) == (model.point(start), model.point(end))
    /// ```
    fn curve(&self, edge: &Edge) -> PyResult<CurveInfo> {
        let edge = edge.resolve(&self.shared)?;
        Ok(CurveInfo::of(&*self.shared.lock()?, edge).map_err(BindError::from)?)
    }

    /// The surface under `face`: its kind, its frame, a plane's outward
    /// normal and a quadric's or a torus's axis and radii; see
    /// `SurfaceInfo`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// [top] = [f for f in record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    ///          if isinstance(f, arris.Face)]
    /// plane = model.surface(top)
    /// assert plane.kind == "plane" and plane.normal == (0.0, 0.0, 1.0)
    /// assert plane.origin[2] == 3.0 and plane.axis is None
    /// ```
    fn surface(&self, face: &Face) -> PyResult<SurfaceInfo> {
        let face = face.resolve(&self.shared)?;
        Ok(SurfaceInfo::of(&*self.shared.lock()?, face).map_err(BindError::from)?)
    }

    /// The axis-aligned box `entity` lies in, as `(min, max)`, grown by the
    /// tolerances it carries: tight for a vertex, a line, a conic and an
    /// analytic face, a control hull's box for a NURBS curve or face, so
    /// loose there; a shell's or a body's is the union of its entities'.
    /// `None` for an entity with an unbounded face (never one of a solid).
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_cylinder((0, 0, 0), (0, 0, 1), 2, 5)
    /// lo, hi = model.bounds(body)
    /// assert all(abs(a - b) < 1e-6 for a, b in zip(lo, (-2, -2, 0)))
    /// assert all(abs(a - b) < 1e-6 for a, b in zip(hi, (2, 2, 5)))
    /// ```
    fn bounds(&self, entity: AnyHandle<'_>) -> PyResult<Option<Corners>> {
        let entity = entity.resolve(&self.shared)?;
        let b = crate::geometry::bounds(&*self.shared.lock()?, entity).map_err(BindError::from)?;
        Ok(b.map(|b| {
            let ([x0, y0, z0], [x1, y1, z1]) = (b.min, b.max);
            ((x0, y0, z0), (x1, y1, z1))
        }))
    }

    /// The frame of a planar `face`: `z` is its outward normal, reversed
    /// for a face reached against its orientation.
    ///
    /// Raises `OpDegenerateError` for a face that is not planar.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// [top] = [f for f in record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    ///          if isinstance(f, arris.Face)]
    /// assert model.face_frame(top).z == (0.0, 0.0, 1.0)
    /// ```
    fn face_frame(&self, py: Python<'_>, face: &Face) -> PyResult<Frame> {
        let face = face.resolve(&self.shared)?;
        let kernel = arris::ops::query::face_frame(&*self.shared.lock()?, face)
            .map_err(|e| self.refuse(py, e))?;
        Ok(Frame::new(kernel))
    }

    /// The outward frame of `face`'s surface at `(u, v)`: `z` is the
    /// normal there, `x` the tangent along `u`.
    ///
    /// Raises `OpDegenerateError` for a point outside the face or at a
    /// singularity of its surface.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// [top] = [f for f in record.generated_from(arris.Role("box", "Face", "Z", "Max"))
    ///          if isinstance(f, arris.Face)]
    /// frame = model.frame_at(top, 0.5, 0.5)
    /// assert frame.z == (0.0, 0.0, 1.0) and frame.origin[2] == 3.0
    /// ```
    fn frame_at(&self, py: Python<'_>, face: &Face, u: f64, v: f64) -> PyResult<Frame> {
        let face = face.resolve(&self.shared)?;
        let uv = arris::math::Point2::new(u, v);
        let kernel = arris::ops::query::frame_at(&*self.shared.lock()?, face, uv)
            .map_err(|e| self.refuse(py, e))?;
        Ok(Frame::new(kernel))
    }

    /// Where a ray meets `body`: the hits by ascending `t`, or only the first
    /// when `all` is false. The ray runs from `origin` along `direction`
    /// (any length but zero), and a hit behind `origin` is never reported.
    ///
    /// A hit names the most specific entity it lands on within its
    /// tolerance; see `RayHit`. Raises `OpDegenerateError` for a zero
    /// direction, for a ray lying in a face (`RayInFace`) and for a hit at a
    /// pole or an apex (`Singular`), each naming the entity, and
    /// `OpInvalidInputError` for a body that fails the checker (debug
    /// builds).
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_box((0, 0, 0), (4, 3, 2))
    /// hits = model.ray_fire(body, (2, 1.5, -1), (0, 0, 2), all=True)
    /// assert [round(h.t, 9) for h in hits] == [1.0, 3.0]
    /// assert hits[1].normal == (0.0, 0.0, 1.0)
    /// first = model.ray_fire(body, (2, 1.5, -1), (0, 0, 2))
    /// assert len(first) == 1 and first[0].t == hits[0].t
    /// ```
    #[pyo3(signature = (body, origin, direction, all=false))]
    fn ray_fire(
        &self,
        py: Python<'_>,
        body: &Body,
        origin: [f64; 3],
        direction: [f64; 3],
        all: bool,
    ) -> PyResult<Vec<RayHit>> {
        let body = body.resolve(&self.shared)?;
        let ray = arris::ops::query::Ray {
            origin: Point3::from(origin),
            direction: Vec3::from(direction),
        };
        let which = if all {
            arris::ops::query::RayHits::All
        } else {
            arris::ops::query::RayHits::First
        };
        let hits = py
            .detach(|| {
                let kernel = self.shared.lock()?;
                Ok::<_, BindError>(arris::ops::query::ray_fire(&kernel, body, &ray, which))
            })?
            .map_err(|e| self.refuse(py, e))?;
        Ok(hits.into_iter().map(|h| RayHit::new(self, h)).collect())
    }

    /// The minimum distance between `a` and `b`, each a point `(x, y, z)` or
    /// a handle: a `Vertex`, `Edge`, `Face` or `Body`, in this model.
    ///
    /// A body is its solid, so a point inside it, or a body inside another,
    /// is at distance `0.0` and the answer is `contained`. The answer names
    /// the closest point on each operand and the vertex, edge or face it
    /// lies on (see `Distance`). It comes from certified critical points,
    /// never a mesh.
    ///
    /// Raises `OpDegenerateError` for a shell or a body that is not a solid
    /// (reason `NotMeasurable`) and for a point that is not finite,
    /// `OpUnsupportedError` for a pair of surfaces with no closed form
    /// (two off-axis cones, say) that no nearer pair rules out,
    /// `OpInvalidInputError` for a body that fails the checker (debug
    /// builds) and `Interrupted` when `cancel` or `budget` stops it.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_box((0, 0, 0), (4, 3, 2))
    /// found = model.distance((1, 1, 5), body)
    /// assert abs(found.distance - 3) < 1e-12 and found.unique
    /// assert isinstance(found.b.entity, arris.Face)
    /// assert model.distance((1, 1, 1), body).contained
    /// ```
    #[pyo3(signature = (a, b, *, cancel=None, budget=None))]
    fn distance(
        &self,
        py: Python<'_>,
        a: DistanceOperand<'_>,
        b: DistanceOperand<'_>,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Distance> {
        let a = a.resolve(&self.shared)?;
        let b = b.resolve(&self.shared)?;
        let limits = Limits::new(cancel, budget);
        let kernel = self.run(py, &limits, move |m, control| {
            arris::ops::query::distance(m, a, b, control)
        })?;
        Ok(Distance::new(self, kernel))
    }

    /// How solid `a` interferes with solid `b`: `"apart"` with their
    /// `Distance`, `"touching"` with the dimension of their contact and a
    /// point of it, or `"overlapping"` with the volume they share and
    /// whether either lies wholly inside the other (see `Interference`). The
    /// shared volume is what `common` returns, measured on a scratch copy:
    /// the model is never written. A body against itself is inside itself.
    ///
    /// Raises what `distance` raises for the pair, and what `common` raises
    /// for a pair that is not apart: `OpUnsupportedError` for a NURBS face
    /// meeting the other solid, `OpDegenerateError` for a contact `common`
    /// cannot read (a tangent contact), and `Interrupted` when `cancel` or
    /// `budget` stops it.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// a, _ = model.primitive_box((0, 0, 0), (2, 2, 2))
    /// flush, _ = model.primitive_box((2, 0, 0), (4, 2, 2))
    /// far, _ = model.primitive_box((9, 0, 0), (11, 2, 2))
    /// assert model.interference(a, flush).contact == "face"
    /// assert abs(model.interference(a, far).distance.distance - 7) < 1e-12
    /// ```
    #[pyo3(signature = (a, b, *, cancel=None, budget=None))]
    fn interference(
        &self,
        py: Python<'_>,
        a: &Body,
        b: &Body,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Interference> {
        let (a, b) = (a.resolve(&self.shared)?, b.resolve(&self.shared)?);
        let limits = Limits::new(cancel, budget);
        let kernel = self.run(py, &limits, move |m, control| {
            arris::ops::query::interference(m, a, b, control)
        })?;
        Ok(Interference::new(self, kernel))
    }

    /// Screens an assembly: a `Clash` for every pair of `bodies` that is not
    /// apart, in ascending pair order, each with the `Interference` of the
    /// lower-indexed body against the other. Pairs whose bounding boxes are
    /// apart cost a box test. Raises as `interference` does, for the first
    /// candidate pair that fails.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// cubes = [model.primitive_box((x, 0, 0), (x + 2, 2, 2))[0] for x in (0, 1, 10)]
    /// clashes = model.interference_many(cubes)
    /// assert [c.operands for c in clashes] == [(0, 1)]
    /// assert abs(clashes[0].interference.volume - 4) < 1e-12
    /// ```
    #[pyo3(signature = (bodies, *, cancel=None, budget=None))]
    fn interference_many(
        &self,
        py: Python<'_>,
        bodies: Vec<PyRef<'_, Body>>,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Vec<Clash>> {
        let bodies = bodies
            .iter()
            .map(|b| b.resolve(&self.shared))
            .collect::<Result<Vec<_>, _>>()?;
        let limits = Limits::new(cancel, budget);
        let kernel = self.run(py, &limits, move |m, control| {
            arris::ops::query::interference_many(m, &bodies, control)
        })?;
        Ok(kernel.into_iter().map(|c| Clash::new(self, c)).collect())
    }

    /// The checker's report on `body`: `level` is `"fast"` (linear in the
    /// body, what every operation asserts of its result in debug builds) or
    /// `"full"` (adds the global rows: face-face intersection, shell
    /// nesting, enclosed volume).
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// report = model.check(body, "full")
    /// assert report and not report.violations and not report.unchecked
    /// ```
    #[pyo3(signature = (body, level="fast"))]
    fn check(&self, py: Python<'_>, body: &Body, level: &str) -> PyResult<Report> {
        let (body, level) = (body.resolve(&self.shared)?, level_of(level)?);
        let report = py.detach(|| {
            let kernel = self.shared.lock()?;
            Ok::<_, BindError>(arris::check::check(&kernel, body, level))
        })?;
        Ok(Report::new(Some(self), report))
    }

    /// The triangle mesh of `body`: every position within `chord` of the
    /// surface it stands on, closed along shared edges, every triangle
    /// counter-clockwise seen from outside. `chord` is the caller's
    /// resolution, not a model tolerance: finite and positive.
    ///
    /// Raises `MeshChordError` for a bad chord, `MeshInvalidInputError` for
    /// a body that fails the checker (debug builds), `MeshFaceError` for a
    /// face that cannot be triangulated, and `Interrupted` when `cancel` or
    /// `budget` stops it.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// mesh = model.tessellate(body, 1e-3)
    /// assert mesh.is_closed() and mesh.n_triangles == 12
    /// assert abs(mesh.signed_volume() - 6) < 1e-9
    /// ```
    #[pyo3(signature = (body, chord, *, cancel=None, budget=None))]
    fn tessellate(
        &self,
        py: Python<'_>,
        body: &Body,
        chord: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Mesh> {
        let body = body.resolve(&self.shared)?;
        let limits = Limits::new(cancel, budget);
        let kernel = self.run_with(
            py,
            &limits,
            move |m, control| arris::mesh::tessellate(m, body, chord, control),
            mesh_error,
            true,
            |error| {
                matches!(
                    error,
                    arris::mesh::MeshError::Interrupted(Interrupted { by: Stop::Poll, .. })
                )
            },
        )?;
        Ok(Mesh::minted_by(self, kernel))
    }

    /// The triangle mesh of some of `body`'s faces, `faces` in any order:
    /// each edge's polyline and each face's triangles are exactly those
    /// `tessellate` gives, so meshes of different faces made at different
    /// times meet bit for bit and `arris.weld` joins them. The mesh holds
    /// the faces asked for, in the body's order, and the edges and vertices
    /// they use.
    ///
    /// Raises `MeshNotInBodyError` for a face the body does not have,
    /// `ForeignHandleError` for a face of another model, and what
    /// `tessellate` raises.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, _ = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// faces = model.faces(body)
    /// mesh = model.tessellate_faces(body, faces[:2], 1e-3)
    /// assert mesh.n_triangles == 4 and not mesh.is_closed()
    /// ```
    #[pyo3(signature = (body, faces, chord, *, cancel=None, budget=None))]
    fn tessellate_faces(
        &self,
        py: Python<'_>,
        body: &Body,
        faces: Vec<PyRef<'_, Face>>,
        chord: f64,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Mesh> {
        let body = body.resolve(&self.shared)?;
        let faces = faces
            .iter()
            .map(|f| f.resolve(&self.shared).map(|f| f.id))
            .collect::<Result<Vec<_>, BindError>>()?;
        let limits = Limits::new(cancel, budget);
        let kernel = self.run_with(
            py,
            &limits,
            move |m, control| {
                arris::mesh::tessellate_faces(
                    m,
                    body,
                    &faces,
                    &arris::mesh::MeshRequest::new(chord),
                    control,
                )
            },
            mesh_error,
            true,
            |error| {
                matches!(
                    error,
                    arris::mesh::MeshError::Interrupted(Interrupted { by: Stop::Poll, .. })
                )
            },
        )?;
        Ok(Mesh::minted_by(self, kernel))
    }
}
