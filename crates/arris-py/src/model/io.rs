//! `Model`'s io: STEP, body JSON and the native format.

use super::*;

#[pymethods]
impl Model {
    /// The STEP text of `bodies`, in the order given, as one product with a
    /// solid entity per lump of each body (AP214). The file declares
    /// `length_unit` (`"mm"` by default; also `"um"`, `"cm"`, `"m"`, `"in"`,
    /// `"ft"`) as its length unit and converts nothing: the coordinates are
    /// written as the model holds them. Deterministic: the same bodies write
    /// the same text.
    ///
    /// Raises `StepUnsupportedError` for a body that is not a solid or has a
    /// form STEP cannot hold, `StepLumpsError` for shells that do not nest,
    /// `StepNoBodiesError` for an empty list, `ForeignHandleError` for a
    /// body of another model and `ValueError` for an unknown unit.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// text = model.write_step([body])
    /// assert text.startswith("ISO-10303-21;") and "MANIFOLD_SOLID_BREP" in text
    /// metres = model.write_step([body], length_unit="m")
    /// assert "SI_UNIT($,.METRE.)" in metres
    /// ```
    #[pyo3(signature = (bodies, *, length_unit="mm"))]
    fn write_step(
        &self,
        py: Python<'_>,
        bodies: Vec<PyRef<'_, Body>>,
        length_unit: &str,
    ) -> PyResult<String> {
        let options = arris::io::step::WriteOptions {
            length_unit: length_unit_of(length_unit)?,
        };
        let bodies = bodies
            .iter()
            .map(|b| b.resolve(&self.shared))
            .collect::<Result<Vec<_>, _>>()?;
        py.detach(|| {
            let kernel = self.shared.lock()?;
            Ok::<_, BindError>(arris::io::step::write_with(&kernel, &bodies, &options))
        })?
        .map_err(|e| step_error(&e).raise(py, Some(self)))
    }

    /// Reads every solid of the STEP file `text` into this model, lengths
    /// converted to `length_unit` (`"mm"` by default; also `"um"`, `"cm"`,
    /// `"m"`, `"in"`, `"ft"`).
    ///
    /// A solid the reader cannot read is a `StepSolid` with a `refusal`
    /// naming the file entity where it stopped and leaves nothing in the
    /// model; one refused solid never hides another. With `heal` (the
    /// default) a vertex or edge lying a little off the faces it meets is
    /// read with its tolerance grown to the gap, and a face written walking
    /// its edges the way its neighbours do is turned back, each named in
    /// the solid's `healing`; `heal=False` refuses them as a `"Gap"` and
    /// a `"Topology"`. Raises `StepParseError`
    /// when the text is not Part 21 at all, `Interrupted` when `cancel` or
    /// `budget` stops it (the model is as it was) and `ValueError` for an
    /// unknown unit.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// back = arris.Model()
    /// [solid] = back.read_step(model.write_step([body])).solids
    /// assert solid.ok and abs(back.mass_properties(solid.body).volume - 6) < 1e-9
    /// ```
    #[pyo3(signature = (text, *, length_unit="mm", heal=true, cancel=None, budget=None))]
    fn read_step(
        &self,
        py: Python<'_>,
        text: &str,
        length_unit: &str,
        heal: bool,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<StepRead> {
        let options = arris::io::step::ReadOptions {
            length_unit: length_unit_of(length_unit)?,
            heal: if heal {
                arris::io::step::Heal::GapsAndOrientation
            } else {
                arris::io::step::Heal::Off
            },
        };
        let limits = Limits::new(cancel, budget);
        let read = self.run_with(
            py,
            &limits,
            |m, control| arris::io::step::read(m, text, &options, control),
            step_read_error,
            true,
            |error| {
                matches!(
                    error,
                    arris::io::step::ReadError::Interrupted(Interrupted { by: Stop::Poll, .. })
                )
            },
        )?;
        Ok(StepRead::minted_by(self, read))
    }

    /// `body` and its `provenance` (empty by default) as body bytes: a
    /// self-contained, versioned record for storage, read by every later
    /// release. The record may name entities outside the body, such as a
    /// boolean's inputs.
    ///
    /// Raises `ForeignHandleError` for a body or record of another model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// data = model.write_body(body, record)
    /// assert data[:8] == b"ARRISBDY"
    /// ```
    #[pyo3(signature = (body, provenance=None))]
    fn write_body<'py>(
        &self,
        py: Python<'py>,
        body: &Body,
        provenance: Option<&Provenance>,
    ) -> PyResult<Bound<'py, pyo3::types::PyBytes>> {
        let bytes = self.write_body_with(py, body, provenance, |m, b, r| {
            arris::io::body::write(m, b, r)
        })?;
        Ok(pyo3::types::PyBytes::new(py, &bytes))
    }

    /// The same body as `write_body`, as one line of JSON text, for diffs
    /// and tests.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// assert model.write_body_json(body, record).startswith('{"magic":"ARRISBDY"')
    /// ```
    #[pyo3(signature = (body, provenance=None))]
    fn write_body_json(
        &self,
        py: Python<'_>,
        body: &Body,
        provenance: Option<&Provenance>,
    ) -> PyResult<String> {
        self.write_body_with(py, body, provenance, |m, b, r| {
            arris::io::body::to_json(m, b, r)
        })
    }

    /// Imports the body in `data` (`write_body`'s bytes) into this model,
    /// checked at the `full` level, and returns it with its record.
    ///
    /// Raises `BodyMagicError` for data that is not body bytes,
    /// `BodyVersionError` for a newer version, `BodyDecodeError` for a
    /// damaged stream, `BodyPrecisionError` for a tolerance this model
    /// cannot hold and `BodyRejectedError` for a body the checker rejects.
    /// On any error the model is as it was.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// imported = arris.Model().read_body(model.write_body(body, record))
    /// assert imported.foreign() == []
    /// ```
    #[pyo3(signature = (data, *, cancel=None, budget=None))]
    fn read_body(
        &self,
        py: Python<'_>,
        data: &[u8],
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Imported> {
        let limits = Limits::new(cancel, budget);
        self.read_body_with(py, &limits, |m, control| {
            arris::io::body::read(m, data, control)
        })
    }

    /// As `read_body`, for `write_body_json`'s text.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// imported = arris.Model().read_body_json(model.write_body_json(body, record))
    /// assert imported.version >= 1
    /// ```
    #[pyo3(signature = (text, *, cancel=None, budget=None))]
    fn read_body_json(
        &self,
        py: Python<'_>,
        text: &str,
        cancel: Option<&Cancel>,
        budget: Option<u64>,
    ) -> PyResult<Imported> {
        let limits = Limits::new(cancel, budget);
        self.read_body_with(py, &limits, |m, control| {
            arris::io::body::from_json(m, text, control)
        })
    }

    /// The whole model, every entity and freed slot, as native-format
    /// bytes: deterministic, and read back to a model that dumps the same.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// data = model.to_native()
    /// assert arris.Model.from_native(data).to_native() == data
    /// ```
    fn to_native<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, pyo3::types::PyBytes>> {
        let bytes = py
            .detach(|| {
                let kernel = self.shared.lock()?;
                Ok::<_, BindError>(arris::io::native::to_bytes(&kernel))
            })?
            .map_err(|e| native_error(&e).raise(py, None))?;
        Ok(pyo3::types::PyBytes::new(py, &bytes))
    }

    /// The whole model as native-format JSON text, for diffs.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// text = model.to_native_json()
    /// assert arris.Model.from_native_json(text).to_native_json() == text
    /// ```
    fn to_native_json(&self, py: Python<'_>) -> PyResult<String> {
        py.detach(|| {
            let kernel = self.shared.lock()?;
            Ok::<_, BindError>(arris::io::native::to_json(&kernel))
        })?
        .map_err(|e| native_error(&e).raise(py, None))
    }

    /// A new model read from `to_native`'s bytes. It is a model of its own:
    /// no handle of the one that wrote them belongs to it.
    ///
    /// Raises `NativeVersionError` for another version and
    /// `NativeDecodeError` for data that is not a model.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// data = model.to_native()
    /// assert arris.Model.from_native(data).to_native() == data
    /// ```
    #[staticmethod]
    fn from_native(py: Python<'_>, data: &[u8]) -> PyResult<Model> {
        let kernel = py
            .detach(|| arris::io::native::from_bytes(data))
            .map_err(|e| native_error(&e).raise(py, None))?;
        Ok(Model::wrapping(kernel))
    }

    /// A new model read from `to_native_json`'s text. Errors as for
    /// `from_native`.
    ///
    /// ```python
    /// import arris
    ///
    /// model = arris.Model()
    /// body, record = model.primitive_box((0, 0, 0), (1, 2, 3))
    /// text = model.to_native_json()
    /// assert arris.Model.from_native_json(text).to_native_json() == text
    /// ```
    #[staticmethod]
    fn from_native_json(py: Python<'_>, text: &str) -> PyResult<Model> {
        let kernel = py
            .detach(|| arris::io::native::from_json(text))
            .map_err(|e| native_error(&e).raise(py, None))?;
        Ok(Model::wrapping(kernel))
    }
}

impl Model {
    /// A body write, with `write` the kernel's encoder: the record must be
    /// this model's.
    fn write_body_with<T: Send>(
        &self,
        py: Python<'_>,
        body: &Body,
        provenance: Option<&Provenance>,
        write: impl FnOnce(
            &topo::Model,
            topo::Body,
            &arris::topo::provenance::Provenance,
        ) -> Result<T, arris::io::body::BodyError>
        + Send,
    ) -> PyResult<T> {
        let body = body.resolve(&self.shared)?;
        let empty = arris::topo::provenance::Provenance::default();
        let record = match provenance {
            Some(record) => record.kernel_in(&self.shared)?,
            None => &empty,
        };
        py.detach(|| {
            let kernel = self.shared.lock()?;
            Ok::<_, BindError>(write(&kernel, body, record))
        })?
        .map_err(|e| body_error(&e).raise(py, Some(self)))
    }

    /// A body read, with `read` the kernel's decoder. Its errors name the
    /// writer's ids, so they are raised with no model.
    fn read_body_with(
        &self,
        py: Python<'_>,
        limits: &Limits,
        read: impl FnOnce(
            &mut topo::Model,
            &Control<'_>,
        ) -> Result<arris::io::body::Imported, arris::io::body::BodyError>
        + Send,
    ) -> PyResult<Imported> {
        let imported = self.run_with(py, limits, read, body_error, false, |error| {
            matches!(
                error,
                arris::io::body::BodyError::Interrupted(Interrupted { by: Stop::Poll, .. })
            )
        })?;
        Ok(Imported::minted_by(self, imported))
    }
}
