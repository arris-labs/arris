//! `Path`, `PathSegment` and `FrameLaw`: the spine a profile is swept
//! along by `Model.sweep`.
//!
//! The kernel validates a path when the sweep asks for its frames, naming
//! the segment it refuses (`OpPathError`); building one here checks only
//! the numbers a segment is made of.

use arris::geom::NurbsCurve;
use arris::geom::{FrameLaw as KernelLaw, Path as Kernel, PathSegment as KernelSegment};
use arris::math::{Axis, Point3};
use arris::ops::OpError;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::kernel_error::op_error;
use crate::model::direction_of;

/// One segment of a `Path`, from where the one before ended.
///
/// Frozen. The first segment starts at its `Path`'s `start`.
///
/// ```python
/// import arris
///
/// bend = arris.PathSegment.arc(to=(4, 4, 0), via=(2.8284271247461903, 1.1715728752538097, 0))
/// assert bend == arris.PathSegment.arc((4, 4, 0), (2.8284271247461903, 1.1715728752538097, 0))
/// ```
#[pyclass(frozen, eq, module = "arris")]
#[derive(Clone, Debug, PartialEq)]
pub struct PathSegment {
    kernel: KernelSegment,
}

#[pymethods]
impl PathSegment {
    /// A straight segment to `to`.
    #[staticmethod]
    fn line(to: [f64; 3]) -> PathSegment {
        PathSegment {
            kernel: KernelSegment::LineTo(Point3::from(to)),
        }
    }

    /// A circular arc through `via` to `to`: `via` is a point on the arc
    /// between its ends, which decides the circle and which way round it
    /// goes. The arc may be longer than half a turn.
    #[staticmethod]
    fn arc(to: [f64; 3], via: [f64; 3]) -> PathSegment {
        PathSegment {
            kernel: KernelSegment::ArcTo {
                via: Point3::from(via),
                to: Point3::from(to),
            },
        }
    }

    /// A helix arc from the current point, about the line through `origin`
    /// along `direction`. Its radius is the current point's distance from
    /// that line. It turns by `angle` radians, right-handed about
    /// `direction` when positive, and advances `pitch * angle / 2π` along
    /// it, so the sign of `angle` is the sense of travel.
    ///
    /// Raises `OpDegenerateError` for an axis that is not finite or has no
    /// direction.
    #[staticmethod]
    fn helix(
        py: Python<'_>,
        origin: [f64; 3],
        direction: [f64; 3],
        pitch: f64,
        angle: f64,
    ) -> PyResult<PathSegment> {
        let axis = direction_of("helix axis direction", direction)
            .and_then(|d| Axis::new(Point3::from(origin), d).map_err(OpError::from))
            .map_err(|e| op_error(&e).raise(py, None))?;
        Ok(PathSegment {
            kernel: KernelSegment::Helix { axis, pitch, angle },
        })
    }

    /// A NURBS curve of `degree` through its `knots`, `control_points` and
    /// `weights` (all 1 when omitted). It must start where the segment
    /// before ends.
    ///
    /// Raises `ValueError` for a knot vector, point count or weight that
    /// does not make a curve.
    #[staticmethod]
    #[pyo3(signature = (degree, knots, control_points, weights=None))]
    fn nurbs(
        degree: usize,
        knots: Vec<f64>,
        control_points: Vec<[f64; 3]>,
        weights: Option<Vec<f64>>,
    ) -> PyResult<PathSegment> {
        let weights = weights.unwrap_or_else(|| vec![1.0; control_points.len()]);
        let points = control_points.into_iter().map(Point3::from).collect();
        let curve = NurbsCurve::new(degree, knots, points, weights)
            .map_err(|e| PyValueError::new_err(e.to_string()))?;
        Ok(PathSegment {
            kernel: KernelSegment::Nurbs(curve),
        })
    }

    fn __repr__(&self) -> String {
        format!("PathSegment({:?})", self.kernel)
    }
}

/// A G1 chain of `PathSegment`s starting at `start`: the spine
/// `Model.sweep` carries a profile along.
///
/// Nothing is checked here: the sweep refuses a gap, a corner or a
/// degenerate segment with `OpPathError`, naming the segment.
///
/// ```python
/// import arris
///
/// spine = arris.Path((0, 0, 0), [arris.PathSegment.line((0, 0, 10))])
/// assert len(spine.segments) == 1
/// ```
#[pyclass(frozen, eq, module = "arris")]
#[derive(Clone, Debug, PartialEq)]
pub struct Path {
    pub(crate) kernel: Kernel,
}

#[pymethods]
impl Path {
    /// The path from `start` through `segments`, each from the end of the
    /// one before.
    #[new]
    fn new(start: [f64; 3], segments: Vec<PyRef<'_, PathSegment>>) -> Path {
        Path {
            kernel: Kernel {
                start: Point3::from(start),
                segments: segments.iter().map(|s| s.kernel.clone()).collect(),
            },
        }
    }

    /// Where the path starts.
    #[getter]
    fn start(&self) -> (f64, f64, f64) {
        let p = self.kernel.start;
        (p.x, p.y, p.z)
    }

    /// The segments.
    #[getter]
    fn segments(&self) -> Vec<PathSegment> {
        self.kernel
            .segments
            .iter()
            .map(|kernel| PathSegment {
                kernel: kernel.clone(),
            })
            .collect()
    }

    fn __repr__(&self) -> String {
        format!("Path(segments={})", self.kernel.segments.len())
    }
}

/// How a profile turns along a path.
///
/// Frozen. `FrameLaw.rotation_minimizing()` is the default of
/// `Model.sweep`.
///
/// ```python
/// import arris
///
/// assert arris.FrameLaw.rotation_minimizing() != arris.FrameLaw.fixed_binormal((0, 0, 1))
/// ```
#[pyclass(frozen, eq, module = "arris")]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrameLaw {
    pub(crate) kernel: KernelLaw,
}

#[pymethods]
impl FrameLaw {
    /// The frame turns no more about the tangent than the path's own
    /// bending forces: parallel transport of the normal plane.
    #[staticmethod]
    fn rotation_minimizing() -> FrameLaw {
        FrameLaw {
            kernel: KernelLaw::RotationMinimizing,
        }
    }

    /// The binormal is `direction` projected perpendicular to the tangent:
    /// a thread or a spring keeps its profile in the axis' plane. The sweep
    /// refuses a path whose tangent runs along it (`OpPathError`).
    ///
    /// Raises `OpDegenerateError` for a direction that is not finite or is
    /// zero.
    #[staticmethod]
    fn fixed_binormal(py: Python<'_>, direction: [f64; 3]) -> PyResult<FrameLaw> {
        let direction = direction_of("binormal direction", direction)
            .map_err(|e| op_error(&e).raise(py, None))?;
        Ok(FrameLaw {
            kernel: KernelLaw::FixedBinormal(arris::math::UnitVec3::new_normalize(direction)),
        })
    }

    fn __repr__(&self) -> String {
        format!("FrameLaw({:?})", self.kernel)
    }
}
