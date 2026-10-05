//! `Profile`: a sketch built from Python values, swept by `extrude` and
//! `revolve`.
//!
//! The kernel validates a profile when a sweep (or a measure) asks for its
//! edges, naming the loop and segment it refuses; building one here only
//! needs its plane to be a plane.

use arris::geom::profile::{Profile as Kernel, ProfileLoop, ProfileSegment};
use arris::math::{Frame, Point2, Point3, Vec2, Vec3};
use arris::ops::{InputReason, OpError, Reason};
use pyo3::prelude::*;

use crate::kernel_error::op_error;
use crate::model::Model;

/// One segment of a path: where it goes, and how it gets there.
///
/// Frozen. A segment starts where the previous one ended; the first starts
/// at its `Loop.path`'s `start`.
///
/// ```python
/// import arris
///
/// quarter = arris.Segment.arc(to=(1, 1), via=(0.7071067811865476, 0.2928932188134524))
/// assert quarter == arris.Segment.arc((1, 1), (0.7071067811865476, 0.2928932188134524))
/// ```
#[pyclass(frozen, eq, module = "arris")]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    kernel: ProfileSegment,
}

#[pymethods]
impl Segment {
    /// A straight segment to `to`, in the profile's own (u, v).
    #[staticmethod]
    fn line(to: [f64; 2]) -> Segment {
        Segment {
            kernel: ProfileSegment::LineTo(Point2::from(to)),
        }
    }

    /// A circular arc through `via` to `to`: `via` is a point on the arc
    /// between its ends, which decides the centre, the radius and which way
    /// round it goes.
    #[staticmethod]
    fn arc(to: [f64; 2], via: [f64; 2]) -> Segment {
        Segment {
            kernel: ProfileSegment::ArcTo {
                to: Point2::from(to),
                via: Point2::from(via),
            },
        }
    }

    /// An arc of the ellipse of `center`, `major` (from the centre to one
    /// end of the major axis) and `minor_radius`, to `to`, turning
    /// counter-clockwise in (u, v) when `ccw` and clockwise otherwise. Both
    /// ends must lie on the ellipse.
    #[staticmethod]
    fn ellipse_arc(
        to: [f64; 2],
        center: [f64; 2],
        major: [f64; 2],
        minor_radius: f64,
        ccw: bool,
    ) -> Segment {
        Segment {
            kernel: ProfileSegment::EllipseTo {
                to: Point2::from(to),
                center: Point2::from(center),
                major: Vec2::from(major),
                minor_radius,
                ccw,
            },
        }
    }

    /// Where the segment ends.
    #[getter]
    fn end(&self) -> (f64, f64) {
        let p = self.kernel.end();
        (p.x, p.y)
    }

    fn __repr__(&self) -> String {
        format!("Segment({:?})", self.kernel)
    }
}

/// One closed loop of a profile: a full circle, a full ellipse, or a chain
/// of segments that returns to where it started. Written in either
/// orientation; the kernel turns the outer loop counter-clockwise and every
/// hole clockwise.
///
/// ```python
/// import arris
///
/// disc = arris.Loop.circle((0, 0), 2)
/// square = arris.Loop.polygon([(0, 0), (4, 0), (4, 4), (0, 4)])
/// assert disc != square
/// ```
#[pyclass(frozen, eq, module = "arris")]
#[derive(Clone, Debug, PartialEq)]
pub struct Loop {
    kernel: ProfileLoop,
}

#[pymethods]
impl Loop {
    /// A full circle.
    #[staticmethod]
    fn circle(center: [f64; 2], radius: f64) -> Loop {
        Loop {
            kernel: ProfileLoop::Circle {
                center: Point2::from(center),
                radius,
            },
        }
    }

    /// A full ellipse: `major` runs from `center` to one end of the major
    /// axis, and `minor_radius` is the other radius.
    #[staticmethod]
    fn ellipse(center: [f64; 2], major: [f64; 2], minor_radius: f64) -> Loop {
        Loop {
            kernel: ProfileLoop::Ellipse {
                center: Point2::from(center),
                major: Vec2::from(major),
                minor_radius,
            },
        }
    }

    /// A chain of `segments` starting at `start`; the last must end there.
    #[staticmethod]
    fn path(start: [f64; 2], segments: Vec<PyRef<'_, Segment>>) -> Loop {
        Loop {
            kernel: ProfileLoop::Path {
                start: Point2::from(start),
                segments: segments.iter().map(|s| s.kernel).collect(),
            },
        }
    }

    /// The straight-sided loop through `points`, closed back to the first.
    #[staticmethod]
    fn polygon(points: Vec<[f64; 2]>) -> Loop {
        let mut points = points.into_iter().map(Point2::from);
        let start = points.next().unwrap_or_default();
        let mut segments: Vec<ProfileSegment> = points.map(ProfileSegment::LineTo).collect();
        segments.push(ProfileSegment::LineTo(start));
        Loop {
            kernel: ProfileLoop::Path { start, segments },
        }
    }

    fn __repr__(&self) -> String {
        format!("Loop({:?})", self.kernel)
    }
}

/// A sketch: closed loops drawn in a plane, the input of `Model.extrude`
/// and `Model.revolve`.
///
/// `(u, v)` in the loops is `origin + u·x + v·y`, where `x` is `x_axis`'s
/// component perpendicular to `normal` and `y = normal × x`. The outer loop
/// is the profile's boundary and each of `holes` lies inside it and outside
/// the others. Nothing is checked here beyond the plane: the sweep (or
/// `area_and_centroid`) refuses an invalid sketch with `OpProfileError`,
/// naming the loop and segment.
///
/// ```python
/// import arris
///
/// plate = arris.Profile(
///     arris.Loop.polygon([(0, 0), (10, 0), (10, 6), (0, 6)]),
///     holes=[arris.Loop.circle((5, 3), 1)],
/// )
/// area, (u, v) = plate.area_and_centroid(arris.Model())
/// assert abs(area - (60 - 3.141592653589793)) < 1e-9
/// ```
#[pyclass(frozen, eq, module = "arris")]
#[derive(Clone, Debug, PartialEq)]
pub struct Profile {
    pub(crate) kernel: Kernel,
}

/// A frame the plane arguments do not make, as the kernel's refusal.
fn frame_refusal(error: arris::math::FrameError) -> OpError {
    use arris::math::FrameError as F;
    OpError::Degenerate {
        entities: Vec::new(),
        reason: match error {
            F::NonFinite => Reason::Input(InputReason::NonFinite {
                what: "profile plane",
            }),
            F::ZeroAxis => Reason::Input(InputReason::NotPositive {
                what: "profile plane normal",
                value: 0.0,
            }),
            F::DegenerateHint => Reason::Input(InputReason::NotPositive {
                what: "profile x axis component across the normal",
                value: 0.0,
            }),
            F::NotOrthonormal => Reason::Input(InputReason::NotPositive {
                what: "profile plane axes",
                value: 0.0,
            }),
        },
    }
}

#[pymethods]
impl Profile {
    /// A profile of `outer` and `holes` in the plane through `origin` with
    /// the given `normal`.
    ///
    /// Raises `OpDegenerateError` for a plane that is not finite or whose
    /// normal is zero or along `x_axis`.
    #[new]
    #[pyo3(signature = (outer, holes=Vec::new(), *, origin=[0.0; 3], normal=[0.0, 0.0, 1.0], x_axis=[1.0, 0.0, 0.0]))]
    fn new(
        py: Python<'_>,
        outer: &Loop,
        holes: Vec<PyRef<'_, Loop>>,
        origin: [f64; 3],
        normal: [f64; 3],
        x_axis: [f64; 3],
    ) -> PyResult<Profile> {
        let plane = Frame::new(Point3::from(origin), Vec3::from(normal), Vec3::from(x_axis))
            .map_err(|e| op_error(&frame_refusal(e)).raise(py, None))?;
        Ok(Profile {
            kernel: Kernel {
                plane,
                outer: outer.kernel.clone(),
                holes: holes.iter().map(|h| h.kernel.clone()).collect(),
            },
        })
    }

    /// The outer loop.
    #[getter]
    fn outer(&self) -> Loop {
        Loop {
            kernel: self.kernel.outer.clone(),
        }
    }

    /// The holes.
    #[getter]
    fn holes(&self) -> Vec<Loop> {
        self.kernel
            .holes
            .iter()
            .map(|kernel| Loop {
                kernel: kernel.clone(),
            })
            .collect()
    }

    /// The area the profile encloses (holes taken out) and its centroid in
    /// the profile's `(u, v)`, to `model`'s tolerance.
    ///
    /// Raises `OpProfileError` for an invalid sketch.
    fn area_and_centroid(&self, py: Python<'_>, model: &Model) -> PyResult<(f64, (f64, f64))> {
        let tolerance = model.shared.lock()?.precision().tolerance();
        let (area, centroid) = self
            .kernel
            .area_and_centroid(tolerance)
            .map_err(|e| op_error(&OpError::Profile(e)).raise(py, Some(model)))?;
        Ok((area, (centroid.x, centroid.y)))
    }

    fn __repr__(&self) -> String {
        format!(
            "Profile(outer={:?}, holes={})",
            self.kernel.outer,
            self.kernel.holes.len()
        )
    }
}
