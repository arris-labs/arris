//! Reflection in a plane: the one improper isometry a mirror needs.

use core::fmt;

use crate::frame::{is_finite3, rescaled};
use crate::{Frame, Point3, UnitVec3, Vec3};

/// Why a point and a direction are not a mirror plane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReflectionError {
    /// A coordinate is NaN or infinite.
    NonFinite,
    /// The normal has zero length: it names no plane.
    Degenerate,
}

impl fmt::Display for ReflectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            ReflectionError::NonFinite => "mirror plane has a non-finite coordinate",
            ReflectionError::Degenerate => "mirror plane has a zero normal",
        })
    }
}

impl std::error::Error for ReflectionError {}

/// Reflection in the plane through `origin` with unit `normal`:
/// `p ↦ p − 2((p − o)·n) n`. Lengths and angles are preserved and
/// handedness is reversed, which is why it is a type of its own and not an
/// [`crate::Isometry`], whose every value is proper (ADR-0031).
///
/// ```
/// use arris_math::{Point3, Reflection, Vec3};
///
/// let r = Reflection::new(Point3::new(0.0, 0.0, 1.0), Vec3::z()).unwrap();
/// assert_eq!(r.apply(Point3::new(1.0, 2.0, 3.0)), Point3::new(1.0, 2.0, -1.0));
/// // Reflecting twice returns the point.
/// let p = Point3::new(0.3, -0.4, 5.0);
/// assert!((r.apply(r.apply(p)) - p).norm() < 1e-15);
/// assert!(Reflection::new(Point3::origin(), Vec3::zeros()).is_err());
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reflection {
    origin: Point3,
    normal: UnitVec3,
}

impl Reflection {
    /// The plane through `origin` with normal along `normal` (normalised).
    /// Errors: a non-finite input, or a zero `normal`.
    pub fn new(origin: Point3, normal: Vec3) -> Result<Self, ReflectionError> {
        if !(is_finite3(&origin.coords) && is_finite3(&normal)) {
            return Err(ReflectionError::NonFinite);
        }
        let normal = rescaled(normal)
            .and_then(|n| UnitVec3::try_new(n, 0.0))
            .ok_or(ReflectionError::Degenerate)?;
        Ok(Reflection { origin, normal })
    }

    /// The same as [`Reflection::new`]: the plane is named by a point on
    /// it and its normal.
    pub fn plane_through(origin: Point3, normal: Vec3) -> Result<Self, ReflectionError> {
        Self::new(origin, normal)
    }

    /// A point on the plane.
    pub fn origin(&self) -> Point3 {
        self.origin
    }

    /// The plane's unit normal.
    pub fn normal(&self) -> UnitVec3 {
        self.normal
    }

    /// The mirror image of `p`.
    pub fn apply(&self, p: Point3) -> Point3 {
        let n = self.normal.into_inner();
        p - 2.0 * (p - self.origin).dot(&n) * n
    }

    /// The image of a displacement: the plane's offset does not act on it.
    pub fn apply_vec(&self, v: Vec3) -> Vec3 {
        let n = self.normal.into_inner();
        v - 2.0 * v.dot(&n) * n
    }

    /// The image of a direction, re-normalised so it is unit to rounding.
    pub fn apply_unit(&self, u: UnitVec3) -> UnitVec3 {
        UnitVec3::new_normalize(self.apply_vec(u.into_inner()))
    }

    /// The image of a frame as a *quadric* is placed after a mirror:
    /// `O′ = R O`, `X′ = R X`, `Y′ = −R Y`, `Z′ = R Z`. Right-handed by
    /// construction (`X′ × Y′ = R Z`); the surface it places is the
    /// mirror image reparametrised by `u ↦ 2π − u` (ADR-0031 §2).
    ///
    /// ```
    /// use arris_math::{Frame, Reflection, Vec3, Point3};
    ///
    /// let r = Reflection::new(Point3::origin(), Vec3::x()).unwrap();
    /// let f = r.apply_frame(&Frame::world());
    /// assert!((f.x().into_inner() - (-Vec3::x())).norm() < 1e-15);
    /// assert!((f.y().into_inner() - (-Vec3::y())).norm() < 1e-15);
    /// assert!((f.z().into_inner() - Vec3::z()).norm() < 1e-15);
    /// ```
    pub fn apply_frame(&self, frame: &Frame) -> Frame {
        self.placed(frame, self.apply_vec(frame.z().into_inner()))
    }

    /// The image of a frame keeping the parametrisation of what it
    /// places: `O′ = R O`, `X′ = R X`, `Y′ = R Y`, `Z′ = X′ × Y′ = −R Z`.
    /// A plane's and a conic curve's frame after a mirror (ADR-0031 §2):
    /// the same parameters, the normal turned the other way.
    pub fn apply_frame_reversed(&self, frame: &Frame) -> Frame {
        self.placed(frame, -self.apply_vec(frame.z().into_inner()))
    }

    /// A right-handed frame at the image of `frame`'s origin with axis
    /// `z` and `x` the image of `frame`'s. The images of an orthonormal
    /// frame's axes are orthonormal, so the construction only
    /// re-orthonormalises to rounding.
    fn placed(&self, frame: &Frame, z: Vec3) -> Frame {
        Frame::orthonormalised(
            self.apply(frame.origin()),
            self.apply_unit(frame.x()),
            UnitVec3::new_normalize(z),
        )
    }
}
