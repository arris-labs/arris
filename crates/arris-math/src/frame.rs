//! Right-handed 3D frames, and 2D frames of either handedness.

use core::fmt;

use nalgebra::UnitQuaternion;

use crate::{Isometry, Point2, Point3, UnitVec2, UnitVec3, Vec2, Vec3};

/// Why an origin and some directions are not a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// A coordinate is NaN or infinite.
    NonFinite,
    /// The axis has zero length.
    ZeroAxis,
    /// The `x` hint has no component perpendicular to the axis beyond
    /// rounding: it is zero, or parallel to `z` to rounding.
    DegenerateHint,
    /// Axes given as a frame are not unit and mutually perpendicular to
    /// rounding, or `z ≠ x × y` for a 3D frame.
    NotOrthonormal,
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            FrameError::NonFinite => "frame has a non-finite coordinate",
            FrameError::ZeroAxis => "frame axis has zero length",
            FrameError::DegenerateHint => "frame x hint is zero or parallel to the axis",
            FrameError::NotOrthonormal => "frame axes are not orthonormal",
        })
    }
}

impl std::error::Error for FrameError {}

/// A right-handed orthonormal frame: an origin and axes `x`, `y`, `z` with
/// `x × y = z`, each unit to within rounding, whatever the constructor was
/// given. Every analytic surface and curve is placed by one, so a
/// transform is a frame change and nothing else (`docs/DATA-MODEL.md`
/// §Conventions).
///
/// Only the validating constructors build one; there is no way to hold a
/// `Frame` whose axes are not orthonormal.
///
/// ```
/// use arris_math::{Frame, Point3, Vec3};
///
/// let f = Frame::new(Point3::new(1.0, 2.0, 3.0), Vec3::z(), Vec3::new(1.0, 1.0, 0.0)).unwrap();
/// let local = Point3::new(1.0, 0.0, 0.0);
/// let world = f.to_world(local);
/// assert!((f.to_local(world) - local).norm() < 1e-15);
/// assert!((f.x().dot(&f.y())).abs() < 1e-15);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(try_from = "FrameRepr", into = "FrameRepr"))]
pub struct Frame {
    origin: Point3,
    x: UnitVec3,
    y: UnitVec3,
    z: UnitVec3,
}

/// The wire form of a [`Frame`]: its four fields as given, validated by
/// [`Frame::from_orthonormal`] on the way in so a stored frame is never
/// less of a frame than a built one.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct FrameRepr {
    origin: Point3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
}

#[cfg(feature = "serde")]
impl From<Frame> for FrameRepr {
    fn from(f: Frame) -> Self {
        FrameRepr {
            origin: f.origin,
            x: f.x.into_inner(),
            y: f.y.into_inner(),
            z: f.z.into_inner(),
        }
    }
}

#[cfg(feature = "serde")]
impl TryFrom<FrameRepr> for Frame {
    type Error = FrameError;

    fn try_from(r: FrameRepr) -> Result<Self, FrameError> {
        Frame::from_orthonormal(r.origin, r.x, r.y, r.z)
    }
}

impl Frame {
    /// The world frame: origin at zero, axes the coordinate axes.
    pub fn world() -> Self {
        Frame {
            origin: Point3::origin(),
            x: Vec3::x_axis(),
            y: Vec3::y_axis(),
            z: Vec3::z_axis(),
        }
    }

    /// A frame with axis `z` (normalised) and `x` the direction of
    /// `x_hint`'s component perpendicular to `z`; `y = z × x`.
    ///
    /// Errors: a non-finite input, a zero `z`, or a hint with no
    /// perpendicular component beyond rounding
    /// ([`crate::RELATIVE_ROUNDING`] of the hint's length): what is left
    /// of a hint along `z` is noise whose direction means nothing, and
    /// may lie along `z` itself. A hint that is *nearly* parallel to `z`,
    /// past rounding, still yields an orthonormal frame, but its `x` is
    /// only as good as the few bits of the perpendicular component; a
    /// caller who wants to reject that compares the hint against `z` with
    /// its own `Tolerance` first.
    pub fn new(origin: Point3, z: Vec3, x_hint: Vec3) -> Result<Self, FrameError> {
        if !(is_finite3(&origin.coords) && is_finite3(&z) && is_finite3(&x_hint)) {
            return Err(FrameError::NonFinite);
        }
        let z = rescaled(z)
            .and_then(|z| UnitVec3::try_new(z, 0.0))
            .ok_or(FrameError::ZeroAxis)?;
        let x_hint = rescaled(x_hint).ok_or(FrameError::DegenerateHint)?;
        let perpendicular = x_hint - z.dot(&x_hint) * z.into_inner();
        if crate::is_negligible(perpendicular.norm(), x_hint.norm()) {
            return Err(FrameError::DegenerateHint);
        }
        let x = UnitVec3::try_new(perpendicular, 0.0).ok_or(FrameError::DegenerateHint)?;
        Ok(Self::orthonormalised(origin, x, z))
    }

    /// A frame with axis `z` and `x` chosen by the rule Open CASCADE's
    /// `gp_Ax3(P, N)` uses (read in the reference tree's `gp` package,
    /// reimplemented): zero the axis coordinate of smallest magnitude, swap
    /// the other two with the sign that keeps the larger one, so a
    /// cylinder built from an axis alone seams where the oracle's does.
    /// For `z` along a coordinate axis: `+z ↦ x = +x`, `+x ↦ x = +z`,
    /// `+y ↦ x = +z`.
    ///
    /// Errors: a non-finite input or a zero `z`.
    pub fn from_z(origin: Point3, z: Vec3) -> Result<Self, FrameError> {
        if !(is_finite3(&origin.coords) && is_finite3(&z)) {
            return Err(FrameError::NonFinite);
        }
        let z = rescaled(z)
            .and_then(|z| UnitVec3::try_new(z, 0.0))
            .ok_or(FrameError::ZeroAxis)?;
        let (a, b, c) = (z.x, z.y, z.z);
        let (aa, ba, ca) = (a.abs(), b.abs(), c.abs());
        let hint = if ba <= aa && ba <= ca {
            if aa > ca {
                Vec3::new(-c, 0.0, a)
            } else {
                Vec3::new(c, 0.0, -a)
            }
        } else if aa <= ba && aa <= ca {
            if ba > ca {
                Vec3::new(0.0, -c, b)
            } else {
                Vec3::new(0.0, c, -b)
            }
        } else if aa > ba {
            Vec3::new(-b, a, 0.0)
        } else {
            Vec3::new(b, -a, 0.0)
        };
        // `hint` is perpendicular to `z` by construction and has the norm of
        // the two larger coordinates, so it is never zero for a unit `z`.
        let x = UnitVec3::try_new(hint, 0.0).ok_or(FrameError::ZeroAxis)?;
        Ok(Self::orthonormalised(origin, x, z))
    }

    /// A frame from axes that already are one: each unit, mutually
    /// perpendicular and `z = x × y`, all to rounding
    /// ([`crate::RELATIVE_ROUNDING`]), stored bit for bit — what the
    /// native format reads a frame back through, so a round trip changes
    /// nothing. Errors: a non-finite input, or
    /// [`FrameError::NotOrthonormal`].
    ///
    /// ```
    /// use arris_math::{Frame, FrameError, Point3, Vec3};
    ///
    /// let f = Frame::from_orthonormal(Point3::origin(), Vec3::y(), Vec3::z(), Vec3::x()).unwrap();
    /// assert_eq!(f.x().into_inner(), Vec3::y());
    /// let bad = Frame::from_orthonormal(Point3::origin(), Vec3::x(), Vec3::x(), Vec3::z());
    /// assert_eq!(bad, Err(FrameError::NotOrthonormal));
    /// ```
    pub fn from_orthonormal(origin: Point3, x: Vec3, y: Vec3, z: Vec3) -> Result<Self, FrameError> {
        if !(is_finite3(&origin.coords) && is_finite3(&x) && is_finite3(&y) && is_finite3(&z)) {
            return Err(FrameError::NonFinite);
        }
        let unit = |v: &Vec3| crate::is_negligible(v.norm() - 1.0, 1.0);
        let perpendicular = |a: &Vec3, b: &Vec3| crate::is_negligible(a.dot(b), 1.0);
        if !(unit(&x) && unit(&y) && unit(&z))
            || !(perpendicular(&x, &y) && perpendicular(&y, &z) && perpendicular(&z, &x))
            || !crate::is_negligible((x.cross(&y) - z).norm(), 1.0)
        {
            return Err(FrameError::NotOrthonormal);
        }
        Ok(Frame {
            origin,
            x: UnitVec3::new_unchecked(x),
            y: UnitVec3::new_unchecked(y),
            z: UnitVec3::new_unchecked(z),
        })
    }

    /// The frame whose axes are the images of the coordinate axes under
    /// `rotation`. Infallible: a unit quaternion's basis is orthonormal.
    pub fn from_rotation(origin: Point3, rotation: &UnitQuaternion<f64>) -> Self {
        let x = UnitVec3::new_normalize(rotation.transform_vector(&Vec3::x()));
        let z = UnitVec3::new_normalize(rotation.transform_vector(&Vec3::z()));
        Self::orthonormalised(origin, x, z)
    }

    /// Rebuilds `y` and `x` from `z` and an `x` that is unit and nearly
    /// perpendicular to it, so the result is orthonormal to rounding
    /// regardless of how good the input was.
    pub(crate) fn orthonormalised(origin: Point3, x: UnitVec3, z: UnitVec3) -> Self {
        let y = UnitVec3::new_normalize(z.cross(&x));
        let x = UnitVec3::new_normalize(y.cross(&z));
        Frame { origin, x, y, z }
    }

    /// The same axes at another origin, bit for bit: a translation that
    /// leaves the orientation untouched, where `transformed` by a pure
    /// translation would re-round the axes through the identity rotation.
    pub const fn with_origin(&self, origin: Point3) -> Frame {
        Frame {
            origin,
            x: self.x,
            y: self.y,
            z: self.z,
        }
    }

    /// The origin.
    pub const fn origin(&self) -> Point3 {
        self.origin
    }

    /// The `x` axis.
    pub const fn x(&self) -> UnitVec3 {
        self.x
    }

    /// The `y` axis, `z × x`.
    pub const fn y(&self) -> UnitVec3 {
        self.y
    }

    /// The `z` axis, `x × y`.
    pub const fn z(&self) -> UnitVec3 {
        self.z
    }

    /// The coordinates of a world point in this frame.
    pub fn to_local(&self, p: Point3) -> Point3 {
        Point3::from(self.vec_to_local(p - self.origin))
    }

    /// The world point at local coordinates `p`.
    pub fn to_world(&self, p: Point3) -> Point3 {
        self.origin + self.vec_to_world(p.coords)
    }

    /// The components of a world vector along the axes.
    pub fn vec_to_local(&self, v: Vec3) -> Vec3 {
        Vec3::new(v.dot(&self.x), v.dot(&self.y), v.dot(&self.z))
    }

    /// The world vector with local components `v`.
    pub fn vec_to_world(&self, v: Vec3) -> Vec3 {
        v.x * self.x.into_inner() + v.y * self.y.into_inner() + v.z * self.z.into_inner()
    }

    /// The rotation taking the coordinate axes onto this frame's axes.
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_basis_unchecked(&[
            self.x.into_inner(),
            self.y.into_inner(),
            self.z.into_inner(),
        ])
    }

    /// The rigid motion taking local coordinates to world coordinates:
    /// `as_isometry().apply(p) == to_world(p)` to rounding.
    pub fn as_isometry(&self) -> Isometry {
        Isometry::new(self.rotation(), self.origin.coords)
    }

    /// This frame moved by `motion`. Moving geometry is moving its frame,
    /// and this is that.
    pub fn transformed(&self, motion: &Isometry) -> Frame {
        Self::orthonormalised(
            motion.apply(self.origin),
            motion.apply_unit(self.x),
            motion.apply_unit(self.z),
        )
    }
}

pub(crate) fn is_finite3(v: &Vec3) -> bool {
    v.iter().all(|c| c.is_finite())
}

/// `v` scaled by a power of two that brings its largest coordinate into
/// `[1, 2)`: the same direction, so the squares its normalisation sums
/// neither underflow nor overflow. A vector whose coordinates are near
/// `1e-154` has squares in the subnormal range, and normalised directly
/// it comes out a few parts in ten thousand off unit length. A power of
/// two scales exactly, so a vector already in range normalises to the
/// same bits as without it. `None` for the zero vector.
pub(crate) fn rescaled(v: Vec3) -> Option<Vec3> {
    let largest = v.amax();
    if largest <= 0.0 {
        return None;
    }
    // In [-1023, 1074]: applied in two halves, since `2^1074` is not an
    // `f64` and neither half overflows or underflows on the way.
    let k = -(largest.log2().floor() as i32);
    Some(v * 2f64.powi(k / 2) * 2f64.powi(k - k / 2))
}

/// Which way a [`Frame2`]'s `y` turns from its `x`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Handedness {
    /// `y` is `x` rotated by +90°: `(u, v)` axes in their natural order.
    Right,
    /// `y` is `x` rotated by −90°: a reflection of the right-handed frame.
    Left,
}

/// An orthonormal frame in a surface's (u, v) plane, of either handedness.
/// A pcurve placed by a left-handed `Frame2` is traversed clockwise in
/// (u, v) — the case of a circle shared by a cap and a wall whose normal
/// opposes the circle's `Z` (`docs/DATA-MODEL.md` §Pcurves).
///
/// ```
/// use arris_math::{Frame2, Handedness, Point2, Vec2};
///
/// let f = Frame2::new(Point2::new(1.0, 1.0), Vec2::new(0.0, 2.0), Handedness::Left).unwrap();
/// assert!(!f.is_right_handed());
/// assert_eq!(f.y().into_inner(), Vec2::new(1.0, 0.0));
/// assert_eq!(f.to_world(Point2::new(1.0, 1.0)), Point2::new(2.0, 2.0));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(try_from = "Frame2Repr", into = "Frame2Repr"))]
pub struct Frame2 {
    origin: Point2,
    x: UnitVec2,
    y: UnitVec2,
}

/// The wire form of a [`Frame2`], validated by [`Frame2::from_orthonormal`]
/// on the way in.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct Frame2Repr {
    origin: Point2,
    x: Vec2,
    y: Vec2,
}

#[cfg(feature = "serde")]
impl From<Frame2> for Frame2Repr {
    fn from(f: Frame2) -> Self {
        Frame2Repr {
            origin: f.origin,
            x: f.x.into_inner(),
            y: f.y.into_inner(),
        }
    }
}

#[cfg(feature = "serde")]
impl TryFrom<Frame2Repr> for Frame2 {
    type Error = FrameError;

    fn try_from(r: Frame2Repr) -> Result<Self, FrameError> {
        Frame2::from_orthonormal(r.origin, r.x, r.y)
    }
}

impl Frame2 {
    /// Origin at zero, `x` along `u`, `y` along `v`: right-handed.
    pub fn identity() -> Self {
        Frame2 {
            origin: Point2::origin(),
            x: Vec2::x_axis(),
            y: Vec2::y_axis(),
        }
    }

    /// A frame with `x` along `x` (normalised) and `y` perpendicular to it
    /// on the side `handedness` says. Errors: a non-finite input or a zero
    /// `x`.
    pub fn new(origin: Point2, x: Vec2, handedness: Handedness) -> Result<Self, FrameError> {
        if !(origin.coords.iter().all(|c| c.is_finite()) && x.iter().all(|c| c.is_finite())) {
            return Err(FrameError::NonFinite);
        }
        let x = UnitVec2::try_new(x, 0.0).ok_or(FrameError::ZeroAxis)?;
        let y = match handedness {
            Handedness::Right => Vec2::new(-x.y, x.x),
            Handedness::Left => Vec2::new(x.y, -x.x),
        };
        Ok(Frame2 {
            origin,
            x,
            y: UnitVec2::new_unchecked(y),
        })
    }

    /// A frame from axes that already are one — each unit and
    /// perpendicular to rounding ([`crate::RELATIVE_ROUNDING`]), of either
    /// handedness — stored bit for bit; what the native format reads a
    /// `Frame2` back through. Errors: a non-finite input, or
    /// [`FrameError::NotOrthonormal`].
    pub fn from_orthonormal(origin: Point2, x: Vec2, y: Vec2) -> Result<Self, FrameError> {
        let finite = |v: &Vec2| v.iter().all(|c| c.is_finite());
        if !(finite(&origin.coords) && finite(&x) && finite(&y)) {
            return Err(FrameError::NonFinite);
        }
        let unit = |v: &Vec2| crate::is_negligible(v.norm() - 1.0, 1.0);
        if !(unit(&x) && unit(&y)) || !crate::is_negligible(x.dot(&y), 1.0) {
            return Err(FrameError::NotOrthonormal);
        }
        Ok(Frame2 {
            origin,
            x: UnitVec2::new_unchecked(x),
            y: UnitVec2::new_unchecked(y),
        })
    }

    /// The origin.
    pub const fn origin(&self) -> Point2 {
        self.origin
    }

    /// The same axes at the origin moved by `by`.
    ///
    /// ```
    /// use arris_math::{Frame2, Point2, Vec2};
    ///
    /// let f = Frame2::identity().translated(Vec2::new(1.0, 2.0));
    /// assert_eq!(f.origin(), Point2::new(1.0, 2.0));
    /// assert_eq!(f.x(), Frame2::identity().x());
    /// ```
    pub fn translated(&self, by: Vec2) -> Frame2 {
        Frame2 {
            origin: self.origin + by,
            x: self.x,
            y: self.y,
        }
    }

    /// The image under `(u, v) ↦ (turn − u, v)`: the origin reflected
    /// about `turn / 2` in `u` and each axis's `u` component negated. The
    /// reflection reverses handedness, and an orthonormal pair stays one
    /// exactly (negating a coordinate is exact).
    ///
    /// ```
    /// use arris_math::{Frame2, Point2};
    ///
    /// let f = Frame2::identity().reflected_u(6.0);
    /// assert_eq!(f.origin(), Point2::new(6.0, 0.0));
    /// assert!(!f.is_right_handed());
    /// ```
    pub fn reflected_u(&self, turn: f64) -> Frame2 {
        Frame2 {
            origin: Point2::new(turn - self.origin.x, self.origin.y),
            x: UnitVec2::new_unchecked(Vec2::new(-self.x.x, self.x.y)),
            y: UnitVec2::new_unchecked(Vec2::new(-self.y.x, self.y.y)),
        }
    }

    /// The `x` axis.
    pub const fn x(&self) -> UnitVec2 {
        self.x
    }

    /// The `y` axis: `x` rotated by ±90° according to the handedness.
    pub const fn y(&self) -> UnitVec2 {
        self.y
    }

    /// Which way `y` turns from `x`.
    pub fn handedness(&self) -> Handedness {
        if self.is_right_handed() {
            Handedness::Right
        } else {
            Handedness::Left
        }
    }

    /// `true` when `x × y > 0`, the (u, v) plane's own orientation.
    pub fn is_right_handed(&self) -> bool {
        self.x.perp(&self.y) > 0.0
    }

    /// The coordinates of a (u, v) point in this frame.
    pub fn to_local(&self, p: Point2) -> Point2 {
        Point2::from(self.vec_to_local(p - self.origin))
    }

    /// The (u, v) point at local coordinates `p`.
    pub fn to_world(&self, p: Point2) -> Point2 {
        self.origin + self.vec_to_world(p.coords)
    }

    /// The components of a (u, v) vector along the axes.
    pub fn vec_to_local(&self, v: Vec2) -> Vec2 {
        Vec2::new(v.dot(&self.x), v.dot(&self.y))
    }

    /// The (u, v) vector with local components `v`.
    pub fn vec_to_world(&self, v: Vec2) -> Vec2 {
        v.x * self.x.into_inner() + v.y * self.y.into_inner()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Found by the `intersect_surfaces` fuzz target (`fuzz/`,
    /// ADR-0024 §5): a hint along `z` up to rounding leaves a residue
    /// that can itself lie along `z`, whose cross product with it is zero
    /// and normalises to NaN axes.
    #[test]
    fn a_hint_along_the_axis_to_rounding_is_degenerate_not_nan() {
        let mut nan = Vec::new();
        for k in 1..=2000 {
            let scale = 0.001 * f64::from(k);
            for z in [
                Vec3::new(-1.0, -1.0, -1.0),
                Vec3::new(1.0, 2.0, 3.0),
                Vec3::new(0.3, -0.7, 0.1),
            ] {
                let hint = z * scale;
                match Frame::new(Point3::origin(), z, hint) {
                    Err(FrameError::DegenerateHint) => {}
                    Ok(f) => nan.push((z, scale, f)),
                    Err(e) => panic!("{z:?} × {scale}: {e}"),
                }
            }
        }
        assert!(
            nan.is_empty(),
            "{} frames built: {:?}",
            nan.len(),
            nan.first()
        );
    }

    /// Found by the `intersect_surfaces` fuzz target beside the one
    /// above: an axis whose coordinates are near `1e-154` has squares in
    /// the subnormal range, and normalised as given it came out 8e-4 off
    /// unit length — a cylinder about it was a different cylinder.
    #[test]
    fn a_tiny_axis_or_hint_still_makes_an_orthonormal_frame() {
        for scale in [1e-160, 1e-155, 1e-154, 1e-150, 1.0, 1e150, 1e154] {
            let z = Vec3::new(-0.7, 3e-300, -0.7) * scale;
            let hint = Vec3::new(0.3, -1.0, 0.2) * scale;
            for f in [
                Frame::new(Point3::origin(), z, hint).unwrap(),
                Frame::from_z(Point3::origin(), z).unwrap(),
            ] {
                for axis in [f.x(), f.y(), f.z()] {
                    assert!(
                        (axis.norm() - 1.0).abs() <= crate::RELATIVE_ROUNDING,
                        "{scale:e}: {axis:?}"
                    );
                }
                assert!(f.x().dot(&f.z()).abs() <= crate::RELATIVE_ROUNDING);
            }
        }
    }

    #[test]
    fn world_frame_is_the_identity() {
        let w = Frame::world();
        let p = Point3::new(1.0, -2.0, 3.0);
        assert_eq!(w.to_local(p), p);
        assert_eq!(w.to_world(p), p);
        assert_eq!(w.rotation(), UnitQuaternion::identity());
    }

    #[test]
    fn from_z_follows_the_axis_rule() {
        let f = Frame::from_z(Point3::origin(), Vec3::z()).unwrap();
        assert_eq!(f.x().into_inner(), Vec3::x());
        assert_eq!(f.y().into_inner(), Vec3::y());
        let f = Frame::from_z(Point3::origin(), Vec3::x()).unwrap();
        assert_eq!(f.x().into_inner(), Vec3::z());
        assert_eq!(f.y().into_inner(), -Vec3::y());
        let f = Frame::from_z(Point3::origin(), Vec3::y()).unwrap();
        assert_eq!(f.x().into_inner(), Vec3::z());
        assert_eq!(f.y().into_inner(), Vec3::x());
    }

    #[test]
    fn errors_name_the_problem() {
        let o = Point3::origin();
        assert_eq!(
            Frame::new(o, Vec3::zeros(), Vec3::x()),
            Err(FrameError::ZeroAxis)
        );
        assert_eq!(
            Frame::new(o, Vec3::z(), Vec3::z() * 2.0),
            Err(FrameError::DegenerateHint)
        );
        assert_eq!(
            Frame::new(o, Vec3::z(), Vec3::zeros()),
            Err(FrameError::DegenerateHint)
        );
        assert_eq!(
            Frame::new(o, Vec3::new(f64::NAN, 0.0, 1.0), Vec3::x()),
            Err(FrameError::NonFinite)
        );
        assert_eq!(Frame::from_z(o, Vec3::zeros()), Err(FrameError::ZeroAxis));
        assert_eq!(
            Frame2::new(Point2::origin(), Vec2::zeros(), Handedness::Right),
            Err(FrameError::ZeroAxis)
        );
    }

    #[test]
    fn frame2_handedness_round_trips() {
        for h in [Handedness::Right, Handedness::Left] {
            let f = Frame2::new(Point2::new(0.5, -0.5), Vec2::new(3.0, 4.0), h).unwrap();
            assert_eq!(f.handedness(), h);
            let p = Point2::new(0.3, 0.9);
            assert!((f.to_local(f.to_world(p)) - p).norm() < 1e-15);
            assert!(f.x().dot(&f.y()).abs() < 1e-15);
        }
        assert!(Frame2::identity().is_right_handed());
    }
}
