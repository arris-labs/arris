//! Rigid motions.

use nalgebra::{Isometry3, Translation3, UnitQuaternion};

use crate::{Point3, UnitVec3, Vec3};

/// A rigid motion: a rotation followed by a translation, `p ↦ R p + t`.
/// Lengths and angles are preserved, so a `Frame` moved by one is still a
/// frame and geometry moved by one is the same geometry in another place
/// (`docs/DATA-MODEL.md` §Conventions: a transform is a frame change).
///
/// ```
/// use arris_math::{Isometry, Point3, Vec3, nalgebra::UnitQuaternion};
/// use core::f64::consts::FRAC_PI_2;
///
/// let quarter_turn = UnitQuaternion::from_axis_angle(&Vec3::z_axis(), FRAC_PI_2);
/// let m = Isometry::new(quarter_turn, Vec3::new(1.0, 0.0, 0.0));
/// let p = m.apply(Point3::new(1.0, 0.0, 0.0));
/// assert!((p - Point3::new(1.0, 1.0, 0.0)).norm() < 1e-15);
/// let back = m.inverse().apply(p);
/// assert!((back - Point3::new(1.0, 0.0, 0.0)).norm() < 1e-15);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Isometry {
    inner: Isometry3<f64>,
}

impl Isometry {
    /// `p ↦ rotation · p + translation`.
    pub fn new(rotation: UnitQuaternion<f64>, translation: Vec3) -> Self {
        Isometry {
            inner: Isometry3::from_parts(Translation3::from(translation), rotation),
        }
    }

    /// The motion that moves nothing.
    pub fn identity() -> Self {
        Isometry {
            inner: Isometry3::identity(),
        }
    }

    /// A pure translation by `v`.
    pub fn from_translation(v: Vec3) -> Self {
        Self::new(UnitQuaternion::identity(), v)
    }

    /// A pure rotation about the origin.
    pub fn from_rotation(rotation: UnitQuaternion<f64>) -> Self {
        Self::new(rotation, Vec3::zeros())
    }

    /// The rotation part.
    pub fn rotation(&self) -> UnitQuaternion<f64> {
        self.inner.rotation
    }

    /// The translation part: where the origin goes.
    pub fn translation(&self) -> Vec3 {
        self.inner.translation.vector
    }

    /// `R p + t`.
    pub fn apply(&self, p: Point3) -> Point3 {
        self.inner.transform_point(&p)
    }

    /// `R v`: a vector is a difference of points, so the translation does
    /// not act on it.
    pub fn apply_vec(&self, v: Vec3) -> Vec3 {
        self.inner.transform_vector(&v)
    }

    /// `R u`, re-normalised so the result is unit to rounding.
    pub fn apply_unit(&self, u: UnitVec3) -> UnitVec3 {
        UnitVec3::new_normalize(self.inner.transform_vector(&u))
    }

    /// The motion that is `self` first, then `next`:
    /// `a.then(&b).apply(p) == b.apply(a.apply(p))` to rounding.
    pub fn then(&self, next: &Isometry) -> Isometry {
        Isometry {
            inner: next.inner * self.inner,
        }
    }

    /// The motion that undoes this one: `m.inverse().apply(m.apply(p)) ==
    /// p` to rounding.
    pub fn inverse(&self) -> Isometry {
        Isometry {
            inner: self.inner.inverse(),
        }
    }
}

impl Default for Isometry {
    fn default() -> Self {
        Self::identity()
    }
}
