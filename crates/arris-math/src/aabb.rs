//! Axis-aligned bounding boxes: the cheap reject a boolean's face pairs
//! and a renderer's camera fit are both over.

use crate::Point3;

/// The axis-aligned box around a set of points: `min ≤ max` on every axis,
/// both finite. A box is never empty; an empty point set has no box.
///
/// ```
/// use arris_math::Aabb;
///
/// let b = Aabb::of_points(&[[0.0, 0.0, 0.0], [2.0, -1.0, 3.0]]).unwrap();
/// assert_eq!(b.min, [0.0, -1.0, 0.0]);
/// assert_eq!(b.max, [2.0, 0.0, 3.0]);
/// assert_eq!(b.extent(), [2.0, 1.0, 3.0]);
/// assert_eq!(b.center(), [1.0, -0.5, 1.5]);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    /// The smallest coordinate on each axis.
    pub min: [f64; 3],
    /// The largest coordinate on each axis.
    pub max: [f64; 3],
}

impl Aabb {
    /// The box around `points`, or `None` for no points.
    pub fn of_points<'a>(points: impl IntoIterator<Item = &'a [f64; 3]>) -> Option<Aabb> {
        let mut it = points.into_iter();
        let first = *it.next()?;
        let mut b = Aabb {
            min: first,
            max: first,
        };
        for p in it {
            b = b.union(Aabb { min: *p, max: *p });
        }
        Some(b)
    }

    /// The smallest box containing both.
    pub fn union(self, other: Aabb) -> Aabb {
        let mut b = self;
        for ((lo, hi), (olo, ohi)) in b
            .min
            .iter_mut()
            .zip(b.max.iter_mut())
            .zip(other.min.iter().zip(other.max.iter()))
        {
            *lo = lo.min(*olo);
            *hi = hi.max(*ohi);
        }
        b
    }

    /// `max - min` per axis.
    pub fn extent(&self) -> [f64; 3] {
        [
            self.max[0] - self.min[0],
            self.max[1] - self.min[1],
            self.max[2] - self.min[2],
        ]
    }

    /// The midpoint.
    pub fn center(&self) -> [f64; 3] {
        [
            0.5 * (self.min[0] + self.max[0]),
            0.5 * (self.min[1] + self.max[1]),
            0.5 * (self.min[2] + self.max[2]),
        ]
    }

    /// The length of the box's diagonal: the scale of what it contains.
    pub fn diagonal(&self) -> f64 {
        let e = self.extent();
        (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]).sqrt()
    }

    /// The box holding one point, of zero extent.
    ///
    /// ```
    /// use arris_math::{Aabb, Point3};
    ///
    /// let b = Aabb::of_point(Point3::new(1.0, 2.0, 3.0));
    /// assert_eq!(b.min, b.max);
    /// assert_eq!(b.extent(), [0.0; 3]);
    /// ```
    pub fn of_point(p: Point3) -> Aabb {
        Aabb {
            min: [p.x, p.y, p.z],
            max: [p.x, p.y, p.z],
        }
    }

    /// `true` when the two boxes share a point, touching included: the
    /// cheap reject before a face pair is intersected. Boxes that only
    /// touch on a face intersect, so a caller that needs a margin
    /// [`Aabb::inflated`] one of them by its tolerance first.
    ///
    /// ```
    /// use arris_math::Aabb;
    ///
    /// let a = Aabb { min: [0.0; 3], max: [1.0; 3] };
    /// let touching = Aabb { min: [1.0, 0.0, 0.0], max: [2.0, 1.0, 1.0] };
    /// let clear = Aabb { min: [1.5, 0.0, 0.0], max: [2.0, 1.0, 1.0] };
    /// assert!(a.intersects(&touching));
    /// assert!(!a.intersects(&clear));
    /// ```
    pub fn intersects(&self, other: &Aabb) -> bool {
        (0..3).all(|i| self.min[i] <= other.max[i] && other.min[i] <= self.max[i])
    }

    /// The distance between the two boxes: the length of the shortest
    /// segment from a point of one to a point of the other, `0.0` when
    /// they overlap or touch. It is a lower bound on the distance between
    /// anything the boxes hold.
    ///
    /// ```
    /// use arris_math::Aabb;
    ///
    /// let a = Aabb { min: [0.0; 3], max: [1.0; 3] };
    /// let b = Aabb { min: [4.0, 5.0, 0.0], max: [5.0, 6.0, 1.0] };
    /// assert_eq!(a.distance(&b), 5.0); // 3 along x, 4 along y
    /// assert_eq!(a.distance(&a), 0.0);
    /// ```
    pub fn distance(&self, other: &Aabb) -> f64 {
        let mut sq = 0.0;
        for i in 0..3 {
            let gap = (self.min[i] - other.max[i])
                .max(other.min[i] - self.max[i])
                .max(0.0);
            sq += gap * gap;
        }
        sq.sqrt()
    }

    /// The box grown by `by` on every side; a negative `by` shrinks it,
    /// and never past a point — the centre holds.
    ///
    /// ```
    /// use arris_math::Aabb;
    ///
    /// let b = Aabb { min: [0.0; 3], max: [2.0; 3] };
    /// assert_eq!(b.inflated(0.5).min, [-0.5; 3]);
    /// assert_eq!(b.inflated(-5.0).extent(), [0.0; 3], "never past a point");
    /// ```
    pub fn inflated(&self, by: f64) -> Aabb {
        let mut out = *self;
        for i in 0..3 {
            let centre = 0.5 * (self.min[i] + self.max[i]);
            out.min[i] = (self.min[i] - by).min(centre);
            out.max[i] = (self.max[i] + by).max(centre);
        }
        out
    }
}
