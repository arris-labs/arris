//! Rational B-spline surfaces.

use arris_math::nalgebra::Vector4;
use arris_math::{Interval, Isometry, Point3, Reflection, UnitVec3, Vec3, is_negligible};

use super::basis::{self, MAX_DEGREE, ORDERS};
use crate::{GeomError, GeomKind, SurfaceEval, SurfaceKind};

/// A rational B-spline surface (`docs/DATA-MODEL.md` §NURBS): degrees
/// `(p, q)`, knot vectors of `n + p + 1` and `m + q + 1` knots, an `n × m`
/// net of control points with positive weights, valid by construction.
///
/// Guarantees: `eval` never panics or allocates for any finite `(u, v)`;
/// the domain is `[knots_u[p], knots_u[n]] × [knots_v[q], knots_v[m]]`;
/// a direction is periodic exactly when its knots and the control net
/// wrap in it (as for [`crate::NurbsCurve`]), and then `eval` wraps that
/// parameter first; a direction that is *closed* without being periodic —
/// clamped at both ends, its first and last rows one row to rounding —
/// wraps a parameter outside its domain by the domain's length
/// ([`NurbsSurface::closure`]).
///
/// ```
/// use arris_geom::NurbsSurface;
/// use arris_math::Point3;
///
/// // A bilinear patch: the unit square of the xy plane.
/// let patch = NurbsSurface::new(
///     [1, 1],
///     [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
///     vec![
///         Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0),
///         Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0),
///     ],
///     vec![1.0; 4],
/// ).unwrap();
/// let e = patch.eval(0.25, 0.75);
/// assert_eq!(e.point, Point3::new(0.25, 0.75, 0.0));
/// assert_eq!(patch.normal(0.5, 0.5).unwrap().into_inner(), arris_math::Vec3::z());
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(try_from = "NurbsSurfaceRepr", into = "NurbsSurfaceRepr")
)]
pub struct NurbsSurface {
    degree: [usize; 2],
    knots: [Vec<f64>; 2],
    counts: [usize; 2],
    /// Row-major over `u`: the point at `(i, j)` is `points[i * m + j]`.
    points: Vec<Point3>,
    weights: Vec<f64>,
    period: [Option<f64>; 2],
    /// Whether each direction is closed and not periodic.
    closed: [bool; 2],
}

/// The wire form of a [`NurbsSurface`]: what [`NurbsSurface::new`] takes,
/// validated by it on the way in.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct NurbsSurfaceRepr {
    degree: [usize; 2],
    knots: [Vec<f64>; 2],
    control_points: Vec<Point3>,
    weights: Vec<f64>,
}

#[cfg(feature = "serde")]
impl From<NurbsSurface> for NurbsSurfaceRepr {
    fn from(s: NurbsSurface) -> Self {
        NurbsSurfaceRepr {
            degree: s.degree,
            knots: s.knots,
            control_points: s.points,
            weights: s.weights,
        }
    }
}

#[cfg(feature = "serde")]
impl TryFrom<NurbsSurfaceRepr> for NurbsSurface {
    type Error = GeomError;

    fn try_from(r: NurbsSurfaceRepr) -> Result<Self, GeomError> {
        NurbsSurface::new(r.degree, r.knots, r.control_points, r.weights)
    }
}

impl NurbsSurface {
    /// A validated surface from its degrees `[p, q]`, knots `[u, v]`, the
    /// control net in row-major order over `u` (`points[i * m + j]` is
    /// `(i, j)`; `n = knots[0].len() − p − 1`, `m = knots[1].len() − q −
    /// 1`) and its weights. Errors: [`GeomError::Degenerate`] naming the
    /// fault, with the rules of [`crate::NurbsCurve::new`] per direction
    /// and the net's size checked against the knots.
    pub fn new(
        degree: [usize; 2],
        knots: [Vec<f64>; 2],
        control_points: Vec<Point3>,
        weights: Vec<f64>,
    ) -> Result<Self, GeomError> {
        let degenerate = |reason: String| GeomError::Degenerate {
            kind: GeomKind::Surface(SurfaceKind::Nurbs),
            reason,
        };
        let mut counts = [0; 2];
        for dir in 0..2 {
            let name = ["u", "v"][dir];
            if degree[dir] == 0 || degree[dir] > MAX_DEGREE {
                return Err(degenerate(format!(
                    "{name}: degree {} is not in 1..={MAX_DEGREE}",
                    degree[dir]
                )));
            }
            counts[dir] = knots[dir]
                .len()
                .checked_sub(degree[dir] + 1)
                .filter(|&n| n > degree[dir])
                .ok_or_else(|| {
                    degenerate(format!(
                        "{name}: {} knots for degree {} leave fewer than {} control points",
                        knots[dir].len(),
                        degree[dir],
                        degree[dir] + 1
                    ))
                })?;
            basis::validate(degree[dir], &knots[dir], counts[dir])
                .map_err(|reason| degenerate(format!("{name}: {reason}")))?;
        }
        let [n, m] = counts;
        if control_points.len() != n * m {
            return Err(degenerate(format!(
                "{} control points for a {n} × {m} net",
                control_points.len()
            )));
        }
        if weights.len() != control_points.len() {
            return Err(degenerate(format!(
                "{} weights for {} control points",
                weights.len(),
                control_points.len()
            )));
        }
        if let Some(i) = control_points
            .iter()
            .position(|p| !p.coords.iter().all(|c| c.is_finite()))
        {
            return Err(degenerate(format!(
                "control point ({}, {}) is not finite",
                i / m,
                i % m
            )));
        }
        if let Some(i) = weights.iter().position(|w| !(w.is_finite() && *w > 0.0)) {
            return Err(degenerate(format!(
                "weight ({}, {}) is {}: weights are finite and positive",
                i / m,
                i % m,
                weights[i]
            )));
        }
        let scale = control_points
            .iter()
            .map(|p| p.coords.norm())
            .fold(0.0, f64::max);
        let wscale = weights.iter().copied().fold(0.0, f64::max);
        let same = |a: usize, b: usize| {
            is_negligible((control_points[a] - control_points[b]).norm(), scale)
                && is_negligible(weights[a] - weights[b], wscale)
        };
        let period = [
            super::spline::detect_period(degree[0], &knots[0], n, |a, b| {
                (0..m).all(|j| same(a * m + j, b * m + j))
            }),
            super::spline::detect_period(degree[1], &knots[1], m, |a, b| {
                (0..n).all(|i| same(i * m + a, i * m + b))
            }),
        ];
        let clamped = |dir: usize| {
            let (k, p, n) = (&knots[dir], degree[dir], counts[dir]);
            k[..=p].iter().all(|&x| x == k[p]) && k[n..].iter().all(|&x| x == k[n])
        };
        let closed = [
            period[0].is_none() && clamped(0) && (0..m).all(|j| same(j, (n - 1) * m + j)),
            period[1].is_none() && clamped(1) && (0..n).all(|i| same(i * m, i * m + m - 1)),
        ];
        Ok(NurbsSurface {
            degree,
            knots,
            counts,
            points: control_points,
            weights,
            period,
            closed,
        })
    }

    /// `[p, q]`.
    pub fn degree(&self) -> [usize; 2] {
        self.degree
    }

    /// The knots of each direction.
    pub fn knots(&self) -> [&[f64]; 2] {
        [&self.knots[0], &self.knots[1]]
    }

    /// `[n, m]`, the control net's size.
    pub fn counts(&self) -> [usize; 2] {
        self.counts
    }

    /// The control net, row-major over `u`.
    pub fn control_points(&self) -> &[Point3] {
        &self.points
    }

    /// The weights in the same order.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// The control point at `(i, j)`, `None` outside the net.
    pub fn control_point(&self, i: usize, j: usize) -> Option<Point3> {
        (i < self.counts[0] && j < self.counts[1]).then(|| self.points[i * self.counts[1] + j])
    }

    /// `[u, v]` domains.
    pub fn domain(&self) -> [Interval; 2] {
        [self.domain_of(0), self.domain_of(1)]
    }

    fn domain_of(&self, dir: usize) -> Interval {
        let k = &self.knots[dir];
        // Validated non-empty and ordered, so the fallback is unreachable.
        Interval::new(k[self.degree[dir]], k[self.counts[dir]]).unwrap_or(Interval::UNIT)
    }

    /// The period of each direction, `None` where the knots do not wrap.
    pub fn period(&self) -> [Option<f64>; 2] {
        self.period
    }

    /// The length each direction closes over: its period where it is
    /// periodic, the domain's length where it is closed without being
    /// periodic — clamped at both ends, with its first and last rows of
    /// control points and weights the same to rounding, as a full turn of
    /// a revolution is (`docs/DATA-MODEL.md` §NURBS) — and `None` where it
    /// is open. `eval` wraps a parameter by it, so `P(u + c, v) = P(u, v)`
    /// for the closure `c` of `u`, and a pcurve may cross the seam
    /// continuously; it is what `Surface::period` reports for a `Nurbs`.
    ///
    /// ```
    /// use arris_geom::Surface;
    /// use arris_math::{Frame, Interval};
    ///
    /// let wall = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
    /// let twin = wall.to_nurbs([Interval::TURN, Interval::UNIT]).unwrap();
    /// // A full turn is clamped, so closed and not periodic.
    /// assert_eq!(twin.period(), [None, None]);
    /// assert_eq!(twin.closure()[0], Some(Interval::TURN.length()));
    /// let (a, b) = (twin.eval(0.5, 0.5).point, twin.eval(0.5 + Interval::TURN.length(), 0.5).point);
    /// assert!((a - b).norm() < 1e-14);
    /// ```
    pub fn closure(&self) -> [Option<f64>; 2] {
        [0, 1].map(|dir| {
            self.period[dir].or_else(|| self.closed[dir].then(|| self.domain_of(dir).length()))
        })
    }

    fn wrap(&self, dir: usize, t: f64) -> f64 {
        let d = self.domain_of(dir);
        match self.period[dir] {
            Some(period) => {
                let w = d.lo() + (t - d.lo()).rem_euclid(period);
                if w >= d.hi() { d.lo() } else { w }
            }
            // A closed direction keeps its own end: the seam's two
            // parameters are one point, but not one derivative.
            None if self.closed[dir] && !d.contains(t) => {
                let w = d.lo() + (t - d.lo()).rem_euclid(d.length());
                if w >= d.hi() { d.lo() } else { w }
            }
            None => t,
        }
    }

    /// The rows of the net that are one point — a pole, an apex: for each
    /// clamped end of each direction whose boundary row of control points
    /// is one point to rounding, `(k, value, point)` with `k` the
    /// direction whose parameter is fixed at `value` along the row.
    pub(crate) fn collapsed_rows(&self) -> Vec<(usize, f64, Point3)> {
        let [n, m] = self.counts;
        let scale = self
            .points
            .iter()
            .map(|p| p.coords.norm())
            .fold(0.0, f64::max);
        let mut out = Vec::new();
        for k in 0..2 {
            let (knots, p, count) = (&self.knots[k], self.degree[k], self.counts[k]);
            let d = self.domain_of(k);
            let ends = [
                (knots[..=p].iter().all(|&x| x == knots[p]), 0, d.lo()),
                (
                    knots[count..].iter().all(|&x| x == knots[count]),
                    count - 1,
                    d.hi(),
                ),
            ];
            for (clamped, row, value) in ends {
                if !clamped {
                    continue;
                }
                let at = |i: usize| {
                    if k == 0 {
                        self.points[row * m + i]
                    } else {
                        self.points[i * m + row]
                    }
                };
                let len = if k == 0 { m } else { n };
                let first = at(0);
                if (1..len).all(|i| is_negligible((at(i) - first).norm(), scale)) {
                    let other = self.domain_of(1 - k).midpoint();
                    let point = if k == 0 {
                        self.eval(value, other).point
                    } else {
                        self.eval(other, value).point
                    };
                    out.push((k, value, point));
                }
            }
        }
        out
    }

    /// The point and its derivatives to second order at `(u, v)`: the
    /// tensor-product de Boor sums on the homogeneous net, then the
    /// quotient rule (*The NURBS Book* A3.6 and §4.5).
    #[allow(clippy::needless_range_loop)] // `i`, `j` index the net and both basis tables
    pub fn eval(&self, u: f64, v: f64) -> SurfaceEval {
        let (u, v) = (self.wrap(0, u), self.wrap(1, v));
        let [p, q] = self.degree;
        let [n, m] = self.counts;
        let su = basis::span(p, &self.knots[0], n, u);
        let sv = basis::span(q, &self.knots[1], m, v);
        let nu = basis::derivatives(p, &self.knots[0], su, u);
        let nv = basis::derivatives(q, &self.knots[1], sv, v);
        // `temp[k][j]`: the u-sums of order k for the j-th active v row,
        // as homogeneous (w·P, w) vectors.
        let mut temp = [[Vector4::zeros(); MAX_DEGREE + 1]; ORDERS];
        for j in 0..=q {
            let col = sv - q + j;
            for i in 0..=p {
                let idx = (su - p + i) * m + col;
                let w = self.weights[idx];
                let h = Vector4::new(
                    w * self.points[idx].x,
                    w * self.points[idx].y,
                    w * self.points[idx].z,
                    w,
                );
                for k in 0..ORDERS {
                    temp[k][j] += nu[k][i] * h;
                }
            }
        }
        // `a[k][l]`: the (k, l)-th mixed homogeneous derivative, k + l ≤ 2.
        let mut a = [[Vector4::zeros(); ORDERS]; ORDERS];
        for k in 0..ORDERS {
            for l in 0..ORDERS - k {
                for j in 0..=q {
                    a[k][l] += nv[l][j] * temp[k][j];
                }
            }
        }
        let xyz = |h: &Vector4<f64>| Vec3::new(h.x, h.y, h.z);
        let w = a[0][0].w;
        let s = xyz(&a[0][0]) / w;
        let du = (xyz(&a[1][0]) - a[1][0].w * s) / w;
        let dv = (xyz(&a[0][1]) - a[0][1].w * s) / w;
        let duu = (xyz(&a[2][0]) - 2.0 * a[1][0].w * du - a[2][0].w * s) / w;
        let dvv = (xyz(&a[0][2]) - 2.0 * a[0][1].w * dv - a[0][2].w * s) / w;
        let duv = (xyz(&a[1][1]) - a[1][0].w * dv - a[0][1].w * du - a[1][1].w * s) / w;
        SurfaceEval {
            point: Point3::from(s),
            du,
            dv,
            duu,
            duv,
            dvv,
        }
    }

    /// `∂P/∂u × ∂P/∂v` normalised, or `None` where the two derivatives
    /// are parallel or one vanishes to rounding
    /// ([`arris_math::is_negligible`] against their lengths' product).
    pub fn normal(&self, u: f64, v: f64) -> Option<UnitVec3> {
        let e = self.eval(u, v);
        let cross = e.du.cross(&e.dv);
        if is_negligible(cross.norm(), e.du.norm() * e.dv.norm()) {
            return None;
        }
        UnitVec3::try_new(cross, 0.0)
    }

    /// The same surface with every control point moved by `motion`.
    pub fn transformed(&self, motion: &Isometry) -> NurbsSurface {
        NurbsSurface {
            degree: self.degree,
            knots: self.knots.clone(),
            counts: self.counts,
            points: self.points.iter().map(|p| motion.apply(*p)).collect(),
            weights: self.weights.clone(),
            period: self.period,
            closed: self.closed,
        }
    }

    /// The mirror image, same parameters: every control point reflected,
    /// so `image.eval(u, v).point == plane.apply(self.eval(u, v).point)`
    /// to rounding and the normal `∂u × ∂v` is `−R n` (ADR-0031 §2).
    pub fn mirrored(&self, plane: &Reflection) -> NurbsSurface {
        NurbsSurface {
            degree: self.degree,
            knots: self.knots.clone(),
            counts: self.counts,
            points: self.points.iter().map(|p| plane.apply(*p)).collect(),
            weights: self.weights.clone(),
            period: self.period,
            closed: self.closed,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_constructor_names_the_fault() {
        let e = NurbsSurface::new(
            [1, 1],
            [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
            vec![Point3::origin(); 3],
            vec![1.0; 3],
        )
        .unwrap_err();
        assert!(e.to_string().contains("2 × 2 net"), "{e}");
        let e = NurbsSurface::new(
            [1, 2],
            [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
            vec![Point3::origin(); 4],
            vec![1.0; 4],
        )
        .unwrap_err();
        assert!(e.to_string().contains("v: 4 knots"), "{e}");
        let e = NurbsSurface::new(
            [1, 1],
            [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
            vec![Point3::origin(); 4],
            vec![1.0, 1.0, -1.0, 1.0],
        )
        .unwrap_err();
        assert!(e.to_string().contains("weight (1, 0)"), "{e}");
    }

    #[test]
    fn a_periodic_direction_wraps() {
        // Uniform quadratic in u with three distinct rows wrapped, linear
        // clamped in v.
        let rows = [
            [Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 1.0)],
            [Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0)],
            [Point3::new(0.0, 1.0, 0.0), Point3::new(0.0, 1.0, 1.0)],
        ];
        let net: Vec<Point3> = [0, 1, 2, 0, 1].iter().flat_map(|&r| rows[r]).collect();
        let s = NurbsSurface::new(
            [2, 1],
            [(0..8).map(f64::from).collect(), vec![0.0, 0.0, 1.0, 1.0]],
            net,
            vec![1.0; 10],
        )
        .unwrap();
        assert_eq!(s.period(), [Some(3.0), None]);
        assert_eq!(s.counts(), [5, 2]);
        let (a, b) = (s.eval(2.2, 0.4), s.eval(5.2, 0.4));
        assert!((a.point - b.point).norm() < 1e-14);
        assert!((a.du - b.du).norm() < 1e-14);
        assert_eq!(s.control_point(4, 1), Some(rows[1][1]));
        assert_eq!(s.control_point(5, 0), None);
        assert_eq!(s.domain()[0], Interval::new(2.0, 5.0).unwrap());
    }
}
