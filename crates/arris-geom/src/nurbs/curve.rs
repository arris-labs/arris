//! Rational B-spline curves in 3D and in the (u, v) plane.

use arris_math::{Interval, Isometry, Point2, Point3, Reflection, Vec2};

use super::spline::{BezierSpan, Spline};
use crate::{Curve2Eval, Curve2Kind, CurveEval, CurveKind, GeomError, GeomKind};

/// A rational B-spline curve in 3D (`docs/DATA-MODEL.md` §NURBS):
/// degree `p`, `n + p + 1` non-decreasing knots, `n` control points and
/// as many positive weights, valid by construction.
///
/// Guarantees: `eval` never panics or allocates for any finite `t`; the
/// domain is `[knots[p], knots[n]]`; `period()` is `Some` exactly when the
/// knots and control points wrap (unclamped knots whose spacing repeats
/// every `n − p` places and whose last `p` control points repeat the
/// first `p`, to rounding), and then `eval` wraps `t` into the domain
/// before evaluating; otherwise a `t` outside the domain evaluates the
/// nearest polynomial piece.
///
/// ```
/// use arris_geom::NurbsCurve;
/// use arris_math::Point3;
///
/// // A quadratic Bézier arc: a quarter circle as a rational curve.
/// let w = core::f64::consts::FRAC_1_SQRT_2;
/// let arc = NurbsCurve::new(
///     2,
///     vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
///     vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
///     vec![1.0, w, 1.0],
/// ).unwrap();
/// let mid = arc.eval(0.5).point;
/// assert!((mid.coords.norm() - 1.0).abs() < 1e-15);
/// assert!(arc.period().is_none());
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(try_from = "NurbsCurveRepr", into = "NurbsCurveRepr")
)]
pub struct NurbsCurve {
    spline: Spline<3>,
}

/// The wire form of a [`NurbsCurve`]: what [`NurbsCurve::new`] takes,
/// validated by it on the way in.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct NurbsCurveRepr {
    degree: usize,
    knots: Vec<f64>,
    control_points: Vec<Point3>,
    weights: Vec<f64>,
}

#[cfg(feature = "serde")]
impl From<NurbsCurve> for NurbsCurveRepr {
    fn from(c: NurbsCurve) -> Self {
        NurbsCurveRepr {
            degree: c.degree(),
            knots: c.knots().to_vec(),
            control_points: c.control_points().to_vec(),
            weights: c.weights().to_vec(),
        }
    }
}

#[cfg(feature = "serde")]
impl TryFrom<NurbsCurveRepr> for NurbsCurve {
    type Error = GeomError;

    fn try_from(r: NurbsCurveRepr) -> Result<Self, GeomError> {
        NurbsCurve::new(r.degree, r.knots, r.control_points, r.weights)
    }
}

/// A rational B-spline curve in a surface's (u, v) plane: the same
/// representation and guarantees as [`NurbsCurve`] in two dimensions.
///
/// ```
/// use arris_geom::NurbsCurve2;
/// use arris_math::Point2;
///
/// let seg = NurbsCurve2::new(
///     1,
///     vec![0.0, 0.0, 2.0, 2.0],
///     vec![Point2::new(0.0, 0.0), Point2::new(4.0, 2.0)],
///     vec![1.0, 1.0],
/// ).unwrap();
/// assert_eq!(seg.eval(1.0).point, Point2::new(2.0, 1.0));
/// assert_eq!(seg.eval(1.0).d1, arris_math::Vec2::new(2.0, 1.0));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(
    feature = "serde",
    serde(try_from = "NurbsCurve2Repr", into = "NurbsCurve2Repr")
)]
pub struct NurbsCurve2 {
    spline: Spline<2>,
}

/// The wire form of a [`NurbsCurve2`], validated by [`NurbsCurve2::new`]
/// on the way in.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
struct NurbsCurve2Repr {
    degree: usize,
    knots: Vec<f64>,
    control_points: Vec<Point2>,
    weights: Vec<f64>,
}

#[cfg(feature = "serde")]
impl From<NurbsCurve2> for NurbsCurve2Repr {
    fn from(c: NurbsCurve2) -> Self {
        NurbsCurve2Repr {
            degree: c.degree(),
            knots: c.knots().to_vec(),
            control_points: c.control_points().to_vec(),
            weights: c.weights().to_vec(),
        }
    }
}

#[cfg(feature = "serde")]
impl TryFrom<NurbsCurve2Repr> for NurbsCurve2 {
    type Error = GeomError;

    fn try_from(r: NurbsCurve2Repr) -> Result<Self, GeomError> {
        NurbsCurve2::new(r.degree, r.knots, r.control_points, r.weights)
    }
}

fn degenerate(kind: GeomKind) -> impl Fn(String) -> GeomError {
    move |reason| GeomError::Degenerate { kind, reason }
}

impl NurbsCurve {
    /// The curve over a spline its producer already validated.
    pub(super) fn from_spline(spline: Spline<3>) -> Self {
        NurbsCurve { spline }
    }

    /// A validated curve. Errors: [`GeomError::Degenerate`] naming what is
    /// wrong — a degree outside `1..=MAX_DEGREE`, the wrong number of
    /// knots or weights, decreasing or non-finite knots, an empty domain,
    /// a knot multiplicity above the degree inside the domain or above
    /// the degree plus one anywhere, a non-finite control point, a
    /// weight that is not finite and positive.
    pub fn new(
        degree: usize,
        knots: Vec<f64>,
        control_points: Vec<Point3>,
        weights: Vec<f64>,
    ) -> Result<Self, GeomError> {
        Spline::new(degree, knots, control_points, weights)
            .map(|spline| NurbsCurve { spline })
            .map_err(degenerate(GeomKind::Curve(CurveKind::Nurbs)))
    }

    /// `p`.
    pub fn degree(&self) -> usize {
        self.spline.degree()
    }

    /// The `n + p + 1` knots, non-decreasing.
    pub fn knots(&self) -> &[f64] {
        self.spline.knots()
    }

    /// The `n` control points, Cartesian.
    pub fn control_points(&self) -> &[Point3] {
        self.spline.points()
    }

    /// The `n` weights, all positive.
    pub fn weights(&self) -> &[f64] {
        self.spline.weights()
    }

    /// `[knots[p], knots[n]]`.
    pub fn domain(&self) -> Interval {
        self.spline.domain()
    }

    /// The domain's length when the knots and control points wrap, else
    /// `None`.
    pub fn period(&self) -> Option<f64> {
        self.spline.period()
    }

    /// The distinct knots strictly inside `range`, ascending: where the
    /// curve's polynomial pieces change over it, which is where a
    /// quadrature or a polygon of the curve is split. A periodic curve's
    /// knots are repeated by whole periods, so a range that runs past the
    /// knots' end — a block of a closed section wrapping round to its
    /// first pave — is broken at every span it covers, as a range inside
    /// the domain is.
    ///
    /// ```
    /// use arris_geom::NurbsCurve;
    /// use arris_math::{Interval, Point3};
    ///
    /// let c = NurbsCurve::new(
    ///     2,
    ///     vec![0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0],
    ///     vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0), Point3::new(3.0, 2.0, 0.0), Point3::new(4.0, 0.0, 0.0)],
    ///     vec![1.0; 4],
    /// ).unwrap();
    /// assert_eq!(c.breaks_within(Interval::new(0.5, 2.0).unwrap()), vec![1.0]);
    /// ```
    pub fn breaks_within(&self, range: Interval) -> Vec<f64> {
        self.spline.breaks_within(range)
    }

    /// The point and its derivatives at `t` (de Boor on the homogeneous
    /// control points, then the quotient rule).
    pub fn eval(&self, t: f64) -> CurveEval {
        let d = self.spline.eval(t);
        CurveEval {
            point: d.point,
            d1: d.d1,
            d2: d.d2,
        }
    }

    /// The same curve with `t` inserted `times` more times as a knot: the
    /// image over the domain is unchanged to rounding, the control
    /// polygon is refined. `t` is wrapped into the domain of a periodic
    /// curve; the result's knots no longer wrap, so its `period()` is
    /// `None` and outside the domain it extrapolates. Errors:
    /// [`GeomError::Degenerate`] when `t` is outside `[knots[p],
    /// knots[n])` or the multiplicity would exceed the degree.
    ///
    /// ```
    /// use arris_geom::NurbsCurve;
    /// use arris_math::Point3;
    ///
    /// let c = NurbsCurve::new(
    ///     2,
    ///     vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    ///     vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
    ///     vec![1.0, 1.0, 1.0],
    /// ).unwrap();
    /// let refined = c.insert_knot(0.5, 1).unwrap();
    /// assert_eq!(refined.control_points().len(), 4);
    /// assert!((refined.eval(0.3).point - c.eval(0.3).point).norm() < 1e-15);
    /// ```
    pub fn insert_knot(&self, t: f64, times: usize) -> Result<Self, GeomError> {
        self.spline
            .insert_knot(t, times)
            .map(|spline| NurbsCurve { spline })
            .map_err(degenerate(GeomKind::Curve(CurveKind::Nurbs)))
    }

    /// The curve over exactly `range`, as a clamped curve in the same
    /// parameter: `piece.eval(t).point == self.eval(t).point` to rounding
    /// for every `t` of `range`, and `piece.domain() == range`. A
    /// periodic curve takes any range of at most one period, wherever it
    /// starts — an edge's range past the knots' end among them. What a
    /// reader that finds an edge's range by its vertices needs of a
    /// closed curve (`arris-io`'s STEP writer).
    ///
    /// Errors: [`GeomError::Degenerate`] for a range outside the domain
    /// of a curve that is not periodic, or longer than one period of one
    /// that is.
    ///
    /// ```
    /// use arris_geom::NurbsCurve;
    /// use arris_math::{Interval, Point3};
    ///
    /// let c = NurbsCurve::new(
    ///     2,
    ///     vec![0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0],
    ///     vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0), Point3::new(3.0, 2.0, 0.0), Point3::new(4.0, 0.0, 0.0)],
    ///     vec![1.0; 4],
    /// ).unwrap();
    /// let piece = c.segment(Interval::new(0.5, 1.5).unwrap()).unwrap();
    /// assert_eq!((piece.domain().lo(), piece.domain().hi()), (0.5, 1.5));
    /// assert!((piece.eval(0.8).point - c.eval(0.8).point).norm() < 1e-15);
    /// ```
    pub fn segment(&self, range: Interval) -> Result<Self, GeomError> {
        self.spline
            .segment(range.lo(), range.hi())
            .map(|spline| NurbsCurve { spline })
            .map_err(degenerate(GeomKind::Curve(CurveKind::Nurbs)))
    }

    /// The curve's polynomial pieces in Bernstein form, one per
    /// non-empty span, ascending: what a curve substituted into an
    /// implicit surface is built from (`crate::intersect_spline`).
    pub(crate) fn bezier_spans(&self) -> Vec<BezierSpan<3>> {
        self.spline.bezier_spans()
    }

    /// The parameter of the nearest point to `p`: the best of `2p + 2`
    /// samples per span, polished by bracketed Newton on the derivative
    /// of the squared distance. It is the nearest *local* minimum from
    /// that sample — never an error, and never a guarantee that a nearer
    /// point the sampling missed does not exist. In the domain; in
    /// `[knots[p], knots[n])` for a periodic curve.
    pub fn project_parameter(&self, p: Point3) -> f64 {
        self.spline.project(&p)
    }

    /// The same curve with every control point moved by `motion`;
    /// weights and knots unchanged, so `moved.eval(t).point ==
    /// motion.apply(self.eval(t).point)` to rounding.
    pub fn transformed(&self, motion: &Isometry) -> NurbsCurve {
        NurbsCurve {
            spline: self.spline.map_points(|p| motion.apply(*p)),
        }
    }

    /// The mirror image, same parameter: every control point reflected;
    /// weights and knots unchanged, so `image.eval(t).point ==
    /// plane.apply(self.eval(t).point)` to rounding.
    pub fn mirrored(&self, plane: &Reflection) -> NurbsCurve {
        NurbsCurve {
            spline: self.spline.map_points(|p| plane.apply(*p)),
        }
    }
}

impl NurbsCurve2 {
    /// The curve over a spline its producer already validated.
    pub(super) fn from_spline(spline: Spline<2>) -> Self {
        NurbsCurve2 { spline }
    }

    /// A validated curve; errors as [`NurbsCurve::new`].
    pub fn new(
        degree: usize,
        knots: Vec<f64>,
        control_points: Vec<Point2>,
        weights: Vec<f64>,
    ) -> Result<Self, GeomError> {
        Spline::new(degree, knots, control_points, weights)
            .map(|spline| NurbsCurve2 { spline })
            .map_err(degenerate(GeomKind::Curve2(Curve2Kind::Nurbs)))
    }

    /// `p`.
    pub fn degree(&self) -> usize {
        self.spline.degree()
    }

    /// The `n + p + 1` knots, non-decreasing.
    pub fn knots(&self) -> &[f64] {
        self.spline.knots()
    }

    /// The `n` control points, Cartesian.
    pub fn control_points(&self) -> &[Point2] {
        self.spline.points()
    }

    /// The `n` weights, all positive.
    pub fn weights(&self) -> &[f64] {
        self.spline.weights()
    }

    /// `[knots[p], knots[n]]`.
    pub fn domain(&self) -> Interval {
        self.spline.domain()
    }

    /// The domain's length when the knots and control points wrap, else
    /// `None`.
    pub fn period(&self) -> Option<f64> {
        self.spline.period()
    }

    /// The distinct knots strictly inside `range`, ascending: where the
    /// curve's polynomial pieces change over it, which is where a
    /// quadrature or a polygon of the curve is split. A periodic curve's
    /// knots are repeated by whole periods, so a range that runs past the
    /// knots' end — a block of a closed section wrapping round to its
    /// first pave — is broken at every span it covers, as a range inside
    /// the domain is.
    ///
    /// ```
    /// use arris_geom::NurbsCurve2;
    /// use arris_math::{Interval, Point2};
    ///
    /// let c = NurbsCurve2::new(
    ///     2,
    ///     vec![0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0],
    ///     vec![Point2::new(0.0, 0.0), Point2::new(1.0, 2.0), Point2::new(3.0, 2.0), Point2::new(4.0, 0.0)],
    ///     vec![1.0; 4],
    /// ).unwrap();
    /// assert_eq!(c.breaks_within(Interval::new(0.5, 2.0).unwrap()), vec![1.0]);
    /// ```
    pub fn breaks_within(&self, range: Interval) -> Vec<f64> {
        self.spline.breaks_within(range)
    }

    /// The point and its derivatives at `t`.
    pub fn eval(&self, t: f64) -> Curve2Eval {
        let d = self.spline.eval(t);
        Curve2Eval {
            point: d.point,
            d1: d.d1,
            d2: d.d2,
        }
    }

    /// The same curve with `t` inserted `times` more times as a knot; as
    /// [`NurbsCurve::insert_knot`].
    pub fn insert_knot(&self, t: f64, times: usize) -> Result<Self, GeomError> {
        self.spline
            .insert_knot(t, times)
            .map(|spline| NurbsCurve2 { spline })
            .map_err(degenerate(GeomKind::Curve2(Curve2Kind::Nurbs)))
    }

    /// The parameter of the nearest point to `p`; as
    /// [`NurbsCurve::project_parameter`].
    pub fn project_parameter(&self, p: Point2) -> f64 {
        self.spline.project(&p)
    }

    /// The same curve with every control point moved by `by`; weights
    /// and knots unchanged, so `moved.eval(t).point == self.eval(t).point
    /// + by` to rounding.
    pub fn translated(&self, by: Vec2) -> NurbsCurve2 {
        NurbsCurve2 {
            spline: self.spline.map_points(|p| p + by),
        }
    }

    /// The image under `(u, v) ↦ (2π − u, v)`: every control point
    /// reflected; weights and knots unchanged, so `image.eval(t).point`
    /// is `self.eval(t).point` with its `u` reflected.
    pub fn reflected(&self) -> NurbsCurve2 {
        NurbsCurve2 {
            spline: self
                .spline
                .map_points(|p| Point2::new(core::f64::consts::TAU - p.x, p.y)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cubic() -> NurbsCurve {
        NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(2.0, -1.0, 1.0),
                Point3::new(3.0, 0.0, 0.0),
                Point3::new(4.0, 2.0, 0.0),
            ],
            vec![1.0, 2.0, 1.0, 0.5, 1.0],
        )
        .unwrap()
    }

    #[test]
    fn a_clamped_curve_starts_and_ends_at_its_control_points() {
        let c = cubic();
        assert_eq!(c.eval(0.0).point, Point3::new(0.0, 0.0, 0.0));
        assert_eq!(c.eval(2.0).point, Point3::new(4.0, 2.0, 0.0));
        assert_eq!(c.domain(), Interval::new(0.0, 2.0).unwrap());
        assert_eq!(c.period(), None);
        assert_eq!(c.degree(), 3);
        assert_eq!(c.knots().len(), 9);
        assert_eq!(c.weights()[1], 2.0);
    }

    #[test]
    fn the_constructor_names_the_fault() {
        let err = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![Point3::origin(); 3],
            vec![1.0, 0.0, 1.0],
        )
        .unwrap_err();
        assert!(matches!(
            err,
            GeomError::Degenerate {
                kind: GeomKind::Curve(CurveKind::Nurbs),
                ..
            }
        ));
        assert!(err.to_string().contains("weight 1"), "{err}");
        let err = NurbsCurve2::new(
            1,
            vec![0.0, 1.0, 1.0],
            vec![Point2::origin(); 2],
            vec![1.0; 2],
        )
        .unwrap_err();
        assert!(err.to_string().contains("3 knots"), "{err}");
        assert!(matches!(
            err,
            GeomError::Degenerate {
                kind: GeomKind::Curve2(Curve2Kind::Nurbs),
                ..
            }
        ));
    }

    #[test]
    fn knot_insertion_refuses_what_would_break_the_curve() {
        let c = cubic();
        assert!(c.insert_knot(2.0, 1).is_err());
        assert!(c.insert_knot(-0.5, 1).is_err());
        assert!(c.insert_knot(1.0, 3).is_err());
        let twice = c.insert_knot(1.0, 2).unwrap();
        assert_eq!(twice.control_points().len(), 7);
        assert_eq!(c.insert_knot(0.5, 0).unwrap(), c);
        let e = c.insert_knot(f64::NAN, 1).unwrap_err();
        assert!(e.to_string().contains("outside"), "{e}");
    }

    #[test]
    fn a_periodic_curve_wraps_and_a_clamped_one_extrapolates() {
        // Uniform quadratic with three distinct control points wrapped.
        let pts = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let c = NurbsCurve::new(
            2,
            (0..8).map(f64::from).collect(),
            vec![pts[0], pts[1], pts[2], pts[0], pts[1]],
            vec![1.0; 5],
        )
        .unwrap();
        assert_eq!(c.period(), Some(3.0));
        let (a, b) = (c.eval(2.7), c.eval(5.7));
        assert!((a.point - b.point).norm() < 1e-14);
        assert!((a.d1 - b.d1).norm() < 1e-14);
        assert!(c.transformed(&Isometry::identity()).period().is_some());
        let unwrapped = c.insert_knot(3.5, 1).unwrap();
        assert_eq!(unwrapped.period(), None);
        assert!((unwrapped.eval(4.2).point - c.eval(4.2).point).norm() < 1e-14);
    }

    #[test]
    fn a_periodic_curves_breaks_follow_a_range_past_its_knots() {
        let pts = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Domain [2, 5], period 3, stored knots 0 to 7.
        let c = NurbsCurve::new(
            2,
            (0..8).map(f64::from).collect(),
            vec![pts[0], pts[1], pts[2], pts[0], pts[1]],
            vec![1.0; 5],
        )
        .unwrap();
        let within = |lo, hi| c.breaks_within(Interval::new(lo, hi).unwrap());
        // Inside the stored knots: those, as they are.
        assert_eq!(within(2.5, 4.5), vec![3.0, 4.0]);
        assert_eq!(within(4.5, 7.4), vec![5.0, 6.0, 7.0]);
        // Past either end: the domain's knots a whole period on.
        assert_eq!(within(6.5, 9.2), vec![7.0, 8.0, 9.0]);
        assert_eq!(within(-2.5, -0.2), vec![-2.0, -1.0]);
        assert_eq!(within(10.5, 12.5), vec![11.0, 12.0]);
        // A clamped curve has no knots but its own, its end among them.
        let clamped = c.segment(Interval::new(2.0, 5.0).unwrap()).unwrap();
        assert_eq!(clamped.period(), None);
        assert_eq!(
            clamped.breaks_within(Interval::new(2.5, 9.0).unwrap()),
            vec![3.0, 4.0, 5.0]
        );
    }
}
