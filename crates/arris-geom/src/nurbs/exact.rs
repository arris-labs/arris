//! Exact rational forms: the conics as rational quadratics, the extrusion
//! of a curve, its revolution about an axis, and — in
//! [`crate::Surface::to_nurbs`] — each analytic surface as one of them.
//!
//! Every form here is the *same point set* as what it stands for, to
//! rounding, never a fit: a conic arc is a rational quadratic Bézier, and a
//! surface of revolution is the tensor product of the generatrix with such
//! arcs (*The NURBS Book* §7.5, and §7.3 for the extrusion). What a reader
//! needs from them is that a file's `SURFACE_OF_REVOLUTION` of a spline, or
//! a `PARABOLA`, has a representation with no approximation to account for.
//!
//! Every angle range is built **clamped**, in arcs of at most a quarter
//! turn each, whose knots at the arc ends are the angles themselves. A full
//! turn is therefore *closed* and not periodic — its first and last control
//! points coincide, and its knots do not wrap — which is what a closed
//! B-spline in a file is too. A periodic form cannot hold this circle: the
//! knots a period repeats are doubled at each arc end, and a domain that
//! starts on a doubled knot ends on one, which the constructor refuses.

use arris_math::real;
use core::f64::consts::{FRAC_PI_2, TAU};

use arris_math::{Frame, Interval, Point3, RELATIVE_ROUNDING, UnitVec3, Vec3, is_negligible};

use super::{NurbsCurve, NurbsSurface};
use crate::{CurveKind, GeomError, GeomKind, SurfaceKind};

fn curve_fault(reason: impl Into<String>) -> GeomError {
    GeomError::Degenerate {
        kind: GeomKind::Curve(CurveKind::Nurbs),
        reason: reason.into(),
    }
}

fn surface_fault(reason: impl Into<String>) -> GeomError {
    GeomError::Degenerate {
        kind: GeomKind::Surface(SurfaceKind::Nurbs),
        reason: reason.into(),
    }
}

/// `(sin t, cos t)`, exact at every multiple of a quarter turn `t` holds
/// exactly: a sphere's pole is `cos(π/2) = 0`, not `6e-17`, so the row it
/// makes collapses to one point without a tolerance.
fn sin_cos_exact(t: f64) -> (f64, f64) {
    let quarters = t / FRAC_PI_2;
    if quarters == quarters.round() && quarters.abs() < 1e9 {
        // Exact: `t` is a multiple of the double nearest `π/2`, which is
        // as near a quarter turn as `f64` can name it.
        return match (quarters as i64).rem_euclid(4) {
            0 => (0.0, 1.0),
            1 => (1.0, 0.0),
            2 => (0.0, -1.0),
            _ => (-1.0, 0.0),
        };
    }
    real::sin_cos(t)
}

/// The arcs a range of angle is cut into, and the ring of control points,
/// weights and knots that carries them: the same for a circle, an ellipse
/// and the angle direction of a surface of revolution.
struct Ring {
    /// `(cos, sin)` of each control point on the unit circle: the arc ends,
    /// and between each pair the tangents' meeting point, pushed out by
    /// `1 / cos(h)` for the half-angle `h`.
    unit: Vec<[f64; 2]>,
    weights: Vec<f64>,
    knots: Vec<f64>,
}

impl Ring {
    /// The ring over `range`, clamped. Errors name what is wrong with the
    /// range.
    fn over(range: Interval) -> Result<Ring, String> {
        let (lo, hi) = (range.lo(), range.hi());
        if !(lo.is_finite() && hi.is_finite()) {
            return Err(format!("the angle range [{lo}, {hi}] is not finite"));
        }
        let mut sweep = range.length();
        if sweep <= 0.0 {
            return Err(format!("the angle range [{lo}, {hi}] is empty"));
        }
        let full = is_negligible(sweep - TAU, TAU);
        if !full && sweep > TAU {
            return Err(format!(
                "the angle range [{lo}, {hi}] is longer than a full turn"
            ));
        }
        if full {
            sweep = TAU;
        }
        let arcs = if full {
            4
        } else {
            (sweep / FRAC_PI_2 * (1.0 - RELATIVE_ROUNDING))
                .ceil()
                .max(1.0) as usize
        };
        // The angle at the `k`-th arc end, `k` counted from the start.
        let at = |k: i64| {
            if k == arcs as i64 {
                lo + sweep
            } else {
                lo + sweep * k as f64 / arcs as f64
            }
        };
        let half = sweep / arcs as f64 / 2.0;
        let (mid_weight, push) = (real::cos(half), 1.0 / real::cos(half));
        let mut unit = Vec::new();
        let mut weights = Vec::new();
        for k in 0..=arcs {
            let (s, c) = sin_cos_exact(at(k as i64));
            unit.push([c, s]);
            weights.push(1.0);
            if k < arcs {
                let (s, c) = sin_cos_exact(at(k as i64) + half);
                unit.push([c * push, s * push]);
                weights.push(mid_weight);
            }
        }
        if full {
            // The last point is the first, exactly, whatever the start:
            // `sin(t + 2π)` is not always `sin t` to the last bit.
            let first = unit[0];
            if let Some(last) = unit.last_mut() {
                *last = first;
            }
        }
        let mut knots = vec![at(0); 3];
        for k in 1..arcs {
            knots.extend([at(k as i64); 2]);
        }
        knots.extend([at(arcs as i64); 3]);
        Ok(Ring {
            unit,
            weights,
            knots,
        })
    }
}

/// `origin + α·ax + β·ay` over the angle `range`, the affine image of the
/// unit circle: an ellipse whose conjugate semi-diameters are `ax` and
/// `ay`, a circle when they are equal in length and perpendicular.
fn conic_arc(origin: Point3, ax: Vec3, ay: Vec3, range: Interval) -> Result<NurbsCurve, GeomError> {
    let ring = Ring::over(range).map_err(curve_fault)?;
    let points = ring
        .unit
        .iter()
        .map(|&[c, s]| origin + c * ax + s * ay)
        .collect();
    NurbsCurve::new(2, ring.knots, points, ring.weights)
}

fn positive(what: &str, x: f64) -> Result<f64, GeomError> {
    if x.is_finite() && x > 0.0 {
        Ok(x)
    } else {
        Err(curve_fault(format!(
            "{what} {x} is not finite and positive"
        )))
    }
}

impl NurbsCurve {
    /// The circle of `frame` and `radius` over the angle `range`, as
    /// rational quadratic arcs of at most a quarter turn each: the same
    /// point set as `Curve::Circle { frame, radius }` over `range`, to
    /// rounding. The parameter is the angle at every arc end and a
    /// monotone reparametrisation between them (a rational arc cannot be
    /// parametrised by its angle). The curve is clamped, its ends exactly
    /// the circle's points; a range of a full turn is closed and not
    /// periodic.
    ///
    /// Errors: [`GeomError::Degenerate`] for a radius that is not finite
    /// and positive, and for a range that is empty, not finite or longer
    /// than a turn.
    ///
    /// ```
    /// use arris_geom::NurbsCurve;
    /// use arris_math::{Frame, Interval};
    ///
    /// let c = NurbsCurve::circle(&Frame::world(), 2.0, Interval::TURN).unwrap();
    /// assert_eq!(c.eval(0.0).point, c.eval(core::f64::consts::TAU).point);
    /// let p = c.eval(0.7).point;
    /// assert!((p.coords.norm() - 2.0).abs() < 1e-14);
    /// ```
    pub fn circle(frame: &Frame, radius: f64, range: Interval) -> Result<Self, GeomError> {
        let r = positive("radius", radius)?;
        conic_arc(
            frame.origin(),
            r * frame.x().into_inner(),
            r * frame.y().into_inner(),
            range,
        )
    }

    /// The ellipse `O + a cos t·X + b sin t·Y` of `frame` over the angle
    /// `range`, as the circle's rational quadratic arcs mapped affinely:
    /// the same point set as `Curve::Ellipse`, and parametrised as
    /// [`NurbsCurve::circle`] is, by the eccentric anomaly at each arc
    /// end.
    ///
    /// Errors: as [`NurbsCurve::circle`], for either radius.
    ///
    /// ```
    /// use arris_geom::NurbsCurve;
    /// use arris_math::{Frame, Interval};
    ///
    /// let e = NurbsCurve::ellipse(&Frame::world(), 3.0, 2.0, Interval::TURN).unwrap();
    /// let p = e.eval(0.0).point;
    /// assert!((p.x - 3.0).abs() < 1e-15 && p.y.abs() < 1e-15);
    /// ```
    pub fn ellipse(
        frame: &Frame,
        major_radius: f64,
        minor_radius: f64,
        range: Interval,
    ) -> Result<Self, GeomError> {
        let a = positive("major radius", major_radius)?;
        let b = positive("minor radius", minor_radius)?;
        conic_arc(
            frame.origin(),
            a * frame.x().into_inner(),
            b * frame.y().into_inner(),
            range,
        )
    }

    /// The parabola `O + F t²·X + 2F t·Y` of `frame` (`PARABOLA` of
    /// ISO 10303-42, focal distance `F`) over `t ∈ range`, as the quadratic
    /// polynomial it is: a Bézier of three control points, all weights `1`,
    /// parametrised by `t` itself, so `eval(t)` is the parabola's own
    /// point.
    ///
    /// Errors: [`GeomError::Degenerate`] for a focal distance that is not
    /// finite and positive, and for a range that is empty or not finite.
    ///
    /// ```
    /// use arris_geom::NurbsCurve;
    /// use arris_math::{Frame, Interval};
    ///
    /// let p = NurbsCurve::parabola(&Frame::world(), 0.5, Interval::new(-2.0, 3.0).unwrap()).unwrap();
    /// let e = p.eval(1.5).point;
    /// assert!((e.x - 0.5 * 1.5 * 1.5).abs() < 1e-15 && (e.y - 1.5).abs() < 1e-15);
    /// ```
    pub fn parabola(
        frame: &Frame,
        focal_distance: f64,
        range: Interval,
    ) -> Result<Self, GeomError> {
        let f = positive("focal distance", focal_distance)?;
        let (t0, t1) = open_range(range)?;
        let (o, x, y) = (
            frame.origin(),
            frame.x().into_inner(),
            frame.y().into_inner(),
        );
        // The blossom of `F t²·X + 2F t·Y` at `(s, t)`.
        let blossom = |s: f64, t: f64| o + (f * s * t) * x + (f * (s + t)) * y;
        NurbsCurve::new(
            2,
            vec![t0, t0, t0, t1, t1, t1],
            vec![blossom(t0, t0), blossom(t0, t1), blossom(t1, t1)],
            vec![1.0; 3],
        )
    }

    /// The hyperbola branch `O + a cosh t·X + b sinh t·Y` of `frame`
    /// (`HYPERBOLA` of ISO 10303-42) over `t ∈ range`, as a rational
    /// quadratic: with `q = eᵗ` the homogeneous form
    /// `(a(q² + 1)/2, b(q² − 1)/2, q)` is quadratic in `q`, so one Bézier
    /// arc carries any finite range exactly. The parameter is `t` at the
    /// two ends and a monotone reparametrisation between them.
    ///
    /// Errors: [`GeomError::Degenerate`] for a semi-axis that is not
    /// finite and positive, for a range that is empty or not finite, and
    /// for one so long that `eᵗ` overflows.
    ///
    /// ```
    /// use arris_geom::NurbsCurve;
    /// use arris_math::{Frame, Interval};
    ///
    /// let h = NurbsCurve::hyperbola(&Frame::world(), 2.0, 1.0, Interval::new(-1.0, 1.0).unwrap()).unwrap();
    /// let p = h.eval(0.3).point;
    /// // On the branch: (x/a)² − (y/b)² = 1.
    /// assert!(((p.x / 2.0).powi(2) - p.y.powi(2) - 1.0).abs() < 1e-14);
    /// ```
    pub fn hyperbola(
        frame: &Frame,
        semi_axis: f64,
        semi_imaginary_axis: f64,
        range: Interval,
    ) -> Result<Self, GeomError> {
        let a = positive("semi-axis", semi_axis)?;
        let b = positive("semi-imaginary axis", semi_imaginary_axis)?;
        let (t0, t1) = open_range(range)?;
        let (q0, q1) = (real::exp(t0), real::exp(t1));
        if !(q0.is_finite() && q1.is_finite() && q0 > 0.0) {
            return Err(curve_fault(format!(
                "the range [{t0}, {t1}] is too long for a hyperbola's rational form"
            )));
        }
        let (o, x, y) = (
            frame.origin(),
            frame.x().into_inner(),
            frame.y().into_inner(),
        );
        // The blossom of `(a(q² + 1)/2, b(q² − 1)/2, q)` at `(qa, qb)`.
        let blossom = |qa: f64, qb: f64| {
            let (w, px, py) = (
                (qa + qb) / 2.0,
                a * (qa * qb + 1.0) / 2.0,
                b * (qa * qb - 1.0) / 2.0,
            );
            (o + (px / w) * x + (py / w) * y, w)
        };
        let ends = [blossom(q0, q0), blossom(q0, q1), blossom(q1, q1)];
        // Weights are homogeneous: scale them to one at the start, so a
        // long range does not carry `eᵗ` into every product.
        let scale = ends[0].1;
        NurbsCurve::new(
            2,
            vec![t0, t0, t0, t1, t1, t1],
            ends.iter().map(|e| e.0).collect(),
            ends.iter().map(|e| e.1 / scale).collect(),
        )
    }
}

/// `(lo, hi)` of a finite, non-empty range.
fn open_range(range: Interval) -> Result<(f64, f64), GeomError> {
    let (lo, hi) = (range.lo(), range.hi());
    if lo.is_finite() && hi.is_finite() && lo < hi {
        Ok((lo, hi))
    } else {
        Err(curve_fault(format!(
            "the range [{lo}, {hi}] is empty or not finite"
        )))
    }
}

impl NurbsSurface {
    /// The extrusion `P(u, v) = C(u) + v·d` of `curve` along `direction`
    /// over `v ∈ range`: degree one in `v`, whose parameter is the
    /// distance along `direction` — so for a unit `direction` it is the
    /// distance itself, as a cylinder's `v` is — and `curve`'s own
    /// parameter in `u`. The weights and knots of `curve` carry over, so
    /// a periodic curve makes a surface periodic in `u` and a closed one a
    /// closed one. Exact: the
    /// image is `C` swept along `d`.
    ///
    /// Errors: [`GeomError::Degenerate`] for a `direction` that is zero or
    /// not finite, and for a `range` that is empty or not finite.
    ///
    /// ```
    /// use arris_geom::{NurbsCurve, NurbsSurface};
    /// use arris_math::{Frame, Interval, Vec3};
    ///
    /// let circle = NurbsCurve::circle(&Frame::world(), 1.0, Interval::TURN).unwrap();
    /// let tube = NurbsSurface::extrusion(&circle, Vec3::z(), Interval::new(0.0, 4.0).unwrap()).unwrap();
    /// assert_eq!(tube.control_point(0, 1), tube.control_point(8, 1));
    /// let p = tube.eval(0.3, 2.5).point;
    /// assert!((p.z - 2.5).abs() < 1e-15 && (p.x.hypot(p.y) - 1.0).abs() < 1e-14);
    /// ```
    pub fn extrusion(
        curve: &NurbsCurve,
        direction: Vec3,
        range: Interval,
    ) -> Result<Self, GeomError> {
        if !(direction.iter().all(|c| c.is_finite()) && direction.norm() > 0.0) {
            return Err(surface_fault(format!(
                "the extrusion direction {direction:?} is zero or not finite"
            )));
        }
        let (v0, v1) = open_range(range).map_err(|_| {
            surface_fault(format!(
                "the extrusion range [{}, {}] is empty or not finite",
                range.lo(),
                range.hi()
            ))
        })?;
        let mut points = Vec::with_capacity(2 * curve.control_points().len());
        let mut weights = Vec::with_capacity(points.capacity());
        for (p, w) in curve.control_points().iter().zip(curve.weights()) {
            points.extend([p + v0 * direction, p + v1 * direction]);
            weights.extend([*w, *w]);
        }
        NurbsSurface::new(
            [curve.degree(), 1],
            [curve.knots().to_vec(), vec![v0, v0, v1, v1]],
            points,
            weights,
        )
    }

    /// The revolution of `curve` about the axis through `origin` along
    /// `axis`, through the angle `angle`: `u` is the angle of rotation
    /// (counter-clockwise about `axis`, from `curve` as placed at `u = 0`)
    /// and `v` is `curve`'s own parameter — the order of a cylinder, a
    /// cone, a sphere and a torus. Built as rational quadratic arcs of at
    /// most a quarter turn, so the image is exact; a full turn is closed in
    /// `u` (not periodic), and a periodic `curve` makes the surface periodic
    /// in `v`.
    /// The parameter `u` is the angle at every arc end, a monotone
    /// reparametrisation between them.
    ///
    /// A control point on the axis, to rounding
    /// ([`arris_math::is_negligible`] against the curve's largest
    /// distance from it and the magnitude of the coordinates that formed
    /// it), makes a *collapsed row* — every `u` at that
    /// `v` is one point — exactly: it is the surface's pole or apex.
    ///
    /// Errors: [`GeomError::Degenerate`] for a curve lying on the axis
    /// (no surface), and for an `angle` range that is empty, not finite or
    /// longer than a full turn.
    ///
    /// ```
    /// use arris_geom::{NurbsCurve, NurbsSurface};
    /// use arris_math::{Interval, Point3, Vec3};
    /// use core::f64::consts::FRAC_PI_2;
    ///
    /// // A quarter of a cylinder of radius 1: a segment parallel to the
    /// // axis, revolved a quarter turn about it.
    /// let ruling = NurbsCurve::new(
    ///     1, vec![0.0, 0.0, 1.0, 1.0],
    ///     vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 2.0)], vec![1.0; 2],
    /// ).unwrap();
    /// let quarter = Interval::new(0.0, FRAC_PI_2).unwrap();
    /// let wall = NurbsSurface::revolution(&ruling, Point3::origin(), Vec3::z_axis(), quarter).unwrap();
    /// let p = wall.eval(wall.domain()[0].hi(), 0.5).point;
    /// assert!(p.x.abs() < 1e-15 && (p.y - 1.0).abs() < 1e-15 && (p.z - 1.0).abs() < 1e-15);
    /// // A segment along the axis sweeps nothing.
    /// let along = NurbsCurve::new(
    ///     1, vec![0.0, 0.0, 1.0, 1.0],
    ///     vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 2.0)], vec![1.0; 2],
    /// ).unwrap();
    /// assert!(NurbsSurface::revolution(&along, Point3::origin(), Vec3::z_axis(), quarter).is_err());
    /// ```
    pub fn revolution(
        curve: &NurbsCurve,
        origin: Point3,
        axis: UnitVec3,
        angle: Interval,
    ) -> Result<Self, GeomError> {
        let ring = Ring::over(angle).map_err(surface_fault)?;
        let a = axis.into_inner();
        // Each control point's foot on the axis and the vector from it.
        let mut spokes: Vec<(Point3, Vec3)> = curve
            .control_points()
            .iter()
            .map(|q| {
                let foot = origin + (q - origin).dot(&a) * a;
                (foot, q - foot)
            })
            .collect();
        let reach = spokes.iter().map(|(_, r)| r.norm()).fold(0.0, f64::max);
        if reach <= 0.0 {
            return Err(surface_fault(
                "the curve lies on the axis: nothing to revolve",
            ));
        }
        // A spoke is zero to rounding when it is no longer than the error
        // of forming it: the world coordinates of the point and of the
        // axis, and the curve's reach, all enter the subtraction.
        for ((_, r), q) in spokes.iter_mut().zip(curve.control_points()) {
            let noise = reach + q.coords.norm() + origin.coords.norm();
            if is_negligible(r.norm(), noise) {
                *r = Vec3::zeros();
            }
        }
        let mut points = Vec::with_capacity(ring.unit.len() * spokes.len());
        let mut weights = Vec::with_capacity(points.capacity());
        for (&[c, s], wa) in ring.unit.iter().zip(&ring.weights) {
            for ((foot, r), wc) in spokes.iter().zip(curve.weights()) {
                points.push(foot + c * r + s * a.cross(r));
                weights.push(wa * wc);
            }
        }
        NurbsSurface::new(
            [2, curve.degree()],
            [ring.knots, curve.knots().to_vec()],
            points,
            weights,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_full_turn_is_closed_and_a_partial_one_is_shorter() {
        let full = NurbsCurve::circle(&Frame::world(), 1.0, Interval::TURN).unwrap();
        assert_eq!(full.period(), None);
        assert_eq!(full.control_points().len(), 9);
        assert_eq!(full.control_points()[0], full.control_points()[8]);
        let start =
            NurbsCurve::circle(&Frame::world(), 1.0, Interval::new(0.3, 3.7).unwrap()).unwrap();
        // 3.4 rad is three arcs of a little over a radian: seven control points.
        assert_eq!(start.control_points().len(), 7);
        let e = start.eval(0.3).point;
        assert!((e - Point3::new(real::cos(0.3f64), real::sin(0.3f64), 0.0)).norm() < 1e-15);
        let e = start.eval(3.7).point;
        assert!((e - Point3::new(real::cos(3.7f64), real::sin(3.7f64), 0.0)).norm() < 1e-15);
    }

    #[test]
    fn the_ranges_a_conic_refuses_are_named() {
        let f = Frame::world();
        let bad = |r: Result<NurbsCurve, GeomError>, needle: &str| {
            let e = r.unwrap_err().to_string();
            assert!(e.contains(needle), "{e}");
        };
        bad(
            NurbsCurve::circle(&f, 1.0, Interval::new(1.0, 1.0).unwrap()),
            "empty",
        );
        bad(
            NurbsCurve::circle(&f, 1.0, Interval::new(0.0, 7.0).unwrap()),
            "longer than a full turn",
        );
        bad(NurbsCurve::circle(&f, -1.0, Interval::TURN), "radius");
        bad(
            NurbsCurve::parabola(&f, 0.0, Interval::UNIT),
            "focal distance",
        );
        bad(
            NurbsCurve::hyperbola(&f, 1.0, 1.0, Interval::new(0.0, 1e4).unwrap()),
            "too long",
        );
    }

    #[test]
    fn a_pole_is_one_point_exactly() {
        let meridian = conic_arc(
            Point3::origin(),
            Vec3::x(),
            Vec3::z(),
            Interval::new(-FRAC_PI_2, FRAC_PI_2).unwrap(),
        )
        .unwrap();
        let s =
            NurbsSurface::revolution(&meridian, Point3::origin(), Vec3::z_axis(), Interval::TURN)
                .unwrap();
        let [n, m] = s.counts();
        for j in [0, m - 1] {
            let first = s.control_point(0, j).unwrap();
            assert!(
                (0..n).all(|i| s.control_point(i, j) == Some(first)),
                "row {j}"
            );
        }
    }
}
