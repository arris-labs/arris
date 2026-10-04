//! Curves and their evaluation.

use core::fmt;

use arris_math::{Aabb, Frame, Interval, Isometry, Point3, Reflection, UnitVec3, Vec3};

use crate::NurbsCurve;

/// A 3D curve with the parametrisation of `docs/DATA-MODEL.md` §Curves
/// (Open CASCADE's, so STEP round-trips without re-parametrising).
///
/// The fields are plain data: a `Curve` is a value the arena stores once
/// and never modifies, and its validity (positive radii, `major_radius ≥
/// minor_radius`) is the checker's to enforce; the NURBS variant is valid
/// by its constructor. Evaluation of any finite parameter never panics.
///
/// ```
/// use arris_geom::Curve;
/// use arris_math::{Frame, Point3};
/// use core::f64::consts::PI;
///
/// let c = Curve::Circle { frame: Frame::world(), radius: 2.0 };
/// let e = c.eval(PI);
/// assert!((e.point - Point3::new(-2.0, 0.0, 0.0)).norm() < 1e-15);
/// assert!((e.d1.y + 2.0).abs() < 1e-15); // tangent at π points along −Y
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Curve {
    /// `P(t) = O + t·D`; `t` is arc length because `D` is unit.
    Line {
        /// `O`.
        origin: Point3,
        /// `D`.
        direction: UnitVec3,
    },
    /// `P(t) = O + R(cos t·X + sin t·Y)`, counter-clockwise about `Z`.
    Circle {
        /// Centre and plane; `X` is where `t = 0`.
        frame: Frame,
        /// `R`.
        radius: f64,
    },
    /// `P(t) = O + a cos t·X + b sin t·Y` with `a ≥ b`.
    Ellipse {
        /// Centre and plane; `X` is the major axis.
        frame: Frame,
        /// `a`.
        major_radius: f64,
        /// `b`.
        minor_radius: f64,
    },
    /// A rational B-spline; parametrised by its knots.
    Nurbs(NurbsCurve),
}

/// The fieldless twin of [`Curve`], for errors and dispatch tables.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum CurveKind {
    /// [`Curve::Line`].
    Line,
    /// [`Curve::Circle`].
    Circle,
    /// [`Curve::Ellipse`].
    Ellipse,
    /// [`Curve::Nurbs`].
    Nurbs,
}

impl fmt::Display for CurveKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            CurveKind::Line => "line",
            CurveKind::Circle => "circle",
            CurveKind::Ellipse => "ellipse",
            CurveKind::Nurbs => "NURBS",
        })
    }
}

/// A curve evaluated at one `t`: the point and its derivatives to second
/// order, with respect to `t` as stored, never normalised.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveEval {
    /// `P(t)`.
    pub point: Point3,
    /// `dP/dt`, the tangent.
    pub d1: Vec3,
    /// `d²P/dt²`.
    pub d2: Vec3,
}

impl Curve {
    /// Which variant this is.
    pub fn kind(&self) -> CurveKind {
        match self {
            Curve::Line { .. } => CurveKind::Line,
            Curve::Circle { .. } => CurveKind::Circle,
            Curve::Ellipse { .. } => CurveKind::Ellipse,
            Curve::Nurbs(_) => CurveKind::Nurbs,
        }
    }

    /// The point and its derivatives at `t`. Defined for every finite
    /// parameter: a periodic curve wraps, a line extends, a clamped NURBS
    /// extrapolates its end piece.
    pub fn eval(&self, t: f64) -> CurveEval {
        match self {
            &Curve::Line { origin, direction } => CurveEval {
                point: origin + t * direction.into_inner(),
                d1: direction.into_inner(),
                d2: Vec3::zeros(),
            },
            &Curve::Circle { frame, radius } => {
                let (st, ct) = t.sin_cos();
                let (x, y) = (frame.x().into_inner(), frame.y().into_inner());
                let radial = ct * x + st * y;
                CurveEval {
                    point: frame.origin() + radius * radial,
                    d1: radius * (-st * x + ct * y),
                    d2: -radius * radial,
                }
            }
            &Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                let (st, ct) = t.sin_cos();
                let (x, y) = (frame.x().into_inner(), frame.y().into_inner());
                let radial = (major_radius * ct) * x + (minor_radius * st) * y;
                CurveEval {
                    point: frame.origin() + radial,
                    d1: (-major_radius * st) * x + (minor_radius * ct) * y,
                    d2: -radial,
                }
            }
            Curve::Nurbs(c) => c.eval(t),
        }
    }

    /// `P(t)` alone.
    pub fn point(&self, t: f64) -> Point3 {
        self.eval(t).point
    }

    /// The parametric domain: [`Interval::REAL`] for a line, the closed
    /// fundamental interval `[0, 2π]` for a circle or an ellipse, the
    /// knot range for a NURBS.
    pub fn domain(&self) -> Interval {
        match self {
            Curve::Line { .. } => Interval::REAL,
            Curve::Circle { .. } | Curve::Ellipse { .. } => Interval::TURN,
            Curve::Nurbs(c) => c.domain(),
        }
    }

    /// The period, `None` for a line or a NURBS whose knots do not wrap.
    pub fn period(&self) -> Option<f64> {
        match self {
            Curve::Line { .. } => None,
            Curve::Circle { .. } | Curve::Ellipse { .. } => Some(core::f64::consts::TAU),
            Curve::Nurbs(c) => c.period(),
        }
    }

    /// The axis-aligned box the curve fills over `range`, or `None` when
    /// the range is not finite — a box has finite corners, and an
    /// unbounded line has no box.
    ///
    /// Exact for a line (its two endpoints) and for a conic (its extrema
    /// per axis, which are the two parameters where each coordinate's
    /// sinusoid turns, taken only when the range reaches them); an outer
    /// bound for a NURBS, whose control hull over the spans the range
    /// touches contains it — of a periodic one, over the spans of its
    /// domain the range reaches once wrapped by whole periods, the whole
    /// domain for a range of a period or more.
    ///
    /// ```
    /// use arris_geom::Curve;
    /// use arris_math::{Frame, Interval};
    /// use core::f64::consts::FRAC_PI_2;
    ///
    /// let c = Curve::Circle { frame: Frame::world(), radius: 2.0 };
    /// // A quarter turn reaches neither extremum of x nor of y: the box
    /// // is the two endpoints, to the rounding of `cos(π/2)`.
    /// let quarter = c.bounds(Interval::new(0.0, FRAC_PI_2).unwrap()).unwrap();
    /// assert!(quarter.min.iter().all(|x| x.abs() < 1e-15));
    /// assert_eq!(quarter.max, [2.0, 2.0, 0.0]);
    /// let whole = c.bounds(Interval::TURN).unwrap();
    /// assert_eq!((whole.min, whole.max), ([-2.0, -2.0, 0.0], [2.0, 2.0, 0.0]));
    /// ```
    pub fn bounds(&self, range: Interval) -> Option<Aabb> {
        if !(range.lo().is_finite() && range.hi().is_finite()) {
            return None;
        }
        match self {
            Curve::Line { .. } => Aabb::of_points(&[
                coords(self.point(range.lo())),
                coords(self.point(range.hi())),
            ]),
            &Curve::Circle { frame, radius } => Some(conic_bounds(&frame, [radius, radius], range)),
            &Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => Some(conic_bounds(&frame, [major_radius, minor_radius], range)),
            Curve::Nurbs(c) => {
                let hull = |range: Interval| {
                    let points: Vec<[f64; 3]> = active_points(c.knots(), c.degree(), range)
                        .filter_map(|i| c.control_points().get(i).map(|p| coords(*p)))
                        .collect();
                    Aabb::of_points(&points)
                };
                let domain = c.domain();
                let Some(period) = c.period() else {
                    return hull(range);
                };
                // A periodic curve's range may run past its domain, as a
                // closed section's edge cut at a seam does: the knots
                // beyond the domain carry only the few points beside its
                // ends, so the range is taken a period at a time, back
                // in the domain.
                if range.length() >= period {
                    return hull(domain);
                }
                let turns = ((range.lo() - domain.lo()) / period).floor();
                let (lo, hi) = (range.lo() - turns * period, range.hi() - turns * period);
                match (Interval::new(lo, hi.min(domain.hi())), hi > domain.hi()) {
                    (Ok(head), false) => hull(head),
                    (Ok(head), true) => {
                        let tail = Interval::new(domain.lo(), hi - period).ok()?;
                        Some(hull(head)?.union(hull(tail)?))
                    }
                    (Err(_), _) => None,
                }
            }
        }
    }

    /// How many straight segments approximate the curve over `range`
    /// within `chord` in 3D: the twin of
    /// [`crate::region2::Piece::segment_count`] for a 3D curve, with the
    /// same bound — a chord over a parameter step `h` deviates at most
    /// `|d2| h² / 8` from a curve whose second derivative is bounded by
    /// `|d2|` — and the same floors and ceiling: one segment for a line,
    /// never fewer than [`crate::region2::MIN_SEGMENTS_PER_TURN`] per
    /// turn of a conic or [`crate::region2::MIN_SEGMENTS_PER_SPAN`] per
    /// knot span of a NURBS (whose second derivative is sampled), never
    /// more than [`crate::region2::MAX_SEGMENTS_PER_PIECE`]. An unbounded
    /// or empty range is one segment; `f64::INFINITY` asks for the
    /// minimum counts alone.
    ///
    /// ```
    /// use arris_geom::Curve;
    /// use arris_math::{Frame, Interval};
    ///
    /// let c = Curve::Circle { frame: Frame::world(), radius: 4.0 };
    /// // sqrt(8 · 1e-3 / 4) ≈ 0.0447 radians per segment: 141 of them.
    /// assert_eq!(c.chord_segments(Interval::TURN, 1e-3), 141);
    /// assert_eq!(c.chord_segments(Interval::TURN, f64::INFINITY), 8);
    /// ```
    pub fn chord_segments(&self, range: Interval, chord: f64) -> usize {
        use crate::region2::{
            CURVATURE_SAMPLES_PER_SPAN, MAX_SEGMENTS_PER_PIECE, MIN_SEGMENTS_PER_SPAN, per_turn,
        };
        let length = range.length();
        if !(length.is_finite() && length > 0.0) {
            return 1;
        }
        let (minimum, d2) = match self {
            Curve::Line { .. } => return 1,
            Curve::Circle { radius, .. } => (per_turn(length), radius.abs()),
            Curve::Ellipse { major_radius, .. } => (per_turn(length), major_radius.abs()),
            Curve::Nurbs(n) => {
                let knots = n.breaks_within(range);
                let spans = knots.len() + 1;
                let samples = spans * CURVATURE_SAMPLES_PER_SPAN;
                let d2 = (0..=samples)
                    .map(|i| n.eval(range.lerp(i as f64 / samples as f64)).d2.norm())
                    .fold(0.0, f64::max);
                (spans * MIN_SEGMENTS_PER_SPAN, d2)
            }
        };
        let from_chord = if chord.is_finite() && chord > 0.0 && d2 > 0.0 {
            (length / (8.0 * chord / d2).sqrt()).ceil()
        } else if chord == f64::INFINITY || d2 == 0.0 {
            0.0
        } else {
            f64::INFINITY
        };
        let wanted = if from_chord.is_finite() {
            from_chord as usize
        } else {
            MAX_SEGMENTS_PER_PIECE
        };
        wanted.max(minimum).min(MAX_SEGMENTS_PER_PIECE)
    }

    /// The same curve moved by `motion`, parametrisation carried along:
    /// `moved.eval(t).point == motion.apply(self.eval(t).point)` to
    /// rounding.
    pub fn transformed(&self, motion: &Isometry) -> Curve {
        match self {
            &Curve::Line { origin, direction } => Curve::Line {
                origin: motion.apply(origin),
                direction: motion.apply_unit(direction),
            },
            &Curve::Circle { frame, radius } => Curve::Circle {
                frame: frame.transformed(motion),
                radius,
            },
            &Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => Curve::Ellipse {
                frame: frame.transformed(motion),
                major_radius,
                minor_radius,
            },
            Curve::Nurbs(c) => Curve::Nurbs(c.transformed(motion)),
        }
    }

    /// The mirror image in the same parameter: `image.eval(t).point ==
    /// plane.apply(self.eval(t).point)` to rounding. A conic's frame is
    /// `X′ = R X`, `Y′ = R Y`, `Z′ = −R Z`, right-handed, so no curve needs
    /// a parameter map and every edge keeps its range (ADR-0031 §2).
    ///
    /// ```
    /// use arris_geom::Curve;
    /// use arris_math::{Frame, Point3, Reflection, Vec3};
    ///
    /// let c = Curve::Circle { frame: Frame::world(), radius: 2.0 };
    /// let r = Reflection::new(Point3::new(1.0, 0.0, 0.0), Vec3::x()).unwrap();
    /// let image = c.mirrored(&r);
    /// let t = 0.7;
    /// assert!((image.point(t) - r.apply(c.point(t))).norm() < 1e-14);
    /// ```
    pub fn mirrored(&self, plane: &Reflection) -> Curve {
        match self {
            &Curve::Line { origin, direction } => Curve::Line {
                origin: plane.apply(origin),
                direction: plane.apply_unit(direction),
            },
            &Curve::Circle { frame, radius } => Curve::Circle {
                frame: plane.apply_frame_reversed(&frame),
                radius,
            },
            &Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => Curve::Ellipse {
                frame: plane.apply_frame_reversed(&frame),
                major_radius,
                minor_radius,
            },
            Curve::Nurbs(c) => Curve::Nurbs(c.mirrored(plane)),
        }
    }
}

/// A point as the array [`Aabb`] speaks in.
pub(crate) fn coords(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

/// The interval of `a cos t + b sin t = M cos(t − φ)` over `range`: the
/// two ends, plus `±M` for each extremum the range reaches. Exact.
pub(crate) fn sinusoid_range(a: f64, b: f64, range: Interval) -> [f64; 2] {
    let at = |t: f64| a * t.cos() + b * t.sin();
    let (mut lo, mut hi) = (
        at(range.lo()).min(at(range.hi())),
        at(range.lo()).max(at(range.hi())),
    );
    let magnitude = a.hypot(b);
    let phase = b.atan2(a);
    for (base, value) in [
        (phase, magnitude),
        (phase + core::f64::consts::PI, -magnitude),
    ] {
        // The first parameter at or above the range's start where the
        // sinusoid turns.
        let turns = ((range.lo() - base) / core::f64::consts::TAU).ceil();
        if base + turns * core::f64::consts::TAU <= range.hi() {
            lo = lo.min(value);
            hi = hi.max(value);
        }
    }
    [lo, hi]
}

/// The interval of `k · t` over `range`.
pub(crate) fn linear_range(k: f64, range: Interval) -> [f64; 2] {
    let (a, b) = (k * range.lo(), k * range.hi());
    [a.min(b), a.max(b)]
}

/// The interval of a product of two intervals: the extremes of the four
/// corner products, which is exact when the two vary independently and an
/// outer bound when they do not.
pub(crate) fn product_range(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    let corners = [a[0] * b[0], a[0] * b[1], a[1] * b[0], a[1] * b[1]];
    [
        corners.iter().copied().fold(f64::INFINITY, f64::min),
        corners.iter().copied().fold(f64::NEG_INFINITY, f64::max),
    ]
}

/// The box of `origin + a cos t·X + b sin t·Y` over `range`, per axis.
fn conic_bounds(frame: &Frame, radii: [f64; 2], range: Interval) -> Aabb {
    let (o, x, y) = (
        frame.origin(),
        radii[0] * frame.x().into_inner(),
        radii[1] * frame.y().into_inner(),
    );
    let mut min = [0.0; 3];
    let mut max = [0.0; 3];
    for k in 0..3 {
        let span = sinusoid_range(x[k], y[k], range);
        min[k] = o[k] + span[0];
        max[k] = o[k] + span[1];
    }
    Aabb { min, max }
}

/// The control-point indices a B-spline's `range` can touch: every span
/// the range overlaps contributes the `degree + 1` points that carry it,
/// and the convex hull of those contains the curve there.
pub(crate) fn active_points(
    knots: &[f64],
    degree: usize,
    range: Interval,
) -> impl Iterator<Item = usize> + '_ {
    let count = knots.len().saturating_sub(degree + 1);
    let (lo, hi) = (range.lo(), range.hi());
    (0..knots.len().saturating_sub(1))
        .filter(move |&k| knots[k] <= hi && knots[k + 1] >= lo && knots[k] < knots[k + 1])
        .flat_map(move |k| k.saturating_sub(degree)..=k.min(count.saturating_sub(1)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn domains_and_periods_follow_the_table() {
        let line = Curve::Line {
            origin: Point3::origin(),
            direction: Vec3::x_axis(),
        };
        assert_eq!(line.domain(), Interval::REAL);
        assert_eq!(line.period(), None);
        assert_eq!(line.kind(), CurveKind::Line);
        let circle = Curve::Circle {
            frame: Frame::world(),
            radius: 1.0,
        };
        assert_eq!(circle.domain(), Interval::TURN);
        assert_eq!(circle.period(), Some(core::f64::consts::TAU));
        assert_eq!(circle.kind().to_string(), "circle");
        let ellipse = Curve::Ellipse {
            frame: Frame::world(),
            major_radius: 2.0,
            minor_radius: 1.0,
        };
        assert_eq!(ellipse.domain(), Interval::TURN);
        assert_eq!(ellipse.kind(), CurveKind::Ellipse);
        assert_eq!(line.point(2.0), Point3::new(2.0, 0.0, 0.0));
    }
}
