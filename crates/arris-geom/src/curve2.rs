//! Curves in a surface's (u, v) plane: the pcurves of
//! `docs/DATA-MODEL.md` §Pcurves.

use core::f64::consts::TAU;
use core::fmt;

use arris_math::{
    Frame2, Interval, Point2, UnitVec2, Vec2, is_negligible, wrap_angle as wrap_turn,
};

use crate::project::ellipse_nearest;
use crate::{AmbiguousLocus, GeomError, GeomKind, NurbsCurve2};

/// A curve in a surface's (u, v) plane, with the parametrisation of
/// `docs/DATA-MODEL.md` §Pcurves. A circle or an ellipse is placed by a
/// [`Frame2`] of either handedness: a left-handed frame traverses it
/// clockwise in (u, v), which is how the pcurve of a 3D circle whose `Z`
/// opposes the plane's normal is written without reversing the curve.
///
/// The fields are plain data, as for [`crate::Curve`]; the NURBS variant
/// is valid by its constructor. Evaluation of any finite parameter never
/// panics.
///
/// ```
/// use arris_geom::Curve2;
/// use arris_math::{Frame2, Handedness, Point2, Vec2};
/// use core::f64::consts::FRAC_PI_2;
///
/// let cw = Frame2::new(Point2::origin(), Vec2::x(), Handedness::Left).unwrap();
/// let c = Curve2::Circle { frame: cw, radius: 2.0 };
/// let p = c.eval(FRAC_PI_2).point;
/// assert!((p - Point2::new(0.0, -2.0)).norm() < 1e-15); // a quarter turn goes to −v
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Curve2 {
    /// `P(t) = O + t·D`; `t` is (u, v) arc length because `D` is unit.
    Line {
        /// `O`.
        origin: Point2,
        /// `D`.
        direction: UnitVec2,
    },
    /// `P(t) = O + R(cos t·X + sin t·Y)`; counter-clockwise in (u, v) for
    /// a right-handed frame, clockwise for a left-handed one.
    Circle {
        /// Centre and axes; `X` is where `t = 0`.
        frame: Frame2,
        /// `R`.
        radius: f64,
    },
    /// `P(t) = O + a cos t·X + b sin t·Y` with `a ≥ b`; the direction of
    /// traversal is the frame's handedness.
    Ellipse {
        /// Centre and axes; `X` is the major axis.
        frame: Frame2,
        /// `a`.
        major_radius: f64,
        /// `b`.
        minor_radius: f64,
    },
    /// A rational B-spline in (u, v); parametrised by its knots.
    Nurbs(NurbsCurve2),
}

/// A 2D curve evaluated at one `t`: the point and its derivatives to
/// second order, with respect to `t` as stored, never normalised.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Curve2Eval {
    /// `P(t)`.
    pub point: Point2,
    /// `dP/dt`.
    pub d1: Vec2,
    /// `d²P/dt²`.
    pub d2: Vec2,
}

/// The nearest point of a 2D curve to a query point, with its parameter.
///
/// `point == curve.point(t)` to rounding and `distance` is the (u, v)
/// distance from the query to it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Curve2Projection {
    /// The parameter of the nearest point: a periodic `t` in `[0, 2π)`.
    pub t: f64,
    /// The nearest point.
    pub point: Point2,
    /// How far the query is from it, `≥ 0`.
    pub distance: f64,
}

/// The fieldless twin of [`Curve2`], for errors and dispatch tables.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Curve2Kind {
    /// [`Curve2::Line`].
    Line,
    /// [`Curve2::Circle`].
    Circle,
    /// [`Curve2::Ellipse`].
    Ellipse,
    /// [`Curve2::Nurbs`].
    Nurbs,
}

impl fmt::Display for Curve2Kind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Curve2Kind::Line => "line",
            Curve2Kind::Circle => "circle",
            Curve2Kind::Ellipse => "ellipse",
            Curve2Kind::Nurbs => "NURBS",
        })
    }
}

/// The magnitude at which a local coordinate of `p` in `frame` is rounding
/// noise.
fn local_noise_scale(frame: &Frame2, p: Point2) -> f64 {
    p.coords.norm() + frame.origin().coords.norm()
}

impl Curve2 {
    /// Which variant this is.
    pub fn kind(&self) -> Curve2Kind {
        match self {
            Curve2::Line { .. } => Curve2Kind::Line,
            Curve2::Circle { .. } => Curve2Kind::Circle,
            Curve2::Ellipse { .. } => Curve2Kind::Ellipse,
            Curve2::Nurbs(_) => Curve2Kind::Nurbs,
        }
    }

    /// The point and its derivatives at `t`. Defined for every finite
    /// parameter: a periodic curve wraps, a line extends, a clamped NURBS
    /// extrapolates its end piece.
    pub fn eval(&self, t: f64) -> Curve2Eval {
        match self {
            &Curve2::Line { origin, direction } => Curve2Eval {
                point: origin + t * direction.into_inner(),
                d1: direction.into_inner(),
                d2: Vec2::zeros(),
            },
            &Curve2::Circle { frame, radius } => {
                let (st, ct) = t.sin_cos();
                let (x, y) = (frame.x().into_inner(), frame.y().into_inner());
                let radial = ct * x + st * y;
                Curve2Eval {
                    point: frame.origin() + radius * radial,
                    d1: radius * (-st * x + ct * y),
                    d2: -radius * radial,
                }
            }
            &Curve2::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                let (st, ct) = t.sin_cos();
                let (x, y) = (frame.x().into_inner(), frame.y().into_inner());
                let radial = (major_radius * ct) * x + (minor_radius * st) * y;
                Curve2Eval {
                    point: frame.origin() + radial,
                    d1: (-major_radius * st) * x + (minor_radius * ct) * y,
                    d2: -radial,
                }
            }
            Curve2::Nurbs(c) => c.eval(t),
        }
    }

    /// `P(t)` alone.
    pub fn point(&self, t: f64) -> Point2 {
        self.eval(t).point
    }

    /// The parametric domain: [`Interval::REAL`] for a line, `[0, 2π]`
    /// for a circle or an ellipse, the knot range for a NURBS.
    pub fn domain(&self) -> Interval {
        match self {
            Curve2::Line { .. } => Interval::REAL,
            Curve2::Circle { .. } | Curve2::Ellipse { .. } => Interval::TURN,
            Curve2::Nurbs(c) => c.domain(),
        }
    }

    /// The period, `None` for a line or a NURBS whose knots do not wrap.
    pub fn period(&self) -> Option<f64> {
        match self {
            Curve2::Line { .. } => None,
            Curve2::Circle { .. } | Curve2::Ellipse { .. } => Some(TAU),
            Curve2::Nurbs(c) => c.period(),
        }
    }

    /// Upper bounds on `|du/dt|` and `|dv/dt|` over `range`: how far the
    /// pcurve travels in each (u, v) direction per unit of parameter, at
    /// most. Exact for a line (its direction's components) and a bound
    /// for a conic (the radius, or the major radius); sampled for a NURBS
    /// at the parameters per knot span its chord deviation is sampled at
    /// (`region2`'s `CURVATURE_SAMPLES_PER_SPAN`), a sampling density like
    /// [`crate::PCURVE_SAMPLES`], not a tolerance. What tessellation sizes an edge's sample count
    /// by, against [`crate::Surface::chord_steps`].
    ///
    /// ```
    /// use arris_geom::Curve2;
    /// use arris_math::{Frame2, Interval};
    ///
    /// let c = Curve2::Circle { frame: Frame2::identity(), radius: 3.0 };
    /// assert_eq!(c.speed_bounds(Interval::TURN), [3.0, 3.0]);
    /// ```
    pub fn speed_bounds(&self, range: Interval) -> [f64; 2] {
        match self {
            Curve2::Line { direction, .. } => [direction.x.abs(), direction.y.abs()],
            Curve2::Circle { radius, .. } => [radius.abs(); 2],
            Curve2::Ellipse { major_radius, .. } => [major_radius.abs(); 2],
            Curve2::Nurbs(n) => {
                if !(range.is_bounded() && range.length() > 0.0) {
                    return [0.0; 2];
                }
                let knots = n.breaks_within(range);
                let samples = (knots.len() + 1) * crate::region2::CURVATURE_SAMPLES_PER_SPAN;
                (0..=samples)
                    .map(|i| n.eval(range.lerp(i as f64 / samples as f64)).d1)
                    .fold([0.0; 2], |m, d| [m[0].max(d.x.abs()), m[1].max(d.y.abs())])
            }
        }
    }

    /// The same curve moved by `by` in (u, v), parametrisation carried
    /// along: `moved.eval(t).point == self.eval(t).point + by` to
    /// rounding. What a boolean does to a section edge's pcurve on a
    /// periodic surface — a whole number of periods along `u` — so the
    /// pcurve lies in the translate of the fundamental domain the face's
    /// loops are written in (`docs/DATA-MODEL.md` §Seams).
    ///
    /// ```
    /// use arris_geom::Curve2;
    /// use arris_math::{Frame2, Point2, Vec2};
    /// use core::f64::consts::TAU;
    ///
    /// let c = Curve2::Circle { frame: Frame2::identity(), radius: 2.0 };
    /// let moved = c.translated(Vec2::new(TAU, 0.0));
    /// assert_eq!(moved.point(0.0), Point2::new(TAU + 2.0, 0.0));
    /// ```
    pub fn translated(&self, by: Vec2) -> Curve2 {
        match self {
            &Curve2::Line { origin, direction } => Curve2::Line {
                origin: origin + by,
                direction,
            },
            &Curve2::Circle { frame, radius } => Curve2::Circle {
                frame: frame.translated(by),
                radius,
            },
            &Curve2::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => Curve2::Ellipse {
                frame: frame.translated(by),
                major_radius,
                minor_radius,
            },
            Curve2::Nurbs(c) => Curve2::Nurbs(c.translated(by)),
        }
    }

    /// The image under `(u, v) ↦ (2π − u, v)`, the map a mirror applies to a
    /// quadric's parameters (ADR-0031 §3): the parameter `t` is untouched,
    /// so the result runs along the same edge, and a frame's handedness
    /// flips with the reflection.
    ///
    /// ```
    /// use arris_geom::Curve2;
    /// use arris_math::{Frame2, Point2};
    /// use core::f64::consts::TAU;
    ///
    /// let c = Curve2::Circle { frame: Frame2::identity(), radius: 1.0 };
    /// let r = c.reflected();
    /// let (p, q) = (c.point(0.4), r.point(0.4));
    /// assert!((q - Point2::new(TAU - p.x, p.y)).norm() < 1e-15);
    /// ```
    pub fn reflected(&self) -> Curve2 {
        let point = |p: Point2| Point2::new(TAU - p.x, p.y);
        let vector = |v: Vec2| Vec2::new(-v.x, v.y);
        match self {
            &Curve2::Line { origin, direction } => Curve2::Line {
                origin: point(origin),
                direction: UnitVec2::new_unchecked(vector(direction.into_inner())),
            },
            Curve2::Circle { frame, radius } => Curve2::Circle {
                frame: frame.reflected_u(TAU),
                radius: *radius,
            },
            Curve2::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => Curve2::Ellipse {
                frame: frame.reflected_u(TAU),
                major_radius: *major_radius,
                minor_radius: *minor_radius,
            },
            Curve2::Nurbs(c) => Curve2::Nurbs(c.reflected()),
        }
    }

    /// The nearest point of the curve to `p` with its parameter, by the
    /// closed form of each analytic variant and by sampling and bracketed
    /// Newton for a NURBS ([`NurbsCurve2::project_parameter`]).
    ///
    /// Errors: [`GeomError::AmbiguousUv`] where the nearest point is not
    /// unique, decided to rounding — a circle's centre, an ellipse's
    /// centre or the open segment of its major axis inside the evolute.
    ///
    /// ```
    /// use arris_geom::Curve2;
    /// use arris_math::{Frame2, Point2};
    /// use core::f64::consts::PI;
    ///
    /// let c = Curve2::Circle { frame: Frame2::identity(), radius: 2.0 };
    /// let proj = c.project(Point2::new(-5.0, 0.0)).unwrap();
    /// assert!((proj.t - PI).abs() < 1e-15 && (proj.distance - 3.0).abs() < 1e-15);
    /// assert!(c.project(Point2::origin()).is_err());
    /// ```
    pub fn project(&self, p: Point2) -> Result<Curve2Projection, GeomError> {
        let ambiguous = |locus| GeomError::AmbiguousUv {
            kind: GeomKind::Curve2(self.kind()),
            locus,
            point: p,
        };
        match self {
            &Curve2::Line { origin, direction } => {
                let t = (p - origin).dot(&direction);
                let point = origin + t * direction.into_inner();
                Ok(Curve2Projection {
                    t,
                    point,
                    distance: (p - point).norm(),
                })
            }
            &Curve2::Circle { frame, radius } => {
                let q = frame.to_local(p);
                let rho = q.coords.norm();
                if is_negligible(rho, local_noise_scale(&frame, p)) {
                    return Err(ambiguous(AmbiguousLocus::Centre));
                }
                let t = wrap_turn(q.y.atan2(q.x));
                Ok(Curve2Projection {
                    t,
                    point: self.point(t),
                    distance: (rho - radius).abs(),
                })
            }
            &Curve2::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                let q = frame.to_local(p);
                let noise = local_noise_scale(&frame, p);
                let (t, distance) = ellipse_nearest(major_radius, minor_radius, q.x, q.y, noise)
                    .map_err(ambiguous)?;
                Ok(Curve2Projection {
                    t,
                    point: self.point(t),
                    distance,
                })
            }
            Curve2::Nurbs(c) => {
                let t = c.project_parameter(p);
                let point = c.eval(t).point;
                Ok(Curve2Projection {
                    t,
                    point,
                    distance: (p - point).norm(),
                })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::Handedness;

    #[test]
    fn domains_periods_and_kinds_follow_the_table() {
        let line = Curve2::Line {
            origin: Point2::origin(),
            direction: Vec2::x_axis(),
        };
        assert_eq!(line.domain(), Interval::REAL);
        assert_eq!(line.period(), None);
        assert_eq!(line.kind().to_string(), "line");
        let circle = Curve2::Circle {
            frame: Frame2::identity(),
            radius: 1.0,
        };
        assert_eq!(circle.domain(), Interval::TURN);
        assert_eq!(circle.period(), Some(TAU));
        let ellipse = Curve2::Ellipse {
            frame: Frame2::identity(),
            major_radius: 2.0,
            minor_radius: 1.0,
        };
        assert_eq!(ellipse.kind(), Curve2Kind::Ellipse);
        assert_eq!(ellipse.point(0.0), Point2::new(2.0, 0.0));
        assert_eq!(line.project(Point2::new(3.0, 4.0)).unwrap().t, 3.0);
    }

    #[test]
    fn a_left_handed_ellipse_runs_clockwise_and_projects() {
        let cw = Frame2::new(Point2::new(1.0, 1.0), Vec2::x(), Handedness::Left).unwrap();
        let e = Curve2::Ellipse {
            frame: cw,
            major_radius: 3.0,
            minor_radius: 1.0,
        };
        let quarter = e.point(core::f64::consts::FRAC_PI_2);
        assert!((quarter - Point2::new(1.0, 0.0)).norm() < 1e-15);
        let proj = e.project(Point2::new(1.0, -4.0)).unwrap();
        assert!((proj.t - core::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert!((proj.distance - 4.0).abs() < 1e-12);
        assert!(matches!(
            e.project(Point2::new(1.0, 1.0)),
            Err(GeomError::AmbiguousUv {
                locus: AmbiguousLocus::Centre,
                ..
            })
        ));
        assert!(matches!(
            e.project(Point2::new(2.0, 1.0)),
            Err(GeomError::AmbiguousUv {
                locus: AmbiguousLocus::MajorAxis,
                ..
            })
        ));
    }
}
