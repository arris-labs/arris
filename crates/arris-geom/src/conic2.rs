//! Two conics in one plane: where an ellipse or a circle meets another,
//! in closed form through the quartic of [`arris_math::roots`]. What the
//! elliptic cylinder's arms reduce to (ADR-0014): two surfaces with
//! parallel axes meet where their sections meet, and a conic lying in a
//! section plane meets the surface where it meets the section.
//!
//! The first conic is parametrised, `E₁(t) = c₁ + a₁ cos t·X₁ + b₁ sin
//! t·Y₁`, and the second is the implicit form `F₂(p) = ((p − c₂)·X₂ /
//! a₂)² + ((p − c₂)·Y₂ / b₂)² − 1`. `F₂(E₁(t))` is a trigonometric
//! polynomial of degree two in `t`, whose zeros are the crossings and
//! whose extrema, through the same polynomial's derivative, split it into
//! monotone arcs; an extremum where the first conic is within the linear
//! tolerance of the second is a touch, exactly as
//! `intersect_curve::conic_cylinder` decides a conic against a cylinder.

use arris_math::real;
use core::f64::consts::{PI, TAU};

use arris_math::roots::{self, RootError};
use arris_math::{Interval, Point2, Tolerance, Vec2, is_negligible, wrap_angle as wrap_turn};

use crate::project::ellipse_distance;

/// Newton steps that polish a candidate parameter from the half-angle
/// quartic: two suffice from a root accurate to rounding, and each is
/// taken only while it reduces the residual.
const NEWTON_POLISH_STEPS: usize = 3;

/// Rounding slack for deciding that a wrapped root is a whole turn from
/// `0`, not a second root beside it: `|2π − t| ≤ WHOLE_TURN_ROUNDING · t`.
/// The root is the quartic's, in coefficients this function expanded from
/// dot products that can themselves have cancelled — the same shape of
/// error [`roots::POLYNOMIAL_ROUNDING`] already budgets for the roots
/// its coefficients carry, so the wrapped root inherits it rather than a
/// second, invented number.
const WHOLE_TURN_ROUNDING: f64 = roots::POLYNOMIAL_ROUNDING;

/// The zeros in `[0, 2π)` of `a₁ cos t + b₁ sin t + a₂ cos 2t + b₂ sin
/// 2t + c₀`, ascending and each once: the real roots of the quartic in
/// `s = tan(t/2)` that `(1 + s²)²` times it is — `(c₀ + a₂ − a₁) s⁴ +
/// (2b₁ − 4b₂) s³ + (2c₀ − 6a₂) s² + (2b₁ + 4b₂) s + (c₀ + a₂ + a₁)` —
/// plus `t = π` (`s = ∞`) when the leading coefficient vanishes to
/// rounding, each candidate polished by Newton on the trigonometric
/// form, then wrapped into the half-open turn. A root within its own
/// rounding of a whole turn is reported at `0`, never at `2π`'s
/// neighbourhood, no tolerance: a wrapped root that close is the same
/// point on the conic, not a second one beside it, and `dedup` after the
/// sort collapses the two if the quartic found both. `None` when the
/// polynomial vanishes identically.
///
/// Errors: a non-finite coefficient.
pub(crate) fn trig2_roots(
    a1: f64,
    b1: f64,
    a2: f64,
    b2: f64,
    c0: f64,
) -> Result<Option<Vec<f64>>, RootError> {
    let c4 = c0 + a2 - a1;
    let c3 = 2.0 * b1 - 4.0 * b2;
    let c2 = 2.0 * c0 - 6.0 * a2;
    let c1 = 2.0 * b1 + 4.0 * b2;
    let c0q = c0 + a2 + a1;
    let scale = c3.abs().max(c2.abs()).max(c1.abs()).max(c0q.abs());
    let at_pi = is_negligible(c4, scale);
    let found = if at_pi {
        roots::cubic(c3, c2, c1, c0q)
    } else {
        roots::quartic(c4, c3, c2, c1, c0q)
    };
    let found = match found {
        Ok(r) => r,
        Err(RootError::Zero) => return Ok(None),
        Err(e) => return Err(e),
    };
    let f = |t: f64| {
        let (st, ct) = real::sin_cos(t);
        let (s2, c2) = real::sin_cos(2.0 * t);
        a1 * ct + b1 * st + a2 * c2 + b2 * s2 + c0
    };
    let df = |t: f64| {
        let (st, ct) = real::sin_cos(t);
        let (s2, c2) = real::sin_cos(2.0 * t);
        -a1 * st + b1 * ct - 2.0 * a2 * s2 + 2.0 * b2 * c2
    };
    let mut out: Vec<f64> = found
        .iter()
        .map(|r| 2.0 * real::atan(r.value))
        .chain(at_pi.then_some(PI))
        .map(|t0| {
            let mut t = t0;
            for _ in 0..NEWTON_POLISH_STEPS {
                let (ft, dft) = (f(t), df(t));
                if ft == 0.0 || dft == 0.0 {
                    break;
                }
                let next = t - ft / dft;
                if f(next).abs() < ft.abs() {
                    t = next;
                } else {
                    break;
                }
            }
            let wrapped = wrap_turn(t);
            // Within the root's own rounding of a whole turn: the same
            // point as `0`, not a second root beside it.
            if TAU - wrapped <= WHOLE_TURN_ROUNDING * wrapped {
                0.0
            } else {
                wrapped
            }
        })
        .collect();
    out.sort_by(f64::total_cmp);
    out.dedup();
    if out.is_empty() {
        // A trigonometric polynomial that is not identically zero has a
        // maximum and a minimum; reaching here means its coefficients
        // are rounding noise, which is the constant case.
        return Ok(None);
    }
    Ok(Some(out))
}

/// A conic in a plane's own coordinates: `c + a cos t·X + b sin t·Y`
/// with `X`, `Y` unit and perpendicular — a circle when `a = b`, in
/// which case `X` is any direction in the plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Conic2 {
    /// `c`.
    pub centre: Point2,
    /// `X`, unit.
    pub x: Vec2,
    /// `Y`, unit and perpendicular to `X`.
    pub y: Vec2,
    /// `a`, along `X`.
    pub a: f64,
    /// `b`, along `Y`.
    pub b: f64,
}

impl Conic2 {
    /// A circle of `radius` about `centre`, its `X` along the plane's.
    #[cfg(test)]
    pub(crate) fn circle(centre: Point2, radius: f64) -> Self {
        Conic2 {
            centre,
            x: Vec2::x(),
            y: Vec2::y(),
            a: radius,
            b: radius,
        }
    }

    /// The point at `t`.
    pub(crate) fn point(&self, t: f64) -> Point2 {
        let (st, ct) = real::sin_cos(t);
        self.centre + (self.a * ct) * self.x + (self.b * st) * self.y
    }

    /// The distance from `p` to the conic, exact through the quartic of
    /// [`crate::Curve::project`]; for a point whose nearest point is not
    /// unique — the centre, the major axis inside the evolute — an upper
    /// bound, the distance to the point at `p`'s own eccentric anomaly,
    /// so a point clear of the conic is never taken for one on it.
    pub(crate) fn distance(&self, p: Point2) -> f64 {
        let d = p - self.centre;
        let noise = p.coords.norm() + self.centre.coords.norm();
        ellipse_distance(self.a, self.b, d.dot(&self.x), d.dot(&self.y), noise)
    }
}

/// How two coplanar conics meet.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum ConicMeet {
    /// The first lies within the tolerance of the second everywhere.
    Coincident,
    /// They never come within the tolerance of each other.
    Empty,
    /// The meetings, ascending by the first conic's parameter: `(t,
    /// touch)`, a touch being an extremum of the residual where the first
    /// conic is within the tolerance of the second, a crossing a sign
    /// change between two extrema.
    Meets(Vec<(f64, bool)>),
}

/// Where `first` meets `second`, by the residual `F₂(E₁(t))` (module
/// docs): its extrema through [`trig2_roots`] of the derivative, a touch
/// at each extremum where `first`'s point is within `tol.linear` of
/// `second` — every extremum a touch is `Coincident` — and one crossing
/// by bracketed Newton on each arc between two extrema that are not
/// touches whose ends differ in sign. A residual with no extremum is
/// constant (a conic concentric with and similar to the other), and is
/// `Coincident` or `Empty` by the distance at two points.
///
/// Errors: a non-finite coefficient, or a crossing bracket Newton could
/// not resolve.
pub(crate) fn conic_pair(
    first: &Conic2,
    second: &Conic2,
    tol: Tolerance,
) -> Result<ConicMeet, RootError> {
    // The two linear forms of the second conic's implicit equation,
    // each along the first: `L_k(t) = A_k cos t + B_k sin t + C_k`.
    let forms = [second.x / second.a, second.y / second.b];
    let offset = first.centre - second.centre;
    let (mut alpha, mut beta, mut gamma, mut delta, mut epsilon, mut zeta) =
        (0.0, 0.0, 0.0, 0.0, 0.0, -1.0);
    for n in forms {
        let (ak, bk, ck) = (
            first.a * first.x.dot(&n),
            first.b * first.y.dot(&n),
            offset.dot(&n),
        );
        alpha += ak * ak;
        beta += bk * bk;
        gamma += 2.0 * ak * bk;
        delta += 2.0 * ak * ck;
        epsilon += 2.0 * bk * ck;
        zeta += ck * ck;
    }
    // `F = α c² + β s² + γ s c + δ c + ε s + ζ` as a trigonometric
    // polynomial in `t` and `2t`.
    let (a1, b1) = (delta, epsilon);
    let (a2, b2) = (0.5 * (alpha - beta), 0.5 * gamma);
    let c0 = 0.5 * (alpha + beta) + zeta;
    let f = |t: f64| {
        let (st, ct) = real::sin_cos(t);
        let (s2, c2) = real::sin_cos(2.0 * t);
        a1 * ct + b1 * st + a2 * c2 + b2 * s2 + c0
    };
    let df = |t: f64| {
        let (st, ct) = real::sin_cos(t);
        let (s2, c2) = real::sin_cos(2.0 * t);
        -a1 * st + b1 * ct - 2.0 * a2 * s2 + 2.0 * b2 * c2
    };
    let within = |t: f64| second.distance(first.point(t)) <= tol.linear;
    let Some(extrema) = trig2_roots(b1, -a1, 2.0 * b2, -2.0 * a2, 0.0)? else {
        // A constant residual: the first conic is a scaled copy of the
        // second about the same centre, on it or clear of it throughout.
        return Ok(if within(0.0) && within(core::f64::consts::FRAC_PI_2) {
            ConicMeet::Coincident
        } else {
            ConicMeet::Empty
        });
    };
    let touches: Vec<bool> = extrema.iter().map(|&t| within(t)).collect();
    if touches.iter().all(|&touch| touch) {
        return Ok(ConicMeet::Coincident);
    }
    let mut meets = Vec::new();
    let n = extrema.len();
    for i in 0..n {
        let (lo, lo_touch) = (extrema[i], touches[i]);
        let (mut hi, hi_touch) = (extrema[(i + 1) % n], touches[(i + 1) % n]);
        if lo_touch {
            meets.push((lo, true));
        }
        if lo_touch || hi_touch {
            continue;
        }
        if hi <= lo {
            hi += TAU;
        }
        if (f(lo) < 0.0) == (f(hi) < 0.0) {
            continue;
        }
        let Ok(bracket) = Interval::new(lo, hi) else {
            continue;
        };
        let t = roots::newton_in_interval(f, df, bracket, 0.0)?;
        meets.push((wrap_turn(t.rem_euclid(TAU)), false));
    }
    meets.sort_by(|a, b| a.0.total_cmp(&b.0));
    Ok(if meets.is_empty() {
        ConicMeet::Empty
    } else {
        ConicMeet::Meets(meets)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::Precision;

    fn tol() -> Tolerance {
        Precision::DEFAULT.tolerance()
    }

    #[test]
    fn the_trigonometric_roots_are_where_the_polynomial_vanishes() {
        // cos t − 1/2: ±π/3.
        let r = trig2_roots(1.0, 0.0, 0.0, 0.0, -0.5).unwrap().unwrap();
        assert_eq!(r.len(), 2);
        assert!((r[0] - PI / 3.0).abs() < 1e-14 && (r[1] - 5.0 * PI / 3.0).abs() < 1e-14);
        // cos 2t: four roots at odd multiples of π/4.
        let r = trig2_roots(0.0, 0.0, 1.0, 0.0, 0.0).unwrap().unwrap();
        assert_eq!(r.len(), 4);
        for (k, t) in r.iter().enumerate() {
            assert!((t - (2 * k + 1) as f64 * PI / 4.0).abs() < 1e-14, "{r:?}");
        }
        // cos t + 1: the double root at π, which the cubic branch carries.
        let r = trig2_roots(1.0, 0.0, 0.0, 0.0, 1.0).unwrap().unwrap();
        assert_eq!(r, vec![PI]);
        assert_eq!(trig2_roots(0.0, 0.0, 0.0, 0.0, 0.0).unwrap(), None);
        assert!(trig2_roots(f64::NAN, 0.0, 0.0, 0.0, 0.0).is_err());
    }

    #[test]
    fn two_circles_meet_as_the_radical_line_says() {
        let a = Conic2::circle(Point2::origin(), 2.0);
        let b = Conic2::circle(Point2::new(3.0, 0.0), 2.0);
        let ConicMeet::Meets(m) = conic_pair(&a, &b, tol()).unwrap() else {
            panic!()
        };
        assert_eq!(m.len(), 2);
        for &(t, touch) in &m {
            assert!(!touch);
            assert!((a.point(t).x - 1.5).abs() < 1e-12, "{m:?}");
        }
        // Touching outside: one touch.
        let c = Conic2::circle(Point2::new(4.0, 0.0), 2.0);
        assert_eq!(
            conic_pair(&a, &c, tol()).unwrap(),
            ConicMeet::Meets(vec![(0.0, true)])
        );
        // Concentric: coincident or empty.
        assert_eq!(conic_pair(&a, &a, tol()).unwrap(), ConicMeet::Coincident);
        let d = Conic2::circle(Point2::origin(), 1.0);
        assert_eq!(conic_pair(&a, &d, tol()).unwrap(), ConicMeet::Empty);
        assert_eq!(conic_pair(&c, &d, tol()).unwrap(), ConicMeet::Empty);
    }

    #[test]
    fn an_ellipse_against_a_circle_crosses_four_times_or_touches() {
        let ellipse = Conic2 {
            centre: Point2::origin(),
            x: Vec2::x(),
            y: Vec2::y(),
            a: 3.0,
            b: 1.0,
        };
        // A circle of radius 2 about the centre: crossed at the four
        // parameters where 9 cos² + sin² = 4.
        let circle = Conic2::circle(Point2::origin(), 2.0);
        let ConicMeet::Meets(m) = conic_pair(&ellipse, &circle, tol()).unwrap() else {
            panic!()
        };
        assert_eq!(m.len(), 4, "{m:?}");
        for &(t, touch) in &m {
            assert!(!touch);
            assert!((ellipse.point(t).coords.norm() - 2.0).abs() < 1e-12);
        }
        // A circle of the major radius touches at both major vertices
        // and crosses nowhere; of the minor radius, at both minor ones.
        let big = Conic2::circle(Point2::origin(), 3.0);
        let ConicMeet::Meets(m) = conic_pair(&ellipse, &big, tol()).unwrap() else {
            panic!()
        };
        assert_eq!(m.len(), 2);
        assert!(m.iter().all(|&(_, touch)| touch));
        assert!(m[0].0.abs() < 1e-12 && (m[1].0 - PI).abs() < 1e-12, "{m:?}");
        // Mixed: a circle of radius 2 centred at (1, 0) touches the
        // ellipse at its right vertex and crosses it twice more.
        let mixed = Conic2::circle(Point2::new(1.0, 0.0), 2.0);
        let ConicMeet::Meets(m) = conic_pair(&ellipse, &mixed, tol()).unwrap() else {
            panic!()
        };
        let touches = m.iter().filter(|&&(_, touch)| touch).count();
        assert_eq!((m.len(), touches), (3, 1), "{m:?}");
    }
}
