//! The quarter turn as a rational quadratic arc, and the angle its
//! parameter stands for.
//!
//! A full turn is four of these, and every one is exact: the arc with
//! homogeneous control points `(1, 0, 1)`, `(1, 1, cos π/4)`, `(0, 1, 1)`
//! — the half-angle parametrisation about the quarter's middle,
//! `tan((θ − θ_mid) / 2) = tan(π/8)·(2s − 1)` — traces `(cos θ, sin θ)`
//! over `s` in `[0, 1]`. It is what puts a turn into a polynomial: a
//! torus's patches in its own two parameters (`crate::trace_torus`), and
//! a conic against a torus in its one (`crate::intersect_curve`), where
//! the quartic implicit form of a torus leaves no trigonometric
//! polynomial of degree two to solve.

use arris_math::real;
use core::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4, SQRT_2};

/// `tan(π/8)`: the half-angle parameter at the end of a quarter turn
/// measured from its middle.
pub(crate) const QUARTER_TAN: f64 = SQRT_2 - 1.0;

/// The cosine, the sine and the weight of quarter turn `quarter`, from
/// `quarter·π/2`, as quadratic polynomials in Bernstein form: the first
/// quarter's turned by whole quarter turns, which is exact. The cosine
/// and the sine are the *homogeneous* coordinates, the point at `s`
/// being each over [`quarter_weight`].
pub(crate) fn quarter_arc(quarter: usize) -> [[f64; 3]; 3] {
    let (c, s) = ([1.0, FRAC_1_SQRT_2, 0.0], [0.0, FRAC_1_SQRT_2, 1.0]);
    let minus = |a: [f64; 3]| a.map(|v| -v);
    let (cos, sin) = match quarter % 4 {
        0 => (c, s),
        1 => (minus(s), c),
        2 => (minus(c), minus(s)),
        _ => (s, minus(c)),
    };
    [cos, sin, [1.0, FRAC_1_SQRT_2, 1.0]]
}

/// The angle at parameter `s` of quarter turn `quarter`; `s` a little
/// outside `[0, 1]` is the angle a little outside the quarter.
pub(crate) fn quarter_angle(quarter: usize, s: f64) -> f64 {
    quarter as f64 * FRAC_PI_2 + FRAC_PI_4 + 2.0 * real::atan(QUARTER_TAN * (2.0 * s - 1.0))
}

/// The parameter of a quarter turn at `angle` from its start, the
/// inverse of [`quarter_angle`], kept inside `[0, 1]`.
pub(crate) fn quarter_param(angle: f64) -> f64 {
    (0.5 * (1.0 + real::tan(0.5 * (angle - FRAC_PI_4)) / QUARTER_TAN)).clamp(0.0, 1.0)
}

/// The weight of a quarter turn at its parameter `s`, the quadratic with
/// the coefficients `(1, cos π/4, 1)`.
pub(crate) fn quarter_weight(s: f64) -> f64 {
    1.0 - (2.0 - SQRT_2) * s * (1.0 - s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bernstein::Binomials;
    use crate::implicit::Implicit;

    /// The arc traces the quarter turn it stands for: its homogeneous
    /// coordinates over its weight are the cosine and the sine of
    /// [`quarter_angle`], and the parameter is its inverse.
    #[test]
    fn a_quarter_arc_traces_its_own_angles() {
        let eval = |c: [f64; 3], s: f64| {
            let (u, v) = (1.0 - s, s);
            u * u * c[0] + 2.0 * u * v * c[1] + v * v * c[2]
        };
        for quarter in 0..4 {
            let [cos, sin, w] = quarter_arc(quarter);
            for k in 0..=8 {
                let s = k as f64 / 8.0;
                let angle = quarter_angle(quarter, s);
                assert!((eval(w, s) - quarter_weight(s)).abs() < 1e-15);
                assert!((eval(cos, s) / eval(w, s) - real::cos(angle)).abs() < 1e-15);
                assert!((eval(sin, s) / eval(w, s) - real::sin(angle)).abs() < 1e-15);
                let back = quarter_param(angle - quarter as f64 * FRAC_PI_2);
                assert!((back - s).abs() < 1e-14, "{quarter} at {s}: {back}");
            }
        }
    }

    /// A conic's quarter arc put into a torus's implicit form is a
    /// polynomial of degree eight whose sign is the torus's own: the
    /// substitution [`Implicit::along`] does for a NURBS span works for
    /// the arc's homogeneous coordinates as well.
    #[test]
    fn a_quarter_of_a_conic_in_a_torus_is_a_polynomial_of_degree_eight() {
        use arris_math::{Frame, Point3, Vec3};

        let torus = crate::Surface::Torus {
            frame: Frame::world(),
            major_radius: 3.0,
            minor_radius: 1.0,
        };
        let implicit = Implicit::of(&torus).unwrap();
        let binomials = Binomials::new(8);
        let (centre, u, v) = (
            Point3::new(0.5, -0.2, 0.3),
            Vec3::new(2.0, 0.4, 0.1),
            Vec3::new(-0.3, 1.7, 0.8),
        );
        for quarter in 0..4 {
            let [cos, sin, w] = quarter_arc(quarter);
            let coords: Vec<Vec<f64>> = (0..3)
                .map(|k| {
                    (0..3)
                        .map(|i| w[i] * centre[k] + cos[i] * u[k] + sin[i] * v[k])
                        .collect()
                })
                .collect();
            let weight = w.to_vec();
            let g = implicit.along([&coords[0], &coords[1], &coords[2], &weight], &binomials);
            assert_eq!(g.len(), 9);
            for k in 0..=8 {
                let s = k as f64 / 8.0;
                let angle = quarter_angle(quarter, s);
                let point = centre + real::cos(angle) * u + real::sin(angle) * v;
                let direct = implicit.value(point) * real::powi(quarter_weight(s), 4);
                let bernstein = {
                    let mut row = g.clone();
                    for level in 1..row.len() {
                        for i in 0..row.len() - level {
                            row[i] = (1.0 - s) * row[i] + s * row[i + 1];
                        }
                    }
                    row[0]
                };
                assert!(
                    (direct - bernstein).abs() < 1e-10 * direct.abs().max(1.0),
                    "{quarter} at {s}: {direct} vs {bernstein}"
                );
            }
        }
    }
}
