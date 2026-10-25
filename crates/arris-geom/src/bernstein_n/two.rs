//! The isolation's unit tests at `N = 2`, from before the two modules became one.

#[cfg(test)]
mod cases {
    use super::super::*;
    use crate::bernstein::Binomials;
    use arris_math::real;

    /// `a₀ + aₛ·s + aₜ·t`.
    fn linear(a0: f64, a_s: f64, a_t: f64) -> PolyN<2> {
        PolyN::new([1, 1], vec![a0, a0 + a_t, a0 + a_s, a0 + a_s + a_t])
    }

    /// `(s − a)² + (t − b)² − ρ²`.
    fn circle(a: f64, b: f64, rho: f64, binomials: &Binomials) -> PolyN<2> {
        let (ds, dt) = (linear(-a, 1.0, 0.0), linear(-b, 0.0, 1.0));
        let one = PolyN::outer([&[1.0; 3], &[1.0; 3]]);
        PolyN::combine(&[
            (1.0, &ds.mul(&ds, binomials)),
            (1.0, &dt.mul(&dt, binomials)),
            (-rho * rho, &one),
        ])
    }

    fn certified(found: &Isolation<2>) -> Vec<[f64; 2]> {
        found
            .zeros
            .iter()
            .filter(|z| z.certified)
            .map(|z| z.at)
            .collect()
    }

    fn assert_near(found: &[[f64; 2]], expected: &[[f64; 2]]) {
        assert_eq!(found.len(), expected.len(), "{found:?} vs {expected:?}");
        for (f, e) in found.iter().zip(expected) {
            assert!(
                (f[0] - e[0]).abs() < 1e-13 && (f[1] - e[1]).abs() < 1e-13,
                "{found:?} vs {expected:?}"
            );
        }
    }

    #[test]
    fn bernstein_n_the_arithmetic_is_the_polynomials_own() {
        let b = Binomials::new(8);
        let p = circle(0.3, 0.6, 0.25, &b);
        let q = linear(0.5, -2.0, 1.5);
        let pq = p.mul(&q, &b);
        assert_eq!(pq.degree(), [3, 3]);
        let (ps, pt) = (p.partial(0), p.partial(1));
        assert_eq!((ps.degree(), pt.degree()), ([1, 2], [2, 1]));
        for i in 0..=10 {
            for j in 0..=10 {
                let (s, t) = (i as f64 / 10.0, j as f64 / 10.0);
                let value = real::powi(s - 0.3, 2) + real::powi(t - 0.6, 2) - 0.0625;
                assert!((p.eval([s, t]) - value).abs() < 1e-15);
                assert!((pq.eval([s, t]) - value * q.eval([s, t])).abs() < 1e-14);
                assert!((ps.eval([s, t]) - 2.0 * (s - 0.3)).abs() < 1e-14);
                assert!((pt.eval([s, t]) - 2.0 * (t - 0.6)).abs() < 1e-14);
            }
        }
        assert_eq!(
            linear(1.0, 0.0, 0.0).partial(0).partial(1).coefficients(),
            [0.0]
        );
    }

    #[test]
    fn bernstein_n_a_polynomial_over_its_linear_factor_is_the_other_factor() {
        let binomials = Binomials::new(16);
        // A quotient of bidegree (5, 3) with nothing special about it.
        let quotient = PolyN::new(
            [5, 3],
            (0..24).map(|k| ((k * 7 + 3) % 11) as f64 - 4.5).collect(),
        );
        // Roots inside the square, on its edges and outside it.
        for root in [0.0, 0.3, 0.5, 0.83, 1.0, -0.4, 1.5, 3.4] {
            let factor = PolyN::outer([&[-root, 1.0 - root], &[1.0]]);
            let product = factor.mul(&quotient, &binomials);
            let floor = 8.0 * f64::EPSILON * 5.5;
            let (found, rounding) = product.over_linear(root, floor, &binomials);
            assert_eq!(found.degree(), [5, 3]);
            for (a, b) in found.coefficients().iter().zip(quotient.coefficients()) {
                assert!((a - b).abs() <= rounding, "{root}: {a} vs {b} ± {rounding}");
            }
            // The rounding carried is a small multiple of what went in.
            assert!(rounding <= 64.0 * floor, "{root}: {rounding} from {floor}");
            // Twice over, for a factor squared.
            let (again, _) = (factor.mul(&product, &binomials))
                .over_linear(root, floor, &binomials)
                .0
                .over_linear(root, floor, &binomials);
            for (a, b) in again.coefficients().iter().zip(quotient.coefficients()) {
                assert!((a - b).abs() <= 1e-12, "{root}: {a} vs {b}");
            }
        }
        // The lines of a polynomial are the polynomial along them.
        let (s, t) = (0.37, 0.81);
        let value = quotient.eval([s, t]);
        assert!((casteljau(&quotient.across(s), t) - value).abs() < 1e-14);
        assert!((casteljau(&quotient.along(t), s) - value).abs() < 1e-14);
    }

    #[test]
    fn bernstein_n_the_parts_of_a_box_are_the_polynomial_on_them() {
        let b = Binomials::new(8);
        let p = circle(0.3, 0.6, 0.25, &b).mul(&linear(0.5, -2.0, 1.5), &b);
        let (lo, hi) = p.split(0, 0.5);
        let ((a, b), (c, d)) = (lo.split(1, 0.5), hi.split(1, 0.5));
        let quarters = [a, b, c, d];
        let grown = p.grown();
        for i in 0..=4 {
            for j in 0..=4 {
                let (s, t) = (i as f64 / 4.0, j as f64 / 4.0);
                for (k, part) in quarters.iter().enumerate() {
                    let at = [0.5 * ((k / 2) as f64 + s), 0.5 * ((k % 2) as f64 + t)];
                    assert!((part.eval([s, t]) - p.eval([at[0], at[1]])).abs() < 1e-15);
                }
                let at = [
                    -MARGIN + (1.0 + 2.0 * MARGIN) * s,
                    -MARGIN + (1.0 + 2.0 * MARGIN) * t,
                ];
                assert!((grown.eval([s, t]) - p.eval([at[0], at[1]])).abs() < 1e-14);
            }
        }
    }

    #[test]
    fn bernstein_n_the_turning_points_of_a_circle_and_its_crossings_with_a_line() {
        let b = Binomials::new(8);
        let f = circle(0.4, 0.55, 0.3, &b);
        let turning = common_zeros(
            [&f, &f.partial(1)],
            [1e-15; 2],
            &[],
            Halving::Scored,
            &mut Meter::default(),
        )
        .unwrap();
        assert_near(&certified(&turning), &[[0.1, 0.55], [0.7, 0.55]]);
        assert_eq!(turning.zeros.len(), 2);
        // s + t = 0.95 through the centre: (0.4 ∓ 0.3/√2, 0.55 ± 0.3/√2).
        let line = linear(-0.95, 1.0, 1.0);
        let d = 0.3 * core::f64::consts::FRAC_1_SQRT_2;
        let crossings = common_zeros(
            [&f, &line],
            [1e-15; 2],
            &[],
            Halving::Scored,
            &mut Meter::default(),
        )
        .unwrap();
        assert_near(
            &certified(&crossings),
            &[[0.4 - d, 0.55 + d], [0.4 + d, 0.55 - d]],
        );
        // A product of known factors: the zeros of each against the line.
        let g = f.mul(&circle(0.4, 0.55, 0.1, &b), &b);
        let e = 0.1 * core::f64::consts::FRAC_1_SQRT_2;
        let crossings = common_zeros(
            [&g, &line],
            [1e-15; 2],
            &[],
            Halving::Scored,
            &mut Meter::default(),
        )
        .unwrap();
        assert_near(
            &certified(&crossings),
            &[
                [0.4 - d, 0.55 + d],
                [0.4 - e, 0.55 + e],
                [0.4 + e, 0.55 - e],
                [0.4 + d, 0.55 - d],
            ],
        );
        assert!(crossings.depth < 16, "{}", crossings.depth);
    }

    #[test]
    fn bernstein_n_a_zero_on_a_line_between_boxes_or_on_the_edge_is_found_once() {
        let b = Binomials::new(8);
        // Turning points on s = ¼ and s = ¾, both on t = ½: corners of
        // boxes at every depth from the second.
        let f = circle(0.5, 0.5, 0.25, &b);
        let found = common_zeros(
            [&f, &f.partial(1)],
            [1e-15; 2],
            &[],
            Halving::Scored,
            &mut Meter::default(),
        )
        .unwrap();
        assert_near(&certified(&found), &[[0.25, 0.5], [0.75, 0.5]]);
        assert_eq!(found.zeros.len(), 2);
        assert!(found.depth < 16, "{}", found.depth);
        // On the square's own edge and corner, and one just outside it,
        // which the margin holds.
        let f = circle(0.25, 0.0, 0.25, &b);
        let found = common_zeros(
            [&f, &f.partial(1)],
            [1e-15; 2],
            &[],
            Halving::Scored,
            &mut Meter::default(),
        )
        .unwrap();
        assert_near(&certified(&found), &[[0.0, 0.0], [0.5, 0.0]]);
        // A zero outside the square is the neighbouring square's: found
        // from here where a margin happens to certify it, and no fault
        // where none does.
        let f = circle(0.249, 0.3, 0.25, &b);
        let found = certified(
            &common_zeros(
                [&f, &f.partial(1)],
                [1e-15; 2],
                &[],
                Halving::Scored,
                &mut Meter::default(),
            )
            .unwrap(),
        );
        let inside = found.iter().filter(|z| z[0] > 0.0).copied();
        assert_near(&inside.collect::<Vec<_>>(), &[[0.499, 0.3]]);
        let outside = found.iter().filter(|z| z[0] <= 0.0).copied();
        let outside: Vec<[f64; 2]> = outside.collect();
        assert!(outside.is_empty() || outside.len() == 1);
        assert_near(&outside, &vec![[-0.001, 0.3]; outside.len()]);
        let f = circle(-0.26, 0.3, 0.25, &b);
        let found = common_zeros(
            [&f, &f.partial(1)],
            [1e-15; 2],
            &[],
            Halving::Scored,
            &mut Meter::default(),
        )
        .unwrap();
        assert!(found.zeros.is_empty());
    }

    #[test]
    fn bernstein_n_a_singular_point_is_a_critical_point_certified_and_a_turning_point_not() {
        let b = Binomials::new(8);
        // (s − 0.3)² − (t − 0.6)²: two lines crossing.
        let (ds, dt) = (linear(-0.3, 1.0, 0.0), linear(-0.6, 0.0, 1.0));
        let f = PolyN::combine(&[(1.0, &ds.mul(&ds, &b)), (-1.0, &dt.mul(&dt, &b))]);
        let critical = common_zeros(
            [&f.partial(0), &f.partial(1)],
            [1e-15; 2],
            &[Gate {
                poly: &f,
                floor: 1e-15,
                one_sided: false,
            }],
            Halving::Scored,
            &mut Meter::default(),
        )
        .unwrap();
        assert_near(&certified(&critical), &[[0.3, 0.6]]);
        // As a zero of (f, f_t) it is not simple: a box, not a zero, and
        // the point is in it.
        let turning = common_zeros(
            [&f, &f.partial(1)],
            [1e-15; 2],
            &[],
            Halving::Scored,
            &mut Meter::default(),
        )
        .unwrap();
        assert!(certified(&turning).is_empty());
        assert_eq!(turning.zeros.len(), 1, "{:?}", turning.zeros);
        assert!(turning.zeros[0].holds([0.3, 0.6], [0.0, 0.0]));
        assert!(turning.zeros[0].volume() < 1e-12);
        // The gate drops a critical point off the curve, and with a
        // floor above the value there keeps it.
        let lifted = PolyN::combine(&[(1.0, &f), (1e-3, &PolyN::outer([&[1.0; 3], &[1.0; 3]]))]);
        let gated = |floor: f64| {
            common_zeros(
                [&lifted.partial(0), &lifted.partial(1)],
                [1e-15; 2],
                &[Gate {
                    poly: &lifted,
                    floor,
                    one_sided: false,
                }],
                Halving::Scored,
                &mut Meter::default(),
            )
            .unwrap()
        };
        assert!(gated(1e-15).zeros.is_empty());
        assert_near(&certified(&gated(2e-3)), &[[0.3, 0.6]]);
    }

    #[test]
    fn bernstein_n_a_curve_of_common_zeros_is_a_continuum() {
        let b = Binomials::new(8);
        // (s − 0.3)(1 + t²): f and f_t both vanish along s = 0.3.
        let t = linear(0.0, 0.0, 1.0);
        let one = PolyN::outer([&[1.0; 3], &[1.0; 3]]);
        let bowl = PolyN::combine(&[(1.0, &one), (1.0, &t.mul(&t, &b))]);
        let f = linear(-0.3, 1.0, 0.0).mul(&bowl, &b);
        let found = common_zeros(
            [&f, &f.partial(1)],
            [1e-15; 2],
            &[],
            Halving::Scored,
            &mut Meter::default(),
        );
        assert!(
            matches!(found, Err(Halt::Fault(Continuum { .. }))),
            "{found:?}"
        );
        // The zero polynomial is flat at once: one box, the square.
        let zero = PolyN::outer([&[0.0; 3], &[0.0; 3]]);
        let found = common_zeros(
            [&zero, &f],
            [1e-15; 2],
            &[],
            Halving::Scored,
            &mut Meter::default(),
        )
        .unwrap();
        assert_eq!(found.zeros.len(), 1);
        assert!(!found.zeros[0].certified);
        assert_eq!(found.boxes, 1);
    }

    #[test]
    fn bernstein_n_zeros_found_twice_across_a_seam_are_one() {
        let zero = |at: [f64; 2], half: f64, certified: bool| Zero {
            at,
            lo: [at[0] - half, at[1] - half],
            hi: [at[0] + half, at[1] + half],
            certified,
        };
        let merged = merge(
            vec![
                zero([6.25, 1.0], 0.1, true),
                zero([6.25 - 6.0, 1.0], 0.05, true),
                zero([-0.01, 3.0], 0.2, true),
                zero([5.99, 3.0], 0.05, true),
                zero([5.9, 3.05], 0.01, false),
                zero([0.05, 2.0], 0.05, false),
                zero([5.95, 2.0], 0.05, false),
            ],
            &Wrap {
                period: [Some(6.0); 2],
                origin: [0.0; 2],
                touch: 1e-12,
            },
        );
        let at: Vec<[f64; 2]> = merged.iter().map(|z| z.at).collect();
        assert_eq!(merged.len(), 3, "{merged:?}");
        assert!((at[0][0] - 0.0).abs() < 1e-12 && at[0][1] == 2.0, "{at:?}");
        assert!(!merged[0].certified);
        assert!((merged[0].hi[0] - merged[0].lo[0] - 0.2).abs() < 1e-12);
        assert!((at[1][0] - 0.25).abs() < 1e-12 && at[1][1] == 1.0, "{at:?}");
        assert_eq!(at[2], [5.99, 3.0]);
    }
}
