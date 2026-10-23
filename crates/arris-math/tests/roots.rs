//! The root finder recovers real roots to the accuracy their coefficients
//! determine, reports no real root for complex pairs, finds a double root
//! once with multiplicity two, and the bracketed Newton never leaves its
//! bracket.

use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64};
use arris_debug::testing::EXACT;
use arris_math::Interval;
use arris_math::roots::{
    POLYNOMIAL_ROUNDING, Roots, cubic, newton_in_interval, quadratic, quartic,
};
use proptest::collection::vec;
use proptest::prelude::*;

/// The floor on a recovered root's error.
/// Roots (and real parts, and imaginary parts) are built at least this
/// far apart, so the polynomial can tell them apart.
const SEPARATION: f64 = 1e-3 * DEFAULT_SCALE;
/// The bracket tolerance handed to Newton.
const NEWTON_TOL: f64 = 1e-13 * DEFAULT_SCALE;

/// Ascending coefficients of the monic polynomial with these real roots
/// and complex pairs `re ± i·im`.
fn expand(real: &[f64], pairs: &[(f64, f64)]) -> Vec<f64> {
    let mut c = vec![1.0];
    let mut multiply = |factor: &[f64]| {
        let mut next = vec![0.0; c.len() + factor.len() - 1];
        for (i, a) in c.iter().enumerate() {
            for (j, b) in factor.iter().enumerate() {
                next[i + j] += a * b;
            }
        }
        c = next;
    };
    for r in real {
        multiply(&[-r, 1.0]);
    }
    for (re, im) in pairs {
        multiply(&[re * re + im * im, -2.0 * re, 1.0]);
    }
    c
}

fn eval(c: &[f64], x: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, &k| acc * x + k)
}

fn eval_abs(c: &[f64], x: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, &k| acc * x.abs() + k.abs())
}

fn derivative(c: &[f64]) -> Vec<f64> {
    c.iter()
        .enumerate()
        .skip(1)
        .map(|(k, a)| a * k as f64)
        .collect()
}

/// The forward error a backward-stable root finder is allowed on a
/// simple root `r` of `c`: the rounding of `p(r)` over the slope there,
/// plus the plan's floor. This is the coefficient representation's own
/// conditioning, not the method's.
fn simple_root_bound(c: &[f64], r: f64) -> f64 {
    let slope = eval(&derivative(c), r).abs();
    EXACT + 4.0 * POLYNOMIAL_ROUNDING * eval_abs(c, r) / slope
}

/// The bound for a double root, which is located as a simple root of
/// `p'`.
fn double_root_bound(c: &[f64], r: f64) -> f64 {
    simple_root_bound(&derivative(c), r)
}

fn solve(c: &[f64]) -> Roots {
    match c.len() {
        3 => quadratic(c[2], c[1], c[0]),
        4 => cubic(c[3], c[2], c[1], c[0]),
        5 => quartic(c[4], c[3], c[2], c[1], c[0]),
        _ => unreachable!(),
    }
    .expect("finite coefficients")
}

fn separated(values: &[f64]) -> bool {
    values
        .iter()
        .enumerate()
        .all(|(i, a)| values[i + 1..].iter().all(|b| (a - b).abs() >= SEPARATION))
}

fn coordinate() -> impl Strategy<Value = f64> {
    finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE)
}

fn separated_roots(n: core::ops::RangeInclusive<usize>) -> impl Strategy<Value = Vec<f64>> {
    vec(coordinate(), n).prop_filter("roots separated", |v| separated(v))
}

#[test]
fn separated_real_roots_are_recovered_with_multiplicity_one() {
    check(separated_roots(2..=4), |mut roots| {
        let c = expand(&roots, &[]);
        let found = solve(&c);
        roots.sort_by(f64::total_cmp);
        prop_assert_eq!(found.len(), roots.len(), "{:?} from {:?}", found, roots);
        for (f, r) in found.iter().zip(&roots) {
            prop_assert_eq!(f.multiplicity, 1);
            let bound = simple_root_bound(&c, *r);
            prop_assert!(
                (f.value - r).abs() <= bound,
                "root {r} found as {} (off by {}, allowed {bound})",
                f.value,
                (f.value - r).abs()
            );
        }
        Ok(())
    });
}

fn pair() -> impl Strategy<Value = (f64, f64)> {
    (coordinate(), finite_f64(SEPARATION..=DEFAULT_SCALE))
}

#[test]
fn complex_pairs_have_no_real_root() {
    check(vec(pair(), 1..=2), |pairs| {
        let c = expand(&[], &pairs);
        let found = solve(&c);
        prop_assert!(found.is_empty(), "{:?} from pairs {:?}", found, pairs);
        Ok(())
    });
}

#[test]
fn a_pair_beside_real_roots_leaves_only_the_real_ones() {
    check((separated_roots(1..=2), pair()), |(mut roots, pair)| {
        let c = expand(&roots, &[pair]);
        let found = solve(&c);
        roots.sort_by(f64::total_cmp);
        prop_assert_eq!(
            found.len(),
            roots.len(),
            "{:?} from {:?} + {:?}",
            found,
            roots,
            pair
        );
        for (f, r) in found.iter().zip(&roots) {
            prop_assert_eq!(f.multiplicity, 1);
            prop_assert!((f.value - r).abs() <= simple_root_bound(&c, *r));
        }
        Ok(())
    });
}

#[test]
fn a_double_root_is_found_once_with_multiplicity_two() {
    check(separated_roots(1..=3), |roots| {
        let (double, others) = (roots[0], &roots[1..]);
        let mut all = vec![double, double];
        all.extend_from_slice(others);
        let c = expand(&all, &[]);
        let found = solve(&c);
        prop_assert_eq!(
            found.total_multiplicity(),
            all.len(),
            "{:?} from {:?}",
            found,
            all
        );
        prop_assert_eq!(found.len(), others.len() + 1);
        let twice = found
            .iter()
            .find(|f| f.multiplicity == 2)
            .ok_or_else(|| TestCaseError::fail(format!("no double root in {found:?}")))?;
        let bound = double_root_bound(&c, double);
        prop_assert!(
            (twice.value - double).abs() <= bound,
            "double root {double} found as {} (allowed {bound})",
            twice.value
        );
        for r in others {
            let hit = found
                .iter()
                .find(|f| f.multiplicity == 1 && (f.value - r).abs() <= simple_root_bound(&c, *r));
            prop_assert!(hit.is_some(), "simple root {r} missing from {:?}", found);
        }
        Ok(())
    });
}

#[test]
fn newton_converges_inside_a_sign_changing_bracket() {
    let bracket = (coordinate(), coordinate()).prop_map(|(a, b)| (a.min(b), a.max(b)));
    check(
        (separated_roots(3..=3), bracket).prop_filter("sign change", |(roots, (lo, hi))| {
            let c = expand(roots, &[]);
            (eval(&c, *lo) < 0.0) != (eval(&c, *hi) < 0.0) && eval(&c, *lo) != 0.0
        }),
        |(roots, (lo, hi))| {
            let c = expand(&roots, &[]);
            let d = derivative(&c);
            let x = newton_in_interval(
                |x| eval(&c, x),
                |x| eval(&d, x),
                Interval::new(lo, hi).unwrap(),
                NEWTON_TOL,
            )
            .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert!(lo <= x && x <= hi, "{x} left [{lo}, {hi}]");
            let nearest = roots
                .iter()
                .copied()
                .min_by(|a, b| (a - x).abs().total_cmp(&(b - x).abs()))
                .unwrap();
            let bound = NEWTON_TOL + simple_root_bound(&c, nearest);
            prop_assert!(
                (x - nearest).abs() <= bound,
                "{x} is {} from the nearest root {nearest}, allowed {bound}",
                (x - nearest).abs()
            );
            Ok(())
        },
    );
}

#[test]
fn newton_refuses_a_bracket_without_a_sign_change() {
    let bracket = (coordinate(), coordinate()).prop_map(|(a, b)| (a.min(b), a.max(b)));
    check(
        (separated_roots(3..=3), bracket).prop_filter("same sign", |(roots, (lo, hi))| {
            let c = expand(roots, &[]);
            (eval(&c, *lo) < 0.0) == (eval(&c, *hi) < 0.0)
                && eval(&c, *lo) != 0.0
                && eval(&c, *hi) != 0.0
        }),
        |(roots, (lo, hi))| {
            let c = expand(&roots, &[]);
            let r = newton_in_interval(
                |x| eval(&c, x),
                |_| 1.0,
                Interval::new(lo, hi).unwrap(),
                NEWTON_TOL,
            );
            prop_assert_eq!(r, Err(arris_math::roots::RootError::NoSignChange));
            Ok(())
        },
    );
}
