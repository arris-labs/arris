//! Property-test configuration and numeric strategies.
//!
//! Every property test in the workspace runs through [`check`]: a seeded
//! `proptest` runner whose case count comes from `ARRIS_PROPTEST_CASES`
//! (default 256) and whose seed comes from `ARRIS_PROPTEST_SEED` (default: a
//! fixed one, so CI is deterministic — `.agents/rules/kernel.md`: no
//! randomness outside property tests, and those are seeded). A failure
//! panics with the shrunk input and the seed that reproduces it.
//!
//! A property whose cases are expensive is *sharded*:
//! [`prop_shards!`](crate::prop_shards)
//! writes one `#[test]` per shard over a body given once, so libtest's
//! thread pool runs the shards concurrently instead of one property
//! sitting on one thread. The `k` shards together run [`cases`] cases and
//! never fewer, each from its own [`shard_seed`]; the derivation ignores
//! `k`, so raising a property's shard count shortens every existing
//! shard's stream to a prefix of what it was rather than re-rolling the
//! corpus.
//!
//! Regressions are not persisted by proptest: a failure becomes a fixture
//! under `tests/fixtures/` with the seed in its commit body
//! (`tests/fixtures/README.md` §Property-test failures).
//!
//! The geometric strategies produce `arris-math` types in random poses;
//! every one is uniform over its space so a property that holds "at 1000
//! cases" has seen the seams, the poles and the octants. `geom` places
//! every surface and curve kind and random NURBS; `body` describes boxes,
//! cylinders and the revolved or extruded quadric solids in random poses
//! that a test builds into a model;
//! `profile` draws sketches, with an axis and the sweep parameters for
//! them, `sweep` is the Pappus oracle a sweep's volume and area are
//! held to, and `recipe` draws whole corpus recipes that both kernels
//! build (ADR-0024 §2).
//!
//! ```
//! use arris_debug::prop::{check, frame, point_in_box, DEFAULT_SCALE};
//! use proptest::prelude::*;
//!
//! check((frame(), point_in_box(DEFAULT_SCALE)), |(f, p)| {
//!     let back = f.to_world(f.to_local(p));
//!     prop_assert!((back - p).norm() <= 1e-12 * DEFAULT_SCALE);
//!     Ok(())
//! });
//! ```

pub mod body;
pub mod geom;
pub mod profile;
pub mod recipe;
pub mod sweep;
pub mod turned;

use core::fmt::Debug;
use core::ops::RangeInclusive;

use arris_math::nalgebra::{Quaternion, UnitQuaternion};
use arris_math::{Frame, Isometry, Point3, UnitVec3, Vec3};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngAlgorithm, TestCaseError, TestError, TestRng, TestRunner};
use sha2::{Digest, Sha256};

/// The environment variable that sets the number of cases per property.
pub const CASES_VAR: &str = "ARRIS_PROPTEST_CASES";
/// The environment variable that sets the seed: 64 hex digits, as printed
/// by a failure.
pub const SEED_VAR: &str = "ARRIS_PROPTEST_SEED";
/// Cases per property when [`CASES_VAR`] is unset.
pub const DEFAULT_CASES: u32 = 256;
/// The seed when [`SEED_VAR`] is unset. Arbitrary and fixed.
pub const DEFAULT_SEED: [u8; 32] = *b"arris-property-tests-seed-v1\0\0\0\0";
/// The half-width of the box [`frame`], [`pose`] and the geometry
/// strategies place their origins in. A test's tolerance is stated
/// relative to it: `1e-12 * DEFAULT_SCALE` is "1e-12·scale".
pub const DEFAULT_SCALE: f64 = 100.0;

/// The number of cases: [`CASES_VAR`], or [`DEFAULT_CASES`].
pub fn cases() -> u32 {
    cases_from(std::env::var(CASES_VAR).ok().as_deref())
}

fn cases_from(value: Option<&str>) -> u32 {
    value
        .and_then(|v| v.trim().parse().ok())
        .filter(|&n| n > 0)
        .unwrap_or(DEFAULT_CASES)
}

/// The seed: [`SEED_VAR`] as 64 hex digits, or [`DEFAULT_SEED`]. A value
/// that does not parse is an error, not a silent fallback.
pub fn seed() -> [u8; 32] {
    match std::env::var(SEED_VAR) {
        Ok(v) => parse_seed(&v).unwrap_or_else(|| panic!("{SEED_VAR}={v:?} is not 64 hex digits")),
        Err(_) => DEFAULT_SEED,
    }
}

fn parse_seed(text: &str) -> Option<[u8; 32]> {
    let text = text.trim();
    if text.len() != 64 {
        return None;
    }
    let mut out = [0u8; 32];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&text[2 * i..2 * i + 2], 16).ok()?;
    }
    Some(out)
}

/// The seed as the hex string [`SEED_VAR`] accepts.
pub fn seed_hex(seed: &[u8; 32]) -> String {
    seed.iter().map(|b| format!("{b:02x}")).collect()
}

/// The proptest configuration: [`cases`] cases, no failure persistence,
/// no forking. For `proptest!` blocks that want it:
/// `#![proptest_config(arris_debug::prop::config())]` — but such a block
/// seeds itself from entropy; [`check`] is the seeded path.
pub fn config() -> Config {
    config_of(cases())
}

/// [`config`] at an explicit case count, which is what one shard runs.
fn config_of(cases: u32) -> Config {
    Config {
        cases,
        failure_persistence: None,
        max_shrink_iters: 4096,
        ..Config::default()
    }
}

/// A runner over [`config`] seeded with `seed`.
pub fn runner_with_seed(seed: &[u8; 32]) -> TestRunner {
    runner_of(seed, cases())
}

/// A runner over `cases` cases seeded with `seed`.
fn runner_of(seed: &[u8; 32], cases: u32) -> TestRunner {
    TestRunner::new_with_rng(
        config_of(cases),
        TestRng::from_seed(RngAlgorithm::ChaCha, seed),
    )
}

/// Runs `test` over [`cases`] values of `strategy` from [`seed`]. On
/// failure, panics with the shrunk input and the `ARRIS_PROPTEST_SEED=…`
/// that reproduces the run.
pub fn check<S, F>(strategy: S, test: F)
where
    S: Strategy,
    S::Value: Debug,
    F: Fn(S::Value) -> Result<(), TestCaseError>,
{
    let seed = seed();
    if let Err(message) = try_check(&seed, strategy, test) {
        panic!("{message}");
    }
}

/// [`check`] with an explicit seed, returning the failure message instead
/// of panicking. What a test of the harness itself uses.
pub fn try_check<S, F>(seed: &[u8; 32], strategy: S, test: F) -> Result<(), String>
where
    S: Strategy,
    S::Value: Debug,
    F: Fn(S::Value) -> Result<(), TestCaseError>,
{
    let mut runner = runner_with_seed(seed);
    match runner.run(&strategy, test) {
        Ok(()) => Ok(()),
        Err(TestError::Fail(reason, value)) => Err(format!(
            "property failed: {reason}\nminimal failing input: {value:#?}\nreproduce with {SEED_VAR}={} {CASES_VAR}={}",
            seed_hex(seed),
            cases()
        )),
        Err(TestError::Abort(reason)) => Err(format!("property aborted: {reason}")),
    }
}

/// The seed shard `shard` draws its cases from: `sha256(base ‖
/// shard.to_le_bytes())`.
///
/// Guarantees: the shard *count* is not in the derivation. Shard `i`'s
/// case stream depends only on `base` and `i`, so raising a property's
/// shard count leaves every existing shard's stream a prefix of what it
/// was and only adds new shards — changing `k` is not a silent re-roll of
/// the corpus.
///
/// ```
/// use arris_debug::prop::{DEFAULT_SEED, shard_seed};
///
/// assert_ne!(shard_seed(&DEFAULT_SEED, 0), shard_seed(&DEFAULT_SEED, 1));
/// assert_eq!(shard_seed(&DEFAULT_SEED, 2), shard_seed(&DEFAULT_SEED, 2));
/// ```
pub fn shard_seed(base: &[u8; 32], shard: u32) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(base);
    hasher.update(shard.to_le_bytes());
    hasher.finalize().into()
}

/// The cases one of `shards` shards runs: [`cases`] over `shards`,
/// rounded up.
///
/// Guarantees: `shards * cases_per_shard(shards) >= cases()`, so a
/// sharded property never runs fewer cases than the configured count —
/// a remainder rounds up rather than being dropped. Panics when `shards`
/// is zero.
///
/// ```
/// use arris_debug::prop::{cases, cases_per_shard};
///
/// assert!(7 * cases_per_shard(7) >= cases());
/// assert_eq!(cases_per_shard(1), cases());
/// ```
pub fn cases_per_shard(shards: u32) -> u32 {
    assert!(shards > 0, "a property has at least one shard");
    cases().div_ceil(shards)
}

/// Runs `test` over shard `shard` of `shards`: [`cases_per_shard`]
/// values of `strategy` from [`shard_seed`] of [`seed`]. On failure,
/// panics with the shrunk input, the shard, and the
/// `ARRIS_PROPTEST_SEED=…` *base* seed that reproduces the whole run.
///
/// Written through [`prop_shards!`](crate::prop_shards) rather than called
/// directly, so the
/// shard index and the shard count cannot disagree.
pub fn check_shard<S, F>(shard: u32, shards: u32, strategy: S, test: F)
where
    S: Strategy,
    S::Value: Debug,
    F: Fn(S::Value) -> Result<(), TestCaseError>,
{
    let base = seed();
    if let Err(message) = try_check_shard(&base, shard, shards, strategy, test) {
        panic!("{message}");
    }
}

/// [`check_shard`] with an explicit base seed, returning the failure
/// message instead of panicking. What a test of the harness itself uses.
pub fn try_check_shard<S, F>(
    base: &[u8; 32],
    shard: u32,
    shards: u32,
    strategy: S,
    test: F,
) -> Result<(), String>
where
    S: Strategy,
    S::Value: Debug,
    F: Fn(S::Value) -> Result<(), TestCaseError>,
{
    assert!(
        shard < shards,
        "shard {shard} is not one of {shards} shards"
    );
    let mut runner = runner_of(&shard_seed(base, shard), cases_per_shard(shards));
    match runner.run(&strategy, test) {
        Ok(()) => Ok(()),
        Err(TestError::Fail(reason, value)) => Err(format!(
            "property failed in shard {shard} of {shards}: {reason}\nminimal failing input: {value:#?}\nreproduce the whole run with {SEED_VAR}={} {CASES_VAR}={} — the base seed and the total, not the shard's",
            seed_hex(base),
            cases()
        )),
        Err(TestError::Abort(reason)) => Err(format!(
            "property aborted in shard {shard} of {shards}: {reason}"
        )),
    }
}

/// Writes one `#[test]` per shard of a property, over a body given once.
///
/// Guarantees: the `k` tests together run at least
/// [`prop::cases`](crate::prop::cases) cases of the strategy, each shard from
/// its own [`shard_seed`](crate::prop::shard_seed) of
/// [`seed`](crate::prop::seed), and `k` is
/// the length of the list of shard names. The tests go in a module named
/// after the property, so a failure reads `property::shard_3` and names
/// the shard it is; the message it panics with names the base seed that
/// reproduces the whole run.
///
/// The shards are named rather than numbered because `macro_rules!`
/// cannot build an ident out of a number, and the workspace is not
/// taking a `seq-macro` dependency to let it: a shard's index is its
/// name's position in the list, and the count is the list's length, so
/// both are visible where the property is written.
///
/// ```
/// use arris_debug::prop;
/// use proptest::prelude::*;
///
/// arris_debug::prop_shards! {
///     /// Four shards of one property, run by libtest concurrently.
///     drawn_values_stay_in_range [shard_0 shard_1 shard_2 shard_3]
///         (v) = prop::finite_f64(0.0..=1.0) => {
///             prop_assert!((0.0..=1.0).contains(&v));
///             Ok(())
///         }
/// }
/// ```
#[macro_export]
macro_rules! prop_shards {
    (
        $(#[$meta:meta])*
        $name:ident [$($shard:ident)+] ($arg:pat_param) = $strategy:expr => $body:block
    ) => {
        $(#[$meta])*
        mod $name {
            #[allow(unused_imports)]
            use super::*;

            $crate::prop_shards!(
                @shards (0u32 $(+ $crate::prop_shards!(@one $shard))+), 0u32,
                [$($shard)+], ($arg) = $strategy => $body
            );
        }
    };
    (@one $shard:ident) => { 1u32 };
    (@shards $shards:expr, $index:expr, [], ($arg:pat_param) = $strategy:expr => $body:block) => {};
    (
        @shards $shards:expr, $index:expr, [$first:ident $($rest:ident)*],
        ($arg:pat_param) = $strategy:expr => $body:block
    ) => {
        #[test]
        fn $first() {
            $crate::prop::check_shard($index, $shards, $strategy, |$arg| $body);
        }

        $crate::prop_shards!(
            @shards $shards, $index + 1u32, [$($rest)*], ($arg) = $strategy => $body
        );
    };
}

/// Finite `f64` values in `range`, both ends included; never NaN or
/// infinite. Shrinks toward zero (or the range end nearest it).
pub fn finite_f64(range: RangeInclusive<f64>) -> impl Strategy<Value = f64> {
    let (lo, hi) = (*range.start(), *range.end());
    assert!(
        lo.is_finite() && hi.is_finite() && lo <= hi,
        "finite_f64 needs a finite, ordered range"
    );
    (lo..=hi).prop_filter("finite", |v| v.is_finite())
}

/// Unit vectors uniformly distributed on the sphere, with length within
/// 1e-15 of one. Uses the area-preserving map from `(z, θ)` uniform in
/// `[−1, 1] × [0, 2π)`, then normalises.
pub fn unit_vec3() -> impl Strategy<Value = UnitVec3> {
    (
        finite_f64(-1.0..=1.0),
        finite_f64(0.0..=core::f64::consts::TAU),
    )
        .prop_map(|(z, theta)| {
            let r = (1.0 - z * z).max(0.0).sqrt();
            UnitVec3::new_normalize(Vec3::new(r * theta.cos(), r * theta.sin(), z))
        })
}

/// Rotations uniformly distributed over SO(3) (Shoemake's subgroup
/// algorithm over three uniforms), as unit quaternions with norm within
/// 1e-15 of one.
pub fn rotation() -> impl Strategy<Value = UnitQuaternion<f64>> {
    (
        finite_f64(0.0..=1.0),
        finite_f64(0.0..=1.0),
        finite_f64(0.0..=1.0),
    )
        .prop_map(|(u1, u2, u3)| {
            let (a, b) = ((1.0 - u1).sqrt(), u1.sqrt());
            let (t2, t3) = (core::f64::consts::TAU * u2, core::f64::consts::TAU * u3);
            UnitQuaternion::new_normalize(Quaternion::new(
                b * t3.cos(),
                a * t2.sin(),
                a * t2.cos(),
                b * t3.sin(),
            ))
        })
}

/// Points with every coordinate in `[−scale, scale]`, uniform in the box.
/// `scale` must be finite and positive.
pub fn point_in_box(scale: f64) -> impl Strategy<Value = Point3> {
    assert!(
        scale.is_finite() && scale > 0.0,
        "point_in_box needs a finite, positive scale"
    );
    (
        finite_f64(-scale..=scale),
        finite_f64(-scale..=scale),
        finite_f64(-scale..=scale),
    )
        .prop_map(|(x, y, z)| Point3::new(x, y, z))
}

/// Positive radii in `range`, both ends included; `range` must start
/// above zero.
pub fn radius(range: RangeInclusive<f64>) -> impl Strategy<Value = f64> {
    assert!(*range.start() > 0.0, "radius needs a positive range");
    finite_f64(range)
}

/// Frames with a uniformly random orientation and an origin uniform in
/// the box of half-width `scale`.
pub fn frame_in(scale: f64) -> impl Strategy<Value = Frame> {
    (point_in_box(scale), rotation()).prop_map(|(origin, q)| Frame::from_rotation(origin, &q))
}

/// [`frame_in`] at [`DEFAULT_SCALE`].
pub fn frame() -> impl Strategy<Value = Frame> {
    frame_in(DEFAULT_SCALE)
}

/// Rigid motions with a uniformly random rotation and a translation
/// uniform in the box of half-width `scale`.
pub fn pose_in(scale: f64) -> impl Strategy<Value = Isometry> {
    (rotation(), point_in_box(scale)).prop_map(|(q, t)| Isometry::new(q, t.coords))
}

/// [`pose_in`] at [`DEFAULT_SCALE`].
pub fn pose() -> impl Strategy<Value = Isometry> {
    pose_in(DEFAULT_SCALE)
}

#[cfg(test)]
crate::prop_shards! {
    /// The macro's own property: two shards, each a real `#[test]`,
    /// together covering the configured cases.
    the_macro_writes_one_test_per_shard [shard_0 shard_1] (v) =
        finite_f64(0.0..=1.0) => {
            prop_assert!((0.0..=1.0).contains(&v));
            Ok(())
        }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::strategy::ValueTree;

    /// Every value shard `shard` of `shards` draws from `base`, in order.
    /// The property holds, so nothing is shrunk and the recorded sequence
    /// is exactly the shard's cases.
    fn drawn(base: &[u8; 32], shard: u32, shards: u32) -> Vec<f64> {
        let seen = std::cell::RefCell::new(Vec::new());
        try_check_shard(base, shard, shards, finite_f64(0.0..=1.0), |v| {
            seen.borrow_mut().push(v);
            Ok(())
        })
        .expect("the property holds");
        seen.into_inner()
    }

    #[test]
    fn one_shard_runs_every_configured_case() {
        assert_eq!(drawn(&DEFAULT_SEED, 0, 1).len(), cases() as usize);
    }

    #[test]
    fn shards_of_one_base_draw_disjoint_streams() {
        let a = drawn(&DEFAULT_SEED, 0, 8);
        let b = drawn(&DEFAULT_SEED, 1, 8);
        assert!(!a.is_empty() && !b.is_empty());
        assert!(
            a.iter().all(|x| !b.contains(x)),
            "two shards of one base drew the same value"
        );
    }

    #[test]
    fn a_shard_at_a_higher_count_is_a_prefix_of_itself_at_a_lower_one() {
        // The shard-seed derivation ignores the shard count, so raising k
        // only shortens each existing shard's stream — it never re-rolls
        // the corpus.
        for shard in 0..8 {
            let eight = drawn(&DEFAULT_SEED, shard, 8);
            let sixteen = drawn(&DEFAULT_SEED, shard, 16);
            assert!(sixteen.len() <= eight.len());
            assert_eq!(sixteen[..], eight[..sixteen.len()], "shard {shard}");
        }
    }

    #[test]
    fn shards_together_run_at_least_the_configured_cases() {
        // Including counts that do not divide the case count, where the
        // remainder has to round up rather than be dropped.
        for shards in [1u32, 3, 5, 8, 16] {
            let total: usize = (0..shards)
                .map(|i| drawn(&DEFAULT_SEED, i, shards).len())
                .sum();
            assert!(
                total >= cases() as usize,
                "{shards} shards ran {total} of {} cases",
                cases()
            );
        }
    }

    #[test]
    fn a_shard_failure_names_the_shard_and_the_base_seed() {
        let failing = |v: f64| {
            prop_assert!(v < 0.5, "too large");
            Ok(())
        };
        let message =
            try_check_shard(&DEFAULT_SEED, 3, 8, finite_f64(0.0..=1.0), failing).unwrap_err();
        assert!(message.contains("shard 3 of 8"), "{message}");
        let printed = message
            .lines()
            .find_map(|l| l.split_once(&format!("{SEED_VAR}=")))
            .and_then(|(_, rest)| rest.split_whitespace().next())
            .expect("the message names the seed");
        assert_eq!(
            parse_seed(printed),
            Some(DEFAULT_SEED),
            "the base seed, not the shard's"
        );
        let again =
            try_check_shard(&DEFAULT_SEED, 3, 8, finite_f64(0.0..=1.0), failing).unwrap_err();
        assert_eq!(message, again, "same seed, same shrunk input, same message");
    }

    #[test]
    fn shard_seeds_differ_from_each_other_and_from_the_base() {
        let seeds: std::collections::BTreeSet<_> =
            (0..32).map(|i| shard_seed(&DEFAULT_SEED, i)).collect();
        assert_eq!(seeds.len(), 32);
        assert!(!seeds.contains(&DEFAULT_SEED));
        assert_eq!(shard_seed(&DEFAULT_SEED, 2), shard_seed(&DEFAULT_SEED, 2));
    }

    #[test]
    fn cases_per_shard_rounds_the_remainder_up() {
        assert_eq!(cases_per_shard(1), cases());
        for shards in 1..=64u32 {
            assert!(shards * cases_per_shard(shards) >= cases());
        }
    }

    #[test]
    fn unit_vec3_has_unit_length() {
        check(unit_vec3(), |v| {
            let len = v.norm();
            prop_assert!((len - 1.0).abs() <= 1e-15, "length {len}");
            Ok(())
        });
    }

    #[test]
    fn rotation_is_a_unit_quaternion() {
        check(rotation(), |q| {
            let n = q.norm();
            prop_assert!((n - 1.0).abs() <= 1e-15, "norm {n}");
            Ok(())
        });
    }

    #[test]
    fn finite_f64_stays_in_range_and_finite() {
        check(finite_f64(-1e3..=1e3), |v| {
            prop_assert!(v.is_finite() && (-1e3..=1e3).contains(&v));
            Ok(())
        });
    }

    #[test]
    fn boxes_radii_and_poses_stay_in_their_ranges() {
        check(
            (
                point_in_box(3.0),
                radius(0.5..=2.0),
                pose_in(3.0),
                frame_in(3.0),
            ),
            |(p, r, m, f)| {
                prop_assert!(p.coords.iter().all(|c| c.abs() <= 3.0));
                prop_assert!((0.5..=2.0).contains(&r));
                prop_assert!(m.translation().iter().all(|c| c.abs() <= 3.0));
                prop_assert!(f.origin().coords.iter().all(|c| c.abs() <= 3.0));
                Ok(())
            },
        );
    }

    #[test]
    fn unit_vec3_covers_every_octant() {
        // Uniform on the sphere: over the configured cases every octant is
        // hit. A strategy that always returned the same vector would pass
        // the length test and fail here.
        let mut runner = runner_with_seed(&DEFAULT_SEED);
        let mut octants = std::collections::BTreeSet::new();
        for _ in 0..cases().max(64) {
            let v = unit_vec3().new_tree(&mut runner).unwrap().current();
            octants.insert((v.x > 0.0, v.y > 0.0, v.z > 0.0));
        }
        assert_eq!(octants.len(), 8);
    }

    #[test]
    fn a_failure_prints_a_seed_that_reproduces_it() {
        let failing = |v: f64| {
            prop_assert!(v < 0.5, "too large");
            Ok(())
        };
        let seed = DEFAULT_SEED;
        let message = try_check(&seed, finite_f64(0.0..=1.0), failing).unwrap_err();
        assert!(message.contains("too large"), "{message}");
        let printed = message
            .lines()
            .find_map(|l| l.strip_prefix("reproduce with ARRIS_PROPTEST_SEED="))
            .and_then(|rest| rest.split_whitespace().next())
            .expect("the message names the seed");
        let reparsed = parse_seed(printed).expect("the printed seed parses");
        assert_eq!(reparsed, seed);
        let again = try_check(&reparsed, finite_f64(0.0..=1.0), failing).unwrap_err();
        assert_eq!(message, again, "same seed, same shrunk input, same message");
        let other = try_check(&[7u8; 32], finite_f64(0.0..=1.0), failing).unwrap_err();
        assert!(other.contains("minimal failing input"));
    }

    #[test]
    fn cases_and_seed_parse_or_fall_back() {
        assert_eq!(cases_from(None), DEFAULT_CASES);
        assert_eq!(cases_from(Some("1000")), 1000);
        assert_eq!(cases_from(Some("0")), DEFAULT_CASES);
        assert_eq!(cases_from(Some("lots")), DEFAULT_CASES);
        assert_eq!(parse_seed(&seed_hex(&DEFAULT_SEED)), Some(DEFAULT_SEED));
        assert_eq!(parse_seed("abc"), None);
        assert_eq!(parse_seed(&"zz".repeat(32)), None);
    }
}
