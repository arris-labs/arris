//! The STEP write → read round trip as a property (ADR-0025):
//! a frustum, a ball, a ring or an elliptic
//! prism from `prop::body` in a random pose, alone or cut, fused or
//! intersected with a box or a cylinder through it — so the files carry
//! fitted section curves, raised tolerances, seams, poles and apexes —
//! written and read back is a body the checker passes at `Full` with
//! nothing unchecked, with the same counts, the same volume, area,
//! centroid and inertia within the entities' own tolerance bound
//! (ADR-0023's first-order form), and the same classification of random
//! probe points. A failure prints the shrunk case and the seed, and
//! becomes a fixture (`tests/fixtures/README.md` §Property-test failures).

use arris_debug::corpus::within_own_tolerance;
use arris_debug::fixtures::Tolerances;
use arris_debug::prop::body::{QuadricPair, QuadricSolid};
use arris_debug::testing::{REL, fail};
use arris_debug::unmetered::step_read;
use arris_debug::unmetered::{common, cut, fuse};
use arris_debug::unmetered::{mass_properties, transform};
use arris_debug::{prop, prop_shards};
use arris_io::arris_check::arris_topo::arris_math::{Isometry, Point3, Vec3};
use arris_io::arris_check::arris_topo::{Body, Model};
use arris_io::arris_check::classify::{Classification, classify_point};
use arris_io::arris_check::{Level, check};
use arris_io::step::{self, ReadOptions};
use arris_ops::measure::MassProperties;
use proptest::prelude::*;

/// What is written: a solid alone, or a boolean of a pair.
#[derive(Debug, Clone, Copy)]
enum Shape {
    /// A quadric solid under a motion.
    Alone(QuadricSolid, Isometry),
    /// `fuse` (0), `common` (1) or `cut` (2) of a pair.
    Boolean(u8, QuadricPair),
}

impl Shape {
    fn build(&self, m: &mut Model) -> Result<Body, TestCaseError> {
        match *self {
            Shape::Alone(solid, pose) => {
                let body = solid.build(m).map_err(fail)?;
                Ok(transform(m, body, &pose).map_err(fail)?.0)
            }
            Shape::Boolean(op, pair) => {
                let (a, b) = pair.build(m).map_err(fail)?;
                let result = match op {
                    0 => fuse(m, a, b),
                    1 => common(m, a, b),
                    _ => cut(m, a, b),
                };
                match result {
                    Ok((body, _)) => Ok(body),
                    Err(e) => match arris_debug::differential::exclusion_of_error(&e) {
                        Some(exclusion) => Err(TestCaseError::reject(exclusion.name)),
                        None => Err(fail(e)),
                    },
                }
            }
        }
    }
}

fn shape() -> impl Strategy<Value = Shape> {
    prop_oneof![
        (prop::body::quadric_solid(), prop::pose())
            .prop_map(|(solid, pose)| Shape::Alone(solid, pose)),
        (0u8..3, prop::body::quadric_pair()).prop_map(|(op, pair)| Shape::Boolean(op, pair)),
    ]
}

/// Vertices, edges, faces, loops and shells of `bodies`, summed.
fn counts(m: &Model, bodies: &[Body]) -> Result<[usize; 5], TestCaseError> {
    let mut n = [0; 5];
    for &b in bodies {
        let c = m.closure(b).map_err(fail)?;
        let mut loops = 0;
        for &f in &c.faces {
            loops += m.face(f).map_err(fail)?.loops().len();
        }
        for (k, x) in [
            c.vertices.len(),
            c.edges.len(),
            c.faces.len(),
            loops,
            c.shells.len(),
        ]
        .into_iter()
        .enumerate()
        {
            n[k] += x;
        }
    }
    Ok(n)
}

/// The mass properties of `bodies` as one solid: volumes and areas
/// summed, the centroid weighted by volume, the inertia about it.
fn mass(m: &Model, bodies: &[Body]) -> Result<MassProperties, TestCaseError> {
    let parts: Vec<MassProperties> = (bodies.iter())
        .map(|&b| mass_properties(m, b).map_err(fail))
        .collect::<Result<_, _>>()?;
    let volume: f64 = parts.iter().map(|p| p.volume).sum();
    let area: f64 = parts.iter().map(|p| p.area).sum();
    let centroid = Point3::from(
        parts
            .iter()
            .fold(Vec3::zeros(), |s, p| s + p.volume * p.centroid.coords)
            / volume,
    );
    let inertia = parts.iter().map(|p| p.inertia_about(centroid)).sum();
    Ok(MassProperties {
        volume,
        area,
        centroid,
        inertia,
    })
}

/// Where `point` is against `bodies`: inside one, on one, or outside all.
fn classify(m: &Model, bodies: &[Body], point: Point3) -> Result<Classification, TestCaseError> {
    let mut found = Classification::Outside;
    for &b in bodies {
        match classify_point(m, b, point).map_err(fail)? {
            Classification::Outside => {}
            c @ Classification::Inside => return Ok(c),
            c @ Classification::On(_) => found = c,
        }
    }
    Ok(found)
}

/// `|a − b| ≤ rel · |b|`, or at rounding.
fn within(name: &str, a: f64, b: f64, rel: f64) -> Result<(), TestCaseError> {
    let bound = rel.max(REL) * b.abs().max(a.abs());
    prop_assert!(
        (a - b).abs() <= bound,
        "{name}: {a} read back of {b}, {:e} apart, above {bound:e}",
        (a - b).abs()
    );
    Ok(())
}

/// `f`, with a panic of the debug build's checker guard that a named
/// exclusion of the differential covers (`differential::EXCLUSIONS`) — a
/// boolean's output failing the checker, a kernel bug waiting on its
/// regression fixture — rejected as that exclusion, as the boolean
/// properties reject it, so the fix that lifts it there lifts it here.
/// Any other panic fails as before.
fn under_exclusions<T>(f: impl FnOnce() -> Result<T, TestCaseError>) -> Result<T, TestCaseError> {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(result) => result,
        Err(payload) => match arris_debug::differential::exclusion_of_panic(&*payload) {
            Some(exclusion) => Err(TestCaseError::reject(exclusion.name)),
            None => std::panic::resume_unwind(payload),
        },
    }
}

fn round_trip(shape: &Shape, probes: &[[f64; 3]]) -> Result<(), TestCaseError> {
    let mut m = Model::default();
    let body = under_exclusions(|| shape.build(&mut m))?;
    let original = check(&m, body, Level::Full);
    prop_assume!(original.is_ok() && original.unchecked().is_empty());
    let text = step::write(&m, &[body]).map_err(fail)?;

    let mut back = Model::new(m.precision()).map_err(fail)?;
    let read = step_read(&mut back, &text, &ReadOptions::default()).map_err(fail)?;
    let mut bodies = Vec::new();
    for solid in &read.solids {
        let read = solid
            .result
            .as_ref()
            .map_err(|r| fail(format!("refused: {r}")))?;
        let report = check(&back, read.body, Level::Full);
        prop_assert!(
            report.is_ok() && report.unchecked().is_empty(),
            "not clean at Full:\n{report}"
        );
        bodies.push(read.body);
    }
    prop_assert_eq!(counts(&back, &bodies)?, counts(&m, &[body])?, "counts");

    let zero = Tolerances {
        volume_rel: 0.0,
        area_rel: 0.0,
        centroid_abs: 0.0,
        inertia_rel: 0.0,
        ..Tolerances::default()
    };
    let own = within_own_tolerance(&zero, &m, body).map_err(fail)?;
    let (a, b) = (mass(&m, &[body])?, mass(&back, &bodies)?);
    within("volume", b.volume, a.volume, own.volume_rel)?;
    within("area", b.area, a.area, own.area_rel)?;
    let apart = (b.centroid - a.centroid).norm();
    prop_assert!(
        apart <= own.centroid_abs.max(REL * a.centroid.coords.norm()),
        "centroid: {} read back of {}, {apart:e} apart, above {:e}",
        b.centroid,
        a.centroid,
        own.centroid_abs
    );
    let scale = a.inertia.iter().fold(0.0, |s: f64, x| s.max(x.abs()));
    for (x, y) in b.inertia.iter().zip(a.inertia.iter()) {
        prop_assert!(
            (x - y).abs() <= own.inertia_rel.max(REL) * scale,
            "inertia: {} read back of {}",
            b.inertia,
            a.inertia
        );
    }

    // Probes in the body's box: the same side of the boundary, where
    // neither is on it within its own tolerance.
    let closure = m.closure(body).map_err(fail)?;
    let (mut lo, mut hi) = (Vec3::repeat(f64::INFINITY), Vec3::repeat(f64::NEG_INFINITY));
    for &v in &closure.vertices {
        let p = m.vertex(v).map_err(fail)?.point().coords;
        lo = lo.inf(&p);
        hi = hi.sup(&p);
    }
    for &e in &closure.edges {
        let Some((c, range)) = m.edge(e).map_err(fail)?.curve() else {
            continue;
        };
        if let Some(b) = m.curve(c).map_err(fail)?.bounds(range) {
            lo = lo.inf(&Vec3::from(b.min));
            hi = hi.sup(&Vec3::from(b.max));
        }
    }
    for f in probes {
        let p = Point3::from(lo + (hi - lo).component_mul(&Vec3::new(f[0], f[1], f[2])));
        let (was, is) = (classify(&m, &[body], p)?, classify(&back, &bodies, p)?);
        let on = |c: &Classification| matches!(c, Classification::On(_));
        prop_assert!(
            was == is || on(&was) || on(&is),
            "{p} was {was:?}, reads back {is:?}"
        );
    }
    Ok(())
}

prop_shards! {
    /// Written and read back, the body is the body.
    a_written_body_reads_back_as_itself
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        ((shape, probes)) = (
            shape(),
            proptest::collection::vec(
                [0.0f64..=1.0, 0.0f64..=1.0, 0.0f64..=1.0],
                8,
            ),
        ) => {
            round_trip(&shape, &probes)
        }
}

/// The round trip at 1000 cases on the fixed seed, shard 3 of 8, shrunk:
/// a frustum of radii 0.26 and 0.5, 1.5 tall and turned half a turn about
/// y, cut by an oblique cylinder of radius 0.15 out through its wide cap.
/// The cut's output failed the checker's L4 before anything was written —
/// the checker's polygon of a circle, not the boolean's or the reader's —
/// which the debug build's guard turned into a panic.
#[test]
fn a_frustum_cut_through_its_wide_cap_reads_back_as_itself() {
    use arris_debug::prop::body::{Cylindrical, QuadricTool};
    use arris_io::arris_check::arris_topo::arris_math::Axis;
    use arris_io::arris_check::arris_topo::arris_math::nalgebra::{Quaternion, UnitQuaternion};

    let half_turn = Isometry::new(
        UnitQuaternion::new_unchecked(Quaternion::new(0.0, 0.0, 1.0, 0.0)),
        Vec3::zeros(),
    );
    let pair = QuadricPair {
        solid: QuadricSolid::Frustum {
            bottom: 0.2584727351076834,
            top: 0.5,
            height: 1.501141612220231,
        },
        tool: QuadricTool::Cylinder(Cylindrical {
            axis: Axis::new(
                Point3::new(0.09947065928656604, 0.20812058424794005, 1.2075182278896701),
                Vec3::new(0.5459932962331497, -0.28606899234602357, 0.7874362527129359),
            )
            .unwrap(),
            radius: 0.1501160103252071,
            height: 3.2936801669847333,
            pose: half_turn,
        }),
        pose: half_turn,
    };
    round_trip(&Shape::Boolean(2, pair), &[[0.5, 0.5, 0.5]]).unwrap();
}
